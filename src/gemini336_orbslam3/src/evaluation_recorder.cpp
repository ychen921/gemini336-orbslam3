#include "common/evaluation_recorder.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <locale>
#include <stdexcept>
#include <type_traits>

namespace gemini336_orbslam3
{
namespace
{
// Metadata can contain paths and user-supplied identifiers; escape JSON control
// characters as well as quotes/backslashes without introducing a dependency.
void write_json_string(std::ostream &output, const std::string &value)
{
    constexpr char hex[] = "0123456789abcdef";
    output << '"';
    for (const unsigned char character : value)
    {
        if (character == '"' || character == '\\')
            output << '\\' << character;
        else if (character < 0x20)
            output << "\\u00" << hex[character >> 4] << hex[character & 0xf];
        else
            output << character;
    }
    output << '"';
}
}

void ImageTimestampStats::observe(int64_t timestamp_ns, double max_gap_sec) noexcept
{
    ++received;
    if (previous_ns)
    {
        if (timestamp_ns == *previous_ns)
            ++duplicates;
        else if (timestamp_ns < *previous_ns)
            ++backwards;
        else
        {
            const double interval = (timestamp_ns - *previous_ns) * 1e-9;
            if (positive_intervals == 0)
                interval_min_sec = interval;
            else
                interval_min_sec = std::min(interval_min_sec, interval);
            interval_max_sec = std::max(interval_max_sec, interval);
            interval_sum_sec += interval;
            ++positive_intervals;
            if (interval > max_gap_sec) ++excessive_gaps;
        }
    }
    // Compare successive callbacks, including a clock reset, rather than silently
    // treating this diagnostic accumulator as an input-admission policy.
    previous_ns = timestamp_ns;
}

EvaluationRecorder::EvaluationRecorder(std::size_t capacity) : capacity_(capacity)
{
    static_assert(std::is_trivially_copyable<EvaluationFrame>::value,
                  "Frame recording must not allocate while copying payloads");
    if (capacity == 0 || capacity > frames_.max_size())
        throw std::invalid_argument("Evaluation frame capacity is outside valid vector capacity");
    frames_.reserve(capacity);
}

void EvaluationRecorder::record(const EvaluationFrame &frame, int64_t first_enqueue_steady_ns) noexcept
{
    // Full aggregates continue after overflow; percentiles later describe the
    // retained prefix only. Never delay tracking to preserve diagnostic records.
    if (processed_ == 0)
    {
        first_enqueue_steady_ns_ = first_enqueue_steady_ns;
        first_sensor_ns_ = frame.sensor_timestamp_ns;
    }
    ++processed_;
    last_return_steady_ns_ = frame.adapter_return_steady_ns;
    last_sensor_ns_ = frame.sensor_timestamp_ns;
    const int state = frame.raw_tracking_state;
    ++state_counts_[state >= -1 && state <= 5 ? state + 1 : 7];
    if (state == 2 || state == 5) tracking_started_ = true;
    if (tracking_started_)
    {
        ++after_first_ok_frames_;
        if (state == 4) ++lost_frames_;
        if (state == 3) ++recently_lost_frames_;
    }
    else
        ++startup_frames_;

    const double adapter_ms =
        (frame.adapter_return_steady_ns - frame.adapter_start_steady_ns) * 1e-6;
    adapter_sum_ms_ += adapter_ms;
    adapter_max_ms_ = std::max(adapter_max_ms_, adapter_ms);
    enqueue_to_return_sum_ms_ +=
        (frame.adapter_return_steady_ns - frame.enqueue_steady_ns) * 1e-6;
    pending_sum_ += frame.pending;
    outstanding_sum_ += frame.outstanding;
    outstanding_sample_peak_ = std::max(outstanding_sample_peak_, frame.outstanding);

    if (frames_.size() == capacity_)
        ++dropped_;
    else
        frames_.push_back(frame);
}

void EvaluationRecorder::write(const std::filesystem::path &directory,
                               const EvaluationMetadata &metadata) const
{
    // Each stream closes explicitly so write failures reach process finalization.
    std::ofstream output;
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.imbue(std::locale::classic());
    output.open(directory / "metadata.json");
    output << "{\n  \"schema_version\": 1,\n  \"frame_record_bytes\": " << sizeof(EvaluationFrame)
           << ",\n  \"frame_capacity\": " << capacity_ << ",\n  \"clock\": \"steady_clock\",\n";
    for (const auto &entry : std::vector<std::pair<std::string, std::string>>{
             {"sensor_mode", metadata.sensor_mode}, {"run_id", metadata.run_id},
             {"bag_id", metadata.bag_id}, {"repository_commit", metadata.repository_commit},
             {"repository_dirty", metadata.repository_dirty}, {"settings_id", metadata.settings_id},
             {"vocabulary_id", metadata.vocabulary_id}})
    {
        output << "  ";
        write_json_string(output, entry.first);
        output << ": ";
        write_json_string(output, entry.second.empty() ? "unknown" : entry.second);
        output << ",\n";
    }
    output << "  \"unavailable_metrics\": [\"tracked_mappoints\", \"keyframe_count\", "
              "\"map_id\", \"map_version\", \"pose\"],\n  \"parameters_as_text\": {";
    for (std::size_t i = 0; i < metadata.parameters.size(); ++i)
    {
        output << (i == 0 ? "\n" : ",\n") << "    ";
        write_json_string(output, metadata.parameters[i].first);
        output << ": ";
        write_json_string(output, metadata.parameters[i].second);
    }
    output << "\n  }\n}\n";
    output.close();

    output.open(directory / "frames.csv");
    output << "sensor_timestamp_ns,enqueue_sequence,processed_index,enqueue_steady_ns,"
              "adapter_start_steady_ns,adapter_return_steady_ns,raw_tracking_state,"
              "pending_before_call,in_flight_before_call,outstanding_before_call,imu_samples,"
              "adapter_time_ms,enqueue_to_return_ms\n" << std::setprecision(17);
    for (const EvaluationFrame &frame : frames_)
        output << frame.sensor_timestamp_ns << ',' << frame.enqueue_sequence << ','
               << frame.processed_index << ',' << frame.enqueue_steady_ns << ','
               << frame.adapter_start_steady_ns << ',' << frame.adapter_return_steady_ns << ','
               << frame.raw_tracking_state << ',' << frame.pending << ',' << frame.in_flight << ','
               << frame.outstanding << ',' << frame.imu_samples << ','
               << (frame.adapter_return_steady_ns - frame.adapter_start_steady_ns) * 1e-6 << ','
               << (frame.adapter_return_steady_ns - frame.enqueue_steady_ns) * 1e-6 << '\n';
    output.close();

    output.open(directory / "summary.csv");
    output << "metric,value,scope\n"
           << "recorded," << frames_.size() << ",retained_frames\n"
           << "dropped," << dropped_ << ",all_processed\n"
           << "capacity," << capacity_ << ",retained_frames\n"
           << "processed," << processed_ << ",all_processed\n"
           << "startup_frames," << startup_frames_ << ",before_first_ok\n"
           << "lost_ratio_denominator," << after_first_ok_frames_ << ",after_first_ok\n"
           << "lost_frames," << lost_frames_ << ",after_first_ok\n"
           << "recently_lost_frames," << recently_lost_frames_ << ",after_first_ok\n";
    for (std::size_t i = 0; i < state_counts_.size(); ++i)
        output << "state_" << (i == 7 ? "unknown" : std::to_string(int(i) - 1)) << ','
               << state_counts_[i] << ",all_processed\n";
    const auto metric = [&](const char *name, double value, bool valid, const char *scope) {
        output << name << ',';
        if (valid) output << value;
        else output << "N/A";
        output << ',' << scope << '\n';
    };
    metric("lost_ratio", after_first_ok_frames_ ? double(lost_frames_) / after_first_ok_frames_ : 0.0,
           after_first_ok_frames_ != 0, "after_first_ok");
    metric("recently_lost_ratio", after_first_ok_frames_ ? double(recently_lost_frames_) / after_first_ok_frames_ : 0.0,
           after_first_ok_frames_ != 0, "after_first_ok");
    metric("adapter_mean_ms", processed_ ? adapter_sum_ms_ / processed_ : 0.0, processed_ != 0, "all_processed");
    metric("adapter_max_ms", adapter_max_ms_, processed_ != 0, "all_processed");
    metric("enqueue_to_return_mean_ms", processed_ ? enqueue_to_return_sum_ms_ / processed_ : 0.0,
           processed_ != 0, "all_processed");
    metric("pending_mean", processed_ ? pending_sum_ / processed_ : 0.0,
           processed_ != 0, "before_successful_calls");
    metric("outstanding_mean", processed_ ? outstanding_sum_ / processed_ : 0.0,
           processed_ != 0, "before_successful_calls");
    metric("outstanding_sample_peak", outstanding_sample_peak_, processed_ != 0, "before_successful_calls");
    const double elapsed = (last_return_steady_ns_ - first_enqueue_steady_ns_) * 1e-9;
    metric("host_throughput_hz", elapsed > 0 ? processed_ / elapsed : 0.0,
           processed_ != 0 && elapsed > 0, "first_enqueue_to_last_return");
    const double sensor_span = (last_sensor_ns_ - first_sensor_ns_) * 1e-9;
    metric("processed_sensor_interval_rate_hz", sensor_span > 0 ? (processed_ - 1) / sensor_span : 0.0,
           processed_ > 1 && sensor_span > 0, "processed_sensor_span");

    // Sorting and temporary allocations occur only after all callbacks stop.
    std::vector<double> durations;
    durations.reserve(frames_.size());
    for (const EvaluationFrame &frame : frames_)
        durations.push_back((frame.adapter_return_steady_ns - frame.adapter_start_steady_ns) * 1e-6);
    std::sort(durations.begin(), durations.end());
    for (const auto &percentile : std::array<std::pair<const char *, double>, 3>{{
             {"adapter_p50_ms", 0.5}, {"adapter_p95_ms", 0.95}, {"adapter_p99_ms", 0.99}}})
    {
        // Nearest-rank percentiles; overflow makes this explicitly a prefix sample.
        const std::size_t rank = durations.empty() ? 0 :
            static_cast<std::size_t>(std::ceil(percentile.second * durations.size())) - 1;
        metric(percentile.first, durations.empty() ? 0.0 : durations[rank], !durations.empty(), "retained_frames");
    }
    output.close();
}
}
