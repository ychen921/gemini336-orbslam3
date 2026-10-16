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
    const TrackingStatistics &tracking = frame.tracking;
    if (tracking.enabled && tracking.has_frame)
    {
        ++snapshot_frames_;
        if (tracking.pose_enabled)
        {
            ++pose_collection_frames_;
            if (tracking.pose_valid) ++pose_valid_samples_;
        }
        last_keyframes_inserted_ = tracking.keyframes_inserted_total;
        tracking_lost_events_ += tracking.tracking_lost_events;
        relocalization_events_ += tracking.relocalization_events;
        if (tracking.tracked_mappoint_inliers_valid)
        {
            const uint32_t inliers = tracking.tracked_mappoint_inliers;
            if (inlier_samples_ == 0) inlier_min_ = inliers;
            else inlier_min_ = std::min(inlier_min_, inliers);
            inlier_max_ = std::max(inlier_max_, inliers);
            inlier_sum_ += inliers;
            ++inlier_samples_;
        }
    }
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
    output << "{\n  \"schema_version\": " << EvaluationSchemaVersion
           << ",\n  \"frame_record_bytes\": " << sizeof(EvaluationFrame)
           << ",\n  \"frame_capacity\": " << capacity_ << ",\n  \"clock\": \"steady_clock\",\n";
    for (const auto &entry : std::vector<std::pair<std::string, std::string>>{
             {"sensor_mode", metadata.sensor_mode}, {"run_id", metadata.run_id},
             {"bag_id", metadata.bag_id}, {"repository_commit", metadata.repository_commit},
             {"repository_dirty", metadata.repository_dirty}, {"settings_id", metadata.settings_id},
             {"vocabulary_id", metadata.vocabulary_id}, {"core_commit", metadata.core_commit},
             {"core_dirty", metadata.core_dirty}})
    {
        output << "  ";
        write_json_string(output, entry.first);
        output << ": ";
        write_json_string(output, entry.second.empty() ? "unknown" : entry.second);
        output << ",\n";
    }
    output << "  \"unavailable_metrics\": [\"unique_tracked_mappoints\", \"map_version\", \"final_optimized_pose\"";
    if (!metadata.pose_enabled) output << ", \"pose\"";
    output << "],\n  \"pose_enabled\": " << (metadata.pose_enabled ? "true" : "false") << ",\n"
              "  \"pose_convention\": \"Twc: left camera to tracking-map world; tx,ty,tz in metres; quaternion qx,qy,qz,qw\",\n"
              "  \"pose_scope\": \"live tracking capture, not final optimized history; map IDs and change counters do not certify a common coordinate version\",\n"
              "  \"pose_valid_definition\": \"opt-in same-frame Core pose in OK/OK_KLT with finite translation and a valid rotation; invalid numeric fields are N/A\",\n"
              "  \"pose_capture_steady_ns_definition\": \"Core map-locked pose/context capture time, including startup with no valid pose; N/A if not captured\",\n"
              "  \"map_change_index_definition\": \"existing Map change counter sampled with pose/context; not a coordinate version\",\n"
              "  \"imu_initialized_definition\": \"map initialization flag at capture; independent of pose validity; N/A for stereo or missing capture\",\n"
              "  \"tracked_mappoint_inliers_definition\": \"persistent non-outlier feature associations in TrackLocalMap; not unique point IDs\",\n"
              "  \"tracking_map_id_definition\": \"map used by frame; merge may rewrite IDs; not a coordinate version\",\n"
              "  \"keyframes_inserted_total_definition\": \"new Atlas admissions during this run; excludes duplicates and merge transfers\",\n"
              "  \"frame_event_scope\": \"normally returned calls; final backend counters also include events in throwing calls\",\n"
              "  \"event_detail_source\": \"slam.log T01_EVENT; sort by steady_ns then sequence\",\n"
              "  \"event_totals_source\": \"backend_summary.csv after successful joins; details require zero logger losses and matching counts\",\n"
              "  \"map_baseline_definition\": \"active map at collection setup, excluded from map_created\",\n"
              "  \"merge_applied_definition\": \"topology transfer committed; does not certify subsequent BA completion\",\n"
              "  \"export_complete_scope\": \"metadata.json,frames.csv,summary.csv,inputs.csv before backend shutdown; excludes backend_summary.csv\",\n"
              "  \"parameters_as_text\": {";
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
              "adapter_time_ms,enqueue_to_return_ms,core_snapshot_valid,core_frame_id,"
              "tracking_map_valid,tracking_map_id,tracked_mappoint_inliers_valid,tracked_mappoint_inliers,"
              "keyframes_inserted_total,tracking_lost_events,relocalization_events,"
              "pose_enabled,pose_valid,pose_capture_steady_ns,map_change_index_valid,map_change_index,"
              "imu_initialized_valid,imu_initialized,tx,ty,tz,qx,qy,qz,qw\n" << std::setprecision(17);
    for (const EvaluationFrame &frame : frames_)
    {
        output << frame.sensor_timestamp_ns << ',' << frame.enqueue_sequence << ','
               << frame.processed_index << ',' << frame.enqueue_steady_ns << ','
               << frame.adapter_start_steady_ns << ',' << frame.adapter_return_steady_ns << ','
               << frame.raw_tracking_state << ',' << frame.pending << ',' << frame.in_flight << ','
               << frame.outstanding << ',' << frame.imu_samples << ','
               << (frame.adapter_return_steady_ns - frame.adapter_start_steady_ns) * 1e-6 << ','
               << (frame.adapter_return_steady_ns - frame.enqueue_steady_ns) * 1e-6;
        const TrackingStatistics &tracking = frame.tracking;
        const bool valid = tracking.enabled && tracking.has_frame;
        const auto value = [&](const auto &number, bool available) {
            output << ',';
            if (available) output << number;
            else output << "N/A";
        };
        value(valid, true);
        value(tracking.core_frame_id, valid);
        value(valid && tracking.map_id_valid, true);
        value(tracking.tracking_map_id, valid && tracking.map_id_valid);
        value(valid && tracking.tracked_mappoint_inliers_valid, true);
        value(tracking.tracked_mappoint_inliers, valid && tracking.tracked_mappoint_inliers_valid);
        value(tracking.keyframes_inserted_total, valid);
        value(tracking.tracking_lost_events, valid);
        value(tracking.relocalization_events, valid);
        const bool pose_collection = valid && tracking.pose_enabled;
        const bool pose_valid = pose_collection && tracking.pose_valid;
        value(tracking.enabled && tracking.pose_enabled, true);
        value(pose_valid, true);
        value(tracking.pose_capture_steady_ns, pose_collection && tracking.pose_capture_steady_ns > 0);
        value(pose_collection && tracking.map_change_index_valid, true);
        value(tracking.map_change_index, pose_collection && tracking.map_change_index_valid);
        value(pose_collection && tracking.imu_initialized_valid, true);
        value(tracking.imu_initialized, pose_collection && tracking.imu_initialized_valid);
        for (const double coordinate : tracking.pose_translation_wc) value(coordinate, pose_valid);
        for (const double coordinate : tracking.pose_quaternion_wc_xyzw) value(coordinate, pose_valid);
        output << '\n';
    }
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
    output << "core_snapshot_frames," << snapshot_frames_ << ",all_processed\n"
           << "core_snapshot_unavailable_frames," << processed_ - snapshot_frames_ << ",all_processed\n"
           << "pose_collection_frames," << pose_collection_frames_ << ",all_processed\n"
           << "pose_valid_samples," << pose_valid_samples_ << ",all_processed\n"
           << "pose_invalid_samples," << pose_collection_frames_ - pose_valid_samples_
           << ",pose_collection_frames_all_processed\n"
           << "pose_disabled_or_unavailable_frames," << processed_ - pose_collection_frames_ << ",all_processed\n"
           << "tracked_mappoint_inliers_valid_samples," << inlier_samples_ << ",all_processed\n"
           << "tracked_mappoint_inliers_unavailable_samples," << processed_ - inlier_samples_ << ",all_processed\n";
    metric("tracked_mappoint_inliers_mean", inlier_samples_ ? double(inlier_sum_) / inlier_samples_ : 0.0,
           inlier_samples_ != 0, "valid_inlier_samples_all_processed");
    metric("tracked_mappoint_inliers_min", inlier_min_, inlier_samples_ != 0, "valid_inlier_samples_all_processed");
    metric("tracked_mappoint_inliers_max", inlier_max_, inlier_samples_ != 0, "valid_inlier_samples_all_processed");
    output << "tracking_lost_events,";
    if (snapshot_frames_ == processed_ && processed_ != 0) output << tracking_lost_events_;
    else output << "N/A";
    output << ",normally_returned_calls\nrelocalization_events,";
    if (snapshot_frames_ == processed_ && processed_ != 0) output << relocalization_events_;
    else output << "N/A";
    output << ",normally_returned_calls\nkeyframes_inserted_last_frame,";
    if (last_keyframes_inserted_) output << *last_keyframes_inserted_;
    else output << "N/A";
    output << ",last_valid_snapshot_before_backend_shutdown\n";
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

void write_evaluation_backend_summary(const std::filesystem::path &directory,
                                      const EvaluationBackendSummary &summary)
{
    const std::filesystem::path temporary = directory / "backend_summary.csv.tmp";
    std::ofstream output;
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.imbue(std::locale::classic());
    output.open(temporary);
    output << "metric,value,scope\ncollection_enabled," << summary.enabled << ",run\n"
           << "backend_summary_available," << (summary.enabled && summary.available) << ",after_backend_join\n";
    const auto value = [&](const std::string &name, uint64_t number) {
        output << name << ',';
        if (summary.enabled && summary.available) output << number;
        else output << "N/A";
        output << ",after_backend_join\n";
    };
    value("keyframes_inserted_total", summary.keyframes_inserted_total);
    for (std::size_t index = 0; index < EvaluationEventNames.size(); ++index)
        value(std::string("event_") + EvaluationEventNames[index], summary.event_counts[index]);
    output.close();
    std::filesystem::rename(temporary, directory / "backend_summary.csv");
}
}
