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
              "  \"final_trajectory_enabled\": " << (metadata.final_trajectory_enabled ? "true" : "false") << ",\n"
              "  \"final_trajectory_manifest\": \"final_trajectory/manifest.json\",\n"
              "  \"unavailable_metrics_scope\": \"live frames.csv; optional post-shutdown trajectory availability is in its separate manifest\",\n"
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
              "  \"export_complete_scope\": \"metadata.json,frames.csv,summary.csv,inputs.csv before backend shutdown; excludes backend_summary.csv and final_trajectory\",\n"
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

namespace
{
void validate_trajectory_transform(const FinalTrajectoryTransform &pose)
{
    double squared_norm = 0.0;
    for (double value : pose.translation)
        if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite trajectory translation");
    for (double value : pose.quaternion_xyzw)
    {
        if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite trajectory quaternion");
        squared_norm += value * value;
    }
    if (!std::isfinite(squared_norm) || std::abs(squared_norm - 1.0) > 1e-3)
        throw std::invalid_argument("Invalid trajectory quaternion norm");
}

void validate_trajectory_counts(const FinalTrajectoryFileSummary &summary)
{
    uint64_t remaining = summary.considered;
    for (uint64_t value : {summary.written, summary.lost, summary.invalid_reference,
                          summary.other_map, summary.invalid_pose, summary.invalid_timestamp})
    {
        if (value > remaining) throw std::invalid_argument("Inconsistent trajectory row counts");
        remaining -= value;
    }
    if (remaining != 0 || (summary.written != 0 &&
        (!std::isfinite(summary.first_timestamp_sec) || !std::isfinite(summary.last_timestamp_sec))))
        throw std::invalid_argument("Invalid trajectory counts or source times");
}

void validate_final_trajectory_result(const FinalTrajectoryExportResult &result, bool inertial)
{
    if (result.pose_target != (inertial ? FinalTrajectoryPoseTarget::ImuBody :
                                       FinalTrajectoryPoseTarget::LeftCamera))
        throw std::invalid_argument("Trajectory sensor frame does not match sensor mode");
    switch (result.status)
    {
    case FinalTrajectoryStatus::Complete:
        if (!result.map_selected || result.frames.written == 0 || result.keyframes.written == 0)
            throw std::invalid_argument("Complete trajectory requires both files and a selected map");
        break;
    case FinalTrajectoryStatus::NoMap:
        if (result.map_selected) throw std::invalid_argument("No-map trajectory selected a map");
        break;
    case FinalTrajectoryStatus::NoKeyFrames:
        if (result.keyframes.written != 0) throw std::invalid_argument("No-keyframe trajectory contains keyframes");
        break;
    case FinalTrajectoryStatus::NoFramePoses:
        if (!result.map_selected || result.frames.written != 0)
            throw std::invalid_argument("No-frame trajectory has inconsistent frame/map metadata");
        break;
    default: throw std::invalid_argument("Unknown final trajectory status");
    }
    if (!result.map_selected)
    {
        if (result.frames.written != 0 || result.keyframes.written != 0)
            throw std::invalid_argument("Trajectory rows have no selected map");
        return;
    }
    if (result.map_keyframes == 0 || result.keyframes.considered != result.map_keyframes ||
        !std::isfinite(result.origin_timestamp_sec) || result.body_from_camera_valid != inertial)
        throw std::invalid_argument("Invalid final trajectory map/calibration metadata");
    validate_trajectory_transform(result.frame_origin_in_map);
    if (inertial) validate_trajectory_transform(result.body_from_camera);
}

struct TrajectoryFileInfo
{
    bool present = false;
    std::uintmax_t bytes = 0;
};

TrajectoryFileInfo inspect_trajectory_file(const std::filesystem::path &path,
                                         const FinalTrajectoryFileSummary &summary, bool map_selected)
{
    validate_trajectory_counts(summary);
    if (!std::filesystem::exists(path))
    {
        if (map_selected || summary.written != 0)
            throw std::runtime_error("Missing trajectory output: " + path.string());
        return {};
    }
    if (!std::filesystem::is_regular_file(std::filesystem::symlink_status(path)))
        throw std::runtime_error("Trajectory output is not a regular file: " + path.string());
    std::ifstream input(path);
    input.exceptions(std::ios::badbit);
    if (!input) throw std::runtime_error("Cannot read trajectory output: " + path.string());
    uint64_t rows = 0;
    std::string line;
    while (std::getline(input, line))
    {
        if (line.empty()) throw std::runtime_error("Empty trajectory row: " + path.string());
        ++rows;
    }
    if (!input.eof() || rows != summary.written)
        throw std::runtime_error("Trajectory row count does not match Core result: " + path.string());
    return {true, std::filesystem::file_size(path)};
}

void write_trajectory_transform(std::ostream &output, const FinalTrajectoryTransform &pose)
{
    output << "{\"translation_m\": [";
    for (std::size_t i = 0; i < pose.translation.size(); ++i)
        output << (i == 0 ? "" : ", ") << pose.translation[i];
    output << "], \"quaternion_xyzw\": [";
    for (std::size_t i = 0; i < pose.quaternion_xyzw.size(); ++i)
        output << (i == 0 ? "" : ", ") << pose.quaternion_xyzw[i];
    output << "]}";
}

void write_trajectory_file_info(std::ostream &output, const char *filename,
                                const TrajectoryFileInfo &info, const FinalTrajectoryFileSummary &summary)
{
    output << "{\"path\": ";
    write_json_string(output, filename);
    output << ", \"present\": " << (info.present ? "true" : "false") << ", \"bytes\": ";
    if (info.present) output << info.bytes;
    else output << "null";
    output << ", \"considered\": " << summary.considered << ", \"written\": " << summary.written
           << ", \"lost\": " << summary.lost << ", \"invalid_reference\": " << summary.invalid_reference
           << ", \"other_map\": " << summary.other_map << ", \"invalid_pose\": " << summary.invalid_pose
           << ", \"invalid_timestamp\": " << summary.invalid_timestamp << ", \"first_timestamp_sec\": ";
    if (summary.written != 0) output << summary.first_timestamp_sec;
    else output << "null";
    output << ", \"last_timestamp_sec\": ";
    if (summary.written != 0) output << summary.last_timestamp_sec;
    else output << "null";
    output << '}';
}
}

FinalTrajectoryExportResult write_evaluation_final_trajectory(
    const std::filesystem::path &directory, const EvaluationMetadata &metadata,
    const FinalTrajectoryExporter &exporter)
{
    const bool inertial = metadata.sensor_mode == "stereo_imu";
    if (metadata.sensor_mode != "stereo" && !inertial)
        throw std::invalid_argument("Final trajectory metadata requires stereo or stereo_imu");
    if (!exporter) throw std::invalid_argument("Final trajectory exporter is empty");
    const std::filesystem::path destination = directory / "final_trajectory";
    const std::filesystem::path staging = directory / "final_trajectory.tmp";
    if (std::filesystem::exists(std::filesystem::symlink_status(destination)))
        throw std::runtime_error("Final trajectory destination already exists");
    if (!std::filesystem::create_directory(staging))
        throw std::runtime_error("Final trajectory staging directory already exists");

    // Backend has already joined. Neither this callback nor publication is used
    // by tracking; an exception deliberately leaves staging for diagnosis.
    const FinalTrajectoryExportResult result = exporter(staging / "CameraTrajectory.txt",
                                                        staging / "KeyFrameTrajectory.txt");
    validate_final_trajectory_result(result, inertial);
    const TrajectoryFileInfo frames = inspect_trajectory_file(
        staging / "CameraTrajectory.txt", result.frames, result.map_selected);
    const TrajectoryFileInfo keyframes = inspect_trajectory_file(
        staging / "KeyFrameTrajectory.txt", result.keyframes, result.map_selected);

    std::ofstream output;
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.imbue(std::locale::classic());
    output.open(staging / "manifest.json.tmp");
    output << std::setprecision(17)
           << "{\n  \"schema_version\": 1,\n  \"evaluation_schema_version\": " << EvaluationSchemaVersion
           << ",\n  \"export_complete\": true,\n  \"available\": "
           << (result.status == FinalTrajectoryStatus::Complete ? "true" : "false") << ",\n  \"status\": ";
    write_json_string(output, final_trajectory_status_name(result.status));
    output << ",\n  \"run_id\": ";
    write_json_string(output, metadata.run_id.empty() ? "unknown" : metadata.run_id);
    output << ",\n  \"sensor_mode\": ";
    write_json_string(output, metadata.sensor_mode);
    output << ",\n  \"scope\": \"post-shutdown final-map reconstruction\",\n"
              "  \"backend_shutdown_returned\": true,\n  \"all_gba_completed\": null,\n"
              "  \"map_selection_policy\": \"largest non-bad keyframe count; ties use lower map ID\",\n"
              "  \"map_selected\": " << (result.map_selected ? "true" : "false") << ",\n  \"map_id\": ";
    if (result.map_selected) output << result.map_id;
    else output << "null";
    output << ",\n  \"map_keyframes\": ";
    if (result.map_selected) output << result.map_keyframes;
    else output << "null";
    output << ",\n  \"origin_keyframe_id\": ";
    if (result.map_selected) output << result.origin_keyframe_id;
    else output << "null";
    output << ",\n  \"origin_timestamp_sec\": ";
    if (result.map_selected) output << result.origin_timestamp_sec;
    else output << "null";
    output << ",\n  \"pose_target\": \"" << (inertial ? "imu_body" : "left_camera")
           << "\",\n  \"imu_initialized\": ";
    if (result.map_selected && inertial) output << (result.imu_initialized ? "true" : "false");
    else output << "null";
    output << ",\n  \"timestamp_unit\": \"ns\",\n"
              "  \"timestamp_source\": \"Core double seconds multiplied by 1e9; not exact original sensor ns\",\n"
              "  \"translation_unit\": \"m\",\n  \"quaternion_order\": \"xyzw\",\n"
              "  \"frame_pose_convention\": \"T_origin_sensor\",\n"
              "  \"keyframe_pose_convention\": \"T_map_sensor\",\n"
              "  \"frame_origin_in_map\": ";
    if (result.map_selected) write_trajectory_transform(output, result.frame_origin_in_map);
    else output << "null";
    output << ",\n  \"body_from_camera\": ";
    if (result.map_selected && result.body_from_camera_valid)
        write_trajectory_transform(output, result.body_from_camera);
    else output << "null";
    output << ",\n  \"camera_reconstruction\": \"frame: T_map_origin * T_origin_sensor * T_sensor_camera; "
              "keyframe: T_map_sensor * T_sensor_camera; T_sensor_camera=T_body_camera for IMU, identity for stereo\",\n"
              "  \"files\": {\n    \"frames\": ";
    write_trajectory_file_info(output, "CameraTrajectory.txt", frames, result.frames);
    output << ",\n    \"keyframes\": ";
    write_trajectory_file_info(output, "KeyFrameTrajectory.txt", keyframes, result.keyframes);
    output << "\n  },\n  \"export_complete_scope\": \"this directory only; process completion still requires STOP_PROCESS\"\n}\n";
    output.close();
    std::filesystem::rename(staging / "manifest.json.tmp", staging / "manifest.json");
    // The session has one serialized publisher. Publish the complete directory
    // last; recheck to avoid replacing a destination created during export.
    if (std::filesystem::exists(std::filesystem::symlink_status(destination)))
        throw std::runtime_error("Final trajectory destination appeared during export");
    std::filesystem::rename(staging, destination);
    return result;
}
}
