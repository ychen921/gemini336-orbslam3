#include "common/evaluation_recorder.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <new>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>
#include <unistd.h>

namespace
{
bool reject_allocation = false;

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

std::string read(const std::filesystem::path &path)
{
    std::ifstream input(path);
    require(bool(input), "missing evaluation artifact");
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

// Frame CSV contains only numeric/N/A cells. Resolve by header so appending
// columns cannot silently shift a test onto a different measurement.
std::vector<std::map<std::string, std::string>> read_frames(const std::filesystem::path &path)
{
    std::ifstream input(path);
    require(bool(input), "missing frame CSV");
    std::string line, cell;
    require(bool(std::getline(input, line)), "missing frame CSV header");
    std::istringstream header(line);
    std::vector<std::string> columns;
    while (std::getline(header, cell, ',')) columns.push_back(cell);
    std::vector<std::map<std::string, std::string>> rows;
    while (std::getline(input, line))
    {
        std::istringstream values(line);
        std::map<std::string, std::string> row;
        for (const std::string &column : columns)
        {
            require(bool(std::getline(values, cell, ',')), "frame CSV omitted a column");
            require(row.emplace(column, cell).second, "frame CSV has a duplicate header");
        }
        require(!std::getline(values, cell, ','), "frame CSV has an extra column");
        rows.push_back(std::move(row));
    }
    return rows;
}

gemini336_orbslam3::EvaluationFrame frame(uint64_t index, int state, int milliseconds)
{
    // A timestamp beyond double's integer precision must round-trip unchanged.
    return {1700000000000000001LL + int64_t(index - 1) * 33333333,
            index + 10, index, int64_t(index) * 1000000000,
            int64_t(index) * 1000000000 + 1000000,
            int64_t(index) * 1000000000 + 1000000 + milliseconds * 1000000LL,
            state, 1, 1, 2, index};
}

void test_overflow_and_export(const std::filesystem::path &directory)
{
    gemini336_orbslam3::EvaluationRecorder recorder(2);
    // Throw on any allocation in the producer path, including full-buffer calls.
    reject_allocation = true;
    recorder.record(frame(1, 1, 1), 1000000000);
    recorder.record(frame(2, 2, 2), 1000000000);
    recorder.record(frame(3, 3, 40), 1000000000);
    recorder.record(frame(4, 4, 50), 1000000000);
    recorder.record(frame(5, 99, 60), 1000000000);
    reject_allocation = false;
    require(recorder.recorded() == 2 && recorder.dropped() == 3 && recorder.capacity() == 2,
            "overflow changed capacity or lost omission accounting");
    require(std::filesystem::is_empty(directory), "producer wrote evaluation artifacts");

    gemini336_orbslam3::EvaluationMetadata metadata;
    metadata.sensor_mode = "stereo_imu";
    metadata.run_id = "quoted\"run\n\\end";
    metadata.parameters.emplace_back("path", "/a\t\"b");
    recorder.write(directory, metadata);
    const std::string csv = read(directory / "frames.csv");
    require(csv.find("1700000000000000001,11,1,") != std::string::npos &&
            csv.find("1700000000033333334,12,2,") != std::string::npos &&
            csv.find("1700000000066666667") == std::string::npos,
            "CSV lost precise timestamps/identity or replaced retained prefix");
    const std::string summary = read(directory / "summary.csv");
    for (const char *expected : {"processed,5,all_processed", "startup_frames,1,before_first_ok",
         "lost_ratio_denominator,4,after_first_ok", "lost_ratio,0.25,after_first_ok",
         "recently_lost_ratio,0.25,after_first_ok", "state_unknown,1,all_processed",
         "adapter_max_ms,60,all_processed", "adapter_p95_ms,2,retained_frames"})
        require(summary.find(expected) != std::string::npos,
                "overflow corrupted full aggregate or percentile scope");
    const std::string json = read(directory / "metadata.json");
    require(json.find("quoted\\\"run\\u000a\\\\end") != std::string::npos &&
            json.find("/a\\u0009\\\"b") != std::string::npos &&
            json.find("\"bag_id\": \"unknown\"") != std::string::npos &&
            json.find("\"pose_enabled\": false") != std::string::npos &&
            json.find("\"pose\"") != std::string::npos,
            "metadata escaping or missing provenance semantics changed");
}

void test_empty_and_failure(const std::filesystem::path &directory)
{
    gemini336_orbslam3::EvaluationRecorder recorder(1);
    recorder.write(directory, {});
    const std::string summary = read(directory / "summary.csv");
    require(summary.find("lost_ratio,N/A,after_first_ok") != std::string::npos &&
            summary.find("adapter_p99_ms,N/A,retained_frames") != std::string::npos &&
            summary.find("host_throughput_hz,N/A,") != std::string::npos,
            "empty input fabricated measurements");
    bool failed = false;
    try { recorder.write(directory / "missing", {}); }
    catch (const std::ios_base::failure &) { failed = true; }
    require(failed, "export failure was silently ignored");
    failed = false;
    try { gemini336_orbslam3::EvaluationRecorder invalid(0); }
    catch (const std::invalid_argument &) { failed = true; }
    require(failed, "zero capacity was accepted");
}

void test_tracking_statistics(const std::filesystem::path &directory)
{
    using namespace gemini336_orbslam3;
    EvaluationRecorder recorder(2);
    std::array<EvaluationFrame, 4> values{{frame(1, 1, 1), frame(2, 2, 2),
                                         frame(3, 0, 3), frame(4, 2, 4)}};
    for (std::size_t index = 0; index < values.size(); ++index)
    {
        TrackingStatistics &tracking = values[index].tracking;
        tracking.enabled = tracking.has_frame = true;
        tracking.core_frame_id = index + 10;
        tracking.map_id_valid = index != 0;
        tracking.tracking_map_id = 42;
        tracking.keyframes_inserted_total = index == 0 ? 1 : (index == 3 ? 3 : 2);
    }
    values[1].tracking.tracked_mappoint_inliers_valid = true; // Valid zero, not N/A.
    values[2].tracking.tracking_lost_events = 1; // Returned NO_IMAGES_YET after reset.
    values[3].tracking.tracked_mappoint_inliers_valid = true;
    values[3].tracking.tracked_mappoint_inliers = 80;
    values[3].tracking.relocalization_events = 1;
    reject_allocation = true;
    for (const EvaluationFrame &value : values) recorder.record(value, 1000000000);
    reject_allocation = false;
    recorder.write(directory, {});
    const std::string csv = read(directory / "frames.csv");
    require(csv.find(",1,10,0,N/A,0,N/A,1,0,0,") != std::string::npos &&
            csv.find(",1,11,1,42,1,0,2,0,0,") != std::string::npos,
            "CSV confused unavailable samples with valid zero inliers");
    const std::string summary = read(directory / "summary.csv");
    for (const char *expected : {"core_snapshot_frames,4,all_processed",
         "tracked_mappoint_inliers_mean,40,valid_inlier_samples_all_processed",
         "tracked_mappoint_inliers_min,0,valid_inlier_samples_all_processed",
         "tracked_mappoint_inliers_max,80,valid_inlier_samples_all_processed",
         "tracking_lost_events,1,normally_returned_calls",
         "relocalization_events,1,normally_returned_calls",
         "keyframes_inserted_last_frame,3,last_valid_snapshot_before_backend_shutdown"})
        require(summary.find(expected) != std::string::npos,
                "Overflow lost tracking aggregates or same-call LOST events");
    EvaluationBackendSummary backend;
    backend.enabled = true;
    write_evaluation_backend_summary(directory, backend);
    require(read(directory / "backend_summary.csv").find("keyframes_inserted_total,N/A,") !=
            std::string::npos, "Failed/uncompleted join fabricated a final count");
    backend.available = true;
    backend.keyframes_inserted_total = 5;
    backend.event_counts[1] = 1;
    write_evaluation_backend_summary(directory, backend);
    const std::string final = read(directory / "backend_summary.csv");
    require(final.find("keyframes_inserted_total,5,after_backend_join") != std::string::npos &&
            final.find("event_tracking_lost,1,after_backend_join") != std::string::npos &&
            !std::filesystem::exists(directory / "backend_summary.csv.tmp"),
            "Final totals mixed last-frame scope or left an incomplete export");
    require(read(directory / "metadata.json").find("\"schema_version\": 3") != std::string::npos,
            "Numeric extension did not version its export schema");
}

void test_pose_export(const std::filesystem::path &directory, bool inertial)
{
    using namespace gemini336_orbslam3;
    std::filesystem::create_directory(directory);
    EvaluationRecorder recorder(4);
    std::array<EvaluationFrame, 6> values{{frame(1, 1, 1), frame(2, 2, 2), frame(3, 2, 3),
                                         frame(4, 2, 4), frame(5, 0, 5), frame(6, 2, 6)}};
    for (std::size_t index = 0; index < values.size(); ++index)
    {
        TrackingStatistics &tracking = values[index].tracking;
        tracking.enabled = tracking.has_frame = tracking.map_id_valid = tracking.pose_enabled = true;
        tracking.core_frame_id = index + 10;
        tracking.map_change_index_valid = true; // A valid zero counter must survive.
        tracking.pose_capture_steady_ns = values[index].adapter_start_steady_ns + 1;
        tracking.imu_initialized_valid = inertial;
        tracking.imu_initialized = inertial && index >= 2;
        tracking.pose_translation_wc = {{99.0, 99.0, 99.0}};
        tracking.pose_quaternion_wc_xyzw = {{0.0, 0.0, 0.0, 1.0}};
    }
    values[1].tracking.pose_valid = true;
    values[1].tracking.pose_translation_wc = {}; // Valid origin, not N/A.
    values[2].tracking.pose_valid = true;
    values[2].tracking.pose_translation_wc = {{-2.0, 1.0, -3.0}};
    values[2].tracking.pose_quaternion_wc_xyzw = {{0.0, 0.0, -0.6, 0.8}};
    values[3].tracking.pose_enabled = false;
    values[3].tracking.pose_valid = true; // Disabled collection must mask stale payloads.
    values[4].tracking.map_id_valid = values[4].tracking.map_change_index_valid = false;
    values[4].tracking.imu_initialized_valid = false;
    values[4].tracking.pose_capture_steady_ns = 0;
    values[5].tracking.pose_valid = true; // Valid overflow sample still affects totals.

    reject_allocation = true;
    for (const EvaluationFrame &value : values) recorder.record(value, 1000000000);
    reject_allocation = false;
    require(recorder.recorded() == 4 && recorder.dropped() == 2 && std::filesystem::is_empty(directory),
            "Pose collection allocated, wrote files or changed overflow behavior");
    EvaluationMetadata metadata;
    metadata.sensor_mode = inertial ? "stereo_imu" : "stereo";
    metadata.pose_enabled = true;
    recorder.write(directory, metadata);
    const auto rows = read_frames(directory / "frames.csv");
    require(rows.size() == 4 && rows[0].at("sensor_timestamp_ns") == "1700000000000000001",
            "Pose extension changed retained rows or exact sensor timestamps");
    require(rows[0].at("pose_enabled") == "1" && rows[0].at("pose_valid") == "0" &&
            rows[0].at("map_change_index_valid") == "1" && rows[0].at("map_change_index") == "0" &&
            rows[0].at("pose_capture_steady_ns") == std::to_string(values[0].tracking.pose_capture_steady_ns),
            "Startup discarded valid capture context or fabricated pose validity");
    for (const char *column : {"tx", "ty", "tz", "qx", "qy", "qz", "qw"})
        require(rows[0].at(column) == "N/A" && rows[3].at(column) == "N/A",
                "Invalid or disabled pose exported numeric storage");
    require(rows[1].at("pose_valid") == "1" && rows[1].at("tx") == "0" &&
            rows[1].at("ty") == "0" && rows[1].at("tz") == "0" && rows[1].at("qw") == "1" &&
            rows[1].at("imu_initialized_valid") == (inertial ? "1" : "0") &&
            rows[1].at("imu_initialized") == (inertial ? "0" : "N/A"),
            "Valid zero pose or uninitialized IMU was confused with unavailable data");
    require(rows[2].at("tx") == "-2" && rows[2].at("ty") == "1" && rows[2].at("tz") == "-3" &&
            std::abs(std::stod(rows[2].at("qz")) + 0.6) < 1e-12 &&
            std::abs(std::stod(rows[2].at("qw")) - 0.8) < 1e-12 &&
            rows[2].at("imu_initialized") == (inertial ? "1" : "N/A"),
            "Pose export changed direction, precision, quaternion order or IMU context");
    require(rows[3].at("pose_enabled") == "0" && rows[3].at("pose_valid") == "0" &&
            rows[3].at("pose_capture_steady_ns") == "N/A" &&
            rows[3].at("map_change_index_valid") == "0" && rows[3].at("map_change_index") == "N/A" &&
            rows[3].at("imu_initialized_valid") == "0" && rows[3].at("imu_initialized") == "N/A",
            "Disabled pose collection leaked context");
    const std::string summary = read(directory / "summary.csv");
    for (const char *expected : {"pose_collection_frames,5,all_processed",
         "pose_valid_samples,3,all_processed", "pose_invalid_samples,2,pose_collection_frames_all_processed",
         "pose_disabled_or_unavailable_frames,1,all_processed"})
        require(summary.find(expected) != std::string::npos, "Overflow lost pose aggregates");
    const std::string json = read(directory / "metadata.json");
    require(json.find("\"pose_enabled\": true") != std::string::npos &&
            json.find("Twc: left camera to tracking-map world") != std::string::npos &&
            json.find("\"final_optimized_pose\"") != std::string::npos &&
            json.find("\"pose\"") == std::string::npos,
            "Metadata omitted pose availability, convention or live/final distinction");
}

void test_image_intervals()
{
    gemini336_orbslam3::ImageTimestampStats stats;
    reject_allocation = true;
    for (const int64_t timestamp : {1000000000LL, 1030000000LL, 1030000000LL,
                                    1020000000LL, 1220000000LL})
        stats.observe(timestamp, 0.1);
    reject_allocation = false;
    require(stats.received == 5 && stats.positive_intervals == 2 && stats.duplicates == 1 &&
            stats.backwards == 1 && stats.excessive_gaps == 1,
            "raw intervals mixed admission or rejection semantics");
    require(stats.interval_min_sec > 0.029 && stats.interval_min_sec < 0.031 &&
            stats.interval_max_sec > 0.199 && stats.interval_max_sec < 0.201,
            "interval units or clock-reset accounting changed");
}
}

// This single-threaded test has no ROS, backend, sensor or executor lifecycle.
void *operator new(std::size_t size)
{
    if (reject_allocation) throw std::bad_alloc();
    if (void *memory = std::malloc(size ? size : 1)) return memory;
    throw std::bad_alloc();
}
void operator delete(void *memory) noexcept { std::free(memory); }
void operator delete(void *memory, std::size_t) noexcept { std::free(memory); }
void *operator new[](std::size_t size) { return ::operator new(size); }
void operator delete[](void *memory) noexcept { ::operator delete(memory); }
void operator delete[](void *memory, std::size_t) noexcept { ::operator delete(memory); }

int main()
{
    const auto directory = std::filesystem::temp_directory_path() /
        ("gemini336-evaluation-test-" + std::to_string(getpid()));
    int result = 0;
    try
    {
        std::filesystem::create_directory(directory);
        test_overflow_and_export(directory);
        test_empty_and_failure(directory);
        test_tracking_statistics(directory);
        test_pose_export(directory / "pose_stereo", false);
        test_pose_export(directory / "pose_imu", true);
        test_image_intervals();
        std::cout << "Evaluation allocation, overflow, integrity and export tests passed\n";
    }
    catch (const std::exception &error)
    {
        reject_allocation = false;
        std::cerr << error.what() << '\n';
        result = 1;
    }
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    return result;
}
