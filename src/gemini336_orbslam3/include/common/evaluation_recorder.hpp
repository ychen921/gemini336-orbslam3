#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include "common/tracking_statistics.hpp"

namespace gemini336_orbslam3
{
inline constexpr int EvaluationSchemaVersion = 2;
// Raw image callback intervals, independent of synchronization and frame drops.
// One reception group owns updates; snapshots require quiescent callbacks.
struct ImageTimestampStats
{
    uint64_t received = 0;
    uint64_t positive_intervals = 0;
    uint64_t duplicates = 0;
    uint64_t backwards = 0;
    uint64_t excessive_gaps = 0;
    std::optional<int64_t> previous_ns;
    double interval_sum_sec = 0.0;
    double interval_min_sec = 0.0;
    double interval_max_sec = 0.0;

    void observe(int64_t timestamp_ns, double max_gap_sec) noexcept;
};

// Fixed-size numerical payload only: never retain images or backend pointers.
// Queue occupancy is sampled before the corresponding adapter call.
struct EvaluationFrame
{
    int64_t sensor_timestamp_ns = 0;
    uint64_t enqueue_sequence = 0;
    uint64_t processed_index = 0;
    int64_t enqueue_steady_ns = 0;
    int64_t adapter_start_steady_ns = 0;
    int64_t adapter_return_steady_ns = 0;
    int32_t raw_tracking_state = 0;
    uint64_t pending = 0;
    uint64_t in_flight = 0;
    uint64_t outstanding = 0;
    uint64_t imu_samples = 0;
    TrackingStatistics tracking;
};

struct EvaluationMetadata
{
    std::string sensor_mode;
    std::string run_id;
    std::string bag_id;
    std::string repository_commit;
    std::string repository_dirty;
    std::string settings_id;
    std::string vocabulary_id;
    std::string core_commit;
    std::string core_dirty;
    // Preserve resolved ROS values as text, including topic/QoS/gap configuration.
    std::vector<std::pair<std::string, std::string>> parameters;
};

// A serialized tracking consumer is the sole writer. Reserve at startup, never
// format, allocate, lock or perform I/O in record(). Export only after it stops.
class EvaluationRecorder
{
public:
    explicit EvaluationRecorder(std::size_t capacity);
    EvaluationRecorder(const EvaluationRecorder &) = delete;
    EvaluationRecorder &operator=(const EvaluationRecorder &) = delete;
    void record(const EvaluationFrame &frame, int64_t first_enqueue_steady_ns) noexcept;
    std::size_t recorded() const noexcept { return frames_.size(); }
    uint64_t dropped() const noexcept { return dropped_; }
    std::size_t capacity() const noexcept { return capacity_; }
    void write(const std::filesystem::path &directory, const EvaluationMetadata &metadata) const;

private:
    const std::size_t capacity_;
    std::vector<EvaluationFrame> frames_;
    uint64_t dropped_ = 0;
    uint64_t processed_ = 0;
    // Core states -1..5 have distinct buckets; unknown raw values share the last.
    std::array<uint64_t, 8> state_counts_{};
    bool tracking_started_ = false;
    uint64_t startup_frames_ = 0;
    uint64_t after_first_ok_frames_ = 0;
    uint64_t lost_frames_ = 0;
    uint64_t recently_lost_frames_ = 0;
    uint64_t snapshot_frames_ = 0;
    uint64_t inlier_samples_ = 0;
    uint64_t inlier_sum_ = 0;
    uint32_t inlier_min_ = 0;
    uint32_t inlier_max_ = 0;
    uint64_t tracking_lost_events_ = 0;
    uint64_t relocalization_events_ = 0;
    std::optional<uint64_t> last_keyframes_inserted_;
    double adapter_sum_ms_ = 0.0;
    double adapter_max_ms_ = 0.0;
    double enqueue_to_return_sum_ms_ = 0.0;
    double pending_sum_ = 0.0;
    double outstanding_sum_ = 0.0;
    uint64_t outstanding_sample_peak_ = 0;
    int64_t first_enqueue_steady_ns_ = 0;
    int64_t last_return_steady_ns_ = 0;
    int64_t first_sensor_ns_ = 0;
    int64_t last_sensor_ns_ = 0;
};

// Independent post-join file. Atomic rename certifies this optional artifact;
// export_complete.txt continues to describe the earlier callback-data export.
void write_evaluation_backend_summary(const std::filesystem::path &directory,
                                      const EvaluationBackendSummary &summary);
}
