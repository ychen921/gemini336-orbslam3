#pragma once

#include "gemini336_orbslam3/logging.hpp"
#include "slam/orbslam3_adapter.hpp"
#include "frontend/imu_frontend.hpp"
#include "common/callback_guard.hpp"
#include <chrono>
#include <deque>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>

namespace gemini336_orbslam3
{
// One serialized tracking consumer owns stages and IMU batches. Admission and
// snapshots share the mutex protecting work locations and accounting. Teardown
// requires executor callback quiescence; backend, IMU and notifications run unlocked.
class TrackingCoordinator
{
public:
    struct Config
    {
        TrackingMode mode = TrackingMode::Stereo;
        std::size_t capacity = 30;
        double imu_wait_timeout_sec = 1.0;
    };
    TrackingCoordinator(Config config, std::shared_ptr<StopControl> stop_control,
                        std::shared_ptr<spdlog::logger> logger, DiagnosticTrace *trace,
                        OrbSlam3Adapter *adapter, ImuFrontend *imu,
                        std::function<void(double)> first_frame_notification = {},
                        std::function<void(TrackingState, TrackingState)> state_notification = {});
    TrackingCoordinator(const TrackingCoordinator &) = delete;
    TrackingCoordinator &operator=(const TrackingCoordinator &) = delete;
    using Clock = std::chrono::steady_clock;

private:
    struct PendingFrame
    {
        StereoFrame frame;
        Clock::time_point received_at;
        uint64_t enqueue_sequence = 0;
    };

    enum class TrackingWorkStage
    {
        Reserved,
        Ready,
        Executing
    };

    enum class TrackingWorkSource
    {
        PrimaryReservation,
        StartupNextReservation
    };

    struct TrackingWork
    {
        PendingFrame pending;
        TrackingWorkStage stage = TrackingWorkStage::Reserved;

        // Keep consumed IMU data with its frame so retries cannot take it again.
        std::optional<ImuBatch> imu_batch;
    };

public:
    // Queue-locked snapshots contain accounting and timing values only.
    struct QueueSnapshot
    {
        std::size_t pending = 0;
        std::size_t in_flight = 0;
        std::size_t outstanding = 0;
        // Legacy pending_peak logs report capacity usage, including reservations.
        std::size_t peak = 0;
        std::size_t capacity = 0;
        uint64_t enqueued = 0;
        uint64_t processed = 0;
        uint64_t startup_discarded = 0;
        uint64_t overload_discarded = 0;
        std::optional<Clock::time_point> startup_started;
        std::optional<Clock::time_point> oldest_received_at;
        bool startup_complete = false;
        std::size_t queued = 0;
        bool accounting_valid = false;
    };

    using FinalSnapshot = QueueSnapshot;
    QueueSnapshot snapshot() const;
    void report(bool final);
    void process_pending_frames(StopReason &failure_reason);
    void release_unfinished_work();
    void report_final_accounting(const FinalSnapshot &final);
    void enqueue(const StereoFrame &frame);
private:
#ifdef GEMINI336_QUEUE_TEST
    friend class SlamNode;
    friend struct SlamNodeQueueTestAccess;
    friend struct SlamTrackingTestAccess;
    friend struct SlamExecutorTestAccess;
    friend struct SlamSnapshotTestAccess;
    std::function<void(const StereoFrame &, const std::vector<ImuMeasurement> &)> test_track_;
    enum class TestWorkPoint { AfterWaiting, AfterReady, BeforeBackend, BeforeBackendPermit, AfterBackendPermit, BeforeImuQuery };
    std::function<void(TestWorkPoint)> test_work_point_;
#endif
    QueueSnapshot snapshot_locked() const;

    struct Statistics
    {
        uint64_t frames = 0;
        double track_sum_ms = 0.0;
        double track_max_ms = 0.0;
        uint64_t intervals = 0;
        double interval_sum_ms = 0.0;
        double interval_min_ms = std::numeric_limits<double>::infinity();
        double interval_max_ms = 0.0;
    };

    enum class QueueFullPolicy
    {
        Reject,
        DiscardOldestQueued
    };

    bool reserve_tracking_work();
    bool reserve_startup_pair();
    void discard_startup_first();
    void log_statistics(const char *scope, const Statistics &stats, double elapsed,
                        uint64_t processed);
    void check_imu_timeline_and_deadlines() const;
    bool prepare_imu_work(bool startup);
    void execute_work(TrackingWorkSource work_source);
    [[noreturn]] void throw_batch_error(const char *reason, double left, double right) const;
    [[noreturn]] void throw_wait_timeout(const char *scope, double waited_sec) const;
    void enqueue_frame(
        const StereoFrame &frame, QueueFullPolicy full_policy = QueueFullPolicy::Reject);
    std::shared_ptr<StopControl> stop_control_;
    std::shared_ptr<spdlog::logger> node_logger_;
    DiagnosticTrace *trace_;
    OrbSlam3Adapter *slam_;
    ImuFrontend *imu_frontend_;
    std::function<void(double)> first_frame_notification_;
    std::function<void(TrackingState, TrackingState)> state_notification_;
    // Fixed after construction; only finite tests override configuration.
    TrackingMode tracking_mode_;
    double imu_wait_timeout_sec_;
    std::size_t pending_frames_capacity_;

    // The mutex protects work locations, admission timestamps, startup state and
    // all accounting. Only the consumer changes each work's stage and IMU batch.
    mutable std::mutex queue_mutex_;
    std::deque<PendingFrame> pending_frames_;
    std::optional<TrackingWork> tracking_work_;
    std::optional<TrackingWork> startup_next_work_;
    std::optional<double> last_received_frame_timestamp_;
    std::optional<double> last_tracked_frame_timestamp_;
    std::optional<Clock::time_point> startup_wait_started_;
    bool startup_complete_ = false;
    uint64_t enqueued_frames_ = 0;
    uint64_t processed_frames_ = 0;
    uint64_t startup_discarded_frames_ = 0;
    uint64_t overload_discarded_frames_ = 0;
    std::size_t outstanding_frames_peak_ = 0;

    // Serialized consumer state remains terminal after any processing exception.
    StopReason tracking_callback_reason_ = StopReason::CallbackError;
    bool tracking_failed_ = false;

    // Sensor timestamps measure spacing; steady-clock times measure throughput.
    double previous_timestamp_ = 0.0;
    Statistics window_;
    Statistics total_;
    Clock::time_point started_;
    Clock::time_point last_report_;
    double enqueue_to_return_sum_ms_ = 0.0;
    double enqueue_to_return_max_ms_ = 0.0;
};
}
