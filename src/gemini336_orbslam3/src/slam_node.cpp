#include "gemini336_orbslam3/logging.hpp"
#include "slam/orbslam3_adapter.hpp"
#include "frontend/stereo_frontend.hpp"
#include "frontend/imu_frontend.hpp"

#include <cstdint>
#include <ctime>
#include <fstream>
#include <sys/resource.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <optional>
#include <utility>
#include <limits>
#include <exception>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstddef>
#include <deque>
#include <sstream>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>

namespace gemini336_orbslam3
{
namespace
{
const char *tracking_state_name(TrackingState state)
{
    switch (state)
    {
    case TrackingState::SystemNotReady: return "SystemNotReady";
    case TrackingState::NoImagesYet: return "NoImagesYet";
    case TrackingState::NotInitialized: return "NotInitialized";
    case TrackingState::Ok: return "Ok";
    case TrackingState::RecentlyLost: return "RecentlyLost";
    case TrackingState::Lost: return "Lost";
    case TrackingState::OkKlt: return "OkKlt";
    case TrackingState::Unknown: return "Unknown";
    }
    return "Unknown";
}
}

class SlamNode : public rclcpp::Node
{
public:
    explicit SlamNode(std::function<void()> request_stop)
        : Node("slam_node"), request_stop_(std::move(request_stop))
    {
        // These settings define component lifetimes and are fixed at startup.
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;

        // Establish the session before constructing any sensor or SLAM component.
        LoggingOptions logging_options;
        logging_options.directory = declare_parameter<std::string>(
            "logging.directory", "", descriptor);
        logging_options.level = declare_parameter<std::string>(
            "logging.level", "info", descriptor);
        logging_ = std::make_unique<LoggingSession>(logging_options);
        node_logger_ = logging_->GetLogger("slam_node");
        diagnostics_logger_ = logging_->GetLogger("tracking_diagnostics");
        tracking_diagnostics_enabled_ = declare_parameter<bool>(
            "diagnostics.tracking_timing", false, descriptor);

        slow_tracking_enabled_ = declare_parameter<bool>(
            "diagnostics.slow_tracking", false, descriptor);
        slow_tracking_threshold_ms_ = declare_parameter<double>(
            "diagnostics.slow_tracking_threshold_ms", 50.0, descriptor);
        if (!std::isfinite(slow_tracking_threshold_ms_) || slow_tracking_threshold_ms_ <= 0.0)
            throw std::invalid_argument("diagnostics.slow_tracking_threshold_ms must be finite and positive");
        // Slow-call diagnostics need the periodic summary even without tracking_timing.
        tracking_diagnostics_enabled_ = tracking_diagnostics_enabled_ || slow_tracking_enabled_;

        // Diagnostics are opt-in and do not change scheduling or sensor policy.
        const std::string trace_path = declare_parameter<std::string>(
            "diagnostics.trace_path", "", descriptor);
        const int64_t trace_capacity = declare_parameter<int64_t>(
            "diagnostics.trace_capacity", 300000, descriptor);
        if (trace_capacity <= 0)
            throw std::invalid_argument("diagnostics.trace_capacity must be positive");
        if (!trace_path.empty())
        {
            require_absolute_path(trace_path, "diagnostics.trace_path");
            if (std::filesystem::exists(trace_path))
                throw std::invalid_argument("diagnostics.trace_path already exists");
            trace_ = std::make_unique<DiagnosticTrace>(trace_path, trace_capacity);
        }

        input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 5.0, descriptor);
        input_timeout_action_ = declare_parameter<std::string>(
            "input_timeout_action", "shutdown", descriptor);
        if (!std::isfinite(input_timeout_sec_) || input_timeout_sec_ < 0.0)
            throw std::invalid_argument("input_timeout_sec must be finite and nonnegative");
        if (input_timeout_action_ != "shutdown" && input_timeout_action_ != "warn")
            throw std::invalid_argument("input_timeout_action must be shutdown or warn");

        // Validate SLAM resources before constructing the backend and subscribers.
        OrbSlam3Config config;
        config.vocabulary_path = declare_parameter<std::string>(
            "vocabulary_path", ORB_SLAM3_DEFAULT_VOCABULARY_PATH, descriptor);
        config.settings_path = declare_parameter<std::string>(
            "settings_path", "", descriptor);
        config.enable_viewer = declare_parameter<bool>(
            "enable_viewer", false, descriptor);

        require_absolute_path(config.vocabulary_path, "vocabulary_path");
        require_absolute_path(config.settings_path, "settings_path");

        const auto left_topic = declare_parameter<std::string>(
            "left_image_topic", "/camera/left_ir/image_raw", descriptor);
        const auto right_topic = declare_parameter<std::string>(
            "right_image_topic", "/camera/right_ir/image_raw", descriptor);
        if (left_topic.empty() || right_topic.empty())
            throw std::invalid_argument("Image topics must not be empty");

        const std::string sensor_mode =
            declare_parameter<std::string>("sensor_mode", "stereo", descriptor);
        if (sensor_mode == "stereo")
        {
            tracking_mode_ = TrackingMode::Stereo;
        }
        else if (sensor_mode == "stereo_imu")
        {
            tracking_mode_ = TrackingMode::StereoImu;

            imu_topic_ = declare_parameter<std::string>(
                "imu_topic", "/camera/gyro_accel/sample", descriptor);
            if (imu_topic_.empty())
                throw std::invalid_argument("imu_topic must not be empty");

            const int64_t pending_capacity = declare_parameter<int64_t>(
                "stereo_imu.pending_frame_capacity", 30, descriptor);
            if (pending_capacity <= 0)
                throw std::invalid_argument("stereo_imu.pending_frame_capacity must be positive");
            pending_frames_capacity_ =
                static_cast<std::size_t>(pending_capacity);

            const double imu_wait_timeout_sec = declare_parameter<double>(
                "stereo_imu.wait_timeout_sec", 1.0, descriptor);
            if (!std::isfinite(imu_wait_timeout_sec) || imu_wait_timeout_sec <= 0.0)
                throw std::invalid_argument("stereo_imu.wait_timeout_sec must be finite and positive");
            imu_wait_timeout_sec_ = imu_wait_timeout_sec;

            const int64_t imu_retry_period_ms = declare_parameter<int64_t>(
                "stereo_imu.retry_period_ms", 5, descriptor);
            if (imu_retry_period_ms <= 0)
                throw std::invalid_argument("stereo_imu.retry_period_ms must be positive");
            imu_retry_period_ms_ = imu_retry_period_ms;
        }
        else
        {
            throw std::invalid_argument("sensor_mode must be stereo or stereo_imu");
        }

        node_logger_->info("Vocabulary: {}", config.vocabulary_path.c_str());
        node_logger_->info("Settings: {}", config.settings_path.c_str());
        node_logger_->info("Viewer: {}", config.enable_viewer ? "enabled" : "disabled");

        // Construct SLAM before accepting frames through the frontend.
        config.tracking_mode = tracking_mode_;
        slam_ = std::make_unique<OrbSlam3Adapter>(config);

        // Construct IMU frontend before stereo frontend.
        if (tracking_mode_ == TrackingMode::StereoImu)
        {
            imu_frontend_ = std::make_unique<ImuFrontend>(
                this, imu_topic_, trace_.get());
            imu_retry_timer_ = create_wall_timer(
                std::chrono::milliseconds(imu_retry_period_ms_),
                [this]() {process_pending_frames(); });
        }

        // Construct Stereo frontend
        stereo_frontend_ = std::make_unique<StereoFrontend>(
            this, left_topic, right_topic,
            [this](const StereoFrame &frame) { on_frame(frame); },
            [this]() { on_input_activity(); }, trace_.get());

        // Wall-clock timers remain independent of sensor timestamps and simulated time.
        started_ = last_report_ = Clock::now();
        if (tracking_diagnostics_enabled_)
        {
            diagnostics_last_report_ = started_;
            diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1),
                [this]() { report_tracking_diagnostics(false); });
        }
        report_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() { report(false); });
        if (input_timeout_sec_ > 0.0)
            input_timer_ = create_wall_timer(
                std::chrono::milliseconds(100), [this]() { check_input_timeout(); });

        node_logger_->info("Input timeout: seconds={:.3f} action={} (armed after first image)",
                    input_timeout_sec_, input_timeout_action_.c_str());
        node_logger_->info("Stereo SLAM initialized: left={} right={}",
                    left_topic.c_str(), right_topic.c_str());
        RCLCPP_INFO(get_logger(), "Stereo SLAM initialized: left=%s right=%s",
                    left_topic.c_str(), right_topic.c_str());
    }

    // main retains node ownership while reporting spin and teardown failures.
    void log_failure(const std::string &message)
    {
        node_logger_->error("{}", message);
    }

    void shutdown()
    {
        // Spin has returned, so no image callback can overlap subscription teardown.
        stop_requested_ = true;
        diagnostics_timer_.reset();
        // Inspect coverage before destroying the frontend or clearing pending frames.
        if (tracking_diagnostics_enabled_)
            report_tracking_diagnostics(true);
        input_timer_.reset();
        report_timer_.reset();
        imu_retry_timer_.reset();
        stereo_frontend_.reset();
        if (imu_frontend_)
        {
            const ImuFrontendStats stats = imu_frontend_->stats();
            node_logger_->info("Final IMU input: received={} accepted={} backwards={} "
                        "overflow={} buffered={}",
                        static_cast<unsigned long long>(stats.received),
                        static_cast<unsigned long long>(stats.accepted),
                        static_cast<unsigned long long>(stats.backwards),
                        static_cast<unsigned long long>(stats.overflow), stats.buffered);
        }
        imu_frontend_.reset();

        // Emit the final statistics while the backend is still available.
        report(true);
        const QueueSnapshot queue = queue_snapshot();
        node_logger_->info("Stereo input stopped; processed={} last_state={} remaining_frames={}",
                    static_cast<unsigned long long>(queue.processed),
                    tracking_state_name(slam_->trackingState()),
                    queue.pending);
        RCLCPP_INFO(get_logger(),
                    "Stereo input stopped; processed=%llu last_state=%s remaining_frames=%zu",
                    static_cast<unsigned long long>(queue.processed),
                    tracking_state_name(slam_->trackingState()),
                    queue.pending);

        // Flush before backend shutdown so an upstream shutdown stall cannot
        // hide callback history. Diagnostic I/O errors do not skip SLAM cleanup.
        if (trace_)
        {
            try { trace_->write(); }
            catch (const std::exception &error)
            {
                node_logger_->error("Diagnostic trace write failed: {}", error.what());
                RCLCPP_ERROR(get_logger(), "Diagnostic trace write failed: %s", error.what());
            }
        }
        {
            const std::lock_guard<std::mutex> lock(queue_mutex_);
            pending_frames_.clear();
        }
        slam_->shutdown();
        node_logger_->info("Stereo SLAM shutdown returned");
        RCLCPP_INFO(get_logger(), "Stereo SLAM shutdown returned");
    }

private:
#ifdef GEMINI336_QUEUE_TEST
    // Test-only construction exercises the real queue without sensors or a backend.
    friend struct SlamNodeQueueTestAccess;
    struct QueueTestTag {};
    explicit SlamNode(QueueTestTag) : Node("slam_queue_test") {}
#endif
    using Clock = std::chrono::steady_clock;

    struct PendingFrame
    {
        StereoFrame frame;
        Clock::time_point received_at;
    };

    enum class TrackingWorkStage
    {
        Reserved,
        Ready,
        Executing
    };

    struct TrackingWork
    {
        PendingFrame pending;
        TrackingWorkStage stage = TrackingWorkStage::Reserved;

        // Keep consumed IMU data with its frame so retries cannot take it again.
        std::optional<ImuBatch> imu_batch;
    };

    struct ReservationSummary
    {
        double timestamp = 0.0;
        Clock::time_point received_at;
        TrackingWorkStage stage = TrackingWorkStage::Reserved;
        bool imu_batch_consumed = false;
    };

    // Value copies retain pixels independently of deque elements. This is not yet
    // the B2 reservation state machine; the sole consumer still pops after tracking.
    struct QueueSnapshot
    {
        std::size_t pending = 0;
        std::size_t in_flight = 0;
        std::size_t outstanding = 0;
        std::size_t peak = 0;
        uint64_t enqueued = 0;
        uint64_t processed = 0;
        uint64_t startup_discarded = 0;
        std::optional<PendingFrame> first;
        std::optional<PendingFrame> second;
        std::optional<ReservationSummary> reservation;
        std::optional<ReservationSummary> startup_next_reservation;
        std::optional<double> last_tracked;
        std::optional<Clock::time_point> startup_started;
        std::optional<Clock::time_point> oldest_received_at;
    };

    QueueSnapshot queue_snapshot() const
    {
        const std::lock_guard<std::mutex> lock(queue_mutex_);
        QueueSnapshot snapshot;
        snapshot.pending = pending_frames_.size();
        snapshot.in_flight =
            (reservation_.has_value() ? 1U : 0U) +
            (startup_next_reservation_.has_value() ? 1U : 0U);
        snapshot.outstanding = snapshot.pending + snapshot.in_flight;
        snapshot.peak = pending_frames_peak_;
        snapshot.enqueued = enqueued_frames_;
        snapshot.processed = processed_frames_;
        snapshot.startup_discarded = startup_discarded_frames_;
        if (!pending_frames_.empty()) snapshot.first = pending_frames_[0];
        if (pending_frames_.size() >= 2) snapshot.second = pending_frames_[1];
        snapshot.last_tracked = last_tracked_frame_timestamp_;
        snapshot.startup_started = startup_wait_started_;
        snapshot.reservation = reservation_;
        snapshot.startup_next_reservation = startup_next_reservation_;

        // Outstanding work includes both queued and reserved frames.
        if (snapshot.first)
            snapshot.oldest_received_at = snapshot.first->received_at;

        if (snapshot.reservation &&
            (!snapshot.oldest_received_at ||
            snapshot.reservation->received_at < *snapshot.oldest_received_at))
        {
            snapshot.oldest_received_at = snapshot.reservation->received_at;
        }

        if (snapshot.startup_next_reservation &&
            (!snapshot.oldest_received_at ||
            snapshot.startup_next_reservation->received_at < *snapshot.oldest_received_at))
        {
            snapshot.oldest_received_at = snapshot.startup_next_reservation->received_at;
        }

        return snapshot;
    }

    bool reserve_tracking_work()
    {
        // A waiting frame remains owned across retries.
        if (tracking_work_)
            return true;

        const std::lock_guard<std::mutex> lock(queue_mutex_);
        if (pending_frames_.empty())
            return false;

        // Retain the frame and its original deadline before removing the queue entry.
        tracking_work_.emplace(TrackingWork{
            pending_frames_.front(),
            TrackingWorkStage::Reserved,
            std::nullopt
        });

        reservation_.emplace(ReservationSummary{
            tracking_work_->pending.frame.timestamp,
            tracking_work_->pending.received_at,
            TrackingWorkStage::Reserved,
            false
        });

        pending_frames_.pop_front();
        return true;
    }

    void on_input_activity()
    {
        if (input_timeout_sec_ == 0.0 || stop_requested_)
            return;

        // Either raw image stream counts as activity, even without a valid stereo pair.
        last_input_activity_ = Clock::now();
        if (input_timeout_reported_)
        {
            node_logger_->info("Image input resumed after timeout");
            RCLCPP_INFO(get_logger(), "Image input resumed after timeout");
        }
        input_timeout_reported_ = false;
    }

    void check_input_timeout()
    {
        if (!last_input_activity_ || input_timeout_reported_ || stop_requested_)
            return;

        // The timeout is armed only after input activity has established a baseline.
        const double idle_sec =
            std::chrono::duration<double>(Clock::now() - *last_input_activity_).count();
        if (idle_sec < input_timeout_sec_)
            return;

        input_timeout_reported_ = true;
        node_logger_->warn("Image input timeout: idle_sec={:.3f} threshold_sec={:.3f} action={}",
                    idle_sec, input_timeout_sec_, input_timeout_action_.c_str());
        RCLCPP_WARN(get_logger(), "Image input timeout: idle_sec=%.3f threshold_sec=%.3f action=%s",
                    idle_sec, input_timeout_sec_, input_timeout_action_.c_str());

        if (input_timeout_action_ == "shutdown")
        {
            // Cancel spin only; main owns teardown after the current callback returns.
            stop_requested_ = true;
            input_timer_->cancel();
            request_stop_();
        }
    }

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

    void log_statistics(const char *scope, const Statistics &stats, double elapsed,
                        uint64_t processed)
    {
        // Timing diagnostics already provide the periodic INFO summary.
        node_logger_->log(tracking_diagnostics_enabled_ && std::string(scope) == "window" ?
                        spdlog::level::debug : spdlog::level::info, "Stereo stats: scope={} frames={} total={} elapsed_sec={:.6f} rate_hz={:.6f} "
                    "track_mean_ms={:.6f} track_max_ms={:.6f} intervals={} "
                    "interval_min_ms={:.6f} interval_mean_ms={:.6f} interval_max_ms={:.6f}",
                    scope, static_cast<unsigned long long>(stats.frames),
                    static_cast<unsigned long long>(processed), elapsed,
                    elapsed > 0.0 ? stats.frames / elapsed : 0.0,
                    stats.frames ? stats.track_sum_ms / stats.frames : 0.0, stats.track_max_ms,
                    static_cast<unsigned long long>(stats.intervals),
                    stats.intervals ? stats.interval_min_ms : 0.0,
                    stats.intervals ? stats.interval_sum_ms / stats.intervals : 0.0,
                    stats.interval_max_ms);
    }

    void report(bool final)
    {
        const QueueSnapshot queue = queue_snapshot();
        const auto now = Clock::now();

        log_statistics(final ? "tail" : "window", window_,
                       std::chrono::duration<double>(now - last_report_).count(), queue.processed);
        if (final)
            log_statistics("total", total_, std::chrono::duration<double>(now - started_).count(),
                           queue.processed);

        if (tracking_mode_ == TrackingMode::StereoImu)
        {
            const double oldest_wait_sec = queue.oldest_received_at ?
                std::chrono::duration<double>(now - *queue.oldest_received_at).count() :
                0.0;
            node_logger_->log(tracking_diagnostics_enabled_ && !final ?
                            spdlog::level::debug : spdlog::level::info, "Stereo-IMU coordination: final={} enqueued={} processed={} "
                        "startup_discarded={} pending={} pending_peak={} oldest_wait_sec={:.6f} "
                        "enqueue_to_return_mean_ms={:.6f} enqueue_to_return_max_ms={:.6f}",
                        final ? "true" : "false",
                        static_cast<unsigned long long>(queue.enqueued),
                        static_cast<unsigned long long>(queue.processed),
                        static_cast<unsigned long long>(queue.startup_discarded),
                        queue.pending, queue.peak, oldest_wait_sec,
                        queue.processed ? enqueue_to_return_sum_ms_ / queue.processed : 0.0,
                        enqueue_to_return_max_ms_);
        }

        // Reset window aggregates without losing the timestamp between adjacent frames.
        window_ = Statistics{};
        last_report_ = now;
    }

    struct TrackingDiagnostics
    {
        uint64_t calls = 0;
        uint64_t cpu_samples = 0;
        double wall_sum_ms = 0.0;
        double wall_max_ms = 0.0;
        double cpu_sum_ms = 0.0;
        double non_cpu_sum_ms = 0.0;
    };

    void report_tracking_diagnostics(bool final)
    {
        const TrackingDiagnostics stats = tracking_diagnostics_;
        const QueueSnapshot queue = queue_snapshot();
        const Clock::time_point now = Clock::now();
        const double elapsed = std::chrono::duration<double>(now - diagnostics_last_report_).count();
        const double oldest_ms = queue.oldest_received_at ?
            std::chrono::duration<double, std::milli>(
                now - *queue.oldest_received_at).count() :
            0.0;
        diagnostics_logger_->info(
            "TRACK_TIMING final={} steady_ns={} window_sec={:.6f} calls={} rate_hz={:.3f} "
            "wall_mean_ms={:.3f} wall_max_ms={:.3f} cpu_samples={} cpu_mean_ms={:.3f} "
            "non_cpu_mean_ms={:.3f} pending={} pending_peak={} oldest_queue_ms={:.3f} log_dropped={}",
            final, std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count(),
            elapsed, stats.calls, elapsed > 0.0 ? stats.calls / elapsed : 0.0,
            stats.calls ? stats.wall_sum_ms / stats.calls : 0.0, stats.wall_max_ms,
            stats.cpu_samples, stats.cpu_samples ? stats.cpu_sum_ms / stats.cpu_samples : 0.0,
            stats.cpu_samples ? stats.non_cpu_sum_ms / stats.cpu_samples : 0.0,
            queue.pending, queue.peak, oldest_ms, logging_->dropped_messages());

        // Query the same interval as tracking without consuming IMU or adding trace events.
        // Seconds below are existing backend boundaries, not reconstructed raw nanoseconds.
        if (final && imu_frontend_ && queue.first)
        {
            const bool have_interval = queue.last_tracked.has_value() || queue.pending >= 2;
            const double left = queue.last_tracked.value_or(queue.first->frame.timestamp);
            const double right = queue.last_tracked ? queue.first->frame.timestamp :
                (queue.second ? queue.second->frame.timestamp : queue.first->frame.timestamp);
            const char *coverage = "AwaitingSecondFrame";
            if (have_interval)
            {
                switch (imu_frontend_->inspectMeasurements(left, right))
                {
                case ImuBatchStatus::Ready: coverage = "Ready"; break;
                case ImuBatchStatus::WaitingForData: coverage = "WaitingForData"; break;
                case ImuBatchStatus::MissingHistory: coverage = "MissingHistory"; break;
                case ImuBatchStatus::BufferOverflow: coverage = "BufferOverflow"; break;
                case ImuBatchStatus::DataGap: coverage = "DataGap"; break;
                case ImuBatchStatus::InvalidRequest: coverage = "InvalidRequest"; break;
                }
            }
            diagnostics_logger_->info(
                "STOP_IMU_COVERAGE steady_ns={} interval_left_sec={:.17g} interval_right_sec={:.17g} "
                "status={} buffered={}",
                std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count(),
                left, right, coverage, imu_frontend_->stats().buffered);
        }
        if (slow_tracking_enabled_)
        {
            diagnostics_logger_->info(
                "SCHEDULER_SUMMARY final={} calls={} scheduler_unavailable={} slow_calls={} suppressed={} "
                "probe_mean_ms={:.6f} probe_max_ms={:.6f} threshold_ms={:.3f}",
                final, scheduler_calls_, scheduler_unavailable_, slow_calls_, slow_suppressed_,
                scheduler_calls_ ? probe_sum_ms_ / scheduler_calls_ : 0.0, probe_max_ms_, slow_tracking_threshold_ms_);
            scheduler_calls_ = scheduler_unavailable_ = slow_calls_ = slow_suppressed_ = 0;
            probe_sum_ms_ = probe_max_ms_ = 0.0;
        }
        tracking_diagnostics_ = TrackingDiagnostics{};
        diagnostics_last_report_ = now;
    }

    struct SchedulerSnapshot
    {
        uint64_t wait_ns = 0;
        long voluntary = 0;
        long involuntary = 0;
        bool scheduler_valid = false;
        bool switches_valid = false;
    };

    SchedulerSnapshot scheduler_snapshot() const
    {
        SchedulerSnapshot sample;
        // Read the calling thread, not the process leader. Disabled schedstats can
        // expose zero/stale counters, so readable counters alone are insufficient.
        int enabled = 0;
        std::ifstream enabled_file("/proc/sys/kernel/sched_schedstats");
        uint64_t runtime_ns = 0, slices = 0;
        std::ifstream stats_file("/proc/thread-self/schedstat");
        sample.scheduler_valid = static_cast<bool>(enabled_file >> enabled) && enabled == 1 &&
            static_cast<bool>(stats_file >> runtime_ns >> sample.wait_ns >> slices);
        rusage usage{};
        sample.switches_valid = getrusage(RUSAGE_THREAD, &usage) == 0;
        if (sample.switches_valid)
        {
            sample.voluntary = usage.ru_nvcsw;
            sample.involuntary = usage.ru_nivcsw;
        }
        return sample;
    }

    void log_slow_tracking(const SchedulerSnapshot &before, const SchedulerSnapshot &after,
                           Clock::time_point start, Clock::time_point end,
                           double cpu_ms, double probe_ms, double queue_before_ms,
                           std::size_t pending_before, std::size_t pending_after)
    {
        ++scheduler_calls_;
        probe_sum_ms_ += probe_ms;
        probe_max_ms_ = std::max(probe_max_ms_, probe_ms);
        const bool scheduler_valid = before.scheduler_valid && after.scheduler_valid &&
            after.wait_ns >= before.wait_ns;
        if (!scheduler_valid) ++scheduler_unavailable_;
        const double wall_ms = std::chrono::duration<double, std::milli>(end - start).count();
        if (wall_ms < slow_tracking_threshold_ms_) return;
        ++slow_calls_;
        // At most one detailed record per second; summaries retain omitted counts.
        if (last_slow_report_ && end - *last_slow_report_ < std::chrono::seconds(1))
        {
            ++slow_suppressed_;
            return;
        }
        last_slow_report_ = end;
        const bool switches_valid = before.switches_valid && after.switches_valid &&
            after.voluntary >= before.voluntary && after.involuntary >= before.involuntary;
        const double wait_ms = scheduler_valid ? (after.wait_ns - before.wait_ns) * 1e-6 : -1.0;
        // Snapshot boundaries include probe skew. Keep the signed residual, which
        // is only an estimate of other blocking, never proof of a particular lock.
        const double residual_ms = scheduler_valid && cpu_ms >= 0.0 ? wall_ms - cpu_ms - wait_ms :
            std::numeric_limits<double>::quiet_NaN();
        diagnostics_logger_->info(
            "SLOW_TRACK start_steady_ns={} end_steady_ns={} wall_ms={:.3f} cpu_ms={:.3f} "
            "scheduler_valid={} scheduler_wait_ms={:.3f} other_wait_estimate_ms={:.3f} "
            "switches_valid={} voluntary_switches={} involuntary_switches={} "
            "pending_before={} pending_after={} queue_before_ms={:.3f} queue_at_return_ms={:.3f} probe_ms={:.3f}",
            std::chrono::duration_cast<std::chrono::nanoseconds>(start.time_since_epoch()).count(),
            std::chrono::duration_cast<std::chrono::nanoseconds>(end.time_since_epoch()).count(),
            wall_ms, cpu_ms, scheduler_valid, wait_ms, residual_ms, switches_valid,
            switches_valid ? after.voluntary - before.voluntary : -1L,
            switches_valid ? after.involuntary - before.involuntary : -1L,
            pending_before, pending_after, queue_before_ms,
            queue_before_ms + wall_ms, probe_ms);
    }

    void track_frame(
        const StereoFrame &frame,
        const std::vector<ImuMeasurement> &imu_measurements = {},
        std::optional<Clock::time_point> received_at = std::nullopt,
        bool complete_reserved_work = false)
    {
        const TrackingState previous_state = slam_->trackingState();

        if (trace_) trace_->record("track_begin", 0, frame.timestamp, imu_measurements.size());
        // Thread CPU excludes backend worker threads; the residual includes scheduling
        // and blocking, and cannot by itself identify a particular lock or scheduler cause.
        // Probes bracket the backend; their cost is excluded from wall_ms.
        // Snapshot before timing probes so lock contention is not backend CPU time.
        const QueueSnapshot queue = queue_snapshot();
        const auto probe_start = Clock::now();
        const SchedulerSnapshot scheduler_before = slow_tracking_enabled_ ? scheduler_snapshot() : SchedulerSnapshot{};
        timespec cpu_start{}, cpu_end{};
        const bool cpu_started = tracking_diagnostics_enabled_ &&
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_start) == 0;
        const auto start = Clock::now();
        const double queue_before_ms = received_at ?
            std::chrono::duration<double, std::milli>(start - *received_at).count() :
            0.0;

        if (tracking_mode_ == TrackingMode::Stereo)
            slam_->track(frame);
        else
            slam_->track(frame, imu_measurements);

        const auto end = Clock::now();
        const bool cpu_finished = cpu_started && clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_end) == 0;

        SchedulerSnapshot scheduler_after{};
        double probe_ms = 0.0;
        if (slow_tracking_enabled_)
        {
            scheduler_after = scheduler_snapshot();
            probe_ms =
                std::chrono::duration<double, std::milli>(start - probe_start).count() +
                std::chrono::duration<double, std::milli>(Clock::now() - end).count();
        }

        uint64_t processed;
        {
            const std::lock_guard<std::mutex> lock(queue_mutex_);

            // Commit queue removal and completion together before operational logging.
            if (tracking_mode_ == TrackingMode::StereoImu)
            {
                if (complete_reserved_work)
                    reservation_.reset();
                else
                    pending_frames_.pop_front();

                last_tracked_frame_timestamp_ = frame.timestamp;
            }
            processed = ++processed_frames_;
        }

        if (complete_reserved_work)
            tracking_work_.reset();

        if (slow_tracking_enabled_)
        {
            const double cpu_ms = cpu_finished ? (cpu_end.tv_sec - cpu_start.tv_sec) * 1000.0 +
                (cpu_end.tv_nsec - cpu_start.tv_nsec) * 1e-6 : -1.0;
            log_slow_tracking(scheduler_before, scheduler_after, start, end, cpu_ms, probe_ms,
                              queue_before_ms, queue.pending, queue_snapshot().pending);
        }
        if (tracking_diagnostics_enabled_)
        {
            const double wall_ms = std::chrono::duration<double, std::milli>(end - start).count();
            ++tracking_diagnostics_.calls;
            tracking_diagnostics_.wall_sum_ms += wall_ms;
            tracking_diagnostics_.wall_max_ms = std::max(tracking_diagnostics_.wall_max_ms, wall_ms);
            if (cpu_finished)
            {
                const double cpu_ms = (cpu_end.tv_sec - cpu_start.tv_sec) * 1000.0 +
                    (cpu_end.tv_nsec - cpu_start.tv_nsec) * 1e-6;
                ++tracking_diagnostics_.cpu_samples;
                tracking_diagnostics_.cpu_sum_ms += cpu_ms;
                tracking_diagnostics_.non_cpu_sum_ms += std::max(0.0, wall_ms - cpu_ms);
            }
        }

        if (trace_) trace_->record("track_end", 0, frame.timestamp);
        const double track_ms =
            std::chrono::duration<double, std::milli>(end - start).count();

        // Preserve the previous timestamp across report windows; count only successful calls.
        for (auto *stats : {&window_, &total_})
        {
            ++stats->frames;
            stats->track_sum_ms += track_ms;
            stats->track_max_ms = std::max(stats->track_max_ms, track_ms);
            if (queue.processed > 0)
            {
                const double interval_ms = (frame.timestamp - previous_timestamp_) * 1000.0;
                ++stats->intervals;
                stats->interval_sum_ms += interval_ms;
                stats->interval_min_ms = std::min(stats->interval_min_ms, interval_ms);
                stats->interval_max_ms = std::max(stats->interval_max_ms, interval_ms);
            }
        }

        // Use this frame's enqueue time independently of the current queue front.
        if (received_at)
        {
            const double elapsed_ms =
                std::chrono::duration<double, std::milli>(end - *received_at).count();
            enqueue_to_return_sum_ms_ += elapsed_ms;
            enqueue_to_return_max_ms_ =
                std::max(enqueue_to_return_max_ms_, elapsed_ms);
        }
        previous_timestamp_ = frame.timestamp;

        // Report only after tracking and its statistics have completed successfully.
        if (processed == 1)
        {
            node_logger_->info("First stereo frame processed: timestamp={:.9f}", frame.timestamp);
            RCLCPP_INFO(get_logger(), "First stereo frame processed: timestamp=%.9f", frame.timestamp);
        }
        const auto state = slam_->trackingState();
        node_logger_->debug("Stereo frame: index={} timestamp={:.9f} track_ms={:.6f} state={}",
                     static_cast<unsigned long long>(processed), frame.timestamp,
                     track_ms, tracking_state_name(state));
        if (state != previous_state)
        {
            node_logger_->info("Tracking state: {} -> {}",
                        tracking_state_name(previous_state), tracking_state_name(state));
            RCLCPP_INFO(get_logger(), "Tracking state: %s -> %s",
                        tracking_state_name(previous_state), tracking_state_name(state));
        }
    }

    void on_frame(const StereoFrame &frame)
    {
        if (stop_requested_)
            return;

        if (tracking_mode_ == TrackingMode::StereoImu)
        {
            enqueue_frame(frame);
            return;
        }

        track_frame(frame);
    }

    void process_pending_frames()
    {
        if (stop_requested_)
            return;

        // A rejected backwards sample is evidence of a discontinuous input timeline.
        // Check even with an empty image queue; do not silently resume after a reset.
        const ImuFrontendStats imu_stats = imu_frontend_->stats();
        if (imu_stats.backwards > 0)
            throw std::runtime_error("IMU timestamp moved backwards: count=" +
                                     std::to_string(imu_stats.backwards));

        const QueueSnapshot queue = queue_snapshot();
        const Clock::time_point now = Clock::now();

        // Keep startup bounded even when early images are discarded.
        if (!queue.last_tracked && queue.startup_started)
        {
            const double startup_wait_sec =
                std::chrono::duration<double>(now - *queue.startup_started).count();
            if (startup_wait_sec >= imu_wait_timeout_sec_)
                throw_wait_timeout("Stereo-IMU startup", startup_wait_sec);
        }

        // Reserved frames retain their original enqueue deadline even when the queue is empty.
        if (queue.oldest_received_at)
        {
            const double frame_wait_sec =
                std::chrono::duration<double>(now - *queue.oldest_received_at).count();
            if (frame_wait_sec >= imu_wait_timeout_sec_)
                throw_wait_timeout("Stereo frame", frame_wait_sec);
        }

        // Verify the first interval before sending either startup image.
        if (!queue.last_tracked)
        {
            if (queue.pending < 2)
                return;

            const double t0 = queue.first->frame.timestamp;
            const double t1 = queue.second->frame.timestamp;
            const ImuBatch batch = imu_frontend_->takeMeasurements(t0, t1);

            switch (batch.status)
            {
            case ImuBatchStatus::WaitingForData:
                return;
            case ImuBatchStatus::MissingHistory:
            {
                // No backend call has occurred; discard and its count commit together.
                const std::lock_guard<std::mutex> lock(queue_mutex_);
                pending_frames_.pop_front();
                ++startup_discarded_frames_;
                return;
            }
            case ImuBatchStatus::BufferOverflow:
                throw_batch_error("IMU buffer overflow during startup", t0, t1);
            case ImuBatchStatus::DataGap:
                throw_batch_error("IMU data gap detected during startup", t0, t1);
            case ImuBatchStatus::InvalidRequest:
                throw_batch_error("IMU startup batch request is invalid", t0, t1);
            case ImuBatchStatus::Ready:
            {
                // Independent values keep both frames alive across unlocked backend calls.
                const StereoFrame first_frame = queue.first->frame;
                track_frame(first_frame, {}, queue.first->received_at);

                const StereoFrame second_frame = queue.second->frame;
                track_frame(second_frame, batch.measurements, queue.second->received_at);
                {
                    const std::lock_guard<std::mutex> lock(queue_mutex_);
                    startup_wait_started_.reset();
                }
                return;
            }
            }
            return;
        }

        // Continue the same reserved frame across IMU retries.
        if (!reserve_tracking_work())
            return;

        if (tracking_work_->stage != TrackingWorkStage::Reserved)
            throw std::logic_error("Cannot resample a ready or executing frame");

        // Keep an independent frame value alive through completion and logging.
        const StereoFrame frame = tracking_work_->pending.frame;
        const Clock::time_point received_at = tracking_work_->pending.received_at;
        ImuBatch batch = imu_frontend_->takeMeasurements(
            *queue.last_tracked, frame.timestamp);

        switch (batch.status)
        {
        case ImuBatchStatus::WaitingForData:
            return;
        case ImuBatchStatus::MissingHistory:
            throw_batch_error("IMU history is missing after tracking started", *queue.last_tracked, frame.timestamp);
        case ImuBatchStatus::BufferOverflow:
            throw_batch_error("IMU buffer overflow", *queue.last_tracked, frame.timestamp);
        case ImuBatchStatus::DataGap:
            throw_batch_error("IMU data gap detected", *queue.last_tracked, frame.timestamp);
        case ImuBatchStatus::InvalidRequest:
            throw_batch_error("IMU batch request is invalid", *queue.last_tracked, frame.timestamp);
        case ImuBatchStatus::Ready:
        {
            tracking_work_->imu_batch.emplace(std::move(batch));
            tracking_work_->stage = TrackingWorkStage::Ready;
            {
                const std::lock_guard<std::mutex> lock(queue_mutex_);
                reservation_->stage = TrackingWorkStage::Ready;
                reservation_->imu_batch_consumed = true;
            }

            // Preserve consumed data if stopping before backend execution.
            if (stop_requested_)
                return;

            tracking_work_->stage = TrackingWorkStage::Executing;
            {
                const std::lock_guard<std::mutex> lock(queue_mutex_);
                reservation_->stage = TrackingWorkStage::Executing;
            }

            track_frame(
                frame, tracking_work_->imu_batch->measurements, received_at, true);
            return;
        }
        }
    }

    // Preserve sub-microsecond timestamp detail in interval failure diagnostics.
    [[noreturn]] void throw_batch_error(const char *reason, double left, double right) const
    {
        const QueueSnapshot queue = queue_snapshot();
        std::ostringstream message;
        message << std::setprecision(17) << reason << ": interval=(" << left << ", "
                << right << "] pending=" << queue.pending << " in_flight=" << queue.in_flight
                << " outstanding=" << queue.outstanding;
        throw std::runtime_error(message.str());
    }

    [[noreturn]] void throw_wait_timeout(const char *scope, double waited_sec) const
    {
        const QueueSnapshot queue = queue_snapshot();
        std::ostringstream message;
        message << scope << " wait timed out: waited_sec=" << waited_sec
                << " threshold_sec=" << imu_wait_timeout_sec_
                << " pending=" << queue.pending
                << " in_flight=" << queue.in_flight
                << " outstanding=" << queue.outstanding;
        throw std::runtime_error(message.str());
    }

    static void require_absolute_path(const std::string &path, const char *name)
    {
        if (path.empty() || !std::filesystem::path(path).is_absolute())
            throw std::invalid_argument(std::string(name) + " must be a nonempty absolute path");
    }

    void enqueue_frame(const StereoFrame &frame)
    {
        // Reject invalid sensor timestamp.
        if (!std::isfinite(frame.timestamp) || frame.timestamp < 0.0)
            throw std::invalid_argument("Stereo timestamp must be finite & nonnegative");

        std::unique_lock<std::mutex> lock(queue_mutex_);
        // Reserved work still occupies capacity until completion or discard.
        const std::size_t in_flight =
            (reservation_.has_value() ? 1U : 0U) +
            (startup_next_reservation_.has_value() ? 1U : 0U);
        const std::size_t outstanding = pending_frames_.size() + in_flight;

        // Require strictly increasing timestamps across accepted frames.
        if (last_received_frame_timestamp_ &&
            frame.timestamp <= *last_received_frame_timestamp_)
        {
            lock.unlock();
            throw std::invalid_argument("Stereo timestamps must be strictly increasing");
        }
        // Stop before exceeding the pending-frame limit.
        if (outstanding >= pending_frames_capacity_)
        {
            const std::size_t pending = pending_frames_.size();
            lock.unlock();
            std::ostringstream message;
            message << std::setprecision(17) << "Pending frame queue is full: capacity="
                    << pending_frames_capacity_ << " pending=" << pending
                    << " incoming_timestamp=" << frame.timestamp
                    << " in_flight=" << in_flight
                    << " outstanding=" << outstanding;
            throw std::runtime_error(message.str());
        }

        // Retain the frame and record its enqueue time.
        pending_frames_.push_back({frame, Clock::now()});
        if (trace_) trace_->record("frame_enqueued", 0, frame.timestamp, pending_frames_.size());
        last_received_frame_timestamp_ = frame.timestamp;
        ++enqueued_frames_;
        pending_frames_peak_ = std::max(pending_frames_peak_, pending_frames_.size());

        // Keep the original startup deadline even if early frames are discarded.
        if (!last_tracked_frame_timestamp_ && !startup_wait_started_)
            startup_wait_started_ = pending_frames_.back().received_at;
    }

    // Declared before all producers so the logging backend is destroyed last.
    std::unique_ptr<LoggingSession> logging_;
    std::shared_ptr<spdlog::logger> node_logger_;
    std::shared_ptr<spdlog::logger> diagnostics_logger_;
    bool tracking_diagnostics_enabled_ = false;
    bool slow_tracking_enabled_ = false;
    double slow_tracking_threshold_ms_ = 50.0;
    uint64_t scheduler_calls_ = 0, scheduler_unavailable_ = 0, slow_calls_ = 0, slow_suppressed_ = 0;
    double probe_sum_ms_ = 0.0, probe_max_ms_ = 0.0;
    std::optional<Clock::time_point> last_slow_report_;
    TrackingDiagnostics tracking_diagnostics_;
    Clock::time_point diagnostics_last_report_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    // All activity and timer callbacks run on main's single-threaded executor.
    std::function<void()> request_stop_;
    double input_timeout_sec_ = 5.0;
    std::string input_timeout_action_;
    std::optional<Clock::time_point> last_input_activity_;
    bool input_timeout_reported_ = false;
    bool stop_requested_ = false;
    rclcpp::TimerBase::SharedPtr input_timer_;

    // Declared first so trace outlives both frontends, including exceptional teardown.
    std::unique_ptr<DiagnosticTrace> trace_;
    std::unique_ptr<OrbSlam3Adapter> slam_;
    std::unique_ptr<StereoFrontend> stereo_frontend_;
    std::unique_ptr<ImuFrontend> imu_frontend_;

    // Sensor timestamps measure frame spacing; steady-clock times measure throughput.
    uint64_t processed_frames_ = 0;
    double previous_timestamp_ = 0.0;
    Statistics window_;
    Statistics total_;
    Clock::time_point started_;
    Clock::time_point last_report_;
    rclcpp::TimerBase::SharedPtr report_timer_;

    // Pending images retain their pixels until tracking completes
    std::string imu_topic_;
    mutable std::mutex queue_mutex_;
    std::deque<PendingFrame> pending_frames_;

    // Queue mutex protects reservation summaries alongside pending frames.
    std::optional<ReservationSummary> reservation_;
    std::optional<ReservationSummary> startup_next_reservation_;

    TrackingMode tracking_mode_ = TrackingMode::Stereo;

    double imu_wait_timeout_sec_ = 1.0;
    uint64_t startup_discarded_frames_ = 0;
    uint64_t enqueued_frames_ = 0;
    std::size_t pending_frames_peak_ = 0;
    double enqueue_to_return_sum_ms_ = 0.0;
    double enqueue_to_return_max_ms_ = 0.0;
    int64_t imu_retry_period_ms_ = 5;
    std::size_t pending_frames_capacity_ = 30;
    std::optional<double> last_received_frame_timestamp_;
    std::optional<double> last_tracked_frame_timestamp_;
    std::optional<Clock::time_point> startup_wait_started_;
    rclcpp::TimerBase::SharedPtr imu_retry_timer_;

    // Tracking owns these across retries, including waits for IMU coverage.
    std::optional<TrackingWork> tracking_work_;
    std::optional<TrackingWork> startup_next_work_;
};
}

#ifndef GEMINI336_QUEUE_TEST
int main(int argc, char **argv)
{
    int result = 0;
    std::shared_ptr<gemini336_orbslam3::SlamNode> node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

    try
    {
        rclcpp::init(argc, argv);
        executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        node = std::make_shared<gemini336_orbslam3::SlamNode>([&executor]() { executor->cancel(); });
        executor->add_node(node);

        // Serial execution protects the frontend, adapter and statistics from overlap.
        executor->spin();
    }
    catch (const std::exception &error)
    {
        if (node) node->log_failure(error.what());
        RCLCPP_ERROR(rclcpp::get_logger("slam_node"), "%s", error.what());
        result = 1;
    }

    // Retain ownership across spin failures so both exit paths explicitly shut down SLAM.
    if (node)
    {
        try
        {
            node->shutdown();
        }
        catch (const std::exception &error)
        {
            node->log_failure(std::string("Shutdown failed: ") + error.what());
            RCLCPP_ERROR(rclcpp::get_logger("slam_node"), "Shutdown failed: %s", error.what());
            result = 1;
        }
    }

    node.reset();
    if (rclcpp::ok())
        rclcpp::shutdown();

    return result;
}

#endif  // GEMINI336_QUEUE_TEST
