#include "gemini336_orbslam3/logging.hpp"
#include "slam/orbslam3_adapter.hpp"
#include "frontend/stereo_frontend.hpp"
#include "frontend/imu_frontend.hpp"

#include <cstdint>
#include <ctime>
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
        node_logger_->info("Stereo input stopped; processed={} last_state={} remaining_frames={}",
                    static_cast<unsigned long long>(processed_frames_),
                    tracking_state_name(slam_->trackingState()),
                    pending_frames_.size());
        RCLCPP_INFO(get_logger(),
                    "Stereo input stopped; processed=%llu last_state=%s remaining_frames=%zu",
                    static_cast<unsigned long long>(processed_frames_),
                    tracking_state_name(slam_->trackingState()),
                    pending_frames_.size());

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
        pending_frames_.clear();
        slam_->shutdown();
        node_logger_->info("Stereo SLAM shutdown returned");
        RCLCPP_INFO(get_logger(), "Stereo SLAM shutdown returned");
    }

private:
    using Clock = std::chrono::steady_clock;

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

    void log_statistics(const char *scope, const Statistics &stats, double elapsed)
    {
        // Timing diagnostics already provide the periodic INFO summary.
        node_logger_->log(tracking_diagnostics_enabled_ && std::string(scope) == "window" ?
                        spdlog::level::debug : spdlog::level::info, "Stereo stats: scope={} frames={} total={} elapsed_sec={:.6f} rate_hz={:.6f} "
                    "track_mean_ms={:.6f} track_max_ms={:.6f} intervals={} "
                    "interval_min_ms={:.6f} interval_mean_ms={:.6f} interval_max_ms={:.6f}",
                    scope, static_cast<unsigned long long>(stats.frames),
                    static_cast<unsigned long long>(processed_frames_), elapsed,
                    elapsed > 0.0 ? stats.frames / elapsed : 0.0,
                    stats.frames ? stats.track_sum_ms / stats.frames : 0.0, stats.track_max_ms,
                    static_cast<unsigned long long>(stats.intervals),
                    stats.intervals ? stats.interval_min_ms : 0.0,
                    stats.intervals ? stats.interval_sum_ms / stats.intervals : 0.0,
                    stats.interval_max_ms);
    }

    void report(bool final)
    {
        const auto now = Clock::now();

        log_statistics(final ? "tail" : "window", window_,
                       std::chrono::duration<double>(now - last_report_).count());
        if (final)
            log_statistics("total", total_, std::chrono::duration<double>(now - started_).count());

        if (tracking_mode_ == TrackingMode::StereoImu)
        {
            const double oldest_wait_sec = pending_frames_.empty() ? 0.0 :
                std::chrono::duration<double>(now - pending_frames_.front().received_at).count();
            node_logger_->log(tracking_diagnostics_enabled_ && !final ?
                            spdlog::level::debug : spdlog::level::info, "Stereo-IMU coordination: final={} enqueued={} processed={} "
                        "startup_discarded={} pending={} pending_peak={} oldest_wait_sec={:.6f} "
                        "enqueue_to_return_mean_ms={:.6f} enqueue_to_return_max_ms={:.6f}",
                        final ? "true" : "false",
                        static_cast<unsigned long long>(enqueued_frames_),
                        static_cast<unsigned long long>(processed_frames_),
                        static_cast<unsigned long long>(startup_discarded_frames_),
                        pending_frames_.size(), pending_frames_peak_, oldest_wait_sec,
                        processed_frames_ ? enqueue_to_return_sum_ms_ / processed_frames_ : 0.0,
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
        const Clock::time_point now = Clock::now();
        const double elapsed = std::chrono::duration<double>(now - diagnostics_last_report_).count();
        const double oldest_ms = pending_frames_.empty() ? 0.0 :
            std::chrono::duration<double, std::milli>(now - pending_frames_.front().received_at).count();
        const TrackingDiagnostics &stats = tracking_diagnostics_;
        diagnostics_logger_->info(
            "TRACK_TIMING final={} steady_ns={} window_sec={:.6f} calls={} rate_hz={:.3f} "
            "wall_mean_ms={:.3f} wall_max_ms={:.3f} cpu_samples={} cpu_mean_ms={:.3f} "
            "non_cpu_mean_ms={:.3f} pending={} pending_peak={} oldest_queue_ms={:.3f} log_dropped={}",
            final, std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count(),
            elapsed, stats.calls, elapsed > 0.0 ? stats.calls / elapsed : 0.0,
            stats.calls ? stats.wall_sum_ms / stats.calls : 0.0, stats.wall_max_ms,
            stats.cpu_samples, stats.cpu_samples ? stats.cpu_sum_ms / stats.cpu_samples : 0.0,
            stats.cpu_samples ? stats.non_cpu_sum_ms / stats.cpu_samples : 0.0,
            pending_frames_.size(), pending_frames_peak_, oldest_ms, logging_->dropped_messages());

        // Query the same interval as tracking without consuming IMU or adding trace events.
        // Seconds below are existing backend boundaries, not reconstructed raw nanoseconds.
        if (final && imu_frontend_ && !pending_frames_.empty())
        {
            const bool have_interval = last_tracked_frame_timestamp_.has_value() || pending_frames_.size() >= 2;
            const double left = last_tracked_frame_timestamp_.value_or(pending_frames_.front().frame.timestamp);
            const double right = last_tracked_frame_timestamp_ ? pending_frames_.front().frame.timestamp :
                pending_frames_[have_interval ? 1 : 0].frame.timestamp;
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
        tracking_diagnostics_ = TrackingDiagnostics{};
        diagnostics_last_report_ = now;
    }

    void track_frame(
        const StereoFrame &frame,
        const std::vector<ImuMeasurement> &imu_measurements = {})
    {
        const TrackingState previous_state = slam_->trackingState();

        if (trace_) trace_->record("track_begin", 0, frame.timestamp, imu_measurements.size());
        // Thread CPU excludes backend worker threads; the residual includes scheduling
        // and blocking, and cannot by itself identify a particular lock or scheduler cause.
        timespec cpu_start{}, cpu_end{};
        const bool cpu_started = tracking_diagnostics_enabled_ &&
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_start) == 0;
        const auto start = Clock::now();
        if (tracking_mode_ == TrackingMode::Stereo)
            slam_->track(frame);
        else
            slam_->track(frame, imu_measurements);

        const auto end = Clock::now();
        const bool cpu_finished = cpu_started && clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_end) == 0;
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
            std::chrono::duration<double, std::milli>(Clock::now() - start).count();

        // Preserve the previous timestamp across report windows; count only successful calls.
        for (auto *stats : {&window_, &total_})
        {
            ++stats->frames;
            stats->track_sum_ms += track_ms;
            stats->track_max_ms = std::max(stats->track_max_ms, track_ms);
            if (processed_frames_ > 0)
            {
                const double interval_ms = (frame.timestamp - previous_timestamp_) * 1000.0;
                ++stats->intervals;
                stats->interval_sum_ms += interval_ms;
                stats->interval_min_ms = std::min(stats->interval_min_ms, interval_ms);
                stats->interval_max_ms = std::max(stats->interval_max_ms, interval_ms);
            }
        }
        // Stereo-IMU tracking always processes the pending front, including startup.
        // Measure through backend return; this excludes upstream middleware waiting.
        if (tracking_mode_ == TrackingMode::StereoImu)
        {
            const double elapsed_ms = std::chrono::duration<double, std::milli>(
                start - pending_frames_.front().received_at).count() + track_ms;
            enqueue_to_return_sum_ms_ += elapsed_ms;
            enqueue_to_return_max_ms_ = std::max(enqueue_to_return_max_ms_, elapsed_ms);
        }
        previous_timestamp_ = frame.timestamp;
        ++processed_frames_;

        // Report only after tracking and its statistics have completed successfully.
        if (processed_frames_ == 1)
        {
            node_logger_->info("First stereo frame processed: timestamp={:.9f}", frame.timestamp);
            RCLCPP_INFO(get_logger(), "First stereo frame processed: timestamp=%.9f", frame.timestamp);
        }
        const auto state = slam_->trackingState();
        node_logger_->debug("Stereo frame: index={} timestamp={:.9f} track_ms={:.6f} state={}",
                     static_cast<unsigned long long>(processed_frames_), frame.timestamp,
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

        const Clock::time_point now = Clock::now();

        // Keep startup bounded even when early images are discarded.
        if (!last_tracked_frame_timestamp_ && startup_wait_started_)
        {
            const double startup_wait_sec =
                std::chrono::duration<double>(now - *startup_wait_started_).count();
            if (startup_wait_sec >= imu_wait_timeout_sec_)
                throw_wait_timeout("Stereo-IMU startup", startup_wait_sec);
        }

        if (pending_frames_.empty())
            return;

        // Each image's deadline starts at enqueue time, not at the latest retry.
        const double frame_wait_sec =
            std::chrono::duration<double>(now - pending_frames_.front().received_at).count();
        if (frame_wait_sec >= imu_wait_timeout_sec_)
            throw_wait_timeout("Stereo frame", frame_wait_sec);

        // Verify the first interval before sending either startup image.
        if (!last_tracked_frame_timestamp_)
        {
            if (pending_frames_.size() < 2)
                return;

            const double t0 = pending_frames_[0].frame.timestamp;
            const double t1 = pending_frames_[1].frame.timestamp;
            const ImuBatch batch = imu_frontend_->takeMeasurements(t0, t1);

            switch (batch.status)
            {
            case ImuBatchStatus::WaitingForData:
                return;
            case ImuBatchStatus::MissingHistory:
                // No image has reached SLAM yet, so the startup origin can advance.
                pending_frames_.pop_front();
                ++startup_discarded_frames_;
                return;
            case ImuBatchStatus::BufferOverflow:
                throw_batch_error("IMU buffer overflow during startup", t0, t1);
            case ImuBatchStatus::DataGap:
                throw_batch_error("IMU data gap detected during startup", t0, t1);
            case ImuBatchStatus::InvalidRequest:
                throw_batch_error("IMU startup batch request is invalid", t0, t1);
            case ImuBatchStatus::Ready:
            {
                // Only the first image has no preceding image interval.
                const StereoFrame &first_frame = pending_frames_.front().frame;
                track_frame(first_frame, {});
                last_tracked_frame_timestamp_ = first_frame.timestamp;
                pending_frames_.pop_front();

                // Reacquire the front after removal; this batch belongs to (t0, t1].
                const StereoFrame &second_frame = pending_frames_.front().frame;
                track_frame(second_frame, batch.measurements);
                last_tracked_frame_timestamp_ = second_frame.timestamp;
                pending_frames_.pop_front();

                startup_wait_started_.reset();
                return;
            }
            }
            return;
        }

        // Continue from the last normally returned tracking call, one image per retry.
        const StereoFrame &frame = pending_frames_.front().frame;
        const ImuBatch batch = imu_frontend_->takeMeasurements(
            *last_tracked_frame_timestamp_, frame.timestamp);

        switch (batch.status)
        {
        case ImuBatchStatus::WaitingForData:
            return;
        case ImuBatchStatus::MissingHistory:
            throw_batch_error("IMU history is missing after tracking started", *last_tracked_frame_timestamp_, frame.timestamp);
        case ImuBatchStatus::BufferOverflow:
            throw_batch_error("IMU buffer overflow", *last_tracked_frame_timestamp_, frame.timestamp);
        case ImuBatchStatus::DataGap:
            throw_batch_error("IMU data gap detected", *last_tracked_frame_timestamp_, frame.timestamp);
        case ImuBatchStatus::InvalidRequest:
            throw_batch_error("IMU batch request is invalid", *last_tracked_frame_timestamp_, frame.timestamp);
        case ImuBatchStatus::Ready:
            // Ready already consumed IMU; propagate tracking failures without retrying.
            track_frame(frame, batch.measurements);
            last_tracked_frame_timestamp_ = frame.timestamp;
            pending_frames_.pop_front();
            return;
        }
    }

    // Preserve sub-microsecond timestamp detail in interval failure diagnostics.
    [[noreturn]] void throw_batch_error(const char *reason, double left, double right) const
    {
        std::ostringstream message;
        message << std::setprecision(17) << reason << ": interval=(" << left << ", "
                << right << "] pending=" << pending_frames_.size();
        throw std::runtime_error(message.str());
    }

    [[noreturn]] void throw_wait_timeout(const char *scope, double waited_sec) const
    {
        std::ostringstream message;
        message << scope << " wait timed out: waited_sec=" << waited_sec
                << " threshold_sec=" << imu_wait_timeout_sec_
                << " pending=" << pending_frames_.size();
        throw std::runtime_error(message.str());
    }

    static void require_absolute_path(const std::string &path, const char *name)
    {
        if (path.empty() || !std::filesystem::path(path).is_absolute())
            throw std::invalid_argument(std::string(name) + " must be a nonempty absolute path");
    }

    struct PendingFrame
    {
        StereoFrame frame;
        Clock::time_point received_at;
    };

    void enqueue_frame(const StereoFrame &frame)
    {
        // Reject invalid sensor timestamp.
        if (!std::isfinite(frame.timestamp) || frame.timestamp < 0.0)
            throw std::invalid_argument("Stereo timestamp must be finite & nonnegative");
        // Require strictly increasing timestamps across accepted frames.
        if (last_received_frame_timestamp_ &&
            frame.timestamp <= *last_received_frame_timestamp_)
            throw std::invalid_argument("Stereo timestamps must be strictly increasing");
        // Stop before exceeding the pending-frame limit.
        if (pending_frames_.size() >= pending_frames_capacity_)
        {
            std::ostringstream message;
            message << std::setprecision(17) << "Pending frame queue is full: capacity="
                    << pending_frames_capacity_ << " pending=" << pending_frames_.size()
                    << " incoming_timestamp=" << frame.timestamp;
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
    std::deque<PendingFrame> pending_frames_;
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
};
}

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
