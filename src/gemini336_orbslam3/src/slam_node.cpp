#include "gemini336_orbslam3/logging.hpp"
#include "slam/orbslam3_adapter.hpp"
#include "frontend/stereo_frontend.hpp"
#include "frontend/imu_frontend.hpp"
#include "common/stop_control.hpp"
#include "common/callback_guard.hpp"
#include "common/context_stop_registration.hpp"
#include "common/process_finalization.hpp"
#include <cstdio>

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
    explicit SlamNode(
        std::shared_ptr<StopControl> stop_control,
        std::function<void()> request_stop,
        std::shared_ptr<LoggingSession> &process_logging)
        : Node("slam_node"),
          stop_control_(std::move(stop_control)),
          request_stop_(std::move(request_stop))
    {
        if (!stop_control_)
        {
            throw std::invalid_argument("SlamNode: stop_control must not be null");
        }
        if (!request_stop_)
        {
            throw std::invalid_argument("SlamNode: request_stop must not be empty");
        }

        const auto context = get_node_base_interface()->get_context();
        callback_guard_ = std::make_shared<CallbackGuard>(stop_control_, request_stop_,
            [context]() { context->shutdown("Executor cancel failed"); },
            [this](std::exception_ptr error) {
                try { std::rethrow_exception(error); }
                catch (const std::exception &e) { log_failure(e.what()); }
                catch (...) { log_failure("Unknown callback exception"); }
            });

        // These settings define component lifetimes and are fixed at startup.
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;

        // Establish the session before constructing any sensor or SLAM component.
        LoggingOptions logging_options;
        logging_options.directory = declare_parameter<std::string>(
            "logging.directory", "", descriptor);
        logging_options.level = declare_parameter<std::string>(
            "logging.level", "info", descriptor);
        logging_ = std::make_shared<LoggingSession>(logging_options);
        // Main retains the session even if later node construction fails.
        process_logging = logging_;
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

            // Capacity includes both queued and reserved frames.
            const int64_t pending_capacity = declare_parameter<int64_t>(
                "stereo.pending_frame_capacity", 30, descriptor);
            if (pending_capacity <= 0)
                throw std::invalid_argument("stereo.pending_frame_capacity must be positive");
            pending_frames_capacity_ =
                static_cast<std::size_t>(pending_capacity);

            const int64_t retry_period_ms = declare_parameter<int64_t>(
                "stereo.retry_period_ms", 5, descriptor);
            if (retry_period_ms <= 0)
                throw std::invalid_argument("stereo.retry_period_ms must be positive");
            tracking_retry_period_ms_ = retry_period_ms;
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
            tracking_retry_period_ms_ = imu_retry_period_ms;
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

        // Each group serializes its callbacks while allowing cross-group concurrency.
        reception_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        tracking_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // Construct IMU frontend before stereo frontend.
        if (tracking_mode_ == TrackingMode::StereoImu)
        {
            imu_frontend_ = std::make_unique<ImuFrontend>(
                this, reception_group_, imu_topic_, trace_.get(), stop_control_, callback_guard_);
        }

        // Both modes consume queued frames through the same scheduling entry.
        tracking_timer_ = create_wall_timer(
            std::chrono::milliseconds(tracking_retry_period_ms_),
            [this]() { tracking_callback(); },
            tracking_group_);

        // Construct Stereo frontend
        stereo_frontend_ = std::make_unique<StereoFrontend>(
            this, reception_group_, left_topic, right_topic,
            [this](const StereoFrame &frame) { on_frame(frame); },
            [this]() { on_input_activity(); }, trace_.get(), callback_guard_);

        // Wall-clock timers remain independent of sensor timestamps and simulated time.
        started_ = last_report_ = Clock::now();
        if (tracking_diagnostics_enabled_)
        {
            diagnostics_last_report_ = started_;
            diagnostics_timer_ = create_wall_timer(
                std::chrono::seconds(1),
                [this]() { callback_guard_->run([this]() { report_tracking_diagnostics(false); }); },
                tracking_group_);
        }

        report_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            [this]() { callback_guard_->run([this]() { report(false); }); },
            tracking_group_);

        if (input_timeout_sec_ > 0.0)
            input_timer_ = create_wall_timer(
                std::chrono::milliseconds(100),
                [this]() { callback_guard_->run([this]() { check_input_timeout(); }); },
                reception_group_);

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

    // Precondition: executor callbacks have finished. This is not a callback entry point.
    void shutdown() noexcept
    {
        if (finalization_started_) return;
        finalization_started_ = true;

        finalize_step("stop_admission", StopReason::ShutdownError, [this]() {
            stop_control_->request_stop(StopReason::Finalization);
        });
        finalize_step("stop_timers", StopReason::ShutdownError, [this]() {
            diagnostics_timer_.reset();
            input_timer_.reset();
            report_timer_.reset();
            tracking_timer_.reset();
        });
        finalize_step("stop_stereo", StopReason::ShutdownError, [this]() {
            if (stereo_frontend_) stereo_frontend_->stop_receiving();
        });
        finalize_step("stop_imu", StopReason::ShutdownError, [this]() {
            if (imu_frontend_) imu_frontend_->stop_receiving();
        });
        finalize_step("interrupt_work", StopReason::ShutdownError, [this]() {
            observe_work_stop(WorkLocation::Finalization);
        });

        // A failed snapshot stays absent; never print zero accounting as a substitute.
        std::optional<FinalSnapshot> final;
        finalize_step("snapshot", StopReason::ShutdownError, [&]() {
            final.emplace(final_snapshot());
        });
        finalize_step("work_report", StopReason::ShutdownError, [&]() {
            if (final) report_final_work(*final);
            else node_logger_->error("Final work snapshot unavailable");
        });
        finalize_step("tracking_report", StopReason::ShutdownError, [this]() {
            if (tracking_diagnostics_enabled_) report_tracking_diagnostics(true);
        });
        finalize_step("imu_report", StopReason::ShutdownError, [this]() {
            if (!imu_frontend_) return;
            const ImuFrontendStats stats = imu_frontend_->stats();
            node_logger_->info("Final IMU input: received={} accepted={} stopped={} backwards={} "
                        "overflow={} buffered={}",
                        static_cast<unsigned long long>(stats.received),
                        static_cast<unsigned long long>(stats.accepted),
                        static_cast<unsigned long long>(stats.stopped),
                        static_cast<unsigned long long>(stats.backwards),
                        static_cast<unsigned long long>(stats.overflow), stats.buffered);
        });
        finalize_step("statistics_report", StopReason::ShutdownError, [this]() { report(true); });
        finalize_step("status_report", StopReason::ShutdownError, [&]() {
            if (!final) return;
#ifdef GEMINI336_QUEUE_TEST
            const TrackingState state = TrackingState::NotInitialized;
#else
            const TrackingState state = slam_->trackingState();
#endif
            node_logger_->info("Stereo input stopped; processed={} last_state={} remaining_frames={}",
                        static_cast<unsigned long long>(final->processed),
                        tracking_state_name(state), final->outstanding);
            RCLCPP_INFO(get_logger(),
                        "Stereo input stopped; processed=%llu last_state=%s remaining_frames=%zu",
                        static_cast<unsigned long long>(final->processed),
                        tracking_state_name(state), final->outstanding);
        });

        // Export before backend shutdown so an upstream stall cannot hide callback history.
        finalize_step("trace_write", StopReason::TraceWriteError, [this]() {
            if (trace_) trace_->write();
        });
        finalize_step("release_stereo", StopReason::ShutdownError, [this]() { stereo_frontend_.reset(); });
        finalize_step("release_imu", StopReason::ShutdownError, [this]() { imu_frontend_.reset(); });
        finalize_step("release_work", StopReason::ShutdownError, [this]() { release_unfinished_work(); });
        const bool backend_returned = finalize_step("backend_shutdown", StopReason::ShutdownError, [this]() {
#ifdef GEMINI336_QUEUE_TEST
            if (test_shutdown_) test_shutdown_();
#else
            if (slam_) slam_->shutdown();
#endif
        });
        if (backend_returned)
        {
            finalize_step("backend_report", StopReason::ShutdownError, [this]() {
                node_logger_->info("Stereo SLAM shutdown returned");
                RCLCPP_INFO(get_logger(), "Stereo SLAM shutdown returned");
            });
            finalize_step("release_backend", StopReason::ShutdownError, [this]() { slam_.reset(); });
        }
        // Failed backend shutdown retains ownership until main teardown;
        // the adapter destructor will not retry an attempted shutdown.
    }

private:
    // No cancel or context calls during finalization. Save failure before best-effort logging.
    template<class Function>
    bool finalize_step(const char *stage, StopReason reason, Function &&function) noexcept
    {
        try
        {
#ifdef GEMINI336_QUEUE_TEST
            if (test_finalize_step_) test_finalize_step_(stage);
#endif
            function();
            return true;
        }
        catch (...)
        {
            const std::exception_ptr error = std::current_exception();
            try { stop_control_->record_failure(reason, error, true); }
            catch (...) { /* Continue remaining safe cleanup even if control storage fails. */ }
            try
            {
                if (node_logger_) node_logger_->error("Finalization step failed: {}", stage);
            }
            catch (...) { /* Reporting must not replace the original cleanup failure. */ }
            return false;
        }
    }

#ifdef GEMINI336_QUEUE_TEST
    // Test-only construction exercises the real queue without sensors or a backend.
    friend struct SlamNodeQueueTestAccess;
    friend struct SlamTrackingTestAccess;
    struct QueueTestTag {};
    explicit SlamNode(QueueTestTag)
        : Node("slam_queue_test"), stop_control_(std::make_shared<StopControl>())
    {
    }
    // Only finite tests replace the adapter call; production keeps direct dispatch.
    std::function<void(const StereoFrame &, const std::vector<ImuMeasurement> &)> test_track_;
    enum class TestWorkPoint { AfterWaiting, AfterReady, BeforeBackend, BeforeBackendPermit, AfterBackendPermit, BeforeImuQuery, AfterBackendReturn };
    std::function<void(TestWorkPoint)> test_work_point_;
    std::function<void(const char *)> test_finalize_step_;
    std::function<void()> test_shutdown_;
#endif
    using Clock = std::chrono::steady_clock;

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

    enum class ImuBatchUse
    {
        NotRequired,
        NotAcquired,
        ConsumedUnused,
        DeliveredToBackend
    };

    enum class WorkInterruptionKind { Stopped, Failed };
    enum class WorkLocation
    {
        Coordination,
        ImuQuery,
        StartupDiscard,
        BeforeBackend,
        Backend,
        Completion,
        AfterBackend,
        Finalization
    };
    enum class WorkReason
    {
        StopObserved,
        UnexpectedException,
        ImuBackwards,
        StartupTimeout,
        FrameTimeout,
        ImuQueryException,
        ImuQueryResult,
        StartupDiscardException,
        BeforeBackendException,
        BackendException,
        AfterBackendException
    };

    // Fixed-size cause data survives logging failures without allocating error strings.
    // A related sequence different from this work identifies a failed predecessor.
    struct WorkInterruption
    {
        WorkInterruptionKind kind = WorkInterruptionKind::Failed;
        WorkLocation location = WorkLocation::Coordination;
        WorkReason reason = WorkReason::UnexpectedException;
        uint64_t related_sequence = 0;  // Zero means no individual work caused the event.
        std::optional<ImuBatchStatus> imu_status;
        std::exception_ptr exception;
    };

    struct ImuInterval
    {
        double left;
        double right;
    };

    struct TrackingWork
    {
        PendingFrame pending;
        TrackingWorkStage stage = TrackingWorkStage::Reserved;

        // Keep consumed IMU data with its frame so retries cannot take it again.
        std::optional<ImuBatch> imu_batch;
        ImuBatchUse batch_use = ImuBatchUse::NotAcquired;
        std::optional<WorkInterruption> interruption;
        std::optional<ImuInterval> imu_interval = std::nullopt;
    };

    struct ReservationSummary
    {
        double timestamp = 0.0;
        Clock::time_point received_at;
        TrackingWorkStage stage = TrackingWorkStage::Reserved;
        ImuBatchUse batch_use = ImuBatchUse::NotAcquired;
        uint64_t enqueue_sequence = 0;
        int64_t timestamp_ns = 0;
        std::optional<WorkInterruption> interruption;
    };

    // Queue-locked value snapshots retain identity and pixels independently of work ownership.
    struct QueueSnapshot
    {
        std::size_t pending = 0;
        std::size_t in_flight = 0;
        std::size_t outstanding = 0;
        // Legacy pending_peak logs report capacity usage, including reservations.
        std::size_t peak = 0;
        uint64_t enqueued = 0;
        uint64_t processed = 0;
        uint64_t startup_discarded = 0;
        uint64_t overload_discarded = 0;
        std::optional<PendingFrame> first;
        std::optional<PendingFrame> second;
        std::optional<ReservationSummary> reservation;
        std::optional<ReservationSummary> startup_next_reservation;
        std::optional<Clock::time_point> startup_started;
        std::optional<Clock::time_point> oldest_received_at;
        bool startup_complete = false;
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
        snapshot.peak = outstanding_frames_peak_;
        snapshot.enqueued = enqueued_frames_;
        snapshot.processed = processed_frames_;
        snapshot.startup_discarded = startup_discarded_frames_;
        snapshot.overload_discarded = overload_discarded_frames_;
        if (!pending_frames_.empty()) snapshot.first = pending_frames_[0];
        if (pending_frames_.size() >= 2) snapshot.second = pending_frames_[1];
        snapshot.startup_started = startup_wait_started_;
        snapshot.startup_complete = startup_complete_;
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

    struct FinalWork
    {
        ReservationSummary identity;
        const char *owner = "queued";
        std::optional<ImuInterval> imu_interval;
        std::size_t batch_samples = 0;
        // Convert exceptions to text so the final snapshot owns no exception payload either.
        std::string exception_message;
    };

    struct FinalCoverage
    {
        const char *source = "none";
        const char *status = "NoOutstanding";
        uint64_t enqueue_sequence = 0;
        std::optional<ImuInterval> interval;
        std::size_t batch_samples = 0;
        std::optional<std::size_t> buffered;
    };

    // Final-only metadata: never retain cv::Mat, an IMU vector, or queue element references.
    struct FinalSnapshot
    {
        Clock::time_point captured_at;
        uint64_t enqueued = 0;
        uint64_t processed = 0;
        uint64_t startup_discarded = 0;
        uint64_t overload_discarded = 0;
        std::size_t queued = 0;
        std::size_t in_flight = 0;
        std::size_t outstanding = 0;
        std::size_t peak = 0;
        bool startup_complete = false;
        std::optional<double> last_completed;
        // Populated by final_snapshot for both tracking modes.
        std::optional<bool> accounting_valid;
        bool identities_valid = true;
        std::vector<FinalWork> work;
        FinalCoverage coverage;
    };

    static const char *work_stage_name(TrackingWorkStage value)
    {
        switch (value)
        {
        case TrackingWorkStage::Reserved: return "Reserved";
        case TrackingWorkStage::Ready: return "Ready";
        case TrackingWorkStage::Executing: return "Executing";
        }
        return "Unknown";
    }

    static const char *batch_use_name(ImuBatchUse value)
    {
        switch (value)
        {
        case ImuBatchUse::NotRequired: return "NotRequired";
        case ImuBatchUse::NotAcquired: return "NotAcquired";
        case ImuBatchUse::ConsumedUnused: return "ConsumedUnused";
        case ImuBatchUse::DeliveredToBackend: return "DeliveredToBackend";
        }
        return "Unknown";
    }

    static const char *imu_status_name(ImuBatchStatus value)
    {
        switch (value)
        {
        case ImuBatchStatus::Stopped: return "Stopped";
        case ImuBatchStatus::Ready: return "Ready";
        case ImuBatchStatus::WaitingForData: return "WaitingForData";
        case ImuBatchStatus::MissingHistory: return "MissingHistory";
        case ImuBatchStatus::BufferOverflow: return "BufferOverflow";
        case ImuBatchStatus::DataGap: return "DataGap";
        case ImuBatchStatus::InvalidRequest: return "InvalidRequest";
        }
        return "Unknown";
    }

    static const char *work_location_name(WorkLocation value)
    {
        switch (value)
        {
        case WorkLocation::Coordination: return "Coordination";
        case WorkLocation::ImuQuery: return "ImuQuery";
        case WorkLocation::StartupDiscard: return "StartupDiscard";
        case WorkLocation::BeforeBackend: return "BeforeBackend";
        case WorkLocation::Backend: return "Backend";
        case WorkLocation::Completion: return "Completion";
        case WorkLocation::AfterBackend: return "AfterBackend";
        case WorkLocation::Finalization: return "Finalization";
        }
        return "Unknown";
    }

    static const char *work_reason_name(WorkReason value)
    {
        switch (value)
        {
        case WorkReason::StopObserved: return "StopObserved";
        case WorkReason::UnexpectedException: return "UnexpectedException";
        case WorkReason::ImuBackwards: return "ImuBackwards";
        case WorkReason::StartupTimeout: return "StartupTimeout";
        case WorkReason::FrameTimeout: return "FrameTimeout";
        case WorkReason::ImuQueryException: return "ImuQueryException";
        case WorkReason::ImuQueryResult: return "ImuQueryResult";
        case WorkReason::StartupDiscardException: return "StartupDiscardException";
        case WorkReason::BeforeBackendException: return "BeforeBackendException";
        case WorkReason::BackendException: return "BackendException";
        case WorkReason::AfterBackendException: return "AfterBackendException";
        }
        return "Unknown";
    }

    // Called only after callbacks finish. Queue metadata is captured together; IMU
    // inspection follows outside the queue lock and never reconsumes a saved interval.
    FinalSnapshot final_snapshot() const
    {
        FinalSnapshot final;
        final.captured_at = Clock::now();
        final.last_completed = last_tracked_frame_timestamp_;
        {
            const std::lock_guard<std::mutex> lock(queue_mutex_);
            final.enqueued = enqueued_frames_;
            final.processed = processed_frames_;
            final.startup_discarded = startup_discarded_frames_;
            final.overload_discarded = overload_discarded_frames_;
            final.queued = pending_frames_.size();
            final.in_flight = (reservation_ ? 1U : 0U) + (startup_next_reservation_ ? 1U : 0U);
            final.outstanding = final.queued + final.in_flight;
            final.peak = outstanding_frames_peak_;
            final.startup_complete = startup_complete_;
            final.work.reserve(final.outstanding);
            for (const PendingFrame &pending : pending_frames_)
            {
                FinalWork work;
                work.identity.timestamp = pending.frame.timestamp;
                work.identity.timestamp_ns = pending.frame.timestamp_ns;
                work.identity.enqueue_sequence = pending.enqueue_sequence;
                work.identity.received_at = pending.received_at;
                work.identity.batch_use = tracking_mode_ == TrackingMode::Stereo ?
                    ImuBatchUse::NotRequired : ImuBatchUse::NotAcquired;
                final.work.push_back(std::move(work));
            }
            const auto append_reserved = [&](const std::optional<ReservationSummary> &summary,
                                             const std::optional<TrackingWork> &payload,
                                             const char *owner) {
                if (!summary) return;
                FinalWork work;
                work.identity = *summary;
                work.owner = owner;
                if (payload)
                {
                    work.imu_interval = payload->imu_interval;
                    if (payload->imu_batch) work.batch_samples = payload->imu_batch->measurements.size();
                }
                final.work.push_back(std::move(work));
            };
            append_reserved(reservation_, tracking_work_, "primary");
            append_reserved(startup_next_reservation_, startup_next_work_, "startup_next");
        }

        // Sorting and exception formatting may allocate; neither belongs under a data lock.
        std::sort(final.work.begin(), final.work.end(), [](const FinalWork &left, const FinalWork &right) {
            return left.identity.enqueue_sequence < right.identity.enqueue_sequence;
        });
        uint64_t previous_sequence = 0;
        for (FinalWork &work : final.work)
        {
            final.identities_valid = final.identities_valid &&
                work.identity.enqueue_sequence > previous_sequence;
            previous_sequence = work.identity.enqueue_sequence;
            if (work.identity.interruption && work.identity.interruption->exception)
            {
                try { std::rethrow_exception(work.identity.interruption->exception); }
                catch (const std::exception &error) { work.exception_message = error.what(); }
                catch (...) { work.exception_message = "Unknown exception"; }
                work.identity.interruption->exception = nullptr;
                std::replace(work.exception_message.begin(), work.exception_message.end(), '\n', ' ');
                std::replace(work.exception_message.begin(), work.exception_message.end(), '\r', ' ');
            }
        }
        final.identities_valid = final.identities_valid && final.work.size() == final.outstanding;
        // Both modes account for every accepted frame, including an empty run.
        final.accounting_valid = final.enqueued == final.outstanding + final.processed +
            final.startup_discarded + final.overload_discarded;

        FinalCoverage &coverage = final.coverage;
        if (imu_frontend_) coverage.buffered = imu_frontend_->stats().buffered;
        if (final.work.empty()) return final;
        if (tracking_mode_ == TrackingMode::Stereo)
        {
            coverage.status = "NotRequired";
            return final;
        }
        // A saved F1 batch takes priority even if its empty-batch F0 failed before completion.
        for (const FinalWork &work : final.work)
        {
            if (work.identity.batch_use != ImuBatchUse::ConsumedUnused &&
                work.identity.batch_use != ImuBatchUse::DeliveredToBackend)
                continue;
            coverage.source = "saved_batch";
            coverage.status = work.imu_interval ? batch_use_name(work.identity.batch_use) : "MissingSavedInterval";
            coverage.enqueue_sequence = work.identity.enqueue_sequence;
            coverage.interval = work.imu_interval;
            coverage.batch_samples = work.batch_samples;
            return final;
        }

        // Inspect only the next scheduling interval; later queued work is not independently ready.
        if (!final.startup_complete)
        {
            if (final.work.size() < 2)
            {
                coverage.status = "AwaitingSecondFrame";
                return final;
            }
            coverage.interval = ImuInterval{final.work[0].identity.timestamp, final.work[1].identity.timestamp};
            coverage.enqueue_sequence = final.work[1].identity.enqueue_sequence;
        }
        else
        {
            if (!final.last_completed)
            {
                coverage.status = "MissingCompletedTimestamp";
                return final;
            }
            coverage.interval = ImuInterval{*final.last_completed, final.work.front().identity.timestamp};
            coverage.enqueue_sequence = final.work.front().identity.enqueue_sequence;
        }
        if (!imu_frontend_)
        {
            coverage.status = "FrontendUnavailable";
            return final;
        }
        coverage.source = "inspection";
        coverage.status = imu_status_name(imu_frontend_->inspectMeasurements(
            coverage.interval->left, coverage.interval->right));
        return final;
    }

    // Basic stop evidence is independent of the optional periodic timing diagnostics.
    void report_final_work(const FinalSnapshot &final)
    {
        node_logger_->info(
            "STOP_ACCOUNTING enqueued={} queued={} in_flight={} processed={} startup_discarded={} "
            "overload_discarded={} outstanding={} peak={} accounting={} identities_valid={}",
            final.enqueued, final.queued, final.in_flight, final.processed, final.startup_discarded,
            final.overload_discarded, final.outstanding, final.peak,
            final.accounting_valid.value_or(false) ? "Valid" : "Invalid",
            final.identities_valid);
        for (const FinalWork &work : final.work)
        {
            const ReservationSummary &identity = work.identity;
            const auto &cause = identity.interruption;
            node_logger_->info(
                "STOP_WORK enqueue_sequence={} timestamp_ns={} timestamp_sec={:.17g} owner={} "
                "received_steady_ns={} wait_ms={:.3f} stage={} batch_use={} batch_samples={} "
                "batch_interval_left_sec={} batch_interval_right_sec={} interruption={} reason={} "
                "location={} related_sequence={} failure_imu_status={} exception=\"{}\"",
                identity.enqueue_sequence, identity.timestamp_ns, identity.timestamp, work.owner,
                std::chrono::duration_cast<std::chrono::nanoseconds>(identity.received_at.time_since_epoch()).count(),
                std::chrono::duration<double, std::milli>(final.captured_at - identity.received_at).count(),
                std::string(work.owner) == "queued" ? "Queued" : work_stage_name(identity.stage),
                batch_use_name(identity.batch_use), work.batch_samples,
                work.imu_interval ? fmt::format("{:.17g}", work.imu_interval->left) : "none",
                work.imu_interval ? fmt::format("{:.17g}", work.imu_interval->right) : "none",
                cause ? (cause->kind == WorkInterruptionKind::Stopped ? "Stopped" : "Failed") : "none",
                cause ? work_reason_name(cause->reason) : "none",
                cause ? work_location_name(cause->location) : "none",
                cause ? cause->related_sequence : 0,
                cause && cause->imu_status ? imu_status_name(*cause->imu_status) : "none",
                work.exception_message);
        }
        const FinalCoverage &coverage = final.coverage;
        node_logger_->info(
            "STOP_IMU_COVERAGE steady_ns={} interval_left_sec={} interval_right_sec={} status={} "
            "buffered={} source={} enqueue_sequence={} batch_samples={}",
            std::chrono::duration_cast<std::chrono::nanoseconds>(final.captured_at.time_since_epoch()).count(),
            coverage.interval ? fmt::format("{:.17g}", coverage.interval->left) : "none",
            coverage.interval ? fmt::format("{:.17g}", coverage.interval->right) : "none",
            coverage.status, coverage.buffered ? std::to_string(*coverage.buffered) : "none",
            coverage.source, coverage.enqueue_sequence, coverage.batch_samples);
    }

    // Quiescent teardown only. The final snapshot remains the accounting record afterwards.
    void release_unfinished_work()
    {
        std::deque<PendingFrame> queued;
        {
            const std::lock_guard<std::mutex> lock(queue_mutex_);
            queued.swap(pending_frames_);
            reservation_.reset();
            startup_next_reservation_.reset();
        }
        // Pixel and IMU destruction must not extend the queue critical section.
        tracking_work_.reset();
        startup_next_work_.reset();
    }

    bool reserve_tracking_work()
    {
        // A waiting frame remains owned across retries.
        if (tracking_work_)
            return true;

        const std::lock_guard<std::mutex> lock(queue_mutex_);
        if (pending_frames_.empty())
            return false;

        // Preserve the original enqueue time when transferring ownership.
        tracking_work_.emplace(TrackingWork{
            pending_frames_.front(),
            TrackingWorkStage::Reserved,
            std::nullopt,
            tracking_mode_ == TrackingMode::Stereo ? ImuBatchUse::NotRequired : ImuBatchUse::NotAcquired,
            std::nullopt
        });
        reservation_.emplace(ReservationSummary{
            tracking_work_->pending.frame.timestamp,
            tracking_work_->pending.received_at,
            TrackingWorkStage::Reserved,
            tracking_work_->batch_use,
            tracking_work_->pending.enqueue_sequence,
            tracking_work_->pending.frame.timestamp_ns,
            std::nullopt
        });
        pending_frames_.pop_front();
        return true;
    }

    bool reserve_startup_pair()
    {
        // A partial execution must not be mistaken for a new startup candidate pair.
        if (!tracking_work_ && startup_next_work_)
            throw std::logic_error("Cannot reserve a new pair after F0 completion");

        if (tracking_work_ && startup_next_work_)
            return true;

        const std::lock_guard<std::mutex> lock(queue_mutex_);

        // Initially need two frames; after MissingHistory, retain F0 and refill F1.
        const std::size_t needed = tracking_work_ ? 1U : 2U;
        if (pending_frames_.size() < needed)
            return false;

        if (!tracking_work_)
        {
            // Retain the frame and its original deadline before removing the queue entry.
            tracking_work_.emplace(TrackingWork{
                pending_frames_.front(),
                TrackingWorkStage::Reserved,
                std::nullopt,
                ImuBatchUse::NotRequired,
                std::nullopt
            });

            reservation_.emplace(ReservationSummary{
                tracking_work_->pending.frame.timestamp,
                tracking_work_->pending.received_at,
                TrackingWorkStage::Reserved,
                tracking_work_->batch_use,
                tracking_work_->pending.enqueue_sequence,
                tracking_work_->pending.frame.timestamp_ns,
                std::nullopt
            });
            pending_frames_.pop_front();
        }

        startup_next_work_.emplace(TrackingWork{
            pending_frames_.front(),
            TrackingWorkStage::Reserved,
            std::nullopt,
            ImuBatchUse::NotAcquired,
            std::nullopt
        });
        startup_next_reservation_.emplace(ReservationSummary{
            startup_next_work_->pending.frame.timestamp,
            startup_next_work_->pending.received_at,
            TrackingWorkStage::Reserved,
            startup_next_work_->batch_use,
            startup_next_work_->pending.enqueue_sequence,
            startup_next_work_->pending.frame.timestamp_ns,
            std::nullopt
        });
        pending_frames_.pop_front();
        return true;
    }

    void discard_startup_first()
    {
        // MissingHistory may discard only candidates whose IMU batch is not consumed.
        if (!tracking_work_ || !startup_next_work_ ||
            tracking_work_->stage != TrackingWorkStage::Reserved ||
            startup_next_work_->stage != TrackingWorkStage::Reserved ||
            tracking_work_->imu_batch || startup_next_work_->imu_batch ||
            tracking_work_->interruption || startup_next_work_->interruption)
        {
            throw std::logic_error("Cannot discard a ready or incomplete startup pair");
        }

        const PendingFrame discarded = tracking_work_->pending;
        {
            const std::lock_guard<std::mutex> lock(queue_mutex_);

            // Promote F1's summary and account for F0's removal atomically.
            reservation_ = startup_next_reservation_;
            reservation_->batch_use = ImuBatchUse::NotRequired;
            startup_next_reservation_.reset();
            ++startup_discarded_frames_;
        }

        // Only tracking accesses these payloads; release the old F0 outside the lock.
        tracking_work_ = std::move(startup_next_work_);
        startup_next_work_.reset();
        // The promoted F1 becomes the next empty-batch F0.
        tracking_work_->batch_use = ImuBatchUse::NotRequired;
        node_logger_->warn(
            "Startup discard: reason=MissingHistory timestamp={:.9f} "
            "enqueue_sequence={} timestamp_ns={}",
            discarded.frame.timestamp, discarded.enqueue_sequence, discarded.frame.timestamp_ns);
    }

    // Tracking calls this, or main after callbacks finish; reception never mutates work.
    // Preserve the first cause when later catches or finalization observe the same work.
    void interrupt_reserved_work(const WorkInterruption &interruption)
    {
        const std::lock_guard<std::mutex> lock(queue_mutex_);
        const auto retain = [&](std::optional<TrackingWork> &work,
                                std::optional<ReservationSummary> &summary) {
            if (!work || work->interruption)
                return;
            work->interruption = interruption;
            if (summary)
                summary->interruption = interruption;
        };
        retain(tracking_work_, reservation_);
        retain(startup_next_work_, startup_next_reservation_);
    }

    bool observe_work_stop(WorkLocation location = WorkLocation::Coordination,
                           uint64_t related_sequence = 0)
    {
        if (!stop_control_->stop_requested())
            return false;
        interrupt_reserved_work(WorkInterruption{
            WorkInterruptionKind::Stopped, location, WorkReason::StopObserved,
            related_sequence, std::nullopt, {}});
        return true;
    }

    // Records call entry after the control gate grants logical backend start.
    // Keep this short lock outside backend timing probes and the backend call.
    void begin_reserved_backend(TrackingWorkSource source)
    {
        TrackingWork &work = source == TrackingWorkSource::PrimaryReservation ?
            *tracking_work_ : *startup_next_work_;
        if (work.interruption || work.stage != TrackingWorkStage::Ready)
            throw std::logic_error("Cannot execute interrupted or unready work");
        const std::lock_guard<std::mutex> lock(queue_mutex_);
        ReservationSummary &summary = source == TrackingWorkSource::PrimaryReservation ?
            *reservation_ : *startup_next_reservation_;
        work.stage = summary.stage = TrackingWorkStage::Executing;
        if (work.batch_use == ImuBatchUse::ConsumedUnused)
            work.batch_use = summary.batch_use = ImuBatchUse::DeliveredToBackend;
    }

    void on_input_activity()
    {
        if (input_timeout_sec_ == 0.0 || stop_control_->stop_requested())
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
        if (!last_input_activity_ || input_timeout_reported_ || stop_control_->stop_requested())
            return;

        // The timeout is armed only after input activity has established a baseline.
        const double idle_sec =
            std::chrono::duration<double>(Clock::now() - *last_input_activity_).count();
        if (idle_sec < input_timeout_sec_)
            return;

        input_timeout_reported_ = true;
        if (input_timeout_action_ == "shutdown")
        {
            // Publish the stop before cancellation or diagnostic logging can fail.
            stop_control_->request_stop(StopReason::InputIdle);
            callback_guard_->cancel();
            input_timer_->cancel();
        }

        node_logger_->warn("Image input timeout: idle_sec={:.3f} threshold_sec={:.3f} action={}",
                    idle_sec, input_timeout_sec_, input_timeout_action_.c_str());
        RCLCPP_WARN(get_logger(), "Image input timeout: idle_sec=%.3f threshold_sec=%.3f action=%s",
                    idle_sec, input_timeout_sec_, input_timeout_action_.c_str());
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
                        "enqueue_to_return_mean_ms={:.6f} enqueue_to_return_max_ms={:.6f} "
                        "in_flight={} outstanding={} overload_discarded={}",
                        final ? "true" : "false",
                        static_cast<unsigned long long>(queue.enqueued),
                        static_cast<unsigned long long>(queue.processed),
                        static_cast<unsigned long long>(queue.startup_discarded),
                        queue.pending, queue.peak, oldest_wait_sec,
                        queue.processed ? enqueue_to_return_sum_ms_ / queue.processed : 0.0,
                        enqueue_to_return_max_ms_, queue.in_flight, queue.outstanding,
                        static_cast<unsigned long long>(queue.overload_discarded));
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
            "non_cpu_mean_ms={:.3f} pending={} pending_peak={} oldest_queue_ms={:.3f} log_dropped={} "
            "in_flight={} outstanding={} enqueued={} processed={} startup_discarded={} overload_discarded={}",
            final, std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count(),
            elapsed, stats.calls, elapsed > 0.0 ? stats.calls / elapsed : 0.0,
            stats.calls ? stats.wall_sum_ms / stats.calls : 0.0, stats.wall_max_ms,
            stats.cpu_samples, stats.cpu_samples ? stats.cpu_sum_ms / stats.cpu_samples : 0.0,
            stats.cpu_samples ? stats.non_cpu_sum_ms / stats.cpu_samples : 0.0,
            queue.pending, queue.peak, oldest_ms, logging_->dropped_messages(),
            queue.in_flight, queue.outstanding, queue.enqueued, queue.processed,
            queue.startup_discarded, queue.overload_discarded);

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
        const std::vector<ImuMeasurement> &imu_measurements,
        std::optional<Clock::time_point> received_at,
        TrackingWorkSource work_source)
    {
        WorkInterruption failure;
        failure.location = WorkLocation::BeforeBackend;
        failure.reason = WorkReason::BeforeBackendException;
        if (work_source == TrackingWorkSource::PrimaryReservation)
            failure.related_sequence = tracking_work_->pending.enqueue_sequence;
        else if (work_source == TrackingWorkSource::StartupNextReservation)
            failure.related_sequence = startup_next_work_->pending.enqueue_sequence;
        try
        {
#ifdef GEMINI336_QUEUE_TEST
            if (test_work_point_) test_work_point_(TestWorkPoint::BeforeBackend);
            const TrackingState previous_state = TrackingState::NotInitialized;
#else
            const TrackingState previous_state = slam_->trackingState();
#endif

            if (trace_) trace_->record("track_begin", 0, frame.timestamp, imu_measurements.size());
            // Thread CPU excludes backend worker threads; the residual includes scheduling
            // and blocking, and cannot by itself identify a particular lock or scheduler cause.
            // Probes bracket the backend; their cost is excluded from wall_ms.
            // Snapshot before timing probes so lock contention is not backend CPU time.
            const QueueSnapshot queue = queue_snapshot();
#ifdef GEMINI336_QUEUE_TEST
            if (!test_track_)
                throw std::logic_error("Tracking test backend is not configured");
#endif
            if (observe_work_stop(WorkLocation::BeforeBackend, failure.related_sequence))
                return;
            const auto probe_start = Clock::now();
            const SchedulerSnapshot scheduler_before = slow_tracking_enabled_ ? scheduler_snapshot() : SchedulerSnapshot{};
#ifdef GEMINI336_QUEUE_TEST
            if (test_work_point_) test_work_point_(TestWorkPoint::BeforeBackendPermit);
#endif
            // Stop and start are serialized without holding a queue or IMU lock.
            if (!stop_control_->try_begin_backend())
            {
                observe_work_stop(WorkLocation::BeforeBackend, failure.related_sequence);
                return;
            }
#ifdef GEMINI336_QUEUE_TEST
            if (test_work_point_) test_work_point_(TestWorkPoint::AfterBackendPermit);
#endif
            // A granted call proceeds even if stop is published before physical entry.
            // Potentially throwing diagnostics finish before declaring backend entry.
            begin_reserved_backend(work_source);
            timespec cpu_start{}, cpu_end{};
            const bool cpu_started = tracking_diagnostics_enabled_ &&
                clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_start) == 0;
            const auto start = Clock::now();
            const double queue_before_ms = received_at ?
                std::chrono::duration<double, std::milli>(start - *received_at).count() :
                0.0;

            failure.location = WorkLocation::Backend;
            failure.reason = WorkReason::BackendException;
#ifdef GEMINI336_QUEUE_TEST
            test_track_(frame, imu_measurements);
#else
            if (tracking_mode_ == TrackingMode::Stereo)
                slam_->track(frame);
            else
                slam_->track(frame, imu_measurements);
#endif

            const auto end = Clock::now();
            failure.location = WorkLocation::Completion;
            failure.reason = WorkReason::AfterBackendException;
            const bool cpu_finished = cpu_started && clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_end) == 0;

            SchedulerSnapshot scheduler_after{};
            double probe_ms = 0.0;
            // A returned backend is processed even if its optional diagnostic probe fails.
            // Capture first, commit below, and only then propagate the diagnostic exception.
            std::exception_ptr probe_failure;
            try
            {
#ifdef GEMINI336_QUEUE_TEST
                if (test_work_point_) test_work_point_(TestWorkPoint::AfterBackendReturn);
#endif
                if (slow_tracking_enabled_)
                {
                    scheduler_after = scheduler_snapshot();
                    probe_ms =
                        std::chrono::duration<double, std::milli>(start - probe_start).count() +
                        std::chrono::duration<double, std::milli>(Clock::now() - end).count();
                }
            }
            catch (...) { probe_failure = std::current_exception(); }

            uint64_t processed;
            {
                const std::lock_guard<std::mutex> lock(queue_mutex_);

                // Commit queue removal and completion together before operational logging.
                switch (work_source)
                {
                case TrackingWorkSource::PrimaryReservation:
                    reservation_.reset();
                    break;
                case TrackingWorkSource::StartupNextReservation:
                    startup_next_reservation_.reset();
                    // F1 completion ends startup in the same accounting transaction.
                    startup_complete_ = true;
                    startup_wait_started_.reset();
                    break;
                }
                processed = ++processed_frames_;
            }

            // Only tracking owns this value; commit before any operational logging.
            if (tracking_mode_ == TrackingMode::StereoImu)
                last_tracked_frame_timestamp_ = frame.timestamp;

            if (work_source == TrackingWorkSource::PrimaryReservation)
                tracking_work_.reset();
            else if (work_source == TrackingWorkSource::StartupNextReservation)
                startup_next_work_.reset();

            failure.location = WorkLocation::AfterBackend;
            if (probe_failure) std::rethrow_exception(probe_failure);
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
#ifdef GEMINI336_QUEUE_TEST
            const TrackingState state = TrackingState::NotInitialized;
#else
            const auto state = slam_->trackingState();
#endif
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
        catch (...)
        {
            failure.exception = std::current_exception();
            if (failure.reason == WorkReason::BackendException)
                tracking_callback_reason_ = StopReason::BackendError;
            interrupt_reserved_work(failure);
            throw;
        }
    }

    void on_frame(const StereoFrame &frame)
    {
        if (stop_control_->stop_requested())
            return;

        const QueueFullPolicy full_policy =
            tracking_mode_ == TrackingMode::Stereo
                ? QueueFullPolicy::DiscardOldestQueued
                : QueueFullPolicy::Reject;

        enqueue_frame(frame, full_policy);
    }

    void tracking_callback() noexcept
    {
        tracking_callback_reason_ = StopReason::CallbackError;
        callback_guard_->run([this]() { process_pending_frames(); }, tracking_callback_reason_);
    }

    void process_pending_frames()
    {
        if (observe_work_stop())
            return;
        // A Reserved work item with a terminal cause must never become retryable.
        if ((tracking_work_ && tracking_work_->interruption) ||
            (startup_next_work_ && startup_next_work_->interruption))
            throw std::logic_error("Cannot retry interrupted tracking work");
        WorkInterruption failure;
        try
        {
            if (tracking_mode_ == TrackingMode::Stereo)
            {
                if (!reserve_tracking_work())
                    return;

                if (tracking_work_->stage != TrackingWorkStage::Reserved)
                    throw std::logic_error("Cannot resample a ready or executing frame");

                const StereoFrame frame = tracking_work_->pending.frame;
                const Clock::time_point received_at =
                    tracking_work_->pending.received_at;

                failure.related_sequence = tracking_work_->pending.enqueue_sequence;

                tracking_work_->stage = TrackingWorkStage::Ready;
                {
                    const std::lock_guard<std::mutex> lock(queue_mutex_);
                    reservation_->stage = TrackingWorkStage::Ready;
                }

                // Preserve unfinished Stereo work if stopping before backend execution.
#ifdef GEMINI336_QUEUE_TEST
                if (test_work_point_) test_work_point_(TestWorkPoint::AfterReady);
#endif
                if (observe_work_stop(WorkLocation::BeforeBackend))
                    return;

                track_frame(
                    frame, {}, received_at,
                    TrackingWorkSource::PrimaryReservation);
                return;
            }

            // A rejected backwards sample is evidence of a discontinuous input timeline.
            // Check even with an empty image queue; do not silently resume after a reset.
            const ImuFrontendStats imu_stats = imu_frontend_->stats();
            if (imu_stats.backwards > 0)
            {
                failure.reason = WorkReason::ImuBackwards;
                throw CallbackFailure(StopReason::SamplingError, "IMU timestamp moved backwards: count=" +
                                         std::to_string(imu_stats.backwards));
            }

            failure.reason = WorkReason::UnexpectedException;
            const QueueSnapshot queue = queue_snapshot();
            const Clock::time_point now = Clock::now();

            // Keep startup bounded even when early images are discarded.
            if (!queue.startup_complete && queue.startup_started)
            {
                const double startup_wait_sec =
                    std::chrono::duration<double>(now - *queue.startup_started).count();
                if (startup_wait_sec >= imu_wait_timeout_sec_)
                {
                    failure.reason = WorkReason::StartupTimeout;
                    throw_wait_timeout("Stereo-IMU startup", startup_wait_sec);
                }
            }

            // Reserved frames retain their original enqueue deadline even when the queue is empty.
            if (queue.oldest_received_at)
            {
                const double frame_wait_sec =
                    std::chrono::duration<double>(now - *queue.oldest_received_at).count();
                if (frame_wait_sec >= imu_wait_timeout_sec_)
                {
                    failure.reason = WorkReason::FrameTimeout;
                    throw_wait_timeout("Stereo frame", frame_wait_sec);
                }
            }

            // Verify the first interval before sending either startup image.
            if (!queue.startup_complete)
            {
                if (!reserve_startup_pair())
                    return;

                // Never consume the same interval again after Ready or backend execution.
                if (tracking_work_->stage != TrackingWorkStage::Reserved ||
                    startup_next_work_->stage != TrackingWorkStage::Reserved)
                {
                    throw std::logic_error("Cannot resample a ready or executing startup pair");
                }

                const double t0 = tracking_work_->pending.frame.timestamp;
                const double t1 = startup_next_work_->pending.frame.timestamp;
                failure.location = WorkLocation::ImuQuery;
                failure.reason = WorkReason::ImuQueryException;
                failure.related_sequence = startup_next_work_->pending.enqueue_sequence;
#ifdef GEMINI336_QUEUE_TEST
                if (test_work_point_) test_work_point_(TestWorkPoint::BeforeImuQuery);
#endif
                ImuBatch batch = imu_frontend_->takeMeasurements(t0, t1);
                failure.reason = WorkReason::ImuQueryResult;
                failure.imu_status = batch.status;

                switch (batch.status)
                {
                case ImuBatchStatus::Stopped:
                    observe_work_stop(WorkLocation::ImuQuery);
                    return;
                case ImuBatchStatus::WaitingForData:
#ifdef GEMINI336_QUEUE_TEST
                    if (test_work_point_) test_work_point_(TestWorkPoint::AfterWaiting);
#endif
                    observe_work_stop();
                    return;
                case ImuBatchStatus::MissingHistory:
                    failure.location = WorkLocation::StartupDiscard;
                    failure.reason = WorkReason::StartupDiscardException;
                    failure.related_sequence = tracking_work_->pending.enqueue_sequence;
                    discard_startup_first();
                    return;
                case ImuBatchStatus::BufferOverflow:
                    throw_batch_error("IMU buffer overflow during startup", t0, t1);
                case ImuBatchStatus::DataGap:
                    throw_batch_error("IMU data gap detected during startup", t0, t1);
                case ImuBatchStatus::InvalidRequest:
                    throw_batch_error("IMU startup batch request is invalid", t0, t1);
                case ImuBatchStatus::Ready:
                {
                    // F1 owns the consumed interval; F0 is tracked with an empty batch.
                    startup_next_work_->imu_batch.emplace(std::move(batch));
                    startup_next_work_->imu_interval = ImuInterval{t0, t1};
                    startup_next_work_->batch_use = ImuBatchUse::ConsumedUnused;
                    failure = WorkInterruption{};
                    tracking_work_->stage = TrackingWorkStage::Ready;
                    startup_next_work_->stage = TrackingWorkStage::Ready;
                    {
                        const std::lock_guard<std::mutex> lock(queue_mutex_);
                        reservation_->stage = TrackingWorkStage::Ready;
                        startup_next_reservation_->stage = TrackingWorkStage::Ready;
                        startup_next_reservation_->batch_use = ImuBatchUse::ConsumedUnused;
                    }

#ifdef GEMINI336_QUEUE_TEST
                    if (test_work_point_) test_work_point_(TestWorkPoint::AfterReady);
#endif
                    if (observe_work_stop(WorkLocation::BeforeBackend))
                        return;

                    // Keep local values alive after track_frame clears the completed work.
                    const StereoFrame first_frame = tracking_work_->pending.frame;
                    const Clock::time_point first_received_at =
                        tracking_work_->pending.received_at;
                    track_frame(
                        first_frame, {}, first_received_at,
                        TrackingWorkSource::PrimaryReservation);

                    // Preserve F1 and its consumed batch if stopping after F0 completes.
                    // Do not introduce another timeout check between F0 and F1.
                    if (observe_work_stop(WorkLocation::AfterBackend))
                        return;

                    const StereoFrame second_frame = startup_next_work_->pending.frame;
                    const Clock::time_point second_received_at =
                        startup_next_work_->pending.received_at;
                    track_frame(
                        second_frame, startup_next_work_->imu_batch->measurements,
                        second_received_at, TrackingWorkSource::StartupNextReservation);
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
            failure.location = WorkLocation::ImuQuery;
            failure.reason = WorkReason::ImuQueryException;
            failure.related_sequence = tracking_work_->pending.enqueue_sequence;
#ifdef GEMINI336_QUEUE_TEST
            if (test_work_point_) test_work_point_(TestWorkPoint::BeforeImuQuery);
#endif
            ImuBatch batch = imu_frontend_->takeMeasurements(
                *last_tracked_frame_timestamp_, frame.timestamp);
            failure.reason = WorkReason::ImuQueryResult;
            failure.imu_status = batch.status;

            switch (batch.status)
            {
            case ImuBatchStatus::Stopped:
                observe_work_stop(WorkLocation::ImuQuery);
                return;
            case ImuBatchStatus::WaitingForData:
#ifdef GEMINI336_QUEUE_TEST
                if (test_work_point_) test_work_point_(TestWorkPoint::AfterWaiting);
#endif
                observe_work_stop();
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
            {
                tracking_work_->imu_batch.emplace(std::move(batch));
                tracking_work_->imu_interval = ImuInterval{*last_tracked_frame_timestamp_, frame.timestamp};
                tracking_work_->batch_use = ImuBatchUse::ConsumedUnused;
                failure = WorkInterruption{};
                tracking_work_->stage = TrackingWorkStage::Ready;
                {
                    const std::lock_guard<std::mutex> lock(queue_mutex_);
                    reservation_->stage = TrackingWorkStage::Ready;
                    reservation_->batch_use = ImuBatchUse::ConsumedUnused;
                }

                // Preserve consumed data if stopping before backend execution.
#ifdef GEMINI336_QUEUE_TEST
                if (test_work_point_) test_work_point_(TestWorkPoint::AfterReady);
#endif
                if (observe_work_stop(WorkLocation::BeforeBackend))
                    return;

                track_frame(
                    frame, tracking_work_->imu_batch->measurements, received_at, TrackingWorkSource::PrimaryReservation);
                return;
            }
            }
        }
        catch (...)
        {
            failure.exception = std::current_exception();
            interrupt_reserved_work(failure);
            throw;
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
        throw CallbackFailure(StopReason::SamplingError, message.str());
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
        throw CallbackFailure(StopReason::Timeout, message.str());
    }

    static void require_absolute_path(const std::string &path, const char *name)
    {
        if (path.empty() || !std::filesystem::path(path).is_absolute())
            throw std::invalid_argument(std::string(name) + " must be a nonempty absolute path");
    }

    enum class QueueFullPolicy
    {
        Reject,
        DiscardOldestQueued
    };

    void enqueue_frame(
        const StereoFrame &frame, QueueFullPolicy full_policy = QueueFullPolicy::Reject)
    {
        // Stereo reception explicitly selects replacement; other callers default to rejection.
        if (full_policy == QueueFullPolicy::DiscardOldestQueued &&
            tracking_mode_ != TrackingMode::Stereo)
            throw std::invalid_argument("Only Stereo may discard queued frames on overload");

        // Reject invalid sensor timestamp before changing any queue state.
        if (!std::isfinite(frame.timestamp))
            throw std::invalid_argument("Stereo timestamp must be finite");
        if (tracking_mode_ == TrackingMode::StereoImu && frame.timestamp < 0.0)
            throw std::invalid_argument("Stereo-IMU timestamp must be nonnegative");

        // Retain dropped pixels until after unlocking; destruction need not block admission.
        std::optional<PendingFrame> discarded;
        std::unique_lock<std::mutex> lock(queue_mutex_);
        // A passed check admits this transaction; a concurrent stop does not roll it back.
        if (stop_control_->stop_requested())
            return;
        const std::size_t in_flight =
            (reservation_.has_value() ? 1U : 0U) +
            (startup_next_reservation_.has_value() ? 1U : 0U);
        const std::size_t outstanding = pending_frames_.size() + in_flight;

        if (last_received_frame_timestamp_ &&
            frame.timestamp <= *last_received_frame_timestamp_)
        {
            lock.unlock();
            throw std::invalid_argument("Stereo timestamps must be strictly increasing");
        }

        const bool full = outstanding >= pending_frames_capacity_;
        // Reserved work is never eligible, even if it occupies the entire capacity.
        if (full && (full_policy == QueueFullPolicy::Reject || pending_frames_.empty()))
        {
            const std::size_t pending = pending_frames_.size();
            lock.unlock();
            std::ostringstream message;
            message << std::setprecision(17) << "Pending frame queue is full: capacity="
                    << pending_frames_capacity_ << " pending=" << pending
                    << " incoming_timestamp=" << frame.timestamp
                    << " in_flight=" << in_flight
                    << " outstanding=" << outstanding;
            throw CallbackFailure(StopReason::Capacity, message.str());
        }

        if (full)
            discarded = pending_frames_.front();

        // Allocate the new deque entry before eviction so allocation failure loses no work.
        // A temporary extra entry is private to this lock; the committed total stays bounded.
        const uint64_t enqueue_sequence = enqueued_frames_ + 1;
        pending_frames_.push_back({frame, Clock::now(), enqueue_sequence});
        if (discarded)
        {
            pending_frames_.pop_front();
            ++overload_discarded_frames_;
        }
        last_received_frame_timestamp_ = frame.timestamp;
        ++enqueued_frames_;
        const std::size_t admitted_outstanding = pending_frames_.size() + in_flight;
        outstanding_frames_peak_ = std::max(outstanding_frames_peak_, admitted_outstanding);

        if (!startup_complete_ && !startup_wait_started_)
            startup_wait_started_ = pending_frames_.back().received_at;

        // Record only after the complete accounting transition, in queue -> trace order.
        if (trace_)
        {
            if (discarded)
                trace_->record("frame_overload_discarded", 0,
                               discarded->frame.timestamp, frame.timestamp);
            trace_->record("frame_enqueued", 0, frame.timestamp, pending_frames_.size());
        }
        const uint64_t overload_discarded = overload_discarded_frames_;
        lock.unlock();

        if (discarded)
        {
            node_logger_->warn(
                "Stereo overload discard: timestamp={:.9f} incoming_timestamp={:.9f} "
                "overload_discarded={} outstanding={} enqueue_sequence={} timestamp_ns={} "
                "incoming_enqueue_sequence={} incoming_timestamp_ns={}",
                discarded->frame.timestamp, frame.timestamp,
                static_cast<unsigned long long>(overload_discarded), admitted_outstanding,
                discarded->enqueue_sequence, discarded->frame.timestamp_ns,
                enqueue_sequence, frame.timestamp_ns);
        }
    }

    // Declared before all producers so the logging backend is destroyed last.
    std::shared_ptr<LoggingSession> logging_;
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
    // Keep groups alive until their timers and frontends are destroyed.
    rclcpp::CallbackGroup::SharedPtr reception_group_;
    rclcpp::CallbackGroup::SharedPtr tracking_group_;

    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    // Shared with main through callback execution and finalization.
    std::shared_ptr<StopControl> stop_control_;
    // Cancels spin; main owns teardown after callbacks finish.
    std::function<void()> request_stop_;
    std::shared_ptr<CallbackGuard> callback_guard_;
    // Tracking-group only; preserves the original backend exception at the outer boundary.
    StopReason tracking_callback_reason_ = StopReason::CallbackError;
    double input_timeout_sec_ = 5.0;
    std::string input_timeout_action_;
    std::optional<Clock::time_point> last_input_activity_;
    bool input_timeout_reported_ = false;
    // Main-only lifecycle state; repeated cleanup never repeats trace export or backend calls.
    bool finalization_started_ = false;
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
    uint64_t overload_discarded_frames_ = 0;
    uint64_t enqueued_frames_ = 0;
    std::size_t outstanding_frames_peak_ = 0;
    double enqueue_to_return_sum_ms_ = 0.0;
    double enqueue_to_return_max_ms_ = 0.0;
    int64_t tracking_retry_period_ms_ = 5;
    std::size_t pending_frames_capacity_ = 30;
    std::optional<double> last_received_frame_timestamp_;
    std::optional<Clock::time_point> startup_wait_started_;
    bool startup_complete_ = false;
    rclcpp::TimerBase::SharedPtr tracking_timer_;

    // Tracking owns these; main may read only after all callbacks have finished.
    std::optional<double> last_tracked_frame_timestamp_;
    // Retain work across retries, including waits for IMU coverage.
    std::optional<TrackingWork> tracking_work_;
    std::optional<TrackingWork> startup_next_work_;
};
}

#ifndef GEMINI336_QUEUE_TEST
int main(int argc, char **argv)
{
    std::exception_ptr startup_failure;
    std::shared_ptr<gemini336_orbslam3::LoggingSession> logging;
    bool logging_finished = false;
    std::shared_ptr<gemini336_orbslam3::SlamNode> node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::shared_ptr<gemini336_orbslam3::StopControl> stop_control;
    rclcpp::Context::SharedPtr context;
    std::unique_ptr<gemini336_orbslam3::ContextStopRegistration> context_stop;

    try
    {
        context = rclcpp::contexts::get_global_default_context();
        rclcpp::init(argc, argv);
        stop_control = std::make_shared<gemini336_orbslam3::StopControl>();
        context_stop = std::make_unique<gemini336_orbslam3::ContextStopRegistration>(
            context, stop_control);
        // Do not start constructing SLAM when shutdown preceded registration.
        if (context->is_valid())
        {
            rclcpp::ExecutorOptions options;
            options.context = context;
            executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(options);
            node = std::make_shared<gemini336_orbslam3::SlamNode>(
                stop_control, [&executor]() { executor->cancel(); }, logging);
            executor->add_node(node);

            // Serial execution protects the frontend, adapter and statistics from overlap.
            executor->spin();
        }
    }
    catch (...)
    {
        startup_failure = std::current_exception();
    }

    // With SingleThreadedExecutor, returning or unwinding spin leaves no callback
    // worker running. A future multi-threaded switch must re-establish this premise.
    gemini336_orbslam3::ProcessCleanup cleanup;
    cleanup.finalize_node = [&]() { if (node) node->shutdown(); };
    cleanup.detach_context = [&]() {
        if (context_stop && !context_stop->close())
            throw std::runtime_error("Failed to remove context shutdown callback");
        context_stop.reset();
    };
    cleanup.release_executor = [&]() { executor.reset(); };
    cleanup.release_node = [&]() { node.reset(); };
    cleanup.shutdown_context = [&]() {
        // The rclcpp wrapper also removes global signal handlers, even if the
        // context was already shut down by a signal or callback failure.
        if (context) rclcpp::shutdown(context, "Process finalization");
    };
    cleanup.finish_logging = [&]() {
        // Report failures after this point only to stderr, never to a closed async pool.
        logging_finished = true;
        if (logging) logging->finish();
    };
    cleanup.release_logging = [&]() { logging.reset(); };
    const auto report = [&](std::exception_ptr error) {
        const auto emit = [&](const char *message) {
            std::fprintf(stderr, "SLAM process failure: %s\n", message);
            if (logging && !logging_finished) logging->GetLogger("slam_node")->error("{}", message);
        };
        try { std::rethrow_exception(error); }
        catch (const std::exception &e) { emit(e.what()); }
        catch (...) { emit("Unknown exception"); }
    };
    return gemini336_orbslam3::finalize_process(stop_control, startup_failure, cleanup, report);
}

#endif  // GEMINI336_QUEUE_TEST
