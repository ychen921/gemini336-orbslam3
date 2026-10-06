#include "gemini336_orbslam3/logging.hpp"
#include "slam/orbslam3_adapter.hpp"
#include "slam/tracking_coordinator.hpp"
#include "frontend/stereo_frontend.hpp"
#include "frontend/imu_frontend.hpp"
#include "common/stop_control.hpp"
#include "common/viewer_stop_notification.hpp"
#include "common/callback_guard.hpp"
#include "common/context_stop_registration.hpp"
#include "common/process_finalization.hpp"
#include "common/process_evidence.hpp"
#include <cstdio>
#include <cstdlib>

#include <cstdint>
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
          stop_control_(std::move(stop_control))
    {
        if (!stop_control_)
        {
            throw std::invalid_argument("SlamNode: stop_control must not be null");
        }
        if (!request_stop)
        {
            throw std::invalid_argument("SlamNode: request_stop must not be empty");
        }

        const auto context = get_node_base_interface()->get_context();
        const ViewerStopNotification viewer_stop(stop_control_, request_stop,
            [context]() { context->shutdown("Viewer executor cancel failed"); });
        callback_guard_ = std::make_shared<CallbackGuard>(stop_control_, std::move(request_stop),
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
        // Context notifications retain only StopControl. The backend observes the
        // same atomic gate before executor callbacks have necessarily returned.
        slam_ = std::make_unique<OrbSlam3Adapter>(config, [control = stop_control_]() noexcept {
            return control->stop_requested();
        }, viewer_stop,
        [session = std::weak_ptr<LoggingSession>(logging_)](const std::string &module,
                                                         bool synchronous) {
            // Factory use is setup-only; retain no node or session ownership in core.
            const auto logging = session.lock();
            if (!logging) throw std::logic_error("ORB-SLAM3 logging session expired");
            return synchronous ? logging->GetSynchronousLogger(module) : logging->GetLogger(module);
        });

        initialize_frontends_and_timers(left_topic, right_topic);

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

        // A failed snapshot stays absent; never print zero accounting as a substitute.
        std::optional<FinalSnapshot> final;
        finalize_step("snapshot", StopReason::ShutdownError, [&]() {
            final.emplace(coordinator_->snapshot());
        });
        finalize_step("work_report", StopReason::ShutdownError, [&]() {
            if (final) coordinator_->report_final_accounting(*final);
            else node_logger_->error("Final work snapshot unavailable");
        });
        finalize_step("imu_report", StopReason::ShutdownError, [this]() {
            if (!imu_frontend_) return;
            const ImuFrontendStats stats = imu_frontend_->stats();
            node_logger_->info("Final IMU input: received={} accepted={} stopped={} backwards={} "
                        "overflow={} buffered={} unavailable={} invalid_values={} invalid_timestamps={} "
                        "timestamp_precision_rejections={} duplicates={}",
                        static_cast<unsigned long long>(stats.received),
                        static_cast<unsigned long long>(stats.accepted),
                        static_cast<unsigned long long>(stats.stopped),
                        static_cast<unsigned long long>(stats.backwards),
                        static_cast<unsigned long long>(stats.overflow), stats.buffered, stats.unavailable,
                        stats.invalid_values, stats.invalid_timestamps,
                        stats.timestamp_precision_rejections, stats.duplicates);
        });
        finalize_step("statistics_report", StopReason::ShutdownError, [this]() { coordinator_->report(true); });
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
        finalize_step("release_work", StopReason::ShutdownError, [this]() { coordinator_->release_unfinished_work(); });
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
    // Production and finite executor tests share the same callback ownership and wiring.
    // The backend (or test replacement) and callback guard must already exist.
    void initialize_frontends_and_timers(const std::string &left_topic,
                                        const std::string &right_topic)
    {
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

        initialize_coordinator();

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
        report_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            [this]() { callback_guard_->run([this]() { coordinator_->report(false); }); },
            tracking_group_);

        if (input_timeout_sec_ > 0.0)
            input_timer_ = create_wall_timer(
                std::chrono::milliseconds(100),
                [this]() { callback_guard_->run([this]() { check_input_timeout(); }); },
                reception_group_);

    }

    void initialize_coordinator()
    {
#ifdef GEMINI336_QUEUE_TEST
        if (coordinator_)
        {
            coordinator_->tracking_mode_ = tracking_mode_;
            coordinator_->pending_frames_capacity_ = pending_frames_capacity_;
            coordinator_->imu_wait_timeout_sec_ = imu_wait_timeout_sec_;
            coordinator_->node_logger_ = node_logger_;
            coordinator_->trace_ = trace_.get();
            coordinator_->imu_frontend_ = imu_frontend_.get();
            return;
        }
#endif
        coordinator_ = std::make_unique<TrackingCoordinator>(
            TrackingCoordinator::Config{tracking_mode_, pending_frames_capacity_, imu_wait_timeout_sec_},
            stop_control_, node_logger_, trace_.get(), slam_.get(), imu_frontend_.get(),
            [this](double timestamp) {
                RCLCPP_INFO(get_logger(), "First stereo frame processed: timestamp=%.9f", timestamp);
            },
            [this](TrackingState previous, TrackingState state) {
                RCLCPP_INFO(get_logger(), "Tracking state: %s -> %s",
                            tracking_state_name(previous), tracking_state_name(state));
            });
    }

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
    friend struct SlamExecutorTestAccess;
    friend struct SlamSnapshotTestAccess;
    struct QueueTestTag {};
    explicit SlamNode(QueueTestTag, const rclcpp::NodeOptions &options = rclcpp::NodeOptions{});
    std::function<void(const char *)> test_finalize_step_;
    std::function<void()> test_shutdown_;
#endif
    using Clock = std::chrono::steady_clock;
    using FinalSnapshot = TrackingCoordinator::FinalSnapshot;

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

    void on_frame(const StereoFrame &frame)
    {
        if (stop_control_->stop_requested())
            return;

        coordinator_->enqueue(frame);
    }

    void tracking_callback() noexcept
    {
        tracking_callback_reason_ = StopReason::CallbackError;
        callback_guard_->run([this]() { coordinator_->process_pending_frames(tracking_callback_reason_); }, tracking_callback_reason_);
    }

    // Preserve sub-microsecond timestamp detail in interval failure diagnostics.

    static void require_absolute_path(const std::string &path, const char *name)
    {
        if (path.empty() || !std::filesystem::path(path).is_absolute())
            throw std::invalid_argument(std::string(name) + " must be a nonempty absolute path");
    }

    // Declared before all producers so the logging backend is destroyed last.
    std::shared_ptr<LoggingSession> logging_;
    std::shared_ptr<spdlog::logger> node_logger_;
    // Keep groups alive until their timers and frontends are destroyed.
    rclcpp::CallbackGroup::SharedPtr reception_group_;
    rclcpp::CallbackGroup::SharedPtr tracking_group_;

    // Shared with main through callback execution and finalization.
    std::shared_ptr<StopControl> stop_control_;
    std::shared_ptr<CallbackGuard> callback_guard_;
    // Tracking-group only; preserves the original backend exception at the outer boundary.
    StopReason tracking_callback_reason_ = StopReason::CallbackError;
    double input_timeout_sec_ = 5.0;
    std::string input_timeout_action_;
    // Raw image activity and the idle timer share reception_group_; tracking never reads these.
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

    std::unique_ptr<TrackingCoordinator> coordinator_;
    std::string imu_topic_;
    TrackingMode tracking_mode_ = TrackingMode::Stereo;
    double imu_wait_timeout_sec_ = 1.0;
    int64_t tracking_retry_period_ms_ = 5;
    std::size_t pending_frames_capacity_ = 30;
    rclcpp::TimerBase::SharedPtr tracking_timer_;
    rclcpp::TimerBase::SharedPtr report_timer_;

};
}

#ifndef GEMINI336_QUEUE_TEST
int main(int argc, char **argv)
{
    std::exception_ptr startup_failure;
    bool callbacks_quiescent = true;
    std::shared_ptr<gemini336_orbslam3::LoggingSession> logging;
    bool logging_finished = false;
    gemini336_orbslam3::LoggingEvidence logging_evidence;
    std::shared_ptr<gemini336_orbslam3::SlamNode> node;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
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
            executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(options, 2);
            node = std::make_shared<gemini336_orbslam3::SlamNode>(
                stop_control, [&executor]() { executor->cancel(); }, logging);
            executor->add_node(node);

            // Humble joins executor workers on normal spin return. CallbackGuard
            // contains project callback failures so they use that controlled path.
            // A cancel before spin is not sufficient. Keep a persistent stop gate
            // visible inside spin as well, covering the check-to-spin race.
            const auto stop_timer = node->create_wall_timer(std::chrono::milliseconds(10),
                [control = stop_control,
                 notify = gemini336_orbslam3::ViewerStopNotification(stop_control,
                     [executor_ptr = executor.get()]() { executor_ptr->cancel(); },
                     [context]() { context->shutdown("Stop timer cancel failed"); })]() {
                    if (control->stop_requested()) notify({});
                });
            if (!stop_control->stop_requested() && context->is_valid())
            {
                callbacks_quiescent = false;
                executor->spin();
                callbacks_quiescent = true;
            }
            stop_timer->cancel();
        }
    }
    catch (...)
    {
        // An escaping executor-internal failure does not establish worker quiescence.
        // Do not destroy objects that callbacks might still use. Some upstream
        // worker failures terminate before reaching this handler; neither is recoverable.
        if (!callbacks_quiescent)
        {
            std::fputs("SLAM executor failed without confirmed worker join; terminating without cleanup\n",
                       stderr);
            std::fflush(stderr);
            std::_Exit(EXIT_FAILURE);
        }
        startup_failure = std::current_exception();
    }

    // No spin started, or spin returned normally after joining all executor workers.
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
        if (logging)
        {
            logging_evidence.status = "failed";
            try { logging->finish(); logging_evidence.status = "ok"; }
            catch (...) {
                logging_evidence.dropped = logging->dropped_messages();
                throw;
            }
            logging_evidence.dropped = logging->dropped_messages();
        }
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
    const int result = gemini336_orbslam3::finalize_process(stop_control, startup_failure, cleanup, report);
    return gemini336_orbslam3::write_process_evidence(stderr, stop_control, logging_evidence, result);
}

#endif  // GEMINI336_QUEUE_TEST
