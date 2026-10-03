// Finite executor integration: real frontend/timer wiring, no DDS player or ORB-SLAM3.
#include "../src/slam_node.cpp"

#include <condition_variable>
#include <iostream>
#include <thread>
#include <spdlog/sinks/base_sink.h>

namespace gemini336_orbslam3
{
struct StereoFrontendTestAccess
{
    static void receive(StereoFrontend &frontend, int seconds)
    {
        auto image = std::make_shared<StereoFrontend::Image>();
        image->header.stamp.sec = seconds;
        image->height = image->width = image->step = 1;
        image->encoding = "mono8";
        image->data = {42};
        // A reception-group timer supplies an already synchronized pair. Conversion,
        // on_frame, admission, scheduling and completion remain production code.
        frontend.stereo_callback(image, image);
    }

    static bool in_group(StereoFrontend &frontend,
                         const rclcpp::CallbackGroup::SharedPtr &group)
    {
        return group->find_subscription_ptrs_if([&](const auto &subscription) {
            return subscription == frontend.left_sub_.getSubscriber();
        }) && group->find_subscription_ptrs_if([&](const auto &subscription) {
            return subscription == frontend.right_sub_.getSubscriber();
        });
    }
};

struct ImuFrontendTestAccess
{
    static void receive(ImuFrontend &frontend, int seconds)
    {
        auto message = std::make_shared<sensor_msgs::msg::Imu>();
        message->header.stamp.sec = seconds;
        message->header.frame_id = "imu";
        message->linear_acceleration.z = 9.8;
        frontend.imu_callback(message);
    }

    static std::optional<double> consumed_until(ImuFrontend &frontend)
    {
        const std::lock_guard<std::mutex> lock(frontend.imu_mutex_);
        return frontend.last_taken_timestamp_;
    }

    static bool in_group(ImuFrontend &frontend,
                         const rclcpp::CallbackGroup::SharedPtr &group)
    {
        return bool(group->find_subscription_ptrs_if([&](const auto &subscription) {
            return subscription == frontend.imu_sub_;
        }));
    }
};

struct SlamExecutorTestAccess
{
    using Clock = std::chrono::steady_clock;

    static void require(bool condition, const char *message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    static std::string read_file(const std::filesystem::path &path)
    {
        std::ifstream input(path);
        require(input.good(), "missing test artifact");
        return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    }

    // Gates establish ordering rather than relying on scheduler timing or sleeps.
    struct Gate
    {
        std::mutex mutex;
        std::condition_variable cv;
        bool reached = false;
        bool released = false;

        void block()
        {
            std::unique_lock<std::mutex> lock(mutex);
            reached = true;
            cv.notify_all();
            require(cv.wait_for(lock, std::chrono::seconds(3), [&]() { return released; }),
                    "callback gate timed out");
        }
        bool entered()
        {
            const std::lock_guard<std::mutex> lock(mutex);
            return reached;
        }
        void release()
        {
            const std::lock_guard<std::mutex> lock(mutex);
            released = true;
            cv.notify_all();
        }
    };

    struct Fixture
    {
        // Each case owns its context so cancel-fallback and shutdown cases are independent.
        rclcpp::Context::SharedPtr context = std::make_shared<rclcpp::Context>();
        std::shared_ptr<SlamNode> node;
        std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
        std::unique_ptr<ContextStopRegistration> registration;
        std::shared_ptr<LoggingSession> logging;
        std::filesystem::path directory;
        std::filesystem::path log_path;
        std::string trace_csv;
        std::string log_text;
        bool cancel_fails = false; // Configured before spin, then read-only.
        std::atomic<unsigned> fallback_calls{0};
        Gate gate;
        std::mutex mutex;
        std::condition_variable cv;
        bool reception_done = false;
        bool spin_returned = false;
        std::atomic<unsigned> backend_calls{0};
        std::vector<rclcpp::TimerBase::SharedPtr> timers;
        std::thread runner;

        explicit Fixture(bool imu, std::size_t trace_capacity = 256)
        {
            context->init(0, nullptr);
            rclcpp::NodeOptions options;
            options.context(context);
            node.reset(new SlamNode(SlamNode::QueueTestTag{}, options));
            rclcpp::ExecutorOptions executor_options;
            executor_options.context = context;
            executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(executor_options, 2);
            directory = std::filesystem::temp_directory_path() /
                ("gemini-executor-" + std::to_string(Clock::now().time_since_epoch().count()));
            LoggingOptions logging_options;
            logging_options.directory = directory;
            logging = std::make_shared<LoggingSession>(logging_options);
            log_path = logging->directory() / "slam.log";
            node->logging_ = logging;
            node->trace_ = std::make_unique<DiagnosticTrace>((directory / "trace.csv").string(), trace_capacity);
            node->tracking_diagnostics_enabled_ = true;
            node->diagnostics_logger_ = logging->GetLogger("diagnostics");
            node->tracking_mode_ = imu ? TrackingMode::StereoImu : TrackingMode::Stereo;
            node->pending_frames_capacity_ = 3;
            node->imu_wait_timeout_sec_ = 30.0;
            node->imu_topic_ = "/finite_executor_imu";
            node->declare_parameter("imu.max_gap_sec", 1.1);
            node->node_logger_ = logging->GetLogger("executor_test");
            node->callback_guard_ = std::make_shared<CallbackGuard>(node->stop_control_,
                [&]() {
                    if (cancel_fails) throw std::runtime_error("injected cancel failure");
                    executor->cancel();
                },
                [&]() { ++fallback_calls; context->shutdown("cancel failure"); },
                [](std::exception_ptr) {});
            node->test_track_ = [&](const StereoFrame &, const std::vector<ImuMeasurement> &) {
                ++backend_calls;
            };
            node->initialize_frontends_and_timers("/finite_executor_left", "/finite_executor_right");
            registration = std::make_unique<ContextStopRegistration>(
                node->get_node_base_interface()->get_context(), node->stop_control_);
            require(executor->get_number_of_threads() == 2, "executor thread count mismatch");
            require(StereoFrontendTestAccess::in_group(*node->stereo_frontend_, node->reception_group_),
                    "image subscriptions are outside reception group");
            if (imu) require(ImuFrontendTestAccess::in_group(*node->imu_frontend_, node->reception_group_),
                             "IMU subscription is outside reception group");
            require(node->tracking_group_->find_timer_ptrs_if([&](const auto &timer) {
                return timer == node->tracking_timer_;
            }) != nullptr, "tracking timer is outside tracking group");
            require(node->tracking_group_->find_timer_ptrs_if([&](const auto &timer) {
                return timer == node->report_timer_;
            }) && node->tracking_group_->find_timer_ptrs_if([&](const auto &timer) {
                return timer == node->diagnostics_timer_;
            }) && node->reception_group_->find_timer_ptrs_if([&](const auto &timer) {
                return timer == node->input_timer_;
            }), "report/diagnostics/idle timer group mismatch");
            executor->add_node(node);
        }

        ~Fixture()
        {
            // Always release a blocked callback before joining, including assertion failure.
            gate.release();
            if (executor) executor->cancel();
            // Context shutdown also covers cancellation before spin has actually started.
            if (context->is_valid()) context->shutdown("fixture cleanup");
            if (runner.joinable()) runner.join();
            // Failure-path callbacks have now ended before any captured object is released.
            timers.clear();
            registration.reset();
            executor.reset();
            node.reset();
            logging.reset();
            std::error_code ignored;
            std::filesystem::remove_all(directory, ignored);
        }

        void timer(const rclcpp::CallbackGroup::SharedPtr &group, std::function<void()> action)
        {
            timers.push_back(node->create_wall_timer(std::chrono::milliseconds(1),
                [this, action]() { node->callback_guard_->run(action); }, group));
        }

        void start()
        {
            // A finite watchdog also handles test setup/handshake failure. The CTest
            // timeout remains the outer bound for an unexpected executor deadlock.
            const auto deadline = Clock::now() + std::chrono::seconds(5);
            timer(node->reception_group_, [this, deadline]() {
                if (Clock::now() >= deadline) throw std::runtime_error("executor test deadline exceeded");
            });
            runner = std::thread([&]() {
                // As in main, an escaping internal executor error cannot safely unwind
                // fixture ownership. Project callback failures must remain contained.
                try { executor->spin(); }
                catch (...) { std::terminate(); }
                const std::lock_guard<std::mutex> lock(mutex);
                spin_returned = true;
                cv.notify_all();
            });
        }

        void image(int seconds) { StereoFrontendTestAccess::receive(*node->stereo_frontend_, seconds); }
        void imu(int seconds) { ImuFrontendTestAccess::receive(*node->imu_frontend_, seconds); }

        void received()
        {
            const std::lock_guard<std::mutex> lock(mutex);
            reception_done = true;
            cv.notify_all();
        }

        void wait_reception()
        {
            std::unique_lock<std::mutex> lock(mutex);
            require(cv.wait_for(lock, std::chrono::seconds(3), [&]() { return reception_done || spin_returned; })
                    && reception_done, "reception did not progress during tracking");
            require(!spin_returned, "spin returned while a callback was still blocked");
        }

        void join()
        {
            std::unique_lock<std::mutex> lock(mutex);
            require(cv.wait_for(lock, std::chrono::seconds(3), [&]() { return spin_returned; }),
                    "executor did not join after callback release");
            lock.unlock();
            runner.join();
        }

        void finish(bool failed)
        {
            const auto trace_stats = node->trace_->stats();
            unsigned shutdown_calls = 0;
            std::vector<std::string> stages;
            node->test_finalize_step_ = [&](const char *stage) {
                require(spin_returned && !runner.joinable(), "finalization preceded worker join");
                stages.emplace_back(stage);
            };
            node->test_shutdown_ = [&]() {
                require(spin_returned && !runner.joinable(), "cleanup preceded worker join");
                ++shutdown_calls;
            };
            ProcessCleanup cleanup;
            cleanup.finalize_node = [&]() { node->shutdown(); };
            cleanup.detach_context = [&]() {
                require(registration->close(), "context callback detach failed");
                registration.reset();
            };
            cleanup.release_executor = [&]() { executor.reset(); };
            cleanup.release_node = [&]() { timers.clear(); node.reset(); };
            cleanup.shutdown_context = [&]() { if (context->is_valid()) context->shutdown("finalization"); };
            cleanup.finish_logging = [&]() {
                require(!node && !executor, "logging finished before callback owners were released");
                logging->finish();
                require(logging->dropped_messages() == 0, "finite test lost log records");
            };
            cleanup.release_logging = [&]() { logging.reset(); };
            const auto control = node->stop_control_;
            const int result = finalize_process(control, {}, cleanup, [](std::exception_ptr) {});
            require(shutdown_calls == 1 && result == (failed ? 1 : 0), "finalization count or exit code mismatch");
            require(std::find(stages.begin(), stages.end(), "trace_write") <
                    std::find(stages.begin(), stages.end(), "release_work"), "trace export followed work release");
            trace_csv = read_file(directory / "trace.csv");
            log_text = read_file(log_path);
            require(trace_csv.find("# dropped_events=" + std::to_string(trace_stats.dropped)) != std::string::npos,
                    "trace export lost dropped count");
            require(static_cast<std::size_t>(std::count(trace_csv.begin(), trace_csv.end(), '\n')) ==
                    trace_stats.recorded + 2, "trace export changed after producer join");
            require(log_text.find("STOP_ACCOUNTING") != std::string::npos &&
                    log_text.find("TRACK_TIMING final=true") != std::string::npos,
                    "final log records were not drained");
        }
    };

    class ThrowOnFirstFrame : public spdlog::sinks::base_sink<std::mutex>
    {
        void sink_it_(const spdlog::details::log_msg &message) override
        {
            const std::string text(message.payload.data(), message.payload.size());
            if (text.find("First stereo frame processed:") != std::string::npos)
                throw std::runtime_error("injected sink failure");
        }
        void flush_() override {}
    };

    enum class Scenario
    {
        StopInBackend, StopBeforePermit, StopAfterPermit, Overload,
        BackendError, StopThenBackendError, CancelFailure, ContextStop, LoggingFailure
    };

    static void coordination(bool imu_mode, Scenario scenario, std::size_t trace_capacity = 256)
    {
        const bool permit_gate = scenario == Scenario::StopBeforePermit || scenario == Scenario::StopAfterPermit;
        int phase = 0; // Reception-only state outlives the fixture failure-path join.
        Fixture f(imu_mode, trace_capacity);
        f.cancel_fails = scenario == Scenario::CancelFailure;
        if (scenario == Scenario::LoggingFailure)
        {
            // Throw synchronously after completion. The normal async logger reports
            // sink failures separately and would not throw at this callback boundary.
            auto sinks = f.node->node_logger_->sinks();
            sinks.push_back(std::make_shared<ThrowOnFirstFrame>());
            f.node->node_logger_ = std::make_shared<spdlog::logger>("throwing", sinks.begin(), sinks.end());
            f.node->node_logger_->set_error_handler([](const std::string &) {
                throw std::runtime_error("injected post-completion logging failure");
            });
        }
        if (permit_gate)
            f.node->test_work_point_ = [&](SlamNode::TestWorkPoint point) {
                if (point == (scenario == Scenario::StopBeforePermit ? SlamNode::TestWorkPoint::BeforeBackendPermit :
                                                                     SlamNode::TestWorkPoint::AfterBackendPermit))
                    f.gate.block();
            };
        f.node->test_track_ = [&](const StereoFrame &, const std::vector<ImuMeasurement> &) {
            ++f.backend_calls;
            if (!permit_gate) f.gate.block();
            if (scenario == Scenario::BackendError || scenario == Scenario::StopThenBackendError)
                throw std::runtime_error("injected backend error");
        };

        f.timer(f.node->reception_group_, [&]() {
            if (phase == 0)
            {
                if (imu_mode) { f.imu(1); f.imu(2); }
                f.image(1);
                if (imu_mode) f.image(2);
                phase = 1;
                return;
            }
            if (phase != 1 || !f.gate.entered()) return;
            phase = 2;
            // The backend/permit gate remains blocked until the test thread has
            // observed this new accepted input and verified spin has not returned.
            if (imu_mode) f.imu(3);
            f.image(imu_mode ? 3 : 2);
            if (scenario == Scenario::Overload)
            {
                if (!imu_mode) f.image(3);
                // Preserve the frontend exception boundary while checking rejection below.
                f.node->callback_guard_->run([&]() { f.image(4); });
            }
            const auto queue = f.node->queue_snapshot();
            require(queue.enqueued == queue.pending + queue.in_flight + queue.processed +
                    queue.startup_discarded + queue.overload_discarded && queue.outstanding <= 3,
                    "concurrent reception violated accounting/capacity");
            require(queue.reservation && queue.reservation->enqueue_sequence == 1,
                    "reception evicted the reserved frame");
            if (imu_mode) require(f.node->imu_frontend_->stats().accepted == 3,
                                  "IMU reception did not advance during tracking");
            if (scenario == Scenario::ContextStop)
                f.node->get_node_base_interface()->get_context()->shutdown("finite executor test");
            else if (scenario != Scenario::BackendError && scenario != Scenario::LoggingFailure &&
                     !f.node->stop_control_->stop_requested())
            {
                f.node->stop_control_->request_stop(StopReason::InputIdle);
                f.node->callback_guard_->cancel();
            }
            f.received();
        });
        f.start();
        f.wait_reception();
        f.gate.release();
        f.join();
        const auto queue = f.node->queue_snapshot();
        const auto control = f.node->stop_control_->snapshot();
        const bool rejected_permit = scenario == Scenario::StopBeforePermit;
        const bool backend_error = scenario == Scenario::BackendError || scenario == Scenario::StopThenBackendError;
        const bool capacity_error = imu_mode && scenario == Scenario::Overload;
        require(f.backend_calls == (rejected_permit ? 0U : 1U) &&
                control.backend_starts == (rejected_permit ? 0U : 1U), "stop/permit allowed another backend call");
        require(queue.processed == (rejected_permit || backend_error ? 0U : 1U),
                "completion was lost or committed after backend failure");
        require(queue.enqueued == queue.pending + queue.in_flight + queue.processed +
                queue.startup_discarded + queue.overload_discarded, "final accounting mismatch");
        require(control.first_stop && control.first_stop->reason ==
                (capacity_error ? StopReason::Capacity : scenario == Scenario::BackendError ? StopReason::BackendError :
                 scenario == Scenario::ContextStop ? StopReason::ContextShutdown :
                 scenario == Scenario::LoggingFailure ? StopReason::CallbackError : StopReason::InputIdle),
                "incorrect first stop reason");
        if (backend_error)
            require(control.first_failure && control.first_failure->reason == StopReason::BackendError &&
                    control.first_exception, "backend failure was lost after stop");
        if (scenario == Scenario::CancelFailure)
            require(f.fallback_calls == 1 && !f.context->is_valid() && control.first_failure &&
                    control.first_failure->reason == StopReason::CancelError, "cancel fallback did not retain failure");
        if (imu_mode)
        {
            require(ImuFrontendTestAccess::consumed_until(*f.node->imu_frontend_) == 2.0,
                    "stop/failed backend consumed another IMU batch");
            require(queue.startup_next_reservation &&
                    queue.startup_next_reservation->batch_use == SlamNode::ImuBatchUse::ConsumedUnused &&
                    queue.startup_next_reservation->enqueue_sequence == 2,
                    "F1 was executed or its consumed batch was lost");
        }
        if (scenario == Scenario::Overload)
        {
            require(queue.enqueued == (imu_mode ? 3U : 4U) &&
                    queue.overload_discarded == (imu_mode ? 0U : 1U) &&
                    queue.pending == (imu_mode ? 1U : 2U), "overload accounting mismatch");
            require(queue.first && queue.first->enqueue_sequence == 3 &&
                    (imu_mode || (queue.second && queue.second->enqueue_sequence == 4)),
                    "overload retained the wrong frame identities");
        }
        if (trace_capacity == 2)
        {
            const auto trace = f.node->trace_->stats();
            require(trace.recorded == 2 && trace.dropped == (imu_mode ? 9U : 2U),
                    "shared trace capacity/dropped accounting mismatch");
        }
        f.finish(backend_error || capacity_error || scenario == Scenario::CancelFailure ||
                 scenario == Scenario::LoggingFailure);
        if (scenario == Scenario::Overload && !imu_mode)
        {
            require(f.trace_csv.find(",0,2,4\n") != std::string::npos &&
                    f.trace_csv.find("frame_overload_discarded,") != std::string::npos &&
                    f.log_text.find("enqueue_sequence=2 timestamp_ns=2000000000") != std::string::npos,
                    "overload event identity was not preserved");
        }
    }

    // Reception and tracking alternate explicit stages; no wall-time assumptions.
    // The real retry entry is called by a tracking-group timer after each input stage.
    static void waiting_and_missing_history()
    {
        std::atomic<int> step{0};
        std::optional<Clock::time_point> deadline;
        std::vector<int> calls;
        Fixture f(true);
        f.node->pending_frames_capacity_ = 6;
        f.node->tracking_timer_->cancel();
        f.node->test_track_ = [&](const StereoFrame &frame, const std::vector<ImuMeasurement> &batch) {
            ++f.backend_calls;
            const int timestamp = static_cast<int>(frame.timestamp);
            calls.push_back(timestamp);
            require(timestamp == 3 ? batch.empty() :
                    batch.size() == 1 && batch.front().timestamp == frame.timestamp,
                    "startup/normal batch contents changed");
        };
        f.timer(f.node->reception_group_, [&]() {
            switch (step.load())
            {
            case 0:
                f.image(1); f.image(2); step = 1; break;
            case 2:
                f.imu(3); f.imu(4); f.image(3); f.image(4); step = 3; break;
            case 4:
                f.image(5); step = 5; break;
            case 8:
                f.imu(5); step = 9; break;
            default: break;
            }
        });
        f.timer(f.node->tracking_group_, [&]() {
            const int current = step.load();
            if (current != 1 && current != 3 && current != 5 && current != 6 && current != 7 && current != 9)
                return;
            f.node->tracking_callback();
            const auto queue = f.node->queue_snapshot();
            require(queue.enqueued == queue.pending + queue.in_flight + queue.processed +
                    queue.startup_discarded + queue.overload_discarded, "retry accounting mismatch");
            if (current == 1)
            {
                deadline = queue.startup_started;
                require(deadline && queue.in_flight == 2 && queue.processed == 0 &&
                        queue.reservation->enqueue_sequence == 1 && queue.startup_next_reservation->enqueue_sequence == 2 &&
                        !ImuFrontendTestAccess::consumed_until(*f.node->imu_frontend_), "Waiting changed ownership/consumption");
            }
            if (current == 3 || current == 5)
            {
                const uint64_t discarded = current == 3 ? 1 : 2;
                require(queue.startup_discarded == discarded && queue.processed == 0 && queue.in_flight == 1 &&
                        queue.reservation->enqueue_sequence == discarded + 1 && queue.startup_started == deadline &&
                        !ImuFrontendTestAccess::consumed_until(*f.node->imu_frontend_),
                        "MissingHistory discarded more than F0 or reset deadline");
            }
            if (current == 6)
                require(queue.processed == 2 && queue.startup_complete && !queue.startup_started &&
                        ImuFrontendTestAccess::consumed_until(*f.node->imu_frontend_) == 4.0,
                        "startup failed to complete after history promotion");
            if (current == 7)
                require(queue.processed == 2 && queue.in_flight == 1 && queue.reservation->enqueue_sequence == 5 &&
                        ImuFrontendTestAccess::consumed_until(*f.node->imu_frontend_) == 4.0,
                        "normal Waiting consumed a batch or overtook the reserved frame");
            // Exercise real report/reset paths under their tracking-group ownership.
            f.node->report(false);
            f.node->report_tracking_diagnostics(false);
            if (current == 9)
            {
                require(queue.processed == 3 && queue.startup_discarded == 2 && queue.outstanding == 0,
                        "final retry lost completion or discard accounting");
                f.node->stop_control_->request_stop(StopReason::InputIdle);
                f.node->callback_guard_->cancel();
            }
            step = current + 1;
        });
        f.start();
        f.join();
        require(step == 10 && calls == std::vector<int>({3, 4, 5}) &&
                !f.node->stop_control_->snapshot().first_failure &&
                ImuFrontendTestAccess::consumed_until(*f.node->imu_frontend_) == 5.0,
                "retry integration failed or repeated a batch");
        f.finish(false);
    }

    static void later_backend_failure(int failing_frame, bool stop_first)
    {
        int phase = 0;
        std::vector<int> calls;
        Fixture f(true);
        f.node->test_track_ = [&](const StereoFrame &frame, const std::vector<ImuMeasurement> &batch) {
            ++f.backend_calls;
            const int timestamp = static_cast<int>(frame.timestamp);
            calls.push_back(timestamp);
            require(timestamp == 1 ? batch.empty() : batch.size() == 1 && batch.front().timestamp == frame.timestamp,
                    "later failure batch contents mismatch");
            if (timestamp == failing_frame)
            {
                f.gate.block();
                throw std::runtime_error("injected later backend failure");
            }
        };
        f.timer(f.node->reception_group_, [&]() {
            if (phase == 0)
            {
                for (int i = 1; i <= 3; ++i) f.imu(i);
                for (int i = 1; i <= 3; ++i) f.image(i);
                phase = 1;
            }
            else if (phase == 1 && f.gate.entered())
            {
                phase = 2;
                f.imu(4); f.image(4);
                if (stop_first)
                {
                    f.node->stop_control_->request_stop(StopReason::InputIdle);
                    f.node->callback_guard_->cancel();
                }
                f.received();
            }
        });
        f.start(); f.wait_reception(); f.gate.release(); f.join();
        const auto queue = f.node->queue_snapshot();
        const auto control = f.node->stop_control_->snapshot();
        const auto &failed = failing_frame == 2 ? queue.startup_next_reservation : queue.reservation;
        require(failed && failed->enqueue_sequence == static_cast<uint64_t>(failing_frame) &&
                failed->batch_use == SlamNode::ImuBatchUse::DeliveredToBackend && failed->interruption &&
                failed->interruption->reason == SlamNode::WorkReason::BackendException,
                "F1/normal failure lost reservation or batch delivery");
        require(queue.enqueued == 4 && queue.processed == static_cast<uint64_t>(failing_frame - 1) &&
                queue.in_flight == 1 && queue.pending == static_cast<std::size_t>(4 - failing_frame) &&
                calls == (failing_frame == 2 ? std::vector<int>{1, 2} : std::vector<int>{1, 2, 3}) &&
                ImuFrontendTestAccess::consumed_until(*f.node->imu_frontend_) == static_cast<double>(failing_frame),
                "F1/normal failure retried or lost completed prefix");
        require(control.first_stop && control.first_stop->reason ==
                (stop_first ? StopReason::InputIdle : StopReason::BackendError) && control.first_failure &&
                control.first_failure->reason == StopReason::BackendError, "later failure replaced first stop cause");
        const auto final = f.node->final_snapshot();
        require(final.coverage.source == std::string("saved_batch") && final.coverage.batch_samples == 1 &&
                ImuFrontendTestAccess::consumed_until(*f.node->imu_frontend_) == static_cast<double>(failing_frame),
                "final coverage reconsumed saved batch");
        f.finish(true);
    }

    // Test each actual group with a held callback, a ready same-group competitor,
    // and demonstrable progress in the opposite group. The competitor must later run.
    static void group_serialization(bool hold_reception)
    {
        std::atomic<bool> active{false};
        std::atomic<bool> completed{false};
        std::atomic<unsigned> competitors{0};
        std::atomic<bool> first{true};
        Fixture f(false);
        f.node->tracking_timer_->cancel();
        const auto held = hold_reception ? f.node->reception_group_ : f.node->tracking_group_;
        const auto other = hold_reception ? f.node->tracking_group_ : f.node->reception_group_;
        f.timer(held, [&]() {
            if (!first.exchange(false)) return;
            active = true;
            f.gate.block();
            active = false;
            completed = true;
        });
        f.timer(held, [&]() {
            require(!active, "MutuallyExclusive group reentered");
            if (completed)
            {
                ++competitors;
                f.node->stop_control_->request_stop(StopReason::InputIdle);
                f.node->callback_guard_->cancel();
            }
        });
        f.timer(other, [&]() { if (active) f.received(); });
        f.start();
        f.wait_reception();
        f.gate.release();
        f.join();
        require(competitors > 0 && !f.node->stop_control_->snapshot().first_failure,
                "same-group competitor failed to run after release");
        f.finish(false);
    }

    static void run()
    {
        waiting_and_missing_history();
        for (int frame : {2, 3})
            for (bool stop_first : {false, true}) later_backend_failure(frame, stop_first);
        coordination(false, Scenario::StopInBackend, 2);
        coordination(true, Scenario::StopInBackend, 2);
        group_serialization(false);
        group_serialization(true);
        for (bool imu : {false, true})
            for (Scenario scenario : {Scenario::StopInBackend, Scenario::StopBeforePermit,
                                      Scenario::StopAfterPermit, Scenario::Overload, Scenario::BackendError,
                                      Scenario::StopThenBackendError, Scenario::CancelFailure, Scenario::ContextStop,
                                      Scenario::LoggingFailure})
                coordination(imu, scenario);

    }
};
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try
    {
        gemini336_orbslam3::SlamExecutorTestAccess::run();
        std::cout << "Finite two-thread executor integration tests passed\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
