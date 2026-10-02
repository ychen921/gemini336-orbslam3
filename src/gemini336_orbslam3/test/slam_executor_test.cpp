// Finite executor integration: real frontend/timer wiring, no DDS player or ORB-SLAM3.
#include "../src/slam_node.cpp"

#include <condition_variable>
#include <iostream>
#include <thread>
#include <spdlog/sinks/null_sink.h>

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
        std::shared_ptr<SlamNode> node{new SlamNode(SlamNode::QueueTestTag{})};
        std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor{
            std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 2)};
        std::unique_ptr<ContextStopRegistration> registration;
        Gate gate;
        std::mutex mutex;
        std::condition_variable cv;
        bool reception_done = false;
        bool spin_returned = false;
        std::atomic<unsigned> backend_calls{0};
        std::vector<rclcpp::TimerBase::SharedPtr> timers;
        std::thread runner;

        explicit Fixture(bool imu)
        {
            node->tracking_mode_ = imu ? TrackingMode::StereoImu : TrackingMode::Stereo;
            node->pending_frames_capacity_ = 3;
            node->imu_wait_timeout_sec_ = 30.0;
            node->imu_topic_ = "/finite_executor_imu";
            node->declare_parameter("imu.max_gap_sec", 1.1);
            node->node_logger_ = std::make_shared<spdlog::logger>(
                "executor_test", std::make_shared<spdlog::sinks::null_sink_mt>());
            node->request_stop_ = [&]() { executor->cancel(); };
            node->callback_guard_ = std::make_shared<CallbackGuard>(node->stop_control_,
                [&]() { executor->cancel(); },
                [&]() { node->get_node_base_interface()->get_context()->shutdown("cancel failure"); },
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
            executor->add_node(node);
        }

        ~Fixture()
        {
            // Always release a blocked callback before joining, including assertion failure.
            gate.release();
            if (executor) executor->cancel();
            if (runner.joinable()) runner.join();
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
            unsigned shutdown_calls = 0;
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
            // Keep the shared test context for subsequent cases; one case shuts it down explicitly.
            cleanup.shutdown_context = []() {};
            cleanup.finish_logging = []() {};
            cleanup.release_logging = []() {};
            const auto control = node->stop_control_;
            const int result = finalize_process(control, {}, cleanup, [](std::exception_ptr) {});
            require(shutdown_calls == 1 && result == (failed ? 1 : 0), "finalization count or exit code mismatch");
        }
    };

    enum class Scenario { StopInBackend, StopBeforePermit, StopAfterPermit, Overload, BackendError, ContextStop };

    static void coordination(bool imu_mode, Scenario scenario)
    {
        const bool permit_gate = scenario == Scenario::StopBeforePermit || scenario == Scenario::StopAfterPermit;
        int phase = 0; // Reception-only state outlives the fixture failure-path join.
        Fixture f(imu_mode);
        if (permit_gate)
            f.node->test_work_point_ = [&](SlamNode::TestWorkPoint point) {
                if (point == (scenario == Scenario::StopBeforePermit ? SlamNode::TestWorkPoint::BeforeBackendPermit :
                                                                     SlamNode::TestWorkPoint::AfterBackendPermit))
                    f.gate.block();
            };
        f.node->test_track_ = [&](const StereoFrame &, const std::vector<ImuMeasurement> &) {
            ++f.backend_calls;
            if (!permit_gate) f.gate.block();
            if (scenario == Scenario::BackendError) throw std::runtime_error("injected backend error");
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
            if (imu_mode) require(f.node->imu_frontend_->stats().accepted == 3,
                                  "IMU reception did not advance during tracking");
            if (scenario == Scenario::ContextStop)
                f.node->get_node_base_interface()->get_context()->shutdown("finite executor test");
            else if (scenario != Scenario::BackendError && !f.node->stop_control_->stop_requested())
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
        const bool backend_error = scenario == Scenario::BackendError;
        const bool capacity_error = imu_mode && scenario == Scenario::Overload;
        require(f.backend_calls == (rejected_permit ? 0U : 1U) &&
                control.backend_starts == (rejected_permit ? 0U : 1U), "stop/permit allowed another backend call");
        require(queue.processed == (rejected_permit || backend_error ? 0U : 1U),
                "completion was lost or committed after backend failure");
        require(queue.enqueued == queue.pending + queue.in_flight + queue.processed +
                queue.startup_discarded + queue.overload_discarded, "final accounting mismatch");
        require(control.first_stop && control.first_stop->reason ==
                (capacity_error ? StopReason::Capacity : backend_error ? StopReason::BackendError :
                 scenario == Scenario::ContextStop ? StopReason::ContextShutdown : StopReason::InputIdle),
                "incorrect first stop reason");
        if (imu_mode)
            require(queue.startup_next_reservation &&
                    queue.startup_next_reservation->batch_use == SlamNode::ImuBatchUse::ConsumedUnused &&
                    queue.startup_next_reservation->enqueue_sequence == 2,
                    "F1 was executed or its consumed batch was lost");
        if (scenario == Scenario::Overload)
            require(queue.enqueued == (imu_mode ? 3U : 4U) &&
                    queue.overload_discarded == (imu_mode ? 0U : 1U) &&
                    queue.pending == (imu_mode ? 1U : 2U), "overload accounting mismatch");
        f.finish(backend_error || capacity_error);
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
        group_serialization(false);
        group_serialization(true);
        for (bool imu : {false, true})
            for (Scenario scenario : {Scenario::StopInBackend, Scenario::StopBeforePermit,
                                      Scenario::StopAfterPermit, Scenario::Overload, Scenario::BackendError})
                coordination(imu, scenario);
        // Last case shuts down the process-wide test context while a callback is blocked.
        coordination(true, Scenario::ContextStop);
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
