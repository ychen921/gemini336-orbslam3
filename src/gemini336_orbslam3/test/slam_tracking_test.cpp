// Exercise real scheduling, IMU queries and completion; never construct ORB-SLAM3.
#include "../src/slam_node.cpp"

#include <iostream>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/null_sink.h>

namespace gemini336_orbslam3
{
struct ImuFrontendTestAccess
{
    static void receive(ImuFrontend &frontend, int seconds)
    {
        auto msg = std::make_shared<sensor_msgs::msg::Imu>();
        msg->header.stamp.sec = seconds;
        msg->header.frame_id = "imu";
        msg->linear_acceleration.z = 9.8;
        frontend.imu_callback(msg);
    }

    static std::optional<double> consumed_until(ImuFrontend &frontend)
    {
        const std::lock_guard<std::mutex> lock(frontend.imu_mutex_);
        return frontend.last_taken_timestamp_;
    }
};

struct SlamTrackingTestAccess
{
    struct BackendFailure : std::runtime_error
    {
        BackendFailure() : std::runtime_error("injected backend failure") {}
    };

    struct LogFailure : std::runtime_error
    {
        LogFailure() : std::runtime_error("injected logging failure") {}
    };

    class ThrowingSink : public spdlog::sinks::base_sink<std::mutex>
    {
        void sink_it_(const spdlog::details::log_msg &) override { throw LogFailure(); }
        void flush_() override {}
    };

    static void require(bool condition, const char *message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    static double stamp(int seconds)
    {
        // Match the real frontend's nanoseconds-to-double conversion exactly.
        return static_cast<double>(int64_t(seconds) * 1000000000LL) * 1e-9;
    }

    struct Call
    {
        double timestamp;
        std::vector<double> imu;
    };

    struct Fixture
    {
        SlamNode node{SlamNode::QueueTestTag{}};
        std::vector<Call> calls;
        std::function<void()> on_track;

        Fixture()
        {
            node.tracking_mode_ = TrackingMode::StereoImu;
            // Tests advance sensor time, never wait for wall-clock timeouts.
            node.imu_wait_timeout_sec_ = 3600.0;
            node.declare_parameter("imu.max_gap_sec", 1.1);
            node.imu_frontend_ = std::make_unique<ImuFrontend>(&node, "/unused_tracking_test");
            node.node_logger_ = std::make_shared<spdlog::logger>(
                "tracking_test", std::make_shared<spdlog::sinks::null_sink_mt>());
            node.test_track_ = [this](const StereoFrame &frame,
                                     const std::vector<ImuMeasurement> &imu) {
                Call call{frame.timestamp, {}};
                for (const auto &measurement : imu)
                    call.imu.push_back(measurement.timestamp);
                calls.push_back(std::move(call));
                // Taking both snapshots would deadlock if backend held either data lock.
                const auto queue = node.queue_snapshot();
                node.imu_frontend_->stats();
                require(queue.enqueued == queue.pending + queue.in_flight +
                        queue.processed + queue.startup_discarded + queue.overload_discarded,
                        "backend entry observed inconsistent accounting");
                require(!frame.left.empty() && !frame.right.empty(),
                        "backend received an invalid frame payload");
                if (on_track) on_track();
            };
        }

        void enqueue(int seconds)
        {
            StereoFrame frame;
            frame.timestamp = stamp(seconds);
            frame.left = cv::Mat(2, 2, CV_8UC1, cv::Scalar(seconds));
            frame.right = frame.left.clone();
            node.enqueue_frame(frame);
        }

        void receive(int seconds)
        {
            ImuFrontendTestAccess::receive(*node.imu_frontend_, seconds);
        }
    };

    static void test_waiting_and_completion()
    {
        Fixture f;
        f.enqueue(1);
        f.enqueue(2);
        f.receive(1);
        const auto initial = f.node.queue_snapshot();
        for (int retry = 0; retry < 3; ++retry)
        {
            f.node.process_pending_frames();
            const auto waiting = f.node.queue_snapshot();
            require(f.calls.empty() && waiting.pending == 0 && waiting.in_flight == 2 &&
                    waiting.processed == 0 && waiting.reservation &&
                    waiting.startup_next_reservation &&
                    !waiting.startup_next_reservation->imu_batch_consumed &&
                    waiting.startup_started == initial.startup_started &&
                    !ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_),
                    "startup Waiting consumed data, lost candidates or reset deadline");
        }

        // Reception during the finite backend call uses the real admission path.
        f.on_track = [&]() {
            if (f.calls.size() == 1) f.enqueue(3);
        };
        f.receive(2);
        f.node.process_pending_frames();
        auto snapshot = f.node.queue_snapshot();
        require(f.calls.size() == 2 && f.calls[0].timestamp == stamp(1) &&
                f.calls[0].imu.empty() && f.calls[1].timestamp == stamp(2) &&
                f.calls[1].imu == std::vector<double>{stamp(2)} &&
                snapshot.processed == 2 && snapshot.pending == 1 && snapshot.peak == 3 &&
                snapshot.overload_discarded == 0 &&
                snapshot.in_flight == 0 && snapshot.startup_complete &&
                !snapshot.startup_started && !f.node.tracking_work_ &&
                !f.node.startup_next_work_,
                "startup completion lost ordering, IMU interval or completion state");

        // Normal processing must retain a single reservation even with an empty queue.
        for (int retry = 0; retry < 3; ++retry)
        {
            f.node.process_pending_frames();
            snapshot = f.node.queue_snapshot();
            require(f.calls.size() == 2 && snapshot.pending == 0 &&
                    snapshot.in_flight == 1 && snapshot.reservation &&
                    snapshot.reservation->timestamp == stamp(3) &&
                    ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == stamp(2),
                    "normal Waiting consumed IMU or lost the reserved frame");
        }
        f.receive(3);
        f.node.process_pending_frames();
        snapshot = f.node.queue_snapshot();
        require(f.calls.size() == 3 && f.calls[2].timestamp == stamp(3) &&
                f.calls[2].imu == std::vector<double>{stamp(3)} &&
                snapshot.processed == 3 && snapshot.outstanding == 0 &&
                snapshot.last_tracked == stamp(3) && !f.node.tracking_work_ &&
                ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == stamp(3),
                "normal completion did not consume and commit exactly once");
        f.node.process_pending_frames();
        require(f.calls.size() == 3, "empty retry called backend again");
    }

    static void test_missing_history()
    {
        Fixture f;
        for (int i = 1; i <= 4; ++i) f.enqueue(i);
        f.receive(3);
        f.receive(4);
        const auto initial = f.node.queue_snapshot();
        for (uint64_t discarded = 1; discarded <= 2; ++discarded)
        {
            f.node.process_pending_frames();
            const auto snapshot = f.node.queue_snapshot();
            require(f.calls.empty() && snapshot.startup_discarded == discarded &&
                    snapshot.pending == 3 - discarded && snapshot.in_flight == 1 &&
                    snapshot.processed == 0 && snapshot.reservation &&
                    snapshot.reservation->timestamp == stamp(discarded + 1) &&
                    !snapshot.startup_next_reservation &&
                    snapshot.enqueued == snapshot.outstanding + snapshot.startup_discarded + snapshot.overload_discarded &&
                    snapshot.startup_started == initial.startup_started &&
                    !ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_),
                    "actual MissingHistory did not discard exactly one F0 per retry");
        }
        f.node.process_pending_frames();
        const auto done = f.node.queue_snapshot();
        require(f.calls.size() == 2 && f.calls[0].timestamp == stamp(3) &&
                f.calls[0].imu.empty() && f.calls[1].timestamp == stamp(4) &&
                f.calls[1].imu == std::vector<double>{stamp(4)} &&
                done.enqueued == 4 && done.processed == 2 &&
                done.startup_discarded == 2 && done.outstanding == 0 &&
                done.startup_complete,
                "startup failed to recover from consecutive MissingHistory");
    }

    static void test_stop_after_f0()
    {
        Fixture f;
        f.enqueue(1);
        f.enqueue(2);
        f.receive(1);
        f.receive(2);
        f.on_track = [&]() { f.node.stop_requested_ = true; };
        f.node.process_pending_frames();
        const auto snapshot = f.node.queue_snapshot();
        require(f.calls.size() == 1 && snapshot.processed == 1 &&
                snapshot.in_flight == 1 && !snapshot.reservation &&
                snapshot.startup_next_reservation &&
                snapshot.startup_next_reservation->stage == SlamNode::TrackingWorkStage::Ready &&
                snapshot.startup_next_reservation->imu_batch_consumed &&
                !snapshot.startup_complete && snapshot.startup_started &&
                !f.node.tracking_work_ && f.node.startup_next_work_ &&
                f.node.startup_next_work_->imu_batch &&
                f.node.startup_next_work_->imu_batch->measurements.size() == 1 &&
                ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == stamp(2),
                "stop after F0 lost the unexecuted F1 or consumed batch");
        f.node.process_pending_frames();
        require(f.calls.size() == 1 && f.node.queue_snapshot().processed == 1,
                "stopped startup retried backend or completion");
    }

    static void test_backend_failure(int failing_frame)
    {
        Fixture f;
        for (int i = 1; i <= failing_frame + (failing_frame == 1); ++i)
        {
            f.enqueue(i);
            f.receive(i);
        }
        f.on_track = [&]() {
            if (f.calls.back().timestamp == stamp(failing_frame)) throw BackendFailure();
        };
        bool failed = false;
        try
        {
            f.node.process_pending_frames();
            if (failing_frame == 3) f.node.process_pending_frames();
        }
        catch (const BackendFailure &) { failed = true; }
        const auto snapshot = f.node.queue_snapshot();
        require(failed && f.calls.size() == static_cast<std::size_t>(failing_frame) &&
                snapshot.processed == static_cast<uint64_t>(failing_frame - 1) &&
                snapshot.in_flight == (failing_frame == 1 ? 2U : 1U) &&
                snapshot.enqueued == snapshot.outstanding + snapshot.processed +
                    snapshot.startup_discarded + snapshot.overload_discarded &&
                snapshot.startup_complete == (failing_frame == 3),
                "backend failure committed failed work or lost accounting");
        const auto &failed_work = failing_frame == 2 ?
            f.node.startup_next_work_ : f.node.tracking_work_;
        require(failed_work && failed_work->stage == SlamNode::TrackingWorkStage::Executing,
                "backend failure lost its execution stage");
        const auto &batch_work = failing_frame <= 2 ?
            f.node.startup_next_work_ : f.node.tracking_work_;
        require(batch_work && batch_work->imu_batch &&
                batch_work->imu_batch->measurements.size() == 1,
                "backend failure lost consumed IMU data");

        // A defensive retry must fail before taking the interval or calling backend.
        bool retry_rejected = false;
        try { f.node.process_pending_frames(); }
        catch (const std::logic_error &) { retry_rejected = true; }
        require(retry_rejected && f.calls.size() == static_cast<std::size_t>(failing_frame) &&
                f.node.queue_snapshot().processed == snapshot.processed &&
                ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) ==
                    stamp(failing_frame == 1 ? 2 : failing_frame),
                "failed backend work was retried or its batch consumed again");
    }

    static void test_logging_failure()
    {
        Fixture f;
        f.enqueue(1);
        f.enqueue(2);
        f.receive(1);
        f.receive(2);
        f.node.node_logger_ = std::make_shared<spdlog::logger>(
            "throwing_tracking_test", std::make_shared<ThrowingSink>());
        // spdlog normally handles sink exceptions; deliberately propagate one here.
        f.node.node_logger_->set_error_handler(
            [](const std::string &) { throw LogFailure(); });
        bool failed = false;
        try { f.node.process_pending_frames(); }
        catch (const LogFailure &) { failed = true; }
        const auto snapshot = f.node.queue_snapshot();
        require(failed && f.calls.size() == 1 && snapshot.processed == 1 &&
                snapshot.in_flight == 1 && !snapshot.reservation &&
                !f.node.tracking_work_ && snapshot.startup_next_reservation &&
                snapshot.startup_next_reservation->stage == SlamNode::TrackingWorkStage::Ready &&
                snapshot.startup_next_reservation->imu_batch_consumed &&
                f.node.startup_next_work_ && f.node.startup_next_work_->imu_batch,
                "logging failure prevented F0 completion or lost F1");
        bool retry_rejected = false;
        try { f.node.process_pending_frames(); }
        catch (const std::logic_error &) { retry_rejected = true; }
        require(retry_rejected && f.calls.size() == 1 &&
                f.node.queue_snapshot().processed == 1,
                "logging failure caused completed F0 to be retried");
    }

    static void run()
    {
        test_waiting_and_completion();
        test_missing_history();
        test_stop_after_f0();
        for (int frame = 1; frame <= 3; ++frame) test_backend_failure(frame);
        test_logging_failure();
    }
};
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try
    {
        gemini336_orbslam3::SlamTrackingTestAccess::run();
        std::cout << "Finite IMU coordination, completion and failure tests passed\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
