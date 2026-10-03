// Exercise real scheduling, IMU queries and completion; never construct ORB-SLAM3.
#include "slam_snapshot_test_access.hpp"

#include <iostream>
#include <cstdlib>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/null_sink.h>
#include <spdlog/sinks/ostream_sink.h>

namespace gemini336_orbslam3
{
struct StereoFrontendTestAccess
{
    static void dispatch(StereoFrontend &frontend, bool left, int seconds)
    {
        auto image = std::make_shared<StereoFrontend::Image>();
        image->header.stamp.sec = seconds;
        image->height = image->width = image->step = 1;
        image->encoding = "mono8";
        image->data = {42};
        std::shared_ptr<void> message = image;
        const auto subscription = left ? frontend.left_sub_.getSubscriber() : frontend.right_sub_.getSubscriber();
        subscription->handle_message(message, rclcpp::MessageInfo{});
    }

    static bool in_group(StereoFrontend &frontend,
                         const rclcpp::CallbackGroup::SharedPtr &group)
    {
        const auto left = group->find_subscription_ptrs_if(
            [&](const rclcpp::SubscriptionBase::SharedPtr &subscription) {
                return subscription == frontend.left_sub_.getSubscriber();
            });
        const auto right = group->find_subscription_ptrs_if(
            [&](const rclcpp::SubscriptionBase::SharedPtr &subscription) {
                return subscription == frontend.right_sub_.getSubscriber();
            });
        return left != nullptr && right != nullptr;
    }

    static void receive(StereoFrontend &frontend,
                        const StereoFrontend::Image::ConstSharedPtr &left,
                        const StereoFrontend::Image::ConstSharedPtr &right)
    {
        frontend.stereo_callback(left, right);
    }
};

struct ImuFrontendTestAccess
{
    static bool subscription_present(const ImuFrontend &frontend)
    {
        return bool(frontend.imu_sub_);
    }

    static void receive(ImuFrontend &frontend, int seconds)
    {
        auto msg = std::make_shared<sensor_msgs::msg::Imu>();
        msg->header.stamp.sec = seconds;
        msg->header.frame_id = "imu";
        msg->linear_acceleration.z = 9.8;
        frontend.imu_callback(msg);
    }

    static void attach_trace(ImuFrontend &frontend, DiagnosticTrace *trace)
    {
        // Test setup only, before any producer or consumer starts.
        frontend.trace_ = trace;
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
        // Keep mutable cancellation injection in the fixture, outside production state.
        std::function<void()> request_stop = []() {};

        explicit Fixture(int64_t capacity = 2000)
        {
            node.callback_guard_ = std::make_shared<CallbackGuard>(node.stop_control_,
                [this]() { request_stop(); }, []() {}, [](std::exception_ptr) {});
            node.tracking_mode_ = TrackingMode::StereoImu;
            // Tests advance sensor time, never wait for wall-clock timeouts.
            node.imu_wait_timeout_sec_ = 3600.0;
            node.declare_parameter("imu.max_gap_sec", 1.1);
            node.declare_parameter("imu.buffer_capacity", capacity);
            node.reception_group_ = node.create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
            node.imu_frontend_ = std::make_unique<ImuFrontend>(
                &node, node.reception_group_, "/unused_tracking_test", nullptr, node.stop_control_, node.callback_guard_);
            node.node_logger_ = std::make_shared<spdlog::logger>(
                "tracking_test", std::make_shared<spdlog::sinks::null_sink_mt>());
            node.test_track_ = [this](const StereoFrame &frame,
                                     const std::vector<ImuMeasurement> &imu) {
                Call call{frame.timestamp, {}};
                for (const auto &measurement : imu)
                    call.imu.push_back(measurement.timestamp);
                calls.push_back(std::move(call));
                // Taking both snapshots would deadlock if backend held either data lock.
                const auto queue = SlamSnapshotTestAccess::snapshot(node);
                node.imu_frontend_->stats();
                require(queue.enqueued == queue.pending + queue.in_flight +
                        queue.processed + queue.startup_discarded + queue.overload_discarded,
                        "backend entry observed inconsistent accounting");
                require(!frame.left.empty() && !frame.right.empty(),
                        "backend received an invalid frame payload");
                const auto &active = queue.reservation ? queue.reservation : queue.startup_next_reservation;
                require(active && active->stage == SlamNode::TrackingWorkStage::Executing,
                        "backend entry has incorrect execution stage");
                if (on_track) on_track();
            };
        }

        void enqueue(int seconds)
        {
            StereoFrame frame;
            frame.timestamp = stamp(seconds);
            frame.timestamp_ns = int64_t(seconds) * 1000000000LL;
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
        const auto initial = SlamSnapshotTestAccess::snapshot(f.node);
        for (int retry = 0; retry < 3; ++retry)
        {
            f.node.process_pending_frames();
            const auto waiting = SlamSnapshotTestAccess::snapshot(f.node);
            require(f.calls.empty() && waiting.pending == 0 && waiting.in_flight == 2 &&
                    waiting.processed == 0 && waiting.reservation &&
                    waiting.startup_next_reservation &&
                    waiting.reservation->enqueue_sequence == 1 &&
                    waiting.reservation->timestamp_ns == 1000000000LL &&
                    waiting.startup_next_reservation->enqueue_sequence == 2 &&
                    waiting.startup_next_reservation->timestamp_ns == 2000000000LL &&
                    !f.node.startup_next_work_->imu_batch &&
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
        auto snapshot = SlamSnapshotTestAccess::snapshot(f.node);
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
            snapshot = SlamSnapshotTestAccess::snapshot(f.node);
            require(f.calls.size() == 2 && snapshot.pending == 0 &&
                    snapshot.in_flight == 1 && snapshot.reservation &&
                    snapshot.reservation->timestamp == stamp(3) &&
                    snapshot.reservation->enqueue_sequence == 3 &&
                    snapshot.reservation->timestamp_ns == 3000000000LL &&
                    ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == stamp(2),
                    "normal Waiting consumed IMU or lost the reserved frame");
        }
        f.receive(3);
        f.node.process_pending_frames();
        snapshot = SlamSnapshotTestAccess::snapshot(f.node);
        require(f.calls.size() == 3 && f.calls[2].timestamp == stamp(3) &&
                f.calls[2].imu == std::vector<double>{stamp(3)} &&
                snapshot.processed == 3 && snapshot.outstanding == 0 &&
                f.node.last_tracked_frame_timestamp_ == stamp(3) && !f.node.tracking_work_ &&
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
        const auto initial = SlamSnapshotTestAccess::snapshot(f.node);
        for (uint64_t discarded = 1; discarded <= 2; ++discarded)
        {
            f.node.process_pending_frames();
            const auto snapshot = SlamSnapshotTestAccess::snapshot(f.node);
            require(f.calls.empty() && snapshot.startup_discarded == discarded &&
                    snapshot.pending == 3 - discarded && snapshot.in_flight == 1 &&
                    snapshot.processed == 0 && snapshot.reservation &&
                    snapshot.reservation->timestamp == stamp(discarded + 1) &&
                    snapshot.reservation->enqueue_sequence == discarded + 1 &&
                    snapshot.reservation->timestamp_ns == int64_t(discarded + 1) * 1000000000LL &&
                    !snapshot.startup_next_reservation &&
                    snapshot.enqueued == snapshot.outstanding + snapshot.startup_discarded + snapshot.overload_discarded &&
                    snapshot.startup_started == initial.startup_started &&
                    !ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_),
                    "actual MissingHistory did not discard exactly one F0 per retry");
        }
        f.node.process_pending_frames();
        const auto done = SlamSnapshotTestAccess::snapshot(f.node);
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
        f.on_track = [&]() { f.node.stop_control_->request_stop(StopReason::InputIdle); };
        f.node.process_pending_frames();
        const auto snapshot = SlamSnapshotTestAccess::snapshot(f.node);
        require(f.calls.size() == 1 && snapshot.processed == 1 &&
                f.node.last_tracked_frame_timestamp_ == stamp(1) &&
                snapshot.in_flight == 1 && !snapshot.reservation &&
                snapshot.startup_next_reservation &&
                snapshot.startup_next_reservation->stage == SlamNode::TrackingWorkStage::Ready &&
                !snapshot.startup_complete && snapshot.startup_started &&
                !f.node.tracking_work_ && f.node.startup_next_work_ &&
                f.node.startup_next_work_->imu_batch &&
                f.node.startup_next_work_->imu_batch->measurements.size() == 1 &&
                ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == stamp(2),
                "stop after F0 lost the unexecuted F1 or consumed batch");
        require(f.node.stop_control_->stop_requested() && !f.node.tracking_failed_,
                "normal stop was classified as a tracking failure");
        f.node.process_pending_frames();
        require(f.calls.size() == 1 && SlamSnapshotTestAccess::snapshot(f.node).processed == 1,
                "stopped startup retried backend or completion");
    }

    static void test_backend_failure(int failing_frame, bool stop_in_backend = false)
    {
        Fixture f;
        for (int i = 1; i <= failing_frame + (failing_frame == 1); ++i)
        {
            f.enqueue(i);
            f.receive(i);
        }
        f.on_track = [&]() {
            if (f.calls.back().timestamp == stamp(failing_frame))
            {
                if (stop_in_backend) f.node.stop_control_->request_stop(StopReason::InputIdle);
                throw BackendFailure();
            }
        };
        bool failed = false;
        try
        {
            f.node.process_pending_frames();
            if (failing_frame == 3) f.node.process_pending_frames();
        }
        catch (const BackendFailure &) { failed = true; }
        const auto snapshot = SlamSnapshotTestAccess::snapshot(f.node);
        require(failed && f.calls.size() == static_cast<std::size_t>(failing_frame) &&
                snapshot.processed == static_cast<uint64_t>(failing_frame - 1) &&
                snapshot.in_flight == (failing_frame == 1 ? 2U : 1U) &&
                snapshot.enqueued == snapshot.outstanding + snapshot.processed +
                    snapshot.startup_discarded + snapshot.overload_discarded &&
                snapshot.startup_complete == (failing_frame == 3),
                "backend failure committed failed work or lost accounting");
        require(f.node.last_tracked_frame_timestamp_ ==
                    (failing_frame == 1 ? std::optional<double>{} :
                     std::optional<double>{stamp(failing_frame - 1)}),
                "backend failure advanced the completed timestamp");
        const auto &failed_summary = failing_frame == 2 ?
            snapshot.startup_next_reservation : snapshot.reservation;
        require(f.node.tracking_failed_ && failed_summary &&
                failed_summary->stage == SlamNode::TrackingWorkStage::Executing,
                "backend failure lost its reservation or execution stage");
        const auto &failed_work = failing_frame == 2 ?
            f.node.startup_next_work_ : f.node.tracking_work_;
        require(failed_work && failed_work->stage == SlamNode::TrackingWorkStage::Executing,
                "backend failure lost its execution stage");
        const auto &batch_work = failing_frame <= 2 ?
            f.node.startup_next_work_ : f.node.tracking_work_;
        require(batch_work && batch_work->imu_batch &&
                batch_work->imu_batch->measurements.size() == 1,
                "backend failure lost consumed IMU data");

        if (failing_frame == 1)
            require(snapshot.startup_next_reservation &&
                    snapshot.startup_next_reservation->stage == SlamNode::TrackingWorkStage::Ready,
                    "F0 failure did not explain the unused F1 batch");
        // Stopped scheduling returns; otherwise the failed-work guard rejects a retry.
        bool retry_rejected = false;
        try { f.node.process_pending_frames(); }
        catch (const std::logic_error &) { retry_rejected = true; }
        require(retry_rejected == !stop_in_backend &&
                f.calls.size() == static_cast<std::size_t>(failing_frame) &&
                SlamSnapshotTestAccess::snapshot(f.node).processed == snapshot.processed &&
                ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) ==
                    stamp(failing_frame == 1 ? 2 : failing_frame),
                "failed backend work was retried or its batch consumed again");
        f.node.stop_control_->request_stop(StopReason::InputIdle);
        require(f.node.tracking_failed_, "stop cleared the failed scheduling state");
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
        const auto snapshot = SlamSnapshotTestAccess::snapshot(f.node);
        require(failed && f.calls.size() == 1 && snapshot.processed == 1 &&
                f.node.last_tracked_frame_timestamp_ == stamp(1) &&
                snapshot.in_flight == 1 && !snapshot.reservation &&
                !f.node.tracking_work_ && snapshot.startup_next_reservation &&
                snapshot.startup_next_reservation->stage == SlamNode::TrackingWorkStage::Ready &&
                f.node.startup_next_work_ && f.node.startup_next_work_->imu_batch,
                "logging failure prevented F0 completion or lost F1");
        require(f.node.tracking_failed_ &&
                f.node.tracking_callback_reason_ == StopReason::CallbackError,
                "post-completion logging failure was mistaken for backend failure");
        bool retry_rejected = false;
        try { f.node.process_pending_frames(); }
        catch (const std::logic_error &) { retry_rejected = true; }
        require(retry_rejected && f.calls.size() == 1 &&
                SlamSnapshotTestAccess::snapshot(f.node).processed == 1,
                "logging failure caused completed F0 to be retried");
    }

    static void test_source_identity()
    {
        Fixture f;
        std::ostringstream logs;
        f.node.node_logger_ = std::make_shared<spdlog::logger>(
            "identity_test", std::make_shared<spdlog::sinks::ostream_sink_mt>(logs));
        StereoFrontend frontend(&f.node, f.node.reception_group_, "/unused_left", "/unused_right",
            [&](const StereoFrame &frame) { f.node.enqueue_frame(frame); });
        require(StereoFrontendTestAccess::in_group(frontend, f.node.reception_group_),
                "image subscriptions are not in reception group");
        bool null_rejected = false;
        try
        {
            StereoFrontend invalid(&f.node, nullptr, "/unused_left", "/unused_right",
                [](const StereoFrame &) {});
        }
        catch (const std::invalid_argument &) { null_rejected = true; }
        require(null_rejected, "Stereo accepted a null reception group");
        const int64_t source_ns = 1700000000123456789LL;
        auto left = std::make_shared<StereoFrontend::Image>();
        left->header.stamp.sec = 1700000000;
        left->header.stamp.nanosec = 123456789;
        left->width = left->height = left->step = 1;
        left->encoding = "mono8";
        left->data = {42};
        auto right = std::make_shared<StereoFrontend::Image>(*left);
        right->header.stamp.nanosec += 100;
        StereoFrontendTestAccess::receive(frontend, left, right);
        const auto first = SlamSnapshotTestAccess::snapshot(f.node);
        require(first.first && first.first->enqueue_sequence == 1 &&
                first.first->frame.timestamp_ns == source_ns &&
                first.first->frame.timestamp == rclcpp::Time(left->header.stamp).seconds() &&
                static_cast<int64_t>(first.first->frame.timestamp * 1e9) != source_ns,
                "frontend failed to preserve exact left source time");
        bool rejected = false;
        try { StereoFrontendTestAccess::receive(frontend, left, right); }
        catch (const std::invalid_argument &) { rejected = true; }
        require(rejected, "duplicate source was accepted");
        left->header.stamp.sec += 1;
        right->header.stamp.sec += 1;
        StereoFrontendTestAccess::receive(frontend, left, right);
        const auto queued = SlamSnapshotTestAccess::snapshot(f.node);
        require(queued.second && queued.second->enqueue_sequence == 2,
                "rejected source consumed an enqueue sequence");
        require(f.node.reserve_startup_pair(), "source pair was not reserved");
        for (int retry = 0; retry < 3; ++retry)
        {
            require(f.node.reserve_startup_pair(), "retry lost source pair");
            const auto held = SlamSnapshotTestAccess::snapshot(f.node);
            require(held.reservation->timestamp_ns == source_ns &&
                    held.reservation->enqueue_sequence == 1 &&
                    held.reservation->received_at == first.first->received_at &&
                    held.startup_next_reservation->timestamp_ns == source_ns + 1000000000LL &&
                    f.node.tracking_work_->pending.frame.timestamp_ns == source_ns,
                    "reservation retry changed source identity");
        }
        f.node.discard_startup_first();
        const auto promoted = SlamSnapshotTestAccess::snapshot(f.node);
        require(promoted.reservation->enqueue_sequence == 2 &&
                promoted.reservation->timestamp_ns == source_ns + 1000000000LL &&
                promoted.reservation->received_at == queued.second->received_at &&
                f.node.tracking_work_->pending.enqueue_sequence == 2 &&
                logs.str().find("enqueue_sequence=1 timestamp_ns=1700000000123456789") != std::string::npos,
                "startup discard lost identity or logged the wrong work");
        left->header.stamp.sec += 1;
        right->header.stamp.sec += 1;
        StereoFrontendTestAccess::receive(frontend, left, right);
        require(f.node.reserve_startup_pair(), "source pair failed to refill");
        const auto refilled = SlamSnapshotTestAccess::snapshot(f.node);
        require(refilled.reservation->enqueue_sequence == 2 &&
                refilled.reservation->received_at == queued.second->received_at &&
                refilled.startup_next_reservation->enqueue_sequence == 3 &&
                refilled.startup_next_reservation->timestamp_ns == source_ns + 2000000000LL &&
                f.node.startup_next_work_->pending.frame.timestamp_ns == source_ns + 2000000000LL,
                "startup refill reassigned existing identity");
    }

    // Terminal scheduling keeps ownership, accounting, and consumed batches intact.
    static void check_terminal_work(const Fixture &f, bool failed)
    {
        const auto queue = SlamSnapshotTestAccess::snapshot(f.node);
        require(f.node.tracking_failed_ == failed &&
                queue.enqueued == queue.pending + queue.in_flight + queue.processed +
                    queue.startup_discarded + queue.overload_discarded,
                "terminal scheduling broke accounting or failure state");

    }

    static void test_stop_checkpoint(bool normal, SlamNode::TestWorkPoint point)
    {
        Fixture f;
        f.enqueue(1);
        f.enqueue(2);
        f.receive(1);
        if (normal)
        {
            f.receive(2);
            f.node.process_pending_frames();
            f.enqueue(3);
        }
        if (point != SlamNode::TestWorkPoint::AfterWaiting)
            f.receive(normal ? 3 : 2);
        f.node.test_work_point_ = [&](SlamNode::TestWorkPoint current) {
            if (current == point) f.node.stop_control_->request_stop(StopReason::InputIdle);
        };
        f.node.process_pending_frames();
        const auto stopped = SlamSnapshotTestAccess::snapshot(f.node);
        const auto batch = normal ? stopped.reservation : stopped.startup_next_reservation;
        const bool waiting = point == SlamNode::TestWorkPoint::AfterWaiting;
        const auto &work = normal ? f.node.tracking_work_ : f.node.startup_next_work_;
        require(batch && work && work->imu_batch.has_value() == !waiting &&
                batch->stage == (waiting ? SlamNode::TrackingWorkStage::Reserved :
                                         SlamNode::TrackingWorkStage::Ready) &&
                f.calls.size() == (normal ? 2U : 0U),
                "stop checkpoint called backend or lost batch state");
        if (normal && !waiting)
        {
            const auto &work = *f.node.tracking_work_;
            require(work.imu_batch && work.imu_batch->measurements.size() == 1 &&
                    work.imu_batch->measurements.front().timestamp == stamp(3),
                    "normal unused batch lost its measurements");
        }
        check_terminal_work(f, false);
        const auto consumed = ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_);
        f.node.process_pending_frames();
        check_terminal_work(f, false);
        // Stop is irreversible; repeated scheduling must preserve the interrupted work.
        f.node.process_pending_frames();
        require(f.node.stop_control_->stop_requested() &&
                SlamSnapshotTestAccess::snapshot(f.node).processed == stopped.processed &&
                ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == consumed &&
                f.calls.size() == (normal ? 2U : 0U), "interrupted work became retryable");
    }

    static void test_finalization_without_retry()
    {
        Fixture f;
        f.enqueue(1);
        f.enqueue(2);
        f.enqueue(3);
        f.receive(1);
        f.node.process_pending_frames();
        const auto before = SlamSnapshotTestAccess::snapshot(f.node);
        f.node.stop_control_->request_stop(StopReason::InputIdle);
        // Repeated scheduling after stop must preserve the pending work.
        f.node.process_pending_frames();
        f.node.process_pending_frames();
        check_terminal_work(f, false);
        const auto after = SlamSnapshotTestAccess::snapshot(f.node);
        require(after.pending == before.pending && after.in_flight == before.in_flight &&
                after.processed == before.processed && after.first->enqueue_sequence == 3 &&
                !f.node.tracking_work_->imu_batch && !f.node.startup_next_work_->imu_batch,
                "finalization changed work ownership or counts");
    }

    static void test_before_backend_failure(bool normal, bool unknown)
    {
        Fixture f;
        f.enqueue(1);
        f.enqueue(2);
        f.receive(1);
        f.receive(2);
        if (normal)
        {
            f.node.process_pending_frames();
            f.enqueue(3);
            f.receive(3);
        }
        f.node.test_work_point_ = [&](SlamNode::TestWorkPoint point) {
            if (point == SlamNode::TestWorkPoint::BeforeBackend)
            {
                if (unknown) throw 42;
                throw LogFailure();
            }
        };
        bool failed = false;
        std::exception_ptr original_exception;
        try { f.node.process_pending_frames(); }
        catch (...) { failed = true; original_exception = std::current_exception(); }
        require(failed && f.calls.size() == (normal ? 2U : 0U),
                "pre-backend failure entered backend");
        check_terminal_work(f, true);
        const auto queue = SlamSnapshotTestAccess::snapshot(f.node);
        const auto batch = normal ? queue.reservation : queue.startup_next_reservation;
        const auto &work = normal ? f.node.tracking_work_ : f.node.startup_next_work_;
        require(work && work->imu_batch &&
                batch->stage == SlamNode::TrackingWorkStage::Ready && original_exception,
                "pre-backend failure falsely marked batch delivered");
        bool original = false;
        try { std::rethrow_exception(original_exception); }
        catch (int value) { original = unknown && value == 42; }
        catch (const LogFailure &) { original = !unknown; }
        require(original, "exception identity was lost");
    }

    static void test_stop_in_normal_backend()
    {
        Fixture f;
        for (int i = 1; i <= 3; ++i) { f.enqueue(i); f.receive(i); }
        f.node.process_pending_frames();
        f.on_track = [&]() { f.node.stop_control_->request_stop(StopReason::InputIdle); };
        f.node.process_pending_frames();
        const auto queue = SlamSnapshotTestAccess::snapshot(f.node);
        require(queue.processed == 3 && queue.outstanding == 0 &&
                f.node.last_tracked_frame_timestamp_ == stamp(3),
                "stop during successful backend prevented completion");
    }

    static void test_query_failure(ImuBatchStatus status, bool normal)
    {
        Fixture f(status == ImuBatchStatus::BufferOverflow ? 2 : 2000);
        if (normal)
        {
            // Seed a prior completion to exercise every normal-query error result,
            // including missing history in a newly supplied frontend.
            f.node.startup_complete_ = true;
            f.node.last_tracked_frame_timestamp_ = stamp(1);
        }
        else f.enqueue(1);
        f.enqueue(2);
        if (status == ImuBatchStatus::MissingHistory)
        {
            f.receive(2);
        }
        else if (status == ImuBatchStatus::DataGap)
        {
            f.receive(1);
            f.receive(3);
        }
        else
        {
            f.receive(1);
            f.receive(2);
            if (status == ImuBatchStatus::BufferOverflow) f.receive(3);
            else
            {
                // An already consumed interval makes this query InvalidRequest.
                require(f.node.imu_frontend_->takeMeasurements(stamp(1), stamp(2)).status ==
                            ImuBatchStatus::Ready, "failed to seed consumed interval");
            }
        }
        const auto consumed = ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_);
        bool failed = false;
        try { f.node.process_pending_frames(); }
        catch (const std::runtime_error &) { failed = true; }
        require(failed && f.calls.empty(), "fatal IMU result did not stop before backend");
        check_terminal_work(f, true);
        const auto queue = SlamSnapshotTestAccess::snapshot(f.node);
        const auto batch = normal ? queue.reservation : queue.startup_next_reservation;
        const auto &work = normal ? f.node.tracking_work_ : f.node.startup_next_work_;
        require(work && !work->imu_batch &&
                batch->stage == SlamNode::TrackingWorkStage::Reserved,
                "failed query falsely consumed a batch");
        bool rejected = false;
        try { f.node.process_pending_frames(); }
        catch (const std::logic_error &) { rejected = true; }
        require(rejected && ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == consumed,
                "failed query was retried");
    }

    enum class CoordinationFailure { StartupTimeout, FrameTimeout, ImuBackwards };

    static void test_coordination_failure(CoordinationFailure reason)
    {
        Fixture f;
        f.enqueue(1);
        f.enqueue(2);
        f.receive(1);
        f.node.process_pending_frames();  // Retain a real Waiting pair.
        if (reason == CoordinationFailure::ImuBackwards)
            f.receive(0);
        else
        {
            const auto expired = SlamNode::Clock::now() - std::chrono::seconds(7200);
            const std::lock_guard<std::mutex> lock(f.node.queue_mutex_);
            if (reason == CoordinationFailure::StartupTimeout)
                f.node.startup_wait_started_ = expired;
            else
            {
                f.node.tracking_work_->pending.received_at = expired;
                f.node.reservation_received_at_ = expired;
            }
        }
        bool failed = false;
        try { f.node.process_pending_frames(); }
        catch (const std::runtime_error &) { failed = true; }
        require(failed && f.calls.empty(), "coordination failure entered backend");
        check_terminal_work(f, true);
        f.node.stop_control_->request_stop(StopReason::InputIdle);
        check_terminal_work(f, true);
    }

    static void test_discard_logging_failure()
    {
        Fixture f;
        for (int i = 1; i <= 3; ++i) f.enqueue(i);
        f.receive(2);
        f.receive(3);
        f.node.node_logger_ = std::make_shared<spdlog::logger>(
            "discard_throwing_test", std::make_shared<ThrowingSink>());
        f.node.node_logger_->set_error_handler([](const std::string &) { throw LogFailure(); });
        bool failed = false;
        try { f.node.process_pending_frames(); }
        catch (const LogFailure &) { failed = true; }
        const auto queue = SlamSnapshotTestAccess::snapshot(f.node);
        require(failed && queue.startup_discarded == 1 && queue.in_flight == 1 && queue.pending == 1 &&
                queue.reservation->enqueue_sequence == 2 &&
                !f.node.tracking_work_->imu_batch &&
                f.node.tracking_failed_,
                "discard logging failure lost promotion or predecessor identity");
        check_terminal_work(f, true);
        bool rejected = false;
        try { f.node.process_pending_frames(); }
        catch (const std::logic_error &) { rejected = true; }
        require(rejected && SlamSnapshotTestAccess::snapshot(f.node).startup_discarded == 1 && f.calls.empty(),
                "discard logging failure allowed a second discard");
    }

    static void test_final_saved_batch(int failing_frame)
    {
        Fixture f;
        for (int i = 1; i <= 3; ++i) { f.enqueue(i); f.receive(i); }
        f.on_track = [&]() {
            if (failing_frame == 0) f.node.stop_control_->request_stop(StopReason::InputIdle);
            else if (f.calls.back().timestamp == stamp(failing_frame)) throw BackendFailure();
        };
        try { f.node.process_pending_frames(); }
        catch (const BackendFailure &) {}
        f.node.stop_control_->request_stop(StopReason::InputIdle);
        const auto consumed = ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_);
        const auto final = f.node.final_snapshot();
        const auto &work = *f.node.startup_next_work_;
        require(final.accounting_valid && final.queued == 1 &&
                final.in_flight == (failing_frame == 1 ? 2U : 1U) &&
                final.outstanding == final.queued + final.in_flight &&
                work.stage == (failing_frame == 2 ? SlamNode::TrackingWorkStage::Executing :
                                                   SlamNode::TrackingWorkStage::Ready) && work.imu_batch &&
                work.imu_batch->measurements.size() == 1 &&
                ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == consumed,
                "final accounting changed ownership or consumed the saved batch");
        // Holding a pixel reference lets us verify that the metadata snapshot owns none.
        cv::Mat retained = f.node.startup_next_work_->pending.frame.left;
        std::ostringstream logs;
        f.node.node_logger_ = std::make_shared<spdlog::logger>(
            "final_test", std::make_shared<spdlog::sinks::ostream_sink_mt>(logs));
        f.node.report_final_accounting(final);
        require(logs.str().find("STOP_ACCOUNTING") != std::string::npos &&
                logs.str().find("accounting=Valid") != std::string::npos,
                "final accounting summary was not reported");
        f.node.release_unfinished_work();
        f.node.release_unfinished_work();
        require(!f.node.tracking_work_ && !f.node.startup_next_work_ &&
                !f.node.reservation_received_at_ && !f.node.startup_next_received_at_ &&
                f.node.pending_frames_.empty() && retained.u->refcount == 1 &&
                final.outstanding == (failing_frame == 1 ? 3U : 2U),
                "release retained payloads or invalidated final metadata");
    }

    static void test_final_accounting_without_consumption()
    {
        Fixture f;
        f.node.trace_ = std::make_unique<DiagnosticTrace>("", 128);
        ImuFrontendTestAccess::attach_trace(*f.node.imu_frontend_, f.node.trace_.get());
        require(f.node.final_snapshot().outstanding == 0, "empty accounting invented work");
        f.enqueue(1);
        require(f.node.final_snapshot().queued == 1, "single queued frame was lost");
        f.enqueue(2);
        f.receive(1);
        f.node.process_pending_frames();
        auto final = f.node.final_snapshot();
        require(final.accounting_valid && final.queued == 0 && final.in_flight == 2,
                "waiting startup reservations disappeared from accounting");
        f.receive(2);
        const auto trace_before = f.node.trace_->stats();
        final = f.node.final_snapshot();
        require(final.accounting_valid &&
                f.node.trace_->stats().recorded == trace_before.recorded &&
                f.node.trace_->stats().dropped == trace_before.dropped &&
                !ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_),
                "final accounting changed trace or consumed startup data");
        f.node.process_pending_frames();
        f.enqueue(3);
        f.node.process_pending_frames();
        final = f.node.final_snapshot();
        require(final.accounting_valid && final.processed == 2 &&
                final.queued == 0 && final.in_flight == 1 &&
                f.node.tracking_work_->pending.enqueue_sequence == 3 &&
                ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == stamp(2),
                "normal waiting accounting changed ownership or consumption");
    }

    static void test_queued_stereo_final()
    {
        Fixture f;
        f.node.tracking_mode_ = TrackingMode::Stereo;
        int calls = 0;
        f.node.imu_frontend_.reset();
        f.node.test_track_ = [&](const StereoFrame &frame, const std::vector<ImuMeasurement> &batch) {
            require(batch.empty(), "Stereo backend received IMU data");
            require(frame.timestamp == calls - 2, "Stereo scheduling violated FIFO");
            const auto queue = SlamSnapshotTestAccess::snapshot(f.node);
            require(queue.in_flight == 1, "Stereo backend lost its reservation");
            ++calls;
        };
        for (int timestamp = -2; timestamp <= 0; ++timestamp)
        {
            StereoFrame frame;
            frame.timestamp = timestamp;
            f.node.on_frame(frame);
        }
        require(calls == 0, "Stereo reception called backend directly");
        for (int expected = 1; expected <= 3; ++expected)
        {
            f.node.process_pending_frames();
            const auto queue = SlamSnapshotTestAccess::snapshot(f.node);
            require(calls == expected && queue.processed == static_cast<uint64_t>(expected) &&
                    queue.in_flight == 0 && queue.pending == static_cast<std::size_t>(3 - expected),
                    "Stereo retry did not commit exactly one frame");
        }
        f.node.process_pending_frames();
        const auto final = f.node.final_snapshot();
        require(calls == 3 && final.processed == 3 && final.enqueued == 3 &&
                final.accounting_valid && final.outstanding == 0,
                "queued Stereo final accounting is invalid");
    }

    static void test_stereo_admission_and_stop()
    {
        // Stopping in Queued, Reserved or Ready must preserve the unfinished identity.
        for (int stage = 0; stage < 3; ++stage)
        {
            Fixture f;
            f.node.tracking_mode_ = TrackingMode::Stereo;
            f.node.imu_frontend_.reset();
            f.node.pending_frames_capacity_ = 1;
            int calls = 0;
            f.node.test_track_ = [&](const StereoFrame &, const std::vector<ImuMeasurement> &) { ++calls; };
            const auto empty = f.node.final_snapshot();
            require(empty.accounting_valid && empty.outstanding == 0,
                    "empty Stereo accounting is invalid");
            StereoFrame frame;
            frame.timestamp = -1;
            f.node.on_frame(frame);
            if (stage == 2)
            {
                f.node.test_work_point_ = [&](SlamNode::TestWorkPoint point) {
                    if (point == SlamNode::TestWorkPoint::AfterReady) f.node.stop_control_->request_stop(StopReason::InputIdle);
                };
                f.node.process_pending_frames();
            }
            else if (stage == 1)
            {
                // A reservation occupies the entire capacity; no queued frame can be evicted.
                require(f.node.reserve_tracking_work(), "Stereo reservation failed");
                frame.timestamp = 0;
                bool rejected = false;
                try { f.node.on_frame(frame); }
                catch (const std::runtime_error &) { rejected = true; }
                const auto full = SlamSnapshotTestAccess::snapshot(f.node);
                require(rejected && full.enqueued == 1 && full.in_flight == 1 &&
                        full.overload_discarded == 0 && full.reservation->timestamp == -1,
                        "capacity-one admission replaced reserved work");
                f.node.stop_control_->request_stop(StopReason::InputIdle);
                f.node.process_pending_frames();
            }
            else
            {
                f.node.stop_control_->request_stop(StopReason::InputIdle);
                f.node.process_pending_frames();
            }
            const auto final = f.node.final_snapshot();
            require(final.queued == (stage == 0 ? 1U : 0U) &&
                    final.in_flight == (stage == 0 ? 0U : 1U),
                    "Stereo stop changed queue ownership");
            require(calls == 0 && final.accounting_valid && final.processed == 0 &&
                    final.outstanding == 1 &&
                    (stage == 0 ? f.node.pending_frames_.front().frame.timestamp :
                                  f.node.tracking_work_->pending.frame.timestamp) == -1,
                    "Stereo stop lost unfinished work or called backend");
        }
        Fixture imu;
        StereoFrame frame;
        frame.timestamp = -1;
        bool rejected = false;
        try { imu.node.on_frame(frame); }
        catch (const std::invalid_argument &) { rejected = true; }
        require(rejected && imu.node.queue_snapshot().enqueued == 0,
                "Stereo-IMU accepted a negative timestamp");
    }

    static void test_stereo_backend_outcomes()
    {
        enum class Outcome { Stop, BackendFailure, LoggingFailure };
        for (const Outcome outcome : {Outcome::Stop, Outcome::BackendFailure, Outcome::LoggingFailure})
        {
            Fixture f;
            f.node.tracking_mode_ = TrackingMode::Stereo;
            f.node.imu_frontend_.reset();
            int calls = 0;
            f.node.test_track_ = [&](const StereoFrame &, const std::vector<ImuMeasurement> &batch) {
                require(batch.empty(), "Stereo backend received IMU data");
                ++calls;
                if (outcome == Outcome::Stop) f.node.stop_control_->request_stop(StopReason::InputIdle);
                if (outcome == Outcome::BackendFailure) throw BackendFailure();
            };
            if (outcome == Outcome::LoggingFailure)
            {
                f.node.node_logger_ = std::make_shared<spdlog::logger>(
                    "stereo_throwing_test", std::make_shared<ThrowingSink>());
                f.node.node_logger_->set_error_handler([](const std::string &) { throw LogFailure(); });
            }
            // Keep a second frame queued to detect accidental draining or duplicate completion.
            for (int timestamp = 1; timestamp <= 2; ++timestamp)
            {
                StereoFrame frame;
                frame.timestamp = timestamp;
                f.node.on_frame(frame);
            }
            bool failed = false;
            try { f.node.process_pending_frames(); }
            catch (const BackendFailure &) { require(outcome == Outcome::BackendFailure, "wrong exception"); failed = true; }
            catch (const LogFailure &) { require(outcome == Outcome::LoggingFailure, "wrong exception"); failed = true; }
            const auto queue = SlamSnapshotTestAccess::snapshot(f.node);
            const bool backend_failed = outcome == Outcome::BackendFailure;
            require(calls == 1 && failed == (outcome != Outcome::Stop) && queue.pending == 1 &&
                    queue.processed == (backend_failed ? 0U : 1U) &&
                    queue.in_flight == (backend_failed ? 1U : 0U),
                    "Stereo outcome lost completion or drained queued work");
            if (backend_failed)
            {
                require(queue.reservation && f.node.tracking_failed_ &&
                        f.node.tracking_callback_reason_ == StopReason::BackendError,
                        "Stereo backend failure lost its cause");
                bool rejected = false;
                try { f.node.process_pending_frames(); }
                catch (const std::logic_error &) { rejected = true; }
                require(rejected && calls == 1, "Stereo failed backend was retried");
            }
            // Model the caller stopping after an exception; B5 owns the real callback boundary.
            f.node.stop_control_->request_stop(StopReason::InputIdle);
            f.node.process_pending_frames();
            const auto final = f.node.final_snapshot();
            require(calls == 1 && final.accounting_valid &&
                    final.outstanding == (backend_failed ? 2U : 1U) &&
                    f.node.pending_frames_.back().enqueue_sequence == 2 &&
                    (!backend_failed || f.node.tracking_work_->pending.enqueue_sequence == 1),
                    "Stereo finalization lost unfinished identities or retried completed work");
        }
    }

    static void test_concurrent_completion(bool fail_normal)
    {
        Fixture f;
        f.node.pending_frames_capacity_ = 3;
        f.enqueue(1); f.enqueue(2);
        for (int i = 1; i <= 5; ++i) f.receive(i);

        // The test thread is the producer; only the consumer accesses tracking payloads.
        std::mutex mutex;
        std::condition_variable changed;
        int entered = 0, released = 0;
        bool abort = false;
        std::atomic<bool> finished{false};
        std::exception_ptr consumer_error, observer_error;
        f.on_track = [&]() {
            std::unique_lock<std::mutex> lock(mutex);
            const int current = ++entered;
            changed.notify_all();
            if (!changed.wait_for(lock, std::chrono::seconds(3), [&]() {
                    return abort || released >= current;
                }) || abort)
                throw std::runtime_error("backend gate timed out or aborted");
            if (fail_normal && current == 3) throw BackendFailure();
        };
        std::thread consumer, observer;
        const auto cleanup = [&]() {
            {
                const std::lock_guard<std::mutex> lock(mutex);
                abort = true;
            }
            changed.notify_all();
            if (consumer.joinable()) consumer.join();
            finished.store(true);
            if (observer.joinable()) observer.join();
        };
        const auto wait_entry = [&](int call) {
            std::unique_lock<std::mutex> lock(mutex);
            require(changed.wait_for(lock, std::chrono::seconds(3), [&]() {
                        return entered >= call || finished.load();
                    }) && entered == call, "consumer failed to reach backend gate");
        };
        const auto release = [&](int call) {
            const std::lock_guard<std::mutex> lock(mutex);
            released = call;
            changed.notify_all();
        };
        try
        {
            consumer = std::thread([&]() {
                try
                {
                    f.node.process_pending_frames();  // F0 and F1 commit independently.
                    f.node.process_pending_frames();  // Normal frame 3.
                }
                catch (...) { consumer_error = std::current_exception(); }
                finished.store(true);
                changed.notify_all();
            });
            observer = std::thread([&]() {
                try
                {
                    uint64_t processed = 0, enqueued = 0;
                    do
                    {
                        const auto q = f.node.queue_snapshot();
                        require(q.enqueued == q.pending + q.in_flight + q.processed +
                                    q.startup_discarded + q.overload_discarded &&
                                q.outstanding == q.pending + q.in_flight && q.outstanding <= 3 &&
                                q.in_flight <= 2 && q.peak <= 3 &&
                                q.processed >= processed && q.enqueued >= enqueued,
                                "observer saw a torn completion/admission snapshot");
                        processed = q.processed;
                        enqueued = q.enqueued;
                        std::this_thread::yield();
                    } while (!finished.load());
                }
                catch (...) { observer_error = std::current_exception(); }
            });
            wait_entry(1);
            f.enqueue(3);  // Must finish while backend F0 has not returned.
            bool rejected = false;
            try { f.enqueue(4); } catch (const std::runtime_error &) { rejected = true; }
            auto q = SlamSnapshotTestAccess::snapshot(f.node);
            require(rejected && q.enqueued == 3 && q.processed == 0 && q.in_flight == 2 &&
                    q.first->enqueue_sequence == 3, "full startup admission corrupted work");
            release(1);
            wait_entry(2);
            q = SlamSnapshotTestAccess::snapshot(f.node);
            require(q.processed == 1 && q.in_flight == 1 && !q.reservation &&
                    q.startup_next_reservation->enqueue_sequence == 2,
                    "F0 completion did not release one capacity slot");
            f.enqueue(4);
            require(SlamSnapshotTestAccess::snapshot(f.node).second->enqueue_sequence == 4,
                    "rejected admission consumed a sequence");
            release(2);
            wait_entry(3);
            q = SlamSnapshotTestAccess::snapshot(f.node);
            require(q.processed == 2 && q.startup_complete && q.reservation->enqueue_sequence == 3,
                    "F1 completion or normal FIFO reservation failed");
            f.enqueue(5);
            rejected = false;
            try { f.enqueue(6); } catch (const std::runtime_error &) { rejected = true; }
            require(rejected && SlamSnapshotTestAccess::snapshot(f.node).enqueued == 5,
                    "normal in-flight capacity was not bounded");
            release(3);
            consumer.join();
            observer.join();
        }
        catch (...) { cleanup(); throw; }
        if (observer_error) std::rethrow_exception(observer_error);
        bool backend_failed = false;
        if (consumer_error)
        {
            try { std::rethrow_exception(consumer_error); }
            catch (const BackendFailure &) { backend_failed = true; }
        }
        require(backend_failed == fail_normal && f.calls.size() == 3 &&
                f.calls[0].timestamp == stamp(1) && f.calls[1].timestamp == stamp(2) &&
                f.calls[2].timestamp == stamp(3) &&
                f.node.last_tracked_frame_timestamp_ == stamp(fail_normal ? 2 : 3) &&
                ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == stamp(3),
                "concurrent completion changed FIFO or consumption");
        // All threads have joined before inspecting private payloads or finalizing.
        const auto final = f.node.final_snapshot();
        std::vector<uint64_t> sequences;
        if (f.node.tracking_work_)
            sequences.push_back(f.node.tracking_work_->pending.enqueue_sequence);
        for (const auto &pending : f.node.pending_frames_)
            sequences.push_back(pending.enqueue_sequence);
        require(final.accounting_valid && final.outstanding == sequences.size() &&
                sequences == (fail_normal ? std::vector<uint64_t>{3, 4, 5} :
                                           std::vector<uint64_t>{4, 5}),
                "remaining work differs from accepted minus completed work");
        if (fail_normal)
        {
            const auto &work = *f.node.tracking_work_;
            require(f.node.tracking_failed_ &&
                    f.node.tracking_callback_reason_ == StopReason::BackendError &&
                    work.stage == SlamNode::TrackingWorkStage::Executing &&
                    work.imu_batch && work.imu_batch->measurements.size() == 1 &&
                    work.imu_batch->measurements.front().timestamp == stamp(3),
                    "normal failed batch or its cause was lost");
        }
    }

    static void test_frontend_callback_boundaries()
    {
        for (bool raw_failure : {false, true})
        {
            Fixture f;
            int cancellations = 0;
            f.request_stop = [&]() { ++cancellations; };
            StereoFrontend frontend(&f.node, f.node.reception_group_, "/boundary_left", "/boundary_right",
                [](const StereoFrame &) { throw std::runtime_error("frame failure"); },
                [raw_failure]() { if (raw_failure) throw 42; }, nullptr, f.node.callback_guard_);
            for (int second = 1; second <= 3; ++second)
            {
                StereoFrontendTestAccess::dispatch(frontend, true, second);
                StereoFrontendTestAccess::dispatch(frontend, false, second);
            }
            const auto stop = f.node.stop_control_->snapshot();
            require(cancellations > 0 && stop.first_failure &&
                    stop.first_failure->reason == StopReason::CallbackError,
                    "registered frontend callback did not contain failure");
            bool original = false;
            try { std::rethrow_exception(stop.first_exception); }
            catch (int value) { original = raw_failure && value == 42; }
            catch (const std::runtime_error &) { original = !raw_failure; }
            require(original, "frontend lost the original callback exception");
        }
    }

    static void test_failure_classification()
    {
        for (const auto reason : {StopReason::Capacity, StopReason::Timeout, StopReason::SamplingError})
        {
            Fixture f;
            f.enqueue(1);
            if (reason == StopReason::Capacity)
            {
                f.node.pending_frames_capacity_ = 1;
                f.node.callback_guard_->run([&]() { f.enqueue(2); });
            }
            else
            {
                if (reason == StopReason::Timeout) f.node.imu_wait_timeout_sec_ = 0.0;
                else { f.receive(2); f.receive(1); }
                f.node.tracking_callback();
            }
            const auto stop = f.node.stop_control_->snapshot();
            require(stop.first_failure && stop.first_failure->reason == reason && f.calls.empty(),
                    "real failure source was misclassified or entered backend");
        }
    }

    static void test_callback_failures()
    {
        for (bool unknown : {false, true})
        {
            Fixture f;
            f.enqueue(1); f.enqueue(2); f.receive(1); f.receive(2);
            int cancellations = 0;
            f.request_stop = [&]() {
                require(f.node.stop_control_->stop_requested(), "cancel preceded failure publication");
                ++cancellations;
            };
            f.on_track = [unknown]() { if (unknown) throw 42; throw BackendFailure(); };
            f.node.tracking_callback();
            const auto stop = f.node.stop_control_->snapshot();
            require(stop.first_failure && stop.first_failure->reason == StopReason::BackendError &&
                    stop.first_exception && cancellations == 1 && f.calls.size() == 1 &&
                    SlamSnapshotTestAccess::snapshot(f.node).processed == 0, "backend callback failure escaped or misclassified");
            bool original = false;
            try { std::rethrow_exception(stop.first_exception); }
            catch (int value) { original = unknown && value == 42; }
            catch (const BackendFailure &) { original = !unknown; }
            require(original, "callback boundary changed the backend exception");
            f.node.tracking_callback();
            require(f.calls.size() == 1, "callback retried failed backend");
        }
        Fixture f;
        f.enqueue(1); f.enqueue(2); f.receive(1); f.receive(2);
        f.node.node_logger_ = std::make_shared<spdlog::logger>(
            "callback_logging_failure", std::make_shared<ThrowingSink>());
        f.node.node_logger_->set_error_handler([](const std::string &) { throw LogFailure(); });
        f.node.tracking_callback();
        require(SlamSnapshotTestAccess::snapshot(f.node).processed == 1 &&
                f.node.stop_control_->snapshot().first_failure->reason == StopReason::CallbackError,
                "post-completion logging failure lost completion or backend classification");
    }

    static void test_stop_boundaries()
    {
        // Force both orderings around the real control gate without executor timing assumptions.
        for (bool stereo : {false, true})
        for (bool granted : {false, true})
        {
            Fixture f;
            if (stereo) f.node.tracking_mode_ = TrackingMode::Stereo;
            f.enqueue(1); f.enqueue(2);
            f.receive(1); f.receive(2);
            f.node.test_work_point_ = [&](SlamNode::TestWorkPoint point) {
                if (point == (granted ? SlamNode::TestWorkPoint::AfterBackendPermit :
                                       SlamNode::TestWorkPoint::BeforeBackendPermit))
                    f.node.stop_control_->request_stop(StopReason::InputIdle);
            };
            f.node.process_pending_frames();
            require(f.calls.size() == (granted ? 1U : 0U) &&
                    f.node.stop_control_->snapshot().backend_starts == f.calls.size() &&
                    SlamSnapshotTestAccess::snapshot(f.node).processed == f.calls.size(),
                    "backend start/stop ordering violated permission or completion");
            const auto before = SlamSnapshotTestAccess::snapshot(f.node);
            StereoFrame incoming;
            incoming.timestamp = 3;
            f.node.enqueue_frame(incoming);
            f.node.process_pending_frames();
            require(SlamSnapshotTestAccess::snapshot(f.node).enqueued == before.enqueued &&
                    SlamSnapshotTestAccess::snapshot(f.node).processed == before.processed,
                    "stopped admission or retry changed accounting");
        }

        for (bool normal : {false, true})
        {
            Fixture f;
            f.enqueue(1); f.enqueue(2); f.receive(1); f.receive(2);
            if (normal)
            {
                f.node.process_pending_frames();
                f.enqueue(3); f.receive(3);
            }
            const auto consumed = ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_);
            const auto stats = f.node.imu_frontend_->stats();
            f.node.test_work_point_ = [&](SlamNode::TestWorkPoint point) {
                if (point == SlamNode::TestWorkPoint::BeforeImuQuery)
                    f.node.stop_control_->request_stop(StopReason::InputIdle);
            };
            f.node.process_pending_frames();
            require(f.calls.size() == (normal ? 2U : 0U) &&
                    ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == consumed,
                    "stop before query consumed IMU or started backend");
            f.receive(4);
            const auto after = f.node.imu_frontend_->stats();
            require(after.received == stats.received + 1 && after.stopped == stats.stopped + 1 &&
                    after.accepted == stats.accepted && after.buffered == stats.buffered,
                    "stopped IMU callback altered accepted history");
            const double left = stamp(normal ? 2 : 1), right = stamp(normal ? 3 : 2);
            require(f.node.imu_frontend_->takeMeasurements(left, right).status == ImuBatchStatus::Stopped &&
                    f.node.imu_frontend_->inspectMeasurements(left, right) == ImuBatchStatus::Ready,
                    "stopped consumption disabled final read-only coverage");
        }
    }

    static void test_idle_stop_control()
    {
        // Direct calls and an unspun timer keep this test finite and hardware-free.
        for (bool shutdown : {false, true})
        {
            Fixture f;
            const auto control = f.node.stop_control_;
            int cancellations = 0;
            f.request_stop = [&]() {
                const StopSnapshot snapshot = control->snapshot();
                require(snapshot.first_stop && snapshot.first_stop->reason == StopReason::InputIdle,
                        "executor cancellation preceded idle stop publication");
                ++cancellations;
            };
            f.node.input_timer_ = f.node.create_wall_timer(std::chrono::hours(1), []() {});
            f.node.input_timeout_action_ = shutdown ? "shutdown" : "warn";
            f.node.input_timeout_sec_ = 1.0;
            f.node.last_input_activity_ = SlamNode::Clock::now() - std::chrono::seconds(2);
            f.node.check_input_timeout();
            require(control->stop_requested() == shutdown && cancellations == (shutdown ? 1 : 0) &&
                    f.node.input_timer_->is_canceled() == shutdown,
                    "idle action changed stop or cancellation policy");
            f.node.check_input_timeout();
            require(cancellations == (shutdown ? 1 : 0), "idle stop was submitted twice");
            if (shutdown)
            {
                const auto before = control->snapshot();
                control->request_stop(StopReason::Finalization);
                const auto after = control->snapshot();
                require(after.first_stop->reason == StopReason::InputIdle &&
                        after.first_stop->time == before.first_stop->time && !after.first_failure,
                        "finalization replaced idle cause or invented failure");
                f.node.on_frame(StereoFrame{});
                require(SlamSnapshotTestAccess::snapshot(f.node).enqueued == 0,
                        "shared stop did not prevent frame admission");
            }
            else
            {
                f.node.on_input_activity();
                require(!f.node.input_timeout_reported_ && !control->stop_requested(),
                        "warn prevented input resumption");
            }
        }

        // A diagnostic failure must occur only after stop and cancellation are published.
        Fixture f;
        bool cancelled = false;
        f.request_stop = [&]() { cancelled = true; };
        f.node.input_timer_ = f.node.create_wall_timer(std::chrono::hours(1), []() {});
        f.node.input_timeout_action_ = "shutdown";
        f.node.last_input_activity_ = SlamNode::Clock::now() - std::chrono::seconds(10);
        f.node.node_logger_ = std::make_shared<spdlog::logger>(
            "idle_throwing_test", std::make_shared<ThrowingSink>());
        f.node.node_logger_->set_error_handler([](const std::string &) { throw LogFailure(); });
        bool failed = false;
        try { f.node.check_input_timeout(); }
        catch (const LogFailure &) { failed = true; }
        require(failed && cancelled && f.node.stop_control_->stop_requested() &&
                f.node.input_timer_->is_canceled(), "idle logging failure prevented stop");
    }

    static void test_finalization_steps()
    {
        for (const std::string failure : {"none", "snapshot", "logging", "trace", "backend", "prior_failure"})
        {
            Fixture f;
            f.enqueue(1); f.enqueue(2); f.receive(1); f.receive(2);
            require(f.node.reserve_startup_pair(), "finalization setup did not reserve work");
            const auto buffered = f.node.imu_frontend_->stats().buffered;
            const auto control = f.node.stop_control_;
            control->request_stop(StopReason::InputIdle);
            const auto original = std::make_exception_ptr(BackendFailure());
            if (failure == "prior_failure") control->record_failure(StopReason::BackendError, original);
            const auto before = control->snapshot();
            std::ostringstream output;
            f.node.node_logger_ = std::make_shared<spdlog::logger>(
                "finalization_test", std::make_shared<spdlog::sinks::ostream_sink_mt>(output));
            if (failure == "logging")
            {
                f.node.node_logger_ = std::make_shared<spdlog::logger>(
                    "finalization_throw", std::make_shared<ThrowingSink>());
                f.node.node_logger_->set_error_handler([](const std::string &) { throw LogFailure(); });
            }
            const auto path = std::filesystem::temp_directory_path() /
                ("gemini-finalization-" + std::to_string(SlamNode::Clock::now().time_since_epoch().count()) + ".csv");
            f.node.trace_ = std::make_unique<DiagnosticTrace>(failure == "trace" ? "" : path.string(), 8);
            f.node.trace_->record("finalization_test", 1);
            int backend_calls = 0;
            f.node.test_shutdown_ = [&]() {
                ++backend_calls;
                require(!f.node.imu_frontend_ && !f.node.tracking_work_ &&
                        !f.node.startup_next_work_ && f.node.pending_frames_.empty(),
                        "backend shutdown preceded work/frontend release");
                if (failure == "backend") throw 42;
            };
            std::vector<std::string> stages;
            f.node.test_finalize_step_ = [&](const char *stage) {
                stages.emplace_back(stage);
                if (std::string(stage) == "snapshot")
                {
                    require(!ImuFrontendTestAccess::subscription_present(*f.node.imu_frontend_) &&
                            f.node.imu_frontend_->stats().buffered == buffered,
                            "IMU stop destroyed history or retained its subscription");
                    if (failure == "snapshot") throw std::runtime_error("snapshot failure");
                }
                if (std::string(stage) == "work_report" && failure == "prior_failure")
                    throw std::runtime_error("later cleanup failure");
            };
            f.node.shutdown();
            const auto after = control->snapshot();
            require(backend_calls == 1 && after.first_stop->reason == before.first_stop->reason &&
                    after.first_stop->time == before.first_stop->time,
                    "finalization changed stop cause or skipped backend");
            require(after.cleanup_failed == (failure != "none") &&
                    bool(after.first_failure) == (failure != "none"), "cleanup failure result mismatch");
            if (failure == "trace") require(after.first_failure->reason == StopReason::TraceWriteError,
                                             "trace failure not classified");
            if (failure == "prior_failure") require(after.first_exception == original &&
                    after.first_failure->reason == StopReason::BackendError, "cleanup replaced original failure");
            if (failure == "snapshot") require(output.str().find("Final work snapshot unavailable") != std::string::npos &&
                    output.str().find("STOP_ACCOUNTING") == std::string::npos, "missing snapshot fabricated accounting");
            if (failure == "none") require(output.str().find("in_flight=2") != std::string::npos,
                                            "unfinished work was not reported before release");
            const auto stage_count = stages.size();
            f.node.shutdown();
            require(stages.size() == stage_count && backend_calls == 1,
                    "repeated shutdown repeated cleanup");
            require(std::find(stages.begin(), stages.end(), "trace_write") <
                    std::find(stages.begin(), stages.end(), "release_work"), "trace came after work release");
            if (failure != "trace")
            {
                require(std::filesystem::exists(path), "earlier failure prevented trace export");
                std::filesystem::remove(path);
            }
        }
    }

    static void test_partial_construction_logging()
    {
        // Default settings_path is invalid before backend construction. Retain the
        // real logging session through that constructor failure without starting SLAM.
        const char *previous = std::getenv("GEMINI336_SLAM_LOG_DIR");
        const std::optional<std::string> saved = previous ? std::optional<std::string>(previous) : std::nullopt;
        const auto root = std::filesystem::temp_directory_path() /
            ("gemini-partial-construction-" + std::to_string(SlamNode::Clock::now().time_since_epoch().count()));
        require(setenv("GEMINI336_SLAM_LOG_DIR", root.c_str(), 1) == 0, "cannot set test log root");
        const auto restore = [&]() {
            if (saved) setenv("GEMINI336_SLAM_LOG_DIR", saved->c_str(), 1);
            else unsetenv("GEMINI336_SLAM_LOG_DIR");
        };
        std::shared_ptr<LoggingSession> logging;
        auto control = std::make_shared<StopControl>();
        bool failed = false;
        try { auto node = std::make_shared<SlamNode>(control, []() {}, logging); }
        catch (const std::invalid_argument &) { failed = true; }
        catch (...) { restore(); throw; }
        restore();
        require(failed && logging, "partial node construction lost process logging ownership");
        logging->GetLogger("partial_construction")->info("constructor failed; main still owns logging");
        logging->finish();
        require(std::filesystem::file_size(logging->directory() / "slam.log") > 0,
                "partial construction diagnostics were lost");
        logging.reset();
        std::filesystem::remove_all(root);
    }

    static void run()
    {
        test_partial_construction_logging();
        test_finalization_steps();
        test_frontend_callback_boundaries();
        test_failure_classification();
        test_callback_failures();
        test_stop_boundaries();
        test_idle_stop_control();
        test_concurrent_completion(false);
        test_concurrent_completion(true);
        for (int failure = 0; failure <= 2; ++failure) test_final_saved_batch(failure);
        test_final_accounting_without_consumption();
        test_queued_stereo_final();
        test_stereo_admission_and_stop();
        test_stereo_backend_outcomes();
        for (bool normal : {false, true})
        {
            for (const auto point : {SlamNode::TestWorkPoint::AfterWaiting,
                                    SlamNode::TestWorkPoint::AfterReady,
                                    SlamNode::TestWorkPoint::BeforeBackend})
                test_stop_checkpoint(normal, point);
            for (bool unknown : {false, true}) test_before_backend_failure(normal, unknown);
            for (const auto status : {ImuBatchStatus::BufferOverflow, ImuBatchStatus::DataGap,
                                     ImuBatchStatus::InvalidRequest})
                test_query_failure(status, normal);
        }
        test_query_failure(ImuBatchStatus::MissingHistory, true);
        for (const auto reason : {CoordinationFailure::StartupTimeout, CoordinationFailure::FrameTimeout,
                                 CoordinationFailure::ImuBackwards})
            test_coordination_failure(reason);
        test_discard_logging_failure();
        test_finalization_without_retry();
        test_stop_in_normal_backend();
        for (int frame = 1; frame <= 3; ++frame) test_backend_failure(frame, true);
        test_source_identity();
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
