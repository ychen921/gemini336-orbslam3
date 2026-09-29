// Exercise real scheduling, IMU queries and completion; never construct ORB-SLAM3.
#include "../src/slam_node.cpp"

#include <iostream>
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
    static void receive(StereoFrontend &frontend,
                        const StereoFrontend::Image::ConstSharedPtr &left,
                        const StereoFrontend::Image::ConstSharedPtr &right)
    {
        frontend.stereo_callback(left, right);
    }
};

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

        explicit Fixture(int64_t capacity = 2000)
        {
            node.tracking_mode_ = TrackingMode::StereoImu;
            // Tests advance sensor time, never wait for wall-clock timeouts.
            node.imu_wait_timeout_sec_ = 3600.0;
            node.declare_parameter("imu.max_gap_sec", 1.1);
            node.declare_parameter("imu.buffer_capacity", capacity);
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
                const auto &active = queue.reservation ? queue.reservation : queue.startup_next_reservation;
                require(active && active->stage == SlamNode::TrackingWorkStage::Executing &&
                        !active->interruption &&
                        active->batch_use == (imu.empty() ? SlamNode::ImuBatchUse::NotRequired :
                                              SlamNode::ImuBatchUse::DeliveredToBackend),
                        "backend entry has incorrect progress or batch state");
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
        const auto initial = f.node.queue_snapshot();
        for (int retry = 0; retry < 3; ++retry)
        {
            f.node.process_pending_frames();
            const auto waiting = f.node.queue_snapshot();
            require(f.calls.empty() && waiting.pending == 0 && waiting.in_flight == 2 &&
                    waiting.processed == 0 && waiting.reservation &&
                    waiting.startup_next_reservation &&
                    waiting.reservation->enqueue_sequence == 1 &&
                    waiting.reservation->timestamp_ns == 1000000000LL &&
                    waiting.startup_next_reservation->enqueue_sequence == 2 &&
                    waiting.startup_next_reservation->timestamp_ns == 2000000000LL &&
                    waiting.startup_next_reservation->batch_use == SlamNode::ImuBatchUse::NotAcquired &&
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
                    snapshot.reservation->enqueue_sequence == 3 &&
                    snapshot.reservation->timestamp_ns == 3000000000LL &&
                    ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == stamp(2),
                    "normal Waiting consumed IMU or lost the reserved frame");
        }
        f.receive(3);
        f.node.process_pending_frames();
        snapshot = f.node.queue_snapshot();
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
        const auto initial = f.node.queue_snapshot();
        for (uint64_t discarded = 1; discarded <= 2; ++discarded)
        {
            f.node.process_pending_frames();
            const auto snapshot = f.node.queue_snapshot();
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
                f.node.last_tracked_frame_timestamp_ == stamp(1) &&
                snapshot.in_flight == 1 && !snapshot.reservation &&
                snapshot.startup_next_reservation &&
                snapshot.startup_next_reservation->stage == SlamNode::TrackingWorkStage::Ready &&
                snapshot.startup_next_reservation->batch_use == SlamNode::ImuBatchUse::ConsumedUnused &&
                !snapshot.startup_complete && snapshot.startup_started &&
                !f.node.tracking_work_ && f.node.startup_next_work_ &&
                f.node.startup_next_work_->imu_batch &&
                f.node.startup_next_work_->imu_batch->measurements.size() == 1 &&
                ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == stamp(2),
                "stop after F0 lost the unexecuted F1 or consumed batch");
        require(snapshot.startup_next_reservation->interruption &&
                snapshot.startup_next_reservation->interruption->kind == SlamNode::WorkInterruptionKind::Stopped &&
                snapshot.startup_next_reservation->interruption->location == SlamNode::WorkLocation::AfterBackend,
                "F1 did not retain the stop after F0 completion");
        f.node.process_pending_frames();
        require(f.calls.size() == 1 && f.node.queue_snapshot().processed == 1,
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
                if (stop_in_backend) f.node.stop_requested_ = true;
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
        const auto snapshot = f.node.queue_snapshot();
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
        require(failed_summary && failed_summary->interruption &&
                failed_summary->interruption->kind == SlamNode::WorkInterruptionKind::Failed &&
                failed_summary->interruption->reason == SlamNode::WorkReason::BackendException &&
                failed_summary->interruption->location == SlamNode::WorkLocation::Backend &&
                failed_summary->interruption->related_sequence == static_cast<uint64_t>(failing_frame) &&
                failed_summary->interruption->exception &&
                failed_summary->batch_use == (failing_frame == 1 ? SlamNode::ImuBatchUse::NotRequired :
                                              SlamNode::ImuBatchUse::DeliveredToBackend),
                "backend failure lost its cause or batch delivery state");
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
            require(snapshot.startup_next_reservation->interruption &&
                    snapshot.startup_next_reservation->interruption->related_sequence == 1 &&
                    snapshot.startup_next_reservation->batch_use == SlamNode::ImuBatchUse::ConsumedUnused &&
                    snapshot.startup_next_reservation->stage == SlamNode::TrackingWorkStage::Ready,
                    "F0 failure did not explain the unused F1 batch");
        // Finalization must not overwrite a failure with a generic stop.
        f.node.stop_requested_ = true;
        f.node.observe_work_stop(SlamNode::WorkLocation::Finalization);
        f.node.observe_work_stop(SlamNode::WorkLocation::Finalization);
        require((failing_frame == 2 ? f.node.queue_snapshot().startup_next_reservation :
                    f.node.queue_snapshot().reservation)->interruption->reason ==
                    SlamNode::WorkReason::BackendException,
                "finalization overwrote the original backend failure");
        f.node.stop_requested_ = false;
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
                f.node.last_tracked_frame_timestamp_ == stamp(1) &&
                snapshot.in_flight == 1 && !snapshot.reservation &&
                !f.node.tracking_work_ && snapshot.startup_next_reservation &&
                snapshot.startup_next_reservation->stage == SlamNode::TrackingWorkStage::Ready &&
                snapshot.startup_next_reservation->batch_use == SlamNode::ImuBatchUse::ConsumedUnused &&
                f.node.startup_next_work_ && f.node.startup_next_work_->imu_batch,
                "logging failure prevented F0 completion or lost F1");
        require(snapshot.startup_next_reservation->interruption &&
                snapshot.startup_next_reservation->interruption->reason == SlamNode::WorkReason::AfterBackendException &&
                snapshot.startup_next_reservation->interruption->location == SlamNode::WorkLocation::AfterBackend &&
                snapshot.startup_next_reservation->interruption->related_sequence == 1 &&
                snapshot.startup_next_reservation->interruption->exception,
                "post-completion logging failure was mistaken for backend failure");
        bool retry_rejected = false;
        try { f.node.process_pending_frames(); }
        catch (const std::logic_error &) { retry_rejected = true; }
        require(retry_rejected && f.calls.size() == 1 &&
                f.node.queue_snapshot().processed == 1,
                "logging failure caused completed F0 to be retried");
    }

    static void test_source_identity()
    {
        Fixture f;
        std::ostringstream logs;
        f.node.node_logger_ = std::make_shared<spdlog::logger>(
            "identity_test", std::make_shared<spdlog::sinks::ostream_sink_mt>(logs));
        StereoFrontend frontend(&f.node, "/unused_left", "/unused_right",
            [&](const StereoFrame &frame) { f.node.enqueue_frame(frame); });
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
        const auto first = f.node.queue_snapshot();
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
        const auto queued = f.node.queue_snapshot();
        require(queued.second && queued.second->enqueue_sequence == 2,
                "rejected source consumed an enqueue sequence");
        require(f.node.reserve_startup_pair(), "source pair was not reserved");
        for (int retry = 0; retry < 3; ++retry)
        {
            require(f.node.reserve_startup_pair(), "retry lost source pair");
            const auto held = f.node.queue_snapshot();
            require(held.reservation->timestamp_ns == source_ns &&
                    held.reservation->enqueue_sequence == 1 &&
                    held.reservation->received_at == first.first->received_at &&
                    held.startup_next_reservation->timestamp_ns == source_ns + 1000000000LL &&
                    f.node.tracking_work_->pending.frame.timestamp_ns == source_ns,
                    "reservation retry changed source identity");
        }
        f.node.discard_startup_first();
        const auto promoted = f.node.queue_snapshot();
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
        const auto refilled = f.node.queue_snapshot();
        require(refilled.reservation->enqueue_sequence == 2 &&
                refilled.reservation->received_at == queued.second->received_at &&
                refilled.startup_next_reservation->enqueue_sequence == 3 &&
                refilled.startup_next_reservation->timestamp_ns == source_ns + 2000000000LL &&
                f.node.startup_next_work_->pending.frame.timestamp_ns == source_ns + 2000000000LL,
                "startup refill reassigned existing identity");
    }

    // Assert both ownership domains, retained identity, and accounting after terminal events.
    static void check_interruption(const Fixture &f, SlamNode::WorkReason reason,
                                   SlamNode::WorkLocation location,
                                   SlamNode::WorkInterruptionKind kind,
                                   std::optional<ImuBatchStatus> imu_status = std::nullopt)
    {
        const auto queue = f.node.queue_snapshot();
        require(queue.enqueued == queue.pending + queue.in_flight + queue.processed +
                queue.startup_discarded + queue.overload_discarded,
                "interruption broke queue accounting");
        const auto check = [&](const std::optional<SlamNode::TrackingWork> &work,
                               const std::optional<SlamNode::ReservationSummary> &summary) {
            require(work.has_value() == summary.has_value(), "summary lost its work");
            if (!work) return;
            require(work->interruption && summary->interruption &&
                    work->interruption->kind == kind && summary->interruption->kind == kind &&
                    work->interruption->reason == reason && summary->interruption->reason == reason &&
                    work->interruption->location == location && summary->interruption->location == location &&
                    work->interruption->imu_status == imu_status && summary->interruption->imu_status == imu_status &&
                    work->stage == summary->stage && work->batch_use == summary->batch_use &&
                    work->pending.enqueue_sequence == summary->enqueue_sequence &&
                    work->pending.frame.timestamp_ns == summary->timestamp_ns &&
                    work->pending.received_at == summary->received_at,
                    "work and summary disagree on interruption or identity");
        };
        check(f.node.tracking_work_, queue.reservation);
        check(f.node.startup_next_work_, queue.startup_next_reservation);
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
            if (current == point) f.node.stop_requested_ = true;
        };
        f.node.process_pending_frames();
        const auto stopped = f.node.queue_snapshot();
        const auto batch = normal ? stopped.reservation : stopped.startup_next_reservation;
        const bool waiting = point == SlamNode::TestWorkPoint::AfterWaiting;
        require(batch && batch->batch_use == (waiting ? SlamNode::ImuBatchUse::NotAcquired :
                                                       SlamNode::ImuBatchUse::ConsumedUnused) &&
                batch->stage == (waiting ? SlamNode::TrackingWorkStage::Reserved :
                                         SlamNode::TrackingWorkStage::Ready) &&
                f.calls.size() == (normal ? 2U : 0U),
                "stop checkpoint called backend or lost batch state");
        if (normal && !waiting)
        {
            const auto final = f.node.final_snapshot();
            require(std::string(final.coverage.source) == "saved_batch" &&
                    std::string(final.coverage.status) == "ConsumedUnused" &&
                    final.coverage.interval->left == stamp(2) && final.coverage.interval->right == stamp(3),
                    "normal unused batch coverage lost its original interval");
        }
        const auto location = waiting ? SlamNode::WorkLocation::Coordination :
                                        SlamNode::WorkLocation::BeforeBackend;
        check_interruption(f, SlamNode::WorkReason::StopObserved, location,
                           SlamNode::WorkInterruptionKind::Stopped);
        const auto consumed = ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_);
        f.node.observe_work_stop(SlamNode::WorkLocation::Finalization);
        f.node.observe_work_stop(SlamNode::WorkLocation::Finalization);
        f.node.process_pending_frames();
        check_interruption(f, SlamNode::WorkReason::StopObserved, location,
                           SlamNode::WorkInterruptionKind::Stopped);
        // Even a mistaken subsequent retry with the stop flag cleared cannot consume again.
        f.node.stop_requested_ = false;
        bool rejected = false;
        try { f.node.process_pending_frames(); }
        catch (const std::logic_error &) { rejected = true; }
        require(rejected && f.node.queue_snapshot().processed == stopped.processed &&
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
        const auto before = f.node.queue_snapshot();
        f.node.stop_requested_ = true;
        // Exercise the exact work finalization operation used by shutdown without a backend.
        f.node.observe_work_stop(SlamNode::WorkLocation::Finalization);
        f.node.observe_work_stop(SlamNode::WorkLocation::Finalization);
        check_interruption(f, SlamNode::WorkReason::StopObserved, SlamNode::WorkLocation::Finalization,
                           SlamNode::WorkInterruptionKind::Stopped);
        const auto after = f.node.queue_snapshot();
        require(after.pending == before.pending && after.in_flight == before.in_flight &&
                after.processed == before.processed && after.first->enqueue_sequence == 3 &&
                after.reservation->batch_use == SlamNode::ImuBatchUse::NotRequired &&
                after.startup_next_reservation->batch_use == SlamNode::ImuBatchUse::NotAcquired,
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
        try { f.node.process_pending_frames(); }
        catch (...) { failed = true; }
        require(failed && f.calls.size() == (normal ? 2U : 0U),
                "pre-backend failure entered backend");
        check_interruption(f, SlamNode::WorkReason::BeforeBackendException,
                           SlamNode::WorkLocation::BeforeBackend, SlamNode::WorkInterruptionKind::Failed);
        const auto queue = f.node.queue_snapshot();
        const auto batch = normal ? queue.reservation : queue.startup_next_reservation;
        require(batch->batch_use == SlamNode::ImuBatchUse::ConsumedUnused &&
                batch->stage == SlamNode::TrackingWorkStage::Ready && batch->interruption->exception,
                "pre-backend failure falsely marked batch delivered");
        bool original = false;
        try { std::rethrow_exception(batch->interruption->exception); }
        catch (int value) { original = unknown && value == 42; }
        catch (const LogFailure &) { original = !unknown; }
        require(original, "exception identity was lost");
    }

    static void test_stop_in_normal_backend()
    {
        Fixture f;
        for (int i = 1; i <= 3; ++i) { f.enqueue(i); f.receive(i); }
        f.node.process_pending_frames();
        f.on_track = [&]() { f.node.stop_requested_ = true; };
        f.node.process_pending_frames();
        const auto queue = f.node.queue_snapshot();
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
        check_interruption(f, SlamNode::WorkReason::ImuQueryResult, SlamNode::WorkLocation::ImuQuery,
                           SlamNode::WorkInterruptionKind::Failed, status);
        const auto queue = f.node.queue_snapshot();
        const auto batch = normal ? queue.reservation : queue.startup_next_reservation;
        require(batch->batch_use == SlamNode::ImuBatchUse::NotAcquired &&
                batch->stage == SlamNode::TrackingWorkStage::Reserved,
                "failed query falsely consumed a batch");
        bool rejected = false;
        try { f.node.process_pending_frames(); }
        catch (const std::logic_error &) { rejected = true; }
        require(rejected && ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == consumed,
                "failed query was retried");
    }

    static void test_coordination_failure(SlamNode::WorkReason reason)
    {
        Fixture f;
        f.enqueue(1);
        f.enqueue(2);
        f.receive(1);
        f.node.process_pending_frames();  // Retain a real Waiting pair.
        if (reason == SlamNode::WorkReason::ImuBackwards)
            f.receive(0);
        else
        {
            const auto expired = SlamNode::Clock::now() - std::chrono::seconds(7200);
            const std::lock_guard<std::mutex> lock(f.node.queue_mutex_);
            if (reason == SlamNode::WorkReason::StartupTimeout)
                f.node.startup_wait_started_ = expired;
            else
            {
                f.node.tracking_work_->pending.received_at = expired;
                f.node.reservation_->received_at = expired;
            }
        }
        bool failed = false;
        try { f.node.process_pending_frames(); }
        catch (const std::runtime_error &) { failed = true; }
        require(failed && f.calls.empty(), "coordination failure entered backend");
        check_interruption(f, reason, SlamNode::WorkLocation::Coordination,
                           SlamNode::WorkInterruptionKind::Failed);
        f.node.stop_requested_ = true;
        f.node.observe_work_stop(SlamNode::WorkLocation::Finalization);
        check_interruption(f, reason, SlamNode::WorkLocation::Coordination,
                           SlamNode::WorkInterruptionKind::Failed);
    }

    static void test_return_probe_failure()
    {
        Fixture f;
        f.enqueue(1);
        f.enqueue(2);
        f.receive(1);
        f.receive(2);
        f.node.test_work_point_ = [](SlamNode::TestWorkPoint point) {
            if (point == SlamNode::TestWorkPoint::AfterBackendReturn) throw LogFailure();
        };
        bool failed = false;
        try { f.node.process_pending_frames(); }
        catch (const LogFailure &) { failed = true; }
        const auto queue = f.node.queue_snapshot();
        require(failed && f.calls.size() == 1 && queue.processed == 1 && !queue.reservation &&
                f.node.last_tracked_frame_timestamp_ == stamp(1) && queue.startup_next_reservation &&
                queue.startup_next_reservation->batch_use == SlamNode::ImuBatchUse::ConsumedUnused &&
                queue.startup_next_reservation->interruption->related_sequence == 1,
                "return probe failure prevented completion or lost F1");
        check_interruption(f, SlamNode::WorkReason::AfterBackendException,
                           SlamNode::WorkLocation::AfterBackend, SlamNode::WorkInterruptionKind::Failed);
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
        const auto queue = f.node.queue_snapshot();
        require(failed && queue.startup_discarded == 1 && queue.in_flight == 1 && queue.pending == 1 &&
                queue.reservation->enqueue_sequence == 2 &&
                queue.reservation->batch_use == SlamNode::ImuBatchUse::NotRequired &&
                queue.reservation->interruption && queue.reservation->interruption->related_sequence == 1,
                "discard logging failure lost promotion or predecessor identity");
        check_interruption(f, SlamNode::WorkReason::StartupDiscardException,
                           SlamNode::WorkLocation::StartupDiscard, SlamNode::WorkInterruptionKind::Failed,
                           ImuBatchStatus::MissingHistory);
        bool rejected = false;
        try { f.node.process_pending_frames(); }
        catch (const std::logic_error &) { rejected = true; }
        require(rejected && f.node.queue_snapshot().startup_discarded == 1 && f.calls.empty(),
                "discard logging failure allowed a second discard");
    }

    static void test_final_saved_batch(int failing_frame)
    {
        Fixture f;
        for (int i = 1; i <= 3; ++i) { f.enqueue(i); f.receive(i); }
        f.on_track = [&]() {
            if (failing_frame == 0) f.node.stop_requested_ = true;
            else if (f.calls.back().timestamp == stamp(failing_frame)) throw BackendFailure();
        };
        try { f.node.process_pending_frames(); }
        catch (const BackendFailure &) {}
        f.node.stop_requested_ = true;
        f.node.observe_work_stop(SlamNode::WorkLocation::Finalization);
        const auto consumed = ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_);
        const auto final = f.node.final_snapshot();
        require(final.accounting_valid == true && final.identities_valid &&
                final.work.size() == final.outstanding && final.queued == 1 &&
                std::string(final.coverage.source) == "saved_batch" &&
                std::string(final.coverage.status) == (failing_frame == 2 ? "DeliveredToBackend" : "ConsumedUnused") &&
                final.coverage.interval && final.coverage.interval->left == stamp(1) &&
                final.coverage.interval->right == stamp(2) && final.coverage.batch_samples == 1 &&
                ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == consumed,
                "final coverage reinterpreted or consumed the saved batch");
        // Holding a pixel reference lets us verify that the metadata snapshot owns none.
        cv::Mat retained = f.node.startup_next_work_->pending.frame.left;
        std::ostringstream logs;
        f.node.node_logger_ = std::make_shared<spdlog::logger>(
            "final_test", std::make_shared<spdlog::sinks::ostream_sink_mt>(logs));
        require(!f.node.tracking_diagnostics_enabled_, "fixture unexpectedly enabled timing");
        f.node.report_final_work(final);
        require(logs.str().find("STOP_ACCOUNTING") != std::string::npos &&
                logs.str().find("STOP_WORK enqueue_sequence=2") != std::string::npos &&
                logs.str().find("source=saved_batch") != std::string::npos,
                "basic stop evidence depends on timing diagnostics");
        f.node.release_unfinished_work();
        f.node.release_unfinished_work();
        require(!f.node.tracking_work_ && !f.node.startup_next_work_ &&
                !f.node.reservation_ && !f.node.startup_next_reservation_ &&
                f.node.pending_frames_.empty() && retained.u->refcount == 1 &&
                final.work.size() == final.outstanding,
                "release retained payloads or invalidated final metadata");
        for (const auto &work : final.work)
            require(!work.identity.interruption || !work.identity.interruption->exception,
                    "final snapshot retained an exception payload");
    }

    static void test_final_inspection()
    {
        Fixture f;
        f.node.trace_ = std::make_unique<DiagnosticTrace>("", 128);
        ImuFrontendTestAccess::attach_trace(*f.node.imu_frontend_, f.node.trace_.get());
        require(std::string(f.node.final_snapshot().coverage.status) == "NoOutstanding",
                "empty final snapshot invented an interval");
        f.enqueue(1);
        require(std::string(f.node.final_snapshot().coverage.status) == "AwaitingSecondFrame",
                "single startup frame invented an interval");
        f.enqueue(2);
        f.receive(1);
        auto final = f.node.final_snapshot();
        require(std::string(final.coverage.status) == "WaitingForData" && final.work.size() == 2,
                "queued startup coverage was not inspected");
        f.node.process_pending_frames();
        final = f.node.final_snapshot();
        require(final.queued == 0 && final.in_flight == 2 &&
                std::string(final.coverage.status) == "WaitingForData",
                "reservation-only final coverage disappeared");
        f.receive(2);
        const auto trace_before = f.node.trace_->stats();
        final = f.node.final_snapshot();
        require(f.node.trace_->stats().recorded == trace_before.recorded &&
                f.node.trace_->stats().dropped == trace_before.dropped,
                "final IMU inspection added trace events");
        require(std::string(final.coverage.status) == "Ready" &&
                !ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_),
                "final inspection consumed startup data");
        f.node.process_pending_frames();
        f.enqueue(3);
        final = f.node.final_snapshot();
        require(final.coverage.interval->left == stamp(2) && final.coverage.interval->right == stamp(3),
                "normal queued coverage used the wrong boundary");
        f.node.process_pending_frames();
        final = f.node.final_snapshot();
        require(final.queued == 0 && final.in_flight == 1 &&
                final.coverage.enqueue_sequence == 3 &&
                ImuFrontendTestAccess::consumed_until(*f.node.imu_frontend_) == stamp(2),
                "normal reservation inspection changed consumption");
    }

    static void test_final_promoted_and_failure()
    {
        Fixture f;
        f.enqueue(1); f.enqueue(2);
        f.receive(2); f.receive(3);
        f.node.process_pending_frames();
        require(std::string(f.node.final_snapshot().coverage.status) == "AwaitingSecondFrame",
                "promoted F0 without replacement invented coverage");
        f.enqueue(3);
        auto final = f.node.final_snapshot();
        require(final.coverage.interval->left == stamp(2) && final.coverage.interval->right == stamp(3) &&
                std::string(final.coverage.status) == "Ready", "promoted F0 ignored queued replacement");
        Fixture failed;
        failed.enqueue(1); failed.enqueue(2);
        failed.receive(1); failed.receive(3);
        try { failed.node.process_pending_frames(); } catch (const std::runtime_error &) {}
        final = failed.node.final_snapshot();
        require(final.work[0].identity.interruption &&
                final.work[0].identity.interruption->imu_status == ImuBatchStatus::DataGap &&
                std::string(final.coverage.source) == "inspection" &&
                !final.work[0].exception_message.empty(), "final snapshot lost original query failure");
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
            const auto queue = f.node.queue_snapshot();
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
            const auto queue = f.node.queue_snapshot();
            require(calls == expected && queue.processed == static_cast<uint64_t>(expected) &&
                    queue.in_flight == 0 && queue.pending == static_cast<std::size_t>(3 - expected),
                    "Stereo retry did not commit exactly one frame");
        }
        f.node.process_pending_frames();
        const auto final = f.node.final_snapshot();
        require(calls == 3 && final.processed == 3 && final.enqueued == 3 &&
                final.accounting_valid && *final.accounting_valid && final.work.empty(),
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
            require(empty.accounting_valid == true && empty.work.empty(),
                    "empty Stereo accounting is invalid");
            StereoFrame frame;
            frame.timestamp = -1;
            f.node.on_frame(frame);
            if (stage == 2)
            {
                f.node.test_work_point_ = [&](SlamNode::TestWorkPoint point) {
                    if (point == SlamNode::TestWorkPoint::AfterReady) f.node.stop_requested_ = true;
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
                const auto full = f.node.queue_snapshot();
                require(rejected && full.enqueued == 1 && full.in_flight == 1 &&
                        full.overload_discarded == 0 && full.reservation->timestamp == -1,
                        "capacity-one admission replaced reserved work");
                f.node.stop_requested_ = true;
                f.node.process_pending_frames();
            }
            else
            {
                f.node.stop_requested_ = true;
                f.node.process_pending_frames();
            }
            const auto final = f.node.final_snapshot();
            require(final.queued == (stage == 0 ? 1U : 0U) &&
                    final.in_flight == (stage == 0 ? 0U : 1U),
                    "Stereo stop changed queue ownership");
            require(calls == 0 && final.accounting_valid == true && final.processed == 0 &&
                    final.outstanding == 1 && final.work.size() == 1 &&
                    final.work[0].identity.timestamp == -1 &&
                    final.work[0].identity.batch_use == SlamNode::ImuBatchUse::NotRequired,
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
                if (outcome == Outcome::Stop) f.node.stop_requested_ = true;
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
            const auto queue = f.node.queue_snapshot();
            const bool backend_failed = outcome == Outcome::BackendFailure;
            require(calls == 1 && failed == (outcome != Outcome::Stop) && queue.pending == 1 &&
                    queue.processed == (backend_failed ? 0U : 1U) &&
                    queue.in_flight == (backend_failed ? 1U : 0U),
                    "Stereo outcome lost completion or drained queued work");
            if (backend_failed)
            {
                require(queue.reservation && queue.reservation->interruption &&
                        queue.reservation->interruption->reason == SlamNode::WorkReason::BackendException &&
                        queue.reservation->interruption->related_sequence == 1,
                        "Stereo backend failure lost its cause");
                bool rejected = false;
                try { f.node.process_pending_frames(); }
                catch (const std::logic_error &) { rejected = true; }
                require(rejected && calls == 1, "Stereo failed backend was retried");
            }
            // Model the caller stopping after an exception; B5 owns the real callback boundary.
            f.node.stop_requested_ = true;
            f.node.process_pending_frames();
            const auto final = f.node.final_snapshot();
            require(calls == 1 && final.accounting_valid == true && final.identities_valid &&
                    final.work.size() == (backend_failed ? 2U : 1U) &&
                    final.work.back().identity.enqueue_sequence == 2 &&
                    final.work.front().identity.enqueue_sequence == (backend_failed ? 1U : 2U),
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
            auto q = f.node.queue_snapshot();
            require(rejected && q.enqueued == 3 && q.processed == 0 && q.in_flight == 2 &&
                    q.first->enqueue_sequence == 3, "full startup admission corrupted work");
            release(1);
            wait_entry(2);
            q = f.node.queue_snapshot();
            require(q.processed == 1 && q.in_flight == 1 && !q.reservation &&
                    q.startup_next_reservation->enqueue_sequence == 2,
                    "F0 completion did not release one capacity slot");
            f.enqueue(4);
            require(f.node.queue_snapshot().second->enqueue_sequence == 4,
                    "rejected admission consumed a sequence");
            release(2);
            wait_entry(3);
            q = f.node.queue_snapshot();
            require(q.processed == 2 && q.startup_complete && q.reservation->enqueue_sequence == 3,
                    "F1 completion or normal FIFO reservation failed");
            f.enqueue(5);
            rejected = false;
            try { f.enqueue(6); } catch (const std::runtime_error &) { rejected = true; }
            require(rejected && f.node.queue_snapshot().enqueued == 5,
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
        for (const auto &work : final.work) sequences.push_back(work.identity.enqueue_sequence);
        require(final.accounting_valid == true && final.identities_valid &&
                sequences == (fail_normal ? std::vector<uint64_t>{3, 4, 5} :
                                           std::vector<uint64_t>{4, 5}),
                "final work set differs from accepted minus completed work");
        if (fail_normal)
            require(final.work.front().identity.interruption->reason == SlamNode::WorkReason::BackendException &&
                    std::string(final.coverage.source) == "saved_batch" &&
                    std::string(final.coverage.status) == "DeliveredToBackend" &&
                    final.coverage.interval->left == stamp(2) && final.coverage.interval->right == stamp(3),
                    "normal failed batch was reinspected or its cause lost");
    }

    static void run()
    {
        test_concurrent_completion(false);
        test_concurrent_completion(true);
        for (int failure = 0; failure <= 2; ++failure) test_final_saved_batch(failure);
        test_final_inspection();
        test_final_promoted_and_failure();
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
        for (const auto reason : {SlamNode::WorkReason::StartupTimeout, SlamNode::WorkReason::FrameTimeout,
                                 SlamNode::WorkReason::ImuBackwards})
            test_coordination_failure(reason);
        test_return_probe_failure();
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
