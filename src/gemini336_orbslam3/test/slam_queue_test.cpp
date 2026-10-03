// Compile the existing node queue directly; do not duplicate its implementation.
#include "slam_snapshot_test_access.hpp"
#include <spdlog/sinks/null_sink.h>

#include <atomic>
#include <iostream>
#include <thread>
#include <spdlog/sinks/ostream_sink.h>

namespace gemini336_orbslam3
{
struct SlamNodeQueueTestAccess
{
    static void require(bool condition, const char *message)
    {
        if (!condition) throw std::runtime_error(message);
    }

    static void test_reservation()
    {
        SlamNode node(SlamNode::QueueTestTag{});
        node.coordinator_->pending_frames_capacity_ = 2;
        require(!node.coordinator_->reserve_tracking_work(), "empty queue produced a reservation");

        for (int i = 1; i <= 2; ++i)
        {
            StereoFrame frame;
            frame.timestamp = i;
            frame.timestamp_ns = int64_t(i) * 1000000000LL;
            frame.left = cv::Mat(2, 2, CV_8UC1, cv::Scalar(i));
            frame.right = frame.left.clone();
            node.coordinator_->enqueue_frame(frame);
        }

        const auto before = SlamSnapshotTestAccess::snapshot(node);
        require(node.coordinator_->reserve_tracking_work(), "failed to reserve first frame");
        const auto reserved = SlamSnapshotTestAccess::snapshot(node);
        require(reserved.pending == 1 && reserved.in_flight == 1 &&
                reserved.outstanding == 2 && reserved.enqueued == 2 &&
                reserved.processed == 0 && reserved.startup_discarded == 0,
                "reservation changed outstanding accounting");
        require(reserved.reservation && reserved.reservation->timestamp == 1 &&
                reserved.reservation->enqueue_sequence == 1 &&
                reserved.reservation->timestamp_ns == 1000000000LL &&
                reserved.first && reserved.first->frame.timestamp == 2 &&
                !reserved.startup_next_reservation,
                "reservation did not preserve FIFO");
        require(reserved.reservation->stage == TrackingCoordinator::TrackingWorkStage::Reserved &&
                node.coordinator_->tracking_work_ && !node.coordinator_->tracking_work_->imu_batch,
                "reservation unexpectedly acquired an IMU batch");
        require(reserved.oldest_received_at == before.oldest_received_at &&
                reserved.startup_started == before.startup_started &&
                reserved.reservation->received_at == before.first->received_at,
                "reservation reset a waiting deadline");
        require(node.coordinator_->tracking_work_->pending.frame.left.at<unsigned char>(0, 0) == 1,
                "reserved pixels did not survive queue removal");

        // Retrying an existing reservation must not remove the next queued frame.
        require(node.coordinator_->reserve_tracking_work(), "failed to retain existing reservation");
        const auto retried = SlamSnapshotTestAccess::snapshot(node);
        require(retried.pending == 1 && retried.in_flight == 1 &&
                retried.outstanding == 2 && retried.enqueued == 2 &&
                retried.reservation && retried.reservation->timestamp == 1 &&
                retried.first && retried.first->frame.timestamp == 2 &&
                retried.oldest_received_at == before.oldest_received_at,
                "retry changed the reserved frame or queue");

        StereoFrame incoming;
        incoming.timestamp = 3;
        incoming.timestamp_ns = int64_t(3) * 1000000000LL;
        bool full_rejected = false;
        try { node.coordinator_->enqueue_frame(incoming); }
        catch (const std::runtime_error &) { full_rejected = true; }
        const auto after = SlamSnapshotTestAccess::snapshot(node);
        require(full_rejected && after.enqueued == 2 && after.pending == 1 &&
                after.in_flight == 1 && after.outstanding == 2 &&
                after.reservation && after.reservation->timestamp == 1 &&
                after.startup_started == before.startup_started,
                "admission ignored reserved capacity or changed rejected state");
    }

    static void test_reserved_frame_with_empty_queue()
    {
        SlamNode node(SlamNode::QueueTestTag{});
        StereoFrame frame;
        frame.timestamp = 1;
        frame.timestamp_ns = int64_t(1) * 1000000000LL;
        frame.left = cv::Mat(2, 2, CV_8UC1, cv::Scalar(7));
        frame.right = frame.left.clone();
        node.coordinator_->enqueue_frame(frame);
        const auto before = SlamSnapshotTestAccess::snapshot(node);

        // The sole frame leaves the queue but remains outstanding across retries.
        require(node.coordinator_->reserve_tracking_work(), "failed to reserve sole frame");
        require(node.coordinator_->reserve_tracking_work(), "empty queue hid an existing reservation");
        const auto snapshot = SlamSnapshotTestAccess::snapshot(node);
        require(snapshot.pending == 0 && !snapshot.first && !snapshot.second &&
                snapshot.in_flight == 1 && snapshot.outstanding == 1 &&
                snapshot.enqueued == 1 && snapshot.processed == 0 &&
                snapshot.reservation && snapshot.reservation->timestamp == 1 &&
                snapshot.oldest_received_at == before.oldest_received_at,
                "empty queue lost reserved work or its deadline");
        require(node.coordinator_->tracking_work_ &&
                node.coordinator_->tracking_work_->pending.frame.right.at<unsigned char>(0, 0) == 7,
                "sole reserved frame lost its pixels");
    }

    static void test_startup_pair()
    {
        SlamNode node(SlamNode::QueueTestTag{});
        node.node_logger_ = std::make_shared<spdlog::logger>(
            "queue_test", std::make_shared<spdlog::sinks::null_sink_mt>());
        node.coordinator_->node_logger_ = node.node_logger_;
        node.coordinator_->pending_frames_capacity_ = 3;
        for (int i = 1; i <= 3; ++i)
        {
            StereoFrame frame;
            frame.timestamp = i;
            frame.timestamp_ns = int64_t(i) * 1000000000LL;
            frame.left = cv::Mat(2, 2, CV_8UC1, cv::Scalar(i));
            frame.right = frame.left.clone();
            node.coordinator_->enqueue_frame(frame);
        }

        const auto initial = SlamSnapshotTestAccess::snapshot(node);
        require(initial.first && initial.second && initial.startup_started,
                "startup test did not retain initial frames and deadline");
        const SlamNode::Clock::time_point second_received_at = initial.second->received_at;

        // Every transition must preserve admission accounting and the startup deadline.
        const auto check_accounting = [&](std::size_t pending, std::size_t in_flight,
                                          uint64_t discarded) {
            const auto snapshot = SlamSnapshotTestAccess::snapshot(node);
            require(snapshot.pending == pending && snapshot.in_flight == in_flight &&
                    snapshot.outstanding == pending + in_flight &&
                    snapshot.outstanding <= node.coordinator_->pending_frames_capacity_ &&
                    snapshot.enqueued == 3 && snapshot.processed == 0 &&
                    snapshot.startup_discarded == discarded &&
                    snapshot.enqueued == snapshot.pending + snapshot.in_flight +
                        snapshot.processed + snapshot.startup_discarded + snapshot.overload_discarded,
                    "startup transition broke accounting");
            require(snapshot.startup_started == initial.startup_started &&
                    !snapshot.startup_complete,
                    "startup transition reset its deadline or completed without backend");
            return snapshot;
        };
        check_accounting(3, 0, 0);

        require(node.coordinator_->reserve_startup_pair(), "failed to reserve startup pair");
        const auto reserved = check_accounting(1, 2, 0);
        require(reserved.reservation && reserved.startup_next_reservation &&
                reserved.reservation->timestamp == 1 &&
                reserved.reservation->enqueue_sequence == 1 &&
                reserved.reservation->timestamp_ns == 1000000000LL &&
                reserved.startup_next_reservation->timestamp == 2 &&
                reserved.first && reserved.first->frame.timestamp == 3 &&
                reserved.oldest_received_at == initial.oldest_received_at,
                "startup reservation did not preserve FIFO or oldest enqueue time");

        require(node.coordinator_->reserve_startup_pair(), "failed to retain startup pair on retry");
        const auto retried = check_accounting(1, 2, 0);
        require(retried.reservation && retried.startup_next_reservation &&
                retried.reservation->timestamp == 1 &&
                retried.startup_next_reservation->timestamp == 2 &&
                retried.first && retried.first->frame.timestamp == 3,
                "startup retry replaced candidates or removed another frame");

        // Exercise the MissingHistory transition directly, without querying IMU.
        node.coordinator_->discard_startup_first();
        const auto discarded = check_accounting(1, 1, 1);
        require(discarded.reservation && discarded.reservation->timestamp == 2 &&
                !discarded.startup_next_reservation &&
                discarded.first && discarded.first->frame.timestamp == 3 &&
                discarded.oldest_received_at == second_received_at &&
                discarded.reservation->received_at == second_received_at,
                "startup discard failed to promote F1 with its original enqueue time");
        require(node.coordinator_->tracking_work_ && !node.coordinator_->startup_next_work_ &&
                node.coordinator_->tracking_work_->pending.frame.timestamp == 2 &&
                node.coordinator_->tracking_work_->pending.frame.left.at<unsigned char>(0, 0) == 2,
                "startup discard lost the promoted frame payload");

        require(node.coordinator_->reserve_startup_pair(), "failed to refill startup F1");
        const auto refilled = check_accounting(0, 2, 1);
        require(refilled.reservation && refilled.startup_next_reservation &&
                refilled.reservation->timestamp == 2 &&
                refilled.startup_next_reservation->timestamp == 3 &&
                !refilled.first && !refilled.second &&
                refilled.oldest_received_at == second_received_at &&
                refilled.reservation->stage == TrackingCoordinator::TrackingWorkStage::Reserved &&
                refilled.startup_next_reservation->stage == TrackingCoordinator::TrackingWorkStage::Reserved &&
                !node.coordinator_->tracking_work_->imu_batch && !node.coordinator_->startup_next_work_->imu_batch,
                "startup refill changed candidate order, deadline or batch state");
        require(node.coordinator_->startup_next_work_ &&
                node.coordinator_->startup_next_work_->pending.frame.right.at<unsigned char>(0, 0) == 3 &&
                !node.coordinator_->tracking_work_->imu_batch && !node.coordinator_->startup_next_work_->imu_batch,
                "startup refill lost pixels or unexpectedly acquired IMU data");

        require(node.coordinator_->reserve_startup_pair(), "empty queue hid the retained startup pair");
        check_accounting(0, 2, 1);
    }

    static void test_startup_shortage()
    {
        SlamNode node(SlamNode::QueueTestTag{});
        node.node_logger_ = std::make_shared<spdlog::logger>(
            "queue_test", std::make_shared<spdlog::sinks::null_sink_mt>());
        node.coordinator_->node_logger_ = node.node_logger_;
        node.coordinator_->pending_frames_capacity_ = 2;
        require(!node.coordinator_->reserve_startup_pair(), "empty queue produced a startup pair");
        const auto empty = SlamSnapshotTestAccess::snapshot(node);
        require(empty.pending == 0 && empty.in_flight == 0 && empty.enqueued == 0 &&
                !empty.startup_started && !node.coordinator_->tracking_work_ && !node.coordinator_->startup_next_work_,
                "empty startup attempt changed state");

        StereoFrame first;
        first.timestamp = 1;
        first.timestamp_ns = int64_t(1) * 1000000000LL;
        node.coordinator_->enqueue_frame(first);
        const auto initial = SlamSnapshotTestAccess::snapshot(node);
        require(initial.startup_started && initial.first,
                "first frame did not establish the startup deadline");

        // A single initial candidate must remain queued across repeated attempts.
        for (int retry = 0; retry < 3; ++retry)
        {
            require(!node.coordinator_->reserve_startup_pair(), "startup accepted only one candidate");
            const auto waiting = SlamSnapshotTestAccess::snapshot(node);
            require(waiting.pending == 1 && waiting.in_flight == 0 &&
                    waiting.outstanding == 1 && waiting.enqueued == 1 &&
                    waiting.processed == 0 && waiting.startup_discarded == 0 &&
                    waiting.first && waiting.first->frame.timestamp == 1 &&
                    waiting.first->received_at == initial.first->received_at &&
                    !waiting.reservation && !waiting.startup_next_reservation &&
                    !node.coordinator_->tracking_work_ && !node.coordinator_->startup_next_work_ &&
                    waiting.startup_started == initial.startup_started &&
                    !waiting.startup_complete,
                    "insufficient startup candidates changed ownership or deadline");
        }

        StereoFrame second;
        second.timestamp = 2;
        second.timestamp_ns = int64_t(2) * 1000000000LL;
        second.left = cv::Mat(2, 2, CV_8UC1, cv::Scalar(2));
        node.coordinator_->enqueue_frame(second);
        require(node.coordinator_->reserve_startup_pair(), "second frame did not enable startup");
        node.coordinator_->discard_startup_first();
        const auto promoted = SlamSnapshotTestAccess::snapshot(node);
        require(promoted.reservation && promoted.reservation->timestamp == 2,
                "startup discard did not preserve the second frame");

        // No replacement exists: retry must retain the promoted F0, not discard it.
        for (int retry = 0; retry < 3; ++retry)
        {
            require(!node.coordinator_->reserve_startup_pair(), "startup refill succeeded without a frame");
            const auto waiting = SlamSnapshotTestAccess::snapshot(node);
            require(waiting.pending == 0 && waiting.in_flight == 1 &&
                    waiting.outstanding == 1 && waiting.enqueued == 2 &&
                    waiting.processed == 0 && waiting.startup_discarded == 1 &&
                    waiting.reservation && waiting.reservation->timestamp == 2 &&
                    waiting.reservation->received_at == promoted.reservation->received_at &&
                    waiting.oldest_received_at == promoted.oldest_received_at &&
                    !waiting.startup_next_reservation &&
                    waiting.startup_started == initial.startup_started &&
                    !waiting.startup_complete &&
                    node.coordinator_->tracking_work_ && !node.coordinator_->startup_next_work_ &&
                    node.coordinator_->tracking_work_->pending.frame.left.at<unsigned char>(0, 0) == 2,
                    "missing replacement changed reserved F0 or deadline");
        }

        StereoFrame third;
        third.timestamp = 3;
        third.timestamp_ns = int64_t(3) * 1000000000LL;
        node.coordinator_->enqueue_frame(third);
        require(node.coordinator_->reserve_startup_pair(), "new frame did not refill retained startup F0");
        const auto refilled = SlamSnapshotTestAccess::snapshot(node);
        require(refilled.pending == 0 && refilled.in_flight == 2 &&
                refilled.outstanding == 2 && refilled.enqueued == 3 &&
                refilled.processed == 0 && refilled.startup_discarded == 1 &&
                refilled.reservation && refilled.reservation->timestamp == 2 &&
                refilled.startup_next_reservation &&
                refilled.startup_next_reservation->timestamp == 3 &&
                refilled.startup_started == initial.startup_started &&
                refilled.oldest_received_at == promoted.oldest_received_at &&
                !refilled.startup_complete,
                "refill after shortage broke accounting, FIFO or deadline");
    }

    static void test_repeated_startup_discard()
    {
        SlamNode node(SlamNode::QueueTestTag{});
        node.node_logger_ = std::make_shared<spdlog::logger>(
            "queue_test", std::make_shared<spdlog::sinks::null_sink_mt>());
        node.coordinator_->node_logger_ = node.node_logger_;
        node.coordinator_->pending_frames_capacity_ = 4;
        for (int i = 1; i <= 4; ++i)
        {
            StereoFrame frame;
            frame.timestamp = i;
            frame.timestamp_ns = int64_t(i) * 1000000000LL;
            node.coordinator_->enqueue_frame(frame);
        }
        const auto initial = SlamSnapshotTestAccess::snapshot(node);

        // Model consecutive MissingHistory results through the real discard operation.
        // Each call discards only F0; the next attempt, not discard itself, refills F1.
        for (uint64_t candidate = 1; candidate <= 3; ++candidate)
        {
            require(node.coordinator_->reserve_startup_pair(), "failed to reserve consecutive startup pair");
            const auto pair = SlamSnapshotTestAccess::snapshot(node);
            require(pair.pending == 3 - candidate && pair.in_flight == 2 &&
                    pair.outstanding == pair.pending + pair.in_flight &&
                    pair.enqueued == 4 && pair.processed == 0 &&
                    pair.startup_discarded == candidate - 1 &&
                    pair.enqueued == pair.outstanding + pair.startup_discarded + pair.overload_discarded &&
                    pair.reservation && pair.reservation->timestamp == candidate &&
                    pair.reservation->enqueue_sequence == candidate &&
                    pair.reservation->timestamp_ns == int64_t(candidate) * 1000000000LL &&
                    pair.startup_next_reservation &&
                    pair.startup_next_reservation->timestamp == candidate + 1 &&
                    pair.startup_next_reservation->enqueue_sequence == candidate + 1 &&
                    pair.startup_started == initial.startup_started &&
                    !pair.startup_complete,
                    "consecutive startup reservation broke accounting or FIFO");

            node.coordinator_->discard_startup_first();
            const auto discarded = SlamSnapshotTestAccess::snapshot(node);
            require(discarded.pending == pair.pending && discarded.in_flight == 1 &&
                    discarded.outstanding == discarded.pending + discarded.in_flight &&
                    discarded.enqueued == 4 && discarded.processed == 0 &&
                    discarded.startup_discarded == candidate &&
                    discarded.enqueued == discarded.outstanding + discarded.startup_discarded + discarded.overload_discarded &&
                    discarded.reservation &&
                    discarded.reservation->timestamp == candidate + 1 &&
                    discarded.reservation->received_at ==
                        pair.startup_next_reservation->received_at &&
                    discarded.oldest_received_at == discarded.reservation->received_at &&
                    !discarded.startup_next_reservation &&
                    node.coordinator_->tracking_work_ &&
                    node.coordinator_->tracking_work_->pending.frame.timestamp == candidate + 1 &&
                    !node.coordinator_->startup_next_work_ &&
                    discarded.startup_started == initial.startup_started &&
                    !discarded.startup_complete,
                    "consecutive discard removed more than F0 or reset the deadline");
        }

        require(!node.coordinator_->reserve_startup_pair(), "exhausted startup queue unexpectedly refilled");
        const auto exhausted = SlamSnapshotTestAccess::snapshot(node);
        require(exhausted.pending == 0 && exhausted.in_flight == 1 &&
                exhausted.enqueued == 4 && exhausted.processed == 0 &&
                exhausted.startup_discarded == 3 && exhausted.reservation &&
                exhausted.reservation->timestamp == 4 &&
                exhausted.startup_started == initial.startup_started,
                "exhausted startup queue lost its final candidate");
    }

    static void test_overload_admission()
    {
        std::ostringstream logs;
        SlamNode node(SlamNode::QueueTestTag{});
        node.coordinator_->tracking_mode_ = TrackingMode::Stereo;
        node.coordinator_->pending_frames_capacity_ = 3;
        node.node_logger_ = std::make_shared<spdlog::logger>(
            "overload_test", std::make_shared<spdlog::sinks::ostream_sink_mt>(logs));
        node.coordinator_->node_logger_ = node.node_logger_;
        node.trace_ = std::make_unique<DiagnosticTrace>("", 32);
        node.coordinator_->trace_ = node.trace_.get();

        StereoFrame frame;
        frame.timestamp = 1;
        frame.timestamp_ns = int64_t(1) * 1000000000LL;
        frame.left = cv::Mat(2, 2, CV_8UC1, cv::Scalar(1));
        node.coordinator_->enqueue_frame(frame);
        require(node.coordinator_->reserve_tracking_work(), "failed to retain frame for overload test");
        for (int i = 2; i <= 3; ++i)
        {
            frame.timestamp = i;
            frame.timestamp_ns = int64_t(i) * 1000000000LL;
            node.coordinator_->enqueue_frame(frame);
        }
        const auto initial = SlamSnapshotTestAccess::snapshot(node);
        require(initial.pending == 2 && initial.in_flight == 1 &&
                initial.peak == 3 && initial.outstanding == 3,
                "capacity peak omitted the reserved frame");

        // Existing callers keep rejecting full queues unless replacement is explicit.
        frame.timestamp = 4;
        frame.timestamp_ns = int64_t(4) * 1000000000LL;
        bool rejected = false;
        try { node.coordinator_->enqueue_frame(frame); }
        catch (const std::runtime_error &) { rejected = true; }
        require(rejected && SlamSnapshotTestAccess::snapshot(node).enqueued == 3 &&
                SlamSnapshotTestAccess::snapshot(node).overload_discarded == 0 && logs.str().empty(),
                "default admission silently enabled overload discard");

        for (int i = 4; i <= 5; ++i)
        {
            frame.timestamp = i;
            frame.timestamp_ns = int64_t(i) * 1000000000LL;
            node.on_frame(frame);
            const auto snapshot = SlamSnapshotTestAccess::snapshot(node);
            require(snapshot.pending == 2 && snapshot.in_flight == 1 &&
                    snapshot.outstanding == 3 && snapshot.peak == 3 &&
                    snapshot.enqueued == static_cast<uint64_t>(i) &&
                    snapshot.overload_discarded == static_cast<uint64_t>(i - 3) &&
                    snapshot.startup_discarded == 0 && snapshot.processed == 0 &&
                    snapshot.enqueued == snapshot.pending + snapshot.in_flight +
                        snapshot.processed + snapshot.startup_discarded + snapshot.overload_discarded &&
                    snapshot.reservation && snapshot.reservation->timestamp == 1 &&
                    snapshot.first && snapshot.first->frame.timestamp == i - 1 &&
                    snapshot.second && snapshot.second->frame.timestamp == i &&
                    snapshot.second->enqueue_sequence == static_cast<uint64_t>(i) &&
                    node.coordinator_->tracking_work_ &&
                    node.coordinator_->tracking_work_->pending.frame.timestamp == 1 &&
                    snapshot.startup_started == initial.startup_started,
                    "overload replacement lost reservation, FIFO or accounting");
        }

        // Validate before eviction: duplicate/non-finite input must not discard anything.
        for (const double invalid : {5.0, 4.0, std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::infinity(),
                                     -std::numeric_limits<double>::infinity()})
        {
            frame.timestamp = invalid;
            rejected = false;
            try { node.on_frame(frame); }
            catch (const std::invalid_argument &) { rejected = true; }
            const auto unchanged = SlamSnapshotTestAccess::snapshot(node);
            require(rejected && unchanged.enqueued == 5 && unchanged.overload_discarded == 2 &&
                    unchanged.pending == 2 && unchanged.in_flight == 1 && unchanged.processed == 0 &&
                    unchanged.first && unchanged.first->enqueue_sequence == 4 &&
                    unchanged.first->frame.timestamp == 4 &&
                    unchanged.second && unchanged.second->enqueue_sequence == 5 &&
                    unchanged.second->frame.timestamp == 5 &&
                    unchanged.reservation && unchanged.reservation->enqueue_sequence == 1 &&
                    node.coordinator_->last_received_frame_timestamp_ == 5.0,
                    "invalid incoming timestamp changed queue identity or accounting");
        }

        node.coordinator_->tracking_mode_ = TrackingMode::StereoImu;
        frame.timestamp = 6;
        frame.timestamp_ns = int64_t(6) * 1000000000LL;
        rejected = false;
        try { node.coordinator_->enqueue_frame(frame, TrackingCoordinator::QueueFullPolicy::DiscardOldestQueued); }
        catch (const std::invalid_argument &) { rejected = true; }
        require(rejected && SlamSnapshotTestAccess::snapshot(node).enqueued == 5 &&
                SlamSnapshotTestAccess::snapshot(node).overload_discarded == 2,
                "Stereo-IMU allowed overload discard");
        rejected = false;
        try { node.coordinator_->enqueue_frame(frame); }
        catch (const std::runtime_error &) { rejected = true; }
        require(rejected && SlamSnapshotTestAccess::snapshot(node).outstanding == 3 &&
                SlamSnapshotTestAccess::snapshot(node).overload_discarded == 2,
                "Stereo-IMU full queue did not preserve failure policy");

        const std::string output = logs.str();
        const std::string marker = "Stereo overload discard:";
        const std::size_t first_log = output.find(marker);
        const std::size_t second_log = first_log == std::string::npos ?
            std::string::npos : output.find(marker, first_log + marker.size());
        require(first_log != std::string::npos && second_log != std::string::npos &&
                output.find(marker, second_log + marker.size()) == std::string::npos &&
                output.find("timestamp=2.000000000 incoming_timestamp=4.000000000") != std::string::npos &&
                output.find("timestamp=3.000000000 incoming_timestamp=5.000000000") != std::string::npos &&
                output.find("enqueue_sequence=2 timestamp_ns=2000000000 incoming_enqueue_sequence=4 incoming_timestamp_ns=4000000000") != std::string::npos &&
                output.find("enqueue_sequence=3 timestamp_ns=3000000000 incoming_enqueue_sequence=5 incoming_timestamp_ns=5000000000") != std::string::npos &&
                node.trace_->stats().recorded == 7 && node.trace_->stats().dropped == 0,
                "overload discard logs or trace accounting mismatch");

        // No queued victim exists when all capacity is reserved.
        SlamNode reserved_only(SlamNode::QueueTestTag{});
        reserved_only.coordinator_->pending_frames_capacity_ = 1;
        frame.timestamp = 1;
        frame.timestamp_ns = int64_t(1) * 1000000000LL;
        reserved_only.coordinator_->enqueue_frame(frame);
        require(reserved_only.coordinator_->reserve_tracking_work(), "failed to fill reserved-only capacity");
        frame.timestamp = 2;
        frame.timestamp_ns = int64_t(2) * 1000000000LL;
        rejected = false;
        try { reserved_only.coordinator_->enqueue_frame(frame, TrackingCoordinator::QueueFullPolicy::DiscardOldestQueued); }
        catch (const std::runtime_error &) { rejected = true; }
        const auto held = SlamSnapshotTestAccess::snapshot(reserved_only);
        require(rejected && held.pending == 0 && held.in_flight == 1 &&
                held.enqueued == 1 && held.overload_discarded == 0 &&
                held.reservation && held.reservation->timestamp == 1,
                "overload admission discarded a reservation without a queued victim");
    }

    static void test_concurrent_overload_snapshots()
    {
        std::ostringstream logs;
        SlamNode node(SlamNode::QueueTestTag{});
        node.coordinator_->pending_frames_capacity_ = 3;
        node.node_logger_ = std::make_shared<spdlog::logger>(
            "concurrent_overload_test", std::make_shared<spdlog::sinks::ostream_sink_mt>(logs));
        node.coordinator_->node_logger_ = node.node_logger_;
        StereoFrame frame;
        frame.timestamp = 1;
        frame.timestamp_ns = int64_t(1) * 1000000000LL;
        node.coordinator_->enqueue_frame(frame);
        require(node.coordinator_->reserve_tracking_work(), "failed to reserve concurrent overload frame");
        for (int i = 2; i <= 3; ++i)
        {
            frame.timestamp = i;
            frame.timestamp_ns = int64_t(i) * 1000000000LL;
            node.coordinator_->enqueue_frame(frame);
        }

        std::atomic<bool> start{false};
        std::exception_ptr producer_error;
        std::thread producer([&]() {
            while (!start.load()) std::this_thread::yield();
            try
            {
                for (int i = 4; i <= 200; ++i)
                {
                    StereoFrame incoming;
                    incoming.timestamp = i;
                    incoming.timestamp_ns = int64_t(i) * 1000000000LL;
                    node.coordinator_->enqueue_frame(incoming, TrackingCoordinator::QueueFullPolicy::DiscardOldestQueued);
                }
            }
            catch (...) { producer_error = std::current_exception(); }
        });
        start.store(true);
        bool consistent = true;
        for (int i = 0; i < 4000; ++i)
        {
            // Validate the production snapshot separately from test-only identities.
            const auto accounting = node.coordinator_->snapshot();
            if (accounting.pending != 2 || accounting.in_flight != 1 ||
                accounting.outstanding != 3 || accounting.peak != 3 ||
                accounting.enqueued != accounting.outstanding + accounting.processed +
                    accounting.startup_discarded + accounting.overload_discarded ||
                accounting.oldest_received_at != node.coordinator_->tracking_work_->pending.received_at)
                consistent = false;
            const auto snapshot = SlamSnapshotTestAccess::snapshot(node);
            if (snapshot.pending != 2 || snapshot.in_flight != 1 ||
                snapshot.outstanding != 3 || snapshot.peak != 3 ||
                snapshot.enqueued != snapshot.pending + snapshot.in_flight +
                    snapshot.processed + snapshot.startup_discarded + snapshot.overload_discarded ||
                !snapshot.reservation || snapshot.reservation->timestamp != 1 ||
                !snapshot.first || !snapshot.second ||
                snapshot.first->frame.timestamp + 1 != snapshot.second->frame.timestamp ||
                snapshot.second->frame.timestamp != snapshot.enqueued)
                consistent = false;
        }
        producer.join();
        if (producer_error) std::rethrow_exception(producer_error);
        require(consistent, "snapshot exposed a partial overload replacement");
        const auto final = SlamSnapshotTestAccess::snapshot(node);
        require(final.enqueued == 200 && final.overload_discarded == 197 &&
                final.first && final.first->frame.timestamp == 199 &&
                final.second && final.second->frame.timestamp == 200,
                "concurrent overload final accounting mismatch");
    }

    static void run()
    {
        test_overload_admission();
        test_concurrent_overload_snapshots();
        test_startup_shortage();
        test_repeated_startup_discard();
        test_startup_pair();
        test_reservation();
        test_reserved_frame_with_empty_queue();

        SlamNode node(SlamNode::QueueTestTag{});
        node.coordinator_->pending_frames_capacity_ = 30;
        node.trace_ = std::make_unique<DiagnosticTrace>("", 30);
        node.coordinator_->trace_ = node.trace_.get();
        std::atomic<bool> start{false};
        std::exception_ptr producer_error;
        std::thread producer([&]() {
            while (!start.load()) std::this_thread::yield();
            try
            {
                for (int i = 1; i <= 30; ++i)
                {
                    StereoFrame frame;
                    frame.timestamp = i;
                    frame.timestamp_ns = int64_t(i) * 1000000000LL;
                    frame.left = cv::Mat(2, 2, CV_8UC1, cv::Scalar(i));
                    frame.right = frame.left.clone();
                    node.coordinator_->enqueue_frame(frame);
                }
            }
            catch (...) { producer_error = std::current_exception(); }
        });
        start.store(true);
        bool consistent = true;
        std::size_t previous = 0;
        // Observe the actual snapshot while admission updates queue and accounting.
        for (int i = 0; i < 4000; ++i)
        {
            const auto accounting = node.coordinator_->snapshot();
            if (accounting.pending < previous || accounting.pending > 30 ||
                accounting.pending != accounting.enqueued || accounting.peak != accounting.pending ||
                accounting.in_flight != 0 || accounting.outstanding != accounting.pending ||
                accounting.oldest_received_at.has_value() != (accounting.pending > 0) ||
                (accounting.pending > 0 && accounting.startup_started != accounting.oldest_received_at))
                consistent = false;
            previous = accounting.pending;
            const auto snapshot = SlamSnapshotTestAccess::snapshot(node);
            if (snapshot.pending > 30 ||
                snapshot.pending != snapshot.enqueued || snapshot.peak != snapshot.pending ||
                snapshot.first.has_value() != (snapshot.pending > 0) ||
                snapshot.second.has_value() != (snapshot.pending >= 2) ||
                (snapshot.first && snapshot.startup_started != snapshot.first->received_at))
                consistent = false;
        }
        producer.join();
        if (producer_error) std::rethrow_exception(producer_error);
        require(consistent, "incoherent concurrent queue snapshot");
        const auto retained = SlamSnapshotTestAccess::snapshot(node);
        require(retained.pending == 30 && retained.first->frame.timestamp == 1 &&
                retained.second->frame.timestamp == 2, "FIFO snapshot mismatch");

        // Rejection must leave admission, accounting and the startup deadline unchanged.
        StereoFrame incoming;
        incoming.timestamp = 31;
        incoming.timestamp_ns = int64_t(31) * 1000000000LL;
        bool full_rejected = false;
        try { node.coordinator_->enqueue_frame(incoming); }
        catch (const std::runtime_error &) { full_rejected = true; }
        incoming.timestamp = 30;
        incoming.timestamp_ns = int64_t(30) * 1000000000LL;
        bool duplicate_rejected = false;
        try { node.coordinator_->enqueue_frame(incoming); }
        catch (const std::invalid_argument &) { duplicate_rejected = true; }
        const auto after = SlamSnapshotTestAccess::snapshot(node);
        require(full_rejected && duplicate_rejected && after.pending == retained.pending &&
                after.enqueued == retained.enqueued && after.startup_started == retained.startup_started,
                "rejection changed queue state");
        require(node.trace_->stats().recorded == 30, "rejected frame was traced as admitted");

        // The value snapshot keeps identity without retaining any image pixels.
        {
            const std::lock_guard<std::mutex> lock(node.coordinator_->queue_mutex_);
            node.coordinator_->pending_frames_.clear();
        }
        require(SlamSnapshotTestAccess::snapshot(node).pending == 0 &&
                retained.first->frame.timestamp_ns == 1000000000LL &&
                retained.second->frame.timestamp_ns == 2000000000LL,
                "snapshot identity changed after queue removal");
    }
};
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try
    {
        gemini336_orbslam3::SlamNodeQueueTestAccess::run();
        std::cout << "Queue admission, reservations, snapshots and ownership tests passed\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
