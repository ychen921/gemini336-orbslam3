// Compile the existing node queue directly; do not duplicate its implementation.
#include "../src/slam_node.cpp"
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
        node.pending_frames_capacity_ = 2;
        require(!node.reserve_tracking_work(), "empty queue produced a reservation");

        for (int i = 1; i <= 2; ++i)
        {
            StereoFrame frame;
            frame.timestamp = i;
            frame.timestamp_ns = int64_t(i) * 1000000000LL;
            frame.left = cv::Mat(2, 2, CV_8UC1, cv::Scalar(i));
            frame.right = frame.left.clone();
            node.enqueue_frame(frame);
        }

        const auto before = node.queue_snapshot();
        require(node.reserve_tracking_work(), "failed to reserve first frame");
        const auto reserved = node.queue_snapshot();
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
        require(reserved.reservation->stage == SlamNode::TrackingWorkStage::Reserved &&
                !reserved.reservation->imu_batch_consumed &&
                node.tracking_work_ && !node.tracking_work_->imu_batch,
                "reservation unexpectedly acquired an IMU batch");
        require(reserved.oldest_received_at == before.oldest_received_at &&
                reserved.startup_started == before.startup_started &&
                reserved.reservation->received_at == before.first->received_at,
                "reservation reset a waiting deadline");
        require(node.tracking_work_->pending.frame.left.at<unsigned char>(0, 0) == 1,
                "reserved pixels did not survive queue removal");

        // Retrying an existing reservation must not remove the next queued frame.
        require(node.reserve_tracking_work(), "failed to retain existing reservation");
        const auto retried = node.queue_snapshot();
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
        try { node.enqueue_frame(incoming); }
        catch (const std::runtime_error &) { full_rejected = true; }
        const auto after = node.queue_snapshot();
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
        node.enqueue_frame(frame);
        const auto before = node.queue_snapshot();

        // The sole frame leaves the queue but remains outstanding across retries.
        require(node.reserve_tracking_work(), "failed to reserve sole frame");
        require(node.reserve_tracking_work(), "empty queue hid an existing reservation");
        const auto snapshot = node.queue_snapshot();
        require(snapshot.pending == 0 && !snapshot.first && !snapshot.second &&
                snapshot.in_flight == 1 && snapshot.outstanding == 1 &&
                snapshot.enqueued == 1 && snapshot.processed == 0 &&
                snapshot.reservation && snapshot.reservation->timestamp == 1 &&
                snapshot.oldest_received_at == before.oldest_received_at,
                "empty queue lost reserved work or its deadline");
        require(node.tracking_work_ &&
                node.tracking_work_->pending.frame.right.at<unsigned char>(0, 0) == 7,
                "sole reserved frame lost its pixels");
    }

    static void test_startup_pair()
    {
        SlamNode node(SlamNode::QueueTestTag{});
        node.node_logger_ = std::make_shared<spdlog::logger>(
            "queue_test", std::make_shared<spdlog::sinks::null_sink_mt>());
        node.pending_frames_capacity_ = 3;
        for (int i = 1; i <= 3; ++i)
        {
            StereoFrame frame;
            frame.timestamp = i;
            frame.timestamp_ns = int64_t(i) * 1000000000LL;
            frame.left = cv::Mat(2, 2, CV_8UC1, cv::Scalar(i));
            frame.right = frame.left.clone();
            node.enqueue_frame(frame);
        }

        const auto initial = node.queue_snapshot();
        require(initial.first && initial.second && initial.startup_started,
                "startup test did not retain initial frames and deadline");
        const SlamNode::Clock::time_point second_received_at = initial.second->received_at;

        // Every transition must preserve admission accounting and the startup deadline.
        const auto check_accounting = [&](std::size_t pending, std::size_t in_flight,
                                          uint64_t discarded) {
            const auto snapshot = node.queue_snapshot();
            require(snapshot.pending == pending && snapshot.in_flight == in_flight &&
                    snapshot.outstanding == pending + in_flight &&
                    snapshot.outstanding <= node.pending_frames_capacity_ &&
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

        require(node.reserve_startup_pair(), "failed to reserve startup pair");
        const auto reserved = check_accounting(1, 2, 0);
        require(reserved.reservation && reserved.startup_next_reservation &&
                reserved.reservation->timestamp == 1 &&
                reserved.reservation->enqueue_sequence == 1 &&
                reserved.reservation->timestamp_ns == 1000000000LL &&
                reserved.startup_next_reservation->timestamp == 2 &&
                reserved.first && reserved.first->frame.timestamp == 3 &&
                reserved.oldest_received_at == initial.oldest_received_at,
                "startup reservation did not preserve FIFO or oldest enqueue time");

        require(node.reserve_startup_pair(), "failed to retain startup pair on retry");
        const auto retried = check_accounting(1, 2, 0);
        require(retried.reservation && retried.startup_next_reservation &&
                retried.reservation->timestamp == 1 &&
                retried.startup_next_reservation->timestamp == 2 &&
                retried.first && retried.first->frame.timestamp == 3,
                "startup retry replaced candidates or removed another frame");

        // Exercise the MissingHistory transition directly, without querying IMU.
        node.discard_startup_first();
        const auto discarded = check_accounting(1, 1, 1);
        require(discarded.reservation && discarded.reservation->timestamp == 2 &&
                !discarded.startup_next_reservation &&
                discarded.first && discarded.first->frame.timestamp == 3 &&
                discarded.oldest_received_at == second_received_at &&
                discarded.reservation->received_at == second_received_at,
                "startup discard failed to promote F1 with its original enqueue time");
        require(node.tracking_work_ && !node.startup_next_work_ &&
                node.tracking_work_->pending.frame.timestamp == 2 &&
                node.tracking_work_->pending.frame.left.at<unsigned char>(0, 0) == 2,
                "startup discard lost the promoted frame payload");

        require(node.reserve_startup_pair(), "failed to refill startup F1");
        const auto refilled = check_accounting(0, 2, 1);
        require(refilled.reservation && refilled.startup_next_reservation &&
                refilled.reservation->timestamp == 2 &&
                refilled.startup_next_reservation->timestamp == 3 &&
                !refilled.first && !refilled.second &&
                refilled.oldest_received_at == second_received_at &&
                refilled.reservation->stage == SlamNode::TrackingWorkStage::Reserved &&
                refilled.startup_next_reservation->stage == SlamNode::TrackingWorkStage::Reserved &&
                !refilled.reservation->imu_batch_consumed &&
                !refilled.startup_next_reservation->imu_batch_consumed,
                "startup refill changed candidate order, deadline or batch state");
        require(node.startup_next_work_ &&
                node.startup_next_work_->pending.frame.right.at<unsigned char>(0, 0) == 3 &&
                !node.tracking_work_->imu_batch && !node.startup_next_work_->imu_batch,
                "startup refill lost pixels or unexpectedly acquired IMU data");

        require(node.reserve_startup_pair(), "empty queue hid the retained startup pair");
        check_accounting(0, 2, 1);
    }

    static void test_startup_shortage()
    {
        SlamNode node(SlamNode::QueueTestTag{});
        node.node_logger_ = std::make_shared<spdlog::logger>(
            "queue_test", std::make_shared<spdlog::sinks::null_sink_mt>());
        node.pending_frames_capacity_ = 2;
        require(!node.reserve_startup_pair(), "empty queue produced a startup pair");
        const auto empty = node.queue_snapshot();
        require(empty.pending == 0 && empty.in_flight == 0 && empty.enqueued == 0 &&
                !empty.startup_started && !node.tracking_work_ && !node.startup_next_work_,
                "empty startup attempt changed state");

        StereoFrame first;
        first.timestamp = 1;
        first.timestamp_ns = int64_t(1) * 1000000000LL;
        node.enqueue_frame(first);
        const auto initial = node.queue_snapshot();
        require(initial.startup_started && initial.first,
                "first frame did not establish the startup deadline");

        // A single initial candidate must remain queued across repeated attempts.
        for (int retry = 0; retry < 3; ++retry)
        {
            require(!node.reserve_startup_pair(), "startup accepted only one candidate");
            const auto waiting = node.queue_snapshot();
            require(waiting.pending == 1 && waiting.in_flight == 0 &&
                    waiting.outstanding == 1 && waiting.enqueued == 1 &&
                    waiting.processed == 0 && waiting.startup_discarded == 0 &&
                    waiting.first && waiting.first->frame.timestamp == 1 &&
                    waiting.first->received_at == initial.first->received_at &&
                    !waiting.reservation && !waiting.startup_next_reservation &&
                    !node.tracking_work_ && !node.startup_next_work_ &&
                    waiting.startup_started == initial.startup_started &&
                    !waiting.startup_complete,
                    "insufficient startup candidates changed ownership or deadline");
        }

        StereoFrame second;
        second.timestamp = 2;
        second.timestamp_ns = int64_t(2) * 1000000000LL;
        second.left = cv::Mat(2, 2, CV_8UC1, cv::Scalar(2));
        node.enqueue_frame(second);
        require(node.reserve_startup_pair(), "second frame did not enable startup");
        node.discard_startup_first();
        const auto promoted = node.queue_snapshot();
        require(promoted.reservation && promoted.reservation->timestamp == 2,
                "startup discard did not preserve the second frame");

        // No replacement exists: retry must retain the promoted F0, not discard it.
        for (int retry = 0; retry < 3; ++retry)
        {
            require(!node.reserve_startup_pair(), "startup refill succeeded without a frame");
            const auto waiting = node.queue_snapshot();
            require(waiting.pending == 0 && waiting.in_flight == 1 &&
                    waiting.outstanding == 1 && waiting.enqueued == 2 &&
                    waiting.processed == 0 && waiting.startup_discarded == 1 &&
                    waiting.reservation && waiting.reservation->timestamp == 2 &&
                    waiting.reservation->received_at == promoted.reservation->received_at &&
                    waiting.oldest_received_at == promoted.oldest_received_at &&
                    !waiting.startup_next_reservation &&
                    waiting.startup_started == initial.startup_started &&
                    !waiting.startup_complete &&
                    node.tracking_work_ && !node.startup_next_work_ &&
                    node.tracking_work_->pending.frame.left.at<unsigned char>(0, 0) == 2,
                    "missing replacement changed reserved F0 or deadline");
        }

        StereoFrame third;
        third.timestamp = 3;
        third.timestamp_ns = int64_t(3) * 1000000000LL;
        node.enqueue_frame(third);
        require(node.reserve_startup_pair(), "new frame did not refill retained startup F0");
        const auto refilled = node.queue_snapshot();
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
        node.pending_frames_capacity_ = 4;
        for (int i = 1; i <= 4; ++i)
        {
            StereoFrame frame;
            frame.timestamp = i;
            frame.timestamp_ns = int64_t(i) * 1000000000LL;
            node.enqueue_frame(frame);
        }
        const auto initial = node.queue_snapshot();

        // Model consecutive MissingHistory results through the real discard operation.
        // Each call discards only F0; the next attempt, not discard itself, refills F1.
        for (uint64_t candidate = 1; candidate <= 3; ++candidate)
        {
            require(node.reserve_startup_pair(), "failed to reserve consecutive startup pair");
            const auto pair = node.queue_snapshot();
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

            node.discard_startup_first();
            const auto discarded = node.queue_snapshot();
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
                    node.tracking_work_ &&
                    node.tracking_work_->pending.frame.timestamp == candidate + 1 &&
                    !node.startup_next_work_ &&
                    discarded.startup_started == initial.startup_started &&
                    !discarded.startup_complete,
                    "consecutive discard removed more than F0 or reset the deadline");
        }

        require(!node.reserve_startup_pair(), "exhausted startup queue unexpectedly refilled");
        const auto exhausted = node.queue_snapshot();
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
        node.tracking_mode_ = TrackingMode::Stereo;
        node.pending_frames_capacity_ = 3;
        node.node_logger_ = std::make_shared<spdlog::logger>(
            "overload_test", std::make_shared<spdlog::sinks::ostream_sink_mt>(logs));
        node.trace_ = std::make_unique<DiagnosticTrace>("", 32);

        StereoFrame frame;
        frame.timestamp = 1;
        frame.timestamp_ns = int64_t(1) * 1000000000LL;
        frame.left = cv::Mat(2, 2, CV_8UC1, cv::Scalar(1));
        node.enqueue_frame(frame);
        require(node.reserve_tracking_work(), "failed to retain frame for overload test");
        for (int i = 2; i <= 3; ++i)
        {
            frame.timestamp = i;
            frame.timestamp_ns = int64_t(i) * 1000000000LL;
            node.enqueue_frame(frame);
        }
        const auto initial = node.queue_snapshot();
        require(initial.pending == 2 && initial.in_flight == 1 &&
                initial.peak == 3 && initial.outstanding == 3,
                "capacity peak omitted the reserved frame");

        // Existing callers keep rejecting full queues unless replacement is explicit.
        frame.timestamp = 4;
        frame.timestamp_ns = int64_t(4) * 1000000000LL;
        bool rejected = false;
        try { node.enqueue_frame(frame); }
        catch (const std::runtime_error &) { rejected = true; }
        require(rejected && node.queue_snapshot().enqueued == 3 &&
                node.queue_snapshot().overload_discarded == 0 && logs.str().empty(),
                "default admission silently enabled overload discard");

        for (int i = 4; i <= 5; ++i)
        {
            frame.timestamp = i;
            frame.timestamp_ns = int64_t(i) * 1000000000LL;
            node.enqueue_frame(frame, SlamNode::QueueFullPolicy::DiscardOldestQueued);
            const auto snapshot = node.queue_snapshot();
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
                    node.tracking_work_ &&
                    node.tracking_work_->pending.frame.timestamp == 1 &&
                    snapshot.startup_started == initial.startup_started,
                    "overload replacement lost reservation, FIFO or accounting");
        }

        // Validate before eviction: duplicate/non-finite input must not discard anything.
        for (const double invalid : {5.0, std::numeric_limits<double>::quiet_NaN()})
        {
            frame.timestamp = invalid;
            rejected = false;
            try { node.enqueue_frame(frame, SlamNode::QueueFullPolicy::DiscardOldestQueued); }
            catch (const std::invalid_argument &) { rejected = true; }
            require(rejected && node.queue_snapshot().enqueued == 5 &&
                    node.queue_snapshot().overload_discarded == 2,
                    "invalid incoming timestamp evicted queued work");
        }

        node.tracking_mode_ = TrackingMode::StereoImu;
        frame.timestamp = 6;
        frame.timestamp_ns = int64_t(6) * 1000000000LL;
        rejected = false;
        try { node.enqueue_frame(frame, SlamNode::QueueFullPolicy::DiscardOldestQueued); }
        catch (const std::invalid_argument &) { rejected = true; }
        require(rejected && node.queue_snapshot().enqueued == 5 &&
                node.queue_snapshot().overload_discarded == 2,
                "Stereo-IMU allowed overload discard");
        rejected = false;
        try { node.enqueue_frame(frame); }
        catch (const std::runtime_error &) { rejected = true; }
        require(rejected && node.queue_snapshot().outstanding == 3 &&
                node.queue_snapshot().overload_discarded == 2,
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
        reserved_only.pending_frames_capacity_ = 1;
        frame.timestamp = 1;
        frame.timestamp_ns = int64_t(1) * 1000000000LL;
        reserved_only.enqueue_frame(frame);
        require(reserved_only.reserve_tracking_work(), "failed to fill reserved-only capacity");
        frame.timestamp = 2;
        frame.timestamp_ns = int64_t(2) * 1000000000LL;
        rejected = false;
        try { reserved_only.enqueue_frame(frame, SlamNode::QueueFullPolicy::DiscardOldestQueued); }
        catch (const std::runtime_error &) { rejected = true; }
        const auto held = reserved_only.queue_snapshot();
        require(rejected && held.pending == 0 && held.in_flight == 1 &&
                held.enqueued == 1 && held.overload_discarded == 0 &&
                held.reservation && held.reservation->timestamp == 1,
                "overload admission discarded a reservation without a queued victim");
    }

    static void test_concurrent_overload_snapshots()
    {
        std::ostringstream logs;
        SlamNode node(SlamNode::QueueTestTag{});
        node.pending_frames_capacity_ = 3;
        node.node_logger_ = std::make_shared<spdlog::logger>(
            "concurrent_overload_test", std::make_shared<spdlog::sinks::ostream_sink_mt>(logs));
        StereoFrame frame;
        frame.timestamp = 1;
        frame.timestamp_ns = int64_t(1) * 1000000000LL;
        node.enqueue_frame(frame);
        require(node.reserve_tracking_work(), "failed to reserve concurrent overload frame");
        for (int i = 2; i <= 3; ++i)
        {
            frame.timestamp = i;
            frame.timestamp_ns = int64_t(i) * 1000000000LL;
            node.enqueue_frame(frame);
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
                    node.enqueue_frame(incoming, SlamNode::QueueFullPolicy::DiscardOldestQueued);
                }
            }
            catch (...) { producer_error = std::current_exception(); }
        });
        start.store(true);
        bool consistent = true;
        for (int i = 0; i < 4000; ++i)
        {
            const auto snapshot = node.queue_snapshot();
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
        const auto final = node.queue_snapshot();
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
        node.pending_frames_capacity_ = 30;
        node.trace_ = std::make_unique<DiagnosticTrace>("", 30);
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
                    node.enqueue_frame(frame);
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
            const auto snapshot = node.queue_snapshot();
            if (snapshot.pending < previous || snapshot.pending > 30 ||
                snapshot.pending != snapshot.enqueued || snapshot.peak != snapshot.pending ||
                snapshot.first.has_value() != (snapshot.pending > 0) ||
                snapshot.second.has_value() != (snapshot.pending >= 2) ||
                (snapshot.first && snapshot.startup_started != snapshot.first->received_at))
                consistent = false;
            previous = snapshot.pending;
        }
        producer.join();
        if (producer_error) std::rethrow_exception(producer_error);
        require(consistent, "incoherent concurrent queue snapshot");
        const auto retained = node.queue_snapshot();
        require(retained.pending == 30 && retained.first->frame.timestamp == 1 &&
                retained.second->frame.timestamp == 2, "FIFO snapshot mismatch");

        // Rejection must leave admission, accounting and the startup deadline unchanged.
        StereoFrame incoming;
        incoming.timestamp = 31;
        incoming.timestamp_ns = int64_t(31) * 1000000000LL;
        bool full_rejected = false;
        try { node.enqueue_frame(incoming); }
        catch (const std::runtime_error &) { full_rejected = true; }
        incoming.timestamp = 30;
        incoming.timestamp_ns = int64_t(30) * 1000000000LL;
        bool duplicate_rejected = false;
        try { node.enqueue_frame(incoming); }
        catch (const std::invalid_argument &) { duplicate_rejected = true; }
        const auto after = node.queue_snapshot();
        require(full_rejected && duplicate_rejected && after.pending == retained.pending &&
                after.enqueued == retained.enqueued && after.startup_started == retained.startup_started,
                "rejection changed queue state");
        require(node.trace_->stats().recorded == 30, "rejected frame was traced as admitted");

        // Simulate removal only to test snapshot lifetime, not B2 completion semantics.
        {
            const std::lock_guard<std::mutex> lock(node.queue_mutex_);
            node.pending_frames_.clear();
        }
        require(node.queue_snapshot().pending == 0 &&
                retained.first->frame.left.at<unsigned char>(0, 0) == 1 &&
                retained.second->frame.right.at<unsigned char>(0, 0) == 2,
                "snapshot pixels did not outlive queue removal");
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
