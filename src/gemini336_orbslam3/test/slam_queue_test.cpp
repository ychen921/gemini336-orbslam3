// Compile the existing node queue directly; do not duplicate its implementation.
#include "../src/slam_node.cpp"

#include <atomic>
#include <iostream>
#include <thread>

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
        node.pending_frames_capacity_ = 3;
        for (int i = 1; i <= 3; ++i)
        {
            StereoFrame frame;
            frame.timestamp = i;
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
                        snapshot.processed + snapshot.startup_discarded,
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

    static void run()
    {
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
        bool full_rejected = false;
        try { node.enqueue_frame(incoming); }
        catch (const std::runtime_error &) { full_rejected = true; }
        incoming.timestamp = 30;
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
