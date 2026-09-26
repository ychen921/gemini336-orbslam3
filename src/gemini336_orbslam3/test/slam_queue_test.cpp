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

    static void run()
    {
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
