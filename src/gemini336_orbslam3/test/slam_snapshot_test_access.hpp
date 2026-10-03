#pragma once

#include "slam_node_test_support.hpp"

namespace gemini336_orbslam3
{
// Test-only identity inspection. Call on the tracking thread, after join, or while
// a test gate has paused tracking. The queue lock alone does not protect work payloads.
struct SlamSnapshotTestAccess
{
    struct FrameIdentity
    {
        double timestamp;
        int64_t timestamp_ns;
    };

    struct PendingIdentity
    {
        FrameIdentity frame;
        SlamNode::Clock::time_point received_at;
        uint64_t enqueue_sequence;
    };

    struct WorkIdentity
    {
        double timestamp;
        SlamNode::Clock::time_point received_at;
        SlamNode::TrackingWorkStage stage;
        uint64_t enqueue_sequence;
        int64_t timestamp_ns;
    };

    struct Snapshot : SlamNode::QueueSnapshot
    {
        std::optional<PendingIdentity> first;
        std::optional<PendingIdentity> second;
        std::optional<WorkIdentity> reservation;
        std::optional<WorkIdentity> startup_next_reservation;
    };

    static Snapshot snapshot(const SlamNode &node)
    {
        const std::lock_guard<std::mutex> lock(node.queue_mutex_);
        Snapshot result;
        result.pending = node.pending_frames_.size();
        result.in_flight = (node.reservation_received_at_ ? 1U : 0U) +
                           (node.startup_next_received_at_ ? 1U : 0U);
        result.outstanding = result.pending + result.in_flight;
        result.peak = node.outstanding_frames_peak_;
        result.enqueued = node.enqueued_frames_;
        result.processed = node.processed_frames_;
        result.startup_discarded = node.startup_discarded_frames_;
        result.overload_discarded = node.overload_discarded_frames_;
        result.startup_started = node.startup_wait_started_;
        result.startup_complete = node.startup_complete_;
        const auto pending_identity = [](const SlamNode::PendingFrame &pending) {
            return PendingIdentity{{pending.frame.timestamp, pending.frame.timestamp_ns},
                                   pending.received_at, pending.enqueue_sequence};
        };
        if (!node.pending_frames_.empty())
        {
            result.first = pending_identity(node.pending_frames_.front());
            result.oldest_received_at = result.first->received_at;
        }
        if (node.pending_frames_.size() >= 2)
            result.second = pending_identity(node.pending_frames_[1]);
        for (const auto &received_at : {node.reservation_received_at_, node.startup_next_received_at_})
        {
            if (received_at && (!result.oldest_received_at || *received_at < *result.oldest_received_at))
                result.oldest_received_at = received_at;
        }
        const auto work_identity = [](const std::optional<SlamNode::TrackingWork> &work,
                                      const std::optional<SlamNode::Clock::time_point> &received_at)
            -> std::optional<WorkIdentity> {
            // Inspect only stable ownership boundaries, where payload and capacity agree.
            if (work.has_value() != received_at.has_value() ||
                (work && work->pending.received_at != *received_at))
                throw std::logic_error("Reservation time disagrees with owned work");
            if (!work) return std::nullopt;
            return WorkIdentity{work->pending.frame.timestamp, work->pending.received_at,
                                work->stage, work->pending.enqueue_sequence,
                                work->pending.frame.timestamp_ns};
        };
        result.reservation = work_identity(node.tracking_work_, node.reservation_received_at_);
        result.startup_next_reservation = work_identity(node.startup_next_work_, node.startup_next_received_at_);
        return result;
    }
};
}
