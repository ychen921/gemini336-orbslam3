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
        TrackingCoordinator::TrackingWorkStage stage;
        uint64_t enqueue_sequence;
        int64_t timestamp_ns;
    };

    struct Snapshot : TrackingCoordinator::QueueSnapshot
    {
        std::optional<PendingIdentity> first;
        std::optional<PendingIdentity> second;
        std::optional<WorkIdentity> reservation;
        std::optional<WorkIdentity> startup_next_reservation;
    };

    static Snapshot snapshot(const SlamNode &node)
    {
        const std::lock_guard<std::mutex> lock(node.coordinator_->queue_mutex_);
        Snapshot result;
        static_cast<TrackingCoordinator::QueueSnapshot &>(result) = node.coordinator_->snapshot_locked();
        const auto pending_identity = [](const TrackingCoordinator::PendingFrame &pending) {
            return PendingIdentity{{pending.frame.timestamp, pending.frame.timestamp_ns},
                                   pending.received_at, pending.enqueue_sequence};
        };
        if (!node.coordinator_->pending_frames_.empty())
        {
            result.first = pending_identity(node.coordinator_->pending_frames_.front());
        }
        if (node.coordinator_->pending_frames_.size() >= 2)
            result.second = pending_identity(node.coordinator_->pending_frames_[1]);
        const auto work_identity = [](const std::optional<TrackingCoordinator::TrackingWork> &work)
            -> std::optional<WorkIdentity> {
            if (!work) return std::nullopt;
            return WorkIdentity{work->pending.frame.timestamp, work->pending.received_at,
                                work->stage, work->pending.enqueue_sequence,
                                work->pending.frame.timestamp_ns};
        };
        result.reservation = work_identity(node.coordinator_->tracking_work_);
        result.startup_next_reservation = work_identity(node.coordinator_->startup_next_work_);
        return result;
    }
};
}
