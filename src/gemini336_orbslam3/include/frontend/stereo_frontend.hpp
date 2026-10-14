#pragma once

#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "common/types.hpp"
#include "common/callback_guard.hpp"
#include "common/diagnostic_trace.hpp"
#include "common/evaluation_recorder.hpp"

namespace gemini336_orbslam3
{
struct StereoFrontendStats
{
    ImageTimestampStats left;
    ImageTimestampStats right;
    uint64_t synchronized = 0;
    uint64_t converted = 0;
    uint64_t empty = 0;
    uint64_t size_mismatch = 0;
    uint64_t conversion_failed = 0;
};

class StereoFrontend
{
public:
    using Image = sensor_msgs::msg::Image;

    using StereoFrameCallback = std::function<void(const StereoFrame &)>;

    // The node must outlive this frontend. Callbacks run synchronously on input delivery.
    StereoFrontend(
        rclcpp::Node *node,
        rclcpp::CallbackGroup::SharedPtr reception_group,
        const std::string &left_image_topic,
        const std::string &right_image_topic,
        StereoFrameCallback callback,
        std::function<void()> input_activity_callback = {},
        DiagnosticTrace *trace = nullptr,
        std::shared_ptr<CallbackGuard> callback_guard = nullptr,
        bool collect_statistics = false);

    // Call only after executor callbacks finish; retain data for final diagnostics.
    void stop_receiving();
    // Reception group owns updates; only read after executor callbacks finish.
    StereoFrontendStats stats() const { return stats_; }

private:
    // Finite tests deliver image pairs directly without spinning an executor.
    friend struct StereoFrontendTestAccess;

    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<
            Image,
            Image>;

    using Synchronizer =
        message_filters::Synchronizer<SyncPolicy>;

    void stereo_callback(
        const Image::ConstSharedPtr &left_msg,
        const Image::ConstSharedPtr &right_msg);

private:
    std::shared_ptr<CallbackGuard> callback_guard_;
    rclcpp::Node *node_;
    DiagnosticTrace *trace_;  // Non-owning; the node stops callbacks before destroying it.
    const bool collect_statistics_;
    double max_gap_sec_ = 0.1;
    StereoFrontendStats stats_;

    StereoFrameCallback frame_callback_;
    // Raw input activity is reported even when no synchronized pair is produced.
    std::function<void()> input_activity_callback_;

    message_filters::Subscriber<Image> left_sub_;
    message_filters::Subscriber<Image> right_sub_;

    // Declared last so synchronization is destroyed before its input subscribers.
    std::unique_ptr<Synchronizer> sync_;
};

}
