#include "frontend/stereo_frontend.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.hpp>

namespace gemini336_orbslam3
{
StereoFrontend::StereoFrontend(
    rclcpp::Node *node,
    const std::string &left_image_topic,
    const std::string &right_image_topic,
    StereoFrameCallback callback)
    : node_(node), frame_callback_(std::move(callback))
{
    // node_ is non-owning; the caller must keep the node alive longer than this frontend.
    if (node_ == nullptr)
    {
        throw std::invalid_argument("StereoFrontend: node must not be null");
    }
    if (!frame_callback_)
    {
        throw std::invalid_argument("StereoFrontend: callback must not be empty");
    }
    if (left_image_topic.empty())
    {
        throw std::invalid_argument("StereoFrontend: left_image_topic must not be empty");
    }
    if (right_image_topic.empty())
    {
        throw std::invalid_argument("StereoFrontend: right_image_topic must not be empty");
    }

    // Read configuration once; the owning node may have already declared it.
    const auto queue_size = node_->has_parameter("stereo.sync_queue_size")
        ? node_->get_parameter("stereo.sync_queue_size").as_int()
        : node_->declare_parameter<int64_t>("stereo.sync_queue_size", 10);
    const auto max_time_diff = node_->has_parameter("stereo.max_time_diff_sec")
        ? node_->get_parameter("stereo.max_time_diff_sec").as_double()
        : node_->declare_parameter<double>("stereo.max_time_diff_sec", 0.0005);

    if (queue_size <= 0 || queue_size > std::numeric_limits<uint32_t>::max())
    {
        throw std::invalid_argument("StereoFrontend: stereo.sync_queue_size must be a positive uint32");
    }
    // Bound conversion to a ROS duration and reject sub-nanosecond tolerances.
    if (!std::isfinite(max_time_diff) || max_time_diff < 1e-9 ||
        max_time_diff > static_cast<double>(std::numeric_limits<int32_t>::max()))
    {
        throw std::invalid_argument(
            "StereoFrontend: stereo.max_time_diff_sec must be finite and between 1e-9 and INT32_MAX");
    }

    left_sub_.subscribe(node_, left_image_topic, rmw_qos_profile_sensor_data);
    right_sub_.subscribe(node_, right_image_topic, rmw_qos_profile_sensor_data);

    SyncPolicy policy(static_cast<uint32_t>(queue_size));
    policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_time_diff));
    sync_ = std::make_shared<Synchronizer>(policy);
    sync_->connectInput(left_sub_, right_sub_);
    sync_->registerCallback(
        std::bind(&StereoFrontend::stereo_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
}

void StereoFrontend::stereo_callback(
    const Image::ConstSharedPtr &left_msg,
    const Image::ConstSharedPtr &right_msg)
{
    const rclcpp::Time left_stamp(left_msg->header.stamp);
    const rclcpp::Time right_stamp(right_msg->header.stamp);
    RCLCPP_DEBUG(
        node_->get_logger(), "Stereo pair: left=%.9f right=%.9f delta=%.9f s",
        left_stamp.seconds(), right_stamp.seconds(),
        std::abs((left_stamp - right_stamp).seconds()));
}
}
