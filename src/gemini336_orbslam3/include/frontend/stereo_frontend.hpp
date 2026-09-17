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

namespace gemini336_orbslam3
{
class StereoFrontend
{
public:
    using Image = sensor_msgs::msg::Image;

    using StereoFrameCallback = std::function<void(const StereoFrame &)>;

    StereoFrontend(
        rclcpp::Node *node,
        const std::string &left_image_topic,
        const std::string &right_image_topic,
        StereoFrameCallback callback,
        std::function<void()> input_activity_callback = {});

private:
    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<
            Image,
            Image>;
            
    using Synchronizer = 
        message_filters::Synchronizer<SyncPolicy>;

    void stereo_callback(
        const Image::ConstSharedPtr& left_msg,
        const Image::ConstSharedPtr& right_msg);

private:
    rclcpp::Node* node_;

    StereoFrameCallback frame_callback_;
    std::function<void()> input_activity_callback_;

    message_filters::Subscriber<Image> left_sub_;
    message_filters::Subscriber<Image> right_sub_;

    std::shared_ptr<Synchronizer> sync_;    
};

}
