#include "frontend/stereo_frontend.hpp"

#include <cmath>
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
}
}
