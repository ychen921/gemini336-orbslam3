#pragma once

#include "frontend/stereo_frontend.hpp"

namespace gemini336_orbslam3
{
struct StereoFrontendTestAccess
{
    static void dispatch(StereoFrontend &frontend, bool left, int seconds)
    {
        auto image = std::make_shared<StereoFrontend::Image>();
        image->header.stamp.sec = seconds;
        image->height = image->width = image->step = 1;
        image->encoding = "mono8";
        image->data = {42};
        std::shared_ptr<void> message = image;
        const auto subscription = left ? frontend.left_sub_.getSubscriber() : frontend.right_sub_.getSubscriber();
        subscription->handle_message(message, rclcpp::MessageInfo{});
    }

    static bool in_group(StereoFrontend &frontend,
                         const rclcpp::CallbackGroup::SharedPtr &group)
    {
        const auto left = group->find_subscription_ptrs_if(
            [&](const rclcpp::SubscriptionBase::SharedPtr &subscription) {
                return subscription == frontend.left_sub_.getSubscriber();
            });
        const auto right = group->find_subscription_ptrs_if(
            [&](const rclcpp::SubscriptionBase::SharedPtr &subscription) {
                return subscription == frontend.right_sub_.getSubscriber();
            });
        return left != nullptr && right != nullptr;
    }

    static void receive(StereoFrontend &frontend,
                        const StereoFrontend::Image::ConstSharedPtr &left,
                        const StereoFrontend::Image::ConstSharedPtr &right)
    {
        frontend.stereo_callback(left, right);
    }

    static void receive(StereoFrontend &frontend, int seconds)
    {
        auto image = std::make_shared<StereoFrontend::Image>();
        image->header.stamp.sec = seconds;
        image->height = image->width = image->step = 1;
        image->encoding = "mono8";
        image->data = {42};
        // A reception-group timer supplies an already synchronized pair. Conversion,
        // on_frame, admission, scheduling and completion remain production code.
        frontend.stereo_callback(image, image);
    }
};
}
