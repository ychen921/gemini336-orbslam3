#pragma once

#include "frontend/imu_frontend.hpp"

namespace gemini336_orbslam3
{
// Direct finite input exercises reception without DDS scheduling.
struct ImuFrontendTestAccess
{
    static bool subscription_present(const ImuFrontend &frontend)
    {
        return bool(frontend.imu_sub_);
    }

    static void receive(ImuFrontend &frontend, int seconds, bool unavailable = false)
    {
        auto msg = std::make_shared<sensor_msgs::msg::Imu>();
        msg->header.stamp.sec = seconds;
        msg->header.frame_id = "imu";
        msg->linear_acceleration.z = 9.8;
        if (unavailable) msg->linear_acceleration_covariance[0] = -1.0;
        frontend.imu_callback(msg);
    }

    static void attach_trace(ImuFrontend &frontend, DiagnosticTrace *trace)
    {
        // Test setup only, before any producer or consumer starts.
        frontend.trace_ = trace;
    }

    static std::optional<double> consumed_until(ImuFrontend &frontend)
    {
        const std::lock_guard<std::mutex> lock(frontend.imu_mutex_);
        return frontend.last_taken_timestamp_;
    }

    static rclcpp::SubscriptionBase::SharedPtr subscription(ImuFrontend &frontend)
    {
        return frontend.imu_sub_;
    }

    static bool in_group(ImuFrontend &frontend,
                         const rclcpp::CallbackGroup::SharedPtr &group)
    {
        return bool(group->find_subscription_ptrs_if([&](const auto &subscription) {
            return subscription == frontend.imu_sub_;
        }));
    }
};
}
