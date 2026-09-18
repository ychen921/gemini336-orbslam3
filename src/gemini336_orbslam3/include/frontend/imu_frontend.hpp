#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "common/types.hpp"

namespace gemini336_orbslam3
{
struct ImuFrontendStats
{
    uint64_t received = 0;
    uint64_t accepted = 0;
    uint64_t unavailable = 0;
    uint64_t invalid_values = 0;
    uint64_t invalid_timestamps = 0;
    uint64_t duplicates = 0;
    uint64_t backwards = 0;
    uint64_t overflow = 0;
    std::size_t buffered = 0;
    std::optional<double> last_overflow_timestamp;
};

// All access, including stats(), must be serialized with the subscription callback.
// The owning node must outlive this frontend; stop spinning before destruction.
class ImuFrontend
{
public:
    ImuFrontend(rclcpp::Node *node, const std::string &imu_topic);

    ImuFrontend(const ImuFrontend &) = delete;
    ImuFrontend &operator=(const ImuFrontend &) = delete;
    ImuFrontend(ImuFrontend &&) = delete;
    ImuFrontend &operator=(ImuFrontend &&) = delete;

    ImuFrontendStats stats() const;

private:
    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

    rclcpp::Node *node_;
    std::size_t buffer_capacity_;
    std::deque<ImuMeasurement> imu_buffer_;
    std::optional<int64_t> last_accepted_timestamp_ns_;
    ImuFrontendStats stats_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};
}
