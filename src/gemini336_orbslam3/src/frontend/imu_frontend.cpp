#include "frontend/imu_frontend.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace gemini336_orbslam3
{
ImuFrontend::ImuFrontend(rclcpp::Node *node, const std::string &imu_topic)
    : node_(node), buffer_capacity_(0)
{
    if (node_ == nullptr)
    {
        throw std::invalid_argument("ImuFrontend: node must not be null");
    }
    if (imu_topic.empty())
    {
        throw std::invalid_argument("ImuFrontend: imu_topic must not be empty");
    }

    // Read configuration once, allowing the owning node to declare parameters first.
    const int64_t qos_depth = node_->has_parameter("imu.qos_depth")
        ? node_->get_parameter("imu.qos_depth").as_int()
        : node_->declare_parameter<int64_t>("imu.qos_depth", 200);
    const int64_t buffer_capacity = node_->has_parameter("imu.buffer_capacity")
        ? node_->get_parameter("imu.buffer_capacity").as_int()
        : node_->declare_parameter<int64_t>("imu.buffer_capacity", 2000);
    if (qos_depth <= 0 ||
        static_cast<uint64_t>(qos_depth) > std::numeric_limits<std::size_t>::max())
    {
        throw std::invalid_argument("ImuFrontend: imu.qos_depth must be a positive size_t");
    }
    if (buffer_capacity <= 0 ||
        static_cast<uint64_t>(buffer_capacity) > imu_buffer_.max_size())
    {
        throw std::invalid_argument("ImuFrontend: imu.buffer_capacity exceeds valid deque capacity");
    }
    buffer_capacity_ = static_cast<std::size_t>(buffer_capacity);

    // DDS depth limits messages waiting for callbacks; buffer_capacity_ separately
    // limits accepted measurements retained for later image/IMU time slicing.
    rclcpp::SensorDataQoS qos;
    qos.keep_last(static_cast<std::size_t>(qos_depth));
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos,
        std::bind(&ImuFrontend::imu_callback, this, std::placeholders::_1));
}

ImuFrontendStats ImuFrontend::stats() const
{
    // Return a copy without exposing buffer ownership. Callers must serialize this
    // read with imu_callback(); the frontend intentionally provides no locking.
    ImuFrontendStats snapshot = stats_;
    snapshot.buffered = imu_buffer_.size();
    return snapshot;
}

void ImuFrontend::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
{
    ++stats_.received;
    // Each rejected message is counted once, at the first failed check.
    // A covariance first element of -1 marks that measurement as unavailable.
    // Orientation is unused, so its availability does not affect acceptance.
    if (msg->linear_acceleration_covariance[0] == -1.0 ||
        msg->angular_velocity_covariance[0] == -1.0)
    {
        ++stats_.unavailable;
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "Dropping IMU sample: acceleration or angular velocity unavailable");
        return;
    }

    // Validate in the message's double precision before narrowing to Vector3f.
    // Finite doubles may still exceed the representable float range.
    const double components[] = {
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
    for (const double value : components)
    {
        if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        {
            ++stats_.invalid_values;
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 5000,
                "Dropping IMU sample: non-finite value or value outside float range");
            return;
        }
    }

    // Use sensor acquisition time, not callback arrival time. Zero is valid at
    // the start of a simulated timeline; negative or unnormalized stamps are not.
    const auto &stamp = msg->header.stamp;
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u)
    {
        ++stats_.invalid_timestamps;
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "Dropping IMU sample: invalid header timestamp");
        return;
    }
    // Compare integer nanoseconds so ordering does not depend on double rounding.
    const int64_t timestamp_ns = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;

    // Compare against the last accepted sample, so rejected data cannot advance
    // the timeline. A clock reset requires explicit coordination with SLAM;
    // accepting it here would mix two timelines in the same buffer.
    if (last_accepted_timestamp_ns_ && timestamp_ns == *last_accepted_timestamp_ns_)
    {
        ++stats_.duplicates;
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "Dropping IMU sample: duplicate timestamp");
        return;
    }
    if (last_accepted_timestamp_ns_ && timestamp_ns < *last_accepted_timestamp_ns_)
    {
        ++stats_.backwards;
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "Dropping IMU sample: backwards timestamp; time resets are not supported");
        return;
    }

    // Preserve source IMU axes and ROS units (m/s^2, rad/s), including gravity.
    // Calibration, bias estimation and camera-frame transforms belong downstream.
    ImuMeasurement measurement;
    measurement.timestamp = static_cast<double>(timestamp_ns) * 1e-9;
    measurement.accel = Eigen::Vector3f(
        static_cast<float>(components[0]), static_cast<float>(components[1]),
        static_cast<float>(components[2]));
    measurement.gyro = Eigen::Vector3f(
        static_cast<float>(components[3]), static_cast<float>(components[4]),
        static_cast<float>(components[5]));

    // Prefer recent data while bounding memory use. Retain the latest eviction
    // timestamp so future interval queries can identify coverage lost to overflow.
    if (imu_buffer_.size() == buffer_capacity_)
    {
        stats_.last_overflow_timestamp = imu_buffer_.front().timestamp;
        imu_buffer_.pop_front();
        ++stats_.overflow;
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "IMU buffer full: dropping oldest sample (capacity=%zu)", buffer_capacity_);
    }
    imu_buffer_.push_back(measurement);
    // Advance acceptance state only after the measurement has been stored.
    last_accepted_timestamp_ns_ = timestamp_ns;
    ++stats_.accepted;
}
}
