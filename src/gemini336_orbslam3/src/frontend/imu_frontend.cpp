#include "frontend/imu_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace gemini336_orbslam3
{
ImuFrontend::ImuFrontend(rclcpp::Node *node, const std::string &imu_topic)
    : node_(node), buffer_capacity_(0), max_gap_sec_(0.0)
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

    // NaN is an unset sentinel, not an operational default: the caller must
    // choose a gap limit appropriate to the configured sensor frequency.
    max_gap_sec_ = node_->has_parameter("imu.max_gap_sec")
        ? node_->get_parameter("imu.max_gap_sec").as_double()
        : node_->declare_parameter<double>("imu.max_gap_sec", std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(max_gap_sec_) || max_gap_sec_ <= 0.0)
    {
        throw std::invalid_argument(
            "ImuFrontend: imu.max_gap_sec must be explicitly set to a finite positive value");
    }

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

ImuBatch ImuFrontend::takeMeasurements(double t_prev, double t_curr)
{
    // Reuse the caller's previous boundary without an epsilon: overlap, skipped
    // intervals and retries after success must not silently lose or resend data.
    if (!std::isfinite(t_prev) || !std::isfinite(t_curr) ||
        t_prev < 0.0 || t_curr <= t_prev ||
        (last_taken_timestamp_ && t_prev != *last_taken_timestamp_))
    {
        return {ImuBatchStatus::InvalidRequest, {}};
    }

    // An empty buffer can still receive its initial history. Once data exists,
    // monotonic acceptance means missing earlier history cannot arrive later.
    if (imu_buffer_.empty())
    {
        return {ImuBatchStatus::WaitingForData, {}};
    }
    if (imu_buffer_.front().timestamp > t_prev)
    {
        // Before the first accepted sample, history never existed. Otherwise,
        // FIFO overflow removed the anchor (successful consumption retains it).
        return {t_prev < *first_accepted_timestamp_ ? ImuBatchStatus::MissingHistory
                                                   : ImuBatchStatus::BufferOverflow, {}};
    }
    if (imu_buffer_.back().timestamp < t_curr)
    {
        return {ImuBatchStatus::WaitingForData, {}};
    }

    // Both searches use upper_bound to exclude the left boundary and include
    // the right. A future sample proves arrival coverage but is not returned.
    const auto after_time = [](double timestamp, const ImuMeasurement &measurement) {
        return timestamp < measurement.timestamp;
    };
    const auto first = std::upper_bound(imu_buffer_.begin(), imu_buffer_.end(), t_prev, after_time);
    const auto end = std::upper_bound(first, imu_buffer_.end(), t_curr, after_time);
    if (first == end)
    {
        return {ImuBatchStatus::DataGap, {}};
    }

    // Check only the bracketing samples needed for this interval. If t_curr is
    // itself a sample, later gaps are irrelevant and no future sample is needed.
    const auto right = (end - 1)->timestamp == t_curr ? end - 1 : end;
    for (auto current = first; current <= right; ++current)
    {
        const double gap = current->timestamp - (current - 1)->timestamp;
        if (gap <= 0.0 || gap > max_gap_sec_)
        {
            return {ImuBatchStatus::DataGap, {}};
        }
    }

    // Allocate/copy before committing consumption, so allocation failure leaves
    // the buffer and successful-query boundary unchanged.
    ImuBatch batch{ImuBatchStatus::Ready, {first, end}};
    imu_buffer_.erase(imu_buffer_.begin(), end - 1);
    last_taken_timestamp_ = t_curr;

    return batch;
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

    // Distinct integer stamps can collapse to the same double at epoch scale.
    // The newest accepted sample remains buffered even after interval consumption.
    const double timestamp = static_cast<double>(timestamp_ns) * 1e-9;
    if (!imu_buffer_.empty() && timestamp <= imu_buffer_.back().timestamp)
    {
        ++stats_.timestamp_precision_rejections;
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "Dropping IMU sample: timestamp is not increasing in double seconds");
        return;
    }

    // Preserve source IMU axes and ROS units (m/s^2, rad/s), including gravity.
    // Calibration, bias estimation and camera-frame transforms belong downstream.
    ImuMeasurement measurement;
    measurement.timestamp = timestamp;
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
    if (!first_accepted_timestamp_)
    {
        first_accepted_timestamp_ = timestamp;
    }
    ++stats_.accepted;
}
}
