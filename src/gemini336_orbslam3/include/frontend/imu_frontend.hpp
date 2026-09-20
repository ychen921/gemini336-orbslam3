#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "common/types.hpp"

namespace gemini336_orbslam3
{
enum class ImuBatchStatus
{
    Ready,
    WaitingForData,
    MissingHistory,
    BufferOverflow,
    DataGap,
    InvalidRequest
};

struct ImuBatch
{
    ImuBatchStatus status = ImuBatchStatus::InvalidRequest;
    std::vector<ImuMeasurement> measurements;
};

// Component-wise summaries of accepted measurements in their source IMU frame.
struct ImuVectorStats
{
    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
};

struct ImuFrontendStats
{
    uint64_t received = 0;
    uint64_t accepted = 0;
    uint64_t unavailable = 0;
    uint64_t invalid_values = 0;
    uint64_t invalid_timestamps = 0;
    uint64_t timestamp_precision_rejections = 0;
    uint64_t duplicates = 0;
    uint64_t backwards = 0;
    uint64_t overflow = 0;
    std::size_t buffered = 0;
    std::optional<double> last_overflow_timestamp;

    // Cumulative diagnostics; extrema are meaningful only when their count is nonzero.
    std::optional<double> first_timestamp;
    std::optional<double> last_timestamp;
    uint64_t interval_count = 0;
    double interval_sum_sec = 0.0;
    double interval_min_sec = 0.0;
    double interval_max_sec = 0.0;
    uint64_t excessive_gaps = 0;
    ImuVectorStats accel;
    ImuVectorStats gyro;
    std::string frame_id;
    uint64_t frame_id_changes = 0;
    uint64_t empty_frame_ids = 0;
};

// All access, including stats(), must be serialized with the subscription callback.
// The owning node must outlive this frontend; stop spinning before destruction.
class ImuFrontend
{
public:
    // Requires an explicit, finite positive imu.max_gap_sec parameter.
    ImuFrontend(rclcpp::Node *node, const std::string &imu_topic);

    ImuFrontend(const ImuFrontend &) = delete;
    ImuFrontend &operator=(const ImuFrontend &) = delete;
    ImuFrontend(ImuFrontend &&) = delete;
    ImuFrontend &operator=(ImuFrontend &&) = delete;

    ImuFrontendStats stats() const;

    // Nonblocking, consuming query for (t_prev, t_curr]. After success, t_prev
    // must equal the previous successful t_curr exactly. Failures change no state.
    // Retains one sample at/before t_curr for boundary checks, never for resending.
    // Ready verifies nonempty output and strictly increasing sample times with
    // gaps <= imu.max_gap_sec from the left anchor through right-side coverage.
    // This does not certify lossless input or SLAM initialization.
    ImuBatch takeMeasurements(double t_prev, double t_curr);

private:
    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

    rclcpp::Node *node_;
    std::size_t buffer_capacity_;
    double max_gap_sec_;

    // Accepted samples and boundaries used to validate consuming interval queries.
    std::deque<ImuMeasurement> imu_buffer_;
    std::optional<int64_t> last_accepted_timestamp_ns_;
    std::optional<double> first_accepted_timestamp_;
    std::optional<double> last_taken_timestamp_;

    ImuFrontendStats stats_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};
}
