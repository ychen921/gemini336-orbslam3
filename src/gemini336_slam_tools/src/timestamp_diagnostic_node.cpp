#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

class TimestampDiagnosticNode : public rclcpp::Node
{
public:
    TimestampDiagnosticNode() : Node("timestamp_diagnostic_node")
    {
        left_ir_topic_ = this->declare_parameter<std::string>("left_ir_topic", "/camera/left_ir/image_raw");
        right_ir_topic_ = this->declare_parameter<std::string>("right_ir_topic", "/camera/right_ir/image_raw");
        imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/camera/gyro_accel/sample");

        report_period_sec_ = this->declare_parameter<double>("report_period_sec", 2.0);
        window_size_ = this->declare_parameter<int>("window_size", 200);

        stereo_pair_tolerance_sec_ = this->declare_parameter<double>("stereo_pair_tolerance_sec", 0.005);
        if (window_size_ <= 0) {
            throw std::invalid_argument("window_size must be positive");
        }
        if (!std::isfinite(report_period_sec_) || report_period_sec_ <= 0.0) {
            throw std::invalid_argument("report_period_sec must be finite and positive");
        }
        if (!std::isfinite(stereo_pair_tolerance_sec_) || stereo_pair_tolerance_sec_ <= 0.0) {
            throw std::invalid_argument("stereo_pair_tolerance_sec must be finite and positive");
        }

        // Sensor data QoS is usually more suitable for camera / IMU streams.
        auto qos = rclcpp::SensorDataQoS();

        // Subscribe to left/right IR image, imu topics
        left_ir_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            left_ir_topic_,
            qos,
            std::bind(&TimestampDiagnosticNode::leftIrCallback, this, std::placeholders::_1));

        right_ir_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            right_ir_topic_,
            qos,
            std::bind(&TimestampDiagnosticNode::rightIrCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_,
            qos,
            std::bind(&TimestampDiagnosticNode::imuCallback, this, std::placeholders::_1));

        report_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(report_period_sec_),
            std::bind(&TimestampDiagnosticNode::printReport, this));

        RCLCPP_INFO(this->get_logger(), "Timestamp Diagnostic Node started.");
        RCLCPP_INFO(this->get_logger(), "Left IR : %s", left_ir_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Right IR: %s", right_ir_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "IMU     : %s", imu_topic_.c_str());
    }

private:
    struct StreamStats
    {
        bool initialized = false;

        rclcpp::Time previous_stamp{0, 0, RCL_ROS_TIME};
        rclcpp::Time latest_stamp{0, 0, RCL_ROS_TIME};

        uint64_t message_count = 0;
        uint64_t non_monotonic_count = 0;

        std::deque<double> dt_history;
    };

    static double stampToSec(const builtin_interfaces::msg::Time & stamp)
    {
        return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
    }

    void updateStreamStats(
        const builtin_interfaces::msg::Time & stamp_msg,
        StreamStats & stats,
        const std::string & stream_name)
    {
        const rclcpp::Time stamp(stamp_msg);
        ++stats.message_count;

        if (!stats.initialized)
        {
            stats.previous_stamp = stamp;
            stats.latest_stamp = stamp;
            stats.initialized = true;
            return;
        }

        const double dt = (stamp - stats.previous_stamp).seconds();

        if (dt <= 0.0) {
            ++stats.non_monotonic_count;

            RCLCPP_WARN(
                this->get_logger(),
                "[%s] Non-monotonic timestamp: previous=%.9f current=%.9f dt=%.9f",
                stream_name.c_str(),
                stats.previous_stamp.seconds(),
                stamp.seconds(),
                dt);
        } else {
            stats.dt_history.push_back(dt);

            if (stats.dt_history.size() > static_cast<size_t>(window_size_)) {
                stats.dt_history.pop_front();
            }
        }

        stats.previous_stamp = stamp;
        stats.latest_stamp = stamp;
    }

    void leftIrCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        updateStreamStats(msg->header.stamp, left_ir_stats_, "Left IR");
        enqueueStereoStamp(rclcpp::Time(msg->header.stamp), true);
    }

    void rightIrCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        updateStreamStats(msg->header.stamp, right_ir_stats_, "Right IR");
        enqueueStereoStamp(rclcpp::Time(msg->header.stamp), false);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        updateStreamStats(msg->header.stamp, imu_stats_, "IMU");
    }

    void enqueueStereoStamp(const rclcpp::Time & stamp, bool is_left)
    {
        auto & have_stamp = is_left ? have_left_stamp_ : have_right_stamp_;
        auto & latest_stamp = is_left ? latest_left_stamp_ : latest_right_stamp_;
        if (have_stamp && stamp < latest_stamp) {
            left_unmatched_count_ += left_stamp_queue_.size();
            right_unmatched_count_ += right_stamp_queue_.size();
            left_stamp_queue_.clear();
            right_stamp_queue_.clear();
        }
        latest_stamp = stamp;
        have_stamp = true;

        auto & queue = is_left ? left_stamp_queue_ : right_stamp_queue_;
        auto & unmatched_count = is_left ? left_unmatched_count_ : right_unmatched_count_;
        queue.push_back(stamp);
        if (queue.size() > static_cast<size_t>(window_size_)) {
            queue.pop_front();
            ++unmatched_count;
        }
        updateStereoDifference();
    }

    void updateStereoDifference()
    {
        // FIFO pairing assumes ordered streams and a tolerance below half a frame period.
        while (!left_stamp_queue_.empty() && !right_stamp_queue_.empty()) {
            const double dt = (left_stamp_queue_.front() - right_stamp_queue_.front()).seconds();
            if (std::abs(dt) <= stereo_pair_tolerance_sec_) {
                stereo_dt_history_.push_back(dt);
                ++stereo_pair_count_;
                if (stereo_dt_history_.size() > static_cast<size_t>(window_size_)) {
                    stereo_dt_history_.pop_front();
                }
                left_stamp_queue_.pop_front();
                right_stamp_queue_.pop_front();
            } else if (dt < 0.0) {
                left_stamp_queue_.pop_front();
                ++left_unmatched_count_;
            } else {
                right_stamp_queue_.pop_front();
                ++right_unmatched_count_;
            }
        }
    }

    void printStreamStats(
        const std::string &name,
        const StreamStats &stats)
    {
        if (!stats.initialized) {
            RCLCPP_INFO(
                this->get_logger(),
                "%-10s : no messages received",
                name.c_str());
            return;
        }

        if (stats.dt_history.empty()) {
            RCLCPP_INFO(
                this->get_logger(),
                "%-10s : count=%lu latest=%.9f non_monotonic=%lu",
                name.c_str(),
                stats.message_count,
                stats.latest_stamp.seconds(),
                stats.non_monotonic_count);
            return;
        }

        const double sum = std::accumulate(stats.dt_history.begin(), stats.dt_history.end(), 0.0);
        const double mean_dt = sum / static_cast<double>(stats.dt_history.size());

        const auto [min_it, max_it] = std::minmax_element(stats.dt_history.begin(), stats.dt_history.end());
        const double frequency = mean_dt > 0.0 ? 1.0 / mean_dt : 0.0;

        RCLCPP_INFO(
            this->get_logger(),
            "%-10s : count=%lu  stamp=%.9f  "
            "mean_dt=%.3f ms  min=%.3f ms  max=%.3f ms  "
            "freq=%.2f Hz  non_monotonic=%lu",
            name.c_str(),
            stats.message_count,
            stats.latest_stamp.seconds(),
            mean_dt * 1000.0,
            (*min_it) * 1000.0,
            (*max_it) * 1000.0,
            frequency,
            stats.non_monotonic_count);
    }

    void printStereoStats()
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Stereo pairs: count=%lu unmatched(L/R)=%lu/%lu pending(L/R)=%zu/%zu tolerance=%.3f ms",
            stereo_pair_count_, left_unmatched_count_, right_unmatched_count_,
            left_stamp_queue_.size(), right_stamp_queue_.size(), stereo_pair_tolerance_sec_ * 1000.0);
        if (stereo_dt_history_.empty()) {
            RCLCPP_INFO(
                this->get_logger(),
                "Stereo Δt  : insufficient data");
            return;
        }

        double sum_abs = 0.0;
        double min_abs = std::numeric_limits<double>::max();
        double max_abs = 0.0;

        for (const double dt : stereo_dt_history_) {
            const double abs_dt = std::abs(dt);

            sum_abs += abs_dt;
            min_abs = std::min(min_abs, abs_dt);
            max_abs = std::max(max_abs, abs_dt);
        }

        const double mean_abs = sum_abs / static_cast<double>(stereo_dt_history_.size());

        const double latest_dt = stereo_dt_history_.back();

        RCLCPP_INFO(
            this->get_logger(),
            "Stereo Δt  : latest(L-R)=%+.3f ms  "
            "mean|Δt|=%.3f ms  min|Δt|=%.3f ms  max|Δt|=%.3f ms",
            latest_dt * 1000.0,
            mean_abs * 1000.0,
            min_abs * 1000.0,
            max_abs * 1000.0);
    }

    void printReport()
    {
        RCLCPP_INFO(
        this->get_logger(),
            "================ Timestamp Diagnostic ================");

        printStreamStats("Left IR", left_ir_stats_);
        printStreamStats("Right IR", right_ir_stats_);
        printStreamStats("IMU", imu_stats_);

        printStereoStats();

        RCLCPP_INFO(
            this->get_logger(),
            "======================================================");
    }
private:
    std::string left_ir_topic_;
    std::string right_ir_topic_;
    std::string imu_topic_;

    double report_period_sec_;
    int window_size_;
    double stereo_pair_tolerance_sec_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_ir_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_ir_sub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::TimerBase::SharedPtr report_timer_;

    StreamStats left_ir_stats_;
    StreamStats right_ir_stats_;
    StreamStats imu_stats_;

    bool have_left_stamp_ = false;
    bool have_right_stamp_ = false;

    rclcpp::Time latest_left_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time latest_right_stamp_{0, 0, RCL_ROS_TIME};

    std::deque<rclcpp::Time> left_stamp_queue_;
    std::deque<rclcpp::Time> right_stamp_queue_;
    uint64_t stereo_pair_count_ = 0;
    uint64_t left_unmatched_count_ = 0;
    uint64_t right_unmatched_count_ = 0;
    std::deque<double> stereo_dt_history_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<TimestampDiagnosticNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
