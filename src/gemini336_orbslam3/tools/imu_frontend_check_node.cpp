#include "frontend/imu_frontend.hpp"

#include <chrono>
#include <cinttypes>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

namespace gemini336_orbslam3
{
class ImuFrontendCheckNode : public rclcpp::Node
{
public:
    ImuFrontendCheckNode()
        : Node("imu_frontend_check_node")
    {
        // Load once so reporting and silence policies remain stable during a run.
        const std::string topic = declare_parameter<std::string>(
            "imu_topic", "/camera/gyro_accel/sample");
        report_interval_sec_ = declare_parameter<double>("report_interval_sec", 5.0);
        input_silence_sec_ = declare_parameter<double>("input_silence_sec", 5.0);
        if (!std::isfinite(report_interval_sec_) || report_interval_sec_ <= 0.0 ||
            !std::isfinite(input_silence_sec_) || input_silence_sec_ <= 0.0)
        {
            throw std::invalid_argument(
                "report_interval_sec and input_silence_sec must be finite positive values");
        }
        frontend_ = std::make_unique<ImuFrontend>(this, topic);

        // A wall timer also operates when simulated ROS time is paused. Its polling
        // interval bounds activity observation resolution, not sensor latency.
        last_report_ = Clock::now();
        last_activity_ = last_report_;
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { tick(); });
        RCLCPP_INFO(get_logger(),
                    "Checking IMU input: topic=%s max_gap_sec=%.9f report_interval_sec=%.3f "
                    "input_silence_sec=%.3f; activity polling=100 ms",
                    topic.c_str(), get_parameter("imu.max_gap_sec").as_double(),
                    report_interval_sec_, input_silence_sec_);
        RCLCPP_INFO(get_logger(),
                    "Observation mode: buffer is not consumed; oldest-sample eviction is expected "
                    "when full. Buffer overflow does not establish ROS message loss.");
    }

    void report(bool final = false)
    {
        // Counter differences measure wall-time reception, independently of the
        // header timestamps used for cumulative sensor-interval diagnostics.
        const ImuFrontendStats stats = frontend_->stats();
        const Clock::time_point now = Clock::now();
        const double elapsed = std::chrono::duration<double>(now - last_report_).count();
        const uint64_t received = stats.received - report_received_;
        const uint64_t accepted = stats.accepted - report_accepted_;
        RCLCPP_INFO(get_logger(),
                    "%s IMU report: received=%" PRIu64 " accepted=%" PRIu64
                    "; window received=%" PRIu64 " accepted=%" PRIu64
                    " elapsed=%.6f s receive_rate=%.3f Hz accept_rate=%.3f Hz",
                    final ? "Final" : "Periodic", stats.received, stats.accepted, received, accepted,
                    elapsed, elapsed > 0.0 ? received / elapsed : 0.0,
                    elapsed > 0.0 ? accepted / elapsed : 0.0);
        RCLCPP_INFO(get_logger(),
                    "Cumulative rejections: unavailable=%" PRIu64 " invalid_values=%" PRIu64
                    " invalid_timestamps=%" PRIu64 " precision=%" PRIu64
                    " duplicates=%" PRIu64 " backwards=%" PRIu64,
                    stats.unavailable, stats.invalid_values, stats.invalid_timestamps,
                    stats.timestamp_precision_rejections, stats.duplicates, stats.backwards);
        RCLCPP_INFO(get_logger(),
                    "Buffer: buffered=%zu overflow=%" PRIu64
                    " (non-consuming observation; eviction is not a ROS loss count)",
                    stats.buffered, stats.overflow);

        // Empty diagnostics remain unavailable instead of printing initialized extrema.
        if (stats.accepted == 0)
        {
            RCLCPP_INFO(get_logger(), "Accepted timestamps, frame_id and axis summaries: unavailable");
        }
        else
        {
            RCLCPP_INFO(get_logger(),
                        "Cumulative timestamps: first=%.9f last=%.9f span=%.9f s; "
                        "latest frame_id='%s' changes=%" PRIu64 " empty=%" PRIu64,
                        *stats.first_timestamp, *stats.last_timestamp,
                        *stats.last_timestamp - *stats.first_timestamp, stats.frame_id.c_str(),
                        stats.frame_id_changes, stats.empty_frame_ids);
            report_vector("accel", "m/s^2", stats.accel);
            report_vector("gyro", "rad/s", stats.gyro);
            if (stats.frame_id_changes > 0)
            {
                RCLCPP_WARN(get_logger(), "Cumulative axis summaries may mix coordinate frames");
            }
        }
        if (stats.interval_count == 0)
        {
            RCLCPP_INFO(get_logger(), "Cumulative timestamp intervals: unavailable (need two accepted samples)");
        }
        else
        {
            RCLCPP_INFO(get_logger(),
                        "Cumulative timestamp intervals: n=%" PRIu64
                        " min=%.9f mean=%.9f max=%.9f s excessive_gaps=%" PRIu64,
                        stats.interval_count, stats.interval_min_sec,
                        stats.interval_sum_sec / static_cast<double>(stats.interval_count),
                        stats.interval_max_sec, stats.excessive_gaps);
        }
        report_received_ = stats.received;
        report_accepted_ = stats.accepted;
        last_report_ = now;
    }

private:
    using Clock = std::chrono::steady_clock;

    void tick()
    {
        // Received includes rejected messages: invalid input is active input, with
        // its quality reflected by rejection counters rather than silence warnings.
        const uint64_t received = frontend_->stats().received;
        const Clock::time_point now = Clock::now();
        if (received != observed_received_)
        {
            if (silence_warned_)
            {
                RCLCPP_INFO(get_logger(), "IMU input resumed");
            }
            last_activity_ = now;
            observed_received_ = received;
            silence_warned_ = false;
        }
        else if (!silence_warned_ &&
                 std::chrono::duration<double>(now - last_activity_).count() >= input_silence_sec_)
        {
            RCLCPP_WARN(get_logger(), "%s (observed silence >= %.3f s); continuing to wait",
                        received == 0 ? "No IMU input since startup" : "IMU input stopped",
                        input_silence_sec_);
            silence_warned_ = true;
        }

        // Silence monitoring continues between reports; it never shuts down the node.
        if (std::chrono::duration<double>(now - last_report_).count() >= report_interval_sec_)
        {
            report();
        }
    }

    void report_vector(const char *name, const char *unit, const ImuVectorStats &stats)
    {
        // Keep all three axes together so units and cumulative scope are explicit.
        RCLCPP_INFO(get_logger(),
                    "Cumulative %s (%s): min=[%.9g,%.9g,%.9g] mean=[%.9g,%.9g,%.9g] max=[%.9g,%.9g,%.9g]",
                    name, unit, stats.min.x(), stats.min.y(), stats.min.z(),
                    stats.mean.x(), stats.mean.y(), stats.mean.z(),
                    stats.max.x(), stats.max.y(), stats.max.z());
    }

    // The single-threaded executor serializes reception, polling and snapshots.
    double report_interval_sec_ = 5.0;
    double input_silence_sec_ = 5.0;
    Clock::time_point last_report_;
    Clock::time_point last_activity_;
    uint64_t observed_received_ = 0;
    uint64_t report_received_ = 0;
    uint64_t report_accepted_ = 0;
    bool silence_warned_ = false;
    std::unique_ptr<ImuFrontend> frontend_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    std::shared_ptr<gemini336_orbslam3::ImuFrontendCheckNode> node;
    try
    {
        node = std::make_shared<gemini336_orbslam3::ImuFrontendCheckNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("imu_frontend_check_node"), "%s", error.what());
        result = 1;
    }

    // Spin has ended, so the final snapshot cannot race an input callback. Keep
    // logging alive until the one final report has been emitted, including on errors.
    if (node) node->report(true);
    node.reset();
    rclcpp::shutdown();
    return result;
}
