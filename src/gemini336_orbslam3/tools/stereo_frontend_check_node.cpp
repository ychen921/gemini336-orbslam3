#include "frontend/stereo_frontend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <string>

namespace gemini336_orbslam3
{
class StereoFrontendCheckNode : public rclcpp::Node
{
public:
    StereoFrontendCheckNode()
        : Node("stereo_frontend_check_node"), last_report_(Clock::now())
    {
        const auto left_topic = declare_parameter<std::string>(
            "left_image_topic", "/camera/left_ir/image_raw");
        const auto right_topic = declare_parameter<std::string>(
            "right_image_topic", "/camera/right_ir/image_raw");

        frontend_ = std::make_unique<StereoFrontend>(
            this, left_topic, right_topic,
            [this](const StereoFrame &frame) { on_frame(frame); });
        timer_ = create_wall_timer(std::chrono::seconds(5), [this]() { report(); });

        RCLCPP_INFO(get_logger(), "Checking stereo input: left=%s right=%s",
                    left_topic.c_str(), right_topic.c_str());
    }

private:
    using Clock = std::chrono::steady_clock;

    void on_frame(const StereoFrame &frame)
    {
        ++total_frames_;
        ++window_frames_;

        // Retain the latest geometry while accumulating errors across all windows.
        left_width_ = frame.left.cols;
        left_height_ = frame.left.rows;
        right_width_ = frame.right.cols;
        right_height_ = frame.right.rows;
        left_type_ = frame.left.type();
        right_type_ = frame.right.type();
        if (frame.left.empty() || frame.right.empty() ||
            frame.left.size() != frame.right.size() ||
            left_type_ != CV_8UC1 || right_type_ != CV_8UC1)
        {
            ++invalid_images_;
        }

        // An invalid timestamp breaks adjacency; never form an interval across it.
        if (!std::isfinite(frame.timestamp))
        {
            ++invalid_timestamps_;
            has_previous_stamp_ = false;
            return;
        }

        // Include signed intervals so duplicate and backward timestamps remain visible.
        // Preserve the previous stamp across reporting windows.
        if (has_previous_stamp_)
        {
            const double interval = frame.timestamp - previous_stamp_;
            if (interval == 0.0) ++duplicate_stamps_;
            if (interval < 0.0) ++backward_stamps_;
            interval_min_ = std::min(interval_min_, interval);
            interval_max_ = std::max(interval_max_, interval);
            interval_sum_ += interval;
            ++interval_count_;
        }
        previous_stamp_ = frame.timestamp;
        has_previous_stamp_ = true;
    }

    void report()
    {
        const auto now = Clock::now();
        const double elapsed = std::chrono::duration<double>(now - last_report_).count();

        RCLCPP_INFO(
            get_logger(),
            "Frames: total=%llu window=%llu rate=%.3f Hz; cumulative errors: "
            "invalid_images=%llu invalid_timestamps=%llu duplicates=%llu backwards=%llu",
            static_cast<unsigned long long>(total_frames_),
            static_cast<unsigned long long>(window_frames_),
            elapsed > 0.0 ? static_cast<double>(window_frames_) / elapsed : 0.0,
            static_cast<unsigned long long>(invalid_images_),
            static_cast<unsigned long long>(invalid_timestamps_),
            static_cast<unsigned long long>(duplicate_stamps_),
            static_cast<unsigned long long>(backward_stamps_));
        if (window_frames_ > 0)
        {
            RCLCPP_INFO(
                get_logger(), "Last images: left=%dx%d type=%d right=%dx%d type=%d (CV_8UC1=%d)",
                left_width_, left_height_, left_type_, right_width_, right_height_, right_type_, CV_8UC1);
        }
        if (interval_count_ > 0)
        {
            RCLCPP_INFO(
                get_logger(), "Timestamp intervals (window): n=%llu min=%.6f mean=%.6f max=%.6f ms",
                static_cast<unsigned long long>(interval_count_), interval_min_ * 1000.0,
                interval_sum_ / static_cast<double>(interval_count_) * 1000.0, interval_max_ * 1000.0);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Timestamp intervals (window): unavailable (no adjacent frames)");
        }

        // Keep cumulative errors and the previous stamp when starting a new window.
        last_report_ = now;
        window_frames_ = 0;
        interval_count_ = 0;
        interval_sum_ = 0.0;
        interval_min_ = std::numeric_limits<double>::infinity();
        interval_max_ = -std::numeric_limits<double>::infinity();
    }

    // The single-threaded executor serializes image callbacks and reporting.
    Clock::time_point last_report_;
    uint64_t total_frames_ = 0;
    uint64_t window_frames_ = 0;

    uint64_t invalid_images_ = 0;
    uint64_t invalid_timestamps_ = 0;
    uint64_t duplicate_stamps_ = 0;
    uint64_t backward_stamps_ = 0;

    uint64_t interval_count_ = 0;
    double previous_stamp_ = 0.0;
    bool has_previous_stamp_ = false;
    double interval_sum_ = 0.0;
    double interval_min_ = std::numeric_limits<double>::infinity();
    double interval_max_ = -std::numeric_limits<double>::infinity();

    int left_width_ = 0;
    int left_height_ = 0;
    int right_width_ = 0;
    int right_height_ = 0;
    int left_type_ = 0;
    int right_type_ = 0;

    std::unique_ptr<StereoFrontend> frontend_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int result = 0;

    try
    {
        rclcpp::spin(std::make_shared<gemini336_orbslam3::StereoFrontendCheckNode>());
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("stereo_frontend_check_node"), "%s", error.what());
        result = 1;
    }

    rclcpp::shutdown();

    return result;
}
