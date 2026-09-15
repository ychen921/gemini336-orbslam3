#include "slam/orbslam3_adapter.hpp"
#include "frontend/stereo_frontend.hpp"

#include <cstdint>
#include <algorithm>
#include <chrono>
#include <limits>
#include <exception>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace gemini336_orbslam3
{
namespace
{
const char *tracking_state_name(TrackingState state)
{
    switch (state)
    {
    case TrackingState::SystemNotReady: return "SystemNotReady";
    case TrackingState::NoImagesYet: return "NoImagesYet";
    case TrackingState::NotInitialized: return "NotInitialized";
    case TrackingState::Ok: return "Ok";
    case TrackingState::RecentlyLost: return "RecentlyLost";
    case TrackingState::Lost: return "Lost";
    case TrackingState::OkKlt: return "OkKlt";
    case TrackingState::Unknown: return "Unknown";
    }
    return "Unknown";
}
}

class SlamNode : public rclcpp::Node
{
public:
    SlamNode() : Node("slam_node")
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;

        OrbSlam3Config config;
        config.vocabulary_path = declare_parameter<std::string>(
            "vocabulary_path", ORB_SLAM3_DEFAULT_VOCABULARY_PATH, descriptor);
        config.settings_path = declare_parameter<std::string>(
            "settings_path", "", descriptor);

        require_absolute_path(config.vocabulary_path, "vocabulary_path");
        require_absolute_path(config.settings_path, "settings_path");
        const auto left_topic = declare_parameter<std::string>(
            "left_image_topic", "/camera/left_ir/image_raw", descriptor);
        const auto right_topic = declare_parameter<std::string>(
            "right_image_topic", "/camera/right_ir/image_raw", descriptor);
        if (left_topic.empty() || right_topic.empty())
            throw std::invalid_argument("Image topics must not be empty");

        RCLCPP_INFO(get_logger(), "Vocabulary: %s", config.vocabulary_path.c_str());
        RCLCPP_INFO(get_logger(), "Settings: %s", config.settings_path.c_str());
        slam_ = std::make_unique<OrbSlam3Adapter>(config);
        frontend_ = std::make_unique<StereoFrontend>(
            this, left_topic, right_topic,
            [this](const StereoFrame &frame) { on_frame(frame); });
        started_ = last_report_ = Clock::now();
        report_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() { report(false); });
        RCLCPP_INFO(get_logger(), "Stereo SLAM initialized: left=%s right=%s",
                    left_topic.c_str(), right_topic.c_str());
    }

    void shutdown()
    {
        // Spin has returned, so no image callback can overlap subscription teardown.
        frontend_.reset();
        report_timer_.reset();
        report(true);
        RCLCPP_INFO(get_logger(), "Stereo input stopped; processed=%llu last_state=%s",
                    static_cast<unsigned long long>(processed_frames_),
                    tracking_state_name(slam_->trackingState()));
        slam_->shutdown();
        RCLCPP_INFO(get_logger(), "Stereo SLAM shutdown returned");
    }

private:
    using Clock = std::chrono::steady_clock;

    struct Statistics
    {
        uint64_t frames = 0;
        double track_sum_ms = 0.0;
        double track_max_ms = 0.0;
        uint64_t intervals = 0;
        double interval_sum_ms = 0.0;
        double interval_min_ms = std::numeric_limits<double>::infinity();
        double interval_max_ms = 0.0;
    };

    void log_statistics(const char *scope, const Statistics &stats, double elapsed)
    {
        RCLCPP_INFO(get_logger(),
                    "Stereo stats: scope=%s frames=%llu total=%llu elapsed_sec=%.6f rate_hz=%.6f "
                    "track_mean_ms=%.6f track_max_ms=%.6f intervals=%llu "
                    "interval_min_ms=%.6f interval_mean_ms=%.6f interval_max_ms=%.6f",
                    scope, static_cast<unsigned long long>(stats.frames),
                    static_cast<unsigned long long>(processed_frames_), elapsed,
                    elapsed > 0.0 ? stats.frames / elapsed : 0.0,
                    stats.frames ? stats.track_sum_ms / stats.frames : 0.0, stats.track_max_ms,
                    static_cast<unsigned long long>(stats.intervals),
                    stats.intervals ? stats.interval_min_ms : 0.0,
                    stats.intervals ? stats.interval_sum_ms / stats.intervals : 0.0,
                    stats.interval_max_ms);
    }

    void report(bool final)
    {
        const auto now = Clock::now();
        log_statistics(final ? "tail" : "window", window_,
                       std::chrono::duration<double>(now - last_report_).count());
        if (final)
            log_statistics("total", total_, std::chrono::duration<double>(now - started_).count());
        window_ = Statistics{};
        last_report_ = now;
    }

    void on_frame(const StereoFrame &frame)
    {
        const auto previous_state = slam_->trackingState();
        // Adapter exceptions propagate to main: retrying after an upstream failure is unsafe.
        const auto start = Clock::now();
        slam_->track(frame);
        const double track_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        // Preserve the previous timestamp across report windows; count only successful calls.
        for (auto *stats : {&window_, &total_})
        {
            ++stats->frames;
            stats->track_sum_ms += track_ms;
            stats->track_max_ms = std::max(stats->track_max_ms, track_ms);
            if (processed_frames_ > 0)
            {
                const double interval_ms = (frame.timestamp - previous_timestamp_) * 1000.0;
                ++stats->intervals;
                stats->interval_sum_ms += interval_ms;
                stats->interval_min_ms = std::min(stats->interval_min_ms, interval_ms);
                stats->interval_max_ms = std::max(stats->interval_max_ms, interval_ms);
            }
        }
        previous_timestamp_ = frame.timestamp;
        ++processed_frames_;
        if (processed_frames_ == 1)
            RCLCPP_INFO(get_logger(), "First stereo frame processed: timestamp=%.9f", frame.timestamp);
        const auto state = slam_->trackingState();
        RCLCPP_DEBUG(get_logger(), "Stereo frame: index=%llu timestamp=%.9f track_ms=%.6f state=%s",
                     static_cast<unsigned long long>(processed_frames_), frame.timestamp,
                     track_ms, tracking_state_name(state));
        if (state != previous_state)
            RCLCPP_INFO(get_logger(), "Tracking state: %s -> %s",
                        tracking_state_name(previous_state), tracking_state_name(state));
    }

    static void require_absolute_path(const std::string &path, const char *name)
    {
        if (path.empty() || !std::filesystem::path(path).is_absolute())
            throw std::invalid_argument(std::string(name) + " must be a nonempty absolute path");
    }

    std::unique_ptr<OrbSlam3Adapter> slam_;
    // Reverse destruction order also stops input first during constructor failure/unwinding.
    std::unique_ptr<StereoFrontend> frontend_;
    uint64_t processed_frames_ = 0;
    double previous_timestamp_ = 0.0;
    Statistics window_;
    Statistics total_;
    Clock::time_point started_;
    Clock::time_point last_report_;
    rclcpp::TimerBase::SharedPtr report_timer_;
};
}

int main(int argc, char **argv)
{
    int result = 0;
    std::shared_ptr<gemini336_orbslam3::SlamNode> node;
    try
    {
        rclcpp::init(argc, argv);
        node = std::make_shared<gemini336_orbslam3::SlamNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("slam_node"), "%s", error.what());
        result = 1;
    }

    // Retain ownership across spin failures so both exit paths explicitly shut down SLAM.
    if (node)
    {
        try
        {
            node->shutdown();
        }
        catch (const std::exception &error)
        {
            RCLCPP_ERROR(rclcpp::get_logger("slam_node"), "Shutdown failed: %s", error.what());
            result = 1;
        }
    }
    node.reset();
    if (rclcpp::ok())
        rclcpp::shutdown();
    return result;
}
