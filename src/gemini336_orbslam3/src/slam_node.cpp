#include "slam/orbslam3_adapter.hpp"
#include "frontend/stereo_frontend.hpp"

#include <cstdint>
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
        RCLCPP_INFO(get_logger(), "Stereo SLAM initialized: left=%s right=%s",
                    left_topic.c_str(), right_topic.c_str());
    }

    void shutdown()
    {
        // Spin has returned, so no image callback can overlap subscription teardown.
        frontend_.reset();
        RCLCPP_INFO(get_logger(), "Stereo input stopped; processed=%llu last_state=%s",
                    static_cast<unsigned long long>(processed_frames_),
                    tracking_state_name(slam_->trackingState()));
        slam_->shutdown();
        RCLCPP_INFO(get_logger(), "Stereo SLAM shutdown returned");
    }

private:
    void on_frame(const StereoFrame &frame)
    {
        const auto previous_state = slam_->trackingState();
        // Adapter exceptions propagate to main: retrying after an upstream failure is unsafe.
        slam_->track(frame);
        ++processed_frames_;
        if (processed_frames_ == 1)
            RCLCPP_INFO(get_logger(), "First stereo frame processed: timestamp=%.9f", frame.timestamp);
        const auto state = slam_->trackingState();
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
