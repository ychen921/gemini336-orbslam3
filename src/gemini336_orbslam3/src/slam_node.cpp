#include "slam/orbslam3_adapter.hpp"

#include <exception>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace gemini336_orbslam3
{
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

        RCLCPP_INFO(get_logger(), "Vocabulary: %s", config.vocabulary_path.c_str());
        RCLCPP_INFO(get_logger(), "Settings: %s", config.settings_path.c_str());
        slam_ = std::make_unique<OrbSlam3Adapter>(config);
        RCLCPP_INFO(get_logger(), "Stereo SLAM initialized; image input is not connected in Phase 3A");
    }

    void shutdown()
    {
        // Called only after spin returns; future input must stop before adapter shutdown.
        slam_->shutdown();
        RCLCPP_INFO(get_logger(), "Stereo SLAM shutdown returned");
    }

private:
    static void require_absolute_path(const std::string &path, const char *name)
    {
        if (path.empty() || !std::filesystem::path(path).is_absolute())
            throw std::invalid_argument(std::string(name) + " must be a nonempty absolute path");
    }

    std::unique_ptr<OrbSlam3Adapter> slam_;
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
