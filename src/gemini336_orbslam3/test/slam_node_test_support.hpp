#pragma once

#include "../src/slam_node.cpp"

namespace gemini336_orbslam3
{
// Tests compile the real node in each executable and skip sensor/backend setup.
inline SlamNode::SlamNode(QueueTestTag, const rclcpp::NodeOptions &options)
    : Node("slam_queue_test", options), stop_control_(std::make_shared<StopControl>())
{
}
}
