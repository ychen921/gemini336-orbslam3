#pragma once

#include "common/callback_guard.hpp"
#include <rclcpp/rclcpp.hpp>

namespace gemini336_orbslam3
{
// Standalone frontend tools stop their own context on callback failure.
inline std::shared_ptr<CallbackGuard> make_frontend_callback_guard(
    rclcpp::Node *node, std::shared_ptr<StopControl> control = {})
{
    if (!control) control = std::make_shared<StopControl>();
    const auto context = node->get_node_base_interface()->get_context();
    const auto logger = node->get_logger();
    const auto shutdown = [context]() { context->shutdown("Frontend callback failure"); };
    return std::make_shared<CallbackGuard>(control, shutdown, shutdown,
        [logger](std::exception_ptr error) {
            try { std::rethrow_exception(error); }
            catch (const std::exception &e) { RCLCPP_ERROR(logger, "%s", e.what()); }
            catch (...) { RCLCPP_ERROR(logger, "Unknown frontend callback exception"); }
        });
}
}
