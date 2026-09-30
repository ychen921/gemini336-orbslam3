#pragma once

#include "common/stop_control.hpp"
#include <rclcpp/context.hpp>
#include <memory>
#include <optional>
#include <stdexcept>

namespace gemini336_orbslam3
{
// Main owns the registration. Notifications may run on the signal handling thread;
// they capture only weak control ownership and never access the node or executor.
class ContextStopRegistration
{
public:
    ContextStopRegistration(rclcpp::Context::SharedPtr context,
                            const std::shared_ptr<StopControl> &control)
        : context_(std::move(context)), control_(control)
    {
        if (!context_ || !control)
            throw std::invalid_argument("Context stop registration requires context and control");
        const auto notify = [weak = control_]() noexcept {
            try
            {
                if (const auto state = weak.lock())
                    state->request_stop(StopReason::ContextShutdown);
            }
            catch (...) { /* A control-storage failure must not escape the signal thread. */ }
        };
        handle_ = context_->add_on_shutdown_callback(notify);
        // Registration does not replay an earlier shutdown. A racing notification is
        // harmless because request_stop preserves the first reason and time.
        if (!context_->is_valid()) notify();
    }

    ContextStopRegistration(const ContextStopRegistration &) = delete;
    ContextStopRegistration &operator=(const ContextStopRegistration &) = delete;
    ~ContextStopRegistration() noexcept { close(); }

    // Main calls this after callbacks finish and before its final context shutdown.
    // Removal does not imply an already running notification has finished.
    bool close() noexcept
    {
        if (!handle_) return true;
        bool removed = false;
        std::exception_ptr error;
        try { removed = context_->remove_on_shutdown_callback(*handle_); }
        catch (...) { error = std::current_exception(); }
        handle_.reset();
        if (!removed)
        {
            try
            {
                if (const auto state = control_.lock())
                    state->record_failure(StopReason::ShutdownError, error, true);
            }
            catch (...) { /* Destruction must remain safe even when control storage fails. */ }
        }
        return removed;
    }

private:
    rclcpp::Context::SharedPtr context_;
    std::weak_ptr<StopControl> control_;
    std::optional<rclcpp::OnShutdownCallbackHandle> handle_;
};
}
