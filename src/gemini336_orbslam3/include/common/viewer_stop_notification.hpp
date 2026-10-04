#pragma once

#include "common/stop_control.hpp"
#include <functional>
#include <memory>
#include <utility>

namespace gemini336_orbslam3
{
// Captured actions must remain valid until Viewer joins, including startup unwind.
class ViewerStopNotification
{
public:
    ViewerStopNotification(std::shared_ptr<StopControl> control,
                           std::function<void()> cancel,
                           std::function<void()> fallback)
        : control_(std::move(control)), cancel_(std::move(cancel)),
          fallback_(std::move(fallback)) {}

    void operator()(std::exception_ptr error) const noexcept
    {
        try
        {
            if (error) control_->record_failure(StopReason::ViewerError, error);
            else control_->request_stop(StopReason::ViewerStop);
        }
        catch (...) { /* Cancellation is still attempted if recording fails. */ }
        try { cancel_(); }
        catch (...)
        {
            record(StopReason::CancelError, std::current_exception());
            try { fallback_(); }
            catch (...) { record(StopReason::ShutdownError, std::current_exception()); }
        }
    }

private:
    void record(StopReason reason, std::exception_ptr error) const noexcept
    {
        try { control_->record_failure(reason, error); }
        catch (...) {}
    }
    std::shared_ptr<StopControl> control_;
    std::function<void()> cancel_;
    std::function<void()> fallback_;
};
}
