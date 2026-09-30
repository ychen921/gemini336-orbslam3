#pragma once

#include "common/stop_control.hpp"
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace gemini336_orbslam3
{
// Explicit source classification without parsing diagnostic strings.
class CallbackFailure : public std::runtime_error
{
public:
    CallbackFailure(StopReason reason, const std::string &message)
        : std::runtime_error(message), reason(reason) {}
    const StopReason reason;
};

// Owns no node or executor. Owners must keep captured resources alive until callbacks finish.
class CallbackGuard
{
public:
    using Action = std::function<void()>;
    using Reporter = std::function<void(std::exception_ptr)>;
    CallbackGuard(std::shared_ptr<StopControl> control, Action cancel, Action fallback,
                  Reporter report)
        : control_(std::move(control)), cancel_(std::move(cancel)),
          fallback_(std::move(fallback)), report_(std::move(report))
    {
        if (!control_ || !cancel_ || !fallback_ || !report_)
            throw std::invalid_argument("CallbackGuard requires control and all handlers");
    }

    template<class Function>
    void run(Function &&function) noexcept
    {
        StopReason reason = StopReason::CallbackError;
        run(std::forward<Function>(function), reason);
    }

    // The tracking callback may refine its reason before unwinding to this boundary.
    template<class Function>
    void run(Function &&function, const StopReason &reason) noexcept
    {
        try { function(); }
        catch (const CallbackFailure &error) { fail(error.reason, std::current_exception()); }
        catch (...) { fail(reason, std::current_exception()); }
    }

    // Also used for normal idle cancellation after its stop reason has been published.
    void cancel() noexcept
    {
        try { cancel_(); }
        catch (...)
        {
            const auto error = std::current_exception();
            save(StopReason::CancelError, error);
            try { fallback_(); }
            catch (...) { save(StopReason::ShutdownError, std::current_exception()); }
            report(error);
        }
    }

private:
    void save(StopReason reason, std::exception_ptr error) noexcept
    {
        try { control_->record_failure(reason, error); }
        catch (...) { /* Still attempt cancellation if control storage itself fails. */ }
    }
    void report(std::exception_ptr error) noexcept
    {
        try { report_(error); }
        catch (...) { save(StopReason::CallbackError, std::current_exception()); }
    }
    void fail(StopReason reason, std::exception_ptr error) noexcept
    {
        save(reason, error);
        cancel();
        report(error);
    }
    std::shared_ptr<StopControl> control_;
    Action cancel_;
    Action fallback_;
    Reporter report_;
};
}
