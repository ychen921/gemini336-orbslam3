#pragma once
#include "common/stop_control.hpp"
#include <functional>
#include <memory>

namespace gemini336_orbslam3
{
struct ProcessCleanup
{
    std::function<void()> finalize_node;
    std::function<void()> detach_context;
    std::function<void()> release_executor;
    std::function<void()> release_node;
    std::function<void()> shutdown_context;
    std::function<void()> finish_logging;
    std::function<void()> release_logging;
};

// Called only before spin starts or after normal spin return has joined all workers.
// Exception unwinding alone is insufficient. This helper does not establish quiescence.
inline int finalize_process(const std::shared_ptr<StopControl> &control,
                            std::exception_ptr startup_failure,
                            const ProcessCleanup &cleanup,
                            const std::function<void(std::exception_ptr)> &report) noexcept
{
    bool failed = false;
    const auto record = [&](std::exception_ptr error, bool cleaning) noexcept {
        failed = true;
        try { if (control) control->record_failure(cleaning ? StopReason::ShutdownError :
                StopReason::CallbackError, error, cleaning); }
        catch (...) {}
    };
    const auto diagnose = [&](std::exception_ptr error) noexcept {
        try { report(error); }
        catch (...) { record(std::current_exception(), true); }
    };
    if (startup_failure)
    {
        record(startup_failure, false);
        diagnose(startup_failure);
    }
    // Every independent cleanup runs even when an earlier one failed.
    for (const auto *step : {&cleanup.finalize_node, &cleanup.detach_context,
                            &cleanup.release_executor, &cleanup.release_node,
                            &cleanup.shutdown_context, &cleanup.finish_logging,
                            &cleanup.release_logging})
    {
        try { (*step)(); }
        catch (...)
        {
            const auto error = std::current_exception();
            record(error, true);
            diagnose(error);
        }
    }
    try
    {
        if (control)
        {
            const auto state = control->snapshot();
            failed = failed || state.first_failure.has_value() || state.cleanup_failed;
        }
    }
    catch (...) { failed = true; }
    return failed ? 1 : 0;
}
}
