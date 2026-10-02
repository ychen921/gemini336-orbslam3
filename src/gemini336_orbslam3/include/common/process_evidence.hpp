#pragma once
#include "common/stop_control.hpp"
#include <cstdio>
#include <memory>
#include <optional>

namespace gemini336_orbslam3
{
struct LoggingEvidence
{
    const char *status = "unavailable";
    std::optional<std::size_t> dropped;
};

inline const char *stop_reason_name(StopReason reason) noexcept
{
    switch (reason)
    {
    case StopReason::Signal: return "Signal";
    case StopReason::ContextShutdown: return "ContextShutdown";
    case StopReason::InputIdle: return "InputIdle";
    case StopReason::Capacity: return "Capacity";
    case StopReason::Timeout: return "Timeout";
    case StopReason::SamplingError: return "SamplingError";
    case StopReason::BackendError: return "BackendError";
    case StopReason::CallbackError: return "CallbackError";
    case StopReason::CancelError: return "CancelError";
    case StopReason::TraceWriteError: return "TraceWriteError";
    case StopReason::ShutdownError: return "ShutdownError";
    case StopReason::Finalization: return "Finalization";
    }
    return "unknown";
}

// Only main, after every cleanup step including logging finish/release. Never use a logger here.
// The externally captured process status remains authoritative if this final write fails.
inline int write_process_evidence(FILE *output, const std::shared_ptr<StopControl> &control,
                                  const LoggingEvidence &logging, int result) noexcept
{
    try
    {
        if (!control) return result ? result : 1;
        const StopSnapshot state = control->snapshot();
        char dropped[32] = "unknown";
        if (logging.dropped) std::snprintf(dropped, sizeof(dropped), "%zu", *logging.dropped);
        const int written = std::fprintf(output,
            "STOP_PROCESS schema_version=1 first_stop=%s first_failure=%s exception_present=%s "
            "cleanup_failed=%s backend_starts=%llu logging_finish_status=%s logging_dropped_messages=%s exit_code=%d\n",
            state.first_stop ? stop_reason_name(state.first_stop->reason) : "none",
            state.first_failure ? stop_reason_name(state.first_failure->reason) : "none",
            state.first_exception ? "true" : "false", state.cleanup_failed ? "true" : "false",
            static_cast<unsigned long long>(state.backend_starts), logging.status, dropped, result);
        if (written < 0 || std::fflush(output) != 0) return 1;
    }
    catch (...) { return 1; }
    return result;
}
}
