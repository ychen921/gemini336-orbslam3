#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <mutex>
#include <optional>

namespace gemini336_orbslam3
{
enum class StopReason
{
    Signal,
    ContextShutdown,
    InputIdle,
    Capacity,
    Timeout,
    SamplingError,
    BackendError,
    CallbackError,
    CancelError,
    TraceWriteError,
    ShutdownError
};

struct StopEvent
{
    StopReason reason;
    std::chrono::steady_clock::time_point time;
};

struct StopSnapshot
{
    bool stop_requested = false;
    std::optional<StopEvent> first_stop;
    std::optional<StopEvent> first_failure;
    std::exception_ptr first_exception;
    bool cleanup_failed = false;
    uint64_t backend_starts = 0;
};

// Own independently of the node so a future context callback can use a weak_ptr.
// No logging, executor calls or data-domain locks belong inside this object.
class StopControl
{
public:
    bool stop_requested() const noexcept
    {
        return stop_requested_.load();
    }

    // The first cause is immutable; repeated requests still leave the gate closed.
    void request_stop(StopReason reason)
    {
        const std::lock_guard<std::mutex> lock(control_mutex_);
        if (!first_stop_)
            first_stop_ = StopEvent{reason, std::chrono::steady_clock::now()};
        stop_requested_.store(true);
    }

    // Failure also requests stop, even if normal shutdown was requested earlier.
    // Capture the first exception independently: the first failure may have none.
    void record_failure(StopReason reason, std::exception_ptr exception = {},
                        bool cleanup_failure = false)
    {
        const std::lock_guard<std::mutex> lock(control_mutex_);
        const StopEvent event{reason, std::chrono::steady_clock::now()};
        if (!first_stop_) first_stop_ = event;
        if (!first_failure_) first_failure_ = event;
        if (!first_exception_ && exception) first_exception_ = exception;
        cleanup_failed_ = cleanup_failed_ || cleanup_failure;
        stop_requested_.store(true);
    }

    // This decision is the logical backend start, serialized with stop requests.
    // A granted call may execute after stop and must not acquire permission twice.
    // Only the tracking consumer calls this; it still owns backend serialization.
    bool try_begin_backend()
    {
        const std::lock_guard<std::mutex> lock(control_mutex_);
        if (stop_requested_.load()) return false;
        ++backend_starts_;
        return true;
    }

    StopSnapshot snapshot() const
    {
        const std::lock_guard<std::mutex> lock(control_mutex_);
        return {stop_requested_.load(), first_stop_, first_failure_, first_exception_,
                cleanup_failed_, backend_starts_};
    }

private:
    mutable std::mutex control_mutex_;
    std::atomic<bool> stop_requested_{false};
    std::optional<StopEvent> first_stop_;
    std::optional<StopEvent> first_failure_;
    std::exception_ptr first_exception_;
    bool cleanup_failed_ = false;
    uint64_t backend_starts_ = 0;
};
}
