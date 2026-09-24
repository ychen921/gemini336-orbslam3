#include "common/stop_control.hpp"

#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{
using namespace gemini336_orbslam3;

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

void ordered_stop_and_start(bool stop_first)
{
    StopControl control;
    std::promise<void> first_done;
    std::future<void> ready = first_done.get_future();
    bool granted = false;
    std::thread first([&]() {
        if (stop_first) control.request_stop(StopReason::InputIdle);
        else granted = control.try_begin_backend();
        first_done.set_value();
    });
    std::thread second([&]() {
        ready.wait();
        if (stop_first) granted = control.try_begin_backend();
        else control.request_stop(StopReason::ContextShutdown);
    });
    first.join();
    second.join();
    const StopSnapshot snapshot = control.snapshot();
    require(granted == !stop_first && snapshot.backend_starts == (stop_first ? 0u : 1u),
            "start permission crossed the stop boundary incorrectly");
    require(control.stop_requested() && !control.try_begin_backend(), "stop did not close gate");
    require(snapshot.first_stop.has_value() && !snapshot.first_failure, "normal stop became failure");
}

void failure_preservation()
{
    StopControl control;
    control.request_stop(StopReason::Signal);
    const StopEvent first = *control.snapshot().first_stop;
    control.record_failure(StopReason::BackendError);
    const std::exception_ptr exception = std::make_exception_ptr(std::runtime_error("first"));
    control.record_failure(StopReason::CallbackError, exception);
    control.record_failure(StopReason::TraceWriteError,
                           std::make_exception_ptr(std::runtime_error("later")), true);
    const StopSnapshot snapshot = control.snapshot();
    require(snapshot.first_stop->reason == first.reason && snapshot.first_stop->time == first.time,
            "first stop was overwritten");
    require(snapshot.first_failure && snapshot.first_failure->reason == StopReason::BackendError &&
            snapshot.first_exception == exception && snapshot.cleanup_failed,
            "failure escalation or first exception preservation failed");
    require(!control.try_begin_backend(), "failure reopened gate");
}

void competing_requests()
{
    StopControl control;
    std::atomic<bool> go{false};
    std::vector<std::thread> workers;
    for (int i = 0; i < 4; ++i)
    {
        workers.emplace_back([&, i]() {
            while (!go.load()) std::this_thread::yield();
            for (int j = 0; j < 1000; ++j)
            {
                if (i % 2) control.record_failure(StopReason::CallbackError);
                else control.request_stop(StopReason::InputIdle);
            }
        });
    }
    bool coherent = true;
    go.store(true);
    // Contend one tracking consumer against stop and read consistent snapshots.
    for (int i = 0; i < 1000; ++i)
    {
        control.try_begin_backend();
        const StopSnapshot snapshot = control.snapshot();
        if (snapshot.stop_requested != snapshot.first_stop.has_value() ||
            (snapshot.first_failure && !snapshot.stop_requested)) coherent = false;
    }
    for (auto &worker : workers) worker.join();
    const StopSnapshot snapshot = control.snapshot();
    require(coherent && snapshot.first_stop && snapshot.first_failure &&
            !control.try_begin_backend(), "concurrent stop snapshot inconsistent");
    require(control.snapshot().backend_starts == snapshot.backend_starts,
            "backend admitted after stop");
}
}

int main()
{
    try
    {
        ordered_stop_and_start(true);
        ordered_stop_and_start(false);
        failure_preservation();
        competing_requests();
        std::cout << "Stop control ordering and failure tests passed\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
