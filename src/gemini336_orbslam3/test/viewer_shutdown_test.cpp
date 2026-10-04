#include "ViewerState.h"
#include "common/viewer_stop_notification.hpp"
#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

using ORB_SLAM3::ViewerState;
using namespace gemini336_orbslam3;
using namespace std::chrono_literals;

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

int main()
{
    try
    {
        // Pre-launch finish must not be erased, and finished is published only
        // after cleanup, rather than when the finish request is first issued.
        ViewerState beforeLaunch;
        beforeLaunch.RequestFinish();
        require(!beforeLaunch.Start(), "pre-launch finish was lost");
        require(!beforeLaunch.IsFinished(), "finish request claimed cleanup completion");
        beforeLaunch.Finish();
        beforeLaunch.Release();
        require(beforeLaunch.IsFinished(), "Release revived a finished viewer");

        // Reset may publish pause before Run starts. The real loop calls Pause
        // before any map/frame read, so startup must preserve this request.
        ViewerState resetBeforeLaunch;
        resetBeforeLaunch.RequestStop();
        require(resetBeforeLaunch.IsStopped(), "dormant reset did not acquire pause");
        require(resetBeforeLaunch.Start(), "pause incorrectly prevented launch");
        require(resetBeforeLaunch.Pause(), "launch lost reset pause");
        resetBeforeLaunch.Release();
        require(!resetBeforeLaunch.Pause(), "reset release did not resume viewer");

        // A paused worker exits on finish without needing Tracking to Release it.
        ViewerState paused;
        require(paused.Start(), "worker did not start");
        paused.RequestStop();
        std::promise<void> reachedPause;
        std::future<void> reached = reachedPause.get_future();
        std::future<void> worker = std::async(std::launch::async, [&]() {
            require(paused.Pause(), "worker did not acknowledge reset pause");
            reachedPause.set_value();
            while (!paused.FinishRequested()) std::this_thread::yield();
            paused.Finish();
        });
        require(reached.wait_for(1s) == std::future_status::ready, "pause acknowledgement timed out");
        paused.RequestFinish();
        require(worker.wait_for(1s) == std::future_status::ready, "paused finish timed out");
        worker.get();
        require(paused.IsFinished(), "worker exited without publishing finished");

        // Background failure remains observable after terminal cleanup.
        ViewerState failed;
        const auto original = std::make_exception_ptr(std::runtime_error("viewer failed"));
        failed.RecordFailure(original);
        failed.RecordFailure(std::make_exception_ptr(std::logic_error("cleanup failed")));
        failed.Finish();
        bool preserved = false;
        try { failed.RethrowFailure(); }
        catch (const std::runtime_error &error) { preserved = std::string(error.what()) == "viewer failed"; }
        require(preserved, "cleanup replaced the original viewer failure");

        // The notification closes backend admission even before an executor can
        // receive cancel. Its persistent gate remains available when spin starts.
        const auto control = std::make_shared<StopControl>();
        int cancels = 0;
        ViewerStopNotification notification(control, [&]() { ++cancels; }, []() {});
        notification({});
        require(!control->try_begin_backend(), "Viewer stop admitted new tracking");
        require(cancels == 1, "Viewer stop did not notify executor");
        notification(original);
        const auto snapshot = control->snapshot();
        require(snapshot.first_stop->reason == StopReason::ViewerStop, "first stop was overwritten");
        require(snapshot.first_failure->reason == StopReason::ViewerError, "Viewer error was lost");
        require(snapshot.first_exception == original, "notification lost original exception");

        // Cancellation failure must stay inside the thread boundary and attempt
        // context fallback without requiring a live node or logger.
        const auto cancelControl = std::make_shared<StopControl>();
        bool fallback = false;
        ViewerStopNotification brokenCancel(cancelControl,
            []() { throw std::runtime_error("cancel failed"); },
            [&]() { fallback = true; });
        brokenCancel({});
        require(fallback, "cancel failure did not attempt fallback");
        require(cancelControl->snapshot().first_failure->reason == StopReason::CancelError,
                "cancel failure was not recorded");
        std::cout << "Viewer coordination and notification tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
