#include "slam/orbslam3_adapter.hpp"
#include <atomic>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace gemini336_orbslam3
{
struct OrbSlam3AdapterTestAccess
{
    static std::unique_ptr<OrbSlam3Adapter> make(TrackingMode mode, std::function<void()> shutdown)
    {
        return std::unique_ptr<OrbSlam3Adapter>(new OrbSlam3Adapter(
            OrbSlam3Adapter::TestTag{}, mode, std::move(shutdown)));
    }
    static bool returned(const OrbSlam3Adapter &adapter)
    {
        return adapter.shutdown_state_ == OrbSlam3Adapter::ShutdownState::Returned;
    }
    static void set_request(OrbSlam3Adapter &adapter, std::function<void()> request)
    {
        adapter.test_request_shutdown_ = std::move(request);
    }
};
}
using namespace gemini336_orbslam3;
void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}
void require_tracking_closed(OrbSlam3Adapter &adapter)
{
    for (bool imu : {false, true})
    {
        bool rejected = false;
        try { if (imu) adapter.track(StereoFrame{}, {}); else adapter.track(StereoFrame{}); }
        catch (const std::logic_error &error)
        {
            rejected = std::string(error.what()) == "Cannot track after ORB-SLAM3 shutdown";
        }
        require(rejected, "tracking entered validation or backend after shutdown began");
    }
}
int main()
{
    try
    {
        // A request from another thread publishes state without invoking blocking
        // shutdown or rejecting a previously admitted tracking call at the adapter.
        for (const auto mode : {TrackingMode::Stereo, TrackingMode::StereoImu})
        {
            int shutdown_calls = 0;
            std::atomic<bool> requested{false};
            auto adapter = OrbSlam3AdapterTestAccess::make(mode, [&]() { ++shutdown_calls; });
            OrbSlam3AdapterTestAccess::set_request(*adapter, [&]() noexcept { requested.store(true); });
            std::thread requester([&]() {
                adapter->request_shutdown();
                adapter->request_shutdown();
            });
            requester.join();
            require(requested.load(), "request was not forwarded");
            require(shutdown_calls == 0, "request invoked blocking shutdown");
            bool reached_validation = false;
            try
            {
                if (mode == TrackingMode::Stereo) adapter->track(StereoFrame{});
                else adapter->track(StereoFrame{}, {});
            }
            catch (const std::invalid_argument &) { reached_validation = true; }
            require(reached_validation, "request changed adapter tracking admission");
            require(adapter->trackingState() == TrackingState::NoImagesYet, "request changed cached state");
            // Reset the test observer only, to verify explicit shutdown also publishes.
            requested.store(false);
            adapter->shutdown();
            require(requested.load(), "explicit shutdown did not publish a request");
            adapter->shutdown();
            require(shutdown_calls == 1, "request changed explicit shutdown attempts");
            require_tracking_closed(*adapter);
        }
        for (const auto mode : {TrackingMode::Stereo, TrackingMode::StereoImu})
        for (int outcome : {0, 1, 2})
        {
            int calls = 0;
            OrbSlam3Adapter *active = nullptr;
            auto adapter = OrbSlam3AdapterTestAccess::make(mode, [&]() {
                ++calls;
                require_tracking_closed(*active);
                bool reentrant_rejected = false;
                try { active->shutdown(); }
                catch (const std::logic_error &) { reentrant_rejected = true; }
                require(reentrant_rejected, "reentrant shutdown was accepted");
                if (outcome == 1) throw std::runtime_error("original shutdown failure");
                if (outcome == 2) throw 42;
            });
            active = adapter.get();
            for (int attempt = 0; attempt < 2; ++attempt)
            {
                bool failed = false;
                try { adapter->shutdown(); }
                catch (const std::runtime_error &error)
                {
                    require(outcome == 1 && std::string(error.what()) == "original shutdown failure",
                            "shutdown replaced the standard exception");
                    failed = true;
                }
                catch (int value) { require(outcome == 2 && value == 42, "unknown exception changed"); failed = true; }
                require(failed == (outcome != 0), "shutdown failure was hidden");
                require_tracking_closed(*adapter);
                require(OrbSlam3AdapterTestAccess::returned(*adapter) == (outcome == 0),
                        "attempted shutdown was reported as returned");
                require(adapter->trackingState() == TrackingState::NoImagesYet, "cached state changed");
            }
            adapter.reset();
            require(calls == 1, "explicit retry or destructor called backend twice");
        }
        for (bool fails : {false, true})
        {
            int calls = 0;
            {
                auto adapter = OrbSlam3AdapterTestAccess::make(TrackingMode::Stereo, [&]() {
                    ++calls;
                    if (fails) throw 42;
                });
            }
            require(calls == 1, "destructor did not make one initial attempt");
        }
        std::cout << "Adapter shutdown tests passed\n";
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
