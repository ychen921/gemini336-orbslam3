#include "slam/orbslam3_adapter.hpp"
#include <TrackingSnapshot.h>

#include <iostream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace gemini336_orbslam3
{
struct OrbSlam3AdapterTestAccess
{
    static std::unique_ptr<OrbSlam3Adapter> make(TrackingMode mode, bool enabled)
    {
        return std::unique_ptr<OrbSlam3Adapter>(new OrbSlam3Adapter(
            OrbSlam3Adapter::TestTag{}, mode, []() {}, enabled));
    }
    static void hooks(OrbSlam3Adapter &adapter, std::function<void()> track,
                      std::function<TrackingStatistics()> statistics,
                      std::function<void()> shutdown,
                      std::function<EvaluationBackendSummary()> final)
    {
        adapter.test_track_ = std::move(track);
        adapter.test_statistics_ = std::move(statistics);
        adapter.test_shutdown_ = std::move(shutdown);
        adapter.test_backend_summary_ = std::move(final);
    }
};
}

namespace
{
using namespace gemini336_orbslam3;

void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}

void exercise(TrackingMode mode, bool enabled)
{
    auto adapter = OrbSlam3AdapterTestAccess::make(mode, enabled);
    require(adapter->trackingStatistics().enabled == enabled &&
            !adapter->trackingStatistics().has_frame && !adapter->backendSummary().available,
            "Initial cache fabricated a frame or final totals");
    unsigned samples = 0, joins = 0, finals = 0;
    bool throw_track = false;
    OrbSlam3AdapterTestAccess::hooks(*adapter,
        [&]() { if (throw_track) throw std::runtime_error("backend failure"); },
        [&]() {
            ++samples;
            TrackingStatistics statistics;
            statistics.enabled = enabled;
            statistics.has_frame = enabled;
            statistics.raw_tracking_state = 0; // Hidden LOST followed by new map.
            statistics.core_frame_id = 17;
            statistics.map_id_valid = statistics.tracked_mappoint_inliers_valid = enabled;
            statistics.tracking_map_id = 42;
            statistics.tracked_mappoint_inliers = 80;
            statistics.keyframes_inserted_total = 3;
            statistics.tracking_lost_events = enabled ? 1 : 0;
            return statistics;
        },
        [&]() { ++joins; },
        [&]() {
            require(joins == 1, "Final counters were queried before join");
            ++finals;
            EvaluationBackendSummary summary;
            summary.enabled = summary.available = true;
            summary.keyframes_inserted_total = 5;
            summary.event_counts[1] = 1;
            return summary;
        });
    StereoFrame frame;
    frame.left = cv::Mat(2, 2, CV_8UC1, cv::Scalar(0));
    frame.right = frame.left.clone();
    frame.timestamp = 1.0;
    frame.timestamp_ns = 1000000000;
    if (mode == TrackingMode::Stereo) adapter->track(frame);
    else adapter->track(frame, {});
    const TrackingStatistics saved = adapter->trackingStatistics();
    require(samples == 1 && saved.has_frame == enabled && saved.tracking_lost_events == unsigned(enabled) &&
            adapter->rawTrackingState() == 0, "Normal return did not publish one coherent cache");
    throw_track = true;
    frame.timestamp = 2.0;
    frame.timestamp_ns = 2000000000;
    ImuMeasurement imu;
    imu.timestamp = 2.0;
    bool failed = false;
    try
    {
        if (mode == TrackingMode::Stereo) adapter->track(frame);
        else adapter->track(frame, {imu});
    }
    catch (const std::runtime_error &error) { failed = std::string_view(error.what()) == "backend failure"; }
    require(failed && samples == 1 && adapter->trackingStatistics().core_frame_id == saved.core_frame_id,
            "Throwing call queried statistics or replaced the last normal frame");
    adapter->shutdown();
    adapter->shutdown();
    require(joins == 1 && finals == unsigned(enabled) && adapter->backendSummary().available == enabled,
            "Final counters ignored collection state or repeated shutdown");
    if (enabled)
        require(adapter->backendSummary().keyframes_inserted_total == 5 &&
                adapter->trackingStatistics().keyframes_inserted_total == 3,
                "After-join totals replaced the earlier per-frame sample");
}

void failed_join()
{
    auto adapter = OrbSlam3AdapterTestAccess::make(TrackingMode::Stereo, true);
    unsigned joins = 0, finals = 0;
    OrbSlam3AdapterTestAccess::hooks(*adapter, {}, {},
        [&]() { ++joins; throw std::runtime_error("join failed"); },
        [&]() { ++finals; return EvaluationBackendSummary{}; });
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        bool failed = false;
        try { adapter->shutdown(); }
        catch (const std::runtime_error &) { failed = true; }
        require(failed, "Join failure was hidden");
    }
    require(joins == 1 && finals == 0 && !adapter->backendSummary().available,
            "Failed join read live counters, retried, or fabricated a final summary");
}
}

int main()
{
    using namespace gemini336_orbslam3;
    try
    {
        for (std::size_t index = 0; index < EvaluationEventNames.size(); ++index)
            require(std::string_view(EvaluationEventNames[index]) == ORB_SLAM3::EvaluationEventNames[index],
                    "Core and wrapper event names/order differ");
        for (const TrackingMode mode : {TrackingMode::Stereo, TrackingMode::StereoImu})
            for (const bool enabled : {false, true}) exercise(mode, enabled);
        failed_join();
        std::cout << "Adapter tracking statistics tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
