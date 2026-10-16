#include "slam/orbslam3_adapter.hpp"
#include <TrackingSnapshot.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace gemini336_orbslam3
{
struct OrbSlam3AdapterTestAccess
{
    static std::unique_ptr<OrbSlam3Adapter> make(TrackingMode mode, bool enabled, bool pose_enabled = false)
    {
        return std::unique_ptr<OrbSlam3Adapter>(new OrbSlam3Adapter(
            OrbSlam3Adapter::TestTag{}, mode, []() {}, enabled, pose_enabled));
    }
    static TrackingStatistics convert(const ORB_SLAM3::TrackingSnapshot &core)
    {
        return OrbSlam3Adapter::convert_tracking_snapshot(core);
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

void exercise(TrackingMode mode, bool enabled, bool pose_enabled = false)
{
    auto adapter = OrbSlam3AdapterTestAccess::make(mode, enabled, pose_enabled);
    require(adapter->trackingStatistics().enabled == enabled &&
            adapter->trackingStatistics().pose_enabled == pose_enabled &&
            !adapter->trackingStatistics().pose_valid &&
            !adapter->trackingStatistics().has_frame && !adapter->backendSummary().available,
            "Initial cache fabricated a frame or final totals");
    unsigned samples = 0, joins = 0, finals = 0;
    bool throw_track = false;
    OrbSlam3AdapterTestAccess::hooks(*adapter,
        [&]() { if (throw_track) throw std::runtime_error("backend failure"); },
        [&]() {
            ++samples;
            ORB_SLAM3::TrackingSnapshot statistics;
            statistics.enabled = enabled;
            statistics.has_frame = enabled;
            statistics.raw_tracking_state = pose_enabled ? 2 : 0;
            statistics.frame_id = 17;
            statistics.map_id_valid = statistics.tracked_mappoint_inliers_valid = enabled;
            statistics.tracking_map_id = 42;
            statistics.tracked_mappoint_inliers = 80;
            statistics.keyframes_inserted_total = 3;
            statistics.tracking_lost_events = enabled ? 1 : 0;
            statistics.pose_enabled = statistics.pose_valid = pose_enabled;
            statistics.pose_capture_steady_ns = 123;
            statistics.map_change_index_valid = pose_enabled;
            statistics.map_change_index = 7;
            statistics.imu_initialized_valid = pose_enabled && mode == TrackingMode::StereoImu;
            statistics.pose_translation_cw = {{1.0f, -2.0f, 3.0f}};
            statistics.pose_quaternion_cw_xyzw = {{0.0f, 0.0f, 0.0f, 1.0f}};
            return OrbSlam3AdapterTestAccess::convert(statistics);
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
            adapter->rawTrackingState() == (pose_enabled ? 2 : 0) &&
            saved.pose_enabled == pose_enabled && saved.pose_valid == pose_enabled,
            "Normal return did not publish one coherent cache");
    if (pose_enabled)
        require(saved.pose_translation_wc == std::array<double, 3>{{-1.0, 2.0, -3.0}} &&
                saved.map_change_index_valid && saved.map_change_index == 7 &&
                saved.pose_capture_steady_ns == 123 &&
                saved.imu_initialized_valid == (mode == TrackingMode::StereoImu) && !saved.imu_initialized,
                "Normal return lost converted pose or initialization context");
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
    const TrackingStatistics after_failure = adapter->trackingStatistics();
    require(after_failure.pose_valid == saved.pose_valid &&
            after_failure.pose_translation_wc == saved.pose_translation_wc &&
            after_failure.pose_quaternion_wc_xyzw == saved.pose_quaternion_wc_xyzw &&
            after_failure.pose_capture_steady_ns == saved.pose_capture_steady_ns,
            "Throwing call changed the previously published pose");
    adapter->shutdown();
    adapter->shutdown();
    require(joins == 1 && finals == unsigned(enabled) && adapter->backendSummary().available == enabled,
            "Final counters ignored collection state or repeated shutdown");
    if (enabled)
        require(adapter->backendSummary().keyframes_inserted_total == 5 &&
                adapter->trackingStatistics().keyframes_inserted_total == 3,
                "After-join totals replaced the earlier per-frame sample");
}

void pose_conversion()
{
    ORB_SLAM3::TrackingSnapshot core;
    core.enabled = core.has_frame = core.map_id_valid = true;
    core.pose_enabled = core.pose_valid = core.map_change_index_valid = true;
    core.raw_tracking_state = 2;
    core.pose_capture_steady_ns = 123;
    core.pose_translation_cw = {{1.0f, 2.0f, 3.0f}};
    const float half_sqrt = std::sqrt(0.5f);
    core.pose_quaternion_cw_xyzw = {{0.0f, 0.0f, half_sqrt, half_sqrt}};
    const auto converted = OrbSlam3AdapterTestAccess::convert(core);
    require(converted.pose_valid && std::abs(converted.pose_translation_wc[0] + 2.0) < 1e-6 &&
            std::abs(converted.pose_translation_wc[1] - 1.0) < 1e-6 &&
            std::abs(converted.pose_translation_wc[2] + 3.0) < 1e-6 &&
            std::abs(converted.pose_quaternion_wc_xyzw[2] + half_sqrt) < 1e-6 &&
            std::abs(converted.pose_quaternion_wc_xyzw[3] - half_sqrt) < 1e-6,
            "Twc inversion ignored rotation or changed quaternion order");

    const auto require_invalid = [](const ORB_SLAM3::TrackingSnapshot &invalid) {
        const auto result = OrbSlam3AdapterTestAccess::convert(invalid);
        require(!result.pose_valid && result.pose_translation_wc == std::array<double, 3>{} &&
                result.pose_quaternion_wc_xyzw == std::array<double, 4>{},
                "Invalid or disabled Core pose leaked numeric coordinates");
    };
    for (int condition = 0; condition < 6; ++condition)
    {
        auto invalid = core;
        switch (condition)
        {
        case 0: invalid.pose_enabled = false; break;
        case 1: invalid.pose_valid = false; break;
        case 2: invalid.has_frame = false; break;
        case 3: invalid.map_id_valid = false; break;
        case 4: invalid.pose_capture_steady_ns = 0; break;
        case 5: invalid.raw_tracking_state = 3; break;
        }
        require_invalid(invalid);
    }
    for (const float value : {std::numeric_limits<float>::quiet_NaN(),
                              std::numeric_limits<float>::infinity()})
    {
        auto invalid = core;
        invalid.pose_translation_cw[0] = value;
        require_invalid(invalid);
        invalid = core;
        invalid.pose_quaternion_cw_xyzw[0] = value;
        require_invalid(invalid);
    }
    auto invalid_rotation = core;
    invalid_rotation.pose_quaternion_cw_xyzw = {};
    require_invalid(invalid_rotation);
    invalid_rotation.pose_quaternion_cw_xyzw[3] = 2.0f;
    require_invalid(invalid_rotation);

    core.pose_translation_cw = {};
    core.pose_quaternion_cw_xyzw = {{0.0f, 0.0f, 0.0f, 1.0f}};
    const auto origin = OrbSlam3AdapterTestAccess::convert(core);
    require(origin.pose_valid && origin.pose_translation_wc == std::array<double, 3>{} &&
            origin.pose_quaternion_wc_xyzw[3] == 1.0, "Valid origin was treated as unavailable");

    core.pose_valid = false;
    core.imu_initialized_valid = true;
    const auto startup = OrbSlam3AdapterTestAccess::convert(core);
    require(!startup.pose_valid && startup.map_change_index_valid && startup.imu_initialized_valid &&
            !startup.imu_initialized && startup.pose_capture_steady_ns == 123,
            "Invalid startup pose discarded independently available map/IMU context");
}

void pose_requires_statistics()
{
    OrbSlam3Config config;
    config.enable_tracking_pose = true;
    bool rejected = false;
    try { OrbSlam3Adapter invalid(config); }
    catch (const std::invalid_argument &error)
    {
        rejected = std::string_view(error.what()) == "Tracking pose collection requires tracking statistics";
    }
    require(rejected, "Pose without evaluation reached resource checks or backend construction");
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
        {
            for (const bool enabled : {false, true}) exercise(mode, enabled);
            exercise(mode, true, true);
        }
        pose_conversion();
        pose_requires_statistics();
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
