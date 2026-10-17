#include "slam/orbslam3_adapter.hpp"
#include <System.h>

#include <iostream>
#include <stdexcept>
#include <utility>

namespace gemini336_orbslam3
{
struct OrbSlam3AdapterTestAccess
{
    static std::unique_ptr<OrbSlam3Adapter> make(TrackingMode mode, std::function<void()> shutdown,
        std::function<ORB_SLAM3::TrajectoryExportResult(const std::string &, const std::string &)> exporter)
    {
        auto adapter = std::unique_ptr<OrbSlam3Adapter>(new OrbSlam3Adapter(
            OrbSlam3Adapter::TestTag{}, mode, std::move(shutdown)));
        adapter->test_final_trajectory_export_ = std::move(exporter);
        return adapter;
    }
};
}

using namespace gemini336_orbslam3;

namespace
{
void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}

void test_success(TrackingMode mode)
{
    int joins = 0;
    int exports = 0;
    bool fail_export = false;
    ORB_SLAM3::TrajectoryExportResult core;
    core.status = ORB_SLAM3::TrajectoryExportStatus::Complete;
    core.map_selected = true;
    core.map_id = 0;
    core.map_keyframes = 42;
    core.origin_keyframe_id = 0;
    core.origin_timestamp_sec = 123.25;
    core.pose_target = mode == TrackingMode::Stereo ? ORB_SLAM3::TrajectoryPoseTarget::LeftCamera :
                                                    ORB_SLAM3::TrajectoryPoseTarget::ImuBody;
    core.imu_initialized = mode == TrackingMode::StereoImu;
    core.body_from_camera_valid = mode == TrackingMode::StereoImu;
    core.body_from_camera.translation = {{0.1, -0.2, 0.3}};
    core.frame_origin_in_map.translation = {{1, 2, 3}};
    core.frames = {15, 5, 1, 2, 3, 1, 3, 123.25, 124.5};
    core.keyframes = {42, 40, 0, 0, 0, 1, 1, 123.25, 124.25};
    auto adapter = OrbSlam3AdapterTestAccess::make(mode, [&]() { ++joins; },
        [&](const std::string &frames, const std::string &keyframes) {
            require(joins == 1 && frames == "/session/frames.txt" && keyframes == "/session/keyframes.txt",
                    "Export preceded shutdown or changed caller paths");
            ++exports;
            if (fail_export) throw std::runtime_error("original export failure");
            return core;
        });
    bool rejected = false;
    try { adapter->exportFinalTrajectories("/session/frames.txt", "/session/keyframes.txt"); }
    catch (const std::logic_error &) { rejected = true; }
    require(rejected && exports == 0 && joins == 0, "Export was admitted before shutdown");
    adapter->shutdown();
    const auto result = adapter->exportFinalTrajectories("/session/frames.txt", "/session/keyframes.txt");
    require(result.status == FinalTrajectoryStatus::Complete && result.map_selected &&
            result.map_id == 0 && result.origin_keyframe_id == 0 && result.map_keyframes == 42 &&
            result.origin_timestamp_sec == 123.25 && result.imu_initialized == core.imu_initialized &&
            result.body_from_camera_valid == core.body_from_camera_valid &&
            result.pose_target == (mode == TrackingMode::Stereo ? FinalTrajectoryPoseTarget::LeftCamera :
                                                                 FinalTrajectoryPoseTarget::ImuBody),
            "Core identity, status or sensor target was not preserved");
    require(result.body_from_camera.translation == core.body_from_camera.translation &&
            result.body_from_camera.quaternion_xyzw == core.body_from_camera.quaternion_xyzw &&
            result.frame_origin_in_map.translation == core.frame_origin_in_map.translation &&
            result.frame_origin_in_map.quaternion_xyzw == core.frame_origin_in_map.quaternion_xyzw &&
            result.frames.considered == 15 && result.frames.written == 5 && result.frames.lost == 1 &&
            result.frames.invalid_reference == 2 && result.frames.other_map == 3 &&
            result.frames.invalid_pose == 1 && result.frames.invalid_timestamp == 3 &&
            result.frames.first_timestamp_sec == 123.25 && result.frames.last_timestamp_sec == 124.5 &&
            result.keyframes.considered == 42 && result.keyframes.written == 40 &&
            result.keyframes.invalid_pose == 1 && result.keyframes.invalid_timestamp == 1 &&
            result.keyframes.first_timestamp_sec == 123.25 && result.keyframes.last_timestamp_sec == 124.25,
            "Coordinate metadata or row summaries changed during conversion");

    // An export failure must leave shutdown success intact, including destruction.
    fail_export = true;
    bool failed = false;
    try { adapter->exportFinalTrajectories("/session/frames.txt", "/session/keyframes.txt"); }
    catch (const std::runtime_error &error) { failed = std::string(error.what()) == "original export failure"; }
    require(failed && exports == 2, "Export failure was hidden or replaced");
    adapter->shutdown();
    adapter.reset();
    require(joins == 1 && exports == 2, "Export failure retried backend shutdown or export");
}

void test_status_and_failed_shutdown()
{
    const ORB_SLAM3::TrajectoryExportStatus statuses[] = {
        ORB_SLAM3::TrajectoryExportStatus::NoMap, ORB_SLAM3::TrajectoryExportStatus::NoKeyFrames,
        ORB_SLAM3::TrajectoryExportStatus::NoFramePoses};
    const FinalTrajectoryStatus expected[] = {
        FinalTrajectoryStatus::NoMap, FinalTrajectoryStatus::NoKeyFrames, FinalTrajectoryStatus::NoFramePoses};
    for (std::size_t index = 0; index < 3; ++index)
    {
        auto adapter = OrbSlam3AdapterTestAccess::make(TrackingMode::Stereo, []() {},
            [&](const std::string &, const std::string &) {
                ORB_SLAM3::TrajectoryExportResult result;
                result.status = statuses[index];
                return result;
            });
        adapter->shutdown();
        require(adapter->exportFinalTrajectories("frames", "keyframes").status == expected[index],
                "No-data status was converted to a successful export");
    }
    int joins = 0;
    int exports = 0;
    auto adapter = OrbSlam3AdapterTestAccess::make(TrackingMode::Stereo,
        [&]() { ++joins; throw std::runtime_error("join failed"); },
        [&](const std::string &, const std::string &) { ++exports; return ORB_SLAM3::TrajectoryExportResult{}; });
    try { adapter->shutdown(); }
    catch (const std::runtime_error &) {}
    bool rejected = false;
    try { adapter->exportFinalTrajectories("frames", "keyframes"); }
    catch (const std::logic_error &) { rejected = true; }
    adapter.reset();
    require(rejected && joins == 1 && exports == 0, "Failed shutdown admitted export or retried join");
}
}

int main()
{
    try
    {
        test_success(TrackingMode::Stereo);
        test_success(TrackingMode::StereoImu);
        test_status_and_failed_shutdown();
        std::cout << "Adapter trajectory export tests passed\n";
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
