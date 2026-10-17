#pragma once

#include <array>
#include <cstdint>

namespace gemini336_orbslam3
{
// Project-owned copies keep Core/Eigen/backend pointers out of the adapter API.
enum class FinalTrajectoryStatus { Complete, NoMap, NoKeyFrames, NoFramePoses };
enum class FinalTrajectoryPoseTarget { LeftCamera, ImuBody };

inline const char *final_trajectory_status_name(FinalTrajectoryStatus status) noexcept
{
    switch (status)
    {
    case FinalTrajectoryStatus::Complete: return "complete";
    case FinalTrajectoryStatus::NoMap: return "no_map";
    case FinalTrajectoryStatus::NoKeyFrames: return "no_keyframes";
    case FinalTrajectoryStatus::NoFramePoses: return "no_frame_poses";
    }
    return "unknown";
}

// T_a_b maps points from b to a. Metres and quaternion order x,y,z,w.
struct FinalTrajectoryTransform
{
    std::array<double, 3> translation{};
    std::array<double, 4> quaternion_xyzw{{0.0, 0.0, 0.0, 1.0}};
};

struct FinalTrajectoryFileSummary
{
    uint64_t considered = 0;
    uint64_t written = 0;
    uint64_t lost = 0;
    uint64_t invalid_reference = 0;
    uint64_t other_map = 0;
    uint64_t invalid_pose = 0;
    uint64_t invalid_timestamp = 0;
    // Original Core double seconds, not exact original integer sensor ns.
    double first_timestamp_sec = 0.0;
    double last_timestamp_sec = 0.0;
};

struct FinalTrajectoryExportResult
{
    FinalTrajectoryStatus status = FinalTrajectoryStatus::NoMap;
    bool map_selected = false;
    uint64_t map_id = 0;
    uint64_t map_keyframes = 0;
    uint64_t origin_keyframe_id = 0;
    double origin_timestamp_sec = 0.0;
    FinalTrajectoryPoseTarget pose_target = FinalTrajectoryPoseTarget::LeftCamera;
    bool imu_initialized = false;
    bool body_from_camera_valid = false;
    FinalTrajectoryTransform body_from_camera;
    // Frame rows: T_origin_sensor. Keyframe rows: T_map_sensor.
    FinalTrajectoryTransform frame_origin_in_map;
    FinalTrajectoryFileSummary frames;
    FinalTrajectoryFileSummary keyframes;
};
}
