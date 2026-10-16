#pragma once

#include <array>
#include <cstdint>

namespace gemini336_orbslam3
{
// Backend-independent values only; recording stays free of ROS/core headers.
struct TrackingStatistics
{
    bool enabled = false;
    bool has_frame = false;
    bool map_id_valid = false;
    bool tracked_mappoint_inliers_valid = false;
    int32_t raw_tracking_state = 0;
    uint32_t tracked_mappoint_inliers = 0;
    uint32_t tracking_lost_events = 0;
    uint32_t relocalization_events = 0;
    uint64_t core_frame_id = 0;
    uint64_t tracking_map_id = 0;
    uint64_t keyframes_inserted_total = 0;
};

inline constexpr std::array<const char *, 8> EvaluationEventNames{{
    "map_baseline", "tracking_lost", "relocalized", "map_created", "map_switched",
    "active_map_reset", "atlas_reset", "merge_applied"}};

struct EvaluationBackendSummary
{
    bool enabled = false;
    bool available = false; // True only after successful worker joins.
    uint64_t keyframes_inserted_total = 0;
    std::array<uint64_t, EvaluationEventNames.size()> event_counts{};
};
}
