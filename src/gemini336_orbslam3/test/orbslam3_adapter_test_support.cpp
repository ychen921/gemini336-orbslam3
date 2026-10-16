#include "slam/orbslam3_adapter.hpp"

#include <System.h>
#include <utility>

namespace gemini336_orbslam3
{
// Complete upstream type is required for unique_ptr cleanup during construction.
OrbSlam3Adapter::OrbSlam3Adapter(TestTag, TrackingMode mode, std::function<void()> shutdown,
                               bool enable_statistics, bool enable_pose)
    : test_shutdown_(std::move(shutdown)), tracking_mode_(mode)
{
    tracking_statistics_.enabled = enable_statistics;
    tracking_statistics_.pose_enabled = enable_pose;
    backend_summary_.enabled = enable_statistics;
}
}
