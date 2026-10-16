#include "slam/orbslam3_adapter.hpp"

#include <System.h>
#include <utility>

namespace gemini336_orbslam3
{
// Complete upstream type is required for unique_ptr cleanup during construction.
OrbSlam3Adapter::OrbSlam3Adapter(TestTag, TrackingMode mode, std::function<void()> shutdown,
                               bool enable_statistics)
    : test_shutdown_(std::move(shutdown)), tracking_mode_(mode)
{
    tracking_statistics_.enabled = enable_statistics;
    backend_summary_.enabled = enable_statistics;
}
}
