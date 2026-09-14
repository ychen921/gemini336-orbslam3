#pragma once

#include <memory>
#include <string>

#include "common/types.hpp"

namespace ORB_SLAM3
{
class System;
}

namespace gemini336_orbslam3
{
struct OrbSlam3Config
{
    std::string vocabulary_path;
    std::string settings_path;
};

// Interface skeleton: lifecycle and tracking definitions belong to later steps.
class OrbSlam3Adapter
{
public:
    explicit OrbSlam3Adapter(const OrbSlam3Config &config);
    ~OrbSlam3Adapter();

    OrbSlam3Adapter(const OrbSlam3Adapter &) = delete;
    OrbSlam3Adapter &operator=(const OrbSlam3Adapter &) = delete;
    OrbSlam3Adapter(OrbSlam3Adapter &&) = delete;
    OrbSlam3Adapter &operator=(OrbSlam3Adapter &&) = delete;

    void track(const StereoFrame &frame);
    void shutdown();

private:
    std::unique_ptr<ORB_SLAM3::System> slam_;
};
}
