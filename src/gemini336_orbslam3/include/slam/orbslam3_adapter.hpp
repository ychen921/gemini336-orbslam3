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

// Intended for one SLAM instance lasting until standalone process teardown.
// Callers must serialize all operations and stop input before shutdown.
// Upstream shutdown does not guarantee thread completion or full resource cleanup;
// repeated construction/destruction and dynamic unloading are not supported.
class OrbSlam3Adapter
{
public:
    explicit OrbSlam3Adapter(const OrbSlam3Config &config);
    ~OrbSlam3Adapter() noexcept;

    OrbSlam3Adapter(const OrbSlam3Adapter &) = delete;
    OrbSlam3Adapter &operator=(const OrbSlam3Adapter &) = delete;
    OrbSlam3Adapter(OrbSlam3Adapter &&) = delete;
    OrbSlam3Adapter &operator=(OrbSlam3Adapter &&) = delete;

    // Definition deferred to the stereo tracking implementation step.
    void track(const StereoFrame &frame);
    // Idempotent after a successful return; this is not a thread-join guarantee.
    void shutdown();

private:
    std::unique_ptr<ORB_SLAM3::System> slam_;
    bool shutdown_called_ = false;
};
}
