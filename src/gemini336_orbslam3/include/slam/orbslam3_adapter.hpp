#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "common/types.hpp"

namespace ORB_SLAM3
{
class System;
}

namespace gemini336_orbslam3
{
enum class TrackingState
{
    SystemNotReady,
    NoImagesYet,
    NotInitialized,
    Ok,
    RecentlyLost,
    Lost,
    OkKlt,
    Unknown
};

enum class TrackingMode
{
    Stereo,
    StereoImu
};

struct OrbSlam3Config
{
    std::string vocabulary_path;
    std::string settings_path;
    bool enable_viewer = false;
    TrackingMode tracking_mode = TrackingMode::Stereo;
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

    // Stereo mode only. Requires matching nonempty 2D MONO8 images and finite, increasing timestamps.
    // Calibration/rectification must match settings; calls are synchronous.
    // Upstream exceptions propagate and do not imply that retrying is safe.
    void track(const StereoFrame &frame);

    // StereoImu mode only; same image requirements as above. IMU values must be finite,
    // with nonnegative timestamps strictly increasing within/across successful calls
    // and no later than this frame. Empty IMU is allowed only for the first frame.
    // The caller supplies temporal coverage and calibration; validation does not
    // guarantee inertial initialization. Upstream exceptions are not safe to retry.
    void track(
        const StereoFrame &frame,
        const std::vector<ImuMeasurement> &imu_measurements);

    // Last normally returned frame state; safe before the first frame and after shutdown.
    TrackingState trackingState() const noexcept;

    // Idempotent after a successful return; this is not a thread-join guarantee.
    void shutdown();

private:
    // Sensor mode is fixed for the lifetime of this SLAM instance.
    const TrackingMode tracking_mode_;
    std::unique_ptr<ORB_SLAM3::System> slam_;

    // Cached state can be queried without accessing upstream after shutdown.
    bool shutdown_called_ = false;
    std::optional<double> last_frame_timestamp_;
    std::optional<double> last_imu_timestamp_;
    TrackingState tracking_state_ = TrackingState::NoImagesYet;
};
}
