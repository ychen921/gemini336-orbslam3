#pragma once

#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "common/types.hpp"

namespace spdlog
{
class logger;
}

namespace ORB_SLAM3
{
class System;
}

namespace gemini336_orbslam3
{
// Keep core headers out of this public adapter interface; the signature matches
// ORB_SLAM3::LoggerFactory. True requests synchronous write-and-flush semantics.
using OrbSlam3LoggerFactory =
    std::function<std::shared_ptr<spdlog::logger>(const std::string &, bool)>;

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
// Callers must serialize operations except request_shutdown(), and stop input before shutdown.
// Shutdown joins background threads. Complete core resource cleanup, repeated
// construction/destruction and dynamic unloading are not supported.
class OrbSlam3Adapter
{
public:
    // The optional stop query must be nonthrowing and thread-safe. It observes stop
    // state only, and its captured resources must outlive backend worker use.
    // Viewer notification must not block on shutdown or capture the owning node;
    // its captured resources must remain valid until the backend has joined Viewer.
    // Logger factory use is setup-only; its session must outlive backend producers.
    explicit OrbSlam3Adapter(const OrbSlam3Config &config,
                            std::function<bool()> external_stop_requested = {},
                            std::function<void(std::exception_ptr)> viewer_stop_notification = {},
                            OrbSlam3LoggerFactory logger_factory = {});
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

    // Publish only; may overlap track(), but never backend release/destruction.
    // Does not change tracking admission or perform a blocking shutdown.
    void request_shutdown() noexcept;

    // At most one upstream attempt. Success is idempotent; failure is rethrown on
    // later explicit calls without retrying upstream. Tracking is disabled at entry.
    // A normal return guarantees Viewer/mapping/loop/GBA thread completion.
    void shutdown();

private:
#ifdef GEMINI336_ADAPTER_TEST
    friend struct OrbSlam3AdapterTestAccess;
    struct TestTag {};
    OrbSlam3Adapter(TestTag, TrackingMode mode, std::function<void()> shutdown);
    std::function<void()> test_shutdown_;
    std::function<void()> test_request_shutdown_;
#endif
    // Sensor mode is fixed for the lifetime of this SLAM instance.
    const TrackingMode tracking_mode_;
    std::unique_ptr<ORB_SLAM3::System> slam_;

    // Cached state can be queried without accessing upstream after shutdown.
    enum class ShutdownState { NotStarted, Attempted, Returned };
    ShutdownState shutdown_state_ = ShutdownState::NotStarted;
    std::exception_ptr shutdown_failure_;
    std::optional<double> last_frame_timestamp_;
    std::optional<double> last_imu_timestamp_;
    TrackingState tracking_state_ = TrackingState::NoImagesYet;
};
}
