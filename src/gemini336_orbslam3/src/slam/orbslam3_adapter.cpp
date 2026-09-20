#include "slam/orbslam3_adapter.hpp"

#include <System.h>
#include <ImuTypes.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace gemini336_orbslam3
{
namespace
{
// Keep upstream sensor types inside the adapter and reject unsupported mode values.
ORB_SLAM3::System::eSensor toOrbSensor(TrackingMode mode)
{
    switch (mode)
    {
    case TrackingMode::Stereo: return ORB_SLAM3::System::STEREO;
    case TrackingMode::StereoImu: return ORB_SLAM3::System::IMU_STEREO;
    default: throw std::invalid_argument("Unsupported ORB-SLAM3 mode");
    }
}

// Convert only: preserve sample order, units and timestamps, including empty input.
// The IMU tracking entry point validates inputs before calling this helper.
std::vector<ORB_SLAM3::IMU::Point> convertImu(
    const std::vector<ImuMeasurement> &measurements)
{
    // Each input produces one output; reserve avoids reallocations while appending.
    std::vector<ORB_SLAM3::IMU::Point> output;
    output.reserve(measurements.size());

    // The scalar constructor takes acceleration, angular velocity, then time.
    for (const ImuMeasurement &measurement : measurements)
    {
        const double timestamp = measurement.timestamp;

        const float accel_x = measurement.accel.x();
        const float accel_y = measurement.accel.y();
        const float accel_z = measurement.accel.z();

        const float gyro_x = measurement.gyro.x();
        const float gyro_y = measurement.gyro.y();
        const float gyro_z = measurement.gyro.z();

        output.emplace_back(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, timestamp);
    }

    return output;
}

void require_readable_file(const std::string &path, const char *name)
{
    if (path.empty())
        throw std::invalid_argument(std::string(name) + " path must not be empty");

    std::error_code error;
    if (!std::filesystem::is_regular_file(path, error) || !std::ifstream(path).good())
        throw std::invalid_argument(std::string(name) + " is not a readable file: " + path);
}

void validate_settings(const std::string &path)
{
    try
    {
        cv::FileStorage settings(path, cv::FileStorage::READ);
        if (!settings.isOpened())
            throw std::invalid_argument("Cannot open ORB-SLAM3 settings: " + path);

        // This adapter excludes atlas persistence from the standalone session lifecycle.
        for (const char *key : {"System.LoadAtlasFromFile", "System.SaveAtlasToFile"})
        {
            const cv::FileNode value = settings[key];
            if (!value.empty() && (!value.isString() || !value.string().empty()))
                throw std::invalid_argument(std::string(key) + " must be absent or an empty string");
        }
    }
    catch (const cv::Exception &error)
    {
        throw std::invalid_argument("Cannot parse ORB-SLAM3 settings '" + path + "': " + error.what());
    }
}

// Share image validation between both tracking modes without changing their state.
void validateStereoFrame(
    const StereoFrame &frame,
    const std::optional<double> &last_frame_timestamp)
{
    if (frame.left.empty() || frame.right.empty())
        throw std::invalid_argument("Stereo images must not be empty");
    if (frame.left.dims != 2 || frame.right.dims != 2)
        throw std::invalid_argument("Stereo images must be two-dimensional");
    if (frame.left.size() != frame.right.size())
        throw std::invalid_argument("Stereo image sizes must match");
    if (frame.left.type() != CV_8UC1 || frame.right.type() != CV_8UC1)
        throw std::invalid_argument("Stereo images must have type CV_8UC1");
    if (!std::isfinite(frame.timestamp))
        throw std::invalid_argument("Stereo timestamp must be finite");
    if (last_frame_timestamp && frame.timestamp <= *last_frame_timestamp)
        throw std::invalid_argument("Stereo timestamps must be strictly increasing");

}

void validateImuMeasurements(
    const std::vector<ImuMeasurement> &imu,
    const std::optional<double> &last_imu_timestamp,
    double frame_timestamp)
{
    // Check across batches first, then within this batch, without changing adapter state.
    // The caller has already validated the frame timestamp.
    std::optional<double> previous_timestamp = last_imu_timestamp;

    for (const ImuMeasurement &measurement : imu)
    {
        if (!std::isfinite(measurement.timestamp))
            throw std::invalid_argument("IMU timestamp must be finite");
        if (measurement.timestamp < 0.0)
            throw std::invalid_argument("IMU timestamp must be non-negative");
        if (measurement.timestamp > frame_timestamp)
            throw std::invalid_argument("IMU timestamp must not exceed the frame timestamp");
        if (previous_timestamp && measurement.timestamp <= *previous_timestamp)
            throw std::invalid_argument("IMU timestamps must be strictly increasing");
        if (!measurement.accel.allFinite())
            throw std::invalid_argument("IMU acceleration must be finite");
        if (!measurement.gyro.allFinite())
            throw std::invalid_argument("IMU angular velocity must be finite");

        previous_timestamp = measurement.timestamp;
    }
}

// Convert upstream state without accessing the backend or changing cached state.
TrackingState toTrackingState(int state)
{
    switch (state)
    {
    case ORB_SLAM3::Tracking::SYSTEM_NOT_READY: return TrackingState::SystemNotReady;
    case ORB_SLAM3::Tracking::NO_IMAGES_YET: return TrackingState::NoImagesYet;
    case ORB_SLAM3::Tracking::NOT_INITIALIZED: return TrackingState::NotInitialized;
    case ORB_SLAM3::Tracking::OK: return TrackingState::Ok;
    case ORB_SLAM3::Tracking::RECENTLY_LOST: return TrackingState::RecentlyLost;
    case ORB_SLAM3::Tracking::LOST: return TrackingState::Lost;
    case ORB_SLAM3::Tracking::OK_KLT: return TrackingState::OkKlt;
    default: return TrackingState::Unknown;
    }
}
}

OrbSlam3Adapter::OrbSlam3Adapter(const OrbSlam3Config &config)
    : tracking_mode_(config.tracking_mode)
{
    // Check file readability and atlas restrictions before upstream starts its worker threads.
    require_readable_file(config.vocabulary_path, "Vocabulary");
    require_readable_file(config.settings_path, "Settings");
    validate_settings(config.settings_path);

    const ORB_SLAM3::System::eSensor sensor = toOrbSensor(tracking_mode_);

    // Upstream may still exit on invalid vocabulary contents or missing parameters.
    slam_ = std::make_unique<ORB_SLAM3::System>(
        config.vocabulary_path, config.settings_path, sensor,
        config.enable_viewer);
}

OrbSlam3Adapter::~OrbSlam3Adapter() noexcept
{
    // Best effort at process teardown, retaining the upstream lifetime limitations.
    try
    {
        shutdown();
    }
    catch (const std::exception &error)
    {
        std::cerr << "ORB-SLAM3 adapter shutdown failed: " << error.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "ORB-SLAM3 adapter shutdown failed with a non-standard exception" << std::endl;
    }
}

// Stereo only
void OrbSlam3Adapter::track(const StereoFrame &frame)
{
    if (shutdown_called_)
        throw std::logic_error("Cannot track after ORB-SLAM3 shutdown");

    // Reject the wrong overload before forwarding any sensor data.
    if (tracking_mode_ != TrackingMode::Stereo)
        throw std::logic_error("track(frame) requires Stereo mode");

    // Validate the input frame before calling upstream.
    validateStereoFrame(frame, last_frame_timestamp_);

    slam_->TrackStereo(frame.left, frame.right, frame.timestamp);

    // Only a normally returning upstream call advances the accepted timestamp.
    // A lost/uninitialized tracking state is not an input error.
    last_frame_timestamp_ = frame.timestamp;

    // Cache the state only after tracking: upstream's initial state field is not initialized.
    tracking_state_ = toTrackingState(slam_->GetTrackingState());
}

void OrbSlam3Adapter::track(
    const StereoFrame &frame,
    const std::vector<ImuMeasurement> &imu)
{
    if (shutdown_called_)
        throw std::logic_error("Cannot track after ORB-SLAM3 shutdown");

    // Inertial input must not be silently ignored by a Stereo backend.
    if (tracking_mode_ != TrackingMode::StereoImu)
        throw std::logic_error("track(frame, imu) requires StereoImu mode");

    // Validate the input frame before calling upstream.
    validateStereoFrame(frame, last_frame_timestamp_);

    // The first frame establishes the image timeline; subsequent frames require
    // IMU input. Coverage and initialization readiness remain the caller's responsibility.
    if (last_frame_timestamp_ && imu.empty())
        throw std::invalid_argument("IMU measurements must not be empty after the first frame");

    // Validate the input IMU measurements before calling upstream.
    validateImuMeasurements(imu, last_imu_timestamp_, frame.timestamp);

    slam_->TrackStereo(frame.left, frame.right, frame.timestamp, convertImu(imu));

    // Only a normally returning upstream call advances the accepted timestamps.
    // A lost/uninitialized tracking state is not an input error.
    last_frame_timestamp_ = frame.timestamp;
    if (!imu.empty())
        last_imu_timestamp_ = imu.back().timestamp;

    // Cache the state only after tracking: upstream's initial state field is not initialized.
    tracking_state_ = toTrackingState(slam_->GetTrackingState());
}

TrackingState OrbSlam3Adapter::trackingState() const noexcept
{
    return tracking_state_;
}

void OrbSlam3Adapter::shutdown()
{
    if (shutdown_called_)
        return;

    slam_->Shutdown();

    // Records only that the call returned, not that every worker has stopped.
    shutdown_called_ = true;
}
}
