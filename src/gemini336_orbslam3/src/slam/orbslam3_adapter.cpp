#include "slam/orbslam3_adapter.hpp"

#include <System.h>
#include <ImuTypes.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gemini336_orbslam3
{
namespace
{
FinalTrajectoryFileSummary convert_trajectory_file_summary(const ORB_SLAM3::TrajectoryFileSummary &core)
{
    return {core.considered, core.written, core.lost, core.invalid_reference,
            core.other_map, core.invalid_pose, core.invalid_timestamp,
            core.first_timestamp_sec, core.last_timestamp_sec};
}

FinalTrajectoryExportResult convert_trajectory_export(const ORB_SLAM3::TrajectoryExportResult &core)
{
    FinalTrajectoryExportResult result;
    switch (core.status)
    {
    case ORB_SLAM3::TrajectoryExportStatus::Complete: result.status = FinalTrajectoryStatus::Complete; break;
    case ORB_SLAM3::TrajectoryExportStatus::NoMap: result.status = FinalTrajectoryStatus::NoMap; break;
    case ORB_SLAM3::TrajectoryExportStatus::NoKeyFrames: result.status = FinalTrajectoryStatus::NoKeyFrames; break;
    case ORB_SLAM3::TrajectoryExportStatus::NoFramePoses: result.status = FinalTrajectoryStatus::NoFramePoses; break;
    default: throw std::invalid_argument("Unknown Core trajectory export status");
    }
    switch (core.pose_target)
    {
    case ORB_SLAM3::TrajectoryPoseTarget::LeftCamera:
        result.pose_target = FinalTrajectoryPoseTarget::LeftCamera; break;
    case ORB_SLAM3::TrajectoryPoseTarget::ImuBody:
        result.pose_target = FinalTrajectoryPoseTarget::ImuBody; break;
    default: throw std::invalid_argument("Unknown Core trajectory pose target");
    }
    result.map_selected = core.map_selected;
    result.map_id = core.map_id;
    result.map_keyframes = core.map_keyframes;
    result.origin_keyframe_id = core.origin_keyframe_id;
    result.origin_timestamp_sec = core.origin_timestamp_sec;
    result.imu_initialized = core.imu_initialized;
    result.body_from_camera_valid = core.body_from_camera_valid;
    result.body_from_camera = {core.body_from_camera.translation, core.body_from_camera.quaternion_xyzw};
    result.frame_origin_in_map = {core.frame_origin_in_map.translation, core.frame_origin_in_map.quaternion_xyzw};
    result.frames = convert_trajectory_file_summary(core.frames);
    result.keyframes = convert_trajectory_file_summary(core.keyframes);
    return result;
}

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

OrbSlam3Adapter::OrbSlam3Adapter(const OrbSlam3Config &config,
                               std::function<bool()> external_stop_requested,
                               std::function<void(std::exception_ptr)> viewer_stop_notification,
                               OrbSlam3LoggerFactory logger_factory)
    : tracking_mode_(config.tracking_mode)
{
    if (config.enable_tracking_pose && !config.enable_tracking_statistics)
        throw std::invalid_argument("Tracking pose collection requires tracking statistics");
    // Check file readability and atlas restrictions before upstream starts its worker threads.
    require_readable_file(config.vocabulary_path, "Vocabulary");
    require_readable_file(config.settings_path, "Settings");
    validate_settings(config.settings_path);

    const ORB_SLAM3::System::eSensor sensor = toOrbSensor(tracking_mode_);
    tracking_statistics_.enabled = config.enable_tracking_statistics;
    tracking_statistics_.pose_enabled = config.enable_tracking_pose;
    backend_summary_.enabled = config.enable_tracking_statistics;

    // Upstream may still exit on invalid vocabulary contents or missing parameters.
    slam_ = std::make_unique<ORB_SLAM3::System>(
        config.vocabulary_path, config.settings_path, sensor,
        config.enable_viewer, 0, std::string{}, std::move(external_stop_requested),
        std::move(viewer_stop_notification), std::move(logger_factory), config.enable_tracking_statistics,
        config.enable_tracking_pose);
}

OrbSlam3Adapter::~OrbSlam3Adapter() noexcept
{
    // Never retry an upstream shutdown whose outcome is already known or uncertain.
    if (shutdown_state_ != ShutdownState::NotStarted) return;
    // Best effort only when the owner never explicitly attempted shutdown.
    try
    {
        shutdown();
    }
    catch (const std::exception &error)
    {
        try { std::cerr << "ORB-SLAM3 adapter shutdown failed: " << error.what() << std::endl; }
        catch (...) {}
    }
    catch (...)
    {
        try { std::cerr << "ORB-SLAM3 adapter shutdown failed with a non-standard exception" << std::endl; }
        catch (...) {}
    }
}

// Stereo only
void OrbSlam3Adapter::track(const StereoFrame &frame)
{
    if (shutdown_state_ != ShutdownState::NotStarted)
        throw std::logic_error("Cannot track after ORB-SLAM3 shutdown");

    // Reject the wrong overload before forwarding any sensor data.
    if (tracking_mode_ != TrackingMode::Stereo)
        throw std::logic_error("track(frame) requires Stereo mode");

    // Validate the input frame before calling upstream.
    validateStereoFrame(frame, last_frame_timestamp_);

#ifdef GEMINI336_ADAPTER_TEST
    if (test_track_) test_track_();
    else
#endif
        slam_->TrackStereo(frame.left, frame.right, frame.timestamp);

    // Only a normally returning upstream call advances the accepted timestamp.
    // A lost/uninitialized tracking state is not an input error.
    last_frame_timestamp_ = frame.timestamp;

    // Cache the state only after tracking: upstream's initial state field is not initialized.
    cache_tracking_statistics();
}

void OrbSlam3Adapter::track(
    const StereoFrame &frame,
    const std::vector<ImuMeasurement> &imu)
{
    if (shutdown_state_ != ShutdownState::NotStarted)
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

#ifdef GEMINI336_ADAPTER_TEST
    if (test_track_) test_track_();
    else
#endif
        slam_->TrackStereo(frame.left, frame.right, frame.timestamp, convertImu(imu));

    // Only a normally returning upstream call advances the accepted timestamps.
    // A lost/uninitialized tracking state is not an input error.
    last_frame_timestamp_ = frame.timestamp;
    if (!imu.empty())
        last_imu_timestamp_ = imu.back().timestamp;

    // Cache the state only after tracking: upstream's initial state field is not initialized.
    cache_tracking_statistics();
}

void OrbSlam3Adapter::cache_tracking_statistics()
{
    TrackingStatistics snapshot;
    snapshot.enabled = tracking_statistics_.enabled;
#ifdef GEMINI336_ADAPTER_TEST
    if (test_statistics_) snapshot = test_statistics_();
    else
#endif
    if (snapshot.enabled)
    {
        // Replaces GetTrackingState: one existing state-mutex acquisition per call.
        snapshot = convert_tracking_snapshot(slam_->GetTrackingSnapshot());
    }
    else
        snapshot.raw_tracking_state = slam_->GetTrackingState();
    tracking_statistics_ = snapshot;
    raw_tracking_state_ = snapshot.raw_tracking_state;
    tracking_state_ = toTrackingState(raw_tracking_state_);
}

TrackingStatistics OrbSlam3Adapter::convert_tracking_snapshot(const ORB_SLAM3::TrackingSnapshot &core) noexcept
{
    TrackingStatistics snapshot;
    snapshot.enabled = core.enabled;
    snapshot.has_frame = core.has_frame;
    snapshot.raw_tracking_state = core.raw_tracking_state;
    snapshot.core_frame_id = core.frame_id;
    snapshot.map_id_valid = core.map_id_valid;
    snapshot.tracking_map_id = core.tracking_map_id;
    snapshot.tracked_mappoint_inliers_valid = core.tracked_mappoint_inliers_valid;
    snapshot.tracked_mappoint_inliers = core.tracked_mappoint_inliers;
    snapshot.keyframes_inserted_total = core.keyframes_inserted_total;
    snapshot.tracking_lost_events = core.tracking_lost_events;
    snapshot.relocalization_events = core.relocalization_events;
    snapshot.pose_enabled = core.pose_enabled;

    // Context can be available during startup without a valid tracking pose.
    if (!core.enabled || !core.has_frame || !core.pose_enabled || !core.map_id_valid) return snapshot;
    snapshot.map_change_index_valid = core.map_change_index_valid;
    if (core.map_change_index_valid) snapshot.map_change_index = core.map_change_index;
    snapshot.imu_initialized_valid = core.imu_initialized_valid;
    if (core.imu_initialized_valid) snapshot.imu_initialized = core.imu_initialized;
    if (core.pose_capture_steady_ns > 0) snapshot.pose_capture_steady_ns = core.pose_capture_steady_ns;
    if (!core.pose_valid || snapshot.pose_capture_steady_ns == 0 ||
        (core.raw_tracking_state != ORB_SLAM3::Tracking::OK &&
         core.raw_tracking_state != ORB_SLAM3::Tracking::OK_KLT))
        return snapshot;

    const Eigen::Vector3d translation_cw(core.pose_translation_cw[0],
        core.pose_translation_cw[1], core.pose_translation_cw[2]);
    const Eigen::Quaterniond rotation_cw(core.pose_quaternion_cw_xyzw[3],
        core.pose_quaternion_cw_xyzw[0], core.pose_quaternion_cw_xyzw[1], core.pose_quaternion_cw_xyzw[2]);
    if (!translation_cw.allFinite() || !rotation_cw.coeffs().allFinite() ||
        std::abs(rotation_cw.squaredNorm() - 1.0) > 1e-4)
        return snapshot;

    // Normalize only the diagnostic copy to remove float roundoff. Twc's
    // translation is -Rcw^T * tcw, not simply -tcw. Fixed-size Eigen uses no heap.
    const Eigen::Quaterniond rotation_wc = rotation_cw.normalized().conjugate();
    const Eigen::Vector3d translation_wc = -(rotation_wc * translation_cw);
    if (!translation_wc.allFinite()) return snapshot;
    snapshot.pose_translation_wc = {{translation_wc.x(), translation_wc.y(), translation_wc.z()}};
    snapshot.pose_quaternion_wc_xyzw = {{rotation_wc.x(), rotation_wc.y(), rotation_wc.z(), rotation_wc.w()}};
    snapshot.pose_valid = true;
    return snapshot;
}

TrackingState OrbSlam3Adapter::trackingState() const noexcept
{
    return tracking_state_;
}

int OrbSlam3Adapter::rawTrackingState() const noexcept
{
    return raw_tracking_state_;
}

TrackingStatistics OrbSlam3Adapter::trackingStatistics() const noexcept
{
    return tracking_statistics_;
}

EvaluationBackendSummary OrbSlam3Adapter::backendSummary() const noexcept
{
    return backend_summary_;
}

void OrbSlam3Adapter::request_shutdown() noexcept
{
    // The backend pointer is stable until serialized teardown; do not touch cached
    // tracking or shutdown state from this concurrent request path.
#ifdef GEMINI336_ADAPTER_TEST
    if (test_request_shutdown_) { test_request_shutdown_(); return; }
#endif
    if (slam_) slam_->RequestShutdown();
}

void OrbSlam3Adapter::shutdown()
{
    if (shutdown_state_ == ShutdownState::Returned) return;
    if (shutdown_state_ == ShutdownState::Attempted)
    {
        if (shutdown_failure_) std::rethrow_exception(shutdown_failure_);
        throw std::logic_error("ORB-SLAM3 shutdown is already in progress");
    }

    // Close tracking before calling upstream, including on an exceptional return.
    shutdown_state_ = ShutdownState::Attempted;
    request_shutdown();
    try
    {
#ifdef GEMINI336_ADAPTER_TEST
        if (test_shutdown_) test_shutdown_();
        else slam_->Shutdown();
#else
        slam_->Shutdown();
#endif
        // Final totals include asynchronous insertion/events after the last frame.
        // A throwing join leaves available=false, without retrying any backend work.
        if (backend_summary_.enabled)
        {
#ifdef GEMINI336_ADAPTER_TEST
            if (test_backend_summary_) backend_summary_ = test_backend_summary_();
            else
#endif
            {
                const ORB_SLAM3::EvaluationEventTotals totals = slam_->GetEvaluationEventTotals();
                static_assert(ORB_SLAM3::EvaluationEventNames.size() == EvaluationEventNames.size(),
                              "Core and wrapper event schemas must agree");
                backend_summary_.keyframes_inserted_total = slam_->GetInsertedKeyFramesTotal();
                backend_summary_.event_counts = totals.counts;
                backend_summary_.available = totals.enabled;
            }
        }
        // All backend worker threads have joined on normal return.
        shutdown_state_ = ShutdownState::Returned;
    }
    catch (...)
    {
        shutdown_failure_ = std::current_exception();
        throw;
    }
}

FinalTrajectoryExportResult OrbSlam3Adapter::exportFinalTrajectories(
    const std::filesystem::path &frames, const std::filesystem::path &keyframes)
{
    if (shutdown_state_ != ShutdownState::Returned)
        throw std::logic_error("Final trajectory export requires successful adapter shutdown");
    // This path never calls Shutdown and never changes its lifecycle state.
#ifdef GEMINI336_ADAPTER_TEST
    if (test_final_trajectory_export_)
        return convert_trajectory_export(test_final_trajectory_export_(frames.string(), keyframes.string()));
#endif
    return convert_trajectory_export(slam_->SaveEvaluationTrajectories(frames.string(), keyframes.string()));
}
}
