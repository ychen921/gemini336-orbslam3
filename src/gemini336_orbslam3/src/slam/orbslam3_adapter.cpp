#include "slam/orbslam3_adapter.hpp"

#include <System.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace gemini336_orbslam3
{
namespace
{
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
}

OrbSlam3Adapter::OrbSlam3Adapter(const OrbSlam3Config &config)
{
    require_readable_file(config.vocabulary_path, "Vocabulary");
    require_readable_file(config.settings_path, "Settings");
    validate_settings(config.settings_path);

    // Upstream may still exit on invalid vocabulary contents or missing parameters.
    slam_ = std::make_unique<ORB_SLAM3::System>(
        config.vocabulary_path, config.settings_path, ORB_SLAM3::System::STEREO, false);
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

void OrbSlam3Adapter::track(const StereoFrame &frame)
{
    if (shutdown_called_)
        throw std::logic_error("Cannot track after ORB-SLAM3 shutdown");

    // Validate the input frame before calling upstream.
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
    if (last_timestamp_ && frame.timestamp <= *last_timestamp_)
        throw std::invalid_argument("Stereo timestamps must be strictly increasing");

    slam_->TrackStereo(frame.left, frame.right, frame.timestamp);
    // Only a normally returning upstream call advances the accepted timestamp.
    // A lost/uninitialized tracking state is not an input error.
    last_timestamp_ = frame.timestamp;
    // Cache the state only after tracking: upstream's initial state field is not initialized.
    switch (slam_->GetTrackingState())
    {
    case ORB_SLAM3::Tracking::SYSTEM_NOT_READY: tracking_state_ = TrackingState::SystemNotReady; break;
    case ORB_SLAM3::Tracking::NO_IMAGES_YET: tracking_state_ = TrackingState::NoImagesYet; break;
    case ORB_SLAM3::Tracking::NOT_INITIALIZED: tracking_state_ = TrackingState::NotInitialized; break;
    case ORB_SLAM3::Tracking::OK: tracking_state_ = TrackingState::Ok; break;
    case ORB_SLAM3::Tracking::RECENTLY_LOST: tracking_state_ = TrackingState::RecentlyLost; break;
    case ORB_SLAM3::Tracking::LOST: tracking_state_ = TrackingState::Lost; break;
    case ORB_SLAM3::Tracking::OK_KLT: tracking_state_ = TrackingState::OkKlt; break;
    default: tracking_state_ = TrackingState::Unknown; break;
    }
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
