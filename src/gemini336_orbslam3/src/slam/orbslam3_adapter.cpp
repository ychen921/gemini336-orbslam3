#include "slam/orbslam3_adapter.hpp"

#include <System.h>

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

void OrbSlam3Adapter::shutdown()
{
    if (shutdown_called_)
        return;

    slam_->Shutdown();
    // Records only that the call returned, not that every worker has stopped.
    shutdown_called_ = true;
}
}
