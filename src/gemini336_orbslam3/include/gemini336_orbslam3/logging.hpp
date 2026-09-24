#pragma once

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>

#include <spdlog/logger.h>

namespace gemini336_orbslam3
{
struct LoggingOptions
{
    // Empty selects GEMINI336_SLAM_LOG_DIR, then the build-time workspace/log_slam.
    std::filesystem::path directory;
    std::string level = "info";
    std::size_t queue_capacity = 8192;
};

// Owns a private backend; producers must stop before session destruction.
// Returned loggers must not be used after the session has been destroyed.
class LoggingSession
{
public:
    explicit LoggingSession(const LoggingOptions &options = {});
    ~LoggingSession();
    LoggingSession(const LoggingSession &) = delete;
    LoggingSession &operator=(const LoggingSession &) = delete;

    std::shared_ptr<spdlog::logger> GetLogger(const std::string &module_name);
    const std::filesystem::path &directory() const;
    std::size_t dropped_messages() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace gemini336_orbslam3
