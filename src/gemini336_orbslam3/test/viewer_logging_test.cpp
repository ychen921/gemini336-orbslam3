#include "gemini336_orbslam3/logging.hpp"
#include <Viewer.h>
#include <MapDrawer.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <unistd.h>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

const std::vector<std::pair<std::string, std::string>> parameters = {
  {"Camera.fps", "30.0"}, {"Camera.width", "640.0"}, {"Camera.height", "480.0"},
  {"Viewer.ViewpointX", "0.0"}, {"Viewer.ViewpointY", "-0.7"},
  {"Viewer.ViewpointZ", "-1.8"}, {"Viewer.ViewpointF", "500.0"},
  {"Viewer.KeyFrameSize", "0.05"}, {"Viewer.KeyFrameLineWidth", "1.0"},
  {"Viewer.GraphLineWidth", "0.9"}, {"Viewer.PointSize", "2.0"},
  {"Viewer.CameraSize", "0.08"}, {"Viewer.CameraLineWidth", "3.0"}
};

void write_settings(const std::filesystem::path & path, const std::string & omitted = {})
{
  std::ofstream file(path);
  file << "%YAML:1.0\n";
  for (const auto & parameter : parameters) {
    if (parameter.first != omitted) {
      file << parameter.first << ": " << parameter.second << '\n';
    }
  }
  require(file.good(), "Cannot write viewer configuration fixture");
}

// Construct only: these tests never call Run or any OpenGL drawing function.
void construct(
  bool viewer, const std::filesystem::path & path,
  std::shared_ptr<spdlog::logger> logger = {})
{
  if (viewer) {
    ORB_SLAM3::Viewer instance(nullptr, nullptr, nullptr, nullptr, path.string(), nullptr, logger);
  } else {
    ORB_SLAM3::MapDrawer instance(nullptr, path.string(), nullptr, logger);
  }
}

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream file(path);
  require(file.good(), "Cannot read viewer test log");
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
}  // namespace

int main()
{
  const auto root = std::filesystem::temp_directory_path() /
    ("viewer_logging_test_" + std::to_string(::getpid()));
  try {
    std::filesystem::create_directories(root);
    const auto settings = root / "viewer.yaml";
    gemini336_orbslam3::LoggingOptions options;
    options.directory = root / "logs";
    gemini336_orbslam3::LoggingSession session(options);
    const auto log_path = session.directory() / "slam.log";
    write_settings(settings);
    construct(true, settings, session.GetLogger("viewer"));
    construct(false, settings, session.GetLogger("map_drawer"));
    construct(true, settings);
    construct(false, settings);

    const std::vector<std::string> viewer_parameters = {
      "Camera.width", "Camera.height", "Viewer.ViewpointX", "Viewer.ViewpointY",
      "Viewer.ViewpointZ", "Viewer.ViewpointF"
    };
    const std::vector<std::string> drawer_parameters = {
      "Viewer.KeyFrameSize", "Viewer.KeyFrameLineWidth", "Viewer.GraphLineWidth",
      "Viewer.PointSize", "Viewer.CameraSize", "Viewer.CameraLineWidth"
    };
    for (bool viewer : {true, false}) {
      const auto logger = session.GetLogger(viewer ? "viewer" : "map_drawer");
      for (const auto & parameter : viewer ? viewer_parameters : drawer_parameters) {
        write_settings(settings, parameter);
        bool original_exception = false;
        try {
          construct(viewer, settings, logger);
        } catch (int error) {
          original_exception = error == -1;
        }
        require(original_exception, "Invalid configuration changed its original exception");
      }
    }
    session.finish();
    const std::string text = read_file(log_path);
    for (bool viewer : {true, false}) {
      const std::string prefix = viewer ? "[viewer] [error] " : "[map_drawer] [error] ";
      for (const auto & parameter : viewer ? viewer_parameters : drawer_parameters) {
        const std::string expected = prefix + "*" + parameter +
          " parameter doesn't exist or is not a real number*";
        require(text.find(expected) != std::string::npos, "Missing configuration error record");
      }
      require(
        text.find(prefix + "**ERROR in the config file, the format is not correct**") !=
        std::string::npos, "Missing original configuration summary");
    }
    require(session.dropped_messages() == 0, "Configuration error records were dropped");
    std::filesystem::remove_all(root);
    std::cout << "viewer_logging_test: PASS\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "viewer_logging_test: " << error.what() << '\n';
    std::filesystem::remove_all(root);
    return 1;
  }
}
