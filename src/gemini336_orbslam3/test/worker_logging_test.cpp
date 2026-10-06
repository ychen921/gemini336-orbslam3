#include "gemini336_orbslam3/logging.hpp"
#include <Tracking.h>
#include <LocalMapping.h>
#include <LoopClosing.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

// Exercise real initialization/reset diagnostics without System, images,
// sensors, GUI or optimization. Protected access is limited to fixture setup.
class TestTracking : public ORB_SLAM3::Tracking
{
public:
  TestTracking(
    ORB_SLAM3::Atlas & atlas, const std::string & settings,
    std::shared_ptr<spdlog::logger> logger = {})
  : Tracking(nullptr, nullptr, nullptr, nullptr, &atlas, nullptr, settings,
      ORB_SLAM3::System::MONOCULAR, nullptr, "logging_test", std::move(logger)) {}
  const std::shared_ptr<spdlog::logger> & logger() const {return mLogger;}
};

class TestLocalMapping : public ORB_SLAM3::LocalMapping
{
public:
  explicit TestLocalMapping(std::shared_ptr<spdlog::logger> logger = {})
  : LocalMapping(nullptr, nullptr, false, false, "logging_test", std::move(logger)) {}
  const std::shared_ptr<spdlog::logger> & logger() const {return mLogger;}
  // Prepare an idle live-worker state without starting its processing loop.
  void prepare_idle() {mbFinished = false;}
};

class TestLoopClosing : public ORB_SLAM3::LoopClosing
{
public:
  explicit TestLoopClosing(std::shared_ptr<spdlog::logger> logger = {})
  : LoopClosing(nullptr, nullptr, nullptr, true, false, {}, std::move(logger)) {}
  const std::shared_ptr<spdlog::logger> & logger() const {return mLogger;}
  void reset()
  {
    mbResetRequested = true;
    ResetIfRequested();
    require(!mbResetRequested, "Loop reset did not complete");
  }
  void observations()
  {
    std::set<ORB_SLAM3::KeyFrame *> empty;
    CheckObservations(empty, empty);
  }
  void finite_gba()
  {
    // Use the GBA thread owner with a finite fake job, never an optimizer.
    require(
      StartGlobalBundleAdjustment(
        [this](std::uint64_t) {
          ORB_SLAM3::Log(mLogger, spdlog::level::info, "GBA_LOGGER_TEST");
        }), "Finite GBA job was rejected");
    JoinGlobalBundleAdjustment(false);
  }
};

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream file(path);
  require(file.good(), "Cannot read worker log");
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
} // namespace

int main()
{
  const auto root = std::filesystem::temp_directory_path() /
    ("worker_logging_test_" + std::to_string(::getpid()));
  try {
    std::filesystem::create_directories(root);
    const auto settings = root / "camera.yaml";
    {
      std::ofstream file(settings);
      file << "%YAML:1.0\nCamera.type: PinHole\n"
        "Camera.fx: 400.0\nCamera.fy: 400.0\nCamera.cx: 320.0\nCamera.cy: 240.0\n"
        "Camera.k1: 0.0\nCamera.k2: 0.0\nCamera.p1: 0.0\nCamera.p2: 0.0\n"
        "Camera.fps: 30.0\nCamera.RGB: 1\n"
        "ORBextractor.nFeatures: 64\nORBextractor.scaleFactor: 1.2\n"
        "ORBextractor.nLevels: 4\nORBextractor.iniThFAST: 20\nORBextractor.minThFAST: 7\n";
      require(file.good(), "Cannot write camera fixture");
    }
    gemini336_orbslam3::LoggingOptions options;
    options.directory = root / "logs";
    gemini336_orbslam3::LoggingSession session(options);
    const auto tracking_logger = session.GetLogger("tracking");
    const auto mapping_logger = session.GetLogger("local_mapping");
    const auto loop_logger = session.GetLogger("loop_closing");
    const auto log_path = session.directory() / "slam.log";
    {
      ORB_SLAM3::Atlas atlas(0, session.GetLogger("atlas"), session.GetLogger("map"));
      TestTracking tracking(atlas, settings.string(), tracking_logger);
      require(tracking.logger() == tracking_logger, "Tracking replaced the injected logger");
      cv::FileStorage missing("%YAML:1.0\nunused: 1\n",
        cv::FileStorage::READ | cv::FileStorage::MEMORY);
      require(!tracking.ParseORBParamFile(missing), "Missing ORB parameters were accepted");
      TestLocalMapping mapper(mapping_logger);
      require(mapper.logger() == mapping_logger, "Mapper replaced the injected logger");
      mapper.prepare_idle();
      mapper.RequestStop();
      require(mapper.Stop(), "Mapper stop did not complete");
      mapper.Release();
      TestLoopClosing loop(loop_logger);
      require(loop.logger() == loop_logger, "Loop closer replaced the injected logger");
      loop.reset();
      loop.observations();       // DEBUG records must be filtered at INFO.
      loop.finite_gba();
      TestTracking fallback_tracking(atlas, settings.string());
      require(fallback_tracking.logger()->name() == "tracking", "Tracking fallback module changed");
      TestLocalMapping fallback_mapper;
      TestLoopClosing fallback_loop;
      require(
        fallback_mapper.logger()->name() == "local_mapping",
        "Mapper fallback module changed");
      require(fallback_loop.logger()->name() == "loop_closing", "Loop fallback module changed");
    }
    session.finish();
    const auto text = read_file(log_path);
    require(
      text.find(
        "[tracking] [info] - Camera: Pinhole") != std::string::npos, "Missing camera report");
    require(text.find(" is pinhole\n") != std::string::npos, "Camera type was lost");
    require(
      text.find(
        "[tracking] [info]  is pinhole") == std::string::npos,
      "Camera report split across records");
    require(
      text.find(
        "[tracking] [error] *ORBextractor.nFeatures parameter") != std::string::npos,
      "Missing parameter error");
    require(
      text.find(
        "[local_mapping] [info] Local Mapping STOP") != std::string::npos, "Missing mapper stop");
    require(
      text.find(
        "[local_mapping] [info] Local Mapping RELEASE") != std::string::npos,
      "Missing mapper release");
    require(
      text.find(
        "[loop_closing] [info] Loop closer reset requested...") != std::string::npos,
      "Missing loop reset");
    require(
      text.find(
        "[loop_closing] [info] GBA_LOGGER_TEST") != std::string::npos,
      "GBA did not share the loop logger");
    require(text.find("[debug]") == std::string::npos, "Debug diagnostics escaped INFO filtering");
    require(session.dropped_messages() == 0, "Worker records were dropped");
    std::filesystem::remove_all(root);
    std::cout << "worker_logging_test: PASS\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "worker_logging_test: " << error.what() << '\n';
    std::filesystem::remove_all(root);
    return 1;
  }
}
