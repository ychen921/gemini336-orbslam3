#include "gemini336_orbslam3/logging.hpp"
#include <Optimizer.h>
#include <spdlog/sinks/base_sink.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

class ThrowingSink : public spdlog::sinks::base_sink<std::mutex>
{
  void sink_it_(const spdlog::details::log_msg &) override
  {
    throw std::runtime_error("injected optimizer log failure");
  }
  void flush_() override {}
};

// No cameras, images, worker launch or numerical optimization. A keyframe whose
// id differs from the map's initial id reaches the real zero-fixed-KF return.
void exercise_abort(std::shared_ptr<spdlog::logger> logger = {})
{
  ORB_SLAM3::Map map(0);
  ORB_SLAM3::KeyFrame keyframe;
  keyframe.mnId = 42;
  keyframe.UpdateMap(&map);
  int fixed = 17;
  int optimized = 17;
  int points = 17;
  int edges = 17;
  ORB_SLAM3::Optimizer::LocalBundleAdjustment(
    &keyframe, nullptr, &map, fixed, optimized, points, edges, std::move(logger));
  require(fixed == 0, "Local BA did not take its original abort path");
  require(
    optimized == 17 && points == 17 && edges == 17,
    "Logging changed counters on the abort path");
  require(keyframe.mnBALocalForKF == 42, "Original local KF marking changed");
}

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream file(path);
  require(file.good(), "Cannot read optimizer test log");
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
}  // namespace

int main()
{
  const auto root = std::filesystem::temp_directory_path() /
    ("optimizer_logging_test_" + std::to_string(::getpid()));
  try {
    gemini336_orbslam3::LoggingOptions options;
    options.directory = root;
    gemini336_orbslam3::LoggingSession session(options);
    const auto log_path = session.directory() / "slam.log";
    for (const std::string module : {"tracking", "local_mapping", "loop_closing"}) {
      exercise_abort(session.GetLogger(module));
    }
    const auto filtered = session.GetLogger("filtered");
    filtered->set_level(spdlog::level::err);
    exercise_abort(filtered);
    // The public default remains usable; fallback allocation is local to this call.
    exercise_abort();
    const auto failing = std::make_shared<spdlog::logger>(
      "failing", std::make_shared<ThrowingSink>());
    failing->set_error_handler(
      [](const std::string &) {
        throw std::runtime_error("injected error handler failure");
      });
    exercise_abort(failing);
    session.finish();
    const std::string text = read_file(log_path);
    for (const std::string module : {"tracking", "local_mapping", "loop_closing"}) {
      const std::string expected = "[" + module + "] [warning] " +
        "LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted";
      require(text.find(expected) != std::string::npos, "Missing caller-module optimizer record");
    }
    require(text.find("[filtered]") == std::string::npos, "WARN escaped ERROR filtering");
    require(session.dropped_messages() == 0, "Optimizer records were dropped");
    std::filesystem::remove_all(root);
    std::cout << "optimizer_logging_test: PASS\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "optimizer_logging_test: " << error.what() << '\n';
    std::filesystem::remove_all(root);
    return 1;
  }
}
