#include "gemini336_orbslam3/logging.hpp"
#include <GeometricTools.h>
#include <ImuTypes.h>
#include <System.h>
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

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream file(path);
  require(file.good(), "Cannot read utility test log");
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

class ThrowingSink : public spdlog::sinks::base_sink<std::mutex>
{
  void sink_it_(const spdlog::details::log_msg &) override
  {
    throw std::runtime_error("injected utility log failure");
  }
  void flush_() override {}
};
}  // namespace

int main()
{
  const auto root = std::filesystem::temp_directory_path() /
    ("utility_logging_test_" + std::to_string(::getpid()));
  const auto original_threshold = ORB_SLAM3::Verbose::th;
  try {
    gemini336_orbslam3::LoggingOptions options;
    options.directory = root;
    options.level = "debug";
    gemini336_orbslam3::LoggingSession session(options);
    const auto log_path = session.directory() / "slam.log";
    const auto geometry = session.GetLogger("geometry");
    const auto imu = session.GetLogger("imu");
    const auto caller = session.GetLogger("loop_closing");
    const auto filtered = session.GetLogger("filtered");
    filtered->set_level(spdlog::level::info);

    // Real comparisons with the original 1e-3 epsilon; no solver or camera.
    const Eigen::Matrix2f zero = Eigen::Matrix2f::Zero();
    cv::Mat equal = cv::Mat::zeros(2, 2, CV_32F);
    cv::Mat wrong_size = cv::Mat::zeros(1, 2, CV_32F);
    require(ORB_SLAM3::GeometricTools::CheckMatrices(equal, zero, geometry), "Equal matrices fail");
    require(
      ORB_SLAM3::GeometricTools::CheckMatrices(
        zero, zero,
        geometry), "Equal Eigen matrices fail");
    equal.at<float>(0, 0) = 1e-3f;
    require(
      ORB_SLAM3::GeometricTools::CheckMatrices(
        equal, zero,
        geometry), "Epsilon boundary changed");
    Eigen::Matrix2f different = zero;
    different(0, 0) = 1e-3f;
    require(
      ORB_SLAM3::GeometricTools::CheckMatrices(
        zero, different,
        geometry), "Eigen epsilon changed");
    require(
      !ORB_SLAM3::GeometricTools::CheckMatrices(
        wrong_size, zero,
        geometry), "Wrong size passes");
    equal.at<float>(0, 0) = 2e-3f;
    require(
      !ORB_SLAM3::GeometricTools::CheckMatrices(
        equal, zero,
        geometry), "Positive difference passes");
    different(0, 0) = -2e-3f;
    require(
      !ORB_SLAM3::GeometricTools::CheckMatrices(
        zero, different,
        geometry), "Negative difference passes");
    require(
      !ORB_SLAM3::GeometricTools::CheckMatrices(
        equal, zero,
        filtered), "Filtering changes result");

    // Populate two deterministic measurements through the public API, without sensors.
    const ORB_SLAM3::IMU::Calib calibration(Sophus::SE3f(), 0.01f, 0.01f, 0.001f, 0.001f);
    ORB_SLAM3::IMU::Preintegrated measurements(ORB_SLAM3::IMU::Bias(), calibration);
    measurements.printMeasurements(imu);
    measurements.IntegrateNewMeasurement(
      Eigen::Vector3f::Zero(),
      Eigen::Vector3f::Zero(), 0.125f);
    measurements.IntegrateNewMeasurement(
      Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
      0.25f);
    measurements.printMeasurements(imu);
    measurements.printMeasurements(filtered);
    require(measurements.dT == 0.375f, "Logging changed measurements");

    ORB_SLAM3::Verbose::SetTh(ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
    ORB_SLAM3::Verbose::PrintMess("NORMAL_RECORD", ORB_SLAM3::Verbose::VERBOSITY_NORMAL, caller);
    ORB_SLAM3::Verbose::PrintMess("THRESHOLD_HIDDEN", ORB_SLAM3::Verbose::VERBOSITY_DEBUG, caller);
    ORB_SLAM3::Verbose::SetTh(ORB_SLAM3::Verbose::VERBOSITY_DEBUG);
    for (const auto level : {ORB_SLAM3::Verbose::VERBOSITY_VERBOSE,
        ORB_SLAM3::Verbose::VERBOSITY_VERY_VERBOSE, ORB_SLAM3::Verbose::VERBOSITY_DEBUG})
    {
      ORB_SLAM3::Verbose::PrintMess("DETAIL_RECORD", level, caller);
    }
    ORB_SLAM3::Verbose::PrintMess("LEVEL_HIDDEN", ORB_SLAM3::Verbose::VERBOSITY_DEBUG, filtered);
    ORB_SLAM3::Verbose::SetTh(ORB_SLAM3::Verbose::VERBOSITY_QUIET);
    ORB_SLAM3::Verbose::PrintMess("QUIET_HIDDEN", ORB_SLAM3::Verbose::VERBOSITY_NORMAL, caller);
    ORB_SLAM3::Verbose::PrintMess(
      "QUIET_LEVEL_RECORD", ORB_SLAM3::Verbose::VERBOSITY_QUIET,
      caller);
    ORB_SLAM3::LogMapStream(
      caller, spdlog::level::debug,
      [](std::ostream & report) {report << "MAP_WRAPPER_RECORD";});

    const auto failing = std::make_shared<spdlog::logger>(
      "failing",
      std::make_shared<ThrowingSink>());
    failing->set_level(spdlog::level::debug);
    failing->set_error_handler(
      [](const std::string &) {throw std::runtime_error("injected error handler failure");});
    require(
      !ORB_SLAM3::GeometricTools::CheckMatrices(
        equal, zero,
        failing), "Log failure changes result");
    measurements.printMeasurements(failing);
    ORB_SLAM3::Verbose::PrintMess("FAILURE_RECORD", ORB_SLAM3::Verbose::VERBOSITY_NORMAL, failing);
    // A disabled report must not run its writer, even with a failing sink.
    failing->set_level(spdlog::level::off);
    bool wrote = false;
    ORB_SLAM3::LogStreamWithFallback(
      failing, "utility", spdlog::level::debug,
      [&](std::ostream &) {wrote = true;});
    require(!wrote, "Disabled report ran its writer");

    // Existing calls without logger arguments remain usable as standalone tools.
    require(
      !ORB_SLAM3::GeometricTools::CheckMatrices(
        wrong_size,
        zero), "Fallback changes size result");
    require(ORB_SLAM3::GeometricTools::CheckMatrices(zero, zero), "Fallback changes Eigen result");
    measurements.printMeasurements();
    ORB_SLAM3::Verbose::SetTh(ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
    ORB_SLAM3::Verbose::PrintMess("STANDALONE_RECORD", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);

    session.finish();
    const std::string text = read_file(log_path);
    for (const std::string message : {
      "[geometry] [warning] wrong cvmat size", "[geometry] [debug] cv mat:",
      "[geometry] [debug] eig mat:", "[geometry] [debug] eig mat 1:",
      "[geometry] [debug] eig mat 2:", "[imu] [debug] pint meas:",
      "[imu] [debug] meas 0.125", "[imu] [debug] meas 0.25", "[imu] [debug] end pint meas:",
      "[loop_closing] [info] NORMAL_RECORD", "[loop_closing] [debug] DETAIL_RECORD",
      "[loop_closing] [info] QUIET_LEVEL_RECORD", "[loop_closing] [debug] MAP_WRAPPER_RECORD"})
    {
      require(text.find(message) != std::string::npos, "Missing utility diagnostic record");
    }
    for (const std::string hidden :
      {"THRESHOLD_HIDDEN", "LEVEL_HIDDEN", "QUIET_HIDDEN", "[filtered]",
        "STANDALONE_RECORD"})
    {
      require(text.find(hidden) == std::string::npos, "Filtering or standalone routing changed");
    }
    require(session.dropped_messages() == 0, "Utility records were dropped");
    ORB_SLAM3::Verbose::SetTh(original_threshold);
    std::filesystem::remove_all(root);
    std::cout << "utility_logging_test: PASS\n";
    return 0;
  } catch (const std::exception & error) {
    ORB_SLAM3::Verbose::SetTh(original_threshold);
    std::cerr << "utility_logging_test: " << error.what() << '\n';
    std::filesystem::remove_all(root);
    return 1;
  }
}
