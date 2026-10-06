#include "gemini336_orbslam3/logging.hpp"
#include <Frame.h>
#include <KeyFrame.h>
#include <Map.h>
#include <MapPoint.h>
#include <spdlog/sinks/base_sink.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

// Only initialize data needed by the diagnostic paths; no camera or workers.
class TestKeyFrame : public ORB_SLAM3::KeyFrame
{
public:
  TestKeyFrame(unsigned long id, ORB_SLAM3::Map * map)
  {
    mnId = id;
    UpdateMap(map);
    SetPose(Sophus::SE3f());
    mbFirstConnection = false;
  }
  void set_points(ORB_SLAM3::MapPoint * point)
  {
    // Connection counting only reads this vector; the default feature count stays zero.
    mvpMapPoints = {point};
  }
};

class TestMapPoint : public ORB_SLAM3::MapPoint
{
public:
  using ORB_SLAM3::MapPoint::MapPoint;
  void observe(ORB_SLAM3::KeyFrame * keyframe)
  {
    mObservations[keyframe] = std::make_tuple(0, -1);
  }
  void missing_reference()
  {
    mBackupRefKFId = 777;
    mBackupReplacedId = -1;
    restore_reference();
  }
  void restore_reference()
  {
    std::map<unsigned long, ORB_SLAM3::KeyFrame *> keyframes;
    std::map<unsigned long, ORB_SLAM3::MapPoint *> points;
    PostLoad(keyframes, points);
    require(GetReferenceKeyFrame() == nullptr, "PostLoad changed missing reference behavior");
  }
};

class ThrowingSink : public spdlog::sinks::base_sink<std::mutex>
{
  void sink_it_(const spdlog::details::log_msg &) override
  {
    throw std::runtime_error("injected data log failure");
  }
  void flush_() override {}
};

void self_parent(ORB_SLAM3::KeyFrame & keyframe)
{
  try {
    keyframe.ChangeParent(&keyframe);
  } catch (const std::invalid_argument & error) {
    require(
      std::string(error.what()) == "The parent and child can not be the same",
      "Logging changed the self-parent exception");
    return;
  }
  throw std::runtime_error("Self-parent no longer throws");
}

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream file(path);
  require(file.good(), "Cannot read data test log");
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
}  // namespace

int main()
{
  const auto root = std::filesystem::temp_directory_path() /
    ("data_logging_test_" + std::to_string(::getpid()));
  try {
    gemini336_orbslam3::LoggingOptions options;
    options.directory = root;
    options.level = "debug";
    gemini336_orbslam3::LoggingSession session(options);
    const auto log_path = session.directory() / "slam.log";
    ORB_SLAM3::Map map(0, session.GetLogger("map"));
    TestKeyFrame keyframe(42, &map);
    TestKeyFrame neighbor(43, &map);
    TestMapPoint point(Eigen::Vector3f(0, 0, -1), &keyframe, &map);
    keyframe.set_points(&point);
    point.observe(&keyframe);
    point.observe(&neighbor);
    keyframe.UpdateConnections(false);
    require(keyframe.GetWeight(&neighbor) == 1, "Connection weight changed");
    point.PrintObservations();
    keyframe.PrintPointDistribution();
    self_parent(keyframe);

    ORB_SLAM3::Frame frame;
    frame.SetPose(Sophus::SE3f());
    frame.N = 2;
    frame.Nleft = 1;
    frame.mvpMapPoints = {&point, &point};
    frame.mvbOutlier = {false, true};
    frame.PrintPointDistribution(map.GetLogger());
    cv::Point2f projection(9, 10);
    float u = 11;
    float v = 12;
    require(!frame.ProjectPointDistort(&point, projection, u, v), "Frame accepted negative depth");
    require(!keyframe.ProjectPointDistort(&point, projection, u, v), "KF accepted negative depth");
    require(
      !keyframe.ProjectPointUnDistort(&point, projection, u, v),
      "KF accepted negative depth");
    require(projection == cv::Point2f(9, 10) && u == 11 && v == 12, "Projection outputs changed");

    // Rebinding must use the new Map's logger, including subsequent PostLoad diagnostics.
    ORB_SLAM3::Map rebound(0, session.GetLogger("rebound"));
    keyframe.UpdateMap(&rebound);
    point.UpdateMap(&rebound);
    self_parent(keyframe);
    point.missing_reference();
    // Serialize the base data only; logger context is supplied after loading.
    std::stringstream saved;
    {
      boost::archive::binary_oarchive archive(saved);
      archive << static_cast<ORB_SLAM3::MapPoint &>(point);
    }
    TestMapPoint restored;
    {
      boost::archive::binary_iarchive archive(saved);
      archive >> static_cast<ORB_SLAM3::MapPoint &>(restored);
    }
    require(restored.GetMap() == nullptr, "Archive unexpectedly restored Map context");
    restored.UpdateMap(&rebound);
    restored.restore_reference();
    const auto filtered = session.GetLogger("filtered");
    filtered->set_level(spdlog::level::err);
    rebound.SetLogger(filtered);
    keyframe.PrintPointDistribution();
    frame.PrintPointDistribution(filtered);
    point.PrintObservations();
    point.missing_reference();

    // Logging failures must preserve the original exception and projection result.
    const auto failing = std::make_shared<spdlog::logger>(
      "failing", std::make_shared<ThrowingSink>());
    failing->set_level(spdlog::level::debug);
    failing->set_error_handler(
      [](const std::string &) {throw std::runtime_error("injected error handler failure");});
    rebound.SetLogger(failing);
    self_parent(keyframe);
    require(!frame.ProjectPointDistort(&point, projection, u, v), "Log failure changed projection");
    ORB_SLAM3::KeyFrame unbound_keyframe;
    ORB_SLAM3::MapPoint unbound_point;
    require(unbound_keyframe.GetMap() == nullptr, "Default KF Map is not null");
    require(unbound_point.GetMap() == nullptr, "Default MP Map is not null");
    self_parent(unbound_keyframe);
    frame.PrintPointDistribution();

    session.finish();
    const std::string text = read_file(log_path);
    for (const std::string message : {
      "[map] [debug] UPDATE_CONN: current KF 42",
      "[map] [debug]   UPDATE_CONN: KF 43 ; num matches: 1",
      "[map] [debug] MP_OBS: MP ", "[map] [debug] --OBS in KF 43 in map ",
      "[map] [debug] Point distribution in KeyFrame: left-> 0 --- right-> 0",
      "[map] [debug] Point distribution in Frame: left-> 1 --- right-> 0",
      "[map] [debug] Negative depth: -1",
      "[map] [error] ERROR: Change parent KF, the parent and child are the same KF",
      "[rebound] [error] ERROR: Change parent KF, the parent and child are the same KF",
      "[rebound] [error] ERROR: MP without KF reference 777; Num obs: 0",
      "[filtered] [error] ERROR: MP without KF reference 777; Num obs: 0"})
    {
      require(text.find(message) != std::string::npos, "Missing data diagnostic record");
    }
    const std::string restored_message =
      "[rebound] [error] ERROR: MP without KF reference 777; Num obs: 0";
    const auto first = text.find(restored_message);
    require(
      first != std::string::npos &&
      text.find(restored_message, first + restored_message.size()) != std::string::npos,
      "Restored MapPoint did not use the rebound Map logger");
    require(text.find("[filtered] [debug]") == std::string::npos, "DEBUG escaped ERROR filtering");
    require(session.dropped_messages() == 0, "Data records were dropped");
    std::filesystem::remove_all(root);
    std::cout << "data_logging_test: PASS\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "data_logging_test: " << error.what() << '\n';
    std::filesystem::remove_all(root);
    return 1;
  }
}
