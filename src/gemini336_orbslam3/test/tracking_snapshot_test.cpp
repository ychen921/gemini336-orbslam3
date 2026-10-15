#include <Atlas.h>
#include <FrameImuState.h>
#include <LocalMapping.h>
#include <LoopClosing.h>
#include <System.h>
#include <Tracking.h>
#include <TrackingSnapshot.h>
#include <spdlog/sinks/null_sink.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <unistd.h>

namespace
{
void require(bool value, const char * message)
{
  if (!value) {throw std::runtime_error(message);}
}

// Only fixture setup touches protected fields. A finite RGB-D Frame constructor
// supplies complete feature/calibration fields without running stereo matching.
// Stereo Track() then exercises startup/early returns below its init threshold,
// without System, live workers or an optimizer.
class TestTracking : public ORB_SLAM3::Tracking
{
public:
  TestTracking(
    ORB_SLAM3::Atlas & atlas, const std::string & settings,
    const std::shared_ptr<spdlog::logger> & logger, bool enabled)
  : Tracking(nullptr, nullptr, nullptr, nullptr, &atlas, nullptr, settings,
      ORB_SLAM3::System::STEREO, nullptr, "snapshot_test", logger, enabled)
  {
    mpReferenceKF = nullptr;
  }

  void seed_previous_measurements()
  {
    mTrackingSnapshot.map_id_valid = true;
    mTrackingSnapshot.tracking_map_id = 987;
    mTrackingSnapshot.tracked_mappoint_inliers_valid = true;
    mTrackingSnapshot.tracked_mappoint_inliers = 75;
  }

  void track_startup_frame(const cv::Mat & image, double timestamp)
  {
    const cv::Mat depth(image.size(), CV_32FC1, cv::Scalar(4.0));
    {
      // Stereo Track() also completes a frame producer. Match GrabImageStereo's
      // begin/replace/unlock sequence before Track acquires its frame/map guards.
      ORB_SLAM3::FrameImuState::Guard frameIMU = mFrameIMU.Lock();
      frameIMU.BeginFrame();
      mCurrentFrame = ORB_SLAM3::Frame(image, depth, timestamp, mpORBextractorLeft,
        mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera);
      mCurrentFrame.mnDataset = 0;
    }
    Track();
  }
};

void test_insertions(const std::shared_ptr<spdlog::logger> & logger)
{
  ORB_SLAM3::KeyFrame initial;
  ORB_SLAM3::KeyFrame culled;
  ORB_SLAM3::KeyFrame later;
  ORB_SLAM3::KeyFrame after_reset;
  initial.mnId = 10;
  culled.mnId = 11;
  later.mnId = 12;
  after_reset.mnId = 10;  // A full tracking reset may reuse KF IDs.
  ORB_SLAM3::Atlas atlas(0, logger, logger);
  atlas.SetTrackingStatisticsEnabled(true);
  ORB_SLAM3::Map * source = atlas.GetCurrentMap();
  initial.UpdateMap(source);
  culled.UpdateMap(source);
  atlas.AddKeyFrame(&initial);
  atlas.AddKeyFrame(&initial);  // Initialization then LocalMapping admission.
  atlas.AddKeyFrame(&culled);
  require(atlas.GetInsertedKeyFramesTotal() == 2, "Duplicate KF admission was counted");
  source->EraseKeyFrame(&culled);
  require(source->KeyFramesInMap() == 1 && atlas.GetInsertedKeyFramesTotal() == 2,
    "Culling changed the insertion total");

  atlas.CreateNewMap();
  ORB_SLAM3::Map * destination = atlas.GetCurrentMap();
  later.UpdateMap(destination);
  atlas.AddKeyFrame(&later);
  // Merge transfers use direct Map insertion, bypassing Atlas admission.
  initial.UpdateMap(destination);
  require(destination->AddKeyFrame(&initial), "Merge fixture did not transfer its KF");
  source->EraseKeyFrame(&initial);
  require(atlas.GetInsertedKeyFramesTotal() == 3, "Merge transfer was counted as a new KF");
  atlas.clearMap();
  require(destination->KeyFramesInMap() == 0 && atlas.GetInsertedKeyFramesTotal() == 3,
    "Active-map reset cleared the run total");

  // The legacy clearAtlas path releases its map set without deleting maps.
  // Reclaim the detached fixture maps explicitly, without changing production ownership.
  atlas.clearAtlas();
  const std::unique_ptr<ORB_SLAM3::Map> detached_source(source);
  const std::unique_ptr<ORB_SLAM3::Map> detached_destination(destination);
  atlas.CreateNewMap();
  after_reset.UpdateMap(atlas.GetCurrentMap());
  atlas.AddKeyFrame(&after_reset);
  require(atlas.GetInsertedKeyFramesTotal() == 4, "Full reset/reused KF ID corrupted the total");
}

void test_disabled_and_concurrent_insertions(const std::shared_ptr<spdlog::logger> & logger)
{
  ORB_SLAM3::KeyFrame disabled_keyframe;
  disabled_keyframe.mnId = 0;
  ORB_SLAM3::Atlas disabled(0, logger, logger);
  disabled_keyframe.UpdateMap(disabled.GetCurrentMap());
  disabled.AddKeyFrame(&disabled_keyframe);
  require(disabled.GetInsertedKeyFramesTotal() == 0, "Default-off collection counted KFs");

  std::vector<std::unique_ptr<ORB_SLAM3::KeyFrame>> keyframes;
  ORB_SLAM3::Atlas atlas(0, logger, logger);
  atlas.SetTrackingStatisticsEnabled(true);
  for(std::uint64_t index = 0; index < 128; ++index) {
    auto keyframe = std::make_unique<ORB_SLAM3::KeyFrame>();
    keyframe->mnId = index;
    keyframe->UpdateMap(atlas.GetCurrentMap());
    keyframes.push_back(std::move(keyframe));
  }
  std::promise<void> start;
  const std::shared_future<void> ready = start.get_future().share();
  auto insert = [&](std::size_t offset) {
      ready.wait();
      for(std::size_t index = offset; index < keyframes.size(); index += 2) {
        atlas.AddKeyFrame(keyframes[index].get());
        atlas.AddKeyFrame(keyframes[index].get());
      }
    };
  auto first = std::async(std::launch::async, insert, 0);
  auto second = std::async(std::launch::async, insert, 1);
  start.set_value();
  std::uint64_t previous = 0;
  while(first.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready ||
    second.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    const std::uint64_t current = atlas.GetInsertedKeyFramesTotal();
    require(current >= previous && current <= keyframes.size(), "Concurrent total was invalid");
    previous = current;
    std::this_thread::yield();
  }
  first.get();
  second.get();
  require(atlas.GetInsertedKeyFramesTotal() == keyframes.size(), "Concurrent insertions were lost");
}

void test_frame_snapshot(
  const std::filesystem::path & settings, const std::shared_ptr<spdlog::logger> & logger,
  bool enabled)
{
  ORB_SLAM3::KeyFrame background_keyframe;
  background_keyframe.mnId = 1;
  ORB_SLAM3::Atlas atlas(0, logger, logger);
  atlas.SetTrackingStatisticsEnabled(enabled);
  TestTracking tracking(atlas, settings.string(), logger, enabled);
  ORB_SLAM3::LocalMapping mapper(nullptr, &atlas, false, false, "snapshot_test", logger);
  ORB_SLAM3::LoopClosing loop(&atlas, nullptr, nullptr, true, false, {}, logger);
  tracking.SetLocalMapper(&mapper);
  tracking.SetLoopClosing(&loop);
  const ORB_SLAM3::TrackingSnapshot before = tracking.CaptureTrackingSnapshot();
  require(before.enabled == enabled && !before.has_frame && !before.map_id_valid &&
    !before.tracked_mappoint_inliers_valid, "Initial snapshot validity was incorrect");

  cv::Mat image(480, 640, CV_8UC1);
  cv::RNG random(12345);
  random.fill(image, cv::RNG::UNIFORM, 0, 256);
  const std::uint64_t first_id = ORB_SLAM3::Frame::nNextId;
  tracking.track_startup_frame(image, 2.0);
  const ORB_SLAM3::TrackingSnapshot first = tracking.CaptureTrackingSnapshot();
  if(!enabled) {
    require(!first.enabled && !first.has_frame && !first.map_id_valid &&
      !first.tracked_mappoint_inliers_valid, "Disabled tracking produced a snapshot");
    return;
  }
  require(tracking.mCurrentFrame.N > 0 && tracking.mCurrentFrame.N < 500,
    "Synthetic frame did not stay in the startup fixture path");
  require(first.enabled && first.has_frame && first.frame_id == first_id &&
    first.sensor_timestamp_sec == 2.0 && first.raw_tracking_state == ORB_SLAM3::Tracking::NOT_INITIALIZED,
    "Startup frame identity/state was incorrect");
  ORB_SLAM3::Map * map = atlas.GetCurrentMap();
  require(first.map_id_valid && first.tracking_map_id == map->GetId() &&
    !first.tracked_mappoint_inliers_valid, "Startup measurements were mislabeled as inliers");

  // Simulate a background insertion and merge ID rewrite after capture.
  // New captures sample the live insertion total, while map identity is cached
  // at Track's synchronization point. Previously captured values remain owned.
  background_keyframe.UpdateMap(map);
  atlas.AddKeyFrame(&background_keyframe);
  {
    const std::lock_guard<std::mutex> lock(map->mMutexMapUpdate);
    map->ChangeId(first.tracking_map_id + 1000);
  }
  const ORB_SLAM3::TrackingSnapshot captured = tracking.CaptureTrackingSnapshot();
  require(captured.tracking_map_id == first.tracking_map_id && captured.keyframes_inserted_total == 1 &&
    first.keyframes_inserted_total == 0, "Capture read live map identity or changed a prior value");

  tracking.seed_previous_measurements();
  const std::uint64_t reversed_id = ORB_SLAM3::Frame::nNextId;
  tracking.track_startup_frame(image, 1.5);  // Creates a map before taking its update lock.
  const ORB_SLAM3::TrackingSnapshot reversed = tracking.CaptureTrackingSnapshot();
  require(reversed.has_frame && reversed.frame_id == reversed_id && reversed.sensor_timestamp_sec == 1.5 &&
    reversed.raw_tracking_state == ORB_SLAM3::Tracking::NO_IMAGES_YET,
    "Map creation replaced the input frame identity in the snapshot");
  require(!reversed.map_id_valid && !reversed.tracked_mappoint_inliers_valid &&
    reversed.tracked_mappoint_inliers == 0 && reversed.keyframes_inserted_total == 1,
    "Early return reused stale measurements or reset the cumulative total");

  // Reinitialization reaches the map-locked stage, but still skips TrackLocalMap.
  tracking.seed_previous_measurements();
  tracking.track_startup_frame(image, 3.0);
  const ORB_SLAM3::TrackingSnapshot resumed = tracking.CaptureTrackingSnapshot();
  require(resumed.map_id_valid && resumed.tracking_map_id == atlas.GetCurrentMap()->GetId() &&
    !resumed.tracked_mappoint_inliers_valid && resumed.tracked_mappoint_inliers == 0,
    "Reinitialization retained inliers from the previous frame");
}
} // namespace

int main()
{
  const std::filesystem::path root = std::filesystem::temp_directory_path() /
    ("tracking_snapshot_test_" + std::to_string(::getpid()));
  try {
    std::filesystem::create_directories(root);
    const std::filesystem::path settings = root / "camera.yaml";
    {
      std::ofstream file(settings);
      file << "%YAML:1.0\nCamera.type: PinHole\n"
        "Camera.fx: 400.0\nCamera.fy: 400.0\nCamera.cx: 320.0\nCamera.cy: 240.0\n"
        "Camera.k1: 0.0\nCamera.k2: 0.0\nCamera.p1: 0.0\nCamera.p2: 0.0\n"
        "Camera.fps: 30.0\nCamera.RGB: 0\nCamera.bf: 40.0\nThDepth: 35.0\n"
        "ORBextractor.nFeatures: 64\nORBextractor.scaleFactor: 1.2\n"
        "ORBextractor.nLevels: 4\nORBextractor.iniThFAST: 20\nORBextractor.minThFAST: 7\n";
      require(file.good(), "Cannot write snapshot settings fixture");
    }
    const auto logger = std::make_shared<spdlog::logger>("snapshot_test",
      std::make_shared<spdlog::sinks::null_sink_mt>());
    test_insertions(logger);
    test_disabled_and_concurrent_insertions(logger);
    test_frame_snapshot(settings, logger, false);
    test_frame_snapshot(settings, logger, true);
    std::filesystem::remove_all(root);
    std::cout << "tracking_snapshot_test passed\n";
    return 0;
  } catch(const std::exception & error) {
    std::cerr << "tracking_snapshot_test failed: " << error.what() << '\n';
    std::filesystem::remove_all(root);
    return 1;
  }
}
