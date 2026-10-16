#include <Atlas.h>
#include <FrameImuState.h>
#include <LocalMapping.h>
#include <LoopClosing.h>
#include <System.h>
#include <Tracking.h>
#include <TrackingSnapshot.h>
#include <spdlog/sinks/null_sink.h>
#include <spdlog/sinks/ostream_sink.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <sstream>
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
    const std::shared_ptr<spdlog::logger> & logger, bool enabled, bool pose_enabled = false)
  : Tracking(nullptr, nullptr, nullptr, nullptr, &atlas, nullptr, settings,
      ORB_SLAM3::System::STEREO, nullptr, "snapshot_test", logger, enabled, pose_enabled)
  {
    mpReferenceKF = nullptr;
  }

  void seed_previous_measurements()
  {
    mTrackingSnapshot.map_id_valid = true;
    mTrackingSnapshot.tracking_map_id = 987;
    mTrackingSnapshot.tracked_mappoint_inliers_valid = true;
    mTrackingSnapshot.tracked_mappoint_inliers = 75;
    mTrackingSnapshot.pose_valid = true;
    mTrackingSnapshot.pose_translation_cw = {{1.0f, 2.0f, 3.0f}};
    mTrackingSnapshot.pose_quaternion_cw_xyzw = {{0.0f, 0.0f, 0.0f, 1.0f}};
    mTrackingSnapshot.map_change_index_valid = true;
    mTrackingSnapshot.map_change_index = 123;
    mTrackingSnapshot.imu_initialized_valid = true;
    mTrackingSnapshot.imu_initialized = true;
    mTrackingSnapshot.pose_capture_steady_ns = 456;
  }

  // Prepare only the source identity used by the production capture helper.
  // A default Frame suffices for pose tests, which never call Track/optimizers.
  void prepare_pose_snapshot(ORB_SLAM3::Map & map, eTrackingState state)
  {
    mState = state;
    mTrackingSnapshot = ORB_SLAM3::TrackingSnapshot{};
    mTrackingSnapshot.enabled = mTrackingSnapshotEnabled;
    mTrackingSnapshot.pose_enabled = mTrackingPoseSnapshotEnabled;
    mTrackingSnapshot.has_frame = true;
    mTrackingSnapshot.frame_id = mCurrentFrame.mnId;
    mTrackingSnapshot.sensor_timestamp_sec = mCurrentFrame.mTimeStamp;
    mTrackingSnapshot.map_id_valid = true;
    mTrackingSnapshot.tracking_map_id = map.GetId();
  }

  ORB_SLAM3::TrackingSnapshot capture_pose(ORB_SLAM3::Map & map)
  {
    const std::lock_guard<std::mutex> lock(map.mMutexMapUpdate);
    CapturePoseSnapshot(&map);
    return CaptureTrackingSnapshot();
  }

  void lose_and_create_map()
  {
    mState = OK;
    SetLostState();
    SetLostState(); // Repeated assignment is not a new entry.
    CreateMapInAtlas();
  }

  void relocalized()
  {
    RecordRelocalization();
    RecordRelocalization(); // One success event per frame.
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
  const ORB_SLAM3::EvaluationEventTotals totals = atlas.GetEvaluationEventTotals();
  require(totals.counts[static_cast<std::size_t>(ORB_SLAM3::EvaluationEvent::MapBaseline)] == 1 &&
    totals.counts[static_cast<std::size_t>(ORB_SLAM3::EvaluationEvent::MapCreated)] == 2 &&
    totals.counts[static_cast<std::size_t>(ORB_SLAM3::EvaluationEvent::ActiveMapReset)] == 1 &&
    totals.counts[static_cast<std::size_t>(ORB_SLAM3::EvaluationEvent::AtlasReset)] == 1,
    "Baseline/new-map/reset events were mixed or reset with the atlas");
}

void test_event_payload()
{
  std::ostringstream text;
  const auto logger = std::make_shared<spdlog::logger>("event_test",
    std::make_shared<spdlog::sinks::ostream_sink_mt>(text));
  ORB_SLAM3::Atlas atlas(0, logger, logger);
  atlas.SetTrackingStatisticsEnabled(true);
  atlas.ChangeMap(atlas.GetCurrentMap()); // Re-selecting the same map is not a switch.
  ORB_SLAM3::EvaluationEventData event;
  event.steady_ns = 456;
  event.map_valid = event.target_map_valid = event.result_map_valid = true;
  event.map_id = 0;
  event.target_map_id = 12;
  event.result_map_id = 0; // Surviving map can be renamed to the source ID.
  event.keyframes_valid = true;
  event.keyframe_id = 5;
  event.target_keyframe_id = 9;
  atlas.RecordEvaluationEvent(ORB_SLAM3::EvaluationEvent::MergeApplied, event);
  const ORB_SLAM3::EvaluationEventTotals totals = atlas.GetEvaluationEventTotals();
  require(totals.counts[static_cast<std::size_t>(ORB_SLAM3::EvaluationEvent::MergeApplied)] == 1 &&
    totals.counts[static_cast<std::size_t>(ORB_SLAM3::EvaluationEvent::MapSwitched)] == 0,
    "Event totals confused a map selection with a switch");
  require(text.str().find("T01_EVENT type=merge_applied sequence=2 steady_ns=456") != std::string::npos &&
    text.str().find("map_valid=1 map_id=0 target_map_valid=1 target_map_id=12 result_map_valid=1 result_map_id=0") !=
    std::string::npos, "Event payload lost original IDs or replaced source time with log time");
}

void test_disabled_and_concurrent_insertions(const std::shared_ptr<spdlog::logger> & logger)
{
  ORB_SLAM3::KeyFrame disabled_keyframe;
  disabled_keyframe.mnId = 0;
  ORB_SLAM3::Atlas disabled(0, logger, logger);
  disabled_keyframe.UpdateMap(disabled.GetCurrentMap());
  disabled.AddKeyFrame(&disabled_keyframe);
  require(disabled.GetInsertedKeyFramesTotal() == 0, "Default-off collection counted KFs");
  for(const std::uint64_t count : disabled.GetEvaluationEventTotals().counts)
    require(count == 0, "Disabled collection recorded events");

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
  bool enabled, bool pose_enabled = false)
{
  ORB_SLAM3::KeyFrame background_keyframe;
  background_keyframe.mnId = 1;
  ORB_SLAM3::Atlas atlas(0, logger, logger);
  atlas.SetTrackingStatisticsEnabled(enabled);
  TestTracking tracking(atlas, settings.string(), logger, enabled, pose_enabled);
  ORB_SLAM3::LocalMapping mapper(nullptr, &atlas, false, false, "snapshot_test", logger);
  ORB_SLAM3::LoopClosing loop(&atlas, nullptr, nullptr, true, false, {}, logger);
  tracking.SetLocalMapper(&mapper);
  tracking.SetLoopClosing(&loop);
  const ORB_SLAM3::TrackingSnapshot before = tracking.CaptureTrackingSnapshot();
  require(before.enabled == enabled && !before.has_frame && !before.map_id_valid &&
    !before.tracked_mappoint_inliers_valid, "Initial snapshot validity was incorrect");
  require(before.pose_enabled == pose_enabled && !before.pose_valid &&
    !before.map_change_index_valid && !before.imu_initialized_valid &&
    before.pose_capture_steady_ns == 0, "Pose collection was enabled or populated before a frame");

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
  require(!first.pose_valid && first.pose_enabled == pose_enabled &&
    first.map_change_index_valid == pose_enabled && !first.imu_initialized_valid &&
    (first.pose_capture_steady_ns > 0) == pose_enabled,
    "Startup did not capture opt-in map context or fabricated a pose");

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
  require(!reversed.pose_valid && !reversed.map_change_index_valid &&
    !reversed.imu_initialized_valid && !reversed.imu_initialized &&
    reversed.pose_capture_steady_ns == 0 &&
    reversed.pose_translation_cw == std::array<float, 3>{} &&
    reversed.pose_quaternion_cw_xyzw == std::array<float, 4>{},
    "Same-call map creation reused the preceding pose or map context");

  // Reinitialization reaches the map-locked stage, but still skips TrackLocalMap.
  tracking.seed_previous_measurements();
  tracking.track_startup_frame(image, 3.0);
  const ORB_SLAM3::TrackingSnapshot resumed = tracking.CaptureTrackingSnapshot();
  require(resumed.map_id_valid && resumed.tracking_map_id == atlas.GetCurrentMap()->GetId() &&
    !resumed.tracked_mappoint_inliers_valid && resumed.tracked_mappoint_inliers == 0,
    "Reinitialization retained inliers from the previous frame");
  require(!resumed.pose_valid && resumed.pose_enabled == pose_enabled &&
    resumed.map_change_index_valid == pose_enabled && !resumed.imu_initialized_valid &&
    !resumed.imu_initialized && resumed.pose_translation_cw == std::array<float, 3>{} &&
    resumed.pose_quaternion_cw_xyzw == std::array<float, 4>{},
    "Reinitialization retained the preceding pose or inertial context");
  tracking.lose_and_create_map();
  const ORB_SLAM3::TrackingSnapshot lost = tracking.CaptureTrackingSnapshot();
  require(lost.raw_tracking_state == ORB_SLAM3::Tracking::NO_IMAGES_YET &&
    lost.frame_id == resumed.frame_id && lost.tracking_lost_events == 1,
    "Same-call new-map creation hid or duplicated LOST entry");
  tracking.track_startup_frame(image, 4.0);
  require(tracking.CaptureTrackingSnapshot().tracking_lost_events == 0,
    "LOST event was reused in the next frame");
  tracking.relocalized();
  const ORB_SLAM3::EvaluationEventTotals totals = atlas.GetEvaluationEventTotals();
  require(tracking.CaptureTrackingSnapshot().relocalization_events == 1 &&
    totals.counts[static_cast<std::size_t>(ORB_SLAM3::EvaluationEvent::TrackingLost)] == 1 &&
    totals.counts[static_cast<std::size_t>(ORB_SLAM3::EvaluationEvent::Relocalized)] == 1,
    "Tracking event totals lost source transitions or counted duplicates");
}

void test_pose_requires_tracking_snapshot(
  const std::filesystem::path & settings, const std::shared_ptr<spdlog::logger> & logger)
{
  ORB_SLAM3::Atlas atlas(0, logger, logger);
  bool rejected = false;
  try {
    TestTracking invalid(atlas, settings.string(), logger, false, true);
  } catch(const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "Tracking accepted pose collection without tracking snapshots");

  rejected = false;
  try {
    // Rejection must precede settings/vocabulary access and worker creation.
    ORB_SLAM3::System invalid("", "", ORB_SLAM3::System::STEREO, false, 0, "",
      {}, {}, {}, false, true);
  } catch(const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "System accepted pose collection without tracking snapshots");
}

void test_pose_snapshot(
  const std::filesystem::path & settings, const std::shared_ptr<spdlog::logger> & logger,
  bool pose_enabled)
{
  ORB_SLAM3::Atlas atlas(0, logger, logger);
  atlas.SetTrackingStatisticsEnabled(true);
  TestTracking tracking(atlas, settings.string(), logger, true, pose_enabled);
  ORB_SLAM3::Map & map = *atlas.GetCurrentMap();
  tracking.mCurrentFrame.mnId = 42;
  tracking.mCurrentFrame.mTimeStamp = 4.25;
  tracking.prepare_pose_snapshot(map, ORB_SLAM3::Tracking::OK);
  const ORB_SLAM3::TrackingSnapshot missing = tracking.capture_pose(map);
  require(!missing.pose_valid, "Default identity storage was accepted without Frame::HasPose");

  const Eigen::Quaternionf rotation(Eigen::AngleAxisf(0.5f, Eigen::Vector3f::UnitZ()));
  const Eigen::Vector3f translation(1.0f, -2.0f, 0.5f);
  const Sophus::SE3f pose(rotation, translation);
  tracking.mCurrentFrame.SetPose(pose);
  tracking.prepare_pose_snapshot(map, ORB_SLAM3::Tracking::OK);
  const std::int64_t before = ORB_SLAM3::EvaluationSteadyNs();
  const ORB_SLAM3::TrackingSnapshot valid = tracking.capture_pose(map);
  const std::int64_t after = ORB_SLAM3::EvaluationSteadyNs();
  if(!pose_enabled) {
    require(!valid.pose_enabled && !valid.pose_valid && !valid.map_change_index_valid &&
      !valid.imu_initialized_valid && valid.pose_capture_steady_ns == 0 &&
      valid.pose_translation_cw == std::array<float, 3>{} &&
      valid.pose_quaternion_cw_xyzw == std::array<float, 4>{},
      "Default-off pose collection read or populated a valid frame pose");
    return;
  }
  require(valid.pose_enabled && valid.pose_valid && valid.frame_id == 42 &&
    valid.sensor_timestamp_sec == 4.25 && valid.map_id_valid &&
    valid.tracking_map_id == map.GetId() && valid.map_change_index_valid &&
    valid.map_change_index == map.GetMapChangeIndex() && !valid.imu_initialized_valid &&
    valid.pose_capture_steady_ns >= before && valid.pose_capture_steady_ns <= after,
    "Pose capture lost same-frame identity, map context or source time");
  require(valid.pose_translation_cw == std::array<float, 3>{{1.0f, -2.0f, 0.5f}} &&
    std::abs(valid.pose_quaternion_cw_xyzw[0]) < 1e-6f &&
    std::abs(valid.pose_quaternion_cw_xyzw[1]) < 1e-6f &&
    std::abs(valid.pose_quaternion_cw_xyzw[2] - std::sin(0.25f)) < 1e-6f &&
    std::abs(valid.pose_quaternion_cw_xyzw[3] - std::cos(0.25f)) < 1e-6f,
    "Live Tcw was inverted, reordered or changed during capture");

  // A background correction after capture must not change published values.
  {
    const std::lock_guard<std::mutex> lock(map.mMutexMapUpdate);
    tracking.mCurrentFrame.SetPose(Sophus::SE3f());
    map.IncreaseChangeIndex();
    map.SetImuInitialized();
    map.ChangeId(valid.tracking_map_id + 1000);
  }
  const ORB_SLAM3::TrackingSnapshot cached = tracking.CaptureTrackingSnapshot();
  require(cached.pose_valid && cached.tracking_map_id == valid.tracking_map_id &&
    cached.map_change_index == valid.map_change_index &&
    cached.pose_capture_steady_ns == valid.pose_capture_steady_ns &&
    cached.pose_translation_cw == valid.pose_translation_cw &&
    cached.pose_quaternion_cw_xyzw == valid.pose_quaternion_cw_xyzw &&
    !cached.imu_initialized_valid, "Snapshot fetched live pose/context after capture");

  // A supplied pose is still invalid outside successful tracking states.
  for(const auto state : {ORB_SLAM3::Tracking::SYSTEM_NOT_READY, ORB_SLAM3::Tracking::NO_IMAGES_YET,
    ORB_SLAM3::Tracking::NOT_INITIALIZED, ORB_SLAM3::Tracking::RECENTLY_LOST, ORB_SLAM3::Tracking::LOST})
  {
    tracking.prepare_pose_snapshot(map, state);
    const ORB_SLAM3::TrackingSnapshot invalid = tracking.capture_pose(map);
    require(!invalid.pose_valid && invalid.map_change_index_valid &&
      invalid.pose_translation_cw == std::array<float, 3>{} &&
      invalid.pose_quaternion_cw_xyzw == std::array<float, 4>{},
      "Unsuccessful tracking state published a pose");
  }
  tracking.prepare_pose_snapshot(map, ORB_SLAM3::Tracking::OK_KLT);
  require(tracking.capture_pose(map).pose_valid, "Successful OK_KLT pose was rejected");

  tracking.prepare_pose_snapshot(map, ORB_SLAM3::Tracking::OK);
  ++tracking.mCurrentFrame.mnId;
  require(!tracking.capture_pose(map).pose_valid, "Replacement frame reused source-frame identity");
  tracking.prepare_pose_snapshot(map, ORB_SLAM3::Tracking::OK);
  tracking.mCurrentFrame.mTimeStamp += 0.25;
  require(!tracking.capture_pose(map).pose_valid, "Mismatched sensor time published a pose");
  tracking.mCurrentFrame.mTimeStamp = std::numeric_limits<double>::infinity();
  tracking.prepare_pose_snapshot(map, ORB_SLAM3::Tracking::OK);
  require(!tracking.capture_pose(map).pose_valid, "Non-finite source time published a pose");
  tracking.mCurrentFrame.mTimeStamp = 4.5;
  tracking.prepare_pose_snapshot(map, ORB_SLAM3::Tracking::OK);
  {
    const std::lock_guard<std::mutex> lock(map.mMutexMapUpdate);
    map.ChangeId(map.GetId() + 1);
  }
  require(!tracking.capture_pose(map).pose_valid, "Mismatched map identity published a pose");

  for(const float value : {std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity()})
  {
    Sophus::SE3f invalid = pose;
    invalid.translation().x() = value;
    tracking.mCurrentFrame.SetPose(invalid);
    tracking.prepare_pose_snapshot(map, ORB_SLAM3::Tracking::OK);
    require(!tracking.capture_pose(map).pose_valid, "Non-finite translation published a pose");
  }
  Sophus::SE3f nonunit = pose;
  // Corrupt a copy through Sophus's raw storage without normalizing it.
  for(int index = 0; index < 4; ++index) nonunit.so3().data()[index] *= 2.0f;
  tracking.mCurrentFrame.SetPose(nonunit);
  tracking.prepare_pose_snapshot(map, ORB_SLAM3::Tracking::OK);
  require(!tracking.capture_pose(map).pose_valid, "Non-unit rotation published a pose");

  // Only the context reader uses this sensor value: no IMU pipeline is run.
  // A separate inertial map starts uninitialized, then becomes initialized.
  ORB_SLAM3::Map inertial_map(0, logger);
  tracking.mSensor = ORB_SLAM3::System::IMU_STEREO;
  tracking.mCurrentFrame.SetPose(pose);
  tracking.prepare_pose_snapshot(inertial_map, ORB_SLAM3::Tracking::OK);
  const ORB_SLAM3::TrackingSnapshot uninitialized = tracking.capture_pose(inertial_map);
  require(uninitialized.pose_valid && uninitialized.imu_initialized_valid &&
    !uninitialized.imu_initialized, "Visual pose validity was confused with IMU initialization");
  {
    const std::lock_guard<std::mutex> lock(inertial_map.mMutexMapUpdate);
    inertial_map.SetImuInitialized();
  }
  tracking.prepare_pose_snapshot(inertial_map, ORB_SLAM3::Tracking::OK);
  const ORB_SLAM3::TrackingSnapshot initialized = tracking.capture_pose(inertial_map);
  require(initialized.pose_valid && initialized.imu_initialized_valid && initialized.imu_initialized,
    "IMU initialization context was not sampled with the pose");
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
    test_event_payload();
    test_disabled_and_concurrent_insertions(logger);
    test_frame_snapshot(settings, logger, false);
    test_frame_snapshot(settings, logger, true);
    test_frame_snapshot(settings, logger, true, true);
    test_pose_requires_tracking_snapshot(settings, logger);
    test_pose_snapshot(settings, logger, false);
    test_pose_snapshot(settings, logger, true);
    std::filesystem::remove_all(root);
    std::cout << "tracking_snapshot_test passed\n";
    return 0;
  } catch(const std::exception & error) {
    std::cerr << "tracking_snapshot_test failed: " << error.what() << '\n';
    std::filesystem::remove_all(root);
    return 1;
  }
}
