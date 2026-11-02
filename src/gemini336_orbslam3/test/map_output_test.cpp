#include "slam/orbslam3_adapter.hpp"

#include <Atlas.h>
#include <KeyFrame.h>
#include <MapPoint.h>
#include <spdlog/sinks/null_sink.h>

#include <cmath>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

namespace gemini336_orbslam3
{
struct OrbSlam3AdapterTestAccess
{
    static std::unique_ptr<OrbSlam3Adapter> make(
        std::function<ORB_SLAM3::MapOutputSnapshot()> collect)
    {
        auto adapter = std::unique_ptr<OrbSlam3Adapter>(new OrbSlam3Adapter(
            OrbSlam3Adapter::TestTag{}, TrackingMode::Stereo, []() {}));
        adapter->test_map_output_ = std::move(collect);
        return adapter;
    }
};
}

namespace
{
void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}

class TestKeyFrame : public ORB_SLAM3::KeyFrame
{
public:
    void mark_bad() { mbBad = true; }
    void corrupt_rotation()
    {
        // Corrupt fixture storage without asking Sophus to invert a bad pose.
        mTcw.so3().data()[3] = 2.0f;
    }
};

class TestAtlas : public ORB_SLAM3::Atlas
{
public:
    using ORB_SLAM3::Atlas::Atlas;
    // The legacy default Atlas constructor is intended for deserialization;
    // initialize its creation seed explicitly in this no-map fixture.
    TestAtlas() { mnLastInitKFidMap = 0; }
    std::shared_mutex &output_mutex() { return mMutexMapOutput; }
    std::mutex &atlas_mutex() { return mMutexAtlas; }
};

class TestMapPoint : public ORB_SLAM3::MapPoint
{
public:
    void mark_bad() { mbBad = true; }
};

// The maps own no fixture objects. No System, worker loop, optimizer, ROS or
// sensor playback is started; threads below only exercise finite lock admission.
void test_values(const std::shared_ptr<spdlog::logger> &logger)
{
    ORB_SLAM3::Atlas atlas(0, logger, logger);
    ORB_SLAM3::Map &map = *atlas.GetCurrentMap();
    ORB_SLAM3::Map other(0, logger);
    TestKeyFrame valid, bad, wrong_map, invalid_pose, invalid_rotation;
    unsigned long id = 0;
    for (TestKeyFrame *keyframe : {&valid, &bad, &wrong_map, &invalid_pose, &invalid_rotation})
    {
        keyframe->mnId = id++;
        keyframe->UpdateMap(&map);
        keyframe->SetPose(Sophus::SE3f());
        map.AddKeyFrame(keyframe);
    }
    bad.mark_bad();
    wrong_map.UpdateMap(&other);
    Sophus::SE3f corrupt;
    corrupt.translation().x() = std::numeric_limits<float>::infinity();
    invalid_pose.SetPose(corrupt);
    invalid_rotation.corrupt_rotation();
    const Eigen::Quaternionf rotation(Eigen::AngleAxisf(0.5f, Eigen::Vector3f::UnitZ()));
    const Sophus::SE3f expected(rotation, Eigen::Vector3f(1.0f, -2.0f, 3.0f));
    valid.SetPose(expected.inverse());

    TestMapPoint point, bad_point, wrong_point, nan_point;
    for (TestMapPoint *candidate : {&point, &bad_point, &wrong_point, &nan_point})
    {
        candidate->UpdateMap(&map);
        candidate->SetWorldPos(Eigen::Vector3f::Zero());
        map.AddMapPoint(candidate);
    }
    map.AddMapPoint(nullptr);
    bad_point.mark_bad();
    wrong_point.UpdateMap(&other);
    nan_point.SetWorldPos(Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(), 0, 0));
    const auto saved = atlas.CollectMapOutput();
    require(saved.status == ORB_SLAM3::MapOutputStatus::Ready &&
            saved.map_id == map.GetId() && saved.map_points.size() == 1 && saved.keyframes.size() == 1,
            "Collection mixed invalid/bad/foreign data into the active map");
    require(saved.map_points[0] == std::array<float, 3>{} && saved.keyframes[0].id == valid.mnId,
            "Valid origin was filtered or keyframe identity was lost");
    const auto &pose = saved.keyframes[0].camera_pose;
    require(std::abs(pose.translation[0] - 1.0) < 1e-5 &&
            std::abs(pose.translation[1] + 2.0) < 1e-5 &&
            std::abs(pose.translation[2] - 3.0) < 1e-5 &&
            std::abs(pose.quaternion_xyzw[2] - std::sin(0.25)) < 1e-5,
            "Keyframe pose is not left-camera Twc");
    valid.SetPose(Sophus::SE3f());
    point.SetWorldPos(Eigen::Vector3f::Ones());
    map.IncreaseChangeIndex();
    const auto optimized = atlas.CollectMapOutput();
    require(saved.map_points[0] == std::array<float, 3>{} && saved.keyframes[0].camera_pose.translation[2] > 2.9 &&
            optimized.map_points[0][0] == 1.0f && optimized.generation == saved.generation,
            "Returned values changed with Core or ordinary geometry changed generation");

    atlas.clearMap();
    const auto cleared = atlas.CollectMapOutput();
    require(cleared.status == ORB_SLAM3::MapOutputStatus::Ready && cleared.keyframes.empty() &&
            cleared.map_points.empty() && cleared.generation > optimized.generation,
            "Reset was confused with missing/busy data or retained a generation");
    // Exercise reference transforms on an empty fixture: default MapPoints do
    // not have the observation graph required by ApplyScaledRotation's normals.
    map.ApplyScaledRotation(Sophus::SE3f(), 1.0f);
    const auto transformed = atlas.CollectMapOutput();
    require(transformed.generation > cleared.generation && transformed.map_id == cleared.map_id,
            "Same-ID reference commit did not invalidate the output generation");
    map.ChangeId(map.GetId());
    const auto renamed = atlas.CollectMapOutput();
    require(renamed.generation > transformed.generation,
            "Merge rename preserving the numeric ID reused a coordinate generation");
}

void test_map_selection(const std::shared_ptr<spdlog::logger> &logger)
{
    TestAtlas atlas;
    atlas.SetLoggers(logger, logger);
    require(atlas.CollectMapOutput().status == ORB_SLAM3::MapOutputStatus::NoActiveMap,
            "Collection created a map instead of reporting absence");
    atlas.CreateNewMap();
    ORB_SLAM3::Map *first = atlas.GetCurrentMap();
    const auto empty = atlas.CollectMapOutput();
    require(empty.status == ORB_SLAM3::MapOutputStatus::Ready && empty.map_points.empty(),
            "Valid empty map was reported unavailable");
    atlas.CreateNewMap();
    ORB_SLAM3::Map *second = atlas.GetCurrentMap();
    const auto switched = atlas.CollectMapOutput();
    require(switched.map_id == second->GetId() && switched.generation > empty.generation,
            "Collection retained the previous active map");
    atlas.ChangeMap(first);
    require(atlas.CollectMapOutput().generation > switched.generation,
            "Returning to a prior map did not replace sampled active context");
    atlas.clearAtlas();
    // Legacy clearAtlas detaches maps without deleting them. Reclaim only fixtures.
    const std::unique_ptr<ORB_SLAM3::Map> first_owner(first), second_owner(second);
    require(atlas.CollectMapOutput().status == ORB_SLAM3::MapOutputStatus::NoActiveMap,
            "Atlas reset retained active-map data");
}

template<class Lock, class Mutex>
void test_contention(ORB_SLAM3::Atlas &atlas, Mutex &mutex)
{
    std::promise<void> entered, release;
    auto done = release.get_future();
    auto worker = std::async(std::launch::async, [&]() {
        const Lock lock(mutex);
        entered.set_value();
        done.wait();
    });
    entered.get_future().wait();
    ORB_SLAM3::MapOutputSnapshot result;
    try { result = atlas.CollectMapOutput(); }
    catch (...) { release.set_value(); worker.get(); throw; }
    release.set_value();
    worker.get();
    require(result.status == ORB_SLAM3::MapOutputStatus::Unavailable &&
            result.map_points.empty() && result.keyframes.empty(),
            "Busy collection blocked, fabricated emptiness, or returned partial data");
    require(atlas.CollectMapOutput().status == ORB_SLAM3::MapOutputStatus::Ready,
            "Skipped collection poisoned the next sample");
}

void test_admission(const std::shared_ptr<spdlog::logger> &logger)
{
    TestAtlas atlas(0, logger, logger);
    test_contention<std::shared_lock<std::shared_mutex>>(atlas, atlas.output_mutex());
    test_contention<std::unique_lock<std::shared_mutex>>(atlas, atlas.output_mutex());
    test_contention<std::unique_lock<std::mutex>>(atlas, atlas.atlas_mutex());
    test_contention<std::unique_lock<std::mutex>>(atlas, atlas.GetCurrentMap()->mMutexMapUpdate);
    // Two concurrent writers can enter; collection is excluded until BOTH exit.
    std::promise<void> release;
    auto ready = release.get_future().share();
    std::promise<void> first, second;
    auto write = [&](std::promise<void> &entered) {
        const ORB_SLAM3::MapOutputMutation admission = atlas.AdmitMapOutputMutation();
        entered.set_value();
        ready.wait();
    };
    auto a = std::async(std::launch::async, [&]() { write(first); });
    auto b = std::async(std::launch::async, [&]() { write(second); });
    first.get_future().wait();
    second.get_future().wait();
    const auto busy = atlas.CollectMapOutput();
    release.set_value();
    a.get();
    b.get();
    require(busy.status == ORB_SLAM3::MapOutputStatus::Unavailable,
            "Worker admission serialized writers or admitted a partial snapshot");
    try
    {
        const ORB_SLAM3::MapOutputMutation admission = atlas.AdmitMapOutputMutation();
        throw std::runtime_error("prepared partial mutation");
    }
    catch (const std::runtime_error &) {}
    bool rejected = false;
    try { atlas.CollectMapOutput(); }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "Failed mutation was published before worker failure reporting");
}

void test_mutation_batch(const std::shared_ptr<spdlog::logger> &logger)
{
    ORB_SLAM3::Atlas atlas(0, logger, logger);
    ORB_SLAM3::Map &map = *atlas.GetCurrentMap();
    TestMapPoint first, second;
    for (TestMapPoint *point : {&first, &second})
    {
        point->UpdateMap(&map);
        point->SetWorldPos(Eigen::Vector3f::Zero());
        map.AddMapPoint(point);
    }
    const auto before = atlas.CollectMapOutput();
    std::promise<void> half_written, release;
    auto resume = release.get_future();
    auto writer = std::async(std::launch::async, [&]() {
        const ORB_SLAM3::MapOutputMutation admission = atlas.AdmitMapOutputMutation();
        first.SetWorldPos(Eigen::Vector3f::Ones());
        half_written.set_value();
        resume.wait();
        second.SetWorldPos(Eigen::Vector3f::Ones());
    });
    half_written.get_future().wait();
    ORB_SLAM3::MapOutputSnapshot during;
    try { during = atlas.CollectMapOutput(); }
    catch (...) { release.set_value(); writer.get(); throw; }
    release.set_value();
    writer.get();
    const auto after = atlas.CollectMapOutput();
    require(during.status == ORB_SLAM3::MapOutputStatus::Unavailable &&
            after.status == ORB_SLAM3::MapOutputStatus::Ready && after.map_points.size() == 2 &&
            after.map_points[0] == std::array<float, 3>{{1, 1, 1}} &&
            after.map_points[1] == std::array<float, 3>{{1, 1, 1}} &&
            before.map_points[0] == std::array<float, 3>{} &&
            before.generation == after.generation,
            "Collection mixed a partial geometry commit or changed owned history");
}

void test_adapter()
{
    using namespace gemini336_orbslam3;
    ORB_SLAM3::MapOutputSnapshot core;
    unsigned calls = 0;
    auto adapter = OrbSlam3AdapterTestAccess::make([&]() { ++calls; return core; });
    require(adapter->collectMapOutput().status == MapOutputStatus::Unavailable &&
            !adapter->collectMapOutput().data, "Adapter fabricated unavailable data");
    core.status = ORB_SLAM3::MapOutputStatus::NoActiveMap;
    const auto absent = adapter->collectMapOutput();
    require(absent.status == MapOutputStatus::NoActiveMap && !absent.data,
            "Adapter lost absence semantics");
    core.status = ORB_SLAM3::MapOutputStatus::Ready;
    core.map_id = 9;
    core.generation = 7;
    core.map_points.push_back({{1, 2, 3}});
    ORB_SLAM3::MapOutputKeyFrame keyframe;
    keyframe.id = 3;
    keyframe.timestamp = 4.25;
    keyframe.camera_pose.translation = {{4, 5, 6}};
    keyframe.camera_pose.quaternion_xyzw = {{0, 0, 0, 1}};
    core.keyframes.push_back(keyframe);
    const auto saved = adapter->collectMapOutput();
    require(saved.status == MapOutputStatus::Ready && saved.data &&
            saved.data->context.map_id == 9 && saved.data->context.generation == 7 &&
            saved.data->map_points[0][2] == 3 && saved.data->keyframes[0].timestamp == 4.25 &&
            saved.data->keyframes[0].translation_wc[1] == 5 &&
            saved.data->keyframes[0].quaternion_wc_xyzw[3] == 1,
            "Adapter lost map identity, timestamps, or value fields");
    core.map_points.clear();
    core.keyframes.clear();
    const auto empty = adapter->collectMapOutput();
    require(empty.status == MapOutputStatus::Ready && empty.data && empty.data->map_points.empty() &&
            saved.data->map_points.size() == 1, "Empty map or independent result ownership was lost");
    adapter->shutdown();
    const unsigned before = calls;
    bool rejected = false;
    try { adapter->collectMapOutput(); }
    catch (const std::logic_error &) { rejected = true; }
    require(rejected && calls == before, "Collection accessed Core after shutdown");
}
}

int main()
{
    try
    {
        const auto logger = std::make_shared<spdlog::logger>("map_output_test",
            std::make_shared<spdlog::sinks::null_sink_mt>());
        test_values(logger);
        test_map_selection(logger);
        test_admission(logger);
        test_mutation_batch(logger);
        test_adapter();
        std::cout << "map_output_test passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "map_output_test failed: " << error.what() << '\n';
        return 1;
    }
}
