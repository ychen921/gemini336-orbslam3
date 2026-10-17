#include <Atlas.h>
#include <KeyFrame.h>
#include <TrajectoryExport.h>
#include <spdlog/sinks/null_sink.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <list>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using ORB_SLAM3::TrajectoryExportStatus;
using ORB_SLAM3::trajectory_export::SaveEvaluation;

void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}

template<class Exception, class Function>
void require_throws(Function&& function, const char* message)
{
    bool caught = false;
    try { function(); }
    catch (const Exception&) { caught = true; }
    require(caught, message);
}

class TemporaryDirectory
{
public:
    TemporaryDirectory()
    {
        char pattern[] = "/tmp/gemini336-trajectory-XXXXXX";
        const char* created = mkdtemp(pattern);
        if (!created) throw std::runtime_error("Cannot create trajectory fixture directory");
        path = created;
    }
    ~TemporaryDirectory()
    {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }
    std::filesystem::path path;
};

// All fixtures are finite maps/history lists. They never construct System,
// start workers, load a vocabulary, optimize a map or launch ROS/GUI/replay.
struct History
{
    std::list<Sophus::SE3f> poses;
    std::list<ORB_SLAM3::KeyFrame*> references;
    std::list<double> times;
    std::list<bool> lost;

    void add(const Sophus::SE3f& pose, ORB_SLAM3::KeyFrame* reference,
             double time, bool is_lost = false)
    {
        poses.push_back(pose);
        references.push_back(reference);
        times.push_back(time);
        lost.push_back(is_lost);
    }
    ORB_SLAM3::trajectory_export::FrameHistory view() const
    {
        return {poses, references, times, lost};
    }
};

class TestKeyFrame : public ORB_SLAM3::KeyFrame
{
public:
    void culled(ORB_SLAM3::KeyFrame* parent, const Sophus::SE3f& child_from_parent)
    {
        mbBad = true;
        mpParent = parent;
        mTcp = child_from_parent;
    }
};

void prepare_keyframe(ORB_SLAM3::KeyFrame& keyframe, ORB_SLAM3::Map& map,
                      unsigned long id, const Sophus::SE3f& camera_in_map,
                      const Sophus::SE3f* body_from_camera = nullptr)
{
    keyframe.mnId = id;
    keyframe.UpdateMap(&map);
    if (body_from_camera) keyframe.mImuCalib.Set(*body_from_camera, 1, 1, 1, 1);
    keyframe.SetPose(camera_in_map.inverse());
    map.AddKeyFrame(&keyframe);
}

std::vector<std::array<double, 8>> read_rows(const std::filesystem::path& path)
{
    std::ifstream input(path);
    require(input.good(), "Trajectory output is missing");
    std::vector<std::array<double, 8>> rows;
    std::string line;
    while (std::getline(input, line))
    {
        std::istringstream text(line);
        std::array<double, 8> row{};
        for (double& value : row)
            require(static_cast<bool>(text >> value) && std::isfinite(value), "Malformed trajectory row");
        std::string extra;
        require(!(text >> extra), "Trajectory row has unexpected columns");
        rows.push_back(row);
    }
    return rows;
}

Sophus::SE3f row_pose(const std::array<double, 8>& row)
{
    return Sophus::SE3f(Eigen::Quaternionf(row[7], row[4], row[5], row[6]),
                       Eigen::Vector3f(row[1], row[2], row[3]));
}

Sophus::SE3f metadata_pose(const ORB_SLAM3::TrajectoryTransform& pose)
{
    const auto& q = pose.quaternion_xyzw;
    const auto& t = pose.translation;
    return Sophus::SE3f(Eigen::Quaternionf(q[3], q[0], q[1], q[2]), Eigen::Vector3f(t[0], t[1], t[2]));
}

void test_reconstruction(const std::shared_ptr<spdlog::logger>& logger, bool inertial)
{
    TemporaryDirectory directory;
    ORB_SLAM3::Atlas atlas(0, logger, logger);
    ORB_SLAM3::Map* map = atlas.GetCurrentMap();
    ORB_SLAM3::KeyFrame first;
    ORB_SLAM3::KeyFrame last;
    const Sophus::SE3f body_from_camera(
        Eigen::Quaternionf(Eigen::AngleAxisf(0.2f, Eigen::Vector3f::UnitY())),
        Eigen::Vector3f(0.1f, -0.02f, 0.03f));
    const Sophus::SE3f origin(
        Eigen::Quaternionf(Eigen::AngleAxisf(0.7f, Eigen::Vector3f::UnitZ())),
        Eigen::Vector3f(10, -2, 3));
    const Sophus::SE3f motion(Sophus::SO3f(), Eigen::Vector3f(3, 4, 0));
    const Sophus::SE3f first_camera = inertial ? origin * body_from_camera : origin;
    const Sophus::SE3f last_camera = inertial ? origin * motion * body_from_camera : origin * motion;
    prepare_keyframe(first, *map, 10, first_camera, inertial ? &body_from_camera : nullptr);
    prepare_keyframe(last, *map, 20, last_camera, inertial ? &body_from_camera : nullptr);
    if (inertial) map->SetImuInitialized();
    History history;
    history.add(Sophus::SE3f(), &first, 100.0);
    history.add(Sophus::SE3f(), &last, 101.0);
    const auto frames = directory.path / "CameraTrajectory.txt";
    const auto keyframes = directory.path / "KeyFrameTrajectory.txt";
    const auto result = SaveEvaluation(true, atlas, history.view(), inertial, frames.string(), keyframes.string());
    require(result.status == TrajectoryExportStatus::Complete && result.map_selected &&
            result.map_id == map->GetId() && result.map_keyframes == 2 && result.origin_keyframe_id == 10,
            "Export did not describe the selected final map");
    require(result.frames.written == 2 && result.keyframes.written == 2 &&
            result.frames.first_timestamp_sec == 100.0 && result.frames.last_timestamp_sec == 101.0,
            "Export row counts/source times are wrong");
    require(result.body_from_camera_valid == inertial && result.imu_initialized == inertial &&
            result.pose_target == (inertial ? ORB_SLAM3::TrajectoryPoseTarget::ImuBody :
                                             ORB_SLAM3::TrajectoryPoseTarget::LeftCamera),
            "Camera/body identity or calibration validity is wrong");
    const auto frame_rows = read_rows(frames);
    const auto keyframe_rows = read_rows(keyframes);
    require(frame_rows.size() == 2 && keyframe_rows.size() == 2, "Missing trajectory rows");
    require(frame_rows[0][0] == 100e9 && frame_rows[1][0] == 101e9,
            "EuRoC timestamp units changed");
    require(row_pose(frame_rows[0]).matrix().isApprox(Eigen::Matrix4f::Identity(), 1e-5f) &&
            row_pose(frame_rows[1]).matrix().isApprox(motion.matrix(), 1e-5f),
            "Frame history was not reconstructed relative to the first sensor pose");
    require(std::abs(row_pose(frame_rows[1]).translation().norm() - 5.0f) < 1e-5f,
            "Known 3-4-5 endpoint distance changed");
    const Sophus::SE3f calibration = inertial ? metadata_pose(result.body_from_camera) : Sophus::SE3f();
    require((metadata_pose(result.frame_origin_in_map) * row_pose(frame_rows[1]) * calibration).matrix()
                .isApprox(last_camera.matrix(), 1e-5f) &&
            (row_pose(keyframe_rows[1]) * calibration).matrix().isApprox(last_camera.matrix(), 1e-5f),
            "Returned metadata cannot reconcile frame/KF origins and camera/body frames");

    // A pose-graph correction moves the reference KF after tracking recorded the
    // relative pose. Export must reconstruct from its current, corrected pose.
    const Sophus::SE3f corrected_motion(
        Eigen::Quaternionf(Eigen::AngleAxisf(0.3f, Eigen::Vector3f::UnitX())),
        Eigen::Vector3f(6, 8, 0));
    const Sophus::SE3f corrected_camera = origin * corrected_motion * calibration;
    last.SetPose(corrected_camera.inverse());
    const auto corrected = SaveEvaluation(true, atlas, history.view(), inertial, frames.string(), keyframes.string());
    require(corrected.status == TrajectoryExportStatus::Complete &&
            row_pose(read_rows(frames)[1]).matrix().isApprox(corrected_motion.matrix(), 1e-5f),
            "Historical frames did not follow their corrected reference keyframe");

    // A later calibration mismatch is counted, rather than mixing sensor frames
    // under a single returned Tbc. An invalid origin is rejected before writing.
    if (inertial)
    {
        last.mImuCalib.Set(Sophus::SE3f(Sophus::SO3f(), Eigen::Vector3f(2, 0, 0)), 1, 1, 1, 1);
        history.add(Sophus::SE3f(), &last, 102.0);
        const auto mismatch = SaveEvaluation(true, atlas, history.view(), true, frames.string(), keyframes.string());
        require(mismatch.frames.invalid_pose == 2 && mismatch.keyframes.invalid_pose == 1,
                "Incompatible calibrations were silently mixed");
        first.mImuCalib.mTcb.translation().x() += 1.0f;
        require_throws<std::invalid_argument>([&]() {
            SaveEvaluation(true, atlas, history.view(), true, frames.string(), keyframes.string());
        }, "Non-inverse Tbc/Tcb was accepted");
    }
}

void test_selection_and_invalid_history(const std::shared_ptr<spdlog::logger>& logger)
{
    TemporaryDirectory directory;
    ORB_SLAM3::Atlas atlas(0, logger, logger);
    ORB_SLAM3::Map* selected = atlas.GetCurrentMap();
    TestKeyFrame first, culled, broken, cycle, other;
    prepare_keyframe(first, *selected, 10, Sophus::SE3f());
    prepare_keyframe(culled, *selected, 11, Sophus::SE3f());
    prepare_keyframe(broken, *selected, 12, Sophus::SE3f());
    prepare_keyframe(cycle, *selected, 13, Sophus::SE3f());
    culled.culled(&first, Sophus::SE3f(Sophus::SO3f(), Eigen::Vector3f(-1, 0, 0)));
    broken.culled(nullptr, Sophus::SE3f());
    cycle.culled(&cycle, Sophus::SE3f());
    atlas.CreateNewMap();
    ORB_SLAM3::Map* other_map = atlas.GetCurrentMap();
    prepare_keyframe(other, *other_map, 20, Sophus::SE3f());
    require(ORB_SLAM3::trajectory_export::SelectMap({other_map, nullptr, selected}) == selected,
            "Map selection depends on pointer order or includes culled KFs");
    History history;
    history.add(Sophus::SE3f(), &first, 100);
    history.add(Sophus::SE3f(), &culled, 101);
    history.add(Sophus::SE3f(), nullptr, 102);
    history.add(Sophus::SE3f(), &broken, 103);
    history.add(Sophus::SE3f(), &cycle, 104);
    history.add(Sophus::SE3f(), &other, 105);
    Sophus::SE3f invalid;
    invalid.translation().x() = std::numeric_limits<float>::quiet_NaN();
    history.add(invalid, &first, 106);
    history.add(Sophus::SE3f(), &first, std::numeric_limits<double>::quiet_NaN());
    history.add(Sophus::SE3f(), &first, 108, true);
    const auto frames = directory.path / "frames.txt";
    const auto keyframes = directory.path / "keyframes.txt";
    const auto result = SaveEvaluation(true, atlas, history.view(), false, frames.string(), keyframes.string());
    require(result.map_id == selected->GetId() && result.map_keyframes == 1 &&
            result.frames.considered == 9 && result.frames.written == 2 && result.frames.lost == 1 &&
            result.frames.invalid_reference == 3 && result.frames.other_map == 1 &&
            result.frames.invalid_pose == 1 && result.frames.invalid_timestamp == 1,
            "Lost/null/cyclic/foreign/nonfinite history accounting is wrong");
    const auto rows = read_rows(frames);
    require(rows.size() == 2 && std::abs(rows[1][1] - 1.0) < 1e-6,
            "Culled reference spanning-tree reconstruction changed");
    TestKeyFrame extra;
    prepare_keyframe(extra, *other_map, 21, Sophus::SE3f());
    const auto larger = SaveEvaluation(true, atlas, history.view(), false, frames.string(), keyframes.string());
    require(larger.map_id == other_map->GetId() && larger.keyframes.written == 2 && larger.frames.written == 1,
            "The trajectory pair did not use the same largest final map");
}

void test_preconditions_and_io(const std::shared_ptr<spdlog::logger>& logger)
{
    TemporaryDirectory directory;
    ORB_SLAM3::Atlas atlas(0, logger, logger);
    History history;
    const auto frames = directory.path / "frames.txt";
    const auto keyframes = directory.path / "keyframes.txt";
    require_throws<std::logic_error>([&]() {
        SaveEvaluation(false, atlas, history.view(), false, frames.string(), keyframes.string());
    }, "Shutdown entry was mistaken for completed shutdown");
    require(!std::filesystem::exists(frames), "Precondition failure touched an output file");
    const auto empty = SaveEvaluation(true, atlas, history.view(), false, frames.string(), keyframes.string());
    require(empty.status == TrajectoryExportStatus::NoKeyFrames && !empty.map_selected &&
            !std::filesystem::exists(frames), "Empty Atlas was reported as a successful export");
    ORB_SLAM3::Atlas no_maps;
    no_maps.SetLoggers(logger, logger);
    require(SaveEvaluation(true, no_maps, history.view(), false, frames.string(), keyframes.string()).status ==
            TrajectoryExportStatus::NoMap, "No maps and no keyframes were conflated");

    ORB_SLAM3::KeyFrame first;
    prepare_keyframe(first, *atlas.GetCurrentMap(), 10, Sophus::SE3f());
    const auto no_frames = SaveEvaluation(true, atlas, history.view(), false, frames.string(), keyframes.string());
    require(no_frames.status == TrajectoryExportStatus::NoFramePoses && no_frames.map_selected &&
            no_frames.frames.written == 0 && no_frames.keyframes.written == 1,
            "A keyframe-only export was accepted as a complete frame trajectory");
    const auto original_rows = read_rows(keyframes);
    require_throws<std::invalid_argument>([&]() {
        SaveEvaluation(true, atlas, history.view(), false, "", keyframes.string());
    }, "An empty output path was accepted");
    require_throws<std::invalid_argument>([&]() {
        SaveEvaluation(true, atlas, history.view(), false, keyframes.string(), keyframes.string());
    }, "Identical output paths were accepted");
    const auto alias = directory.path / "alias.txt";
    std::filesystem::create_symlink(keyframes, alias);
    require_throws<std::invalid_argument>([&]() {
        SaveEvaluation(true, atlas, history.view(), false, keyframes.string(), alias.string());
    }, "Aliased output paths were accepted");
    require(read_rows(keyframes) == original_rows, "Rejected paths overwrote an existing trajectory");
    history.add(Sophus::SE3f(), &first, 100);
    history.lost.clear();
    require_throws<std::invalid_argument>([&]() {
        SaveEvaluation(true, atlas, history.view(), false, frames.string(), keyframes.string());
    }, "Mismatched history lengths were accepted");
    history.lost.push_back(false);
    require_throws<std::ios_base::failure>([&]() {
        SaveEvaluation(true, atlas, history.view(), false,
                       (directory.path / "missing/frames.txt").string(), keyframes.string());
    }, "Output open failure was hidden");
    require_throws<std::ios_base::failure>([&]() {
        SaveEvaluation(true, atlas, history.view(), false, frames.string(), directory.path.string());
    }, "Second-file failure was reported as a complete trajectory pair");
    // /dev/full accepts open but rejects writes, including a final buffered close.
    require_throws<std::ios_base::failure>([&]() {
        SaveEvaluation(true, atlas, history.view(), false, "/dev/full", keyframes.string());
    }, "Buffered write/close failure was reported as success");
}
}

int main()
{
    try
    {
        const auto logger = std::make_shared<spdlog::logger>("trajectory_fixture",
            std::make_shared<spdlog::sinks::null_sink_mt>());
        test_reconstruction(logger, false);
        test_reconstruction(logger, true);
        test_selection_and_invalid_history(logger);
        test_preconditions_and_io(logger);
        std::cout << "Trajectory export tests passed\n";
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
