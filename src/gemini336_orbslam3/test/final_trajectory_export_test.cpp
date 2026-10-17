#include "common/evaluation_recorder.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>

using namespace gemini336_orbslam3;

namespace
{
void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}

class TemporaryDirectory
{
public:
    TemporaryDirectory()
    {
        char pattern[] = "/tmp/gemini-final-trajectory-XXXXXX";
        const char *created = mkdtemp(pattern);
        if (!created) throw std::runtime_error("Cannot create final trajectory fixture directory");
        path = created;
    }
    ~TemporaryDirectory()
    {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }
    std::filesystem::path path;
};

void write(const std::filesystem::path &path, const std::string &text)
{
    std::ofstream output;
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.open(path);
    output << text;
    output.close();
}

std::string read(const std::filesystem::path &path)
{
    std::ifstream input(path);
    require(input.good(), "Expected artifact is missing");
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

FinalTrajectoryExportResult complete_result(bool inertial = false)
{
    FinalTrajectoryExportResult result;
    result.status = FinalTrajectoryStatus::Complete;
    result.map_selected = true;
    result.map_keyframes = 1;
    result.origin_timestamp_sec = 1.25;
    result.pose_target = inertial ? FinalTrajectoryPoseTarget::ImuBody : FinalTrajectoryPoseTarget::LeftCamera;
    result.imu_initialized = inertial;
    result.body_from_camera_valid = inertial;
    result.body_from_camera.translation = {{0.1, 0.2, 0.3}};
    result.frame_origin_in_map.translation = {{1, 2, 3}};
    result.frames = {1, 1, 0, 0, 0, 0, 0, 1.25, 1.25};
    result.keyframes = result.frames;
    return result;
}

const std::string row = "1250000000.000000 0 0 0 0 0 0 1\n";

void test_success(bool inertial, const std::filesystem::path &artifacts)
{
    TemporaryDirectory directory;
    EvaluationMetadata metadata;
    metadata.sensor_mode = inertial ? "stereo_imu" : "stereo";
    metadata.run_id = "run\"id\n";
    metadata.final_trajectory_enabled = true;
    write(directory.path / "frames.csv", "original frame data\n");
    write(directory.path / "export_complete.txt", "original marker\n");
    int calls = 0;
    const auto result = write_evaluation_final_trajectory(directory.path, metadata,
        [&](const std::filesystem::path &frames, const std::filesystem::path &keyframes) {
            ++calls;
            require(frames.parent_path() == directory.path / "final_trajectory.tmp" &&
                    frames.filename() == "CameraTrajectory.txt" && keyframes.filename() == "KeyFrameTrajectory.txt" &&
                    !std::filesystem::exists(directory.path / "final_trajectory"),
                    "Trajectory was published before the exporter finished");
            write(frames, row);
            write(keyframes, row);
            return complete_result(inertial);
        });
    const auto published = directory.path / "final_trajectory";
    const std::string manifest = read(published / "manifest.json");
    require(calls == 1 && result.status == FinalTrajectoryStatus::Complete &&
            !std::filesystem::exists(directory.path / "final_trajectory.tmp") &&
            read(published / "CameraTrajectory.txt") == row && read(published / "KeyFrameTrajectory.txt") == row,
            "A complete trajectory pair was not published together");
    require(manifest.find("\"export_complete\": true") != std::string::npos &&
            manifest.find("\"available\": true") != std::string::npos &&
            manifest.find("\"map_id\": 0") != std::string::npos &&
            manifest.find("\"origin_keyframe_id\": 0") != std::string::npos &&
            manifest.find("run\\\"id\\u000a") != std::string::npos &&
            manifest.find("\"frame_pose_convention\": \"T_origin_sensor\"") != std::string::npos &&
            manifest.find("\"keyframe_pose_convention\": \"T_map_sensor\"") != std::string::npos &&
            manifest.find("\"bytes\": " + std::to_string(row.size())) != std::string::npos &&
            manifest.find("\"all_gba_completed\": null") != std::string::npos,
            "Manifest lost zero-valued IDs, file information, escaping or coordinate scope");
    require(manifest.find(inertial ? "\"pose_target\": \"imu_body\"" : "\"pose_target\": \"left_camera\"") !=
            std::string::npos &&
            (manifest.find("\"body_from_camera\": null") != std::string::npos) == !inertial,
            "Manifest camera/body metadata is incorrect");
    require(read(directory.path / "frames.csv") == "original frame data\n" &&
            read(directory.path / "export_complete.txt") == "original marker\n",
            "Post-join publication changed pre-join data or its completion marker");
    bool rejected = false;
    try
    {
        write_evaluation_final_trajectory(directory.path, metadata,
            [&](const std::filesystem::path &, const std::filesystem::path &) { ++calls; return complete_result(inertial); });
    }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected && calls == 1 && read(published / "manifest.json") == manifest,
            "Repeated publication overwrote a completed trajectory");
    if (!artifacts.empty())
        std::filesystem::copy(published, artifacts / metadata.sensor_mode, std::filesystem::copy_options::recursive);
}

void test_no_data(const std::filesystem::path &artifacts)
{
    for (FinalTrajectoryStatus status : {FinalTrajectoryStatus::NoMap, FinalTrajectoryStatus::NoKeyFrames,
                                       FinalTrajectoryStatus::NoFramePoses})
    {
        TemporaryDirectory directory;
        EvaluationMetadata metadata;
        metadata.sensor_mode = "stereo";
        write_evaluation_final_trajectory(directory.path, metadata,
            [&](const std::filesystem::path &frames, const std::filesystem::path &keyframes) {
                FinalTrajectoryExportResult result;
                if (status == FinalTrajectoryStatus::NoFramePoses)
                {
                    result = complete_result();
                    result.frames = {};
                    write(frames, "");
                    write(keyframes, row);
                }
                result.status = status;
                return result;
            });
        const std::string manifest = read(directory.path / "final_trajectory/manifest.json");
        require(manifest.find("\"available\": false") != std::string::npos &&
                manifest.find(std::string("\"status\": \"") + final_trajectory_status_name(status) + '"') !=
                    std::string::npos &&
                manifest.find("\"first_timestamp_sec\": null") != std::string::npos,
                "A no-data export fabricated an available trajectory or timestamp");
        if (!artifacts.empty())
            std::filesystem::copy(directory.path / "final_trajectory", artifacts / final_trajectory_status_name(status),
                                  std::filesystem::copy_options::recursive);
    }
}

void test_failures()
{
    for (const std::string failure : {"backend", "missing", "rows", "nan", "manifest", "collision", "target"})
    {
        TemporaryDirectory directory;
        EvaluationMetadata metadata;
        metadata.sensor_mode = "stereo";
        write(directory.path / "frames.csv", "preserve\n");
        bool failed = false;
        try
        {
            write_evaluation_final_trajectory(directory.path, metadata,
                [&](const std::filesystem::path &frames, const std::filesystem::path &keyframes) {
                    write(frames, row);
                    if (failure == "backend") throw std::runtime_error("injected Core write failure");
                    if (failure != "missing") write(keyframes, row);
                    auto result = complete_result();
                    if (failure == "rows") result.frames.considered = result.frames.written = 2;
                    if (failure == "nan") result.frame_origin_in_map.translation[0] = std::numeric_limits<double>::quiet_NaN();
                    if (failure == "target") result.pose_target = FinalTrajectoryPoseTarget::ImuBody;
                    if (failure == "manifest") std::filesystem::create_directory(frames.parent_path() / "manifest.json.tmp");
                    if (failure == "collision")
                    {
                        std::filesystem::create_directory(directory.path / "final_trajectory");
                        write(directory.path / "final_trajectory/sentinel", "keep");
                    }
                    return result;
                });
        }
        catch (const std::exception &) { failed = true; }
        require(failed && !std::filesystem::exists(directory.path / "final_trajectory/manifest.json") &&
                std::filesystem::exists(directory.path / "final_trajectory.tmp/CameraTrajectory.txt") &&
                read(directory.path / "frames.csv") == "preserve\n",
                "Export failure published completion, discarded staging or changed raw data");
        if (failure == "collision")
            require(read(directory.path / "final_trajectory/sentinel") == "keep", "Publication overwrote another destination");
    }
    TemporaryDirectory directory;
    EvaluationMetadata metadata;
    metadata.sensor_mode = "stereo";
    std::filesystem::create_directory(directory.path / "final_trajectory.tmp");
    write(directory.path / "final_trajectory.tmp/sentinel", "keep");
    int calls = 0;
    bool rejected = false;
    try
    {
        write_evaluation_final_trajectory(directory.path, metadata,
            [&](const std::filesystem::path &, const std::filesystem::path &) { ++calls; return complete_result(); });
    }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected && calls == 0 && read(directory.path / "final_trajectory.tmp/sentinel") == "keep",
            "Existing staging was silently retried or overwritten");
}
}

int main(int argc, char **argv)
{
    try
    {
        const std::filesystem::path artifacts = argc == 2 ? argv[1] : "";
        test_success(false, artifacts);
        test_success(true, artifacts);
        test_no_data(artifacts);
        test_failures();
        std::cout << "Final trajectory publication tests passed\n";
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
