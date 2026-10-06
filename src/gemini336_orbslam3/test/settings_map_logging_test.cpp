#include "gemini336_orbslam3/logging.hpp"
#include <Atlas.h>
#include <Settings.h>
#include <System.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <sys/wait.h>
#include <unistd.h>

namespace
{
void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}

std::string read_file(const std::filesystem::path &path)
{
    std::ifstream file(path);
    require(file.good(), "Cannot read test artifact");
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

void write_file(const std::filesystem::path &path, const std::string &text)
{
    std::ofstream file(path);
    file << text;
    require(file.good(), "Cannot write settings fixture");
}

// The existing PreSave path discards empty maps. Prepare an empty serialized
// fixture directly so this test needs no real frames, vocabulary or optimizer.
class ArchiveMap : public ORB_SLAM3::Map
{
public:
    ArchiveMap() : Map(0)
    {
        mnBackupKFinitialID = std::numeric_limits<unsigned long>::max();
        mnBackupKFlowerID = std::numeric_limits<unsigned long>::max();
    }
};

class ArchiveAtlas : public ORB_SLAM3::Atlas
{
public:
    ArchiveAtlas()
    {
        mnLastInitKFidMap = 0;
        ORB_SLAM3::Map* map = new ArchiveMap();
        mspMaps.insert(map);
        mvpBackupMaps.push_back(map);
    }
};

std::string serialize_atlas(ORB_SLAM3::Atlas *atlas)
{
    std::ostringstream data;
    {
        boost::archive::binary_oarchive archive(data);
        archive << atlas;
    }
    return data.str();
}
}

int main()
{
    char temporary[] = "/tmp/gemini336-settings-map-XXXXXX";
    const char *created = mkdtemp(temporary);
    if (!created) return 1;
    const std::filesystem::path root(created);
    try
    {
        const std::string valid_settings = R"(%YAML:1.0
Camera.type: PinHole
Camera1.fx: 400.0
Camera1.fy: 400.0
Camera1.cx: 320.0
Camera1.cy: 240.0
Camera.width: 640
Camera.height: 480
Camera.fps: 30
Camera.RGB: 0
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
)";
        const auto settings_path = root / "settings.yaml";
        write_file(settings_path, valid_settings);
        gemini336_orbslam3::LoggingOptions options;
        options.directory = root / "normal";
        {
            gemini336_orbslam3::LoggingSession session(options);
            std::string report_text;
            {
                ORB_SLAM3::Settings settings(settings_path.string(), ORB_SLAM3::System::MONOCULAR,
                    session.GetLogger("settings"), session.GetSynchronousLogger("settings"));
                std::ostringstream original_report;
                original_report << settings << std::endl;
                report_text = original_report.str();
                ORB_SLAM3::LogStream(session.GetLogger("system"), spdlog::level::info,
                    [&settings](std::ostream &output) { output << settings << std::endl; });
                bool evaluated = false;
                ORB_SLAM3::LogStream(session.GetLogger("system"), spdlog::level::debug,
                    [&evaluated](std::ostream &) { evaluated = true; });
                require(!evaluated, "Disabled report was evaluated");

                ORB_SLAM3::KeyFrame keyframe;
                keyframe.mnId = 42;
                ORB_SLAM3::Atlas atlas(0, session.GetLogger("atlas"), session.GetLogger("map"));
                require(atlas.GetCurrentMap()->GetLogger() == session.GetLogger("map"),
                        "Initial map lost injected logger");
                atlas.GetCurrentMap()->AddKeyFrame(&keyframe);
                atlas.CreateNewMap();
                require(atlas.GetCurrentMap()->GetLogger() == session.GetLogger("map"),
                        "New map lost injected logger");

                ArchiveAtlas original;
                original.SetLoggers(session.GetLogger("atlas"), session.GetLogger("map"));
                const std::string archive_data = serialize_atlas(&original);
                original.SetLoggers(session.GetLogger("different_atlas"), session.GetLogger("different_map"));
                require(archive_data == serialize_atlas(&original), "Logger handles changed archive bytes");
                std::istringstream data(archive_data);
                ORB_SLAM3::Atlas *loaded_pointer = nullptr;
                {
                    boost::archive::binary_iarchive archive(data);
                    archive >> loaded_pointer;
                }
                std::unique_ptr<ORB_SLAM3::Atlas> loaded(loaded_pointer);
                loaded->SetLoggers(session.GetLogger("atlas"), session.GetLogger("map"));
                loaded->SetKeyFrameDababase(nullptr);
                loaded->SetORBVocabulary(nullptr);
                loaded->PostLoad();
                require(loaded->GetAllMaps().size() == 1 &&
                        loaded->GetAllMaps().front()->GetLogger() == session.GetLogger("map"),
                        "Restored map did not reconnect to session");
                loaded->CreateNewMap();
                require(loaded->GetCurrentMap()->GetLogger() == session.GetLogger("map"),
                        "Map created after load lost session");
            }
            // Core objects stop producing before session drain/flush.
            session.finish();
            const std::string log = read_file(session.directory() / "slam.log");
            require(log.find("[settings] [info] Loading settings from") != std::string::npos &&
                    log.find("[settings] [warning] Camera1.k1 optional parameter") != std::string::npos &&
                    log.find("[atlas] [info] Creation of new map") != std::string::npos &&
                    log.find("[map] [info] First KF:42; Map init KF:0") != std::string::npos,
                    "Settings/atlas/map messages did not share session file");
            std::istringstream original_lines(report_text);
            std::string line;
            while (std::getline(original_lines, line))
                require(log.find("[system] [info] " + line + "\n") != std::string::npos,
                        "Settings report lost content or per-line module prefix");
            require(session.dropped_messages() == 0, "Unexpected stage 2 queue loss");
        }

        // Parent has no live asynchronous session at fork. Child exercises the
        // actual Settings exit paths, without constructing a System or worker.
        const std::string invalid[] = {
            "%YAML:1.0\nCamera.type: PinHole\n",
            "%YAML:1.0\nCamera.type: PinHole\nCamera1.fx: 400\n",
            "%YAML:1.0\nCamera.type: UnknownCamera\n"
        };
        const std::string expected[] = {
            "Camera1.fx required parameter does not exist, aborting...",
            "Camera1.fx parameter must be a real number, aborting...",
            "Error: UnknownCamera not known",
            "[ERROR]: could not open configuration file at:"
        };
        for (int index = 0; index < 4; ++index)
        {
            const auto fixture = root / ("invalid_" + std::to_string(index) + ".yaml");
            if (index < 3) write_file(fixture, invalid[index]);
            const auto child_root = root / ("fatal_" + std::to_string(index));
            const pid_t child = fork();
            require(child != -1, "Cannot fork settings failure test");
            if (child == 0)
            {
                try
                {
                    options.directory = child_root;
                    gemini336_orbslam3::LoggingSession session(options);
                    ORB_SLAM3::Settings settings(fixture.string(), ORB_SLAM3::System::MONOCULAR,
                        session.GetLogger("settings"), session.GetSynchronousLogger("settings"));
                    std::_Exit(17); // Reaching here means the original exit path was lost.
                }
                catch (...) { std::_Exit(18); }
            }
            int status = 0;
            require(waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 255,
                    "Settings failure changed exit(-1) behavior");
            const std::string log = read_file(child_root / "latest" / "slam.log");
            require(log.find("[settings] [critical] " + expected[index]) != std::string::npos,
                    "Fatal settings diagnostic was not flushed before exit");
        }
        std::filesystem::remove_all(root);
    }
    catch (const std::exception &error)
    {
        std::fprintf(stderr, "%s\n", error.what());
        std::filesystem::remove_all(root);
        return 1;
    }
    return 0;
}
