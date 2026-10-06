#include "gemini336_orbslam3/logging.hpp"
#include <Logging.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <sys/wait.h>
#include <unistd.h>

#include <spdlog/sinks/base_sink.h>

namespace
{
void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

std::string read_file(const std::filesystem::path &path)
{
    std::ifstream file(path);
    require(file.good(), "Cannot read test log");
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

// Standard sink errors must allow later sinks to write the same record.
class ThrowingSink : public spdlog::sinks::base_sink<std::mutex>
{
    void sink_it_(const spdlog::details::log_msg &) override { throw std::runtime_error("injected sink failure"); }
    void flush_() override {}
};

class UnknownThrowingSink : public spdlog::sinks::base_sink<std::mutex>
{
    void sink_it_(const spdlog::details::log_msg &) override { throw 42; }
    void flush_() override {}
};
}

int main()
{
    char temporary[] = "/tmp/gemini336-core-logging-XXXXXX";
    const char *created = mkdtemp(temporary);
    if (!created) return 1;
    const std::filesystem::path root(created);
    try
    {
        // No System, ROS, GUI, map or optimizer is constructed by these tests.
        const auto standalone = ORB_SLAM3::MakeLoggerFactory({});
        const auto system = ORB_SLAM3::GetModuleLogger(standalone, "system");
        const auto tracking = ORB_SLAM3::GetModuleLogger(standalone, "tracking", true);
        require(system->sinks().front() == tracking->sinks().front(), "Fallback sinks differ");
        require(system->should_log(spdlog::level::info) &&
                !system->should_log(spdlog::level::debug), "Fallback level changed");
        require(tracking->flush_level() == spdlog::level::trace, "Fallback fatal log does not flush");
        ORB_SLAM3::Log(system, spdlog::level::info, "STANDALONE_RECORD");
        bool null_rejected = false;
        try
        {
            ORB_SLAM3::GetModuleLogger([](const std::string &, bool) {
                return std::shared_ptr<spdlog::logger>{};
            }, "system");
        }
        catch (const std::runtime_error &) { null_rejected = true; }
        require(null_rejected, "Configured null factory silently fell back");
        bool failure_propagated = false;
        try
        {
            ORB_SLAM3::GetModuleLogger([](const std::string &, bool) -> std::shared_ptr<spdlog::logger> {
                throw std::runtime_error("factory failure");
            }, "system");
        }
        catch (const std::runtime_error &) { failure_propagated = true; }
        require(failure_propagated, "Factory startup failure was hidden");

        gemini336_orbslam3::LoggingOptions options;
        options.directory = root;
        {
            auto session = std::make_shared<gemini336_orbslam3::LoggingSession>(options);
            const ORB_SLAM3::LoggerFactory factory = ORB_SLAM3::MakeLoggerFactory(
                [weak = std::weak_ptr<gemini336_orbslam3::LoggingSession>(session)](
                    const std::string &module, bool synchronous) {
                    const auto logging = weak.lock();
                    if (!logging) throw std::logic_error("expired test session");
                    return synchronous ? logging->GetSynchronousLogger(module) : logging->GetLogger(module);
                });
            const auto async = ORB_SLAM3::GetModuleLogger(factory, "system");
            const auto sync = ORB_SLAM3::GetModuleLogger(factory, "system", true);
            require(async != sync && async->sinks() == sync->sinks(), "Factory used different sinks");
            require(sync == session->GetSynchronousLogger("system"), "Sync logger was not cached");
            std::vector<std::thread> producers;
            for (const std::string module : {"tracking", "local_mapping"})
            {
                const auto logger = ORB_SLAM3::GetModuleLogger(factory, module);
                producers.emplace_back([logger]() {
                    for (int index = 0; index < 20; ++index)
                        ORB_SLAM3::Log(logger, spdlog::level::info, "event=CORE_RECORD index={}", index);
                });
            }
            // Invalid formatting must never be evaluated below the configured level.
            ORB_SLAM3::Log(async, spdlog::level::debug, "FILTERED {1}", 0);
            ORB_SLAM3::Log(sync, spdlog::level::critical, "SYNC_RECORD");
            require(read_file(session->directory() / "slam.log").find("SYNC_RECORD") != std::string::npos,
                    "Synchronous return did not flush file");
            for (auto &producer : producers) producer.join();
            ORB_SLAM3::Log(async, spdlog::level::info, "LAST_RECORD");
            session->finish();
            const std::string records = read_file(session->directory() / "slam.log");
            require(records.find("[tracking]") != std::string::npos &&
                    records.find("[local_mapping]") != std::string::npos &&
                    records.find("[T") != std::string::npos &&
                    records.find("LAST_RECORD") != std::string::npos &&
                    records.find("FILTERED") == std::string::npos, "Core routing/filter/drain failed");
            std::size_t count = 0;
            for (std::size_t pos = 0; (pos = records.find("event=CORE_RECORD", pos)) != std::string::npos; ++pos)
                ++count;
            require(count == 40 && session->dropped_messages() == 0, "Core records were lost");
            bool closed = false;
            try { session->GetSynchronousLogger("after_finish"); }
            catch (const std::logic_error &) { closed = true; }
            require(closed, "Closed session accepted sync logger");
            session.reset();
            // Retaining the factory/logger must not retain the session or its pool.
            bool expired = false;
            try { ORB_SLAM3::GetModuleLogger(factory, "after_release"); }
            catch (const std::logic_error &) { expired = true; }
            require(expired, "Factory retained session ownership");
        }
        {
            options.level = "off";
            gemini336_orbslam3::LoggingSession session(options);
            ORB_SLAM3::Log(session.GetSynchronousLogger("fatal"), spdlog::level::critical, "OFF_RECORD");
            session.finish();
            require(read_file(session.directory() / "slam.log").empty(), "Sync logger ignored off level");
            options.level = "info";
        }
        {
            gemini336_orbslam3::LoggingSession session(options);
            const auto logger = session.GetSynchronousLogger("format_failure");
            ORB_SLAM3::Log(logger, spdlog::level::warn, "Invalid argument index {1}", 0);
            bool reported = false;
            try { session.finish(); }
            catch (const std::runtime_error &) { reported = true; }
            require(reported, "Enabled format failure was hidden from finish result");
        }
        {
            gemini336_orbslam3::LoggingSession session(options);
            const auto logger = session.GetSynchronousLogger("failure");
            // Mutate only before logging. A failed first sink must not prevent the file sink attempt.
            logger->sinks().insert(logger->sinks().begin(), std::make_shared<ThrowingSink>());
            ORB_SLAM3::Log(logger, spdlog::level::critical, "SURVIVING_FILE_RECORD");
            require(read_file(session.directory() / "slam.log").find("SURVIVING_FILE_RECORD") != std::string::npos,
                    "Sink failure prevented file write/flush");
            bool reported = false;
            try { session.finish(); }
            catch (const std::runtime_error &) { reported = true; }
            require(reported, "Core write failure was absent from finish result");
        }
        {
            gemini336_orbslam3::LoggingSession session(options);
            const auto logger = session.GetSynchronousLogger("unknown_failure");
            logger->sinks().push_back(std::make_shared<UnknownThrowingSink>());
            // spdlog reports unknown failures and rethrows; the core guard must
            // return normally while retaining the session's failure evidence.
            ORB_SLAM3::Log(logger, spdlog::level::critical, "UNKNOWN_FAILURE_RECORD");
            bool reported = false;
            try { session.finish(); }
            catch (const std::runtime_error &) { reported = true; }
            require(reported, "Unknown core logging exception lost failure evidence");
        }
        // Fork only after every asynchronous session has finished, so the child
        // does not inherit a live pool or locked sink. It creates its own session.
        const auto fatal_root = root / "fatal";
        const pid_t child = fork();
        require(child != -1, "Cannot fork fatal logging test");
        if (child == 0)
        {
            try
            {
                options.directory = fatal_root;
                gemini336_orbslam3::LoggingSession session(options);
                ORB_SLAM3::Log(session.GetSynchronousLogger("settings"), spdlog::level::critical,
                              "FATAL_BEFORE_EXIT");
                std::_Exit(17); // Deliberately skip destructors and finish().
            }
            catch (...) { std::_Exit(18); }
        }
        int status = 0;
        require(waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 17,
                "Fatal logging child failed");
        require(read_file(fatal_root / "latest" / "slam.log").find("FATAL_BEFORE_EXIT") != std::string::npos,
                "Abrupt exit lost synchronous fatal record");
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
