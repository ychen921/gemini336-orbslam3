#include "gemini336_orbslam3/logging.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <spdlog/sinks/base_sink.h>
#include <mutex>
#include <cstdio>
#include <unistd.h>

namespace
{
// Redirect before sink construction and restore only after the async writer has stopped.
class StdoutCapture
{
public:
    explicit StdoutCapture(const std::filesystem::path &path)
    {
        std::fflush(stdout);
        saved_ = dup(STDOUT_FILENO);
        FILE *file = std::fopen(path.c_str(), "w");
        if (saved_ == -1 || !file || dup2(fileno(file), STDOUT_FILENO) == -1)
        {
            if (file) std::fclose(file);
            if (saved_ != -1) close(saved_);
            throw std::runtime_error("Cannot capture stdout");
        }
        std::fclose(file);
    }
    ~StdoutCapture()
    {
        std::fflush(stdout);
        if (dup2(saved_, STDOUT_FILENO) == -1) std::abort();
        close(saved_);
    }
    StdoutCapture(const StdoutCapture &) = delete;
    StdoutCapture &operator=(const StdoutCapture &) = delete;

private:
    int saved_ = -1;
};

class ThrowingSink : public spdlog::sinks::base_sink<std::mutex>
{
    void sink_it_(const spdlog::details::log_msg &) override { throw std::runtime_error("injected write failure"); }
    void flush_() override {}
};
void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}
}

int main()
{
    // Keep validation independent of ROS, sensors, and the workspace log directory.
    char temporary[] = "/tmp/gemini336-logging-XXXXXX";
    const char *created = mkdtemp(temporary);
    if (!created) return 1;
    const std::filesystem::path root(created);
    try
    {
        gemini336_orbslam3::LoggingOptions options;
        options.directory = root;
        std::filesystem::path first;
        {
            StdoutCapture capture(root / "console.log");
            gemini336_orbslam3::LoggingSession session(options);
            first = session.directory();
            const auto logger = session.GetLogger("test");
            require(logger == session.GetLogger("test"), "Module logger must be reused");
            logger->debug("FILTERED_RECORD");
            std::vector<std::thread> producers;
            for (int producer = 0; producer < 4; ++producer)
            {
                producers.emplace_back([&session, producer]() {
                    const auto module = session.GetLogger("producer_" + std::to_string(producer));
                    for (int index = 0; index < 100; ++index)
                        module->info("event=TEST producer={} index={}", producer, index);
                });
            }
            for (std::thread &producer : producers) producer.join();
            require(session.dropped_messages() == 0, "Unexpected queue loss");
        }
        // Destruction must drain every accepted record before returning.
        std::ifstream file(first / "slam.log");
        std::string line;
        int count = 0;
        while (std::getline(file, line))
        {
            require(line.find("FILTERED_RECORD") == std::string::npos, "Level filter failed");
            require(line.find("[T") != std::string::npos &&
                    line.find("[producer_") != std::string::npos &&
                    line.find("[info] event=TEST") != std::string::npos, "Record format failed");
            ++count;
        }
        require(count == 400, "Shutdown lost records");
        // With color disabled automatically for a file, both sinks must produce identical records.
        std::ifstream console(root / "console.log");
        std::ifstream mirrored_file(first / "slam.log");
        count = 0;
        while (std::getline(console, line))
        {
            std::string file_line;
            require(std::getline(mirrored_file, file_line).good(), "Console has extra records");
            require(line == file_line, "Console and file records differ");
            require(line.find('\033') == std::string::npos, "Redirected console contains ANSI colors");
            ++count;
        }
        require(count == 400, "Console shutdown lost records");
        require(!std::getline(mirrored_file, line), "Console missed file records");
        {
            gemini336_orbslam3::LoggingSession second(options);
            require(second.directory() != first, "Session collision");
            require(std::filesystem::canonical(root / "latest") == second.directory(),
                    "Latest did not follow the new session");
            require(std::filesystem::file_size(second.directory() / "slam.log") == 0,
                    "Infrastructure must not emit application events");
        }
        {
            StdoutCapture capture(root / "finish_console.log");
            gemini336_orbslam3::LoggingSession session(options);
            session.GetLogger("explicit_finish")->info("last record");
            session.finish();
            session.finish();
            require(std::filesystem::file_size(session.directory() / "slam.log") > 0,
                    "explicit finish did not drain records");
            bool closed = false;
            try { session.GetLogger("after_finish"); }
            catch (const std::logic_error &) { closed = true; }
            require(closed, "finished logging accepted new producers");
        }
        {
            std::ifstream console_file(root / "finish_console.log");
            require(static_cast<bool>(std::getline(console_file, line)) &&
                    line.find("last record") != std::string::npos,
                    "explicit finish did not drain console records");
        }
        {
            gemini336_orbslam3::LoggingSession session(options);
            const auto logger = session.GetLogger("write_failure");
            // Configure sinks before starting a producer; no concurrent sink mutation.
            logger->sinks().push_back(std::make_shared<ThrowingSink>());
            logger->info("queued failure");
            for (int attempt = 0; attempt < 2; ++attempt)
            {
                bool failed = false;
                try { session.finish(); }
                catch (const std::runtime_error &) { failed = true; }
                require(failed, "async write failure or previous finish failure was lost");
            }
        }
        for (const bool invalid_level : {false, true})
        {
            options.directory = invalid_level ? root : std::filesystem::path("relative");
            options.level = invalid_level ? "typo" : "info";
            bool rejected = false;
            try { gemini336_orbslam3::LoggingSession invalid(options); }
            catch (const std::invalid_argument &) { rejected = true; }
            require(rejected, "Invalid configuration accepted");
        }
        std::filesystem::remove_all(root);
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        std::filesystem::remove_all(root);
        return 1;
    }
    return 0;
}
