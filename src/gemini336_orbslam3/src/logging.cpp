#include "gemini336_orbslam3/logging.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>
#include <unistd.h>

#include <spdlog/async_logger.h>
#include <spdlog/details/periodic_worker.h>
#include <spdlog/details/thread_pool.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace gemini336_orbslam3
{
struct LoggingSession::Impl
{
    // Keep sink alive until queued records and the periodic flush have finished.
    std::filesystem::path directory;
    std::shared_ptr<spdlog::sinks::basic_file_sink_mt> sink;
    std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> console_sink;
    std::shared_ptr<spdlog::details::thread_pool> pool;
    std::unique_ptr<spdlog::details::periodic_worker> flusher;
    spdlog::level::level_enum level;
    std::mutex mutex;
    std::map<std::string, std::shared_ptr<spdlog::logger>> loggers;

    std::shared_ptr<std::atomic<bool>> write_failed = std::make_shared<std::atomic<bool>>(false);
    bool finished = false;
    std::size_t final_dropped = 0;
    std::exception_ptr finish_failure;

    void finish()
    {
        if (finished)
        {
            if (finish_failure) std::rethrow_exception(finish_failure);
            return;
        }
        finished = true;
        try
        {
            flusher.reset();
            final_dropped = pool ? pool->overrun_counter() : 0;
            pool.reset(); // Joins after draining; asynchronous flush is not a barrier.
            if (final_dropped != 0)
                std::fprintf(stderr, "Logging session %s: dropped_messages=%zu\n",
                             directory.c_str(), final_dropped);
            if (sink) sink->flush();
            if (console_sink) console_sink->flush();
            if (write_failed->load()) throw std::runtime_error("Logging backend reported a write failure");
        }
        catch (...)
        {
            finish_failure = std::current_exception();
            throw;
        }
    }

    ~Impl()
    {
        try { finish(); }
        catch (...) { std::fprintf(stderr, "Logging finalization failed\n"); }
    }
};

LoggingSession::LoggingSession(const LoggingOptions &options)
    : impl_(std::make_unique<Impl>())
{
    // Reject typos instead of silently disabling or changing logging verbosity.
    if (options.level != "trace" && options.level != "debug" &&
        options.level != "info" && options.level != "warn" &&
        options.level != "error" && options.level != "critical" && options.level != "off")
        throw std::invalid_argument("Invalid logging.level: " + options.level);
    if (options.queue_capacity == 0)
        throw std::invalid_argument("Logging queue capacity must be positive");
    impl_->level = spdlog::level::from_str(options.level);

    std::filesystem::path root = options.directory;
    if (root.empty())
    {
        const char *environment = std::getenv("GEMINI336_SLAM_LOG_DIR");
        root = environment && *environment ? environment : GEMINI336_DEFAULT_LOG_DIRECTORY;
    }
    if (!root.is_absolute())
        throw std::invalid_argument("Logging directory must be an absolute path");
    std::filesystem::create_directories(root);

    // Atomic directory creation separates even same-process sessions in one second.
    const std::time_t now = std::time(nullptr);
    std::tm local_time{};
    if (!localtime_r(&now, &local_time))
        throw std::runtime_error("Cannot resolve logging session time");
    std::ostringstream name;
    name << std::put_time(&local_time, "%Y%m%d_%H%M%S") << '_' << getpid();
    for (std::size_t suffix = 0;; ++suffix)
    {
        impl_->directory = root / (name.str() +
            (suffix == 0 ? "" : "_" + std::to_string(suffix)));
        if (std::filesystem::create_directory(impl_->directory)) break;
    }

    impl_->sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
        (impl_->directory / "slam.log").string(), false);
    impl_->sink->set_pattern("[%Y-%m-%dT%H:%M:%S.%e%z] [T%t] [%n] [%l] %v");
    // Share one console sink across modules; automatic color stays absent in redirected output.
    impl_->console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>(spdlog::color_mode::automatic);
    impl_->console_sink->set_pattern("[%Y-%m-%dT%H:%M:%S.%e%z] [T%t] [%n] [%^%l%$] %v");
    impl_->pool = std::make_shared<spdlog::details::thread_pool>(options.queue_capacity, 1);
    impl_->flusher = std::make_unique<spdlog::details::periodic_worker>([this]() {
        try
        {
            impl_->sink->flush();
            impl_->console_sink->flush();
        }
        catch (...)
        {
            impl_->write_failed->store(true);
            std::fprintf(stderr, "Logging periodic flush failed\n");
        }
    }, std::chrono::seconds(1));

    // A unique staging link permits atomic replacement without an unlink gap.
    const std::filesystem::path staging = root / (".latest_" + impl_->directory.filename().string());
    std::filesystem::create_directory_symlink(impl_->directory.filename(), staging);
    try { std::filesystem::rename(staging, root / "latest"); }
    catch (...)
    {
        std::error_code ignored;
        std::filesystem::remove(staging, ignored);
        throw;
    }
}

LoggingSession::~LoggingSession() = default;

std::shared_ptr<spdlog::logger> LoggingSession::GetLogger(const std::string &module_name)
{
    // Keep module tokens safe for line-oriented parsing.
    if (module_name.empty() || module_name.find_first_not_of(
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-.") != std::string::npos)
        throw std::invalid_argument("Invalid logging module name: " + module_name);
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->finished) throw std::logic_error("Logging session is finished");
    const auto existing = impl_->loggers.find(module_name);
    if (existing != impl_->loggers.end()) return existing->second;

    // Never block sensor producers on disk throughput; expose queue losses.
    const std::vector<spdlog::sink_ptr> sinks{impl_->sink, impl_->console_sink};
    std::shared_ptr<spdlog::logger> logger = std::make_shared<spdlog::async_logger>(
        module_name, sinks.begin(), sinks.end(), impl_->pool, spdlog::async_overflow_policy::overrun_oldest);
    // Shared error state outlives asynchronous logger callbacks without capturing Impl.
    logger->set_error_handler([failed = impl_->write_failed](const std::string &message) {
        failed->store(true);
        std::fprintf(stderr, "Logging write failed: %s\n", message.c_str());
    });
    logger->set_level(impl_->level);
    impl_->loggers.emplace(module_name, logger);
    return logger;
}

const std::filesystem::path &LoggingSession::directory() const
{
    return impl_->directory;
}

std::size_t LoggingSession::dropped_messages() const
{
    return impl_->pool ? impl_->pool->overrun_counter() : impl_->final_dropped;
}
void LoggingSession::finish()
{
    impl_->finish();
}
}  // namespace gemini336_orbslam3
