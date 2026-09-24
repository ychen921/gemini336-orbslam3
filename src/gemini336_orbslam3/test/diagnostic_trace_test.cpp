#include "common/diagnostic_trace.hpp"

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace
{
void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

void check_trace(const std::filesystem::path &path, std::size_t capacity)
{
    constexpr int producers = 4;
    constexpr int per_producer = 2000;
    constexpr int total = producers * per_producer;
    gemini336_orbslam3::DiagnosticTrace trace(path.string(), capacity);
    std::atomic<bool> start{false};
    std::vector<std::thread> workers;
    for (int producer = 0; producer < producers; ++producer)
    {
        workers.emplace_back([&, producer]() {
            while (!start.load()) std::this_thread::yield();
            for (int i = 0; i < per_producer; ++i)
            {
                const int id = producer * per_producer + i;
                trace.record("sample", id, id, -id);
            }
        });
    }
    start.store(true);
    bool snapshots_valid = true;
    uint64_t previous_attempts = 0;
    // Finite observation loop checks consistency while producers contend on record().
    for (int i = 0; i < 2000; ++i)
    {
        const auto stats = trace.stats();
        const uint64_t attempts = stats.recorded + stats.dropped;
        if (stats.capacity != capacity || stats.recorded > capacity ||
            attempts < previous_attempts || attempts > total ||
            (stats.dropped && stats.recorded != capacity))
            snapshots_valid = false;
        previous_attempts = attempts;
    }
    for (auto &worker : workers) worker.join();
    require(snapshots_valid, "inconsistent live trace snapshot");
    const auto stats = trace.stats();
    const std::size_t expected = std::min(capacity, std::size_t(total));
    require(stats.recorded == expected && stats.dropped == total - expected,
            "final capacity/dropped accounting mismatch");

    // Export only after producer join; verify the existing CSV contract and payloads.
    trace.write();
    std::ifstream input(path);
    std::string line;
    std::getline(input, line);
    require(line == "event,steady_ns,sensor_ns,value1,value2", "CSV header changed");
    std::set<int64_t> ids;
    int64_t previous_time = 0;
    bool footer = false;
    while (std::getline(input, line))
    {
        if (line.rfind("# dropped_events=", 0) == 0)
        {
            require(line == "# dropped_events=" + std::to_string(stats.dropped), "wrong footer");
            footer = true;
            require(!std::getline(input, line), "data after footer");
            break;
        }
        std::istringstream row(line);
        std::string name, time, sensor, first, second;
        require(bool(std::getline(row, name, ',') && std::getline(row, time, ',') &&
                     std::getline(row, sensor, ',') && std::getline(row, first, ',') &&
                     std::getline(row, second)), "malformed event");
        const int64_t timestamp = std::stoll(time);
        const int64_t id = std::stoll(sensor);
        require(name == "sample" && timestamp >= previous_time && id >= 0 && id < total &&
                std::stod(first) == id && std::stod(second) == -id && ids.insert(id).second,
                "event order/payload/uniqueness mismatch");
        previous_time = timestamp;
    }
    require(footer && ids.size() == expected, "missing rows or footer");
}
}

int main()
{
    const auto path = std::filesystem::temp_directory_path() /
        ("gemini336-trace-test-" + std::to_string(getpid()) + ".csv");
    int result = 0;
    try
    {
        check_trace(path, 10000);  // Lossless: every producer event retained.
        check_trace(path, 127);    // Overflow: preserve prefix and count every omission.
        check_trace(path, 0);      // Empty capacity must never append.
        std::cout << "Trace concurrency, accounting and CSV tests passed\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    std::error_code error;
    std::filesystem::remove(path, error);
    return result;
}
