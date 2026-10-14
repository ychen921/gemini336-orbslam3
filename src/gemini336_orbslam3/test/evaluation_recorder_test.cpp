#include "common/evaluation_recorder.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <new>
#include <stdexcept>
#include <unistd.h>

namespace
{
bool reject_allocation = false;

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

std::string read(const std::filesystem::path &path)
{
    std::ifstream input(path);
    require(bool(input), "missing evaluation artifact");
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

gemini336_orbslam3::EvaluationFrame frame(uint64_t index, int state, int milliseconds)
{
    // A timestamp beyond double's integer precision must round-trip unchanged.
    return {1700000000000000001LL + int64_t(index - 1) * 33333333,
            index + 10, index, int64_t(index) * 1000000000,
            int64_t(index) * 1000000000 + 1000000,
            int64_t(index) * 1000000000 + 1000000 + milliseconds * 1000000LL,
            state, 1, 1, 2, index};
}

void test_overflow_and_export(const std::filesystem::path &directory)
{
    gemini336_orbslam3::EvaluationRecorder recorder(2);
    // Throw on any allocation in the producer path, including full-buffer calls.
    reject_allocation = true;
    recorder.record(frame(1, 1, 1), 1000000000);
    recorder.record(frame(2, 2, 2), 1000000000);
    recorder.record(frame(3, 3, 40), 1000000000);
    recorder.record(frame(4, 4, 50), 1000000000);
    recorder.record(frame(5, 99, 60), 1000000000);
    reject_allocation = false;
    require(recorder.recorded() == 2 && recorder.dropped() == 3 && recorder.capacity() == 2,
            "overflow changed capacity or lost omission accounting");
    require(std::filesystem::is_empty(directory), "producer wrote evaluation artifacts");

    gemini336_orbslam3::EvaluationMetadata metadata;
    metadata.sensor_mode = "stereo_imu";
    metadata.run_id = "quoted\"run\n\\end";
    metadata.parameters.emplace_back("path", "/a\t\"b");
    recorder.write(directory, metadata);
    const std::string csv = read(directory / "frames.csv");
    require(csv.find("1700000000000000001,11,1,") != std::string::npos &&
            csv.find("1700000000033333334,12,2,") != std::string::npos &&
            csv.find("1700000000066666667") == std::string::npos,
            "CSV lost precise timestamps/identity or replaced retained prefix");
    const std::string summary = read(directory / "summary.csv");
    for (const char *expected : {"processed,5,all_processed", "startup_frames,1,before_first_ok",
         "lost_ratio_denominator,4,after_first_ok", "lost_ratio,0.25,after_first_ok",
         "recently_lost_ratio,0.25,after_first_ok", "state_unknown,1,all_processed",
         "adapter_max_ms,60,all_processed", "adapter_p95_ms,2,retained_frames"})
        require(summary.find(expected) != std::string::npos,
                "overflow corrupted full aggregate or percentile scope");
    const std::string json = read(directory / "metadata.json");
    require(json.find("quoted\\\"run\\u000a\\\\end") != std::string::npos &&
            json.find("/a\\u0009\\\"b") != std::string::npos &&
            json.find("\"bag_id\": \"unknown\"") != std::string::npos,
            "metadata escaping or missing provenance semantics changed");
}

void test_empty_and_failure(const std::filesystem::path &directory)
{
    gemini336_orbslam3::EvaluationRecorder recorder(1);
    recorder.write(directory, {});
    const std::string summary = read(directory / "summary.csv");
    require(summary.find("lost_ratio,N/A,after_first_ok") != std::string::npos &&
            summary.find("adapter_p99_ms,N/A,retained_frames") != std::string::npos &&
            summary.find("host_throughput_hz,N/A,") != std::string::npos,
            "empty input fabricated measurements");
    bool failed = false;
    try { recorder.write(directory / "missing", {}); }
    catch (const std::ios_base::failure &) { failed = true; }
    require(failed, "export failure was silently ignored");
    failed = false;
    try { gemini336_orbslam3::EvaluationRecorder invalid(0); }
    catch (const std::invalid_argument &) { failed = true; }
    require(failed, "zero capacity was accepted");
}

void test_image_intervals()
{
    gemini336_orbslam3::ImageTimestampStats stats;
    reject_allocation = true;
    for (const int64_t timestamp : {1000000000LL, 1030000000LL, 1030000000LL,
                                    1020000000LL, 1220000000LL})
        stats.observe(timestamp, 0.1);
    reject_allocation = false;
    require(stats.received == 5 && stats.positive_intervals == 2 && stats.duplicates == 1 &&
            stats.backwards == 1 && stats.excessive_gaps == 1,
            "raw intervals mixed admission or rejection semantics");
    require(stats.interval_min_sec > 0.029 && stats.interval_min_sec < 0.031 &&
            stats.interval_max_sec > 0.199 && stats.interval_max_sec < 0.201,
            "interval units or clock-reset accounting changed");
}
}

// This single-threaded test has no ROS, backend, sensor or executor lifecycle.
void *operator new(std::size_t size)
{
    if (reject_allocation) throw std::bad_alloc();
    if (void *memory = std::malloc(size ? size : 1)) return memory;
    throw std::bad_alloc();
}
void operator delete(void *memory) noexcept { std::free(memory); }
void operator delete(void *memory, std::size_t) noexcept { std::free(memory); }
void *operator new[](std::size_t size) { return ::operator new(size); }
void operator delete[](void *memory) noexcept { ::operator delete(memory); }
void operator delete[](void *memory, std::size_t) noexcept { ::operator delete(memory); }

int main()
{
    const auto directory = std::filesystem::temp_directory_path() /
        ("gemini336-evaluation-test-" + std::to_string(getpid()));
    int result = 0;
    try
    {
        std::filesystem::create_directory(directory);
        test_overflow_and_export(directory);
        test_empty_and_failure(directory);
        test_image_intervals();
        std::cout << "Evaluation allocation, overflow, integrity and export tests passed\n";
    }
    catch (const std::exception &error)
    {
        reject_allocation = false;
        std::cerr << error.what() << '\n';
        result = 1;
    }
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    return result;
}
