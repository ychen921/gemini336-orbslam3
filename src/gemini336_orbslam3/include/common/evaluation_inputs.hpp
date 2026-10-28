#pragma once

#include "common/evaluation_recorder.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>

namespace gemini336_orbslam3
{
// Export-only values keep file formatting independent of ROS and live producers.
struct EvaluationStereoInput
{
    ImageTimestampStats left;
    ImageTimestampStats right;
    uint64_t synchronized = 0;
    uint64_t converted = 0;
    uint64_t empty = 0;
    uint64_t size_mismatch = 0;
    uint64_t conversion_failed = 0;
};

struct EvaluationImuInput
{
    uint64_t received = 0;
    uint64_t accepted = 0;
    uint64_t stopped = 0;
    uint64_t unavailable = 0;
    uint64_t invalid_values = 0;
    uint64_t invalid_timestamps = 0;
    uint64_t timestamp_precision_rejections = 0;
    uint64_t duplicates = 0;
    uint64_t backwards = 0;
    uint64_t overflow = 0;
    std::size_t buffered = 0;
    std::size_t buffered_peak = 0;
    uint64_t excessive_gaps = 0;
    uint64_t interval_count = 0;
    double interval_sum_sec = 0.0;
    double interval_min_sec = 0.0;
    double interval_max_sec = 0.0;
};

struct EvaluationQueueInput
{
    uint64_t enqueued = 0;
    uint64_t processed = 0;
    uint64_t startup_discarded = 0;
    uint64_t overload_discarded = 0;
    std::size_t outstanding = 0;
    std::size_t peak = 0;
    std::size_t capacity = 0;
    bool accounting_valid = false;
};

// Capture only after executor callbacks finish. Absence distinguishes an unused
// frontend from one that received no samples; no component ownership is retained.
struct EvaluationInputSnapshot
{
    std::optional<EvaluationStereoInput> stereo;
    std::optional<EvaluationImuInput> imu;
    EvaluationQueueInput queue;
    std::size_t recorded = 0;
    uint64_t dropped = 0;
};

void write_evaluation_inputs(const std::filesystem::path &directory,
                             const EvaluationInputSnapshot &snapshot);

// Call only after recorder and input exports both succeed. This marker does not
// certify backend shutdown or the later backend/trajectory exports.
void write_evaluation_export_complete(const std::filesystem::path &directory,
                                      std::size_t recorded, uint64_t dropped);
}
