#include "common/evaluation_inputs.hpp"

#include <fstream>
#include <iomanip>
#include <locale>
#include <string>
#include <utility>

namespace gemini336_orbslam3
{
void write_evaluation_inputs(const std::filesystem::path &directory,
                             const EvaluationInputSnapshot &snapshot)
{
    std::ofstream output;
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.imbue(std::locale::classic());
    output.open(directory / "inputs.csv");
    output << "metric,value\n" << std::setprecision(17);
    const auto metric = [&](const char *name, const auto &value) {
        output << name << ',' << value << '\n';
    };
    const auto intervals = [&](const std::string &prefix, uint64_t count, double sum,
                               double minimum, double maximum) {
        output << prefix << "_interval_count," << count << '\n';
        output << prefix << "_interval_min_ms,";
        if (count) output << minimum * 1000.0;
        else output << "N/A";
        output << '\n' << prefix << "_interval_mean_ms,";
        if (count) output << sum * 1000.0 / count;
        else output << "N/A";
        output << '\n' << prefix << "_interval_max_ms,";
        if (count) output << maximum * 1000.0;
        else output << "N/A";
        output << '\n';
    };
    if (snapshot.stereo)
    {
        const EvaluationStereoInput &stats = *snapshot.stereo;
        for (const auto &side : {std::make_pair("image_left", &stats.left),
                                 std::make_pair("image_right", &stats.right)})
        {
            output << side.first << "_received," << side.second->received << '\n'
                   << side.first << "_duplicates," << side.second->duplicates << '\n'
                   << side.first << "_backwards," << side.second->backwards << '\n'
                   << side.first << "_excessive_gaps," << side.second->excessive_gaps << '\n';
            intervals(side.first, side.second->positive_intervals, side.second->interval_sum_sec,
                      side.second->interval_min_sec, side.second->interval_max_sec);
        }
        metric("stereo_synchronized", stats.synchronized);
        metric("stereo_converted", stats.converted);
        metric("stereo_rejected_empty", stats.empty);
        metric("stereo_rejected_size_mismatch", stats.size_mismatch);
        metric("stereo_rejected_conversion", stats.conversion_failed);
    }
    if (snapshot.imu)
    {
        // IMU intervals describe accepted samples; rejected timestamps retain
        // the existing last-accepted admission semantics, unlike raw images.
        const EvaluationImuInput &stats = *snapshot.imu;
        metric("imu_received", stats.received);
        metric("imu_accepted", stats.accepted);
        metric("imu_stopped", stats.stopped);
        metric("imu_unavailable", stats.unavailable);
        metric("imu_invalid_values", stats.invalid_values);
        metric("imu_invalid_timestamps", stats.invalid_timestamps);
        metric("imu_timestamp_precision_rejections", stats.timestamp_precision_rejections);
        metric("imu_duplicates", stats.duplicates);
        metric("imu_backwards", stats.backwards);
        metric("imu_overflow", stats.overflow);
        metric("imu_buffered", stats.buffered);
        metric("imu_buffered_peak", stats.buffered_peak);
        metric("imu_accepted_excessive_gaps", stats.excessive_gaps);
        intervals("imu_accepted", stats.interval_count, stats.interval_sum_sec,
                  stats.interval_min_sec, stats.interval_max_sec);
    }
    else
        metric("imu_received", "N/A");

    const EvaluationQueueInput &queue = snapshot.queue;
    metric("enqueued", queue.enqueued);
    metric("processed", queue.processed);
    metric("startup_discarded", queue.startup_discarded);
    metric("overload_discarded", queue.overload_discarded);
    metric("outstanding", queue.outstanding);
    metric("outstanding_peak", queue.peak);
    metric("queue_capacity", queue.capacity);
    metric("accounting_valid", queue.accounting_valid ? "true" : "false");
    metric("evaluation_recorded", snapshot.recorded);
    metric("evaluation_dropped", snapshot.dropped);
    output.close();
}

void write_evaluation_export_complete(const std::filesystem::path &directory,
                                      std::size_t recorded, uint64_t dropped)
{
    std::ofstream output;
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.open(directory / "export_complete.txt");
    output << "schema_version=" << EvaluationSchemaVersion << "\nrecorded=" << recorded
           << "\ndropped=" << dropped << '\n';
    output.close();
}
}
