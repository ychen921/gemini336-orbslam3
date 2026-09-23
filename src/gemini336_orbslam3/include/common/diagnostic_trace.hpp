#pragma once

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

namespace gemini336_orbslam3
{
// Optional single-executor recorder. Owners outlive all non-owning frontend
// references. Reserve once; never allocate or write files on sensor callbacks.
class DiagnosticTrace
{
public:
    DiagnosticTrace(const std::string &path, std::size_t capacity)
        : path_(path), capacity_(capacity)
    {
        events_.reserve(capacity_);
    }

    // Event names must be string literals. Sensor ns are exact only for ROS
    // callbacks; double-valued query/frame boundaries use value1/value2.
    void record(const char *event, int64_t sensor_ns = 0,
                double value1 = 0.0, double value2 = 0.0)
    {
        if (events_.size() == capacity_)
        {
            ++dropped_;
            return;
        }
        const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        events_.push_back({event, now, sensor_ns, value1, value2});
    }

    // Called only after subscriptions/timers stop. A full trace preserves the
    // first events and explicitly counts omissions instead of silently wrapping.
    void write() const
    {
        std::ofstream output;
        output.exceptions(std::ios::failbit | std::ios::badbit);
        output.open(path_);
        output << "event,steady_ns,sensor_ns,value1,value2\n" << std::setprecision(17);
        for (const Event &event : events_)
            output << event.name << ',' << event.steady_ns << ',' << event.sensor_ns
                   << ',' << event.value1 << ',' << event.value2 << '\n';
        output << "# dropped_events=" << dropped_ << '\n';
        output.close();
    }

private:
    struct Event
    {
        const char *name;
        int64_t steady_ns;
        int64_t sensor_ns;
        double value1;
        double value2;
    };
    std::string path_;
    std::size_t capacity_;
    std::vector<Event> events_;
    uint64_t dropped_ = 0;
};
}
