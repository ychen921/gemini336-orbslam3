#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <vector>

namespace gemini336_orbslam3
{
struct DiagnosticTraceStats
{
    std::size_t recorded = 0;
    std::size_t capacity = 0;
    uint64_t dropped = 0;
};

// Optional concurrent recorder. Owners outlive all non-owning frontend
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
        // Never acquire a frontend or node lock from this recorder. Timestamp inside
        // the lock keeps stored event order consistent with the recorded clock.
        const std::lock_guard<std::mutex> lock(trace_mutex_);
        if (events_.size() == capacity_)
        {
            ++dropped_;
            return;
        }
        const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        events_.push_back({event, now, sensor_ns, value1, value2});
    }

    // Return one consistent integrity snapshot while producers may still be active.
    DiagnosticTraceStats stats() const
    {
        const std::lock_guard<std::mutex> lock(trace_mutex_);
        return {events_.size(), capacity_, dropped_};
    }

    // Called only after all producers have finished (and their threads are joined).
    // No lock is held during I/O; concurrent record() or destruction is forbidden.
    // A full trace preserves the
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
    mutable std::mutex trace_mutex_;
    std::vector<Event> events_;
    uint64_t dropped_ = 0;
};
}
