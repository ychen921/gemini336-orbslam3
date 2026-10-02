#include "frontend/imu_frontend.hpp"

#include <atomic>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace gemini336_orbslam3
{
// Direct finite input avoids DDS scheduling and exercises the actual reception path.
struct ImuFrontendTestAccess
{
    static rclcpp::SubscriptionBase::SharedPtr subscription(ImuFrontend &frontend)
    {
        return frontend.imu_sub_;
    }

    static void receive(ImuFrontend &frontend, int seconds, bool unavailable = false)
    {
        auto msg = std::make_shared<sensor_msgs::msg::Imu>();
        msg->header.stamp.sec = seconds;
        msg->header.frame_id = "imu";
        msg->linear_acceleration.z = 9.8;
        if (unavailable) msg->linear_acceleration_covariance[0] = -1.0;
        frontend.imu_callback(msg);
    }
};
}

namespace
{
using namespace gemini336_orbslam3;

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

void concurrent_queries()
{
    rclcpp::Node node("imu_concurrency_test");
    node.declare_parameter("imu.max_gap_sec", 1.1);
    node.declare_parameter("imu.buffer_capacity", 4096);
    // No export in this test; the recorder outlives the frontend and all workers.
    DiagnosticTrace trace("", 10000);
    const auto reception_group = node.create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    ImuFrontend frontend(&node, reception_group, "/unused_imu_test", &trace);
    require(reception_group->find_subscription_ptrs_if(
                [&](const rclcpp::SubscriptionBase::SharedPtr &subscription) {
                    return subscription == ImuFrontendTestAccess::subscription(frontend);
                }) != nullptr,
            "IMU subscription is not in reception group");
    bool rejected = false;
    try { ImuFrontend invalid(&node, nullptr, "/unused_null_group"); }
    catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "IMU accepted a null reception group");
    constexpr int last = 2001;
    std::atomic<bool> start{false}, done{false}, valid{true};
    std::thread producer([&]() {
        while (!start.load()) std::this_thread::yield();
        for (int i = 1; i <= last; ++i)
            ImuFrontendTestAccess::receive(frontend, i);
        done.store(true);
    });
    std::thread observer([&]() {
        start.store(true);
        // Snapshots must never expose the middle of a sample acceptance.
        for (int i = 0; i < 4000; ++i)
        {
            const ImuFrontendStats stats = frontend.stats();
            if (stats.received != stats.accepted || stats.buffered > stats.accepted ||
                (stats.accepted && (!stats.first_timestamp || !stats.last_timestamp ||
                 stats.interval_count + 1 != stats.accepted)))
                valid.store(false);
            frontend.inspectMeasurements(1.0, 2.0);
            trace.record("test_observer", i);
        }
    });
    int consumed = 0;
    // A finite retry budget plus the CTest timeout bounds failures without ROS spinning.
    for (int attempts = 0; attempts < 1000000 && consumed < last - 1; ++attempts)
    {
        // Match the frontend conversion; integer seconds multiplied via ns can round.
        const double left = static_cast<double>(int64_t(consumed + 1) * 1000000000LL) * 1e-9;
        const double right = static_cast<double>(int64_t(consumed + 2) * 1000000000LL) * 1e-9;
        const ImuBatch batch = frontend.takeMeasurements(left, right);
        if (batch.status == ImuBatchStatus::WaitingForData)
        {
            std::this_thread::yield();
            continue;
        }
        if (batch.status != ImuBatchStatus::Ready || batch.measurements.size() != 1 ||
            batch.measurements.front().timestamp != right)
        {
            valid.store(false);
            break;
        }
        ++consumed;
    }
    producer.join();
    observer.join();
    const DiagnosticTraceStats trace_stats = trace.stats();
    require(trace_stats.recorded == 2 * last + 4000 && trace_stats.dropped == 0,
            "shared IMU/observer trace accounting mismatch");
    require(done.load() && valid.load() && consumed == last - 1, "concurrent batch/snapshot mismatch");
    const ImuFrontendStats stats = frontend.stats();
    require(stats.accepted == last && stats.buffered == 1 && stats.overflow == 0,
            "final coverage/accounting mismatch");
    require(frontend.takeMeasurements(last - 1, last).status == ImuBatchStatus::InvalidRequest,
            "consumed interval was accepted twice");
}

void rejection_and_overflow()
{
    rclcpp::Node node("imu_policy_test");
    node.declare_parameter("imu.max_gap_sec", 1.1);
    node.declare_parameter("imu.buffer_capacity", 2);
    const auto reception_group = node.create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto path = std::filesystem::temp_directory_path() /
        ("imu-rejections-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".csv");
    DiagnosticTrace trace(path.string(), 32);
    ImuFrontend frontend(&node, reception_group, "/unused_imu_policy_test", &trace);
    ImuFrontendTestAccess::receive(frontend, 1);
    ImuFrontendTestAccess::receive(frontend, 1);
    ImuFrontendTestAccess::receive(frontend, 0);
    ImuFrontendTestAccess::receive(frontend, 2, true);
    ImuFrontendTestAccess::receive(frontend, 2);
    require(frontend.inspectMeasurements(1, 2) == ImuBatchStatus::Ready, "inspect not ready");
    require(frontend.stats().buffered == 2, "inspect consumed data");
    ImuFrontendTestAccess::receive(frontend, 3);
    const ImuFrontendStats stats = frontend.stats();
    require(stats.received == 6 && stats.accepted == 3 && stats.duplicates == 1 &&
            stats.backwards == 1 && stats.unavailable == 1 && stats.overflow == 1 &&
            stats.buffered == 2, "rejection/overflow counters changed");
    require(frontend.takeMeasurements(1, 3).status == ImuBatchStatus::BufferOverflow,
            "overflow did not preserve missing anchor detection");
    require(frontend.takeMeasurements(2, 3).status == ImuBatchStatus::Ready,
            "failed query changed consumption boundary");
    trace.write(); // No producers remain; validate identity rather than global adjacency.
    std::ifstream input(path);
    std::string line;
    unsigned verified = 0;
    while (std::getline(input, line))
    {
        std::istringstream fields(line);
        std::string event, steady, sensor;
        std::getline(fields, event, ',');
        std::getline(fields, steady, ',');
        std::getline(fields, sensor, ',');
        if (event == "imu_reject_duplicates" || event == "imu_reject_backwards" || event == "imu_reject_unavailable")
        {
            const int64_t expected = event == "imu_reject_duplicates" ? 1000000000LL :
                event == "imu_reject_unavailable" ? 2000000000LL : 0;
            require(std::stoll(sensor) == expected, "reject trace lost exact sensor timestamp");
            ++verified;
        }
    }
    input.close();
    std::filesystem::remove(path);
    require(verified == 3, "reject trace events missing");
}
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try
    {
        concurrent_queries();
        rejection_and_overflow();
        std::cout << "IMU concurrency and policy tests passed\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
