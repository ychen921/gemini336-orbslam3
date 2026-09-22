#include "slam_node_testable.cpp"
#include <iostream>
#include <thread>
#include <cstdlib>

// Only the backend is replaced. Frontend validation and interval consumption are real.
namespace ORB_SLAM3 { class System {}; }
namespace fixture
{
struct Call { double timestamp; std::vector<gemini336_orbslam3::ImuMeasurement> imu; };
std::vector<Call> calls;
int attempts = 0;
int fail_on = 0;
int shutdowns = 0;
int assertions = 0;
void check(bool condition, const char *message)
{
    ++assertions;
    if (!condition) throw std::runtime_error(message);
}
template<class F> void fails(F action, const std::string &expected)
{
    try { action(); }
    catch (const std::exception &e)
    {
        check(std::string(e.what()).find(expected) != std::string::npos, e.what());
        return;
    }
    throw std::runtime_error("Expected failure: " + expected);
}
}
namespace gemini336_orbslam3
{
OrbSlam3Adapter::OrbSlam3Adapter(const OrbSlam3Config &config) : tracking_mode_(config.tracking_mode)
{
    if (const char *value = std::getenv("TEST_FAIL_ON")) fixture::fail_on = std::stoi(value);
}
OrbSlam3Adapter::~OrbSlam3Adapter() noexcept = default;
void OrbSlam3Adapter::track(const StereoFrame &frame)
{
    fixture::check(tracking_mode_ == TrackingMode::Stereo, "Stereo overload mismatch");
    ++fixture::attempts;
    fixture::calls.push_back({frame.timestamp, {}});
}
void OrbSlam3Adapter::track(const StereoFrame &frame, const std::vector<ImuMeasurement> &imu)
{
    fixture::check(tracking_mode_ == TrackingMode::StereoImu, "IMU overload mismatch");
    if (++fixture::attempts == fixture::fail_on) throw std::runtime_error("injected backend failure");
    if (!fixture::calls.empty()) fixture::check(!imu.empty(), "Repeated empty IMU batch");
    fixture::calls.push_back({frame.timestamp, imu});
}
TrackingState OrbSlam3Adapter::trackingState() const noexcept { return TrackingState::NotInitialized; }
void OrbSlam3Adapter::shutdown() { ++fixture::shutdowns; }
}
using namespace gemini336_orbslam3;
using namespace fixture;

struct Session
{
    std::shared_ptr<SlamNode> node;
    explicit Session(std::vector<std::string> extra = {}, bool stereo = false)
    {
        calls.clear(); attempts = fail_on = shutdowns = 0;
        std::vector<std::string> args = {"test", "--ros-args", "-p", "settings_path:=/stub/settings",
            "-p", stereo ? "sensor_mode:=stereo" : "sensor_mode:=stereo_imu",
            "-p", "imu.max_gap_sec:=0.02", "-p", "input_timeout_sec:=0.0"};
        args.insert(args.end(), extra.begin(), extra.end());
        std::vector<const char *> argv;
        for (const auto &arg : args) argv.push_back(arg.c_str());
        rclcpp::init(static_cast<int>(argv.size()), argv.data());
        try { node = std::make_shared<SlamNode>([] {}); }
        catch (...) { rclcpp::shutdown(); throw; }
    }
    ~Session()
    {
        if (node && !node->stop_requested_) node->shutdown();
        node.reset();
        rclcpp::shutdown();
    }
    void image(double time)
    {
        StereoFrame frame{cv::Mat(2, 2, CV_8UC1), cv::Mat(2, 2, CV_8UC1), time};
        node->on_frame(frame);
    }
    void imu(int milliseconds)
    {
        auto msg = std::make_shared<sensor_msgs::msg::Imu>();
        msg->header.stamp.sec = milliseconds / 1000;
        msg->header.stamp.nanosec = (milliseconds % 1000) * 1000000;
        msg->header.frame_id = "imu";
        msg->linear_acceleration.z = 9.81;
        node->imu_frontend_->imu_callback(msg);
    }
    void range(int first, int last) { for (int i = first; i <= last; i += 5) imu(i); }
    void tick() { node->process_pending_frames(); }
    void balanced()
    {
        check(node->enqueued_frames_ == node->processed_frames_ + node->startup_discarded_frames_ +
              node->pending_frames_.size(), "Frame accounting mismatch");
    }
};
int main(int argc, char **argv)
{
    // Exercise production main and its exit code independently from direct-state tests.
    if (argc > 1) return production_main(argc, argv);
    try
    {
        {
            Session s({}, true); s.image(1.0);
            check(!s.node->imu_frontend_ && !s.node->imu_retry_timer_, "Stereo created IMU resources");
            check(calls.size() == 1 && s.node->processed_frames_ == 1, "Stereo tracking regression");
        }
        {
            Session s; s.image(1.0); s.tick(); s.image(1.03); s.range(995, 1025); s.tick();
            check(calls.empty() && s.node->pending_frames_.size() == 2, "Waiting consumed images");
            s.imu(1030); s.tick();
            check(calls.size() == 2 && calls[0].imu.empty() && calls[1].imu.size() == 6, "Startup batches");
            check(calls[1].imu.front().timestamp > 1.0 && calls[1].imu.back().timestamp == 1.03, "Startup bounds");
            s.image(1.06); s.range(1035, 1060); s.tick(); s.balanced();
            check(calls.size() == 3 && calls[2].imu.front().timestamp > 1.03, "Repeated IMU");
            check(s.node->processed_frames_ == 3 && s.node->pending_frames_peak_ == 2, "Statistics");
            check(s.node->enqueue_to_return_sum_ms_ >= s.node->enqueue_to_return_max_ms_, "Latency statistics");
        }
        {
            Session s; s.image(1.0); s.image(1.03); s.range(1010, 1060);
            const auto origin = s.node->startup_wait_started_; s.tick();
            check(s.node->startup_discarded_frames_ == 1 && s.node->startup_wait_started_ == origin, "Startup deadline reset");
            s.image(1.06); s.tick(); s.balanced(); check(calls.size() == 2, "Startup recovery");
        }
        {
            Session s; s.image(1.0); s.image(1.03); s.imu(1000); s.imu(1030);
            fails([&] { s.tick(); }, "interval=(1,"); s.balanced();
        }
        {
            Session s({"-p", "imu.buffer_capacity:=3"}); s.range(1000, 1050);
            s.image(1.0); s.image(1.03); fails([&] { s.tick(); }, "overflow");
        }
        {
            Session s; s.range(1000, 1030); s.imu(1020);
            fails([&] { s.tick(); }, "backwards");
        }
        {
            Session s({"-p", "stereo_imu.pending_frame_capacity:=1"}); s.image(1.0);
            fails([&] { s.image(1.03); }, "capacity=1"); s.balanced();
        }
        {
            Session s; s.image(1.0); *s.node->startup_wait_started_ -= std::chrono::seconds(2);
            fails([&] { s.tick(); }, "threshold_sec=1");
        }
        {
            Session s; s.image(1.0); s.image(1.03); s.range(995, 1030); s.tick();
            s.image(1.06); s.node->pending_frames_.front().received_at -= std::chrono::seconds(2);
            fails([&] { s.tick(); }, "Stereo frame wait timed out");
        }
        {
            Session s; s.image(1.0); s.image(1.03); s.range(995, 1030); fail_on = 2;
            fails([&] { s.tick(); }, "injected backend failure");
            check(attempts == 2 && calls.size() == 1 && s.node->processed_frames_ == 1, "Failure retried or miscounted");
            s.balanced(); s.node->shutdown();
            check(shutdowns == 1 && s.node->pending_frames_.empty(), "Shutdown cleanup");
        }
        {
            Session s; s.image(1.0); s.node->shutdown();
            check(shutdowns == 1 && !s.node->imu_frontend_ && !s.node->stereo_frontend_ &&
                  !s.node->imu_retry_timer_ && s.node->pending_frames_.empty(), "Waiting shutdown");
        }
        {
            Session s; s.image(1.0); s.image(1.03); s.range(995, 1030); s.tick();
            s.image(1.06); s.node->last_tracked_frame_timestamp_ = 1.02;
            fails([&] { s.tick(); }, "request is invalid: interval=");
        }
        {
            Session s; s.range(1010, 1060); s.image(1.03);
            s.node->last_tracked_frame_timestamp_ = 1.0;
            fails([&] { s.tick(); }, "history is missing after tracking started");
        }
        {
            Session s({"-p", "imu.buffer_capacity:=3"}); s.range(900, 1010);
            s.image(1.0); s.image(1.01); s.tick();
            check(calls.size() == 2, "Harmless old IMU eviction stopped tracking");
        }
        {
            Session s;
            fails([&] { s.image(-1.0); }, "finite");
            fails([&] { s.image(std::numeric_limits<double>::quiet_NaN()); }, "finite");
            s.image(1.0);
            fails([&] { s.image(1.0); }, "increasing");
            fails([&] { s.image(0.9); }, "increasing");
            s.balanced();
        }
        for (const auto &parameter : std::vector<std::string>{"sensor_mode:=bad", "imu_topic:=''",
            "stereo_imu.pending_frame_capacity:=0", "stereo_imu.pending_frame_capacity:=-1",
            "stereo_imu.wait_timeout_sec:=0.0", "stereo_imu.wait_timeout_sec:=-1.0",
            "stereo_imu.retry_period_ms:=0", "stereo_imu.retry_period_ms:=-1"})
        {
            fails([&] { Session s({"-p", parameter}); }, "must");
        }
        std::cout << "PASS assertions=" << assertions << std::endl;
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "FAIL: " << e.what() << std::endl;
        return 1;
    }
}
