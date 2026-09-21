#include "slam/orbslam3_adapter.hpp"
#include "frontend/stereo_frontend.hpp"
#include "frontend/imu_frontend.hpp"

#include <cstdint>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <optional>
#include <utility>
#include <limits>
#include <exception>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <cstddef>
#include <deque>

#include <rclcpp/rclcpp.hpp>

namespace gemini336_orbslam3
{
namespace
{
const char *tracking_state_name(TrackingState state)
{
    switch (state)
    {
    case TrackingState::SystemNotReady: return "SystemNotReady";
    case TrackingState::NoImagesYet: return "NoImagesYet";
    case TrackingState::NotInitialized: return "NotInitialized";
    case TrackingState::Ok: return "Ok";
    case TrackingState::RecentlyLost: return "RecentlyLost";
    case TrackingState::Lost: return "Lost";
    case TrackingState::OkKlt: return "OkKlt";
    case TrackingState::Unknown: return "Unknown";
    }
    return "Unknown";
}
}

class SlamNode : public rclcpp::Node
{
public:
    explicit SlamNode(std::function<void()> request_stop)
        : Node("slam_node"), request_stop_(std::move(request_stop))
    {
        // These settings define component lifetimes and are fixed at startup.
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;

        input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 5.0, descriptor);
        input_timeout_action_ = declare_parameter<std::string>(
            "input_timeout_action", "shutdown", descriptor);
        if (!std::isfinite(input_timeout_sec_) || input_timeout_sec_ < 0.0)
            throw std::invalid_argument("input_timeout_sec must be finite and nonnegative");
        if (input_timeout_action_ != "shutdown" && input_timeout_action_ != "warn")
            throw std::invalid_argument("input_timeout_action must be shutdown or warn");

        // Validate SLAM resources before constructing the backend and subscribers.
        OrbSlam3Config config;
        config.vocabulary_path = declare_parameter<std::string>(
            "vocabulary_path", ORB_SLAM3_DEFAULT_VOCABULARY_PATH, descriptor);
        config.settings_path = declare_parameter<std::string>(
            "settings_path", "", descriptor);
        config.enable_viewer = declare_parameter<bool>(
            "enable_viewer", false, descriptor);

        require_absolute_path(config.vocabulary_path, "vocabulary_path");
        require_absolute_path(config.settings_path, "settings_path");

        const auto left_topic = declare_parameter<std::string>(
            "left_image_topic", "/camera/left_ir/image_raw", descriptor);
        const auto right_topic = declare_parameter<std::string>(
            "right_image_topic", "/camera/right_ir/image_raw", descriptor);
        if (left_topic.empty() || right_topic.empty())
            throw std::invalid_argument("Image topics must not be empty");

        const std::string sensor_mode =
            declare_parameter<std::string>("sensor_mode", "stereo", descriptor);
        if (sensor_mode == "stereo")
        {
            tracking_mode_ = TrackingMode::Stereo;
        }
        else if (sensor_mode == "stereo_imu")
        {
            tracking_mode_ = TrackingMode::StereoImu;
            
            imu_topic_ = declare_parameter<std::string>(
                "imu_topic", "/camera/gyro_accel/sample", descriptor);
            if (imu_topic_.empty())
                throw std::invalid_argument("imu_topic must not be empty");

            const int64_t pending_capacity = declare_parameter<int64_t>(
                "stereo_imu.pending_frame_capacity", 30, descriptor);
            if (pending_capacity <= 0)
                throw std::invalid_argument("stereo_imu.pending_frame_capacity must be positive");
            pending_frames_capacity_ =
                static_cast<std::size_t>(pending_capacity);
            
            const double imu_wait_timeout_sec = declare_parameter<double>(
                "stereo_imu.wait_timeout_sec", 1.0, descriptor);
            if (!std::isfinite(imu_wait_timeout_sec) || imu_wait_timeout_sec <= 0.0)
                throw std::invalid_argument("stereo_imu.wait_timeout_sec must be finite and positive");
            imu_wait_timeout_sec_ = imu_wait_timeout_sec;

            const int64_t imu_retry_period_ms = declare_parameter<int64_t>(
                "stereo_imu.retry_period_ms", 5, descriptor);
            if (imu_retry_period_ms <= 0)
                throw std::invalid_argument("stereo_imu.retry_period_ms must be positive");
            imu_retry_period_ms_ = imu_retry_period_ms;
        }
        else
        {
            throw std::invalid_argument("sensor_mode must be stereo or stereo_imu");
        }

        // Reject this mode until its frontend and tracking callbacks are connected.
        if (tracking_mode_ == TrackingMode::StereoImu)
            throw std::runtime_error(
                "Stereo-IMU wiring is not implemented yet");

        RCLCPP_INFO(get_logger(), "Vocabulary: %s", config.vocabulary_path.c_str());
        RCLCPP_INFO(get_logger(), "Settings: %s", config.settings_path.c_str());
        RCLCPP_INFO(get_logger(), "Viewer: %s", config.enable_viewer ? "enabled" : "disabled");

        // Construct SLAM before accepting frames through the frontend.
        slam_ = std::make_unique<OrbSlam3Adapter>(config);
        stereo_frontend_ = std::make_unique<StereoFrontend>(
            this, left_topic, right_topic,
            [this](const StereoFrame &frame) { on_frame(frame); },
            [this]() { on_input_activity(); });

        // Wall-clock timers remain independent of sensor timestamps and simulated time.
        started_ = last_report_ = Clock::now();
        report_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() { report(false); });
        if (input_timeout_sec_ > 0.0)
            input_timer_ = create_wall_timer(
                std::chrono::milliseconds(100), [this]() { check_input_timeout(); });

        RCLCPP_INFO(get_logger(), "Input timeout: seconds=%.3f action=%s (armed after first image)",
                    input_timeout_sec_, input_timeout_action_.c_str());
        RCLCPP_INFO(get_logger(), "Stereo SLAM initialized: left=%s right=%s",
                    left_topic.c_str(), right_topic.c_str());
    }

    void shutdown()
    {
        // Spin has returned, so no image callback can overlap subscription teardown.
        input_timer_.reset();
        stereo_frontend_.reset();
        report_timer_.reset();

        // Emit the final statistics while the backend is still available.
        report(true);
        RCLCPP_INFO(get_logger(), "Stereo input stopped; processed=%llu last_state=%s",
                    static_cast<unsigned long long>(processed_frames_),
                    tracking_state_name(slam_->trackingState()));

        slam_->shutdown();
        RCLCPP_INFO(get_logger(), "Stereo SLAM shutdown returned");
    }

private:
    using Clock = std::chrono::steady_clock;

    void on_input_activity()
    {
        if (input_timeout_sec_ == 0.0 || stop_requested_)
            return;

        // Either raw image stream counts as activity, even without a valid stereo pair.
        last_input_activity_ = Clock::now();
        if (input_timeout_reported_)
            RCLCPP_INFO(get_logger(), "Image input resumed after timeout");
        input_timeout_reported_ = false;
    }

    void check_input_timeout()
    {
        if (!last_input_activity_ || input_timeout_reported_ || stop_requested_)
            return;

        // The timeout is armed only after input activity has established a baseline.
        const double idle_sec =
            std::chrono::duration<double>(Clock::now() - *last_input_activity_).count();
        if (idle_sec < input_timeout_sec_)
            return;

        input_timeout_reported_ = true;
        RCLCPP_WARN(get_logger(), "Image input timeout: idle_sec=%.3f threshold_sec=%.3f action=%s",
                    idle_sec, input_timeout_sec_, input_timeout_action_.c_str());

        if (input_timeout_action_ == "shutdown")
        {
            // Cancel spin only; main owns teardown after the current callback returns.
            stop_requested_ = true;
            input_timer_->cancel();
            request_stop_();
        }
    }

    struct Statistics
    {
        uint64_t frames = 0;
        double track_sum_ms = 0.0;
        double track_max_ms = 0.0;
        uint64_t intervals = 0;
        double interval_sum_ms = 0.0;
        double interval_min_ms = std::numeric_limits<double>::infinity();
        double interval_max_ms = 0.0;
    };

    void log_statistics(const char *scope, const Statistics &stats, double elapsed)
    {
        RCLCPP_INFO(get_logger(),
                    "Stereo stats: scope=%s frames=%llu total=%llu elapsed_sec=%.6f rate_hz=%.6f "
                    "track_mean_ms=%.6f track_max_ms=%.6f intervals=%llu "
                    "interval_min_ms=%.6f interval_mean_ms=%.6f interval_max_ms=%.6f",
                    scope, static_cast<unsigned long long>(stats.frames),
                    static_cast<unsigned long long>(processed_frames_), elapsed,
                    elapsed > 0.0 ? stats.frames / elapsed : 0.0,
                    stats.frames ? stats.track_sum_ms / stats.frames : 0.0, stats.track_max_ms,
                    static_cast<unsigned long long>(stats.intervals),
                    stats.intervals ? stats.interval_min_ms : 0.0,
                    stats.intervals ? stats.interval_sum_ms / stats.intervals : 0.0,
                    stats.interval_max_ms);
    }

    void report(bool final)
    {
        const auto now = Clock::now();

        log_statistics(final ? "tail" : "window", window_,
                       std::chrono::duration<double>(now - last_report_).count());
        if (final)
            log_statistics("total", total_, std::chrono::duration<double>(now - started_).count());

        // Reset window aggregates without losing the timestamp between adjacent frames.
        window_ = Statistics{};
        last_report_ = now;
    }

    void on_frame(const StereoFrame &frame)
    {
        if (stop_requested_)
            return;

        const auto previous_state = slam_->trackingState();
        // Adapter exceptions propagate to main: retrying after an upstream failure is unsafe.
        const auto start = Clock::now();
        slam_->track(frame);
        const double track_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();

        // Preserve the previous timestamp across report windows; count only successful calls.
        for (auto *stats : {&window_, &total_})
        {
            ++stats->frames;
            stats->track_sum_ms += track_ms;
            stats->track_max_ms = std::max(stats->track_max_ms, track_ms);
            if (processed_frames_ > 0)
            {
                const double interval_ms = (frame.timestamp - previous_timestamp_) * 1000.0;
                ++stats->intervals;
                stats->interval_sum_ms += interval_ms;
                stats->interval_min_ms = std::min(stats->interval_min_ms, interval_ms);
                stats->interval_max_ms = std::max(stats->interval_max_ms, interval_ms);
            }
        }
        previous_timestamp_ = frame.timestamp;
        ++processed_frames_;

        // Report only after tracking and its statistics have completed successfully.
        if (processed_frames_ == 1)
            RCLCPP_INFO(get_logger(), "First stereo frame processed: timestamp=%.9f", frame.timestamp);
        const auto state = slam_->trackingState();
        RCLCPP_DEBUG(get_logger(), "Stereo frame: index=%llu timestamp=%.9f track_ms=%.6f state=%s",
                     static_cast<unsigned long long>(processed_frames_), frame.timestamp,
                     track_ms, tracking_state_name(state));
        if (state != previous_state)
            RCLCPP_INFO(get_logger(), "Tracking state: %s -> %s",
                        tracking_state_name(previous_state), tracking_state_name(state));
    }

    static void require_absolute_path(const std::string &path, const char *name)
    {
        if (path.empty() || !std::filesystem::path(path).is_absolute())
            throw std::invalid_argument(std::string(name) + " must be a nonempty absolute path");
    }

    void process_pending_frames()
    {
        if (stop_requested_)
            return;

        const Clock::time_point now = Clock::now();

        // Keep startup bounded even when early images are discarded.
        if (!last_tracked_frame_timestamp_ && startup_wait_started_)
        {
            const double startup_wait_sec =
                std::chrono::duration<double>(now - *startup_wait_started_).count();
            if (startup_wait_sec >= imu_wait_timeout_sec_)
                throw std::runtime_error("Stereo-IMU startup wait timed out");
        }

        if (pending_frames_.empty())
            return;

        // Each image's deadline starts at enqueue time, not at the latest retry.
        const double frame_wait_sec =
            std::chrono::duration<double>(now - pending_frames_.front().received_at).count();
        if (frame_wait_sec >= imu_wait_timeout_sec_)
            throw std::runtime_error("Stereo frame wait timed out");

        // Verify the first interval before sending either startup image.
        if (!last_tracked_frame_timestamp_)
        {
            if (pending_frames_.size() < 2)
                return;

            const double t0 = pending_frames_[0].frame.timestamp;
            const double t1 = pending_frames_[1].frame.timestamp;
            const ImuBatch batch = imu_frontend_->takeMeasurements(t0, t1);

            switch (batch.status)
            {
            case ImuBatchStatus::WaitingForData:
                return;
            case ImuBatchStatus::MissingHistory:
                // No image has reached SLAM yet, so the startup origin can advance.
                pending_frames_.pop_front();
                ++startup_discarded_frames_;
                return;
            case ImuBatchStatus::BufferOverflow:
                throw std::runtime_error("IMU buffer overflow during startup");
            case ImuBatchStatus::DataGap:
                throw std::runtime_error("IMU data gap detected during startup");
            case ImuBatchStatus::InvalidRequest:
                throw std::runtime_error("IMU startup batch request is invalid");
            case ImuBatchStatus::Ready:
            {
                // Only the first image has no preceding image interval.
                const StereoFrame &first_frame = pending_frames_.front().frame;
                slam_->track(first_frame, {});
                last_tracked_frame_timestamp_ = first_frame.timestamp;
                pending_frames_.pop_front();

                // Reacquire the front after removal; this batch belongs to (t0, t1].
                const StereoFrame &second_frame = pending_frames_.front().frame;
                slam_->track(second_frame, batch.measurements);
                last_tracked_frame_timestamp_ = second_frame.timestamp;
                pending_frames_.pop_front();

                startup_wait_started_.reset();
                return;
            }
            }
            return;
        }

        // Continue from the last normally returned tracking call, one image per retry.
        const StereoFrame &frame = pending_frames_.front().frame;
        const ImuBatch batch = imu_frontend_->takeMeasurements(
            *last_tracked_frame_timestamp_, frame.timestamp);

        switch (batch.status)
        {
        case ImuBatchStatus::WaitingForData:
            return;
        case ImuBatchStatus::MissingHistory:
            throw std::runtime_error("IMU history is missing after tracking started");
        case ImuBatchStatus::BufferOverflow:
            throw std::runtime_error("IMU buffer overflow");
        case ImuBatchStatus::DataGap:
            throw std::runtime_error("IMU data gap detected");
        case ImuBatchStatus::InvalidRequest:
            throw std::runtime_error("IMU batch request is invalid");
        case ImuBatchStatus::Ready:
            // Ready already consumed IMU; propagate tracking failures without retrying.
            slam_->track(frame, batch.measurements);
            last_tracked_frame_timestamp_ = frame.timestamp;
            pending_frames_.pop_front();
            return;
        }
    }

    struct PendingFrame
    {
        StereoFrame frame;
        Clock::time_point received_at;
    };

    void enqueue_frame(const StereoFrame &frame)
    {
        // Reject invalid sensor timestamp.
        if (!std::isfinite(frame.timestamp) || frame.timestamp < 0.0)
            throw std::invalid_argument("Stereo timestamp must be finite & nonnegative");
        // Require strictly increasing timestamps across accepted frames.
        if (last_received_frame_timestamp_ &&
            frame.timestamp <= *last_received_frame_timestamp_)
            throw std::invalid_argument("Stereo timestamps must be strictly increasing");
        // Stop before exceeding the pending-frame limit.
        if (pending_frames_.size() >= pending_frames_capacity_)
            throw std::runtime_error("Pending frame queue is full; cannot accept new frames");

        // Retain the frame and record its enqueue time.
        pending_frames_.push_back({frame, Clock::now()});
        last_received_frame_timestamp_ = frame.timestamp;

        // Keep the original startup deadline even if early frames are discarded.
        if (!last_tracked_frame_timestamp_ && !startup_wait_started_)
            startup_wait_started_ = pending_frames_.back().received_at;
    }

    // All activity and timer callbacks run on main's single-threaded executor.
    std::function<void()> request_stop_;
    double input_timeout_sec_ = 5.0;
    std::string input_timeout_action_;
    std::optional<Clock::time_point> last_input_activity_;
    bool input_timeout_reported_ = false;
    bool stop_requested_ = false;
    rclcpp::TimerBase::SharedPtr input_timer_;

    std::unique_ptr<OrbSlam3Adapter> slam_;
    std::unique_ptr<StereoFrontend> stereo_frontend_;
    std::unique_ptr<ImuFrontend> imu_frontend_;

    // Sensor timestamps measure frame spacing; steady-clock times measure throughput.
    uint64_t processed_frames_ = 0;
    double previous_timestamp_ = 0.0;
    Statistics window_;
    Statistics total_;
    Clock::time_point started_;
    Clock::time_point last_report_;
    rclcpp::TimerBase::SharedPtr report_timer_;

    // Pending images retain their pixels until tracking completes
    std::string imu_topic_;
    std::deque<PendingFrame> pending_frames_;
    TrackingMode tracking_mode_ = TrackingMode::Stereo;

    double imu_wait_timeout_sec_ = 1.0;
    uint64_t startup_discarded_frames_ = 0;
    int64_t imu_retry_period_ms_ = 5;
    std::size_t pending_frames_capacity_ = 30;
    std::optional<double> last_received_frame_timestamp_;
    std::optional<double> last_tracked_frame_timestamp_;
    std::optional<Clock::time_point> startup_wait_started_;
};
}

int main(int argc, char **argv)
{
    int result = 0;
    std::shared_ptr<gemini336_orbslam3::SlamNode> node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

    try
    {
        rclcpp::init(argc, argv);
        executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        node = std::make_shared<gemini336_orbslam3::SlamNode>([&executor]() { executor->cancel(); });
        executor->add_node(node);

        // Serial execution protects the frontend, adapter and statistics from overlap.
        executor->spin();
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("slam_node"), "%s", error.what());
        result = 1;
    }

    // Retain ownership across spin failures so both exit paths explicitly shut down SLAM.
    if (node)
    {
        try
        {
            node->shutdown();
        }
        catch (const std::exception &error)
        {
            RCLCPP_ERROR(rclcpp::get_logger("slam_node"), "Shutdown failed: %s", error.what());
            result = 1;
        }
    }

    node.reset();
    if (rclcpp::ok())
        rclcpp::shutdown();

    return result;
}
