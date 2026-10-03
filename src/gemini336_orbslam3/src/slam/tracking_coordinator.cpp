#include "slam/tracking_coordinator.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

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

TrackingCoordinator::TrackingCoordinator(
    Config config, std::shared_ptr<StopControl> stop_control,
    std::shared_ptr<spdlog::logger> logger, DiagnosticTrace *trace,
    OrbSlam3Adapter *adapter, ImuFrontend *imu,
    std::function<void(double)> first_frame_notification,
    std::function<void(TrackingState, TrackingState)> state_notification)
    : stop_control_(std::move(stop_control)), node_logger_(std::move(logger)),
      trace_(trace), slam_(adapter), imu_frontend_(imu),
      first_frame_notification_(std::move(first_frame_notification)),
      state_notification_(std::move(state_notification)),
      tracking_mode_(config.mode), imu_wait_timeout_sec_(config.imu_wait_timeout_sec),
      pending_frames_capacity_(config.capacity), started_(Clock::now()), last_report_(started_)
{
    if (!stop_control_) throw std::invalid_argument("TrackingCoordinator: stop control is required");
    if (pending_frames_capacity_ == 0) throw std::invalid_argument("TrackingCoordinator: capacity must be positive");
}

void TrackingCoordinator::enqueue(const StereoFrame &frame)
{
    if (stop_control_->stop_requested()) return;
    enqueue_frame(frame, tracking_mode_ == TrackingMode::Stereo ?
        QueueFullPolicy::DiscardOldestQueued : QueueFullPolicy::Reject);
}

TrackingCoordinator::QueueSnapshot TrackingCoordinator::snapshot() const
{
    const std::lock_guard<std::mutex> lock(queue_mutex_);
    return snapshot_locked();
}

TrackingCoordinator::QueueSnapshot TrackingCoordinator::snapshot_locked() const
{
    QueueSnapshot snapshot;
    snapshot.pending = pending_frames_.size();
    snapshot.in_flight =
        (tracking_work_.has_value() ? 1U : 0U) +
        (startup_next_work_.has_value() ? 1U : 0U);
    snapshot.outstanding = snapshot.pending + snapshot.in_flight;
    snapshot.peak = outstanding_frames_peak_;
    snapshot.capacity = pending_frames_capacity_;
    snapshot.enqueued = enqueued_frames_;
    snapshot.processed = processed_frames_;
    snapshot.startup_discarded = startup_discarded_frames_;
    snapshot.overload_discarded = overload_discarded_frames_;
    snapshot.startup_started = startup_wait_started_;
    snapshot.startup_complete = startup_complete_;

    // Reservations retain their original deadline even when the queue is empty.
    if (!pending_frames_.empty())
        snapshot.oldest_received_at = pending_frames_.front().received_at;
    for (const auto *work : {&tracking_work_, &startup_next_work_})
    {
        if (*work && (!snapshot.oldest_received_at ||
                      (*work)->pending.received_at < *snapshot.oldest_received_at))
            snapshot.oldest_received_at = (*work)->pending.received_at;
    }
    snapshot.queued = snapshot.pending;
    snapshot.accounting_valid = snapshot.enqueued == snapshot.outstanding + snapshot.processed +
        snapshot.startup_discarded + snapshot.overload_discarded;

    return snapshot;
}

void TrackingCoordinator::report_final_accounting(const FinalSnapshot &final)
{
    node_logger_->info(
        "STOP_ACCOUNTING enqueued={} queued={} in_flight={} processed={} startup_discarded={} "
        "overload_discarded={} outstanding={} peak={} accounting={}",
        final.enqueued, final.queued, final.in_flight, final.processed, final.startup_discarded,
        final.overload_discarded, final.outstanding, final.peak,
        final.accounting_valid ? "Valid" : "Invalid");
}

void TrackingCoordinator::release_unfinished_work()
{
    std::deque<PendingFrame> queued;
    std::optional<TrackingWork> primary;
    std::optional<TrackingWork> next;
    {
        const std::lock_guard<std::mutex> lock(queue_mutex_);
        queued.swap(pending_frames_);
        primary.swap(tracking_work_);
        next.swap(startup_next_work_);
    }
    // Quiescent teardown releases pixels and consumed IMU data after unlocking.
}

bool TrackingCoordinator::reserve_tracking_work()
{
    // A waiting frame remains owned across retries.
    if (tracking_work_)
        return true;

    const std::lock_guard<std::mutex> lock(queue_mutex_);
    if (pending_frames_.empty())
        return false;

    // Preserve the original enqueue time when transferring ownership.
    tracking_work_.emplace(TrackingWork{
        std::move(pending_frames_.front()),
        TrackingWorkStage::Reserved,
        std::nullopt
    });
    pending_frames_.pop_front();
    return true;
}

bool TrackingCoordinator::reserve_startup_pair()
{
    // A partial execution must not be mistaken for a new startup candidate pair.
    if (!tracking_work_ && startup_next_work_)
        throw std::logic_error("Cannot reserve a new pair after F0 completion");

    if (tracking_work_ && startup_next_work_)
        return true;

    const std::lock_guard<std::mutex> lock(queue_mutex_);

    // Initially need two frames; after MissingHistory, retain F0 and refill F1.
    const std::size_t needed = tracking_work_ ? 1U : 2U;
    if (pending_frames_.size() < needed)
        return false;

    if (!tracking_work_)
    {
        // Retain the frame and its original deadline before removing the queue entry.
        tracking_work_.emplace(TrackingWork{
            std::move(pending_frames_.front()),
            TrackingWorkStage::Reserved,
            std::nullopt
        });

        pending_frames_.pop_front();
    }

    startup_next_work_.emplace(TrackingWork{
        std::move(pending_frames_.front()),
        TrackingWorkStage::Reserved,
        std::nullopt
    });
    pending_frames_.pop_front();
    return true;
}

void TrackingCoordinator::discard_startup_first()
{
    // MissingHistory may discard only candidates whose IMU batch is not consumed.
    if (!tracking_work_ || !startup_next_work_ ||
        tracking_work_->stage != TrackingWorkStage::Reserved ||
        startup_next_work_->stage != TrackingWorkStage::Reserved ||
        tracking_work_->imu_batch || startup_next_work_->imu_batch ||
        tracking_failed_)
    {
        throw std::logic_error("Cannot discard a ready or incomplete startup pair");
    }

    const PendingFrame discarded = tracking_work_->pending;
    std::optional<TrackingWork> released;
    {
        const std::lock_guard<std::mutex> lock(queue_mutex_);

        // Commit promotion and discard accounting together; destroy F0 outside the lock.
        released.swap(tracking_work_);
        tracking_work_.swap(startup_next_work_);
        ++startup_discarded_frames_;
    }

    node_logger_->warn(
        "Startup discard: reason=MissingHistory timestamp={:.9f} "
        "enqueue_sequence={} timestamp_ns={}",
        discarded.frame.timestamp, discarded.enqueue_sequence, discarded.frame.timestamp_ns);
}

void TrackingCoordinator::log_statistics(const char *scope, const Statistics &stats, double elapsed,
                        uint64_t processed)
{
    node_logger_->info("Stereo stats: scope={} frames={} total={} elapsed_sec={:.6f} rate_hz={:.6f} "
                "track_mean_ms={:.6f} track_max_ms={:.6f} intervals={} "
                "interval_min_ms={:.6f} interval_mean_ms={:.6f} interval_max_ms={:.6f}",
                scope, static_cast<unsigned long long>(stats.frames),
                static_cast<unsigned long long>(processed), elapsed,
                elapsed > 0.0 ? stats.frames / elapsed : 0.0,
                stats.frames ? stats.track_sum_ms / stats.frames : 0.0, stats.track_max_ms,
                static_cast<unsigned long long>(stats.intervals),
                stats.intervals ? stats.interval_min_ms : 0.0,
                stats.intervals ? stats.interval_sum_ms / stats.intervals : 0.0,
                stats.interval_max_ms);
}

void TrackingCoordinator::report(bool final)
{
    const QueueSnapshot queue = snapshot();
    const auto now = Clock::now();

    log_statistics(final ? "tail" : "window", window_,
                   std::chrono::duration<double>(now - last_report_).count(), queue.processed);
    if (final)
        log_statistics("total", total_, std::chrono::duration<double>(now - started_).count(),
                       queue.processed);

    if (tracking_mode_ == TrackingMode::StereoImu)
    {
        const double oldest_wait_sec = queue.oldest_received_at ?
            std::chrono::duration<double>(now - *queue.oldest_received_at).count() :
            0.0;
        node_logger_->info("Stereo-IMU coordination: final={} enqueued={} processed={} "
                    "startup_discarded={} pending={} pending_peak={} oldest_wait_sec={:.6f} "
                    "enqueue_to_return_mean_ms={:.6f} enqueue_to_return_max_ms={:.6f} "
                    "in_flight={} outstanding={} overload_discarded={}",
                    final ? "true" : "false",
                    static_cast<unsigned long long>(queue.enqueued),
                    static_cast<unsigned long long>(queue.processed),
                    static_cast<unsigned long long>(queue.startup_discarded),
                    queue.pending, queue.peak, oldest_wait_sec,
                    queue.processed ? enqueue_to_return_sum_ms_ / queue.processed : 0.0,
                    enqueue_to_return_max_ms_, queue.in_flight, queue.outstanding,
                    static_cast<unsigned long long>(queue.overload_discarded));
    }

    // Reset window aggregates without losing the timestamp between adjacent frames.
    window_ = Statistics{};
    last_report_ = now;
}

void TrackingCoordinator::execute_work(TrackingWorkSource work_source)
{
    TrackingWork &work = work_source == TrackingWorkSource::PrimaryReservation ?
        *tracking_work_ : *startup_next_work_;
    if (tracking_failed_ || work.stage != TrackingWorkStage::Ready)
        throw std::logic_error("Cannot execute failed or unready work");
    // Completion removes the owned work; keep frame metadata alive for statistics and notices.
    const StereoFrame frame = work.pending.frame;
    const Clock::time_point received_at = work.pending.received_at;
    const std::vector<ImuMeasurement> empty_batch;
    const std::vector<ImuMeasurement> &imu_measurements = work.imu_batch ?
        work.imu_batch->measurements : empty_batch;
#ifdef GEMINI336_QUEUE_TEST
    if (test_work_point_) test_work_point_(TestWorkPoint::BeforeBackend);
    const TrackingState previous_state = TrackingState::NotInitialized;
#else
    const TrackingState previous_state = slam_->trackingState();
#endif

    if (trace_) trace_->record("track_begin", 0, frame.timestamp, imu_measurements.size());
    const QueueSnapshot queue = snapshot();
#ifdef GEMINI336_QUEUE_TEST
    if (!test_track_)
        throw std::logic_error("Tracking test backend is not configured");
#endif
    if (stop_control_->stop_requested())
        return;
#ifdef GEMINI336_QUEUE_TEST
    if (test_work_point_) test_work_point_(TestWorkPoint::BeforeBackendPermit);
#endif
    // Stop and start are serialized without holding a queue or IMU lock.
    if (!stop_control_->try_begin_backend())
        return;
#ifdef GEMINI336_QUEUE_TEST
    if (test_work_point_) test_work_point_(TestWorkPoint::AfterBackendPermit);
#endif
    // A granted call proceeds even if stop is published before physical entry.
    // Only tracking owns this work; no queue lock is needed for its stage.
    work.stage = TrackingWorkStage::Executing;
    const auto start = Clock::now();
    try
    {
#ifdef GEMINI336_QUEUE_TEST
        test_track_(frame, imu_measurements);
#else
        if (tracking_mode_ == TrackingMode::Stereo)
            slam_->track(frame);
        else
            slam_->track(frame, imu_measurements);
#endif
    }
    catch (...)
    {
        // Preserve the backend's exception; the callback boundary records and cancels.
        tracking_callback_reason_ = StopReason::BackendError;
        throw;
    }

    const auto end = Clock::now();
    uint64_t processed;
    std::optional<TrackingWork> completed;
    {
        const std::lock_guard<std::mutex> lock(queue_mutex_);

        // Commit queue removal and completion together before operational logging.
        switch (work_source)
        {
        case TrackingWorkSource::PrimaryReservation:
            completed.swap(tracking_work_);
            break;
        case TrackingWorkSource::StartupNextReservation:
            completed.swap(startup_next_work_);
            // F1 completion ends startup in the same accounting transaction.
            startup_complete_ = true;
            startup_wait_started_.reset();
            break;
        }
        if (tracking_mode_ == TrackingMode::StereoImu)
            last_tracked_frame_timestamp_ = frame.timestamp;
        processed = ++processed_frames_;
    }

    if (trace_) trace_->record("track_end", 0, frame.timestamp);
    const double track_ms =
        std::chrono::duration<double, std::milli>(end - start).count();

    // Preserve the previous timestamp across report windows; count only successful calls.
    for (auto *stats : {&window_, &total_})
    {
        ++stats->frames;
        stats->track_sum_ms += track_ms;
        stats->track_max_ms = std::max(stats->track_max_ms, track_ms);
        if (queue.processed > 0)
        {
            const double interval_ms = (frame.timestamp - previous_timestamp_) * 1000.0;
            ++stats->intervals;
            stats->interval_sum_ms += interval_ms;
            stats->interval_min_ms = std::min(stats->interval_min_ms, interval_ms);
            stats->interval_max_ms = std::max(stats->interval_max_ms, interval_ms);
        }
    }

    // Use this frame's enqueue time independently of the current queue front.
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(end - received_at).count();
    enqueue_to_return_sum_ms_ += elapsed_ms;
    enqueue_to_return_max_ms_ =
        std::max(enqueue_to_return_max_ms_, elapsed_ms);
    previous_timestamp_ = frame.timestamp;

    // Report only after tracking and its statistics have completed successfully.
    if (processed == 1)
    {
        node_logger_->info("First stereo frame processed: timestamp={:.9f}", frame.timestamp);
        if (first_frame_notification_) first_frame_notification_(frame.timestamp);
    }
#ifdef GEMINI336_QUEUE_TEST
    const TrackingState state = TrackingState::NotInitialized;
#else
    const auto state = slam_->trackingState();
#endif
    node_logger_->debug("Stereo frame: index={} timestamp={:.9f} track_ms={:.6f} state={}",
                 static_cast<unsigned long long>(processed), frame.timestamp,
                 track_ms, tracking_state_name(state));
    if (state != previous_state)
    {
        node_logger_->info("Tracking state: {} -> {}",
                    tracking_state_name(previous_state), tracking_state_name(state));
        if (state_notification_) state_notification_(previous_state, state);
    }
}

void TrackingCoordinator::check_imu_timeline_and_deadlines() const
{
    // Check even with no images: a rejected backwards sample marks a broken timeline.
    const ImuFrontendStats imu_stats = imu_frontend_->stats();
    if (imu_stats.backwards > 0)
        throw CallbackFailure(StopReason::SamplingError, "IMU timestamp moved backwards: count=" +
                              std::to_string(imu_stats.backwards));

    const QueueSnapshot queue = snapshot();
    const Clock::time_point now = Clock::now();
    // Promotion preserves the overall startup deadline even when F0 is discarded.
    if (!queue.startup_complete && queue.startup_started)
    {
        const double waited = std::chrono::duration<double>(now - *queue.startup_started).count();
        if (waited >= imu_wait_timeout_sec_) throw_wait_timeout("Stereo-IMU startup", waited);
    }
    if (queue.oldest_received_at)
    {
        const double waited = std::chrono::duration<double>(now - *queue.oldest_received_at).count();
        if (waited >= imu_wait_timeout_sec_) throw_wait_timeout("Stereo frame", waited);
    }
}

bool TrackingCoordinator::prepare_imu_work(bool startup)
{
    if (tracking_work_->stage != TrackingWorkStage::Reserved ||
        (startup && startup_next_work_->stage != TrackingWorkStage::Reserved))
        throw std::logic_error(startup ? "Cannot resample a ready or executing startup pair" :
                               "Cannot resample a ready or executing frame");

    const double left = startup ? tracking_work_->pending.frame.timestamp :
        *last_tracked_frame_timestamp_;
    const double right = startup ? startup_next_work_->pending.frame.timestamp :
        tracking_work_->pending.frame.timestamp;
#ifdef GEMINI336_QUEUE_TEST
    if (test_work_point_) test_work_point_(TestWorkPoint::BeforeImuQuery);
#endif
    ImuBatch batch = imu_frontend_->takeMeasurements(left, right);
    switch (batch.status)
    {
    case ImuBatchStatus::Stopped:
        return false;
    case ImuBatchStatus::WaitingForData:
#ifdef GEMINI336_QUEUE_TEST
        if (test_work_point_) test_work_point_(TestWorkPoint::AfterWaiting);
#endif
        return false;
    case ImuBatchStatus::MissingHistory:
        if (startup)
        {
            discard_startup_first();
            return false;
        }
        throw_batch_error("IMU history is missing after tracking started", left, right);
    case ImuBatchStatus::BufferOverflow:
        throw_batch_error(startup ? "IMU buffer overflow during startup" : "IMU buffer overflow", left, right);
    case ImuBatchStatus::DataGap:
        throw_batch_error(startup ? "IMU data gap detected during startup" : "IMU data gap detected", left, right);
    case ImuBatchStatus::InvalidRequest:
        throw_batch_error(startup ? "IMU startup batch request is invalid" : "IMU batch request is invalid", left, right);
    case ImuBatchStatus::Ready:
        // Startup F1 owns the interval; F0 executes with an empty batch.
        if (startup)
        {
            startup_next_work_->imu_batch.emplace(std::move(batch));
            startup_next_work_->stage = TrackingWorkStage::Ready;
        }
        else
            tracking_work_->imu_batch.emplace(std::move(batch));
        tracking_work_->stage = TrackingWorkStage::Ready;
        return true;
    }
    return false;
}

void TrackingCoordinator::process_pending_frames(StopReason &failure_reason)
{
    if (stop_control_->stop_requested()) return;
    // A processing failure is terminal; stop alone leaves retained work and batches intact.
    if (tracking_failed_) throw std::logic_error("Cannot retry failed tracking work");
    tracking_callback_reason_ = StopReason::CallbackError;
    try
    {
        const bool with_imu = tracking_mode_ == TrackingMode::StereoImu;
        // Preserve the existing checks even when acquisition will find no work.
        if (with_imu) check_imu_timeline_and_deadlines();
        const bool startup = with_imu && !snapshot().startup_complete;

        // Acquire: retain candidates and their original enqueue times across retries.
        if (startup ? !reserve_startup_pair() : !reserve_tracking_work()) return;

        // Wait: successful sampling stores the batch on its owner before execution.
        if (with_imu)
        {
            if (!prepare_imu_work(startup)) return;
        }
        else
        {
            if (tracking_work_->stage != TrackingWorkStage::Reserved)
                throw std::logic_error("Cannot resample a ready or executing frame");
            tracking_work_->stage = TrackingWorkStage::Ready;
        }
#ifdef GEMINI336_QUEUE_TEST
        if (test_work_point_) test_work_point_(TestWorkPoint::AfterReady);
#endif
        if (stop_control_->stop_requested()) return;

        // Execute and complete: each successful backend return commits ownership and
        // accounting before diagnostics and synchronous node notifications.
        execute_work(TrackingWorkSource::PrimaryReservation);
        if (!startup || stop_control_->stop_requested()) return;
        // F0 completion must not introduce a new deadline check before F1 executes.
        execute_work(TrackingWorkSource::StartupNextReservation);
    }
    catch (...)
    {
        tracking_failed_ = true;
        failure_reason = tracking_callback_reason_;
        throw;
    }
}

[[noreturn]] void TrackingCoordinator::throw_batch_error(const char *reason, double left, double right) const
{
    const QueueSnapshot queue = snapshot();
    std::ostringstream message;
    message << std::setprecision(17) << reason << ": interval=(" << left << ", "
            << right << "] pending=" << queue.pending << " in_flight=" << queue.in_flight
            << " outstanding=" << queue.outstanding;
    throw CallbackFailure(StopReason::SamplingError, message.str());
}

[[noreturn]] void TrackingCoordinator::throw_wait_timeout(const char *scope, double waited_sec) const
{
    const QueueSnapshot queue = snapshot();
    std::ostringstream message;
    message << scope << " wait timed out: waited_sec=" << waited_sec
            << " threshold_sec=" << imu_wait_timeout_sec_
            << " pending=" << queue.pending
            << " in_flight=" << queue.in_flight
            << " outstanding=" << queue.outstanding;
    throw CallbackFailure(StopReason::Timeout, message.str());
}

void TrackingCoordinator::enqueue_frame(
        const StereoFrame &frame, QueueFullPolicy full_policy)
{
    // Stereo reception explicitly selects replacement; other callers default to rejection.
    if (full_policy == QueueFullPolicy::DiscardOldestQueued &&
        tracking_mode_ != TrackingMode::Stereo)
        throw std::invalid_argument("Only Stereo may discard queued frames on overload");

    // Reject invalid sensor timestamp before changing any queue state.
    if (!std::isfinite(frame.timestamp))
        throw std::invalid_argument("Stereo timestamp must be finite");
    if (tracking_mode_ == TrackingMode::StereoImu && frame.timestamp < 0.0)
        throw std::invalid_argument("Stereo-IMU timestamp must be nonnegative");

    // Retain dropped pixels until after unlocking; destruction need not block admission.
    std::optional<PendingFrame> discarded;
    std::unique_lock<std::mutex> lock(queue_mutex_);
    // A passed check admits this transaction; a concurrent stop does not roll it back.
    if (stop_control_->stop_requested())
        return;
    const std::size_t in_flight =
        (tracking_work_.has_value() ? 1U : 0U) +
        (startup_next_work_.has_value() ? 1U : 0U);
    const std::size_t outstanding = pending_frames_.size() + in_flight;

    if (last_received_frame_timestamp_ &&
        frame.timestamp <= *last_received_frame_timestamp_)
    {
        lock.unlock();
        throw std::invalid_argument("Stereo timestamps must be strictly increasing");
    }

    const bool full = outstanding >= pending_frames_capacity_;
    // Reserved work is never eligible, even if it occupies the entire capacity.
    if (full && (full_policy == QueueFullPolicy::Reject || pending_frames_.empty()))
    {
        const std::size_t pending = pending_frames_.size();
        lock.unlock();
        std::ostringstream message;
        message << std::setprecision(17) << "Pending frame queue is full: capacity="
                << pending_frames_capacity_ << " pending=" << pending
                << " incoming_timestamp=" << frame.timestamp
                << " in_flight=" << in_flight
                << " outstanding=" << outstanding;
        throw CallbackFailure(StopReason::Capacity, message.str());
    }

    if (full)
        discarded = pending_frames_.front();

    // Allocate the new deque entry before eviction so allocation failure loses no work.
    // A temporary extra entry is private to this lock; the committed total stays bounded.
    const uint64_t enqueue_sequence = enqueued_frames_ + 1;
    pending_frames_.push_back({frame, Clock::now(), enqueue_sequence});
    if (discarded)
    {
        pending_frames_.pop_front();
        ++overload_discarded_frames_;
    }
    last_received_frame_timestamp_ = frame.timestamp;
    ++enqueued_frames_;
    const std::size_t admitted_outstanding = pending_frames_.size() + in_flight;
    outstanding_frames_peak_ = std::max(outstanding_frames_peak_, admitted_outstanding);

    if (!startup_complete_ && !startup_wait_started_)
        startup_wait_started_ = pending_frames_.back().received_at;

    // Record only after the complete accounting transition, in queue -> trace order.
    if (trace_)
    {
        if (discarded)
            trace_->record("frame_overload_discarded", 0,
                           discarded->frame.timestamp, frame.timestamp);
        trace_->record("frame_enqueued", 0, frame.timestamp, pending_frames_.size());
    }
    const uint64_t overload_discarded = overload_discarded_frames_;
    lock.unlock();

    if (discarded)
    {
        node_logger_->warn(
            "Stereo overload discard: timestamp={:.9f} incoming_timestamp={:.9f} "
            "overload_discarded={} outstanding={} enqueue_sequence={} timestamp_ns={} "
            "incoming_enqueue_sequence={} incoming_timestamp_ns={}",
            discarded->frame.timestamp, frame.timestamp,
            static_cast<unsigned long long>(overload_discarded), admitted_outstanding,
            discarded->enqueue_sequence, discarded->frame.timestamp_ns,
            enqueue_sequence, frame.timestamp_ns);
    }
}
}
