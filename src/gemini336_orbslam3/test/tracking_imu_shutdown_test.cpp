#include <FrameImuState.h>

#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>

namespace
{
using namespace std::chrono_literals;
using ORB_SLAM3::FrameImuState;

void require(bool condition, const char* message)
{
    if(!condition) throw std::runtime_error(message);
}

// Use the same state/guard implementation as Tracking, without constructing
// cameras, frames, maps or an optimizer. Promises establish ordering; CTest's
// timeout bounds a broken notification/lock path rather than hiding it.
void test_completion_and_replacement_guard()
{
    FrameImuState state;
    {
        FrameImuState::Guard producer = state.Lock();
        producer.BeginFrame();
        producer.StartPreintegration();
    }

    std::promise<void> waiting;
    std::promise<void> acquired;
    std::promise<void> release;
    const std::shared_future<void> released = release.get_future().share();
    auto consumer = std::async(std::launch::async, [&]() {
        FrameImuState::Guard guard = state.Lock();
        waiting.set_value();
        guard.WaitUntilReady();
        acquired.set_value();
        // Retain the guard through the simulated map/frame update.
        require(released.wait_for(2s) == std::future_status::ready, "ready guard was not released");
        return guard.Generation();
    });
    waiting.get_future().get();
    std::future<void> ready = acquired.get_future();
    const bool waited = ready.wait_for(20ms) != std::future_status::ready;
    std::uint64_t generation;
    {
        // This models shutdown arriving during a live producer: it still
        // publishes its normal completion, rather than forging failure/readiness.
        FrameImuState::Guard producer = state.Lock();
        generation = producer.Generation();
        producer.Complete(true);
    }
    const bool became_ready = ready.wait_for(2s) == std::future_status::ready;
    std::promise<void> replacing;
    auto replacement = std::async(std::launch::async, [&]() {
        replacing.set_value();
        FrameImuState::Guard producer = state.Lock();
        producer.BeginFrame();
        producer.Complete(false);
        return producer.Generation();
    });
    replacing.get_future().get();
    // Always release before an assertion so futures can clean up on failure.
    const bool replacement_blocked = replacement.wait_for(20ms) != std::future_status::ready;
    release.set_value();
    const std::uint64_t consumed = consumer.get();
    const std::uint64_t next = replacement.get();
    require(waited && became_ready, "live producer readiness was lost or bypassed");
    require(replacement_blocked, "frame replacement bypassed the ready guard");
    require(consumed == generation && next > generation, "consumer used the wrong generation");
}

void test_unavailable_and_failure()
{
    for(bool failure : {false, true})
    {
        FrameImuState state;
        {
            FrameImuState::Guard producer = state.Lock();
            producer.BeginFrame();
        }
        std::promise<void> waiting;
        auto consumer = std::async(std::launch::async, [&]() {
            FrameImuState::Guard guard = state.Lock();
            waiting.set_value();
            try { guard.WaitUntilReady(); }
            catch(const std::runtime_error& error) { return std::string(error.what()); }
            return std::string();
        });
        waiting.get_future().get();
        {
            FrameImuState::Guard producer = state.Lock();
            producer.StartPreintegration();
            if(failure)
                producer.Fail(std::make_exception_ptr(std::runtime_error("original preintegration failure")));
            else
                producer.Complete(false);
        }
        const std::string message = consumer.get();
        require(failure ? message == "original preintegration failure" : message.find("unavailable") != std::string::npos,
                "terminal producer result was not reported to its waiter");
    }
}

void test_generation_cannot_satisfy_old_wait()
{
    FrameImuState state;
    {
        FrameImuState::Guard producer = state.Lock();
        producer.BeginFrame();
    }
    std::promise<void> waiting;
    auto consumer = std::async(std::launch::async, [&]() {
        FrameImuState::Guard guard = state.Lock();
        waiting.set_value();
        try { guard.WaitUntilReady(); }
        catch(const std::runtime_error& error) { return std::string(error.what()); }
        return std::string();
    });
    waiting.get_future().get();
    {
        FrameImuState::Guard producer = state.Lock();
        producer.BeginFrame();
        producer.StartPreintegration();
        producer.Complete(true);
    }
    require(consumer.get().find("invalidated") != std::string::npos,
            "a new ready frame satisfied an old generation's waiter");
}

void test_reset_releases_map_waiter()
{
    FrameImuState state;
    std::mutex map_mutex;
    {
        FrameImuState::Guard producer = state.Lock();
        producer.BeginFrame();
    }
    std::promise<void> waiting;
    auto mapping = std::async(std::launch::async, [&]() {
        const std::lock_guard<std::mutex> map_lock(map_mutex);
        FrameImuState::Guard guard = state.Lock();
        waiting.set_value();
        try { guard.WaitUntilReady(); }
        catch(const std::runtime_error& error) { return std::string(error.what()); }
        return std::string();
    });
    waiting.get_future().get();
    {
        // Tracking invalidates and releases its guard BEFORE waiting for reset.
        // This wakes a mapper that already holds the map-update mutex.
        FrameImuState::Guard producer = state.Lock();
        producer.Invalidate();
    }
    require(mapping.get().find("invalidated") != std::string::npos,
            "reset left mapping waiting for a producer that will not run");
    const std::lock_guard<std::mutex> map_lock(map_mutex);
}

void test_producer_finishes_before_taking_map_lock()
{
    FrameImuState state;
    std::mutex map_mutex;
    {
        FrameImuState::Guard producer = state.Lock();
        producer.BeginFrame();
    }
    std::promise<void> waiting;
    auto mapping = std::async(std::launch::async, [&]() {
        const std::lock_guard<std::mutex> map_lock(map_mutex);
        FrameImuState::Guard guard = state.Lock();
        waiting.set_value();
        guard.WaitUntilReady();
        return guard.Generation();
    });
    waiting.get_future().get();
    std::uint64_t generation;
    {
        // Match Track's preintegration stage: complete without the map mutex,
        // then drop the frame guard before proceeding into its map-locked stage.
        FrameImuState::Guard producer = state.Lock();
        generation = producer.Generation();
        producer.StartPreintegration();
        producer.Complete(true);
    }
    {
        const std::lock_guard<std::mutex> map_lock(map_mutex);
    }
    require(mapping.get() == generation, "map waiter did not consume producer completion");
}
}

int main()
{
    try
    {
        test_completion_and_replacement_guard();
        test_unavailable_and_failure();
        test_generation_cannot_satisfy_old_wait();
        test_reset_releases_map_waiter();
        test_producer_finishes_before_taking_map_lock();
        std::cout << "Tracking IMU shutdown tests passed\n";
    }
    catch(const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
