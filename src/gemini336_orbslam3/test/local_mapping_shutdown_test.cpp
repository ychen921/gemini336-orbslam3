#include <LocalMapping.h>

#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
using namespace std::chrono_literals;

// Exercise idle lifecycle paths without maps, sensors, or a running SLAM System.
// Protected access prepares pending reset state and exercises finish publication.
class TestLocalMapping : public ORB_SLAM3::LocalMapping
{
public:
    TestLocalMapping() : LocalMapping(nullptr, nullptr, false, false, "shutdown_test") {}

    void publish_reset(bool active_map)
    {
        const std::lock_guard<std::mutex> lock(mMutexReset);
        if (active_map) mbResetRequestedActiveMap = true;
        else mbResetRequested = true;
    }

    bool reset_pending()
    {
        const std::lock_guard<std::mutex> lock(mMutexReset);
        return mbResetRequested || mbResetRequestedActiveMap;
    }

    void publish_finished() { SetFinish(); }
    void publish_failure()
    {
        RecordFailure(std::make_exception_ptr(std::runtime_error("original mapping failure")));
    }
};

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

template<typename Predicate>
void wait_until(Predicate condition, const char *message)
{
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!condition())
    {
        if (std::chrono::steady_clock::now() >= deadline)
            throw std::runtime_error(message);
        std::this_thread::sleep_for(1ms);
    }
}

// Keep the mapper alive until its worker exits, including on assertion failures.
class MappingWorker
{
public:
    explicit MappingWorker(TestLocalMapping &mapper)
        : mapper_(mapper), thread_([&mapper]() { mapper.Run(); }) {}

    ~MappingWorker() { finish(); }

    void finish()
    {
        if (!thread_.joinable()) return;
        mapper_.RequestFinish();
        thread_.join();
    }

private:
    TestLocalMapping &mapper_;
    std::thread thread_;
};

void test_finish_before_run()
{
    for (bool active_map : {false, true})
    {
        TestLocalMapping mapper;
        mapper.publish_reset(active_map);
        mapper.RequestFinish();
        mapper.Run();
        require(!mapper.reset_pending(), "finish before Run left a reset unacknowledged");
        require(mapper.isFinished() && mapper.isStopped(), "finish state was incomplete");
        require(!mapper.AcceptKeyFrames(), "finished mapper reopened admission");
    }
}

void test_reset_while_paused()
{
    for (bool active_map : {false, true})
    {
        TestLocalMapping mapper;
        mapper.RequestStop();
        MappingWorker worker(mapper);
        wait_until([&]() { return mapper.isStopped(); }, "mapper did not pause");

        auto reset = std::async(std::launch::async, [&]() {
            if (active_map) mapper.RequestResetActiveMap(nullptr);
            else mapper.RequestReset();
        });
        const bool acknowledged = reset.wait_for(2s) == std::future_status::ready;
        // Explicit finish is still required: reset acknowledgement does not end Run.
        const bool still_running = !mapper.isFinished();
        worker.finish();
        require(acknowledged, "paused mapper did not acknowledge reset");
        reset.get();
        require(still_running, "reset unexpectedly finished the mapper");
        require(mapper.isFinished() && mapper.isStopped(), "paused finish was incomplete");
        require(!mapper.AcceptKeyFrames(), "paused finish reopened admission");
    }
}

void test_release_and_finish()
{
    TestLocalMapping mapper;
    std::promise<void> start;
    const std::shared_future<void> ready = start.get_future().share();
    // Both operations acquire stop/finish locks. A CTest timeout catches deadlock;
    // repeated finite contention exercises the previous opposite lock orders.
    std::thread release([&]() {
        ready.wait();
        for (int i = 0; i < 1000; ++i) mapper.Release();
    });
    std::thread finish([&]() {
        ready.wait();
        for (int i = 0; i < 1000; ++i) mapper.publish_finished();
    });
    start.set_value();
    release.join();
    finish.join();
    require(mapper.isFinished() && mapper.isStopped(), "release overwrote finished state");
}

void test_failure_waits_for_explicit_finish()
{
    for(bool active_map : {false, true})
    {
        TestLocalMapping mapper;
        mapper.publish_reset(active_map);
        mapper.publish_failure();
        MappingWorker worker(mapper);
        wait_until([&]() { return mapper.isStopped() && !mapper.reset_pending(); },
                   "failed mapper did not settle pending reset");
        require(!mapper.isFinished() && !mapper.AcceptKeyFrames(),
                "worker failure implicitly finished mapping or reopened admission");
        mapper.Release();
        require(mapper.isStopped(), "Release resumed a failed mapper");
        bool reported = false;
        try
        {
            if(active_map) mapper.RequestResetActiveMap(nullptr);
            else mapper.RequestReset();
        }
        catch(const std::runtime_error& error)
        {
            reported = std::string(error.what()) == "original mapping failure";
        }
        require(reported, "reset caller lost the original worker failure");
        worker.finish();
        require(mapper.isFinished(), "failed mapper did not finish after RequestFinish");
        reported = false;
        try { mapper.RethrowFailure(); }
        catch(const std::runtime_error& error)
        {
            reported = std::string(error.what()) == "original mapping failure";
        }
        require(reported, "finish erased the original worker failure");
    }
}
}

int main()
{
    try
    {
        test_finish_before_run();
        test_reset_while_paused();
        test_release_and_finish();
        test_failure_waits_for_explicit_finish();
        std::cout << "LocalMapping shutdown tests passed\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
