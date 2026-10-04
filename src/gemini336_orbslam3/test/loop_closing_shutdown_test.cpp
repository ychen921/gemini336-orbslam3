#include <LoopClosing.h>

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace
{
using namespace std::chrono_literals;

// Use the real launch/join/reset lifecycle with fake optimization work. No maps,
// sensors or complete System are constructed, and no optimizer is executed.
class TestLoopClosing : public ORB_SLAM3::LoopClosing
{
public:
    explicit TestLoopClosing(std::function<bool()> stop = {})
        : LoopClosing(nullptr, nullptr, nullptr, true, false, std::move(stop)) {}

    ~TestLoopClosing()
    {
        // An assertion failure must still reap the finite fake job's handle.
        try { JoinGlobalBundleAdjustment(false); }
        catch (...) {}
    }

    bool start(std::function<void(std::uint64_t)> work)
    {
        return StartGlobalBundleAdjustment(std::move(work));
    }
    void join(bool discard = false) { JoinGlobalBundleAdjustment(discard); }

    std::uint64_t generation()
    {
        const std::lock_guard<std::mutex> lock(mMutexGBA);
        return mnFullBAIdx;
    }
    bool has_thread() const { return mThreadGBA.joinable(); }

    void publish_reset()
    {
        const std::lock_guard<std::mutex> lock(mMutexReset);
        mbResetRequested = true;
    }
    bool reset_pending()
    {
        const std::lock_guard<std::mutex> lock(mMutexReset);
        return mbResetRequested;
    }
};

class LoopWorker
{
public:
    explicit LoopWorker(TestLoopClosing &loop)
        : loop_(loop), worker_([&loop]() { loop.Run(); }) {}
    ~LoopWorker() { finish(); }

    void finish()
    {
        loop_.RequestFinish();
        if (worker_.joinable()) worker_.join();
    }

private:
    TestLoopClosing &loop_;
    std::thread worker_;
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

void test_launch_gates()
{
    std::atomic<bool> stopped{true};
    TestLoopClosing loop([&]() noexcept { return stopped.load(); });
    std::atomic<int> jobs{0};
    require(!loop.start([&](std::uint64_t) { ++jobs; }), "global stop admitted a new GBA");
    require(!loop.has_thread(), "rejected launch left a thread handle");

    // The global request gates GBA, but Run remains available for reset callers.
    LoopWorker worker(loop);
    wait_until([&]() { return !loop.isFinished(); }, "global stop unexpectedly exited Run");
    loop.RequestReset();
    require(!loop.isFinished(), "global stop prevented reset service");
    worker.finish();
    stopped.store(false);
    require(!loop.start([&](std::uint64_t) { ++jobs; }), "finish admitted a new GBA");
    require(jobs.load() == 0, "a rejected GBA executed");
}

void test_finish_launch_race()
{
    for (int iteration = 0; iteration < 50; ++iteration)
    {
        TestLoopClosing loop;
        std::atomic<int> jobs{0};
        std::promise<void> go;
        const std::shared_future<void> ready = go.get_future().share();
        auto stopper = std::async(std::launch::async, [&]() { ready.wait(); loop.RequestFinish(); });
        go.set_value();
        const bool accepted = loop.start([&](std::uint64_t) { ++jobs; });
        stopper.get();
        loop.join();
        // A job granted before finish completes; a finish granted first rejects it.
        require(jobs.load() == (accepted ? 1 : 0), "launch/finish lost or duplicated a job");
        require(!loop.start([&](std::uint64_t) { ++jobs; }), "launch succeeded after finish");
        require(!loop.has_thread() && !loop.isRunningGBA() && loop.isFinishedGBA(),
                "joined GBA retained running state or a handle");
    }
}

void test_discard_and_relaunch()
{
    TestLoopClosing loop;
    std::promise<void> release;
    const std::shared_future<void> ready = release.get_future().share();
    std::promise<std::uint64_t> entered;
    std::atomic<int> applied{0};
    require(loop.start([&](std::uint64_t generation) {
        entered.set_value(generation);
        // Bound fake work so failed assertions can also clean up their threads.
        require(ready.wait_for(2s) == std::future_status::ready, "fake GBA was not released");
        if (loop.generation() == generation) ++applied;
    }), "initial GBA was rejected");
    const std::uint64_t old_generation = entered.get_future().get();
    auto joined = std::async(std::launch::async, [&]() { loop.join(true); });
    // This query needs the GBA mutex, proving join does not keep that mutex held.
    wait_until([&]() { return loop.generation() != old_generation; }, "result was not invalidated");
    const bool waited = joined.wait_for(0ms) != std::future_status::ready;
    release.set_value();
    joined.get();
    require(waited, "join returned before optimization completed");
    require(applied.load() == 0, "obsolete result was applied");
    require(loop.start([&](std::uint64_t generation) {
        require(generation > old_generation, "relaunch reused an old generation");
        ++applied;
    }), "relaunch was rejected");
    loop.join();
    require(applied.load() == 1 && !loop.has_thread(), "relaunch was not joined exactly once");
}

void test_finish_waits_before_reset_acknowledgement()
{
    TestLoopClosing loop;
    std::promise<void> release;
    const std::shared_future<void> ready = release.get_future().share();
    require(loop.start([&](std::uint64_t) {
        require(ready.wait_for(2s) == std::future_status::ready, "fake GBA was not released");
    }), "GBA was rejected");
    const std::uint64_t generation = loop.generation();
    loop.publish_reset();
    loop.RequestFinish();
    LoopWorker worker(loop);
    wait_until([&]() { return loop.generation() != generation; }, "pending reset did not invalidate GBA");
    const bool reset_still_pending = loop.reset_pending();
    const bool still_running = !loop.isFinished();
    release.set_value();
    worker.finish();
    require(reset_still_pending && still_running, "reset/finish preceded the GBA join");
    require(!loop.reset_pending() && loop.isFinished() && !loop.has_thread(),
            "finish left pending reset or an unjoined GBA");
    loop.RethrowFailure();
}

void test_worker_failure()
{
    TestLoopClosing loop;
    require(loop.start([](std::uint64_t) { throw std::runtime_error("original GBA failure"); }),
            "failing GBA was rejected");
    loop.RequestFinish();
    loop.Run();
    require(loop.isFinished() && loop.isFinishedGBA() && !loop.has_thread(),
            "failure prevented lifecycle cleanup");
    for (bool reset : {false, true})
    {
        bool reported = false;
        try
        {
            if (reset) loop.RequestReset();
            else loop.RethrowFailure();
        }
        catch (const std::runtime_error &error)
        {
            reported = std::string(error.what()) == "original GBA failure";
        }
        require(reported, "worker/reset lost the original failure");
    }
}
}

int main()
{
    try
    {
        test_launch_gates();
        test_finish_launch_race();
        test_discard_and_relaunch();
        test_finish_waits_before_reset_acknowledgement();
        test_worker_failure();
        std::cout << "LoopClosing shutdown tests passed\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
