#include "common/process_finalization.hpp"
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace gemini336_orbslam3;
void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}
int main()
{
    try
    {
        for (int failure = -1; failure < 7; ++failure)
        for (bool unknown : {false, true})
        {
            auto control = std::make_shared<StopControl>();
            control->request_stop(StopReason::InputIdle);
            const auto first_stop = control->snapshot().first_stop;
            std::vector<int> order;
            bool logging_alive = true;
            bool executor_alive = true;
            bool node_alive = true;
            ProcessCleanup cleanup;
            const auto action = [&](int id) {
                return [&, id]() {
                    require(logging_alive, "logging released before its producers");
                    order.push_back(id);
                    if (id == 2) executor_alive = false;
                    if (id == 3) { require(!executor_alive, "node released before executor"); node_alive = false; }
                    if (id == 5) require(!node_alive, "logging finished before node destruction");
                    if (id == 6) logging_alive = false;
                    if (failure == id)
                    {
                        if (unknown) throw 42;
                        throw std::runtime_error("cleanup failure");
                    }
                };
            };
            cleanup = {action(0), action(1), action(2), action(3), action(4), action(5), action(6)};
            const int result = finalize_process(control, {}, cleanup,
                [](std::exception_ptr) { throw std::runtime_error("report failure"); });
            require(order == std::vector<int>({0,1,2,3,4,5,6}), "failure skipped later cleanup");
            require(result == (failure < 0 ? 0 : 1), "late failure did not affect exit status");
            const auto state = control->snapshot();
            require(state.first_stop->reason == first_stop->reason && state.first_stop->time == first_stop->time,
                    "cleanup overwrote stop reason");
            require(state.cleanup_failed == (failure >= 0), "cleanup failure flag incorrect");
            if (failure >= 0)
            {
                bool original = false;
                try { std::rethrow_exception(state.first_exception); }
                catch (int value) { original = unknown && value == 42; }
                catch (const std::runtime_error &error) { original = !unknown && std::string(error.what()) == "cleanup failure"; }
                require(original, "diagnostics overwrote original cleanup exception");
            }
        }
        const auto noop = []() {};
        const ProcessCleanup cleanup{noop, noop, noop, noop, noop, noop, noop};
        const auto error = std::make_exception_ptr(std::runtime_error("construction failed"));
        require(finalize_process({}, error, cleanup, [](std::exception_ptr) {}) == 1,
                "failure before control construction was lost");
        auto normal = std::make_shared<StopControl>();
        normal->request_stop(StopReason::ContextShutdown);
        require(finalize_process(normal, {}, cleanup, [](std::exception_ptr) {}) == 0,
                "normal context stop became an error");
        auto control = std::make_shared<StopControl>();
        control->record_failure(StopReason::BackendError, error);
        require(finalize_process(control, {}, cleanup, [](std::exception_ptr) {}) == 1,
                "existing callback failure was lost");
        std::cout << "Process finalization tests passed\n";
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
