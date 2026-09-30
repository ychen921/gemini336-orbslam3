#include "common/callback_guard.hpp"
#include <iostream>

using namespace gemini336_orbslam3;
void require(bool value) { if (!value) throw std::runtime_error("callback guard assertion failed"); }
int main()
{
    try
    {
        for (bool unknown : {false, true})
        for (bool cancel_fails : {false, true})
        for (bool fallback_fails : {false, true})
        {
            auto control = std::make_shared<StopControl>();
            control->request_stop(StopReason::InputIdle);
            int cancelled = 0, fallback = 0, reports = 0;
            CallbackGuard guard(control,
                [&]() { require(control->snapshot().first_failure.has_value()); ++cancelled;
                        if (cancel_fails) throw 7; },
                [&]() { ++fallback; if (fallback_fails) throw std::runtime_error("fallback"); },
                [&](std::exception_ptr) { ++reports; throw std::runtime_error("logger"); });
            guard.run([&]() { if (unknown) throw 42; throw std::runtime_error("original"); });
            const auto state = control->snapshot();
            require(state.first_stop->reason == StopReason::InputIdle &&
                    state.first_failure->reason == StopReason::CallbackError &&
                    cancelled == 1 && fallback == (cancel_fails ? 1 : 0) && reports >= 1);
            bool preserved = false;
            try { std::rethrow_exception(state.first_exception); }
            catch (int value) { preserved = unknown && value == 42; }
            catch (const std::runtime_error &e) { preserved = !unknown && std::string(e.what()) == "original"; }
            require(preserved);
        }
        for (const auto reason : {StopReason::Capacity, StopReason::Timeout, StopReason::SamplingError})
        {
            auto control = std::make_shared<StopControl>();
            CallbackGuard guard(control, []() {}, []() {}, [](std::exception_ptr) {});
            guard.run([&]() { throw CallbackFailure(reason, "classified"); });
            require(control->snapshot().first_failure->reason == reason);
        }
        auto control = std::make_shared<StopControl>();
        int fallback = 0;
        CallbackGuard guard(control, []() { throw 1; }, [&]() { ++fallback; }, [](std::exception_ptr) {});
        guard.cancel();
        require(control->snapshot().first_failure->reason == StopReason::CancelError && fallback == 1);
    }
    catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
