#include "common/context_stop_registration.hpp"
#include <atomic>
#include <iostream>
#include <thread>

using namespace gemini336_orbslam3;
void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}
rclcpp::Context::SharedPtr make_context()
{
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    return context;
}
int main()
{
    try
    {
        for (bool stopped_before_registration : {false, true})
        {
            auto context = make_context();
            auto control = std::make_shared<StopControl>();
            if (stopped_before_registration) context->shutdown("before registration");
            ContextStopRegistration registration(context, control);
            if (!stopped_before_registration) context->shutdown("test notification");
            const auto state = control->snapshot();
            require(state.first_stop && state.first_stop->reason == StopReason::ContextShutdown &&
                    !state.first_failure && !control->try_begin_backend(), "context stop did not close gate");
            require(registration.close() && registration.close(), "close was not idempotent");
        }
        for (bool failure : {false, true})
        {
            auto context = make_context();
            auto control = std::make_shared<StopControl>();
            if (failure) control->record_failure(StopReason::BackendError,
                std::make_exception_ptr(std::runtime_error("original")));
            else control->request_stop(StopReason::InputIdle);
            const auto before = control->snapshot();
            ContextStopRegistration registration(context, control);
            context->shutdown("preserve first cause");
            const auto after = control->snapshot();
            require(after.first_stop->reason == before.first_stop->reason &&
                    after.first_stop->time == before.first_stop->time &&
                    after.first_exception == before.first_exception &&
                    bool(after.first_failure) == failure, "notification overwrote prior cause");
        }
        for (bool explicit_close : {false, true})
        {
            auto context = make_context();
            auto control = std::make_shared<StopControl>();
            {
                ContextStopRegistration registration(context, control);
                if (explicit_close) require(registration.close(), "explicit removal failed");
            }
            context->shutdown("after detach");
            require(!control->stop_requested(), "detached callback still ran");
        }
        {
            auto context = make_context();
            auto control = std::make_shared<StopControl>();
            const std::weak_ptr<StopControl> weak = control;
            ContextStopRegistration registration(context, control);
            control.reset();
            require(weak.expired(), "registration retained control ownership");
            context->shutdown("expired control");
        }
        // Both calls may overlap; never assume removal interrupts an active callback.
        for (int i = 0; i < 16; ++i)
        {
            auto context = make_context();
            auto control = std::make_shared<StopControl>();
            ContextStopRegistration registration(context, control);
            std::atomic<bool> start{false};
            std::thread shutdown([&]() {
                while (!start.load()) std::this_thread::yield();
                context->shutdown("concurrent detach");
            });
            start.store(true);
            const bool removed = registration.close();
            shutdown.join();
            require(removed && !control->snapshot().first_failure, "concurrent removal failed");
            if (control->stop_requested())
                require(control->snapshot().first_stop->reason == StopReason::ContextShutdown,
                        "concurrent notification invented a cause");
        }
        std::cout << "Context stop registration tests passed\n";
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
