#include "common/process_evidence.hpp"
#include "common/process_finalization.hpp"
#include <stdexcept>
#include <string>

int main()
{
    using namespace gemini336_orbslam3;
    for (bool failure : {false, true})
    {
        auto control = std::make_shared<StopControl>();
        control->request_stop(StopReason::InputIdle);
        LoggingEvidence evidence;
        ProcessCleanup cleanup;
        cleanup.finalize_node = []() {};
        cleanup.detach_context = []() {};
        cleanup.release_executor = []() {};
        cleanup.release_node = []() {};
        cleanup.shutdown_context = []() {};
        cleanup.finish_logging = [&]() {
            evidence.status = failure ? "failed" : "ok";
            evidence.dropped = 0;
            if (failure) throw std::runtime_error("flush failure");
        };
        cleanup.release_logging = []() {};
        const int code = finalize_process(control, {}, cleanup, [](std::exception_ptr) {});
        FILE *output = std::tmpfile();
        if (!output) return 1;
        const int result = write_process_evidence(output, control, evidence, code);
        std::rewind(output);
        char buffer[1024]{};
        const bool read = std::fgets(buffer, sizeof(buffer), output) != nullptr;
        std::fclose(output);
        const std::string text(buffer);
        if (!read || result != (failure ? 1 : 0) || text.find("first_stop=InputIdle") == std::string::npos ||
            text.find(failure ? "first_failure=ShutdownError" : "first_failure=none") == std::string::npos ||
            text.find(failure ? "logging_finish_status=failed" : "logging_finish_status=ok") == std::string::npos)
            return 1;
    }
    return 0;
}
