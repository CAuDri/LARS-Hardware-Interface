#include "logger.h"

#include <cassert>
#include <cstdarg>
#include <cstdint>

namespace {

uint32_t normal_calls = 0;
uint32_t inline_calls = 0;
uint32_t clear_calls = 0;
uint32_t argument_evaluations = 0;
uint32_t last_level = LOG_LEVEL_NONE;
bool last_success = false;

int evaluatedArgument() {
    ++argument_evaluations;
    return 42;
}

void expectNormalCall(bool enabled, uint32_t previous_calls, uint32_t level, bool success) {
    assert(normal_calls == previous_calls + (enabled ? 1U : 0U));
    if (enabled) {
        assert(last_level == level);
        assert(last_success == success);
    }
}

}  // namespace

extern "C" bool logger_init(void) { return true; }
extern "C" void logger_trace_init(void) {}

extern "C" void logger_log_message(uint32_t level, bool success, const char* format, ...) {
    assert(format != nullptr);
    ++normal_calls;
    last_level = level;
    last_success = success;
}

extern "C" void logger_log_inline(uint32_t level, bool success, const char* format, ...) {
    assert(format != nullptr);
    ++inline_calls;
    last_level = level;
    last_success = success;
}

extern "C" void logger_clear_terminal(void) { ++clear_calls; }

int main() {
    uint32_t previous_calls = normal_calls;
    LogError("error %d", evaluatedArgument());
    expectNormalCall(TEST_ERROR_ENABLED, previous_calls, LOG_LEVEL_ERROR, false);

    previous_calls = normal_calls;
    LogWarning("warning %d", evaluatedArgument());
    expectNormalCall(TEST_WARNING_ENABLED, previous_calls, LOG_LEVEL_WARNING, false);

    previous_calls = normal_calls;
    LogInfo("info %d", evaluatedArgument());
    expectNormalCall(TEST_INFO_ENABLED, previous_calls, LOG_LEVEL_INFO, false);

    previous_calls = normal_calls;
    LogSuccess("success %d", evaluatedArgument());
    expectNormalCall(TEST_INFO_ENABLED, previous_calls, LOG_LEVEL_INFO, true);

    previous_calls = normal_calls;
    LogDebug("debug %d", evaluatedArgument());
    expectNormalCall(TEST_DEBUG_ENABLED, previous_calls, LOG_LEVEL_DEBUG, false);

    previous_calls = normal_calls;
    LogEmpty();
    expectNormalCall(TEST_INFO_ENABLED, previous_calls, LOG_LEVEL_INFO, false);

    const uint32_t previous_inline_calls = inline_calls;
    LogInline("inline %d", evaluatedArgument());
    assert(inline_calls == previous_inline_calls + (TEST_ERROR_ENABLED ? 1U : 0U));

    const uint32_t previous_clear_calls = clear_calls;
    LogClear();
    assert(clear_calls == previous_clear_calls + (TEST_ERROR_ENABLED ? 1U : 0U));

    const uint32_t expected_evaluations = TEST_ERROR_ENABLED + TEST_WARNING_ENABLED + TEST_INFO_ENABLED +
                                          TEST_INFO_ENABLED + TEST_DEBUG_ENABLED + TEST_ERROR_ENABLED;
    assert(argument_evaluations == expected_evaluations);
    return 0;
}
