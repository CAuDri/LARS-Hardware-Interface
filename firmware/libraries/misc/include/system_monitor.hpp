/**
 * @file system_monitor.hpp
 *
 * @brief CAuDri - System Monitor for background health checks and various helper functions
 */
#pragma once

#include <array>

#include "cmsis_os2.h"
#include "main.h"
#include "system_check.hpp"

constexpr uint32_t SYSTEM_MONITOR_THREAD_STACK_SIZE = 2048;
constexpr size_t SYSTEM_MONITOR_TRACE_STATE_COUNT = 4;

/**
 * @brief Possible reasons for system wakeup/reset
 *
 * The system can wake up or reset due to one of the following reasons:
 * - UNKNOWN: Wakeup reason could not be determined
 * - BROWN_OUT_RESET: Reset due to brown-out or power up
 * - EXTERNAL_RESET: Reset triggered by external pin
 * - POWER_ON_RESET: Power-on reset (unused on STM32F4)
 * - SOFTWARE_RESET: Reset triggered by software (NVIC_SystemReset)
 * - IWDG_RESET: Independent Watchdog reset
 * - WWDG_RESET: Window Watchdog reset
 * - LOW_POWER_RESET: Low power reset
 */
enum class SystemWakeupReason {
    UNKNOWN,          // Wakeup reason could not be determined
    BROWN_OUT_RESET,  // Reset due to brown-out or power up
    EXTERNAL_RESET,   // Reset triggered by external pin
    POWER_ON_RESET,   // Power-on reset (unused on STM32F4) -> covered by BROWN_OUT_RESET
    SOFTWARE_RESET,   // Reset triggered by software (NVIC_SystemReset)
    IWDG_RESET,       // Independent Watchdog reset
    WWDG_RESET,       // Window Watchdog reset
    LOW_POWER_RESET   // Low power reset
};

/**
 * @brief SystemMonitor class for performing background health checks
 */
class SystemMonitor {
   public:
    /**
     * @brief Configuration struct for SystemMonitor
     *
     * @param reset_gpio_port GPIO port override for controlling external power
     * @param reset_gpio_pin GPIO pin override for controlling external power
     * @param check_interval_ms Interval between system checks in milliseconds
     * @param thread_priority Priority of the monitoring thread
     */
    struct Config {
        GPIO_TypeDef* reset_gpio_port = nullptr;
        uint16_t reset_gpio_pin = 0;

        uint32_t check_interval_ms = 1000;
        osPriority_t thread_priority = osPriorityNormal;
    };

    using SystemStateCallback = void (*)(SystemCheck::SystemState state);

    SystemMonitor();
    SystemMonitor(SystemCheck& system_check, const Config& config);
    ~SystemMonitor();

    bool init(SystemCheck& system_check, const Config& config);
    bool start();

    static void systemReset();
    static bool resetPeripherals(uint32_t off_timeout_ms);
    static void enterBootloader();
    static SystemWakeupReason getWakeupReason();

    SystemCheck::SystemState getSystemState();
    bool getSystemCheckResult(SystemCheck::Result& result);
    bool getLastSystemCheckResult(SystemCheck::Result& result, uint32_t* age_ms = nullptr) const;

    bool registerSystemStateCallback(SystemStateCallback callback);
    bool waitForSystemError(uint32_t timeout_ms);
    bool waitForSystemOK(uint32_t timeout_ms);

   private:
    const Config* config = nullptr;
    SystemCheck* system_check = nullptr;

    static SystemMonitor* instance;

    SystemCheck::Result last_check_result{};
    uint32_t last_check_time_ms = 0;
    osMutexId_t result_mutex = nullptr;
    osMutexAttr_t result_mutex_attributes{};
    StaticSemaphore_t result_mutex_control_block{};

    SystemStateCallback system_state_callback = nullptr;

    osEventFlagsId_t system_event_flags = nullptr;
    osEventFlagsAttr_t event_flags_attributes{};
    StaticEventGroup_t event_flags_control_block{};

    osThreadId_t monitor_thread_id = nullptr;
    osThreadAttr_t thread_attributes{};
    StaticTask_t thread_control_block{};
    uint32_t thread_stack[SYSTEM_MONITOR_THREAD_STACK_SIZE / 4]{};

    void* trace_state_machine = nullptr;
    std::array<void*, SYSTEM_MONITOR_TRACE_STATE_COUNT> trace_state_handles{};
    bool trace_initialized = false;
    bool trace_failed = false;

    bool initTraceStateMachine();
    void traceSystemState(SystemCheck::SystemState state);

    bool performSystemCheck();
    bool publishSystemState();

    void monitorThread(void* arg);
};
