/**
 * @file system_monitor.cpp
 *
 * @brief CAuDri - System Monitor for background health checks and various helper functions
 */
#include "system_monitor.hpp"

#include "logger.h"
#include "usbd_core.h"

// Definitions for external power control GPIO (if not defined in main.h)
#ifndef PWR_EXT_ENABLE_GPIO_Port
    #define PWR_EXT_ENABLE_GPIO_Port nullptr
    #define PWR_EXT_ENABLE_Pin 0
#endif

constexpr uint32_t SYSTEM_RESET_DELAY_MS = 2000;      // Delay before system reset in milliseconds
constexpr uint32_t ENTER_BOOTLOADER_DELAY_MS = 2000;  // Delay before entering bootloader in milliseconds
constexpr uint32_t DEFAULT_EXT_RESET_TIME_MS = 500;   // Default time to hold external power reset low in milliseconds

constexpr uint32_t THREAD_START_FLAG = 0x01;

constexpr uint32_t SYSTEM_EVENT_ERROR_FLAG = 0x01;
constexpr uint32_t SYSTEM_EVENT_WARNING_FLAG = 0x02;
constexpr uint32_t SYSTEM_EVENT_OK_FLAG = 0x04;

/**
 * @brief Construct a new SystemMonitor
 */
SystemMonitor::SystemMonitor() {}

/**
 * @brief Construct a new SystemMonitor with the given SystemCheck and configuration
 *
 * @param system_check Reference to the SystemCheck instance to use
 * @param config Configuration for the SystemMonitor
 */
SystemMonitor::SystemMonitor(SystemCheck& system_check, const Config& config) { init(system_check, config); }

/**
 * @brief Destruct the SystemMonitor and clean up resources
 */
SystemMonitor::~SystemMonitor() {
    // Cleanup thread if running
    if (monitor_thread_id != nullptr) {
        osThreadTerminate(monitor_thread_id);
    }
}

/**
 * @brief Initialize the SystemMonitor with the given SystemCheck and configuration
 *
 * @param system_check Reference to the SystemCheck instance to use
 * @param config Configuration for the SystemMonitor
 */
bool SystemMonitor::init(SystemCheck& system_check, const Config& config) {
    this->config = &config;
    this->system_check = &system_check;

    if (instance != nullptr) {
        LogWarning(
            "System Monitor: Another instance has already been initialized, overwriting the static instance pointer");
    }
    instance = this;

    event_flags_attributes.name = "System Monitor Events";
    event_flags_attributes.cb_mem = &event_flags_control_block;
    event_flags_attributes.cb_size = sizeof(event_flags_control_block);

    system_event_flags = osEventFlagsNew(&event_flags_attributes);
    if (system_event_flags == nullptr) {
        LogError("System Monitor: Failed to create event flags");
        return false;
    }

    thread_attributes.name = "System Monitor";
    thread_attributes.stack_mem = thread_stack;
    thread_attributes.stack_size = sizeof(thread_stack);
    thread_attributes.cb_mem = &thread_control_block;
    thread_attributes.cb_size = sizeof(thread_control_block);
    thread_attributes.priority = config.thread_priority;

    monitor_thread_id = osThreadNew(
        // Helper function for using a non-static method as the thread entry point
        // The 'this' pointer is passed as the user argument to the lambda
        [](void* arg) -> void {
            auto* obj = static_cast<SystemMonitor*>(arg);
            obj->monitorThread(arg);
        },
        this,
        &thread_attributes);

    if (monitor_thread_id == nullptr) {
        LogError("System Monitor: Failed to create monitor thread");
        return false;
    }

    LogInfo("System Monitor: Initialized");
    return true;
}

/**
 * @brief Start the SystemMonitor
 *
 * @return true if the monitor thread was successfully started, false otherwise
 */
bool SystemMonitor::start() {
    if (monitor_thread_id == nullptr) {
        LogWarning("System Monitor: Cannot start, not initialized");
        return false;
    }

    // Signal the thread to start monitoring
    if (osThreadFlagsSet(monitor_thread_id, THREAD_START_FLAG) != osOK) {
        LogError("System Monitor: Failed to set start flag for monitor thread");
        return false;
    }

    return true;
}

/**
 * @brief Perform a system check and return the current system state
 *
 * @return The current SystemState after performing the check
 */
SystemCheck::SystemState SystemMonitor::getSystemState() {
    if (!performSystemCheck()) {
        LogWarning("System Monitor: Failed to perform system check");
        return SystemCheck::SystemState::ERROR;
    }

    return last_check_result.system_state;
}

/**
 * @brief Perform a system check and return the last check result
 *
 * @param result Reference to a Result struct to populate with the last check result
 * @return true if the result was successfully retrieved, false otherwise
 */
bool SystemMonitor::getSystemCheckResult(SystemCheck::Result& result) {
    if (!performSystemCheck()) {
        LogWarning("System Monitor: Failed to perform system check");
        return false;
    }

    result = last_check_result;
    return true;
}

bool SystemMonitor::registerSystemStateCallback(SystemStateCallback callback) {
    if (callback == nullptr) {
        LogError("System Monitor: System state callback is null");
        return false;
    }
    if (system_state_callback != nullptr) {
        LogWarning("System Monitor: Overwriting existing system state callback");
    }

    system_state_callback = callback;
    return true;
}

/**
 * @brief Suspend the calling thread until the system enters ERROR state or timeout occurs
 *
 * @param timeout_ms Maximum time to wait in milliseconds (0 = wait indefinitely)
 * @return true if the system entered ERROR state, false if the timeout was reached or an error occurred
 */
bool SystemMonitor::waitForSystemError(uint32_t timeout_ms) {
    if (system_event_flags == nullptr) {
        LogError("System Monitor: Event flags not initialized");
        return false;
    }

    auto flags = osEventFlagsWait(system_event_flags, SYSTEM_EVENT_ERROR_FLAG, osFlagsNoClear, timeout_ms);
    if (flags & osFlagsError) {
        LogError("System Monitor: Error waiting for system error event, flags: 0x%08lX", flags);
        return false;
    } else if (flags & SYSTEM_EVENT_ERROR_FLAG) {
        return true;
    } else {
        return false;  // Timeout reached
    }
}

/**
 * @brief Suspend the calling thread until the system enters OK state or timeout occurs
 *
 * @param timeout_ms Maximum time to wait in milliseconds (0 = wait indefinitely)
 * @return true if the system entered OK state, false if the timeout was reached or an error occurred
 */
bool SystemMonitor::waitForSystemOK(uint32_t timeout_ms) {
    if (system_event_flags == nullptr) {
        LogError("System Monitor: Event flags not initialized");
        return false;
    }

    auto flags = osEventFlagsWait(system_event_flags, SYSTEM_EVENT_OK_FLAG, osFlagsNoClear, timeout_ms);
    if (flags & osFlagsError) {
        LogError("System Monitor: Error waiting for system OK event, flags: 0x%08lX", flags);
        return false;
    } else if (flags & SYSTEM_EVENT_OK_FLAG) {
        return true;
    } else {
        return false;  // Timeout reached
    }
}

/**
 * @brief Reset external peripherals by toggling the external power control GPIO
 *
 * @param timeout_ms Duration to keep the power off before turning it back on (default: 500 ms)
 * @return true if the peripherals were successfully reset, false otherwise
 */
bool SystemMonitor::resetPeripherals(uint32_t timeout_ms = DEFAULT_EXT_RESET_TIME_MS) {
    GPIO_TypeDef* port;
    uint16_t pin;

    if (instance != nullptr && instance->config != nullptr && instance->config->reset_gpio_port != nullptr) {
        port = instance->config->reset_gpio_port;
        pin = instance->config->reset_gpio_pin;
    } else {
        port = PWR_EXT_ENABLE_GPIO_Port;
        pin = PWR_EXT_ENABLE_Pin;
    }

    // Enable power for external components
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    osDelay(timeout_ms);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    osDelay(100);  // Wait a bit for peripherals to stabilize

    return false;
}

/**
 * @brief Trigger a system reset (microcontroller reboot)
 */
void SystemMonitor::systemReset() {
    LogError("System Monitor: System reset triggered, shutting down in %.1f seconds...", SYSTEM_RESET_DELAY_MS / 1000.0f);
    osDelay(SYSTEM_RESET_DELAY_MS);
    NVIC_SystemReset();
}

/**
 * @brief Enter the system bootloader (on ROM) for firmware updates
 */
void SystemMonitor::enterBootloader() {
    LogError("System Monitor: Entering bootloader, shutting down in %.1f seconds...", ENTER_BOOTLOADER_DELAY_MS / 1000.0f);
    HAL_GPIO_WritePin(DEBUG_LED_RED_GPIO_Port, DEBUG_LED_RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DEBUG_LED_GREEN_GPIO_Port, DEBUG_LED_GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DEBUG_LED_BLUE_GPIO_Port, DEBUG_LED_BLUE_Pin, GPIO_PIN_SET);

    // Deinitialize USB devices
    extern USBD_HandleTypeDef hUsbDeviceHS;
    extern USBD_HandleTypeDef hUsbDeviceFS;
    USBD_DeInit(&hUsbDeviceHS);
    USBD_DeInit(&hUsbDeviceFS);

    osDelay(ENTER_BOOTLOADER_DELAY_MS);

    // Disable all interrupts
    __disable_irq();

    // Disable peripherals and de-initialize HAL
    HAL_RCC_DeInit();
    HAL_DeInit();

    // Reconfigure the system clock to default state
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // Clear and disable all interrupts in NVIC
    for (uint32_t i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    // Re-enable interrupts
    __enable_irq();

    // Remap system memory to 0x00000000
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
    __DSB();
    __ISB();

    // Set bootloader vector table offset
    SCB->VTOR = 0x00000000;

    // Switch to main stack pointer since we are probably already in thread mode
    __set_CONTROL(0);
    __DSB();
    __ISB();

    static constexpr uint32_t SYSMEM_ADDRESS = 0x1FFF0000;          // STM32F4 system memory start address
    uint32_t jump_address = *(__IO uint32_t*)(SYSMEM_ADDRESS + 4);  // Bootloader reset vector

    // Jump to bootloader
    typedef void (*pFunction)(void);
    pFunction jump_to_bootloader = (pFunction)jump_address;

    __set_MSP(*(__IO uint32_t*)SYSMEM_ADDRESS);
    __DSB();
    __ISB();

    jump_to_bootloader();

    // We should never reach here, infinite loop as a fail-safe
    LogError("System Monitor: Somehow returned from bootloader, entering infinite loop");
    while (true) {
    }
}

/**
 * @brief Get the reason for the last system wakeup/reset
 *
 * @return The SystemWakeupReason enum value representing the wakeup reason
 */
SystemWakeupReason SystemMonitor::getWakeupReason() {
    uint32_t reset_cause = RCC->CSR;

    if (reset_cause & RCC_CSR_BORRSTF) {
        LogDebug("System Monitor: Woke up after power up or brown-out event");
        return SystemWakeupReason::BROWN_OUT_RESET;
    } else if (reset_cause & RCC_CSR_PINRSTF) {
        LogDebug("System Monitor: Woke up after external reset pin was triggered");
        return SystemWakeupReason::EXTERNAL_RESET;
    } else if (reset_cause & RCC_CSR_SFTRSTF) {
        LogDebug("System Monitor: Woke up after software reset");
        return SystemWakeupReason::SOFTWARE_RESET;
    } else if (reset_cause & RCC_CSR_IWDGRSTF) {
        LogDebug("System Monitor: Woke up after Independent Watchdog reset");
        return SystemWakeupReason::IWDG_RESET;
    } else if (reset_cause & RCC_CSR_WWDGRSTF) {
        LogDebug("System Monitor: Woke up after Window Watchdog reset");
        return SystemWakeupReason::WWDG_RESET;
    } else if (reset_cause & RCC_CSR_LPWRRSTF) {
        LogDebug("System Monitor: Woke up after automatic low power reset");
        return SystemWakeupReason::LOW_POWER_RESET;
    } else {
        return SystemWakeupReason::UNKNOWN;
    }
}

/**
 * @brief Perform a system check using the assigned SystemCheck instance
 *
 * @return true if the system check was successful, false otherwise
 */
bool SystemMonitor::performSystemCheck() {
    if (system_check == nullptr) {
        LogWarning("System Monitor: No SystemCheck instance assigned");
        return false;
    }

    SystemCheck::Result result;
    if (!system_check->performCheck(result)) {
        LogError("System Monitor: System check failed");
        return false;
    }

    this->last_check_result = result;
    this->last_check_time_ms = osKernelGetTickCount();
    return true;
}

/**
 * @brief Publish the current system state by setting appropriate event flags
 *
 * @return true if the system state was published successfully, false otherwise
 */
bool SystemMonitor::publishSystemState() {
    if (system_event_flags == nullptr) {
        LogError("System Monitor: Event flags not initialized");
        return false;
    }

    constexpr uint32_t all_flags = SYSTEM_EVENT_ERROR_FLAG | SYSTEM_EVENT_WARNING_FLAG | SYSTEM_EVENT_OK_FLAG;
    auto flags = osEventFlagsClear(system_event_flags, all_flags);
    if (flags & osFlagsError) {
        LogError("System Monitor: Failed to clear previous system event flags, flags: 0x%08lX", flags);
        return false;
    }

    switch (last_check_result.system_state) {
        case SystemCheck::SystemState::OK:
            osEventFlagsSet(system_event_flags, SYSTEM_EVENT_OK_FLAG);
            break;
        case SystemCheck::SystemState::WARNING:
            osEventFlagsSet(system_event_flags, SYSTEM_EVENT_WARNING_FLAG);
            break;
        case SystemCheck::SystemState::ERROR:
            osEventFlagsSet(system_event_flags, SYSTEM_EVENT_ERROR_FLAG);
            break;
        default:
            LogError("System Monitor: Unknown system state, cannot publish");
            return false;
    }
    return true;
}

/**
 * @brief Monitor thread function for periodic system checks
 *
 */
void SystemMonitor::monitorThread(void* arg) {
    // Wait for the start signal from the main application
    uint32_t flags = osThreadFlagsWait(THREAD_START_FLAG, osFlagsWaitAny, osWaitForever);
    if (!(flags & THREAD_START_FLAG) || (flags & osFlagsError)) {
        LogError("System Monitor: Error starting monitor thread, flags: 0x%08lX", flags);
        osDelay(osWaitForever);
    }

    LogInfo("System Monitor: Background monitoring started");
    while (true) {
        SystemCheck::SystemState current_state = last_check_result.system_state;
        if (!performSystemCheck()) {
            LogError("System Monitor: Failed to perform system check, retrying...");
            osDelay(config->check_interval_ms);
            continue;
        }
        SystemCheck::SystemState new_state = last_check_result.system_state;

        if (new_state == current_state) {
            // No state change
            osDelay(config->check_interval_ms);
            continue;
        }

        switch (new_state) {
            case SystemCheck::SystemState::OK:
                LogSuccess("System Monitor: System check sucessful, entering OK state");
                break;
            case SystemCheck::SystemState::WARNING:
                LogWarning("System Monitor: System check returned warning, entering WARNING state");
                break;
            case SystemCheck::SystemState::ERROR:
                LogError("System Monitor: System check failed, entering ERROR state");
                break;
        }

        // Raise the appropriate event flags
        publishSystemState();

        // Call the registered callback
        if (system_state_callback != nullptr) {
            system_state_callback(new_state);
        }

        // Print the system check result to the log
        system_check->logResult(last_check_result);
    }
}