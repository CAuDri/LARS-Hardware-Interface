/**
 * @file system_check.cpp
 *
 * @brief CAuDri - Utility for runtime system checks
 */
#include "system_check.hpp"

#include "logger.h"

#define LogResult(...) LogInfo(__VA_ARGS__)

constexpr size_t LOG_STATE_PADDING = 15;

SystemCheck::SystemCheck() = default;
SystemCheck::~SystemCheck() = default;

/**
 * @brief Register a driver for system checks
 *
 * @param driver Reference to the driver to register
 * @param system_critical Whether the driver is critical for system operation
 *
 * @return true if registration was successful, false if maximum number of drivers reached
 */
bool SystemCheck::registerDriver(Driver& driver, bool system_critical) {
    if (driver_count >= MAX_REGISTERED_DRIVERS) {
        LogError("SystemCheck: Cannot register driver '%s', maximum number reached (%zu)", driver.getName(), MAX_REGISTERED_DRIVERS);
        return false;
    }
    drivers[driver_count] = &driver;
    driver_critical_flags[driver_count] = system_critical;
    driver_count++;
    LogDebug("SystemCheck: Registered driver '%s'", driver.getName());
    return true;
}

/**
 * @brief Perform a system check and populate the result structure
 *
 * @param result Reference to a Result struct to populate
 *
 * @return true if the check was performed successfully
 */
bool SystemCheck::performCheck(Result& result) {
    bool critical_error_found = false;
    bool warning_found = false;

    // Check status of each registered driver
    result.driver_count = driver_count;
    for (size_t i = 0; i < driver_count; i++) {
        Driver* driver = drivers[i];
        DriverStatus& status = result.driver_status[i];

        status.name = driver->getName();
        status.system_critical = driver_critical_flags[i];
        status.state = driver->getState();
        status.connection_state = driver->getConnectionState();

        // Non-critical drivers can only raise a warning and won't affect the overall system state
        if (status.system_critical) {
            if (status.state == Driver::State::ERROR) {
                critical_error_found = true;
            } else if (status.state != Driver::State::RUNNING) {
                warning_found = true;
            }
        } else {
            if (status.state == Driver::State::ERROR) {
                warning_found = true;
            }
        }
    }

    // Determine overall system state
    if (critical_error_found) {
        result.system_state = SystemState::ERROR;
    } else if (warning_found) {
        result.system_state = SystemState::WARNING;
    } else {
        result.system_state = SystemState::OK;
    }

    return true;
}

void SystemCheck::logResult(const Result& result) {
    LogResult(" ");
    LogResult("-------- System Check Result --------");
    LogResult(" ");
    LogResult("System State:     %s",
              (result.system_state == SystemState::OK)        ? (LOG_COLOR_GREEN "OK" LOG_COLOR_RESET)
              : (result.system_state == SystemState::WARNING) ? (LOG_COLOR_YELLOW "WARNING" LOG_COLOR_RESET)
                                                              : (LOG_COLOR_RED "ERROR" LOG_COLOR_RESET));
    LogResult(" ");

    // Log status of each registered driver
    if (result.driver_count == 0) {
        LogResult(LOG_COLOR_YELLOW "No drivers registered for system check." LOG_COLOR_RESET);
        return;
    } else {
        LogResult(LOG_COLOR_BLUE "  Driver          State        Connection" LOG_COLOR_RESET);
        LogResult(LOG_COLOR_BLUE "-----------------------------------------" LOG_COLOR_RESET);
        for (size_t i = 0; i < result.driver_count; i++) {
            logDriverStatus(result.driver_status[i]);
        }
    }
    LogResult(" ");
}

void SystemCheck::logDriverStatus(const DriverStatus& status) {
    const char* state_str = nullptr;
    const char* connection_str = nullptr;
    const char* state_color = LOG_COLOR_RESET;
    const char* connection_color = LOG_COLOR_RESET;

    switch (status.state) {
        case Driver::State::ERROR:
            state_str = "    ERROR    ";
            state_color = LOG_COLOR_RED;
            break;
        case Driver::State::UNINITIALIZED:
            state_str = "UNINITIALIZED";
            state_color = LOG_COLOR_YELLOW;
            break;
        case Driver::State::INITIALIZED:
            state_str = " INITIALIZED ";
            state_color = LOG_COLOR_YELLOW;
            break;
        case Driver::State::RUNNING:
            state_str = "   RUNNING   ";
            state_color = LOG_COLOR_GREEN;
            break;
        default:
            state_str = "   INVALID   ";
            break;
    }

    switch (status.connection_state) {
        case Driver::ConnectionState::DISCONNECTED:
            connection_str = "DISCONNECTED";
            connection_color = LOG_COLOR_RED;
            break;
        case Driver::ConnectionState::CONNECTING:
            connection_str = " CONNECTING ";
            connection_color = LOG_COLOR_YELLOW;
            break;
        case Driver::ConnectionState::CONNECTED:
            connection_str = " CONNECTED  ";
            connection_color = LOG_COLOR_GREEN;
            break;
        case Driver::ConnectionState::UNKNOWN:
            connection_str = "  UNKNOWN   ";
            connection_color = LOG_COLOR_YELLOW;
            break;
        default:
            connection_str = "  INVALID   ";
            break;
    }

    LogResult("%s    %s%s" LOG_COLOR_RESET "   %s%s" LOG_COLOR_RESET, status.name, state_color, state_str, connection_color, connection_str);
}