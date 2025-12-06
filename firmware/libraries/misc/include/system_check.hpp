/**
 * @file system_check.hpp
 *
 * @brief CAuDri - Utility for runtime system checks
 */
#pragma once

#include <array>
#include <cstddef>

#include "driver.hpp"

constexpr size_t MAX_REGISTERED_DRIVERS = 10;

class SystemCheck {
   public:
    /**
     * @brief Possible overall system states
     *
     * The system can be in one of the following states:
     * - OK: All registered components are functioning properly
     * - WARNING: One or more components are in a non-critical error state
     * - ERROR: One or more components are in a critical error state
     */
    enum class SystemState { OK, WARNING, ERROR };

    /**
     * @brief Struct for individual driver status
     *
     * @param name Name of the driver
     * @param system_critical Whether the driver is critical for system operation
     * @param state Current state of the driver
     * @param connection_state Current connection state of the driver
     */
    struct DriverStatus {
        const char* name = nullptr;
        bool system_critical = true;
        Driver::State state;
        Driver::ConnectionState connection_state;
    };

    /**
     * @brief Result struct for system check results
     *
     * @param system_state Overall system state
     * @param driver_count Number of registered drivers
     * @param driver_status Array of individual driver statuses
     */
    struct Result {
        SystemState system_state = SystemState::OK;

        bool rc_remote_connected = false;
        bool microros_connected = false;

        size_t driver_count = 0;
        std::array<DriverStatus, MAX_REGISTERED_DRIVERS> driver_status{};
    };

    SystemCheck();
    ~SystemCheck();

    bool performCheck(Result& result);
    void logResult(const Result& result);

    bool registerDriver(Driver& driver, bool system_critical = true);

   private:
    void logDriverStatus(const DriverStatus& status);

    size_t driver_count = 0;
    std::array<Driver*, MAX_REGISTERED_DRIVERS> drivers{};
    std::array<bool, MAX_REGISTERED_DRIVERS> driver_critical_flags{};
};
