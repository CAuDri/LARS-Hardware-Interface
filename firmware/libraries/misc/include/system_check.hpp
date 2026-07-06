/**
 * @file system_check.hpp
 *
 * @brief CAuDri - Utility for runtime system checks
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "client.hpp"
#include "driver.hpp"
#include "node.hpp"

constexpr size_t MAX_REGISTERED_DRIVERS = 10;
constexpr size_t MAX_REGISTERED_ROS_NODES = 10;

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
     * @brief Status of the single micro-ROS client.
     */
    struct ClientStatus {
        bool registered = false;
        ros::Client::State client_state = ros::Client::State::UNINITIALIZED;
        ros::ConnectionState connection_state = ros::ConnectionState::UNKNOWN;
        ros::Executor::State executor_state = ros::Executor::State::UNINITIALIZED;
        bool time_synchronized = false;
        uint32_t time_sync_age_ms = UINT32_MAX;
        rcl_ret_t client_last_error = RCL_RET_OK;
        rcl_ret_t executor_last_error = RCL_RET_OK;
        rmw_ret_t time_sync_last_error = RMW_RET_OK;
    };

    /**
     * @brief Status of one registered micro-ROS node.
     */
    struct NodeStatus {
        const char* name = nullptr;
        ros::EntityState state = ros::EntityState::UNINITIALIZED;
        rcl_ret_t last_error = RCL_RET_OK;
    };

    /**
     * @brief Result struct for system check results
     *
     * @param system_state Overall system state
     * @param driver_count Number of registered drivers
     * @param driver_status Array of individual driver statuses
     */
    struct Result {
        SystemState system_state = SystemState::ERROR;

        size_t driver_count = 0;
        std::array<DriverStatus, MAX_REGISTERED_DRIVERS> driver_status{};

        ClientStatus client_status{};

        size_t node_count = 0;
        std::array<NodeStatus, MAX_REGISTERED_ROS_NODES> node_status{};
    };

    SystemCheck();
    ~SystemCheck();

    bool performCheck(Result& result);
    void logResult(const Result& result);

    bool registerDriver(Driver& driver, bool system_critical = true);
    bool registerClient(ros::Client& client);
    bool registerNode(const ros::Node& node);

   private:
    void logDriverStatus(const DriverStatus& status);
    void logClientStatus(const ClientStatus& status);
    void logNodeStatus(const NodeStatus& status);

    size_t driver_count = 0;
    std::array<Driver*, MAX_REGISTERED_DRIVERS> drivers{};
    std::array<bool, MAX_REGISTERED_DRIVERS> driver_critical_flags{};

    ros::Client* client = nullptr;
    size_t node_count = 0;
    std::array<const ros::Node*, MAX_REGISTERED_ROS_NODES> nodes{};
};
