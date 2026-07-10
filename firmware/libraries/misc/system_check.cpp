/**
 * @file system_check.cpp
 *
 * @brief CAuDri - Utility for runtime system checks
 */
#include "system_check.hpp"

#include <cstring>

#include "logger.h"

#define LogResult(...) LogInfo(__VA_ARGS__)

constexpr int LOG_NAME_WIDTH = 16;
constexpr int LOG_STATE_WIDTH = 16;

namespace {

const char* systemStateString(SystemCheck::SystemState state) {
    switch (state) {
        case SystemCheck::SystemState::OK:
            return "OK";
        case SystemCheck::SystemState::WARNING:
            return "WARNING";
        case SystemCheck::SystemState::ERROR:
            return "ERROR";
    }
    return "INVALID";
}

const char* systemStateColor(SystemCheck::SystemState state) {
    switch (state) {
        case SystemCheck::SystemState::OK:
            return LOG_COLOR_GREEN;
        case SystemCheck::SystemState::WARNING:
            return LOG_COLOR_YELLOW;
        case SystemCheck::SystemState::ERROR:
            return LOG_COLOR_RED;
    }
    return LOG_COLOR_RESET;
}

const char* driverStateString(Driver::State state) {
    switch (state) {
        case Driver::State::ERROR:
            return "ERROR";
        case Driver::State::UNINITIALIZED:
            return "UNINITIALIZED";
        case Driver::State::INITIALIZED:
            return "INITIALIZED";
        case Driver::State::RUNNING:
            return "RUNNING";
    }
    return "INVALID";
}

const char* driverStateColor(Driver::State state) {
    switch (state) {
        case Driver::State::ERROR:
            return LOG_COLOR_RED;
        case Driver::State::UNINITIALIZED:
        case Driver::State::INITIALIZED:
            return LOG_COLOR_YELLOW;
        case Driver::State::RUNNING:
            return LOG_COLOR_GREEN;
    }
    return LOG_COLOR_RESET;
}

const char* driverConnectionString(Driver::ConnectionState state) {
    switch (state) {
        case Driver::ConnectionState::DISCONNECTED:
            return "DISCONNECTED";
        case Driver::ConnectionState::CONNECTING:
            return "CONNECTING";
        case Driver::ConnectionState::CONNECTED:
            return "CONNECTED";
        case Driver::ConnectionState::UNKNOWN:
            return "UNKNOWN";
    }
    return "INVALID";
}

const char* driverConnectionColor(Driver::ConnectionState state) {
    switch (state) {
        case Driver::ConnectionState::DISCONNECTED:
            return LOG_COLOR_RED;
        case Driver::ConnectionState::CONNECTING:
        case Driver::ConnectionState::UNKNOWN:
            return LOG_COLOR_YELLOW;
        case Driver::ConnectionState::CONNECTED:
            return LOG_COLOR_GREEN;
    }
    return LOG_COLOR_RESET;
}

const char* rosConnectionString(ros::ConnectionState state) {
    switch (state) {
        case ros::ConnectionState::UNKNOWN:
            return "UNKNOWN";
        case ros::ConnectionState::CONNECTING:
            return "CONNECTING";
        case ros::ConnectionState::CONNECTED:
            return "CONNECTED";
        case ros::ConnectionState::DISCONNECTED:
            return "DISCONNECTED";
    }
    return "INVALID";
}

const char* rosConnectionColor(ros::ConnectionState state) {
    switch (state) {
        case ros::ConnectionState::UNKNOWN:
        case ros::ConnectionState::CONNECTING:
            return LOG_COLOR_YELLOW;
        case ros::ConnectionState::CONNECTED:
            return LOG_COLOR_GREEN;
        case ros::ConnectionState::DISCONNECTED:
            return LOG_COLOR_RED;
    }
    return LOG_COLOR_RESET;
}

const char* rosClientStateString(ros::Client::State state) {
    switch (state) {
        case ros::Client::State::ERROR:
            return "ERROR";
        case ros::Client::State::UNINITIALIZED:
            return "UNINITIALIZED";
        case ros::Client::State::INITIALIZED:
            return "INITIALIZED";
        case ros::Client::State::CONNECTING:
            return "CONNECTING";
        case ros::Client::State::CONNECTED:
            return "CONNECTED";
        case ros::Client::State::DISCONNECTED:
            return "DISCONNECTED";
        case ros::Client::State::STOPPING:
            return "STOPPING";
        case ros::Client::State::STOPPED:
            return "STOPPED";
    }
    return "INVALID";
}

const char* rosClientStateColor(ros::Client::State state) {
    switch (state) {
        case ros::Client::State::ERROR:
            return LOG_COLOR_RED;
        case ros::Client::State::CONNECTED:
            return LOG_COLOR_GREEN;
        case ros::Client::State::UNINITIALIZED:
        case ros::Client::State::INITIALIZED:
        case ros::Client::State::CONNECTING:
        case ros::Client::State::DISCONNECTED:
        case ros::Client::State::STOPPING:
        case ros::Client::State::STOPPED:
            return LOG_COLOR_YELLOW;
    }
    return LOG_COLOR_RESET;
}

const char* rosExecutorStateString(ros::Executor::State state) {
    switch (state) {
        case ros::Executor::State::ERROR:
            return "ERROR";
        case ros::Executor::State::UNINITIALIZED:
            return "UNINITIALIZED";
        case ros::Executor::State::INITIALIZED:
            return "INITIALIZED";
        case ros::Executor::State::SPINNING:
            return "SPINNING";
        case ros::Executor::State::STOPPED:
            return "STOPPED";
    }
    return "INVALID";
}

const char* rosExecutorStateColor(ros::Executor::State state) {
    switch (state) {
        case ros::Executor::State::ERROR:
            return LOG_COLOR_RED;
        case ros::Executor::State::SPINNING:
            return LOG_COLOR_GREEN;
        case ros::Executor::State::UNINITIALIZED:
        case ros::Executor::State::INITIALIZED:
        case ros::Executor::State::STOPPED:
            return LOG_COLOR_YELLOW;
    }
    return LOG_COLOR_RESET;
}

const char* rosEntityStateString(ros::EntityState state) {
    switch (state) {
        case ros::EntityState::ERROR:
            return "ERROR";
        case ros::EntityState::UNINITIALIZED:
            return "UNINITIALIZED";
        case ros::EntityState::INITIALIZED:
            return "INITIALIZED";
        case ros::EntityState::RUNNING:
            return "RUNNING";
    }
    return "INVALID";
}

const char* rosEntityStateColor(ros::EntityState state) {
    switch (state) {
        case ros::EntityState::ERROR:
            return LOG_COLOR_RED;
        case ros::EntityState::UNINITIALIZED:
        case ros::EntityState::INITIALIZED:
            return LOG_COLOR_YELLOW;
        case ros::EntityState::RUNNING:
            return LOG_COLOR_GREEN;
    }
    return LOG_COLOR_RESET;
}

const char* rclErrorString(rcl_ret_t error) {
    switch (error) {
        case RCL_RET_OK:
            return "OK";
        case RCL_RET_ERROR:
            return "ERROR";
        case RCL_RET_TIMEOUT:
            return "TIMEOUT";
        case RCL_RET_BAD_ALLOC:
            return "BAD_ALLOC";
        case RCL_RET_INVALID_ARGUMENT:
            return "BAD_ARG";
        case RCL_RET_UNSUPPORTED:
            return "UNSUPPORT";
        case RCL_RET_ALREADY_INIT:
            return "ALREADY_INIT";
        case RCL_RET_NOT_INIT:
            return "NOT_INIT";
        case RCL_RET_NODE_INVALID:
            return "NODE_INVALID";
        case RCL_RET_PUBLISHER_INVALID:
            return "PUB_INVALID";
        case RCL_RET_SUBSCRIPTION_INVALID:
            return "SUB_INVALID";
        case RCL_RET_SERVICE_INVALID:
            return "SRV_INVALID";
        case RCL_RET_WAIT_SET_EMPTY:
            return "WAIT_EMPTY";
        default:
            return "UNKNOWN";
    }
}

const char* rclErrorInfo(rcl_ret_t error) { return error == RCL_RET_OK ? "" : rclErrorString(error); }

const char* rmwErrorString(rmw_ret_t error) {
    switch (error) {
        case RMW_RET_OK:
            return "OK";
        case RMW_RET_ERROR:
            return "ERROR";
        case RMW_RET_TIMEOUT:
            return "TIMEOUT";
        case RMW_RET_UNSUPPORTED:
            return "UNSUPPORT";
        case RMW_RET_BAD_ALLOC:
            return "BAD_ALLOC";
        case RMW_RET_INVALID_ARGUMENT:
            return "BAD_ARG";
        case RMW_RET_INCORRECT_RMW_IMPLEMENTATION:
            return "RMW_IMPL";
        case RMW_RET_NODE_NAME_NON_EXISTENT:
            return "NODE_MISSING";
        default:
            return "UNKNOWN";
    }
}

const char* rmwErrorInfo(rmw_ret_t error) { return error == RMW_RET_OK ? "" : rmwErrorString(error); }

bool isClientHealthy(const SystemCheck::ClientStatus& status) {
    return status.client_state == ros::Client::State::CONNECTED &&
           status.connection_state == ros::ConnectionState::CONNECTED &&
           status.executor_state == ros::Executor::State::SPINNING &&
           status.time_synchronized;
}

}  // namespace

SystemCheck::SystemCheck() = default;
SystemCheck::~SystemCheck() = default;

/**
 * @brief Register a driver for system checks.
 * @param driver Reference to the driver to register.
 * @param system_critical Whether the driver is critical for system operation.
 * @return true if registration was successful.
 */
bool SystemCheck::registerDriver(Driver& driver, bool system_critical) {
    if (driver_count >= MAX_REGISTERED_DRIVERS) {
        LogError("SystemCheck: Cannot register driver '%s', maximum number reached (%zu)",
                 driver.getName(),
                 MAX_REGISTERED_DRIVERS);
        return false;
    }

    drivers[driver_count] = &driver;
    driver_critical_flags[driver_count] = system_critical;
    driver_count++;
    LogDebug("SystemCheck: Registered driver '%s'", driver.getName());
    return true;
}

/**
 * @brief Register the single micro-ROS client for system checks.
 * @param client Reference to the micro-ROS client.
 * @return true if the client was registered.
 */
bool SystemCheck::registerClient(ros::Client& client) {
    if (this->client != nullptr) {
        LogError("SystemCheck: Cannot register another micro-ROS client");
        return false;
    }

    this->client = &client;
    LogDebug("SystemCheck: Registered micro-ROS client");
    return true;
}

/**
 * @brief Register a micro-ROS node for system checks.
 * @param node Reference to the ROS node to monitor.
 * @return true if the node was registered.
 */
bool SystemCheck::registerNode(const ros::Node& node) {
    if (node_count >= MAX_REGISTERED_ROS_NODES) {
        LogError("SystemCheck: Cannot register ROS node '%s', maximum number reached (%zu)",
                 node.getName(),
                 MAX_REGISTERED_ROS_NODES);
        return false;
    }

    nodes[node_count] = &node;
    node_count++;
    LogDebug("SystemCheck: Registered ROS node '%s'", node.getName());
    return true;
}

/**
 * @brief Perform a system check and populate the result structure.
 * @param result Reference to a Result struct to populate.
 * @return true if the check was performed successfully.
 */
bool SystemCheck::performCheck(Result& result) {
    bool critical_error_found = false;
    bool warning_found = false;

    result.driver_count = driver_count;
    for (size_t i = 0; i < driver_count; i++) {
        Driver* driver = drivers[i];
        DriverStatus& status = result.driver_status[i];

        status.name = driver->getName();
        status.system_critical = driver_critical_flags[i];
        status.state = driver->getState();
        status.connection_state = driver->getConnectionState();

        if (status.system_critical) {
            if (status.state == Driver::State::ERROR) {
                critical_error_found = true;
            } else if (status.state != Driver::State::RUNNING) {
                warning_found = true;
            }

            if (status.connection_state == Driver::ConnectionState::DISCONNECTED) {
                critical_error_found = true;
            } else if (status.connection_state != Driver::ConnectionState::CONNECTED) {
                warning_found = true;
            }
        } else {
            if (status.state == Driver::State::ERROR ||
                status.connection_state == Driver::ConnectionState::DISCONNECTED) {
                warning_found = true;
            }
        }
    }

    result.client_status.registered = client != nullptr;
    if (client != nullptr) {
        ClientStatus& status = result.client_status;
        status.client_state = client->getState();
        status.connection_state = client->getConnectionState();
        status.executor_state = client->getExecutor().getState();
        status.time_synchronized = client->isTimeSynchronized();
        status.time_sync_age_ms = client->getTimeSyncAgeMs();
        status.client_last_error = client->getLastError();
        status.executor_last_error = client->getExecutor().getLastError();
        status.time_sync_last_error = client->getLastTimeSyncError();

        if (status.client_state == ros::Client::State::ERROR ||
            status.executor_state == ros::Executor::State::ERROR) {
            critical_error_found = true;
        } else if (!isClientHealthy(status)) {
            warning_found = true;
        }
    }

    result.node_count = node_count;
    for (size_t i = 0; i < node_count; i++) {
        const ros::Node* node = nodes[i];
        NodeStatus& status = result.node_status[i];

        status.name = node->getName();
        status.state = node->getState();
        status.last_error = node->getLastError();

        if (status.state == ros::EntityState::ERROR) {
            warning_found = true;
        } else if (result.client_status.connection_state == ros::ConnectionState::CONNECTED &&
                   status.state != ros::EntityState::RUNNING) {
            warning_found = true;
        }
    }

    if (critical_error_found) {
        result.system_state = SystemState::ERROR;
    } else if (warning_found) {
        result.system_state = SystemState::WARNING;
    } else {
        result.system_state = SystemState::OK;
    }

    return true;
}

/**
 * @brief Print a formatted system check result.
 * @param result Result to print.
 */
void SystemCheck::logResult(const Result& result) {
    LogResult(" ");
    LogResult("------------- System Check Result -------------");
    LogResult(" ");
    LogResult("System State:      %s%s" LOG_COLOR_RESET,
              systemStateColor(result.system_state),
              systemStateString(result.system_state));
    LogResult(" ");

    LogResult(LOG_COLOR_BLUE "micro-ROS          State             Info" LOG_COLOR_RESET);
    LogResult(LOG_COLOR_BLUE "-----------------------------------------------" LOG_COLOR_RESET);
    logClientStatus(result.client_status);
    LogResult(" ");

    if (result.driver_count == 0) {
        LogResult(LOG_COLOR_YELLOW "No drivers registered for system check." LOG_COLOR_RESET);
    } else {
        LogResult(LOG_COLOR_BLUE "Driver             State             Connection" LOG_COLOR_RESET);
        LogResult(LOG_COLOR_BLUE "-----------------------------------------------" LOG_COLOR_RESET);
        for (size_t i = 0; i < result.driver_count; i++) {
            logDriverStatus(result.driver_status[i]);
        }
    }
    LogResult(" ");

    if (result.node_count == 0) {
        LogResult(LOG_COLOR_YELLOW "No ROS nodes registered for system check." LOG_COLOR_RESET);
    } else {
        LogResult(LOG_COLOR_BLUE "Node               State             Info" LOG_COLOR_RESET);
        LogResult(LOG_COLOR_BLUE "-----------------------------------------------" LOG_COLOR_RESET);
        for (size_t i = 0; i < result.node_count; i++) {
            logNodeStatus(result.node_status[i]);
        }
    }
    LogResult(" ");
}

void SystemCheck::logDriverStatus(const DriverStatus& status) {
    const char* critical_marker = status.system_critical ? " *" : "";
    const size_t marker_length = status.system_critical ? 2U : 0U;
    const size_t name_length = status.name == nullptr ? 0U : std::strlen(status.name);
    const size_t display_length = name_length + marker_length;
    const int padding = display_length < static_cast<size_t>(LOG_NAME_WIDTH)
                            ? static_cast<int>(static_cast<size_t>(LOG_NAME_WIDTH) - display_length)
                            : 0;

    LogResult("%s%s%*s   %s%-15s" LOG_COLOR_RESET "  %s%s" LOG_COLOR_RESET,
              status.name,
              critical_marker,
              padding,
              "",
              driverStateColor(status.state),
              driverStateString(status.state),
              driverConnectionColor(status.connection_state),
              driverConnectionString(status.connection_state));
}

void SystemCheck::logClientStatus(const ClientStatus& status) {
    if (!status.registered) {
        LogResult("%-*s   %s%-*s" LOG_COLOR_RESET,
                  LOG_NAME_WIDTH,
                  "Client",
                  LOG_COLOR_YELLOW,
                  LOG_STATE_WIDTH,
                  "UNREGISTERED");
        return;
    }

    LogResult("%-*s   %s%-*s" LOG_COLOR_RESET,
              LOG_NAME_WIDTH,
              "Connection",
              rosConnectionColor(status.connection_state),
              LOG_STATE_WIDTH,
              rosConnectionString(status.connection_state));

    if (status.time_synchronized) {
        LogResult("%-*s   %s%-*s" LOG_COLOR_RESET "  Age: %lu,%02lus",
                  LOG_NAME_WIDTH,
                  "Time",
                  LOG_COLOR_GREEN,
                  LOG_STATE_WIDTH,
                  "SYNCED",
                  static_cast<unsigned long>(status.time_sync_age_ms / 1000U),
                  static_cast<unsigned long>((status.time_sync_age_ms % 1000U) / 10U));
    } else {
        LogResult("%-*s   %s%-*s" LOG_COLOR_RESET "  " LOG_COLOR_RED "%s" LOG_COLOR_RESET,
                  LOG_NAME_WIDTH,
                  "Time",
                  LOG_COLOR_YELLOW,
                  LOG_STATE_WIDTH,
                  "UNSYNCED",
                  rmwErrorInfo(status.time_sync_last_error));
    }

    const char* client_error =
        status.client_state != ros::Client::State::CONNECTED ? rclErrorInfo(status.client_last_error) : "";
    LogResult("%-*s   %s%-*s" LOG_COLOR_RESET "%s" LOG_COLOR_RED "%s" LOG_COLOR_RESET,
              LOG_NAME_WIDTH,
              "Client",
              rosClientStateColor(status.client_state),
              LOG_STATE_WIDTH,
              rosClientStateString(status.client_state),
              client_error[0] == '\0' ? "" : "  ",
              client_error);

    const char* executor_error =
        status.executor_state != ros::Executor::State::SPINNING ? rclErrorInfo(status.executor_last_error) : "";
    LogResult("%-*s   %s%-*s" LOG_COLOR_RESET "%s" LOG_COLOR_RED "%s" LOG_COLOR_RESET,
              LOG_NAME_WIDTH,
              "Executor",
              rosExecutorStateColor(status.executor_state),
              LOG_STATE_WIDTH,
              rosExecutorStateString(status.executor_state),
              executor_error[0] == '\0' ? "" : "  ",
              executor_error);
}

void SystemCheck::logNodeStatus(const NodeStatus& status) {
    const char* node_error = status.state != ros::EntityState::RUNNING ? rclErrorInfo(status.last_error) : "";
    LogResult("%-*s   %s%-*s" LOG_COLOR_RESET "%s" LOG_COLOR_RED "%s" LOG_COLOR_RESET,
              LOG_NAME_WIDTH,
              status.name,
              rosEntityStateColor(status.state),
              LOG_STATE_WIDTH,
              rosEntityStateString(status.state),
              node_error[0] == '\0' ? "" : "  ",
              node_error);
}
