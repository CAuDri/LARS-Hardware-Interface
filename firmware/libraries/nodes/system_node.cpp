/**
 * @file system_node.cpp
 *
 * @brief CAuDri - micro-ROS hardware operations node implementation
 */

#include "system_node.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.h>

#include <cstring>

#include "logger.h"
#include "system_monitor.hpp"
#include "type_support.hpp"

ROS_DECLARE_MESSAGE_TYPE(std_msgs, Empty);

static constexpr uint32_t SYSTEM_NODE_START_FLAG = 0x01U;
static constexpr uint32_t SYSTEM_NODE_THREAD_POLL_MS = 50U;
static constexpr const char* SYSTEM_NODE_NAME = "system";
static constexpr const char* SYSTEM_NODE_HARDWARE_ID = "hardware_interface";

SystemNode::SystemNode() = default;

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

uint8_t systemDiagnosticLevel(SystemCheck::SystemState state) {
    switch (state) {
        case SystemCheck::SystemState::OK:
            return diagnostic_msgs__msg__DiagnosticStatus__OK;
        case SystemCheck::SystemState::WARNING:
            return diagnostic_msgs__msg__DiagnosticStatus__WARN;
        case SystemCheck::SystemState::ERROR:
            return diagnostic_msgs__msg__DiagnosticStatus__ERROR;
    }
    return diagnostic_msgs__msg__DiagnosticStatus__ERROR;
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

uint8_t driverDiagnosticLevel(const SystemCheck::DriverStatus& status) {
    if (status.state == Driver::State::ERROR) {
        if (status.system_critical) {
            return diagnostic_msgs__msg__DiagnosticStatus__ERROR;
        }
        return diagnostic_msgs__msg__DiagnosticStatus__WARN;
    }
    if (status.connection_state == Driver::ConnectionState::DISCONNECTED) {
        if (status.system_critical) {
            return diagnostic_msgs__msg__DiagnosticStatus__ERROR;
        }
        return diagnostic_msgs__msg__DiagnosticStatus__WARN;
    }
    if (status.state != Driver::State::RUNNING ||
        (status.connection_state != Driver::ConnectionState::CONNECTED &&
         status.connection_state != Driver::ConnectionState::UNKNOWN)) {
        return diagnostic_msgs__msg__DiagnosticStatus__WARN;
    }
    return diagnostic_msgs__msg__DiagnosticStatus__OK;
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

uint8_t clientDiagnosticLevel(const SystemCheck::ClientStatus& status) {
    if (!status.registered) {
        return diagnostic_msgs__msg__DiagnosticStatus__STALE;
    }
    if (status.client_state == ros::Client::State::ERROR ||
        status.executor_state == ros::Executor::State::ERROR) {
        return diagnostic_msgs__msg__DiagnosticStatus__ERROR;
    }
    if (status.client_state != ros::Client::State::CONNECTED ||
        status.connection_state != ros::ConnectionState::CONNECTED ||
        status.executor_state != ros::Executor::State::SPINNING ||
        !status.time_synchronized) {
        return diagnostic_msgs__msg__DiagnosticStatus__WARN;
    }
    return diagnostic_msgs__msg__DiagnosticStatus__OK;
}

const char* diagnosticMessage(uint8_t level) {
    switch (level) {
        case diagnostic_msgs__msg__DiagnosticStatus__OK:
            return "OK";
        case diagnostic_msgs__msg__DiagnosticStatus__WARN:
            return "Warning";
        case diagnostic_msgs__msg__DiagnosticStatus__ERROR:
            return "Error";
        case diagnostic_msgs__msg__DiagnosticStatus__STALE:
            return "Stale";
    }
    return "Unknown";
}

}  // namespace

rcl_ret_t SystemNode::init(Client& client,
                           DriveController& drive_controller,
                           SystemMonitor& system_monitor,
                           const Config& config) {
    if (getState() != EntityState::UNINITIALIZED) {
        return RCL_RET_ALREADY_INIT;
    }
    if (config.heartbeat_topic == nullptr || config.heartbeat_topic[0] == '\0' ||
        config.diagnostics_topic == nullptr || config.diagnostics_topic[0] == '\0' ||
        config.reset_service == nullptr || config.reset_service[0] == '\0' ||
        config.emergency_stop_service == nullptr || config.emergency_stop_service[0] == '\0' ||
        config.heartbeat_period_ms == 0U || config.diagnostics_period_ms == 0U ||
        config.reset_delay_ms == 0U) {
        LogError("SystemNode: Invalid configuration");
        return RCL_RET_INVALID_ARGUMENT;
    }

    this->client = &client;
    this->drive_controller = &drive_controller;
    this->system_monitor = &system_monitor;
    this->config = config;

    rcl_ret_t result = Node::init(client, SYSTEM_NODE_NAME);
    if (result != RCL_RET_OK) {
        LogError("SystemNode: Failed to initialize ROS node: %d", static_cast<int>(result));
        return result;
    }

    result = heartbeat_publisher.init(*this, config.heartbeat_topic, config.heartbeat_publisher_config);
    if (result != RCL_RET_OK) {
        LogError("SystemNode: Failed to initialize heartbeat publisher: %d", static_cast<int>(result));
        return result;
    }

    result = diagnostics_publisher.init(*this, config.diagnostics_topic, config.diagnostics_publisher_config);
    if (result != RCL_RET_OK) {
        LogError("SystemNode: Failed to initialize diagnostics publisher: %d", static_cast<int>(result));
        return result;
    }

    result = reset_service.init(*this, config.reset_service, this, &SystemNode::onResetRequest, config.service_config);
    if (result != RCL_RET_OK) {
        LogError("SystemNode: Failed to initialize reset service: %d", static_cast<int>(result));
        return result;
    }
    configureTriggerResponse(reset_service.response(), reset_response_buffer.data(), reset_response_buffer.size());

    result = emergency_stop_service.init(
        *this, config.emergency_stop_service, this, &SystemNode::onEmergencyStopRequest, config.service_config);
    if (result != RCL_RET_OK) {
        LogError("SystemNode: Failed to initialize emergency stop service: %d", static_cast<int>(result));
        return result;
    }
    configureTriggerResponse(
        emergency_stop_service.response(), emergency_stop_response_buffer.data(), emergency_stop_response_buffer.size());

    thread_attributes.name = "System Node";
    thread_attributes.priority = config.thread_priority;
    thread_attributes.stack_mem = thread_stack.data();
    thread_attributes.stack_size = sizeof(thread_stack);
    thread_attributes.cb_mem = &thread_control_block;
    thread_attributes.cb_size = sizeof(thread_control_block);

    thread_id = osThreadNew(
        [](void* arg) -> void {
            auto* obj = static_cast<SystemNode*>(arg);
            obj->thread();
        },
        this,
        &thread_attributes);

    if (thread_id == nullptr) {
        LogError("SystemNode: Failed to create thread");
        (void)fini();
        return RCL_RET_ERROR;
    }

    LogInfo("SystemNode: Initialized");
    return RCL_RET_OK;
}

rcl_ret_t SystemNode::start() {
    if (started) {
        return RCL_RET_OK;
    }
    if (getState() != EntityState::INITIALIZED) {
        LogError("SystemNode: Cannot start, node is not initialized");
        return RCL_RET_NOT_INIT;
    }
    if (thread_id == nullptr) {
        LogError("SystemNode: Cannot start without thread");
        markError(RCL_RET_ERROR);
        return RCL_RET_ERROR;
    }

    const uint32_t flags = osThreadFlagsSet(thread_id, SYSTEM_NODE_START_FLAG);
    if ((flags & osFlagsError) != 0U) {
        LogError("SystemNode: Failed to start thread, flags: 0x%08lX", flags);
        markError(RCL_RET_ERROR);
        return RCL_RET_ERROR;
    }

    started = true;
    LogInfo("SystemNode: Started");
    return RCL_RET_OK;
}

void SystemNode::onResetRequest(const std_srvs__srv__Trigger_Request* request,
                                std_srvs__srv__Trigger_Response* response) {
    (void)request;

    if (drive_controller == nullptr) {
        fillTriggerResponse(response, false, "Drive controller unavailable");
        return;
    }

    drive_controller->emergencyStop();
    requestHardwareReset(osKernelGetTickCount());
    fillTriggerResponse(response, true, "Hardware reset scheduled");
    LogWarning("SystemNode: Hardware reset requested from ROS");
}

void SystemNode::onEmergencyStopRequest(const std_srvs__srv__Trigger_Request* request,
                                        std_srvs__srv__Trigger_Response* response) {
    (void)request;

    if (drive_controller == nullptr) {
        fillTriggerResponse(response, false, "Drive controller unavailable");
        return;
    }

    drive_controller->emergencyStop();
    fillTriggerResponse(response, true, "Emergency stop triggered");
    LogWarning("SystemNode: Emergency stop requested from ROS");
}

void SystemNode::thread() {
    const uint32_t flags = osThreadFlagsWait(SYSTEM_NODE_START_FLAG, osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) != 0U) {
        LogError("SystemNode: Start flag wait failed: 0x%08lX", flags);
        markError(RCL_RET_ERROR);
        osDelay(osWaitForever);
    }

    uint32_t last_heartbeat_ms = osKernelGetTickCount() - config.heartbeat_period_ms;
    uint32_t last_diagnostics_ms = osKernelGetTickCount() - config.diagnostics_period_ms;

    while (true) {
        const uint32_t now_ms = osKernelGetTickCount();

        if ((now_ms - last_heartbeat_ms) >= config.heartbeat_period_ms) {
            publishHeartbeat();
            last_heartbeat_ms = now_ms;
        }

        if ((now_ms - last_diagnostics_ms) >= config.diagnostics_period_ms) {
            publishDiagnostics();
            last_diagnostics_ms = now_ms;
        }

        handlePendingReset(now_ms);
        osDelay(SYSTEM_NODE_THREAD_POLL_MS);
    }
}

void SystemNode::publishHeartbeat() {
    const rcl_ret_t result = heartbeat_publisher.publish(heartbeat_message);
    if (result == RCL_RET_OK) {
        heartbeat_publish_failure_reported = false;
        return;
    }
    if (result == RCL_RET_NOT_INIT) {
        return;
    }
    if (!heartbeat_publish_failure_reported) {
        LogWarning("SystemNode: Failed to publish heartbeat: %d", static_cast<int>(result));
        heartbeat_publish_failure_reported = true;
    }
}

void SystemNode::publishDiagnostics() {
    if (client == nullptr || system_monitor == nullptr) {
        return;
    }

    diagnostics_publisher.beginArray(client->getRosTime());

    SystemCheck::Result result{};
    uint32_t snapshot_age_ms = 0U;
    if (!system_monitor->getLastSystemCheckResult(result, &snapshot_age_ms)) {
        return;
    }

    publishSystemDiagnostic(result, snapshot_age_ms);
    publishClientDiagnostic(result.client_status);
    for (size_t i = 0; i < result.driver_count; i++) {
        publishDriverDiagnostic(result.driver_status[i]);
    }

    const rcl_ret_t publish_result = diagnostics_publisher.publish();
    if (publish_result == RCL_RET_OK) {
        diagnostics_publish_failure_reported = false;
    } else if (publish_result != RCL_RET_NOT_INIT && !diagnostics_publish_failure_reported) {
        LogWarning("SystemNode: Failed to publish diagnostics: %d", static_cast<int>(publish_result));
        diagnostics_publish_failure_reported = true;
    }
}

void SystemNode::publishSystemDiagnostic(const SystemCheck::Result& result, uint32_t age_ms) {
    const uint8_t level = systemDiagnosticLevel(result.system_state);
    if (!diagnostics_publisher.beginStatus("system", level, diagnosticMessage(level), SYSTEM_NODE_HARDWARE_ID)) {
        return;
    }

    diagnostics_publisher.addValue("state", systemStateString(result.system_state));
    diagnostics_publisher.addValue("snapshot_age_ms", age_ms);
    diagnostics_publisher.addValue("drivers", static_cast<uint32_t>(result.driver_count));
    diagnostics_publisher.addValue("ros_nodes", static_cast<uint32_t>(result.node_count));
}

void SystemNode::publishClientDiagnostic(const SystemCheck::ClientStatus& status) {
    const uint8_t level = clientDiagnosticLevel(status);
    if (!diagnostics_publisher.beginStatus("micro_ros", level, diagnosticMessage(level), SYSTEM_NODE_HARDWARE_ID)) {
        return;
    }

    diagnostics_publisher.addValue("client", rosClientStateString(status.client_state));
    diagnostics_publisher.addValue("executor", rosExecutorStateString(status.executor_state));
    diagnostics_publisher.addValue("time_sync_age_ms_2", status.time_sync_age_ms);
}

void SystemNode::publishDriverDiagnostic(const SystemCheck::DriverStatus& status) {
    const uint8_t level = driverDiagnosticLevel(status);
    const char* name = status.name == nullptr ? "driver" : status.name;
    if (!diagnostics_publisher.beginStatus(name, level, diagnosticMessage(level), SYSTEM_NODE_HARDWARE_ID)) {
        return;
    }

    diagnostics_publisher.addValue("critical", status.system_critical);
    diagnostics_publisher.addValue("state", driverStateString(status.state));
    diagnostics_publisher.addValue("connection", driverConnectionString(status.connection_state));
}

void SystemNode::handlePendingReset(uint32_t now_ms) {
    if (!reset_pending) {
        return;
    }
    if ((now_ms - reset_request_time_ms) < config.reset_delay_ms) {
        return;
    }

    if (drive_controller != nullptr) {
        drive_controller->emergencyStop();
    }

    LogWarning("SystemNode: Performing hardware reset");
    osDelay(50);
    SystemMonitor::systemReset();
}

void SystemNode::requestHardwareReset(uint32_t now_ms) {
    const int32_t lock_state = osKernelLock();
    reset_request_time_ms = now_ms;
    reset_pending = true;
    (void)osKernelRestoreLock(lock_state);
}

void SystemNode::configureTriggerResponse(std_srvs__srv__Trigger_Response& response, char* buffer, size_t buffer_size) {
    if (buffer == nullptr || buffer_size == 0U) {
        return;
    }

    buffer[0] = '\0';
    response.success = false;
    response.message.data = buffer;
    response.message.size = 0U;
    response.message.capacity = buffer_size;
}

void SystemNode::fillTriggerResponse(std_srvs__srv__Trigger_Response* response, bool success, const char* message) {
    if (response == nullptr) {
        return;
    }

    response->success = success;

    if (response->message.data == nullptr || response->message.capacity == 0U) {
        return;
    }

    const char* source = message == nullptr ? "" : message;
    const size_t max_length = response->message.capacity - 1U;
    size_t length = std::strlen(source);
    if (length > max_length) {
        length = max_length;
    }

    std::memcpy(response->message.data, source, length);
    response->message.data[length] = '\0';
    response->message.size = length;
}
