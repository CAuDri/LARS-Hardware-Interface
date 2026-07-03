/**
 * @file client.cpp
 *
 * @brief CAuDri - micro-ROS client connection and session lifecycle
 */

#include "client.hpp"

#include <rmw_microros/rmw_microros.h>

#include "microros_allocator.h"

namespace ros {

Client* Client::instance = nullptr;

Client::Client() = default;

rcl_ret_t Client::init(const Config& client_config) {
    if (state != State::UNINITIALIZED) {
        return RCL_RET_ALREADY_INIT;
    }
    if (!validateConfig(client_config) || instance != nullptr) {
        state = State::ERROR;
        last_error = RCL_RET_INVALID_ARGUMENT;
        return last_error;
    }

    instance = this;
    config = &client_config;

    mutex_attributes = {
        .name = "ROS Session",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
        .cb_mem = &mutex_control_block,
        .cb_size = sizeof(mutex_control_block),
    };
    session_mutex = osMutexNew(&mutex_attributes);
    if (session_mutex == nullptr) {
        cleanupInitFailure();
        return RCL_RET_ERROR;
    }

    event_attributes = {
        .name = "ROS Connection",
        .attr_bits = 0,
        .cb_mem = &event_control_block,
        .cb_size = sizeof(event_control_block),
    };
    connection_events = osEventFlagsNew(&event_attributes);
    if (connection_events == nullptr) {
        cleanupInitFailure();
        return RCL_RET_ERROR;
    }

    rcl_ret_t result = microros_set_default_allocator();
    if (result != RCL_RET_OK) {
        cleanupInitFailure();
        return result;
    }
    allocator = microros_get_allocator();

    const Transport& transport = config->transport;
    if (rmw_uros_set_custom_transport(
            transport.framing, transport.context, transport.open, transport.close, transport.write, transport.read) != RMW_RET_OK) {
        cleanupInitFailure();
        return RCL_RET_ERROR;
    }

    result = executor.createTask(config->executor_task_priority, session_mutex, executorError, static_cast<void*>(this));
    if (result != RCL_RET_OK) {
        cleanupInitFailure();
        return result;
    }

    thread_attributes = {
        .name = "ROS Client",
        .attr_bits = osThreadDetached,
        .cb_mem = &thread_control_block,
        .cb_size = sizeof(thread_control_block),
        .stack_mem = thread_stack,
        .stack_size = sizeof(thread_stack),
        .priority = config->client_task_priority,
        .tz_module = 0,
        .reserved = 0,
    };

    state = State::INITIALIZED;
    publishConnectionState(ConnectionState::DISCONNECTED);
    thread_id = osThreadNew(threadEntry, this, &thread_attributes);
    if (thread_id == nullptr) {
        cleanupInitFailure();
        return RCL_RET_ERROR;
    }

    return RCL_RET_OK;
}

rcl_ret_t Client::fini(uint32_t timeout_ms) {
    if (state == State::UNINITIALIZED || state == State::STOPPED) {
        return RCL_RET_OK;
    }
    if (connection_events == nullptr || thread_id == nullptr) {
        return RCL_RET_NOT_INIT;
    }

    state = State::STOPPING;
    stop_requested = true;
    (void)osEventFlagsSet(connection_events, ROS_STOP_CLIENT_FLAG);
    const uint32_t flags = osEventFlagsWait(connection_events, ROS_CLIENT_STOPPED_FLAG, osFlagsWaitAny, timeout_ms);
    if ((flags & osFlagsError) != 0U || (flags & ROS_CLIENT_STOPPED_FLAG) == 0U) {
        return RCL_RET_TIMEOUT;
    }

    (void)executor.destroyTask();
    (void)osThreadTerminate(thread_id);
    thread_id = nullptr;
    (void)osEventFlagsDelete(connection_events);
    connection_events = nullptr;
    (void)osMutexDelete(session_mutex);
    session_mutex = nullptr;
    instance = nullptr;
    config = nullptr;
    state = State::STOPPED;
    return RCL_RET_OK;
}

bool Client::waitForConnection(uint32_t timeout_ms) const {
    if (connection_events == nullptr) {
        return false;
    }
    if (isConnected()) {
        return true;
    }
    const uint32_t flags =
        osEventFlagsWait(connection_events, ROS_CONNECTION_ESTABLISHED_FLAG, osFlagsWaitAny | osFlagsNoClear, timeout_ms);
    return (flags & osFlagsError) == 0U && (flags & ROS_CONNECTION_ESTABLISHED_FLAG) != 0U;
}

bool Client::waitForDisconnect(uint32_t timeout_ms) const {
    if (connection_events == nullptr) {
        return false;
    }
    if (connection_state == ConnectionState::DISCONNECTED) {
        return true;
    }
    const uint32_t flags =
        osEventFlagsWait(connection_events, ROS_CONNECTION_LOST_FLAG, osFlagsWaitAny | osFlagsNoClear, timeout_ms);
    return (flags & osFlagsError) == 0U && (flags & ROS_CONNECTION_LOST_FLAG) != 0U;
}

void Client::signalPossibleDisconnect(rcl_ret_t error) {
    last_error = error;
    if (connection_events != nullptr) {
        (void)osEventFlagsSet(connection_events, ROS_TEST_CONNECTION_FLAG);
    }
}

void Client::threadEntry(void* argument) { static_cast<Client*>(argument)->thread(); }

void Client::executorError(void* context, rcl_ret_t error) {
    static_cast<Client*>(context)->signalPossibleDisconnect(error);
}

void Client::thread() {
    while (!stop_requested) {
        state = State::CONNECTING;
        publishConnectionState(ConnectionState::CONNECTING);
        rcl_ret_t result = connectSession();

        if (result == RCL_RET_OK) {
            state = State::CONNECTED;
            publishConnectionState(ConnectionState::CONNECTED);

            while (!stop_requested && isConnected()) {
                const uint32_t flags = osEventFlagsWait(
                    connection_events, ROS_TEST_CONNECTION_FLAG | ROS_STOP_CLIENT_FLAG, osFlagsWaitAny, config->connection_health_interval_ms);
                if ((flags & osFlagsError) == 0U && (flags & ROS_STOP_CLIENT_FLAG) != 0U) {
                    stop_requested = true;
                    break;
                }
                if ((flags & osFlagsError) != 0U && flags != osFlagsErrorTimeout) {
                    result = RCL_RET_ERROR;
                    break;
                }

                if (!pingAgent()) {
                    result = RCL_RET_ERROR;
                    break;
                }
            }
        }

        if (support_active) {
            state = stop_requested ? State::STOPPING : State::DISCONNECTED;
            publishConnectionState(ConnectionState::DISCONNECTED);
            (void)disconnectSession(result == RCL_RET_OK && stop_requested);
        }
        publishConnectionState(ConnectionState::DISCONNECTED);
        if (!stop_requested) {
            state = State::DISCONNECTED;
            const uint32_t flags = osEventFlagsWait(
                connection_events, ROS_STOP_CLIENT_FLAG, osFlagsWaitAny, config->connection_retry_interval_ms);
            if ((flags & osFlagsError) == 0U && (flags & ROS_STOP_CLIENT_FLAG) != 0U) {
                stop_requested = true;
            }
        }
    }

    if (support_active) {
        (void)disconnectSession(isConnected());
    }
    publishConnectionState(ConnectionState::DISCONNECTED);
    state = State::STOPPED;
    (void)osEventFlagsSet(connection_events, ROS_CLIENT_STOPPED_FLAG);
    osThreadExit();
}

rcl_ret_t Client::connectSession() {
    if (osMutexAcquire(session_mutex, osWaitForever) != osOK) {
        return RCL_RET_ERROR;
    }

    rcl_ret_t result = RCL_RET_OK;
    if (!pingAgent()) {
        result = RCL_RET_ERROR;
    } else {
        support = {};
        result = rclc_support_init(&support, 0, nullptr, &allocator);
        support_active = support.context.impl != nullptr;
    }
    if (result == RCL_RET_OK) {
        result = executor.nativeInit(&support.context, &allocator);
    }
    if (result == RCL_RET_OK) {
        result = executor.prepare();
    }
    if (result == RCL_RET_OK) {
        result = executor.startSpinning();
    }
    (void)osMutexRelease(session_mutex);

    if (result != RCL_RET_OK) {
        last_error = result;
        if (support_active) {
            (void)disconnectSession(false);
        }
    } else {
        last_error = RCL_RET_OK;
    }
    return result;
}

rcl_ret_t Client::disconnectSession(bool agent_available) {
    executor.requestStop();
    rcl_ret_t result = executor.waitForStop(osWaitForever);
    if (osMutexAcquire(session_mutex, osWaitForever) != osOK) {
        return RCL_RET_ERROR;
    }

    if (!agent_available && support_active) {
        rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
        if (rmw_context != nullptr) {
            (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
        }
    }

    const rcl_ret_t executor_result = executor.nativeFini();
    if (result == RCL_RET_OK) {
        result = executor_result;
    }
    if (support_active) {
        const rcl_ret_t support_result = rclc_support_fini(&support);
        if (result == RCL_RET_OK) {
            result = support_result;
        }
    }
    support = {};
    support_active = false;
    (void)osMutexRelease(session_mutex);
    return result;
}

bool Client::pingAgent() {
    if (osMutexGetOwner(session_mutex) == osThreadGetId()) {
        return rmw_uros_ping_agent(config->ping_timeout_ms, config->ping_attempts) == RMW_RET_OK;
    }
    if (osMutexAcquire(session_mutex, osWaitForever) != osOK) {
        return false;
    }
    const bool connected = rmw_uros_ping_agent(config->ping_timeout_ms, config->ping_attempts) == RMW_RET_OK;
    (void)osMutexRelease(session_mutex);
    return connected;
}

void Client::publishConnectionState(ConnectionState new_state) {
    connection_state = new_state;
    if (connection_events == nullptr) {
        return;
    }

    if (new_state == ConnectionState::CONNECTED) {
        (void)osEventFlagsClear(connection_events, ROS_CONNECTION_LOST_FLAG);
        (void)osEventFlagsSet(connection_events, ROS_CONNECTION_ESTABLISHED_FLAG);
    } else if (new_state == ConnectionState::DISCONNECTED) {
        (void)osEventFlagsClear(connection_events, ROS_CONNECTION_ESTABLISHED_FLAG);
        (void)osEventFlagsSet(connection_events, ROS_CONNECTION_LOST_FLAG);
    }
}

bool Client::validateConfig(const Config& client_config) const {
    const Transport& transport = client_config.transport;
    return transport.context != nullptr && transport.open != nullptr && transport.close != nullptr &&
           transport.write != nullptr && transport.read != nullptr && client_config.connection_retry_interval_ms > 0U &&
           client_config.connection_health_interval_ms > 0U && client_config.ping_timeout_ms > 0 &&
           client_config.ping_attempts > 0U;
}

void Client::cleanupInitFailure() {
    last_error = RCL_RET_ERROR;
    state = State::ERROR;
    (void)executor.destroyTask();
    if (connection_events != nullptr) {
        (void)osEventFlagsDelete(connection_events);
        connection_events = nullptr;
    }
    if (session_mutex != nullptr) {
        (void)osMutexDelete(session_mutex);
        session_mutex = nullptr;
    }
    config = nullptr;
    instance = nullptr;
}

}  // namespace ros
