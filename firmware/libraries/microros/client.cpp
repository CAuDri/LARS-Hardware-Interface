/**
 * @file client.cpp
 *
 * @brief CAuDri - micro-ROS client connection and session lifecycle
 */

#include "client.hpp"

#include <rmw_microros/rmw_microros.h>

#include "logger.h"
#include "microros_allocator.h"

namespace ros {

Client* Client::instance = nullptr;

/**
 * @brief Construct an uninitialized micro-ROS client.
 *
 * Native and RTOS resources are deliberately created by init(), allowing the
 * Client object itself to be declared statically.
 */
Client::Client() = default;

/**
 * @brief Configure the transport and start the background connection thread.
 * @param client_config Static configuration that must outlive this client.
 * @return RCL_RET_OK on success, otherwise the first initialization error.
 */
rcl_ret_t Client::init(const Config& client_config) {
    LogDebug("micro-ROS Client: Initializing");
    if (state != State::UNINITIALIZED) {
        LogError("micro-ROS Client: Cannot initialize an already initialized client");
        return RCL_RET_ALREADY_INIT;
    }
    if (!validateConfig(client_config) || instance != nullptr) {
        LogError("micro-ROS Client: Invalid configuration or another client is already active");
        state = State::ERROR;
        last_error = RCL_RET_INVALID_ARGUMENT;
        return last_error;
    }

    instance = this;
    config = &client_config;

    // Recursive locking is required because a connection operation may call a
    // helper such as pingAgent(), which protects itself with the same mutex.
    // Priority inheritance prevents the lower-priority connection thread from
    // indefinitely blocking the real-time executor thread.
    mutex_attributes = {
        .name = "ROS Session",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
        .cb_mem = &mutex_control_block,
        .cb_size = sizeof(mutex_control_block),
    };
    session_mutex = osMutexNew(&mutex_attributes);
    if (session_mutex == nullptr) {
        LogError("micro-ROS Client: Failed to create the session mutex");
        cleanupInitFailure(RCL_RET_ERROR);
        return RCL_RET_ERROR;
    }

    // Event flags provide state-change notifications without polling. They are
    // also used to wake the connection thread early after an executor error.
    event_attributes = {
        .name = "ROS Connection",
        .attr_bits = 0,
        .cb_mem = &event_control_block,
        .cb_size = sizeof(event_control_block),
    };
    connection_events = osEventFlagsNew(&event_attributes);
    if (connection_events == nullptr) {
        LogError("micro-ROS Client: Failed to create connection event flags");
        cleanupInitFailure(RCL_RET_ERROR);
        return RCL_RET_ERROR;
    }

    // Install the shared FreeRTOS allocator before calling any rcl/rmw API.
    // From this point onward both firmware and micro-ROS use the same heap.
    rcl_ret_t result = microros_set_default_allocator();
    if (result != RCL_RET_OK) {
        LogError("micro-ROS Client: Failed to install the FreeRTOS allocator: %d", (int)result);
        cleanupInitFailure(result);
        return result;
    }
    allocator = microros_get_allocator();

    // rmw stores the callback pointers and context; Config must therefore
    // remain valid for the complete lifetime of the Client.
    const Transport& transport = config->transport;
    if (rmw_uros_set_custom_transport(
            transport.framing, transport.context, transport.open, transport.close, transport.write, transport.read) != RMW_RET_OK) {
        LogError("micro-ROS Client: Failed to configure the custom transport");
        cleanupInitFailure(RCL_RET_ERROR);
        return RCL_RET_ERROR;
    }

    // The executor thread is created once and sleeps until a native session is
    // ready. Keeping it alive avoids RTOS object churn during reconnection.
    result = executor.createThread(config->executor_thread_priority, session_mutex, executorError, static_cast<void*>(this));
    if (result != RCL_RET_OK) {
        LogError("micro-ROS Client: Failed to create the executor thread: %d", (int)result);
        cleanupInitFailure(result);
        return result;
    }

    thread_attributes = {
        .name = "ROS Client",
        .attr_bits = osThreadDetached,
        .cb_mem = &thread_control_block,
        .cb_size = sizeof(thread_control_block),
        .stack_mem = thread_stack,
        .stack_size = sizeof(thread_stack),
        .priority = config->client_thread_priority,
        .tz_module = 0,
        .reserved = 0,
    };

    state = State::INITIALIZED;
    publishConnectionState(ConnectionState::DISCONNECTED);
    thread_id = osThreadNew(
        // A non-capturing lambda can be converted to the C function pointer
        // expected by CMSIS-RTOS. The user argument carries the Client object.
        [](void* argument) -> void { static_cast<Client*>(argument)->thread(); },
        this,
        &thread_attributes);
    if (thread_id == nullptr) {
        LogError("micro-ROS Client: Failed to create the connection thread");
        cleanupInitFailure(RCL_RET_ERROR);
        return RCL_RET_ERROR;
    }

    LogInfo("micro-ROS Client: Initialized; waiting for an agent");
    return RCL_RET_OK;
}

/**
 * @brief Stop both background threads and release native resources.
 * @param timeout_ms Maximum wait in milliseconds, or osWaitForever.
 * @return RCL_RET_OK on success or RCL_RET_TIMEOUT if shutdown does not finish.
 */
rcl_ret_t Client::fini(uint32_t timeout_ms) {
    if (state == State::UNINITIALIZED || state == State::STOPPED) {
        return RCL_RET_OK;
    }
    if (connection_events == nullptr || thread_id == nullptr) {
        return RCL_RET_NOT_INIT;
    }

    state = State::STOPPING;
    LogDebug("micro-ROS Client: Stopping");
    stop_requested = true;
    (void)osEventFlagsSet(connection_events, ROS_STOP_CLIENT_FLAG);
    const uint32_t flags = osEventFlagsWait(connection_events, ROS_CLIENT_STOPPED_FLAG, osFlagsWaitAny, timeout_ms);
    if ((flags & osFlagsError) != 0U || (flags & ROS_CLIENT_STOPPED_FLAG) == 0U) {
        LogError("micro-ROS Client: Timed out while stopping the connection thread");
        return RCL_RET_TIMEOUT;
    }

    (void)executor.destroyThread();
    (void)osThreadTerminate(thread_id);
    thread_id = nullptr;
    (void)osEventFlagsDelete(connection_events);
    connection_events = nullptr;
    (void)osMutexDelete(session_mutex);
    session_mutex = nullptr;
    instance = nullptr;
    config = nullptr;
    state = State::STOPPED;
    LogInfo("micro-ROS Client: Stopped");
    return RCL_RET_OK;
}

/** @return Current client lifecycle state. */
Client::State Client::getState() const { return state; }

/** @return Current micro-ROS agent connection state. */
ConnectionState Client::getConnectionState() const { return connection_state; }

/** @return Most recent rcl error observed by the client or executor. */
rcl_ret_t Client::getLastError() const { return last_error; }

/** @return True while a complete micro-ROS session is active. */
bool Client::isConnected() const { return connection_state == ConnectionState::CONNECTED; }

/**
 * @brief Wait until a micro-ROS session is connected.
 * @param timeout_ms Maximum wait in milliseconds; zero performs an immediate check.
 * @return true when connected, otherwise false.
 */
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

/**
 * @brief Wait until the current micro-ROS session is disconnected.
 * @param timeout_ms Maximum wait in milliseconds; zero performs an immediate check.
 * @return true when disconnected, otherwise false.
 */
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

/**
 * @brief Request an agent health check after an entity reports an error.
 * @param error rcl error that caused the health check request.
 */
void Client::signalPossibleDisconnect(rcl_ret_t error) {
    // Entity and executor failures do not necessarily mean that the agent has
    // disappeared. Wake the connection thread so it can verify the session.
    last_error = error;
    if (connection_events != nullptr) {
        (void)osEventFlagsSet(connection_events, ROS_TEST_CONNECTION_FLAG);
    }
}

/** @return Executor owned by this client. */
Executor& Client::getExecutor() { return executor; }

/** @return Read-only executor owned by this client. */
const Executor& Client::getExecutor() const { return executor; }

void Client::executorError(void* context, rcl_ret_t error) {
    static_cast<Client*>(context)->signalPossibleDisconnect(error);
}

void Client::thread() {
    while (!stop_requested) {
        // Remember whether this iteration owned a complete session. Failed
        // discovery attempts are expected and should not flood the info log.
        bool session_connected = false;
        state = State::CONNECTING;
        publishConnectionState(ConnectionState::CONNECTING);
        LogDebug("micro-ROS Client: Searching for agent");
        rcl_ret_t result = connectSession();

        if (result == RCL_RET_OK) {
            state = State::CONNECTED;
            publishConnectionState(ConnectionState::CONNECTED);
            session_connected = true;
            LogInfo("micro-ROS Client: Connected to agent");

            // A timeout is the normal health-check interval. An entity or
            // executor can set ROS_TEST_CONNECTION_FLAG to request an earlier
            // ping when a communication error suggests that the agent vanished.
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
                    LogWarning("micro-ROS Client: Agent health check failed");
                    result = RCL_RET_ERROR;
                    break;
                }
            }
        }

        // support_active can also be true after a partially failed support
        // initialization. Always roll it back before the next attempt.
        if (support_active) {
            state = stop_requested ? State::STOPPING : State::DISCONNECTED;
            publishConnectionState(ConnectionState::DISCONNECTED);
            (void)disconnectSession(result == RCL_RET_OK && stop_requested);
        }
        publishConnectionState(ConnectionState::DISCONNECTED);
        if (!stop_requested) {
            state = State::DISCONNECTED;
            if (session_connected) {
                LogInfo("micro-ROS Client: Disconnected; reconnecting automatically");
            }
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
    // All rcl/rclc operations share one recursive priority-inheritance mutex.
    // The executor uses the same mutex while spinning, preventing concurrent
    // access to the session and to rcutils' global error state.
    if (osMutexAcquire(session_mutex, osWaitForever) != osOK) {
        LogError("micro-ROS Client: Failed to lock the session for connection");
        return RCL_RET_ERROR;
    }

    rcl_ret_t result = RCL_RET_OK;
    if (!pingAgent()) {
        result = RCL_RET_ERROR;
    } else {
        // rclc_support_init creates the native context and XRCE session. A
        // non-null context marks partial ownership even if initialization
        // returns an error, so cleanup can remain deterministic.
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
        LogDebug("micro-ROS Client: Connection attempt failed: %d", (int)result);
        if (support_active) {
            (void)disconnectSession(false);
        }
    } else {
        last_error = RCL_RET_OK;
    }
    return result;
}

rcl_ret_t Client::disconnectSession(bool agent_available) {
    // Stop executor access before destroying any native session resources.
    executor.requestStop();
    rcl_ret_t result = executor.waitForStop(osWaitForever);
    if (osMutexAcquire(session_mutex, osWaitForever) != osOK) {
        LogError("micro-ROS Client: Failed to lock the session for shutdown");
        return RCL_RET_ERROR;
    }

    if (!agent_available && support_active) {
        // Without an agent, waiting for XRCE entity-destruction replies only
        // delays reconnect. A zero timeout makes local cleanup immediate.
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
    if (result != RCL_RET_OK) {
        LogWarning("micro-ROS Client: Session cleanup returned: %d", (int)result);
    }
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

    // Keep the two level-triggered notification flags mutually exclusive.
    // Waiters use osFlagsNoClear, allowing more than one observer to inspect
    // the current state without consuming the notification.
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

void Client::cleanupInitFailure(rcl_ret_t error) {
    // Preserve the originating error so getLastError() agrees with init().
    last_error = error;
    state = State::ERROR;
    (void)executor.destroyThread();
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
