/**
 * @file client.cpp
 *
 * @brief CAuDri - micro-ROS client connection and session lifecycle
 */

#include "client.hpp"

#include <rmw_microros/rmw_microros.h>

#include <climits>

#include "logger.h"
#include "microros_trace.hpp"
#include "microros_allocator.h"
#include "node.hpp"
#include "publisher.hpp"
#include "service.hpp"
#include "subscriber.hpp"

namespace ros {

constexpr int64_t NANOSECONDS_PER_SECOND = 1000000000LL;

Client* Client::instance = nullptr;

int64_t Client::getMonotonicTimeNs() {
    TimeOut_t current_time{};
    vTaskSetTimeOutState(&current_time);

    // Include the FreeRTOS overflow counter so ROS time remains monotonic
    // across the raw TickType_t wraparound.
    const uint64_t ticks =
        (static_cast<uint64_t>(current_time.xOverflowCount) << (sizeof(TickType_t) * CHAR_BIT)) + current_time.xTimeOnEntering;
    return static_cast<int64_t>(ticks / configTICK_RATE_HZ) * NANOSECONDS_PER_SECOND +
           static_cast<int64_t>(ticks % configTICK_RATE_HZ) * NANOSECONDS_PER_SECOND / configTICK_RATE_HZ;
}

/**
 * @brief Construct an uninitialized micro-ROS client.
 *
 * rclc/session and RTOS resources are deliberately created by init(), allowing the
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
    trace::initClient();
    trace::setClientState(static_cast<size_t>(State::UNINITIALIZED));
    if (state != State::UNINITIALIZED) {
        LogError("micro-ROS Client: Cannot initialize an already initialized client");
        trace::incrementErrors();
        return RCL_RET_ALREADY_INIT;
    }
    if (!validateConfig(client_config) || instance != nullptr) {
        LogError("micro-ROS Client: Invalid configuration or another client is already active");
        state = State::ERROR;
        trace::setClientState(static_cast<size_t>(state));
        trace::incrementErrors();
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
        trace::incrementErrors();
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
        trace::incrementErrors();
        cleanupInitFailure(RCL_RET_ERROR);
        return RCL_RET_ERROR;
    }

    // Install the shared FreeRTOS allocator before calling any rcl/rmw API.
    // From this point onward both firmware and micro-ROS use the same heap.
    rcl_ret_t result = microros_set_default_allocator();
    if (result != RCL_RET_OK) {
        LogError("micro-ROS Client: Failed to install the FreeRTOS allocator: %d", (int)result);
        trace::incrementErrors();
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
        trace::incrementErrors();
        cleanupInitFailure(RCL_RET_ERROR);
        return RCL_RET_ERROR;
    }

    // The executor thread is created once and sleeps until an rclc session is
    // ready. Keeping it alive avoids RTOS object churn during reconnection.
    result = executor.createThread(config->executor_thread_priority, session_mutex, executorError, static_cast<void*>(this));
    if (result != RCL_RET_OK) {
        LogError("micro-ROS Client: Failed to create the executor thread: %d", (int)result);
        trace::incrementErrors();
        cleanupInitFailure(result);
        return result;
    }

    thread_attributes = {
        .name = "ROS Client",
        .attr_bits = osThreadDetached,
        .cb_mem = &thread_control_block,
        .cb_size = sizeof(thread_control_block),
        .stack_mem = thread_stack.data(),
        .stack_size = sizeof(thread_stack),
        .priority = config->client_thread_priority,
        .tz_module = 0,
        .reserved = 0,
    };

    state = State::INITIALIZED;
    trace::setClientState(static_cast<size_t>(state));
    publishConnectionState(ConnectionState::DISCONNECTED);
    thread_id = osThreadNew(
        // A non-capturing lambda can be converted to the C function pointer
        // expected by CMSIS-RTOS. The user argument carries the Client object.
        [](void* argument) -> void { static_cast<Client*>(argument)->thread(); },
        this,
        &thread_attributes);
    if (thread_id == nullptr) {
        LogError("micro-ROS Client: Failed to create the connection thread");
        trace::incrementErrors();
        cleanupInitFailure(RCL_RET_ERROR);
        return RCL_RET_ERROR;
    }

    return RCL_RET_OK;
}

/**
 * @brief Stop both background threads and release rclc resources.
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
    trace::setClientState(static_cast<size_t>(state));
    LogDebug("micro-ROS Client: Stopping");
    stop_requested = true;
    (void)osEventFlagsSet(connection_events, ROS_STOP_CLIENT_FLAG);
    const uint32_t flags = osEventFlagsWait(connection_events, ROS_CLIENT_STOPPED_FLAG, osFlagsWaitAny, timeout_ms);
    if ((flags & osFlagsError) != 0U || (flags & ROS_CLIENT_STOPPED_FLAG) == 0U) {
        LogError("micro-ROS Client: Timed out while stopping the connection thread");
        trace::incrementErrors();
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
    trace::setClientState(static_cast<size_t>(state));
    LogInfo("micro-ROS Client: Stopped");
    return RCL_RET_OK;
}

/**
 * @brief Get the client lifecycle state.
 * @return Current client lifecycle state.
 */
Client::State Client::getState() const { return state; }

/**
 * @brief Get the micro-ROS agent connection state.
 * @return Current micro-ROS agent connection state.
 */
ConnectionState Client::getConnectionState() const { return connection_state; }

/**
 * @brief Get the most recent client or executor error.
 * @return Most recent rcl/rclc error observed by the client or executor.
 */
rcl_ret_t Client::getLastError() const { return last_error; }

/**
 * @brief Check whether a complete micro-ROS session is active.
 * @return True while a complete micro-ROS session is active.
 */
bool Client::isConnected() const { return connection_state == ConnectionState::CONNECTED; }

/**
 * @brief Check whether ROS time is synchronized with the agent.
 * @return True after the latest agent time synchronization succeeded.
 */
bool Client::isTimeSynchronized() const { return time_synchronized; }

/**
 * @brief Get the latest micro-ROS time synchronization result.
 * @return Result of the latest micro-ROS time synchronization attempt.
 */
rmw_ret_t Client::getLastTimeSyncError() const { return last_time_sync_error; }

/**
 * @brief Read the current synchronized ROS epoch time without locking the session.
 * @return ROS time, or a zero timestamp while the agent time is unsynchronized.
 */
builtin_interfaces__msg__Time Client::getRosTime() const {
    builtin_interfaces__msg__Time ros_time{};
    int64_t epoch_ns = 0;
    int64_t monotonic_ns = 0;
    bool synchronized = false;

    // Copy the two 64-bit values atomically with respect to the connection
    // thread. Cortex-M4 cannot copy int64_t atomically.
    taskENTER_CRITICAL();
    synchronized = time_synchronized;
    epoch_ns = synchronized_epoch_ns;
    monotonic_ns = synchronized_monotonic_ns;
    taskEXIT_CRITICAL();

    if (!synchronized) {
        return ros_time;
    }

    const int64_t now_ns = getMonotonicTimeNs();
    const int64_t current_epoch_ns = epoch_ns + (now_ns - monotonic_ns);
    if (current_epoch_ns <= 0) {
        return ros_time;
    }

    // ROS Time stores seconds and the sub-second nanosecond remainder in
    // separate fields, matching builtin_interfaces/msg/Time.
    const int64_t seconds = current_epoch_ns / NANOSECONDS_PER_SECOND;
    const int64_t nanoseconds = current_epoch_ns % NANOSECONDS_PER_SECOND;
    ros_time.sec = static_cast<int32_t>(seconds);
    ros_time.nanosec = static_cast<uint32_t>(nanoseconds);
    return ros_time;
}

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
 * @param error rcl/rclc error that caused the health check request.
 */
void Client::signalPossibleDisconnect(rcl_ret_t error) {
    // Entity and executor failures do not necessarily mean that the agent has
    // disappeared. Wake the connection thread so it can verify the session.
    last_error = error;
    if (connection_events != nullptr) {
        (void)osEventFlagsSet(connection_events, ROS_TEST_CONNECTION_FLAG);
    }
}

/**
 * @brief Get the executor owned by this client.
 * @return Executor owned by this client.
 */
Executor& Client::getExecutor() { return executor; }

/**
 * @brief Get the read-only executor owned by this client.
 * @return Read-only executor owned by this client.
 */
const Executor& Client::getExecutor() const { return executor; }

/**
 * @brief Get the namespace prefix configured for this client.
 * @return Client base namespace, or an empty string when no prefix is configured.
 */
const char* Client::getBaseNamespace() const {
    return config == nullptr || config->base_namespace == nullptr ? "" : config->base_namespace;
}

void Client::executorError(void* context, rcl_ret_t error) {
    static_cast<Client*>(context)->signalPossibleDisconnect(error);
}

void Client::thread() {
    bool waiting_for_agent_logged = false;

    while (!stop_requested) {
        // Remember whether this iteration owned a complete session. Failed
        // discovery attempts are expected and should not flood the info log.
        bool session_connected = false;
        state = State::CONNECTING;
        trace::setClientState(static_cast<size_t>(state));
        publishConnectionState(ConnectionState::CONNECTING);
        if (!waiting_for_agent_logged) {
            LogInfo("micro-ROS Client: Waiting for agent");
            waiting_for_agent_logged = true;
        }
        rcl_ret_t result = connectSession();

        if (result == RCL_RET_OK) {
            state = State::CONNECTED;
            trace::setClientState(static_cast<size_t>(state));
            (void)osEventFlagsClear(connection_events, ROS_TEST_CONNECTION_FLAG);
            publishConnectionState(ConnectionState::CONNECTED);
            session_connected = true;
            waiting_for_agent_logged = false;
            LogSuccess("micro-ROS Client: Connected to agent");

            // Keep connected-state health checks lightweight. Periodic time
            // synchronization maintains ROS time and also proves that the
            // current XRCE session still has a responding agent behind it.
            while (!stop_requested && isConnected()) {
                const uint32_t flags = osEventFlagsWait(
                    connection_events, ROS_TEST_CONNECTION_FLAG | ROS_STOP_CLIENT_FLAG, osFlagsWaitAny, config->connection_health_interval_ms);
                if ((flags & osFlagsError) == 0U) {
                    if ((flags & ROS_STOP_CLIENT_FLAG) != 0U) {
                        stop_requested = true;
                        break;
                    }
                    if ((flags & ROS_TEST_CONNECTION_FLAG) != 0U) {
                        LogWarning("micro-ROS Client: Communication error reported; reconnecting");
                        result = last_error == RCL_RET_OK ? RCL_RET_ERROR : last_error;
                        break;
                    }
                } else if (flags != osFlagsErrorTimeout) {
                    result = RCL_RET_ERROR;
                    break;
                }
                const uint32_t time_sync_interval =
                    isTimeSynchronized() ? ROS_TIME_SYNC_INTERVAL_MS : ROS_INITIAL_TIME_SYNC_RETRY_INTERVAL_MS;
                if (osKernelGetTickCount() - last_time_sync_attempt_ms >= time_sync_interval) {
                    // Retry quickly after the first failure. A restarted agent
                    // no longer knows the old XRCE session, so repeated clock
                    // sync failures mean the client has to recreate all rclc
                    // entities against a fresh session.
                    if (!synchronizeTime() && consecutive_time_sync_failures == ROS_TIME_SYNC_FAILURE_RECONNECT_THRESHOLD) {
                        LogWarning("micro-ROS Client: Time synchronization failed repeatedly; reconnecting");
                        result = last_time_sync_error == RMW_RET_OK ? RCL_RET_ERROR : RCL_RET_TIMEOUT;
                        break;
                    }
                }
            }
        }

        // support_active can also be true after a partially failed support
        // initialization. Always roll back before the next attempt.
        if (support_active) {
            state = stop_requested ? State::STOPPING : State::DISCONNECTED;
            trace::setClientState(static_cast<size_t>(state));
            publishConnectionState(ConnectionState::DISCONNECTED);
            (void)disconnectSession(result == RCL_RET_OK && stop_requested);
        }
        publishConnectionState(ConnectionState::DISCONNECTED);
        if (!stop_requested) {
            state = State::DISCONNECTED;
            trace::setClientState(static_cast<size_t>(state));
            if (session_connected) {
                trace::incrementReconnects();
                LogInfo("micro-ROS Client: Disconnected; reconnecting automatically");
                waiting_for_agent_logged = false;
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
    trace::setClientState(static_cast<size_t>(state));
    (void)osEventFlagsSet(connection_events, ROS_CLIENT_STOPPED_FLAG);
    osThreadExit();
}

rcl_ret_t Client::connectSession() {
    trace::incrementConnectionAttempts();
    // All rcl/rclc operations share one recursive priority-inheritance mutex.
    // The executor uses the same mutex while spinning, preventing concurrent
    // access to the session and to rcutils' global error state.
    if (osMutexAcquire(session_mutex, osWaitForever) != osOK) {
        LogError("micro-ROS Client: Failed to lock the session for connection");
        trace::incrementErrors();
        return RCL_RET_ERROR;
    }

    rcl_ret_t result = RCL_RET_OK;
    bool agent_found = false;
    LogDebug("micro-ROS Client: Pinging agent");
    if (!pingAgent()) {
        result = RCL_RET_ERROR;
    } else {
        agent_found = true;
        LogInfo("micro-ROS Client: Agent found; creating session");
        // rclc_support_init creates the rcl context and XRCE session. A
        // non-null context marks partial ownership even if initialization
        // returns an error, so cleanup can remain deterministic.
        support = {};
        result = rclc_support_init(&support, 0, nullptr, &allocator);
        support_active = support.context.impl != nullptr;
    }
    if (result == RCL_RET_OK) {
        // Start time synchronization for the new session. The connection
        // thread keeps retrying until ROS time is available.
        (void)synchronizeTime();
        consecutive_time_sync_failures = 0;
    }
    if (result == RCL_RET_OK) {
        result = executor.initRclcExecutor(&support.context, &allocator);
    }
    if (result == RCL_RET_OK) {
        result = initEntities();
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
        // A failed ping simply means the agent is not available yet. Only log
        // failures that happen after discovery succeeded, because those point
        // to a real session setup or cleanup problem.
        if (agent_found || support_active) {
            LogWarning("micro-ROS Client: Session setup failed: %d", (int)result);
            trace::incrementErrors();
        }
        if (support_active) {
            (void)disconnectSession(false);
        }
    } else {
        last_error = RCL_RET_OK;
    }
    return result;
}

rcl_ret_t Client::disconnectSession(bool agent_available) {
    // Stop executor access before destroying any rclc session resources.
    clearSynchronizedTime();
    executor.requestStop();
    rcl_ret_t result = executor.waitForStop(osWaitForever);
    if (osMutexAcquire(session_mutex, osWaitForever) != osOK) {
        LogError("micro-ROS Client: Failed to lock the session for shutdown");
        trace::incrementErrors();
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

    const rcl_ret_t entities_result = finiEntities();
    if (result == RCL_RET_OK) {
        result = entities_result;
    }

    const rcl_ret_t executor_result = executor.finiRclcExecutor();
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
        trace::incrementErrors();
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

bool Client::synchronizeTime() {
    const bool mutex_owned = osMutexGetOwner(session_mutex) == osThreadGetId();
    if (!mutex_owned && osMutexAcquire(session_mutex, osWaitForever) != osOK) {
        last_time_sync_error = RMW_RET_ERROR;
        clearSynchronizedTime();
        LogError("micro-ROS Client: Failed to lock the session for time synchronization");
        trace::incrementErrors();
        return false;
    }

    last_time_sync_attempt_ms = osKernelGetTickCount();
    const rmw_ret_t result = rmw_uros_sync_session(ROS_TIME_SYNC_TIMEOUT_MS);
    const bool synchronized = result == RMW_RET_OK && rmw_uros_epoch_synchronized();
    if (synchronized) {
        const int64_t epoch_ns = rmw_uros_epoch_nanos();
        const int64_t monotonic_ns = getMonotonicTimeNs();

        // Store a self-contained clock snapshot. Readers extrapolate it from
        // the local monotonic clock and therefore never touch mutable session
        // memory or contend with executor/session operations.
        taskENTER_CRITICAL();
        synchronized_epoch_ns = epoch_ns;
        synchronized_monotonic_ns = monotonic_ns;
        time_synchronized = true;
        taskEXIT_CRITICAL();
        consecutive_time_sync_failures = 0;
    } else {
        clearSynchronizedTime();
        if (consecutive_time_sync_failures < UINT8_MAX) {
            consecutive_time_sync_failures++;
        }
    }
    last_time_sync_error = synchronized ? RMW_RET_OK : (result == RMW_RET_OK ? RMW_RET_ERROR : result);

    if (!mutex_owned) {
        (void)osMutexRelease(session_mutex);
    }

    if (synchronized) {
        LogDebug("micro-ROS Client: Time synchronized with agent");
    } else if (consecutive_time_sync_failures <= 1U ||
               consecutive_time_sync_failures == ROS_TIME_SYNC_FAILURE_RECONNECT_THRESHOLD) {
        LogWarning("micro-ROS Client: Time synchronization with agent failed: %d", (int)last_time_sync_error);
    } else {
        LogDebug("micro-ROS Client: Time synchronization with agent failed: %d", (int)last_time_sync_error);
    }
    return synchronized;
}

void Client::clearSynchronizedTime() {
    taskENTER_CRITICAL();
    time_synchronized = false;
    synchronized_epoch_ns = 0;
    synchronized_monotonic_ns = 0;
    taskEXIT_CRITICAL();
}

void Client::publishConnectionState(ConnectionState new_state) {
    connection_state = new_state;
    trace::setConnectionState(new_state);
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

rcl_ret_t Client::initEntities() {
    // Entity creation is ordered from parent to child: nodes first, then the
    // entities that depend on them. Public init() only registers wrappers, so
    // this function can safely recreate everything after every reconnect.
    for (size_t i = 0; i < node_count; i++) {
        const rcl_ret_t result = nodes[i]->initRclcNode(&support);
        if (result != RCL_RET_OK) {
            return result;
        }
    }
    for (size_t i = 0; i < publisher_count; i++) {
        const rcl_ret_t result = publishers[i]->initRclcPublisher();
        if (result != RCL_RET_OK) {
            return result;
        }
    }
    for (size_t i = 0; i < subscription_count; i++) {
        const rcl_ret_t result = subscriptions[i]->initRclcSubscriber();
        if (result != RCL_RET_OK) {
            return result;
        }
    }
    for (size_t i = 0; i < service_count; i++) {
        const rcl_ret_t result = services[i]->initRclcService();
        if (result != RCL_RET_OK) {
            return result;
        }
    }
    return RCL_RET_OK;
}

rcl_ret_t Client::finiEntities() {
    rcl_ret_t result = RCL_RET_OK;

    // Destruction runs in reverse dependency order. Executor-backed entities
    // are removed from the executor before their rcl handles are finalized.
    for (size_t i = service_count; i > 0U; i--) {
        const rcl_ret_t fini_result = services[i - 1U]->finiRclcService();
        if (result == RCL_RET_OK) {
            result = fini_result;
        }
    }
    for (size_t i = subscription_count; i > 0U; i--) {
        const rcl_ret_t fini_result = subscriptions[i - 1U]->finiRclcSubscriber();
        if (result == RCL_RET_OK) {
            result = fini_result;
        }
    }
    for (size_t i = publisher_count; i > 0U; i--) {
        const rcl_ret_t fini_result = publishers[i - 1U]->finiRclcPublisher();
        if (result == RCL_RET_OK) {
            result = fini_result;
        }
    }
    for (size_t i = node_count; i > 0U; i--) {
        const rcl_ret_t fini_result = nodes[i - 1U]->finiRclcNode();
        if (result == RCL_RET_OK) {
            result = fini_result;
        }
    }

    return result;
}

rcl_ret_t Client::registerNode(Node* node) {
    if (node == nullptr || session_mutex == nullptr) {
        return RCL_RET_INVALID_ARGUMENT;
    }
    rcl_ret_t result = lockSession(osWaitForever);
    if (result != RCL_RET_OK) {
        return result;
    }
    // Registries are fixed-size arrays on purpose: no allocation is performed
    // when application code registers entities before the agent is connected.
    for (size_t i = 0; i < node_count; i++) {
        if (nodes[i] == node) {
            unlockSession();
            return RCL_RET_OK;
        }
    }
    if (node_count >= nodes.size()) {
        result = RCL_RET_ERROR;
    } else {
        nodes[node_count++] = node;
        trace::setRegisteredNodeCount(node_count);
    }
    unlockSession();
    return result;
}

rcl_ret_t Client::unregisterNode(Node* node) {
    if (node == nullptr || session_mutex == nullptr) {
        return RCL_RET_INVALID_ARGUMENT;
    }
    rcl_ret_t result = lockSession(osWaitForever);
    if (result != RCL_RET_OK) {
        return result;
    }
    for (size_t i = 0; i < node_count; i++) {
        if (nodes[i] == node) {
            for (size_t j = i; j + 1U < node_count; j++) {
                nodes[j] = nodes[j + 1U];
            }
            nodes[--node_count] = nullptr;
            trace::setRegisteredNodeCount(node_count);
            unlockSession();
            return RCL_RET_OK;
        }
    }
    unlockSession();
    return RCL_RET_ERROR;
}

rcl_ret_t Client::registerPublisher(BasePublisher* publisher) {
    if (publisher == nullptr || session_mutex == nullptr) {
        return RCL_RET_INVALID_ARGUMENT;
    }
    rcl_ret_t result = lockSession(osWaitForever);
    if (result != RCL_RET_OK) {
        return result;
    }
    for (size_t i = 0; i < publisher_count; i++) {
        if (publishers[i] == publisher) {
            unlockSession();
            return RCL_RET_OK;
        }
    }
    if (publisher_count >= publishers.size()) {
        result = RCL_RET_ERROR;
    } else {
        publishers[publisher_count++] = publisher;
    }
    unlockSession();
    return result;
}

rcl_ret_t Client::unregisterPublisher(BasePublisher* publisher) {
    if (publisher == nullptr || session_mutex == nullptr) {
        return RCL_RET_INVALID_ARGUMENT;
    }
    rcl_ret_t result = lockSession(osWaitForever);
    if (result != RCL_RET_OK) {
        return result;
    }
    for (size_t i = 0; i < publisher_count; i++) {
        if (publishers[i] == publisher) {
            for (size_t j = i; j + 1U < publisher_count; j++) {
                publishers[j] = publishers[j + 1U];
            }
            publishers[--publisher_count] = nullptr;
            unlockSession();
            return RCL_RET_OK;
        }
    }
    unlockSession();
    return RCL_RET_ERROR;
}

rcl_ret_t Client::registerSubscriber(BaseSubscriber* subscriber) {
    if (subscriber == nullptr || session_mutex == nullptr) {
        return RCL_RET_INVALID_ARGUMENT;
    }
    rcl_ret_t result = lockSession(osWaitForever);
    if (result != RCL_RET_OK) {
        return result;
    }
    for (size_t i = 0; i < subscription_count; i++) {
        if (subscriptions[i] == subscriber) {
            unlockSession();
            return RCL_RET_OK;
        }
    }
    if (subscription_count >= subscriptions.size()) {
        result = RCL_RET_ERROR;
    } else {
        subscriptions[subscription_count++] = subscriber;
    }
    unlockSession();
    return result;
}

rcl_ret_t Client::unregisterSubscriber(BaseSubscriber* subscriber) {
    if (subscriber == nullptr || session_mutex == nullptr) {
        return RCL_RET_INVALID_ARGUMENT;
    }
    rcl_ret_t result = lockSession(osWaitForever);
    if (result != RCL_RET_OK) {
        return result;
    }
    for (size_t i = 0; i < subscription_count; i++) {
        if (subscriptions[i] == subscriber) {
            for (size_t j = i; j + 1U < subscription_count; j++) {
                subscriptions[j] = subscriptions[j + 1U];
            }
            subscriptions[--subscription_count] = nullptr;
            unlockSession();
            return RCL_RET_OK;
        }
    }
    unlockSession();
    return RCL_RET_ERROR;
}

rcl_ret_t Client::registerService(BaseService* service) {
    if (service == nullptr || session_mutex == nullptr) {
        return RCL_RET_INVALID_ARGUMENT;
    }
    rcl_ret_t result = lockSession(osWaitForever);
    if (result != RCL_RET_OK) {
        return result;
    }
    for (size_t i = 0; i < service_count; i++) {
        if (services[i] == service) {
            unlockSession();
            return RCL_RET_OK;
        }
    }
    if (service_count >= services.size()) {
        result = RCL_RET_ERROR;
    } else {
        services[service_count++] = service;
    }
    unlockSession();
    return result;
}

rcl_ret_t Client::unregisterService(BaseService* service) {
    if (service == nullptr || session_mutex == nullptr) {
        return RCL_RET_INVALID_ARGUMENT;
    }
    rcl_ret_t result = lockSession(osWaitForever);
    if (result != RCL_RET_OK) {
        return result;
    }
    for (size_t i = 0; i < service_count; i++) {
        if (services[i] == service) {
            for (size_t j = i; j + 1U < service_count; j++) {
                services[j] = services[j + 1U];
            }
            services[--service_count] = nullptr;
            unlockSession();
            return RCL_RET_OK;
        }
    }
    unlockSession();
    return RCL_RET_ERROR;
}

rcl_ret_t Client::lockSession(uint32_t timeout_ms) {
    if (session_mutex == nullptr) {
        return RCL_RET_NOT_INIT;
    }
    return osMutexAcquire(session_mutex, timeout_ms) == osOK ? RCL_RET_OK : RCL_RET_TIMEOUT;
}

void Client::unlockSession() {
    if (session_mutex != nullptr) {
        (void)osMutexRelease(session_mutex);
    }
}

bool Client::validateConfig(const Config& client_config) const {
    const Transport& transport = client_config.transport;
    return transport.context != nullptr && transport.open != nullptr && transport.close != nullptr &&
           transport.write != nullptr && transport.read != nullptr && client_config.base_namespace != nullptr &&
           client_config.connection_retry_interval_ms > 0U && client_config.connection_health_interval_ms > 0U &&
           client_config.ping_timeout_ms > 0 && client_config.ping_attempts > 0U;
}

void Client::cleanupInitFailure(rcl_ret_t error) {
    // Preserve the originating error so getLastError() agrees with init().
    last_error = error;
    state = State::ERROR;
    trace::setClientState(static_cast<size_t>(state));
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
