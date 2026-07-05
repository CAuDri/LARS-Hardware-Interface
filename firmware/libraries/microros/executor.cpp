/**
 * @file executor.cpp
 *
 * @brief CAuDri - Static RTOS executor lifecycle for micro-ROS
 */

#include "executor.hpp"

#include "client.hpp"
#include "logger.h"

namespace ros {

/**
 * @brief Construct an executor without creating rclc or RTOS resources.
 */
Executor::Executor() { rclc_executor = rclc_executor_get_zero_initialized_executor(); }

/**
 * @brief Get the executor lifecycle state.
 * @return Current executor lifecycle state.
 */
Executor::State Executor::getState() const { return state; }

/**
 * @brief Get the most recent rclc executor error.
 * @return Most recent rclc executor error.
 */
rcl_ret_t Executor::getLastError() const { return last_error; }

rcl_ret_t Executor::createThread(osPriority_t priority, osMutexId_t mutex, ErrorCallback callback, void* callback_context) {
    if (thread_id != nullptr) {
        return RCL_RET_ALREADY_INIT;
    }
    if (mutex == nullptr || callback == nullptr) {
        return RCL_RET_INVALID_ARGUMENT;
    }

    session_mutex = mutex;
    error_callback = callback;
    error_context = callback_context;

    // The stopped flag acknowledges that no spin operation can still be using
    // rclc entities. Client waits for it before tearing down a session.
    event_attributes = {
        .name = "ROS Executor State",
        .attr_bits = 0,
        .cb_mem = &event_control_block,
        .cb_size = sizeof(event_control_block),
    };
    state_events = osEventFlagsNew(&event_attributes);
    if (state_events == nullptr) {
        return RCL_RET_ERROR;
    }

    thread_attributes = {
        .name = "ROS Executor",
        .attr_bits = osThreadDetached,
        .cb_mem = &thread_control_block,
        .cb_size = sizeof(thread_control_block),
        .stack_mem = thread_stack.data(),
        .stack_size = sizeof(thread_stack),
        .priority = priority,
        .tz_module = 0,
        .reserved = 0,
    };
    thread_id = osThreadNew(
        // Pass this object through the CMSIS user argument so the C callback
        // can execute the private C++ member function.
        [](void* argument) -> void { static_cast<Executor*>(argument)->thread(); },
        this,
        &thread_attributes);
    if (thread_id == nullptr) {
        (void)osEventFlagsDelete(state_events);
        state_events = nullptr;
        return RCL_RET_ERROR;
    }

    return RCL_RET_OK;
}

rcl_ret_t Executor::destroyThread() {
    spin_requested = false;
    if (thread_id != nullptr) {
        (void)osThreadTerminate(thread_id);
        thread_id = nullptr;
    }
    if (state_events != nullptr) {
        (void)osEventFlagsDelete(state_events);
        state_events = nullptr;
    }
    session_mutex = nullptr;
    error_callback = nullptr;
    error_context = nullptr;
    return RCL_RET_OK;
}

rcl_ret_t Executor::initRclcExecutor(rcl_context_t* context, const rcl_allocator_t* allocator) {
    if (context == nullptr || allocator == nullptr) {
        return RCL_RET_INVALID_ARGUMENT;
    }
    if (state != State::UNINITIALIZED && state != State::STOPPED) {
        return RCL_RET_ALREADY_INIT;
    }

    // rclc storage is recreated for every session while the containing C++
    // object and its RTOS thread remain alive across reconnects.
    rclc_executor = rclc_executor_get_zero_initialized_executor();
    rcl_ret_t result = rclc_executor_init(&rclc_executor, context, ROS_EXECUTOR_HANDLE_CAPACITY, allocator);
    if (result == RCL_RET_OK) {
        result = rclc_executor_set_semantics(&rclc_executor, RCLC_SEMANTICS_RCLCPP_EXECUTOR);
    }
    if (result != RCL_RET_OK) {
        if (rclc_executor.type != RCLC_EXECUTOR_NOT_INITIALIZED) {
            (void)rclc_executor_fini(&rclc_executor);
        }
        rclc_executor = rclc_executor_get_zero_initialized_executor();
        setError(result);
        LogError("micro-ROS Executor: rclc initialization failed: %d", (int)result);
        return result;
    }

    last_error = RCL_RET_OK;
    state = State::INITIALIZED;
    LogDebug("micro-ROS Executor: rclc executor initialized with %lu handles", (uint32_t)ROS_EXECUTOR_HANDLE_CAPACITY);
    return RCL_RET_OK;
}

rcl_ret_t Executor::prepare() {
    if (state != State::INITIALIZED) {
        return RCL_RET_NOT_INIT;
    }

    // Explicit preparation allocates the wait set before the real-time spin
    // loop starts, keeping allocation out of normal callback dispatch.
    const rcl_ret_t result = rclc_executor_prepare(&rclc_executor);
    if (result != RCL_RET_OK) {
        setError(result);
        LogError("micro-ROS Executor: Failed to prepare wait set: %d", (int)result);
    }
    return result;
}

rcl_ret_t Executor::startSpinning() {
    if (thread_id == nullptr || state != State::INITIALIZED) {
        return RCL_RET_NOT_INIT;
    }

    (void)osEventFlagsClear(state_events, ROS_EXECUTOR_STOPPED_FLAG);
    spin_requested = true;
    state = State::SPINNING;
    const uint32_t flags = osThreadFlagsSet(thread_id, ROS_EXECUTOR_WAKE_FLAG);
    if ((flags & osFlagsError) != 0U) {
        spin_requested = false;
        state = State::INITIALIZED;
        LogError("micro-ROS Executor: Failed to wake executor thread");
        return RCL_RET_ERROR;
    }
    return RCL_RET_OK;
}

void Executor::requestStop() {
    spin_requested = false;
    if (thread_id != nullptr) {
        (void)osThreadFlagsSet(thread_id, ROS_EXECUTOR_WAKE_FLAG);
    }
}

rcl_ret_t Executor::waitForStop(uint32_t timeout_ms) {
    if (state != State::SPINNING) {
        return RCL_RET_OK;
    }

    const uint32_t flags = osEventFlagsWait(state_events, ROS_EXECUTOR_STOPPED_FLAG, osFlagsWaitAny, timeout_ms);
    return (flags & osFlagsError) == 0U && (flags & ROS_EXECUTOR_STOPPED_FLAG) != 0U ? RCL_RET_OK : RCL_RET_TIMEOUT;
}

rcl_ret_t Executor::finiRclcExecutor() {
    if (state == State::UNINITIALIZED || state == State::STOPPED) {
        return RCL_RET_OK;
    }
    if (state == State::SPINNING) {
        return RCL_RET_ERROR;
    }

    const rcl_ret_t result =
        rclc_executor.type == RCLC_EXECUTOR_NOT_INITIALIZED ? RCL_RET_OK : rclc_executor_fini(&rclc_executor);
    rclc_executor = rclc_executor_get_zero_initialized_executor();
    last_error = result;
    state = result == RCL_RET_OK ? State::STOPPED : State::ERROR;
    if (result != RCL_RET_OK) {
        LogWarning("micro-ROS Executor: rclc cleanup returned: %d", (int)result);
    }
    return result;
}

rcl_ret_t Executor::addSubscription(rcl_subscription_t* subscription,
                                    void* message,
                                    rclc_subscription_callback_with_context_t callback,
                                    void* context,
                                    rclc_executor_handle_invocation_t invocation) {
    if (state != State::INITIALIZED || subscription == nullptr || message == nullptr || callback == nullptr) {
        return RCL_RET_INVALID_ARGUMENT;
    }

    const rcl_ret_t result =
        rclc_executor_add_subscription_with_context(&rclc_executor, subscription, message, callback, context, invocation);
    if (result != RCL_RET_OK) {
        last_error = result;
    }
    return result;
}

rcl_ret_t Executor::removeSubscription(const rcl_subscription_t* subscription) {
    if (state == State::UNINITIALIZED || state == State::STOPPED || subscription == nullptr) {
        return RCL_RET_OK;
    }

    const rcl_ret_t result = rclc_executor_remove_subscription(&rclc_executor, subscription);
    if (result != RCL_RET_OK) {
        last_error = result;
    }
    return result;
}

void Executor::thread() {
    while (true) {
        if (!spin_requested) {
            // The RTOS thread persists between sessions. Only the rclc
            // executor is destroyed and recreated during a reconnect.
            state = state == State::SPINNING ? State::INITIALIZED : state;
            (void)osEventFlagsSet(state_events, ROS_EXECUTOR_STOPPED_FLAG);
            (void)osThreadFlagsWait(ROS_EXECUTOR_WAKE_FLAG, osFlagsWaitAny, osWaitForever);
            continue;
        }

        if (osMutexAcquire(session_mutex, osWaitForever) != osOK) {
            setError(RCL_RET_ERROR);
            LogError("micro-ROS Executor: Failed to lock the session mutex");
            error_callback(error_context, RCL_RET_ERROR);
            continue;
        }

        // Hold the session mutex only across the rclc spin call. The short
        // configured timeout gives the connection thread regular opportunities
        // to synchronize time, initialize, or destroy the session.
        rcl_ret_t result = RCL_RET_OK;
        if (spin_requested) {
            result = rclc_executor_spin_some(&rclc_executor, ROS_EXECUTOR_SPIN_TIMEOUT_NS);
        }
        (void)osMutexRelease(session_mutex);

        if (result != RCL_RET_OK && result != RCL_RET_TIMEOUT && result != RCL_RET_WAIT_SET_EMPTY) {
            last_error = result;
            LogWarning("micro-ROS Executor: Spin failed: %d", (int)result);
            error_callback(error_context, result);
        }
        osDelay(ROS_EXECUTOR_SPIN_PERIOD_MS);
    }
}

void Executor::setError(rcl_ret_t error) {
    last_error = error;
    state = State::ERROR;
    spin_requested = false;
}

}  // namespace ros
