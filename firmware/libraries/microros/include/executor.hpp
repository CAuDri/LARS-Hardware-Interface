/**
 * @file executor.hpp
 *
 * @brief CAuDri - Static RTOS executor for micro-ROS callback dispatch
 */
#pragma once

#include <cmsis_os2.h>
#include <rclc/executor.h>

#include <cstdint>

#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"

constexpr uint32_t ROS_EXECUTOR_SPIN_PERIOD_MS = 1;
constexpr uint64_t ROS_EXECUTOR_SPIN_TIMEOUT_NS = 10000;
constexpr uint32_t ROS_EXECUTOR_THREAD_STACK_SIZE = 8192;
constexpr uint32_t ROS_EXECUTOR_WAKE_FLAG = 0x01U;
constexpr uint32_t ROS_EXECUTOR_STOPPED_FLAG = 0x01U;

namespace ros {

class Client;

/**
 * @brief Owns the native rclc executor and its persistent RTOS thread
 */
class Executor {
   public:
    enum class State { ERROR, UNINITIALIZED, INITIALIZED, SPINNING, STOPPED };

    Executor();
    Executor(const Executor&) = delete;
    Executor& operator=(const Executor&) = delete;

    State getState() const;
    rcl_ret_t getLastError() const;

   private:
    friend class Client;

    using ErrorCallback = void (*)(void* context, rcl_ret_t error);

    rclc_executor_t native_executor{};
    volatile State state = State::UNINITIALIZED;
    volatile bool spin_requested = false;
    rcl_ret_t last_error = RCL_RET_OK;

    osMutexId_t session_mutex = nullptr;
    ErrorCallback error_callback = nullptr;
    void* error_context = nullptr;

    osThreadId_t thread_id = nullptr;
    osThreadAttr_t thread_attributes{};
    StaticTask_t thread_control_block{};
    uint32_t thread_stack[ROS_EXECUTOR_THREAD_STACK_SIZE / sizeof(uint32_t)]{};

    osEventFlagsId_t state_events = nullptr;
    osEventFlagsAttr_t event_attributes{};
    StaticEventGroup_t event_control_block{};

    rcl_ret_t createThread(osPriority_t priority, osMutexId_t mutex, ErrorCallback callback, void* callback_context);
    rcl_ret_t destroyThread();
    rcl_ret_t nativeInit(rcl_context_t* context, const rcl_allocator_t* allocator);
    rcl_ret_t prepare();
    rcl_ret_t startSpinning();
    void requestStop();
    rcl_ret_t waitForStop(uint32_t timeout_ms);
    rcl_ret_t nativeFini();

    void thread();
    void setError(rcl_ret_t error);
};

}  // namespace ros
