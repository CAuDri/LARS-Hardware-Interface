/**
 * @file executor.hpp
 *
 * @brief CAuDri - Static RTOS executor for micro-ROS callback dispatch
 */
#pragma once

#include <cmsis_os2.h>
#include <rclc/executor.h>

#include <array>
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
class BaseSubscriber;
class BaseService;

/**
 * @brief Owns the rclc executor and its persistent RTOS thread
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
    friend class BaseSubscriber;
    friend class BaseService;

    using ErrorCallback = void (*)(void* context, rcl_ret_t error);

    rclc_executor_t rclc_executor{};
    volatile State state = State::UNINITIALIZED;
    volatile bool spin_requested = false;
    rcl_ret_t last_error = RCL_RET_OK;

    osMutexId_t session_mutex = nullptr;
    ErrorCallback error_callback = nullptr;
    void* error_context = nullptr;

    osThreadId_t thread_id = nullptr;
    osThreadAttr_t thread_attributes{};
    StaticTask_t thread_control_block{};
    std::array<uint32_t, ROS_EXECUTOR_THREAD_STACK_SIZE / sizeof(uint32_t)> thread_stack{};

    osEventFlagsId_t state_events = nullptr;
    osEventFlagsAttr_t event_attributes{};
    StaticEventGroup_t event_control_block{};

    rcl_ret_t createThread(osPriority_t priority, osMutexId_t mutex, ErrorCallback callback, void* callback_context);
    rcl_ret_t destroyThread();
    rcl_ret_t initRclcExecutor(rcl_context_t* context, const rcl_allocator_t* allocator);
    rcl_ret_t prepare();
    rcl_ret_t startSpinning();
    void requestStop();
    rcl_ret_t waitForStop(uint32_t timeout_ms);
    rcl_ret_t finiRclcExecutor();
    rcl_ret_t addSubscription(rcl_subscription_t* subscription,
                              void* message,
                              rclc_subscription_callback_with_context_t callback,
                              void* context,
                              rclc_executor_handle_invocation_t invocation);
    rcl_ret_t removeSubscription(const rcl_subscription_t* subscription);
    rcl_ret_t addService(rcl_service_t* service,
                         void* request,
                         void* response,
                         rclc_service_callback_with_context_t callback,
                         void* context);
    rcl_ret_t removeService(const rcl_service_t* service);

    void thread();
    void setError(rcl_ret_t error);
};

}  // namespace ros
