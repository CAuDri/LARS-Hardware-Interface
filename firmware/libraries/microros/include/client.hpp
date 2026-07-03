/**
 * @file client.hpp
 *
 * @brief CAuDri - micro-ROS client configuration and session lifecycle
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include <cmsis_os2.h>
#include <rmw_microros/custom_transport.h>

constexpr uint32_t ROS_CLIENT_THREAD_STACK_SIZE = 4096;
constexpr uint32_t ROS_EXECUTOR_THREAD_STACK_SIZE = 8192;

constexpr size_t ROS_MAX_NODES = 15;
constexpr size_t ROS_MAX_PUBLISHERS = 40;
constexpr size_t ROS_MAX_SUBSCRIPTIONS = 30;
constexpr size_t ROS_MAX_SERVICES = 1;
constexpr size_t ROS_MAX_SERVICE_CLIENTS = 1;
constexpr size_t ROS_EXECUTOR_HANDLE_CAPACITY =
    ROS_MAX_SUBSCRIPTIONS + ROS_MAX_SERVICES + ROS_MAX_SERVICE_CLIENTS;

static_assert(ROS_MAX_NODES <= RMW_UXRCE_MAX_NODES);
static_assert(ROS_MAX_PUBLISHERS <= RMW_UXRCE_MAX_PUBLISHERS);
static_assert(ROS_MAX_SUBSCRIPTIONS <= RMW_UXRCE_MAX_SUBSCRIPTIONS);
static_assert(ROS_MAX_SERVICES <= RMW_UXRCE_MAX_SERVICES);
static_assert(ROS_MAX_SERVICE_CLIENTS <= RMW_UXRCE_MAX_CLIENTS);

namespace ros {

/**
 * @brief Owns the micro-ROS support/session lifecycle and executor
 */
class Client {
   public:
    /**
     * @brief Non-owning custom transport configuration
     *
     * Callback signatures are the native Jazzy micro-ROS custom transport
     * signatures. The callback context and everything it references must
     * outlive the client.
     */
    struct Transport {
        bool framing = true;
        void* context = nullptr;
        open_custom_func open = nullptr;
        close_custom_func close = nullptr;
        write_custom_func write = nullptr;
        read_custom_func read = nullptr;
    };

    /**
     * @brief Device-specific client configuration
     */
    struct Config {
        Transport transport{};
        osPriority_t client_task_priority = osPriorityNormal1;
        osPriority_t executor_task_priority = osPriorityRealtime;
    };
};

}  // namespace ros
