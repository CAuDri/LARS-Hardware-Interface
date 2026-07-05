/**
 * @file client.hpp
 *
 * @brief CAuDri - micro-ROS client configuration and session lifecycle
 */
#pragma once

#include <builtin_interfaces/msg/time.h>
#include <cmsis_os2.h>
#include <rclc/rclc.h>
#include <rmw_microros/custom_transport.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "FreeRTOS.h"
#include "state.hpp"
#include "event_groups.h"
#include "executor.hpp"
#include "semphr.h"
#include "task.h"

constexpr uint32_t ROS_CLIENT_THREAD_STACK_SIZE = 4096;
constexpr uint32_t ROS_CONNECTION_RETRY_INTERVAL_MS = 1000;
constexpr uint32_t ROS_CONNECTION_HEALTH_INTERVAL_MS = 1000;
constexpr int ROS_AGENT_PING_TIMEOUT_MS = 50;
constexpr uint8_t ROS_AGENT_PING_ATTEMPTS = 1;
constexpr uint32_t ROS_INITIAL_TIME_SYNC_RETRY_INTERVAL_MS = 1000;
constexpr uint32_t ROS_TIME_SYNC_INTERVAL_MS = 30000;
constexpr int ROS_TIME_SYNC_TIMEOUT_MS = 250;
constexpr uint8_t ROS_TIME_SYNC_FAILURE_RECONNECT_THRESHOLD = 3;

constexpr uint32_t ROS_CONNECTION_ESTABLISHED_FLAG = 0x01U;
constexpr uint32_t ROS_CONNECTION_LOST_FLAG = 0x02U;
constexpr uint32_t ROS_TEST_CONNECTION_FLAG = 0x04U;
constexpr uint32_t ROS_STOP_CLIENT_FLAG = 0x08U;
constexpr uint32_t ROS_CLIENT_STOPPED_FLAG = 0x10U;

constexpr size_t ROS_MAX_NODES = 15;
constexpr size_t ROS_MAX_PUBLISHERS = 40;
constexpr size_t ROS_MAX_SUBSCRIPTIONS = 30;
constexpr size_t ROS_MAX_SERVICES = 1;
constexpr size_t ROS_MAX_SERVICE_CLIENTS = 1;
constexpr size_t ROS_EXECUTOR_HANDLE_CAPACITY = ROS_MAX_SUBSCRIPTIONS + ROS_MAX_SERVICES + ROS_MAX_SERVICE_CLIENTS;

static_assert(ROS_MAX_NODES <= RMW_UXRCE_MAX_NODES);
static_assert(ROS_MAX_PUBLISHERS <= RMW_UXRCE_MAX_PUBLISHERS);
static_assert(ROS_MAX_SUBSCRIPTIONS <= RMW_UXRCE_MAX_SUBSCRIPTIONS);
static_assert(ROS_MAX_SERVICES <= RMW_UXRCE_MAX_SERVICES);
static_assert(ROS_MAX_SERVICE_CLIENTS <= RMW_UXRCE_MAX_CLIENTS);

namespace ros {

class Node;
class BasePublisher;
class BaseSubscriber;
class BaseService;
class BaseServiceClient;

/**
 * @brief Owns the micro-ROS support/session lifecycle and executor
 */
class Client {
   public:
    enum class State { ERROR, UNINITIALIZED, INITIALIZED, CONNECTING, CONNECTED, DISCONNECTED, STOPPING, STOPPED };

    /**
     * @brief Non-owning custom transport configuration
     *
     * Callback signatures are the Jazzy micro-ROS custom transport callbacks.
     * The callback context and everything it references must outlive the
     * client.
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
     *
     * @param transport Non-owning custom transport configuration
     * @param client_thread_priority Priority of the connection thread (default: osPriorityNormal1)
     * @param executor_thread_priority Priority of the executor thread (default: osPriorityRealtime)
     * @param connection_retry_interval_ms Interval between connection attempts (default: 1000 ms)
     * @param connection_health_interval_ms Connected-state wake interval for event and time-sync checks (default: 1000 ms)
     * @param ping_timeout_ms Timeout for pinging the agent (default: 50 ms)
     * @param ping_attempts Number of ping attempts before considering the agent unavailable (default: 1)
     * @param base_namespace Optional namespace prefix joined with every node namespace
     */
    struct Config {
        Transport transport{};
        const char* base_namespace = "";
        osPriority_t client_thread_priority = osPriorityNormal1;
        osPriority_t executor_thread_priority = osPriorityRealtime;
        uint32_t connection_retry_interval_ms = ROS_CONNECTION_RETRY_INTERVAL_MS;
        uint32_t connection_health_interval_ms = ROS_CONNECTION_HEALTH_INTERVAL_MS;
        int ping_timeout_ms = ROS_AGENT_PING_TIMEOUT_MS;
        uint8_t ping_attempts = ROS_AGENT_PING_ATTEMPTS;
    };

    Client();
    ~Client() = default;
    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;

    rcl_ret_t init(const Config& client_config);
    rcl_ret_t fini(uint32_t timeout_ms = osWaitForever);

    State getState() const;
    ConnectionState getConnectionState() const;
    rcl_ret_t getLastError() const;

    bool isConnected() const;
    bool isTimeSynchronized() const;
    rmw_ret_t getLastTimeSyncError() const;
    builtin_interfaces__msg__Time getRosTime() const;
    bool waitForConnection(uint32_t timeout_ms = 0) const;
    bool waitForDisconnect(uint32_t timeout_ms = 0) const;
    void signalPossibleDisconnect(rcl_ret_t error);

    Executor& getExecutor();
    const Executor& getExecutor() const;
    const char* getBaseNamespace() const;

   private:
    friend class Executor;
    friend class Node;
    friend class BasePublisher;
    friend class BaseSubscriber;
    friend class BaseService;
    friend class BaseServiceClient;

    static Client* instance;

    const Config* config = nullptr;
    volatile State state = State::UNINITIALIZED;
    volatile ConnectionState connection_state = ConnectionState::UNKNOWN;
    volatile bool stop_requested = false;
    rcl_ret_t last_error = RCL_RET_OK;

    volatile bool time_synchronized = false;
    rmw_ret_t last_time_sync_error = RMW_RET_ERROR;
    int64_t synchronized_epoch_ns = 0;
    int64_t synchronized_monotonic_ns = 0;
    uint32_t last_time_sync_attempt_ms = 0;
    uint8_t consecutive_time_sync_failures = 0;

    rcl_allocator_t allocator{};
    rclc_support_t support{};
    bool support_active = false;

    Executor executor{};

    osMutexId_t session_mutex = nullptr;
    osMutexAttr_t mutex_attributes{};
    StaticSemaphore_t mutex_control_block{};

    osEventFlagsId_t connection_events = nullptr;
    osEventFlagsAttr_t event_attributes{};
    StaticEventGroup_t event_control_block{};

    osThreadId_t thread_id = nullptr;
    osThreadAttr_t thread_attributes{};
    StaticTask_t thread_control_block{};
    std::array<uint32_t, ROS_CLIENT_THREAD_STACK_SIZE / sizeof(uint32_t)> thread_stack{};

    std::array<Node*, ROS_MAX_NODES> nodes{};
    std::array<BasePublisher*, ROS_MAX_PUBLISHERS> publishers{};
    std::array<BaseSubscriber*, ROS_MAX_SUBSCRIPTIONS> subscriptions{};
    std::array<BaseService*, ROS_MAX_SERVICES> services{};
    std::array<BaseServiceClient*, ROS_MAX_SERVICE_CLIENTS> service_clients{};
    size_t node_count = 0;
    size_t publisher_count = 0;
    size_t subscription_count = 0;
    size_t service_count = 0;
    size_t service_client_count = 0;

    static void executorError(void* context, rcl_ret_t error);
    static int64_t getMonotonicTimeNs();
    void thread();
    rcl_ret_t connectSession();
    rcl_ret_t disconnectSession(bool agent_available);
    bool pingAgent();
    bool synchronizeTime();
    void clearSynchronizedTime();
    void publishConnectionState(ConnectionState new_state);

    rcl_ret_t initEntities();
    rcl_ret_t finiEntities();
    rcl_ret_t registerNode(Node* node);
    rcl_ret_t unregisterNode(Node* node);
    rcl_ret_t registerPublisher(BasePublisher* publisher);
    rcl_ret_t unregisterPublisher(BasePublisher* publisher);
    rcl_ret_t registerSubscriber(BaseSubscriber* subscriber);
    rcl_ret_t unregisterSubscriber(BaseSubscriber* subscriber);

    rcl_ret_t lockSession(uint32_t timeout_ms);
    void unlockSession();
    bool validateConfig(const Config& client_config) const;
    void cleanupInitFailure(rcl_ret_t error);
};

}  // namespace ros
