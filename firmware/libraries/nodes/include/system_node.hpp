/**
 * @file system_node.hpp
 *
 * @brief CAuDri - micro-ROS node for hardware operations and heartbeat
 */
#pragma once

#include <cmsis_os2.h>
#include <std_msgs/msg/empty.h>
#include <std_srvs/srv/trigger.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "client.hpp"
#include "drive_controller.hpp"
#include "node.hpp"
#include "publisher.hpp"
#include "diagnostics.hpp"
#include "service.hpp"
#include "system_monitor.hpp"
#include "type_support.hpp"

ROS_DECLARE_SERVICE_TYPE(std_srvs, Trigger);

using ros::BasePublisher;
using ros::BaseService;
using ros::Client;
using ros::EntityState;
using ros::Node;
using ros::Publisher;
using ros::Service;
using ros::service_types::std_srvs_Trigger;

constexpr const char* SYSTEM_NODE_HEARTBEAT_TOPIC = "heartbeat";
constexpr const char* SYSTEM_NODE_RESET_SERVICE = "reset";
constexpr const char* SYSTEM_NODE_EMERGENCY_STOP_SERVICE = "emergency_stop";
constexpr uint32_t SYSTEM_NODE_HEARTBEAT_PERIOD_MS = 1000;
constexpr const char* SYSTEM_NODE_DIAGNOSTICS_TOPIC = "/diagnostics";
constexpr uint32_t SYSTEM_NODE_DIAGNOSTICS_PERIOD_MS = 250;
constexpr uint32_t SYSTEM_NODE_RESET_DELAY_MS = 500;
constexpr uint32_t SYSTEM_NODE_THREAD_STACK_SIZE = 1536;
constexpr size_t SYSTEM_NODE_RESPONSE_BUFFER_SIZE = 96;

/**
 * @brief Exposes board-level ROS operations.
 *
 * The node keeps safety-sensitive service callbacks short. Emergency stop is
 * forwarded to DriveController immediately. Hardware reset is acknowledged and
 * then performed by the node thread after a short delay so the service response
 * can leave the board before the MCU resets.
 */
class SystemNode : public Node {
   public:
    struct Config {
        const char* heartbeat_topic = SYSTEM_NODE_HEARTBEAT_TOPIC;
        const char* diagnostics_topic = SYSTEM_NODE_DIAGNOSTICS_TOPIC;
        const char* reset_service = SYSTEM_NODE_RESET_SERVICE;
        const char* emergency_stop_service = SYSTEM_NODE_EMERGENCY_STOP_SERVICE;

        uint32_t heartbeat_period_ms = SYSTEM_NODE_HEARTBEAT_PERIOD_MS;
        uint32_t diagnostics_period_ms = SYSTEM_NODE_DIAGNOSTICS_PERIOD_MS;
        uint32_t reset_delay_ms = SYSTEM_NODE_RESET_DELAY_MS;

        osPriority_t thread_priority = osPriorityLow;
        BasePublisher::Config heartbeat_publisher_config{true, 0};
        BasePublisher::Config diagnostics_publisher_config{true, 0};
        BaseService::Config service_config{};
    };

    SystemNode();
    ~SystemNode() = default;
    SystemNode(const SystemNode&) = delete;
    SystemNode& operator=(const SystemNode&) = delete;

    rcl_ret_t init(Client& client,
                   DriveController& drive_controller,
                   SystemMonitor& system_monitor,
                   const Config& config);
    rcl_ret_t start();

   private:
    Client* client = nullptr;
    DriveController* drive_controller = nullptr;
    SystemMonitor* system_monitor = nullptr;
    Config config{};
    bool started = false;

    Publisher<std_msgs__msg__Empty> heartbeat_publisher{};
    std_msgs__msg__Empty heartbeat_message{};
    ros::diagnostics::DiagnosticPublisher diagnostics_publisher{};

    Service<std_srvs_Trigger, SystemNode> reset_service{};
    Service<std_srvs_Trigger, SystemNode> emergency_stop_service{};
    std::array<char, SYSTEM_NODE_RESPONSE_BUFFER_SIZE> reset_response_buffer{};
    std::array<char, SYSTEM_NODE_RESPONSE_BUFFER_SIZE> emergency_stop_response_buffer{};

    volatile bool reset_pending = false;
    volatile uint32_t reset_request_time_ms = 0;
    bool heartbeat_publish_failure_reported = false;
    bool diagnostics_publish_failure_reported = false;

    osThreadId_t thread_id = nullptr;
    osThreadAttr_t thread_attributes{};
    StaticTask_t thread_control_block{};
    std::array<uint32_t, SYSTEM_NODE_THREAD_STACK_SIZE / sizeof(uint32_t)> thread_stack{};

    void onResetRequest(const std_srvs__srv__Trigger_Request* request, std_srvs__srv__Trigger_Response* response);
    void onEmergencyStopRequest(const std_srvs__srv__Trigger_Request* request,
                                std_srvs__srv__Trigger_Response* response);
    void thread();
    void publishHeartbeat();
    void publishDiagnostics();
    void publishSystemDiagnostic(const SystemCheck::Result& result, uint32_t age_ms);
    void publishClientDiagnostic(const SystemCheck::ClientStatus& status);
    void publishDriverDiagnostic(const SystemCheck::DriverStatus& status);
    void handlePendingReset(uint32_t now_ms);
    void requestHardwareReset(uint32_t now_ms);

    static void configureTriggerResponse(std_srvs__srv__Trigger_Response& response, char* buffer, size_t buffer_size);
    static void fillTriggerResponse(std_srvs__srv__Trigger_Response* response, bool success, const char* message);
};
