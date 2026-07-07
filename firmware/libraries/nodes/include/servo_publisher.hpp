/**
 * @file servo_publisher.hpp
 *
 * @brief CAuDri - micro-ROS node for publishing servo feedback
 */
#pragma once

#include <cmsis_os2.h>
#include <lars_msgs/msg/float32_stamped.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "client.hpp"
#include "node.hpp"
#include "publisher.hpp"
#include "servo.hpp"

using ros::BasePublisher;
using ros::Client;
using ros::EntityState;
using ros::Node;
using ros::Publisher;

constexpr size_t SERVO_PUBLISHER_MAX_SERVOS = 2;
constexpr uint32_t SERVO_PUBLISHER_THREAD_STACK_SIZE = 2048;
constexpr uint32_t SERVO_PUBLISHER_DEFAULT_PERIOD_MS = 20;
constexpr uint32_t SERVO_PUBLISHER_DEFAULT_READ_FAILURE_THRESHOLD = 5;
constexpr uint32_t SERVO_PUBLISHER_DEFAULT_RECOVERY_PROBE_INTERVAL_MS = 1000;

/**
 * @brief Publishes analog servo feedback values through micro-ROS.
 *
 * The node owns one ROS publisher per registered servo. Each servo is read
 * sequentially from a dedicated RTOS thread, so a slow or failing servo only
 * affects its own topic and does not stop the other registered servos.
 */
class ServoPublisher : public Node {
   public:
    /**
     * @brief Configuration for the servo feedback publisher node.
     */
    struct Config {
        uint32_t publish_period_ms = SERVO_PUBLISHER_DEFAULT_PERIOD_MS;
        uint32_t read_failure_threshold = SERVO_PUBLISHER_DEFAULT_READ_FAILURE_THRESHOLD;
        uint32_t recovery_probe_interval_ms = SERVO_PUBLISHER_DEFAULT_RECOVERY_PROBE_INTERVAL_MS;
        osPriority_t thread_priority = osPriorityNormal;
        BasePublisher::Config publisher_config{true, 0};
    };

    ServoPublisher();
    ~ServoPublisher() = default;
    ServoPublisher(const ServoPublisher&) = delete;
    ServoPublisher& operator=(const ServoPublisher&) = delete;

    rcl_ret_t init(Client& client, const Config& config);
    bool registerServo(Servo& servo, const char* topic_name, const char* frame_id);
    rcl_ret_t start();

    size_t getServoCount() const;

   private:
    struct ServoSlot {
        Servo* servo = nullptr;
        const char* topic_name = nullptr;
        const char* frame_id = nullptr;
        Publisher<lars_msgs__msg__Float32Stamped> publisher{};
        lars_msgs__msg__Float32Stamped message{};
        uint32_t consecutive_read_failures = 0;
        uint32_t last_recovery_probe_ms = 0;
        bool publisher_initialized = false;
        bool available = true;
        bool read_failure_reported = false;
        bool publish_failure_reported = false;
    };

    Client* client = nullptr;
    Config config{};
    std::array<ServoSlot, SERVO_PUBLISHER_MAX_SERVOS> servos{};
    size_t servo_count = 0;
    bool started = false;

    osThreadId_t thread_id = nullptr;
    osThreadAttr_t thread_attributes{};
    StaticTask_t thread_control_block{};
    std::array<uint32_t, SERVO_PUBLISHER_THREAD_STACK_SIZE / sizeof(uint32_t)> thread_stack{};

    rcl_ret_t initPublishers();
    void thread();
    void publishServos(uint32_t now_ms);
    void publishServo(ServoSlot& slot, uint32_t now_ms);
    bool shouldProbeServo(const ServoSlot& slot, uint32_t now_ms) const;
    void handleReadFailure(ServoSlot& slot);
    void handleReadRecovery(ServoSlot& slot);
    void handlePublishResult(ServoSlot& slot, rcl_ret_t result);
};
