/**
 * @file motor_publisher.hpp
 *
 * @brief CAuDri - micro-ROS node for publishing motor controller feedback
 */
#pragma once

#include <cmsis_os2.h>
#include <lars_msgs/msg/motor_feedback.h>
#include <lars_msgs/msg/motor_telemetry.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "client.hpp"
#include "node.hpp"
#include "publisher.hpp"
#include "vesc.hpp"

constexpr size_t MOTOR_PUBLISHER_MAX_MOTORS = 2;
constexpr uint32_t MOTOR_PUBLISHER_THREAD_STACK_SIZE = 2048;
constexpr uint32_t MOTOR_PUBLISHER_DEFAULT_PERIOD_MS = 20;
constexpr uint32_t MOTOR_PUBLISHER_DEFAULT_TELEMETRY_PERIOD_MS = 200;

/**
 * @brief Publishes VESC motor feedback through micro-ROS.
 *
 * The node separates high-rate controller feedback from slower telemetry. RPM
 * and motor current are published at the configured feedback rate. Temperature,
 * input-current, voltage, energy, and PID position are published at a lower
 * telemetry rate to keep the ROS transport load small.
 */
class MotorPublisher {
   public:
    /**
     * @brief Runtime state of the motor feedback publisher node.
     */
    enum class State { ERROR, UNINITIALIZED, INITIALIZED, RUNNING };

    /**
     * @brief Configuration for the motor feedback publisher node.
     */
    struct Config {
        uint32_t publish_period_ms = MOTOR_PUBLISHER_DEFAULT_PERIOD_MS;
        uint32_t telemetry_period_ms = MOTOR_PUBLISHER_DEFAULT_TELEMETRY_PERIOD_MS;
        osPriority_t thread_priority = osPriorityNormal;
    };

    MotorPublisher();
    ~MotorPublisher() = default;
    MotorPublisher(const MotorPublisher&) = delete;
    MotorPublisher& operator=(const MotorPublisher&) = delete;

    rcl_ret_t init(ros::Client& client, const Config& config);
    bool registerMotor(VESC& motor, const char* feedback_topic, const char* telemetry_topic, const char* frame_id);
    rcl_ret_t start();

    State getState() const;
    size_t getMotorCount() const;
    const ros::Node& getNode() const;

   private:
    struct MotorSlot {
        VESC* motor = nullptr;
        const char* feedback_topic = nullptr;
        const char* telemetry_topic = nullptr;
        const char* frame_id = nullptr;
        ros::Publisher<lars_msgs__msg__MotorFeedback> feedback_publisher{};
        ros::Publisher<lars_msgs__msg__MotorTelemetry> telemetry_publisher{};
        lars_msgs__msg__MotorFeedback feedback_message{};
        lars_msgs__msg__MotorTelemetry telemetry_message{};
        uint32_t last_telemetry_publish_ms = 0;
        bool publishers_initialized = false;
        bool connection_failure_reported = false;
        bool feedback_publish_failure_reported = false;
        bool telemetry_publish_failure_reported = false;
    };

    ros::Client* client = nullptr;
    Config config{};
    ros::Node node{};
    std::array<MotorSlot, MOTOR_PUBLISHER_MAX_MOTORS> motors{};
    size_t motor_count = 0;
    volatile State state = State::UNINITIALIZED;

    osThreadId_t thread_id = nullptr;
    osThreadAttr_t thread_attributes{};
    StaticTask_t thread_control_block{};
    std::array<uint32_t, MOTOR_PUBLISHER_THREAD_STACK_SIZE / sizeof(uint32_t)> thread_stack{};

    rcl_ret_t initPublishers();
    void thread();
    void publishMotors(uint32_t now_ms);
    void publishMotor(MotorSlot& slot, uint32_t now_ms);
    void fillFeedbackMessage(MotorSlot& slot, const vesc::Status& status);
    void fillTelemetryMessage(MotorSlot& slot, const vesc::Status& status);
    void handleUnavailableMotor(MotorSlot& slot);
    void handleFeedbackPublishResult(MotorSlot& slot, rcl_ret_t result);
    void handleTelemetryPublishResult(MotorSlot& slot, rcl_ret_t result);
};
