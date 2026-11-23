/**
 * @file autonomous_control.hpp
 *
 * @brief CAuDri - micro-ROS node for forwarding autonomous drive commands
 */
#pragma once

#include <builtin_interfaces/msg/time.h>
#include <lars_msgs/msg/motor_current_command.h>
#include <lars_msgs/msg/motor_rpm_command.h>
#include <lars_msgs/msg/steering_angle_command.h>

#include "client.hpp"
#include "drive_controller.hpp"
#include "node.hpp"
#include "subscriber.hpp"

using ros::BaseSubscriber;
using ros::Client;
using ros::EntityState;
using ros::Node;
using ros::Subscriber;

constexpr const char* AUTONOMOUS_CONTROL_DEFAULT_MOTOR_RPM_TOPIC = "command/motor_rpm";
constexpr const char* AUTONOMOUS_CONTROL_DEFAULT_MOTOR_CURRENT_TOPIC = "command/motor_current";
constexpr const char* AUTONOMOUS_CONTROL_DEFAULT_STEERING_ANGLE_TOPIC = "command/steering_angle";

/**
 * @brief Receives autonomous ROS commands and forwards them to the drive controller.
 *
 * The node translates ROS messages into drive-controller command structs,
 * converting ROS send time to RTOS ticks while preserving command age.
 * Messages older than one second are rejected before reaching the controller.
 * DriveController validates and forwards them to the actuators in this callback
 * context. Its controller thread monitors command timeouts and vehicle state.
 */
class AutonomousControl : public Node {
   public:
    /**
     * @brief Configuration for autonomous command topics and subscriber QoS.
     */
    struct Config {
        const char* motor_rpm_topic = AUTONOMOUS_CONTROL_DEFAULT_MOTOR_RPM_TOPIC;
        const char* motor_current_topic = AUTONOMOUS_CONTROL_DEFAULT_MOTOR_CURRENT_TOPIC;
        const char* steering_angle_topic = AUTONOMOUS_CONTROL_DEFAULT_STEERING_ANGLE_TOPIC;
        BaseSubscriber::Config subscriber_config{};
    };

    AutonomousControl();
    ~AutonomousControl() = default;
    AutonomousControl(const AutonomousControl&) = delete;
    AutonomousControl& operator=(const AutonomousControl&) = delete;

    rcl_ret_t init(Client& client, DriveController& drive_controller, const Config& config);

   private:
    Client* client = nullptr;
    DriveController* drive_controller = nullptr;

    Subscriber<lars_msgs__msg__MotorRpmCommand, AutonomousControl> motor_rpm_subscriber{};
    Subscriber<lars_msgs__msg__MotorCurrentCommand, AutonomousControl> motor_current_subscriber{};
    Subscriber<lars_msgs__msg__SteeringAngleCommand, AutonomousControl> steering_angle_subscriber{};

    bool motor_rpm_failure_reported = false;
    bool motor_current_failure_reported = false;
    bool steering_failure_reported = false;

    bool getCommandTimestamp(const builtin_interfaces__msg__Time& stamp, uint32_t& timestamp_ticks) const;
    void onMotorRpmCommand(const lars_msgs__msg__MotorRpmCommand* message);
    void onMotorCurrentCommand(const lars_msgs__msg__MotorCurrentCommand* message);
    void onSteeringAngleCommand(const lars_msgs__msg__SteeringAngleCommand* message);
};
