/**
 * @file autonomous_control.hpp
 *
 * @brief CAuDri - micro-ROS node for forwarding autonomous drive commands
 */
#pragma once

#include <lars_msgs/msg/motor_current_command.h>
#include <lars_msgs/msg/motor_rpm_command.h>
#include <lars_msgs/msg/steering_angle_command.h>

#include "client.hpp"
#include "drive_controller.hpp"
#include "node.hpp"
#include "subscriber.hpp"

constexpr const char* AUTONOMOUS_CONTROL_DEFAULT_MOTOR_RPM_TOPIC = "command/motor_rpm";
constexpr const char* AUTONOMOUS_CONTROL_DEFAULT_MOTOR_CURRENT_TOPIC = "command/motor_current";
constexpr const char* AUTONOMOUS_CONTROL_DEFAULT_STEERING_ANGLE_TOPIC = "command/steering_angle";

/**
 * @brief Receives autonomous ROS commands and forwards them to the drive controller.
 *
 * The node only translates ROS messages into drive-controller command structs.
 * It does not write to the motor controller or steering servo directly, so all
 * mode checks, stale-command fallbacks, and actuator writes stay centralized in
 * DriveController.
 */
class AutonomousControl : public ros::Node {
   public:
    /**
     * @brief Configuration for autonomous command topics and subscriber QoS.
     */
    struct Config {
        const char* motor_rpm_topic = AUTONOMOUS_CONTROL_DEFAULT_MOTOR_RPM_TOPIC;
        const char* motor_current_topic = AUTONOMOUS_CONTROL_DEFAULT_MOTOR_CURRENT_TOPIC;
        const char* steering_angle_topic = AUTONOMOUS_CONTROL_DEFAULT_STEERING_ANGLE_TOPIC;
        ros::BaseSubscriber::Config subscriber_config{};
    };

    AutonomousControl();
    ~AutonomousControl() = default;
    AutonomousControl(const AutonomousControl&) = delete;
    AutonomousControl& operator=(const AutonomousControl&) = delete;

    rcl_ret_t init(ros::Client& client, DriveController& drive_controller, const Config& config);

   private:
    DriveController* drive_controller = nullptr;
    Config config{};

    ros::Subscriber<lars_msgs__msg__MotorRpmCommand, AutonomousControl> motor_rpm_subscriber{};
    ros::Subscriber<lars_msgs__msg__MotorCurrentCommand, AutonomousControl> motor_current_subscriber{};
    ros::Subscriber<lars_msgs__msg__SteeringAngleCommand, AutonomousControl> steering_angle_subscriber{};

    bool motor_rpm_failure_reported = false;
    bool motor_current_failure_reported = false;
    bool steering_failure_reported = false;

    void onMotorRpmCommand(const lars_msgs__msg__MotorRpmCommand* message);
    void onMotorCurrentCommand(const lars_msgs__msg__MotorCurrentCommand* message);
    void onSteeringAngleCommand(const lars_msgs__msg__SteeringAngleCommand* message);
};
