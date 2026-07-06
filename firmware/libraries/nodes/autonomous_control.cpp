/**
 * @file autonomous_control.cpp
 *
 * @brief CAuDri - micro-ROS autonomous command subscriber node implementation
 */

#include "autonomous_control.hpp"

#include "logger.h"
#include "type_support.hpp"

ROS_DECLARE_MESSAGE_TYPE(lars_msgs, MotorCurrentCommand);
ROS_DECLARE_MESSAGE_TYPE(lars_msgs, MotorRpmCommand);
ROS_DECLARE_MESSAGE_TYPE(lars_msgs, SteeringAngleCommand);

static constexpr const char* AUTONOMOUS_CONTROL_NODE_NAME = "autonomous_control";

/**
 * @brief Construct an uninitialized autonomous control node.
 */
AutonomousControl::AutonomousControl() = default;

/**
 * @brief Initialize the internal ROS node and its command subscribers.
 * @param client micro-ROS client that owns the session and entity lifecycle.
 * @param drive_controller Drive controller that applies validated commands.
 * @param config Node topic and subscriber configuration.
 * @return RCL_RET_OK on success, otherwise an rcl error code.
 */
rcl_ret_t AutonomousControl::init(ros::Client& client, DriveController& drive_controller, const Config& config) {
    if (node.getState() != ros::EntityState::UNINITIALIZED) {
        return RCL_RET_ALREADY_INIT;
    }
    if (config.motor_rpm_topic == nullptr || config.motor_rpm_topic[0] == '\0' ||
        config.motor_current_topic == nullptr || config.motor_current_topic[0] == '\0' ||
        config.steering_angle_topic == nullptr || config.steering_angle_topic[0] == '\0') {
        LogError("AutonomousControl: Invalid topic configuration");
        return RCL_RET_INVALID_ARGUMENT;
    }

    this->drive_controller = &drive_controller;
    this->config = config;

    rcl_ret_t result = node.init(client, AUTONOMOUS_CONTROL_NODE_NAME);
    if (result != RCL_RET_OK) {
        LogError("AutonomousControl: Failed to initialize ROS node: %d", static_cast<int>(result));
        return result;
    }

    result = motor_rpm_subscriber.init(
        node, config.motor_rpm_topic, this, &AutonomousControl::onMotorRpmCommand, config.subscriber_config);
    if (result != RCL_RET_OK) {
        LogError("AutonomousControl: Failed to initialize RPM subscriber: %d", static_cast<int>(result));
        return result;
    }

    result = motor_current_subscriber.init(
        node, config.motor_current_topic, this, &AutonomousControl::onMotorCurrentCommand, config.subscriber_config);
    if (result != RCL_RET_OK) {
        LogError("AutonomousControl: Failed to initialize current subscriber: %d", static_cast<int>(result));
        return result;
    }

    result = steering_angle_subscriber.init(
        node, config.steering_angle_topic, this, &AutonomousControl::onSteeringAngleCommand, config.subscriber_config);
    if (result != RCL_RET_OK) {
        LogError("AutonomousControl: Failed to initialize steering subscriber: %d", static_cast<int>(result));
        return result;
    }

    LogInfo("AutonomousControl: Initialized");
    return RCL_RET_OK;
}

/**
 * @brief Get the internal ROS node used by the subscribers.
 * @return Read-only reference to the internal ROS node.
 */
const ros::Node& AutonomousControl::getNode() const { return node; }

void AutonomousControl::onMotorRpmCommand(const lars_msgs__msg__MotorRpmCommand* message) {
    if (message == nullptr || drive_controller == nullptr) {
        if (!motor_rpm_failure_reported) {
            LogWarning("AutonomousControl: Failed to forward RPM command");
            motor_rpm_failure_reported = true;
        }
        return;
    }

    // Command freshness is measured from local receive time inside the drive
    // controller. The ROS header stamp is kept for host-side debugging and
    // latency analysis, but is not required for safe actuator fallback.
    DriveController::AutonomousMotorCommand command{};
    command.mode = DriveController::AutonomousMotorMode::RPM;
    command.rpm = message->rpm;

    if (drive_controller->updateAutonomousCommand(command)) {
        motor_rpm_failure_reported = false;
    } else if (!motor_rpm_failure_reported) {
        LogWarning("AutonomousControl: Failed to forward RPM command");
        motor_rpm_failure_reported = true;
    }
}

void AutonomousControl::onMotorCurrentCommand(const lars_msgs__msg__MotorCurrentCommand* message) {
    if (message == nullptr || drive_controller == nullptr) {
        if (!motor_current_failure_reported) {
            LogWarning("AutonomousControl: Failed to forward current command");
            motor_current_failure_reported = true;
        }
        return;
    }

    DriveController::AutonomousMotorCommand command{};
    command.mode = DriveController::AutonomousMotorMode::CURRENT;
    command.current = message->current;

    if (drive_controller->updateAutonomousCommand(command)) {
        motor_current_failure_reported = false;
    } else if (!motor_current_failure_reported) {
        LogWarning("AutonomousControl: Failed to forward current command");
        motor_current_failure_reported = true;
    }
}

void AutonomousControl::onSteeringAngleCommand(const lars_msgs__msg__SteeringAngleCommand* message) {
    if (message == nullptr || drive_controller == nullptr) {
        if (!steering_failure_reported) {
            LogWarning("AutonomousControl: Failed to forward steering command");
            steering_failure_reported = true;
        }
        return;
    }

    DriveController::AutonomousSteeringCommand command{};
    command.angle_deg = message->angle;

    if (drive_controller->updateAutonomousCommand(command)) {
        steering_failure_reported = false;
    } else if (!steering_failure_reported) {
        LogWarning("AutonomousControl: Failed to forward steering command");
        steering_failure_reported = true;
    }
}
