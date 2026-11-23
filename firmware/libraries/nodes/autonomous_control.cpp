/**
 * @file autonomous_control.cpp
 *
 * @brief CAuDri - micro-ROS autonomous command subscriber node implementation
 */

#include "autonomous_control.hpp"

#include <rcl/time.h>

#include "logger.h"
#include "type_support.hpp"

ROS_DECLARE_MESSAGE_TYPE(lars_msgs, MotorCurrentCommand);
ROS_DECLARE_MESSAGE_TYPE(lars_msgs, MotorRpmCommand);
ROS_DECLARE_MESSAGE_TYPE(lars_msgs, SteeringAngleCommand);

static constexpr const char* AUTONOMOUS_CONTROL_NODE_NAME = "autonomous_control";
static constexpr uint32_t AUTONOMOUS_CONTROL_MAX_COMMAND_AGE_MS = 1000;

/**
 * @brief Construct an uninitialized autonomous control node.
 */
AutonomousControl::AutonomousControl() = default;

/**
 * @brief Initialize the internal ROS node and its command subscribers.
 *
 * @param client micro-ROS client that owns the session and entity lifecycle.
 * @param drive_controller Drive controller that applies validated commands.
 * @param config Node topic and subscriber configuration.
 * @return RCL_RET_OK on success, otherwise an rcl error code.
 */
rcl_ret_t AutonomousControl::init(Client& client, DriveController& drive_controller, const Config& config) {
    if (getState() != EntityState::UNINITIALIZED) {
        return RCL_RET_ALREADY_INIT;
    }
    if (config.motor_rpm_topic == nullptr || config.motor_rpm_topic[0] == '\0' ||
        config.motor_current_topic == nullptr || config.motor_current_topic[0] == '\0' ||
        config.steering_angle_topic == nullptr || config.steering_angle_topic[0] == '\0') {
        LogError("AutonomousControl: Invalid topic configuration");
        return RCL_RET_INVALID_ARGUMENT;
    }

    this->client = &client;
    this->drive_controller = &drive_controller;

    rcl_ret_t result = Node::init(client, AUTONOMOUS_CONTROL_NODE_NAME);
    if (result != RCL_RET_OK) {
        LogError("AutonomousControl: Failed to initialize ROS node: %d", static_cast<int>(result));
        return result;
    }

    result = motor_rpm_subscriber.init(
        *this, config.motor_rpm_topic, this, &AutonomousControl::onMotorRpmCommand, config.subscriber_config);
    if (result != RCL_RET_OK) {
        LogError("AutonomousControl: Failed to initialize RPM subscriber: %d", static_cast<int>(result));
        return result;
    }

    result = motor_current_subscriber.init(
        *this, config.motor_current_topic, this, &AutonomousControl::onMotorCurrentCommand, config.subscriber_config);
    if (result != RCL_RET_OK) {
        LogError("AutonomousControl: Failed to initialize current subscriber: %d", static_cast<int>(result));
        return result;
    }

    result = steering_angle_subscriber.init(
        *this, config.steering_angle_topic, this, &AutonomousControl::onSteeringAngleCommand, config.subscriber_config);
    if (result != RCL_RET_OK) {
        LogError("AutonomousControl: Failed to initialize steering subscriber: %d", static_cast<int>(result));
        return result;
    }

    LogInfo("AutonomousControl: Initialized");
    return RCL_RET_OK;
}

/**
 * @brief Convert ROS send time to RTOS ticks without resetting command age.
 */
bool AutonomousControl::getCommandTimestamp(const builtin_interfaces__msg__Time& stamp, uint32_t& timestamp_ticks) const {
    if (client == nullptr || stamp.sec < 0 || stamp.nanosec >= 1000000000U ||
        (stamp.sec == 0 && stamp.nanosec == 0U)) {
        return false;
    }

    // Sample ticks first so a delay between clock reads cannot make a command younger.
    const uint32_t receive_ticks = osKernelGetTickCount();
    const auto now = client->getRosTime();
    if (now.sec == 0 && now.nanosec == 0U) {
        return false;
    }
    const rcl_time_point_t sent{RCL_S_TO_NS(stamp.sec) + stamp.nanosec, RCL_ROS_TIME};
    const rcl_time_point_t current{RCL_S_TO_NS(now.sec) + now.nanosec, RCL_ROS_TIME};
    rcl_duration_t age{};
    if (rcl_difference_times(&sent, &current, &age) != RCL_RET_OK || age.nanoseconds < 0 ||
        age.nanoseconds > RCL_MS_TO_NS(AUTONOMOUS_CONTROL_MAX_COMMAND_AGE_MS)) {
        return false;
    }

    // Round age up to ticks so conversion never extends command lifetime.
    const int64_t age_ticks =
        (age.nanoseconds * osKernelGetTickFreq() + RCL_S_TO_NS(1) - 1) / RCL_S_TO_NS(1);
    timestamp_ticks = receive_ticks - static_cast<uint32_t>(age_ticks);
    return true;
}

void AutonomousControl::onMotorRpmCommand(const lars_msgs__msg__MotorRpmCommand* message) {
    DriveController::MotorCommand command{};
    if (message == nullptr || drive_controller == nullptr ||
        !getCommandTimestamp(message->header.stamp, command.timestamp_ticks)) {
        if (!motor_rpm_failure_reported) {
            LogWarning("AutonomousControl: Failed to forward RPM command");
            motor_rpm_failure_reported = true;
        }
        return;
    }

    command.mode = DriveController::MotorCommand::Mode::RPM;
    command.rpm = message->rpm;

    if (drive_controller->updateMotorCommand(command)) {
        motor_rpm_failure_reported = false;
    } else if (!motor_rpm_failure_reported) {
        LogWarning("AutonomousControl: Failed to forward RPM command");
        motor_rpm_failure_reported = true;
    }
}

void AutonomousControl::onMotorCurrentCommand(const lars_msgs__msg__MotorCurrentCommand* message) {
    DriveController::MotorCommand command{};
    if (message == nullptr || drive_controller == nullptr ||
        !getCommandTimestamp(message->header.stamp, command.timestamp_ticks)) {
        if (!motor_current_failure_reported) {
            LogWarning("AutonomousControl: Failed to forward current command");
            motor_current_failure_reported = true;
        }
        return;
    }

    command.mode = DriveController::MotorCommand::Mode::CURRENT;
    command.current = message->current;

    if (drive_controller->updateMotorCommand(command)) {
        motor_current_failure_reported = false;
    } else if (!motor_current_failure_reported) {
        LogWarning("AutonomousControl: Failed to forward current command");
        motor_current_failure_reported = true;
    }
}

void AutonomousControl::onSteeringAngleCommand(const lars_msgs__msg__SteeringAngleCommand* message) {
    DriveController::SteeringCommand command{};
    if (message == nullptr || drive_controller == nullptr ||
        !getCommandTimestamp(message->header.stamp, command.timestamp_ticks)) {
        if (!steering_failure_reported) {
            LogWarning("AutonomousControl: Failed to forward steering command");
            steering_failure_reported = true;
        }
        return;
    }

    command.angle_deg = message->angle;

    if (drive_controller->updateSteeringCommand(command)) {
        steering_failure_reported = false;
    } else if (!steering_failure_reported) {
        LogWarning("AutonomousControl: Failed to forward steering command");
        steering_failure_reported = true;
    }
}
