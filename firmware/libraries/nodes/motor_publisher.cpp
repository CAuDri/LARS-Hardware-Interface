/**
 * @file motor_publisher.cpp
 *
 * @brief CAuDri - micro-ROS motor feedback publisher node implementation
 */

#include "motor_publisher.hpp"

#include <cstring>

#include "logger.h"
#include "type_support.hpp"

ROS_DECLARE_MESSAGE_TYPE(lars_msgs, MotorFeedback);
ROS_DECLARE_MESSAGE_TYPE(lars_msgs, MotorTelemetry);

static constexpr uint32_t MOTOR_PUBLISHER_START_FLAG = 0x01U;
static constexpr const char* MOTOR_PUBLISHER_NODE_NAME = "motor_feedback";
static constexpr BasePublisher::Config MOTOR_PUBLISHER_CONFIG{true, 0};

/**
 * @brief Construct an uninitialized motor feedback publisher node.
 */
MotorPublisher::MotorPublisher() = default;

/**
 * @brief Initialize the internal ROS node and create the static publisher thread.
 * @param client micro-ROS client that owns the session and entity lifecycle.
 * @param config Node configuration copied into this object.
 * @return RCL_RET_OK on success, otherwise an rcl error code.
 */
rcl_ret_t MotorPublisher::init(Client& client, const Config& config) {
    if (getState() != EntityState::UNINITIALIZED) {
        return RCL_RET_ALREADY_INIT;
    }
    if (config.publish_period_ms == 0U || config.telemetry_period_ms == 0U) {
        LogError("MotorPublisher: Invalid configuration");
        return RCL_RET_INVALID_ARGUMENT;
    }

    this->client = &client;
    this->config = config;

    rcl_ret_t result = Node::init(client, MOTOR_PUBLISHER_NODE_NAME);
    if (result != RCL_RET_OK) {
        LogError("MotorPublisher: Failed to initialize ROS node: %d", static_cast<int>(result));
        return result;
    }

    thread_attributes.name = "Motor Publisher";
    thread_attributes.priority = config.thread_priority;
    thread_attributes.stack_mem = thread_stack.data();
    thread_attributes.stack_size = sizeof(thread_stack);
    thread_attributes.cb_mem = &thread_control_block;
    thread_attributes.cb_size = sizeof(thread_control_block);

    thread_id = osThreadNew(
        [](void* arg) -> void {
            auto* obj = static_cast<MotorPublisher*>(arg);
            obj->thread();
        },
        this,
        &thread_attributes);

    if (thread_id == nullptr) {
        LogError("MotorPublisher: Failed to create publisher thread");
        (void)fini();
        return RCL_RET_ERROR;
    }

    LogInfo("MotorPublisher: Initialized");
    return RCL_RET_OK;
}

/**
 * @brief Register a VESC driver and topics before starting the node.
 * @param motor VESC driver whose status should be published.
 * @param feedback_topic High-rate ROS topic for RPM and motor current.
 * @param telemetry_topic Low-rate ROS topic for slower telemetry values.
 * @param frame_id Frame id written into both stamped messages.
 * @return true when the motor was registered.
 */
bool MotorPublisher::registerMotor(VESC& motor,
                                   const char* feedback_topic,
                                   const char* telemetry_topic,
                                   const char* frame_id) {
    if (getState() == EntityState::UNINITIALIZED || getState() == EntityState::ERROR) {
        LogError("MotorPublisher: Cannot register motor before initialization or in error state");
        return false;
    }
    if (started) {
        LogError("MotorPublisher: Cannot register motor after node has been started");
        return false;
    }
    if (motor_count >= MOTOR_PUBLISHER_MAX_MOTORS) {
        LogError("MotorPublisher: Maximum motor count reached (%u)", static_cast<unsigned>(MOTOR_PUBLISHER_MAX_MOTORS));
        return false;
    }
    if (feedback_topic == nullptr || feedback_topic[0] == '\0' || telemetry_topic == nullptr ||
        telemetry_topic[0] == '\0' || frame_id == nullptr || frame_id[0] == '\0') {
        LogError("MotorPublisher: Invalid motor topic or frame id");
        return false;
    }

    MotorSlot& slot = motors[motor_count];
    slot.motor = &motor;
    slot.feedback_topic = feedback_topic;
    slot.telemetry_topic = telemetry_topic;
    slot.frame_id = frame_id;
    slot.last_telemetry_publish_ms = 0;
    slot.connection_failure_reported = false;
    slot.feedback_publish_failure_reported = false;
    slot.telemetry_publish_failure_reported = false;

    motor_count++;
    LogInfo("MotorPublisher: Registered motor '%s' on '%s' and '%s'", motor.getName(), feedback_topic, telemetry_topic);
    return true;
}

/**
 * @brief Initialize publishers for all registered motors and start the publisher thread.
 * @return RCL_RET_OK on success, otherwise an rcl error code.
 */
rcl_ret_t MotorPublisher::start() {
    if (started) {
        return RCL_RET_OK;
    }
    if (getState() == EntityState::UNINITIALIZED || getState() == EntityState::ERROR) {
        LogError("MotorPublisher: Cannot start, node is not initialized");
        return RCL_RET_NOT_INIT;
    }
    if (motor_count == 0U) {
        LogError("MotorPublisher: Cannot start without registered motors");
        markError(RCL_RET_INVALID_ARGUMENT);
        return RCL_RET_INVALID_ARGUMENT;
    }

    const rcl_ret_t result = initPublishers();
    if (result != RCL_RET_OK) {
        LogError("MotorPublisher: Failed to initialize publishers: %d", static_cast<int>(result));
        markError(result);
        return result;
    }

    const uint32_t flags = osThreadFlagsSet(thread_id, MOTOR_PUBLISHER_START_FLAG);
    if ((flags & osFlagsError) != 0U) {
        LogError("MotorPublisher: Failed to start publisher thread, flags: 0x%08lX", flags);
        markError(RCL_RET_ERROR);
        return RCL_RET_ERROR;
    }

    started = true;
    return RCL_RET_OK;
}

/**
 * @brief Get the number of registered VESC drivers.
 * @return Number of registered motors.
 */
size_t MotorPublisher::getMotorCount() const { return motor_count; }

rcl_ret_t MotorPublisher::initPublishers() {
    for (size_t i = 0; i < motor_count; ++i) {
        MotorSlot& slot = motors[i];
        if (slot.publishers_initialized) {
            continue;
        }

        // The frame id points to application-owned static storage to avoid heap
        // allocations for the fixed message headers.
        slot.feedback_message.header.frame_id.data = const_cast<char*>(slot.frame_id);
        slot.feedback_message.header.frame_id.size = std::strlen(slot.frame_id);
        slot.feedback_message.header.frame_id.capacity = slot.feedback_message.header.frame_id.size + 1U;

        slot.telemetry_message.header.frame_id.data = const_cast<char*>(slot.frame_id);
        slot.telemetry_message.header.frame_id.size = std::strlen(slot.frame_id);
        slot.telemetry_message.header.frame_id.capacity = slot.telemetry_message.header.frame_id.size + 1U;

        rcl_ret_t result = slot.feedback_publisher.init(*this, slot.feedback_topic, MOTOR_PUBLISHER_CONFIG);
        if (result != RCL_RET_OK) {
            LogError("MotorPublisher: Failed to register feedback publisher '%s': %d",
                     slot.feedback_topic,
                     static_cast<int>(result));
            return result;
        }

        result = slot.telemetry_publisher.init(*this, slot.telemetry_topic, MOTOR_PUBLISHER_CONFIG);
        if (result != RCL_RET_OK) {
            LogError("MotorPublisher: Failed to register telemetry publisher '%s': %d",
                     slot.telemetry_topic,
                     static_cast<int>(result));
            return result;
        }

        slot.publishers_initialized = true;
    }

    return RCL_RET_OK;
}

void MotorPublisher::thread() {
    const uint32_t flags = osThreadFlagsWait(MOTOR_PUBLISHER_START_FLAG, osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) != 0U) {
        LogError("MotorPublisher: Start flag wait failed: 0x%08lX", flags);
        markError(RCL_RET_ERROR);
        osDelay(osWaitForever);
        return;
    }

    LogSuccess("MotorPublisher: Started with %u motor(s)", static_cast<unsigned>(motor_count));

    while (true) {
        if (client == nullptr) {
            LogError("MotorPublisher: Client disappeared, stopping node");
            markError(RCL_RET_ERROR);
            osDelay(osWaitForever);
            return;
        }

        if (!client->isConnected()) {
            client->waitForConnection(osWaitForever);
        }

        const uint32_t loop_start_ms = osKernelGetTickCount();
        publishMotors(loop_start_ms);
        osDelayUntil(loop_start_ms + config.publish_period_ms);
    }
}

void MotorPublisher::publishMotors(uint32_t now_ms) {
    for (size_t i = 0; i < motor_count; ++i) {
        publishMotor(motors[i], now_ms);
    }
}

void MotorPublisher::publishMotor(MotorSlot& slot, uint32_t now_ms) {
    if (slot.motor == nullptr || !slot.publishers_initialized) {
        return;
    }

    if (slot.motor->getState() != Driver::State::RUNNING ||
        slot.motor->getConnectionState() != Driver::ConnectionState::CONNECTED) {
        handleUnavailableMotor(slot);
        return;
    }

    vesc::Status* status = slot.motor->getRawStatus();
    if (status == nullptr) {
        handleUnavailableMotor(slot);
        return;
    }

    slot.connection_failure_reported = false;
    fillFeedbackMessage(slot, *status);
    handleFeedbackPublishResult(slot, slot.feedback_publisher.publish(slot.feedback_message));

    if (now_ms - slot.last_telemetry_publish_ms >= config.telemetry_period_ms) {
        slot.last_telemetry_publish_ms = now_ms;
        fillTelemetryMessage(slot, *status);
        handleTelemetryPublishResult(slot, slot.telemetry_publisher.publish(slot.telemetry_message));
    }
}

void MotorPublisher::fillFeedbackMessage(MotorSlot& slot, const vesc::Status& status) {
    slot.feedback_message.header.stamp = client->getRosTime();
    slot.feedback_message.rpm = status.status1.rpm;
    slot.feedback_message.motor_current = status.status1.current;
}

void MotorPublisher::fillTelemetryMessage(MotorSlot& slot, const vesc::Status& status) {
    slot.telemetry_message.header.stamp = client->getRosTime();
    slot.telemetry_message.input_current = status.status4.current_in;
    slot.telemetry_message.input_voltage = status.status5.v_in;
    slot.telemetry_message.fet_temperature = status.status4.temp_fet;
    slot.telemetry_message.motor_temperature = status.status4.temp_motor;
    slot.telemetry_message.consumed_amp_hours = status.status2.amp_hours;
    slot.telemetry_message.charged_amp_hours = status.status2.amp_hours_charged;
    slot.telemetry_message.pid_position = status.status4.pid_pos_now;
}

void MotorPublisher::handleUnavailableMotor(MotorSlot& slot) {
    if (!slot.connection_failure_reported) {
        LogWarning("MotorPublisher: Motor '%s' is not connected, skipping publishers", slot.motor->getName());
        slot.connection_failure_reported = true;
    }
}

void MotorPublisher::handleFeedbackPublishResult(MotorSlot& slot, rcl_ret_t result) {
    if (result == RCL_RET_OK) {
        slot.feedback_publish_failure_reported = false;
        return;
    }
    if (result == RCL_RET_NOT_INIT || result == RCL_RET_TIMEOUT) {
        return;
    }
    if (!slot.feedback_publish_failure_reported) {
        LogWarning("MotorPublisher: Failed to publish '%s': %d", slot.feedback_topic, static_cast<int>(result));
        slot.feedback_publish_failure_reported = true;
    }
}

void MotorPublisher::handleTelemetryPublishResult(MotorSlot& slot, rcl_ret_t result) {
    if (result == RCL_RET_OK) {
        slot.telemetry_publish_failure_reported = false;
        return;
    }
    if (result == RCL_RET_NOT_INIT || result == RCL_RET_TIMEOUT) {
        return;
    }
    if (!slot.telemetry_publish_failure_reported) {
        LogWarning("MotorPublisher: Failed to publish '%s': %d", slot.telemetry_topic, static_cast<int>(result));
        slot.telemetry_publish_failure_reported = true;
    }
}
