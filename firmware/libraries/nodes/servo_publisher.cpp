/**
 * @file servo_publisher.cpp
 *
 * @brief CAuDri - micro-ROS servo feedback publisher node implementation
 */

#include "servo_publisher.hpp"

#include <cstring>

#include "logger.h"
#include "type_support.hpp"

ROS_DECLARE_MESSAGE_TYPE(lars_msgs, Float32Stamped);

static constexpr uint32_t SERVO_PUBLISHER_START_FLAG = 0x01U;
static constexpr const char* SERVO_PUBLISHER_NODE_NAME = "servo_feedback";

/**
 * @brief Construct an uninitialized servo feedback publisher node.
 */
ServoPublisher::ServoPublisher() = default;

/**
 * @brief Initialize the internal ROS node and create the static publisher thread.
 * @param client micro-ROS client that owns the session and entity lifecycle.
 * @param config Node configuration copied into this object.
 * @return RCL_RET_OK on success, otherwise an rcl error code.
 */
rcl_ret_t ServoPublisher::init(ros::Client& client, const Config& config) {
    if (getState() != ros::EntityState::UNINITIALIZED) {
        return RCL_RET_ALREADY_INIT;
    }
    if (config.publish_period_ms == 0U || config.read_failure_threshold == 0U ||
        config.recovery_probe_interval_ms == 0U) {
        LogError("ServoPublisher: Invalid configuration");
        return RCL_RET_INVALID_ARGUMENT;
    }

    this->client = &client;
    this->config = config;

    rcl_ret_t result = ros::Node::init(client, SERVO_PUBLISHER_NODE_NAME);
    if (result != RCL_RET_OK) {
        LogError("ServoPublisher: Failed to initialize ROS node: %d", static_cast<int>(result));
        return result;
    }

    thread_attributes.name = "Servo Publisher";
    thread_attributes.priority = config.thread_priority;
    thread_attributes.stack_mem = thread_stack.data();
    thread_attributes.stack_size = sizeof(thread_stack);
    thread_attributes.cb_mem = &thread_control_block;
    thread_attributes.cb_size = sizeof(thread_control_block);

    thread_id = osThreadNew(
        [](void* arg) -> void {
            auto* obj = static_cast<ServoPublisher*>(arg);
            obj->thread();
        },
        this,
        &thread_attributes);

    if (thread_id == nullptr) {
        LogError("ServoPublisher: Failed to create publisher thread");
        (void)fini();
        return RCL_RET_ERROR;
    }

    LogInfo("ServoPublisher: Initialized");
    return RCL_RET_OK;
}

/**
 * @brief Register a servo driver and topic before starting the node.
 * @param servo Servo driver whose analog feedback should be published.
 * @param topic_name ROS topic name for this servo, relative to the client namespace unless absolute.
 * @param frame_id Frame id written into the stamped message header.
 * @return true when the servo was registered.
 */
bool ServoPublisher::registerServo(Servo& servo, const char* topic_name, const char* frame_id) {
    if (getState() == ros::EntityState::UNINITIALIZED || getState() == ros::EntityState::ERROR) {
        LogError("ServoPublisher: Cannot register servo before initialization or in error state");
        return false;
    }
    if (started) {
        LogError("ServoPublisher: Cannot register servo after node has been started");
        return false;
    }
    if (servo_count >= SERVO_PUBLISHER_MAX_SERVOS) {
        LogError("ServoPublisher: Maximum servo count reached (%u)", static_cast<unsigned>(SERVO_PUBLISHER_MAX_SERVOS));
        return false;
    }
    if (topic_name == nullptr || topic_name[0] == '\0' || frame_id == nullptr || frame_id[0] == '\0') {
        LogError("ServoPublisher: Invalid servo topic or frame id");
        return false;
    }

    ServoSlot& slot = servos[servo_count];
    slot.servo = &servo;
    slot.topic_name = topic_name;
    slot.frame_id = frame_id;
    slot.available = true;
    slot.consecutive_read_failures = 0;
    slot.read_failure_reported = false;
    slot.publish_failure_reported = false;

    servo_count++;
    LogInfo("ServoPublisher: Registered servo '%s' on '%s'", servo.getName(), topic_name);
    return true;
}

/**
 * @brief Initialize publishers for all registered servos and start the publisher thread.
 * @return RCL_RET_OK on success, otherwise an rcl error code.
 */
rcl_ret_t ServoPublisher::start() {
    if (started) {
        return RCL_RET_OK;
    }
    if (getState() == ros::EntityState::UNINITIALIZED || getState() == ros::EntityState::ERROR) {
        LogError("ServoPublisher: Cannot start, node is not initialized");
        return RCL_RET_NOT_INIT;
    }
    if (servo_count == 0U) {
        LogError("ServoPublisher: Cannot start without registered servos");
        markError(RCL_RET_INVALID_ARGUMENT);
        return RCL_RET_INVALID_ARGUMENT;
    }

    const rcl_ret_t result = initPublishers();
    if (result != RCL_RET_OK) {
        LogError("ServoPublisher: Failed to initialize publishers: %d", static_cast<int>(result));
        markError(result);
        return result;
    }

    const uint32_t flags = osThreadFlagsSet(thread_id, SERVO_PUBLISHER_START_FLAG);
    if ((flags & osFlagsError) != 0U) {
        LogError("ServoPublisher: Failed to start publisher thread, flags: 0x%08lX", flags);
        markError(RCL_RET_ERROR);
        return RCL_RET_ERROR;
    }

    started = true;
    return RCL_RET_OK;
}

/**
 * @brief Get the number of registered servo drivers.
 * @return Number of registered servos.
 */
size_t ServoPublisher::getServoCount() const { return servo_count; }

rcl_ret_t ServoPublisher::initPublishers() {
    for (size_t i = 0; i < servo_count; ++i) {
        ServoSlot& slot = servos[i];
        if (slot.publisher_initialized) {
            continue;
        }

        // The frame id points to application-owned static storage to avoid a
        // heap allocation for every message.
        slot.message.header.frame_id.data = const_cast<char*>(slot.frame_id);
        slot.message.header.frame_id.size = std::strlen(slot.frame_id);
        slot.message.header.frame_id.capacity = slot.message.header.frame_id.size + 1U;
        slot.message.data = 0.0F;

        const rcl_ret_t result = slot.publisher.init(*this, slot.topic_name, config.publisher_config);
        if (result != RCL_RET_OK) {
            LogError("ServoPublisher: Failed to register publisher '%s': %d", slot.topic_name, static_cast<int>(result));
            return result;
        }

        slot.publisher_initialized = true;
    }

    return RCL_RET_OK;
}

void ServoPublisher::thread() {
    const uint32_t flags = osThreadFlagsWait(SERVO_PUBLISHER_START_FLAG, osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) != 0U) {
        LogError("ServoPublisher: Start flag wait failed: 0x%08lX", flags);
        markError(RCL_RET_ERROR);
        osDelay(osWaitForever);
        return;
    }

    LogSuccess("ServoPublisher: Started with %u servo(s)", static_cast<unsigned>(servo_count));

    while (true) {
        if (client == nullptr) {
            LogError("ServoPublisher: Client disappeared, stopping node");
            markError(RCL_RET_ERROR);
            osDelay(osWaitForever);
            return;
        }

        if (!client->isConnected()) {
            client->waitForConnection(osWaitForever);
        }

        const uint32_t loop_start_ms = osKernelGetTickCount();
        publishServos(loop_start_ms);
        osDelayUntil(loop_start_ms + config.publish_period_ms);
    }
}

void ServoPublisher::publishServos(uint32_t now_ms) {
    for (size_t i = 0; i < servo_count; ++i) {
        publishServo(servos[i], now_ms);
    }
}

void ServoPublisher::publishServo(ServoSlot& slot, uint32_t now_ms) {
    if (slot.servo == nullptr || !slot.publisher_initialized) {
        return;
    }
    if (!shouldProbeServo(slot, now_ms)) {
        return;
    }

    float angle_deg = 0.0F;
    if (!slot.servo->getAngle(angle_deg)) {
        handleReadFailure(slot);
        return;
    }

    handleReadRecovery(slot);

    const builtin_interfaces__msg__Time stamp = client->getRosTime();
    slot.message.header.stamp = stamp;
    slot.message.data = angle_deg;

    const rcl_ret_t result = slot.publisher.publish(slot.message);
    handlePublishResult(slot, result);
}

bool ServoPublisher::shouldProbeServo(const ServoSlot& slot, uint32_t now_ms) const {
    if (slot.available) {
        return true;
    }

    // Once a servo is marked unavailable, probe it at a slower interval. This
    // still detects recovery, but avoids spending every publish cycle on a
    // known-bad ADC path and keeps the log output readable.
    return now_ms - slot.last_recovery_probe_ms >= config.recovery_probe_interval_ms;
}

void ServoPublisher::handleReadFailure(ServoSlot& slot) {
    if (slot.consecutive_read_failures < UINT32_MAX) {
        slot.consecutive_read_failures++;
    }

    if (slot.consecutive_read_failures >= config.read_failure_threshold) {
        slot.last_recovery_probe_ms = osKernelGetTickCount();
        slot.available = false;

        if (!slot.read_failure_reported) {
            LogWarning("ServoPublisher: Servo '%s' stopped delivering feedback, skipping '%s'",
                       slot.servo->getName(),
                       slot.topic_name);
            slot.read_failure_reported = true;
        }
    }
}

void ServoPublisher::handleReadRecovery(ServoSlot& slot) {
    if (!slot.available || slot.read_failure_reported) {
        LogInfo("ServoPublisher: Servo '%s' feedback recovered", slot.servo->getName());
    }

    slot.available = true;
    slot.consecutive_read_failures = 0;
    slot.read_failure_reported = false;
}

void ServoPublisher::handlePublishResult(ServoSlot& slot, rcl_ret_t result) {
    if (result == RCL_RET_OK) {
        slot.publish_failure_reported = false;
        return;
    }

    // RCL_RET_NOT_INIT and RCL_RET_TIMEOUT are expected while the session is
    // reconnecting or temporarily congested. Other errors are logged once per
    // failure streak; the publisher wrapper still forwards communication
    // errors to the client so reconnect handling remains centralized.
    if (result == RCL_RET_NOT_INIT || result == RCL_RET_TIMEOUT) {
        return;
    }

    if (!slot.publish_failure_reported) {
        LogWarning("ServoPublisher: Failed to publish '%s': %d", slot.topic_name, static_cast<int>(result));
        slot.publish_failure_reported = true;
    }
}
