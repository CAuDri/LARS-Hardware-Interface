/**
 * @file imu_publisher.cpp
 *
 * @brief CAuDri - micro-ROS IMU publisher node implementation
 */

#include "imu_publisher.hpp"

#include <cstring>

#include "logger.h"
#include "type_support.hpp"

ROS_DECLARE_MESSAGE_TYPE(sensor_msgs, Imu);

static constexpr uint32_t IMU_PUBLISHER_START_FLAG = 0x01U;
static constexpr const char* IMU_PUBLISHER_NODE_NAME = "imu_feedback";

ImuPublisher::ImuPublisher() = default;

rcl_ret_t ImuPublisher::init(Client& client, const Config& config) {
    if (getState() != EntityState::UNINITIALIZED) {
        return RCL_RET_ALREADY_INIT;
    }
    if (config.publish_period_ms == 0U || config.read_failure_threshold == 0U || config.sample_ready_timeout_ms == 0U ||
        config.recovery_probe_interval_ms == 0U) {
        LogError("ImuPublisher: Invalid configuration");
        return RCL_RET_INVALID_ARGUMENT;
    }

    this->client = &client;
    this->config = config;

    rcl_ret_t result = Node::init(client, IMU_PUBLISHER_NODE_NAME);
    if (result != RCL_RET_OK) {
        LogError("ImuPublisher: Failed to initialize ROS node: %d", static_cast<int>(result));
        return result;
    }

    thread_attributes.name = "IMU Publisher";
    thread_attributes.priority = config.thread_priority;
    thread_attributes.stack_mem = thread_stack.data();
    thread_attributes.stack_size = sizeof(thread_stack);
    thread_attributes.cb_mem = &thread_control_block;
    thread_attributes.cb_size = sizeof(thread_control_block);

    thread_id = osThreadNew(
        [](void* arg) -> void {
            auto* obj = static_cast<ImuPublisher*>(arg);
            obj->thread();
        },
        this,
        &thread_attributes);

    if (thread_id == nullptr) {
        LogError("ImuPublisher: Failed to create publisher thread");
        (void)fini();
        return RCL_RET_ERROR;
    }

    LogInfo("ImuPublisher: Initialized");
    return RCL_RET_OK;
}

bool ImuPublisher::registerImu(ImuDriver& imu, const char* imu_topic, const char* frame_id) {
    if (getState() == EntityState::UNINITIALIZED || getState() == EntityState::ERROR) {
        LogError("ImuPublisher: Cannot register IMU before initialization or in error state");
        return false;
    }
    if (started) {
        LogError("ImuPublisher: Cannot register IMU after node has been started");
        return false;
    }
    if (imu_count >= IMU_PUBLISHER_MAX_IMUS) {
        LogError("ImuPublisher: Maximum IMU count reached (%u)", static_cast<unsigned>(IMU_PUBLISHER_MAX_IMUS));
        return false;
    }
    if (imu_topic == nullptr || imu_topic[0] == '\0' || frame_id == nullptr || frame_id[0] == '\0') {
        LogError("ImuPublisher: Invalid IMU topic or frame id");
        return false;
    }
    ImuSlot& slot = imus[imu_count];
    slot.imu = &imu;
    slot.imu_topic = imu_topic;
    slot.frame_id = frame_id;
    slot.consecutive_read_failures = 0;
    slot.last_sample_ms = 0;
    slot.last_recovery_probe_ms = 0;
    slot.readiness_bypassed = false;
    slot.read_failure_reported = false;
    slot.imu_publish_failure_reported = false;

    imu_count++;
    LogInfo("ImuPublisher: Registered IMU '%s' on '%s'", imu.getName(), imu_topic);
    return true;
}

rcl_ret_t ImuPublisher::start() {
    if (started) {
        return RCL_RET_OK;
    }
    if (getState() == EntityState::UNINITIALIZED || getState() == EntityState::ERROR) {
        LogError("ImuPublisher: Cannot start, node is not initialized");
        return RCL_RET_NOT_INIT;
    }
    if (imu_count == 0U) {
        LogError("ImuPublisher: Cannot start without registered IMUs");
        markError(RCL_RET_INVALID_ARGUMENT);
        return RCL_RET_INVALID_ARGUMENT;
    }
    for (size_t i = 0; i < imu_count; ++i) {
        if (imus[i].imu == nullptr || imus[i].imu->getState() != Driver::State::RUNNING ||
            imus[i].imu->getConnectionState() != Driver::ConnectionState::CONNECTED) {
            LogError("ImuPublisher: IMU '%s' is not running and connected",
                     imus[i].imu != nullptr ? imus[i].imu->getName() : "unknown");
            return RCL_RET_NOT_INIT;
        }
    }

    const rcl_ret_t result = initPublishers();
    if (result != RCL_RET_OK) {
        LogError("ImuPublisher: Failed to initialize publishers: %d", static_cast<int>(result));
        markError(result);
        return result;
    }

    const uint32_t now_ms = osKernelGetTickCount();
    for (size_t i = 0; i < imu_count; ++i) {
        imus[i].last_sample_ms = now_ms;
        imus[i].last_recovery_probe_ms = now_ms;
    }

    const uint32_t flags = osThreadFlagsSet(thread_id, IMU_PUBLISHER_START_FLAG);
    if ((flags & osFlagsError) != 0U) {
        LogError("ImuPublisher: Failed to start publisher thread, flags: 0x%08lX", flags);
        markError(RCL_RET_ERROR);
        return RCL_RET_ERROR;
    }

    started = true;
    return RCL_RET_OK;
}

size_t ImuPublisher::getImuCount() const { return imu_count; }

rcl_ret_t ImuPublisher::initPublishers() {
    for (size_t i = 0; i < imu_count; ++i) {
        ImuSlot& slot = imus[i];
        if (slot.publishers_initialized) {
            continue;
        }

        slot.imu_message.header.frame_id.data = const_cast<char*>(slot.frame_id);
        slot.imu_message.header.frame_id.size = std::strlen(slot.frame_id);
        slot.imu_message.header.frame_id.capacity = slot.imu_message.header.frame_id.size + 1U;

        rcl_ret_t result = slot.imu_publisher.init(*this, slot.imu_topic, config.publisher_config);
        if (result != RCL_RET_OK) {
            LogError("ImuPublisher: Failed to register IMU publisher '%s': %d", slot.imu_topic, static_cast<int>(result));
            return result;
        }

        slot.publishers_initialized = true;
    }

    return RCL_RET_OK;
}

void ImuPublisher::thread() {
    const uint32_t flags = osThreadFlagsWait(IMU_PUBLISHER_START_FLAG, osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) != 0U) {
        LogError("ImuPublisher: Start flag wait failed: 0x%08lX", flags);
        markError(RCL_RET_ERROR);
        osDelay(osWaitForever);
        return;
    }

    LogSuccess("ImuPublisher: Started with %u IMU(s)", static_cast<unsigned>(imu_count));

    while (true) {
        if (client == nullptr) {
            LogError("ImuPublisher: Client disappeared, stopping node");
            markError(RCL_RET_ERROR);
            osDelay(osWaitForever);
            return;
        }

        if (!client->isConnected()) {
            client->waitForConnection(osWaitForever);
        }

        const uint32_t loop_start_ms = osKernelGetTickCount();
        publishImus(loop_start_ms);
        osDelayUntil(loop_start_ms + config.publish_period_ms);
    }
}

void ImuPublisher::publishImus(uint32_t now_ms) {
    for (size_t i = 0; i < imu_count; ++i) {
        publishImu(imus[i], now_ms);
    }
}

void ImuPublisher::publishImu(ImuSlot& slot, uint32_t now_ms) {
    if (slot.imu == nullptr || !slot.publishers_initialized) {
        return;
    }
    if (slot.imu->getState() != Driver::State::RUNNING) {
        handleReadFailure(slot, now_ms);
        return;
    }
    if (!shouldProbeImu(slot, now_ms)) {
        return;
    }

    const bool sample_ready = slot.imu->isSampleReady();
    if (!sample_ready && !slot.readiness_bypassed) {
        if (now_ms - slot.last_sample_ms < config.sample_ready_timeout_ms) {
            return;
        }

        LogWarning("ImuPublisher: IMU '%s' data-ready signal timed out, falling back to polling", slot.imu->getName());
        slot.readiness_bypassed = true;
    } else if (sample_ready && slot.readiness_bypassed) {
        LogInfo("ImuPublisher: IMU '%s' data-ready signal recovered", slot.imu->getName());
        slot.readiness_bypassed = false;
    }

    ImuSample sample{};
    if (!slot.imu->readSample(sample)) {
        handleReadFailure(slot, now_ms);
        return;
    }

    slot.last_sample_ms = now_ms;
    handleReadRecovery(slot);
    fillMessages(slot, sample);
    handleImuPublishResult(slot, slot.imu_publisher.publish(slot.imu_message));
}

bool ImuPublisher::shouldProbeImu(const ImuSlot& slot, uint32_t now_ms) const {
    if (slot.imu->getConnectionState() == Driver::ConnectionState::CONNECTED) {
        return true;
    }

    return now_ms - slot.last_recovery_probe_ms >= config.recovery_probe_interval_ms;
}

void ImuPublisher::fillMessages(ImuSlot& slot, const ImuSample& sample) {
    const builtin_interfaces__msg__Time stamp = client->getRosTime();
    slot.imu_message.header.stamp = stamp;

    slot.imu_message.orientation.x = sample.orientation_xyzw[0];
    slot.imu_message.orientation.y = sample.orientation_xyzw[1];
    slot.imu_message.orientation.z = sample.orientation_xyzw[2];
    slot.imu_message.orientation.w = sample.orientation_xyzw[3];
    slot.imu_message.angular_velocity.x = sample.angular_velocity_radps[0];
    slot.imu_message.angular_velocity.y = sample.angular_velocity_radps[1];
    slot.imu_message.angular_velocity.z = sample.angular_velocity_radps[2];
    slot.imu_message.linear_acceleration.x = sample.acceleration_mps2[0];
    slot.imu_message.linear_acceleration.y = sample.acceleration_mps2[1];
    slot.imu_message.linear_acceleration.z = sample.acceleration_mps2[2];
}

void ImuPublisher::handleReadFailure(ImuSlot& slot, uint32_t now_ms) {
    if (slot.consecutive_read_failures < UINT32_MAX) {
        slot.consecutive_read_failures++;
    }

    if (slot.imu->getConnectionState() != Driver::ConnectionState::CONNECTED) {
        slot.last_recovery_probe_ms = now_ms;
    }

    if (slot.consecutive_read_failures >= config.read_failure_threshold && !slot.read_failure_reported) {
        LogWarning("ImuPublisher: IMU '%s' stopped delivering samples", slot.imu->getName());
        slot.read_failure_reported = true;
    }
}

void ImuPublisher::handleReadRecovery(ImuSlot& slot) {
    if (slot.read_failure_reported) {
        LogInfo("ImuPublisher: IMU '%s' samples recovered", slot.imu->getName());
    }

    slot.consecutive_read_failures = 0;
    slot.read_failure_reported = false;
}

void ImuPublisher::handleImuPublishResult(ImuSlot& slot, rcl_ret_t result) {
    if (result == RCL_RET_OK) {
        slot.imu_publish_failure_reported = false;
        return;
    }
    if (result == RCL_RET_NOT_INIT || result == RCL_RET_TIMEOUT) {
        return;
    }
    if (!slot.imu_publish_failure_reported) {
        LogWarning("ImuPublisher: Failed to publish '%s': %d", slot.imu_topic, static_cast<int>(result));
        slot.imu_publish_failure_reported = true;
    }
}
