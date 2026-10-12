/**
 * @file tof_publisher.cpp
 *
 * @brief Generic micro-ROS Time-of-Flight point cloud publisher implementation
 */
#include "tof_publisher.hpp"

#include <sensor_msgs/msg/point_field.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "logger.h"
#include "type_support.hpp"

ROS_DECLARE_MESSAGE_TYPE(sensor_msgs, PointCloud2);

static constexpr uint32_t TOF_PUBLISHER_START_FLAG = 0x01U;
static constexpr const char* TOF_PUBLISHER_NODE_NAME = "tof_feedback";
static constexpr size_t TOF_POINT_FIELD_COUNT = 3;
static constexpr size_t TOF_POINT_COMPONENT_COUNT = 3;
static constexpr size_t TOF_POINT_SIZE_BYTES = TOF_POINT_COMPONENT_COUNT * sizeof(float);
static constexpr float TOF_MAX_FOV_RAD = 3.141592654F;

// All registered sensors publish sequentially from one thread, so one XYZ
// scratch buffer can back every PointCloud2 message without concurrent access.
// Both the scratch data and the dedicated thread stack are CPU-only and can
// therefore use CCMRAM safely.
static float tof_point_data[TOF_MAX_ZONES * TOF_POINT_COMPONENT_COUNT]
    __attribute__((section(".ccmram.tof_publisher"), aligned(4)));

static uint32_t tof_publisher_thread_stack[TOF_PUBLISHER_THREAD_STACK_SIZE / sizeof(uint32_t)]
    __attribute__((section(".ccmram.tof_publisher"), aligned(8)));

static char tof_x_name[] = "x";
static char tof_y_name[] = "y";
static char tof_z_name[] = "z";
static sensor_msgs__msg__PointField tof_point_fields[TOF_POINT_FIELD_COUNT]{};
static bool tof_publisher_claimed = false;

// PointCloud2 describes its packed byte buffer through a separate field table.
// The names and table are static to avoid allocating ROS strings or sequences.
static void initializePointField(sensor_msgs__msg__PointField& field, char* name, uint32_t offset) {
    field.name.data = name;
    field.name.size = std::strlen(name);
    field.name.capacity = field.name.size + 1U;
    field.offset = offset;
    field.datatype = sensor_msgs__msg__PointField__FLOAT32;
    field.count = 1U;
}

/**
 * @brief Construct an uninitialized generic ToF publisher node.
 */
TofPublisher::TofPublisher() = default;

/**
 * @brief Register the ROS node and create its static publisher thread.
 *
 * The thread remains blocked until start() finishes publisher registration.
 * Only one instance is permitted because PointCloud storage is shared.
 *
 * @param client micro-ROS client that owns entity and session lifecycles.
 * @param config Publisher timing, priority, QoS, and failure policy.
 * @return RCL_RET_OK when initialization completed, otherwise an rcl error.
 */
rcl_ret_t TofPublisher::init(Client& client, const Config& config) {
    if (getState() != EntityState::UNINITIALIZED) {
        return RCL_RET_ALREADY_INIT;
    }
    if (tof_publisher_claimed) {
        LogError("TofPublisher: Only one instance can use the shared CCMRAM buffers");
        return RCL_RET_ALREADY_INIT;
    }
    if (config.publish_period_ms == 0U || config.read_failure_threshold == 0U) {
        LogError("TofPublisher: Invalid configuration");
        return RCL_RET_INVALID_ARGUMENT;
    }

    this->client = &client;
    this->config = config;

    rcl_ret_t result = Node::init(client, TOF_PUBLISHER_NODE_NAME);
    if (result != RCL_RET_OK) {
        LogError("TofPublisher: Failed to initialize ROS node: %d", static_cast<int>(result));
        return result;
    }

    initializePointField(tof_point_fields[0], tof_x_name, 0U);
    initializePointField(tof_point_fields[1], tof_y_name, sizeof(float));
    initializePointField(tof_point_fields[2], tof_z_name, 2U * sizeof(float));

    thread_attributes.name = "ToF Publisher";
    thread_attributes.priority = config.thread_priority;
    thread_attributes.stack_mem = tof_publisher_thread_stack;
    thread_attributes.stack_size = sizeof(tof_publisher_thread_stack);
    thread_attributes.cb_mem = &thread_control_block;
    thread_attributes.cb_size = sizeof(thread_control_block);

    thread_id = osThreadNew([](void* arg) -> void { static_cast<TofPublisher*>(arg)->thread(); }, this, &thread_attributes);
    if (thread_id == nullptr) {
        LogError("TofPublisher: Failed to create publisher thread");
        (void)fini();
        return RCL_RET_ERROR;
    }

    tof_publisher_claimed = true;
    LogInfo("TofPublisher: Initialized");
    return RCL_RET_OK;
}

/**
 * @brief Register a ToF driver and its PointCloud2 topic.
 *
 * Geometry is captured at registration time, so the driver configuration must
 * remain unchanged afterward. Topic and frame strings must remain valid for
 * the node's lifetime.
 *
 * @param tof Initialized generic ToF driver.
 * @param topic ROS PointCloud2 topic name.
 * @param frame_id Optical frame whose axes are x-right, y-down, z-forward.
 * @return true when the sensor was registered.
 */
bool TofPublisher::registerTof(TofDriver& tof, const char* topic, const char* frame_id) {
    if (getState() == EntityState::UNINITIALIZED || getState() == EntityState::ERROR || started) {
        LogError("TofPublisher: Cannot register ToF in current state");
        return false;
    }
    if (sensor_count >= TOF_PUBLISHER_MAX_SENSORS) {
        LogError("TofPublisher: Maximum ToF count reached (%u)", static_cast<unsigned>(TOF_PUBLISHER_MAX_SENSORS));
        return false;
    }
    if (topic == nullptr || topic[0] == '\0' || frame_id == nullptr || frame_id[0] == '\0') {
        LogError("TofPublisher: Invalid topic or frame id");
        return false;
    }
    const TofGeometry geometry = tof.getGeometry();
    const size_t zone_count = static_cast<size_t>(geometry.rows) * geometry.columns;
    if (geometry.rows == 0U || geometry.columns == 0U || zone_count > TOF_MAX_ZONES ||
        !std::isfinite(geometry.horizontal_fov_rad) || !std::isfinite(geometry.vertical_fov_rad) ||
        geometry.horizontal_fov_rad < 0.0F || geometry.vertical_fov_rad < 0.0F || geometry.horizontal_fov_rad >= TOF_MAX_FOV_RAD ||
        geometry.vertical_fov_rad >= TOF_MAX_FOV_RAD || (geometry.columns > 1U && geometry.horizontal_fov_rad == 0.0F) ||
        (geometry.rows > 1U && geometry.vertical_fov_rad == 0.0F)) {
        LogError("TofPublisher: Invalid geometry for ToF '%s'", tof.getName());
        return false;
    }

    TofSlot& slot = sensors[sensor_count++];
    slot.tof = &tof;
    slot.topic = topic;
    slot.frame_id = frame_id;
    slot.geometry = geometry;
    // A pinhole projection intersects each zone ray with an image plane at
    // z=1. tan(FoV/2) is therefore the plane's half-width or half-height.
    slot.horizontal_scale = std::tan(geometry.horizontal_fov_rad * 0.5F);
    slot.vertical_scale = std::tan(geometry.vertical_fov_rad * 0.5F);
    LogInfo("TofPublisher: Registered ToF '%s' on '%s'", tof.getName(), topic);
    return true;
}

/**
 * @brief Register all PointCloud2 publishers and release the worker thread.
 * @return RCL_RET_OK when publishing can begin, otherwise an rcl error.
 */
rcl_ret_t TofPublisher::start() {
    if (started) {
        return RCL_RET_OK;
    }
    if (getState() == EntityState::UNINITIALIZED || getState() == EntityState::ERROR) {
        return RCL_RET_NOT_INIT;
    }
    if (sensor_count == 0U) {
        LogError("TofPublisher: Cannot start without registered sensors");
        markError(RCL_RET_INVALID_ARGUMENT);
        return RCL_RET_INVALID_ARGUMENT;
    }
    for (size_t i = 0; i < sensor_count; ++i) {
        if (sensors[i].tof == nullptr || sensors[i].tof->getState() != Driver::State::RUNNING) {
            LogError("TofPublisher: ToF '%s' is not running",
                     sensors[i].tof != nullptr ? sensors[i].tof->getName() : "unknown");
            return RCL_RET_NOT_INIT;
        }
    }

    const rcl_ret_t result = initPublishers();
    if (result != RCL_RET_OK) {
        markError(result);
        return result;
    }

    const uint32_t flags = osThreadFlagsSet(thread_id, TOF_PUBLISHER_START_FLAG);
    if ((flags & osFlagsError) != 0U) {
        LogError("TofPublisher: Failed to start publisher thread: 0x%08lX", flags);
        markError(RCL_RET_ERROR);
        return RCL_RET_ERROR;
    }

    started = true;
    return RCL_RET_OK;
}

/**
 * @brief Get the number of registered ToF drivers.
 * @return Registered driver count.
 */
size_t TofPublisher::getTofCount() const { return sensor_count; }

rcl_ret_t TofPublisher::initPublishers() {
    for (size_t i = 0; i < sensor_count; ++i) {
        TofSlot& slot = sensors[i];
        slot.message.header.frame_id.data = const_cast<char*>(slot.frame_id);
        slot.message.header.frame_id.size = std::strlen(slot.frame_id);
        slot.message.header.frame_id.capacity = slot.message.header.frame_id.size + 1U;
        slot.message.height = slot.geometry.rows;
        slot.message.width = slot.geometry.columns;
        slot.message.fields.data = tof_point_fields;
        slot.message.fields.size = TOF_POINT_FIELD_COUNT;
        slot.message.fields.capacity = TOF_POINT_FIELD_COUNT;
        slot.message.is_bigendian = false;
        slot.message.point_step = TOF_POINT_SIZE_BYTES;
        slot.message.row_step = TOF_POINT_SIZE_BYTES * slot.geometry.columns;
        slot.message.data.data = reinterpret_cast<uint8_t*>(tof_point_data);
        slot.message.data.size = TOF_POINT_SIZE_BYTES * slot.geometry.rows * slot.geometry.columns;
        slot.message.data.capacity = sizeof(tof_point_data);
        slot.message.is_dense = false;

        const rcl_ret_t result = slot.publisher.init(*this, slot.topic, config.publisher_config);
        if (result != RCL_RET_OK) {
            LogError("TofPublisher: Failed to register publisher '%s': %d", slot.topic, static_cast<int>(result));
            return result;
        }
        slot.publisher_initialized = true;
    }
    return RCL_RET_OK;
}

void TofPublisher::thread() {
    const uint32_t flags = osThreadFlagsWait(TOF_PUBLISHER_START_FLAG, osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) != 0U) {
        LogError("TofPublisher: Start flag wait failed: 0x%08lX", flags);
        markError(RCL_RET_ERROR);
        osDelay(osWaitForever);
        return;
    }

    LogSuccess("TofPublisher: Started with %u sensor(s)", static_cast<unsigned>(sensor_count));
    while (true) {
        if (client == nullptr) {
            markError(RCL_RET_ERROR);
            osDelay(osWaitForever);
            return;
        }
        if (!client->isConnected()) {
            client->waitForConnection(osWaitForever);
        }

        const uint32_t loop_start_ms = osKernelGetTickCount();
        publishSensors();
        osDelayUntil(loop_start_ms + config.publish_period_ms);
    }
}

void TofPublisher::publishSensors() {
    for (size_t i = 0; i < sensor_count; ++i) {
        publishSensor(sensors[i]);
    }
}

void TofPublisher::publishSensor(TofSlot& slot) {
    if (slot.tof == nullptr || !slot.publisher_initialized) {
        return;
    }
    if (slot.tof->getState() != Driver::State::RUNNING || slot.tof->getConnectionState() != Driver::ConnectionState::CONNECTED) {
        handleReadFailure(slot);
        return;
    }
    if (!slot.tof->isSampleReady()) {
        return;
    }

    TofSample sample{};
    if (!slot.tof->readSample(sample)) {
        handleReadFailure(slot);
        return;
    }

    const size_t expected_zones = static_cast<size_t>(slot.geometry.rows) * slot.geometry.columns;
    if (sample.zone_count != expected_zones) {
        LogError("TofPublisher: ToF '%s' returned %u zones, expected %u",
                 slot.tof->getName(),
                 sample.zone_count,
                 static_cast<unsigned>(expected_zones));
        handleReadFailure(slot);
        return;
    }

    handleReadRecovery(slot);
    fillPointCloud(slot, sample);
    handlePublishResult(slot, slot.publisher.publish(slot.message));
}

void TofPublisher::fillPointCloud(TofSlot& slot, const TofSample& sample) {
    slot.message.header.stamp = client->getRosTime();
    const float invalid = std::numeric_limits<float>::quiet_NaN();

    for (size_t row = 0; row < slot.geometry.rows; ++row) {
        // Use zone centers rather than grid edges and normalize the image plane
        // to [-1, 1]. Positive vertical coordinates point downward according to
        // the ROS optical-frame convention.
        const float vertical = 2.0F * (static_cast<float>(row) + 0.5F) / slot.geometry.rows - 1.0F;
        for (size_t column = 0; column < slot.geometry.columns; ++column) {
            const size_t zone = row * slot.geometry.columns + column;
            const size_t point = zone * TOF_POINT_COMPONENT_COUNT;
            if (!sample.valid[zone]) {
                // Preserve the organized grid instead of removing invalid
                // zones; NaNs let point-cloud consumers identify missing rays.
                tof_point_data[point] = invalid;
                tof_point_data[point + 1U] = invalid;
                tof_point_data[point + 2U] = invalid;
                continue;
            }

            const float horizontal = 2.0F * (static_cast<float>(column) + 0.5F) / slot.geometry.columns - 1.0F;
            const float distance_m = static_cast<float>(sample.distance_mm[zone]) / 1000.0F;
            const float x_ratio = horizontal * slot.horizontal_scale;
            const float y_ratio = vertical * slot.vertical_scale;

            // ToF samples contain depth on the optical z-axis. Intersecting
            // the zone ray (x_ratio, y_ratio, 1) with that depth plane gives
            // the lateral coordinates directly; no ray-length normalization
            // is needed.
            const float z = distance_m;
            tof_point_data[point] = x_ratio * z;
            tof_point_data[point + 1U] = y_ratio * z;
            tof_point_data[point + 2U] = z;
        }
    }
}

void TofPublisher::handleReadFailure(TofSlot& slot) {
    if (slot.consecutive_read_failures < UINT32_MAX) {
        slot.consecutive_read_failures++;
    }
    if (slot.consecutive_read_failures >= config.read_failure_threshold && !slot.read_failure_reported) {
        LogWarning("TofPublisher: ToF '%s' stopped delivering samples", slot.tof->getName());
        slot.read_failure_reported = true;
    }
}

void TofPublisher::handleReadRecovery(TofSlot& slot) {
    if (slot.read_failure_reported) {
        LogInfo("TofPublisher: ToF '%s' samples recovered", slot.tof->getName());
    }
    slot.consecutive_read_failures = 0;
    slot.read_failure_reported = false;
}

void TofPublisher::handlePublishResult(TofSlot& slot, rcl_ret_t result) {
    if (result == RCL_RET_OK) {
        slot.publish_failure_reported = false;
        return;
    }
    if (result == RCL_RET_NOT_INIT || result == RCL_RET_TIMEOUT) {
        return;
    }
    if (!slot.publish_failure_reported) {
        LogWarning("TofPublisher: Failed to publish '%s': %d", slot.topic, static_cast<int>(result));
        slot.publish_failure_reported = true;
    }
}
