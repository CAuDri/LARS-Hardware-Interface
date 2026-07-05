/**
 * @file publisher.cpp
 *
 * @brief CAuDri - RCL lifecycle for reconnectable micro-ROS publishers
 */

#include "publisher.hpp"

#include "client.hpp"
#include "logger.h"

namespace ros {

/**
 * @brief Construct an unconfigured publisher wrapper.
 */
BasePublisher::BasePublisher() { rcl_publisher = rcl_get_zero_initialized_publisher(); }

/**
 * @brief Configure and register a publisher for client-managed rclc initialization.
 * @param parent_node Configured parent node.
 * @param topic ROS topic name relative to the parent node namespace or absolute.
 * @param message_type_support Generated ROS message type support.
 * @param publisher_config Publisher QoS and synchronization configuration.
 * @return RCL_RET_OK when the publisher was registered, otherwise an rcl error code.
 */
rcl_ret_t BasePublisher::init(Node& parent_node,
                              const char* topic,
                              const rosidl_message_type_support_t* message_type_support,
                              const Config& publisher_config) {
    if (state != EntityState::UNINITIALIZED) {
        return RCL_RET_ALREADY_INIT;
    }
    if (topic == nullptr || topic[0] == '\0' || message_type_support == nullptr || parent_node.getClient() == nullptr) {
        state = EntityState::ERROR;
        last_error = RCL_RET_INVALID_ARGUMENT;
        LogError("micro-ROS Publisher: Invalid arguments for '%s'", topic == nullptr ? "<null>" : topic);
        return last_error;
    }

    node = &parent_node;
    topic_name = topic;
    type_support = message_type_support;
    config = publisher_config;
    connection_state = ConnectionState::DISCONNECTED;

    Client* client = node->getClient();
    const rcl_ret_t result = client->registerPublisher(this);
    if (result != RCL_RET_OK) {
        state = EntityState::ERROR;
        last_error = result;
        LogError("micro-ROS Publisher: Failed to register '%s': %d", topic_name, (int)result);
        return result;
    }

    registered = true;
    state = EntityState::INITIALIZED;
    last_error = RCL_RET_OK;
    LogDebug("micro-ROS Publisher: Registered '%s'", topic_name);
    return RCL_RET_OK;
}

/**
 * @brief Finalize rclc publisher state and unregister the wrapper from its client.
 * @return RCL_RET_OK on success, otherwise the first rcl error code.
 */
rcl_ret_t BasePublisher::fini() {
    if (state == EntityState::UNINITIALIZED) {
        return RCL_RET_OK;
    }
    if (node == nullptr || node->getClient() == nullptr) {
        return RCL_RET_NOT_INIT;
    }

    Client* client = node->getClient();
    rcl_ret_t result = client->lockSession(osWaitForever);
    if (result != RCL_RET_OK) {
        return result;
    }
    result = finiRclcPublisher();
    const rcl_ret_t unregister_result = client->unregisterPublisher(this);
    client->unlockSession();

    registered = false;
    node = nullptr;
    topic_name = nullptr;
    type_support = nullptr;
    state = EntityState::UNINITIALIZED;
    connection_state = ConnectionState::UNKNOWN;
    return result == RCL_RET_OK ? unregister_result : result;
}

/**
 * @brief Get the publisher lifecycle state.
 * @return Current wrapper lifecycle state.
 */
EntityState BasePublisher::getState() const { return state; }

/**
 * @brief Get the most recent publisher lifecycle or publish error.
 * @return Most recent rcl/rclc lifecycle or publish error.
 */
rcl_ret_t BasePublisher::getLastError() const { return last_error; }

/**
 * @brief Get the configured ROS topic name.
 * @return Configured ROS topic name, or nullptr before init().
 */
const char* BasePublisher::getTopicName() const { return topic_name; }

/**
 * @brief Check whether the rclc publisher has been created for the active session.
 * @return true while rcl_publisher_t is valid for the active session.
 */
bool BasePublisher::isActive() const { return rcl_active; }

rcl_ret_t BasePublisher::publishRaw(const void* message) {
    if (message == nullptr || node == nullptr || node->getClient() == nullptr) {
        return RCL_RET_INVALID_ARGUMENT;
    }
    if (!rcl_active || !node->isActive() || !node->getClient()->isConnected()) {
        return RCL_RET_NOT_INIT;
    }

    Client* client = node->getClient();
    rcl_ret_t result = client->lockSession(config.publish_mutex_timeout_ms);
    if (result != RCL_RET_OK) {
        last_error = result;
        return result;
    }

    if (!rcl_active || !node->isActive() || !client->isConnected()) {
        client->unlockSession();
        return RCL_RET_NOT_INIT;
    }

    client->unlockSession();

    result = rcl_publish(&rcl_publisher, message, nullptr);

    last_error = result;
    if (result != RCL_RET_OK) {
        client->signalPossibleDisconnect(result);
    }
    return result;
}

rcl_ret_t BasePublisher::initRclcPublisher() {
    if (state == EntityState::UNINITIALIZED || node == nullptr || type_support == nullptr) {
        return RCL_RET_NOT_INIT;
    }
    if (rcl_active) {
        return RCL_RET_OK;
    }

    rcl_node_t* rcl_node = node->getRclcNode();
    if (rcl_node == nullptr) {
        return RCL_RET_NOT_INIT;
    }

    rcl_publisher = rcl_get_zero_initialized_publisher();
    const rcl_ret_t result = config.best_effort
                                 ? rclc_publisher_init_best_effort(&rcl_publisher, rcl_node, type_support, topic_name)
                                 : rclc_publisher_init_default(&rcl_publisher, rcl_node, type_support, topic_name);
    last_error = result;
    if (result != RCL_RET_OK) {
        connection_state = ConnectionState::DISCONNECTED;
        LogWarning("micro-ROS Publisher: Failed to create rclc publisher '%s': %d", topic_name, (int)result);
        return result;
    }

    rcl_active = true;
    state = EntityState::RUNNING;
    connection_state = ConnectionState::CONNECTED;
    LogDebug("micro-ROS Publisher: rclc publisher '%s' initialized", topic_name);
    return RCL_RET_OK;
}

rcl_ret_t BasePublisher::finiRclcPublisher() {
    if (!rcl_active) {
        state = registered ? EntityState::INITIALIZED : EntityState::UNINITIALIZED;
        connection_state = registered ? ConnectionState::DISCONNECTED : ConnectionState::UNKNOWN;
        rcl_publisher = rcl_get_zero_initialized_publisher();
        return RCL_RET_OK;
    }

    rcl_ret_t result = RCL_RET_OK;
    rcl_node_t* rcl_node = node == nullptr ? nullptr : node->getRclcNode();
    if (rcl_node != nullptr) {
        result = rcl_publisher_fini(&rcl_publisher, rcl_node);
    }
    rcl_publisher = rcl_get_zero_initialized_publisher();
    rcl_active = false;
    state = registered ? EntityState::INITIALIZED : EntityState::UNINITIALIZED;
    connection_state = registered ? ConnectionState::DISCONNECTED : ConnectionState::UNKNOWN;
    last_error = result;
    if (result != RCL_RET_OK) {
        LogWarning("micro-ROS Publisher: rclc cleanup returned for '%s': %d", topic_name, (int)result);
    }
    return result;
}

}  // namespace ros
