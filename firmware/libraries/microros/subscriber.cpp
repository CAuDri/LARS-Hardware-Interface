/**
 * @file subscriber.cpp
 *
 * @brief CAuDri - RCL lifecycle for reconnectable micro-ROS subscribers
 */

#include "subscriber.hpp"

#include "client.hpp"
#include "logger.h"

namespace ros {

/**
 * @brief Construct an unconfigured subscriber wrapper.
 */
BaseSubscriber::BaseSubscriber() { rcl_subscription = rcl_get_zero_initialized_subscription(); }

/**
 * @brief Configure and register a subscriber for client-managed rclc initialization.
 * @param parent_node Configured parent node.
 * @param topic ROS topic name relative to the parent node namespace or absolute.
 * @param message_type_support Generated ROS message type support.
 * @param subscriber_message_storage Message instance used by the executor.
 * @param dispatch_function Static dispatch function for the typed callback.
 * @param subscriber_config Subscriber QoS and executor invocation configuration.
 * @return RCL_RET_OK when the subscriber was registered, otherwise an rcl error code.
 */
rcl_ret_t BaseSubscriber::init(Node& parent_node,
                               const char* topic,
                               const rosidl_message_type_support_t* message_type_support,
                               void* subscriber_message_storage,
                               DispatchFunction dispatch_function,
                               const Config& subscriber_config) {
    if (state != EntityState::UNINITIALIZED) {
        return RCL_RET_ALREADY_INIT;
    }
    if (topic == nullptr || topic[0] == '\0' || message_type_support == nullptr || subscriber_message_storage == nullptr ||
        dispatch_function == nullptr || parent_node.getClient() == nullptr) {
        state = EntityState::ERROR;
        last_error = RCL_RET_INVALID_ARGUMENT;
        return last_error;
    }

    node = &parent_node;
    topic_name = topic;
    type_support = message_type_support;
    message_storage = subscriber_message_storage;
    dispatch = dispatch_function;
    config = subscriber_config;
    connection_state = ConnectionState::DISCONNECTED;

    Client* client = node->getClient();
    const rcl_ret_t result = client->registerSubscriber(this);
    if (result != RCL_RET_OK) {
        state = EntityState::ERROR;
        last_error = result;
        return result;
    }

    registered = true;
    state = EntityState::INITIALIZED;
    last_error = RCL_RET_OK;
    LogDebug("micro-ROS Subscriber: Registered '%s'", topic_name);
    return RCL_RET_OK;
}

/**
 * @brief Finalize rclc subscriber state and unregister the wrapper from its client.
 * @return RCL_RET_OK on success, otherwise the first rcl error code.
 */
rcl_ret_t BaseSubscriber::fini() {
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
    result = finiRclcSubscriber();
    const rcl_ret_t unregister_result = client->unregisterSubscriber(this);
    client->unlockSession();

    registered = false;
    node = nullptr;
    topic_name = nullptr;
    type_support = nullptr;
    message_storage = nullptr;
    dispatch = nullptr;
    state = EntityState::UNINITIALIZED;
    connection_state = ConnectionState::UNKNOWN;
    return result == RCL_RET_OK ? unregister_result : result;
}

/**
 * @brief Get the subscriber lifecycle state.
 * @return Current wrapper lifecycle state.
 */
EntityState BaseSubscriber::getState() const { return state; }

/**
 * @brief Get the most recent subscriber lifecycle error.
 * @return Most recent rcl/rclc lifecycle error.
 */
rcl_ret_t BaseSubscriber::getLastError() const { return last_error; }

/**
 * @brief Get the configured ROS topic name.
 * @return Configured ROS topic name, or nullptr before init().
 */
const char* BaseSubscriber::getTopicName() const { return topic_name; }

/**
 * @brief Check whether the rclc subscriber has been created for the active session.
 * @return true while rcl_subscription_t is valid for the active session.
 */
bool BaseSubscriber::isActive() const { return rcl_active; }

void BaseSubscriber::executorCallback(const void* message, void* context) {
    auto* subscriber = static_cast<BaseSubscriber*>(context);
    if (subscriber != nullptr && subscriber->dispatch != nullptr && message != nullptr) {
        subscriber->dispatch(subscriber, message);
    }
}

rcl_ret_t BaseSubscriber::initRclcSubscriber() {
    if (state == EntityState::UNINITIALIZED || node == nullptr || type_support == nullptr || message_storage == nullptr) {
        return RCL_RET_NOT_INIT;
    }
    if (rcl_active) {
        return RCL_RET_OK;
    }

    rcl_node_t* rcl_node = node->getRclcNode();
    if (rcl_node == nullptr || node->getClient() == nullptr) {
        return RCL_RET_NOT_INIT;
    }

    rcl_subscription = rcl_get_zero_initialized_subscription();
    rcl_ret_t result = config.best_effort
                           ? rclc_subscription_init_best_effort(&rcl_subscription, rcl_node, type_support, topic_name)
                           : rclc_subscription_init_default(&rcl_subscription, rcl_node, type_support, topic_name);
    last_error = result;
    if (result != RCL_RET_OK) {
        connection_state = ConnectionState::DISCONNECTED;
        LogWarning("micro-ROS Subscriber: Failed to create rclc subscriber '%s': %d", topic_name, (int)result);
        return result;
    }

    result = node->getClient()->getExecutor().addSubscription(
        &rcl_subscription, message_storage, &BaseSubscriber::executorCallback, this, config.invocation);
    last_error = result;
    if (result != RCL_RET_OK) {
        const rcl_ret_t fini_result = rcl_subscription_fini(&rcl_subscription, rcl_node);
        (void)fini_result;
        rcl_subscription = rcl_get_zero_initialized_subscription();
        LogWarning("micro-ROS Subscriber: Failed to add '%s' to executor: %d", topic_name, (int)result);
        return result;
    }

    executor_registered = true;
    rcl_active = true;
    state = EntityState::RUNNING;
    connection_state = ConnectionState::CONNECTED;
    LogDebug("micro-ROS Subscriber: rclc subscriber '%s' initialized", topic_name);
    return RCL_RET_OK;
}

rcl_ret_t BaseSubscriber::finiRclcSubscriber() {
    if (!rcl_active) {
        state = registered ? EntityState::INITIALIZED : EntityState::UNINITIALIZED;
        connection_state = registered ? ConnectionState::DISCONNECTED : ConnectionState::UNKNOWN;
        rcl_subscription = rcl_get_zero_initialized_subscription();
        executor_registered = false;
        return RCL_RET_OK;
    }

    rcl_ret_t result = RCL_RET_OK;
    if (node != nullptr && node->getClient() != nullptr && executor_registered) {
        result = node->getClient()->getExecutor().removeSubscription(&rcl_subscription);
    }

    rcl_node_t* rcl_node = node == nullptr ? nullptr : node->getRclcNode();
    if (rcl_node != nullptr) {
        const rcl_ret_t fini_result = rcl_subscription_fini(&rcl_subscription, rcl_node);
        if (result == RCL_RET_OK) {
            result = fini_result;
        }
    }

    rcl_subscription = rcl_get_zero_initialized_subscription();
    executor_registered = false;
    rcl_active = false;
    state = registered ? EntityState::INITIALIZED : EntityState::UNINITIALIZED;
    connection_state = registered ? ConnectionState::DISCONNECTED : ConnectionState::UNKNOWN;
    last_error = result;
    if (result != RCL_RET_OK) {
        LogWarning("micro-ROS Subscriber: rclc cleanup returned for '%s': %d", topic_name, (int)result);
    }
    return result;
}

}  // namespace ros
