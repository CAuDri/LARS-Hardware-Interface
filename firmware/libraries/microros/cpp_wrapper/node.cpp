/**
 * @file node.cpp
 *
 * @brief CAuDri - RCL lifecycle for reconnectable micro-ROS nodes
 */

#include "node.hpp"

#include <cstdio>
#include <cstring>

#include "client.hpp"
#include "logger.h"

namespace ros {

/**
 * @brief Construct an unconfigured node wrapper.
 */
Node::Node() { rcl_node = rcl_get_zero_initialized_node(); }

/**
 * @brief Configure and register a node for client-managed rclc initialization.
 * @param node_client Client that owns the session this node belongs to.
 * @param node_name ROS node name.
 * @param node_namespace ROS node namespace; use an empty string for the root namespace.
 * @return RCL_RET_OK when the node was registered, otherwise an rcl error code.
 */
rcl_ret_t Node::init(Client& node_client, const char* node_name, const char* node_namespace) {
    trace::initNode(trace_state, node_name);
    if (state != EntityState::UNINITIALIZED) {
        trace::incrementErrors();
        return RCL_RET_ALREADY_INIT;
    }
    if (node_name == nullptr || node_name[0] == '\0' || node_namespace == nullptr) {
        state = EntityState::ERROR;
        trace::setNodeState(trace_state, state);
        trace::incrementErrors();
        last_error = RCL_RET_INVALID_ARGUMENT;
        return last_error;
    }

    client = &node_client;
    name = node_name;
    local_namespace_name = node_namespace;
    const rcl_ret_t namespace_result = composeNamespace();
    if (namespace_result != RCL_RET_OK) {
        state = EntityState::ERROR;
        trace::setNodeState(trace_state, state);
        trace::incrementErrors();
        last_error = namespace_result;
        return namespace_result;
    }
    connection_state = ConnectionState::DISCONNECTED;

    const rcl_ret_t result = client->registerNode(this);
    if (result != RCL_RET_OK) {
        state = EntityState::ERROR;
        trace::setNodeState(trace_state, state);
        trace::incrementErrors();
        last_error = result;
        return result;
    }

    registered = true;
    state = EntityState::INITIALIZED;
    trace::setNodeState(trace_state, state);
    last_error = RCL_RET_OK;
    LogDebug("micro-ROS Node: Registered '%s'", name);
    return RCL_RET_OK;
}

/**
 * @brief Finalize rclc node state and unregister the wrapper from its client.
 * @return RCL_RET_OK on success, otherwise the first rcl error code.
 */
rcl_ret_t Node::fini() {
    if (state == EntityState::UNINITIALIZED) {
        return RCL_RET_OK;
    }
    if (client == nullptr) {
        return RCL_RET_NOT_INIT;
    }

    rcl_ret_t result = client->lockSession(osWaitForever);
    if (result != RCL_RET_OK) {
        return result;
    }
    result = finiRclcNode();
    const rcl_ret_t unregister_result = client->unregisterNode(this);
    client->unlockSession();

    registered = false;
    client = nullptr;
    name = nullptr;
    local_namespace_name = "";
    namespace_name[0] = '\0';
    state = EntityState::UNINITIALIZED;
    trace::setNodeState(trace_state, state);
    connection_state = ConnectionState::UNKNOWN;
    return result == RCL_RET_OK ? unregister_result : result;
}

/**
 * @brief Get the wrapper lifecycle state.
 * @return Current wrapper lifecycle state.
 */
EntityState Node::getState() const { return state; }

/**
 * @brief Get the node connection state for the active session.
 * @return Current rcl-session connection state.
 */
ConnectionState Node::getConnectionState() const { return connection_state; }

/**
 * @brief Get the most recent node lifecycle error.
 * @return Most recent rcl/rclc lifecycle error.
 */
rcl_ret_t Node::getLastError() const { return last_error; }

/**
 * @brief Get the configured ROS node name.
 * @return Configured ROS node name, or nullptr before init().
 */
const char* Node::getName() const { return name; }

/**
 * @brief Get the resolved ROS namespace.
 * @return Client base namespace and node namespace joined together, or an empty string before init().
 */
const char* Node::getNamespace() const { return namespace_name.data(); }

/**
 * @brief Check whether the rclc node has been created for the active session.
 * @return true while rcl_node_t is valid for the active session.
 */
bool Node::isActive() const { return rcl_active; }

rcl_ret_t Node::composeNamespace() {
    const char* base_namespace = client == nullptr ? "" : client->getBaseNamespace();
    const char* local_namespace = local_namespace_name == nullptr ? "" : local_namespace_name;

    const size_t base_length = std::strlen(base_namespace);
    const size_t local_length = std::strlen(local_namespace);
    if (base_length == 0U && local_length == 0U) {
        namespace_name[0] = '\0';
        return RCL_RET_OK;
    }

    auto trimTrailingSlashes = [](const char* text, size_t length) -> size_t {
        size_t end = length;
        while (end > 0U && text[end - 1U] == '/') {
            end--;
        }
        return end;
    };

    const size_t base_start = 0U;
    const size_t base_end = trimTrailingSlashes(base_namespace, base_length);
    size_t local_start = 0U;
    const size_t local_end = trimTrailingSlashes(local_namespace, local_length);

    if (base_end > base_start && local_end > 0U && local_namespace[0] == '/') {
        local_start = 1U;
    }

    const bool has_base = base_end > base_start;
    const bool has_local = local_end > local_start;
    if (!has_base && !has_local) {
        namespace_name[0] = '/';
        namespace_name[1] = '\0';
        return RCL_RET_OK;
    }

    int written = 0;
    if (has_base && has_local) {
        written = std::snprintf(namespace_name.data(),
                                namespace_name.size(),
                                "%.*s/%.*s",
                                static_cast<int>(base_end - base_start),
                                base_namespace + base_start,
                                static_cast<int>(local_end - local_start),
                                local_namespace + local_start);
    } else if (has_base) {
        written = std::snprintf(namespace_name.data(),
                                namespace_name.size(),
                                "%.*s",
                                static_cast<int>(base_end - base_start),
                                base_namespace + base_start);
    } else {
        written = std::snprintf(namespace_name.data(),
                                namespace_name.size(),
                                "%.*s",
                                static_cast<int>(local_end - local_start),
                                local_namespace + local_start);
    }

    if (written < 0 || static_cast<size_t>(written) >= namespace_name.size()) {
        namespace_name[0] = '\0';
        return RCL_RET_INVALID_ARGUMENT;
    }
    return RCL_RET_OK;
}

rcl_ret_t Node::initRclcNode(rclc_support_t* support) {
    if (state == EntityState::UNINITIALIZED || support == nullptr) {
        return RCL_RET_NOT_INIT;
    }
    if (rcl_active) {
        return RCL_RET_OK;
    }

    rcl_node = rcl_get_zero_initialized_node();
    const rcl_ret_t result = rclc_node_init_default(&rcl_node, name, namespace_name.data(), support);
    last_error = result;
    if (result != RCL_RET_OK) {
        connection_state = ConnectionState::DISCONNECTED;
        trace::setNodeState(trace_state, state);
        LogWarning("micro-ROS Node: Failed to create rclc node '%s': %d", name, (int)result);
        trace::incrementErrors();
        return result;
    }

    rcl_active = true;
    state = EntityState::RUNNING;
    trace::setNodeState(trace_state, state);
    trace::incrementActiveNodeCount();
    connection_state = ConnectionState::CONNECTED;
    LogDebug("micro-ROS Node: rclc node '%s' initialized", name);
    return RCL_RET_OK;
}

rcl_ret_t Node::finiRclcNode() {
    if (!rcl_active) {
        state = registered ? EntityState::INITIALIZED : EntityState::UNINITIALIZED;
        trace::setNodeState(trace_state, state);
        connection_state = registered ? ConnectionState::DISCONNECTED : ConnectionState::UNKNOWN;
        rcl_node = rcl_get_zero_initialized_node();
        return RCL_RET_OK;
    }

    const rcl_ret_t result = rcl_node_fini(&rcl_node);
    rcl_node = rcl_get_zero_initialized_node();
    rcl_active = false;
    trace::decrementActiveNodeCount();
    state = registered ? EntityState::INITIALIZED : EntityState::UNINITIALIZED;
    trace::setNodeState(trace_state, state);
    connection_state = registered ? ConnectionState::DISCONNECTED : ConnectionState::UNKNOWN;
    last_error = result;
    if (result != RCL_RET_OK) {
        LogWarning("micro-ROS Node: rclc cleanup returned for '%s': %d", name, (int)result);
        trace::incrementErrors();
    }
    return result;
}

rcl_node_t* Node::getRclcNode() { return rcl_active ? &rcl_node : nullptr; }

Client* Node::getClient() { return client; }

}  // namespace ros
