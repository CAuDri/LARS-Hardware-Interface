/**
 * @file service.cpp
 *
 * @brief CAuDri - RCL lifecycle for reconnectable micro-ROS services
 */

#include "service.hpp"

#include "client.hpp"
#include "logger.h"

namespace ros {

/**
 * @brief Construct an unconfigured service wrapper.
 */
BaseService::BaseService() { rcl_service = rcl_get_zero_initialized_service(); }

/**
 * @brief Configure and register a service for client-managed rclc initialization.
 * @param parent_node Configured parent node.
 * @param name ROS service name relative to the parent node namespace or absolute.
 * @param service_type_support Generated ROS service type support.
 * @param service_request_storage Request instance used by the executor.
 * @param service_response_storage Response instance used by the executor.
 * @param dispatch_function Static dispatch function for the typed callback.
 * @param service_config Service QoS configuration.
 * @return RCL_RET_OK when the service was registered, otherwise an rcl error code.
 */
rcl_ret_t BaseService::init(Node& parent_node,
                            const char* name,
                            const rosidl_service_type_support_t* service_type_support,
                            void* service_request_storage,
                            void* service_response_storage,
                            DispatchFunction dispatch_function,
                            const Config& service_config) {
    if (state != EntityState::UNINITIALIZED) {
        return RCL_RET_ALREADY_INIT;
    }
    if (name == nullptr || name[0] == '\0' || service_type_support == nullptr || service_request_storage == nullptr ||
        service_response_storage == nullptr || dispatch_function == nullptr || parent_node.getClient() == nullptr) {
        state = EntityState::ERROR;
        last_error = RCL_RET_INVALID_ARGUMENT;
        LogError("micro-ROS Service: Invalid arguments for '%s'", name == nullptr ? "<null>" : name);
        return last_error;
    }

    node = &parent_node;
    service_name = name;
    type_support = service_type_support;
    request_storage = service_request_storage;
    response_storage = service_response_storage;
    dispatch = dispatch_function;
    config = service_config;
    connection_state = ConnectionState::DISCONNECTED;

    Client* client = node->getClient();
    const rcl_ret_t result = client->registerService(this);
    if (result != RCL_RET_OK) {
        state = EntityState::ERROR;
        last_error = result;
        LogError("micro-ROS Service: Failed to register '%s': %d", service_name, (int)result);
        return result;
    }

    registered = true;
    state = EntityState::INITIALIZED;
    last_error = RCL_RET_OK;
    LogDebug("micro-ROS Service: Registered '%s'", service_name);
    return RCL_RET_OK;
}

/**
 * @brief Finalize rclc service state and unregister the wrapper from its client.
 * @return RCL_RET_OK on success, otherwise the first rcl error code.
 */
rcl_ret_t BaseService::fini() {
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
    result = finiRclcService();
    const rcl_ret_t unregister_result = client->unregisterService(this);
    client->unlockSession();

    registered = false;
    node = nullptr;
    service_name = nullptr;
    type_support = nullptr;
    request_storage = nullptr;
    response_storage = nullptr;
    dispatch = nullptr;
    state = EntityState::UNINITIALIZED;
    connection_state = ConnectionState::UNKNOWN;
    return result == RCL_RET_OK ? unregister_result : result;
}

/**
 * @brief Get the service lifecycle state.
 * @return Current wrapper lifecycle state.
 */
EntityState BaseService::getState() const { return state; }

/**
 * @brief Get the most recent service lifecycle error.
 * @return Most recent rcl/rclc lifecycle error.
 */
rcl_ret_t BaseService::getLastError() const { return last_error; }

/**
 * @brief Get the configured ROS service name.
 * @return Configured ROS service name, or nullptr before init().
 */
const char* BaseService::getServiceName() const { return service_name; }

/**
 * @brief Check whether the rclc service has been created for the active session.
 * @return true while rcl_service_t is valid for the active session.
 */
bool BaseService::isActive() const { return rcl_active; }

void BaseService::executorCallback(const void* request, void* response, void* context) {
    auto* service = static_cast<BaseService*>(context);
    if (service != nullptr && service->dispatch != nullptr && request != nullptr && response != nullptr) {
        service->dispatch(service, request, response);
    }
}

rcl_ret_t BaseService::initRclcService() {
    if (state == EntityState::UNINITIALIZED || node == nullptr || type_support == nullptr || request_storage == nullptr ||
        response_storage == nullptr) {
        return RCL_RET_NOT_INIT;
    }
    if (rcl_active) {
        return RCL_RET_OK;
    }

    rcl_node_t* rcl_node = node->getRclcNode();
    if (rcl_node == nullptr || node->getClient() == nullptr) {
        return RCL_RET_NOT_INIT;
    }

    rcl_service = rcl_get_zero_initialized_service();
    rcl_ret_t result = config.best_effort
                           ? rclc_service_init_best_effort(&rcl_service, rcl_node, type_support, service_name)
                           : rclc_service_init_default(&rcl_service, rcl_node, type_support, service_name);
    last_error = result;
    if (result != RCL_RET_OK) {
        connection_state = ConnectionState::DISCONNECTED;
        LogWarning("micro-ROS Service: Failed to create rclc service '%s': %d", service_name, (int)result);
        return result;
    }

    result = node->getClient()->getExecutor().addService(
        &rcl_service, request_storage, response_storage, &BaseService::executorCallback, this);
    last_error = result;
    if (result != RCL_RET_OK) {
        const rcl_ret_t fini_result = rcl_service_fini(&rcl_service, rcl_node);
        (void)fini_result;
        rcl_service = rcl_get_zero_initialized_service();
        LogWarning("micro-ROS Service: Failed to add '%s' to executor: %d", service_name, (int)result);
        return result;
    }

    executor_registered = true;
    rcl_active = true;
    state = EntityState::RUNNING;
    connection_state = ConnectionState::CONNECTED;
    LogDebug("micro-ROS Service: rclc service '%s' initialized", service_name);
    return RCL_RET_OK;
}

rcl_ret_t BaseService::finiRclcService() {
    if (!rcl_active) {
        state = registered ? EntityState::INITIALIZED : EntityState::UNINITIALIZED;
        connection_state = registered ? ConnectionState::DISCONNECTED : ConnectionState::UNKNOWN;
        rcl_service = rcl_get_zero_initialized_service();
        executor_registered = false;
        return RCL_RET_OK;
    }

    rcl_ret_t result = RCL_RET_OK;
    if (node != nullptr && node->getClient() != nullptr && executor_registered) {
        result = node->getClient()->getExecutor().removeService(&rcl_service);
    }

    rcl_node_t* rcl_node = node == nullptr ? nullptr : node->getRclcNode();
    if (rcl_node != nullptr) {
        const rcl_ret_t fini_result = rcl_service_fini(&rcl_service, rcl_node);
        if (result == RCL_RET_OK) {
            result = fini_result;
        }
    }

    rcl_service = rcl_get_zero_initialized_service();
    executor_registered = false;
    rcl_active = false;
    state = registered ? EntityState::INITIALIZED : EntityState::UNINITIALIZED;
    connection_state = registered ? ConnectionState::DISCONNECTED : ConnectionState::UNKNOWN;
    last_error = result;
    if (result != RCL_RET_OK) {
        LogWarning("micro-ROS Service: rclc cleanup returned for '%s': %d", service_name, (int)result);
    }
    return result;
}

}  // namespace ros
