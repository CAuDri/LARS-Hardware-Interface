/**
 * @file node.hpp
 *
 * @brief CAuDri - Reconnectable micro-ROS node wrapper
 */
#pragma once

#include <cstddef>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw/validate_namespace.h>

#include <array>

#include "microros_trace.hpp"
#include "state.hpp"

constexpr size_t ROS_NODE_NAMESPACE_BUFFER_SIZE = RMW_NAMESPACE_MAX_LENGTH + 1U;

namespace ros {

class Client;

class Node {
   public:
    Node();
    ~Node() = default;
    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    rcl_ret_t init(Client& client, const char* name, const char* namespace_name = "");
    rcl_ret_t fini();

    EntityState getState() const;
    ConnectionState getConnectionState() const;
    rcl_ret_t getLastError() const;
    const char* getName() const;
    const char* getNamespace() const;
    bool isActive() const;

   private:
    friend class Client;
    friend class BasePublisher;
    friend class BaseSubscriber;
    friend class BaseService;

    Client* client = nullptr;
    const char* name = nullptr;
    const char* local_namespace_name = "";
    std::array<char, ROS_NODE_NAMESPACE_BUFFER_SIZE> namespace_name{};
    rcl_node_t rcl_node{};
    volatile EntityState state = EntityState::UNINITIALIZED;
    volatile ConnectionState connection_state = ConnectionState::UNKNOWN;
    rcl_ret_t last_error = RCL_RET_OK;
    bool registered = false;
    bool rcl_active = false;
    trace::StateMachine trace_state{};

    rcl_ret_t composeNamespace();
    rcl_ret_t initRclcNode(rclc_support_t* support);
    rcl_ret_t finiRclcNode();
    rcl_node_t* getRclcNode();
    Client* getClient();
};

}  // namespace ros
