/**
 * @file publisher.hpp
 *
 * @brief CAuDri - Reconnectable micro-ROS publisher wrapper
 */
#pragma once

#include <rcl/publisher.h>
#include <rcl/rcl.h>
#include <rclc/publisher.h>

#include <cstdint>

#include "state.hpp"
#include "node.hpp"
#include "type_support.hpp"

namespace ros {

class BasePublisher {
   public:
    struct Config {
        bool best_effort = false;
        uint32_t publish_mutex_timeout_ms = 0;
    };

    BasePublisher();
    ~BasePublisher() = default;
    BasePublisher(const BasePublisher&) = delete;
    BasePublisher& operator=(const BasePublisher&) = delete;

    rcl_ret_t init(Node& parent_node,
                   const char* topic,
                   const rosidl_message_type_support_t* message_type_support,
                   const Config& publisher_config);
    rcl_ret_t fini();

    EntityState getState() const;
    rcl_ret_t getLastError() const;
    const char* getTopicName() const;
    bool isActive() const;

   protected:
    rcl_ret_t publishRaw(const void* message);

   private:
    friend class Client;

    Node* node = nullptr;
    const char* topic_name = nullptr;
    const rosidl_message_type_support_t* type_support = nullptr;
    Config config{};
    rcl_publisher_t rcl_publisher{};
    volatile EntityState state = EntityState::UNINITIALIZED;
    volatile ConnectionState connection_state = ConnectionState::UNKNOWN;
    rcl_ret_t last_error = RCL_RET_OK;
    bool registered = false;
    bool rcl_active = false;

    rcl_ret_t initRclcPublisher();
    rcl_ret_t finiRclcPublisher();
};

template <typename Message>
class Publisher : public BasePublisher {
   public:
    rcl_ret_t init(Node& parent_node, const char* topic, const Config& publisher_config = Config()) {
        return BasePublisher::init(parent_node, topic, MessageTypeSupport<Message>::get(), publisher_config);
    }

    rcl_ret_t publish(const Message& message) { return publishRaw(&message); }
    rcl_ret_t publish(const Message* message) { return publishRaw(message); }
};

}  // namespace ros
