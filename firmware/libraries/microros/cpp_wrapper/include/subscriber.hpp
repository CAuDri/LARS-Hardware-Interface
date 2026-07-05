/**
 * @file subscriber.hpp
 *
 * @brief CAuDri - Reconnectable micro-ROS subscriber wrapper
 */
#pragma once

#include <rcl/rcl.h>
#include <rcl/subscription.h>
#include <rclc/executor.h>
#include <rclc/subscription.h>

#include "state.hpp"
#include "node.hpp"
#include "type_support.hpp"

namespace ros {

/**
 * @brief Placeholder type used when a subscriber uses a free/static callback.
 */
class NoCallbackClass {};

class BaseSubscriber {
   public:
    struct Config {
        bool best_effort = false;
        rclc_executor_handle_invocation_t invocation = ON_NEW_DATA;
    };

    BaseSubscriber();
    ~BaseSubscriber() = default;
    BaseSubscriber(const BaseSubscriber&) = delete;
    BaseSubscriber& operator=(const BaseSubscriber&) = delete;

    rcl_ret_t fini();

    EntityState getState() const;
    rcl_ret_t getLastError() const;
    const char* getTopicName() const;
    bool isActive() const;

   protected:
    using DispatchFunction = void (*)(BaseSubscriber* subscriber, const void* message);

    rcl_ret_t init(Node& parent_node,
                   const char* topic,
                   const rosidl_message_type_support_t* message_type_support,
                   void* message_storage,
                   DispatchFunction dispatch_function,
                   const Config& subscriber_config);

   private:
    friend class Client;

    Node* node = nullptr;
    const char* topic_name = nullptr;
    const rosidl_message_type_support_t* type_support = nullptr;
    void* message_storage = nullptr;
    DispatchFunction dispatch = nullptr;
    Config config{};
    rcl_subscription_t rcl_subscription{};
    volatile EntityState state = EntityState::UNINITIALIZED;
    volatile ConnectionState connection_state = ConnectionState::UNKNOWN;
    rcl_ret_t last_error = RCL_RET_OK;
    bool registered = false;
    bool rcl_active = false;
    bool executor_registered = false;

    static void executorCallback(const void* message, void* context);
    rcl_ret_t initRclcSubscriber();
    rcl_ret_t finiRclcSubscriber();
};

/**
 * @brief Typed subscriber with allocation-free callback storage.
 *
 * Callback execution happens in the micro-ROS executor thread. Keep callbacks
 * short, avoid blocking operations, and do not call entity init()/fini() from a
 * callback. Publishing from a callback is supported by the recursive session
 * mutex. The message pointer is valid only during the callback.
 */
template <typename Message, typename CallbackClass = NoCallbackClass>
class Subscriber : public BaseSubscriber {
   public:
    using Callback = void (*)(const Message* message, void* context);

    /**
     * @brief Configure a subscriber with a free or static callback.
     *
     * Use this overload for plain C-style callbacks. The optional context
     * pointer is stored unchanged and passed back on every callback, which is
     * useful for connecting the callback to existing driver/controller objects
     * without dynamic allocation.
     *
     * @code
     * static void onModeCommand(const std_msgs__msg__UInt8* msg, void* context) {
     *     (void)context;
     *     LogInfo("Mode command: %u", msg->data);
     * }
     *
     * ros::Subscriber<std_msgs__msg__UInt8> mode_subscriber;
     * mode_subscriber.init(hardware_node, "command/mode", onModeCommand);
     * @endcode
     *
     * @code
     * struct CommandContext {
     *     DriveController* drive_controller;
     * };
     *
     * static void onDriveCommand(const std_msgs__msg__UInt8* msg, void* context) {
     *     auto* command_context = static_cast<CommandContext*>(context);
     *     command_context->drive_controller->setModeFromRos(msg->data);
     * }
     *
     * CommandContext command_context{&drive_controller};
     * drive_subscriber.init(hardware_node, "command/drive_mode", onDriveCommand, &command_context);
     * @endcode
     */
    rcl_ret_t init(Node& parent_node,
                   const char* topic,
                   Callback callback,
                   void* context = nullptr,
                   const Config& subscriber_config = Config()) {
        if (callback == nullptr) {
            return RCL_RET_INVALID_ARGUMENT;
        }
        function_callback = callback;
        function_context = context;
        instance = nullptr;
        member_callback = nullptr;
        return BaseSubscriber::init(parent_node,
                                    topic,
                                    MessageTypeSupport<Message>::get(),
                                    &stored_message,
                                    &Subscriber::dispatchMessage,
                                    subscriber_config);
    }

    /**
     * @brief Configure a subscriber with a member-function callback.
     *
     * @code
     * class ExampleNode {
     *    public:
     *     rcl_ret_t init(ros::Node& node) {
     *         return mode_subscriber.init(node, "command/example", this, &ExampleNode::onCommand);
     *     }
     *
     *    private:
     *     void onCommand(const std_msgs__msg__UInt8* msg) {
     *         LogInfo("Command: %u", msg->data);
     *     }
     *
     *     ros::Subscriber<std_msgs__msg__UInt8, ExampleNode> mode_subscriber;
     * };
     * @endcode
     */
    rcl_ret_t init(Node& parent_node,
                   const char* topic,
                   CallbackClass* callback_instance,
                   void (CallbackClass::*callback)(const Message* message),
                   const Config& subscriber_config = Config()) {
        if (callback_instance == nullptr || callback == nullptr) {
            return RCL_RET_INVALID_ARGUMENT;
        }
        function_callback = nullptr;
        function_context = nullptr;
        instance = callback_instance;
        member_callback = callback;
        return BaseSubscriber::init(parent_node,
                                    topic,
                                    MessageTypeSupport<Message>::get(),
                                    &stored_message,
                                    &Subscriber::dispatchMessage,
                                    subscriber_config);
    }

    /**
     * @brief Get the executor-owned message storage.
     * @return Message instance passed to rclc_executor for incoming data.
     */
    Message& message() { return stored_message; }

    /**
     * @brief Get the read-only executor-owned message storage.
     * @return Message instance passed to rclc_executor for incoming data.
     */
    const Message& message() const { return stored_message; }

   private:
    Message stored_message{};
    Callback function_callback = nullptr;
    void* function_context = nullptr;
    CallbackClass* instance = nullptr;
    void (CallbackClass::*member_callback)(const Message* message) = nullptr;

    static void dispatchMessage(BaseSubscriber* subscriber, const void* message) {
        auto* typed_subscriber = static_cast<Subscriber*>(subscriber);
        const Message* typed_message = static_cast<const Message*>(message);

        if (typed_subscriber->member_callback != nullptr && typed_subscriber->instance != nullptr) {
            (typed_subscriber->instance->*typed_subscriber->member_callback)(typed_message);
        } else if (typed_subscriber->function_callback != nullptr) {
            typed_subscriber->function_callback(typed_message, typed_subscriber->function_context);
        }
    }
};

}  // namespace ros
