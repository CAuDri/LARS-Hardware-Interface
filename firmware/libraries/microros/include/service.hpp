/**
 * @file service.hpp
 *
 * @brief CAuDri - Reconnectable micro-ROS service wrapper
 */
#pragma once

#include <rcl/rcl.h>
#include <rcl/service.h>
#include <rclc/executor.h>
#include <rclc/service.h>

#include "node.hpp"
#include "state.hpp"
#include "type_support.hpp"

namespace ros {

/**
 * @brief Placeholder type used when a service uses a free/static callback.
 */
class NoServiceCallbackClass {};

class BaseService {
   public:
    struct Config {
        bool best_effort = false;
    };

    BaseService();
    ~BaseService() = default;
    BaseService(const BaseService&) = delete;
    BaseService& operator=(const BaseService&) = delete;

    rcl_ret_t fini();

    EntityState getState() const;
    rcl_ret_t getLastError() const;
    const char* getServiceName() const;
    bool isActive() const;

   protected:
    using DispatchFunction = void (*)(BaseService* service, const void* request, void* response);

    rcl_ret_t init(Node& parent_node,
                   const char* name,
                   const rosidl_service_type_support_t* service_type_support,
                   void* request_storage,
                   void* response_storage,
                   DispatchFunction dispatch_function,
                   const Config& service_config);

   private:
    friend class Client;

    Node* node = nullptr;
    const char* service_name = nullptr;
    const rosidl_service_type_support_t* type_support = nullptr;
    void* request_storage = nullptr;
    void* response_storage = nullptr;
    DispatchFunction dispatch = nullptr;
    Config config{};
    rcl_service_t rcl_service{};
    volatile EntityState state = EntityState::UNINITIALIZED;
    volatile ConnectionState connection_state = ConnectionState::UNKNOWN;
    rcl_ret_t last_error = RCL_RET_OK;
    bool registered = false;
    bool rcl_active = false;
    bool executor_registered = false;

    static void executorCallback(const void* request, void* response, void* context);
    rcl_ret_t initRclcService();
    rcl_ret_t finiRclcService();
};

/**
 * @brief Typed service with allocation-free request and response storage.
 *
 * Service callbacks run in the micro-ROS executor thread. Keep callbacks short
 * and avoid blocking operations. The request pointer is valid only during the
 * callback. The response pointer refers to the service-owned response storage
 * and must be filled before the callback returns.
 *
 * For services containing strings, sequences, or other dynamically backed
 * fields, configure bounded backing memory in request() and/or response()
 * before the service connects to the agent.
 */
template <typename ServiceType, typename CallbackClass = NoServiceCallbackClass>
class Service : public BaseService {
   public:
    using Request = typename ServiceType::Request;
    using Response = typename ServiceType::Response;
    using Callback = void (*)(const Request* request, Response* response, void* context);

    /**
     * @brief Configure a service with a free or static callback.
     *
     * @code
     * static void onTrigger(const std_srvs__srv__Trigger_Request* request,
     *                       std_srvs__srv__Trigger_Response* response,
     *                       void* context) {
     *     (void)request;
     *     (void)context;
     *     response->success = true;
     * }
     *
     * ROS_DECLARE_SERVICE_TYPE(std_srvs, Trigger);
     * ros::Service<ros::service_types::std_srvs_Trigger> trigger_service;
     * trigger_service.init(hardware_node, "trigger", onTrigger);
     * @endcode
     */
    rcl_ret_t init(Node& parent_node,
                   const char* name,
                   Callback callback,
                   void* context = nullptr,
                   const Config& service_config = Config()) {
        if (callback == nullptr) {
            return RCL_RET_INVALID_ARGUMENT;
        }
        function_callback = callback;
        function_context = context;
        instance = nullptr;
        member_callback = nullptr;
        return BaseService::init(parent_node,
                                 name,
                                 ServiceTypeSupport<ServiceType>::get(),
                                 &stored_request,
                                 &stored_response,
                                 &Service::dispatchRequest,
                                 service_config);
    }

    /**
     * @brief Configure a service with a member-function callback.
     *
     * @code
     * class ExampleNode {
     *    public:
     *     rcl_ret_t init(ros::Node& node) {
     *         return trigger_service.init(node, "trigger", this, &ExampleNode::onTrigger);
     *     }
     *
     *    private:
     *     void onTrigger(const std_srvs__srv__Trigger_Request* request,
     *                    std_srvs__srv__Trigger_Response* response) {
     *         (void)request;
     *         response->success = true;
     *     }
     *
     *     ros::Service<ros::service_types::std_srvs_Trigger, ExampleNode> trigger_service;
     * };
     * @endcode
     */
    rcl_ret_t init(Node& parent_node,
                   const char* name,
                   CallbackClass* callback_instance,
                   void (CallbackClass::*callback)(const Request* request, Response* response),
                   const Config& service_config = Config()) {
        if (callback_instance == nullptr || callback == nullptr) {
            return RCL_RET_INVALID_ARGUMENT;
        }
        function_callback = nullptr;
        function_context = nullptr;
        instance = callback_instance;
        member_callback = callback;
        return BaseService::init(parent_node,
                                 name,
                                 ServiceTypeSupport<ServiceType>::get(),
                                 &stored_request,
                                 &stored_response,
                                 &Service::dispatchRequest,
                                 service_config);
    }

    /**
     * @brief Get the executor-owned request storage.
     * @return Request instance passed to rclc_executor for incoming requests.
     */
    Request& request() { return stored_request; }

    /**
     * @brief Get the read-only executor-owned request storage.
     * @return Request instance passed to rclc_executor for incoming requests.
     */
    const Request& request() const { return stored_request; }

    /**
     * @brief Get the executor-owned response storage.
     * @return Response instance passed to rclc_executor for outgoing responses.
     */
    Response& response() { return stored_response; }

    /**
     * @brief Get the read-only executor-owned response storage.
     * @return Response instance passed to rclc_executor for outgoing responses.
     */
    const Response& response() const { return stored_response; }

   private:
    Request stored_request{};
    Response stored_response{};
    Callback function_callback = nullptr;
    void* function_context = nullptr;
    CallbackClass* instance = nullptr;
    void (CallbackClass::*member_callback)(const Request* request, Response* response) = nullptr;

    static void dispatchRequest(BaseService* service, const void* request, void* response) {
        auto* typed_service = static_cast<Service*>(service);
        const Request* typed_request = static_cast<const Request*>(request);
        Response* typed_response = static_cast<Response*>(response);

        if (typed_service->member_callback != nullptr && typed_service->instance != nullptr) {
            (typed_service->instance->*typed_service->member_callback)(typed_request, typed_response);
        } else if (typed_service->function_callback != nullptr) {
            typed_service->function_callback(typed_request, typed_response, typed_service->function_context);
        }
    }
};

}  // namespace ros
