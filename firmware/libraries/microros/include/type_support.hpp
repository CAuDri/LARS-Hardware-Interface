/**
 * @file type_support.hpp
 *
 * @brief CAuDri - Type-support traits for generated ROS message types
 */
#pragma once

#include <rosidl_runtime_c/message_type_support_struct.h>

namespace ros {

template <typename Message>
struct MessageTypeSupport;

}  // namespace ros

#define ROS_DECLARE_MESSAGE_TYPE_SUPPORT(C_TYPE, PACKAGE, INTERFACE)     \
    namespace ros {                                                      \
    template <>                                                          \
    struct MessageTypeSupport<C_TYPE> {                                  \
        static const rosidl_message_type_support_t* get() {               \
            return ROSIDL_GET_MSG_TYPE_SUPPORT(PACKAGE, msg, INTERFACE); \
        }                                                                \
    };                                                                   \
    }  // namespace ros

#define ROS_DECLARE_MESSAGE_TYPE(PACKAGE, INTERFACE) \
    ROS_DECLARE_MESSAGE_TYPE_SUPPORT(PACKAGE##__msg__##INTERFACE, PACKAGE, INTERFACE)
