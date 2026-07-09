/**
 * @file diagnostics.hpp
 *
 * @brief CAuDri - Static helpers for publishing ROS diagnostic messages
 */
#pragma once

#include <builtin_interfaces/msg/time.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "publisher.hpp"

constexpr size_t ROS_DIAGNOSTIC_MAX_VALUES = 6;
constexpr size_t ROS_DIAGNOSTIC_MAX_STATUSES = 12;
constexpr size_t ROS_DIAGNOSTIC_VALUE_BUFFER_SIZE = 24;

namespace ros::diagnostics {

/**
 * @brief Reusable fixed-capacity DiagnosticArray publisher.
 *
 * The helper reuses all storage between publishes. Names, messages, hardware
 * IDs, and keys are expected to point to static storage. Values are copied into
 * fixed buffers.
 */
class DiagnosticPublisher {
   public:
    DiagnosticPublisher();
    ~DiagnosticPublisher() = default;
    DiagnosticPublisher(const DiagnosticPublisher&) = delete;
    DiagnosticPublisher& operator=(const DiagnosticPublisher&) = delete;

    rcl_ret_t init(Node& node, const char* topic, const BasePublisher::Config& publisher_config = BasePublisher::Config());

    void beginArray(const builtin_interfaces__msg__Time& stamp);
    bool beginStatus(const char* name, uint8_t level, const char* message, const char* hardware_id);
    bool addValue(const char* key, const char* value);
    bool addValue(const char* key, bool value);
    bool addValue(const char* key, uint32_t value);
    bool addValue(const char* key, int32_t value);
    rcl_ret_t publish();

   private:
    Publisher<diagnostic_msgs__msg__DiagnosticArray> publisher{};
    diagnostic_msgs__msg__DiagnosticArray message{};
    diagnostic_msgs__msg__DiagnosticStatus* active_status = nullptr;

    std::array<diagnostic_msgs__msg__DiagnosticStatus, ROS_DIAGNOSTIC_MAX_STATUSES> statuses{};
    std::array<std::array<diagnostic_msgs__msg__KeyValue, ROS_DIAGNOSTIC_MAX_VALUES>, ROS_DIAGNOSTIC_MAX_STATUSES> values{};
    std::array<std::array<std::array<char, ROS_DIAGNOSTIC_VALUE_BUFFER_SIZE>, ROS_DIAGNOSTIC_MAX_VALUES>, ROS_DIAGNOSTIC_MAX_STATUSES> value_buffers{};

    char empty_string[1] = "";
    size_t active_status_index = 0U;

    void configureStorage();
    static void setString(rosidl_runtime_c__String& string, const char* value);
    bool addFormattedValue(const char* key, const char* format, long value);
    bool addFormattedValue(const char* key, const char* format, unsigned long value);
};

}  // namespace ros::diagnostics
