/**
 * @file diagnostics.cpp
 *
 * @brief CAuDri - Static ROS diagnostics message helpers
 */

#include "diagnostics.hpp"

#include <cstdio>
#include <cstring>

#include "type_support.hpp"

ROS_DECLARE_MESSAGE_TYPE(diagnostic_msgs, DiagnosticArray);

namespace ros::diagnostics {

DiagnosticPublisher::DiagnosticPublisher() { configureStorage(); }

rcl_ret_t DiagnosticPublisher::init(Node& node,
                                    const char* topic,
                                    const BasePublisher::Config& publisher_config) {
    return publisher.init(node, topic, publisher_config);
}

void DiagnosticPublisher::beginArray(const builtin_interfaces__msg__Time& stamp) {
    message.header.stamp = stamp;
    message.status.size = 0U;
    active_status = nullptr;
    active_status_index = 0U;
}

bool DiagnosticPublisher::beginStatus(const char* name,
                                      uint8_t level,
                                      const char* status_message,
                                      const char* hardware_id) {
    if (message.status.size >= statuses.size()) {
        active_status = nullptr;
        return false;
    }

    active_status_index = message.status.size;
    active_status = &statuses[active_status_index];
    message.status.size++;

    diagnostic_msgs__msg__DiagnosticStatus& status = *active_status;
    status.level = level;
    setString(status.name, name);
    setString(status.message, status_message);
    setString(status.hardware_id, hardware_id);
    status.values.size = 0U;
    return true;
}

bool DiagnosticPublisher::addValue(const char* key, const char* value) {
    if (active_status == nullptr || key == nullptr || value == nullptr ||
        active_status->values.size >= values[active_status_index].size()) {
        return false;
    }

    const size_t value_index = active_status->values.size;
    diagnostic_msgs__msg__KeyValue& value_slot = values[active_status_index][value_index];
    auto& buffer = value_buffers[active_status_index][value_index];

    setString(value_slot.key, key);

    char* value_buffer = buffer.data();
    const size_t max_length = buffer.size() - 1U;
    size_t length = std::strlen(value);
    if (length > max_length) {
        length = max_length;
    }

    std::memcpy(value_buffer, value, length);
    value_buffer[length] = '\0';
    value_slot.value.data = value_buffer;
    value_slot.value.size = length;
    value_slot.value.capacity = buffer.size();

    active_status->values.size++;
    return true;
}

bool DiagnosticPublisher::addValue(const char* key, bool value) { return addValue(key, value ? "true" : "false"); }

bool DiagnosticPublisher::addValue(const char* key, uint32_t value) {
    return addFormattedValue(key, "%lu", static_cast<unsigned long>(value));
}

bool DiagnosticPublisher::addValue(const char* key, int32_t value) {
    return addFormattedValue(key, "%ld", static_cast<long>(value));
}

rcl_ret_t DiagnosticPublisher::publish() { return publisher.publish(message); }

void DiagnosticPublisher::configureStorage() {
    message.header.frame_id.data = empty_string;
    message.header.frame_id.size = 0U;
    message.header.frame_id.capacity = sizeof(empty_string);

    message.status.data = statuses.data();
    message.status.size = 0U;
    message.status.capacity = statuses.size();

    for (size_t status_index = 0; status_index < statuses.size(); status_index++) {
        diagnostic_msgs__msg__DiagnosticStatus& status = statuses[status_index];
        status.values.data = values[status_index].data();
        status.values.size = 0U;
        status.values.capacity = values[status_index].size();

        status.name.data = empty_string;
        status.name.size = 0U;
        status.name.capacity = sizeof(empty_string);

        status.message.data = empty_string;
        status.message.size = 0U;
        status.message.capacity = sizeof(empty_string);

        status.hardware_id.data = empty_string;
        status.hardware_id.size = 0U;
        status.hardware_id.capacity = sizeof(empty_string);

        for (size_t value_index = 0; value_index < values[status_index].size(); value_index++) {
            values[status_index][value_index].key.data = empty_string;
            values[status_index][value_index].key.size = 0U;
            values[status_index][value_index].key.capacity = sizeof(empty_string);

            values[status_index][value_index].value.data = value_buffers[status_index][value_index].data();
            values[status_index][value_index].value.size = 0U;
            values[status_index][value_index].value.capacity = value_buffers[status_index][value_index].size();
            value_buffers[status_index][value_index][0] = '\0';
        }
    }
}

void DiagnosticPublisher::setString(rosidl_runtime_c__String& string, const char* value) {
    const char* source = value == nullptr ? "" : value;
    string.data = const_cast<char*>(source);
    string.size = std::strlen(source);
    string.capacity = string.size + 1U;
}

bool DiagnosticPublisher::addFormattedValue(const char* key, const char* format, long value) {
    if (active_status == nullptr || key == nullptr || format == nullptr ||
        active_status->values.size >= values[active_status_index].size()) {
        return false;
    }

    const size_t value_index = active_status->values.size;
    auto& buffer = value_buffers[active_status_index][value_index];
    diagnostic_msgs__msg__KeyValue& value_slot = values[active_status_index][value_index];
    const int written = std::snprintf(buffer.data(), buffer.size(), format, value);
    if (written < 0) {
        return false;
    }

    setString(value_slot.key, key);
    value_slot.value.data = buffer.data();
    value_slot.value.size = written < static_cast<int>(buffer.size()) ? static_cast<size_t>(written)
                                                                      : buffer.size() - 1U;
    value_slot.value.capacity = buffer.size();

    active_status->values.size++;
    return true;
}

bool DiagnosticPublisher::addFormattedValue(const char* key, const char* format, unsigned long value) {
    if (active_status == nullptr || key == nullptr || format == nullptr ||
        active_status->values.size >= values[active_status_index].size()) {
        return false;
    }

    const size_t value_index = active_status->values.size;
    auto& buffer = value_buffers[active_status_index][value_index];
    diagnostic_msgs__msg__KeyValue& value_slot = values[active_status_index][value_index];
    const int written = std::snprintf(buffer.data(), buffer.size(), format, value);
    if (written < 0) {
        return false;
    }

    setString(value_slot.key, key);
    value_slot.value.data = buffer.data();
    value_slot.value.size = written < static_cast<int>(buffer.size()) ? static_cast<size_t>(written)
                                                                      : buffer.size() - 1U;
    value_slot.value.capacity = buffer.size();

    active_status->values.size++;
    return true;
}

}  // namespace ros::diagnostics
