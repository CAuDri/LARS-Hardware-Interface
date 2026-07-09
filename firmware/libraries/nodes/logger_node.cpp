/**
 * @file logger_node.cpp
 *
 * @brief CAuDri - firmware logger /rosout publisher implementation
 */

#include "logger_node.hpp"

#include <cstring>

#include "logger.h"
#include "type_support.hpp"

ROS_DECLARE_MESSAGE_TYPE(rcl_interfaces, Log);

static LoggerNode* active_logger_node = nullptr;

LoggerNode::LoggerNode() = default;

rcl_ret_t LoggerNode::init(Client& client) { return init(client, Config{}); }

rcl_ret_t LoggerNode::init(Client& client, const Config& config) {
    if (getState() != EntityState::UNINITIALIZED) {
        return RCL_RET_ALREADY_INIT;
    }
    if (config.topic == nullptr || config.topic[0] == '\0' ||
        config.logger_name == nullptr || config.logger_name[0] == '\0') {
        return RCL_RET_INVALID_ARGUMENT;
    }

    this->client = &client;
    this->config = config;

    rcl_ret_t result = Node::init(client, LOGGER_NODE_NAME);
    if (result != RCL_RET_OK) {
        return result;
    }

    configureMessage();

    result = rosout_publisher.init(*this, config.topic, config.publisher_config);
    if (result != RCL_RET_OK) {
        (void)fini();
        return result;
    }

    active_logger_node = this;
    return RCL_RET_OK;
}

bool LoggerNode::isReady() const {
    return client != nullptr && client->isConnected() && isActive() && rosout_publisher.isActive();
}

bool LoggerNode::publishLog(uint32_t level, uint32_t timestamp_ms, const char* text, uint32_t length) {
    if (!isReady() || text == nullptr) {
        return false;
    }

    const size_t copy_length = length < (text_buffer.size() - 1U) ? length : text_buffer.size() - 1U;
    std::memcpy(text_buffer.data(), text, copy_length);
    text_buffer[copy_length] = '\0';

    setTimestamp(timestamp_ms);
    message.level = mapLogLevel(level);
    message.msg.data = text_buffer.data();
    message.msg.size = copy_length;
    message.msg.capacity = text_buffer.size();

    return rosout_publisher.publish(message) == RCL_RET_OK;
}

void LoggerNode::configureMessage() {
    message.name.data = const_cast<char*>(config.logger_name);
    message.name.size = std::strlen(config.logger_name);
    message.name.capacity = message.name.size + 1U;

    message.msg.data = text_buffer.data();
    message.msg.size = 0U;
    message.msg.capacity = text_buffer.size();

    message.file.data = const_cast<char*>("");
    message.file.size = 0U;
    message.file.capacity = 1U;

    message.function.data = const_cast<char*>("");
    message.function.size = 0U;
    message.function.capacity = 1U;

    message.line = 0U;
}

void LoggerNode::setTimestamp(uint32_t timestamp_ms) {
    message.stamp.sec = static_cast<int32_t>(timestamp_ms / 1000U);
    message.stamp.nanosec = (timestamp_ms % 1000U) * 1000000U;
}

uint8_t LoggerNode::mapLogLevel(uint32_t level) {
    switch (level) {
        case LOG_LEVEL_ERROR:
            return rcl_interfaces__msg__Log__ERROR;
        case LOG_LEVEL_WARNING:
            return rcl_interfaces__msg__Log__WARN;
        case LOG_LEVEL_DEBUG:
            return rcl_interfaces__msg__Log__DEBUG;
        case LOG_LEVEL_INFO:
        case LOG_LEVEL_SUCCESS:
        default:
            return rcl_interfaces__msg__Log__INFO;
    }
}

extern "C" bool logger_ros_ready(void) {
    return active_logger_node != nullptr && active_logger_node->isReady();
}

extern "C" bool logger_ros_publish(uint32_t level, uint32_t timestamp_ms, const char* text, uint32_t length) {
    return active_logger_node != nullptr && active_logger_node->publishLog(level, timestamp_ms, text, length);
}
