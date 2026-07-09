/**
 * @file logger_node.hpp
 *
 * @brief CAuDri - micro-ROS node for forwarding firmware logs to /rosout
 */
#pragma once

#include <rcl_interfaces/msg/log.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "client.hpp"
#include "node.hpp"
#include "publisher.hpp"

using ros::BasePublisher;
using ros::Client;
using ros::EntityState;
using ros::Node;
using ros::Publisher;

constexpr const char* LOGGER_NODE_NAME = "logger";
constexpr const char* LOGGER_NODE_ROSOUT_TOPIC = "/rosout";
constexpr const char* LOGGER_NODE_LOGGER_NAME = "hardware_interface";
constexpr size_t LOGGER_NODE_TEXT_BUFFER_SIZE = 256;

/**
 * @brief Publishes firmware logger messages as standard ROS log messages.
 *
 * The C logger owns buffering and formatting of printf-style arguments. This
 * node owns the ROS message and publisher state so logger.c does not need to
 * depend on C++ wrapper internals.
 */
class LoggerNode : public Node {
   public:
    struct Config {
        const char* topic = LOGGER_NODE_ROSOUT_TOPIC;
        const char* logger_name = LOGGER_NODE_LOGGER_NAME;
        BasePublisher::Config publisher_config{true, 0};
    };

    LoggerNode();
    ~LoggerNode() = default;
    LoggerNode(const LoggerNode&) = delete;
    LoggerNode& operator=(const LoggerNode&) = delete;

    rcl_ret_t init(Client& client);
    rcl_ret_t init(Client& client, const Config& config);
    bool isReady() const;
    bool publishLog(uint32_t level, uint32_t timestamp_ms, const char* text, uint32_t length);

   private:
    Client* client = nullptr;
    Config config{};
    Publisher<rcl_interfaces__msg__Log> rosout_publisher{};
    rcl_interfaces__msg__Log message{};
    std::array<char, LOGGER_NODE_TEXT_BUFFER_SIZE> text_buffer{};

    void configureMessage();
    void setTimestamp(uint32_t timestamp_ms);
    static uint8_t mapLogLevel(uint32_t level);
};
