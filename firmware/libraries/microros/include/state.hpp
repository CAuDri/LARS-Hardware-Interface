/**
 * @file state.hpp
 *
 * @brief CAuDri - Common state definitions for micro-ROS wrappers
 */
#pragma once

namespace ros {

enum class EntityState { ERROR, UNINITIALIZED, INITIALIZED, RUNNING };

enum class ConnectionState { UNKNOWN, CONNECTING, CONNECTED, DISCONNECTED };

}  // namespace ros
