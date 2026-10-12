/**
 * CAuDri - Optional TraceRecorder instrumentation for the micro-ROS wrapper
 *
 * @file microros_trace.hpp
 *
 * @brief Optional TraceRecorder instrumentation for the micro-ROS wrapper
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "state.hpp"
#include "debug_config.h"

#ifndef MICROROS_TRACE_ENABLED
#define MICROROS_TRACE_ENABLED 1
#endif

#ifdef DEBUG_USE_TRACE_RECORDER
#ifdef DEBUG_TRACE_LOG_NODE_STATE
#if MICROROS_TRACE_ENABLED
#define MICROROS_NODE_TRACE_ENABLED
#endif
#endif
#endif

namespace ros::trace {

constexpr size_t MAX_TRACE_STATES = 8;

/**
 * @brief Opaque wrapper around a TraceRecorder state machine.
 *
 * The public wrapper headers deliberately do not include TraceRecorder headers.
 * The concrete TraceRecorder handle types are pointer-sized, so they are stored
 * opaquely here and converted only inside microros_trace.cpp.
 */
struct StateMachine {
    void* handle = nullptr;
    std::array<void*, MAX_TRACE_STATES> states{};
    size_t state_count = 0;
    bool initialized = false;
    bool failed = false;
};

#ifdef MICROROS_NODE_TRACE_ENABLED
using NodeStateMachine = StateMachine;
#else
struct NodeStateMachine {};
#endif

/**
 * @brief Opaque wrapper around a TraceRecorder counter.
 */
struct Counter {
    void* handle = nullptr;
    bool initialized = false;
    bool failed = false;
};

void initClient();
void setClientState(size_t state_index);
void setConnectionState(ConnectionState state);
void setExecutorState(size_t state_index);

void initNode(NodeStateMachine& node_trace, const char* node_name);
void setNodeState(NodeStateMachine& node_trace, EntityState state);

void incrementConnectionAttempts();
void incrementReconnects();
void incrementErrors();
void setRegisteredNodeCount(size_t count);
void setActiveNodeCount(size_t count);
void incrementActiveNodeCount();
void decrementActiveNodeCount();

}  // namespace ros::trace
