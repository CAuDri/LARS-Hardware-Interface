/**
 * CAuDri - TraceRecorder state machines and counters for micro-ROS
 *
 * @file microros_trace.cpp
 *
 * @brief TraceRecorder state machines and counters for micro-ROS
 */

#include "microros_trace.hpp"

#if MICROROS_TRACE_ENABLED
#include <trcRecorder.h>
#endif

#include <climits>

namespace ros::trace {

namespace {

constexpr const char* CLIENT_STATES[] = {
    "ERROR",
    "UNINITIALIZED",
    "INITIALIZED",
    "CONNECTING",
    "CONNECTED",
    "DISCONNECTED",
    "STOPPING",
    "STOPPED",
};

constexpr const char* CONNECTION_STATES[] = {
    "UNKNOWN",
    "CONNECTING",
    "CONNECTED",
    "DISCONNECTED",
};

constexpr const char* EXECUTOR_STATES[] = {
    "ERROR",
    "UNINITIALIZED",
    "INITIALIZED",
    "SPINNING",
    "STOPPED",
};

constexpr const char* NODE_STATES[] = {
    "ERROR",
    "UNINITIALIZED",
    "INITIALIZED",
    "RUNNING",
};

constexpr size_t CLIENT_STATE_COUNT = sizeof(CLIENT_STATES) / sizeof(CLIENT_STATES[0]);
constexpr size_t CONNECTION_STATE_COUNT = sizeof(CONNECTION_STATES) / sizeof(CONNECTION_STATES[0]);
constexpr size_t EXECUTOR_STATE_COUNT = sizeof(EXECUTOR_STATES) / sizeof(EXECUTOR_STATES[0]);
constexpr size_t NODE_STATE_COUNT = sizeof(NODE_STATES) / sizeof(NODE_STATES[0]);
constexpr size_t TRACE_NAME_BUFFER_SIZE = 28;

StateMachine client_state_machine;
StateMachine connection_state_machine;
StateMachine executor_state_machine;

Counter connection_attempt_counter;
Counter reconnect_counter;
Counter error_counter;
Counter registered_node_counter;
Counter active_node_counter;

#if MICROROS_TRACE_ENABLED
TraceStateMachineHandle_t toStateMachineHandle(void* handle) {
    return static_cast<TraceStateMachineHandle_t>(handle);
}

TraceStateMachineStateHandle_t toStateHandle(void* handle) {
    return static_cast<TraceStateMachineStateHandle_t>(handle);
}

TraceCounterHandle_t toCounterHandle(void* handle) {
    return static_cast<TraceCounterHandle_t>(handle);
}

void printRegistrationError(uint32_t code) {
    // Compact user events do not need a registered string channel. That makes
    // them useful when the failure itself was caused by entry-table exhaustion.
    (void)xTracePrintCompactF1("micro-ROS Trace", "Registration failed: %u", code);
}

void initStateMachine(StateMachine& state_machine, const char* name, const char* const* states, size_t state_count) {
    if (state_machine.initialized || state_machine.failed) {
        return;
    }
    if (state_count > state_machine.states.size()) {
        state_machine.failed = true;
        printRegistrationError(1U);
        return;
    }

    TraceStateMachineHandle_t handle = nullptr;
    if (xTraceStateMachineCreate(name, &handle) != TRC_SUCCESS || handle == nullptr) {
        state_machine.failed = true;
        printRegistrationError(2U);
        return;
    }

    for (size_t i = 0; i < state_count; i++) {
        TraceStateMachineStateHandle_t state_handle = nullptr;
        if (xTraceStateMachineStateCreate(handle, states[i], &state_handle) != TRC_SUCCESS || state_handle == nullptr) {
            state_machine.failed = true;
            printRegistrationError(3U);
            return;
        }
        state_machine.states[i] = state_handle;
    }

    state_machine.handle = handle;
    state_machine.state_count = state_count;
    state_machine.initialized = true;
}

void setState(StateMachine& state_machine, size_t state_index) {
    if (!state_machine.initialized || state_index >= state_machine.state_count) {
        return;
    }
    if (xTraceStateMachineSetState(toStateMachineHandle(state_machine.handle), toStateHandle(state_machine.states[state_index])) !=
        TRC_SUCCESS) {
        state_machine.failed = true;
        printRegistrationError(4U);
    }
}

void initCounter(Counter& counter, const char* name, int32_t initial_value, int32_t lower_limit, int32_t upper_limit) {
    if (counter.initialized || counter.failed) {
        return;
    }

    TraceCounterHandle_t handle = nullptr;
    if (xTraceCounterCreate(name, initial_value, lower_limit, upper_limit, &handle) != TRC_SUCCESS || handle == nullptr) {
        counter.failed = true;
        printRegistrationError(5U);
        return;
    }
    counter.handle = handle;
    counter.initialized = true;
}

void setCounter(Counter& counter, int32_t value) {
    if (!counter.initialized) {
        return;
    }
    if (xTraceCounterSet(toCounterHandle(counter.handle), value) != TRC_SUCCESS) {
        counter.failed = true;
        printRegistrationError(6U);
    }
}

void addCounter(Counter& counter, int32_t value) {
    if (!counter.initialized) {
        return;
    }
    TraceBaseType_t current_value = 0;
    if (xTraceCounterGet(toCounterHandle(counter.handle), &current_value) != TRC_SUCCESS ||
        xTraceCounterSet(toCounterHandle(counter.handle), current_value + value) != TRC_SUCCESS) {
        counter.failed = true;
        printRegistrationError(7U);
    }
}
#else
void initStateMachine(StateMachine&, const char*, const char* const*, size_t) {}
void setState(StateMachine&, size_t) {}
void initCounter(Counter&, const char*, int32_t, int32_t, int32_t) {}
void setCounter(Counter&, int32_t) {}
void addCounter(Counter&, int32_t) {}
#endif

size_t toIndex(ConnectionState state) {
    switch (state) {
        case ConnectionState::UNKNOWN:
            return 0;
        case ConnectionState::CONNECTING:
            return 1;
        case ConnectionState::CONNECTED:
            return 2;
        case ConnectionState::DISCONNECTED:
            return 3;
    }
    return 0;
}

size_t toIndex(EntityState state) {
    switch (state) {
        case EntityState::ERROR:
            return 0;
        case EntityState::UNINITIALIZED:
            return 1;
        case EntityState::INITIALIZED:
            return 2;
        case EntityState::RUNNING:
            return 3;
    }
    return 0;
}

int32_t clampCounterValue(size_t value) {
    return value > static_cast<size_t>(INT_MAX) ? INT_MAX : static_cast<int32_t>(value);
}

void composeNodeTraceName(char* buffer, size_t size, const char* node_name) {
    if (buffer == nullptr || size == 0U) {
        return;
    }
    if (node_name == nullptr || node_name[0] == '\0') {
        node_name = "unnamed";
    }

    constexpr const char PREFIX[] = "micro-ROS Node ";
    size_t index = 0;
    for (size_t i = 0; PREFIX[i] != '\0' && index + 1U < size; i++) {
        buffer[index++] = PREFIX[i];
    }
    for (size_t i = 0; node_name[i] != '\0' && index + 1U < size; i++) {
        buffer[index++] = node_name[i];
    }
    buffer[index] = '\0';
}

}  // namespace

void initClient() {
    initStateMachine(client_state_machine, "micro-ROS Client", CLIENT_STATES, CLIENT_STATE_COUNT);
    initStateMachine(connection_state_machine, "micro-ROS Connection", CONNECTION_STATES, CONNECTION_STATE_COUNT);
    initStateMachine(executor_state_machine, "micro-ROS Executor", EXECUTOR_STATES, EXECUTOR_STATE_COUNT);

    initCounter(connection_attempt_counter, "micro-ROS Connect Attempts", 0, 0, INT_MAX);
    initCounter(reconnect_counter, "micro-ROS Reconnects", 0, 0, INT_MAX);
    initCounter(error_counter, "micro-ROS Errors", 0, 0, INT_MAX);
    initCounter(registered_node_counter, "micro-ROS Nodes Registered", 0, 0, INT_MAX);
    initCounter(active_node_counter, "micro-ROS Nodes Active", 0, 0, INT_MAX);
}

void setClientState(size_t state_index) {
    initClient();
    setState(client_state_machine, state_index);
}

void setConnectionState(ConnectionState state) {
    initClient();
    setState(connection_state_machine, toIndex(state));
}

void setExecutorState(size_t state_index) {
    initClient();
    setState(executor_state_machine, state_index);
}

void initNode(StateMachine& node_trace, const char* node_name) {
    char trace_name[TRACE_NAME_BUFFER_SIZE] = {};
    composeNodeTraceName(trace_name, sizeof(trace_name), node_name);
    initStateMachine(node_trace, trace_name, NODE_STATES, NODE_STATE_COUNT);
}

void setNodeState(StateMachine& node_trace, EntityState state) {
    setState(node_trace, toIndex(state));
}

void incrementConnectionAttempts() {
    initClient();
    addCounter(connection_attempt_counter, 1);
}

void incrementReconnects() {
    initClient();
    addCounter(reconnect_counter, 1);
}

void incrementErrors() {
    initClient();
    addCounter(error_counter, 1);
}

void setRegisteredNodeCount(size_t count) {
    initClient();
    setCounter(registered_node_counter, clampCounterValue(count));
}

void setActiveNodeCount(size_t count) {
    initClient();
    setCounter(active_node_counter, clampCounterValue(count));
}

void incrementActiveNodeCount() {
    initClient();
    addCounter(active_node_counter, 1);
}

void decrementActiveNodeCount() {
    initClient();
    addCounter(active_node_counter, -1);
}

}  // namespace ros::trace
