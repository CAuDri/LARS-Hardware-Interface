/**
 * @file driver.hpp
 *
 * @brief CAuDri - Abstract Driver Interface for all peripheral drivers
 * 
 * This file defines an abstract 'Driver' class that serves as a common interface for all peripheral drivers in the system.
 * A driver can represent any hardware component such as sensors, actuators, communication interfaces, etc.
 * 
 * The Driver class provides a standardized way to manage the lifecycle and state of drivers.
 * Methods for initialization, starting, stopping, and error handling must be implemented by derived classes.
 */
#pragma once

#include <array>
#include <cstdio>
#include <cstddef>

#include <trcRecorder.h>

constexpr size_t TRACE_OBJECT_NAME_MAX_LENGTH = 28;
constexpr size_t DRIVER_STATE_TRACE_STATE_COUNT = 4;
constexpr size_t DRIVER_CONNECTION_TRACE_STATE_COUNT = 4;

class Driver {
   public:
   /**
    * @brief Possible driver states
    * 
    * The driver can be in one of the following states:
    * - ERROR: The driver encountered a critical error and is no longer functional
    * - UNINITIALIZED: The driver has not been initialized yet
    * - INITIALIZED: The driver has been initialized and is ready to be started
    * - RUNNING: The driver is running and functional
    */
    enum class State { ERROR, UNINITIALIZED, INITIALIZED, RUNNING };

    /**
     * @brief Possible connection states for drivers that manage connections
     * 
     * The connection can be in one of the following states:
     * - DISCONNECTED: No active connection
     * - CONNECTING: In the process of establishing a connection
     * - CONNECTED: Active and healthy connection
     * - UNKNOWN: Connection state is not applicable or cannot be determined
     */
    enum class ConnectionState { DISCONNECTED, CONNECTING, CONNECTED, UNKNOWN };

    /**
     * @brief Virtual destructor for the driver interface
     * 
     * Ensures proper cleanup of derived classes.
     */
    virtual ~Driver() = default;

    /**
     * @brief Get the current state of the driver
     * 
     * @return The current driver state
     */
    State getState() const { return state; }

    /**
     * @brief Get the current connection state of the driver
     * 
     * @return The current connection state (UNKNOWN if not applicable)
     */
    ConnectionState getConnectionState() const { return connection_state; }

    /**
     * @brief Get the name of the driver
     * 
     * @return Pointer to a string containing the driver name
     */
    const char* getName() const { return name; }

   protected:
    State state = State::UNINITIALIZED;
    ConnectionState connection_state = ConnectionState::UNKNOWN;

    /**
     * @brief Constructor for the common driver interface
     * 
     * Initializes the driver state to UNINITIALIZED and sets up Tracealyzer state machines for state changes.
     * 
     * @param name The name of the driver
     */
    explicit Driver(const char* name) : name(name) {
        // Tracealyzer state machines can only be initialized once the recorder is running.
        // Should the driver be constructed before that, we defer initialization until the first state change.
        if (initTraceStateMachines()) {
            tracing_initialized = true;
            tracing_enabled = true;
        }
    }

    /**
     * @brief Set the current state of the driver
     * 
     * Updates the driver state and logs the state change to Tracealyzer if initialized.
     * An ERROR state is considered final and cannot be changed once set.
     * 
     * @param new_state The new state to set
     */
    void setState(const State new_state) {
        // An ERROR state is considered final, we cannot transition out of it
        // This is a safeguard to prevent unintended state changes after a critical error
        if (state == new_state || state == State::ERROR) {
            return;
        }
        state = new_state;

        // Try to initialize Tracealyzer state machines on the first state change if not already done.
        if (!tracing_initialized) {
            if (initTraceStateMachines()) {
                tracing_enabled = true;
            }
            tracing_initialized = true;
        }

        if (tracing_enabled) {
            traceState(state_machine, state_handles, stateToIndex(state));
        }
    }

    /**
     * @brief Set the current connection state of the driver
     * 
     * UNKNOWN is considered the default state for drivers that do/can not manage the connection state.
     * 
     * @param new_connection The new connection state to set
     */
    void setConnectionState(const ConnectionState new_connection) {
        if (connection_state == new_connection) {
            return;
        }
        connection_state = new_connection;

        // Try to initialize Tracealyzer state machines on the first state change if not already done.
        if (!tracing_initialized) {
            if (initTraceStateMachines()) {
                tracing_enabled = true;
            }
            tracing_initialized = true;
        }

        if (tracing_enabled) {
            traceState(connection_machine, connection_handles, connectionToIndex(connection_state));
        }
    }

   private:
    const char* name = nullptr;
    std::array<char, TRACE_OBJECT_NAME_MAX_LENGTH> trace_name{};

    bool tracing_initialized = false;
    bool tracing_enabled = false;
    TraceStateMachineHandle_t state_machine = nullptr;
    TraceStateMachineHandle_t connection_machine = nullptr;
    std::array<TraceStateMachineStateHandle_t, DRIVER_STATE_TRACE_STATE_COUNT> state_handles{};
    std::array<TraceStateMachineStateHandle_t, DRIVER_CONNECTION_TRACE_STATE_COUNT> connection_handles{};

    /**
     * @brief Convert a driver state to the matching TraceRecorder state index
     */
    static size_t stateToIndex(State driver_state) {
        switch (driver_state) {
            case State::ERROR:
                return 0U;
            case State::UNINITIALIZED:
                return 1U;
            case State::INITIALIZED:
                return 2U;
            case State::RUNNING:
                return 3U;
        }
        return 0U;
    }

    /**
     * @brief Convert a connection state to the matching TraceRecorder state index
     */
    static size_t connectionToIndex(ConnectionState driver_connection) {
        switch (driver_connection) {
            case ConnectionState::DISCONNECTED:
                return 0U;
            case ConnectionState::CONNECTING:
                return 1U;
            case ConnectionState::CONNECTED:
                return 2U;
            case ConnectionState::UNKNOWN:
                return 3U;
        }
        return 3U;
    }

    /**
     * @brief Build a short TraceRecorder object name within the configured symbol limit
     */
    void composeTraceName(const char* prefix, const char* suffix) {
        const char* driver_name = name == nullptr ? "Unknown" : name;
        const int written =
            snprintf(trace_name.data(), trace_name.size(), "%s%s%s", prefix, driver_name, suffix);
        if (written < 0) {
            trace_name[0] = '\0';
            return;
        }
        trace_name[trace_name.size() - 1U] = '\0';
    }

    /**
     * @brief Report trace setup failures without depending on registered log channels
     */
    void reportTraceFailure(uint32_t code) {
        (void)xTracePrintCompactF1("Driver Trace", "Registration failed: %u", code);
    }

    /**
     * @brief Initialize a TraceRecorder state machine and its states
     */
    template <size_t STATE_COUNT>
    bool initStateMachine(TraceStateMachineHandle_t& machine,
                          std::array<TraceStateMachineStateHandle_t, STATE_COUNT>& states,
                          const char* const* state_names,
                          const char* name_prefix,
                          const char* name_suffix) {
        composeTraceName(name_prefix, name_suffix);
        if (xTraceStateMachineCreate(trace_name.data(), &machine) != TRC_SUCCESS || machine == nullptr) {
            reportTraceFailure(1U);
            return false;
        }

        for (size_t i = 0; i < STATE_COUNT; i++) {
            if (xTraceStateMachineStateCreate(machine, state_names[i], &states[i]) != TRC_SUCCESS || states[i] == nullptr) {
                reportTraceFailure(2U);
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Set a TraceRecorder state machine state if tracing is available
     */
    template <size_t STATE_COUNT>
    void traceState(TraceStateMachineHandle_t machine,
                    const std::array<TraceStateMachineStateHandle_t, STATE_COUNT>& states,
                    size_t index) {
        if (machine == nullptr || index >= states.size() || states[index] == nullptr) {
            return;
        }
        if (xTraceStateMachineSetState(machine, states[index]) != TRC_SUCCESS) {
            reportTraceFailure(3U);
        }
    }

    /**
     * @brief Initialize Tracealyzer state machines for driver state and connection state
     */
    bool initTraceStateMachines() {
        if (tracing_initialized) {
            return true;
        }

        static constexpr const char* STATE_NAMES[] = {"ERROR", "UNINITIALIZED", "INITIALIZED", "RUNNING"};
        static constexpr const char* CONNECTION_NAMES[] = {"DISCONNECTED", "CONNECTING", "CONNECTED", "UNKNOWN"};

        if (!initStateMachine(state_machine, state_handles, STATE_NAMES, "Drv ", " State")) {
            return false;
        }
        if (!initStateMachine(connection_machine, connection_handles, CONNECTION_NAMES, "Drv ", " Conn")) {
            return false;
        }

        traceState(state_machine, state_handles, stateToIndex(state));
        traceState(connection_machine, connection_handles, connectionToIndex(connection_state));

        return true;
    }
};
