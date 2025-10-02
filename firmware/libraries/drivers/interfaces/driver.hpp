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

#include <trcRecorder.h>

constexpr size_t CHANNEL_NAME_MAX_LENGTH = 32;

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
     * @brief Get the name of the driver
     * 
     * @return Pointer to a string containing the driver name
     */
    const char* getName() const { return name; }

   protected:
    State state = State::UNINITIALIZED;

    /**
     * @brief Constructor for the common driver interface
     * 
     * Initializes the driver state to UNINITIALIZED and sets up a Tracealyzer channel for logging state changes.
     * 
     * @param name The name of the driver
     */
    explicit Driver(const char* name) : name(name) {
        // State changes will be logged on a dedicated "driver/<name>_state" channel in Tracealyzer
        snprintf(channel_name.data(), CHANNEL_NAME_MAX_LENGTH, "driver/%s_state", name);
        channel_name[CHANNEL_NAME_MAX_LENGTH - 1] = '\0'; // Ensure null termination

        if (xTraceStringRegister(channel_name.data(), &state_channel) != TRC_SUCCESS) {
            return;
        }
        trace_initialized = true;
        xTracePrint(state_channel, "UNINITIALIZED");
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

        if (trace_initialized) {
            switch (state) {
                case State::ERROR:
                    xTracePrint(state_channel, "ERROR");
                    break;
                case State::UNINITIALIZED:
                    xTracePrint(state_channel, "UNINITIALIZED");
                    break;
                case State::INITIALIZED:
                    xTracePrint(state_channel, "INITIALIZED");
                    break;
                case State::RUNNING:
                    xTracePrint(state_channel, "RUNNING");
                    break;
                default:
                    xTracePrint(state_channel, "UNKNOWN");
                    break;
            }
        }
    }

   private:
    const char* name = nullptr;
    std::array<char, CHANNEL_NAME_MAX_LENGTH> channel_name{};

    bool trace_initialized = false;
    TraceStringHandle_t state_channel = nullptr;
};