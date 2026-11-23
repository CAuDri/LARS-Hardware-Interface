/**
 * @file drive_controller.cpp
 *
 * @brief CAuDri - Drive controller for handling drive commands and drive modes.
 *
 * This class manages the drive modes of the vehicle (i.e. idle, manual, autonomous) and adds a bit of safety
 * around the control commands received from the RC receiver and autonomous driving stack.
 *
 * It ensures that the vehicle can be safely stopped in case of emergencies and will perform mandatory stops when
 * switching between drive modes as required by the CAuDri-Challenge regulations.
 *
 * This implementation is tightly coupled to the RCReceiver, VESC, and Servo drivers.
 * Driver interfaces could be used, if you want to make it more generic and reusable for other driver implementations.
 *
 *
 * TODO: Split up manual and autonomous control and move out of this class?
 * TODO: Add configurable hard limits for manual/autonomous control
 * TODO: Command timeout check is intervall based and can effectively double the allowed timeout duration, I dont care.
 */
#include "drive_controller.hpp"

#include <cmath>

#include "FreeRTOS.h"
#include "blink_animation.hpp"
#include "logger.h"

static constexpr uint32_t MANDATORY_STOP_TIME_MS = 1000;  // Time to wait after switching to manual mode (CAuDri regulations)

static constexpr uint32_t EMERGENCY_STOP_TIME_MS = 5000;  // Time to wait after emergency stop before allowing restart
static constexpr uint32_t EMERGENCY_MAX_TIMEOUT_MS = 60000;  // Maximum time to wait in emergency stop before error is raised

static constexpr uint32_t MAX_INIT_WAIT_TIME_MS = 60000;  // Maximum time to wait for initialization (e.g. RC connection)

static constexpr uint32_t RC_SWITCH_STATE_1_THRESHOLD = 900;   // Threshold for RC switch state 1
static constexpr uint32_t RC_SWITCH_STATE_2_THRESHOLD = 1500;  // Threshold for RC switch state 2

static constexpr uint32_t RC_CHANNEL_CENTER_VALUE = 1000;  // Center value for RC channel (joystick center)
static constexpr uint32_t RC_CHANNEL_MAX_VALUE = 1800;     // Maximum value for RC channel (joystick high)
static constexpr uint32_t THROTTLE_DEADZONE = 50;  // Deadzone around center for throttle channel

static constexpr uint32_t CONTROLLER_THREAD_UPDATE_TIME_MS = 100;  // Timeout for the controller thread update loop

// Thread flags for the controller thread
static constexpr uint32_t START_THREAD_FLAG = 0x01;
static constexpr uint32_t MODE_SWITCH_FLAG = 0x02;

static constexpr const char* DRIVE_MODE_TRACE_STATES[] = {
    "IDLE", "MANUAL", "AUTONOMOUS", "MANDATORY_STOP", "EMERGENCY_STOP"};

/**
 * @brief Construct a new drive controller instance for later initialization
 */
DriveController::DriveController() {}


/**
 * @brief Construct a new drive controller instance with configuration and driver references
 *
 * @param config Configuration for the drive controller
 * @param rc_receiver Reference to the RC receiver driver
 * @param vesc Reference to the VESC driver
 * @param servo Reference to the servo driver
 * @param status_light Reference to the status light
 */
DriveController::DriveController(const Config& config, RCReceiver& rc_receiver, VESC& vesc, Servo& servo, Light& status_light) {
    init(config, rc_receiver, vesc, servo, status_light);
}

/**
 * @brief Destroy the drive controller instance and terminate the controller thread
 */
DriveController::~DriveController() {
    osThreadTerminate(controller_thread);
    setState(State::UNINITIALIZED);
    if (control_mutex != nullptr) {
        (void)osMutexDelete(control_mutex);
    }
}

/**
 * @brief Initialize the drive controller with configuration and driver references
 *
 * This will set up the necessary callbacks and create the controller thread.
 *
 * @param config Configuration for the drive controller
 * @param rc_receiver Reference to the RC receiver driver
 * @param vesc Reference to the VESC driver
 * @param servo Reference to the servo driver
 * @param status_light Reference to the status light
 */
bool DriveController::init(const Config& config, RCReceiver& rc_receiver, VESC& vesc, Servo& servo, Light& status_light) {
    if (state != State::UNINITIALIZED) {
        return false;
    }
    setState(State::INITIALIZING);

    this->config = &config;
    if (config.throttle_channel == crsf::INVALID_CHANNEL || config.steering_channel == crsf::INVALID_CHANNEL ||
        config.mode_switch_channel == crsf::INVALID_CHANNEL || config.autonomous_command_timeout_ms == 0U) {
        LogError("Drive Controller: Invalid channel configuration");
        setState(State::ERROR);
        return false;
    }

    mutex_attributes.name = "Drive Control";
    mutex_attributes.attr_bits = osMutexRecursive | osMutexPrioInherit;
    mutex_attributes.cb_mem = &mutex_control_block;
    mutex_attributes.cb_size = sizeof(mutex_control_block);
    control_mutex = osMutexNew(&mutex_attributes);
    if (control_mutex == nullptr) {
        LogError("Drive Controller: Failed to create control mutex");
        setState(State::ERROR);
        return false;
    }

    this->rc_receiver = &rc_receiver;
    this->vesc = &vesc;
    this->servo = &servo;

    // Register the RC receiver callback
    rc_callback = RCReceiver::ChannelCallback::from<DriveController, &DriveController::remoteControlCallback>(this);
    if (!this->rc_receiver->registerChannelCallback(rc_callback)) {
        LogError("Drive Controller: Failed to register RC receiver channel callback");
        setState(State::ERROR);
        return false;
    }

    (void)initDriveModeTracing();

    // Initialize the light dispatcher with the status light
    if (!light_dispatcher.registerLight(status_light)) {
        LogWarning("Drive Controller: Failed to register status light with light dispatcher");
    }
    if (!light_dispatcher.start()) {
        LogError("Drive Controller: Failed to start status light dispatcher");
        setState(State::ERROR);
        return false;
    }

    thread_attributes.name = "Drive Controller";
    thread_attributes.priority = config.thread_priority;
    thread_attributes.stack_mem = &thread_stack;
    thread_attributes.stack_size = sizeof(thread_stack);
    thread_attributes.cb_mem = &thread_control_block;
    thread_attributes.cb_size = sizeof(thread_control_block);

    controller_thread = osThreadNew(
        [](void* arg) -> void {
            auto* obj = static_cast<DriveController*>(arg);
            obj->controllerThread(arg);
        },
        this,
        &thread_attributes);
    if (controller_thread == nullptr) {
        LogError("Drive Controller: Failed to create controller thread");
        setState(State::ERROR);
        return false;
    }
    return true;
}

/**
 * @brief Start the drive controller thread
 *
 * The controller enters RUNNING after the thread has started and the RC receiver is connected.
 */
bool DriveController::start() {
    if (state == State::RUNNING) {
        return true;
    }
    if (state != State::INITIALIZING) {
        LogError("Drive Controller: Cannot start, driver not initialized");
        return false;
    }

    // Check if any of the drivers are in ERROR state
    if (rc_receiver->getState() == Driver::State::ERROR || vesc->getState() == Driver::State::ERROR ||
        servo->getState() == Driver::State::ERROR) {
        LogError("Drive Controller: Cannot start, one or more drivers in ERROR state");
        setState(State::ERROR);
        return false;
    }

    osThreadFlagsSet(controller_thread, START_THREAD_FLAG);
    return true;
}

/**
 * @brief Restart the drive controller after an emergency stop has been triggered
 *
 * It will notify the controller thread that any external error conditions have been resolved.
 * This can only be done while the controller is still in the RUNNING state.
 */
bool DriveController::restart() {
    if (state != State::RUNNING) {
        LogWarning("Drive Controller: Cannot restart, controller not running");
        return false;
    }
    osThreadFlagsSet(controller_thread, START_THREAD_FLAG);
    return true;
}

/**
 * @brief Update the controller lifecycle state. ERROR is final.
 *
 * @param new_state The new state to set
 */
bool DriveController::setState(State new_state) {
    if (state == new_state) {
        return true;
    }
    if (state == State::ERROR) {
        LogError("Drive Controller: Cannot change state from ERROR");
        return false;
    }

    state = new_state;
    return true;
}

/**
 * @brief Change the drive mode of the controller
 *
 * @param mode The new drive mode to set
 */
bool DriveController::changeDriveMode(DriveMode mode) {
    if (control_mutex == nullptr || osMutexAcquire(control_mutex, osWaitForever) != osOK) {
        return false;
    }
    if (state != State::RUNNING) {
        LogWarning("Drive Controller: Cannot set drive mode, controller not running");
        (void)osMutexRelease(control_mutex);
        return false;
    }
    if (mode == current_drive_mode) {
        (void)osMutexRelease(control_mutex);
        return true;
    }

    if (mode == DriveMode::EMERGENCY_STOP) {
        emergencyStop();
        (void)osMutexRelease(control_mutex);
        return true;
    }

    // Emergency stop must be cleared by the controller thread itself
    if (current_drive_mode == DriveMode::EMERGENCY_STOP) {
        LogWarning("Drive Controller: Cannot manually change drive mode from EMERGENCY_STOP");
        (void)osMutexRelease(control_mutex);
        return false;
    }

    // The CAuDri-Challenge requires a mandatory stop when switching to manual mode
    // We ensure this by adding an intermediate MANDATORY_STOP state
    if (current_drive_mode == DriveMode::AUTONOMOUS && (mode == DriveMode::MANUAL || mode == DriveMode::IDLE)) {
        setDriveMode(DriveMode::MANDATORY_STOP);
        (void)osMutexRelease(control_mutex);
        return true;
    };
    // The controller thread will do the actual mode switch later
    if (current_drive_mode == DriveMode::MANDATORY_STOP) {
        (void)osMutexRelease(control_mutex);
        return false;
    }

    setDriveMode(mode);
    (void)osMutexRelease(control_mutex);
    return true;
}

/**
 * @brief Set autonomous control readiness without changing the selected drive mode.
 *
 * Disabling readiness immediately stops autonomous actuation. Re-enabling does
 * not replay commands.
 */
void DriveController::enableAutonomousControl(bool enabled) {
    if (control_mutex == nullptr || osMutexAcquire(control_mutex, osWaitForever) != osOK) {
        return;
    }
    if (autonomous_control_enabled == enabled) {
        (void)osMutexRelease(control_mutex);
        return;
    }
    autonomous_control_enabled = enabled;
    if (!enabled && current_drive_mode == DriveMode::AUTONOMOUS) {
        stopVehicle();
    }
    (void)osMutexRelease(control_mutex);
}

/**
 * @brief Immediately stop the vehicle and start the emergency recovery procedure.
 */
void DriveController::emergencyStop() {
    LogDebug("Drive Controller: Emergency stop triggered");
    setDriveMode(DriveMode::EMERGENCY_STOP);
}

/**
 * @brief Validate and forward a motor command in autonomous mode.
 *
 * RPM and current commands share one timestamp for the command watchdog.
 * @return true when the command was accepted by the motor driver.
 */
bool DriveController::updateMotorCommand(const MotorCommand& command) {
    if (control_mutex == nullptr || osMutexAcquire(control_mutex, osWaitForever) != osOK) {
        return false;
    }
    if (state != State::RUNNING || current_drive_mode != DriveMode::AUTONOMOUS ||
        !autonomous_control_enabled || isCommandStale(command.timestamp_ticks)) {
        (void)osMutexRelease(control_mutex);
        return false;
    }

    bool success = false;
    switch (command.mode) {
        case MotorCommand::Mode::RPM:
            success = vesc->setRPM(command.rpm);
            break;
        case MotorCommand::Mode::CURRENT:
            if (!std::isfinite(command.current)) {
                (void)osMutexRelease(control_mutex);
                return false;
            }
            success = vesc->setCurrent(command.current);
            break;
        case MotorCommand::Mode::NONE:
        default:
            (void)osMutexRelease(control_mutex);
            return false;
    }

    if (!success) {
        stopVehicle();
        (void)osMutexRelease(control_mutex);
        return false;
    }
    motor_command = command;
    (void)osMutexRelease(control_mutex);
    return true;
}

/**
 * @brief Validate and forward a steering command in autonomous mode.
 * @return true when the command was accepted by the servo driver.
 */
bool DriveController::updateSteeringCommand(const SteeringCommand& command) {
    if (control_mutex == nullptr || osMutexAcquire(control_mutex, osWaitForever) != osOK) {
        return false;
    }
    if (state != State::RUNNING || current_drive_mode != DriveMode::AUTONOMOUS ||
        !autonomous_control_enabled || isCommandStale(command.timestamp_ticks) ||
        !std::isfinite(command.angle_deg)) {
        (void)osMutexRelease(control_mutex);
        return false;
    }

    if (!servo->setAngle(command.angle_deg)) {
        stopVehicle();
        (void)osMutexRelease(control_mutex);
        return false;
    }
    steering_command = command;
    (void)osMutexRelease(control_mutex);
    return true;
}

/**
 * @brief Immediately stop the vehicle without changing the drive mode
 *
 * This is used internally during emergency stop procedures.
 */
void DriveController::stopVehicle() {
    if (control_mutex == nullptr || osMutexAcquire(control_mutex, osWaitForever) != osOK) {
        return;
    }
    vesc->setRPM(0);
    servo->setAngle(0.0f);
    (void)osMutexRelease(control_mutex);
}

/**
 * @brief Handle the emergency stop procedure
 *
 * This will stop the vehicle, wait for recovery conditions, and handle any errors.
 *
 * @return true if the emergency stop was cleared successfully, false if an error occurred or the timeout expired
 */
bool DriveController::handleEmergencyStop() {
    stopVehicle();

    // Clear all currently pending thread flags
    osThreadFlagsClear(START_THREAD_FLAG | MODE_SWITCH_FLAG);
    osDelay(EMERGENCY_STOP_TIME_MS);

    // Shutdown if any of the drivers are in ERROR state
    if (rc_receiver->getState() == Driver::State::ERROR || vesc->getState() == Driver::State::ERROR ||
        servo->getState() == Driver::State::ERROR) {
        LogError("Drive Controller: One or more drivers in ERROR state during emergency stop");
        return false;
    }

    uint32_t emergency_max_timeout_stamp = osKernelGetTickCount() + EMERGENCY_MAX_TIMEOUT_MS;
    while (osKernelGetTickCount() < emergency_max_timeout_stamp) {
        osDelay(100);

        // Check if all drivers are connected and running
        if (!rc_receiver->isConnected()) {
            continue;
        }
        if (rc_receiver->getState() != Driver::State::RUNNING || vesc->getState() != Driver::State::RUNNING ||
            servo->getState() != Driver::State::RUNNING) {
            continue;
        }
        break;
    }
    if (osKernelGetTickCount() >= emergency_max_timeout_stamp) {
        LogError("Drive Controller: Emergency stop timeout expired, unable to recover");
        return false;
    }

    // Wait for the external restart signal
    LogDebug("Drive Controller: Waiting for restart signal after emergency stop");
    uint32_t flags = osThreadFlagsWait(START_THREAD_FLAG, osFlagsWaitAny, emergency_max_timeout_stamp - osKernelGetTickCount());
    if (flags == osFlagsErrorTimeout) {
        LogError("Drive Controller: Emergency stop restart timeout expired, unable to recover");
        return false;
    }
    if ((flags & osFlagsError) != 0U || (flags & START_THREAD_FLAG) == 0U) {
        LogError("Drive Controller: Unexpected error waiting for restart signal, flags: 0x%08lX", flags);
        return false;
    }
    LogInfo("Drive Controller: Emergency stop cleared, resuming operation");
    return true;
}

/**
 * @brief Handle mode switch based on RC channel input
 *
 * @param channel_value The value of the mode switch channel
 * @return true if the mode switch was successful, false otherwise
 */
bool DriveController::handleModeSwitch(uint16_t channel_value) {
    DriveMode new_mode = current_drive_mode;
    if (channel_value > RC_SWITCH_STATE_2_THRESHOLD) {
        new_mode = DriveMode::AUTONOMOUS;
    } else if (channel_value > RC_SWITCH_STATE_1_THRESHOLD) {
        new_mode = DriveMode::MANUAL;
    } else {
        new_mode = DriveMode::IDLE;
    }

    if (new_mode == current_drive_mode) {
        return true;
    }
    if (current_drive_mode == DriveMode::EMERGENCY_STOP || current_drive_mode == DriveMode::MANDATORY_STOP) {
        return false;
    }
    return changeDriveMode(new_mode);
}

/**
 * @brief Handle throttle input from RC channel
 *
 * @param channel_value The value of the throttle channel
 * @return true if the throttle command was sent successfully, false otherwise
 */
bool DriveController::handleManualThrottle(uint16_t channel_value) {
    int32_t centered_input = static_cast<int32_t>(channel_value) - RC_CHANNEL_CENTER_VALUE;

    if (std::abs(centered_input) < static_cast<int32_t>(THROTTLE_DEADZONE)) {
        centered_input = 0;
    } else if (centered_input > 0) {
        centered_input -= (THROTTLE_DEADZONE);
    } else {
        centered_input += (THROTTLE_DEADZONE);
    }

    // Map the centered input to a RPM value in the max RPM range configured in the VESC driver
    int32_t max_rpm = vesc->getConfig().max_rpm;
    int32_t rpm = (centered_input * max_rpm) / static_cast<int32_t>(RC_CHANNEL_MAX_VALUE - RC_CHANNEL_CENTER_VALUE);

    return vesc->setRPM(rpm);
}

/**
 * @brief Handle steering input from RC channel
 *
 * @param channel_value The value of the steering channel
 * @return true if the steering command was sent successfully, false otherwise
 */
bool DriveController::handleManualSteering(uint16_t channel_value) {
    // Map RC channel value (200-1800) to servo angle (-30 to 30 degrees)
    float angle = ((static_cast<float>(channel_value) - RC_CHANNEL_CENTER_VALUE) * 30.0f) /
                  (RC_CHANNEL_MAX_VALUE - RC_CHANNEL_CENTER_VALUE);
    return servo->setAngle(angle);
}

/**
 * @brief Check command age on the local RTOS tick clock.
 */
bool DriveController::isCommandStale(uint32_t timestamp_ticks) const {
    return osKernelGetTickCount() - timestamp_ticks > pdMS_TO_TICKS(config->autonomous_command_timeout_ms);
}

/**
 * @brief Monitor autonomous command age
 *
 * Stale motor commands stop the motor. Stale steering commands release the servo.
 * @return true when all required stop commands succeeded.
 */
bool DriveController::checkAutonomousCommandTimeouts() {
    if (control_mutex == nullptr || osMutexAcquire(control_mutex, osWaitForever) != osOK) {
        return false;
    }
    if (current_drive_mode != DriveMode::AUTONOMOUS) {
        (void)osMutexRelease(control_mutex);
        return true;
    }
    if (!autonomous_control_enabled) {
        stopVehicle();
        (void)osMutexRelease(control_mutex);
        return true;
    }

    // If the motor command is stale, we stop the motor
    bool motor_ok = true;
    if (motor_command.mode == MotorCommand::Mode::NONE || isCommandStale(motor_command.timestamp_ticks)) {
        motor_ok = vesc->setRPM(0);
    }

    // If the steering command is stale, we release the servo but won't center it
    bool steering_ok = true;
    if (isCommandStale(steering_command.timestamp_ticks)) {
        steering_ok = servo->release();
    }
    if (!motor_ok || !steering_ok) {
        stopVehicle();
    }
    (void)osMutexRelease(control_mutex);
    return motor_ok && steering_ok;
}

/**
 * @brief Callback for RC receiver channel updates
 *
 * This function is called whenever new channel data is received from the RC receiver.
 *
 * @param channels The current channel data from the RC receiver
 */
void DriveController::remoteControlCallback(const crsf::ChannelData& channels) {
    if (control_mutex == nullptr || osMutexAcquire(control_mutex, osWaitForever) != osOK) {
        return;
    }
    if (state != State::RUNNING) {
        (void)osMutexRelease(control_mutex);
        return;
    }

    // Handle mode switch
    uint16_t mode_switch_value = channels[config->mode_switch_channel - 1];
    handleModeSwitch(mode_switch_value);

    // Only process throttle and steering in MANUAL mode
    if (current_drive_mode != DriveMode::MANUAL) {
        (void)osMutexRelease(control_mutex);
        return;
    }

    // Send motor and servo commands based on RC input
    uint16_t throttle_value = channels[config->throttle_channel - 1];
    handleManualThrottle(throttle_value);

    uint16_t steering_value = channels[config->steering_channel - 1];
    handleManualSteering(steering_value);
    (void)osMutexRelease(control_mutex);
}

/**
 * @brief Internal helper to set the current drive mode
 *
 * Will additionally update Tracealyzer drive mode state and notify the controller thread.
 *
 * @param mode The new drive mode to set
 */
void DriveController::setDriveMode(DriveMode mode) {
    if (control_mutex == nullptr || osMutexAcquire(control_mutex, osWaitForever) != osOK) {
        return;
    }
    current_drive_mode = mode;
    if (mode != DriveMode::MANUAL) {
        stopVehicle();
    }
    traceDriveMode(mode);
    osThreadFlagsSet(controller_thread, MODE_SWITCH_FLAG);
    (void)osMutexRelease(control_mutex);
}

/**
 * @brief Initialize a native TraceRecorder drive mode state machine
 */
bool DriveController::initDriveModeTracing() {
    if (drive_mode_trace_initialized) {
        return true;
    }
    if (drive_mode_trace_failed) {
        return false;
    }

    if (xTraceStateMachineCreate("Drive Mode", &drive_mode_machine) != TRC_SUCCESS || drive_mode_machine == nullptr) {
        (void)xTracePrintCompactF1("Drive Controller Trace", "Registration failed: %u", 1U);
        drive_mode_trace_failed = true;
        return false;
    }

    for (size_t i = 0; i < DRIVE_MODE_TRACE_STATE_COUNT; i++) {
        if (xTraceStateMachineStateCreate(drive_mode_machine, DRIVE_MODE_TRACE_STATES[i], &drive_mode_trace_states[i]) != TRC_SUCCESS ||
            drive_mode_trace_states[i] == nullptr) {
            (void)xTracePrintCompactF1("Drive Controller Trace", "Registration failed: %u", 2U);
            drive_mode_trace_failed = true;
            return false;
        }
    }

    drive_mode_trace_initialized = true;
    traceDriveMode(current_drive_mode);
    return true;
}

/**
 * @brief Update the native TraceRecorder drive mode state machine
 */
void DriveController::traceDriveMode(DriveMode mode) {
    if (!drive_mode_trace_initialized && !initDriveModeTracing()) {
        return;
    }

    size_t state_index = 0U;
    switch (mode) {
        case DriveMode::IDLE:
            state_index = 0U;
            break;
        case DriveMode::MANUAL:
            state_index = 1U;
            break;
        case DriveMode::AUTONOMOUS:
            state_index = 2U;
            break;
        case DriveMode::MANDATORY_STOP:
            state_index = 3U;
            break;
        case DriveMode::EMERGENCY_STOP:
            state_index = 4U;
            break;
    }

    if (xTraceStateMachineSetState(drive_mode_machine, drive_mode_trace_states[state_index]) != TRC_SUCCESS) {
        (void)xTracePrintCompactF1("Drive Controller Trace", "Registration failed: %u", 3U);
    }
}

/**
 * @brief Main controller thread function
 *
 * This thread manages the drive mode state machine and handles emergency stops.
 *
 * @param arg Pointer to the DriveController instance
 */
void DriveController::controllerThread(void* arg) {
    // Wait for the start signal
    osThreadFlagsWait(START_THREAD_FLAG, osFlagsWaitAny, osWaitForever);

    BlinkAnimation manual_mode_animation(osWaitForever, 1000, 0.5f, config->manual_mode_color);
    BlinkAnimation warning_animation(osWaitForever, 500, 0.5f, COLOR_RED);

    // Wait for the RC receiver to be connected, since this usually takes a bit longer
    if (!rc_receiver->isConnected()) {
        LogInfo("Drive Controller: Waiting for RC receiver to connect...");
        if (!rc_receiver->waitForConnect(MAX_INIT_WAIT_TIME_MS)) {
            LogError("Drive Controller: Timeout waiting for RC receiver to connect, shutting down");
            setState(State::ERROR);
            osDelay(osWaitForever);
        }
    }
    setState(State::RUNNING);
    LogInfo("Drive Controller: Started controller thread, awaiting commands");

    while (state == State::RUNNING) {
        uint32_t wait_timeout_ticks;
        if (current_drive_mode == DriveMode::AUTONOMOUS) {
            wait_timeout_ticks = pdMS_TO_TICKS(config->autonomous_command_timeout_ms);
        } else {
            wait_timeout_ticks = pdMS_TO_TICKS(CONTROLLER_THREAD_UPDATE_TIME_MS);
        }

        uint32_t flags = osThreadFlagsWait(MODE_SWITCH_FLAG, osFlagsWaitAny, wait_timeout_ticks);

        if ((flags & osFlagsError) != 0U && flags != osFlagsErrorTimeout) {
            LogError("Drive Controller: Unexpected error waiting for thread flags, flags: 0x%08lX", flags);
            emergencyStop();
            setState(State::ERROR);
        } else if ((flags & osFlagsError) == 0U && (flags & MODE_SWITCH_FLAG) == 0U) {
            LogError("Drive Controller: Unexpected thread flags received: 0x%08lX", flags);
            emergencyStop();
            setState(State::ERROR);
        }

        // Check if all drivers are currently still running
        if (rc_receiver->getState() != Driver::State::RUNNING || vesc->getState() != Driver::State::RUNNING ||
            servo->getState() != Driver::State::RUNNING) {
            LogError("Drive Controller: One or more drivers not running");
            emergencyStop();
        }
        if (!rc_receiver->isConnected()) {
            LogWarning("Drive Controller: RC Receiver disconnected, performing emergency stop");
            emergencyStop();
        }

        if (flags == osFlagsErrorTimeout && current_drive_mode != DriveMode::AUTONOMOUS) {
            continue;
        }

        // Check if a mode switch was received
        const bool mode_switch_received = (flags & osFlagsError) == 0U && (flags & MODE_SWITCH_FLAG) != 0U;
        if (mode_switch_received) {
            LogDebug("Drive Controller: Drive mode changed to %s",
                     (current_drive_mode == DriveMode::IDLE)             ? "IDLE"
                     : (current_drive_mode == DriveMode::MANUAL)         ? "MANUAL"
                     : (current_drive_mode == DriveMode::AUTONOMOUS)     ? "AUTONOMOUS"
                     : (current_drive_mode == DriveMode::MANDATORY_STOP) ? "MANDATORY_STOP"
                     : (current_drive_mode == DriveMode::EMERGENCY_STOP) ? "EMERGENCY_STOP"
                                                                         : "UNKNOWN");
        }

        switch (current_drive_mode) {
            case DriveMode::IDLE:
                light_dispatcher.turnOff();
                break;

            case DriveMode::MANUAL:
                // Blink the debug light every second to indicate manual mode
                // This is mandatory for the CAuDri-Challenge
                light_dispatcher.runAnimation(&manual_mode_animation);
                break;

            case DriveMode::AUTONOMOUS:
                // Command handling is done in updateMotorCommand/updateSteeringCommand
                if (autonomous_control_enabled) {
                    light_dispatcher.turnOn(config->autonomous_mode_color);
                } else if (light_dispatcher.getCurrentAnimation() != &warning_animation) {
                    light_dispatcher.runAnimation(&warning_animation);
                }
                if (!checkAutonomousCommandTimeouts()) {
                    LogWarning("Drive Controller: Failed to stop outdated autonomous commands");
                }
                break;

            case DriveMode::MANDATORY_STOP:
                light_dispatcher.turnOn(config->mandatory_stop_color);

                // The breaking command has already been sent, we just wait for the timeout to expire
                osDelay(MANDATORY_STOP_TIME_MS);
                if (osMutexAcquire(control_mutex, osWaitForever) != osOK) {
                    LogError("Drive Controller: Failed to lock control mutex after mandatory stop");
                    setState(State::ERROR);
                    break;
                }
                if (current_drive_mode == DriveMode::MANDATORY_STOP) {
                    setDriveMode(DriveMode::MANUAL);
                }
                (void)osMutexRelease(control_mutex);
                break;

            case DriveMode::EMERGENCY_STOP:
                light_dispatcher.turnOn(config->emergency_stop_color);
                if (!handleEmergencyStop()) {
                    setState(State::ERROR);
                } else {
                    // Emergency stop cleared, switch to IDLE mode
                    setDriveMode(DriveMode::IDLE);
                }
                break;

            default:
                LogError("Drive Controller: Unknown drive mode");
                emergencyStop();
                setState(State::ERROR);
                break;
        }
    }
    LogError("Drive Controller: Exiting controller thread");
    setState(State::ERROR);
    osDelay(osWaitForever);
}
