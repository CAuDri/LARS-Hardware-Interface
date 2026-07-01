/**
 * @file drive_controller.cpp
 *
 * @brief CAuDri - Drive controller node for handling drive commands and drive modes.
 *
 * This class manages the drive modes of the vehicle (i.e. idle, manual, autonomous) and adds a bit of safety
 * around the control commands received from the RC receiver and autonomous driving stack.
 *
 * It ensures that the vehicle can be safely stopped in case of emergencies and will perform mandatory stops when
 * switching between drive modes as required by the CAuDri-Challenge regulations.
 *
 * This implementation is tightly coupled to the RCReceiver, VESC, and Servo drivers.
 * Driver interfaces could be used, if you want to make it more generic and reusable for other driver implementations.
 */
#include "drive_controller.hpp"

#include "blink_animation.hpp"
#include "logger.h"

static constexpr uint32_t MANDATORY_STOP_TIME_MS = 1000;  // Time to wait after switching to manual mode (CAuDri regulations)

static constexpr uint32_t EMERGENCY_STOP_TIME_MS = 5000;  // Time to wait after emergency stop before allowing restart
static constexpr uint32_t EMERGENCY_MAX_TIMEOUT_MS = 60000;  // Maximum time to wait in emergency stop before error is raised

static constexpr uint32_t MAX_INIT_WAIT_TIME_MS = 60000;  // Maximum time to wait for initialization (e.g. RC connection)

static constexpr uint32_t RC_SWITCH_STATE_0_THRESHOLD = 0;     // Threshold for RC switch state 0
static constexpr uint32_t RC_SWITCH_STATE_1_THRESHOLD = 900;   // Threshold for RC switch state 1
static constexpr uint32_t RC_SWITCH_STATE_2_THRESHOLD = 1500;  // Threshold for RC switch state 2

static constexpr uint32_t RC_CHANNEL_MIN_VALUE = 200;      // Minimum value for RC channel (joystick low)
static constexpr uint32_t RC_CHANNEL_CENTER_VALUE = 1000;  // Center value for RC channel (joystick center)
static constexpr uint32_t RC_CHANNEL_MAX_VALUE = 1800;     // Maximum value for RC channel (joystick high)

static constexpr uint32_t THROTTLE_DEADZONE = 50;  // Deadzone around center for throttle channel

static constexpr uint32_t CONTROLLER_THREAD_UPDATE_TIME_MS = 100;  // Timeout for the controller thread update loop

// Thread flags for the controller thread
static constexpr uint32_t START_THREAD_FLAG = 0x01;
static constexpr uint32_t MODE_SWITCH_FLAG = 0x02;

/**
 * @brief Construct a new drive controller object for later initialization
 */
DriveController::DriveController() {}


/**
 * @brief Construct a new drive controller object with configuration and driver references
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
 * @brief Destroy the drive controller object and terminate the controller thread
 */
DriveController::~DriveController() {
    osThreadTerminate(controller_thread);
    setState(NodeState::UNINITIALIZED);
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
    if (state != NodeState::UNINITIALIZED) {
        return false;
    }
    setState(NodeState::INITIALIZING);

    this->config = &config;
    if (config.throttle_channel == crsf::INVALID_CHANNEL || config.steering_channel == crsf::INVALID_CHANNEL ||
        config.mode_switch_channel == crsf::INVALID_CHANNEL) {
        LogError("Drive Controller: Invalid channel configuration");
        setState(NodeState::ERROR);
        return false;
    }

    this->rc_receiver = &rc_receiver;
    this->vesc = &vesc;
    this->servo = &servo;

    // Register the RC receiver callback
    rc_callback = RCReceiver::ChannelCallback::from<DriveController, &DriveController::remoteControlCallback>(this);
    if (!this->rc_receiver->registerChannelCallback(rc_callback)) {
        LogError("Drive Controller: Failed to register RC receiver channel callback");
        setState(NodeState::ERROR);
        return false;
    }

    // Register the Tracealyzer channel for drive mode logging
    if (xTraceStringRegister("Drive Mode", &drive_mode_channel) != TRC_SUCCESS) {
        LogDebug("Drive Controller: Failed to register Tracealyzer channel for drive mode");
    }

    // Initialize the light dispatcher with the status light
    if (!light_dispatcher.registerLight(status_light)) {
        LogWarning("Drive Controller: Failed to register status light with light dispatcher");
    }

    thread_attributes.name = "Drive Controller";
    thread_attributes.priority = config.thread_priority;
    thread_attributes.stack_mem = &thread_stack;
    thread_attributes.stack_size = sizeof(thread_stack);
    thread_attributes.cb_mem = &thread_control_block;
    thread_attributes.cb_size = sizeof(thread_control_block);

    controller_thread = osThreadNew(
        // Helper function for using a non-static method as the thread entry point
        // The 'this' pointer is passed as the user argument to the lambda
        [](void* arg) -> void {
            auto* obj = static_cast<DriveController*>(arg);
            obj->controllerThread(arg);
        },
        this,
        &thread_attributes);
    if (controller_thread == nullptr) {
        LogError("Drive Controller: Failed to create controller thread");
        setState(NodeState::ERROR);
        return false;
    }
    return true;
}

/**
 * @brief Start the drive controller thread
 *
 * This will set the controller state to RUNNING and signal the controller thread to begin operation.
 */
bool DriveController::start() {
    if (state != NodeState::INITIALIZING) {
        LogError("Drive Controller: Cannot start, driver not initialized");
        return false;
    }

    // Check if any of the drivers are in ERROR state
    if (rc_receiver->getState() == Driver::State::ERROR || vesc->getState() == Driver::State::ERROR ||
        servo->getState() == Driver::State::ERROR) {
        LogError("Drive Controller: Cannot start, one or more drivers in ERROR state");
        setState(NodeState::ERROR);
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
    if (state != NodeState::RUNNING) {
        LogWarning("Drive Controller: Cannot restart, controller not running");
        return false;
    }
    osThreadFlagsSet(controller_thread, START_THREAD_FLAG);
    return true;
}

/**
 * @brief Temporary helper until the micro-ROS nodes are implemented
 *
 * TODO: Remove this function and use proper micro-ROS node state management
 *
 * @param new_state The new state to set
 */
bool DriveController::setState(NodeState new_state) {
    if (state == new_state) {
        return true;
    }
    if (state == NodeState::ERROR) {
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
    if (state != NodeState::RUNNING) {
        LogWarning("Drive Controller: Cannot set drive mode, controller not running");
        return false;
    }
    if (mode == current_drive_mode) {
        return true;
    }

    if (mode == DriveMode::EMERGENCY_STOP) {
        emergencyStop();
        return true;
    } else if (mode == DriveMode::IDLE) {
        stopVehicle();
    }

    // Emergency stop must be cleared by the controller thread itself
    if (current_drive_mode == DriveMode::EMERGENCY_STOP) {
        LogWarning("Drive Controller: Cannot manually change drive mode from EMERGENCY_STOP");
        return false;
    }

    // The CAuDri-Challenge requires a mandatory stop when switching to manual mode
    // We ensure this by adding an intermediate MANDATORY_STOP state
    if (current_drive_mode == DriveMode::AUTONOMOUS && (mode == DriveMode::MANUAL || mode == DriveMode::IDLE)) {
        setDriveMode(DriveMode::MANDATORY_STOP);
        stopVehicle();
        return true;
    };
    // The controller thread will do the actual mode switch later
    if (current_drive_mode == DriveMode::MANDATORY_STOP) {
        return false;
    }

    setDriveMode(mode);
    return true;
}

/**
 * @brief Trigger an emergency stop procedure
 *
 * This will immediately stop the vehicle and start a recovery procedure.
 * In case of non-recoverable errors, the controller will enter the ERROR state.
 */
void DriveController::emergencyStop() {
    LogDebug("Drive Controller: Emergency stop triggered");
    setDriveMode(DriveMode::EMERGENCY_STOP);

    vesc->setRPM(0);
    servo->setAngle(0.0f);
}

/**
 * @brief Immediately stop the vehicle without changing the drive mode
 *
 * This is used internally during emergency stop procedures.
 */
void DriveController::stopVehicle() {
    vesc->setRPM(0);
    servo->setAngle(0.0f);
}

/**
 * @brief Handle the emergency stop procedure
 *
 * This will stop the vehicle, wait for recovery conditions, and handle any errors.
 */
bool DriveController::handleEmergencyStop() {
    stopVehicle();

    // Clear all currently pending thread flags
    osThreadFlagsClear(0xFFFFFFFF);

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
    if (!(flags & START_THREAD_FLAG)) {
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
bool DriveController::handleThrottle(uint16_t channel_value) {
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
bool DriveController::handleSteering(uint16_t channel_value) {
    // Map RC channel value (200-1800) to servo angle (-30 to 30 degrees)
    float angle = ((static_cast<float>(channel_value) - RC_CHANNEL_CENTER_VALUE) * 30.0f) /
                  (RC_CHANNEL_MAX_VALUE - RC_CHANNEL_CENTER_VALUE);
    return servo->setAngle(angle);
}

/**
 * @brief Callback for RC receiver channel updates
 *
 * This function is called whenever new channel data is received from the RC receiver.
 * It processes the relevant channels for drive control and updates the vehicle state accordingly.
 *
 * @param channels The current channel data from the RC receiver
 */
void DriveController::remoteControlCallback(const crsf::ChannelData& channels) {
    if (state != NodeState::RUNNING) {
        return;
    }

    last_rc_update_timestamp = osKernelGetTickCount();

    // Handle mode switch
    uint16_t mode_switch_value = channels[config->mode_switch_channel - 1];
    handleModeSwitch(mode_switch_value);

    // Only process throttle and steering in MANUAL mode
    if (current_drive_mode != DriveMode::MANUAL) {
        return;
    }

    // Send motor and servo commands based on RC input
    uint16_t throttle_value = channels[config->throttle_channel - 1];
    handleThrottle(throttle_value);

    uint16_t steering_value = channels[config->steering_channel - 1];
    handleSteering(steering_value);
}

/**
 * @brief Internal helper to set the current drive mode
 *
 * Will additionally create a Tracealyzer event for the mode change and notify the controller thread.
 *
 * @param mode The new drive mode to set
 */
void DriveController::setDriveMode(DriveMode mode) {
    current_drive_mode = mode;
    if (drive_mode_channel != nullptr) {
        switch (mode) {
            case DriveMode::IDLE:
                xTracePrint(drive_mode_channel, "IDLE");
                break;
            case DriveMode::MANUAL:
                xTracePrint(drive_mode_channel, "MANUAL");
                break;
            case DriveMode::AUTONOMOUS:
                xTracePrint(drive_mode_channel, "AUTONOMOUS");
                break;
            case DriveMode::MANDATORY_STOP:
                xTracePrint(drive_mode_channel, "MANDATORY_STOP");
                break;
            case DriveMode::EMERGENCY_STOP:
                xTracePrint(drive_mode_channel, "EMERGENCY_STOP");
                break;
            default:
                xTracePrint(drive_mode_channel, "INVALID");
                break;
        }
    }
    osThreadFlagsSet(controller_thread, MODE_SWITCH_FLAG);
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

    // Wait for the RC receiver to be connected, since this usually takes a bit longer
    if (!rc_receiver->isConnected()) {
        LogInfo("Drive Controller: Waiting for RC receiver to connect...");
        if (!rc_receiver->waitForConnect(MAX_INIT_WAIT_TIME_MS)) {
            LogError("Drive Controller: Timeout waiting for RC receiver to connect, shutting down");
            setState(NodeState::ERROR);
            osDelay(osWaitForever);
        }
    }
    setState(NodeState::RUNNING);
    LogInfo("Drive Controller: Started controller thread, awaiting commands");

    while (state == NodeState::RUNNING) {
        // The thread will be notified of mode switches to speed up the response time
        uint32_t flags = osThreadFlagsWait(MODE_SWITCH_FLAG, osFlagsWaitAny, CONTROLLER_THREAD_UPDATE_TIME_MS);
        if (!(flags & MODE_SWITCH_FLAG || flags == osFlagsErrorTimeout)) {
            LogError("Drive Controller: Unexpected error waiting for thread flags, flags: 0x%08lX", flags);
            emergencyStop();
            setState(NodeState::ERROR);
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

        if (flags == osFlagsErrorTimeout) {
            continue;
        }

        LogDebug("Drive Controller: Drive mode changed to %s",
                 (current_drive_mode == DriveMode::IDLE)             ? "IDLE"
                 : (current_drive_mode == DriveMode::MANUAL)         ? "MANUAL"
                 : (current_drive_mode == DriveMode::AUTONOMOUS)     ? "AUTONOMOUS"
                 : (current_drive_mode == DriveMode::MANDATORY_STOP) ? "MANDATORY_STOP"
                 : (current_drive_mode == DriveMode::EMERGENCY_STOP) ? "EMERGENCY_STOP"
                                                                     : "UNKNOWN");

        switch (current_drive_mode) {
            case DriveMode::IDLE:
                light_dispatcher.turnOff();
                break;

            case DriveMode::MANUAL:
                light_dispatcher.runAnimation(&manual_mode_animation);
                // Blink the debug light every second to indicate manual mode
                // This is mandatory for the CAuDri-Challenge
                // TODO: Add light
                break;

            case DriveMode::AUTONOMOUS:
                light_dispatcher.turnOn(config->autonomous_mode_color);
                break;

            case DriveMode::MANDATORY_STOP:
                light_dispatcher.turnOn(config->mandatory_stop_color);

                // The breaking command has already been sent, we just wait for the timeout to expire
                osDelay(MANDATORY_STOP_TIME_MS);
                setDriveMode(DriveMode::MANUAL);
                break;

            case DriveMode::EMERGENCY_STOP:
                light_dispatcher.turnOn(config->emergency_stop_color);
                if (!handleEmergencyStop()) {
                    setState(NodeState::ERROR);
                } else {
                    // Emergency stop cleared, switch to IDLE mode
                    setDriveMode(DriveMode::IDLE);
                }
                break;

            default:
                LogError("Drive Controller: Unknown drive mode");
                emergencyStop();
                setState(NodeState::ERROR);
                break;
        }
    }
    LogError("Drive Controller: Exiting controller thread");
    setState(NodeState::ERROR);
    osDelay(osWaitForever);
}