/**
 * @file main.cpp
 *
 * @brief CAuDri - Main entry point for the application code
 */

#include "main.h"

// #include "blink_animation.hpp"
#include "autonomous_control.hpp"
#include "config/config.h"
#include "gpio_light.hpp"
#include "light_dispatcher.hpp"
#include "logger.h"
#if DEBUG_LOG_OUTPUT == LOG_OUTPUT_ROS
    #include "logger_node.hpp"
#endif
#include "motor_publisher.hpp"
#include "pulse_animation.hpp"
#include "servo_publisher.hpp"
#include "system_node.hpp"
#include "ws2812_light.hpp"

/**
 * Forward function declarations
 */
extern "C" void mainTask();
void onSystemStateChange(SystemCheck::SystemState state);

/**
 * All global objects that can be statically initialized
 */
WS2812<13> ws2812_top("WS2812 Top");  // WS2812 driver for the top LED string

RCReceiver rc_receiver;      // RC Receiver driver for handling remote control input
VESC motor("VESC Driver");   // VESC driver for motor control
Servo servo("Servo Front");  // Servo driver for steering control and feedback

DriveController drive_controller;  // Drive controller for managing vehicle drive modes and high-level control

SystemCheck system_check;      // System check utility for monitoring states of various drivers and components
SystemMonitor system_monitor;  // System monitor for periodic system checks and handling system state changes

WS2812Light<1> onboard_led_1(ws2812_top, 0);  // Onboard RGB LED 1
WS2812Light<6> left_test_lights(ws2812_top, 1);
WS2812Light<6> right_test_lights(ws2812_top, 7);

GPIOLight debug_led_red(DEBUG_LED_RED_GPIO_Port, DEBUG_LED_RED_Pin, COLOR_RED);          // Onboard debug LED (red)
GPIOLight debug_led_green(DEBUG_LED_GREEN_GPIO_Port, DEBUG_LED_GREEN_Pin, COLOR_GREEN);  // Onboard debug LED (green)
GPIOLight debug_led_blue(DEBUG_LED_BLUE_GPIO_Port, DEBUG_LED_BLUE_Pin, COLOR_BLUE);      // Onboard debug LED (blue)

LightDispatcher light_dispatcher("Light Dispatcher");

ros::Client microros_client;              // micro-ROS client for communication with the ROS 2 agent

#if DEBUG_LOG_OUTPUT == LOG_OUTPUT_ROS
LoggerNode logger_node;                   // ROS 2 node for publishing firmware log messages to /rosout
#endif
ServoPublisher servo_publisher;           // ROS 2 node for publishing servo feedback messages
MotorPublisher motor_publisher;           // ROS 2 node for publishing motor feedback and telemetry messages
AutonomousControl autonomous_control;     // ROS 2 node for receiving autonomous drive commands
SystemNode system_node;                   // ROS 2 node for heartbeat and hardware operation services

/**
 * @brief Main entry point called from the RTOS task in the auto-generated main.c
 */
void mainTask() {
    /**
     * Application-specific startup checks
     */
    LogClear();
    LogInfo("Main: Starting...");

    // Enter bootloader mode if the user button is pressed during startup
    if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_RESET) {
        LogWarning("Main: User button pressed during startup, entering bootloader...");
        SystemMonitor::enterBootloader();
    }

    // On first power-up no hardware reset of the peripherals is necessary
    if (SystemMonitor::getWakeupReason() != SystemWakeupReason::BROWN_OUT_RESET) {
        LogInfo("Main: Resetting peripherals...");
        SystemMonitor::resetPeripherals(500);
    } else {
        SystemMonitor::resetPeripherals(0);
    }

    /**
     * Initialize the micro-ROS client and start agent discovery
     */
    microros_client.init(microros_client_config);

    /**
     * Initialize all peripheral drivers
     */
    ws2812_top.init(ws2812_config);
    ws2812_top.setColor(COLOR_ORANGE);

    rc_receiver.init(rc_config);
    rc_receiver.start();

    motor.init(vesc_config);
    motor.start();

    servo.init(servo_config, servo_calibration);
    servo.start();

    drive_controller.init(drive_controller_config, rc_receiver, motor, servo, onboard_led_1);

    /**
     * Initialize all micro-ROS nodes
     */
    #if DEBUG_LOG_OUTPUT == LOG_OUTPUT_ROS
    logger_node.init(microros_client);
    #endif

    servo_publisher.init(microros_client, servo_publisher_config);
    servo_publisher.registerServo(servo, "measure/steering_angle", "servo");
    servo_publisher.start();

    motor_publisher.init(microros_client, motor_publisher_config);
    motor_publisher.registerMotor(motor, "measure/motor_feedback", "telemetry/motor", "motor");
    motor_publisher.start();

    autonomous_control.init(microros_client, drive_controller, autonomous_control_config);

    system_node.init(microros_client, drive_controller, system_monitor, system_node_config);
    system_node.start();

    /**
     * Register components with the system check for monitoring
     */
    system_check.registerDriver(rc_receiver, true);
    system_check.registerDriver(motor, true);
    system_check.registerDriver(servo, false);
    system_check.registerDriver(ws2812_top, false);
    system_check.registerClient(microros_client);
    #if DEBUG_LOG_OUTPUT == LOG_OUTPUT_ROS
    system_check.registerNode(logger_node);
    #endif
    system_check.registerNode(servo_publisher);
    system_check.registerNode(motor_publisher);
    system_check.registerNode(autonomous_control);
    system_check.registerNode(system_node);

    /**
     * Initialize and start the system monitor and high-level drive control
     */
    osDelay(500);

    system_monitor.init(system_check, system_monitor_config);
    system_monitor.registerSystemStateCallback(onSystemStateChange);
    system_monitor.start();

    LogInfo("Main: Initialization complete");

    /**
     * Main loop - all application functionality is handled by background threads.
     */
    while (true) {
        osDelay(1000);
        debug_led_green.turnOn();
        osDelay(100);
        debug_led_green.turnOff();
    }
}

/**
 * @brief Callback function for system state changes
 *
 * Will be called by the SystemMonitor whenever the system check detects a change in overall system state.
 * The possible states are:
 * - OK: All components (drivers, nodes, etc.) are functioning properly
 * - WARNING: One or more components are in a non-critical error state
 * - ERROR: One or more components are in a critical error state, it is unsafe to operate the vehicle
 *
 * @param state The new system state
 */
void onSystemStateChange(SystemCheck::SystemState state) {
    switch (state) {
        case SystemCheck::SystemState::OK:
            LogInfo("Main: System state OK, starting drive controller");
            drive_controller.start();
            break;
        case SystemCheck::SystemState::WARNING:
            LogWarning("Main: System state WARNING, disabling autonomous control");
            // TODO: Implement behavior for WARNING state
            break;
        case SystemCheck::SystemState::ERROR:
            LogError("Main: System state ERROR, stopping drive controller");
            drive_controller.emergencyStop();
            break;
    }
}
