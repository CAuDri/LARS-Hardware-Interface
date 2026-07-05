/**
 * @file main.cpp
 *
 * @brief CAuDri - Main entry point for the application code
 */

#include "main.h"

#include <algorithm>
#include <bitset>

#include <std_msgs/msg/u_int32.h>

// #include "blink_animation.hpp"
#include "config/config.h"
#include "gpio_light.hpp"
#include "light_dispatcher.hpp"
#include "logger.h"
#include "node.hpp"
#include "pulse_animation.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "thread_safe_adc.h"
#include "type_support.hpp"
#include "ws2812_light.hpp"

ROS_DECLARE_MESSAGE_TYPE(std_msgs, UInt32);

/**
 * Forward function declarations
 */
extern "C" void mainTask();
void onSystemStateChange(SystemCheck::SystemState state);
void onMicrorosTestCommand(const std_msgs__msg__UInt32* message, void* context);

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

ros::Client microros_client;
ros::Node microros_hardware_node;
ros::Publisher<std_msgs__msg__UInt32> microros_heartbeat_publisher;
ros::BasePublisher::Config microros_heartbeat_config{true, 5};
std_msgs__msg__UInt32 microros_heartbeat_message{};
ros::Subscriber<std_msgs__msg__UInt32> microros_test_subscriber;
ros::BaseSubscriber::Config microros_test_subscriber_config{true};

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
     * Initialize all peripheral drivers
     */
    ws2812_top.init(ws2812_config);
    ws2812_top.setColor(COLOR_GREEN);

    rc_receiver.init(rc_config);
    rc_receiver.start();

    motor.init(vesc_config);
    motor.start();

    servo.init(servo_config, servo_calibration);
    servo.start();

    const rcl_ret_t microros_result = microros_client.init(microros_client_config);
    if (microros_result != RCL_RET_OK) {
        LogError("Main: Failed to initialize micro-ROS client: %d", static_cast<int>(microros_result));
    }
    rcl_ret_t microros_entity_result = microros_hardware_node.init(microros_client, "hardware_interface");
    if (microros_entity_result != RCL_RET_OK) {
        LogError("Main: Failed to initialize micro-ROS test node: %d", static_cast<int>(microros_entity_result));
    }
    microros_entity_result = microros_heartbeat_publisher.init(microros_hardware_node, "heartbeat", microros_heartbeat_config);
    if (microros_entity_result != RCL_RET_OK) {
        LogError("Main: Failed to initialize micro-ROS heartbeat publisher: %d", static_cast<int>(microros_entity_result));
    }
    microros_entity_result = microros_test_subscriber.init(
        microros_hardware_node, "command/test", onMicrorosTestCommand, nullptr, microros_test_subscriber_config);
    if (microros_entity_result != RCL_RET_OK) {
        LogError("Main: Failed to initialize micro-ROS test subscriber: %d", static_cast<int>(microros_entity_result));
    }

    /**
     * Register components with the system check for monitoring
     */
    system_check.registerDriver(rc_receiver, true);
    system_check.registerDriver(motor, true);
    system_check.registerDriver(servo, false);
    system_check.registerDriver(ws2812_top, false);

    /**
     * Initialize and start the system monitor and high-level drive control
     */
    drive_controller.init(drive_controller_config, rc_receiver, motor, servo, onboard_led_1);

    osDelay(500);

    system_monitor.init(system_check, system_monitor_config);
    system_monitor.registerSystemStateCallback(onSystemStateChange);
    system_monitor.start();

    LogInfo("Main: Initialization complete");

    /**
     * Main loop - nothing is actually done here, all functionality is handled in background tasks.
     * Here you can add any additional code for debugging or testing purposes.
     */
    while (true) {
        osDelay(1000);
        microros_heartbeat_message.data++;
        const rcl_ret_t heartbeat_result = microros_heartbeat_publisher.publish(microros_heartbeat_message);
        if (heartbeat_result != RCL_RET_OK && heartbeat_result != RCL_RET_NOT_INIT && heartbeat_result != RCL_RET_TIMEOUT) {
            LogWarning("Main: Failed to publish micro-ROS heartbeat: %d", static_cast<int>(heartbeat_result));
        }
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

/**
 * @brief Receive a harmless test command from ROS to validate subscriber dispatch.
 * @param message Received UInt32 message.
 * @param context Optional callback context, unused for this test subscriber.
 */
void onMicrorosTestCommand(const std_msgs__msg__UInt32* message, void* context) {
    (void)context;
    if (message != nullptr) {
        LogInfo("Main: Received micro-ROS test command: %lu", message->data);
    }
}
