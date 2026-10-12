/**
 * @file config.h
 *
 * @brief CAuDri - Configuration file for the Hardware Interface firmware
 *
 * This file is used to set up all config parameters for hardware drivers and micro-ROS nodes.
 * It defines additional pin mappings and configuration options specific to the Hardware Interface.
 */

#pragma once

#include "autonomous_control.hpp"
#include "bno055.hpp"
#include "client.hpp"
#include "drive_controller.hpp"
#include "imu_publisher.hpp"
#include "main.h"
#include "motor_publisher.hpp"
#include "rc_receiver.hpp"
#include "servo.hpp"
#include "servo_publisher.hpp"
#include "system_monitor.hpp"
#include "system_node.hpp"
#include "tof_publisher.hpp"
#include "usb_cdc_transport.h"
#include "usb_device.h"
#include "vesc.hpp"
#include "vl53l5cx.hpp"
#include "ws2812.hpp"

/*****************  RC Channel Mappings *******************/

constexpr crsf::Channel RC_THROTTLE_CHANNEL = crsf::CHANNEL_3;
constexpr crsf::Channel RC_STEERING_CHANNEL = crsf::CHANNEL_1;
constexpr crsf::Channel RC_MODE_SWITCH_CHANNEL = crsf::CHANNEL_7;

/***************** System Configuration ****************/

SystemMonitor::Config system_monitor_config{
    .reset_gpio_port = PWR_EXT_ENABLE_GPIO_Port,
    .reset_gpio_pin = PWR_EXT_ENABLE_Pin,
    .check_interval_ms = 500,
};

DriveController::Config drive_controller_config{
    .throttle_channel = RC_THROTTLE_CHANNEL,
    .steering_channel = RC_STEERING_CHANNEL,
    .mode_switch_channel = RC_MODE_SWITCH_CHANNEL,
};

/***************** Micro-ROS Configuration ****************/

usb_cdc_transport_config_t microros_usb_transport_config{
    .usb_device = &hUsbHostPort,
    .rx_dma = &hdma_memtomem_dma2_stream3,
};

ros::Client::Config microros_client_config{
    .transport =
        {
            .framing = true,
            .context = &microros_usb_transport_config,
            .open = usb_cdc_transport_open,
            .close = usb_cdc_transport_close,
            .write = usb_cdc_transport_write,
            .read = usb_cdc_transport_read,
        },
    .base_namespace = "/hardware",
    .client_thread_priority = osPriorityHigh,
    .executor_thread_priority = osPriorityRealtime,
};

/***************** Driver Configurations *******************/

RCReceiver::Config rc_config{
    .huart = &huart2,
    .baud_rate = 420000,
    .task_priority = osPriorityRealtime1,
};

VESC::Config vesc_config{
    .hcan = &hcan1,
    .vesc_id = 51,
    .max_rpm = 10000,
};

Servo::Config servo_config{
    .htim = &htim3,
    .tim_channel = TIM_CHANNEL_1,
    .hadc = &hadc2,
    .adc_channel = ADC_CHANNEL_4,
    .inverted = false,
};

Servo::Calibration servo_calibration{
    .min_angle = -30.0f,
    .max_angle = 30.0f,
    .min_pulse_us = 1100,
    .center_pulse_us = 1500,
    .max_pulse_us = 1900,
};

BNO055::Config bno055_config{
    .hi2c = &hi2c2,
    .address = BNO055_I2C_ADDR_LO,
    .reset_port = I2C2_GPIO4_GPIO_Port,
    .reset_pin = I2C2_GPIO4_Pin,
    .interrupt_port = I2C2_GPIO3_GPIO_Port,
    .interrupt_pin = I2C2_GPIO3_Pin,
};

VL53L5CX::Config vl53l5cx_config{
    .hi2c = &hi2c1,
    .address = 0x10,
    .lpn_port = I2C1_GPIO8_GPIO_Port,
    .lpn_pin = I2C1_GPIO8_Pin,
    .interrupt_port = I2C1_GPIO7_GPIO_Port,
    .interrupt_pin = I2C1_GPIO7_Pin,
};

WS2812Driver::Config ws2812_config{
    .htim = &htim4,
    .tim_channel = TIM_CHANNEL_3,
    .hdma = &hdma_tim4_ch3,
};

/***************** ROS Node Configuration ****************/

ServoPublisher::Config servo_publisher_config{
    .publish_period_ms = SERVO_PUBLISHER_DEFAULT_PERIOD_MS,
    .thread_priority = osPriorityAboveNormal,
    .publisher_config = {true, 0},
};

ImuPublisher::Config imu_publisher_config{
    .publish_period_ms = IMU_PUBLISHER_DEFAULT_PERIOD_MS,
    .thread_priority = osPriorityNormal1,
    .publisher_config = {true, 0},
};

TofPublisher::Config tof_publisher_config{
    .publish_period_ms = TOF_PUBLISHER_DEFAULT_PERIOD_MS,
    .thread_priority = osPriorityNormal,
    .publisher_config = {false, 0},
};

MotorPublisher::Config motor_publisher_config{
    .publish_period_ms = MOTOR_PUBLISHER_DEFAULT_PERIOD_MS,
    .telemetry_period_ms = MOTOR_PUBLISHER_DEFAULT_TELEMETRY_PERIOD_MS,
    .thread_priority = osPriorityAboveNormal1,
};

AutonomousControl::Config autonomous_control_config{
    .motor_rpm_topic = AUTONOMOUS_CONTROL_DEFAULT_MOTOR_RPM_TOPIC,
    .motor_current_topic = AUTONOMOUS_CONTROL_DEFAULT_MOTOR_CURRENT_TOPIC,
    .steering_angle_topic = AUTONOMOUS_CONTROL_DEFAULT_STEERING_ANGLE_TOPIC,
    .subscriber_config = {},
};

SystemNode::Config system_node_config{
    // TODO: Revert after demo
    .thread_priority = osPriorityAboveNormal2,
    .heartbeat_publisher_config = {true, 0},
    .diagnostics_publisher_config = {false, 5},
    .service_config = {},
};
