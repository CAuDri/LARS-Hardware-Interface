/**
 * @file config.h
 *
 * @brief CAuDri - Configuration file for the Hardware Interface firmware
 *
 * This file is used to set up all config parameters for hardware drivers and micro-ROS nodes.
 * It defines additional pin mappings and configuration options specific to the Hardware Interface.
 */

#pragma once

#include "client.hpp"
#include "drive_controller.hpp"
#include "main.h"
#include "motor_publisher.hpp"
#include "rc_receiver.hpp"
#include "servo.hpp"
#include "servo_publisher.hpp"
#include "system_monitor.hpp"
#include "usb_cdc_transport.h"
#include "usb_device.h"
#include "vesc.hpp"
#include "ws2812.hpp"

/*****************  RC Channel Mappings *******************/

constexpr crsf::Channel RC_THROTTLE_CHANNEL = crsf::CHANNEL_3;
constexpr crsf::Channel RC_STEERING_CHANNEL = crsf::CHANNEL_1;
constexpr crsf::Channel RC_MODE_SWITCH_CHANNEL = crsf::CHANNEL_7;

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
    .client_thread_priority = osPriorityNormal1,
    .executor_thread_priority = osPriorityRealtime,
};

/***************** ROS Node Configuration ****************/

ServoPublisher::Config servo_publisher_config{
    .publish_period_ms = SERVO_PUBLISHER_DEFAULT_PERIOD_MS,
    .read_failure_threshold = SERVO_PUBLISHER_DEFAULT_READ_FAILURE_THRESHOLD,
    .recovery_probe_interval_ms = SERVO_PUBLISHER_DEFAULT_RECOVERY_PROBE_INTERVAL_MS,
    .thread_priority = osPriorityNormal,
    .publisher_config = {true, 0},
};

MotorPublisher::Config motor_publisher_config{
    .publish_period_ms = MOTOR_PUBLISHER_DEFAULT_PERIOD_MS,
    .telemetry_period_ms = MOTOR_PUBLISHER_DEFAULT_TELEMETRY_PERIOD_MS,
    .thread_priority = osPriorityNormal,
};

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

/***************** Driver Configurations *******************/

RCReceiver::Config rc_config{
    .huart = &huart2,
    .baud_rate = 420000,
    .task_priority = osPriorityHigh,
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

WS2812Driver::Config ws2812_config{
    .htim = &htim4,
    .tim_channel = TIM_CHANNEL_3,
    .hdma = &hdma_tim4_ch3,
};
