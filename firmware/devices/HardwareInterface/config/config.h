/**
 * @file config.h
 *
 * @brief CAuDri - Configuration file for the Hardware Interface firmware
 *
 * This file is used to set up all config parameters for hardware drivers and micro-ROS nodes.
 * It defines additional pin mappings and configuration options specific to the Hardware Interface.
 */

#pragma once

#include "main.h"
#include "rc_receiver.hpp"
#include "servo.hpp"
#include "vesc.hpp"

RCReceiver::Config rc_config{
    .huart = &huart2,
    .baud_rate = 420000,
    .task_priority = osPriorityHigh,
};

VESC::Config vesc_config{
    .hcan = &hcan1,
    .vesc_id = 51,
};

Servo::Config servo_config{
    .htim = &htim3,
    .tim_channel = TIM_CHANNEL_1,
    .inverted = false,
};

Servo::Calibration servo_calibration{
    .min_angle = -30.0f,
    .max_angle = 30.0f,
    .min_pulse_us = 1100,
    .center_pulse_us = 1500,
    .max_pulse_us = 1900,
};