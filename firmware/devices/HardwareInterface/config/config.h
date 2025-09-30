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

RCReceiver::Config rc_config{
    .huart = &huart2,
    .baud_rate = 420000,
    .task_priority = osPriorityHigh,
};