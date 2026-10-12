/**
 * @file platform.h
 *
 * @brief CAuDri platform adapter for the ST VL53L5CX ULD
 */
#pragma once

#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef struct {
    uint16_t address;
    I2C_HandleTypeDef* hi2c;
} VL53L5CX_Platform;

#define VL53L5CX_NB_TARGET_PER_ZONE 1U

// Keep only outputs used by the generic ToF interface and validity filtering.
#define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L5CX_DISABLE_RANGE_SIGMA_MM
#define VL53L5CX_DISABLE_MOTION_INDICATOR

uint8_t VL53L5CX_RdByte(VL53L5CX_Platform* platform, uint16_t address, uint8_t* value);
uint8_t VL53L5CX_WrByte(VL53L5CX_Platform* platform, uint16_t address, uint8_t value);
uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform* platform, uint16_t address, uint8_t* values, uint32_t size);
uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform* platform, uint16_t address, uint8_t* values, uint32_t size);
void VL53L5CX_SwapBuffer(uint8_t* buffer, uint16_t size);
uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform* platform, uint32_t wait_ms);
