/**
 * @file thread_safe_i2c.h
 *
 * @brief CAuDri - Asynchronous I2C master wrapper for multi-threaded applications
 */
#pragma once

#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

bool I2C_Init();
HAL_StatusTypeDef I2C_Transmit(I2C_HandleTypeDef* hi2c, uint16_t address, uint8_t* data, uint16_t size);
HAL_StatusTypeDef I2C_Receive(I2C_HandleTypeDef* hi2c, uint16_t address, uint8_t* data, uint16_t size);
HAL_StatusTypeDef I2C_MemWrite(I2C_HandleTypeDef* hi2c, uint16_t address, uint16_t memAddress, uint8_t* data, uint16_t size);
HAL_StatusTypeDef I2C_MemRead(I2C_HandleTypeDef* hi2c, uint16_t address, uint16_t memAddress, uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif