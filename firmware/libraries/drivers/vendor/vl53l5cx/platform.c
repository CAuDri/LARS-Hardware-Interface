/**
 * @file platform.c
 *
 * @brief CAuDri platform adapter for the ST VL53L5CX ULD
 */
#include "platform.h"

#include <cmsis_os2.h>
#include <stddef.h>
#include <string.h>

#include "thread_safe_i2c.h"

static uint8_t validate_transfer(VL53L5CX_Platform* platform, uint8_t* values, uint32_t size) {
    return platform != NULL && platform->hi2c != NULL && values != NULL && size > 0U && size <= UINT16_MAX;
}

uint8_t VL53L5CX_RdByte(VL53L5CX_Platform* platform, uint16_t address, uint8_t* value) {
    return VL53L5CX_RdMulti(platform, address, value, 1U);
}

uint8_t VL53L5CX_WrByte(VL53L5CX_Platform* platform, uint16_t address, uint8_t value) {
    return VL53L5CX_WrMulti(platform, address, &value, 1U);
}

uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform* platform, uint16_t address, uint8_t* values, uint32_t size) {
    if (!validate_transfer(platform, values, size)) {
        return (uint8_t)HAL_ERROR;
    }
    return (uint8_t)I2C_MemRead(platform->hi2c, platform->address, address, values, (uint16_t)size);
}

uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform* platform, uint16_t address, uint8_t* values, uint32_t size) {
    if (!validate_transfer(platform, values, size)) {
        return (uint8_t)HAL_ERROR;
    }
    return (uint8_t)I2C_MemWrite(platform->hi2c, platform->address, address, values, (uint16_t)size);
}

void VL53L5CX_SwapBuffer(uint8_t* buffer, uint16_t size) {
    if (buffer == NULL || (size % sizeof(uint32_t)) != 0U) {
        return;
    }

    for (uint16_t offset = 0; offset < size; offset += sizeof(uint32_t)) {
        const uint32_t swapped = ((uint32_t)buffer[offset] << 24U) |
                                 ((uint32_t)buffer[offset + 1U] << 16U) |
                                 ((uint32_t)buffer[offset + 2U] << 8U) |
                                 (uint32_t)buffer[offset + 3U];
        memcpy(&buffer[offset], &swapped, sizeof(swapped));
    }
}

uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform* platform, uint32_t wait_ms) {
    (void)platform;
    osDelay(wait_ms);
    return 0U;
}
