/**
 * @file thread_safe_adc.h
 *
 * @brief CAuDri - Thread-Safe ADC Wrapper
 *
 */
#pragma once

#include <cmsis_os2.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ADC_ChannelCallback_t)(uint32_t channel, uint32_t value);

HAL_StatusTypeDef ADC_Init();

HAL_StatusTypeDef ADC_RegisterChannel(ADC_HandleTypeDef* hadc, uint32_t channel);
HAL_StatusTypeDef ADC_GetValue(ADC_HandleTypeDef* hadc, uint32_t channel, uint16_t* value);

#ifdef __cplusplus
}
#endif