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

/** Register a HAL channel selector using the default 28-cycle sampling time. */
HAL_StatusTypeDef ADC_RegisterChannel(ADC_HandleTypeDef* hadc, uint32_t channel);

/**
 * Register a HAL channel selector using an explicit ADC_SAMPLETIME_* value.
 * Call during initialization, before concurrent access. The first registration
 * defines the sampling time; registering the same physical channel again is a
 * no-op. The caller selects a channel appropriate for the ADC instance.
 */
HAL_StatusTypeDef ADC_RegisterChannelWithSamplingTime(ADC_HandleTypeDef* hadc,
                                                      uint32_t channel,
                                                      uint32_t sampling_time);
HAL_StatusTypeDef ADC_GetValue(ADC_HandleTypeDef* hadc, uint32_t channel, uint16_t* value);

#ifdef __cplusplus
}
#endif
