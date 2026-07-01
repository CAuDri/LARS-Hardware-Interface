/**
 * @file thread_safe_adc.c
 *
 * @brief CAuDri - Thread-Safe ADC Wrapper
 * This file provides a thread-safe wrapper for ADC operations using the STM32 HAL.
 * A timer can be configured to periodically trigger ADC conversions.
 * Measurements will be continuously stored in a DMA buffer and can be retrieved safely from multiple threads.
 */
#include "thread_safe_adc.h"

#include <cmsis_os2.h>
#include <string.h>

#include "logger.h"
#include "main.h"

#define ADC_TIMER_HANDLE &htim5                             // Default timer handle for ADC triggering
#define ADC_TIMER_CHANNEL TIM_CHANNEL_1                     // Default timer channel for ADC triggering
#define ADC_EXT_TRIGGER_SOURCE ADC_EXTERNALTRIGCONV_T5_CC1  // Corresponding external trigger source for ADC conversions

#define ADC_MAX_SEQUENCE_LENGTH 16  // Length of the DMA buffer (max 16 ADC channels)
#define ADC_MAX_CHANNELS 19         // Maximum number of supported ADC channels

#define ADC_SAMPLING_TIME ADC_SAMPLETIME_28CYCLES  // ADC sampling time for each channel
#define ADC_DEFAULT_SAMPLE_RATE_HZ 1000U           // Default sample rate


__weak ADC_HandleTypeDef hadc3;  // Since ADC 3 may currently not be used. TODO: Check if weak declaration can be done in general.

/**
 * @brief Structure to hold ADC instance data
 */
typedef struct {
    ADC_HandleTypeDef* hadc;  // Pointer to the HAL ADC handle
    bool initialized;         // Flag indicating if the ADC instance has been initialized for this implementation
    uint32_t channel_count;   // Number of registered channels
    uint16_t dma_buffer[ADC_MAX_SEQUENCE_LENGTH];    // DMA buffer to hold ADC conversion results
    uint16_t registered_channels[ADC_MAX_CHANNELS];  // Map of registered channels to DMA buffer indices (sequencer ranks)
} ThreadSafeADC_t;

static ThreadSafeADC_t adc1 = {.hadc = &hadc1, .initialized = false, .dma_buffer = {0}, .registered_channels = {0}};
static ThreadSafeADC_t adc2 = {.hadc = &hadc2, .initialized = false, .dma_buffer = {0}, .registered_channels = {0}};
static ThreadSafeADC_t adc3 = {.hadc = &hadc3, .initialized = false, .dma_buffer = {0}, .registered_channels = {0}};

static HAL_StatusTypeDef ADC_InitInstance(ADC_HandleTypeDef* hadc);
static bool ADC_IsValidChannel(uint32_t channel);

/**
 * @brief Initializes the thread-safe ADC wrapper with the specified sample rate.
 * This function configures the timer to trigger ADC conversions at the desired sample rate.
 *
 * @param sample_rate_hz The desired sample rate in Hz.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef ADC_Init() {
    LogDebug("ADC: Initializing thread-safe ADC wrapper");

    uint32_t sample_rate_hz = ADC_DEFAULT_SAMPLE_RATE_HZ;
    TIM_HandleTypeDef* htim = ADC_TIMER_HANDLE;

    if (htim == NULL) {
        LogError("ADC: Invalid timer handle for triggering ADC conversions");
        return HAL_ERROR;
    }

    if (sample_rate_hz == 0) {
        LogError("ADC: Sample rate must be greater than 0 Hz");
        return HAL_ERROR;
    }

    // Timers can be on different APB buses and thus have different clock frequencies
    uint32_t timer_clock_freq = 0;
    if (htim->Instance == TIM1 || htim->Instance == TIM8 || htim->Instance == TIM9 || htim->Instance == TIM10 ||
        htim->Instance == TIM11) {
        // TIM1, TIM8, TIM9, TIM10, TIM11 are on APB2
        // This might only be valid for STM32F4xx and needs to be adapted for other STM32 series
        timer_clock_freq = HAL_RCC_GetPCLK2Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) {
            // Timer clock is automatically doubled if APB prescaler > 1
            timer_clock_freq *= 2;
        }
    } else {
        // All other timers are on APB1
        timer_clock_freq = HAL_RCC_GetPCLK1Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
            timer_clock_freq *= 2;
        }
    }

    // Calculate the timer prescaler and period to achieve the desired sample rate
    uint32_t prescaler = (timer_clock_freq / 1000000u) - 1;  // 1 us resolution
    uint32_t period = (1000000u / sample_rate_hz) - 1;

    if (prescaler > 0xFFFF || period > 0xFFFF) {
        LogError("ADC: Calculated prescaler or period exceeds 16-bit limit (Prescaler: %lu, Period: %lu)", prescaler, period);
        return HAL_ERROR;
    }

    LogDebug("ADC: Configuring timer for ADC triggering: Timer Clock = %lu Hz, Prescaler = %lu, Period = %lu", timer_clock_freq, prescaler, period);

    // Configure the timer for periodic triggering
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim->Init.Prescaler = prescaler;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = period;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(htim) != HAL_OK) {
        LogError("ADC: Failed to initialize timer base");
        return HAL_ERROR;
    }

    if (ADC_EXT_TRIGGER_SOURCE == ADC_EXTERNALTRIGCONV_T2_TRGO || ADC_EXT_TRIGGER_SOURCE == ADC_EXTERNALTRIGCONV_T3_TRGO ||
        ADC_EXT_TRIGGER_SOURCE == ADC_EXTERNALTRIGCONV_T8_TRGO) {
        // These timers can generate a TRGO event directly
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK) {
            LogError("ADC: Failed to configure timer master synchronization");
            return HAL_ERROR;
        }
    } else {
        // For other timers, we need to use a specific channel to generate the trigger
        TIM_OC_InitTypeDef sConfigOC = {0};
        sConfigOC.OCMode = TIM_OCMODE_TOGGLE;  // Toggle output on match
        sConfigOC.Pulse = period / 2;          // CC event in the middle of the period
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

        if (HAL_TIM_OC_Init(htim) != HAL_OK) {
            LogError("ADC: Failed to initialize timer output compare");
            return HAL_ERROR;
        }
        if (HAL_TIM_OC_ConfigChannel(htim, &sConfigOC, ADC_TIMER_CHANNEL) != HAL_OK) {
            LogError("ADC: Failed to configure timer output compare channel");
            return HAL_ERROR;
        }
        if (HAL_TIM_OC_Start(htim, ADC_TIMER_CHANNEL) != HAL_OK) {
            LogError("ADC: Failed to start timer output compare channel");
            return HAL_ERROR;
        }
    }

    // Start the timer
    if (HAL_TIM_Base_Start(htim) != HAL_OK) {
        LogError("ADC: Failed to start timer");
        return HAL_ERROR;
    }
    LogDebug("ADC: Timer initialized for ADC triggering at %lu Hz", sample_rate_hz);

    // Initilialize all ADC instances
    if (ADC_InitInstance(adc1.hadc) != HAL_OK) {
        LogError("ADC: Failed to initialize ADC1 instance");
        return HAL_ERROR;
    }
    if (ADC_InitInstance(adc2.hadc) != HAL_OK) {
        LogError("ADC: Failed to initialize ADC2 instance");
        return HAL_ERROR;
    }
    // if (ADC_InitInstance(adc3.hadc) != HAL_OK) {
    //     LogError("ADC: Failed to initialize ADC3 instance");
    //     return HAL_ERROR;
    // }

    return HAL_OK;
}

HAL_StatusTypeDef ADC_RegisterChannel(ADC_HandleTypeDef* hadc, uint32_t adc_channel) {
    if (hadc == NULL) {
        LogError("ADC: Invalid ADC handle in ADC_RegisterChannel");
        return HAL_ERROR;
    }
    if (!ADC_IsValidChannel(adc_channel)) {
        LogError("ADC: Invalid ADC channel %lu in ADC_RegisterChannel", adc_channel);
        return HAL_ERROR;
    }

    ThreadSafeADC_t* adc_instance = NULL;
    if (hadc->Instance == ADC1) {
        adc_instance = &adc1;
    } else if (hadc->Instance == ADC2) {
        adc_instance = &adc2;
    } else if (hadc->Instance == ADC3) {
        adc_instance = &adc3;
    } else {
        LogError("ADC: Unsupported ADC instance in ADC_RegisterChannel");
        return HAL_ERROR;
    }

    if (!adc_instance->initialized) {
        LogError("ADC: ADC instance not initialized in ADC_RegisterChannel");
        return HAL_ERROR;
    }

    // Check if the channel is already registered
    if (adc_instance->registered_channels[adc_channel] != 0) {
        LogDebug("ADC: Channel %lu already registered for this ADC instance", adc_channel);
        return HAL_OK;
    }

    if (adc_instance->channel_count >= ADC_MAX_SEQUENCE_LENGTH) {
        LogError("ADC: Maximum number of registered channels reached for this ADC instance");
        return HAL_ERROR;
    }
    if (adc_channel < ADC_CHANNEL_0 || adc_channel > ADC_CHANNEL_18) {
        LogError("ADC: Invalid ADC channel %lu", adc_channel);
        return HAL_ERROR;
    }

    // Stop the ADC to reconfigure channels
    HAL_ADC_Stop_DMA(hadc);

    // Reconfigure the ADC channel sequence
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = adc_channel;
    sConfig.Rank = adc_instance->channel_count + 1;  // Ranks start from 1
    sConfig.SamplingTime = ADC_SAMPLING_TIME;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        LogError("ADC: Failed to configure ADC channel %lu", adc_channel);
        return HAL_ERROR;
    }

    hadc->Init.NbrOfConversion = adc_instance->channel_count + 1;
    if (HAL_ADC_Init(hadc) != HAL_OK) {
        LogError("ADC: Failed to re-initialize ADC after channel configuration");
        return HAL_ERROR;
    }

    // Restart the ADC in DMA mode with updated channel count
    // The DMA will automatically wrap around after all channels are converted
    if (HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_instance->dma_buffer, adc_instance->channel_count + 1) != HAL_OK) {
        LogError("ADC: Failed to restart ADC DMA after channel registration");
        return HAL_ERROR;
    }

    // Store the rank (dma buffer index) for the registered channel
    adc_instance->registered_channels[adc_channel] = adc_instance->channel_count + 1;  // 1-indexed
    adc_instance->channel_count++;

    LogDebug("ADC: Registered channel %lu for ADC instance %s",
             adc_channel,
             (hadc->Instance == ADC1)   ? "ADC1"
             : (hadc->Instance == ADC2) ? "ADC2"
                                        : "ADC3");
    return HAL_OK;
}

/**
 * @brief Retrieves the latest ADC value for the specified channel.
 * The values are conitinuously updated in the DMA buffer and simply read from there.
 * This function is thread-safe due to atomic 16-bit reads on Cortex-M4.
 */
HAL_StatusTypeDef ADC_GetValue(ADC_HandleTypeDef* hadc, uint32_t adc_channel, uint16_t* value) {
    if (hadc == NULL || value == NULL) {
        LogError("ADC: Invalid parameters in ADC_GetValue");
        return HAL_ERROR;
    }
    if (!ADC_IsValidChannel(adc_channel)) {
        LogError("ADC: Invalid ADC channel %lu in ADC_GetValue", adc_channel);
        return HAL_ERROR;
    }

    ThreadSafeADC_t* adc_instance = NULL;
    if (hadc->Instance == ADC1) {
        adc_instance = &adc1;
    } else if (hadc->Instance == ADC2) {
        adc_instance = &adc2;
    } else if (hadc->Instance == ADC3) {
        adc_instance = &adc3;
    } else {
        LogError("ADC: Unsupported ADC instance in ADC_GetValue");
        return HAL_ERROR;
    }

    if (!adc_instance->initialized) {
        LogError("ADC: ADC instance not initialized in ADC_GetValue");
        return HAL_ERROR;
    }

    uint32_t rank = adc_instance->registered_channels[adc_channel];
    if (rank == 0) {
        LogError("ADC: Channel %lu not registered for this ADC instance", adc_channel);
        return HAL_ERROR;
    }

    // Atomic read of the ADC value from the DMA buffer
    // Convert from 1-indexed rank to 0-indexed buffer
    *value = adc_instance->dma_buffer[rank - 1];
    return HAL_OK;
}


/**
 * @brief Initializes the specified ADC instance for thread-safe operation.
 * This function configures the ADC for DMA mode and sets up the necessary parameters.
 *
 * @param hadc Pointer to the ADC handle.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef ADC_InitInstance(ADC_HandleTypeDef* hadc) {
    if (hadc == NULL) {
        LogError("ADC: Invalid ADC handle in ADC_InitInstance");
        return HAL_ERROR;
    }

    // Check if the ADC instance is already initialized
    if ((hadc->Instance == ADC1 && adc1.initialized) || (hadc->Instance == ADC2 && adc2.initialized) ||
        (hadc->Instance == ADC3 && adc3.initialized)) {
        LogDebug("ADC: ADC instance already initialized");
        return HAL_OK;
    }

    // Check if a DMA handle is associated with the ADC
    // This needs to be done beforehand in CubeMX or manually
    if (hadc->DMA_Handle == NULL) {
        LogError("ADC: ADC instance has no associated DMA handle");
        LogError("ADC: Configure DMA for the ADC instance in CubeMX before using the thread-safe ADC wrapper");
        return HAL_ERROR;
    }

    // Re-init the ADC with the required settings
    hadc->Init.ScanConvMode = ENABLE;
    hadc->Init.ContinuousConvMode = DISABLE;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConv = ADC_EXT_TRIGGER_SOURCE;
    hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc->Init.NbrOfConversion = 0;  // Will be updated when channels are registered
    hadc->Init.DMAContinuousRequests = ENABLE;
    if (HAL_ADC_Init(hadc) != HAL_OK) {
        LogError("ADC: Failed to initialize ADC instance");
        return HAL_ERROR;
    }

    // Re-init the DMA in circular mode
    hadc->DMA_Handle->Init.Mode = DMA_CIRCULAR;
    if (HAL_DMA_Init(hadc->DMA_Handle) != HAL_OK) {
        LogError("ADC: Failed to initialize DMA for ADC instance");
        return HAL_ERROR;
    }

    // Start the ADC in DMA mode
    uint16_t* dma_buffer = NULL;
    if (hadc->Instance == ADC1) {
        dma_buffer = adc1.dma_buffer;
    } else if (hadc->Instance == ADC2) {
        dma_buffer = adc2.dma_buffer;
    } else if (hadc->Instance == ADC3) {
        dma_buffer = adc3.dma_buffer;
    } else {
        LogError("ADC: Unsupported ADC instance");
        return HAL_ERROR;
    }

    if (HAL_ADC_Start_DMA(hadc, (uint32_t*)dma_buffer, ADC_MAX_SEQUENCE_LENGTH) != HAL_OK) {
        LogError("ADC: Failed to start ADC in DMA mode");
        return HAL_ERROR;
    }
    LogDebug("ADC: ADC instance initialized for thread-safe operation");

    // Disable DMA callbacks to decrease CPU load (except for error handling)
    __HAL_DMA_DISABLE_IT(hadc->DMA_Handle, DMA_IT_TC | DMA_IT_HT);

    // Mark the ADC instance as initialized
    if (hadc->Instance == ADC1) {
        adc1.initialized = true;
        adc1.hadc = hadc;
    } else if (hadc->Instance == ADC2) {
        adc2.initialized = true;
        adc2.hadc = hadc;
    } else if (hadc->Instance == ADC3) {
        adc3.initialized = true;
        adc3.hadc = hadc;
    }

    return HAL_OK;
}

/**
 * @brief Checks if the given ADC channel is valid.
 *
 * @param channel The ADC channel to check.
 * @return true if the channel is valid, false otherwise.
 */
static bool ADC_IsValidChannel(uint32_t channel) {
    return (channel == ADC_CHANNEL_0 || channel == ADC_CHANNEL_1 || channel == ADC_CHANNEL_2 || channel == ADC_CHANNEL_3 ||
            channel == ADC_CHANNEL_4 || channel == ADC_CHANNEL_5 || channel == ADC_CHANNEL_6 || channel == ADC_CHANNEL_7 ||
            channel == ADC_CHANNEL_8 || channel == ADC_CHANNEL_9 || channel == ADC_CHANNEL_10 || channel == ADC_CHANNEL_11 ||
            channel == ADC_CHANNEL_12 || channel == ADC_CHANNEL_13 || channel == ADC_CHANNEL_14 || channel == ADC_CHANNEL_15 ||
            channel == ADC_CHANNEL_16 || channel == ADC_CHANNEL_17 || channel == ADC_CHANNEL_18);
}

/**
 * @brief Error callback for ADC conversion errors.
 * This function is called by the HAL in case of an ADC error.
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc) {
    LogError("ADC: ADC conversion error occurred on instance %s",
             (hadc->Instance == ADC1)   ? "ADC1"
             : (hadc->Instance == ADC2) ? "ADC2"
                                        : "ADC3");
}