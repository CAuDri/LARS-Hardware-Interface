/**
 * @file main.cpp
 *
 * @brief CAuDri - Main entry point for the application code
 */

#include "main.h"

#include "config.h"
#include "logger.h"

// CAuDri - This is necessary for OpenOCD to show the correct FreeRTOS task list
// TODO: Find a better place for this (must be linked into the final binary)
const volatile UBaseType_t uxTopUsedPriority = configMAX_PRIORITIES - 1;

extern "C" void mainTask();

RCReceiver rc_receiver;

// Callback for channel 5
void remoteSwitchCallback(uint16_t value) { 
    if (value > 1500) {
        HAL_GPIO_WritePin(DEBUG_LED_RED_GPIO_Port, DEBUG_LED_RED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DEBUG_LED_GREEN_GPIO_Port, DEBUG_LED_GREEN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DEBUG_LED_BLUE_GPIO_Port, DEBUG_LED_BLUE_Pin, GPIO_PIN_RESET);
    } else if (value > 900) {
        HAL_GPIO_WritePin(DEBUG_LED_RED_GPIO_Port, DEBUG_LED_RED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DEBUG_LED_GREEN_GPIO_Port, DEBUG_LED_GREEN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DEBUG_LED_BLUE_GPIO_Port, DEBUG_LED_BLUE_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(DEBUG_LED_RED_GPIO_Port, DEBUG_LED_RED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DEBUG_LED_GREEN_GPIO_Port, DEBUG_LED_GREEN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DEBUG_LED_BLUE_GPIO_Port, DEBUG_LED_BLUE_Pin, GPIO_PIN_SET);
    }
}

/**
 * @brief Main entry point called from the RTOS task in main.c
 */
void mainTask() {
    osDelay(100);
    LogClear();
    LogInfo("Main Task: Starting up...");
    osDelay(100);

    rc_receiver.init(rc_config);
    rc_receiver.registerChannelCallback(6, RCReceiver::ChannelCallback::from<&remoteSwitchCallback>(), true);
    rc_receiver.start();

    // Enable power to external components
    HAL_GPIO_WritePin(PWR_EXT_ENABLE_GPIO_Port, PWR_EXT_ENABLE_Pin, GPIO_PIN_SET);
    osDelay(500);

    for (;;) {
        osDelay(2000);
        // HAL_GPIO_TogglePin(DEBUG_LED_BLUE_GPIO_Port, DEBUG_LED_BLUE_Pin);
        crsf::ChannelData channels;
        // if (rc_receiver.getChannelData(channels)) {
        //     LogInfo("%u %u %u %u", channels[0], channels[1], channels[2], channels[3]);
        //     LogInfo("%u %u %u %u", channels[4], channels[5], channels[6], channels[7]);
        //     LogInfo("%u %u %u %u", channels[8], channels[9], channels[10], channels[11]);
        //     LogInfo("%u %u %u %u", channels[12], channels[13], channels[14], channels[15]);

        // } else {
        //     LogWarning("Failed to get channel data");
        // }
        // LogInfo("Main task running...");
    }
}