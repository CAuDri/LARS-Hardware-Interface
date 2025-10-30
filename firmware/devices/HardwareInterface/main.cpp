/**
 * @file main.cpp
 *
 * @brief CAuDri - Main entry point for the application code
 */

#include "main.h"

#include "config/config.h"
#include "logger.h"

#include <algorithm>

// CAuDri - This is necessary for OpenOCD to show the correct FreeRTOS task list
// TODO: Find a better place for this (must be linked into the final binary)
const volatile UBaseType_t uxTopUsedPriority = configMAX_PRIORITIES - 1;

extern "C" void mainTask();

RCReceiver rc_receiver;
VESC motor("VESC Driver");
Servo servo("Servo Front");

static bool rc_control_enabled = false;

// Callback for channel 5
void remoteSwitchCallback(uint16_t value) {
    if (value > 1500) {
        HAL_GPIO_WritePin(DEBUG_LED_RED_GPIO_Port, DEBUG_LED_RED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DEBUG_LED_GREEN_GPIO_Port, DEBUG_LED_GREEN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DEBUG_LED_BLUE_GPIO_Port, DEBUG_LED_BLUE_Pin, GPIO_PIN_SET);
        rc_control_enabled = false;
    } else if (value > 900) {
        HAL_GPIO_WritePin(DEBUG_LED_RED_GPIO_Port, DEBUG_LED_RED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DEBUG_LED_GREEN_GPIO_Port, DEBUG_LED_GREEN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DEBUG_LED_BLUE_GPIO_Port, DEBUG_LED_BLUE_Pin, GPIO_PIN_RESET);
        rc_control_enabled = true;
    } else {
        HAL_GPIO_WritePin(DEBUG_LED_RED_GPIO_Port, DEBUG_LED_RED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DEBUG_LED_GREEN_GPIO_Port, DEBUG_LED_GREEN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DEBUG_LED_BLUE_GPIO_Port, DEBUG_LED_BLUE_Pin, GPIO_PIN_RESET);
        motor.setRPM(0);
        rc_control_enabled = false;
    }
}

void motorChannelCallback(uint16_t value) {
    if (!rc_control_enabled) {
        // motor.setRPM(0);
        return;
    }

    LogInfo("Motor Channel Value: %d", value);

    if (value < 1050 && value > 950) {
        // Deadband around center
        motor.setRPM(0);
        return;
    }

    // Map RC channel value (200-1800) to RPM ~(-3000 to 3000)
    int32_t rpm = (static_cast<int32_t>(value) - 1000) * 3;

    // Clamp RPM to [-3000, 3000]
    if (rpm > 3000) {
        rpm = 3000;
    } else if (rpm < -3000) {
        rpm = -3000;
    }

    LogInfo("Motor RPM: %d, value: %d", rpm, value);
    motor.setRPM(rpm);
}

void steeringChannelCallback(uint16_t value) {
    if (!rc_control_enabled) {
        return;
    }

    // Map RC channel value (100-1900) to pulse width (1200-1800)
    uint32_t pulse_width = (static_cast<uint32_t>(value) - 100) * 600 / 1800 + 1200;
    LogInfo("Steering Pulse Width Command: %u", pulse_width);
    servo.setPulseWidth(pulse_width);
}


/**
 * @brief Main entry point called from the RTOS task in main.c
 */
void mainTask() {
    printf("Main Task: Starting up...\n");

    // Enable power for external components
    osDelay(100);
    HAL_GPIO_WritePin(PWR_EXT_ENABLE_GPIO_Port, PWR_EXT_ENABLE_Pin, GPIO_PIN_SET);
    osDelay(100);

    rc_receiver.init(rc_config);

    motor.init(vesc_config);
    motor.start();

    servo.init(servo_config, servo_calibration);

    rc_receiver.registerChannelCallback(6, RCReceiver::ChannelCallback::from<&remoteSwitchCallback>(), true);
    rc_receiver.registerChannelCallback(1, RCReceiver::ChannelCallback::from<&motorChannelCallback>(), true);
    rc_receiver.registerChannelCallback(3, RCReceiver::ChannelCallback::from<&steeringChannelCallback>(), true);
    rc_receiver.start();

    for (;;) {
        osDelay(2000);
        // static uint16_t angle = 0;
        // angle += 10;
        // if (angle > 180) {
        //     angle = 0;
        // }
        // servo.setAngle(static_cast<float>(angle));

        // CAN_TxHeaderTypeDef tx_header{};
        // uint8_t tx_data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
        // uint32_t tx_mailbox;
        // tx_header.StdId = 0x123;
        // tx_header.ExtId = 0x123456;
        // tx_header.IDE = CAN_ID_EXT;
        // tx_header.RTR = CAN_RTR_DATA;
        // tx_header.DLC = 8;
        // HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);

        // HAL_GPIO_TogglePin(DEBUG_LED_BLUE_GPIO_Port, DEBUG_LED_BLUE_Pin);

        // auto can_state = HAL_CAN_GetState(&hcan1);
        // switch (can_state) {
        //     case HAL_CAN_STATE_RESET:
        //         LogInfo("CAN1 State: RESET");
        //         break;
        //     case HAL_CAN_STATE_READY:
        //         LogInfo("CAN1 State: READY");
        //         break;
        //     case HAL_CAN_STATE_LISTENING:
        //         LogInfo("CAN1 State: LISTENING");
        //         break;
        //     case HAL_CAN_STATE_SLEEP_PENDING:
        //         LogInfo("CAN1 State: SLEEP_PENDING");
        //         break;
        //     case HAL_CAN_STATE_SLEEP_ACTIVE:
        //         LogInfo("CAN1 State: SLEEP_ACTIVE");
        //         break;
        //     case HAL_CAN_STATE_ERROR:
        //         LogInfo("CAN1 State: ERROR");
        //         break;
        //     default:
        //         LogInfo("CAN1 State: UNKNOWN, state value: %d", static_cast<int>(can_state));
        //         break;
        // }

        // auto can_error = HAL_CAN_GetError(&hcan1);
        // LogInfo("CAN1 Error Code: 0x%08lX", can_error);

        // CAN_RxHeaderTypeDef rx_header;
        // uint8_t rx_data[8];
        // if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
        //     auto rx_message = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
        //     if (rx_message == HAL_OK) {
        //         LogInfo("CAN1 RX Message Pending: ID=0x%08lX, DLC=%u", rx_header.ExtId, rx_header.DLC);
        //     } else {
        //         LogInfo("CAN1 RX Message Pending: None");
        //     }
        // }

        // bool tx0_pending = HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX0 );
        // bool tx1_pending = HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX1 );
        // bool tx2_pending = HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX2 );
        // LogInfo("CAN1 TX Message Pending on Mailboxes: 0=%s, 1=%s, 2=%s",
        //         tx0_pending ? "YES" : "NO",
        //         tx1_pending ? "YES" : "NO",
        //         tx2_pending ? "YES" : "NO");

        // motor.setRPM(1000);

        // crsf::ChannelData channels;
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