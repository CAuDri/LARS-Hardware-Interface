/**
 * @file thread_safe_can.h
 *
 * @brief CAuDri - Thread-safe CAN wrapper for multi-threaded applications
 *
 * This file provides a thread-safe wrapper for CAN communication using the STM32 HAL.
 * A mutex is used to ensure that CAN operations are not interrupted by other threads.
 *
 * CAN_AddTxMessage, CAN_GetRxMessage and CAN_ConfigFilter simply wrap the corresponding HAL functions.
 * Additionally, CAN_ConfigAndAllocateFilter is provided to automatically allocate an available filter bank.
 *
 * A background "CAN Dispatcher" thread will be started to handle incoming CAN messages and dispatch them to user-defined callbacks based on the configured filter banks.
 * CAN_RegisterRxCallback can be used to register a callback for a specific filter bank. The callback will be called from the dispatcher thread context with the full CAN frame data.
 */
#pragma once

#include <cmsis_os2.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

// The CAN dispatcher thread should run at a high priority to ensure no messages are lost
#define CAN_DISPATCHER_THREAD_PRIORITY (osPriorityHigh)

// Type definition for RX callback functions
typedef void (*CAN_RxCallback_t)(CAN_RxHeaderTypeDef pHeader, uint8_t aData[]);

bool CAN_Init();

HAL_StatusTypeDef CAN_ConfigFilter(CAN_HandleTypeDef* hcan, const CAN_FilterTypeDef* sFilterConfig);
HAL_StatusTypeDef CAN_ConfigAndAllocateFilter(CAN_HandleTypeDef* hcan, const CAN_FilterTypeDef* sFilterConfig);

HAL_StatusTypeDef CAN_RegisterRxCallback(CAN_HandleTypeDef* hcan, uint32_t filter_bank, CAN_RxCallback_t callback);

HAL_StatusTypeDef CAN_AddTxMessage(CAN_HandleTypeDef* hcan,
                                   const CAN_TxHeaderTypeDef* pHeader,
                                   const uint8_t aData[],
                                   uint32_t* pTxMailbox);
HAL_StatusTypeDef CAN_GetRxMessage(CAN_HandleTypeDef* hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef* pHeader, uint8_t aData[]);

#ifdef __cplusplus
}
#endif