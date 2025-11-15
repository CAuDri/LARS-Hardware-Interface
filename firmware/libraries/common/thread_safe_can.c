/**
 * @file thread_safe_can.c
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
#include "thread_safe_can.h"

#include <cmsis_os2.h>
#include <logger.h>

// This implementation relies on the HAL_CAN_RegisterCallback function to register the necessary callbacks
#if (USE_HAL_CAN_REGISTER_CALLBACKS != 1)
    #error "thread_safe_can.c requires USE_HAL_CAN_REGISTER_CALLBACKS to be enabled in stm32f4xx_hal_conf.h"
#endif

// Define "LOG_VERBOSE" to enable verbose logging of the CAN interface
#ifdef LOG_VERBOSE
    #define LogVerbose(...) LogDebug(__VA_ARGS__)
#else
    #define LogVerbose(...)
#endif

#define CAN_MUTEX_TIMEOUT_MS 100
#define CAN_MESSAGE_QUEUE_SIZE 16

// TODO: Don't be as dumb as before and do this properly
// __weak CAN_HandleTypeDef hcan1 = {0};
__weak CAN_HandleTypeDef hcan2 = {0};

/**
 * @brief Structure for CAN message queue entries
 *
 * Used to pass CAN messages from the ISR context to the dispatcher thread.
 */
typedef struct {
    CAN_HandleTypeDef* hcan;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CAN_MessageQueueItem_t;

// Static allocation for the CAN dispatcher thread
static osThreadId_t can_dispatcher_thread_id;
static StaticTask_t can_dispatcher_thread_cb;
static uint32_t can_dispatcher_thread_stack[256];
static osThreadAttr_t can_dispatcher_thread_attr = {
    .name = "CAN Dispatcher",
    .cb_mem = &can_dispatcher_thread_cb,
    .cb_size = sizeof(can_dispatcher_thread_cb),
    .stack_mem = can_dispatcher_thread_stack,
    .stack_size = sizeof(can_dispatcher_thread_stack),
    .priority = CAN_DISPATCHER_THREAD_PRIORITY,
};

// Message queue to store incoming CAN messages for the dispatcher thread
static osMessageQueueId_t can_message_queue;
static uint8_t can_message_queue_buffer[CAN_MESSAGE_QUEUE_SIZE * sizeof(CAN_MessageQueueItem_t)];
static StaticQueue_t can_message_queue_cb;
static osMessageQueueAttr_t can_message_queue_attr = {
    .name = "CAN Message Queue",
    .cb_mem = &can_message_queue_cb,
    .cb_size = sizeof(can_message_queue_cb),
    .mq_mem = &can_message_queue_buffer,
    .mq_size = sizeof(can_message_queue_buffer),
};

// Mutexes for CAN peripherals, they will be locked during transmit and config operations
static osMutexId_t can1_mutex;
static StaticQueue_t can1_mutex_cb;
static osMutexAttr_t can1_mutex_attr = {  //
    .name = "CAN1 Mutex",
    .attr_bits = osMutexRecursive,
    .cb_mem = &can1_mutex_cb,
    .cb_size = sizeof(can1_mutex_cb)};

static osMutexId_t can2_mutex;
static StaticQueue_t can2_mutex_cb;
static osMutexAttr_t can2_mutex_attr = {  //
    .name = "CAN2 Mutex",
    .attr_bits = osMutexRecursive,
    .cb_mem = &can2_mutex_cb,
    .cb_size = sizeof(can2_mutex_cb)};

// For each filter bank a callback can be registered
static CAN_RxCallback_t can_rx_callbacks[28] = {0};

static bool can_initialized = false;

void CAN_DispatcherThread(void* argument);
void CAN_Fifo0CallbackHandler(CAN_HandleTypeDef* hcan);
void CAN_Fifo1CallbackHandler(CAN_HandleTypeDef* hcan);
void CAN_ErrorCallbackHandler(CAN_HandleTypeDef* hcan);

/**
 * @brief Initialize the thread-safe CAN wrapper
 *
 * This function must be called before any other CAN functions.
 * It creates the necessary mutexes for thread safety.
 */
bool CAN_Init() {
    // Check if any of the CAN peripherals are actually used
    // We have weak definitions of hcan1 and hcan2 that will be overridden by the CubeMX-generated code
    if (hcan1.Instance == NULL && hcan2.Instance == NULL) {
        LogWarning("CAN: CAN_Init called but no CAN peripherals are configured");
        return false;
    }

    can1_mutex = osMutexNew(&can1_mutex_attr);
    if (can1_mutex == NULL) {
        LogError("CAN: Failed to create CAN1 mutex");
        return false;
    }

    can2_mutex = osMutexNew(&can2_mutex_attr);
    if (can2_mutex == NULL) {
        LogError("CAN: Failed to create CAN2 mutex");
        osMutexDelete(can1_mutex);
        return false;
    }

    can_message_queue =
        osMessageQueueNew(CAN_MESSAGE_QUEUE_SIZE, sizeof(CAN_MessageQueueItem_t), &can_message_queue_attr);

    // Start the CAN dispatcher thread for handling RX callbacks
    can_dispatcher_thread_id = osThreadNew(CAN_DispatcherThread, NULL, &can_dispatcher_thread_attr);
    if (can_dispatcher_thread_id == NULL) {
        LogError("CAN: Failed to create CAN dispatcher thread");
        return false;
    }

    // Start the CAN peripherals and enable interrupts
    uint32_t notify_flags = CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY |
                            CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF;
    if (hcan1.Instance != NULL && hcan1.State != HAL_CAN_STATE_LISTENING) {
        if (HAL_CAN_ActivateNotification(&hcan1, notify_flags) != HAL_OK) {
            LogError("CAN: Failed to activate CAN1 interrupt notifications");
            return false;
        }
        if (HAL_CAN_Start(&hcan1) != HAL_OK) {
            LogError("CAN: Failed to start CAN1 peripheral");
            return false;
        }
        LogVerbose("CAN: CAN1 peripheral started");
    }
    if (hcan2.Instance != NULL && hcan2.State != HAL_CAN_STATE_LISTENING) {
        if (HAL_CAN_Start(&hcan2) != HAL_OK) {
            LogError("CAN: Failed to start CAN2 peripheral");
            return false;
        }
        if (HAL_CAN_ActivateNotification(&hcan2, notify_flags) != HAL_OK) {
            LogError("CAN: Failed to activate CAN2 interrupt notifications");
            return false;
        }
        LogVerbose("CAN: CAN2 peripheral started");
    }

    LogDebug("CAN: Thread-safe CAN wrapper initialized");

    can_initialized = true;
    return true;
}

/**
 * @brief Thread-safe CAN filter configuration
 *
 * Same usage as HAL_CAN_ConfigFilter, but thread-safe.
 * Can not be called from ISR context.
 *
 * @param hcan Pointer to a configured CAN handle
 * @param sFilterConfig Pointer to a CAN_FilterTypeDef structure that contains the filter configuration
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef CAN_ConfigFilter(CAN_HandleTypeDef* hcan, const CAN_FilterTypeDef* sFilterConfig) {
    if (!can_initialized) {
        LogError("CAN: CAN_ConfigFilter called before CAN_Init");
        return HAL_ERROR;
    }

    osMutexId_t mutex = (hcan->Instance == CAN1) ? can1_mutex : can2_mutex;
    if (osMutexAcquire(mutex, CAN_MUTEX_TIMEOUT_MS) != osOK) {
        LogDebug("CAN: Mutex acquire failed on filter config in thread '%s'",
                 osThreadGetName(osThreadGetId()));
        return HAL_TIMEOUT;
    }

    // We can only configure filters when the CAN peripheral is stopped
    if (hcan->State != HAL_CAN_STATE_READY) {
        if (hcan->State == HAL_CAN_STATE_LISTENING) {
            HAL_StatusTypeDef status = HAL_CAN_Stop(hcan);
            if (status != HAL_OK) {
                LogError("CAN: Failed to stop CAN peripheral for filter config on filter bank %lu",
                         sFilterConfig->FilterBank);
                osMutexRelease(mutex);
                return status;
            }
        } else {
            LogError("CAN: Failed to configure filter bank %lu, CAN in unexpected state %d",
                     sFilterConfig->FilterBank,
                     hcan->State);
            osMutexRelease(mutex);
            return HAL_ERROR;
        }
    }

    HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(hcan, sFilterConfig);
    if (status != HAL_OK) {
        LogError("CAN: Failed to configure filter bank %lu", sFilterConfig->FilterBank);
        if (HAL_CAN_Start(hcan) != HAL_OK) {  // Try to restart CAN even on failure
            LogError("CAN: Failed to (re)start CAN peripheral after filter config failure");
        }
        osMutexRelease(mutex);
        return status;
    }

    // Restart CAN after the filter has been configured
    status = HAL_CAN_Start(hcan);
    if (status != HAL_OK) {
        LogError("CAN: Failed to (re)start CAN peripheral after filter config on filter bank %lu",
                 sFilterConfig->FilterBank);
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK ||
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
        LogError("CAN: Failed to reactivate notifications after filter config");
        status = HAL_ERROR;
    }

    LogVerbose("CAN: Configured filter bank %lu", sFilterConfig->FilterBank);

    osMutexRelease(mutex);
    return status;
}

/**
 * @brief Thread-safe CAN filter configuration with filter bank allocation
 *
 * This is a helper function that configures a CAN filter and automatically
 * allocates the lowest available filter bank for the given CAN peripheral.
 * Can not be called from ISR context.
 *
 * The filter bank set in the sFilterConfig will be overwritten by
 * the allocated filter bank.
 *
 * @brief CAN_ConfigAndAllocateFilter
 * @param hcan Pointer to a configured CAN handle
 * @param sFilterConfig Pointer to a CAN_FilterTypeDef structure that contains the filter configuration
 * @return HAL_StatusTypeDef HAL_OK if successful
 */
HAL_StatusTypeDef CAN_ConfigAndAllocateFilter(CAN_HandleTypeDef* hcan, const CAN_FilterTypeDef* sFilterConfig) {
    if (!can_initialized) {
        LogError("CAN: CAN_ConfigAndAllocateFilter called before CAN_Init");
        return HAL_ERROR;
    }

    CAN_FilterTypeDef filter_config = *sFilterConfig;

    osMutexId_t mutex = (hcan->Instance == CAN1) ? can1_mutex : can2_mutex;
    if (osMutexAcquire(mutex, CAN_MUTEX_TIMEOUT_MS) != osOK) {
        LogDebug("CAN: Mutex acquire failed on filter allocation in thread '%s'",
                 osThreadGetName(osThreadGetId()));
        return HAL_TIMEOUT;
    }

    // The bxCAN peripheral has 28 filter banks in total, split between CAN1 and CAN2.
    // The split point can be configured and is stored in the CAN1 FMR register.
    // uint8_t filter_bank_split = (CAN1->FMR >> 8) & 0x3F;  // bits 8-13
    uint8_t filter_bank_split = 14;  // Default
    uint8_t start_bank = (hcan->Instance == CAN1) ? 0 : filter_bank_split;
    uint8_t end_bank = (hcan->Instance == CAN1) ? filter_bank_split : 28;

    // All active filter banks are stored in the FA1R register as a bitmask
    uint32_t active_banks = CAN1->FA1R;

    // Shift active banks to find the lowest available filter bank
    bool found = false;
    for (uint32_t i = start_bank; i < end_bank; i++) {
        if ((active_banks & (1 << i)) == 0) {
            filter_config.FilterBank = i + start_bank;
            found = true;
            break;
        }
    }
    if (!found) {
        LogError("CAN: No available filter banks for CAN%d", (hcan->Instance == CAN1) ? 1 : 2);
        LogDebug("CAN: Active filter banks bitmask: 0x%08lX", active_banks);
        osMutexRelease(mutex);
        return HAL_ERROR;
    }

    LogVerbose("CAN: Allocated filter bank %lu for CAN%d",
               filter_config.FilterBank,
               (hcan->Instance == CAN1) ? 1 : 2);

    // Just hope that the mutex will instantly be reacquired in CAN_ConfigFilter
    // If you ever suspect a race condition in this implementation, start here and protect the whole function call
    osMutexRelease(mutex);
    return CAN_ConfigFilter(hcan, &filter_config);
}

/**
 * @brief Set a callback for CAN RX interrupt for a specific filter bank
 *
 * This function sets a user-defined callback to be called when a message
 * is received that matches the specified filter bank.
 *
 * @param hcan Pointer to a configured CAN handle
 * @param filter_bank The filter bank number (0-27) to register the callback for
 * @param callback The callback function to be called on message reception
 */
HAL_StatusTypeDef CAN_RegisterRxCallback(CAN_HandleTypeDef* hcan, uint32_t filter_bank, CAN_RxCallback_t callback) {
    if (filter_bank >= 28) {
        LogError("CAN: Invalid filter bank %lu for RX callback registration", filter_bank);
        return HAL_ERROR;
    }

    // Register the global callback handler if not already done
    if (hcan->RxFifo0MsgPendingCallback != CAN_Fifo0CallbackHandler ||
        hcan->RxFifo1MsgPendingCallback != CAN_Fifo1CallbackHandler) {
        LogVerbose("CAN: Registering global RX FIFO callbacks for CAN%d", (hcan->Instance == CAN1) ? 1 : 2);

        // The CAN peripheral must be stopped to register callbacks
        HAL_CAN_Stop(hcan);

        if (HAL_CAN_RegisterCallback(hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0CallbackHandler) != HAL_OK ||
            HAL_CAN_RegisterCallback(hcan, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID, CAN_Fifo1CallbackHandler) != HAL_OK) {
            LogError("CAN: Failed to register global RX FIFO callbacks for CAN%d",
                     (hcan->Instance == CAN1) ? 1 : 2);
            return HAL_ERROR;
        }

        if (HAL_CAN_RegisterCallback(hcan, HAL_CAN_ERROR_CB_ID, CAN_ErrorCallbackHandler) != HAL_OK) {
            LogError("CAN: Failed to register global error callback for CAN%d",
                     (hcan->Instance == CAN1) ? 1 : 2);
            return HAL_ERROR;
        }

        LogDebug("CAN: Registered global callbacks for CAN%d", (hcan->Instance == CAN1) ? 1 : 2);

        if (HAL_CAN_Start(hcan) != HAL_OK) {
            LogError(
                "CAN: Failed to restart CAN peripheral after registering RX callbacks for CAN%d",
                (hcan->Instance == CAN1) ? 1 : 2);
            return HAL_ERROR;
        }
    }

    if (can_rx_callbacks[filter_bank] != NULL) {
        LogWarning("CAN: Overwriting existing RX callback for filter bank %lu", filter_bank);
    }

    can_rx_callbacks[filter_bank] = callback;
    LogDebug("CAN: Registered RX callback for filter bank %lu on CAN%d",
             filter_bank,
             (hcan->Instance == CAN1) ? 1 : 2);
    return HAL_OK;
}

/**
 * @brief Thread-safe CAN transmit
 *
 * Same usage as HAL_CAN_AddTxMessage, but thread-safe.
 * Can not be called from ISR context.
 */
HAL_StatusTypeDef CAN_AddTxMessage(CAN_HandleTypeDef* hcan,
                                   const CAN_TxHeaderTypeDef* pHeader,
                                   const uint8_t aData[],
                                   uint32_t* pTxMailbox) {
    if (!can_initialized) {
        LogWarning("CAN: CAN_AddTxMessage called before CAN_Init");
        return HAL_ERROR;
    }

    osMutexId_t mutex = (hcan->Instance == CAN1) ? can1_mutex : can2_mutex;
    if (osMutexAcquire(mutex, CAN_MUTEX_TIMEOUT_MS) != osOK) {
        LogDebug("CAN: Mutex acquire failed on transmit in thread '%s'", osThreadGetName(osThreadGetId()));
        return HAL_TIMEOUT;
    }

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, pHeader, aData, pTxMailbox);
    if (status != HAL_OK) {
        LogVerbose("CAN: Failed to add TX message to mailbox");
        osMutexRelease(mutex);
        return status;
    }

    osMutexRelease(mutex);
    return HAL_OK;
}

/**
 * @brief Thread-safe CAN receive
 *
 * Same usage as HAL_CAN_GetRxMessage, but thread-safe.
 * Can not be called from ISR context.
 */
HAL_StatusTypeDef CAN_GetRxMessage(CAN_HandleTypeDef* hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef* pHeader, uint8_t aData[]) {
    if (!can_initialized) {
        LogError("CAN: CAN_GetRxMessage called before CAN_Init");
        return HAL_ERROR;
    }

    osMutexId_t mutex = (hcan->Instance == CAN1) ? can1_mutex : can2_mutex;
    if (osMutexAcquire(mutex, CAN_MUTEX_TIMEOUT_MS) != osOK) {
        LogDebug("CAN: Mutex acquire failed on receive in thread '%s'", osThreadGetName(osThreadGetId()));
        return HAL_TIMEOUT;
    }

    HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(hcan, RxFifo, pHeader, aData);
    if (status != HAL_OK) {
        LogError("CAN: Failed to get RX message from FIFO %lu", RxFifo);
        osMutexRelease(mutex);
        return status;
    }

    osMutexRelease(mutex);
    return HAL_OK;
}

/**
 * @brief CAN Dispatcher Thread
 *
 * This thread waits for incoming CAN messages and dispatches them to the
 * registered callbacks.
 */
void CAN_DispatcherThread(void* arg) {
    (void)arg;
    LogDebug("CAN: CAN dispatcher thread started");

    while (1) {
        // Wait for a message to arrive in the queue
        CAN_MessageQueueItem_t message;
        osStatus_t status = osMessageQueueGet(can_message_queue, &message, NULL, osWaitForever);
        if (status != osOK) {
            LogWarning("CAN: Failed to get message from dispatcher queue, status: %d", status);
            continue;
        }

        // Call the registered callback for the matching filter bank
        if (message.header.FilterMatchIndex < 28) {
            if (can_rx_callbacks[message.header.FilterMatchIndex] != NULL) {
                can_rx_callbacks[message.header.FilterMatchIndex](message.header, message.data);
            } else {
                LogVerbose("CAN: Filter bank %lu received message but no callback registered",
                         message.header.FilterMatchIndex);
            }
        }
    }
}

void CAN_Fifo0CallbackHandler(CAN_HandleTypeDef* hcan) {
    CAN_MessageQueueItem_t msg;
    msg.hcan = hcan;

    LogVerbose("CAN: FIFO 0 callback triggered for CAN%d", (hcan->Instance == CAN1) ? 1 : 2);

    // Retrieve all messages from the FIFO
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msg.header, msg.data) != HAL_OK) {
            LogWarning("CAN: Failed to get message from FIFO 0 in RX callback loop");
            break;
        }
        osStatus_t os_status = osMessageQueuePut(can_message_queue, &msg, 0, 0);
        if (os_status != osOK) {
            LogWarning(
                "CAN: Failed to post message to dispatcher queue from FIFO 0 callback, status: %d", os_status);
        }
    }
}

void CAN_Fifo1CallbackHandler(CAN_HandleTypeDef* hcan) {
    CAN_MessageQueueItem_t message;
    message.hcan = hcan;

    LogVerbose("CAN: FIFO 1 callback triggered for CAN%d", (hcan->Instance == CAN1) ? 1 : 2);

    // Retrieve all messages from the FIFO
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &message.header, message.data) != HAL_OK) {
            LogWarning("CAN: Failed to get message from FIFO 1 in RX callback loop");
            break;
        }
        osStatus_t os_status = osMessageQueuePut(can_message_queue, &message, 0, 0);
        if (os_status != osOK) {
            LogWarning(
                "CAN: Failed to post message to dispatcher queue from FIFO 1 callback, status: %d", os_status);
        }
    }
}

void CAN_TxCallbackHandler(CAN_HandleTypeDef* hcan) {
    LogVerbose("CAN: TX callback triggered for CAN%d", (hcan->Instance == CAN1) ? 1 : 2);
    // Currently not used, but could be implemented to notify when a message has been transmitted
}

void CAN_ErrorCallbackHandler(CAN_HandleTypeDef* hcan) {
    const char* preamble = "CAN: Error callback triggered for CAN%d - Error:";
    uint32_t error_code = HAL_CAN_GetError(hcan);
    switch (error_code) {
        case HAL_CAN_ERROR_NONE:
            LogWarning("%s No Error", preamble);
            break;
        case HAL_CAN_ERROR_EWG:
            LogVerbose("%s Warning Error", preamble);
            break;
        case HAL_CAN_ERROR_EPV:
            LogVerbose("%s Passive Error", preamble);
            break;
        case HAL_CAN_ERROR_BOF:
            LogVerbose("%s Bus-Off Error", preamble);
            break;
        case HAL_CAN_ERROR_STF:
            LogVerbose("%s Stuff Error", preamble);
            break;
        case HAL_CAN_ERROR_FOR:
            LogVerbose("%s Form Error", preamble);
            break;
        case HAL_CAN_ERROR_ACK:
            LogVerbose("%s Acknowledgment Error", preamble);
            break;
        case HAL_CAN_ERROR_BR:
            LogVerbose("%s Bit Recessive Error", preamble);
            break;
        case HAL_CAN_ERROR_BD:
            LogVerbose("%s Bit Dominant Error", preamble);
            break;
        case HAL_CAN_ERROR_CRC:
            LogVerbose("%s CRC Error", preamble);
            break;
        case HAL_CAN_ERROR_RX_FOV0:
            LogVerbose("%s FIFO 0 Overrun Error", preamble);
            break;
        case HAL_CAN_ERROR_RX_FOV1:
            LogVerbose("%s FIFO 1 Overrun Error", preamble);
            break;
        case HAL_CAN_ERROR_TX_ALST0:
            LogVerbose("%s TX Mailbox 0 Lost Arbitration Error", preamble);
            break;
        case HAL_CAN_ERROR_TX_ALST1:
            LogVerbose("%s TX Mailbox 1 Lost Arbitration Error", preamble);
            break;
        case HAL_CAN_ERROR_TX_ALST2:
            LogVerbose("%s TX Mailbox 2 Lost Arbitration Error", preamble);
            break;
        case HAL_CAN_ERROR_TX_TERR0:
            LogVerbose("%s TX Mailbox 0 Transmission Error", preamble);
            break;
        case HAL_CAN_ERROR_TX_TERR1:
            LogVerbose("%s TX Mailbox 1 Transmission Error", preamble);
            break;
        case HAL_CAN_ERROR_TX_TERR2:
            LogVerbose("%s TX Mailbox 2 Transmission Error", preamble);
            break;
        case HAL_CAN_ERROR_TIMEOUT:
            LogVerbose("%s Timeout Error", preamble);
            break;
        case HAL_CAN_ERROR_NOT_INITIALIZED:
            LogVerbose("%s Not Initialized Error", preamble);
            break;
        case HAL_CAN_ERROR_NOT_READY:
            LogVerbose("%s Not Ready Error", preamble);
            break;
        case HAL_CAN_ERROR_NOT_STARTED:
            LogVerbose("%s Not Started Error", preamble);
            break;
        case HAL_CAN_ERROR_PARAM:
            LogVerbose("%s Parameter Error", preamble);
            break;
        case HAL_CAN_ERROR_INVALID_CALLBACK:
            LogVerbose("%s Invalid Callback Error", preamble);
            break;
        case HAL_CAN_ERROR_INTERNAL:
            LogVerbose("%s Internal Error", preamble);
            break;
        default:
            LogWarning("%s Unknown Error Code: 0x%08lX", preamble, error_code);
            break;
    }
}
