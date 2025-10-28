/**
 * @file thread_safe_i2c.c
 *
 * @brief CAuDri - Asynchronous I2C wrapper for multi-threaded applications
 *
 * This file provides a thread-safe wrapper for I2C communication using the HAL library and CMSISv2.
 * Assumes that the HAL library is already initialized and the I2C1 and I2C2 peripherals are properly configured.
 *
 * Only the I2C1 and I2C2 peripherals in master mode are supported.
 * Can not be called from ISR context.
 */
#include "thread_safe_i2c.h"

#include <cmsis_os2.h>
#include <logger.h>

#ifdef LOG_VERBOSE
    #define LogVerbose(...) LogDebug(__VA_ARGS__)
#else
    #define LogVerbose(...)
#endif

// Thread flag (task notification) set by the I2C callbacks
// Change if the flags are already used elsewhere
#define I2C_COMPLETE_FLAG 0x20
#define I2C_ERROR_FLAG 0x40
#define I2C_ABORT_FLAG 0x80

#define I2C_THREAD_FLAGS (I2C_COMPLETE_FLAG | I2C_ERROR_FLAG | I2C_ABORT_FLAG)

// Timeout for threads trying to access the I2C interface.
// Some mem write/read operations can be multiple kB in size and take some time to finish.
#define I2C_MUTEX_TIMEOUT_MS 100

// Timeout values for I2C operations (per tranfered byte)
// They can have a significant impact on the overall performance and should be kept as low as possible
#define I2C_MAX_TX_TIMEOUT_MS 10
#define I2C_MAX_RX_TIMEOUT_MS 10

#define I2C_MEMADD_SIZE I2C_MEMADD_SIZE_16BIT

static osMutexId_t i2c1_mutex;
static osMutexId_t i2c2_mutex;

static osMutexAttr_t i2c1_mutex_attr = {.name = "I2C1_Mutex", .attr_bits = osMutexRecursive};
static osMutexAttr_t i2c2_mutex_attr = {.name = "I2C2_Mutex", .attr_bits = osMutexRecursive};

static osThreadId_t i2c1_thread_id;
static osThreadId_t i2c2_thread_id;

static bool i2c_initialized = false;

/**
 * @brief Initialize the thread-safe I2C wrapper
 *
 * This function must be called before any other I2C functions.
 * It creates the necessary mutexes for thread safety.
 */
bool I2C_Init() {
    i2c1_mutex = osMutexNew(&i2c1_mutex_attr);
    if (i2c1_mutex == NULL) {
        LogError("I2C: Failed to create I2C1 mutex");
        return false;
    }

    i2c2_mutex = osMutexNew(&i2c2_mutex_attr);
    if (i2c2_mutex == NULL) {
        LogError("I2C: Failed to create I2C2 mutex");
        osMutexDelete(i2c1_mutex);
        return false;
    }

    i2c_initialized = true;
    return true;
}

/**
 * @brief Thread-safe I2C master transmit
 *
 * Same usage as HAL_I2C_Master_Transmit_IT, but thread-safe.
 * Can not be called from ISR context.
 *
 * @param hi2c Pointer to a configured I2C handle
 * @param address 7-bit I2C device address
 * @param data Pointer to data buffer to transmit
 * @param size Amount of data to transmit in bytes
 */
HAL_StatusTypeDef I2C_Transmit(I2C_HandleTypeDef* hi2c, uint16_t address, uint8_t* data, uint16_t size) {
    if (!i2c_initialized) {
        LogError("I2C: I2C_Transmit called before I2C_Init");
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    osMutexId_t mutex = (hi2c->Instance == I2C1) ? i2c1_mutex : i2c2_mutex;
    if (osMutexAcquire(mutex, I2C_MUTEX_TIMEOUT_MS) != osOK) {
        LogDebug("I2C: Mutex acquire failed on transmit in thread '%s'", osThreadGetName(osThreadGetId()));
        return HAL_TIMEOUT;
    }

    if (hi2c->Instance == I2C1) {
        i2c1_thread_id = osThreadGetId();
    } else {
        i2c2_thread_id = osThreadGetId();
    }

    osThreadFlagsClear(I2C_THREAD_FLAGS);

    status = HAL_I2C_Master_Transmit_IT(hi2c, address, data, size);
    if (status != HAL_OK) {
        osMutexRelease(mutex);
        return status;
    }

    // Timeout depends on the transfered size to allow larger data transfers
    uint32_t flags = osThreadFlagsWait(
        I2C_COMPLETE_FLAG | I2C_ERROR_FLAG, osFlagsWaitAny, I2C_MAX_TX_TIMEOUT_MS * size);

    if (flags == osFlagsErrorTimeout) {
        LogDebug("I2C: Transmit timeout in thread '%s'", osThreadGetName(osThreadGetId()));
    }

    if (flags >= osFlagsError || flags == I2C_ERROR_FLAG) {
        HAL_I2C_StateTypeDef state = HAL_I2C_GetState(hi2c);
        if (state == HAL_I2C_STATE_BUSY || state == HAL_I2C_STATE_BUSY_TX) {
            // Reset the HAL state and I2C peripheral to stop any ongoing transfers
            HAL_I2C_Master_Abort_IT(hi2c, address);

            flags = osThreadFlagsWait(I2C_ABORT_FLAG, osFlagsWaitAny, I2C_MAX_TX_TIMEOUT_MS);
            if (flags >= osFlagsError) {
                // If the abort fails, the I2C bus might be deadlocked
                // Not much we can do here except logging the error
                LogWarning("I2C: Abort transmit timeout, I2C state: 0x%02X", HAL_I2C_GetState(hi2c));
            }
        }

        osThreadFlagsClear(I2C_THREAD_FLAGS);
        osMutexRelease(mutex);

        // LogWarning("I2C: Transmit error in thread '%s', flags: 0x%08lX, HAL error: %ld",
        //            osThreadGetName(osThreadGetId()),
        //            flags,
        //            HAL_I2C_GetError(hi2c));
        return HAL_ERROR;
    }

    LogVerbose("I2C: Transmit complete in thread '%s'", osThreadGetName(osThreadGetId()));

    if (hi2c->Instance == I2C1) {
        i2c1_thread_id = NULL;
    } else {
        i2c2_thread_id = NULL;
    }
    osMutexRelease(mutex);
    return HAL_OK;
}

/**
 * @brief Thread-safe I2C master receive
 *
 * Same usage as HAL_I2C_Master_Receive_IT, but thread-safe.
 * Can not be called from ISR context.
 * 
 * @param hi2c Pointer to a configured I2C handle
 * @param address 7-bit I2C device address
 * @param data Pointer to data buffer to receive
 * @param size Amount of data to receive in bytes
 */
HAL_StatusTypeDef I2C_Receive(I2C_HandleTypeDef* hi2c, uint16_t address, uint8_t* data, uint16_t size) {
    if (!i2c_initialized) {
        LogError("I2C: I2C_Receive called before I2C_Init");
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    osMutexId_t mutex = (hi2c->Instance == I2C1) ? i2c1_mutex : i2c2_mutex;
    if (osMutexAcquire(mutex, I2C_MUTEX_TIMEOUT_MS) != osOK) {
        LogDebug("I2C: Mutex acquire failed on receive in thread '%s'", osThreadGetName(osThreadGetId()));
        return HAL_TIMEOUT;
    }

    if (hi2c->Instance == I2C1) {
        i2c1_thread_id = osThreadGetId();
    } else {
        i2c2_thread_id = osThreadGetId();
    }

    osThreadFlagsClear(I2C_THREAD_FLAGS);

    status = HAL_I2C_Master_Receive_IT(hi2c, address, data, size);
    if (status != HAL_OK) {
        osMutexRelease(mutex);
        return status;
    }

    // Timeout depends on the transfered size to allow larger data transfers
    uint32_t flags = osThreadFlagsWait(
        I2C_COMPLETE_FLAG | I2C_ERROR_FLAG, osFlagsWaitAny, I2C_MAX_RX_TIMEOUT_MS * size);

    if (flags == osFlagsErrorTimeout) {
        LogDebug("I2C: Receive timeout in thread '%s'", osThreadGetName(osThreadGetId()));
    }

    if (flags >= osFlagsError || flags == I2C_ERROR_FLAG) {
        HAL_I2C_StateTypeDef state = HAL_I2C_GetState(hi2c);
        if (state == HAL_I2C_STATE_BUSY || state == HAL_I2C_STATE_BUSY_RX) {
            // Reset the HAL state and I2C peripheral to stop any ongoing transfers
            HAL_I2C_Master_Abort_IT(hi2c, address);

            flags = osThreadFlagsWait(I2C_ABORT_FLAG, osFlagsWaitAny, I2C_MAX_RX_TIMEOUT_MS);
            if (flags >= osFlagsError) {
                // If the abort fails, the I2C bus might be deadlocked
                // Not much we can do here except logging the error
                LogWarning("I2C: Abort receive timeout, I2C state: 0x%02X", HAL_I2C_GetState(hi2c));
            }
        }

        osThreadFlagsClear(I2C_THREAD_FLAGS);
        osMutexRelease(mutex);

        // LogWarning("I2C: Receive error in thread '%s', flags: 0x%08lX, HAL error: %ld",
        //            osThreadGetName(osThreadGetId()),
        //            flags,
        //            HAL_I2C_GetError(hi2c));
        return HAL_ERROR;
    }

    LogVerbose("I2C: Receive complete in thread '%s'", osThreadGetName(osThreadGetId()));

    if (hi2c->Instance == I2C1) {
        i2c1_thread_id = NULL;
    } else {
        i2c2_thread_id = NULL;
    }
    osMutexRelease(mutex);
    return HAL_OK;
}

/**
 * @brief Thread-safe I2C memory write
 *
 * Same usage as HAL_I2C_Mem_Write_IT, but thread-safe.
 * Can not be called from ISR context.
 * 
 * @param hi2c Pointer to a configured I2C handle
 * @param address 7-bit I2C device address
 * @param memAddress Memory address to write to
 * @param data Pointer to data buffer to write
 * @param size Amount of data to write in bytes
 */
HAL_StatusTypeDef I2C_MemWrite(I2C_HandleTypeDef* hi2c, uint16_t address, uint16_t memAddress, uint8_t* data, uint16_t size) {
    if (!i2c_initialized) {
        LogError("I2C: I2C_MemWrite called before I2C_Init");
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    osMutexId_t mutex = (hi2c->Instance == I2C1) ? i2c1_mutex : i2c2_mutex;
    if (osMutexAcquire(mutex, I2C_MUTEX_TIMEOUT_MS) != osOK) {
        LogDebug("I2C: Mutex acquire failed on mem write in thread '%s'", osThreadGetName(osThreadGetId()));
        return HAL_TIMEOUT;
    }

    if (hi2c->Instance == I2C1) {
        i2c1_thread_id = osThreadGetId();
    } else {
        i2c2_thread_id = osThreadGetId();
    }

    osThreadFlagsClear(I2C_THREAD_FLAGS);

    status = HAL_I2C_Mem_Write_IT(hi2c, address, memAddress, I2C_MEMADD_SIZE, data, size);
    if (status != HAL_OK) {
        osMutexRelease(mutex);
        return status;
    }

    // Timeout depends on the transfered size to allow larger data transfers
    uint32_t flags = osThreadFlagsWait(
        I2C_COMPLETE_FLAG | I2C_ERROR_FLAG, osFlagsWaitAny, I2C_MAX_TX_TIMEOUT_MS * size);

    if (flags == osFlagsErrorTimeout) {
        LogDebug("I2C: Mem write timeout in thread '%s'", osThreadGetName(osThreadGetId()));
        // Stop ongoing transfer on timeout to avoid setting the thread flag later
        HAL_I2C_Master_Abort_IT(hi2c, address);
    }

    if (flags >= osFlagsError || flags == I2C_ERROR_FLAG) {
        HAL_I2C_StateTypeDef state = HAL_I2C_GetState(hi2c);
        if (state == HAL_I2C_STATE_BUSY || state == HAL_I2C_STATE_BUSY_TX) {
            // Reset the HAL state and I2C peripheral to stop any ongoing transfers
            HAL_I2C_Master_Abort_IT(hi2c, address);

            flags = osThreadFlagsWait(I2C_ABORT_FLAG, osFlagsWaitAny, I2C_MAX_TX_TIMEOUT_MS);
            if (flags >= osFlagsError) {
                // If the abort fails, the I2C bus might be deadlocked
                // Not much we can do here except logging the error
                LogWarning("I2C: Abort mem write timeout, I2C state: 0x%02X", HAL_I2C_GetState(hi2c));
            }
        }

        osThreadFlagsClear(I2C_THREAD_FLAGS);
        osMutexRelease(mutex);

        LogWarning("I2C: Mem write error in thread '%s', flags: 0x%08lX, HAL error: %ld",
                   osThreadGetName(osThreadGetId()),
                   flags,
                   HAL_I2C_GetError(hi2c));
        return HAL_ERROR;
    }

    LogVerbose("I2C: Mem write complete in thread '%s'", osThreadGetName(osThreadGetId()));

    if (hi2c->Instance == I2C1) {
        i2c1_thread_id = NULL;
    } else {
        i2c2_thread_id = NULL;
    }
    osMutexRelease(mutex);
    return HAL_OK;
}

/**
 * @brief Thread-safe I2C memory read
 *
 * Same usage as HAL_I2C_Mem_Read_IT, but thread-safe.
 * Can not be called from ISR context.
 * 
 * @param hi2c Pointer to a configured I2C handle
 * @param address 7-bit I2C device address
 * @param memAddress Memory address to read from
 * @param data Pointer to data buffer to receive
 * @param size Amount of data to read in bytes
 */
HAL_StatusTypeDef I2C_MemRead(I2C_HandleTypeDef* hi2c, uint16_t address, uint16_t memAddress, uint8_t* data, uint16_t size) {
    if (!i2c_initialized) {
        LogError("I2C: I2C_MemRead called before I2C_Init");
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    osMutexId_t mutex = (hi2c->Instance == I2C1) ? i2c1_mutex : i2c2_mutex;
    if (osMutexAcquire(mutex, I2C_MUTEX_TIMEOUT_MS) != osOK) {
        LogDebug("I2C: Mutex acquire failed on mem read in thread '%s'", osThreadGetName(osThreadGetId()));
        return HAL_TIMEOUT;
    }

    if (hi2c->Instance == I2C1) {
        i2c1_thread_id = osThreadGetId();
    } else {
        i2c2_thread_id = osThreadGetId();
    }

    osThreadFlagsClear(I2C_THREAD_FLAGS);

    status = HAL_I2C_Mem_Read_IT(hi2c, address, memAddress, I2C_MEMADD_SIZE, data, size);
    if (status != HAL_OK) {
        osMutexRelease(mutex);
        return status;
    }

    // Timeout depends on the transfered size to allow larger data transfers
    uint32_t flags = osThreadFlagsWait(
        I2C_COMPLETE_FLAG | I2C_ERROR_FLAG, osFlagsWaitAny, I2C_MAX_RX_TIMEOUT_MS * size);

    if (flags == osFlagsErrorTimeout) {
        LogDebug("I2C: Mem read timeout in thread '%s'", osThreadGetName(osThreadGetId()));
    }

    if (flags >= osFlagsError || flags == I2C_ERROR_FLAG) {
        HAL_I2C_StateTypeDef state = HAL_I2C_GetState(hi2c);
        if (state == HAL_I2C_STATE_BUSY || state == HAL_I2C_STATE_BUSY_RX) {
            // Reset the HAL state and I2C peripheral to stop any ongoing transfers
            HAL_I2C_Master_Abort_IT(hi2c, address);

            flags = osThreadFlagsWait(I2C_ABORT_FLAG, osFlagsWaitAny, I2C_MAX_RX_TIMEOUT_MS);
            if (flags >= osFlagsError) {
                // If the abort fails, the I2C bus might be deadlocked
                // Not much we can do here except logging the error
                LogWarning("I2C: Abort mem read timeout, I2C state: 0x%02X", HAL_I2C_GetState(hi2c));
            }
        }

        osThreadFlagsClear(I2C_THREAD_FLAGS);
        osMutexRelease(mutex);

        LogWarning("I2C: Mem read error in thread '%s', flags: 0x%08lX, HAL error: %ld",
                   osThreadGetName(osThreadGetId()),
                   flags,
                   HAL_I2C_GetError(hi2c));
        return HAL_ERROR;
    }

    LogVerbose("I2C: Mem read complete in thread '%s'", osThreadGetName(osThreadGetId()));

    if (hi2c->Instance == I2C1) {
        i2c1_thread_id = NULL;
    } else {
        i2c2_thread_id = NULL;
    }
    osMutexRelease(mutex);
    return HAL_OK;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    osThreadFlagsSet((hi2c->Instance == I2C1) ? i2c1_thread_id : i2c2_thread_id, I2C_COMPLETE_FLAG);
    LogVerbose("Master Tx complete callback for I2C%d", (hi2c->Instance == I2C1) ? 1 : 2);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    osThreadFlagsSet((hi2c->Instance == I2C1) ? i2c1_thread_id : i2c2_thread_id, I2C_COMPLETE_FLAG);
    LogVerbose("Master Rx complete callback for I2C%d", (hi2c->Instance == I2C1) ? 1 : 2);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    osThreadFlagsSet((hi2c->Instance == I2C1) ? i2c1_thread_id : i2c2_thread_id, I2C_COMPLETE_FLAG);
    LogVerbose("Memory Tx complete callback for I2C%d", (hi2c->Instance == I2C1) ? 1 : 2);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    osThreadFlagsSet((hi2c->Instance == I2C1) ? i2c1_thread_id : i2c2_thread_id, I2C_COMPLETE_FLAG);
    LogVerbose("Memory Rx complete callback for I2C%d", (hi2c->Instance == I2C1) ? 1 : 2);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
    osThreadFlagsSet((hi2c->Instance == I2C1) ? i2c1_thread_id : i2c2_thread_id, I2C_ERROR_FLAG);
    // LogError("I2C error callback for I2C%d", (hi2c->Instance == I2C1) ? 1 : 2);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef* hi2c) {
    osThreadFlagsSet((hi2c->Instance == I2C1) ? i2c1_thread_id : i2c2_thread_id, I2C_ABORT_FLAG);
    // LogError("I2C abort complete callback for I2C%d", (hi2c->Instance == I2C1) ? 1 : 2);
}