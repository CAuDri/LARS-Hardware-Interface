/**
 * @file logger.c
 *
 * @brief CAuDri - Logger implementation for the Hardware Interface
 *
 * Queued UART and USB backends send records to a dedicated background task.
 * Inline and Tracealyzer backends bypass that queue.
 */

#include "logger.h"

#include <stdarg.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <cmsis_os.h>
#include <rmw_microros/rmw_microros.h>
#include <stm32f4xx_hal.h>
#include <trcRecorder.h>

#include "usb_device.h"
#include "usbd_cdc_if.h"

extern UART_HandleTypeDef huart3;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_HandleTypeDef hUsbDeviceHS;

#ifndef DEBUG_LOG_HANDLE
#if DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART || DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART_INLINE
#define DEBUG_LOG_HANDLE huart3
#elif DEBUG_LOG_OUTPUT == LOG_OUTPUT_USB_CDC
#define DEBUG_LOG_HANDLE hUsbDeviceFS
#endif
#endif

#ifndef LOG_PRINTF_HANDLE
#if DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART || DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART_INLINE
#define LOG_PRINTF_HANDLE DEBUG_LOG_HANDLE
#else
#define LOG_PRINTF_HANDLE huart3
#endif
#endif

// Logger task configuration
#define LOG_THREAD_STACK_SIZE 2048
#define LOG_THREAD_PRIORITY osPriorityLow

// Number of recent records retained while the consumer is unavailable.
#define LOG_MESSAGE_QUEUE_SIZE 48

// Buffer size for the log message arguments (32-bit words)
// When increasing this value, the call to logger_parse_arguments() needs to be adjusted manually
#define LOG_ARG_BUFFER_SIZE 8

// Maximum time the logger task waits before checking a pending transfer again.
#define LOG_MAX_TX_TIMEOUT_MS 10

#define LOG_USB_TX_RETRIES 100
#define LOG_USB_TX_RETRY_DELAY_MS 5

// Flags for the logger thread
#define LOG_UART_TX_COMPLETE_FLAG 0x01
#define LOG_USB_TX_COMPLETE_FLAG 0x02
#define LOG_ERROR_PENDING_FLAG 0x04

static osMessageQueueId_t logger_queue = NULL;
static osThreadId_t logger_thread = NULL;

// Buffer to store the format string
static char string_buffer[256] = {0};

// Buffer to store the formatted log message with arguments
static char tx_buffer[256] = {0};

static UART_HandleTypeDef* huart = NULL;
static USBD_HandleTypeDef* husb = NULL;
static bool transmission_pending = false;

static const char log_error_message[] = "Logger: Error occurred during logging, messages may be lost.";

static void logger_uart_tx_complete(UART_HandleTypeDef* huart);
static int8_t logger_usb_tx_complete(uint8_t* buf, uint32_t* len, uint8_t epnum);
static void logger_thread_func(void* argument);

static void logger_mark_error(void) {
    if (logger_thread != NULL) {
        osThreadFlagsSet(logger_thread, LOG_ERROR_PENDING_FLAG);
    }
}

// Initialize TraceRecorder channels if using TraceRecorder output
#if DEBUG_LOG_OUTPUT == LOG_OUTPUT_TRACE
TraceStringHandle_t logger_error_channel;
TraceStringHandle_t logger_warning_channel;
TraceStringHandle_t logger_info_channel;
TraceStringHandle_t logger_debug_channel;

void logger_trace_init(void) {
    xTraceStringRegister("LOG ERROR", &logger_error_channel);
    xTraceStringRegister("LOG WARN", &logger_warning_channel);
    xTraceStringRegister("LOG INFO", &logger_info_channel);
    xTraceStringRegister("LOG DEBUG", &logger_debug_channel);
}
#else
void logger_trace_init(void) {}
#endif

// Basic __io_putchar implementation to be able to use printf
// Will default to output over UART3 (ST-Link on Nucleo boards) if no handle is defined
int __io_putchar(int ch) {
    HAL_UART_Transmit(&LOG_PRINTF_HANDLE, (uint8_t*)&ch, 1, 1);
    return ch;
}

/**
 * @brief Log message struct used to pass log messages to the log thread.
 *
 * Contains the log level, timestamp, format string and a buffer for the variadic arguments.
 *
 * @param level:          Log level of the message (LOG_LEVEL_ERROR, LOG_LEVEL_WARNING, LOG_LEVEL_INFO, LOG_LEVEL_DEBUG)
 * @param timestamp:      Timestamp of the message in milliseconds
 * @param format_string:  Format string for the message
 * @param arg_buffer:     Buffer to store the variadic arguments (32-bit words, aligned)
 */
typedef struct {
    uint32_t level;
    bool success;
    uint32_t timestamp;
    const char* format_string;
    uint32_t arg_buffer[LOG_ARG_BUFFER_SIZE];
} log_message_t;

static StaticQueue_t logger_queue_control_block;
static uint32_t logger_queue_storage[(LOG_MESSAGE_QUEUE_SIZE * sizeof(log_message_t) + sizeof(uint32_t) - 1U) /
                                     sizeof(uint32_t)];
static const osMessageQueueAttr_t logger_queue_attributes = {
    .name = "LogQueue",
    .cb_mem = &logger_queue_control_block,
    .cb_size = sizeof(logger_queue_control_block),
    .mq_mem = logger_queue_storage,
    .mq_size = sizeof(logger_queue_storage),
};

static StaticTask_t logger_thread_control_block;
static uint64_t logger_thread_stack[LOG_THREAD_STACK_SIZE / sizeof(uint64_t)];
static const osThreadAttr_t logger_thread_attributes = {
    .name = "Logger",
    .cb_mem = &logger_thread_control_block,
    .cb_size = sizeof(logger_thread_control_block),
    .stack_mem = logger_thread_stack,
    .stack_size = sizeof(logger_thread_stack),
    .priority = LOG_THREAD_PRIORITY,
};

/**
 * @brief Initialize the queued UART or USB logger backend.
 *
 * Inline, Tracealyzer, disabled, and reserved backends require no logger task.
 */
bool logger_init(void) {
    if (DEBUG_LOG_OUTPUT != LOG_OUTPUT_UART && DEBUG_LOG_OUTPUT != LOG_OUTPUT_USB_CDC) {
        return true;
    }

    if (logger_thread != NULL) {
        LogError("Logger: Thread already running");
        return false;
    }

    if (DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART) {
        huart = (UART_HandleTypeDef*)&DEBUG_LOG_HANDLE;

        HAL_StatusTypeDef status =
            HAL_UART_RegisterCallback(huart, HAL_UART_TX_COMPLETE_CB_ID, logger_uart_tx_complete);

        if (status != HAL_OK) {
            LogInline("Logger: UART callback registration failed");
            return false;
        }
    } else if (DEBUG_LOG_OUTPUT == LOG_OUTPUT_USB_CDC) {
        husb = (USBD_HandleTypeDef*)&DEBUG_LOG_HANDLE;

        // USB CDC does not support callback registration, we will use the pUserData pointer to get the CDC interface
        // This is a bit hacky and might only work with certain versions of the USB stack/CubeMX
        USBD_CDC_ItfTypeDef* cdc_interface = (USBD_CDC_ItfTypeDef*)husb->pUserData[husb->classId];
        if (cdc_interface == NULL) {
            LogInline("Logger: Error retrieving CDC interface");
            return false;
        }
        cdc_interface->TransmitCplt = logger_usb_tx_complete;
    } else {
        LogInline("Logger: No valid output handle");
        return false;
    }

    logger_queue = osMessageQueueNew(LOG_MESSAGE_QUEUE_SIZE, sizeof(log_message_t), &logger_queue_attributes);
    if (logger_queue == NULL) {
        LogInline("Logger: Queue creation failed");
        return false;
    }

    logger_thread = osThreadNew(logger_thread_func, NULL, &logger_thread_attributes);

    if (logger_thread == NULL) {
        LogInline("Logger: Thread creation failed");
        osMessageQueueDelete(logger_queue);
        logger_queue = NULL;
        return false;
    }
    return true;
}

/**
 * @brief Log a message with a specific log level.
 *
 * This function is wrapped by the LogError(), LogWarning(), LogInfo() and LogDebug() macros and should not be called directly.
 * Queued backends return after a non-blocking queue operation.
 *
 * Some types will be promoted when being passed to a variadic function. (e.g. float to double)
 * The stack pointer needs to be 64-bit memory aligned when the function is called.
 * This is necessary to reconstruct the variadic arguments in the logger thread.
 *
 * The compiler will throw a warning if the provided arguments do not match the format specifiers in the format string.
 *
 * @param log_level: Log level of the message (LOG_LEVEL_ERROR, LOG_LEVEL_WARNING, LOG_LEVEL_INFO, LOG_LEVEL_DEBUG)
 * @param message:   Format string for the message
 * @param ...:       Arguments for the format string (same as printf)
 */
static uint32_t logger_get_timestamp(void) {
    if (DEBUG_LOG_TIMESTAMP == LOG_TIMESTAMP_HAL) {
        return HAL_GetTick();
    }
    if (DEBUG_LOG_TIMESTAMP == LOG_TIMESTAMP_SYS) {
        return osKernelGetTickCount();
    }
    if (DEBUG_LOG_TIMESTAMP == LOG_TIMESTAMP_ROS) {
        return (uint32_t)rmw_uros_epoch_millis();
    }
    return 0;
}

static void logger_print_inline_v(uint32_t level, bool success, const char* message, va_list args) {
    const char* level_text = "[UNKNOWN]";
    const char* color = "";

    if (success) {
        level_text = "[INFO]";
        color = LOG_COLOR_GREEN;
    } else {
        switch (level) {
            case LOG_LEVEL_ERROR:
                level_text = "[ERROR]";
                color = LOG_COLOR_RED;
                break;
            case LOG_LEVEL_WARNING:
                level_text = "[WARN]";
                color = LOG_COLOR_YELLOW;
                break;
            case LOG_LEVEL_INFO:
                level_text = "[INFO]";
                break;
            case LOG_LEVEL_DEBUG:
                level_text = "[DEBUG]";
                color = LOG_COLOR_BLUE;
                break;
            default:
                break;
        }
    }

    printf("%s[%08lu]%-7s ", color, (unsigned long)logger_get_timestamp(), level_text);
    vprintf(message, args);
    printf("%s\n", LOG_COLOR_RESET);
}

void logger_log_inline(uint32_t level, bool success, const char* message, ...) {
    va_list args;
    va_start(args, message);

#if DEBUG_LOG_OUTPUT == LOG_OUTPUT_TRACE
    TraceStringHandle_t channel = success                 ? logger_info_channel
                                  : level == LOG_LEVEL_ERROR   ? logger_error_channel
                                  : level == LOG_LEVEL_WARNING ? logger_warning_channel
                                  : level == LOG_LEVEL_DEBUG   ? logger_debug_channel
                                                               : logger_info_channel;
    xTraceVPrintF(channel, message, &args);
#elif DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART || DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART_INLINE || \
    DEBUG_LOG_OUTPUT == LOG_OUTPUT_USB_CDC
    logger_print_inline_v(level, success, message, args);
#else
    (void)level;
    (void)success;
    (void)message;
#endif

    va_end(args);
}

void logger_log_message(uint32_t log_level, bool success, const char* message, ...) {
    va_list args;
    va_start(args, message);

#if DEBUG_LOG_OUTPUT == LOG_OUTPUT_TRACE
    TraceStringHandle_t channel = success                     ? logger_info_channel
                                  : log_level == LOG_LEVEL_ERROR   ? logger_error_channel
                                  : log_level == LOG_LEVEL_WARNING ? logger_warning_channel
                                  : log_level == LOG_LEVEL_DEBUG   ? logger_debug_channel
                                                                   : logger_info_channel;
    xTraceVPrintF(channel, message, &args);
    va_end(args);
    return;
#elif DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART_INLINE
    logger_print_inline_v(log_level, success, message, args);
    va_end(args);
    return;
#elif DEBUG_LOG_OUTPUT != LOG_OUTPUT_UART && DEBUG_LOG_OUTPUT != LOG_OUTPUT_USB_CDC
    (void)log_level;
    (void)success;
    (void)message;
    va_end(args);
    return;
#endif

    // Fallback before the queue has been created. Once it exists, messages can
    // safely be queued even if the logger task has not been scheduled yet.
    if (logger_queue == NULL) {
        logger_print_inline_v(log_level, success, message, args);
        va_end(args);
        return;
    }

    log_message_t msg = {0};
    msg.level = log_level;
    msg.success = success;
    msg.format_string = message;
    msg.timestamp = logger_get_timestamp();

    // Extract the variadic arguments and store them in the log message buffer
    for (uint32_t i = 0; i < LOG_ARG_BUFFER_SIZE; i++) {
        msg.arg_buffer[i] = va_arg(args, uint32_t);
    }

    va_end(args);

    // Keep a bounded sliding window while the logger task is unavailable. Queue
    // operations are non-blocking; eviction only happens when the queue is full.
    osStatus_t status = osMessageQueuePut(logger_queue, &msg, 0, 0);
    if (status == osErrorResource) {
        log_message_t discarded_message;
        osMessageQueueGet(logger_queue, &discarded_message, NULL, 0);
        status = osMessageQueuePut(logger_queue, &msg, 0, 0);
        logger_mark_error();
    } else if (status != osOK) {
        logger_mark_error();
    }
}

/**
 * @brief Clear the terminal for backends that support ANSI output.
 */
void logger_clear_terminal(void) {
#if DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART_INLINE
    printf(LOG_CLEAR_SCREEN LOG_COLOR_RESET);
#elif DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART || DEBUG_LOG_OUTPUT == LOG_OUTPUT_USB_CDC
    logger_log_message(LOG_LEVEL_ERROR, false, LOG_CLEAR_SCREEN);
#endif
}

/**
 * @brief Wrapper for vsnprintf with forced 16-byte stack alignment.
 *
 * Same stack alignment as for the logger_log_message function. This ensures proper argument
 * handling. This is a workaround for the fact that the stack alignment is not guaranteed to be
 * 16-byte by default and should work on most ARM architectures.
 */
// __attribute__((force_align_arg_pointer, aligned(16)))
static int logger_parse_arguments(const char* src_buffer, uint32_t mock_arg, ...) {
    va_list args;
    va_start(args, mock_arg);

    // The variadic arguments should have the same memory layout as the arguments in the logger_log_message function.
    // This is architecture and compiler dependent and might not work on other platforms.
    int length = vsnprintf(tx_buffer, sizeof(tx_buffer), src_buffer, args);

    va_end(args);
    return length;
}

static bool logger_wait_for_transmission(void) {
    if (!transmission_pending) {
        return true;
    }

    uint32_t completion_flag = huart != NULL ? LOG_UART_TX_COMPLETE_FLAG : LOG_USB_TX_COMPLETE_FLAG;
    uint32_t flags = osThreadFlagsWait(completion_flag, osFlagsWaitAny, LOG_MAX_TX_TIMEOUT_MS);
    if ((flags & completion_flag) == 0U) {
        return false;
    }

    transmission_pending = false;
    return true;
}

static bool logger_transmit_message(char* buffer, uint32_t length) {
    if (huart) {
        // Send the formatted message over UART using DMA transfer
        HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart, (uint8_t*)buffer, length);

        if (status != HAL_OK) {
            LogInline("Logger: UART DMA transmit failed, status: %d", status);
            return false;
        }
    } else if (husb) {
        if (husb->dev_state != USBD_STATE_CONFIGURED) {
            return false;
        }

        // Send the formatted message over USB CDC
        uint32_t status = USBD_CDC_SetTxBuffer(husb, (uint8_t*)buffer, length);

        if (status != USBD_OK) {
            LogInline("Logger: USB CDC set TX buffer failed, status: %lu", status);
            return false;
        }

        for (uint32_t i = 0; i < LOG_USB_TX_RETRIES; i++) {
            status = USBD_CDC_TransmitPacket(husb);
            if (status == USBD_OK) {
                break;
            }
            osDelay(LOG_USB_TX_RETRY_DELAY_MS);
        }
        if (status != USBD_OK) {
            LogInline("Logger: USB CDC transmit packet failed, status: %lu", status);
            return false;
        }
    } else {
        // Shouldn't happen
        LogInline("Logger: No valid output handle");
        return false;
    }

    transmission_pending = true;
    return true;
}

/**
 * @brief Format and transmit one queued log record.
 *
 * @param msg Pointer to the record to process
 */
static void logger_parse_message(const log_message_t* msg) {
    // The transfer backend may still own tx_buffer. Never format another
    // message into it until completion has been observed.
    if (!logger_wait_for_transmission()) {
        return;
    }

    const char* log_level_str = "";
    const char* log_color_str = "";

    if (msg->success) {
        log_level_str = "[INFO]";
        log_color_str = LOG_COLOR_GREEN;
    } else switch (msg->level) {
        case LOG_LEVEL_ERROR:
            log_level_str = "[ERROR]";
            log_color_str = LOG_COLOR_RED;
            break;
        case LOG_LEVEL_WARNING:
            log_level_str = "[WARN]";
            log_color_str = LOG_COLOR_YELLOW;
            break;
        case LOG_LEVEL_INFO:
            log_level_str = "[INFO]";
            break;
        case LOG_LEVEL_DEBUG:
            log_level_str = "[DEBUG]";
            log_color_str = LOG_COLOR_BLUE;
            break;
        default:
            log_level_str = "[UNKNOWN]";
            break;
    }

    // Format the log message with timestamp, log level and color
    snprintf(string_buffer,
             sizeof(string_buffer),
             "%s[%08lu]%-7s %s%s\n",
             log_color_str,
             (unsigned long)msg->timestamp,
             log_level_str,
             msg->format_string,
             LOG_COLOR_RESET);

    // Arguments need to be passed manually to properly reconstruct the variadic arguments
    int length = logger_parse_arguments(string_buffer,
                                        0,
                                        msg->arg_buffer[0],
                                        msg->arg_buffer[1],
                                        msg->arg_buffer[2],
                                        msg->arg_buffer[3],
                                        msg->arg_buffer[4],
                                        msg->arg_buffer[5],
                                        msg->arg_buffer[6],
                                        msg->arg_buffer[7]);

    if (length < 0) {
        return;
    }

    uint32_t transmit_length = (uint32_t)length;
    if (transmit_length >= sizeof(tx_buffer)) {
        transmit_length = sizeof(tx_buffer) - 1U;
        logger_mark_error();
    }

    (void)logger_transmit_message(tx_buffer, transmit_length);
}

/**
 * @brief Logger thread function to process log messages from the queue.
 *
 * This function runs in a dedicated thread and waits for log messages in the message queue.
 * When a new message is available, it calls logger_parse_message() to format and send the message over UART.
 *
 * @param argument: Pointer to the thread argument (not used)
 */
static void logger_thread_func(void* argument) {
    (void)argument;
    log_message_t msg;

    while (1) {
#if DEBUG_LOG_OUTPUT == LOG_OUTPUT_USB_CDC
        // Preserve a sliding window of recent messages while disconnected.
        while (husb->dev_state != USBD_STATE_CONFIGURED) {
            // USB reset/disconnect cancels ownership of the previous endpoint
            // buffer and does not necessarily invoke TransmitCplt.
            transmission_pending = false;
            (void)osThreadFlagsClear(LOG_USB_TX_COMPLETE_FLAG);
            osDelay(100);
        }
#endif

        // Wait for new log messages in the queue
        osStatus_t status = osMessageQueueGet(logger_queue, &msg, NULL, osWaitForever);

        if (status != osOK) {
            LogInline("Logger: Queue get failed, status: %d", status);
            continue;
        }
        uint32_t flags = osThreadFlagsWait(LOG_ERROR_PENDING_FLAG, osFlagsWaitAny, 0);
        if ((flags & LOG_ERROR_PENDING_FLAG) != 0) {
            // Notify about previous logging errors
            log_message_t error_msg = {0};
            error_msg.level = LOG_LEVEL_ERROR;
            error_msg.timestamp = logger_get_timestamp();
            error_msg.format_string = log_error_message;
            logger_parse_message(&error_msg);
        }
        logger_parse_message(&msg);
    }
}

/**
 * @brief DMA transfer complete callback for UART transmission
 */
static void logger_uart_tx_complete(UART_HandleTypeDef* huart) {
    if (huart != (UART_HandleTypeDef*)&DEBUG_LOG_HANDLE) {
        return;
    }
    // Notify the log thread that the DMA transfer is complete
    osThreadFlagsSet(logger_thread, LOG_UART_TX_COMPLETE_FLAG);
}

/**
 * @brief USB CDC transmission complete callback
 */
static int8_t logger_usb_tx_complete(uint8_t* buf, uint32_t* len, uint8_t epnum) {
    (void)buf;
    (void)len;
    (void)epnum;
    // Notify the log thread that the USB CDC transfer is complete
    osThreadFlagsSet(logger_thread, LOG_USB_TX_COMPLETE_FLAG);
    return USBD_OK;
}
