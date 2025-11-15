/**
 * @file logger.c
 *
 * @brief CAuDri - Logger implementation for the Hardware Interface
 *
 * Contains the implementation of the UART logger output using DMA and a dedicated logging thread.
 * Log messages are sent to a message queue and processed by the logger thread in the background.
 * This allows for non-blocking logging and reduces the impact on real-time performance.
 */

#include "logger.h"

#include <trcRecorder.h>

#include "usb_device.h"
#include "usbd_cdc_if.h"

// Thread configuration
#define LOG_THREAD_STACK_SIZE 1024
#define LOG_THREAD_PRIORITY osPriorityLow

// Nr of messages that can be stored in the log message queue
// If this number is too low, log messages might be dropped on high system load
#define LOG_MESSAGE_QUEUE_SIZE 48

// Buffer size for the log message arguments (32-bit words)
// When increasing this value, the call to logger_parse_arguments() needs to be adjusted manually
#define LOG_ARG_BUFFER_SIZE 8

// Timeout for UART/USB transmissions
// A new transmission will be started if the previous one did not complete within this time
#define LOG_MAX_TX_TIMEOUT_MS 10

#define LOG_USB_TX_RETRIES 100
#define LOG_USB_TX_RETRY_DELAY_MS 20

// Flags for the logger thread
#define LOG_UART_TX_COMPLETE_FLAG 0x01
#define LOG_USB_TX_COMPLETE_FLAG 0x02

static osMessageQueueId_t logger_queue = NULL;
static osMessageQueueAttr_t logger_queue_attr = {0};

static osThreadId_t logger_thread = NULL;
static osThreadAttr_t logger_thread_attr = {0};
static bool logger_thread_running = false;

// Buffer to store the format string
static char string_buffer[256] = {0};

// Buffer to store the formatted log message with arguments
static char tx_buffer[256] = {0};

static UART_HandleTypeDef* huart = NULL;
static USBD_HandleTypeDef* husb = NULL;

static bool initialized = false;

static void logger_uart_tx_complete(UART_HandleTypeDef* huart);
static int8_t logger_usb_tx_complete(uint8_t* buf, uint32_t* len, uint8_t epnum);
static void logger_thread_func(void* argument);

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
    uint32_t timestamp;
    const char* format_string;
    uint32_t arg_buffer[LOG_ARG_BUFFER_SIZE];
} log_message_t;

/**
 * @brief Initialize the logger thread for logging messages over UART.
 *
 * Calling this function is only necessary when using the LOG_OUTPUT_UART option.
 * Log messages will be stored in a message queue and processed by the logger thread in the background.
 *
 * @param config: Configuration struct for the logger thread
 */
bool logger_init(void) {
    if (DEBUG_LOG_OUTPUT != LOG_OUTPUT_UART && DEBUG_LOG_OUTPUT != LOG_OUTPUT_USB_CDC) {
        return true;
    }

    if (logger_thread_running) {
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
            LogInline("Logger: Error retrieving CDC interface, will not use callbacks");
        } else {
            cdc_interface->TransmitCplt = logger_usb_tx_complete;
        }
    } else {
        LogInline("Logger: No valid output handle");
        return false;
    }

    // The logger queue and thread will be allocated dynamically for now
    logger_queue_attr.name = "LogQueue";
    logger_queue = osMessageQueueNew(LOG_MESSAGE_QUEUE_SIZE, sizeof(log_message_t), &logger_queue_attr);
    if (logger_queue == NULL) {
        LogInline("Logger: Queue creation failed");
        return false;
    }

    logger_thread_attr.name = "Logger";
    logger_thread_attr.priority = LOG_THREAD_PRIORITY;
    logger_thread_attr.stack_size = LOG_THREAD_STACK_SIZE;

    logger_thread = osThreadNew(logger_thread_func, NULL, &logger_thread_attr);

    if (logger_thread == NULL) {
        LogInline("Logger: Thread creation failed");
        return false;
    }

    uint32_t flags = osThreadFlagsSet(logger_thread, LOG_UART_TX_COMPLETE_FLAG);
    if (flags != LOG_UART_TX_COMPLETE_FLAG) {
        LogInline("Logger: Thread flags set failed");
        return false;
    }

    initialized = true;
    return true;
}

/**
 * @brief Log a message with a specific log level.
 *
 * This function is wrapped by the LogError(), LogWarning(), LogInfo() and LogDebug() macros and should not be called directly.
 * It is only used when using the LOG_OUTPUT_UART option.
 *
 * Some types will be promoted when being passed to a variadic function. (e.g. float to double)
 * The stack pointer needs to be 64-bit memory aligned when the function is called.
 * This is neccessary to properly reconstruct the variadic arguments in the logger thread.
 *
 * The compiler will throw a warning if the provided arguments do not match the format specifiers in the format string.
 *
 * @param log_level: Log level of the message (LOG_LEVEL_ERROR, LOG_LEVEL_WARNING, LOG_LEVEL_INFO, LOG_LEVEL_DEBUG)
 * @param message:   Format string for the message
 * @param ...:       Arguments for the format string (same as printf)
 */
void logger_log_message(uint32_t log_level, const char* message, ...) {
    // Fallback in case the log thread is not running
    if (!logger_thread_running) {
        LogInline(message);
        return;
    }

    log_message_t msg = {0};
    msg.level = log_level;
    msg.format_string = message;


    if (DEBUG_LOG_TIMESTAMP == LOG_TIMESTAMP_SYS) {
        // Milliseconds since boot
        msg.timestamp = HAL_GetTick();
    } else if (DEBUG_LOG_TIMESTAMP == LOG_TIMESTAMP_ROS) {
        // Syncronized time with ROS
        msg.timestamp = (uint32_t)rmw_uros_epoch_millis();
    }

    // Extract the variadic arguments and store them in the log message buffer
    va_list args;
    va_start(args, message);

    for (uint32_t i = 0; i < LOG_ARG_BUFFER_SIZE; i++) {
        msg.arg_buffer[i] = va_arg(args, uint32_t);
    }

    va_end(args);

    // Log Messages will be saved in the log queue and later retrieved by the log thread
    osStatus_t status = osMessageQueuePut(logger_queue, &msg, 0, 0);

    if (status != osOK && DEBUG_LOG_LEVEL >= LOG_LEVEL_DEBUG) {
        // TODO: Find a better way to report dropped log messages that won't interfere with system performance
        // LogInline("Logger: Log Queue full, dropping message, status: %d", status);
        return;
    }
}

/**
 * @brief Clear the terminal screen.
 *
 * This function is wrapped by the LogClear() macro and should not be called directly.
 * It is only used when using the LOG_OUTPUT_UART option.
 */
void logger_clear_terminal(void) { logger_log_message(LOG_LEVEL_ERROR, "\033[2J\033[H"); }

/**
 * @brief Wrapper for vsnprintf with forced 16-byte stack alignment.
 *
 * Same stack alignment as for the logger_log_message function. This ensures proper argument
 * handling. This is a workaround for the fact that the stack alignment is not guaranteed to be
 * 16-byte by default and should work on most ARM architectures.
 */
// __attribute__((force_align_arg_pointer, aligned(16)))
static uint32_t logger_parse_arguments(char* src_buffer, uint32_t mock_arg, ...) {
    va_list args;
    va_start(args, mock_arg);

    // The variadic arguments should have the same memory layout as the arguments in the logger_log_message function.
    // This is architecture and compiler dependent and might not work on other platforms.
    uint32_t length = vsnprintf(tx_buffer, sizeof(tx_buffer), src_buffer, args);

    va_end(args);
    return length;
}

static void logger_transmit_message(char* buffer, uint32_t length) {
    if (huart) {
        // Send the formatted message over UART using DMA transfer
        HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(huart, (uint8_t*)buffer, length);

        if (status != HAL_OK) {
            LogInline("Logger: UART DMA transmit failed, status: %d", status);
            return;
        }
    } else if (husb) {
        // Send the formatted message over USB CDC
        uint32_t status = USBD_CDC_SetTxBuffer(husb, (uint8_t*)buffer, length);

        if (status != USBD_OK) {
            LogInline("Logger: USB CDC set TX buffer failed, status: %lu", status);
            return;
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
            return;
        }
    } else {
        // Shouldn't happen
        LogInline("Logger: No valid output handle");
        return;
    }
}

/**
 * @brief Parse and format a log message and send it over UART using DMA.
 *
 * This function is called by the logger thread to process log messages from the queue.
 * It formats the log message with timestamp, log level and color codes, then sends it over UART using DMA.
 *
 * @param msg: Pointer to the log message to be processed
 */
static void logger_parse_message(const log_message_t* msg) {
    const char* log_level_str = "";
    const char* log_color_str = "";

    switch (msg->level) {
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
        case LOG_LEVEL_SUCCESS:
            log_level_str = "[INFO]";
            log_color_str = LOG_COLOR_GREEN;
            break;
        default:
            log_level_str = "[UNKNOWN]";
            break;
    }

    // Format the log message with timestamp, log level and color
    snprintf(string_buffer,
             sizeof(string_buffer),
             "%s[%08ld]%-7s %s%s\n",
             log_color_str,
             msg->timestamp,
             log_level_str,
             msg->format_string,
             LOG_COLOR_RESET);

    // Wait for UART/USB transmission to complete
    // If the previous transmission did not complete within the timeout, we will start the next one anyway
    osThreadFlagsWait(LOG_UART_TX_COMPLETE_FLAG | LOG_USB_TX_COMPLETE_FLAG, osFlagsWaitAny, LOG_MAX_TX_TIMEOUT_MS);

    // Arguments need to be passed manually to properly reconstruct the variadic arguments
    uint32_t length = logger_parse_arguments(string_buffer,
                                             0,
                                             msg->arg_buffer[0],
                                             msg->arg_buffer[1],
                                             msg->arg_buffer[2],
                                             msg->arg_buffer[3],
                                             msg->arg_buffer[4],
                                             msg->arg_buffer[5],
                                             msg->arg_buffer[6],
                                             msg->arg_buffer[7]);

    logger_transmit_message(tx_buffer, length);

    // Check for truncation errors during formatting
    if (length >= sizeof(tx_buffer)) {
        logger_transmit_message("\x1B[31m --- LOG MESSAGE TRUNCATED --- \x1B[0m\n", 40);
    }
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
    logger_thread_running = true;
    log_message_t msg;

    #if DEBUG_LOG_OUTPUT == LOG_OUTPUT_USB_CDC
    // We will wait until the USB connection is established before processing log messages
    // This will decrease the chance of losing log messages during startup
    while (husb->dev_state != USBD_STATE_CONFIGURED) {
        osDelay(100);
    }
    osDelay(1000);
    #endif

    while (1) {
        // Wait for new log messages in the queue
        osStatus_t status = osMessageQueueGet(logger_queue, &msg, NULL, osWaitForever);

        if (status != osOK) {
            LogInline("Logger: Queue get failed, status: %d", status);
            continue;
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
    // Notify the log thread that the USB CDC transfer is complete
    osThreadFlagsSet(logger_thread, LOG_USB_TX_COMPLETE_FLAG);
    return USBD_OK;
}