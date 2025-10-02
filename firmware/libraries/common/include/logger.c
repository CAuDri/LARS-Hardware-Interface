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

// Buffer size for the log message arguments (32-bit words)
// When increasing this value, the call to logger_parse_arguments() needs to be adjusted manually
#define LOG_ARG_BUFFER_SIZE 8

// Flags for the logger thread
#define LOG_DMA_COMPLETE_FLAG 0x01

/**
 * @brief Log message struc used to pass log messages to the log thread.
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
} LogMessage;

static osMessageQueueId_t logger_queue = NULL;
static osMessageQueueAttr_t logger_queue_attr = {0};

static osThreadId_t logger_thread = NULL;
static osThreadAttr_t logger_thread_attr = {0};
static bool logger_thread_running = false;

// Buffer to store the format string
static char string_buffer[256] = {0};

// Buffer to store the formatted log message with arguments
static char log_tx_buffer[256] = {0};

static UART_HandleTypeDef* logger_huart = NULL;

static void logger_dma_tx_complete(DMA_HandleTypeDef* hdma);
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

/**
 * @brief Initialize the logger thread for logging messages over UART.
 *
 * Calling this function is only necessary when using the LOG_OUTPUT_UART option.
 * Log messages will be stored in a message queue and processed by the logger thread in the background.
 *
 * @param config: Configuration struct for the logger thread
 */
bool logger_init(const LoggerConfig* config) {
// The log thread is only needed when using the normal UART output
#if DEBUG_LOG_OUTPUT != LOG_OUTPUT_UART
    return true;
#endif

    if (config == NULL) {
        LogInline("Logger: Invalid configuration");
        return false;
    }
    if (config->logger_queue_size == 0) {
        LogInline("Logger: Invalid queue size");
        return false;
    }
    if (logger_thread_running) {
        LogError("Logger: Thread already running");
        return false;
    }

    logger_huart = config->huart;

    HAL_StatusTypeDef dma_status =
        HAL_DMA_RegisterCallback(config->hdma, HAL_DMA_XFER_CPLT_CB_ID, logger_dma_tx_complete);

    if (dma_status != HAL_OK) {
        LogInline("Logger: DMA callback registration failed");
        return false;
    }

    // The logger queue will be allocated dynamically for now
    logger_queue_attr.name = "LogQueue";
    logger_queue = osMessageQueueNew(config->logger_queue_size, sizeof(LogMessage), &logger_queue_attr);
    if (logger_queue == NULL) {
        LogInline("Logger: Queue creation failed");
        return false;
    }

    logger_thread_attr.name = "Logger";
    logger_thread_attr.priority = config->logger_thread_priority;
    logger_thread_attr.stack_size = config->logger_stack_size;

    logger_thread = osThreadNew(logger_thread_func, NULL, &logger_thread_attr);

    if (logger_thread == NULL) {
        LogInline("Logger: Thread creation failed");
        return false;
    }

    uint32_t flags = osThreadFlagsSet(logger_thread, LOG_DMA_COMPLETE_FLAG);
    if (flags != LOG_DMA_COMPLETE_FLAG) {
        LogInline("Logger: Thread flags set failed");
        return false;
    }

    // Wait a moment to ensure that the logger thread is running
    osDelay(2);

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

    LogMessage msg = {0};
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

    if (status != osOK) {
        LogInline("Logger: Log Queue full, dropping message");
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
__attribute__((force_align_arg_pointer, aligned(16))) static uint32_t logger_parse_arguments(char* src_buffer,
                                                                                             uint32_t mock_arg,
                                                                                             ...) {
    va_list args;
    va_start(args, mock_arg);

    // The variadic arguments should have the same memory layout as the arguments in the logger_log_message function.
    // This is architecture and compiler dependent and might not work on other platforms.
    uint32_t length = vsnprintf(log_tx_buffer, sizeof(log_tx_buffer), src_buffer, args);

    va_end(args);
    return length;
}

/**
 * @brief Parse and format a log message and send it over UART using DMA.
 *
 * This function is called by the logger thread to process log messages from the queue.
 * It formats the log message with timestamp, log level and color codes, then sends it over UART using DMA.
 *
 * @param msg: Pointer to the log message to be processed
 */
static void logger_parse_message(const LogMessage* msg) {
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

    osThreadFlagsWait(LOG_DMA_COMPLETE_FLAG, osFlagsWaitAny, 10);

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

    // Send the formatted message over UART using DMA transfer
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(logger_huart, (uint8_t*)log_tx_buffer, length);

    if (status != HAL_OK) {
        LogInline("Logger: UART DMA transmit failed, status: %d", status);
        return;
    }

    // Check for truncation errors during formatting
    if (length >= sizeof(log_tx_buffer)) {
        HAL_UART_Transmit_DMA(
            logger_huart, (uint8_t*)"\x1B[31m --- LOG MESSAGE TRUNCATED --- \x1B[0m\n", 40);
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

    LogMessage msg;

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
static void logger_dma_tx_complete(DMA_HandleTypeDef* hdma) {
    // Notify the log thread that the DMA transfer is complete
    osThreadFlagsSet(logger_thread, 0x01);
}