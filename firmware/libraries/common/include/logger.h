/**
 * @file logger.h
 * @brief CAuDri - Logging interface for our Bussystem
 * 
 * Different log output options are available, including UART, USB CDC, Tracealyzer and ROS2.
 * The interface should be fully compatible with the FreeRTOS Plus Logger.
 * 
 * ----- Usage -----
 * 
 * - LogError()   : Logs an error message (critical error)
 * - LogWarning() : Logs a warning message (non-critical error)
 * - LogInfo()    : Logs an info message (general information)
 * - LogDebug()   : Logs a debug message (more detailed information)
 * 
 * - LogSuccess() : Logs a success message (highlighted in green, INFO level)
 * - LogClear()   : Clears the terminal screen
 * - LogEmpty()   : Prints an empty line
 * 
 * Arguments can be passed to the log functions like printf, no \n is needed:
 *   LogInfo("Received data: %d", data);
 * 
 * ----- Config options ----- 
 *  
 * - DEBUG_LOG_LEVEL: Set the (minimum) log level for the logger
 *      - LOG_LEVEL_NONE
 *      - LOG_LEVEL_ERROR
 *      - LOG_LEVEL_WARNING
 *      - LOG_LEVEL_INFO
 *      - LOG_LEVEL_DEBUG
 * 
 * - DEBUG_LOG_OUTPUT: Set the output for the logger
 *      - LOG_OUTPUT_UART        : Output over UART using a dedicated logger thread
 *      - LOG_OUTPUT_UART_INLINE : Output over UART directly from the calling thread (not recommended)
 *      - LOG_OUTPUT_USB_CDC     : Same as UART but over USB (WIP)
 *      - LOG_OUTPUT_TRACE       : Tracealyzer event output
 *      - LOG_OUTPUT_ROS         : Output over ROS2 /rosout (WIP)
 * 
 * - DEBUG_LOG_TIMESTAMP: Set the timestamp used for the logger
 *     - LOG_TIMESTAMP_NONE
 *     - LOG_TIMESTAMP_SYS    : FreeRTOS tick (ms elapsed since last reset)
 *     - LOG_TIMESTAMP_ROS    : ROS timestamp (ms since epoch, if synced with agent)
 */
#pragma once

// Device specific config that contains the macro definitions
#include "debug_config.h"

#include <stm32f4xx_hal.h>
#include <cmsis_os.h>
#include <rmw_microros/rmw_microros.h>

#include "trcRecorder.h"

// Log level needs to be set with DEGUB_LOG_LEVEL
#define LOG_LEVEL_NONE    0
#define LOG_LEVEL_ERROR   1
#define LOG_LEVEL_WARNING 2
#define LOG_LEVEL_INFO    3
#define LOG_LEVEL_DEBUG   4

// Additional log level for success messages (highlighted)
#define LOG_LEVEL_SUCCESS LOG_LEVEL_DEBUG + 1 

// Log output can be set with DEBUG_LOG_OUTPUT
#define LOG_OUTPUT_NONE     0
#define LOG_OUTPUT_UART     1
#define LOG_OUTPUT_USB_CDC  2
#define LOG_OUTPUT_TRACE    3
#define LOG_OUTPUT_ROS      4

// Log timestamp can be set with DEBUG_LOG_TIMESTAMP
#define LOG_TIMESTAMP_NONE  0
#define LOG_TIMESTAMP_ROS   1   // ROS time
#define LOG_TIMESTAMP_SYS   2   // FreeRTOS tick

// ANSI codes for terminal output
#define LOG_COLOR_RED     "\x1B[31m"
#define LOG_COLOR_YELLOW  "\x1B[33m"
#define LOG_COLOR_BLUE    "\x1B[34m"
#define LOG_COLOR_GREEN   "\x1B[32m"
#define LOG_COLOR_RESET   "\x1B[0m"

#define CLEAR_SCREEN  "\033[H\033[J"

// Default log level
#ifndef DEBUG_LOG_LEVEL
    #define DEBUG_LOG_LEVEL LOG_LEVEL_DEBUG
#endif

// Default log output
#ifndef DEBUG_LOG_OUTPUT
    #define DEBUG_LOG_OUTPUT LOG_OUTPUT_UART
#endif

// Default log timestamp
#ifndef DEBUG_LOG_TIMESTAMP
    #define DEBUG_LOG_TIMESTAMP LOG_TIMESTAMP_SYS
#endif

// Default log handle
extern UART_HandleTypeDef huart3;
#ifndef DEBUG_LOG_HANDLE
    #if DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART || DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART_INLINE
        #define DEBUG_LOG_HANDLE huart3
    #elif DEBUG_LOG_OUTPUT == LOG_OUTPUT_USB_CDC
        extern USBD_HandleTypeDef hUsbDeviceFS;
        #define DEBUG_LOG_HANDLE hUsbDeviceFS
    #endif
#else
    #if DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART || DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART_INLINE
        #define LOG_PRINTF_HANDLE DEBUG_LOG_HANDLE
    #endif
#endif

#ifndef LOG_PRINTF_HANDLE
    #define LOG_PRINTF_HANDLE huart3
#endif

#if DEBUG_LOG_TIMESTAMP == LOG_TIMESTAMP_ROS
    #define TIMESTAMP _TIMESTAMP_ROS
#elif DEBUG_LOG_TIMESTAMP == LOG_TIMESTAMP_SYS
    #define TIMESTAMP _TIMESTAMP_SYS
#else
    #define TIMESTAMP 
#endif


#ifdef __cplusplus
extern "C" {
#endif

extern bool logger_init(void);
extern void logger_trace_init(void);
extern void logger_log_message(uint32_t log_level, const char* message, ...)
    __attribute__((format(printf, 2, 3)));
extern void logger_clear_terminal(void);

#ifdef __cplusplus
}
#endif

/**
 * @brief Implementation of the UART output using a dedicated logger thread for formatting and sending log messages.
 * 
 * This implementation is thread-safe, non-blocking and ensures that log messages are displayed in the correct order.
 * The logger thread has a low priority, so the log messages might be displayed with a slight delay.
 * If the log message queue is full, the log message will be discarded and a warning message will be printed.
 * The implementation of the logger thread can be found in logger.c.
 *  
 * Use LOG_OUTPUT_UART_INLINE to print log messages directly to the UART from the calling thread.
 */
#if DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART
    // If no UART handle is defined, use the default UART3 (ST-Link on Nucleo)
    #ifndef DEBUG_LOG_HANDLE
        #ifdef __cplusplus
        extern "C" {
        #endif
        extern UART_HandleTypeDef huart3;
        #ifdef __cplusplus
        }
        #endif
        #define DEBUG_LOG_HANDLE huart3
    #endif

    #define _LOG_ERROR(message, ...)   do { logger_log_message(LOG_LEVEL_ERROR,   message __VA_OPT__(,) __VA_ARGS__); } while(0)
    #define _LOG_WARNING(message, ...) do { logger_log_message(LOG_LEVEL_WARNING, message __VA_OPT__(,) __VA_ARGS__); } while(0)
    #define _LOG_INFO(message, ...)    do { logger_log_message(LOG_LEVEL_INFO,    message __VA_OPT__(,) __VA_ARGS__); } while(0)
    #define _LOG_DEBUG(message, ...)   do { logger_log_message(LOG_LEVEL_DEBUG,   message __VA_OPT__(,) __VA_ARGS__); } while(0)

    #define _LOG_SUCCESS(message, ...) do { logger_log_message(LOG_LEVEL_SUCCESS, message __VA_OPT__(,) __VA_ARGS__); } while(0)
    #define _LOG_CLEAR()               do { logger_clear_terminal(); } while(0)
    #define _LOG_INLINE(message, ...)  do { printf(LOG_COLOR_RED); printf("[%08ld]",  HAL_GetTick()); \
                                            printf(message __VA_OPT__(,) __VA_ARGS__); printf("%s\n", LOG_COLOR_RESET); } while(0)

/**
 * @brief Implementation of the UART output using inline printf.
 * 
 * This will print the log message directly to the UART without the use of an additional logger thread.
 * This implementation is not thread-safe and can block the calling thread for a short period of time (multiple ms).
 * It WILL lead to a race condition when multiple threads are trying to print log messages at the same time.
 * 
 * Only use this implementation for debugging purposes, if no log messages are received otherwise.
 * 
 */
#elif DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART_INLINE
    #ifndef DEBUG_LOG_HANDLE
        #define DEBUG_LOG_HANDLE huart3
        #ifdef __cplusplus
        extern "C" {
        #endif
        extern UART_HandleTypeDef huart3;
        #ifdef __cplusplus
        }
        #endif
    #endif

    #define _LOG(message, ...) printf(message __VA_OPT__(,) __VA_ARGS__)
    #define _TIMESTAMP_SYS printf("[%08ld]",  HAL_GetTick())
    #define _TIMESTAMP_ROS printf("[%010lu]", (uint32_t)rmw_uros_epoch_millis())

    #define _LOG_ERROR(message, ...)   do { _LOG(LOG_COLOR_RED);    TIMESTAMP; _LOG("[ERROR] "); _LOG(message, __VA_ARGS__); _LOG(LOG_COLOR_RESET "\n"); } while(0)
    #define _LOG_WARNING(message, ...) do { _LOG(LOG_COLOR_YELLOW); TIMESTAMP; _LOG("[WARN]  "); _LOG(message, __VA_ARGS__); _LOG(LOG_COLOR_RESET "\n"); } while(0)
    #define _LOG_INFO(message, ...)    do {                     TIMESTAMP; _LOG("[INFO]  "); _LOG(message, __VA_ARGS__); _LOG(            "\n"); } while(0)
    #define _LOG_DEBUG(message, ...)   do { _LOG(LOG_COLOR_BLUE);   TIMESTAMP; _LOG("[DEBUG] "); _LOG(message, __VA_ARGS__); _LOG(LOG_COLOR_RESET "\n"); } while(0)

    #define _LOG_SUCCESS(message, ...) do { _LOG(LOG_COLOR_GREEN);  TIMESTAMP; _LOG("[INFO]  "); _LOG(message, __VA_ARGS__); _LOG(LOG_COLOR_RESET "\n"); } while(0)
    #define _LOG_CLEAR()               do { _LOG(CLEAR_SCREEN LOG_COLOR_RESET); } while(0)
    #define _LOG_INLINE(message, ...)  do { _LOG_ERROR(message, __VA_ARGS__); } while(0)

/**
 * @brief Implementation of the log output using USB CDC.
 * 
 * Same as the UART implementation, but using USB CDC instead.
 */
#elif DEBUG_LOG_OUTPUT == LOG_OUTPUT_USB_CDC
    // If no USB handle is defined, use the default USB FS device if not in use
    #ifndef DEBUG_LOG_HANDLE
        #define DEBUG_LOG_HANDLE hUsbDeviceFS
        #ifdef __cplusplus
        extern "C" {
        #endif
        extern USBD_HandleTypeDef hUsbDeviceFS;
        #ifdef __cplusplus
        }
        #endif
    #endif

    #define _LOG_ERROR(message, ...)   do { logger_log_message(LOG_LEVEL_ERROR,   message __VA_OPT__(,) __VA_ARGS__); } while(0)
    #define _LOG_WARNING(message, ...) do { logger_log_message(LOG_LEVEL_WARNING, message __VA_OPT__(,) __VA_ARGS__); } while(0)
    #define _LOG_INFO(message, ...)    do { logger_log_message(LOG_LEVEL_INFO,    message __VA_OPT__(,) __VA_ARGS__); } while(0)
    #define _LOG_DEBUG(message, ...)   do { logger_log_message(LOG_LEVEL_DEBUG,   message __VA_OPT__(,) __VA_ARGS__); } while(0)

    #define _LOG_SUCCESS(message, ...) do { logger_log_message(LOG_LEVEL_SUCCESS, message __VA_OPT__(,) __VA_ARGS__); } while(0)
    #define _LOG_CLEAR()               do { logger_clear_terminal(); } while(0)
    #define _LOG_INLINE(message, ...)  do { printf(LOG_COLOR_RED); printf("[%08ld]",  HAL_GetTick()); \
                                            printf(message __VA_OPT__(,) __VA_ARGS__); printf("%s\n", LOG_COLOR_RESET); } while(0)


/**
 * @brief Implementation of the log output using Percepio Tracealyzer.
 * 
 * This implementation uses the trace recorder API to log messages to the Tracealyzer event buffer.
 * The log messages will be displayed in the Tracealyzer event viewer.
 * This is by far the most efficient imlpementation and offers a sub-microsecond precision for timestamps.
 */
#elif DEBUG_LOG_OUTPUT == LOG_OUTPUT_TRACE
    extern TraceStringHandle_t logger_error_channel;
    extern TraceStringHandle_t logger_warning_channel;
    extern TraceStringHandle_t logger_info_channel;
    extern TraceStringHandle_t logger_debug_channel;
    extern void logger_trace_init(void);

    #define _TIMESTAMP_SYS // Timestamp will be added by Tracealyzer
    #define _TIMESTAMP_ROS // Timestamp will be added by Tracealyzer

    #define _LOG_ERROR(message, ...)   do { xTracePrintF(logger_error_channel, message __VA_OPT__(,) __VA_ARGS__); } while(0)
    #define _LOG_WARNING(message, ...) do { xTracePrintF(logger_warning_channel, message __VA_OPT__(,) __VA_ARGS__); } while(0)
    #define _LOG_INFO(message, ...)    do { xTracePrintF(logger_info_channel, message __VA_OPT__(,) __VA_ARGS__); } while(0)
    #define _LOG_DEBUG(message, ...)   do { xTracePrintF(logger_debug_channel, message __VA_OPT__(,) __VA_ARGS__); } while(0)

    #define _LOG_SUCCESS(message, ...) do { xTracePrintF(logger_info_channel, message __VA_OPT__(,) __VA_ARGS__); } while(0)
    #define _LOG_CLEAR()
    #define _LOG_INLINE(message, ...)  do { xTracePrintF(logger_error_channel, message __VA_OPT__(,) __VA_ARGS__); } while(0)

#else
    #define _LOG_ERROR(message, ...)
    #define _LOG_WARNING(message, ...)
    #define _LOG_INFO(message, ...)
    #define _LOG_DEBUG(message, ...)
    #define _LOG_SUCCESS(message, ...)
    #define _LOG_CLEAR()
    #define _LOG_INLINE(message, ...)
#endif


// Depending on the log level, the log messages will be displayed or ignored
// This will be evaluated at compile time for performance reasons
#if DEBUG_LOG_LEVEL >= LOG_LEVEL_DEBUG 
    #define LogError(message, ...)   do { _LOG_ERROR(message, __VA_ARGS__); } while(0)
    #define LogWarning(message, ...) do { _LOG_WARNING(message, __VA_ARGS__); } while(0)
    #define LogInfo(message, ...)    do { _LOG_INFO(message, __VA_ARGS__); } while(0)
    #define LogDebug(message, ...)   do { _LOG_DEBUG(message, __VA_ARGS__); } while(0)
#elif DEBUG_LOG_LEVEL >= LOG_LEVEL_INFO
    #define LogError(message, ...)   do { _LOG_ERROR(message, __VA_ARGS__); } while(0)
    #define LogWarning(message, ...) do { _LOG_WARNING(message, __VA_ARGS__); } while(0)
    #define LogInfo(message, ...)    do { _LOG_INFO(message, __VA_ARGS__); } while(0)
    #define LogDebug(message, ...)
#elif DEBUG_LOG_LEVEL >= LOG_LEVEL_WARNING
    #define LogError(message, ...)   do { _LOG_ERROR(message, __VA_ARGS__); } while(0)
    #define LogWarning(message, ...) do { _LOG_WARNING(message, __VA_ARGS__); } while(0)
    #define LogInfo(message, ...)
    #define LogDebug(message, ...)
#elif DEBUG_LOG_LEVEL >= LOG_LEVEL_ERROR
    #define LogError(message, ...)   do { _LOG_ERROR(message, __VA_ARGS__); } while(0)
    #define LogWarning(message, ...)
    #define LogInfo(message, ...)
    #define LogDebug(message, ...)
#else
    #define LogError(message, ...)
    #define LogWarning(message, ...)
    #define LogInfo(message, ...)
    #define LogDebug(message, ...)
#endif

// Success messages with additional highlighting (INFO verbosity)
#if DEBUG_LOG_LEVEL >= LOG_LEVEL_INFO
    #define LogSuccess( message, ... )   do { _LOG_SUCCESS( message, __VA_ARGS__ ); } while(0)
#else
    #define LogSuccess( message, ... )
#endif

// LogClear() can be used to clear the terminal screen on supported output devices
#if DEBUG_LOG_LEVEL >= LOG_LEVEL_ERROR
    #define LogClear() _LOG_CLEAR()
#else
    #define LogClear()
#endif

// LogInline() can be used to print a log message directly, without the use of the logger thread
// This might be neccessary for debugging purposes, or in cases where the logger thread is not running (e.g. error handling)
// It will have ERROR verbosity by default. Not recommended for regular use.
#if DEBUG_LOG_LEVEL >= LOG_LEVEL_ERROR
    #define LogInline(message, ...) _LOG_INLINE(message, __VA_ARGS__)
#else
    #define LogInline(message, ...)
#endif
