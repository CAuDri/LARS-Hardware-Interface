/**
 * @file logger.h
 * @brief Compile-time filtered firmware logging interface.
 *
 * Calls below DEBUG_LOG_LEVEL, or all calls when DEBUG_LOG_OUTPUT is
 * LOG_OUTPUT_NONE, compile to no-ops. Their arguments are not evaluated and
 * their format strings are not referenced by the resulting binary.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Temporary compatibility includes for existing consumers. These are removed
 * in the follow-up dependency-cleanup commit once consumers include what they
 * use directly.
 */
#include <cmsis_os.h>

#include "logger_format.h"
#include "main.h"

/* Severity values used by DEBUG_LOG_LEVEL. */
#define LOG_LEVEL_NONE 0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARNING 2
#define LOG_LEVEL_INFO 3
#define LOG_LEVEL_DEBUG 4

/* Backend values used by DEBUG_LOG_OUTPUT. */
#define LOG_OUTPUT_NONE 0
#define LOG_OUTPUT_UART 1
#define LOG_OUTPUT_USB_CDC 2
#define LOG_OUTPUT_TRACE 3
#define LOG_OUTPUT_ROS 4
#define LOG_OUTPUT_UART_INLINE 5

/* Timestamp values used by DEBUG_LOG_TIMESTAMP. */
#define LOG_TIMESTAMP_NONE 0
#define LOG_TIMESTAMP_ROS 1
#define LOG_TIMESTAMP_SYS 2

/* Device-specific DEBUG_LOG_* definitions. */
#include "debug_config.h"

#ifndef DEBUG_LOG_LEVEL
#define DEBUG_LOG_LEVEL LOG_LEVEL_DEBUG
#endif

#ifndef DEBUG_LOG_OUTPUT
#define DEBUG_LOG_OUTPUT LOG_OUTPUT_UART
#endif

#ifndef DEBUG_LOG_TIMESTAMP
#define DEBUG_LOG_TIMESTAMP LOG_TIMESTAMP_SYS
#endif

#ifdef __cplusplus
extern "C" {
#endif

bool logger_init(void);
void logger_trace_init(void);

void logger_log_message(uint32_t level, bool success, const char* format, ...)
    __attribute__((format(printf, 3, 4)));

void logger_log_inline(uint32_t level, bool success, const char* format, ...)
    __attribute__((format(printf, 3, 4)));

void logger_clear_terminal(void);

#ifdef __cplusplus
}
#endif

/* ROS output is reserved but not implemented yet. */
#if DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART || DEBUG_LOG_OUTPUT == LOG_OUTPUT_UART_INLINE || \
    DEBUG_LOG_OUTPUT == LOG_OUTPUT_USB_CDC || DEBUG_LOG_OUTPUT == LOG_OUTPUT_TRACE
#define LOGGER_BACKEND_ENABLED 1
#else
#define LOGGER_BACKEND_ENABLED 0
#endif

#if LOGGER_BACKEND_ENABLED && DEBUG_LOG_LEVEL >= LOG_LEVEL_ERROR
#define LogError(...) \
    do { logger_log_message(LOG_LEVEL_ERROR, false, __VA_ARGS__); } while (0)
#define LogInline(...) \
    do { logger_log_inline(LOG_LEVEL_ERROR, false, __VA_ARGS__); } while (0)
#define LogClear() \
    do { logger_clear_terminal(); } while (0)
#else
#define LogError(...) ((void)0)
#define LogInline(...) ((void)0)
#define LogClear() ((void)0)
#endif

#if LOGGER_BACKEND_ENABLED && DEBUG_LOG_LEVEL >= LOG_LEVEL_WARNING
#define LogWarning(...) \
    do { logger_log_message(LOG_LEVEL_WARNING, false, __VA_ARGS__); } while (0)
#else
#define LogWarning(...) ((void)0)
#endif

#if LOGGER_BACKEND_ENABLED && DEBUG_LOG_LEVEL >= LOG_LEVEL_INFO
#define LogInfo(...) \
    do { logger_log_message(LOG_LEVEL_INFO, false, __VA_ARGS__); } while (0)
#define LogSuccess(...) \
    do { logger_log_message(LOG_LEVEL_INFO, true, __VA_ARGS__); } while (0)
#define LogEmpty() \
    do { logger_log_message(LOG_LEVEL_INFO, false, ""); } while (0)
#else
#define LogInfo(...) ((void)0)
#define LogSuccess(...) ((void)0)
#define LogEmpty() ((void)0)
#endif

#if LOGGER_BACKEND_ENABLED && DEBUG_LOG_LEVEL >= LOG_LEVEL_DEBUG
#define LogDebug(...) \
    do { logger_log_message(LOG_LEVEL_DEBUG, false, __VA_ARGS__); } while (0)
#else
#define LogDebug(...) ((void)0)
#endif
