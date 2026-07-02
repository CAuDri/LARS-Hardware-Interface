/**
 * @file debug_config.h
 *
 * @brief CAuDri - Debug configuration file for the Hardware Interface
 *
 * This file is used to set up all debug-related configuration parameters for the application.
 */

#pragma once

/* ------ Logger Configuration ------ */

#ifndef DEBUG_LOG_LEVEL
#define DEBUG_LOG_LEVEL LOG_LEVEL_DEBUG  // Set the minimum compiled log level
#endif

#ifndef DEBUG_LOG_OUTPUT
#define DEBUG_LOG_OUTPUT LOG_OUTPUT_USB_CDC  // Select the logging backend
#endif

#ifndef DEBUG_LOG_HANDLE
#define DEBUG_LOG_HANDLE hUsbDeviceHS  // Backend handle (e.g. huartX or hUsbDeviceX)
#endif

#ifndef DEBUG_LOG_TIMESTAMP
#define DEBUG_LOG_TIMESTAMP LOG_TIMESTAMP_SYS  // Select the timestamp source
#endif

/* ------ Trace Recorder Configuration ------ */

// #define DEBUG_USE_TRACE_RECORDER                // Uncomment to enable trace recording
#define DEBUG_WAIT_FOR_TRACEALYZER              // Uncomment to wait for Tracealyzer to start recording
