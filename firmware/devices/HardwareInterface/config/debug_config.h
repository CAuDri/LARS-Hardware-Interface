/**
 * @file debug_config.h
 *
 * @brief CAuDri - Debug configuration file for the Hardware Interface
 *
 * This file is used to set up all debug-related configuration parameters for the application.
 */

#pragma once

#include "main.h"

/* ------ Logger Configuration ------ */

#define DEBUG_LOG_LEVEL LOG_LEVEL_DEBUG        // Set the log level
#define DEBUG_LOG_OUTPUT LOG_OUTPUT_UART       // Set the log output
#define DEBUG_LOG_HANDLE huart3                // Set the handle for the log output (e.g., huartX)
#define DEBUG_LOG_TIMESTAMP LOG_TIMESTAMP_SYS  // Set the log timestamp

/* ------ Trace Recorder Configuration ------ */

// #define DEBUG_USE_TRACE_RECORDER  // Enable Tracealyzer trace recording
#define DEBUG_WAIT_FOR_TRACEALYZER  // Halt the firmware and wait for Tracealyzer to connect
#define DEBUG_TRACE_RECORDER_USB_HANDLE hUsbDebugPort
