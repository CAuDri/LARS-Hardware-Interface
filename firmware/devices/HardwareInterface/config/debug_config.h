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

#define DEBUG_LOG_LEVEL LOG_LEVEL_INFO         // Set the log level
#define DEBUG_LOG_OUTPUT LOG_OUTPUT_UART       // Set the log output
#define DEBUG_LOG_HANDLE huart3                // Set the handle for the log output (e.g., huartX)
#define DEBUG_LOG_TIMESTAMP LOG_TIMESTAMP_SYS  // Set the log timestamp

/* ------ Trace Recorder Configuration ------ */

// #define DEBUG_USE_TRACE_RECORDER  // Enable Tracealyzer trace recording
// #define DEBUG_WAIT_FOR_TRACEALYZER  // With recording enabled, wait for Tracealyzer to connect

#define DEBUG_TRACE_RECORDER_USB_HANDLE hUsbDebugPort

// Uncomment to enable specific state machine tracing
// #define DEBUG_TRACE_LOG_NODE_STATE
// #define DEBUG_TRACE_LOG_DRIVER_STATE
// #define DEBUG_TRACE_LOG_DRIVER_CONNECTION

#define DEBUG_TRACE_BUFFER_SIZE 40000 // Tracealyzer stream buffer size in bytes
#define DEBUG_TRACE_ENTRY_SLOTS 100 // Tracealyzer symbol table entry slots
