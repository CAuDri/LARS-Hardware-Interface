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
#define DEBUG_LOG_OUTPUT LOG_OUTPUT_TRACE       // Set the log output
#define DEBUG_LOG_HANDLE huart3                // Set the handle for the log output (e.g., huartX)
#define DEBUG_LOG_TIMESTAMP LOG_TIMESTAMP_SYS  // Set the log timestamp

/* ------ Trace Recorder Configuration ------ */

#define DEBUG_USE_TRACE_RECORDER  // Uncomment to enable Tracealyzer trace recording
#define DEBUG_WAIT_FOR_TRACEALYZER  // Halt the firmware and wait for Tracealyzer to connect
#define DEBUG_TRACE_RECORDER_USB_HANDLE hUsbDebugPort

// Uncomment to enable specific state machine tracing
// #define DEBUG_TRACE_LOG_NODE_STATE
// #define DEBUG_TRACE_LOG_DRIVER_STATE
// #define DEBUG_TRACE_LOG_DRIVER_CONNECTION

// Streaming buffer in bytes. Increase if Tracealyzer reports dropped events.
#define DEBUG_TRACE_BUFFER_SIZE 16384

// Number of symbol table entries for Tracealyzer (Threads, State Machines, Events, etc.)
// Increase if Tracealyzer does not show thread names correctly (uses lots of RAM)
#define DEBUG_TRACE_ENTRY_SLOTS 80
