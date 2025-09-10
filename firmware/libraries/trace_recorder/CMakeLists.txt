# CAuDri - CMake configuration for Percepio TraceRecorder

cmake_minimum_required(VERSION 3.16)

# Toggle Tracealyzer on/off from the parent project
option(ENABLE_TRACERECORDER "Build Percepio TraceRecorder" ON)

# Path to FreeRTOS (override from parent if different)
set(FREERTOS_ROOT
  "${CMAKE_SOURCE_DIR}/board/Middlewares/Third_Party/FreeRTOS"
  CACHE PATH "Path to FreeRTOS root"
)

# Path to TraceRecorder config (can be overridden from parent)
set(TRACE_RECORDER_CONFIG_DIR
  "${CMAKE_CURRENT_LIST_DIR}/config"
  CACHE PATH "Directory that contains trcConfig.h"
)

# Select a stream port (subdirectory in streamports/)
set(TRC_STREAMPORT "STM32_USB_CDC")


# If disabled, provide a no-op target so parent code can still depend on it.
if(NOT ENABLE_TRACERECORDER)
  add_library(TraceRecorder INTERFACE)
  target_include_directories(TraceRecorder INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}/include" "${TRACE_RECORDER_CONFIG_DIR}"
  )
  return()
endif()

# Core TraceRecorder sources
set(TRC_SRC
  ${CMAKE_CURRENT_LIST_DIR}/kernelports/FreeRTOS/trcKernelPort.c
  #   ${CMAKE_CURRENT_LIST_DIR}/hardwareports/ARM_Cortex_M/trcHardwarePort.c

  trcAssert.c
  trcCounter.c
  trcDependency.c
  trcDiagnostics.c
  trcEntryTable.c
  trcError.c
  trcEvent.c
  trcEventBuffer.c
  trcExtension.c
  trcHardwarePort.c
  trcHeap.c
  trcISR.c
  trcInternalEventBuffer.c
  trcInterval.c
  trcMultiCoreEventBuffer.c
  trcObject.c
  trcPrint.c
  trcRunnable.c
  trcSnapshotRecorder.c
  trcStackMonitor.c
  trcStateMachine.c
  trcStaticBuffer.c
  trcStreamingRecorder.c
  trcString.c
  trcTask.c
  trcTimestamp.c
)

# Trace Recorder inlude directories
set(TRC_INCLUDES
  "${CMAKE_CURRENT_LIST_DIR}/include"
  "${TRACE_RECORDER_CONFIG_DIR}"
  "${CMAKE_CURRENT_LIST_DIR}/kernelports/FreeRTOS/include"
  "${CMAKE_CURRENT_LIST_DIR}/kernelports/FreeRTOS/config"

  "${FREERTOS_ROOT}/Source/include"
  "${FREERTOS_ROOT}/Source/portable/GCC/ARM_CM4F"
  "${FREERTOS_ROOT}/Source/CMSIS_RTOS_V2"

  "${BOARD_DIR}/Core/Inc" # FreeRTOSConfig.h
  "${BOARD_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include/" # stm32f4xx.h
  "${BOARD_DIR}/Drivers/CMSIS/Include" # core_cm4.h
)

# Stream port sources (based on TRC_STREAMPORT)
if(DEFINED TRC_STREAMPORT AND TRC_STREAMPORT STREQUAL "SEGGER_RTT")
  list(APPEND TRC_SRC
    "${CMAKE_CURRENT_LIST_DIR}/streamports/SEGGER_RTT/trcStreamPort.c"
    "${CMAKE_CURRENT_LIST_DIR}/streamports/SEGGER_RTT/SEGGER_RTT.c"
  )
elseif(DEFINED TRC_STREAMPORT AND TRC_STREAMPORT STREQUAL "STM32_USB_CDC")
  list(APPEND TRC_SRC
    "${CMAKE_CURRENT_LIST_DIR}/streamports/STM32_USB_CDC/trcStreamPort.c"
  )
  list(APPEND TRC_INCLUDES
    "${CMAKE_CURRENT_LIST_DIR}/streamports/STM32_USB_CDC/config"
    "${CMAKE_CURRENT_LIST_DIR}/streamports/STM32_USB_CDC/include"
    "${BOARD_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc"
    "${BOARD_DIR}/USB_DEVICE/App"
    "${BOARD_DIR}/USB_DEVICE/Target"
    "${BOARD_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc"
    "${BOARD_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
  )
else()
  message(FATAL_ERROR "Unknown TRC_STREAMPORT: ${TRC_STREAMPORT}")
endif()

# Build as OBJECT library to allow inclusion in other targets
add_library(TraceRecorder OBJECT ${TRC_SRC})

# Public includes: vendor + config + FreeRTOS
target_include_directories(TraceRecorder PUBLIC ${TRC_INCLUDES}
)

# Keep vendor code quiet (optional)
target_compile_options(TraceRecorder PRIVATE
  -Wno-unused-parameter -Wno-missing-field-initializers
)

# Recommended optimization flags for smaller code size
target_compile_options(TraceRecorder PUBLIC -ffunction-sections -fdata-sections)
