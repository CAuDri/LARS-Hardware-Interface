# CAuDri - Top-level CMake Configuration for the Hardware Interface
#
# Will be used by the build script (build and flash commands) by default.
# If you need to run CMake manually, configure like:
#   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=board/cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release

cmake_minimum_required(VERSION 3.22)

project(HardwareInterface LANGUAGES C CXX ASM)

# Apply the board selection to every target created below, including auxiliary
# libraries such as TraceRecorder that include STM32 device headers directly.
add_compile_definitions(
    USE_HAL_DRIVER
    STM32F429xx
    # micro-XRCE-DDS exposes FreeRTOS mutex fields in public structs when this
    # definition is enabled. The firmware target must match the static
    # micro-ROS library build or custom transport callbacks read the wrong
    # uxrCustomTransport layout.
    PLATFORM_NAME_FREERTOS
)

add_executable(${PROJECT_NAME})

# Include the common CMake configuration for all devices
include("../../toolchain/firmware_common.cmake")

# Include the CubeMX-generated CMake configuration
add_subdirectory("board/cmake/stm32cubemx" "${CMAKE_BINARY_DIR}/stm32cubemx")

# The generated Get_SerialNum() currently uses uninitialized locals. Our stable
# callback replaces it before USB starts, so suppress warnings only for this
# generated translation unit without modifying code that CubeMX will overwrite.
caudri_suppress_generated_source_warnings(
    ${PROJECT_NAME}
    "${CMAKE_CURRENT_LIST_DIR}/board/USB_DEVICE/App/usbd_desc.c"
    -Wno-uninitialized
    -Wno-maybe-uninitialized
)

# ---------------- Device-specific application ----------------

target_sources(${PROJECT_NAME} PRIVATE
    main.cpp

    # FreeRTOS heap
    ${LIB_DIR}/common/custom_heap_4.c

    # Common helpers
    ${LIB_DIR}/common/global_symbols.cpp
    ${LIB_DIR}/common/logger.c
    ${LIB_DIR}/common/thread_safe_can.c
    ${LIB_DIR}/common/thread_safe_i2c.c
    ${LIB_DIR}/common/thread_safe_adc.c
    ${LIB_DIR}/common/usb_device_helper.c

    # Drivers
    ${LIB_DIR}/drivers/rc_receiver.cpp
    ${LIB_DIR}/drivers/vesc.cpp
    ${LIB_DIR}/drivers/servo.cpp
    ${LIB_DIR}/drivers/ws2812.cpp

    # Light implementations and helpers
    ${LIB_DIR}/lights/gpio_light.cpp
    ${LIB_DIR}/lights/light_dispatcher.cpp
    ${LIB_DIR}/lights/animations/animation.cpp

    # ROS application nodes
    ${LIB_DIR}/nodes/autonomous_control.cpp
    ${LIB_DIR}/nodes/logger_node.cpp
    ${LIB_DIR}/nodes/motor_publisher.cpp
    ${LIB_DIR}/nodes/servo_publisher.cpp
    ${LIB_DIR}/nodes/system_node.cpp

    # micro-ROS wrapper
    ${LIB_DIR}/microros/cpp_wrapper/client.cpp
    ${LIB_DIR}/microros/cpp_wrapper/executor.cpp
    ${LIB_DIR}/microros/cpp_wrapper/microros_trace.cpp
    ${LIB_DIR}/microros/cpp_wrapper/node.cpp
    ${LIB_DIR}/microros/cpp_wrapper/publisher.cpp
    ${LIB_DIR}/microros/cpp_wrapper/diagnostics.cpp
    ${LIB_DIR}/microros/cpp_wrapper/service.cpp
    ${LIB_DIR}/microros/cpp_wrapper/subscriber.cpp
    ${LIB_DIR}/microros/runtime/microros_allocator.c
    ${LIB_DIR}/microros/runtime/microros_time.c
    ${LIB_DIR}/microros/transports/usb_cdc_transport.c

    # Misc
    ${LIB_DIR}/misc/system_check.cpp
    ${LIB_DIR}/misc/system_monitor.cpp
    ${LIB_DIR}/misc/drive_controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    config
    microros_lib/inc

    ${LIB_DIR}/common/include
    ${LIB_DIR}/drivers/include
    ${LIB_DIR}/drivers/interfaces
    ${LIB_DIR}/lights/include
    ${LIB_DIR}/lights/animations/include
    ${LIB_DIR}/nodes/include
    ${LIB_DIR}/microros/cpp_wrapper/include
    ${LIB_DIR}/microros/runtime/include
    ${LIB_DIR}/microros/transports/include
    ${LIB_DIR}/misc/include
)

# ---------------- TraceRecorder ----------------

# If you need a specific stream port, set it before the add_subdirectory call, e.g.:
set(TRC_STREAMPORT "STM32_USB_CDC")
set(BOARD_DIR "${CMAKE_CURRENT_LIST_DIR}/board")
set(TRACE_RECORDER_APP_CONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/config")
add_subdirectory("${LIB_DIR}/trace_recorder" "${CMAKE_BINARY_DIR}/trace_recorder")

# Pull object files into firmware and enable the include guard in FreeRTOSConfig.h
target_sources(${PROJECT_NAME} PRIVATE $<TARGET_OBJECTS:TraceRecorder>)
target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_TRACERECORDER=1)

# ---------------- Linking ----------------

# Ensure the CubeMX/FreeRTOS target compiles with TraceRecorder usage
target_link_libraries(stm32cubemx INTERFACE TraceRecorder)

# Link the application with the necessary libraries
target_link_libraries(${PROJECT_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/microros_lib/libmicroros.a
    stm32cubemx
    TraceRecorder
)
