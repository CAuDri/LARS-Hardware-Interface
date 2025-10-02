# CAuDri - Top-level CMake Configuration for the Hardware Interface
#
# Will be used by the build script (build and flash commands) by default.
# If you need to run CMake manually, configure like:
#   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=board/cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release

cmake_minimum_required(VERSION 3.22)

project(HardwareInterface LANGUAGES C CXX ASM)

add_executable(${PROJECT_NAME})

# Include the common CMake configuration for all devices
include("../../toolchain/firmware_common.cmake")

# Include the CubeMX-generated CMake configuration
add_subdirectory("board/cmake/stm32cubemx" "${CMAKE_BINARY_DIR}/stm32cubemx")

# ---------------- Device specific includes ----------------

# --- Application sources ---
target_sources(${PROJECT_NAME} PRIVATE
    main.cpp

    # Common source files
    # "${LIB_DIR}/common/logger.c"

    # FreeRTOS source files
    "board/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c"
)

# --- Application includes ---
target_include_directories(${PROJECT_NAME} PRIVATE
    "config"
    "${LIB_DIR}/common/include"
)

# ---------------- TraceRecorder Library ----------------
# If you need a specific stream port, set it before the add_subdirectory, e.g.:
set(TRC_STREAMPORT "STM32_USB_CDC")
set(BOARD_DIR "${CMAKE_CURRENT_LIST_DIR}/board")
add_subdirectory("${LIB_DIR}/trace_recorder" "${CMAKE_BINARY_DIR}/trace_recorder")

# Pull object files into firmware and enable the include guard in FreeRTOSConfig.h
target_sources(${PROJECT_NAME} PRIVATE $<TARGET_OBJECTS:TraceRecorder>)
target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_TRACERECORDER=1)

# ---------------- Linking ----------------
target_link_libraries(${PROJECT_NAME}
    ${TOOLCHAIN_LINK_LIBRARIES}
    ${CMAKE_SOURCE_DIR}/microros_lib/libmicroros.a
    stm32cubemx
    TraceRecorder
)
