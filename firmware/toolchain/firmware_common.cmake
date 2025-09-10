# CAuDri - Common CMake configuration for all devices
# Offers common language settings and helper functions. Should be included from the top-level CMakeLists.txt of each device.

enable_language(C CXX ASM)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type (Debug/Release). Default to Debug if not provided.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# FIRMWARE_DIR: absolute path to firmware/
# LIB_DIR:      absolute path to firmware/libraries/
get_filename_component(FIRMWARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(LIB_DIR "${FIRMWARE_DIR}/libraries")

target_link_options(${PROJECT_NAME} PRIVATE
  --specs=nosys.specs -u _printf_float
)

# Post-build artifacts (hex/bin/size)
if(NOT CMAKE_OBJCOPY)
  set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
endif()
if(NOT CMAKE_SIZE)
  set(CMAKE_SIZE arm-none-eabi-size)
endif()

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
)

# Keep linker MAP in 'clean' (assumes toolchain adds -Wl,-Map=...)
set_target_properties(${PROJECT_NAME} PROPERTIES
  ADDITIONAL_CLEAN_FILES "${PROJECT_NAME}.map"
)

# Workaround: Remove bogus ob lib when using C++ (recommended by CubeMX)
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
