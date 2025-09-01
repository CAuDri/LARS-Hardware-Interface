######################################################################################
##                                                                                  ##
##  CAuDri - CMake configuration for building the static micro-ROS library          ##
##                                                                                  ##  
##  This file is used in conjunction with the device specific colcon.meta in        ##
##  the generate_lib.sh script.                                                     ##
##  The library will be built seperately for each device and copied to the          ##
##  corresponding device directory.                                                 ##
##                                                                                  ##
##  Thread safety for the micro-XRCE-DDS client is based on the FreeRTOS kernel.    ##
##  Thus it requires a reference to the FreeRTOS headers and                        ##
##  device specific configuration (FreeRTOSConfig.h).                               ##
##                                                                                  ##                        
######################################################################################

SET(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

## Set the device path exported in generate_lib.sh
set(DEVICE_PATH $ENV{DEVICE_PATH})

## Include the auto-generated toolchain file for retrieving compiler flags
include("${DEVICE_PATH}/board/cmake/gcc-arm-none-eabi.cmake")

# set(MCU_TARGET   "${TARGET_FLAGS}")
set(MCU_C_FLAGS   "${CMAKE_C_FLAGS}")
# set(MCU_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
# set(MCU_ASM_FLAGS "${CMAKE_ASM_FLAGS}")
# set(MCU_TOOLCHAIN_LIBS "${TOOLCHAIN_LINK_LIBRARIES}")

## Clear auto-generated linker flags to not leak into the static lib build
set(CMAKE_EXE_LINKER_FLAGS    "")
set(CMAKE_SHARED_LINKER_FLAGS "")
set(CMAKE_MODULE_LINKER_FLAGS "")

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fno-threadsafe-statics")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -x assembler-with-cpp -MMD -MP")

## Include device specific FreeRTOS headers
include_directories(${DEVICE_PATH}/board/Core/Inc)  # FreeRTOSConfig.h
include_directories(${DEVICE_PATH}/board/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F) # portmacro.h
include_directories(${DEVICE_PATH}/board/Middlewares/Third_Party/FreeRTOS/Source/include) # FreeRTOS.h

## Micro-ROS configuration
add_compile_definitions(PLATFORM_NAME_FREERTOS
                        CLOCK_MONOTONIC=0)
                        
## When including device specific headers (like FreeRTOSConfig.h) in the build process, this flag will be used to identify the build as a micro-ROS build.
## This is to avoid including device specific headers in the micro-ROS client library which could lead to conflicts with the application code.
add_compile_definitions(MICRO_ROS_BUILD)

set(__BIG_ENDIAN__ 0)
