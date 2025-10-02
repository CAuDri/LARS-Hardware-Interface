#!/bin/bash

# CAuDri - Bash script for building and flashing code to the Hardware Interface and other devices.
# It is a simple wrapper around CMake and OpenOCD and will detect all configured targets/devices.
# In the Dev Container it will be sourced by default and provide the following shell commands:
#
#   build [--clean] [--all] <DeviceName>
#       Build the firmware for a specific device.
#       Options:
#         -c, --clean   Clean build artifacts before building.
#         -a, --all     Clean all build artifacts including the micro-ROS library.
#
#   flash [--clean] [--all] <DeviceName>
#       Build and flash the firmware to a specific device.
#       The device must be connected via ST-Link.
#       Options:
#         -c, --clean   Clean build artifacts before flashing.
#         -a, --all     Clean all build artifacts including the micro-ROS library.
#
#   clean [--all] [--global] <DeviceName>
#       Clean build artifacts for a specific device.
#       Options:
#         -a, --all     Clean all build artifacts including the micro-ROS library.
#         -g, --global  Clean all build artifacts for all devices and the micro-ROS workspace itself.
#
# The device name must correspond to the directory name in the 'devices' folder.
# Use any of the above commands without arguments for help.

# Colors for terminal output
COLOR_RED='\033[0;31m'
COLOR_GREEN='\033[0;32m'
COLOR_RESET='\033[0m'

# Global Variables
BASE_DIR=$(git rev-parse --show-toplevel 2>/dev/null)
TOOLCHAIN_FILE="${BASE_DIR}/firmware/toolchain/toolchain.cmake"
OPENOCD_CFG="${BASE_DIR}/firmware/toolchain/openocd.cfg"

# Initialize device-specific variables
_init_vars() {
    DEVICE_NAME=$1
    DEVICE_DIR="${BASE_DIR}/firmware/devices/${DEVICE_NAME}"
    BUILD_DIR="${DEVICE_DIR}/build"
}

_list_devices() {
    echo -e "Known devices are:"
    ls "${BASE_DIR}/firmware/devices/"
    echo ""
}

_clean_build_artifacts() {
    echo "Cleaning build artifacts for ${DEVICE_NAME}"
    rm -rf "${BUILD_DIR}"
}

_clean_microros_lib() {
    local MICROROS_LIB_DIR="${DEVICE_DIR}/microros/lib"
    echo "Cleaning micro-ROS library for ${DEVICE_NAME}"
    rm -rf "${MICROROS_LIB_DIR}"
}

_clean_all_devices() {
    echo "Cleaning all devices and micro-ROS firmware"
    find "${BASE_DIR}/firmware/devices/" -type d -name "build" -exec rm -rf {} +
    find "${BASE_DIR}/firmware/devices/" -type d -name "lib" -exec rm -rf {} +
    rm -rf "${BASE_DIR}/firmware/libraries/microros/firmware"
}

_show_build_help() {
    echo "Usage: build [--clean] [--all] <DeviceName>"
    echo "Build the Firmware for a specific device"
    echo "Options:"
    echo "  -c, --clean: Clean build artifacts before building"
    echo "  -a, --all: Clean all build artifacts including the micro-ROS library"
}

_show_flash_help() {
    echo "Usage: flash [--clean] [--all] <DeviceName>"
    echo "Flash the Firmware to a specific device"
    echo "The device must be connected to the computer via a ST-Link"
    echo "Options:"
    echo "  -c, --clean: Clean build artifacts before flashing"
    echo "  -a, --all: Clean all build artifacts including the micro-ROS library"
}

_show_clean_help() {
    echo "Usage: clean [--all] [--global] <DeviceName>"
    echo "Clean build artifacts for a specific device"
    echo "Options:"
    echo "  -a, --all: Clean all build artifacts including the micro-ROS library"
    echo "  -g, --global: Clean all build artifacts for all devices and the micro-ROS workspace itself"
}

# Build micro-ROS library for a specific device
# This will run the microros_generate_lib.sh script for the specified device
# The micro-ROS library will be copied to the microros/lib directory in the device folder
_build_microros() {
    echo "Building micro-ROS library for ${DEVICE_NAME}"
    pushd "${BASE_DIR}/firmware/libraries/microros" >/dev/null
    bash microros_generate_lib.sh "${DEVICE_NAME}"
    if [ $? -ne 0 ]; then
        echo "Failed to build micro-ROS library for ${DEVICE_NAME}"
        popd >/dev/null
        return 1
    fi
    echo "Micro-ROS library built successfully for ${DEVICE_NAME}"
    popd >/dev/null
}

# Wrapper function for building a specific target
# This will create a build directory in the device folder and run cmake to build the firmware.
# If the micro-ROS library has not yet been created for the the device, it will be built first.
#
# Usage: build [--clean] [--all] <DeviceName>
# Options:
#   -c, --clean: Clean build artifacts before building
#   -a, --all: Clean all build artifacts including the micro-ROS library
#
build() {
    local CLEAN=false
    local CLEAN_ALL=false

    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
        -c | --clean)
            CLEAN=true
            shift
            ;;
        -a | --all)
            CLEAN=true
            CLEAN_ALL=true
            shift
            ;;
        -*)
            echo "Unknown option: $1\n"
            _show_build_help
            return 1
            ;;
        *)
            DEVICE_NAME="$1"
            shift
            ;;
        esac
    done

    echo "Building device: ${DEVICE_NAME}"

    # Check if a device name was provided
    if [ -z "$DEVICE_NAME" ]; then
        _show_build_help
        return 1
    fi

    _init_vars "$DEVICE_NAME"

    # Check if the device directory exists
    if [ ! -d "${DEVICE_DIR}" ]; then
        echo -e "\n${COLOR_RED}Error: Device directory not found: ${DEVICE_DIR}${COLOR_RESET}\n"
        _list_devices
        return 1
    fi

    # Clean build artifacts if requested
    if $CLEAN; then
        _clean_build_artifacts
        if $CLEAN_ALL; then
            _clean_microros_lib
        fi
    fi

    # Build microros library if it doesn't exist
    if [ ! -d "${DEVICE_DIR}/microros/lib" ]; then
        echo
        _build_microros "$1"
        if [ $? -ne 0 ]; then
            return 1
        fi
    fi

    echo "Building device: ${DEVICE_NAME}"
    mkdir -p "${BUILD_DIR}"
    pushd "${BUILD_DIR}" >/dev/null || return 1

    # --- THIS IS THE ACTUAL BUILD STEP ---
    cmake -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" ..
    cmake --build . -- -j$(nproc)

    if [ $? -ne 0 ]; then
        echo -e "\n${COLOR_RED}CMake build failed! Check error log for information.${COLOR_RESET}\n"
        popd >/dev/null
        return 1
    fi

    # Print the path to the build directory
    echo -e "\n${COLOR_GREEN}Build finished sucessfully!${COLOR_RESET}\nExecutable can be found at: ${BUILD_DIR}/${DEVICE_NAME}.elf\n"
    popd >/dev/null
}

# Flash the firmware to a specific device
# This will build the firmware first and then flash it to a connected device using openocd.
# The device must be connected via a ST-Link. OpenOCD will automatically detect connected devices.
#
# Usage: flash [--clean] [--all] <DeviceName>
# Options:
#   -c, --clean: Clean build artifacts before flashing
#   -a, --all: Clean all build artifacts including the micro-ROS library
#
flash() {
    local CLEAN=false
    local CLEAN_ALL=false

    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
        -c | --clean)
            CLEAN=true
            shift
            ;;
        -a | --all)
            CLEAN=true
            CLEAN_ALL=true
            shift
            ;;
        -*)
            echo -e "Unknown option: $1\n"
            _show_flash_help
            return 1
            ;;
        *)
            DEVICE_NAME="$1"
            shift
            ;;
        esac
    done

    # Check if a device name was provided
    if [ -z "$DEVICE_NAME" ]; then
        _show_flash_help
        return 1
    fi

    _init_vars "$DEVICE_NAME"

    # Check if the device directory exists
    if [ ! -d "${DEVICE_DIR}" ]; then
        echo -e "\n${COLOR_RED}Error: Device directory not found: ${DEVICE_DIR}${COLOR_RESET}\n"
        _list_devices
        return 1
    fi

    # Clean build artifacts if requested
    if $CLEAN; then
        _clean_build_artifacts
        if $CLEAN_ALL; then
            _clean_microros_lib
        fi
    fi

    # Build the firmware before flashing
    build "$DEVICE_NAME"

    if [ $? -ne 0 ]; then
        echo "Build failed. Flash aborted."
        return 1
    fi

    # Flash the device
    echo "Flashing device: ${DEVICE_NAME}"
    sudo openocd -f "${OPENOCD_CFG}" -c "program ${BUILD_DIR}/${DEVICE_NAME}.elf verify reset exit"

    if [ $? -eq 0 ]; then
        echo -e "\n${COLOR_GREEN}Firmware has been successfully flashed to the ${DEVICE_NAME}!${COLOR_RESET}\n"
    else
        echo -e "\n${COLOR_RED}Firmware could not be flashed to the ${DEVICE_NAME}!\nCheck the error log for more information.${COLOR_RESET}\n"
        return 1
    fi
}

# Clean build artifacts for a specific device
# This will remove the build directory in the device folder.
# If the --all flag is provided, the micro-ROS library will also be removed.
#
# Usage: clean [--all] [--global] <DeviceName>
# Options:
#   -a, --all: Clean all build artifacts including the micro-ROS library
#   -g, --global: Clean all build artifacts for all devices and the micro-ROS firmware workspace itself
clean() {
    local CLEAN_ALL=false
    local CLEAN_GLOBAL=false

    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
        -a | --all)
            CLEAN_ALL=true
            shift
            ;;
        -g | --global)
            CLEAN_GLOBAL=true
            shift
            ;;
        -*)
            echo -e "Unknown option: $1\n"
            _show_clean_help
            return 1
            ;;
        *)
            DEVICE_NAME="$1"
            shift
            ;;
        esac
    done

    # Clean all devices and micro-ROS firmware, no device name must be provided
    if $CLEAN_GLOBAL; then
        _clean_all_devices
        return
    fi

    # Check if a device name was provided
    if [ -z "$DEVICE_NAME" ]; then
        _show_clean_help
        return 1
    fi

    _init_vars "$DEVICE_NAME"

    # Check if the device directory exists
    if [ ! -d "${DEVICE_DIR}" ]; then
        echo -e "\n${COLOR_RED}Error: Device directory not found: ${DEVICE_DIR}${COLOR_RESET}\n"
        _list_devices
        return 1
    fi

    _clean_build_artifacts
    if $CLEAN_ALL; then
        _clean_microros_lib
    fi
}
