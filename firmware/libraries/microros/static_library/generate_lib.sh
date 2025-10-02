#!/bin/bash

# CAuDri - Bash script for generating the static micro-ROS library
# Requires a sourced micro-ros-setup installation
# The first argument needs to be the device name (name of subdirectory in devices/)

# Path to GNU ARM toolchain
export TOOLCHAIN_PREFIX=/usr/bin/arm-none-eabi-

# Set the RMW implementation for micro-ROS (will currently be set automatically)
unset RMW_IMPLEMENTATION

#### Init ####
set -e

# Get the absolute path of the bash script
DIR="$( dirname -- "${BASH_SOURCE[0]}"; )";
DIR="$( realpath -e -- "$DIR"; )";

# Get the absolute path of the firmware directory (TODO find a more flexible solution)
export FIRMWARE_PATH=$(cd ${DIR} && cd ../../.. && pwd)

pushd ${DIR} > /dev/null

# List all available devices
list_devices() {
    echo "Available devices are:"
    for dir in ${FIRMWARE_PATH}/devices/*/ ; do
        echo "    $(basename $dir)"
    done
}

# Check if a device name argument was passed
if [ -z "$1" ]; then
    echo "Error: No device name provided."
    echo "Usage: $0 [device_name]"
    list_devices
    exit 1
fi

export DEVICE_PATH="${FIRMWARE_PATH}/devices/$1"

TOOLCHAIN_FILE="${DEVICE_PATH}/board/cmake/gcc-arm-none-eabi.cmake"
COLCON_META_FILE="${DEVICE_PATH}/config/colcon.meta"

# Check if the device directory exists
if ! [ -d "$DEVICE_PATH" ]; then
    echo "Error: Device directory not found: $DEVICE_PATH"
    list_devices
    exit 1
fi

# Check if a Cube-MX generated toolchain file is present
if ! [ -f "$TOOLCHAIN_FILE" ]; then
    echo "Error: No toolchain file could be found at $TOOLCHAIN_FILE"
    exit 1
fi

#### Build static library ####

LIB_PATH="${DEVICE_PATH}/microros_lib"

# Somehow the build process for some ROS packages will fail due to CRLF line endings in certain files.
# This happend after a recent update of the micro-ros-setup package and hopefully won't persist in the future.
# This is a temporary workaround to fix the issue, it will start a background process automatically fixing line endings.
# If you should ever stumble upon this, please uncomment the following code block and test if the build succeeds. If so just remove it.
echo "Starting temporary line-ending watcher..."
WATCH_DIR="firmware/dev_ws/"
timeout 100s bash -lc '
  stamp=$(mktemp); touch "$stamp"

  while :; do
    if [ -d firmware/dev_ws ]; then
      # Only consider files newer than last pass
      find firmware/dev_ws -type f \( -name "*.sh" -o -name "*.bash" \) -newer "$stamp" -print0 \
        | xargs -0 -r dos2unix >/dev/null 2>&1 || true
      touch "$stamp"
    fi
    sleep 0.1
  done
' &

# Generate workspace if no firmware directory is present
if [ ! -d "firmware" ]; then
    echo -e "\nGenerating micro-ROS workspace\n"
    sudo apt update && rosdep update --rosdistro $ROS_DISTRO

    ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
fi

# # Copy the kitcar_interfaces submodule to the workspace
# if [ -d "firmware/mcu_ws/src/kitcar-interfaces" ]; then
#     rm -rf firmware/mcu_ws/src/kitcar-interfaces/*
# else
#     mkdir -p firmware/mcu_ws/src/kitcar-interfaces
# fi
# echo -e "\nCopying kitcar_interfaces \n"
# cp -R "$REPO_PATH/libraries/kitcar-interfaces" ./firmware/mcu_ws/src/

# Remove the existing library in the device directory if present
if [ -d "${LIB_PATH}" ]; then
    rm -rf $LIB_PATH
fi

# Build micro-ROS library
echo -e "\nBuilding micro-ROS library\n"
export DEVICE_PATH="${FIRMWARE_PATH}/devices/$1"
ros2 run micro_ros_setup build_firmware.sh $FIRMWARE_PATH/libraries/microros/static_library/toolchain.cmake $DEVICE_PATH/config/colcon.meta

# Copy micro-ROS library to device directory
echo -e "\nCopying micro-ROS library to device directory\n"
mkdir -p $LIB_PATH/inc
cp -R firmware/build/include/* $LIB_PATH/inc
cp -R firmware/build/libmicroros.a $LIB_PATH

# Fix nested include paths
pushd firmware/mcu_ws > /dev/null
    INCLUDE_ROS2_PACKAGES=$(colcon list | awk '{print $1}' | awk -v d=" " '{s=(NR==1?s:s d)$0}END{print s}')
popd > /dev/null
for var in ${INCLUDE_ROS2_PACKAGES}; do
    if [ -d "$LIB_PATH/inc/${var}/${var}" ]; then
        rsync -r $LIB_PATH/inc/${var}/${var}/* $LIB_PATH/inc/${var}
        rm -rf $LIB_PATH/inc/${var}/${var}
    fi
done

# Remove build directory
if [ -d "firmware/build" ]; then
    rm -rf firmware/build
fi

echo -e "\nMicro-ROS static library built sucessfully"
echo -e "Copied to ${LIB_PATH}"