#!/usr/bin/env bash

# CAuDri - Generate a device-specific libmicroros.a using a sourced micro-ROS setup.
#
# Usage:
#   ./generate_lib.sh [--clean-workspace] <DeviceName>
#
# The generated micro-ROS workspace is retained between builds. Pass
# --clean-workspace to recreate it before building.

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly FIRMWARE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
readonly WORKSPACE_DIR="${SCRIPT_DIR}/firmware"
readonly TOOLCHAIN_FILE="${SCRIPT_DIR}/toolchain.cmake"

CLEAN_WORKSPACE=false
DEVICE_NAME=""

usage() {
    printf 'Usage: %s [--clean-workspace] <DeviceName>\n' "$(basename -- "$0")"
}

list_devices() {
    printf 'Available devices:\n'
    local device_dir
    for device_dir in "${FIRMWARE_DIR}"/devices/*/; do
        [ -d "${device_dir}" ] || continue
        printf '  %s\n' "$(basename -- "${device_dir}")"
    done
}

fail() {
    printf 'Error: %s\n' "$*" >&2
    exit 1
}

require_command() {
    command -v "$1" >/dev/null 2>&1 || fail "Required command not found: $1"
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --clean-workspace)
            CLEAN_WORKSPACE=true
            ;;
        -h|--help)
            usage
            list_devices
            exit 0
            ;;
        -*)
            usage >&2
            fail "Unknown option: $1"
            ;;
        *)
            [ -z "${DEVICE_NAME}" ] || fail "Only one device may be specified"
            DEVICE_NAME="$1"
            ;;
    esac
    shift
done

if [ -z "${DEVICE_NAME}" ]; then
    usage >&2
    list_devices >&2
    fail "No device specified"
fi

[ -n "${ROS_DISTRO:-}" ] || fail 'ROS_DISTRO is not set; source the ROS 2 and micro-ROS setup first'

require_command ros2
require_command colcon
require_command cmake

readonly DEVICE_DIR="${FIRMWARE_DIR}/devices/${DEVICE_NAME}"
readonly DEVICE_TOOLCHAIN="${DEVICE_DIR}/board/cmake/gcc-arm-none-eabi.cmake"
readonly COLCON_META_FILE="${DEVICE_DIR}/config/colcon.meta"
readonly OUTPUT_DIR="${DEVICE_DIR}/microros_lib"
readonly STAGING_DIR="${DEVICE_DIR}/microros_lib.tmp"

[ -d "${DEVICE_DIR}" ] || {
    list_devices >&2
    fail "Unknown device: ${DEVICE_NAME}"
}
[ -f "${DEVICE_TOOLCHAIN}" ] || fail "Device toolchain not found: ${DEVICE_TOOLCHAIN}"
[ -f "${COLCON_META_FILE}" ] || fail "micro-ROS configuration not found: ${COLCON_META_FILE}"
[ -f "${TOOLCHAIN_FILE}" ] || fail "micro-ROS toolchain not found: ${TOOLCHAIN_FILE}"

export TOOLCHAIN_PREFIX="${TOOLCHAIN_PREFIX:-/usr/bin/arm-none-eabi-}"
export DEVICE_PATH="${DEVICE_DIR}"
unset RMW_IMPLEMENTATION

if [ "${CLEAN_WORKSPACE}" = true ] && [ -d "${WORKSPACE_DIR}" ]; then
    printf 'Removing micro-ROS workspace: %s\n' "${WORKSPACE_DIR}"
    rm -rf -- "${WORKSPACE_DIR}"
fi

cd -- "${SCRIPT_DIR}"

if [ ! -d "${WORKSPACE_DIR}" ]; then
    printf 'Creating micro-ROS %s static-library workspace\n' "${ROS_DISTRO}"
    ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
fi

printf 'Building micro-ROS for %s using ROS_DISTRO=%s\n' "${DEVICE_NAME}" "${ROS_DISTRO}"
ros2 run micro_ros_setup build_firmware.sh "${TOOLCHAIN_FILE}" "${COLCON_META_FILE}"

readonly BUILD_OUTPUT="${WORKSPACE_DIR}/build"
[ -f "${BUILD_OUTPUT}/libmicroros.a" ] || fail "Build completed without producing libmicroros.a"
[ -d "${BUILD_OUTPUT}/include" ] || fail "Build completed without producing include files"

rm -rf -- "${STAGING_DIR}"
mkdir -p -- "${STAGING_DIR}/inc"
cp -a -- "${BUILD_OUTPUT}/include/." "${STAGING_DIR}/inc/"
cp -- "${BUILD_OUTPUT}/libmicroros.a" "${STAGING_DIR}/libmicroros.a"

# Some generated packages install headers as <package>/<package>/... . Flatten
# those directories so application includes match the ROS-generated paths.
for package_dir in "${STAGING_DIR}"/inc/*/; do
    [ -d "${package_dir}" ] || continue
    package_name="$(basename -- "${package_dir}")"
    nested_include="${package_dir}/${package_name}"
    if [ -d "${nested_include}" ]; then
        cp -a -- "${nested_include}/." "${package_dir}/"
        rm -rf -- "${nested_include}"
    fi
done

rm -rf -- "${OUTPUT_DIR}"
mv -- "${STAGING_DIR}" "${OUTPUT_DIR}"

printf 'micro-ROS library generated successfully: %s\n' "${OUTPUT_DIR}"
