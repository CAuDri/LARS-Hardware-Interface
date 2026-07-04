#!/bin/bash

# CAuDri - Helper script for building and starting the micro-ROS agent in the devcontainer.
# In VS Code, the "Micro-ROS Agent" action button starts this script through the
# "Connect micro-ROS Agent" task. The script keeps the interactive action-button
# workflow, but it can also be used non-interactively from a terminal:
#
#   .devcontainer/script/container_start_microros_agent.sh --dev /dev/ttyACM0
#   MICROROS_AGENT_BAUDRATE=921600 .devcontainer/script/container_start_microros_agent.sh

set -o pipefail

MICROROS_AGENT_WS="${MICROROS_AGENT_WS:-${HOME}/microros_agent_ws}"
MICROROS_AGENT_BAUDRATE="${MICROROS_AGENT_BAUDRATE:-921600}"
MICROROS_AGENT_DEVICE="${MICROROS_AGENT_DEVICE:-}"

source_ros_environment() {
    local distro="${ROS_DISTRO:-jazzy}"

    if [ -f "/opt/ros/${distro}/setup.bash" ]; then
        source "/opt/ros/${distro}/setup.bash"
    else
        echo "ROS 2 setup not found for ROS_DISTRO=${distro}."
        return 1
    fi

    # The Dockerfile installs micro_ros_setup into /uros_ws. Sourcing it gives us
    # the create_agent_ws.sh/build_agent.sh entrypoints used below.
    if [ -f "/uros_ws/install/local_setup.bash" ]; then
        source "/uros_ws/install/local_setup.bash"
    fi

    if [ -f "/uros_ws/install/micro_ros_setup/share/micro_ros_setup/local_setup.bash" ]; then
        source "/uros_ws/install/micro_ros_setup/share/micro_ros_setup/local_setup.bash"
    fi

    if [ -f "${MICROROS_AGENT_WS}/install/setup.bash" ]; then
        source "${MICROROS_AGENT_WS}/install/setup.bash"
    fi
}

show_help() {
    echo "Usage: container_start_microros_agent.sh [--clean] [--build-only] [--dev <serial-device>]"
    echo ""
    echo "Options:"
    echo "  -c, --clean       Recreate the local micro-ROS agent workspace before building."
    echo "  -b, --build-only  Build the agent workspace and exit without starting the agent."
    echo "  -d, --dev DEVICE  Start the agent directly with DEVICE instead of showing the menu."
    echo "  -h, --help        Show this help text."
    echo ""
    echo "Environment:"
    echo "  MICROROS_AGENT_WS        Workspace path (default: ${HOME}/microros_agent_ws)"
    echo "  MICROROS_AGENT_BAUDRATE  Serial baud rate passed to the agent (default: 921600)"
    echo "  MICROROS_AGENT_DEVICE    Same as --dev when set"
}

micro_ros_agent_available() {
    source_ros_environment >/dev/null 2>&1 || return 1
    ros2 pkg prefix micro_ros_agent >/dev/null 2>&1
}

build_micro_ros_agent() {
    echo "Building micro-ROS agent workspace in ${MICROROS_AGENT_WS}"

    source_ros_environment || return 1

    mkdir -p "${MICROROS_AGENT_WS}"
    pushd "${MICROROS_AGENT_WS}" >/dev/null || return 1

    # create_agent_ws.sh is idempotent enough for normal use and fetches the
    # Jazzy-compatible micro_ros_agent sources defined by micro_ros_setup.
    ros2 run micro_ros_setup create_agent_ws.sh src || {
        popd >/dev/null
        return 1
    }

    ros2 run micro_ros_setup build_agent.sh || {
        popd >/dev/null
        return 1
    }

    popd >/dev/null || return 1
    source_ros_environment || return 1

    echo "micro-ROS agent build finished."
}

recreate_micro_ros_agent_workspace() {
    if [ -d "${MICROROS_AGENT_WS}" ]; then
        echo "Removing ${MICROROS_AGENT_WS}"
        rm -rf "${MICROROS_AGENT_WS}"
    fi
}

ensure_micro_ros_agent() {
    if micro_ros_agent_available; then
        return 0
    fi

    echo "micro_ros_agent is not available yet."
    build_micro_ros_agent
}

# KITcar-style menu helper. It receives the available device paths directly
# instead of keeping a separate display/path mapping. That deliberately keeps the
# interactive selector tied to real /dev/tty* nodes, which are what the
# micro-ROS agent reopens after a USB disconnect/reconnect.
choose_from_menu() {
    local prompt="$1" outvar="$2"
    shift
    shift
    local options=("$@") cur=0 count=${#options[@]} index=0
    local key

    tput civis 2>/dev/null || true
    printf "$prompt\n"
    while true; do
        index=0
        for option in "${options[@]}"; do
            if [ "${index}" -eq "${cur}" ]; then
                echo -e " >\e[7m${option}\e[0m"
            else
                echo "  ${option}"
            fi
            index=$((index + 1))
        done

        read -r -s -n3 key
        if [[ "${key}" == $'\e[A' ]]; then
            cur=$((cur - 1))
            [ "${cur}" -lt 0 ] && cur=0
        elif [[ "${key}" == $'\e[B' ]]; then
            cur=$((cur + 1))
            [ "${cur}" -ge "${count}" ] && cur=$((count - 1))
        elif [[ "${key}" == "" ]]; then
            break
        fi

        echo -en "\e[${count}A"
    done

    tput cnorm 2>/dev/null || true
    printf -v "${outvar}" "%s" "${options[${cur}]}"
}

start_micro_ros_agent() {
    local device="$1"
    local result

    source_ros_environment || return 1

    # Match the KITcar action-button workflow: CTRL+C should stop the running
    # agent and return to device selection, not close the whole helper script.
    trap ":" SIGINT

    echo "Starting micro_ros_agent"
    echo "  device:   ${device}"
    echo "  baudrate: ${MICROROS_AGENT_BAUDRATE}"
    echo ""

    ros2 run micro_ros_agent micro_ros_agent serial --dev "${device}" -b "${MICROROS_AGENT_BAUDRATE}"
    result=$?

    trap "exit 0" SIGINT
    return "${result}"
}

select_serial_device() {
    trap "exit 0" SIGINT
    clear 2>/dev/null || true

    while true; do
        local devices=()
        local selected_device
        local device

        while IFS= read -r device; do
            devices+=("${device}")
        done < <(find /dev -maxdepth 1 \( -name 'ttyUSB*' -o -name 'ttyACM*' \) | sort)

        if [ ${#devices[@]} -eq 0 ]; then
            echo "No serial devices found. Please connect a device and try again."
            echo "Press any key to retry or CTRL+C to exit..."
            read -r -n 1
            continue
        fi

        choose_from_menu "Select a serial device to connect the micro-ROS agent [CTRL+C to quit]:" selected_device "${devices[@]}"
        echo ""
        start_micro_ros_agent "${selected_device}"
        echo ""
        echo "micro_ros_agent exited. Returning to device selection."
    done
}

CLEAN=false
BUILD_ONLY=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        -c | --clean)
            CLEAN=true
            shift
            ;;
        -b | --build-only)
            BUILD_ONLY=true
            shift
            ;;
        -d | --dev)
            if [ -z "${2:-}" ]; then
                echo "--dev requires a serial device path."
                exit 1
            fi
            MICROROS_AGENT_DEVICE="$2"
            shift 2
            ;;
        -h | --help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

trap 'exit 0' SIGINT

if ${CLEAN}; then
    recreate_micro_ros_agent_workspace
fi

ensure_micro_ros_agent || {
    echo "Failed to prepare the micro-ROS agent."
    exit 1
}

if ${BUILD_ONLY}; then
    exit 0
fi

if [ -n "${MICROROS_AGENT_DEVICE}" ]; then
    start_micro_ros_agent "${MICROROS_AGENT_DEVICE}"
else
    select_serial_device
fi
