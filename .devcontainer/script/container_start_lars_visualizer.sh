#!/bin/bash

# CAuDri - Helper script for building and starting the LARS RViz visualizer.
# The visualizer workspace intentionally lives outside the firmware repository,
# because it is a host-side ROS application and may later become its own shared
# repository together with the LARS message package.

set -e
set -o pipefail

LARS_ROS_WS="${LARS_ROS_WS:-/home/caudri/lars_ros_ws}"
LARS_VISUALIZER_MOCK=false
LARS_VISUALIZER_BUILD_ONLY=false
LARS_VISUALIZER_CLEAN=false

show_help() {
    echo "Usage: container_start_lars_visualizer.sh [--mock] [--clean] [--build-only]"
    echo ""
    echo "Options:"
    echo "  -m, --mock        Start mock hardware publishers with the visualizer."
    echo "  -c, --clean       Remove build/install/log before building."
    echo "  -b, --build-only  Build the visualizer workspace and exit."
    echo "  -h, --help        Show this help text."
    echo ""
    echo "Environment:"
    echo "  LARS_ROS_WS       Workspace path (default: ${LARS_ROS_WS})"
}

source_ros_environment() {
    local distro="${ROS_DISTRO:-jazzy}"

    if [ ! -f "/opt/ros/${distro}/setup.bash" ]; then
        echo "ROS 2 setup not found for ROS_DISTRO=${distro}."
        return 1
    fi

    source "/opt/ros/${distro}/setup.bash"

    if [ -f "${LARS_ROS_WS}/install/setup.bash" ]; then
        source "${LARS_ROS_WS}/install/setup.bash"
    fi
}

ensure_workspace_layout() {
    mkdir -p "${LARS_ROS_WS}/src"

    # Keep the host workspace tied to the firmware-side ROS packages without
    # copying generated interface or description sources around.
    ln -sfn "/hardware_interface/firmware/libraries/lars_ros_interfaces/lars_msgs" \
        "${LARS_ROS_WS}/src/lars_msgs"
    ln -sfn "/hardware_interface/.devcontainer/tools/ros_packages/lars_description" \
        "${LARS_ROS_WS}/src/lars_description"
}

build_visualizer_workspace() {
    echo "Building LARS visualizer workspace in ${LARS_ROS_WS}"

    source_ros_environment
    ensure_workspace_layout

    pushd "${LARS_ROS_WS}" >/dev/null

    if ${LARS_VISUALIZER_CLEAN}; then
        rm -rf build install log
    fi

    colcon build --symlink-install

    popd >/dev/null
}

cleanup_stale_visualizer_processes() {
    # A crashed RViz process can leave helper nodes alive. If a second visualizer
    # is started afterwards, multiple joint-state bridges publish the same wheel
    # joints and RViz shows visible jitter.
    pkill -f "${LARS_ROS_WS}/install/lars_visualization/lib/lars_visualization/joint_state_bridge.py" 2>/dev/null || true
    pkill -f "${LARS_ROS_WS}/install/lars_visualization/lib/lars_visualization/mock_hardware.py" 2>/dev/null || true
}

while [ $# -gt 0 ]; do
    case "$1" in
        -m|--mock)
            LARS_VISUALIZER_MOCK=true
            ;;
        -c|--clean)
            LARS_VISUALIZER_CLEAN=true
            ;;
        -b|--build-only)
            LARS_VISUALIZER_BUILD_ONLY=true
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
    shift
done

ensure_workspace_layout

if [ ! -f "${LARS_ROS_WS}/install/setup.bash" ] || ${LARS_VISUALIZER_CLEAN} || ${LARS_VISUALIZER_BUILD_ONLY}; then
    build_visualizer_workspace
fi

if ${LARS_VISUALIZER_BUILD_ONLY}; then
    exit 0
fi

source_ros_environment
cleanup_stale_visualizer_processes

echo "Starting LARS visualizer"
echo "  workspace: ${LARS_ROS_WS}"
echo "  mock:      ${LARS_VISUALIZER_MOCK}"
echo ""

ros2 launch lars_visualization visualizer.launch.py mock:="${LARS_VISUALIZER_MOCK}"
