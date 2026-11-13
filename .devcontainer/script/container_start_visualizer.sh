#!/bin/bash

# CAuDri - Helper script for starting the local ROS Visualizer dashboard.
# Lichtblick provides the browser UI, while foxglove_bridge exposes the ROS 2
# graph through a local WebSocket connection.

set -e
set -o pipefail

VISUALIZER_ROS_WS="${VISUALIZER_ROS_WS:-${HOME}/ros_visualizer_ws}"
VISUALIZER_IMAGE="${VISUALIZER_IMAGE:-ghcr.io/lichtblick-suite/lichtblick:latest}"
VISUALIZER_CONTAINER="${VISUALIZER_CONTAINER:-ros-visualizer}"
VISUALIZER_PORT="${VISUALIZER_PORT:-8080}"
VISUALIZER_OPEN="${VISUALIZER_OPEN:-true}"
VISUALIZER_AUTO_CONNECT="${VISUALIZER_AUTO_CONNECT:-true}"
VISUALIZER_LAYOUT="${VISUALIZER_LAYOUT:-/hardware_interface/.devcontainer/tools/lichtblick/default_layout.json}"
VISUALIZER_LAYOUT_NAME="${VISUALIZER_LAYOUT_NAME:-ROS Visualizer}"
VISUALIZER_LAYOUT_ID="${VISUALIZER_LAYOUT_ID:-ros_visualizer}"
VISUALIZER_BOOTSTRAP="${VISUALIZER_BOOTSTRAP:-/hardware_interface/.devcontainer/tools/lichtblick/visualizer.html}"
VISUALIZER_EXTENSION_DIR="${VISUALIZER_EXTENSION_DIR:-/hardware_interface/.devcontainer/tools/lichtblick/extensions/hardware-interface-panels}"
VISUALIZER_EXTENSION_PACKAGE="${VISUALIZER_EXTENSION_PACKAGE:-${VISUALIZER_EXTENSION_DIR}/caudri.hardware-interface-panels-0.1.0.foxe}"
VISUALIZER_BRIDGE_ADDRESS="${VISUALIZER_BRIDGE_ADDRESS:-0.0.0.0}"
VISUALIZER_BRIDGE_PORT="${VISUALIZER_BRIDGE_PORT:-8765}"
VISUALIZER_BRIDGE_BROWSER_HOST="${VISUALIZER_BRIDGE_BROWSER_HOST:-localhost}"
VISUALIZER_BRIDGE_CLIENT_TOPIC_WHITELIST="${VISUALIZER_BRIDGE_CLIENT_TOPIC_WHITELIST:-['^/hardware/command/.*$']}"
VISUALIZER_MODEL="${VISUALIZER_MODEL:-true}"
VISUALIZER_DOCKER_ROOT="${VISUALIZER_DOCKER_ROOT:-/tmp/ros-visualizer-docker}"
VISUALIZER_NOTICE_FILE="${VISUALIZER_NOTICE_FILE:-${HOME}/.cache/ros-visualizer-host-container-notice}"
VISUALIZER_ASSUME_YES="${VISUALIZER_ASSUME_YES:-false}"

lichtblick_pid=""
bridge_pid=""
description_pid=""
DOCKER_CMD=()

show_help() {
    echo "Usage: container_start_visualizer.sh [--no-open] [--lichtblick-port PORT] [--bridge-port PORT]"
    echo ""
    echo "Options:"
    echo "  --no-open              Start servers and do not open the browser."
    echo "  --lichtblick-port PORT Local Lichtblick HTTP port (default: ${VISUALIZER_PORT})."
    echo "  --bridge-port PORT     Foxglove WebSocket bridge port (default: ${VISUALIZER_BRIDGE_PORT})."
    echo "  -h, --help             Show this help text."
    echo ""
    echo "Environment:"
    echo "  VISUALIZER_ROS_WS                         Host-side ROS workspace with LARS messages."
    echo "  VISUALIZER_IMAGE               Lichtblick container image."
    echo "  VISUALIZER_LAYOUT              Layout data used for the ROS visualizer."
    echo "  VISUALIZER_LAYOUT_NAME         Display name for the visualizer layout."
    echo "  VISUALIZER_LAYOUT_ID           Stable local ID for the visualizer layout."
    echo "  VISUALIZER_EXTENSION_DIR       Local custom Lichtblick extension source."
    echo "  VISUALIZER_OPEN                Set to false to skip opening the browser."
    echo "  VISUALIZER_AUTO_CONNECT        Set to false to skip the startup bridge URL."
    echo "  VISUALIZER_ASSUME_YES          Set to true to skip the first-run host container prompt."
    echo "  VISUALIZER_BRIDGE_BROWSER_HOST            Hostname the browser should use for the bridge."
    echo "  VISUALIZER_BRIDGE_CLIENT_TOPIC_WHITELIST  Topics the web UI may publish to."
    echo "  VISUALIZER_MODEL         Set to false to skip the temporary URDF model."
}

source_base_ros_environment() {
    local distro="${ROS_DISTRO:-jazzy}"

    if [ ! -f "/opt/ros/${distro}/setup.bash" ]; then
        echo "ROS 2 setup not found for ROS_DISTRO=${distro}."
        return 1
    fi

    source "/opt/ros/${distro}/setup.bash"
}

ensure_ros_workspace_layout() {
    mkdir -p "${VISUALIZER_ROS_WS}/src"

    local lars_msgs_source="/hardware_interface/firmware/libraries/lars_ros_interfaces/lars_msgs"
    local lars_description_source="/hardware_interface/.devcontainer/tools/ros_packages/lars_description"

    if [ ! -d "${lars_msgs_source}" ]; then
        echo "Required ROS package not found: ${lars_msgs_source}"
        return 1
    fi
    if [ ! -d "${lars_description_source}" ]; then
        echo "Required ROS package not found: ${lars_description_source}"
        return 1
    fi

    if [ -e "${VISUALIZER_ROS_WS}/src/lars_msgs" ] && [ ! -L "${VISUALIZER_ROS_WS}/src/lars_msgs" ]; then
        echo "Cannot link ${lars_msgs_source} into ${VISUALIZER_ROS_WS}/src/lars_msgs; the path already exists and is not a symlink."
        return 1
    fi
    if [ -e "${VISUALIZER_ROS_WS}/src/lars_description" ] && [ ! -L "${VISUALIZER_ROS_WS}/src/lars_description" ]; then
        echo "Cannot link ${lars_description_source} into ${VISUALIZER_ROS_WS}/src/lars_description; the path already exists and is not a symlink."
        return 1
    fi

    ln -sfn "${lars_msgs_source}" "${VISUALIZER_ROS_WS}/src/lars_msgs"
    ln -sfn "${lars_description_source}" "${VISUALIZER_ROS_WS}/src/lars_description"
}

ensure_ros_overlay() {
    source_base_ros_environment
    ensure_ros_workspace_layout

    if [ -f "${VISUALIZER_ROS_WS}/install/lars_msgs/share/lars_msgs/package.bash" ] \
        && [ -f "${VISUALIZER_ROS_WS}/install/lars_description/share/lars_description/package.bash" ] \
        && [ -x "${VISUALIZER_ROS_WS}/install/lars_description/lib/lars_description/imu_orientation_tf_bridge.py" ]; then
        return 0
    fi

    echo "Building visualizer ROS overlay in ${VISUALIZER_ROS_WS}"
    pushd "${VISUALIZER_ROS_WS}" >/dev/null
    colcon build --symlink-install --packages-select lars_msgs lars_description
    popd >/dev/null
}

source_ros_environment() {
    source_base_ros_environment

    # The bridge needs the workspace overlay to resolve custom lars_msgs schemas.
    if [ -f "${VISUALIZER_ROS_WS}/install/setup.bash" ]; then
        source "${VISUALIZER_ROS_WS}/install/setup.bash"
    fi
}

ensure_foxglove_bridge_available() {
    if ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
        return 0
    fi

    local distro="${ROS_DISTRO:-jazzy}"
    local package_name="ros-${distro}-foxglove-bridge"

    echo "foxglove_bridge is not installed in this container; installing ${package_name}."

    if ! sudo apt-get update; then
        echo "Could not update apt package lists."
        return 1
    fi

    if ! sudo apt-get install -y "${package_name}"; then
        echo "Could not install ${package_name}."
        echo "Rebuild the devcontainer or install it manually with:"
        echo "  sudo apt-get update && sudo apt-get install -y ${package_name}"
        return 1
    fi

    source_base_ros_environment

    if ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
        return 0
    fi

    echo "${package_name} installed, but ros2 still cannot find foxglove_bridge."
    return 1
}

ensure_robot_state_publisher_available() {
    if ros2 pkg prefix robot_state_publisher >/dev/null 2>&1; then
        return 0
    fi

    local distro="${ROS_DISTRO:-jazzy}"
    local package_name="ros-${distro}-robot-state-publisher"

    echo "robot_state_publisher is not installed in this container; installing ${package_name}."

    if ! sudo apt-get update; then
        echo "Could not update apt package lists."
        return 1
    fi

    if ! sudo apt-get install -y "${package_name}"; then
        echo "Could not install ${package_name}."
        return 1
    fi

    source_base_ros_environment

    if ros2 pkg prefix robot_state_publisher >/dev/null 2>&1; then
        return 0
    fi

    echo "${package_name} installed, but ros2 still cannot find robot_state_publisher."
    return 1
}

open_url_best_effort() {
    local url="$1"

    echo "Lichtblick URL:"
    echo "  ${url}"
    echo ""

    if [ "${VISUALIZER_OPEN}" != "true" ]; then
        return 0
    fi

    if [ -n "${BROWSER:-}" ] && [ -x "${BROWSER}" ]; then
        "${BROWSER}" "${url}" >/dev/null 2>&1 && return 0
    fi

    if command -v xdg-open >/dev/null 2>&1; then
        xdg-open "${url}" >/dev/null 2>&1 && return 0
    fi

    if command -v wslview >/dev/null 2>&1; then
        wslview "${url}" >/dev/null 2>&1 && return 0
    fi

    if command -v cmd.exe >/dev/null 2>&1; then
        cmd.exe /C start "" "${url}" >/dev/null 2>&1 && return 0
    fi

    echo "Could not open Lichtblick automatically. Open the URL above manually."
}

lichtblick_startup_url() {
    local base_url="http://localhost:${VISUALIZER_PORT}"
    local path="/visualizer.html"

    if [ "${VISUALIZER_AUTO_CONNECT}" != "true" ]; then
        echo "${base_url}${path}"
        return 0
    fi

    local bridge_url="ws://${VISUALIZER_BRIDGE_BROWSER_HOST}:${VISUALIZER_BRIDGE_PORT}"

    echo "${base_url}${path}?ds=foxglove-websocket&ds.url=${bridge_url}"
}

set_docker_command() {
    if ! command -v docker >/dev/null 2>&1; then
        return 1
    fi

    if docker info >/dev/null 2>&1; then
        DOCKER_CMD=(docker)
        return 0
    fi

    if sudo docker info >/dev/null 2>&1; then
        DOCKER_CMD=(sudo docker)
        return 0
    fi

    return 1
}

docker_available() {
    set_docker_command
}

ensure_docker_cli_available() {
    if command -v docker >/dev/null 2>&1; then
        return 0
    fi

    echo "Docker CLI is not installed in this container; installing docker.io on demand."
    echo "The devcontainer image stays small, and Lichtblick will use the mounted host Docker socket when available."

    if ! sudo apt-get update; then
        echo "Could not update apt package lists."
        return 1
    fi

    if ! sudo apt-get install -y docker.io; then
        echo "Could not install docker.io."
        return 1
    fi
}

ensure_docker_available() {
    if [ ! -S /var/run/docker.sock ]; then
        echo "The host Docker socket is not mounted at /var/run/docker.sock."
        echo "Rebuild or recreate the devcontainer so VS Code applies this mount from .devcontainer/devcontainer.json:"
        echo "  source=/var/run/docker.sock,target=/var/run/docker.sock,type=bind"
        echo ""
        echo "If you need a temporary workaround, run Lichtblick on the host and import:"
        echo "  ${VISUALIZER_LAYOUT}"
        echo ""
        return 1
    fi

    ensure_docker_cli_available || return 1

    if docker_available; then
        return 0
    fi

    echo "A Docker socket is mounted at /var/run/docker.sock, but the Docker CLI cannot access it."
    echo "Check host socket permissions, or use sudo inside the devcontainer."
    return 1
}

node_version_available() {
    command -v node >/dev/null 2>&1 || return 1
    node -e 'const [major, minor] = process.versions.node.split(".").map(Number); process.exit(major > 18 || (major === 18 && minor >= 18) ? 0 : 1)' >/dev/null 2>&1
}

ensure_node_available() {
    if node_version_available && command -v npm >/dev/null 2>&1; then
        return 0
    fi

    echo "Node.js >= 18.18 and npm are required to package the ROS Visualizer extension."
    echo "Installing nodejs and npm on demand."

    if ! sudo apt-get update; then
        echo "Could not update apt package lists."
        return 1
    fi

    if ! sudo apt-get install -y nodejs npm; then
        echo "Could not install nodejs and npm."
        return 1
    fi

    if node_version_available && command -v npm >/dev/null 2>&1; then
        return 0
    fi

    echo "Installed Node.js is too old for create-lichtblick-extension."
    echo "Install Node.js >= 18.18, then rerun this launcher."
    return 1
}

lichtblick_extension_package_stale() {
    if [ ! -f "${VISUALIZER_EXTENSION_PACKAGE}" ]; then
        return 0
    fi

    local package_input
    for package_input in src package.json package-lock.json tsconfig.json README.md CHANGELOG.md; do
        if [ ! -e "${VISUALIZER_EXTENSION_DIR}/${package_input}" ]; then
            return 0
        fi
    done

    find "${VISUALIZER_EXTENSION_DIR}/src" \
        "${VISUALIZER_EXTENSION_DIR}/package.json" \
        "${VISUALIZER_EXTENSION_DIR}/package-lock.json" \
        "${VISUALIZER_EXTENSION_DIR}/tsconfig.json" \
        "${VISUALIZER_EXTENSION_DIR}/README.md" \
        "${VISUALIZER_EXTENSION_DIR}/CHANGELOG.md" \
        -newer "${VISUALIZER_EXTENSION_PACKAGE}" -print -quit | grep -q .
}

ensure_lichtblick_extension_package() {
    if [ ! -d "${VISUALIZER_EXTENSION_DIR}" ]; then
        echo "ROS Visualizer extension source not found: ${VISUALIZER_EXTENSION_DIR}"
        return 1
    fi

    if ! lichtblick_extension_package_stale; then
        return 0
    fi

    ensure_node_available || return 1

    echo "Packaging ROS Visualizer extension in ${VISUALIZER_EXTENSION_DIR}"
    pushd "${VISUALIZER_EXTENSION_DIR}" >/dev/null || return 1

    if [ ! -d node_modules ]; then
        if ! npm install; then
            popd >/dev/null
            echo "Could not install ROS Visualizer extension dependencies."
            return 1
        fi
    fi

    if ! npm run package; then
        popd >/dev/null
        echo "Could not package the ROS Visualizer extension."
        return 1
    fi
    popd >/dev/null

    if [ -f "${VISUALIZER_EXTENSION_PACKAGE}" ]; then
        return 0
    fi

    echo "Expected extension package was not created: ${VISUALIZER_EXTENSION_PACKAGE}"
    return 1
}

confirm_host_container_once() {
    if [ "${VISUALIZER_ASSUME_YES}" = "true" ] || [ -f "${VISUALIZER_NOTICE_FILE}" ]; then
        return 0
    fi

    echo "Starting the ROS Visualizer dashboard will create an additional container on the host Docker daemon:"
    echo "  container: ${VISUALIZER_CONTAINER}"
    echo "  image:     ${VISUALIZER_IMAGE}"
    echo "  port:      ${VISUALIZER_PORT}:8080"
    echo ""

    if [ ! -t 0 ]; then
        echo "No interactive terminal is available for confirmation."
        echo "Set VISUALIZER_ASSUME_YES=true to acknowledge this notice non-interactively."
        return 1
    fi

    read -r -p "Continue? [y/N] " answer
    case "${answer}" in
        y|Y|yes|YES)
            mkdir -p "$(dirname "${VISUALIZER_NOTICE_FILE}")"
            touch "${VISUALIZER_NOTICE_FILE}"
            ;;
        *)
            echo "Cancelled."
            return 1
            ;;
    esac
}

lichtblick_container_running() {
    "${DOCKER_CMD[@]}" ps --format '{{.Names}}' | grep -qx "${VISUALIZER_CONTAINER}"
}

wait_for_lichtblick_container() {
    for _ in {1..60}; do
        if lichtblick_container_running \
            && "${DOCKER_CMD[@]}" exec "${VISUALIZER_CONTAINER}" sh -lc 'test -f /src/index.html' >/dev/null 2>&1; then
            return 0
        fi

        if [ -n "${lichtblick_pid}" ] && ! kill -0 "${lichtblick_pid}" 2>/dev/null; then
            wait "${lichtblick_pid}" 2>/dev/null || true
            return 1
        fi

        sleep 0.5
    done

    return 1
}

port_in_use() {
    local port="$1"

    ss -ltn 2>/dev/null | grep -qE ":${port}[[:space:]]"
}

stage_lichtblick_layout() {
    mkdir -p "${VISUALIZER_DOCKER_ROOT}"

    if [ -f "${VISUALIZER_LAYOUT}" ]; then
        python3 - "${VISUALIZER_LAYOUT}" "${VISUALIZER_DOCKER_ROOT}/visualizer_layout.json" "${VISUALIZER_LAYOUT_ID}" "${VISUALIZER_LAYOUT_NAME}" <<'PY'
import json
import sys
from datetime import datetime, timezone

layout_path, output_path, layout_id, layout_name = sys.argv[1:5]

with open(layout_path, "r", encoding="utf-8") as layout_file:
    layout_data = json.load(layout_file)

now = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
layout = {
    "id": layout_id,
    "name": layout_name,
    "permission": "CREATOR_WRITE",
    "baseline": {
        "data": layout_data,
        "savedAt": now,
    },
}

with open(output_path, "w", encoding="utf-8") as output_file:
    json.dump(layout, output_file, separators=(",", ":"))
PY
    else
        echo "ROS visualizer layout not found: ${VISUALIZER_LAYOUT}"
        echo "Starting without a custom visualizer layout."
    fi

    if [ -f "${VISUALIZER_BOOTSTRAP}" ]; then
        cp "${VISUALIZER_BOOTSTRAP}" "${VISUALIZER_DOCKER_ROOT}/visualizer.html"
    else
        echo "ROS visualizer bootstrap page not found: ${VISUALIZER_BOOTSTRAP}"
        return 1
    fi

    ensure_lichtblick_extension_package || return 1

    cp "${VISUALIZER_EXTENSION_PACKAGE}" "${VISUALIZER_DOCKER_ROOT}/hardware_interface_panels.foxe"

    python3 - "${VISUALIZER_EXTENSION_DIR}" "${VISUALIZER_EXTENSION_PACKAGE}" "${VISUALIZER_DOCKER_ROOT}/hardware_interface_panels_extension.json" <<'PY'
import json
import os
import re
import sys

extension_dir, package_path, output_path = sys.argv[1:4]

with open(os.path.join(extension_dir, "package.json"), "r", encoding="utf-8") as package_file:
    package_info = json.load(package_file)

def read_optional(name):
    path = os.path.join(extension_dir, name)
    if not os.path.isfile(path):
        return ""
    with open(path, "r", encoding="utf-8") as file:
        return file.read()

publisher = package_info.get("publisher", "")
normalized_publisher = re.sub(r"[^A-Za-z0-9_\s]+", "", publisher)
extension_id = f"{normalized_publisher}.{package_info['name']}"

info = dict(package_info)
info.update({
    "id": extension_id,
    "namespace": "local",
    "qualifiedName": package_info.get("displayName") or package_info["name"],
    "readme": read_optional("README.md"),
    "changelog": read_optional("CHANGELOG.md"),
    "externalId": package_info["name"],
    "size": os.path.getsize(package_path),
})

with open(output_path, "w", encoding="utf-8") as output_file:
    json.dump({"info": info}, output_file, separators=(",", ":"))
PY
}

start_lichtblick() {
    stage_lichtblick_layout

    "${DOCKER_CMD[@]}" rm -f "${VISUALIZER_CONTAINER}" >/dev/null 2>&1 || true

    "${DOCKER_CMD[@]}" create \
        --name "${VISUALIZER_CONTAINER}" \
        -p "${VISUALIZER_PORT}:8080" \
        "${VISUALIZER_IMAGE}" >/dev/null

    copy_lichtblick_visualizer_files

    "${DOCKER_CMD[@]}" start -a "${VISUALIZER_CONTAINER}" &
    lichtblick_pid=$!

    if wait_for_lichtblick_container; then
        return 0
    fi

    echo "Lichtblick container ${VISUALIZER_CONTAINER} did not start."
    echo "If Docker reports 'address already in use', check listeners on the Docker host."
    echo "In VS Code's Ports view, stop forwarding the selected Lichtblick HTTP port."
    echo "The HTTP server runs on host Docker; only the WebSocket bridge needs forwarding."
    echo "A port check inside the devcontainer cannot detect host-side listeners."
    return 1
}

copy_lichtblick_visualizer_files() {
    "${DOCKER_CMD[@]}" cp "${VISUALIZER_DOCKER_ROOT}/visualizer.html" "${VISUALIZER_CONTAINER}:/src/visualizer.html"

    if [ -f "${VISUALIZER_DOCKER_ROOT}/visualizer_layout.json" ]; then
        "${DOCKER_CMD[@]}" cp "${VISUALIZER_DOCKER_ROOT}/visualizer_layout.json" "${VISUALIZER_CONTAINER}:/src/visualizer_layout.json"
    fi

    "${DOCKER_CMD[@]}" cp "${VISUALIZER_DOCKER_ROOT}/hardware_interface_panels.foxe" "${VISUALIZER_CONTAINER}:/src/hardware_interface_panels.foxe"
    "${DOCKER_CMD[@]}" cp "${VISUALIZER_DOCKER_ROOT}/hardware_interface_panels_extension.json" "${VISUALIZER_CONTAINER}:/src/hardware_interface_panels_extension.json"
}

start_bridge() {
    if port_in_use "${VISUALIZER_BRIDGE_PORT}"; then
        echo "Bridge port ${VISUALIZER_BRIDGE_PORT} is already in use; reusing the existing WebSocket bridge."
        return 0
    fi

    ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
        address:="${VISUALIZER_BRIDGE_ADDRESS}" \
        port:="${VISUALIZER_BRIDGE_PORT}" \
        client_topic_whitelist:="${VISUALIZER_BRIDGE_CLIENT_TOPIC_WHITELIST}" \
        remote_access:=false &
    bridge_pid=$!
}

cleanup_stale_description_processes() {
    pkill -f "${VISUALIZER_ROS_WS}/install/lars_description/lib/lars_description/mock_joint_states.py" 2>/dev/null || true
    pkill -f "${VISUALIZER_ROS_WS}/install/lars_description/lib/lars_description/mock_scene_markers.py" 2>/dev/null || true
    pkill -f "${VISUALIZER_ROS_WS}/install/lars_description/lib/lars_description/imu_orientation_tf_bridge.py" 2>/dev/null || true
    pkill -f "${VISUALIZER_ROS_WS}/install/lars_description/lib/lars_description/steering_joint_state_bridge.py" 2>/dev/null || true
}

start_dummy_description() {
    if [ "${VISUALIZER_MODEL}" != "true" ]; then
        return 0
    fi

    ensure_robot_state_publisher_available || return 1
    source_ros_environment
    cleanup_stale_description_processes

    ros2 launch lars_description dummy_visualization.launch.py &
    description_pid=$!
}

cleanup() {
    if [ -n "${description_pid}" ]; then
        kill "${description_pid}" 2>/dev/null || true
        wait "${description_pid}" 2>/dev/null || true
    fi

    if [ -n "${bridge_pid}" ]; then
        kill "${bridge_pid}" 2>/dev/null || true
        wait "${bridge_pid}" 2>/dev/null || true
    fi

    if [ -n "${lichtblick_pid}" ]; then
        "${DOCKER_CMD[@]}" rm -f "${VISUALIZER_CONTAINER}" >/dev/null 2>&1 || true
        wait "${lichtblick_pid}" 2>/dev/null || true
    fi

}

while [ $# -gt 0 ]; do
    case "$1" in
        --no-open)
            VISUALIZER_OPEN=false
            ;;
        --lichtblick-port)
            shift
            VISUALIZER_PORT="$1"
            ;;
        --bridge-port)
            shift
            VISUALIZER_BRIDGE_PORT="$1"
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

trap cleanup EXIT INT TERM

ensure_ros_overlay
source_ros_environment
ensure_foxglove_bridge_available
ensure_docker_available
confirm_host_container_once

echo "Starting ROS Visualizer dashboard"
echo "  workspace:       ${VISUALIZER_ROS_WS}"
echo "  image:           ${VISUALIZER_IMAGE}"
echo "  layout:          ${VISUALIZER_LAYOUT}"
echo "  HTTP host port:  ${VISUALIZER_PORT}"
echo "  bridge port:     ${VISUALIZER_BRIDGE_PORT}"
echo "  publish topics:  ${VISUALIZER_BRIDGE_CLIENT_TOPIC_WHITELIST}"
echo "  dummy model:     ${VISUALIZER_MODEL}"
echo ""

start_lichtblick
start_dummy_description
start_bridge
# Emit browser URLs only after Docker has bound the host HTTP port.
open_url_best_effort "$(lichtblick_startup_url)"

if [ -n "${bridge_pid}" ]; then
    wait "${bridge_pid}"
elif [ -n "${lichtblick_pid}" ]; then
    wait "${lichtblick_pid}"
fi
