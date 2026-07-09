#!/bin/bash

# CAuDri - Helper script for starting the local LARS Lichtblick dashboard.
# Lichtblick provides the browser UI, while foxglove_bridge exposes the ROS 2
# graph through a local WebSocket connection.

set -e
set -o pipefail

LARS_ROS_WS="${LARS_ROS_WS:-/home/caudri/lars_ros_ws}"
LARS_LICHTBLICK_IMAGE="${LARS_LICHTBLICK_IMAGE:-ghcr.io/lichtblick-suite/lichtblick:latest}"
LARS_LICHTBLICK_CONTAINER="${LARS_LICHTBLICK_CONTAINER:-lars-lichtblick}"
LARS_LICHTBLICK_PORT="${LARS_LICHTBLICK_PORT:-8080}"
LARS_LICHTBLICK_OPEN="${LARS_LICHTBLICK_OPEN:-true}"
LARS_LICHTBLICK_AUTO_CONNECT="${LARS_LICHTBLICK_AUTO_CONNECT:-true}"
LARS_LICHTBLICK_LAYOUT="${LARS_LICHTBLICK_LAYOUT:-/hardware_interface/.devcontainer/tools/lichtblick/lars_default_layout.json}"
LARS_LICHTBLICK_LAYOUT_NAME="${LARS_LICHTBLICK_LAYOUT_NAME:-LARS Visualizer}"
LARS_LICHTBLICK_LAYOUT_ID="${LARS_LICHTBLICK_LAYOUT_ID:-lars_visualizer}"
LARS_LICHTBLICK_BOOTSTRAP="${LARS_LICHTBLICK_BOOTSTRAP:-/hardware_interface/.devcontainer/tools/lichtblick/lars_visualizer.html}"
LARS_LICHTBLICK_EXTENSION_DIR="${LARS_LICHTBLICK_EXTENSION_DIR:-/hardware_interface/.devcontainer/tools/lichtblick/extensions/lars-lichtblick-panels}"
LARS_LICHTBLICK_EXTENSION_PACKAGE="${LARS_LICHTBLICK_EXTENSION_PACKAGE:-${LARS_LICHTBLICK_EXTENSION_DIR}/lars.lars-lichtblick-panels-0.1.0.foxe}"
LARS_BRIDGE_ADDRESS="${LARS_BRIDGE_ADDRESS:-0.0.0.0}"
LARS_BRIDGE_PORT="${LARS_BRIDGE_PORT:-8765}"
LARS_BRIDGE_BROWSER_HOST="${LARS_BRIDGE_BROWSER_HOST:-localhost}"
LARS_BRIDGE_CLIENT_TOPIC_WHITELIST="${LARS_BRIDGE_CLIENT_TOPIC_WHITELIST:-['^/hardware/command/.*$']}"
LARS_LICHTBLICK_DUMMY_MODEL="${LARS_LICHTBLICK_DUMMY_MODEL:-true}"
LARS_LICHTBLICK_DOCKER_ROOT="${LARS_LICHTBLICK_DOCKER_ROOT:-/tmp/lars-lichtblick-docker}"
LARS_LICHTBLICK_NOTICE_FILE="${LARS_LICHTBLICK_NOTICE_FILE:-${HOME}/.cache/lars-lichtblick-host-container-notice}"
LARS_LICHTBLICK_ASSUME_YES="${LARS_LICHTBLICK_ASSUME_YES:-false}"

lichtblick_pid=""
bridge_pid=""
description_pid=""
DOCKER_CMD=()

show_help() {
    echo "Usage: container_start_lars_lichtblick.sh [--no-open] [--lichtblick-port PORT] [--bridge-port PORT]"
    echo ""
    echo "Options:"
    echo "  --no-open              Start servers and do not open the browser."
    echo "  --lichtblick-port PORT Local Lichtblick HTTP port (default: ${LARS_LICHTBLICK_PORT})."
    echo "  --bridge-port PORT     Foxglove WebSocket bridge port (default: ${LARS_BRIDGE_PORT})."
    echo "  -h, --help             Show this help text."
    echo ""
    echo "Environment:"
    echo "  LARS_ROS_WS                         Host-side ROS workspace with LARS messages."
    echo "  LARS_LICHTBLICK_IMAGE               Lichtblick container image."
    echo "  LARS_LICHTBLICK_LAYOUT              Layout data used for the LARS visualizer."
    echo "  LARS_LICHTBLICK_LAYOUT_NAME         Display name for the LARS layout."
    echo "  LARS_LICHTBLICK_LAYOUT_ID           Stable local ID for the LARS layout."
    echo "  LARS_LICHTBLICK_EXTENSION_DIR       Local custom Lichtblick extension source."
    echo "  LARS_LICHTBLICK_OPEN                Set to false to skip opening the browser."
    echo "  LARS_LICHTBLICK_AUTO_CONNECT        Set to false to skip the startup bridge URL."
    echo "  LARS_LICHTBLICK_ASSUME_YES          Set to true to skip the first-run host container prompt."
    echo "  LARS_BRIDGE_BROWSER_HOST            Hostname the browser should use for the bridge."
    echo "  LARS_BRIDGE_CLIENT_TOPIC_WHITELIST  Topics the web UI may publish to."
    echo "  LARS_LICHTBLICK_DUMMY_MODEL         Set to false to skip the temporary URDF model."
}

source_base_ros_environment() {
    local distro="${ROS_DISTRO:-jazzy}"

    if [ ! -f "/opt/ros/${distro}/setup.bash" ]; then
        echo "ROS 2 setup not found for ROS_DISTRO=${distro}."
        return 1
    fi

    source "/opt/ros/${distro}/setup.bash"
}

ensure_lars_ros_workspace_layout() {
    mkdir -p "${LARS_ROS_WS}/src"

    local lars_msgs_source="/hardware_interface/firmware/libraries/lars_ros_interfaces/lars_msgs"
    local lars_description_source="/hardware_interface/.devcontainer/tools/ros_packages/lars_description"

    if [ ! -d "${lars_msgs_source}" ]; then
        echo "Required LARS ROS package not found: ${lars_msgs_source}"
        return 1
    fi
    if [ ! -d "${lars_description_source}" ]; then
        echo "Required LARS ROS package not found: ${lars_description_source}"
        return 1
    fi

    if [ -e "${LARS_ROS_WS}/src/lars_msgs" ] && [ ! -L "${LARS_ROS_WS}/src/lars_msgs" ]; then
        echo "Cannot link ${lars_msgs_source} into ${LARS_ROS_WS}/src/lars_msgs; the path already exists and is not a symlink."
        return 1
    fi
    if [ -e "${LARS_ROS_WS}/src/lars_description" ] && [ ! -L "${LARS_ROS_WS}/src/lars_description" ]; then
        echo "Cannot link ${lars_description_source} into ${LARS_ROS_WS}/src/lars_description; the path already exists and is not a symlink."
        return 1
    fi

    ln -sfn "${lars_msgs_source}" "${LARS_ROS_WS}/src/lars_msgs"
    ln -sfn "${lars_description_source}" "${LARS_ROS_WS}/src/lars_description"
}

ensure_lars_ros_overlay() {
    source_base_ros_environment
    ensure_lars_ros_workspace_layout

    if [ -f "${LARS_ROS_WS}/install/lars_msgs/share/lars_msgs/package.bash" ] \
        && [ -f "${LARS_ROS_WS}/install/lars_description/share/lars_description/package.bash" ]; then
        return 0
    fi

    echo "Building LARS ROS overlay in ${LARS_ROS_WS}"
    pushd "${LARS_ROS_WS}" >/dev/null
    colcon build --symlink-install --packages-select lars_msgs lars_description
    popd >/dev/null
}

source_ros_environment() {
    source_base_ros_environment

    # The bridge needs the workspace overlay to resolve custom lars_msgs schemas.
    if [ -f "${LARS_ROS_WS}/install/setup.bash" ]; then
        source "${LARS_ROS_WS}/install/setup.bash"
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

    if [ "${LARS_LICHTBLICK_OPEN}" != "true" ]; then
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
    local base_url="http://localhost:${LARS_LICHTBLICK_PORT}"
    local path="/lars_visualizer.html"

    if [ "${LARS_LICHTBLICK_AUTO_CONNECT}" != "true" ]; then
        echo "${base_url}${path}"
        return 0
    fi

    local bridge_url="ws://${LARS_BRIDGE_BROWSER_HOST}:${LARS_BRIDGE_PORT}"

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
        echo "  ${LARS_LICHTBLICK_LAYOUT}"
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

    echo "Node.js >= 18.18 and npm are required to package the LARS Lichtblick extension."
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
    if [ ! -f "${LARS_LICHTBLICK_EXTENSION_PACKAGE}" ]; then
        return 0
    fi

    find "${LARS_LICHTBLICK_EXTENSION_DIR}/src" \
        "${LARS_LICHTBLICK_EXTENSION_DIR}/package.json" \
        "${LARS_LICHTBLICK_EXTENSION_DIR}/tsconfig.json" \
        -newer "${LARS_LICHTBLICK_EXTENSION_PACKAGE}" -print -quit | grep -q .
}

ensure_lichtblick_extension_package() {
    if [ ! -d "${LARS_LICHTBLICK_EXTENSION_DIR}" ]; then
        echo "LARS Lichtblick extension source not found: ${LARS_LICHTBLICK_EXTENSION_DIR}"
        return 1
    fi

    if ! lichtblick_extension_package_stale; then
        return 0
    fi

    ensure_node_available || return 1

    echo "Packaging LARS Lichtblick extension in ${LARS_LICHTBLICK_EXTENSION_DIR}"
    pushd "${LARS_LICHTBLICK_EXTENSION_DIR}" >/dev/null

    if [ ! -d node_modules ]; then
        npm install
    fi

    npm run package
    popd >/dev/null

    if [ -f "${LARS_LICHTBLICK_EXTENSION_PACKAGE}" ]; then
        return 0
    fi

    echo "Expected extension package was not created: ${LARS_LICHTBLICK_EXTENSION_PACKAGE}"
    return 1
}

confirm_host_container_once() {
    if [ "${LARS_LICHTBLICK_ASSUME_YES}" = "true" ] || [ -f "${LARS_LICHTBLICK_NOTICE_FILE}" ]; then
        return 0
    fi

    echo "Starting the LARS Lichtblick dashboard will create an additional container on the host Docker daemon:"
    echo "  container: ${LARS_LICHTBLICK_CONTAINER}"
    echo "  image:     ${LARS_LICHTBLICK_IMAGE}"
    echo "  port:      ${LARS_LICHTBLICK_PORT}:8080"
    echo ""

    if [ ! -t 0 ]; then
        echo "No interactive terminal is available for confirmation."
        echo "Set LARS_LICHTBLICK_ASSUME_YES=true to acknowledge this notice non-interactively."
        return 1
    fi

    read -r -p "Continue? [y/N] " answer
    case "${answer}" in
        y|Y|yes|YES)
            mkdir -p "$(dirname "${LARS_LICHTBLICK_NOTICE_FILE}")"
            touch "${LARS_LICHTBLICK_NOTICE_FILE}"
            ;;
        *)
            echo "Cancelled."
            return 1
            ;;
    esac
}

lichtblick_container_running() {
    "${DOCKER_CMD[@]}" ps --format '{{.Names}}' | grep -qx "${LARS_LICHTBLICK_CONTAINER}"
}

wait_for_lichtblick_container() {
    for _ in {1..60}; do
        if lichtblick_container_running \
            && "${DOCKER_CMD[@]}" exec "${LARS_LICHTBLICK_CONTAINER}" sh -lc 'test -f /src/index.html' >/dev/null 2>&1; then
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
    mkdir -p "${LARS_LICHTBLICK_DOCKER_ROOT}"

    if [ -f "${LARS_LICHTBLICK_LAYOUT}" ]; then
        python3 - "${LARS_LICHTBLICK_LAYOUT}" "${LARS_LICHTBLICK_DOCKER_ROOT}/lars_visualizer_layout.json" "${LARS_LICHTBLICK_LAYOUT_ID}" "${LARS_LICHTBLICK_LAYOUT_NAME}" <<'PY'
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
        echo "LARS visualizer layout not found: ${LARS_LICHTBLICK_LAYOUT}"
        echo "Starting without a custom LARS layout."
    fi

    if [ -f "${LARS_LICHTBLICK_BOOTSTRAP}" ]; then
        cp "${LARS_LICHTBLICK_BOOTSTRAP}" "${LARS_LICHTBLICK_DOCKER_ROOT}/lars_visualizer.html"
    else
        echo "LARS visualizer bootstrap page not found: ${LARS_LICHTBLICK_BOOTSTRAP}"
        return 1
    fi

    ensure_lichtblick_extension_package || return 1

    cp "${LARS_LICHTBLICK_EXTENSION_PACKAGE}" "${LARS_LICHTBLICK_DOCKER_ROOT}/lars_lichtblick_panels.foxe"

    python3 - "${LARS_LICHTBLICK_EXTENSION_DIR}" "${LARS_LICHTBLICK_EXTENSION_PACKAGE}" "${LARS_LICHTBLICK_DOCKER_ROOT}/lars_lichtblick_panels_extension.json" <<'PY'
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

    "${DOCKER_CMD[@]}" rm -f "${LARS_LICHTBLICK_CONTAINER}" >/dev/null 2>&1 || true

    "${DOCKER_CMD[@]}" create \
        --name "${LARS_LICHTBLICK_CONTAINER}" \
        -p "${LARS_LICHTBLICK_PORT}:8080" \
        "${LARS_LICHTBLICK_IMAGE}" >/dev/null

    copy_lichtblick_visualizer_files

    "${DOCKER_CMD[@]}" start -a "${LARS_LICHTBLICK_CONTAINER}" &
    lichtblick_pid=$!

    if wait_for_lichtblick_container; then
        return 0
    fi

    echo "Lichtblick container ${LARS_LICHTBLICK_CONTAINER} did not start."
    return 1
}

copy_lichtblick_visualizer_files() {
    "${DOCKER_CMD[@]}" cp "${LARS_LICHTBLICK_DOCKER_ROOT}/lars_visualizer.html" "${LARS_LICHTBLICK_CONTAINER}:/src/lars_visualizer.html"

    if [ -f "${LARS_LICHTBLICK_DOCKER_ROOT}/lars_visualizer_layout.json" ]; then
        "${DOCKER_CMD[@]}" cp "${LARS_LICHTBLICK_DOCKER_ROOT}/lars_visualizer_layout.json" "${LARS_LICHTBLICK_CONTAINER}:/src/lars_visualizer_layout.json"
    fi

    "${DOCKER_CMD[@]}" cp "${LARS_LICHTBLICK_DOCKER_ROOT}/lars_lichtblick_panels.foxe" "${LARS_LICHTBLICK_CONTAINER}:/src/lars_lichtblick_panels.foxe"
    "${DOCKER_CMD[@]}" cp "${LARS_LICHTBLICK_DOCKER_ROOT}/lars_lichtblick_panels_extension.json" "${LARS_LICHTBLICK_CONTAINER}:/src/lars_lichtblick_panels_extension.json"
}

start_bridge() {
    if port_in_use "${LARS_BRIDGE_PORT}"; then
        echo "Bridge port ${LARS_BRIDGE_PORT} is already in use; reusing the existing WebSocket bridge."
        return 0
    fi

    ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
        address:="${LARS_BRIDGE_ADDRESS}" \
        port:="${LARS_BRIDGE_PORT}" \
        client_topic_whitelist:="${LARS_BRIDGE_CLIENT_TOPIC_WHITELIST}" \
        remote_access:=false &
    bridge_pid=$!
}

cleanup_stale_description_processes() {
    pkill -f "${LARS_ROS_WS}/install/lars_description/lib/lars_description/mock_joint_states.py" 2>/dev/null || true
    pkill -f "${LARS_ROS_WS}/install/lars_description/lib/lars_description/mock_scene_markers.py" 2>/dev/null || true
    pkill -f "${LARS_ROS_WS}/install/lars_description/lib/lars_description/steering_joint_state_bridge.py" 2>/dev/null || true
}

start_dummy_description() {
    if [ "${LARS_LICHTBLICK_DUMMY_MODEL}" != "true" ]; then
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
        "${DOCKER_CMD[@]}" rm -f "${LARS_LICHTBLICK_CONTAINER}" >/dev/null 2>&1 || true
        wait "${lichtblick_pid}" 2>/dev/null || true
    fi

}

while [ $# -gt 0 ]; do
    case "$1" in
        --no-open)
            LARS_LICHTBLICK_OPEN=false
            ;;
        --lichtblick-port)
            shift
            LARS_LICHTBLICK_PORT="$1"
            ;;
        --bridge-port)
            shift
            LARS_BRIDGE_PORT="$1"
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

ensure_lars_ros_overlay
source_ros_environment
ensure_foxglove_bridge_available
ensure_docker_available
confirm_host_container_once

echo "Starting LARS Lichtblick dashboard"
echo "  workspace:       ${LARS_ROS_WS}"
echo "  image:           ${LARS_LICHTBLICK_IMAGE}"
echo "  layout:          ${LARS_LICHTBLICK_LAYOUT}"
echo "  Lichtblick URL:  http://localhost:${LARS_LICHTBLICK_PORT}"
echo "  startup URL:     $(lichtblick_startup_url)"
echo "  bridge URL:      ws://${LARS_BRIDGE_BROWSER_HOST}:${LARS_BRIDGE_PORT}"
echo "  publish topics:  ${LARS_BRIDGE_CLIENT_TOPIC_WHITELIST}"
echo "  dummy model:     ${LARS_LICHTBLICK_DUMMY_MODEL}"
echo ""

start_lichtblick
start_dummy_description
start_bridge
open_url_best_effort "$(lichtblick_startup_url)"

if [ -n "${bridge_pid}" ]; then
    wait "${bridge_pid}"
elif [ -n "${lichtblick_pid}" ]; then
    wait "${lichtblick_pid}"
fi
