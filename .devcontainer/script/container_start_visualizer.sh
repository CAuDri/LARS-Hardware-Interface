#!/bin/bash

# CAuDri - Helper script for starting the local ROS Visualizer dashboard.
# Lichtblick provides the browser UI, while foxglove_bridge exposes the ROS 2
# graph through a local WebSocket connection.

set -e
set -o pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
readonly LICHTBLICK_DIR="${REPO_ROOT}/.devcontainer/tools/lichtblick"
readonly ROS_PACKAGE_DIR="${REPO_ROOT}/.devcontainer/tools/ros_packages"
readonly LICHTBLICK_WEB_HELPER="${LICHTBLICK_DIR}/lichtblick_web.py"
VISUALIZER_ROS_WS="${VISUALIZER_ROS_WS:-${HOME}/ros_visualizer_ws}"
VISUALIZER_VERSION="${VISUALIZER_VERSION:-1.29.1}"
VISUALIZER_WEB_ROOT="${VISUALIZER_WEB_ROOT:-${XDG_CACHE_HOME:-${HOME}/.cache}/ros-visualizer/${VISUALIZER_VERSION}}"
VISUALIZER_PORT="${VISUALIZER_PORT:-8080}"
VISUALIZER_OPEN="${VISUALIZER_OPEN:-true}"
VISUALIZER_AUTO_CONNECT="${VISUALIZER_AUTO_CONNECT:-true}"
VISUALIZER_LAYOUT="${VISUALIZER_LAYOUT:-${LICHTBLICK_DIR}/default_layout.json}"
VISUALIZER_LAYOUT_NAME="${VISUALIZER_LAYOUT_NAME:-ROS Visualizer}"
VISUALIZER_LAYOUT_ID="${VISUALIZER_LAYOUT_ID:-ros_visualizer}"
VISUALIZER_BOOTSTRAP="${VISUALIZER_BOOTSTRAP:-${LICHTBLICK_DIR}/visualizer.html}"
VISUALIZER_EXTENSION_DIR="${VISUALIZER_EXTENSION_DIR:-${LICHTBLICK_DIR}/extensions/hardware-interface-panels}"
VISUALIZER_EXTENSION_PACKAGE="${VISUALIZER_EXTENSION_PACKAGE:-${VISUALIZER_EXTENSION_DIR}/caudri.hardware-interface-panels-0.1.0.foxe}"
VISUALIZER_BRIDGE_ADDRESS="${VISUALIZER_BRIDGE_ADDRESS:-0.0.0.0}"
VISUALIZER_BRIDGE_PORT="${VISUALIZER_BRIDGE_PORT:-8765}"
VISUALIZER_BRIDGE_BROWSER_HOST="${VISUALIZER_BRIDGE_BROWSER_HOST:-localhost}"
VISUALIZER_BRIDGE_CLIENT_TOPIC_WHITELIST="${VISUALIZER_BRIDGE_CLIENT_TOPIC_WHITELIST:-['^/hardware/command/.*$']}"
VISUALIZER_MODEL="${VISUALIZER_MODEL:-true}"
VISUALIZER_RUN_ROOT=""

lichtblick_pid=""
ros_pids=()

show_help() {
    echo "Usage: ${0##*/} [--no-open] [--lichtblick-port PORT] [--bridge-port PORT]"
    echo ""
    echo "Options:"
    echo "  --no-open              Start servers and do not open the browser."
    echo "  --lichtblick-port PORT Local Lichtblick HTTP port (default: ${VISUALIZER_PORT})."
    echo "  --bridge-port PORT     Foxglove WebSocket bridge port (default: ${VISUALIZER_BRIDGE_PORT})."
    echo "  -h, --help             Show this help text."
    echo ""
    echo "Environment:"
    echo "  VISUALIZER_ROS_WS                                  ROS workspace for the visualizer and hardware messages."
    echo "  VISUALIZER_VERSION                                 Web release to download on first use (default: 1.29.1)."
    echo "  VISUALIZER_WEB_ROOT                                Directory for cached or preinstalled Lichtblick web assets."
    echo "  VISUALIZER_LAYOUT                                  Layout data used for the ROS visualizer."
    echo "  VISUALIZER_LAYOUT_NAME                             Display name for the visualizer layout."
    echo "  VISUALIZER_LAYOUT_ID                               Stable local ID for the visualizer layout."
    echo "  VISUALIZER_EXTENSION_DIR                           Local custom Lichtblick extension source."
    echo "  VISUALIZER_OPEN                                    Set to false to skip opening the browser."
    echo "  VISUALIZER_AUTO_CONNECT                            Set to false to skip the startup bridge URL."
    echo "  VISUALIZER_BRIDGE_BROWSER_HOST                     Hostname the browser should use for the bridge."
    echo "  VISUALIZER_BRIDGE_CLIENT_TOPIC_WHITELIST           Topics the web UI may publish to."
    echo "  VISUALIZER_MODEL                                   Set to false to skip the robot model and visualization bridges."
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

    local package source
    for package in lars_msgs lars_description lars_visualizer; do
        if [ "${package}" = lars_msgs ]; then
            source="${REPO_ROOT}/firmware/libraries/lars_ros_interfaces/lars_msgs"
        else
            source="${ROS_PACKAGE_DIR}/${package}"
        fi
        if [ ! -d "${source}" ]; then
            echo "Required ROS package not found: ${source}"
            return 1
        fi
        if [ -e "${VISUALIZER_ROS_WS}/src/${package}" ] && [ ! -L "${VISUALIZER_ROS_WS}/src/${package}" ]; then
            echo "Cannot link ${source}; ${VISUALIZER_ROS_WS}/src/${package} already exists and is not a symlink."
            return 1
        fi
        ln -sfn "${source}" "${VISUALIZER_ROS_WS}/src/${package}"
    done
}

ensure_ros_overlay() {
    ensure_ros_workspace_layout

    local packages=(lars_msgs)
    local required=("${VISUALIZER_ROS_WS}/install/lars_msgs/share/lars_msgs/package.bash")
    if [ "${VISUALIZER_MODEL}" = true ]; then
        packages+=(lars_description lars_visualizer)
        required+=(
            "${VISUALIZER_ROS_WS}/install/lars_description/share/lars_description/package.bash"
            "${VISUALIZER_ROS_WS}/install/lars_description/share/lars_description/launch/robot_state_publisher.launch.py"
            "${VISUALIZER_ROS_WS}/install/lars_description/share/lars_description/urdf/lars.urdf"
            "${VISUALIZER_ROS_WS}/install/lars_description/share/lars_description/meshes/base_link.dae"
            "${VISUALIZER_ROS_WS}/install/lars_visualizer/share/lars_visualizer/package.bash"
            "${VISUALIZER_ROS_WS}/install/lars_visualizer/share/lars_visualizer/launch/visualizer.launch.py"
            "${VISUALIZER_ROS_WS}/install/lars_visualizer/lib/lars_visualizer/imu_orientation_tf_bridge.py"
            "${VISUALIZER_ROS_WS}/install/lars_visualizer/lib/lars_visualizer/steering_joint_state_bridge.py"
        )
    fi

    local artifact
    for artifact in "${required[@]}"; do
        if [ ! -f "${artifact}" ]; then
            echo "Building visualizer ROS overlay in ${VISUALIZER_ROS_WS}"
            (cd "${VISUALIZER_ROS_WS}" && colcon build --symlink-install --packages-select "${packages[@]}")
            return
        fi
    done
}

install_packages() {
    echo "Installing optional visualizer dependencies: $*"
    sudo apt-get update && sudo apt-get install -y "$@"
}

ensure_ros_packages() {
    local packages=(foxglove_bridge)
    if [ "${VISUALIZER_MODEL}" = true ]; then
        packages+=(robot_state_publisher)
    fi

    local package
    local missing=()
    for package in "${packages[@]}"; do
        if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
            missing+=("ros-${ROS_DISTRO}-${package//_/-}")
        fi
    done
    if [ "${#missing[@]}" -gt 0 ]; then
        install_packages "${missing[@]}" || return 1
        for package in "${packages[@]}"; do
            ros2 pkg prefix "${package}" >/dev/null || return 1
        done
    fi
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

node_version_available() {
    command -v node >/dev/null 2>&1 || return 1
    node -e 'const [major, minor] = process.versions.node.split(".").map(Number); process.exit(major > 18 || (major === 18 && minor >= 18) ? 0 : 1)' >/dev/null 2>&1
}

ensure_node_available() {
    if node_version_available && command -v npm >/dev/null 2>&1; then
        return 0
    fi

    install_packages nodejs npm || return 1
    if ! node_version_available || ! command -v npm >/dev/null 2>&1; then
        echo "Node.js >= 18.18 and npm are required to package the ROS Visualizer extension."
        return 1
    fi
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
    (
        cd "${VISUALIZER_EXTENSION_DIR}" || exit 1
        if [ ! -d node_modules ]; then
            npm ci || exit 1
        fi
        npm run package
    ) || return 1

    if [ ! -f "${VISUALIZER_EXTENSION_PACKAGE}" ]; then
        echo "Expected extension package was not created: ${VISUALIZER_EXTENSION_PACKAGE}"
        return 1
    fi
}

port_in_use() {
    local port="$1"

    ss -ltn 2>/dev/null | grep -qE ":${port}[[:space:]]"
}

stage_lichtblick_layout() {
    mkdir -p "${VISUALIZER_RUN_ROOT}"

    if [ -f "${VISUALIZER_LAYOUT}" ]; then
        python3 - "${VISUALIZER_LAYOUT}" "${VISUALIZER_RUN_ROOT}/visualizer_layout.json" "${VISUALIZER_LAYOUT_ID}" "${VISUALIZER_LAYOUT_NAME}" <<'PY'
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
        cp "${VISUALIZER_BOOTSTRAP}" "${VISUALIZER_RUN_ROOT}/visualizer.html"
    else
        echo "ROS visualizer bootstrap page not found: ${VISUALIZER_BOOTSTRAP}"
        return 1
    fi

    ensure_lichtblick_extension_package || return 1

    cp "${VISUALIZER_EXTENSION_PACKAGE}" "${VISUALIZER_RUN_ROOT}/hardware_interface_panels.foxe"

    python3 - "${VISUALIZER_EXTENSION_DIR}" "${VISUALIZER_EXTENSION_PACKAGE}" "${VISUALIZER_RUN_ROOT}/hardware_interface_panels_extension.json" <<'PY'
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

ensure_lichtblick_web_assets() {
    python3 "${LICHTBLICK_WEB_HELPER}" install \
        --version "${VISUALIZER_VERSION}" \
        --directory "${VISUALIZER_WEB_ROOT}"
}

wait_for_lichtblick_http() {
    local attempt
    for attempt in {1..40}; do
        if ! kill -0 "${lichtblick_pid}" 2>/dev/null; then
            wait "${lichtblick_pid}" 2>/dev/null || true
            return 1
        fi
        if python3 - "${VISUALIZER_PORT}" <<'HTTP_CHECK'
import sys
from urllib.request import ProxyHandler, build_opener

try:
    opener = build_opener(ProxyHandler({}))
    with opener.open(f"http://127.0.0.1:{sys.argv[1]}/visualizer.html", timeout=0.5) as response:
        sys.exit(0 if response.status == 200 else 1)
except OSError:
    sys.exit(1)
HTTP_CHECK
        then
            return 0
        fi
        sleep 0.25
    done
    return 1
}

start_lichtblick() {
    if port_in_use "${VISUALIZER_PORT}"; then
        echo "HTTP port ${VISUALIZER_PORT} is already in use inside the devcontainer."
        echo "Stop that server or choose another port with --lichtblick-port."
        return 1
    fi

    ensure_lichtblick_web_assets
    VISUALIZER_RUN_ROOT=$(mktemp -d "${TMPDIR:-/tmp}/ros-visualizer.XXXXXX")
    stage_lichtblick_layout

    python3 "${LICHTBLICK_WEB_HELPER}" serve \
        --directory "${VISUALIZER_WEB_ROOT}" \
        --overlay "${VISUALIZER_RUN_ROOT}" \
        --bind 0.0.0.0 \
        --port "${VISUALIZER_PORT}" &
    lichtblick_pid=$!

    if wait_for_lichtblick_http; then
        return 0
    fi
    echo "Lichtblick HTTP server did not start on port ${VISUALIZER_PORT}."
    return 1
}

start_bridge() {
    if port_in_use "${VISUALIZER_BRIDGE_PORT}"; then
        echo "Bridge port ${VISUALIZER_BRIDGE_PORT} is already in use; reusing the existing WebSocket bridge."
        return 0
    fi

    # Background shell jobs inherit ignored SIGINT; restore it for ROS shutdown.
    env --default-signal=INT ros2 launch --noninteractive foxglove_bridge foxglove_bridge_launch.xml \
        address:="${VISUALIZER_BRIDGE_ADDRESS}" \
        port:="${VISUALIZER_BRIDGE_PORT}" \
        client_topic_whitelist:="${VISUALIZER_BRIDGE_CLIENT_TOPIC_WHITELIST}" \
        remote_access:=false &
    ros_pids+=("$!")
}

start_visualizer() {
    if [ "${VISUALIZER_MODEL}" != "true" ]; then
        return 0
    fi

    env --default-signal=INT ros2 launch --noninteractive lars_visualizer visualizer.launch.py &
    ros_pids+=("$!")
}

cleanup() {
    # SIGINT lets ros2 launch stop its nodes; SIGTERM can leave them orphaned.
    local pid
    for pid in "${ros_pids[@]}"; do
        kill -INT "${pid}" 2>/dev/null || true
        wait "${pid}" 2>/dev/null || true
    done

    if [ -n "${lichtblick_pid}" ]; then
        kill "${lichtblick_pid}" 2>/dev/null || true
        wait "${lichtblick_pid}" 2>/dev/null || true
    fi

    if [ -n "${VISUALIZER_RUN_ROOT}" ]; then
        rm -rf -- "${VISUALIZER_RUN_ROOT}"
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

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

source_base_ros_environment
ensure_ros_packages
ensure_ros_overlay
# The WebSocket bridge needs the overlay to resolve custom lars_msgs schemas.
source "${VISUALIZER_ROS_WS}/install/setup.bash"

echo "Starting ROS Visualizer"
echo "  workspace:       ${VISUALIZER_ROS_WS}"
echo "  web version:     ${VISUALIZER_VERSION}"
echo "  layout:          ${VISUALIZER_LAYOUT}"
echo "  HTTP port:       ${VISUALIZER_PORT}"
echo "  bridge port:     ${VISUALIZER_BRIDGE_PORT}"
echo "  publish topics:  ${VISUALIZER_BRIDGE_CLIENT_TOPIC_WHITELIST}"
echo "  robot model:     ${VISUALIZER_MODEL}"
echo ""

start_lichtblick
start_visualizer
start_bridge
# Both servers run in this devcontainer and are forwarded by VS Code.
open_url_best_effort "$(lichtblick_startup_url)"

wait -n "${lichtblick_pid}" "${ros_pids[@]}"
