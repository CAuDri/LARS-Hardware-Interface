#!/usr/bin/env bash

# CAuDri - Select an STM32CubeMX .ioc file and open it in STM32CubeMX.
#
# A native STM32CubeMX installation is preferred. If none is found, the script
# can install Flatpak and the community-maintained Flathub CubeMX package after
# asking for permission.

set -u

readonly CUBEMX_APP_ID="com.st.STM32CubeMX"
readonly CUBEMX_FLATHUB_URL="https://flathub.org/apps/com.st.STM32CubeMX"
readonly COLOR_YELLOW='\033[33m'
readonly COLOR_RED='\033[31m'
readonly COLOR_RESET='\033[0m'

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "${script_dir}/../.." && pwd)"

print_error() {
    printf '%bError: %s%b\n' "${COLOR_RED}" "$*" "${COLOR_RESET}" >&2
}

print_info() {
    printf '%b%s%b\n' "${COLOR_YELLOW}" "$*" "${COLOR_RESET}"
}

confirm() {
    local answer=""
    printf '%b%s [y/N] %b' "${COLOR_YELLOW}" "$1" "${COLOR_RESET}"
    read -r answer
    [[ "${answer}" == "y" || "${answer}" == "Y" || "${answer}" == "yes" || "${answer}" == "YES" ]]
}

find_native_cubemx() {
    local candidate

    if command -v STM32CubeMX >/dev/null 2>&1; then
        command -v STM32CubeMX
        return 0
    fi

    for candidate in \
        "${STM32CubeMX_PATH:-}" \
        "${HOME}/STM32CubeMX/STM32CubeMX" \
        "/opt/STMicroelectronics/STM32Cube/STM32CubeMX/STM32CubeMX" \
        "/opt/st/stm32cubemx/STM32CubeMX"; do
        if [[ -d "${candidate}" ]]; then
            candidate="${candidate}/STM32CubeMX"
        fi
        if [[ -n "${candidate}" && -x "${candidate}" ]]; then
            printf '%s\n' "${candidate}"
            return 0
        fi
    done

    return 1
}

install_flatpak() {
    print_info ""
    print_info "Flatpak is required to install STM32CubeMX from Flathub."
    print_info "Package information: ${CUBEMX_FLATHUB_URL}"
    printf '\n'

    if ! confirm "Install Flatpak in this dev container?"; then
        return 1
    fi

    if ! command -v sudo >/dev/null 2>&1; then
        print_error "sudo is unavailable; install Flatpak manually and retry."
        return 1
    fi

    sudo apt-get update && sudo apt-get install -y flatpak
}

ensure_dbus() {
    if [[ -S /run/dbus/system_bus_socket ]]; then
        return 0
    fi

    if ! command -v dbus-daemon >/dev/null 2>&1; then
        print_error "dbus-daemon is unavailable; install the dbus package and retry."
        return 1
    fi
    if ! command -v sudo >/dev/null 2>&1; then
        print_error "sudo is unavailable; a D-Bus system bus cannot be started."
        return 1
    fi

    print_info "Starting an isolated D-Bus system bus inside the dev container..."
    sudo mkdir -p /run/dbus
    sudo dbus-daemon --system --fork
}

run_flatpak() {
    if [[ -n "${DBUS_SESSION_BUS_ADDRESS:-}" ]]; then
        flatpak "$@"
        return $?
    fi

    if ! command -v dbus-run-session >/dev/null 2>&1; then
        print_error "dbus-run-session is unavailable; install the dbus package and retry."
        return 1
    fi

    dbus-run-session -- flatpak "$@"
}

install_cubemx_flatpak() {
    print_info ""
    print_info "STM32CubeMX is available through an unverified, community-maintained Flathub wrapper."
    print_info "Review it here before continuing: ${CUBEMX_FLATHUB_URL}"
    printf '\n'

    if ! confirm "Install the latest STM32CubeMX package for this container user?"; then
        return 1
    fi

    run_flatpak remote-add --user --if-not-exists flathub \
        https://dl.flathub.org/repo/flathub.flatpakrepo && \
        run_flatpak install --user --noninteractive -y flathub "${CUBEMX_APP_ID}"
}

select_ioc_file() {
    local -n selected_output=$1
    local choice
    local candidate_ioc
    local -a ioc_files=()

    while IFS= read -r -d '' candidate_ioc; do
        ioc_files+=("${candidate_ioc}")
    done < <(find "${repo_dir}/firmware/devices" -type f -name '*.ioc' -print0 | sort -z)

    if (( ${#ioc_files[@]} == 0 )); then
        print_error "no .ioc files were found below ${repo_dir}/firmware/devices."
        return 1
    fi

    print_info "Select the STM32CubeMX project to open:"
    PS3="$(printf '%bSelection: %b' "${COLOR_YELLOW}" "${COLOR_RESET}")"
    select choice in "${ioc_files[@]}" "Cancel"; do
        if [[ "${choice:-}" == "Cancel" ]]; then
            return 1
        fi
        if [[ -n "${choice:-}" ]]; then
            selected_output="${choice}"
            return 0
        fi
        print_info "Please enter a number from 1 to $(( ${#ioc_files[@]} + 1 ))."
    done
}

find_device_ioc() {
    local -n selected_output=$1
    local device_name=$2
    local device_dir="${repo_dir}/firmware/devices/${device_name}"
    local candidate_ioc
    local -a ioc_files=()

    if [[ ! -d "${device_dir}" ]]; then
        print_error "device directory does not exist: ${device_dir}"
        return 1
    fi

    while IFS= read -r -d '' candidate_ioc; do
        ioc_files+=("${candidate_ioc}")
    done < <(find "${device_dir}" -type f -name '*.ioc' -print0 | sort -z)

    if (( ${#ioc_files[@]} == 0 )); then
        print_error "no .ioc file was found for device ${device_name}."
        return 1
    fi
    if (( ${#ioc_files[@]} > 1 )); then
        print_error "multiple .ioc files were found for device ${device_name}; run the script directly to select one."
        return 1
    fi

    selected_output="${ioc_files[0]}"
}

check_display() {
    if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
        print_error "no graphical display was forwarded into the dev container."
        printf 'Reopen the project in VS Code and verify that GUI application forwarding is enabled.\n' >&2
        return 1
    fi
}

main() {
    local ioc_file=""
    local native_cubemx=""
    local device_name=""

    if [[ "${1:-}" == "--list" ]]; then
        find "${repo_dir}/firmware/devices" -type f -name '*.ioc' -print | sort
        return 0
    fi

    if [[ "${1:-}" == "--device" ]]; then
        if [[ -z "${2:-}" ]]; then
            print_error "--device requires a device directory name."
            return 1
        fi
        device_name=$2
    elif [[ -n "${1:-}" ]]; then
        print_error "unknown argument: $1"
        return 1
    fi

    check_display || return 1
    if [[ -n "${device_name}" ]]; then
        find_device_ioc ioc_file "${device_name}" || return 1
    else
        select_ioc_file ioc_file || return 0
    fi

    if native_cubemx="$(find_native_cubemx)"; then
        print_info "Opening ${ioc_file} with ${native_cubemx}"
        "${native_cubemx}" "${ioc_file}"
        return $?
    fi

    if ! command -v flatpak >/dev/null 2>&1; then
        install_flatpak || {
            print_info "STM32CubeMX was not installed."
            return 0
        }
    fi

    ensure_dbus || return 1

    if ! run_flatpak info --user "${CUBEMX_APP_ID}" >/dev/null 2>&1 && \
       ! run_flatpak info --system "${CUBEMX_APP_ID}" >/dev/null 2>&1; then
        install_cubemx_flatpak || {
            print_info "STM32CubeMX was not installed."
            return 0
        }
    fi

    print_info "Opening ${ioc_file} with STM32CubeMX from Flathub"
    run_flatpak run --filesystem="${repo_dir}" "${CUBEMX_APP_ID}" "${ioc_file}"
}

main "$@"
