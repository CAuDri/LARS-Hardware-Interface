#!/bin/bash

# CAuDri - Bash script adding custom udev rules to your host system
# This will grant serial device permissions for users in the dev container and add symlinks for known devices.
# This script is intended to be run on the host system, not in the container!
# It will be executed from the windows devcontainer setup script.

# Exit on error
set -e

# Name of the udev rules file 
UDEV_RULE_FILE="99-hardware-interface.rules"

# Path to the udev rules directory
UDEV_DIR="/etc/udev/rules.d"

# Check if the script is being run on the host system
if [ -f /.dockerenv ]; then
    echo -e "\e[31mThis script is intended to be run on the host system, running it in the container will have no effect.\e[0m"
    echo "\e[31mExiting...\e[0m"
    exit 1
fi

# Check if the udev rule is already present on the system
if [ -f "${UDEV_DIR}/${UDEV_RULE_FILE}" ]; then
    echo "The udev rule is already present on your system."
    exit 1
fi

# Ask the user if they want to add the udev rules to their system
echo -e "This script will add a udev rule to your host system to allow the devcontainer to mount serial devices."
echo -e "\033[1mDo you want to continue? (y/n)\033[0m"
read -p "" answer
case ${answer:0:1} in
    y|Y )
        echo -e "\nAdding udev rules to your system..."

        # Create the udev rules directory if it does not exist
        if [ ! -d "${UDEV_DIR}" ]; then
            echo "Creating udev rules directory..."
            sudo mkdir -p "${UDEV_DIR}"
        fi

        # Copy the udev rules file to the udev rules directory
        sudo cp ".devcontainer/udev/${UDEV_RULE_FILE}" "${UDEV_DIR}"

        # Reload the udev rules
        sudo udevadm control --reload-rules 
        sudo udevadm trigger

        echo -e "\n\e[32mUdev rules added successfully.\e[0m"
        exit 0
        ;;
    * )
        echo -e "\nIf you want to mount serial devices in the devcontainer (for flashing/debugging), you will need to add the udev rules manually or rerun this script."
        echo -e "Alternativley, you can mount the full /dev directory in the devcontainer, this is not recommended and might cause issues with other devices on your host system."
        echo -e "\nExiting..."
        exit 1
        ;;
esac