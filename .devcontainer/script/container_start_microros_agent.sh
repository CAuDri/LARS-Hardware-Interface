#!/bin/bash

# CAuDri - Bash script for starting the micro-ROS agent with a selected serial device.
# In VS Code, you can use the "Connect micro-ROS Agent" action button to run this script.

build_micro_ros_agent() {
    echo "Building micro-ROS agent..."
    rosdep update --rosdistro $ROS_DISTRO

    # Create a workspace in th user's home directory
    mkdir -p ~/uros_ws/src

    # Source the ROS 2 environment
    source /opt/ros/$ROS_DISTRO/setup.bash

    # Get the absolute path of the bash script (can not be sourced)
    DIR="$( dirname -- "${BASH_SOURCE[0]}"; )";
    DIR="$( realpath -e -- "$DIR"; )";

    # Get the absolute path of the repository (Sorry)
    export REPO_PATH=$(cd ${DIR} && cd ../.. && pwd)

    # Check if the submodule directory is empty
    if [ ! -d "$REPO_PATH/libraries/kitcar-interfaces" ] || 
       [ -z "$(ls -A "$REPO_PATH/libraries/kitcar-interfaces" 2>/dev/null)" ]; then
        echo -e "\nThe kitcar-interfaces submodule is empty. Press enter to update..."
        read -n 1
        git submodule update --init --recursive
        if [ $? -ne 0 ]; then
            echo -e "\nFailed to update the kitcar-interfaces submodule. Continue anyway?"
            read -n 1
            exit 1
        fi
    fi

    # Copy the kitcar-interfaces submodule to the workspace
    cp -r $REPO_PATH/libraries/kitcar-interfaces ~/uros_ws/src
    if [ $? -ne 0 ]; then
        echo "\nFailed to copy the kitcar-interfaces submodule. Continue anyway?"
        read -n 1
        exit 1
    fi

    # Build all packages in the workspace
    pushd ~/uros_ws > /dev/null
    colcon build

    # Source the workspace
    source ~/uros_ws/install/setup.bash

    # Build the micro-ROS agent
    pushd ~/uros_ws > /dev/null
    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh

    echo "source ~/uros_ws/install/setup.bash" >> ~/.bashrc
    popd > /dev/null
}

# Function to choose an option from a menu
choose_from_menu() {
    local prompt="$1" outvar="$2"
    shift
    shift
    local options=("$@") cur=0 count=${#options[@]} index=0
    local esc=$(echo -en "\e") # cache ESC as test doesn't allow esc codes
    # Hide cursor
    tput civis
    printf "$prompt\n"
    while true
    do
        # list all options (option list is zero-based)
        index=0 
        for o in "${options[@]}"
        do
            if [ "$index" == "$cur" ]
            then echo -e " >\e[7m$o\e[0m" # mark & highlight the current option
            else echo "  $o"
            fi
            index=$(( $index + 1 ))
        done
        read -s -n3 key # wait for user to key in arrows or ENTER
        if [[ $key == $'\e[A' ]] # up arrow
        then cur=$(( $cur - 1 ))
            [ "$cur" -lt 0 ] && cur=0
        elif [[ $key == $'\e[B' ]] # down arrow
        then cur=$(( $cur + 1 ))
            [ "$cur" -ge $count ] && cur=$(( $count - 1 ))
        elif [[ $key == "" ]] # nothing, i.e the read delimiter - ENTER
        then break
        fi
        echo -en "\e[${count}A" # go up to the beginning to re-render
    done
    # Show cursor
    tput cnorm
    # Export the selection to the requested output variable
    printf -v $outvar "${options[$cur]}"
}

# Function to start the micro-ROS agent
start_micro_ros_agent() {
    # Trap SIGINT (Ctrl+C) to return to the menu
    trap "select_serial_device" SIGINT

    local device="$1"
    echo "Starting micro_ros_agent with device: $device"

    ros2 run micro_ros_agent micro_ros_agent serial --dev "$device" -b 921600

    # If ros2 command exits (e.g., due to Ctrl+C), return to menu
    select_serial_device
}

# Function to list and select serial devices
select_serial_device() {
    # Trap SIGINT (Ctrl+C) to exit the script
    trap "exit_script" SIGINT

    clear
    while true; do
        local devices=($(find /dev -name 'ttyUSB*' -or -name 'ttyACM*'))
        local device

        if [ ${#devices[@]} -eq 0 ]; then
            echo "No serial devices found. Please connect a device and try again."
            echo "Press any key to retry or CTRL+C to exit..."
            read -n 1
            continue
        fi

        choose_from_menu "Select a serial device to connect the micro-ROS agent [CTRL+C to quit]:" device "${devices[@]}"
        echo ""
        start_micro_ros_agent "$device"
    done
}

exit_script() {
    exit 0
}

### Entry point ####

# Always rebuild the agent if the "-c" flag is passed to the script
if [ "$1" == "-c" ]; then
    build_micro_ros_agent
fi

# Check if the uros_ws workspace is present in the user's home directory
if [ ! -d ~/uros_ws/install ]; then
    echo -e "Micro-ROS workspace not found. Building the micro-ROS agent...\n"
    sleep 1
    build_micro_ros_agent

    if [ $? -ne 0 ]; then
        echo "Build failed. Press any key to exit..."
        read -n 1
        exit 1
    fi
fi

source ~/uros_ws/install/setup.bash

# Check if the micro-ROS agent is already built
ros2 pkg list | grep micro_ros_agent > /dev/null
if [ $? -ne 0 ]; then
    echo "Micro-ROS agent not found. Starting the build process..."
    sleep 2
    build_micro_ros_agent

    if [ $? -ne 0 ]; then
        echo "Build failed. Press any key to exit..."
        read -n 1
        exit 1
    else
        source ~/uros_ws/install/setup.bash
    fi
fi

select_serial_device
