# CAuDri - Dockerfile for setting up the Devcontainer
#

# ROS 2 base image for a full ROS 2 installation
FROM ros:jazzy

# For Ubuntu versions >= 24.04 you should remove the default user to avoid UID conflicts
RUN userdel -r ubuntu

ARG USERNAME=caudri
ARG USER_UID=1000
ARG USER_GID=$USER_UID 

# Install all necessary dependencies
RUN apt-get update && apt-get install -y \
    # Common tools
    sudo \
    git \
    curl \
    udev \
    rsync \
    dos2unix \
    # Build tools
    build-essential \
    cmake \
    gcc-arm-none-eabi \
    gdb-multiarch \
    openocd \
    dfu-util \
    clang-format 
# && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install the micro-ROS setup package (copied from the microros:base Dockerfile)
RUN mkdir -p uros_ws
WORKDIR /uros_ws
RUN git clone --depth 1 -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup \
    &&  . /opt/ros/$ROS_DISTRO/setup.sh \
    &&  apt update \
    &&  apt install -y ed python3-pip \
    &&  apt install -y ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    &&  apt remove -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    &&  apt autoremove -y \
    &&  rosdep update --rosdistro $ROS_DISTRO\
    &&  rosdep install --from-paths src --ignore-src -y \
    &&  colcon build \
    &&  rm -rf log/ build/ src/* \
    &&  rm -rf /var/lib/apt/lists/*

# Create a non-root user with home directory and sudo privileges
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -a -G dialout,plugdev $USERNAME

# Set the working directory to the user's home directory
WORKDIR /home/$USERNAME

# Create a directory for the repository
RUN mkdir -p /hardware_interface \
    && chown -R $USERNAME:$USERNAME /hardware_interface

# Source ROS2 and micro-ROS packages and set library paths
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /uros_ws/install/local_setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /uros_ws/install/micro_ros_setup/share/micro_ros_setup/local_setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/uros_ws/install/micro_ros_msgs/lib:/uros_ws/install/micro_ros_agent/lib" >> /home/$USERNAME/.bashrc

# Source the build script in the user's .bashrc
RUN echo "source /hardware_interface/firmware/build.sh" >> /home/$USERNAME/.bashrc

# Fix windows line endings and file system monitoring
RUN git config --global core.autocrlf input \
    # Disable file system monitoring due to unusual warnings in WSL based containers
    # Might slow down certain git operations, omit if necessary
    && git config --global core.fsMonitor "false"

# Switch to the created user and set up the environment
USER $USERNAME
ENV SHELL=/bin/bash
CMD ["/bin/bash"]
ENTRYPOINT []