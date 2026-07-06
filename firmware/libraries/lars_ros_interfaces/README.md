# LARS ROS interfaces

This directory contains ROS interface packages shared by the STM32 firmware,
micro-ROS generation flow, and host-side ROS tools.

## Packages

- `lars_msgs`: LARS-specific messages for firmware telemetry and autonomous
  control commands.

Prefer standard ROS messages when they describe the data cleanly. Add custom
interfaces here only when the vehicle semantics, field names, or units need to
be explicit on both the firmware and host side.

## Host-side overlay

The devcontainer visualization scripts build a host-side ROS overlay at:

```bash
/home/caudri/lars_ros_ws
```

They symlink `lars_msgs` into that workspace, run `colcon build`, and source
the overlay before starting tools that need custom message schemas. To do the
same manually:

```bash
mkdir -p /home/caudri/lars_ros_ws/src
ln -sfn /hardware_interface/firmware/libraries/lars_ros_interfaces/lars_msgs \
  /home/caudri/lars_ros_ws/src/lars_msgs

source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
cd /home/caudri/lars_ros_ws
colcon build --symlink-install --packages-select lars_msgs
source install/setup.bash
```

After sourcing the overlay, ROS CLI tools should resolve the interfaces:

```bash
ros2 interface show lars_msgs/msg/SteeringAngleCommand
```

## Changing interfaces

- Keep message field names semantic and unit-specific where possible.
- Include `std_msgs/Header` on telemetry and commands that need timing or
  freshness checks.
- Update the host-side overlay after message changes before launching
  Lichtblick, RQt, or other visualization tools.
- Treat message changes as a firmware/high-level-stack compatibility boundary
  until the interface package is moved to its final shared repository or
  submodule.
