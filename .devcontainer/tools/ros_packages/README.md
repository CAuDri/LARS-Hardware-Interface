# ROS Visualizer packages

`lars_description` contains the robot URDF, ten colored DAE meshes referenced
by it, and a launch file for `robot_state_publisher`. The URDF root is `base_link`; the mesh scale converts
assembly-global millimetres to metres, and inverse visual placements preserve
the CAD mounting positions.

```bash
ros2 launch lars_description robot_state_publisher.launch.py
```

`lars_visualizer` provides the LARS model integration. It includes that launch file and starts the two visualization
bridges. 

```bash
ros2 launch lars_visualizer visualizer.launch.py
```

- `steering_joint_state_bridge.py` reads
  `/hardware/measure/steering_angle` (`lars_msgs/Float32Stamped`), converts
  degrees to radians and publishes the two front wheel joints on `/joint_states`
  at 20 Hz. It starts with neutral joints and holds the last valid feedback.
- `imu_orientation_tf_bridge.py` reads `/hardware/measure/imu` and publishes
  `imu_visualization_world -> base_link`. It publishes a neutral orientation
  at 20 Hz until the first valid IMU quaternion arrives, then uses IMU timestamps.

The bridges expose their topic/frame names as ROS parameters. Front steering
rotates about base Z at the CAD wheel origins; it is a visualization approximation,
not a calibrated steering model. The ToF links use optical X right, Y down,
Z forward. Their positions and mounting directions remain in the URDF.

The Lichtblick launcher links these packages and the firmware's `lars_msgs`
into `${VISUALIZER_ROS_WS:-$HOME/ros_visualizer_ws}` and builds the overlay on demand.
