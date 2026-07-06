# LARS description

Temporary host-side robot description for the LARS debug visualizers.

The current URDF is a simple dummy model that gives RViz/Lichtblick a stable
vehicle frame tree and visible chassis while the exported CAD/URDF model is not
available yet. Replace `urdf/lars_dummy.urdf` with the real LARS description, or
add the real model beside it and update `launch/dummy_visualization.launch.py`.

Frames:

- `base_link`
- `chassis_link`
- `front_left_wheel_link`
- `front_right_wheel_link`
- `rear_left_wheel_link`
- `rear_right_wheel_link`
- `front_left_servo_link`
- `front_right_servo_link`
