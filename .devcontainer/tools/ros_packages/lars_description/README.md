# LARS description

Temporary host-side robot description for the LARS debug visualizers.

The current URDF is a dummy model that gives RViz/Lichtblick a stable base and
wheel frame tree while the exported CAD/URDF model is not available yet. The
dummy launch starts `robot_state_publisher` and a small bridge that maps the
real `/hardware/measure/steering_angle_front` feedback topic onto the front
wheel steering joints. It does not publish fake servo feedback or motor speed.

Replace `urdf/lars_dummy.urdf` with the real LARS description, or add the real
model beside it and update `launch/dummy_visualization.launch.py`.

Frames:

- `base_link`
- `front_left_wheel_link`
- `front_right_wheel_link`
- `rear_left_wheel_link`
- `rear_right_wheel_link`
