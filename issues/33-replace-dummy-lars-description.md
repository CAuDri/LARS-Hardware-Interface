# Replace dummy `lars_description` with exported LARS URDF

The current `lars_description` package contains a temporary dummy URDF so RViz
and Lichtblick can show a working 3D vehicle model before the real geometry is
available.

## Current state

- Temporary package: `.devcontainer/tools/ros_packages/lars_description`
- Dummy model: `urdf/lars_dummy.urdf`
- Launch file: `launch/dummy_visualization.launch.py`
- The Lichtblick launcher starts this model by default with
  `LARS_LICHTBLICK_DUMMY_MODEL=true`.
- The dummy launch maps real `/hardware/measure/steering_angle_front` feedback
  onto front wheel joint states for visualization. It must not publish fake
  servo angle or motor speed data.

## Follow-up work

- Export the real LARS URDF/CAD model.
- Add meshes/materials to `lars_description`.
- Keep stable base frames where possible:
  - `base_link`
  - wheel links
  - `imu_link`
  - ToF/sensor frames
- Replace the temporary simplified geometry with the real model once available.
- Update RViz/Lichtblick layouts if frame or topic names change.
