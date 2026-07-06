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

## Follow-up work

- Export the real LARS URDF/CAD model.
- Add meshes/materials to `lars_description`.
- Keep stable base frames where possible:
  - `base_link`
  - `chassis_link`
  - wheel links
  - steering/servo links
  - `imu_link`
  - ToF/sensor frames
- Add dynamic joint support for steering angles and wheel motion once those
  states are available from the hardware interface.
- Update RViz/Lichtblick layouts if frame or topic names change.
