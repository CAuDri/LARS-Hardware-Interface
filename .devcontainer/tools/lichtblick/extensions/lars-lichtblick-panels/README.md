# LARS Lichtblick Panels

Custom panels for the LARS ROS Visualizer dashboard.

These panels add the controls that are awkward to build with Lichtblick's
generic panels alone: a compact hardware header, teleop controls that publish
periodically while overrides are enabled, and quick light/blinker commands.

## Panels

- `LARS Header`: shows board heartbeat state from `/hardware/heartbeat` and
  provides `Reset` and `Emergency Stop` service buttons.
- `LARS Teleop`: publishes steering and motor override commands at a configurable
  rate. Sliders and number fields stay in sync, RPM/current modes are mutually
  exclusive, and `Stop override` sends zero commands before disabling overrides.
- `LARS Lights`: sends one-shot light and blinker command messages.

Panel settings are available in Lichtblick for topic names, message types,
limits, publish rate, and light command payloads.

The extension is packaged automatically by
`.devcontainer/script/container_start_lars_lichtblick.sh` before the dashboard
opens.
