# Hardware Interface Panels

Custom panels for the ROS Visualizer dashboard.

These panels add the controls that are awkward to build with Lichtblick's
generic panels alone: a compact hardware header, teleop controls that publish
periodically while overrides are enabled, and quick light/blinker commands.

## Telemetry panels

Each telemetry panel has independent topic/field settings, units, scaling,
ranges, decimal precision, colors and optional high-value warning thresholds.
Fields accept dot paths and array indices; they do not evaluate FoxQL expressions.
Missing, invalid or stale values appear as a dash. Timeout uses data-source time
so paused playback does not expire the display. A zero timeout disables expiry.

- `RPM`: plain ring with a centered value and unit.
- `Temperature`: horizontal scale, optionally vertical.
- `Thermometer`: vertical column with a bulb, numeric value and threshold marks.
- `Steering`: the upper colored marker indicates the actual angle; the
  simple wheel rotates to the target angle. A zero-centered scale compares both.
  Value scales convert source angles to degrees; wheel rotation scaling and
  inversion affect only the drawing, not the displayed measurements.
- `Current & Voltage`: two stacked linear scales. Current grows from zero
  in either direction. Voltage is displayed as voltage, not state of charge.

The default layout uses these panels for steering and motor telemetry. They add
no sparklines or separate plots. Existing analysis plots remain in the Motor tab.

## Panels

- `Header`: shows board heartbeat state from `/hardware/heartbeat` and
  provides `Reset` and `Emergency Stop` service buttons.
- `Teleop`: publishes steering and motor override commands at a configurable
  rate. Sliders and number fields stay in sync, RPM/current modes are mutually
  exclusive, and `Stop override` sends zero commands before disabling overrides.
- `Lights`: sends one-shot light and blinker command messages.

Panel settings are available in Lichtblick for topic names, message types,
limits, publish rate, and light command payloads.

The extension is packaged automatically by
`.devcontainer/script/container_start_visualizer.sh` before the dashboard
opens.
