# LARS ROS Visualizer

This folder contains the Lichtblick dashboard used for debugging the LARS
hardware interface from the devcontainer. It starts a browser-based ROS
visualizer, connects it to the local ROS graph, and loads the LARS dashboard
layout with the custom control panels.

Use this when you want the full ROS debug view: robot model, telemetry, plots,
logs, teleop controls, lights, and hardware operation buttons.

## Start It

Use the VS Code action button:

```text
ROS Visualizer
```

or run the launcher manually:

```bash
.devcontainer/script/container_start_lars_lichtblick.sh
```

The launcher starts:

- Lichtblick on `http://localhost:8080`
- `foxglove_bridge` on `ws://localhost:8765`
- the `LARS Visualizer` layout from `lars_default_layout.json`
- the custom `LARS Lichtblick Panels` extension

The browser opens at:

```text
http://localhost:8080/lars_visualizer.html?ds=foxglove-websocket&ds.url=ws://localhost:8765
```

That page selects the LARS layout and connects it to the local ROS bridge. The
vendor default layout is left alone.

## First Launch

Lichtblick runs as a sibling container on the host Docker daemon. The
devcontainer reaches it through the mounted Docker socket at
`/var/run/docker.sock`.

On first launch, the script asks before creating the host-side container
`lars-lichtblick`. The acknowledgement is stored at:

```text
~/.cache/lars-lichtblick-host-container-notice
```

For non-interactive use, set:

```bash
LARS_LICHTBLICK_ASSUME_YES=true
```

If the Docker socket is missing or unreachable, rebuild or recreate the
devcontainer so VS Code applies the socket mount.

The launcher installs a few tools only when needed:

- `ros-${ROS_DISTRO}-foxglove-bridge`
- the Docker CLI
- Node.js/npm for packaging the custom Lichtblick extension

These are intentionally not baked into the devcontainer image.

## Dashboard

The default `LARS Visualizer` workspace has four tabs:

- `Visualization`: 3D robot view, battery and temperature gauges, diagnostics,
  and ROS logs.
- `Teleop`: steering and motor override controls, light/blinker buttons, and
  command/feedback plots.
- `ROS Graph`: topic graph, parameters, diagnostics, and warning/error logs.
- `Motor`: motor command, feedback, voltage, temperature, and telemetry views.

The header panel shows whether `/hardware/heartbeat` is active and provides
compact `Reset` and `Emergency Stop` buttons. The services behind those buttons
are tracked in the hardware operations issue and still need firmware/ROS-side
implementation.

The custom panels expose topic names, message types, limits, publish rate, and
light command payloads in the Lichtblick panel settings. The current defaults
match the development namespace `/hardware`, publish teleop commands at 20 Hz,
and use the firmware-side safety checks as the final authority.

The 3D panel loads the temporary `lars_description` URDF from the ROS 2
`/robot_description` topic. In the layout this topic is enabled under the 3D
panel's `topics` config so Lichtblick treats it as a URDF renderable, not just
as a raw `std_msgs/String` topic.

## Files

- `lars_default_layout.json`: the LARS dashboard layout.
- `lars_visualizer.html`: small startup page that selects the LARS layout and
  extension before opening Lichtblick.
- `extensions/lars-lichtblick-panels`: custom Lichtblick panels used by the
  layout.
