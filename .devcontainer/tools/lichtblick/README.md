# ROS Visualizer

This folder contains the Lichtblick dashboard used for debugging the
hardware interface from the devcontainer. It starts a browser-based ROS
visualizer, connects it to the local ROS graph, and loads the visualizer dashboard
layout with the custom control panels. The dashboard is shared across vehicles; LARS is
the currently bundled model, not a requirement for telemetry or controls.

Use this when you want the full ROS debug view: robot model, telemetry, plots,
logs, teleop controls, lights, and hardware operation buttons.

## Run

Use the VS Code action button:

```text
ROS Visualizer
```

or run the launcher manually:

```bash
.devcontainer/script/container_start_visualizer.sh
```

The launcher starts:

- Lichtblick on `http://localhost:8080`
- `foxglove_bridge` on `ws://localhost:8765`
- the `ROS Visualizer` layout from `default_layout.json`
- the custom `Hardware Interface Panels` extension

The browser opens at:

```text
http://localhost:8080/visualizer.html?ds=foxglove-websocket&ds.url=ws://localhost:8765
```

That page selects the visualizer layout and connects it to the local ROS bridge. The
vendor default layout is left alone.

## First Launch

Lichtblick and the ROS WebSocket bridge run inside the devcontainer. The
visualizer is optional: building or opening the devcontainer does not download
or start it. No additional container or Docker socket is needed.

On first launch, the script downloads the official Lichtblick **1.29.1** web
bundle (about 38 MiB) and caches it under
`~/.cache/ros-visualizer/1.29.1`. Later launches reuse the cached files. The
cache respects `XDG_CACHE_HOME`; set `VISUALIZER_VERSION` to select another
release, or `VISUALIZER_WEB_ROOT` to use a preinstalled web bundle containing
`index.html`.

A small Python HTTP server serves the bundle together with the visualizer startup
page, layout and extension. The launcher installs ROS packages
(`ros-${ROS_DISTRO}-foxglove-bridge` and, when the model is enabled,
`ros-${ROS_DISTRO}-robot-state-publisher`) and Node.js/npm for extension
packaging only when needed. These tools are not baked into the devcontainer
image.

VS Code forwards HTTP port **8080** and WebSocket port **8765** to the host.
This uses the same devcontainer forwarding on Windows and Linux; the browser
connects to `localhost` on the host. No Docker port publishing is required.
After changing `devcontainer.json`, use **Dev Containers: Rebuild Container**
to apply the updated configuration.

The forwarding requires the same local port as the container port so the
startup URL stays valid. If a host port is occupied, VS Code reports the
conflict instead of silently picking a different number. Check the **Ports**
view for the actual local addresses and remove any stale forwards from the
previous setup.

To use HTTP port 8081 instead, run:

```bash
.devcontainer/script/container_start_visualizer.sh --lichtblick-port 8081
```

Port 8081 has forwarding attributes configured as well. For other custom
ports, add them in the VS Code Ports view if automatic detection does not
forward them.

Keep the launcher running while using the dashboard. **Ctrl+C** stops the HTTP
server and ROS processes started by that launch and removes its temporary
layout/extension directory. The downloaded web bundle remains cached.

## Dashboard

The default `ROS Visualizer` workspace has four tabs:

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

The default 3D model is LARS, provided by the `lars_description` URDF from the ROS 2
`/robot_description` topic. In the layout this topic is enabled under the 3D
panel's `topics` config so Lichtblick treats it as a URDF renderable, not just
as a raw `std_msgs/String` topic. The `lars_visualizer` bridge publishes the orientation
from `/hardware/measure/imu` as the visualization-only transform
`imu_visualization_world -> base_link`; the model and its
steering joints remain below that frame. A semi-transparent 10 cm grid stays
fixed in `imu_visualization_world` as a level reference.
Before the first valid IMU message, the bridge publishes a neutral orientation
at 20 Hz so the model and reference grid also work without connected hardware.

## Files

- `default_layout.json`: the visualizer dashboard layout.
- `visualizer.html`: small startup page that selects the visualizer layout and
  extension before opening Lichtblick.
- `lichtblick_web.py`: downloads the optional web bundle and serves it with the
  visualizer startup files.
- `extensions/hardware-interface-panels`: custom Lichtblick panels used by the
  layout.

The `lars_description` package contains the URDF, its colored Collada meshes
and the robot state publisher launch file. The separate `lars_visualizer`
package provides the steering and IMU bridges and launches the complete model.
The launcher builds these packages together with `lars_msgs` in an external ROS
overlay when needed. `VISUALIZER_MODEL=false` skips the LARS model and its bridges
while keeping telemetry and control panels available. The overlay defaults to
`$HOME/ros_visualizer_ws`; set `VISUALIZER_ROS_WS` to use another workspace.
Launcher configuration uses the `VISUALIZER_*` environment variables listed by
`container_start_visualizer.sh --help`.
See the [ROS package overview](../ros_packages/README.md) for launch commands,
sensor frames and the visualization-only steering assumptions.

The ROS Graph tab includes the custom **Topic Monitor** for live frequency and
jitter debugging. Click a topic to see its message-period graph; enable the
`/hardware/` or all-topic subscription option in the panel settings temporarily to compare streams.
See the [extension README](extensions/hardware-interface-panels/README.md#topic-timing-monitor)
for timestamp semantics, stale detection and history limits.
