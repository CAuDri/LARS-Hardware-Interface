# Hardware Interface Panels

## Unreleased

- Use opaque, left-aligned topic/type column headers; replace visible age with
  message type and let an entire row toggle its period graph, including by keyboard.
- Keep the graph closed until a topic is selected and use the full list height
  while no graph is open.

- Simplify Topic Monitor: move controls to panel settings, remove count/type/status
  columns, use dimming and color for status, and show only average period beside
  the graph. Remove the age ramp so it cannot overwhelm the period scale.

- Add Topic Monitor with rolling receive rates, explicit stale detection, and an
  interactive period/jitter graph using receive or header timestamps.
- Add temporary hardware/all-topic subscriptions to Topic Monitor and replace
  the native Data Source panel in the default ROS Graph tab.

- Add a separate Thermometer panel and shorten panel names.

- Add configurable RPM, steering, temperature, and current/voltage panels.
- Use the telemetry panels in the default visualizer layout.

## 0.1.0

- Add custom teleop panel.
- Add custom light command panel.
