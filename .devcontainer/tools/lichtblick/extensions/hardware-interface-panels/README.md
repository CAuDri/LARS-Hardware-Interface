# Hardware Interface Panels

The `Topic Monitor` in the ROS Graph tab lists discovered topics with receive
frequency and message type. Click anywhere on a topic row to open its message-period
graph and average; click it again to close. No graph is opened by default. Rows
are also selectable with Enter or Space.
Unsubscribed/waiting rows are dimmed, stale topics turn amber, and the selected
row is highlighted. Hover a row for its full ROS type and status, or the graph for
individual period values.

## Topic timing monitor

All configuration lives in the panel settings sidebar:

- **Subscriptions:** temporarily subscribe to `/hardware/` or all discovered
  topics. Both options start off whenever the panel is reopened. Selecting a topic
  subscribes to that topic even without a bulk subscription. The Header only
  subscribes to its heartbeat topic. Filtering the list does not change subscriptions.
- **Window:** the rolling rate/graph window, default 30 seconds (1–120 seconds).
  Rates include the current quiet interval. At least two messages are needed for
  a rate estimate.
- **Stale after:** default 2 seconds; stale topics turn amber and show 0 Hz even
  without a new message. Age refreshes every 250 ms. Increase this for slow topics.
- **Period timestamps:** receive time for transport/browser timing, or
  `header.stamp` for source timestamp spacing. Frequency and age always measure
  reception. Missing/zero headers and backward jumps omit affected intervals;
  the graph shows an empty-state hint when no valid periods are available.
- **Reference period:** optional graph reference line in milliseconds; 0 hides it.
  Selecting a different topic resets this reference.
- **Reset measurements:** the action in the Timing settings menu clears history.

The graph autoscales to completed periods, without a reception-age ramp changing
its scale. Only the average is displayed alongside it. Min/max reduction preserves
brief spikes when graph points need to be reduced.

This monitor is intended for live debugging. Seeking/backward source time and
changing timestamp basis clear history. Topic selection preserves histories for
other actively subscribed topics. Only timestamps/periods are retained, capped at
5,000 samples per topic and 256 topics; notices appear if history is limited.

All-topic subscriptions include large camera/pointcloud messages and increase
traffic and decoding load. Browser scheduling, bridge batching and transport
latency affect receive-time jitter; a timing gap alone cannot locate a loss.
Subscriptions are released when the panel unmounts; turn bulk reception off in
settings explicitly when done debugging.

Run `npm run typecheck` to check the extension types.

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
