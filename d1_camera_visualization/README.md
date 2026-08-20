# D1 camera visualization

`depth_debug_visualizer` is a backend-neutral diagnostic adapter. It subscribes
to the canonical RealSense-compatible `16UC1` raw and color-aligned depth
streams and publishes RGB Plasma images for RViz. Neither perception nor task
control consumes these debug topics.

The default contract and fixed ranges are defined in `config/depth_debug.yaml`:

- sensor-valid range: 0.20–10.0 m;
- Plasma display range: 0.20–2.0 m;
- diagnostic output rate limit: 10 Hz.

The MoveIt launch starts this node only when RViz is requested, so headless task
execution does not pay the visualization cost.
