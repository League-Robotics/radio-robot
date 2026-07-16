---
status: pending
---

# TestGUI: rolling 10-second strip charts in the telemetry section

## Description

Add a second set of graph tabs — the same four series as the top graphs:
**wheel speed, wheel position, heading, distance** — but located down in
the **telemetry section**, using the currently unused space on the right.

Two distinct roles for the two locations:

- **Top graphs** (existing): long-term recording of the full run history.
- **Telemetry-section graphs** (new): a **rolling strip chart** that shows
  at most the **last 10 seconds**. Once 10 seconds have been recorded, the
  oldest data scrolls off the left edge — a continuously scrolling 10-second
  window of wheel speed, wheel position, heading, and distance.

## Layout

- Tabbed the same way as the tabs in **playfield mode**, but positioned to
  the **right of the telemetry section**, filling the unused right-hand space.
- Four tabs: wheel speed, wheel position, heading, distance.

## Notes / where to look

- `host/robot_radio/testgui/` (`canvas.py` and the telemetry pane layout).
- Reuse the existing series/plotting infrastructure where possible; the
  strip chart is a windowed (last-10-s) view of the same telemetry stream,
  not a separate data source.
- Coordinate with the graph-persistence fix
  ([[testgui-graphs-not-persistent-on-view-switch]]) so both the top and
  telemetry graphs keep correct per-series buffers.
