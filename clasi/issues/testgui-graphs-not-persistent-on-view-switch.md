---
status: pending
---

# TestGUI telemetry graphs are not persistent across view switches

## Description

In the TestGUI, the wheel speed, wheel position, heading, and distance
graphs lose their history when you switch between them. Repro:

1. View the wheel speed graph and let data accumulate.
2. Switch to another graph (wheel position / heading / distance).
3. Switch back to wheel speed.

Observed: the earlier graph's data is wrecked — the existing series is
deleted and then repopulated with *wrong* data. Something is being
reset/updated on the view switch rather than each graph keeping its own
accumulated history.

Expected: each graph retains its correct, complete history across view
switches. Switching views should only change what is displayed, not
mutate or discard the underlying per-series data buffers.

## Notes / where to look

- Likely the graph/plot widgets share a single data buffer or the switch
  handler re-initializes the series instead of selecting an existing one.
- Suspect the canvas / plotting code in `host/robot_radio/testgui/`
  (e.g. `canvas.py`, currently dirty in the working tree).
