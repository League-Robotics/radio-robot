---
status: pending
---

# Move turn_shape.py out of testgui and collapse its three sim-capture paths to one

**Source:** code review 2026-07-30, `05-testgui-testkit.md` MAJOR §3.
**Priority:** P1 — this is the file the review guidelines cite BY NAME as a
known-bad placement, and it internally reproduces the "three divergent sim
capture paths gave three different answers" incident.
**Goal served:** "what does a turn actually look like in sim" must have one
answer. Three capture functions at three fidelity levels, each documented as
"more faithful" than the last, guarantees two of them mislead.

## What is wrong

`testgui/turn_shape.py` (364 lines) is a diagnostic tool living in the
shipped GUI package, with three capture entry points:
- `capture_turn()` — deterministic stepping, ideal-chip defaults
- `capture_turn_live()` — real tick thread, "GUI-FAITHFUL"
- `capture_turn_gui()` — through `SimTransport` + real calibration push +
  OTOS enabled, "MOST GUI-FAITHFUL" (and the only one that flips the
  firmware's heading-source policy the way the GUI actually does)

## What to do

1. Move the module to `src/tests/sim/turn_shape.py` (test tooling belongs in
   `src/tests/`, never the importable host package — the same rule the
   testkit relocation issue applies).
2. Keep **`capture_turn_gui()` as the only source of truth**. Delete
   `capture_turn_live()`. If a fast ideal-chip smoke variant is genuinely
   used, keep `capture_turn()` but demote it explicitly:

```python
def capture_turn(...):
    """QUICK ideal-chip smoke check ONLY -- not a source of truth. For any
    number you intend to compare against hardware or quote in a ticket, use
    capture_turn_gui(): it is the ONLY path that pushes the active robot's
    calibration and enables OTOS, which flips the firmware's heading-source
    policy exactly as a real GUI session does."""
```

3. Update the `-m robot_radio.testgui.turn_shape` invocation docs/callers to
   the new location.

## Acceptance

- `src/host/robot_radio/testgui/turn_shape.py` is gone;
  `grep -rn "turn_shape" src/host/` returns nothing.
- At most two capture functions remain, one explicitly demoted; anything
  quoting turn-shape numbers uses `capture_turn_gui()`.
- The relocated tool still runs (`uv run python -m` the new path) and its
  output for a standard 90° turn matches pre-move values.
