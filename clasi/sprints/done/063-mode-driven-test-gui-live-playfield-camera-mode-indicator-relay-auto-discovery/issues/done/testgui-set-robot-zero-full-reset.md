---
status: done
sprint: '063'
tickets:
- 063-004
---

# Test GUI: "Set Robot @ 0,0" must fully reset pose (heading + encoders + internal position)

## Problem

The "Set Robot @ 0,0" button is currently **display-only** — it re-anchors the on-screen
avatar to the field centre but does **not** reset the robot's actual state. In particular the
heading is not reset. The operator expects this button to put the robot truly "back to
zero-zero."

Current behaviour (`_set_origin` in `host/robot_radio/testgui/__main__.py`):
`trace_model.anchor(0,0,0)` → `trace_model.clear()` → `canvas_ctrl.reset_avatar_to_center()`
→ `canvas_ctrl.refresh()`. No wire command is sent to the robot.

## Intent

Clicking "Set Robot @ 0,0" should reset **everything** to the origin:
1. **Reset heading** to 0.
2. **Zero the encoders** (`ZERO enc`).
3. **Update the robot's internal position** to (0, 0) with heading 0 — a camera-style pose
   update, i.e. send the setpose/`SI` command (`SI 0 0 0`, mm/centidegrees per the wire format)
   so the firmware's internal pose estimate is (0, 0, heading 0).
4. Reset the display (avatar to centre, heading 0; clear/anchor traces) as it does today.

So after the click, the robot's internal pose AND the GUI display both read (0, 0, 0).

## Guidance (for planning — not prescriptive)

- Reuse existing helpers: `operations.build_setpose_command(x_cm, y_cm, yaw_rad)` wraps
  `robot_radio.robot.sync_pose.pose_to_setpose_line` and produces the `SI` wire string; the
  `ZERO enc` command already exists (see `OpsController.on_zero_encoders`). Order likely:
  `ZERO enc` then `SI 0 0 0`.
- Because this now sends wire commands, the button behaviour depends on a connected transport.
  Decide (in planning) whether the button becomes connection-gated, or no-ops with a clear log
  message when disconnected. In Sim mode, sending `SI`/`ZERO` should still be valid.
- Keep the display reset (`reset_avatar_to_center`, trace anchor/clear) so the GUI matches the
  commanded state.
- Confirm the firmware `SI` semantics (does it set heading too?) against
  https://robots.jointheleague.org/ / the protocol docs before finalizing the command sequence.

## Acceptance (behavioural)
- Clicking "Set Robot @ 0,0" sends `ZERO enc` and a setpose to (0,0,heading 0), and resets the
  display; the robot's reported pose (telemetry / camera) reads ~(0,0,0) afterward.
- Clear log lines show what was sent.
- Sensible behaviour when no transport is connected (gated or logged no-op).
- Headless tests cover the command sequence construction (Qt-free) and that the handler emits
  the expected wire strings with a fake transport, via
  `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui -q`.
