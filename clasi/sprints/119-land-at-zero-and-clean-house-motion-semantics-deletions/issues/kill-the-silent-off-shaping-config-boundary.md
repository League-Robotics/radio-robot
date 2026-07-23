---
status: in-progress
filed: 2026-07-22
filed_by: team-lead (turn-execution review R3/D1/F1, claims verified against code)
related:
- wire-testgui-live-push-of-estimator-stop-lead.md
- land-at-zero-completion-delete-stop-lead.md
sprint: '119'
tickets:
- 119-001
---

# Kill the silent-off shaping/anticipation config boundary

## Description

The single most expensive defect of the turn campaign was a process defect:
`SimHarness` constructs `MoveQueue` with shaping/anticipation OFF
(`sim_harness.h:185`, 4-arg ctor; defaults `move_queue.h:166-168,348`) and the
sim build deliberately excludes `config/boot_config.cpp`
(`src/sim/CMakeLists.txt:68-75`), so a correctness feature that changes turn
accuracy ~20× is silently off in any session that doesn't push
`EstimatorConfigPatch` over the wire. The GUI connect flow now pushes (commit
5ea57201) — but verified 2026-07-22, every OTHER entry point still runs
silent-off:

- `SimLoop.configure_from_robot()` pushes only Tier-1 calibration + Tier-2
  motor config (`sim_loop.py:460-520`) — never estimator/shaper.
- Bench scripts `turn_prediction_capture.py:170`, `estimator_capture.py:256`
  (bare `configure_from_robot`) run unshaped.
- repl.py / cli.py / robot_mcp.py: no estimator push at all.

And the off state is invisible: no telemetry flag (16-bit set at
`telemetry.h:100-115` has nothing for it), no log, no GUI banner;
`shapeAndStage()` early-returns silently (`move_queue.cpp:143`).

## Proposed fix (both halves)

1. **Default-on at the composition seam that already reads the robot JSON:**
   extend `SimLoop.configure_from_robot()` to also push `estimator_kwargs()`
   (the proven wire mechanism from commit 5ea57201) — one change covers every
   configure_from_robot caller (GUI, bench scripts, tests, future scripts).
   The TestGUI's own push becomes redundant-but-harmless (idempotent acks) or
   is deduplicated. Sim sessions then match a serial boot's baked config by
   default; the "sim/production boundary" remains only for callers that
   deliberately skip configure_from_robot.
2. **Loud off-state:** a telemetry flags bit (pick a free bit in the frame-v2
   16-bit flag word; document in docs/protocol-v4.md — this is a wire-visible
   addition, keep it append-only) set on every frame while a MOVE is active
   with angular+linear shaping disabled; host: TestGUI status-bar banner +
   log line when the bit is seen; bench tooling prints it. A feature with a
   20× accuracy delta may not have an invisible off state.

Note: if `land-at-zero-completion-delete-stop-lead.md` lands first, the
pushed key set shrinks (no stop_lead_ms) — the two tickets must agree on the
final field list; the shaper limits (a_max/a_decel/alpha_max/alpha_decel/
j_max/yaw_jerk_max) and estimator weights remain pushed either way.

## Acceptance

- A bare `SimLoop` + `configure_from_robot()` session (no GUI, no manual
  push) runs Tour 1 with shaping/anticipation active — verified by per-leg
  accuracy matching the GUI-path bands, and by read-back/ack counts.
- Both bench capture scripts inherit the push with zero script changes.
- Flags bit verified in sim: strip the push → bit asserts, GUI banner shows;
  push present → bit clear.
- Button-acceptance suite unaffected (still green at its tightened bands).
