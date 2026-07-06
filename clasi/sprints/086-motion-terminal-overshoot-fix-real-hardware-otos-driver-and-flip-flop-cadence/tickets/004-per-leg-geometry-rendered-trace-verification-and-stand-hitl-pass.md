---
id: "004"
title: "Per-leg geometry + rendered-trace verification and stand HITL pass"
status: open
use-cases: [SUC-001, SUC-002, SUC-003, SUC-004]
depends-on: ["002", "003"]
github-issue: ""
issue: motion-turn-drive-terminal-overshoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Per-leg geometry + rendered-trace verification and stand HITL pass

## Description

The final ticket in the motion-overshoot fix set. Depends on both 002 (the
motor-loop root fix) and 003 (Planner decel anticipation) landing. Closes
the parent issue (`motion-turn-drive-terminal-overshoot.md`).

**Verification bar (mandated — this is how the bug shipped in sprints
084/085 undetected)**:

- **Per-leg geometry vs. sim ground truth**: assert EACH leg's actual
  heading/position change (from `sim_get_true_pose_*`) vs. commanded, within
  a tight tolerance, AND that the wheels are settled — no reverse-spin
  residual — at completion. **Endpoint-distance-only tour tests are
  BANNED.** This replaces/extends the endpoint-only assertion
  `tests/testgui/test_tour1_geometry.py` currently carries.
- **Rendered-tour check**: capture a Tour 1 AND Tour 2 trace image a human
  can eyeball (matching the established `tests/bench/velocity_chart.py`/
  `tests/playfield/plot_square.py` charting precedent — reuse that pattern,
  don't invent a new one). It must look like the intended figure, not a
  tangle.
- **Stand HITL pass** (`.claude/rules/hardware-bench-testing.md`): deploy
  to the robot on the stand and confirm turns land accurately with no
  backtrack/wander; drives don't overshoot; `STOP` works; the reversal/
  wedge armor is intact (no runaway, no persistent wedge latch) — this is
  the SAME stand pass ticket 002's Invariant A/B tests validated in sim,
  now confirmed on real hardware with both phases of the fix in place.

Retune (don't just re-run) the two existing tests whose loose tolerances
masked this bug:
- `tests/testgui/test_tour1_geometry.py` — its current xfail path
  documents the OLD (already-diagnosed, 085-scoped-out) `rotSlip`/RT-coast
  source of drift; with 002+003 landed, either it now passes at a
  meaningfully tighter tolerance, or the remaining residual is
  re-documented against the new, smaller gap (not left at the old, looser
  number).
- `tests/sim/unit/test_motion_commands_arc_turn.py`'s `RT 9000` case,
  currently tolerant to ±10° over-rotation from "the SMOOTH-stop ramp's
  coast" — tighten to reflect the fixed behavior.

## Acceptance Criteria

- [ ] A new per-leg geometry test (`tests/sim/system/`, new — this
      directory currently only has a README) asserts every leg of at least
      Tour 1 against sim ground truth: heading/position change within a
      tight tolerance of commanded, AND no reverse-spin residual velocity
      at each leg's completion.
- [ ] Tour 1 and Tour 2 rendered trace images are produced and manually
      confirmed (by the implementing session) to trace the intended figure,
      not a tangle — attach or reference the image path in this ticket's
      completion notes.
- [ ] `tests/testgui/test_tour1_geometry.py`'s current xfail/loose-tolerance
      path is revisited: either tightened to pass, or re-documented against
      the new, smaller residual with an explicit reason it isn't fully
      closed.
- [ ] `tests/sim/unit/test_motion_commands_arc_turn.py`'s `RT 9000` ±10°
      tolerance is tightened to match the fixed behavior.
- [ ] Stand HITL pass completed and recorded: wheel spin-up/turn/stop in
      both directions on the stand; turns land accurately (no backtrack/
      wander); drives stop at commanded distance without overshoot; `STOP`
      halts immediately; `wedged()`/`wedgeSuspect()` show no runaway/
      persistent-latch behavior across the session.
- [ ] The parent issue (`motion-turn-drive-terminal-overshoot.md`) is
      closeable.

## Implementation Plan

**Approach**: This ticket is verification-first — it may require no
production code change beyond what 002/003 already landed, unless the
stand HITL pass or the per-leg test surfaces something those tickets'
sim-only acceptance missed (an accepted, expected possibility per the
architecture doc's own "verification may find something sim couldn't," not
a sign this ticket was miscategorized — if it does, fix it here and
document the deviation).

**Files to create/modify**:
- New `tests/sim/system/test_tour_geometry.py` (or similarly named) — per-leg
  ground-truth assertions for Tour 1 (and Tour 2 if time allows within this
  ticket's scope).
- A charting script/utility for the rendered trace, following
  `tests/bench/velocity_chart.py`/`tests/playfield/plot_square.py`'s
  existing pattern (matplotlib, output under a `tests/sim/system/out/`-style
  directory).
- `tests/testgui/test_tour1_geometry.py` — retuned tolerances/xfail status.
- `tests/sim/unit/test_motion_commands_arc_turn.py` — retuned `RT 9000`
  tolerance.

**Testing plan**:
- Run the new per-leg geometry test and the rendered-trace generation.
- Run the retuned `test_tour1_geometry.py` and `test_motion_commands_arc_turn.py`.
- Run the full `tests/sim/` and `tests/testgui/` suites to confirm no
  regressions elsewhere.
- Stand HITL pass per `.claude/rules/hardware-bench-testing.md`'s standing
  verification gate (this ticket touches motor/motion HAL behavior via 002/
  003, so the gate applies): sensors alive, wheels drive/encoders run in
  both directions, round-trip over the real link, explicit turn/drive/STOP
  exercise with armor observation.

**Documentation updates**: Close the parent issue file. If
`docs/protocol-v2.md` or any architecture doc references the pre-fix
tolerance/behavior, update it to match.
