---
id: '004'
title: Per-leg geometry + rendered-trace verification and stand HITL pass
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
depends-on:
- '002'
- '003'
github-issue: ''
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

- [x] A new per-leg geometry test (`tests/sim/system/`, new — this
      directory currently only has a README) asserts every leg of at least
      Tour 1 against sim ground truth: heading/position change within a
      tight tolerance of commanded, AND no reverse-spin residual velocity
      at each leg's completion.
- [x] Tour 1 and Tour 2 rendered trace images are produced and manually
      confirmed (by the implementing session) to trace the intended figure,
      not a tangle — attach or reference the image path in this ticket's
      completion notes.
- [x] `tests/testgui/test_tour1_geometry.py`'s current xfail/loose-tolerance
      path is revisited: either tightened to pass, or re-documented against
      the new, smaller residual with an explicit reason it isn't fully
      closed.
- [x] `tests/sim/unit/test_motion_commands_arc_turn.py`'s `RT 9000` ±10°
      tolerance is tightened to match the fixed behavior.
- [x] Stand HITL pass completed and recorded: wheel spin-up/turn/stop in
      both directions on the stand; turns land accurately (no backtrack/
      wander); drives stop at commanded distance without overshoot; `STOP`
      halts immediately; `wedged()`/`wedgeSuspect()` show no runaway/
      persistent-latch behavior across the session. **(Done — team-lead
      stand HITL session 2026-07-06, fw `0.20260706.17`. See notes below.)**
- [x] The parent issue (`motion-turn-drive-terminal-overshoot.md`) is
      closeable. **(Unblocked — HITL passed.)**

## Completion Notes (software half — 086-004 programmer session)

**Per-leg geometry test**: `tests/sim/system/test_tour_geometry.py` (new).
Drives `robot_radio.testgui.commands.TOUR_1`/`TOUR_2` (the SAME canonical
leg lists the GUI's tour buttons use — imported, not duplicated) directly
through `libfirmware_host` via the `sim` fixture, exactly like
`test_motion_overshoot_regression.py`. For every leg: accumulates the
UNWRAPPED heading delta tick-by-tick (needed for Tour 2's >180°-magnitude
turns, e.g. `RT -21700`, which a naive single-shot wrapped diff would
corrupt), measures settled position/heading change after an 800 ms
post-completion settle window (matching 086-001/002/003's own precedent),
and samples per-wheel velocity in that same window to assert no sustained
reverse-spin residual (bound: 2.0 mm/s, matching
`test_motion_overshoot_regression.py`).

Measured per-leg numbers (deterministic, 2026-07-06, no error knobs set):
- D legs: settled distance +0.35% to +3.60% over commanded (worst case:
  `D 200 200 240` at +8.64 mm/+3.60%); tolerance
  `max(10.0mm, 0.015 * target_mm)` — ~1.16x headroom on the tightest case.
- RT legs (8 distinct across both tours, magnitudes 90-217°): settled
  heading change +5.15° to +7.20° over commanded; flat ±8.0° tolerance —
  ~1.11x headroom over the worst case (`RT 12400`, +7.20°).
- Every D leg's heading drift and every RT leg's net translation measured
  exactly 0.00 (asserted against small non-zero tolerances: 1.0° / 5.0mm).
- Worst-case post-completion residual velocity across all 28 legs (both
  tours): 1.61 mm/s (an `RT 9000` leg) — well inside the 2.0 mm/s bound.

Both `test_tour1_...` and `test_tour2_...` pass. Full numbers and rationale
are in the test file's module docstring.

**No new defect found.** The ~5-7° per-turn RT coast is `handleRT`'s own
already-documented, deliberately-open-loop, no-coast-anticipation-bar
characteristic (`source/commands/motion_commands.cpp`'s doc comment) — a
different, smaller, BOUNDED residual from the reverse-spin defect 086-002
fixed, not newly surfaced by this ticket's per-leg test. 086-003's
`STOP_ROTATION` anticipation is a documented approximation (that ticket's
own completion notes) and does not close this residual to near-zero the
way it closed `D 200 200 500`'s. Verified this is real plant behavior, not
a stale build: `tests/_infra/sim/build/libfirmware_host.dylib`'s mtime
postdates every 086-002/003 source file it links.

**Rendered trace images**: `tests/sim/system/render_tour_trace.py` (new,
follows `tests/playfield/plot_square.py`'s Agg-backend/`savefig()` CLI-tool
pattern). Output:
- `tests/sim/system/out/tour1_trace.png` — Tour 1, 13 legs, ends (24, -70)mm,
  h=-141.8°, 74mm from origin.
- `tests/sim/system/out/tour2_trace.png` — Tour 2, 15 legs, ends (46, -46)mm,
  h=-176.8°, 65mm from origin.

Both PNGs (git-ignored build artifacts — see `.gitignore`) were opened and
inspected by this session. Each shows a clean, deterministic, closed(-ish)
polygon with a distinct vertex at every leg's completion — NOT the chaotic
"off-field tangle" failure mode the pre-fix bug produced. Because each
`RT 9000`-family turn actually rotates ~90+6.4° (not exactly 90°, per the
per-leg numbers above), the accumulated ~6-7°/turn error visibly skews the
tour's shape from a perfect rectilinear spiral into a "pinwheel" with some
line crossings near the center — this is real, correctly-plotted ground
truth (independently verified vertex-by-vertex against the commanded
turn/drive sequence, not a rendering bug), and is the same already-scoped
RT/ROTATION open-loop characteristic described above, not a new defect.

**`tests/testgui/test_tour1_geometry.py` retune**: re-measured (2026-07-06,
3 repeat runs each) fused-pose distance-from-origin: Tour 1 ~51-52mm (was
~20-40mm pre-086 — essentially unchanged, within noise); Tour 2 ~53mm (was
~95-175mm pre-086 — a real improvement, 086-003's anticipation helps more
the more turns a tour chains). `_ORIGIN_TOL_MM` tightened 300.0 -> 100.0mm
(~1.9x headroom over the worst observed 53.2mm) — a real tightening of the
rubber-stamp bound, documented as NOT a claim the residual itself shrank to
near-zero (it's the same RT-coast characteristic as above). Both tour tests
plus the third (stop-reactivation) test in that file pass.

**`tests/sim/unit/test_motion_commands_arc_turn.py` retune**: `RT 9000`
(±10° -> ±7°, measured 96.3669° = +6.37°, bit-exact/deterministic across
runs) and its mirror `RT -9000` (same retune, measured -96.3669°). Also
tightened `test_turn_reaches_absolute_heading_from_nonzero_start`'s ±13°
wrap-boundary bound to ±5° (measured 1.95° wrapped residual) per
architecture-update.md's "Impact on Existing Components" table, which
explicitly earmarked this test for 086-004 (086-002's own completion notes
left it at ±13° pending this ticket). The other three tests in this file
(`TURN`-from-zero ±8°, shortest-path-around-wrap ±10°, `R`-curvature ±14°)
were NOT retuned — not named by this ticket's acceptance criteria or by
086-002/003's completion notes, so left alone to stay in scope. All 14
tests in the file pass.

**Full suite**: `uv run python -m pytest tests/sim -q` — 250 passed (was
248; +2 new). `QT_QPA_PLATFORM=offscreen uv run python -m pytest
tests/testgui -q` — 364 passed, no regressions.

**Files touched**: `tests/sim/system/test_tour_geometry.py` (new),
`tests/sim/system/render_tour_trace.py` (new),
`tests/sim/system/out/*.png` (new, git-ignored),
`tests/testgui/test_tour1_geometry.py` (retuned),
`tests/sim/unit/test_motion_commands_arc_turn.py` (retuned), `.gitignore`
(added `tests/sim/system/out/`).

## Stand HITL Pass (team-lead session, 2026-07-06, fw `0.20260706.17`)

Robot on the stand (wheels free), direct USB serial (`/dev/cu.usbmodem2121102`),
`DEV WD 5000` watchdog fed with `PING` throughout, `DEV DT PORTS 1 2`.
Instrumented script sampled per-wheel `vel` and `encpose` heading through and
1.3 s past each turn's completion. This is the check the sim structurally CANNOT
make: sim `min_duty=0`, so `armoredWrite()`'s reversal path never fires there —
only real hardware (`min_duty>0`) exercises the zero-crossing armor the
backtrack/reverse-spin defect lived behind.

Results (all criteria met):
- **`RT +90` (`RT 9000`)**: completed `EVT done RT reason=rot` at 99.9°;
  **post-completion heading spread 0.0° — NO backtrack**; **reverse-spin
  samples 0 — NO reverse spin**. This is the exact behavior the user reported
  as broken ("completes the turn and then backtracks... wanders around") — now
  clean on hardware.
- **`RT -90` (`RT -9000`)**: completed, returned to 0.2°; 0.0° backtrack,
  0 reverse-spin samples.
- **`D 200 200 500`**: settled `encpose` x = **501 mm** (commanded 500) —
  **NO overshoot** (pre-fix this family overshot to ~535 mm). `EVT done D
  reason=dist`.
- **`STOP`**: halts; encoders hold.
- **Armor intact**: at rest after a motion, `wedged=1` briefly latches
  (`wsus=0`); a follow-up run confirmed it is the documented transient
  raw-latch — a fresh `DEV M 1 STATE` read `wedged=0`, the very next `RT 9000`
  drove and completed normally, and both `RT +90`/`RT -90` in the main run ran
  back-to-back (the second after the first had latched). Self-heals on the next
  drive, `wsus` never qualified, no runaway (everything stopped on `STOP`), no
  persistent latch. The 086 fix did not touch the wedge detector (086-002),
  consistent with this being pre-existing benign behavior.

Both directions drive, encoders increment in proportion/direction, round-trip
over the real link works — the standing verification gate
(`.claude/rules/hardware-bench-testing.md`) is satisfied. The parent issue
`motion-turn-drive-terminal-overshoot.md` is confirmed fixed on real hardware
and is now closeable.

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
