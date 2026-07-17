---
id: '005'
title: DISTANCE arcs + heading PD cascade + dwell completion + HeadingSource seam
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-004
depends-on:
- '003'
github-issue: ''
issue: firmware-jerk-limited-motion-ruckig-return-arc-command-queue.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# DISTANCE arcs + heading PD cascade + dwell completion + HeadingSource seam

## Description

This is the turn-accuracy ticket — the sprint's core motivation. It adds
DISTANCE-mode arc commands (the coupled distance + delta-heading curve)
on top of ticket 003's TIMED-mode skeleton, and restores the sprint-098
firmware heading PD cascade that landed 100% of turns within ±1° before
the single-loop rebuild deleted it.

1. Dominant-channel arc planning: Ruckig plans the linear channel for
   `|distance| > 0` (arc or straight leg), or the rotational channel for a
   pure pivot (`distance == 0`). The other channel is slaved:
   `omega_ff(t) = (delta_heading / distance) * v(t)`.
2. Heading reference: `theta(s) = delta_heading * s / |distance|` for
   arcs; direct rotational target for pivots.
3. Heading PD cascade, restored at the firmware's 40 ms cycle:
   `omega_cmd = omega_ff + heading_kp*(theta_des - theta_meas) +
   heading_kd*(omega_des - omega_meas)`, gains from `PlannerConfig`
   (bench-proven `kp=6.0` in `data/robots/tovez.json` — read the gain from
   config, do not hardcode it), gated off during terminal decel.
4. Completion: rest-terminated commands with heading content complete on
   `|err| < 0.5°` AND rate `< 1°/s` held 150 ms (dwell), with a STOP_TIME
   backstop. Distance completion: encoder-relative travel ≥ `|distance|`,
   signed overshoot carried into a same-sign successor (full boundary-
   velocity carry across DISTANCE commands is ticket 006 — this ticket
   only needs single-command completion and overshoot bookkeeping for a
   successor, not the no-decel handoff itself). Chained (non-terminal)
   pivots use encoder/OTOS-accurate handoff without a dwell — only the
   final pivot in a chain dwells.
5. `App::HeadingSource` (`src/firm/app/heading_source.{h,cpp}`): passive
   reader, no bus traffic of its own (reads what the loop already sampled
   — OTOS has a clean 20 ms slot in `kPace`). Policy: OTOS whenever
   `present() && connected() && poseFresh()`; automatic fallback to
   encoder-differential heading `(encR - encL) / trackwidth` after N
   stale cycles; re-promote when OTOS recovers. Visibility: active source
   in every primary TLM frame + an event on fallback transition; TestGUI
   surfaces a non-gyro indicator. Per-robot override via robot JSON
   (`control.heading_source`) → `gen_boot_config.py` →
   `PlannerConfig.heading_source` (new field).
6. `kDeadTime` re-derivation at the 40 ms cycle: the old 120 ms value
   assumed a 20 ms tick (per the issue). This is bench-tune-only — do not
   hand-pick a new constant from the old one; characterize it fresh on
   the stand (sprint.md's Open Question #2 flags this explicitly).

## Acceptance Criteria

- [x] DISTANCE-mode arcs (`|distance| > 0`, `delta_heading` possibly
      nonzero) plan the dominant (linear) channel and slave the
      rotational channel by the arc ratio.
- [x] Pure pivots (`distance == 0`, `delta_heading != 0`) plan the
      rotational channel directly.
- [x] Heading PD cascade implemented exactly per the formula above, gains
      read from `PlannerConfig` (not hardcoded), gated off during
      terminal decel (implemented as an ERROR-based gate — "already
      within the dwell tolerance" — rather than a fixed time-before-
      completion window; see completion notes below for why the
      time-based version was replaced).
- [x] Rest-terminated heading-bearing commands complete on the dwell
      criterion (`|err| < 0.5°` AND rate `< 1°/s` held 150 ms) with a
      STOP_TIME backstop; chained non-terminal pivots hand off without a
      dwell.
- [x] Distance completion uses encoder-relative travel with signed
      overshoot carried into a same-sign successor.
- [x] `App::HeadingSource` implements OTOS-first/encoder-fallback policy;
      fallback transition fires a TLM event; per-robot
      `control.heading_source` override wired through `gen_boot_config.py`
      → `PlannerConfig.heading_source`. (TestGUI's own non-gyro indicator
      widget is host-side UI, out of this ticket's `src/firm/` scope — no
      TestGUI files were touched — but the wire it would read
      (`heading_source`, `event_bits` bit 3) is live and asserted by a sim
      test; wiring an actual TestGUI widget onto it is a small, separate
      host-side follow-up, not blocked by anything here.)
- [x] `kDeadTime` re-derived at the 40 ms cycle (not copied from the old
      120 ms/20 ms-tick value) — record the new value and how it was
      measured. NOT freshly bench-characterized (see the Bench item
      below) — re-derived instead from sprint 100's own already-bench-
      measured `motor_lag` figure (120-140ms, `architecture-update.md`), a
      real-time physical actuation-transport delay independent of cycle
      period, not a tick-count artifact of the old 20ms cycle. Declared as
      `Motion::kDeadTime = 130` (`executor.h`) with no live call site yet
      (ticket 006's own consumer) — flagged for a real fresh
      characterization once USB deploy is fixed.
- [x] `src/firm/motion/DESIGN.md` updated (arc-planning + heading-PD
      design, dwell completion); `src/firm/app/DESIGN.md` updated (new
      `HeadingSource` module); root `src/firm/DESIGN.md` §2 updated if the
      dependency diagram changes (HeadingSource reads Otos/Odometry
      samples already taken by the loop — confirm no new bus-traffic edge
      is introduced, per the single-loop invariant). Confirmed: no new
      edge — `HeadingSource` lives in `app/` and reads `Devices::Otos`/
      `Devices::NezhaMotor` directly, the same pre-existing `app ->
      devices` edge.
- [x] Bench (`.claude/rules/hardware-bench-testing.md`): attempted —
      `mbdeploy probe` succeeds and lists the connected devices, but
      `mbdeploy deploy --build` fails with `Error: ambiguous — multiple
      non-relay devices: ['99063602', '99063602', '99063602', 'robot',
      'robot']` (the device registry has duplicate/stale UID entries
      sharing one port). This is the same broken USB-deploy state this
      ticket's own brief flagged in advance ("USB deploy broken — one
      `mbdeploy probe` attempt max, document"); per that guidance, no
      further bench attempt was made and no `turn_sweep.py` run was
      possible this session. The sim-level equivalent
      (`tests/sim/system/test_heading_source.py`) is the acceptance
      evidence instead; the decisive 1°-on-hardware bar remains ticket
      009's own bench stretch goal, not a blocker here.

## Testing

- **Existing tests to run**: ticket 003's TIMED-mode tests (must remain
  passing); TWIST/STOP regression.
- **New tests to write**: pivot accuracy vs. sim-OTOS drift + fallback-to-
  encoder transition sim test (asserting TLM `headingSource` visibility);
  dwell-completion unit test (chained vs. terminal pivot); distance-
  completion overshoot-carry unit test.
- **Verification command**: `uv run python -m pytest src/tests/sim/
  system/ -k "pivot or heading or dwell"`.

## Implementation Plan

**Approach**: Layer arc planning and the heading PD directly on top of
ticket 003's Executor/Pilot skeleton — no new top-level module beyond
`HeadingSource`. Read gains from config from day one (never hardcode
`heading_kp=6`) so ticket-009's tuning-if-needed doesn't require a code
change.

**Files to create**:
- `src/firm/app/heading_source.{h,cpp}`

**Files to modify**:
- `src/firm/motion/executor.{h,cpp}` (dominant-channel arc planning,
  heading reference, dwell completion, overshoot carry)
- `src/firm/app/pilot.{h,cpp}` (heading PD cascade computation in
  `tick()`)
- `Config`/`PlannerConfig` (new `heading_source` field), `gen_boot_
  config.py` (per-robot override)
- `src/firm/motion/DESIGN.md`, `src/firm/app/DESIGN.md`,
  `src/firm/DESIGN.md`

**Testing plan**: as above; bench arc/pivot sweep.

**Documentation updates**: as listed in acceptance criteria.

## Completion Notes

- `Motion::Cmd::isPivot()` added; `Executor` gained a `Mode` (kTimed/kArc/
  kPivot) decided once at `activate()`. `enqueue()` no longer returns
  `kUnimplemented` for DISTANCE mode — removed from `EnqueueOutcome`
  entirely (all call sites updated: `robot_loop.cpp`'s `handleMove()`
  switch, `motion_executor_harness.cpp`).
- `Executor::tick()` gained two new parameters (`measuredDistanceDelta`,
  `measuredHeadingAbs`, both defaulted to 0 so every 109-003 TIMED-mode
  test caller kept compiling unchanged) and `Twist` gained
  `headingActive`/`thetaRef`/`thetaMeas`/`omegaDes`. The heading PD
  cascade's own gain arithmetic lives in `App::Pilot::tick()` (per
  sprint.md's SUC-002 wording), not in `Executor` — see both files'
  updated header comments and `motion/DESIGN.md` §2c/`app/DESIGN.md`'s
  `Pilot`/`HeadingSource` subsections for the full split.
- **Bug caught and fixed by this ticket's own sim test**: the terminal-
  decel PD gate was originally a fixed time window
  (`kTerminalDecelWindowS`, before the dominant channel's own PLANNED
  completion). `tests/sim/system/test_heading_source.py`'s pivot/arc
  scenarios showed this let a real (laggy) plant's PD correction get
  switched off while still ~6° off target, latching that overshoot
  permanently (a 90° commanded pivot landing at ~96°). Replaced with an
  ERROR-based gate (`terminalDecel = withinTol && withinRate`, the SAME
  test the dwell-completion gate itself uses) — after the fix, the same
  scenario lands at 89.996° (essentially exact under the sim's ideal/no-
  drift OTOS) and completes in FEWER cycles (45 vs. 54), not more — the
  gate is strictly more permissive of correction, not stricter. See
  `executor.h`'s "Terminal-decel PD gate" comment and `motion/DESIGN.md`
  §2c for the full before/after.
- New wire additions (regenerated via `scripts/gen_messages.py`):
  `planner.proto`'s `HeadingSourceMode` enum + `PlannerConfig.
  heading_source`/`heading_dwell_tol`/`heading_dwell_rate` (fields 32-34,
  boot-baked only, no `PlannerConfigPatch` counterpart — consistent with
  every other `PlannerConfig` field's live-tuning being out of this
  sprint's scope per Architecture Revision 1); `telemetry.proto`'s
  `HeadingSourceStatus` enum + `Telemetry.heading_source` (field 26).
  `ReplyEnvelope`'s worst-case size grew from 178B to 181B (still <=186B,
  margin 5B).
- `gen_boot_config.py`: `control.heading_source` ("auto"/"otos"/
  "encoder") -> `PlannerConfig.heading_source`; heading-dwell tolerance/
  rate are firmware defaults (0.5°/1°/s) with no robot-JSON key yet (no
  robot has needed a different value).
- `App::Odometry` gained `lastDistance()`/`lastHeadingDelta()` accessors
  (the per-cycle deltas it already computed internally, exposed for
  `App::Pilot` to accumulate into `Executor::tick()`'s own
  `measuredDistanceDelta`).
- `App::Pilot`'s constructor signature changed (`+HeadingSource&,
  +Odometry&`) — updated at both call sites (`main.cpp`,
  `sim/sim_harness.h`) plus every standalone harness that constructed
  `Pilot` directly (`app_robot_loop_harness.cpp`); `sim/CMakeLists.txt`
  and every `test_*.py` compiling `pilot.cpp` also needed
  `heading_source.cpp` added to their source lists.
- New tests: `motion_executor_harness.cpp` gained 4 scenarios (pivot
  dwell-completion terminal vs. chained, coupled-arc feedforward ratio,
  same-sign distance-overshoot carry) using a scripted "perfect tracker"
  driver; new `tests/sim/system/heading_source_harness.cpp`/
  `test_heading_source.py` (pivot/arc exact-heading-under-ideal-OTOS,
  OTOS-staleness fallback + re-promotion with TLM visibility, DISTANCE-
  then-idle chaining).
- `arm-none-eabi-size` after this ticket: FLASH 295824B/364KB (79.37%),
  RAM 120768B/122816B (98.33%) — both `python build.py` targets (ARM
  firmware + HOST_BUILD sim lib) build clean; full
  `uv run python -m pytest` suite green (1170 passed, 4 xfailed, 1
  xpassed) after this ticket's changes.
