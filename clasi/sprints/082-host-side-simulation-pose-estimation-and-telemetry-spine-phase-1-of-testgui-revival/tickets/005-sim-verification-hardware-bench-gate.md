---
id: '005'
title: Sim verification + hardware bench gate
status: open
use-cases: [SUC-005]
depends-on: ['004']
github-issue: ''
issue: plan-revive-testgui-against-the-new-tree-simulator.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim verification + hardware bench gate

## Description

Close out the sprint with sim-level verification against ground truth (the
sprint-081 ctypes harness) and a live hardware bench session. No production
`source/` file changes in this ticket -- test files and (if not already done
in ticket 004) the `tests/_infra/sim/CMakeLists.txt` source-list addition for
this sprint's new files.

**Stakeholder-approved scope decision (recorded on this sprint's
`stakeholder_approval` gate, 2026-07-05): accept sim-only OTOS/fused pose for
082.** The hardware bench gate in this ticket covers **encoders and `TLM`
round-trip only**. The OTOS-alive check
(`.claude/rules/hardware-bench-testing.md`'s standing "sensors are alive"
item, as it applies to OTOS specifically) is **explicitly out of reach this
sprint** -- no real `Hal::Odometer` leaf exists in `Subsystems::NezhaHardware`
(architecture-update.md "Grounding," fact 2; Open Question 3) -- and must be
**recorded as such in the bench report**, not silently omitted or skipped
without comment. A real-hardware OTOS driver is deferred to its own later
sprint, out of scope here.

## Acceptance Criteria

### Sim verification (against sprint-081's `libfirmware_host`)

- [ ] A drive sequence (e.g. straight + turn, using `DEV DT VW`/`WHEELS`)
      run through the sim shows `TLM`'s `pose=` and `encpose=` tracking the
      ctypes ground-truth pose (`sim_get_true_pose_*`) within the plant's
      documented tolerance, for the whole sequence, not just at rest.
- [ ] With `SimOdometer`'s error knobs (noise/scale/drift) set to non-zero
      values, `TLM`'s `otos=` diverges from ground truth by roughly the
      configured amount over the same drive sequence.
- [ ] With all of `SimOdometer`'s error knobs zeroed, `TLM`'s `otos=`
      re-converges to (matches) ground truth.
- [ ] A `STREAM`/`SNAP` shape test: all documented fields present when their
      source exists; `otos=` (and any EKF-corrected portion of `pose=` that
      depends on it) omitted -- not zero-filled -- when
      `hardware.odometer() == nullptr`; `STREAM`/`SNAP` share one
      monotonically increasing `seq=`; `STREAM <ms>` clamps below 20 ms.
- [ ] `mode=` reads `I` at rest and `S` during an active `DEV DT VW`/
      `WHEELS` drive, confirmed over the sim's wire surface (not just unit-
      tested against `drivetrain.active()` directly).
- [ ] All new sim tests are placed under `tests/sim/` per the project's
      existing domain split (`tests/CLAUDE.md`), not commingled with
      `tests/bench/`/`tests/playfield/`.

### Hardware bench gate (`.claude/rules/hardware-bench-testing.md`)

- [ ] Deploy to the robot on the stand (`mbdeploy deploy --build`).
- [ ] Encoders alive: `DEV M <n> STATE` (or `TLM`'s `enc=`) shows plausible,
      changing values as wheels are commanded.
- [ ] Wheels drive and encoders increment in the expected direction, roughly
      proportional to commanded speed, for BOTH directions (`DEV DT VW`/
      `WHEELS`, positive and negative).
- [ ] `TLM`'s `enc=`/`encpose=` visibly move correctly (matching the
      commanded direction) while driving; `pose=` is dead-reckoning-only on
      real hardware this sprint (no odometer present) and is expected to
      equal `encpose=` in that case (Decision 1's documented degradation --
      confirm this is what is actually observed, not just assumed).
- [ ] Round-trip over the real serial link: `STREAM`/`SNAP` produce
      well-formed frames over USB serial at the bench.
- [ ] **Bench report explicitly states**: "OTOS-alive check not performed --
      no real-hardware `Hal::Odometer` leaf exists in `Subsystems::NezhaHardware`
      this sprint (stakeholder-approved scope decision, 2026-07-05); `otos=`
      and OTOS-corrected `pose=` are sim-verified only." This sentence (or
      equivalent) must appear in the recorded bench transcript/report, not be
      left implicit.

## Implementation Plan

### Approach

1. Confirm `tests/_infra/sim/CMakeLists.txt` includes every new source file
   from tickets 001-004 (`source/estimation/ekf_tiny.cpp`,
   `source/subsystems/pose_estimator.cpp`, `source/telemetry/tlm_frame.cpp`,
   `source/commands/telemetry_commands.cpp`) plus the `libraries/tinyekf`
   include path -- add whichever of these ticket 004 did not already cover.
2. Write the sim tests per Acceptance Criteria, driving the sim through its
   Python harness (`tests/_infra/sim/firmware.py`'s `Sim` class /
   `host/robot_radio/io/sim_conn.py`, per sprint 081's final shape).
3. Schedule and run the live bench session; capture the full command
   transcript (not a paraphrase) for the sprint/ticket record.
4. Write up the bench report with the OTOS-gap sentence explicitly included.

### Files to create

- `tests/sim/unit/test_pose_estimate_tolerance.py` (or equivalent name per
  the sim test suite's existing naming convention -- check
  `tests/sim/unit/` for the established pattern before naming) -- ground-
  truth tolerance test.
- `tests/sim/unit/test_otos_divergence.py` -- error-knob divergence/
  reconvergence test.
- `tests/sim/unit/test_tlm_stream_snap.py` -- frame-shape/`seq=`/clamp test.

### Files to modify

- `tests/_infra/sim/CMakeLists.txt` -- add this sprint's new source files if
  not already added by ticket 004.

### Testing plan

- `uv run python -m pytest tests/sim` -- all new tests green, no existing
  regressions.
- Live hardware bench session per `.claude/rules/hardware-bench-testing.md`
  -- transcript recorded in the ticket/sprint closure notes.

### Documentation updates

- Record the bench transcript and the explicit OTOS-gap statement in the
  ticket's own closure notes (or `clasi/sprints/082-.../tickets/done/` once
  moved, per the project's standing ticket-completion convention) so the gap
  is auditable later, not just asserted here.
