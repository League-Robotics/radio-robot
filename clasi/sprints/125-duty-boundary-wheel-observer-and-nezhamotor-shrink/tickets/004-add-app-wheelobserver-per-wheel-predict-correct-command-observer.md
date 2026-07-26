---
id: '004'
title: 'Add App::WheelObserver: per-wheel predict-correct command observer'
status: open
use-cases:
- SUC-002
depends-on:
- '003'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add App::WheelObserver: per-wheel predict-correct command observer

## Description

New `App::WheelObserver` (`src/firm/app/wheel_observer.{h,cpp}`),
constructed twice (one per wheel), owned by `App::Drive` (ticket 007
wires ownership; this ticket can build/test the class standalone).
Predicts a wheel's state from its last commanded duty and this wheel's
own prior estimate; corrects on each fresh encoder sample. Folds in,
replacing the three ad hoc mechanisms ticket 003 deleted: the freshness
gate (item 7), glitch/innovation rejection (item 8 — an innovation bound
rejects an implausible step, streak-of-3 re-accept, matching today's
`kMaxPlausibleStepSpeed`/streak logic), the velocity estimate (item 9,
one model replacing the EMA/least-squares A/B pair), and wedge detection
as an innovation outcome (item 12's wedge half — commanded ≠ moving for N
samples). Model defaults match `TestSim::WheelPlant`'s own first-order
`(dutyVelMax, tau)` shape (sim-exact by construction; NOT yet
hardware-fitted — sprint 126's job, Open Questions). See sprint
architecture Step 3 (`App::WheelObserver`) and Design Rationale
Decision 2 (wedge detection moves here, stays base-side protection).

Zero bus traffic — pure computation over a `WheelSample`-shaped input
(position, fresh flag, `appliedDuty`, `t`, `busOk`) each cycle; this is a
hard constraint (the brick's bus budget has no room for observer-driven
extra transactions).

## Acceptance Criteria

- [ ] `App::WheelObserver` exists, zero bus/I2C access anywhere in its
      implementation (grep-enforceable: no `I2CBus`/bus include).
- [ ] **[off-hardware]** Against `TestSim::WheelPlant`, the observer's
      steady-state velocity estimate matches the plant's true velocity
      within a stated tolerance.
- [ ] **[off-hardware]** A sim scenario using the plant's freeze-position
      fault knob shows the estimate CONTINUES predicting (not frozen)
      until a fresh sample resumes correcting it.
- [ ] **[off-hardware]** A commanded-but-not-moving scenario (duty
      nonzero, encoder unchanged for N samples) sets the observer's own
      `wedged` output true; a normal-motion scenario never does.

## Testing

- **Existing tests to run**: none directly superseded (this is new code);
  confirm `devices_motor_harness.cpp`'s glitch/freshness scenarios (if
  any survive ticket 003's deletion) don't silently duplicate this
  ticket's own coverage.
- **New tests to write**: a dedicated `wheel_observer_harness.cpp` (or
  `motion`-adjacent sim harness) covering exact-tracking, predict-through-
  freeze, innovation rejection (a deliberate outlier sample is rejected,
  streak-of-3 re-accepted), and the wedge-innovation outcome.
- **Verification command**: `uv run pytest` plus the new harness's own
  `ctest` target.
