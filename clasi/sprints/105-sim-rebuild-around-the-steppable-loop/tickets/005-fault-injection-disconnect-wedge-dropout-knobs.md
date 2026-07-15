---
id: "005"
title: "Fault injection: disconnect, wedge, dropout knobs"
status: open
use-cases: [SUC-022]
depends-on: ["003", "004"]
github-issue: ""
issue: later/sim-hardware-fault-injection.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fault injection: disconnect, wedge, dropout knobs

## Description

`clasi/issues/later/sim-hardware-fault-injection.md` was retargeted
2026-07-14: *"a thin steppable-loop sim over the devices layer's
HOST_BUILD fakes, whose scripted I2CBus can natively fake NAKs, stale
reads, and wedge latch-ups — a better fault-injection seam than SimMotor
ever was."* This ticket delivers that ask against the sprint's own plant
(ticket 003) and harness (ticket 004).

Three fault knobs are added to the plant: (a) **motor disconnect** — script
a NAK/error status for a named port's transactions, verifying
`NezhaMotor::connected()` and the loop's `frame.connLeft`/`connRight`
telemetry fields go false and recover when cleared; (b) **encoder wedge**
— freeze a motor's reported position at its current value while the
plant's internal velocity state keeps advancing, verifying `wedged()`/
`wedgeSuspect()` latch and the loop's `kFaultWedgeLatch` telemetry bit
sets; (c) **encoder dropout** — hold a configurable fraction of cycles'
scripted reads at the last value (stale-not-fresh), exercising the
freshness-gate's outlier/glitch handling under sustained partial data loss
(reusing the pattern `devices_motor_harness.cpp` scenario 8 already
proves in isolation, now driven through the full loop).

**OTOS staleness injection (the parked issue's fourth sketch item) is
explicitly OUT of scope.** The firmware does not fuse OTOS at all yet
(`App::Odometry`'s own file header: "no pose fusion happens here... the
robot does not fuse") — there is no firmware reaction to verify against.
Revisit once host-side fusion (106+) exists.

## Acceptance Criteria

- [ ] Motor disconnect knob: a plant-level API (e.g.
      `WheelPlant::setDisconnected(bool)`) scripts NAK/error status for the
      named motor's `I2CBus` transactions while active; a `sim_api`-driven
      pytest scenario shows `connLeft`/`connRight` flip false in decoded
      telemetry while the knob is active, and recover to true once cleared
      and the plant resumes normal scripted responses.
- [ ] Encoder wedge knob: a plant-level API (e.g.
      `WheelPlant::freezePosition(bool)`) freezes the scripted encoder
      reading at its current value while the plant's own internal velocity
      state keeps advancing (i.e. the plant "knows" it should be moving but
      reports no motion); a pytest scenario drives a twist with the knob
      active and shows `kFaultWedgeLatch` set in decoded telemetry within
      `MotorArmor`'s own documented wedge threshold (`kWedgeThreshold`,
      per `devices_motor_harness.cpp` scenario 4's own precedent).
- [ ] Encoder dropout knob: a plant-level API (e.g.
      `WheelPlant::setDropoutRate(float fraction)`) holds that fraction of
      cycles' scripted reads at the last value instead of a fresh one; a
      pytest scenario at a moderate dropout rate (e.g. 20-30%) shows
      `encGlitchCount()`/telemetry stay sane (no false wedge latch, no
      velocity starved to ~0) — matching the freshness-gate contract
      `devices_motor_harness.cpp` scenario 8 already proves in isolation.
- [ ] `clasi/issues/later/sim-hardware-fault-injection.md` (moved into this
      sprint's `issues/` directory by ticket creation) is updated in place:
      mark disconnect/wedge/dropout as delivered (with a pointer to the
      plant API and the pytest scenarios), and OTOS staleness as still
      deferred with the reason (no firmware consumer yet) restated.

## Testing

- **Existing tests to run**: ticket 003's plant tests, ticket 004's
  `sim_api` harness tests — this ticket extends both, must not regress
  either. `tests/sim/unit/test_devices_motor.py` (confirm the underlying
  wedge-detector/freshness-gate behavior this ticket exercises through the
  full loop is unchanged in isolation).
- **New tests to write**: three pytest scenarios (one per knob) under
  `tests/sim/system/` (or a dedicated `tests/sim/system/faults/`
  subdirectory — ticket-time call), each driving `sim_api` with the
  relevant knob active and asserting the telemetry/fault-bit behavior
  described above.
- **Verification command**: `uv run python -m pytest tests/sim/system/ -k fault -v`.

## Implementation Plan

**Approach**: extend `WheelPlant` (ticket 003) with three orthogonal state
toggles (disconnect/freeze/dropout-rate), each affecting only HOW the plant
scripts its next `I2CBus` response — no change to the underlying
duty→velocity→position integration. Each knob is tested by driving a full
`sim_api` twist scenario with the knob active for a bounded window, then
inspecting decoded telemetry for the expected fault signal.

**Files to create**:
- Three pytest scenario files under `tests/sim/system/` (or `.../faults/`)
  — one per knob.

**Files to modify**:
- `tests/sim/plant/wheel_plant.h` / `.cpp` — add the three knob APIs.
- `clasi/sprints/105-sim-rebuild-around-the-steppable-loop/issues/
  sim-hardware-fault-injection.md` — update status/scope notes per the
  acceptance criteria above.

**Testing plan**: three independent pytest scenarios, each isolating one
knob (the other two left in their default/inactive state) to keep failure
attribution unambiguous.

**Documentation updates**: the retargeted issue file itself (see Files to
modify above) — no other external doc changes needed; ticket 006's
`tests/CLAUDE.md` update will mention fault-injection as part of the sim
tier's overall description.
