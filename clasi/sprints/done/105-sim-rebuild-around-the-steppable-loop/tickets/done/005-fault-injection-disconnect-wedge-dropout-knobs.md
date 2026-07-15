---
id: '005'
title: 'Fault injection: disconnect, wedge, dropout knobs'
status: done
use-cases:
- SUC-022
depends-on:
- '003'
- '004'
github-issue: ''
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

- [x] Motor disconnect knob: a plant-level API (e.g.
      `WheelPlant::setDisconnected(bool)`) scripts NAK/error status for the
      named motor's `I2CBus` transactions while active; a `sim_api`-driven
      pytest scenario shows `connLeft`/`connRight` flip false in decoded
      telemetry while the knob is active, and recover to true once cleared
      and the plant resumes normal scripted responses.
- [x] Encoder wedge knob: a plant-level API (e.g.
      `WheelPlant::freezePosition(bool)`) freezes the scripted encoder
      reading at its current value while the plant's own internal velocity
      state keeps advancing (i.e. the plant "knows" it should be moving but
      reports no motion); a pytest scenario drives a twist with the knob
      active and shows `kFaultWedgeLatch` set in decoded telemetry within
      `MotorArmor`'s own documented wedge threshold (`kWedgeThreshold`,
      per `devices_motor_harness.cpp` scenario 4's own precedent).
- [x] Encoder dropout knob: a plant-level API (e.g.
      `WheelPlant::setDropoutRate(float fraction)`) holds that fraction of
      cycles' scripted reads at the last value instead of a fresh one; a
      pytest scenario at a moderate dropout rate (e.g. 20-30%) shows
      `encGlitchCount()`/telemetry stay sane (no false wedge latch, no
      velocity starved to ~0) — matching the freshness-gate contract
      `devices_motor_harness.cpp` scenario 8 already proves in isolation.
- [x] `clasi/issues/later/sim-hardware-fault-injection.md` (moved into this
      sprint's `issues/` directory by ticket creation) is updated in place:
      mark disconnect/wedge/dropout as delivered (with a pointer to the
      plant API and the pytest scenarios), and OTOS staleness as still
      deferred with the reason (no firmware consumer yet) restated.

## Completion Notes (2026-07-15)

Implemented exactly per plan, no scope changes:

- `tests/sim/plant/wheel_plant.{h,cpp}` — added `setDisconnected(bool)`,
  `freezePosition(bool)`, `setDropoutRate(float)` plus the backing state
  (`disconnected_`, `freezePosition_`/`frozenPosition_`, `dropoutRate_`/
  `dropoutAccum_`/`lastReportedPosition_`). `scriptEncoderResponse()` is no
  longer `const` (the dropout accumulator and last-reported-position cache
  are call-order state); `step()`'s duty→velocity→position integration is
  completely untouched by any knob. Disconnect scripts the SAME write/read
  *count* with a NAK status instead of `kOk`, so the shared-FIFO exact-count
  contract (003/004's own "CRITICAL prior finding") stays intact — only the
  status byte changes, never the transaction shape.
- `tests/sim/support/sim_api.{h}` — added `plantLeft()`/`plantRight()`
  accessors (non-const) so a scenario can reach the knobs directly; no other
  `SimApi` change needed (`scriptCycleBusResponses()` already calls
  `scriptEncoderResponse()` unconditionally every cycle, so an active knob
  is picked up automatically with zero `SimApi`-side branching).
- `tests/sim/system/faults/fault_knobs_harness.cpp` +
  `test_fault_knobs.py` — one C++ harness (three scenarios, mirroring
  `sim_api_harness.cpp`'s own multi-scenario-single-binary shape) + one
  pytest compile-and-run wrapper (mirrors `test_sim_api.py` exactly,
  including the full HOST_BUILD dependency-source list). Placed under a new
  `faults/` subdirectory per the ticket's own "ticket-time call" — keeps
  the three fault scenarios grouped and greppable, distinct from
  `sim_api_harness.cpp`'s own non-fault scenarios.
- Issue file updated in place (see the issue's own "DELIVERED" block) and
  the ticket's `completes_issue: true` will archive it via
  `move_issue_to_done` alongside this ticket's own move to `done/`.

**Verification**: `uv run python -m pytest tests/sim/system/ -k fault -v`
→ 1 passed (`test_fault_knobs_harness_compiles_and_passes`, which runs all
three C++ scenarios inside one binary). Full sim suite green (see the
sprint-level report for the final count) — no regression in ticket
003's plant tests or ticket 004's `sim_api` tests, both re-run explicitly.

**Surprise (none structural)**: the raw `wedged()` latch is UNCONDITIONAL
(motor_armor.h's `updateWedgeDetector()` — not gated on `appliedDuty()`), so
the wedge scenario did not strictly need the twist injection to prove the
fault-bit path; the twist is kept anyway to match the ticket's own "drives a
twist with the knob active" wording and to exercise the historical
moving-but-stuck flavor, not just the idle-parked one
`devices_motor_harness.cpp` scenario 4(a) already covers.

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
