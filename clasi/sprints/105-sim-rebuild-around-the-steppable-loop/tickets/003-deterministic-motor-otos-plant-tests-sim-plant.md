---
id: "003"
title: "Deterministic motor+OTOS plant (tests/sim/plant/)"
status: open
use-cases: [SUC-020]
depends-on: []
github-issue: ""
issue: ""
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Deterministic motor+OTOS plant (tests/sim/plant/)

## Description

No plant model exists post-102 (the old `SimMotor`/`PhysicsWorld` stack was
deleted, not migrated). `Devices::I2CBus`'s `HOST_BUILD` fork
(`i2c_bus_host.cpp`) offers only a static, pre-scripted FIFO
(`scriptWrite()`/`scriptRead()`) — no live responder, and `scriptWrite()`
does not even record the payload a caller sent. A live plant does not need
byte-level interception: `Devices::NezhaMotor::appliedDuty()` is already a
public getter, and `devices_motor_harness.cpp`'s own scenario 6 ("PID-on
chases a velocity target") already proves the exact pattern this ticket
generalizes — read `appliedDuty()` after a cycle, compute a first-order-lag
plant response, script the resulting encoder reading for the NEXT cycle via
the same two-write-one-read shape `scriptEncoderRequestCollect()` already
establishes.

This ticket builds a small, seeded, deterministic plant class covering both
motors + OTOS (not one leaf in isolation): duty→velocity→position
integration per wheel, and an OTOS register responder deriving pose from
the SAME two wheel positions via `BodyKinematics::forward()` — the exact
function `App::Odometry` itself calls. The plant carries **no heading state
or wrap/projection logic of its own** (architecture-update.md Decision 3)
— this directly addresses the carried B3 caution
(`docs/code_review/2026-07-13-devices-drive-review.md`): the deleted
pre-rebuild sim's 180°/360° pivot runs both converged on ~272-273°,
suspected to be an angle-wrap attractor in the OLD sim plant's own heading
math. Do NOT port any formula from the deleted `drive/` v2 sim plant, even
adjusted — build this fresh, reusing only `BodyKinematics::forward()`
(unchanged, already-proven production code).

## Acceptance Criteria

- [ ] Plant lives in `tests/sim/plant/` (test-only — never linked into the
      ARM firmware image; not `source/`).
  - [ ] Motor plant: reads `NezhaMotor::appliedDuty()`, integrates a
      first-order duty→velocity response with a time constant in the
      120-140ms range (matching the bench-characterized actuation lag —
      see `.clasi/knowledge/actuation-latency-delay-in-plan.md` /
      the ~120-140ms figure cited in the sprint's own dispatch context),
      then integrates velocity→position; schedules the resulting encoder
      reading via `Devices::I2CBus::scriptWrite()`/`scriptRead()` following
      `scriptEncoderRequestCollect()`'s existing two-write-one-read
      convention.
  - [ ] OTOS plant: answers `Devices::Otos`'s burst-read registers with a
      pose computed from the SAME two wheel positions via
      `BodyKinematics::forward()` — no independent heading integrator, no
      atan2/wrap logic of its own anywhere in the plant's own source files
      (a `grep -n "atan2\|fmod.*M_PI\|wrapAngle" tests/sim/plant/` finding
      nothing is part of this ticket's own self-check).
- [ ] Deterministic and seeded: two runs of the SAME command script with
      the SAME seed produce bit-identical (or explicitly-tolerance-
      documented, if any float noise model is added) trajectories — proven
      by a test that runs the plant twice and diffs the output.
- [ ] A velocity-step scenario (constant commanded duty held for many
      cycles) shows a visible RAMP (not an instantaneous step) in the
      plant's own simulated velocity, converging with a time constant in
      the 120-140ms range (a curve-fit or a "half of final value reached
      within [X,Y]ms" style assertion — ticket-time choice of exact
      tolerance).
- [ ] A direct pivot-style scenario (differential duty commanding a
      turn-in-place) is exercised and its resulting heading (computed
      ENTIRELY through `Odometry`'s own integration over the plant's two
      wheel positions, not read from the plant directly) is sane — this is
      the sprint's own "re-verify B3 doesn't reappear" check flagged in
      architecture-update.md Decision 3's own description.

## Testing

- **Existing tests to run**: `tests/sim/unit/test_devices_motor.py`
  (confirm `NezhaMotor`'s scripted-fake interaction pattern this ticket
  reuses still passes unmodified), `tests/sim/unit/test_devices_otos.py`.
- **New tests to write**: `tests/sim/plant/` gains its own small
  HOST_BUILD test harness (plant-only, no full loop yet — that is ticket
  004's job) proving determinism, the velocity ramp, and the no-heading-
  state self-check above.
- **Verification command**: `uv run python -m pytest tests/sim/plant/ -v`
  (new directory; add to `pyproject.toml` `testpaths` if not already
  covered by the existing `tests/sim` glob — confirm at ticket time).

## Implementation Plan

**Approach**: One plant class (or two small cooperating classes — a
`WheelPlant` per motor + an `OtosPlant` composing both `WheelPlant`
positions through `BodyKinematics::forward()`) driven by an OUTER stepping
loop (owned by ticket 004's `sim_api`, not this ticket) that, each virtual
cycle: (1) reads `appliedDuty()` from the PREVIOUS cycle's write, (2)
advances the plant's internal velocity/position state by one cycle's worth
of virtual time, (3) calls `scriptEncoderRequestCollect()`-equivalent
helpers to pre-load the `I2CBus` FIFO for the NEXT cycle's
`requestSample()`/`tick()`/OTOS-read calls. This ticket builds the plant
class and its own direct-call test harness (constructing `NezhaMotor`/
`Otos` + `I2CBus` directly, no full `RobotLoop`); ticket 004 wires it INTO
the full composed harness.

**Files to create**:
- `tests/sim/plant/wheel_plant.h` / `.cpp` — per-wheel duty→velocity→
  position first-order model + I2CBus scripting helper.
- `tests/sim/plant/otos_plant.h` / `.cpp` — OTOS register responder
  deriving pose via `BodyKinematics::forward()` from two `WheelPlant`
  positions.
- `tests/sim/plant/plant_harness.cpp` + matching pytest wrapper(s) for the
  determinism/ramp/no-heading-state acceptance criteria.

**Files to modify**: none (pure addition; no production code touched).

**Testing plan**: plant-only unit tests (no full loop dependency) prove
physics correctness and determinism in isolation, keeping this ticket
independently verifiable before ticket 004 composes it into the full
harness.

**Documentation updates**: a file-header comment on `wheel_plant.h`/
`otos_plant.h` stating explicitly (mirroring this ticket's own Description)
that no formula was ported from the deleted `drive/` v2 sim plant, and that
heading is deliberately absent from the plant's own state — a durable,
greppable note for any future maintainer tempted to "simplify" by adding a
plant-level heading shortcut.
