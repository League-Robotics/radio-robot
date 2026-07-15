---
id: '003'
title: Deterministic motor+OTOS plant (tests/sim/plant/)
status: done
use-cases:
- SUC-020
depends-on: []
github-issue: ''
issue: ''
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

- [x] Plant lives in `tests/sim/plant/` (test-only — never linked into the
      ARM firmware image; not `source/`).
  - [x] Motor plant: reads `NezhaMotor::appliedDuty()`, integrates a
      first-order duty→velocity response with a time constant in the
      120-140ms range (matching the bench-characterized actuation lag —
      see `.clasi/knowledge/actuation-latency-delay-in-plan.md` /
      the ~120-140ms figure cited in the sprint's own dispatch context),
      then integrates velocity→position; schedules the resulting encoder
      reading via `Devices::I2CBus::scriptWrite()`/`scriptRead()` following
      `scriptEncoderRequestCollect()`'s existing two-write-one-read
      convention.
  - [x] OTOS plant: answers `Devices::Otos`'s burst-read registers with a
      pose computed from the SAME two wheel positions via
      `BodyKinematics::forward()` — no independent heading integrator, no
      atan2/wrap logic of its own anywhere in the plant's own source files
      (a `grep -n "atan2\|fmod.*M_PI\|wrapAngle" tests/sim/plant/` finding
      nothing is part of this ticket's own self-check).
- [x] Deterministic and seeded: two runs of the SAME command script with
      the SAME seed produce bit-identical (or explicitly-tolerance-
      documented, if any float noise model is added) trajectories — proven
      by a test that runs the plant twice and diffs the output.
- [x] A velocity-step scenario (constant commanded duty held for many
      cycles) shows a visible RAMP (not an instantaneous step) in the
      plant's own simulated velocity, converging with a time constant in
      the 120-140ms range (a curve-fit or a "half of final value reached
      within [X,Y]ms" style assertion — ticket-time choice of exact
      tolerance).
- [x] A direct pivot-style scenario (differential duty commanding a
      turn-in-place) is exercised and its resulting heading (computed
      ENTIRELY through `Odometry`'s own integration over the plant's two
      wheel positions, not read from the plant directly) is sane — this is
      the sprint's own "re-verify B3 doesn't reappear" check flagged in
      architecture-update.md Decision 3's own description.

## Completion Notes (2026-07-15)

Implemented exactly per plan — `tests/sim/plant/wheel_plant.{h,cpp}`,
`otos_plant.{h,cpp}`, `plant_harness.cpp` + `test_plant.py`. No `source/`
files touched.

- **`TestSim::WheelPlant`**: `dutyVelMax`/`tau` first-order lag
  (`alpha = 1 - exp(-dt/tau)`), `kDefaultTau=0.13f` [s] (mid the 120-140ms
  bench range), `kDefaultDutyVelMax=500.0f` [mm/s]. `scriptEncoderResponse()`
  mirrors `scriptEncoderRequestCollect()` exactly, with an explicit
  `writeCount` parameter (default 1) instead of the existing single-device
  harnesses' "always over-provision 2" convention — see below for why.
- **`TestSim::OtosPlant`**: no state but two wheel positions + its own
  `x_/y_/heading_` accumulator, updated via the literal same three lines
  `App::Odometry::integrate()` uses (`BodyKinematics::forward()` +
  `cosf`/`sinf` midpoint-arc) — duplicated, not re-derived. Identity-mounting
  assumption documented (`OtosConfig` offsets all zero in every scenario) so
  `scriptPoseResponse()` needs no inverse lever-arm/mounting-yaw transform.
- **Test harness** (`plant_harness.cpp`): 3 scenarios — a single-motor ramp
  scenario (isolated bus, safe to use the "always 2 writes" convention), and
  a shared `runScenario()` helper (2 motors + `Otos` + `Odometry`, exact bus
  scripting) reused by both the pivot scenario and the determinism scenario.
  `test_plant.py` adds a second pytest test that greps `tests/sim/plant/*.h`
  and `*.cpp` for `atan2|fmod.*M_PI|wrapAngle` and asserts it finds nothing
  — the ticket's own self-check, automated rather than left as a manual
  review step.

**Surprise worth flagging for ticket 004 (sim_api)**: `Devices::I2CBus`'s
HOST_BUILD scripted fake uses **one global write FIFO and one global read
FIFO per direction, shared across every device address on the bus** — not
per-address (confirmed by reading `i2c_bus_host.cpp`; `i2c_bus.h`'s own file
header already says this explicitly). The existing single-device harnesses
(`devices_motor_harness.cpp` scenario 6, `app_odometry_harness.cpp`) get
away with unconditionally scripting "2 writes always" per
`requestSample()`/`tick()` cycle (`scriptEncoderRequestCollect()`'s own
documented "harmless slack" precedent) *only* because they never mix a
second device address into the same bus instance. The moment a scenario
composes two motors + OTOS on one shared `I2CBus` (as this ticket's pivot/
determinism scenarios and ticket 004's full `sim_api` composition both
must), that "always over-provision" convention silently desyncs: an
unconsumed slack write for one address gets popped by the very next
`write()` call to a *different* address, corrupting that call's own address
match (confirmed by deliberately reproducing the bug during implementation —
`Devices::Otos`'s burst read started reporting failure once a leftover
motor-address slack write drifted into its own turn). The fix used here:
script the *exact* write count each cycle (2 only on a motor's own very
first `tick()`, 1 every cycle after, since the leaf's write-on-change guard
provably never issues a second duty write once the target stops changing)
rather than a fixed over-provisioned count. Ticket 004's `sim_api` — which
composes the SAME two motors + OTOS on one shared bus every cycle for the
life of the harness — will need this same exact-count discipline (or an
equivalent fix to the scripting convention itself); it cannot reuse the
existing single-device harnesses' "always 2" shortcut unmodified.

Verification: `uv run python -m pytest tests/sim/plant/ -v` (2 passed);
`uv run python -m pytest tests/sim/unit/test_devices_motor.py
tests/sim/unit/test_devices_otos.py -v` (regression, 2 passed); full suite
`uv run python -m pytest` (565 passed, 0 failed). Compiled clean with
`-std=c++20 -Wall -Wextra -Wpedantic -DHOST_BUILD` (zero warnings).

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
