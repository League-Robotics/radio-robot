---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 092 Use Cases

This sprint closes out three pool issues left over from sprint 089's own
bench pass (2026-07-07): the D/T terminal-reverse bug is only partially
fixed, `PoseEstimator`'s fused pose is frozen on real hardware, and the
OTOS driver's host-side lever-arm compensation needs a faithful upstream
port plus a clean re-test of whether the chip's own `REG_OFFSET` register
actually works. The relay dongle is unplugged this sprint — no radio path —
so every SUC below treats **sim as the blocking acceptance gate** and bench
verification as a secondary, best-effort confirmation with an explicit
descope path to a follow-on issue if the bench step cannot be completed.

Parent use cases are drawn from `docs/usecases.md` where an existing UC
applies; several SUCs are internal (control-correctness / driver-fidelity
concerns) with no user-visible behavior and are marked `Parent: N/A`.

## SUC-001: `D`/`T` (and the `TURN`/`RT` rotational channel) no longer reverse-creep after their stop-triggered decel

Parent: UC-003 (Drive Robot a Specific Distance), UC-002 (Drive Robot for
Timed Duration).

- **Actor**: Python host / stakeholder on the bench; sim test suite.
- **Preconditions**: Sprint 089's Ruckig migration is in place
  (`Motion::JerkTrajectory`, `Subsystems::Planner`'s `armDistanceStopDecel()`/
  `armVelocityStopDecel()`/`armRotationalStopDecel()`). The confirmed
  hardware defect (`clasi/issues/d-t-terminal-reverse-persists-decel-reseed-from-plan-velocity.md`):
  on real hardware the bench-tuned velocity PID tracks loosely enough
  (measured ~250-310 mm/s on a commanded 200) that `JerkTrajectory::sample()`'s
  plan-believed velocity is a poor proxy for the real wheel speed at the
  exact instant a stop-triggered decel is armed, producing 11-23 mm of
  reverse wheel motion after `EVT done`.
- **Main Flow**:
  1. Host sends `D <l> <r> <mm>` or `T <l> <r> <ms>` (or `TURN`/`RT`).
  2. The goal's stop condition fires; `Planner` arms a stop-triggered
     decel-to-rest re-solve on the affected channel(s), exactly as sprint
     089 designed.
  3. **New this sprint**: at that exact arm instant only (not the routine
     per-tick sample, not the goal-start solve), the seed velocity fed into
     the decel re-solve is nudged toward the measured wheel velocity by at
     most a bounded, ticket-owned correction — never fully trusting
     measurement, never reopening Decision 8 (089)'s general "seed from the
     plan's own last sample" contract for any other solve.
  4. The resulting decel-to-rest trajectory more closely matches the real
     wheel's actual speed at the handoff, so the velocity PID no longer sees
     a large negative error to brake against.
- **Postconditions**: The commanded velocity trace still never reverses
  sign in sim (089's own no-reverse property, preserved); on hardware, the
  measured reverse creep after `EVT done` is materially reduced from the
  089-007 baseline (11-23 mm), ideally to near-zero.
- **Acceptance Criteria**:
  - [ ] Sim (BLOCKING): a Planner-level test injects a synthetic
        post-arm observation showing measured velocity persistently faster
        than the channel's own plan-believed velocity at the exact tick a
        stop-triggered decel is armed (mirroring 089-006's synthetic-
        observation pattern) and asserts the resulting decel trajectory's
        sampled velocity (a) never reverses sign, and (b) converges
        monotonically to rest with no dip-then-rebound (the 087-009
        limit-cycle signature) — proving the correction does not reopen
        that bug class.
  - [ ] Sim (BLOCKING): a second scenario proves the correction is bounded —
        an extreme synthetic divergence (far beyond anything physically
        plausible) still produces a seed correction capped at the
        ticket-owned constant, not a value that fully snaps to measurement.
  - [ ] Bench (BEST-EFFORT, descope on failure): `D 200 200 1000` and
        `T 200 200 1000` on the stand — measured reverse encoder motion
        after `EVT done` is re-measured against the 089-007 baseline
        (11-21 mm / 19-23 mm) and recorded, whether or not it fully
        eliminates the creep. If the bench step cannot be completed (robot
        wedges, relay/serial unavailable, or a regression is found that
        cannot be resolved in-sprint), the ticket records the sim result as
        the completed deliverable and files a follow-on issue rather than
        blocking sprint close.

## SUC-002: Fused pose (`pose=`) accumulates from real wheel motion on hardware, unblocking `TURN` completion and `G` arrival

Parent: N/A for the underlying defect (no dedicated master UC for
`PoseEstimator` fusion internals) — the user-visible effect reaches UC-015
(Drive to Relative XY Position, `G`) and the (unlisted) `TURN` verb.

- **Actor**: Python host / stakeholder on the bench.
- **Preconditions**: `clasi/issues/poseestimator-fused-pose-frozen-on-hardware.md`
  — during 089-007's bench pass, `Subsystems::PoseEstimator`'s fused pose
  (`TLM pose=`) stayed frozen at `(0, 0, -7)` across 1.3+ m of real encoder
  travel. `RT` (which reads the raw encoder-arc differential directly, not
  `fusedPose`) was unaffected; `TURN`'s `STOP_HEADING` and `G`'s
  target-region detection both depend on `fusedPose` and so cannot complete
  on hardware today. The sim plant does not reproduce this — this is a
  hardware-only defect.
- **Main Flow**:
  1. Root-cause via code investigation: compare the working sim path
     (`Hal::SimOdometer`/`Hal::NullOdometer` feeding
     `Subsystems::PoseEstimator::tick()`) against the real path
     (`Hal::OtosOdometer`, `source/runtime/main_loop.cpp`'s
     `poseEstimator_.tick(...)` call site, and the encoder-only
     dead-reckoning accumulator `PoseEstimator` runs internally from
     `leftObs`/`rightObs` regardless of any odometer).
  2. Identify which of `encoderPose()` (pure dead-reckoning) and/or
     `fusedPose()` (EKF-corrected) actually froze on the bench run, to
     localize the defect to `tick()`'s early guard (step 1, gates both
     readings) versus the EKF predict/update path alone (steps 3-4, would
     leave `encoderPose()` unaffected).
  3. Land the most plausible fix.
- **Postconditions**: On hardware, `fusedPose`/`encoderPose` (whichever was
  frozen) accumulate correctly from real wheel motion; `TURN`'s
  `STOP_HEADING` and `G`'s arrival detection can complete on hardware.
  Sim stays green (no regression to `Subsystems::PoseEstimator`'s existing,
  already-passing sim coverage).
- **Acceptance Criteria**:
  - [ ] Sim (BLOCKING): full `tests/sim` suite stays green after the fix —
        `Subsystems::PoseEstimator`'s existing sim coverage is a regression
        guard, not a reproduction of the hardware defect (which sim cannot
        reproduce).
  - [ ] Code investigation (BLOCKING): the root-cause finding (which layer,
        which specific mechanism) is recorded in the ticket's completion
        notes, whether or not a bench re-test is achievable.
  - [ ] Bench (BEST-EFFORT, descope on failure): re-run the `TURN` accuracy
        check (086/087 tolerance bars) and the `G` settle smoke check from
        089-007. If unreachable (relay unplugged, hardware unavailable, or
        the fix cannot be confirmed live), descope to a fresh follow-on
        issue rather than blocking sprint close.

## SUC-003: The OTOS driver faithfully mirrors the upstream SparkFun library

Parent: N/A — internal driver-fidelity/build concern.

- **Actor**: Firmware build; sim/unit test suite.
- **Preconditions**: `clasi/issues/otos-lever-arm-necessity-and-library-port.md`.
  `source/hal/otos/otos_odometer.{h,cpp}` is a partial, hand-rolled driver
  (register map, scaling constants match upstream, but `setOffset`/
  `getOffset`, full signal-process config, and other upstream primitives are
  either absent or unexercised).
- **Main Flow**:
  1. Port the upstream SparkFun OTOS library (Arduino C++ reference) into
     `Hal::OtosOdometer` near line-by-line: register map, scaling constants,
     `setOffset`/`getOffset`, `setPosition`, `setLinearScalar`/
     `setAngularScalar`, signal-process config, IMU calibration, product-ID
     check — conforming to this project's naming rules (CamelCase types /
     lowerCamelCase functions, no units in identifiers, wire/register names
     exempt).
  2. Extend `tests/sim/unit/` coverage for the ported surface (register
     scaling round-trips, offset set/get plumbing at the `Hal::Odometer`
     interface level via the sim leaf/mock bus).
- **Postconditions**: `Hal::OtosOdometer` exposes the full upstream
  primitive surface (in particular `setOffset`/`getOffset`, the mechanism
  SUC-004's bench re-test needs); existing OTOS behavior (position/velocity
  read, lever-arm math, rate limiting, bus-clearance safety from 086-007) is
  unchanged.
- **Acceptance Criteria**:
  - [ ] Sim (BLOCKING): `uv run python -m pytest tests/sim` green, including
        new unit coverage for the ported register surface.
  - [ ] The ported surface's naming conforms to
        `.claude/rules/naming-and-style.md` (wire/register token names
        exempt, per that rule).
  - [ ] No behavior change to the existing, already-verified 086-007
        bus-clearance/rate-limiting safety mechanisms.

## SUC-004: The chip's `REG_OFFSET` mounting-offset register is bench-re-tested, and the lever-arm architecture is finalized to exactly one end state

Parent: N/A — internal driver-architecture concern.

- **Actor**: Stakeholder / bench operator.
- **Preconditions**: SUC-003's ported driver exposes `setOffset`/
  `getOffset`. The prior "REG_OFFSET is unwritable" claim
  (`source_old/hal/real/OtosSensor.cpp`) is suspect — see the issue's own
  argument that it used the identical write path/scaling that this driver's
  existing position-register writes already prove work.
- **Main Flow**:
  1. Bench: write `REG_OFFSET` with the real mounting offset, read it back,
     then drive a pure in-place spin and check for the lever-arm phantom-
     translation arc (the `db11b7c` signature) `source/hal/lever_arm.h`
     documents.
  2. If the chip honors the register (non-zero readback, phantom arc
     disappears): delete `source/hal/lever_arm.h` and all host-side
     lever-arm compensation, folding the register write into `begin()`.
  3. If the bench cannot be run, wedges, or the register still reads back
     zero: **default disposition is FOLD, not delete** — keep host-side
     compensation but fold `LeverArm::sensorToCentre()`/`centreToSensor()`
     directly into `OtosOdometer` (its one production consumer), since
     deleting the compensation requires positive confirmation the chip
     honors the register, and an inconclusive/unavailable bench must not be
     read as that confirmation.
- **Postconditions**: `source/hal/lever_arm.h` is either deleted (register
  confirmed working) or folded into `OtosOdometer` (default/conservative
  outcome) — never left standalone. `tests/sim/unit/lever_arm_harness.cpp`/
  `test_lever_arm.py` are deleted (if folded, merged into
  `otos_odometer_harness.cpp`'s own assertions; if deleted outright, their
  coverage is confirmed subsumed by the chip-native path).
- **Acceptance Criteria**:
  - [ ] Code (BLOCKING, either outcome): the driver is left in exactly one
        of the two end states; `uv run python -m pytest tests/sim` green.
  - [ ] Bench (BEST-EFFORT, descope on failure): a clean bench verdict on
        whether `REG_OFFSET` compensates on this unit is recorded. If
        unreachable, the ticket records the conservative FOLD outcome and
        files a fresh follow-on issue carrying the re-test forward with
        fresh evidence, rather than blocking sprint close.
