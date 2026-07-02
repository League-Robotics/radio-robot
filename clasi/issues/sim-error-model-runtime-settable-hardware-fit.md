---
status: pending
severity: high
---

# Sim error model: all error parameters runtime-settable, modeled in the plant, and fittable to real hardware by regression

## Goal (the rule this issue exists to serve)

**The simulator and the real robot must be tunable to behave identically.**
The workflow we are building toward: run the real hardware, record the path
it actually takes (camera truth + the three telemetry poses), then fit the
simulator's error functions to that recording by regression and extract the
error parameters. Loading those fitted parameters into the simulator makes a
sim run reproduce the hardware run. Every design decision below is in
service of that fitting loop.

## Problems today

1. **Error parameters are not uniformly settable at runtime.** Some sim
   error knobs exist only behind the ctypes C-ABI
   (`sim_set_motor_slip`, `sim_set_encoder_noise`, OTOS noise setters in
   [tests/_infra/sim/firmware.py](../../tests/_infra/sim/firmware.py)),
   reachable only by a host process linked to the sim library. Others are
   compile-time constants. Nothing that is conceptually a simulator
   parameter should require a recompile — we already have a key=value
   configuration command mechanism (`SET`/`GET`, ConfigRegistry); the
   simulator needs the equivalent command surface for plant/error values
   (e.g. a sim-build-only `SIMSET`/`SIMGET` or a reserved key namespace).

2. **The error model is physically incomplete.** The current "turn slip"
   only corrupts the *reported* encoders; the plant body never scrubs:
   `set_field_profile` stores `-slip_turn_extra` into
   `PhysicsWorld::_rotationalSlip` and `effectiveSlip(<=0)` maps to 1.0
   ([source/hal/sim/PhysicsWorld.cpp](../../source/hal/sim/PhysicsWorld.cpp)
   sub-step B). Meanwhile the firmware's turn control compensates for 8%
   scrub (`rotationalSlip=0.92` baked into
   [source/robot/DefaultConfig.cpp](../../source/robot/DefaultConfig.cpp)),
   so a "zero-error" sim structurally over-rotates: measured RT 9000 →
   95.2° true, RT 4500 → 45.6° (the 45° case only looks right because two
   errors cancel). A plant that cannot scrub can never be fitted to a robot
   that does.

3. **Errors must live entirely inside the simulated plant/worldview.** The
   plant applies the configured error functions to produce erroneous sensor
   values internally; those values then flow out through the normal firmware
   sensor path and appear in telemetry. No host-side error injection, no
   host-side compensation (see the TestGUI scrub-knob desync of 2026-07-02,
   and [tlm-three-world-poses-encoder-only-pose.md](tlm-three-world-poses-encoder-only-pose.md)).

## Requirements

- **Runtime-settable sim parameters.** Every simulator error/plant parameter
  is settable and readable through wire commands while the sim runs —
  nothing configurable only at compile time, nothing reachable only through
  the C-ABI. The ctypes setters may remain as a thin wrapper over the same
  registry.
- **A full set of error knobs on the plant**, each a parameter of a defined
  error function so it can be a regression variable. Initial set (extend as
  the fitting work demands):
  - per-wheel encoder scale error, slip fraction, Gaussian noise (exist)
  - body rotational scrub (true-pose rotation efficiency — currently
    impossible, see problem 2), straight-line slip
  - trackwidth error (plant-vs-config mismatch)
  - OTOS linear/yaw noise (exist), OTOS scale and mounting/lever-arm error
    (overlaps [sim-otos-fidelity-ground-truth-and-lever-arm.md](sim-otos-fidelity-ground-truth-and-lever-arm.md),
    sprint 066)
  - motor/actuation asymmetry (per-side gain), response lag / coast
- **Telemetry exposes the results.** The three world poses (encoder-only,
  OTOS, EKF) come out in TLM so the fitting tool can compare sim and
  hardware trajectories signal-by-signal — depends on
  [tlm-three-world-poses-encoder-only-pose.md](tlm-three-world-poses-encoder-only-pose.md).
- **Fit tooling (host side).** A script that takes a recorded hardware run
  (wire log + camera truth) and regresses the sim error parameters to
  minimize trajectory disagreement, emitting a parameter file the sim can
  load. This is the acceptance vehicle for the whole issue.

## Acceptance

- Every documented sim error parameter can be set/read over the wire at
  runtime; a test sweeps each knob and observes the corresponding telemetry
  change.
- With body-scrub set to match `rotationalSlip=0.92`, RT 9000 lands on 90°
  true in sim (closing today's 95.2° gap); with all knobs zero AND
  `rotationalSlip` effectively 1.0, RT is exact.
- End-to-end demo: record a hardware Tour 1, fit parameters, replay Tour 1
  in sim with fitted parameters, and show trajectory agreement within a
  stated tolerance.

## Related

- [set-config-not-propagated-to-planner.md](set-config-not-propagated-to-planner.md)
  — fitted calibration values must actually take effect on the robot.
- Sprint 066 roadmap issue
  [sim-otos-fidelity-ground-truth-and-lever-arm.md](sim-otos-fidelity-ground-truth-and-lever-arm.md)
  covers the OTOS ground-truth/lever-arm slice of this.
