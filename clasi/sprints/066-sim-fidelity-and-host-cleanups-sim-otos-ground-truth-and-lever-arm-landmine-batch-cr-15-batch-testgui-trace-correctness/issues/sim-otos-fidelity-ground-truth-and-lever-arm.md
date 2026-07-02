---
status: in-progress
review: docs/code_review/2026-07-01-full-codebase-review.md
findings: CR-07, CR-08
severity: medium
sprint: '066'
tickets:
- 066-001
---

# Sim OTOS fidelity: sample plant ground truth and model the lever arm

## Problem

Two structural fidelity gaps that make the simulator blind to the OTOS bug
classes that have hurt the most on hardware.

**(a) Sim OTOS re-integrates commanded wheel speeds (CR-07).**
`SimHardware::advance` feeds `SimOdometer::tick(velL, velR, tw, dt)`
([SimHardware.cpp:63-65](../../source/hal/sim/SimHardware.cpp)), which runs
the same differential-kinematics integration the encoders/odometry use
([SimOdometer.cpp:87-127](../../source/hal/sim/SimOdometer.cpp)). The real
OTOS is an independent ground-truth-tracking sensor. Consequences: the sim
OTOS can never disagree with the encoders except via injected noise (so EKF
fusion is validated in a regime that doesn't exist on hardware), and with
slip configured, plant heading applies `effectiveSlip`
([PhysicsWorld.cpp:95-96](../../source/hal/sim/PhysicsWorld.cpp)) while the
sim-OTOS integration applies none — the sim OTOS diverges from sim truth in
exactly the way the real OTOS does *not*.

**(b) No lever arm (CR-08).** The real driver must subtract `R(hF)·odomOff`
because the chip's offset register is unwritable
([OtosSensor.cpp:122-148](../../source/hal/real/OtosSensor.cpp)); a past
regression here (`db11b7c`) produced 433 mm of phantom translation on a pure
spin. `SimOdometer::readTransformed` reports the robot centre directly
([SimOdometer.cpp:16-32](../../source/hal/sim/SimOdometer.cpp)), so the
compensation path has zero sim coverage.

## Fix direction

- SimOdometer samples `plant.truePose*()` (+ noise/drift/quantization)
  instead of re-integrating wheel velocities.
- Model the sensor at `odomOffX/Y` (sensor pose = centre + R(h)·odomOff) so
  the same `readTransformed` compensation math runs in sim.

## Acceptance / tests

- New sim test: pure spin → OTOS-derived robot-centre translation ≈ 0
  (lever-arm compensation exercised end to end).
- New sim test: with turn-slip configured, encoder pose and OTOS pose
  disagree in sim the way they do on hardware (OTOS ≈ plant truth), and the
  fused estimate tracks OTOS, not the encoders.
- Existing golden-TLM / observation-model tests still pass (or are updated
  with the documented behavior change).
