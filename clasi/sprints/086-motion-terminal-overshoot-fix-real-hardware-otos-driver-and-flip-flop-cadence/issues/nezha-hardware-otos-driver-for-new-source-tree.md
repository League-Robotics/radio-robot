---
status: in-progress
sprint: 086
tickets:
- 086-005
- 086-006
- 086-007
---

# Real-hardware OTOS driver for NezhaHardware (new source/ tree)

## Context

The new `source/` tree has **no OTOS driver on real hardware**:
`Subsystems::NezhaHardware` exposes no `Hal::Odometer` leaf, so the only
concrete odometer in the tree is `Hal::SimOdometer` (sprint 081). Sprint
082 (firmware pose estimation + telemetry) therefore produces a fused /
OTOS pose that is **sim-only** — on the real robot, encoder dead-reckoning
(`encpose`) works but `otos` and the fused `pose` are dead, and the
hardware-bench gate's "OTOS alive" check cannot pass.

This was an explicit, stakeholder-approved deferral at sprint 082 planning
(2026-07-05): accept sim-only OTOS for 082, and split the real-hardware
OTOS driver into its own later sprint.

## Scope

Add a real `Hal::Odometer` leaf for the SparkFun OTOS on `NezhaHardware`
(I2C driver + lever-arm/offset handling), wire it through
`Subsystems::Hardware::odometer()` (the seam sprint 082 adds as a defaulted
`nullptr` virtual), and re-run the full hardware-bench gate so the
OTOS-alive check and on-hardware fused pose pass. Port/adapt from the
parked `source_old` OTOS driver + `OtosCommands` as reference; conform to
the new HAL/`Hal::Odometer` interface and the project coding standards
(CamelCase, no units in identifiers, wire keys stable).

## Dependencies / sequencing

- Depends on sprint **082** (defines `Hal::Odometer` consumer +
  `Hardware::odometer()` seam + telemetry `otos=` field).
- Naturally sequences alongside or after the TestGUI-revival program
  (082 estimation+telemetry → 083 motion+config → 084 host revival); this
  driver is what gives TestGUI a live OTOS trace on the real robot rather
  than only against the simulator.

## Acceptance (high level)

- `NezhaHardware::odometer()` returns a live OTOS-backed `Hal::Odometer`.
- On the stand: OTOS reports plausible, changing pose/velocity; fused
  `pose` on hardware tracks reality; the bench-gate OTOS-alive check passes.
