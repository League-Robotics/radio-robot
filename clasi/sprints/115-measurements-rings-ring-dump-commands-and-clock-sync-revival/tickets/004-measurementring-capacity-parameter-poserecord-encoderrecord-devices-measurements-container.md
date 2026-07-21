---
id: '004'
title: MeasurementRing capacity parameter + PoseRecord/EncoderRecord + Devices::Measurements
  container
status: open
use-cases:
- SUC-115-001
- SUC-115-002
- SUC-115-004
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# MeasurementRing capacity parameter + PoseRecord/EncoderRecord + Devices::Measurements container

## Description

Foundation ticket for the sprint (see sprint.md Architecture Steps 3-4).
Today `src/firm/devices/measurement_ring.h`'s `Devices::MeasurementRing<T>`
has zero production/app-graph publishers (only one existing unit test,
`src/tests/sim/unit/measurement_ring_harness.cpp`, instantiates it
directly). This ticket builds the concrete record types and the
five-ring container everything downstream in this sprint (005-010)
depends on — no producer or consumer is wired up yet.

## Implementation Plan

- **Approach**:
  1. Add a compile-time capacity template parameter to
     `Devices::MeasurementRing<T>` — `MeasurementRing<T, Slots = 6>` —
     defaulting to the CURRENT value so
     `src/tests/sim/unit/measurement_ring_harness.cpp`'s existing
     `MeasurementRing<int>` usage (hardcoded 6-slot/5-published-depth
     expectations, e.g. `expectedValues[Devices::MeasurementRing<int>::kDepth]`)
     keeps compiling and passing unchanged. `kDepth` stays derived from
     `Slots` the same way it derives from the current fixed `kSlots`
     today (`kDepth = Slots - 1`).
  2. Add `Devices::PoseRecord{stamp, v_x, v_y, omega, x, y, heading}` and
     `Devices::EncoderRecord{stamp, velocity, position}` — devices/-local
     record types (mirrors the existing `PoseReading` precedent in
     `device_types.h`), NO source field (the ring is the source). Unit
     tags: `stamp` `// [us]`, `v_x`/`v_y` `// [mm/s]`, `omega` `// [rad/s]`,
     `x`/`y` `// [mm]`, `heading` `// [rad]`, `velocity` `// [mm/s]`,
     `position` `// [mm]`.
  3. Add `Devices::Measurements` (new file, `src/firm/devices/measurements.h`)
     — one struct with exactly five named members: `external`, `otos`,
     `encoderPose` (each `MeasurementRing<PoseRecord, N>`), `encoderLeft`,
     `encoderRight` (each `MeasurementRing<EncoderRecord, N>`). Normal-build
     `N` ≈ 8 (a named constant, not a magic number at each declaration).
  4. Construct one `Devices::Measurements` instance in `main.cpp` (ARM)
     and the sim composition root (`src/sim/sim_harness.h` or wherever
     the sim wires its own device graph) — construction only, no producer
     publishes into it yet (that is tickets 005/006) and no consumer
     reads from it yet (ticket 007).
- **Files to create**: `src/firm/devices/measurements.h`. Record types
  may live in this same file or in `device_types.h` alongside
  `PoseReading` — implementer's choice, follow whichever the file's own
  existing organization suggests.
- **Files to modify**: `src/firm/devices/measurement_ring.h` (capacity
  template parameter), `src/firm/main.cpp` (construct the
  `Measurements` instance), sim composition root (same).
- **Testing plan**: extend `measurement_ring_harness.cpp` or add a new
  sim-unit test file covering: (a) the existing 6-slot behavior is
  unchanged at the default `Slots`, (b) a non-default `Slots` (e.g. a
  small capture-style value) publishes/evicts correctly at its own
  capacity, (c) a freshly-constructed `Devices::Measurements` has all
  five rings reporting `latest().valid == false`.
- **Documentation updates**: none beyond this sprint's own `sprint.md`
  Architecture section (already written) and inline doc comments on the
  new types, matching `measurement_ring.h`'s own header-comment density
  and style.

## Acceptance Criteria

- [ ] `MeasurementRing<T, Slots = 6>` compiles with
      `measurement_ring_harness.cpp` unchanged and its existing scenarios
      still passing.
- [ ] `PoseRecord`/`EncoderRecord` match the field lists above, with unit
      tags on every field, no source field.
- [ ] `Devices::Measurements` exposes exactly the five named members
      (`external`, `otos`, `encoderPose`, `encoderLeft`, `encoderRight`),
      no others.
- [ ] `main.cpp` and the sim composition root each construct one
      `Devices::Measurements` instance.
- [ ] No producer publishes into any ring yet (verified: a boot-then-idle
      sim run shows every ring's `latest().valid == false`) — that is
      tickets 005/006's job, not this one's.
- [ ] New unit test coverage per the Testing Plan above passes.

## Testing

- **Existing tests to run**: `src/tests/sim/unit/measurement_ring_harness.cpp`
  (must still pass unchanged); full `uv run python -m pytest` sim suite;
  `just build-clean`.
- **New tests to write**: see Implementation Plan's Testing Plan bullet.
- **Verification command**: `uv run pytest`
