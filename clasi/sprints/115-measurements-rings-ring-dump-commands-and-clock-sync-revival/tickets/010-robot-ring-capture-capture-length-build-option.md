---
id: '010'
title: ROBOT_RING_CAPTURE capture-length build option
status: open
use-cases:
- SUC-115-004
depends-on:
- '004'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# ROBOT_RING_CAPTURE capture-length build option

## Description

Depends on ticket 004 (needs the capacity-parameterized
`MeasurementRing<T, Slots>` template). Adds a `ROBOT_RING_CAPTURE`
compile-time option that grows the four high-rate rings (`encoderLeft`,
`encoderRight`, `encoderPose`, `otos`) to a much larger capacity for a
future long-duration bench capture (exercised for real in sprint 116's
own capture script/bench gate — NOT this ticket). `external` stays at
normal size regardless (it is low-rate by nature and has no publisher
yet this sprint). This ticket's own scope is: the option builds and
boots with a heap high-water check — not a full multi-thousand-record
capture run, which sprint 116 owns.

## Implementation Plan

- **Approach**: add a `ROBOT_RING_CAPTURE` CMake option (boolean,
  default OFF), following the existing `ROBOT_RUN_MODE=SIM`-style option
  precedent (`justfile`'s `build-sim` recipe / root `CMakeLists.txt`).
  When ON, compile a much larger `Slots` value into
  `Devices::Measurements`'s four high-rate ring members (a named
  constant, not a magic number — e.g. `kCaptureSlots` vs. the normal
  `kNormalSlots` from ticket 004, selected via `#ifdef`/CMake-injected
  macro at the point `Devices::Measurements`'s member types are
  declared). Add a boot-time log line reporting a heap high-water mark
  (using whatever heap-introspection facility CODAL/the vendor SDK
  already exposes — check for an existing high-water helper before
  adding a new one) so a capture-build's first real boot can be checked
  against the linker map's own ~100KB heap-slack estimate (per the arc
  issue's own "Capture-build RAM" risk note — the number is
  runtime-unverified today).
- **Files to modify**: root `CMakeLists.txt` (or firmware-specific CMake
  include, whichever already hosts build options), `src/firm/devices/measurements.h`
  (capacity constants), `src/firm/main.cpp` (boot-time heap-high-water
  log line).
- **Testing plan**: build with `ROBOT_RING_CAPTURE=ON`, confirm it
  compiles and boots on the stand with the heap log line present; build
  with it OFF (default) and confirm ring capacities/RAM footprint match
  ticket 004's baseline (no regression). A short (not long-duration)
  sim run with the capture build confirms the larger rings actually hold
  more history than the normal build's ~8 slots, proving the capacity
  wiring works, without attempting the full multi-thousand-record
  capture sprint 116 owns.
- **Documentation updates**: a one-line mention in `justfile`/build docs
  of how to invoke a capture build, if the project's existing build-doc
  location has a natural place for it (implementer's judgment — do not
  create a new doc file for one build flag if an existing one already
  lists build options).

## Acceptance Criteria

- [ ] `ROBOT_RING_CAPTURE=ON` build compiles; the four high-rate rings
      report the larger compile-time capacity (verifiable via a unit
      test reading the ring's own capacity constant, or a boot log).
- [ ] `ROBOT_RING_CAPTURE` OFF (default) produces the normal ~8-slot
      capacities with no RAM regression versus ticket 004's baseline.
- [ ] Capture-build boots on the stand and logs a heap high-water
      figure.
- [ ] `external`'s capacity is unaffected by the `ROBOT_RING_CAPTURE`
      flag either way.
- [ ] No full long-duration capture run is required or attempted by this
      ticket — that is sprint 116's own scope.

## Testing

- **Existing tests to run**: full `uv run python -m pytest` sim suite
  with `ROBOT_RING_CAPTURE` OFF (default path, must be unaffected); `just
  build-clean` for both option states.
- **New tests to write**: a unit test confirming the capacity constant
  selected matches the `ROBOT_RING_CAPTURE` build flag; see
  Implementation Plan's Testing Plan bullet for the boot-time check.
- **Verification command**: `uv run pytest`
