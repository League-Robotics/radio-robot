---
id: '007'
title: "app/Preamble — boot-time device-detection driver"
status: open
use-cases: [SUC-007]
depends-on: ['003']
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# app/Preamble — boot-time device-detection driver

## Description

Build `source/app/preamble.{h,cpp}`: an app-level driver that sequences
each leaf's own already-existing boot-time detection state machine
(`NezhaMotor::begin()`, `Otos::begin()`, `ColorSensorLeaf::beginStep(nowUs)`,
`LineSensorLeaf::beginStep(nowUs)` — all unchanged, KEPT) to a `done()`
terminal signal, replacing `DeviceBus::runPreamble()` (deleted, ticket
003) with a flatter equivalent over the bare leaves.

Depends on ticket 003 (the leaves this drives are no longer reached
through `DeviceBus`).

## Acceptance Criteria

- [ ] `Preamble::step()` advances each leaf's own detection entry point at
      most once per call (one bounded probe action per pass, matching the
      archived plan's boot-loop framing) — no leaf's own retry loop is
      reimplemented inside `Preamble`, only sequenced/called.
- [ ] `Preamble::done()` returns true once every leaf has reached a
      terminal state — present-and-ready, OR confirmed-absent after
      exhausting its own retry budget. An absent sensor does not hang boot
      forever (a bounded worst case, matching the retired
      `DeviceBus::kMaxPreambleTicks`'s defensive-bound spirit — this
      ticket picks its own bound, documented, not copied verbatim from a
      deleted constant).
- [ ] No I2C traffic is issued by any leaf before `Preamble` has begun
      probing it (confirms boot ordering is deterministic, not
      accidentally overlapping with steady-state reads).
- [ ] A decision is made and documented on whether to keep an explicit
      boot power-settle wait (mirroring the retired
      `DeviceBus::kPowerSettleMs`) or rely on each leaf's own retry
      pacing — either is acceptable, but the choice must be stated, not
      left implicit.
- [ ] A host-buildable test proves `Preamble::done()` is reachable with
      one or more leaves scripted absent (using each leaf's own
      `HOST_BUILD` scripted-fake I2C responses to simulate a NAK/no-chip
      condition).

## Implementation Plan

**Approach**: Read each leaf's actual `begin()`/`beginStep(nowUs)`
signature and terminal-state accessor (`connected()`/`present()`) directly
— `otos.h`, `color_sensor.h`, `line_sensor.h` — confirmed during this
sprint's planning to already expose exactly what a sequencer needs.
`NezhaMotor::begin()` is a single-shot call (not a `beginStep` state
machine like color/line) — `Preamble` calls it once per motor, not
repeatedly.

**Files to create/modify**:
- `source/app/preamble.h`, `source/app/preamble.cpp` (new)

**Testing plan**:
- Existing tests to run: none directly (new file); confirm each leaf's own
  existing `devices_*` detection-state-machine tests stay green
  (`Preamble` is a new caller, not a modifier, of that logic).
- New tests to write: the done()-reachable-with-absent-leaves test
  (Acceptance Criteria above); a test confirming `step()` never issues
  more than one probe action per leaf per call (assert scripted I2C
  transaction counts advance by at most the expected amount per `step()`
  call).
- Verification command: `uv run python -m pytest tests/sim/unit/ -k preamble`
  (once the test file exists).

**Documentation updates**: document the power-settle-wait decision (kept
or dropped, and why) directly in `preamble.h`'s header comment.
