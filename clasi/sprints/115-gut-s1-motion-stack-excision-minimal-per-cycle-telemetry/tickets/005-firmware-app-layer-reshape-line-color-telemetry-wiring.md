---
id: '005'
title: Firmware app-layer reshape + line/color telemetry wiring
status: open
use-cases: [SUC-045, SUC-046, SUC-047, SUC-048]
depends-on: ["003"]
github-issue: ''
issue: telemetry-frame-tightening-amendment-to-gut-s1.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware app-layer reshape + line/color telemetry wiring

## Description

The ticket that makes the firmware compile again after ticket
002's deletions and ticket 003's proto rewrite: fixes every remaining
call site, reshapes `App::Telemetry`'s `Frame`, and wires the two
already-rate-limited line/color leaves into the loop for the first time.
This is the largest single ticket in the sprint by file count; it is
still one coherent unit because every file changes for the same reason
(the app layer now matches the minimal, pilot-free, executor-free,
new-frame-shaped world tickets 002/003 established).

## Acceptance Criteria

- [ ] `main.cpp` (:147-161 verified): drops the
      `Motion::Executor`/`App::HeadingSource`/`App::Pilot` construction
      block and `plannerConfig`/`.configure()`/`.configureHeading()`
      calls entirely; `App::RobotLoop`'s constructor call drops the
      `pilot` argument.
- [ ] `robot_loop.{h,cpp}`: `Pilot&` constructor parameter,
      `drainPilotEvents()`, the MOVE dispatch case, and all `pilot_.*`
      call sites removed. The motor request/tick interleave
      (verified unchanged at :579-582 — the 0x46 encoder-select latch
      trap) and the deadman-expiry branch (verified unchanged at
      :607-621, minus the `pilot_.flush()` line inside it, which goes
      with `pilot_`) are otherwise **untouched in their surrounding
      logic** — do not "clean up" or reorder this block beyond removing
      the pilot-specific lines; both traps are load-bearing precedent
      (091/112-005) unrelated to this sprint's own scope. `updateTlm()`
      reshaped to stage `EncoderReading`/`OtosReading` with sample
      times and assemble the single `flags` word. Rate-limited,
      alternating line/color reads added at the kPace block (at most
      one of {line, color} per cycle — never both, per the 098-004
      per-pass-read regression precedent).
- [ ] `drive.{h,cpp}`: `configure(const msg::PlannerConfig&)`,
      `actuationLag_`, `a_x`/`alpha` acceleration-feedforward staging
      removed (verified present today at drive.h:41-101). `setTwist()`
      gains a `v_y` parameter that is accepted and ignored (wire-forward
      for sprint 116's MoveTwist — sprint.md Decision 5); the one
      existing call site (`RobotLoop`'s `Twist`-arm handler) passes 0
      for it. `tick()` becomes a pure `BodyKinematics::inverse()` →
      `setVelocity()` follower, no feedforward term.
- [ ] `odometry.{h,cpp}`: `lastDistance()`/`lastHeadingDelta()` accessors
      removed (verified at odometry.h:78-88); `integrate()`,
      `applyOtosSample()`, `reset()` untouched.
- [ ] `telemetry.{h,cpp}`: `Frame` reshaped to two `EncoderReading`-
      shaped members, one `OtosReading` member, single
      `ackCorr`/`ackErr`/`ackFresh`, one `flags` assembly point, packed
      `line`/`color` staging fields (verified today's `Frame` at
      telemetry.h:108-138 for the "before" shape to diff against).
      Primary emission cadence constant changed from 40 ms
      (`kPrimaryPeriod`) to 20 ms, matching `kCycle` — closes
      `kcycle-kprimaryperiod-mismatch.md`; both constants' stale
      "~25 Hz" doc-comment labels fixed in the same edit.
- [ ] `App::Drive` has zero remaining reference to `msg::PlannerConfig`
      or any `Motion::*` symbol (grep-verifiable).
- [ ] `python build.py` builds the ARM firmware target clean (this is
      the ticket where that first becomes true again after ticket 002).
- [ ] `app_robot_loop_harness.cpp`/`test_app_robot_loop.py`,
      `app_drive_harness.cpp`/`test_app_drive.py`,
      `app_telemetry_harness.cpp`/`test_app_telemetry.py` updated for
      the new signatures/frame shape (full green-suite confirmation is
      ticket 009's job; this ticket's own acceptance is that these three
      harnesses compile and their existing test intent still holds
      against the new shapes).

## Implementation Plan

**Approach**: Fix in dependency order within the file set: `drive.{h,cpp}`
and `odometry.{h,cpp}` first (self-contained, no cross-file coupling
beyond their own headers), then `telemetry.{h,cpp}` (the `Frame`
reshape other files read from), then `robot_loop.{h,cpp}` (consumes all
three), then `main.cpp` (consumes `RobotLoop`'s updated constructor).
Add line/color call sites last, once the frame has somewhere to put
them.

**Files to modify**: `src/firm/main.cpp`,
`src/firm/app/robot_loop.{h,cpp}`, `src/firm/app/drive.{h,cpp}`,
`src/firm/app/odometry.{h,cpp}`, `src/firm/app/telemetry.{h,cpp}`,
`src/tests/sim/unit/app_robot_loop_harness.cpp` + `test_app_robot_loop.py`,
`src/tests/sim/unit/app_drive_harness.cpp` + `test_app_drive.py`,
`src/tests/sim/unit/app_telemetry_harness.cpp` + `test_app_telemetry.py`.

**Testing plan**: `python build.py` (ARM target) clean. The three
`app_*` unit-test pairs updated and passing individually
(`uv run python -m pytest src/tests/sim/unit/test_app_robot_loop.py
src/tests/sim/unit/test_app_drive.py src/tests/sim/unit/test_app_telemetry.py`).
Full-suite green bar and the sim system-test suite (which exercises this
code through `SimHarness`) are ticket 006/009's job — `SimHarness`
itself isn't fixed until ticket 006, so this ticket's own tests are
scoped to what's host-testable without it.

**Documentation updates**: `src/firm/app/DESIGN.md` — update its
description of `RobotLoop`'s cycle schedule and `Telemetry`'s frame
shape if it names the deleted Pilot/Executor/HeadingSource
collaborators or the old flat encoder fields.
