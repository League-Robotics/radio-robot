---
id: '001'
title: Consolidate main.cpp onto Rt::MainLoop (structural only)
status: done
use-cases: [SUC-001, SUC-002, SUC-003, SUC-006, SUC-007]
depends-on: []
github-issue: ''
issue: restore-pose-estimation-otos-encoders-delayed-camera-fixes.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Consolidate main.cpp onto Rt::MainLoop (structural only)

## Description

`main.cpp` currently hand-rolls its own bare loop body
(`hardware.tick(now); drivetrain.tick(...); bb.motors = ...; bb.drivetrain
= ...; bb.loopNow = now;`) instead of calling `Rt::MainLoop::tick(bb,
now)` — the same function `tests/_infra/sim/sim_api.cpp`'s `SimHandle`
already calls. This is sprint 094's deliberate "explicit inline loop, no
`MainLoop` wrapper" decision (`main.cpp`'s own file header).

This ticket reverses that decision (pre-approved by this sprint's
architecture-update.md, D1) as a **pure structural move, zero behavior
change**: `main.cpp` switches to `Rt::MainLoop::tick(bb, now)`, byte-
identical to what it does today. This is Foundation work — every later
ticket in this sprint needs `MainLoop` to be the live loop before it can
extend `MainLoop`'s pass/commit steps. Landing it alone, first, means a
regression from the refactor itself is trivially distinguishable from a
regression introduced by new behavior in a later ticket.

`Rt::MainLoop`'s constructor signature is **not** changed by this ticket
(still `MainLoop(Hardware&, Drivetrain&)`) — ticket 004 grows it to add
`PoseEstimator&`. This ticket only removes the duplicate hand-rolled loop
body from `main.cpp` in favor of the one `MainLoop::tick()`/`commit()`
`sim_api.cpp` already exercises.

## Acceptance Criteria

- [x] `main.cpp` constructs one `Rt::MainLoop loop(hardware, drivetrain)`
      and its `for(;;)` body calls `loop.tick(bb, now)` in place of the
      previous hand-rolled `hardware.tick()`/`drivetrain.tick()`/commit
      sequence.
- [x] No other behavior in `main.cpp` changes: comms tick, command
      routing, `configurator.applyOne(bb)`, `tickTelemetry(bb, router,
      now)`, and `uBit.sleep(1)` keep their existing relative order around
      the new `loop.tick(bb, now)` call.
- [x] Full existing sim/unit suite passes unchanged (`uv run python -m
      pytest`) — no test's assertions reference a value this refactor
      could plausibly move. (1282 passed, 5 xfailed, 0 failed.)
- [ ] Bench smoke: on the stand, `S`/binary `drive`, `TLM`/binary `stream`
      behave identically to a pre-ticket build — same wheel response, same
      TLM field values (per `.claude/rules/hardware-bench-testing.md`'s
      standing verification gate). **DEFERRED** — no robot USB-attached this
      session (only a relay dongle connected); firmware build was verified
      clean (`just build`, both MICROBIT hex and host sim lib). Deferred to
      the team-lead's sprint bench gate.
- [x] `git diff` review confirms `main_loop.h`/`main_loop.cpp` are
      untouched by this ticket (the constructor/tick signature change is
      ticket 004's job, not this one's). (`tests/_infra/sim/sim_api.cpp`
      confirmed untouched too — empty `git diff --stat` on all three files.)

## Implementation Plan

**Approach**: mechanical refactor only. Read `source/main.cpp`'s current
loop body and `tests/_infra/sim/sim_api.cpp`'s `sim_tick()` to confirm
they already call the identical `MainLoop::tick()`/`commit()` sequence;
replace `main.cpp`'s hand-rolled block with the equivalent `MainLoop`
construction + `loop.tick(bb, now)` call, preserving every other line's
position (comm tick, command routing, `configurator.applyOne()`,
`tickTelemetry()`, `uBit.sleep(1)`) exactly where it is today.

**Files to modify**:
- `source/main.cpp` — construct `static Rt::MainLoop loop(hardware,
  drivetrain);` after `drivetrain`/`hardware` are constructed and
  configured; replace the loop body's `hardware.tick(now); drivetrain
  .tick(now, bb.segmentIn, bb.replaceIn, bb.driveIn); bb.motors = ...;
  bb.drivetrain = ...; bb.loopNow = now;` with `loop.tick(bb, now);`.

**Files NOT to touch**: `source/runtime/main_loop.{h,cpp}` (unchanged
this ticket), `tests/_infra/sim/sim_api.cpp` (already conforms).

**Testing plan**:
- Run the full sim suite (`uv run python -m pytest`) — expect zero
  changes in pass/fail status versus the pre-ticket baseline.
- Bench: `mbdeploy deploy --build`, then drive both directions, read
  `TLM`, confirm identical response to a pre-ticket build (spot-check,
  not a full regression sweep — this ticket changes no computed value).

**Documentation updates**: none required (internal refactor; no wire/
config-visible change).
