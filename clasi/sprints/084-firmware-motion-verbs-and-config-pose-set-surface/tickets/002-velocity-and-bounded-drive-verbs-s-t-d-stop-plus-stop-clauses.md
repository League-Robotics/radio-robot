---
id: '002'
title: 'Velocity and bounded-drive verbs: S / T / D / STOP plus stop= clauses'
status: open
use-cases: [SUC-001]
depends-on: ['001']
github-issue: ''
issue: firmware-closed-loop-motion-verbs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Velocity and bounded-drive verbs: S / T / D / STOP plus stop= clauses

## Description

Register `S`/`T`/`D`/`STOP` as top-level wire verbs (matching sprint 082's
own `STREAM`/`SNAP` top-level precedent and `docs/protocol-v2.md` §10,
which already documents these four byte-for-byte), staging a
`msg::PlannerCommand` into `Subsystems::Planner` (ticket 001) instead of
calling it directly from the handler — a new, production (not
`DevLoopState`) outbox, per architecture-update.md Decision 7. Also lands
`stop=<kind>:<args>` clause parsing for the `{t, d, heading, pos, rot}`
kinds ticket 001 implements (§10's clause grammar; `sensor`/`color`/`line`
clauses are parsed syntactically but rejected with `ERR badarg` — no
sensor `Hal` leaf exists yet, matching ticket 001's `Motion::
evaluateStopCondition` scope).

This ticket also introduces the new production streaming-drive watchdog
(`sTimeout`, distinct from `DEV WD`'s `SerialSilenceWatchdog` — see
architecture-update.md Decision 2's key table and its "Consequences" risk
note: these two watchdogs serve different purposes at different
timescales and must not be conflated). `S`'s handler feeds it exactly the
way `DEV DT VW` feeds `SerialSilenceWatchdog` today.

**Wire keys stay stable.** `S`/`T`/`D`/`STOP`'s verb tokens, argument
shapes, `OK`/`ERR`/`EVT` reply text, and `stop=` clause syntax are exactly
as already documented in `docs/protocol-v2.md` §10 — this ticket
implements that existing contract; it does not rename or reshape any of
it.

## Acceptance Criteria

- [ ] New `source/commands/motion_commands.{h,cpp}` registers `S <l> <r>`,
      `T <l> <r> <ms> [stop=...]`, `D <l> <r> <mm> [stop=...]`, `STOP` as
      top-level verbs, matching `docs/protocol-v2.md` §10's existing wire
      shape and range checks (`S`/`T`/`D` velocity range ±1000 mm/s; `T`
      duration 1-30000 ms; `D` distance 1-10000 mm) exactly.
- [ ] A new `MotionLoopState` struct (own file/header, **not**
      `DevLoopState`) holds the staged `msg::PlannerCommand` outbox and the
      `sTimeout` watchdog state; `source/dev_loop.{h,cpp}` gains one field
      (`Subsystems::Planner*`) and one step: feed `leftObs`/`rightObs`/
      `fusedPose` into `planner.tick()`, drain `hasCommand()`/
      `takeCommand()` into `drivetrain.apply()`, drain `hasEvent()`/
      `takeEvent()` into the captured reply sink.
- [ ] `stop=t:<ms>`, `stop=d:<mm>`, `stop=heading:<cdeg>:<eps_cdeg>`,
      `stop=rot:<arc_mm>` parse and fire per §10's existing clause table;
      up to `kMaxStopConds` (4) clauses per command, OR-combined with the
      verb's own built-in stop.
- [ ] `stop=sensor:...`/`stop=color:...`/`stop=line:...` are recognized
      syntactically (do not fall through to `ERR unknown`/`badarg
      missing key`-class parse failures meant for genuinely malformed
      input) but rejected with `ERR badarg` — documented in
      `docs/protocol-v2.md` as "not yet supported; requires a future
      sensor `Hal` leaf," not silently ignored.
- [ ] `D 200 200 500` moves true pose ~500 mm (sim) and emits
      `EVT done D reason=dist`; `T`/`S` behave per §10; `STOP` halts
      immediately with no `EVT`.
- [ ] `S`'s streaming watchdog (`sTimeout`, default matching the old
      table's 500 ms) fires `EVT safety_stop reason=watchdog` when no `S`
      arrives within the window — verified distinct from `DEV WD`'s
      watchdog (different state, different default, independently
      settable once ticket 006 wires `sTimeout` into `SET`/`GET`).
- [ ] No wire key/verb/reply-string renamed from what `docs/protocol-v2.md`
      §10 already documents.

## Implementation Plan

**Approach:** Thin wire-parsing layer over ticket 001's `Subsystems::
Planner`. Port `parseS`/`parseT`/`parseD`/`mc_packStopKVs`/
`mc_parseStopTokenInto` from `source_old/commands/MotionCommands.cpp`
(grammar only — the handler bodies change completely, since they now
stage a `msg::PlannerCommand` instead of calling `Superstructure::
requestGoal`/`Planner::beginX()` through a `CommandQueue`).

**Files to create:**
- `source/commands/motion_commands.h`, `source/commands/motion_commands.cpp`

**Files to modify:**
- `source/dev_loop.h`, `source/dev_loop.cpp` (new `Planner*` field, new
  per-pass drain step)
- `source/main.cpp` (construct `Subsystems::Planner`, `MotionLoopState`;
  concatenate `motionCommands()`'s table)
- `docs/protocol-v2.md` (no change needed for `S`/`T`/`D`/`STOP` grammar
  itself — already documented; add the `sensor`/`color`/`line` clause
  "not yet supported" note and the `sTimeout` key's existence, cross-
  referencing ticket 006 for `SET`/`GET`)

**Testing plan:**
- Sim-level tests (`tests/sim/`) driving `libfirmware_host` directly:
  `D 200 200 500` geometry, `T`/`S` duration/streaming behavior, each
  implemented `stop=` clause firing before/after the built-in stop,
  rejected `sensor:`/`color:`/`line:` clauses, `STOP`'s immediate halt,
  the `sTimeout` watchdog firing independently of `DEV WD`.
- Existing `tests/sim/`, `tests/bench/`, `tests/playfield/`,
  `tests/unit/` suites must stay green (no DEV/telemetry regression).

**Documentation updates:** `docs/protocol-v2.md` §10's `stop=` clause
table gains the explicit "sensor/color/line: recognized, not yet
implemented" note; a new `sTimeout` key entry is stubbed (fully specified
once ticket 006 lands `SET`/`GET`).
