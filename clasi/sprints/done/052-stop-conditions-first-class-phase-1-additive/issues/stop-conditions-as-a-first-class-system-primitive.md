---
status: in-progress
sprint: '052'
tickets:
- 052-001
- 052-002
- 052-003
- 052-004
- 052-005
---

# Stop Conditions as a First-Class System Primitive

## Context

Today a "stop condition" is implemented but only half-exposed. The robot already
treats **VW (body twist v, ω) as its one internal motion primitive** — every verb
(S, T, D, G, R, TURN, RT) funnels through `handleVW` into a twist that the
`BodyVelocityController` runs. A real unified `StopCondition` abstraction already
exists ([StopCondition.h](source/control/StopCondition.h)) with 9 kinds, and
`MotionCommand` already holds up to 4 of them, OR-combined and evaluated every tick.
The safety/watchdog is already the always-on backstop.

But three things don't match the intended design:

1. **Command identity leaks into the internals.** S/T/D/R don't cleanly become
   "twist + stop conditions." Each handler *stringifies* its identity (`t=`, `dist=`,
   `stream=`, `radius=`…) into a fake VW `ParsedCommand`, pushes it on the queue, and
   `handleVW` re-parses those strings to demux back into 8 separate `Goal`s and
   `begin*()` methods — even round-tripping T/D from (v,ω) **back** to wheel speeds
   ([MotionCommandHandlers.cpp:938-1013](source/app/MotionCommandHandlers.cpp#L938)).
   This is leftover scaffolding from incremental "behavior-preservation" seams
   (sprints 026/042). It is exactly the mirroring we want gone: past the command
   processor the system should only know *twist + stop conditions*, not which verb
   produced them.

2. **No general stop-clause grammar.** Only `sensor=` is attachable, and only to
   T/D/TURN. VW/S/R cannot carry a stop at all — so "send a primitive plus the
   condition that ends it, on one line" is only partially possible.

3. **No stop-reason reported back.** `EVT done T` does not say *why* the motion ended
   (time vs. sensor vs. distance). The host should always receive "whatever stopped
   the motion."

**Intended outcome:** any motion primitive can carry one or more `stop=` clauses on
the same line; the firmware runs purely on *twist + stop conditions* with no per-verb
mirroring; and every completion reports the reason it stopped, with the watchdog as
the universal backstop.

## Target mental model

Two layers, both already mostly present:

- **A stop-condition layer** (generalized, attachable to any command, evaluated each
  tick, OR-combined, reports the reason that fired). The watchdog is always implicitly
  present as the backstop.
- **A twist source**: either a **constant twist** (VW / S / R / T / D — "hold this
  twist until a stop fires") or a **closed-loop controller** (G / TURN / RT — produces
  a time-varying twist toward a goal whose terminal condition is itself a
  `StopCondition`: POSITION / HEADING / ROTATION).

T is just `VW + time-stop`. D is just `VW + distance-stop`. S/R/VW are `VW + (only the
watchdog backstop)`. The closed-loop family keeps its controllers but is terminated
through the same stop-condition array.

---

## Phase 1 — Additive (no canary break, immediately usable)

Deliver the user-facing capability on top of today's routing. Nothing here changes
the byte-exact golden-TLM behavior of existing commands.

### 1a. Unified `stop=` parser

Add one parser that turns a `stop=<kind>:<args>` token into a `StopCondition`, reusing
the existing factory helpers in [StopCondition.h](source/control/StopCondition.h)
(`makeTimeStop`, `makeDistanceStop`, `makeLineAnyStop`, `makeSensorStop`,
`makeColorStop`, `makeHeadingStop`, `makeRotationStop`). Generalize the existing
[`mc_parseSensorToken`](source/app/MotionCommandHandlers.cpp#L46) rather than writing a
new channel lookup.

Grammar (repeatable; up to `MotionCommand::kMaxStopConds` = 4):

| Clause                              | Maps to               |
|-------------------------------------|-----------------------|
| `stop=t:<ms>`                       | `makeTimeStop`        |
| `stop=d:<mm>`                       | `makeDistanceStop`    |
| `stop=line:<ge\|le>:<thr>`          | `makeLineAnyStop`     |
| `stop=sensor:<ch>:<ge\|le>:<thr>`   | `makeSensorStop` (ch = `line0`…`colorC`) |
| `stop=color:<h>:<s>:<v>:<dist>`     | `makeColorStop`       |
| `stop=heading:<cdeg>:<eps_cdeg>`    | `makeHeadingStop` (cdeg→rad) |
| `stop=rot:<arc_mm>`                 | `makeRotationStop`    |

`sensor=line0:ge:512` stays as a back-compat alias for `stop=sensor:line0:ge:512`.
Each `stop=` token calls `MotionCommand::addStop(...)` on the active command — the same
path the current `sensor=` forwarding already uses in `handleVW`
([MotionCommandHandlers.cpp:879-888](source/app/MotionCommandHandlers.cpp#L879)).

Make the clause acceptable on **VW, S, R, T, D** (and TURN, where it already works). For
VW/S/R, attaching a `stop=` simply means the command is no longer open-ended (it gains
real stop conditions instead of relying only on the watchdog). T/D keep their positional
time/distance arg AND may add further `stop=` clauses (OR-combined).

### 1b. Report the stop reason

- Add a fired-condition record to `MotionCommand`: when `tick()` finds the stop that
  fires, store its `StopCondition::Kind` (and, for SENSOR, the channel). Today `tick()`
  already evaluates the array and tears down on first fire — just remember which one.
- Extend [`MotionCommand::emitEvt`](source/control/MotionCommand.h#L257) to append
  `reason=<token>` after the existing `<base> [#id]`, so the wire becomes
  `EVT done T #12 reason=time` — purely additive (extra trailing token; existing hosts
  that match on the verb still work).
- Reason tokens: `time`, `dist`, `rot`, `heading`, `pos`, `line`, `color`,
  `<channel>` (e.g. `line0`) for SENSOR, and `watchdog` for the safety path.
- Add `reason=watchdog` at the safety-stop emit site in
  [`Superstructure::evaluateSafety`](source/superstructure/Superstructure.cpp#L127)
  (the watchdog injects `X` + emits `EVT safety_stop` there, not via `MotionCommand`).

### 1c. Host-side support ([host/robot_radio/robot/protocol.py](host/robot_radio/robot/protocol.py))

- Add a small `stop=` builder (e.g. `Stop.time(ms)`, `Stop.dist(mm)`, `Stop.line(...)`)
  and let `vw()`, `drive()` (S), `arc()` (R), `timed()`, `distance()`, `turn()` accept
  an optional `stop=[...]` list that is appended as `stop=` tokens.
- Parse `reason=` out of `EVT` lines and have `wait_for_evt_done(...)` return the reason
  alongside the existing `"done"|"safety_stop"|"timeout"` result.

### 1d. Docs

Update [docs/protocol-v2.md](docs/protocol-v2.md) §10 (Motion Commands / EVT Completion
Events, lines ~589-699) to document the `stop=` grammar and the new `reason=` field, and
add a `stop=` column/note to the verb table in [source/COMMANDS.md](source/COMMANDS.md).

---

## Phase 2 — Collapse the open-loop family (re-baselines the canary)

Remove the verb-identity mirroring so the internals run purely on *twist + stops*.

- **Eliminate the stringify/re-parse round-trip** for the open-loop family: handlers
  build their `StopCondition`s directly instead of packing `t=`/`dist=`/`stream=`/
  `radius=` strings for `handleVW` to re-parse. Remove the `vwHasKey`/`vwScanKV`/
  `packKVArg` machinery and the `inverse()` round-trip in
  [handleVW](source/app/MotionCommandHandlers.cpp#L938) for T/D.
- **Collapse `Goal::{STREAM, TIMED, DISTANCE, ARC, VELOCITY}` into one** velocity goal
  that carries `(twist, stops[], style, doneLabel, streamSeed)`. Extend `GoalRequest`
  ([Superstructure.h:71](source/superstructure/Superstructure.h#L71)) with an inline
  `StopCondition stops[4]; uint8_t nStops; bool streamSeed; const char* doneLabel;` and
  route S/T/D/R/VW through it. **Keep** `GOTO`/`TURN`/`ROTATE` — those are closed-loop
  controllers, terminated through the same stop array.
- **Shrink `MotionCommand::Origin`** ([MotionCommand.h:47](source/control/MotionCommand.h#L47))
  to its only real job: a `retargetable` flag for the VW-keepalive guard
  ([MotionCommandHandlers.cpp:1047-1072](source/app/MotionCommandHandlers.cpp#L1047)).
- The wire-facing `EVT done T/D/G` labels are preserved by having each handler still
  pass its `doneLabel` ("EVT done T", …) via
  [`setDoneEvt`](source/control/MotionCommand.h#L115) — the label is a passthrough
  string, not internal control state.

### Phase 2 migration risks (must preserve)

- **Distance encoder reset:** `Robot::distanceDrive` = `beginDistance` + atomic
  `resetEncoders`. The collapsed velocity path must keep that reset (or prove the
  `MotionBaseline.enc0Mm` snapshot makes it redundant) before removing it.
- **Stream-seed vs. ramp:** S seeds the BVC immediately (no trapezoid ramp); VW ramps.
  Preserve this as the `streamSeed` flag, not a separate `Goal`.
- **Keepalive guard:** only a re-targetable (VW-origin) command may have its target
  updated by a bare VW keepalive; non-retargetable commands must still reply `busy=…`.
- **Golden-TLM canary:** collapsing the routing (and likely moving the `begin` out of
  the queue-drain hop) changes byte-exact TLM/timing. Plan to **re-baseline** the canary
  and review the diff deliberately, not silence it.

---

## Critical files

- [source/control/StopCondition.h](source/control/StopCondition.h) — kinds + factory
  helpers (reuse; no new kinds needed). Possibly add a reason-token mapping helper here.
- [source/control/MotionCommand.h](source/control/MotionCommand.h) /
  `MotionCommand.cpp` — record fired stop; append `reason=` in `emitEvt`; shrink `Origin`
  (Phase 2).
- [source/app/MotionCommandHandlers.cpp](source/app/MotionCommandHandlers.cpp) — add the
  `stop=` parser (generalize `mc_parseSensorToken`); accept `stop=` on VW/S/R/T/D;
  Phase 2: remove the string round-trip and build `StopCondition`s directly.
- [source/superstructure/Superstructure.h](source/superstructure/Superstructure.h) /
  `Superstructure.cpp` — `reason=watchdog` at the safety emit; Phase 2: extend
  `GoalRequest` with stops/flags and collapse the open-loop `Goal`s.
- [host/robot_radio/robot/protocol.py](host/robot_radio/robot/protocol.py) — `stop=`
  builders; parse/return `reason=`.
- [docs/protocol-v2.md](docs/protocol-v2.md), [source/COMMANDS.md](source/COMMANDS.md) —
  document `stop=` and `reason=`.

## Verification

- **Firmware unit tests:** add cases that attach each `stop=` kind to VW/S/R and assert
  the correct `StopCondition` is built and that `EVT done … reason=<x>` carries the right
  token. Extend the existing `StopCondition`/`MotionCommand` tests.
- **Golden-TLM canary:** Phase 1 must leave it byte-exact (additive only). Phase 2
  re-baselines it — review the diff.
- **Sim + bench:** run the simulator, then a bench/floor check per
  [smoke-ritual](.clasi/knowledge/) and the camera-verified harness
  (`tests/bench/playfield_camera_run.py`): e.g. `VW 200 0 stop=line:ge:512` stops on the
  line and reports `reason=line`; `VW 200 0 stop=d:300` stops at ~300 mm with
  `reason=dist`; pulling the link mid-`VW` still yields `EVT safety_stop reason=watchdog`.
- Confirm a bare `T 200 200 1000` (no `stop=`) still produces `EVT done T` with the new
  trailing `reason=time` and that the Python host's `wait_for_evt_done` returns the reason.

## Execution note

This is a CLASI project. Implement via the sprint process (sprint-planner → tickets →
programmer), one sprint per phase, with an execution lock and version bump per commit per
the project rules — not as ad-hoc edits.
