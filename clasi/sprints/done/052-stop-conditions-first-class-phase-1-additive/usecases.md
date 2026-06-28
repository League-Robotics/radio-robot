---
status: ready
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 052 Use Cases

## SUC-001: Attach a stop clause to any open-loop motion command

- **Actor**: Host programmer / script
- **Preconditions**: Robot is idle; firmware running sprint-052 build.
- **Main Flow**:
  1. Host sends `VW 200 0 stop=d:300` (or `S`, `R`, `T`, or `D` with a `stop=` clause).
  2. Firmware parses the `stop=<kind>:<args>` token via the unified stop parser.
  3. A matching `StopCondition` is added to the active `MotionCommand`.
  4. The robot drives until the stop condition fires (in this case ~300 mm travelled).
  5. `MotionCommand::tick()` detects the fired condition; the command terminates.
- **Postconditions**: Robot has stopped; an `EVT done … reason=dist` line has been emitted.
- **Acceptance Criteria**:
  - [ ] `stop=t:<ms>`, `stop=d:<mm>`, `stop=line:<ge|le>:<thr>`, `stop=sensor:<ch>:<ge|le>:<thr>`, `stop=color:<h>:<s>:<v>:<dist>`, `stop=heading:<cdeg>:<eps_cdeg>`, `stop=rot:<arc_mm>` each accepted on VW.
  - [ ] Up to 4 `stop=` clauses may be stacked on one command (kMaxStopConds = 4).
  - [ ] `sensor=line0:ge:512` (legacy form) continues to work as before (back-compat alias).
  - [ ] VW, S, R, T, D each accept `stop=` clauses.
  - [ ] T and D retain their positional time/distance args and may have additional `stop=` clauses OR-combined.

## SUC-002: Host receives the stop reason on every EVT completion

- **Actor**: Host script / robot_radio library user
- **Preconditions**: A motion command is in flight.
- **Main Flow**:
  1. A stop condition fires during `tick()`.
  2. `MotionCommand` records which `StopCondition::Kind` (and channel for SENSOR) fired.
  3. `emitEvt` appends `reason=<token>` after the existing `EVT done … [#id]` base.
  4. The host receives e.g. `EVT done T #12 reason=time`.
- **Postconditions**: Host knows exactly why the motion ended.
- **Acceptance Criteria**:
  - [ ] Each stop kind maps to the correct reason token: `time`, `dist`, `rot`, `heading`, `pos`, `line`, `color`, `<channel>` (SENSOR), `watchdog`.
  - [ ] A bare command with no explicit `stop=` (e.g. `T 200 200 1000`) still emits `reason=time` (the implicit time stop fires).
  - [ ] Reason token is a trailing additive token — existing hosts that match `EVT done T` by prefix still work.
  - [ ] `EVT safety_stop reason=watchdog` is emitted by `Superstructure::evaluateSafety` on watchdog fire.

## SUC-003: Python host builds stop clauses and inspects stop reason

- **Actor**: Python host using `robot_radio.robot.protocol`
- **Preconditions**: `NezhaProtocol` instance connected.
- **Main Flow**:
  1. Host calls `proto.vw(200, 0, stop=[Stop.dist(300)])`.
  2. The library appends `stop=d:300` to the wire command.
  3. Host calls `result, reason = proto.wait_for_evt_done("VW", timeout_ms=5000)`.
  4. Library parses `reason=` from the incoming EVT line and returns it alongside the result.
- **Postconditions**: Caller has both the completion outcome and the stop reason without parsing raw EVT strings.
- **Acceptance Criteria**:
  - [ ] `Stop` builder class provides `Stop.time(ms)`, `Stop.dist(mm)`, `Stop.line(cmp, thr)`, `Stop.sensor(ch, cmp, thr)`, `Stop.color(h, s, v, dist)`, `Stop.heading(cdeg, eps_cdeg)`, `Stop.rot(arc_mm)`.
  - [ ] `vw()`, `drive()`, `arc()`, `timed()`, `distance()`, `turn()` accept optional `stop=[...]` list.
  - [ ] `wait_for_evt_done` returns `(outcome, reason)` tuple; reason is the token string or `None` if absent (backward-compat default).
  - [ ] `parse_response` populates `kv['reason']` when present in an EVT line.

## SUC-004: Documentation reflects new stop= grammar and reason= field

- **Actor**: Developer reading protocol docs
- **Preconditions**: Sprint 052 code is implemented.
- **Main Flow**:
  1. Developer reads `docs/protocol-v2.md` §10 to learn the stop clause grammar.
  2. Developer reads `source/COMMANDS.md` to see `stop=` column in the verb table.
- **Postconditions**: Docs are accurate and self-consistent with the implementation.
- **Acceptance Criteria**:
  - [ ] `docs/protocol-v2.md` §10 documents the `stop=` grammar table for all 7 kinds.
  - [ ] `docs/protocol-v2.md` §10 documents the `reason=` trailing token and lists all reason tokens.
  - [ ] `source/COMMANDS.md` verb table has a `stop=` column or note for VW, S, R, T, D.
