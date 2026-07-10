---
id: '006'
title: 'Firmware: retire migrated motion + liveness text families (S/D/T/RT/MOVE/MOVER,
  ECHO/VER, ParsedCommand)'
status: open
use-cases: [SUC-006, SUC-009]
depends-on: ['005']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: retire migrated motion + liveness text families (S/D/T/RT/MOVE/MOVER, ECHO/VER, ParsedCommand)

## Description

Delete the motion and liveness text handlers whose binary replacement is
proven, now that ticket 005's gate confirmed the host is fully on the
binary path for them:

- `parseS`/`handleS` (`S`) — superseded by the binary `drive` arm.
  **Binary parity: 095, hardware-bench-smoke-tested** (drive verified on
  the stand over serial and relay).
- `parseD`/`handleD` (`D`), `parseT`/`handleT` (`T`), `parseRT`/`handleRT`
  (`RT`) — superseded by the binary `segment` arm. **Binary parity: 096,
  sim-exhaustive** (differential-vs-google.protobuf byte-parity + fuzz +
  behavioral tests); hardware bench for `segment`/`MOVE`/`MOVER`
  specifically is deferred to the team-lead's post-sprint consolidated
  session (per `sprint.md`'s own sequencing).
- `parseMove`/`handleMove` (`MOVE`), `parseMover`/`handleMover` (`MOVER`)
  — superseded by the binary `segment`/`replace` arms directly (095's own
  `MotionSegment` message was designed 1:1 for this shape). **Binary
  parity: 095/096, sim-exhaustive**; hardware bench deferred as above.
- `ECHO`'s registration in `systemCommands()` — superseded by the binary
  `echo` arm. **Binary parity: 095, hardware-bench-smoke-tested.**
- `handleVer`/`VER` — its content (`fw`/`proto`) is a strict subset of the
  binary `id` arm's `DeviceId.fw_version`/`.proto_version` fields.
  **Binary parity: 095, hardware-bench-smoke-tested** (`id`).
- `source/types/command_types.h`'s `ParsedCommand` struct — zero
  references anywhere in the tree (grep-confirmed during architecture
  research); mechanical deletion, no binary-parity argument needed.

**The consolidated hardware-in-the-loop bench (team-lead-run, post-sprint)
is the final gate for every binary arm's real-hardware behavior** — this
ticket's own acceptance is `tests/sim` green plus a successful ARM build
with the flash delta recorded, per `.claude/rules/hardware-bench-testing.md`
and the sprint's own bench-gate framing. It does not substitute for that
consolidated session.

**Explicitly PRESERVED, unregistered, byte-for-byte unchanged** (see
`architecture-update.md` Decision 5 — `sprint.md`'s literal "delete...
the stop-clause text grammar" phrasing is NOT honored literally here,
because none of the following have any binary replacement, proven or
otherwise):

- `parseR`/`handleR` (`R`), `parseTURN`/`handleTURN` (`TURN`),
  `parseG`/`handleG` (`G`) — Planner-bound, zero live consumer since
  093/094; 095's own r1 revision explicitly removed the `motion` oneof
  arm from the schema for being oversized and deferred it indefinitely.
- The shared stop-clause grammar helpers: `parseStopClauseValue`,
  `collectStopClauses`, `packStopKVs`, `kMaxStopConds`, `replyStopBadarg`
  (exist only to serve the above).
- `handleTlm` (one-shot `TLM` verb) and `handleQlen` (`QLEN`) —
  bench-diagnostic verbs with no `NezhaProtocol` wrapper and no proven
  binary substitute; not named in the issue's deletion list.
- `StreamingDriveWatchdog` (`motion_commands.h`) — already dead code
  predating this sprint; not actioned.
- The text rump itself: `STOP`, `PING`, `ID`, `HELLO`, `HELP`
  (`system_commands.cpp`, `motion_commands.cpp`'s `handleStop`) — byte-
  for-byte unchanged. A bare-terminal typed `STOP` halting a moving robot
  is the safety affordance this preservation exists for (SUC-009) —
  verified on the bench by the team-lead's post-sprint consolidated
  session, not by this ticket, but this ticket's own job is to guarantee
  nothing in its diff touches `handleStop`'s registration or body.

Also preserved, untouched (separate module boundaries, not this ticket's
concern): `config_commands.{h,cpp}` (ticket 007), `telemetry_commands.
{h,cpp}`/`tlm_frame.{h,cpp}` (ticket 008), `dev_commands.{h,cpp}`,
`otos_commands.{h,cpp}`, `pose_commands.{h,cpp}`.

## Acceptance Criteria

- [ ] `grep -n '"S"\|"D"\|"T"\|"RT"\|"MOVE"\|"MOVER"'
      source/commands/motion_commands.cpp` (registration call sites in
      `motionCommands()` only) returns no hits for these six verbs.
- [ ] `grep -n '"ECHO"' source/commands/system_commands.cpp` (registration)
      returns no hits; `handleVer` is deleted.
- [ ] `grep -rn "ParsedCommand" source/` returns no hits.
- [ ] `parseR`/`handleR`/`parseTURN`/`handleTURN`/`parseG`/`handleG`, the
      shared stop-clause grammar helpers, `handleTlm`, `handleQlen`, and
      `StreamingDriveWatchdog` are all still present and compile
      (unregistered, source unchanged).
- [ ] `STOP`, `PING`, `ID`, `HELLO`, `HELP` registrations and handler
      bodies are byte-for-byte unchanged (diff review, not just grep).
- [ ] `config_commands.{h,cpp}`, `telemetry_commands.{h,cpp}`,
      `tlm_frame.{h,cpp}`, `dev_commands.{h,cpp}`, `otos_commands.
      {h,cpp}`, `pose_commands.{h,cpp}` are untouched by this ticket's
      diff.
- [ ] Any `tests/sim/unit/*` test currently exercising a deleted text verb
      (`S`/`D`/`T`/`RT`/`MOVE`/`MOVER`/`ECHO`/`VER` as TEXT) is re-pointed
      at the equivalent binary arm — coverage is maintained, not dropped.
- [ ] `tests/sim` is green.
- [ ] `just build` (ARM) succeeds; the flash delta (`.map` before/after)
      is measured and recorded in this ticket's completion notes.
- [ ] Completion notes explicitly state: this ticket's own gate is sim +
      ARM-build-clean; the consolidated HITL bench (team-lead, post-
      sprint) is the final real-hardware gate for these binary arms.

## Implementation Plan

### Approach

1. Delete `parseS`/`handleS`, `parseD`/`handleD`, `parseT`/`handleT`,
   `parseRT`/`handleRT`, `parseMove`/`handleMove`, `parseMover`/
   `handleMover` from `motion_commands.cpp`, and their six
   `motionCommands()` registration lines. Leave every other function in
   the file untouched.
2. Delete `ECHO`'s registration line from `systemCommands()` and
   `handleVer` from `system_commands.cpp`.
3. Delete `ParsedCommand` from `command_types.h`.
4. Update any `tests/sim/unit/*` test exercising a deleted verb as text to
   drive the equivalent binary arm instead (per ticket 001-005's own
   established binary send/receive pattern in the sim harness).
5. Build (`just build`), capture the `.map` flash delta.

### Files to modify

- `source/commands/motion_commands.{h,cpp}`
- `source/commands/system_commands.cpp`
- `source/types/command_types.h`
- Affected `tests/sim/unit/*` test files (re-pointed, not deleted, unless
  a test's entire purpose was proving text-verb behavior with no binary
  analog worth keeping — document any such removal explicitly).

### Testing plan

- `tests/sim` full run — must be green.
- `just build` — ARM build must succeed; record `.map` flash delta.
- Grep-clean checks listed in Acceptance Criteria.

### Documentation updates

- None in this ticket (ticket 009 owns the `docs/protocol-v3.md`
  rewrite, done after 006/007/008 land so it documents the final,
  stable surface).
