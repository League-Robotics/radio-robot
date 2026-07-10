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

**REVISED SCOPE — see `architecture-update-r1.md` Decision 8.** The
original plan below (deleting S/D/T/RT/MOVE/MOVER/ECHO/VER) assumed
`NezhaProtocol` was the only host path to these verbs. Ticket 003's own
implementation, plus the team-lead's own follow-up grep, found that is
false: every one of these verbs has at least one LIVE, non-test,
production consumer that reaches the wire directly, bypassing
`NezhaProtocol` entirely, and has NOT migrated to the binary plane:

- **`S`/`T`/`D`/`RT`**: TestGUI's manual command panel
  (`testgui/commands.py`'s `COMMANDS` table + `build_wire_string()`,
  wired into `testgui/__main__.py`) sends these as raw text via
  `transport.command()` for ANY connected transport (Serial/Relay/Sim
  alike) — completely independent of `NezhaProtocol`.
- **`D`**: also live via `calibration/linear.py` (raw pyserial,
  `RelaySerial`/`DirectSerial`).
- **`T`**: also live via `calibration/angular.py` (same raw-pyserial
  transport).
- **`RT`**: also live via `rogo turn`'s DEFAULT (non-`--open-loop`) path
  (`cli.py`'s `cmd_turn`, `proto.send(f"RT {rel_cdeg} #{corr}", ...)` —
  sent directly, not through the Legacy Verb Translator).
- **`MOVE`**: live via `tests/bench/dtr_drive_demo.py`/
  `random_segment_demo.py` (raw text, bypassing the proven binary
  `segment` arm's own `rogo binary segment` path).
- **`MOVER`**: live via `tests/bench/gamepad_teleop.py` (raw text,
  bypassing the binary `replace` arm's own `rogo binary replace` path).
- **`VER`**: live via TestGUI's own connect-time firmware-version check
  (`testgui/__main__.py`, sends raw text `VER`, parses `fw=` out of the
  reply — a connect-critical check, not incidental).
- **`ECHO`**: the only consumer FOUND is a bench protocol-verification
  script (`tests/bench/comms_plane_verify.py`); no production tool was
  confirmed to depend on it, but it is preserved with the rest below
  rather than singled out, given how dense the findings are elsewhere.

None of these are the calibration-tool-only gap ticket 003 surfaced —
TestGUI's command panel and the `rogo turn` default path are NEW findings
from the team-lead's own follow-up sweep, layered on top of ticket 003's.
This is exactly the scenario the issue's own rule exists for: **"a text
family is deleted only after its binary replacement is bench-proven AND
its consumers migrated."** The binary replacements ARE proven (095/096).
Their consumers are NOT migrated. Migrating them is the separate,
already-filed `realign-host-tooling-to-gutted-four-verb-wire-surface.md`
issue's own scope (now updated to explicitly own this work), not
something this ticket can absorb.

**Revised scope: delete ONLY `source/types/command_types.h`'s
`ParsedCommand` struct** — zero references anywhere in the tree
(grep-confirmed both during original architecture research and again for
this revision); a genuinely dead, unregistered, unreachable-by-any-consumer
type, unlike every verb above. **Preserve S/D/T/RT/MOVE/MOVER/ECHO/VER in
full, byte-for-byte unchanged, unregistered status unchanged (still
registered/live exactly as they are today).** This is not a partial
completion of the original plan — it is this sprint's own correct,
evidence-based scope, recorded in `architecture-update-r1.md`.

The original binary-parity evidence below remains true and is preserved
for whenever `realign-host-tooling` clears the way for actual deletion in
a future sprint:

- `S` — superseded by the binary `drive` arm. **Binary parity: 095,
  hardware-bench-smoke-tested** (drive verified on the stand over serial
  and relay).
- `D`/`T`/`RT` — superseded by the binary `segment` arm. **Binary parity:
  096, sim-exhaustive** (differential-vs-google.protobuf byte-parity +
  fuzz + behavioral tests).
- `MOVE`/`MOVER` — superseded by the binary `segment`/`replace` arms
  directly (095's own `MotionSegment` message was designed 1:1 for this
  shape). **Binary parity: 095/096, sim-exhaustive.**
- `ECHO` — superseded by the binary `echo` arm. **Binary parity: 095,
  hardware-bench-smoke-tested.**
- `VER` — its content (`fw`/`proto`) is a strict subset of the binary `id`
  arm's `DeviceId.fw_version`/`.proto_version` fields. **Binary parity:
  095, hardware-bench-smoke-tested** (`id`).

**The consolidated hardware-in-the-loop bench (team-lead-run, post-sprint)
remains the final gate for every binary arm's real-hardware behavior**,
per `.claude/rules/hardware-bench-testing.md` — unaffected by this
revision; it was never contingent on this ticket's own deletion scope.

**Explicitly PRESERVED, unregistered, byte-for-byte unchanged** (see
`architecture-update.md` Decision 5 — `sprint.md`'s literal "delete...
the stop-clause text grammar" phrasing was already NOT honored literally
here, because none of the following have any binary replacement, proven or
otherwise — this revision adds S/D/T/RT/MOVE/MOVER/ECHO/VER to the
preserved set above for a DIFFERENT reason, live unmigrated consumers,
not "no replacement exists"):

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

- [ ] `grep -rn "ParsedCommand" source/` returns no hits.
- [ ] Before any deletion, re-verify (fresh grep, not a stale citation of
      this ticket's own Description) that each of `S`/`D`/`T`/`RT`/
      `MOVE`/`MOVER`/`ECHO`/`VER` still has at least one live consumer
      among: `testgui/commands.py`'s `COMMANDS` table +
      `testgui/__main__.py`'s connect-time `VER` check,
      `calibration/linear.py`, `calibration/angular.py`, `cli.py`'s
      `cmd_turn` default path, `tests/bench/dtr_drive_demo.py`,
      `random_segment_demo.py`, `gamepad_teleop.py`,
      `comms_plane_verify.py`. If — and only if — this fresh check finds a
      SPECIFIC verb's live consumer(s) have since migrated (e.g.
      `realign-host-tooling` landed first), that SPECIFIC verb may be
      deleted following the original binary-parity citations preserved in
      this ticket's Description; do not delete any verb whose consumer
      list still shows a live text sender.
- [ ] `S`/`D`/`T`/`RT`/`MOVE`/`MOVER`/`ECHO`/`VER` registrations and
      handler bodies are byte-for-byte unchanged UNLESS the re-verification
      above found a specific one safe (expected outcome this sprint: all
      eight unchanged).
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
- [ ] `tests/sim` is green (expected: unaffected, since no motion/liveness
      verb is expected to be deleted this sprint).
- [ ] `just build` (ARM) succeeds; the flash delta (`.map` before/after)
      is measured and recorded — expected to be negligible this sprint
      (a single zero-reference struct), not the family-scale reduction the
      issue originally estimated. Do not imply a larger reduction was
      achieved.
- [ ] Completion notes explicitly state: (a) this ticket's own gate is sim
      + ARM-build-clean; the consolidated HITL bench (team-lead,
      post-sprint) is the final real-hardware gate for the binary arms
      that already exist; (b) the motion/liveness text families are
      preserved this sprint per `architecture-update-r1.md` Decision 8,
      deferred to `realign-host-tooling-to-gutted-four-verb-wire-surface.md`.

## Implementation Plan

### Approach

1. Re-verify the live-consumer list above with a fresh grep (do not rely
   solely on this ticket's own Description — the codebase may have moved
   since this revision was written).
2. Delete `ParsedCommand` from `command_types.h`.
3. If (and only if) the re-verification found a specific verb safe,
   delete that verb's `parseFn`/`handlerFn` pair and its
   `motionCommands()`/`systemCommands()` registration, following the
   binary-parity citation already recorded in this ticket's Description
   for that verb. Otherwise, make no further source changes.
4. Build (`just build`), capture the `.map` flash delta (expected
   negligible).

### Files to modify

- `source/types/command_types.h` (`ParsedCommand` deleted)
- `source/commands/motion_commands.{h,cpp}` — untouched, UNLESS the
  re-verification in step 1 found a specific verb safe to delete.
- `source/commands/system_commands.cpp` — untouched, UNLESS the
  re-verification in step 1 found `ECHO`/`VER` specifically safe.

### Testing plan

- `tests/sim` full run — must be green (expected unaffected).
- `just build` — ARM build must succeed; record `.map` flash delta.
- Grep-clean checks listed in Acceptance Criteria, including the
  live-consumer re-verification.

### Documentation updates

- None in this ticket (ticket 009 owns the `docs/protocol-v3.md`
  rewrite; it must now describe S/D/T/RT/MOVE/MOVER/ECHO/VER as still
  LIVE on the text plane, not retired, per this revision).
