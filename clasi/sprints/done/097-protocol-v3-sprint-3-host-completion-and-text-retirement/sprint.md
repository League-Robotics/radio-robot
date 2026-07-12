---
id: 097
title: 'Protocol v3 Sprint 3: Host completion and text retirement'
status: done
branch: sprint/097-protocol-v3-sprint-3-host-completion-and-text-retirement
use-cases: []
issues:
- protocol-v3-schema-driven-binary-command-plane-protobuf.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 097: Protocol v3 Sprint 3: Host completion and text retirement

## Goals

Complete the host-side migration to the binary plane and retire the text
grammar down to its minimal rump. This is Sprint 3 (final) of the 3-sprint
protocol-v3 program described in
`clasi/issues/protocol-v3-schema-driven-binary-command-plane-protobuf.md`.
Depends on Sprint 095 (codec foundation) and Sprint 096 (binary telemetry +
config) having landed and bench-proven — this sprint is where the dual-stack
migration style ends: a text family is deleted only after its binary
replacement is bench-proven, and by the end of this sprint every migrated
family has been.

## Problem

Sprints 095/096 stood up the binary plane alongside the text plane without
deleting anything. That leaves both `NezhaProtocol`'s hand-written text
parsers and the generated binary path live simultaneously — the duplication
this whole program exists to remove is still present until the host finishes
converting and the now-redundant text code is deleted.

## Solution

Convert all remaining `NezhaProtocol` methods to envelope builders — the
class's public API is unchanged throughout, so TestGUI, gamepad teleop,
bench scripts, and the MCP server change zero call sites. Verify each of
those consumers against the unchanged API. Build the `rogo` REPL as a
text-v2-to-envelope translator with a `--decode` pretty-printer, so a human
at a terminal keeps typing familiar syntax while the wire carries binary.
Delete: migrated text parse functions, the five SET/GET strcmp chains, the
snprintf TLM/CFG/reply emitters, the stop-clause text grammar, host
`parse_tlm`/`parse_cfg`, and the vestigial `ParsedCommand` type. Retain a
minimal text rump — PING, ID, HELLO, HELP, STOP stay hand-typeable, with
STOP kept explicitly as the bare-terminal safety affordance (a human with a
raw serial terminal and no host program must still be able to halt the
robot). Rewrite `docs/protocol-v2.md` into a protocol-v3 document.

## Success Criteria

- Full binary regression passes over both USB serial and the radio relay on
  the bench.
- A bare-terminal typed `STOP` halts a moving robot (verifies the text rump
  works standalone, with no host program involved).
- A TestGUI session and a gamepad teleop session both run correctly over
  radio against the fully-binary firmware.
- A final flash report is recorded; the issue's own estimate is a net flash
  reduction vs. the pre-protocol-v3 baseline (Sprint 095's dual-stack peak
  was +12-15 KB; this sprint is expected to reclaim more than that via
  deletion).
- Every deletion target listed above is confirmed gone (grep-clean) except
  the named text rump.

## Scope

### In Scope

- Converting all remaining `NezhaProtocol` methods to envelope builders
  (public API unchanged).
- Verifying TestGUI, teleop, bench scripts, and the MCP server against the
  unchanged `NezhaProtocol` API.
- `rogo` REPL: text-v2-to-envelope translator plus `--decode`
  pretty-printer.
- Deleting migrated text parse functions, the SET/GET strcmp chains,
  snprintf TLM/CFG/reply emitters, the stop-clause text grammar, host
  `parse_tlm`/`parse_cfg`, and `ParsedCommand`.
- Retaining the text rump: PING, ID, HELLO, HELP, STOP.
- Rewriting `docs/protocol-v2.md` into a protocol-v3 document.
- Final flash-footprint report (before/after the full 3-sprint program).

### Out of Scope

- Any new binary functionality beyond what 095/096 established — this
  sprint is conversion and deletion, not feature growth.
- Camera-fix / pose-estimation work on the binary plane — Sprint 098 (D),
  which starts only after this sprint lands and must express its FIX/SI/
  ZERO surface as binary `CommandEnvelope` oneof arms (there will be no
  text verb path left to reuse).
- Any further wire-format change beyond what the issue's Sprint 3 breakdown
  specifies (e.g. no COBS framing, no envelope schema changes) — those stay
  out of scope per the issue's explicit deferrals.

## Test Strategy

Full binary regression suite (sim + differential codec tests carried
forward from 095/096) must stay green. Bench gate per
`.claude/rules/hardware-bench-testing.md`: full binary regression over both
serial and relay; a typed `STOP` from a bare terminal halting a moving
robot; a combined TestGUI + teleop session over radio; a final flash report
comparing against the pre-protocol-v3 baseline. Host-side: verify TestGUI,
teleop, bench scripts, and the MCP server all pass their existing test
suites unmodified against the now-binary-only `NezhaProtocol` implementation
(the whole point of the compatibility-shim design is that these call sites
need no test changes, only a green re-run).

## Architecture Notes

`NezhaProtocol` is the deliberate compatibility shim referenced throughout
the issue — its method bodies change (text parse -> envelope build) but its
signatures do not, which is what keeps this sprint's blast radius contained
to `host/robot_radio/robot/protocol.py`, `io/serial_conn.py`, and `io/cli.py`
rather than every consumer. After this sprint, `source/commands/` should be
roughly 1,000-1,300 lines (down from ~4,900 pre-program) per the issue's own
estimate — record the actual figure in the closing ticket. The text rump
(PING/ID/HELLO/HELP/STOP, ~120 lines) is a permanent fixture, not a
temporary migration artifact — it is the safety affordance for a human with
nothing but a serial terminal.

## GitHub Issues

(None — tracked via the CLASI issue file referenced above.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

**r2 note (Eric, 2026-07-10 — see `architecture-update-r2.md` Decision
9): the firmware text plane is gutted unconditionally this sprint; a
`rogo` translator proxy is the host's own text-compatibility story.**
006/007/008 no longer depend on 005 — see r2 for the full model change.

| # | Title | Depends On |
|---|-------|------------|
| 001 | Binary telemetry push-frame queue (fix corr_id=0 drop in SerialConnection) | — |
| 002 | NezhaProtocol core conversion (liveness/drive/config) + Legacy Verb Translator | — |
| 003 | NezhaProtocol telemetry conversion (stream/snap) + 9-file consumer sweep + delete parse_tlm/parse_cfg | 001 |
| 004 | rogo translator proxy: text-v2 socket server fronting the real binary robot connection | 002, 003 |
| 005 | Light end-to-end verification: binary command plane + rogo proxy in sim; testgui baseline | 001, 002, 003, 004 |
| 006 | Firmware: gut the migrated motion + liveness text families | — |
| 007 | Firmware: gut the text config family | — |
| 008 | Firmware: gut the text telemetry family | — |
| 009 | Protocol documentation: pure-binary wire + rump + rogo proxy | 004, 006, 007, 008 |
| 010 | Migration closure: grep-clean verification, line-count and flash/RAM report | 009 |

Tickets execute serially in the order listed for a coherent build-up
(host completion 001-005, firmware gut 006-008, docs 009, closure 010),
even though 006/007/008 have no formal `depends-on` edge on 001-005 —
their deletion is unconditional under Decision 9, not gated on host
readiness. `completes_issue` is `false` on all tickets this sprint
(team-lead's own explicit call on 010, unchanged by r2).
