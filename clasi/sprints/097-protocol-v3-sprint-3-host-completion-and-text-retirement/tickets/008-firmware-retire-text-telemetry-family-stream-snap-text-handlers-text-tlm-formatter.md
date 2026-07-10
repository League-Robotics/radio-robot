---
id: '008'
title: 'Firmware: retire text telemetry family (STREAM/SNAP text handlers + text TLM
  formatter)'
status: open
use-cases: [SUC-008]
depends-on: ['005']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: retire text telemetry family (STREAM/SNAP text handlers + text TLM formatter)

## Description

Delete `handleStream`/`handleSnap`'s `telemetryCommands()` registrations,
`kStreamSchema`, and `telemetryEmit()`/`Telemetry::buildTlmFrame()` (the
text-only formatter) — now that ticket 005's gate confirmed
`NezhaProtocol.stream()`/`.snap()` (ticket 003) and every internal
`TLMFrame` consumer (ticket 003's nine-file sweep) work correctly over the
binary `stream` arm, including `snap()`'s host-side arm-wait-disarm
synthesis.

`tickTelemetry()`'s `bb.telemetryBinary` branch stays structurally, but
since only the binary `stream` arm can set `bb.telemetryPeriod`/
`.telemetryBinary` once `handleStream` is gone, the text-emission branch
becomes unreachable — remove it (or leave a documented dead-branch note,
per this ticket's own judgment call, cited explicitly either way in the
implementation).

**Binary parity: 096, sim-exhaustive** (differential-vs-google.protobuf
byte-parity + fuzz + behavioral tests for `Telemetry`/`StreamControl`,
plus 096's own periodic-tick acceptance criteria: monotonic `seq=`,
correct on/off behavior). Hardware bench (stream text vs. binary TLM at
matched rates, compare `tlm_drop_rate()`) is part of the team-lead's
post-sprint consolidated session, per `sprint.md`'s own sequencing — this
ticket's own gate is sim + ARM-build-clean, not a substitute.

`Telemetry::tick()`/`buildTelemetryMessage()` (the binary formatter,
shared machinery both planes used to rely on) are UNTOUCHED — only the
TEXT-only `buildTlmFrame()` is deleted. `handleTlm` (one-shot `TLM`
verb, a disjoint text surface per 096's own Step 1 finding) and
`handleQlen` are explicitly OUT of scope (ticket 006's preservation list)
— do not touch `motion_commands.cpp`.

## Acceptance Criteria

- [ ] `STREAM`/`SNAP` are no longer registered as text verbs (grep
      `telemetryCommands()`'s body for `"STREAM"`/`"SNAP"` registration
      calls — none remain).
- [ ] `Telemetry::buildTlmFrame()` (text formatter,
      `source/telemetry/tlm_frame.{h,cpp}`) is deleted.
      `Telemetry::tick()`/`buildTelemetryMessage()` (binary, shared) are
      byte-for-byte unchanged.
- [ ] `tickTelemetry()`'s text-emission branch is removed (or explicitly
      documented as an intentionally-retained dead branch, with rationale
      — pick one and state it in the completion notes).
- [ ] `handleTlm`/`handleQlen` remain registered in `motionCommands()`,
      byte-for-byte unchanged; `motion_commands.cpp` is untouched by this
      ticket's diff.
- [ ] Any `tests/sim/unit/*` test currently exercising text STREAM/SNAP is
      re-pointed at the binary `stream` arm (including a case exercising
      the host's `snap()`-equivalent arm-wait-disarm sequence, or the
      firmware-side portion of it) — coverage maintained, not dropped.
- [ ] `tests/sim` is green.
- [ ] `just build` (ARM) succeeds; the flash delta (`.map` before/after)
      is measured and recorded in this ticket's completion notes.
- [ ] Completion notes explicitly state: this ticket's own gate is sim +
      ARM-build-clean; the consolidated HITL bench (team-lead, post-
      sprint) is the final real-hardware gate, including the "text vs.
      binary TLM at matched rates, `tlm_drop_rate()`" bench criterion from
      the issue's own Sprint 2 bench gate.

## Implementation Plan

### Approach

1. Delete `handleStream`/`handleSnap` and their `telemetryCommands()`
   registrations, and `kStreamSchema`, from `telemetry_commands.{h,cpp}`.
2. Delete `telemetryEmit()` (the text-path caller) and
   `Telemetry::buildTlmFrame()` (`tlm_frame.{h,cpp}`).
3. In `tickTelemetry()`, remove the now-unreachable text-emission branch
   (or document why it's deliberately kept as dead code — state the
   choice explicitly).
4. Confirm `handleTlm`/`handleQlen`/`motion_commands.cpp` are untouched.
5. Update any `tests/sim/unit/*` test exercising text STREAM/SNAP to drive
   the binary `stream` arm.
6. Build (`just build`), capture the `.map` flash delta.

### Files to modify

- `source/commands/telemetry_commands.{h,cpp}`
- `source/telemetry/tlm_frame.{h,cpp}`
- Affected `tests/sim/unit/*` test files

### Testing plan

- `tests/sim` full run — must be green.
- `just build` — ARM build must succeed; record `.map` flash delta.
- Grep-clean checks listed in Acceptance Criteria.

### Documentation updates

- None in this ticket (ticket 009 owns `docs/protocol-v3.md`).
