---
id: "096"
title: "Protocol v3 Sprint 2: Binary telemetry and config plane"
status: roadmap
branch: sprint/096-protocol-v3-sprint-2-binary-telemetry-and-config-plane
use-cases: []
issues:
- protocol-v3-schema-driven-binary-command-plane-protobuf.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 096: Protocol v3 Sprint 2: Binary telemetry and config plane

## Goals

Extend the binary command plane landed in Sprint 095 (A) to cover telemetry
and config — the two remaining high-traffic surfaces the text plane still
carries. This is Sprint 2 of the 3-sprint protocol-v3 program described in
`clasi/issues/protocol-v3-schema-driven-binary-command-plane-protobuf.md`.
Depends on Sprint 095 (codec foundation, `wire_runtime`, `BinaryChannel`,
generated field-descriptor tables) having landed and bench-proven.

## Problem

Telemetry replies are still assembled by hand-written `snprintf` emitters,
and config get/set still runs through five parallel `strcmp` config-key
chains plus 47 hand-written range checks scattered across
`source/commands/`. The generated schema pipeline (extended in 095 with
validation options and field-descriptor tables) can drive all of this
mechanically, but nothing wires telemetry or config onto the binary plane
yet — they are the two verb families still fully carried by the text
grammar established pre-protocol-v3.

## Solution

Add a Telemetry reply arm to `ReplyEnvelope` plus `StreamControl.binary` to
select the binary emitter; construct the host `TLMFrame` dataclass directly
from the decoded protobuf `Telemetry` message. Mark config fields
`proto3 optional` so the generator maps them to `Opt<T>`; generate a
config-merge function (apply-present-fields) that replaces the five strcmp
chains on the binary path, and a generated `ConfigSnapshot` slice encoder
that replaces the CFG snprintf emitters. Generated validation (from the
`min`/`max`/`abs_max`/`req` options added in 095) replaces the 47 hand
range checks on the binary path. Host gains binary set/get config calls
through the same envelope machinery from 095. Retool
`scripts/check_config_sync.py` to diff the pydantic config model against
the generated pb2 descriptors instead of its current mechanism, closing the
config-sync divergence risk the issue calls out.

## Success Criteria

- Text-vs-binary TLM streamed at matched rates on the bench shows no
  regression in `tlm_drop_rate()` for the binary path relative to text.
- A gamepad teleop session runs cleanly on binary TLM with Ack `q`/`rem`
  flow control (the existing MOVE/MOVER flow-control mechanism, now
  exercised over the binary reply channel).
- Every config slice round-trips correctly over the binary set/get path on
  the bench.
- Changing a PID gain over the binary config path produces an observable,
  correct wheel-behavior change on the stand.
- `check_config_sync.py` runs clean against pydantic vs. pb2 descriptors.

## Scope

### In Scope

- `ReplyEnvelope` Telemetry arm; `StreamControl.binary` selector.
- Host `TLMFrame`-from-pb2 construction.
- `proto3 optional` config fields; generator `Opt<T>` mapping (already
  present in the generator per the issue's "key asset already in the tree"
  note — this sprint exercises it for config specifically).
- Generated config merge (apply-present-fields), replacing the five strcmp
  config chains on the binary path.
- Generated `ConfigSnapshot` slice encode, replacing the CFG snprintf
  emitters on the binary path.
- Generated validation replacing the 47 hand range checks on the binary
  path.
- Host binary set/get config calls.
- `scripts/check_config_sync.py` retooled to diff pydantic vs. pb2
  descriptors.

### Out of Scope

- Deleting the text-plane TLM/CFG snprintf emitters or the strcmp chains
  themselves — they stay live (dual stack) until Sprint 097 (C) proves the
  binary replacement and retires text.
- `NezhaProtocol` public-API conversion, `rogo` REPL translator — Sprint 097
  (C).
- Any config field or verb not already covered by the parked-093/094 text
  families (config/pose/otos/dev) that 095's binary arms and this sprint's
  config-plane work reactivate — new functionality beyond restoring existing
  parked coverage is out of scope; see Sprint 098 (D) for the pose/OTOS
  content itself.

## Test Strategy

Bench-focused per `.claude/rules/hardware-bench-testing.md`: stream text vs.
binary TLM at matched rates and compare drop rate; run a gamepad teleop
session on binary TLM plus Ack-based flow control; round-trip every config
slice over the binary path; change a PID gain over binary and observe the
resulting wheel behavior on the stand. Sim-side: extend/adapt the
parked-093/094 config and telemetry test coverage noted as a risk in the
issue ("parked text families have no live regression tests") onto the new
binary arms so they have fresh sim coverage, not just bench coverage.

## Architecture Notes

Builds directly on 095's `wire_runtime`, `BinaryChannel`, and generated
field-descriptor tables — no new codec mechanism, only new message types and
generator emission targets (Telemetry, ConfigDelta/ConfigSnapshot merge and
encode). Config presence semantics (`Opt<T>` reaching `configure()` paths) is
flagged in the issue as risk #4, quarantined to this sprint — do not let it
leak into 095's or 097's scope. Note per the issue: the currently-parked
text families (config/pose/otos/dev, unregistered since 093/094) get their
functionality back through binary arms in 095/096 — the text versions stay
parked (not yet deleted) until Sprint 097.

## GitHub Issues

(None — tracked via the CLASI issue file referenced above.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
