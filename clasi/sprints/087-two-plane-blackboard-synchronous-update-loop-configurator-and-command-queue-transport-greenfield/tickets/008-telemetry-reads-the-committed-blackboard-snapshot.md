---
id: "008"
title: "Telemetry reads the committed blackboard snapshot"
status: open
use-cases: [SUC-001, SUC-006]
depends-on: ["002", "007"]
github-issue: ""
issue: plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md
# completes_issue: Controls whether linked issues are archived when this ticket
# is moved to done. Default: true (archive when all referencing tickets are done).
# Set to false (scalar) to suppress archival for ALL linked issues on this ticket.
# Set to a mapping {filename.md: false} to suppress archival per issue filename.
# Use false for tickets that partially address a multi-sprint umbrella issue.
completes_issue: true
# exception: Written by a lower agent when it cannot proceed (see architecture §exception-protocol).
# exception:
#   thrown_by: "programmer"          # "programmer" | "sprint-planner"
#   thrown_at: "2026-05-07T14:23:00Z"
#   attempted: |
#     Description of what was attempted before giving up.
#   conflict: "architecture-update.md §3 — reason the agent is blocked"
#   surface: "internal"              # "user-visible" | "internal"
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Telemetry reads the committed blackboard snapshot

## Description

Adapt `Telemetry` (today driven by `TelemetryState`'s four subsystem
pointers) to read exclusively from the committed `Rt::Blackboard` snapshot
(`x[k+1]`) via a single `tick(now, bb)` call, per `architecture-update.md`'s
Reference code (`telemetry.tick(now, bb)`, called right after commit, in
ticket 007's loop). `TelemetryState` itself is deleted as part of ticket
006's `*State`-struct removal; this ticket is where `Telemetry`'s *own*
internals (its TLM-frame assembly) are re-pointed at blackboard cells
instead of the four pointers. Sequenced after the loop (007) so it can be
verified against the real integration point rather than in isolation only.

## Acceptance Criteria

- [ ] `Telemetry::tick(uint32_t now, const Rt::Blackboard& bb)` reads every
      field its TLM frame emits (`enc`, `pose`, `vel`, `line`, `color`,
      `twist`, `otos`, `ekf_rej` — per `docs/protocol-v2.md`'s TLM field
      bitmask) from `bb`'s state cells, holding no `Subsystems::*`
      reference.
- [ ] TLM frame content (`STREAM` and `SNAP`) is byte-identical to today's
      for the same underlying subsystem state — confirmed by running the
      existing telemetry/TLM tests (`test_tlm_frame.py`,
      `test_tlm_stream_snap.py`) with unchanged asserted behavior.
- [ ] `seq=`/`t=` fields and the `STREAM`-channel binding behavior
      (telemetry binds to whichever channel issued `STREAM`, independent of
      which channel later commands arrive on) are unchanged.
- [ ] Grepping `source/telemetry/` for `Subsystems::` outside comments
      returns nothing.

## Implementation Plan

**Approach.** Modify `source/telemetry/tlm_frame.{h,cpp}` (confirm the
exact file/class that owns `Telemetry`'s assembly logic during
implementation) to take a `const Rt::Blackboard&` instead of four
subsystem pointers.

**Files to modify:**
- `source/telemetry/tlm_frame.{h,cpp}`
- `source/commands/telemetry_commands.{h,cpp}` (already largely rewritten
  in ticket 006 as part of the `*State` deletion — this ticket finishes
  `Telemetry`'s own internals specifically)
- `tests/sim/unit/tlm_frame_harness.cpp`, `test_tlm_frame.py`,
  `test_tlm_stream_snap.py`

**Testing plan:**
- Re-run the existing TLM test suite against the blackboard-driven
  `Telemetry`, confirming byte-identical frame content for equivalent
  state.
- Add a test constructing a bare `Rt::Blackboard` with known field values
  (no live subsystems) and asserting the emitted TLM frame matches —
  directly exercising SUC-002-style isolated testability for `Telemetry`
  too.
- **Verification command**: `uv run pytest tests/sim/unit/test_tlm_frame.py tests/sim/unit/test_tlm_stream_snap.py`

**Documentation updates:** none — `docs/protocol-v2.md`'s TLM format is
unchanged.
