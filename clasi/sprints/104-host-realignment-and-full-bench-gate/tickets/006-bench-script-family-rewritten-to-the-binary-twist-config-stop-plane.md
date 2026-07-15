---
id: '006'
title: Bench script family rewritten to the binary twist/config/stop plane
status: open
use-cases:
- SUC-016
depends-on:
- '001'
- '003'
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench script family rewritten to the binary twist/config/stop plane

## Description

`tests/bench/rig_dev.py`/`rig_soak.py` (and
`tests/bench/device_bus_bringup.py`,
`tests/unit/test_device_bus_bringup_bench.py`) assume the pre-103
segment/drive wire surface and `Devices::DeviceBus`-era bringup image —
both retired by sprint 103 (103's own Impact on Existing Components
section flags this explicitly as deferred to sprint 104). This ticket
rewrites the interactive and soak bench scripts onto
`twist`/`config`/`stop` + the hardened ack-ring matcher, and resolves the
`DeviceBus`-era bringup script (rewrite against `Preamble`, 103's
replacement, or retire with a documented reason).

Depends on ticket 001 (complete command surface — `config()` needed for
any bench script that wants to exercise config application) and ticket
003 (hardened, shared ack-ring matcher + `TelemetrySecondary` — bench
scripts read telemetry directly, not just through `NezhaProtocol`'s
convenience methods).

## Acceptance Criteria

- [ ] `rig_dev.py` drives the rig interactively over the binary plane:
      connect, send twist/config/stop, observe ack + telemetry, matching
      its pre-103 interactive-session purpose but against the new wire
      surface.
- [ ] `rig_soak.py` runs a sustained twist/stop loop over the binary
      plane, logging: TLM drop rate, fault/event bits over time, encoder
      motion per commanded twist (sanity-checking the plant responds, not
      just that acks arrive). Must run against both direct USB and the
      radio relay (a `--relay`/`--serial` style flag, matching the
      project's existing bench-script convention per
      `.claude/knowledge/relay-transport-and-stand-vs-floor.md` and
      similar).
- [ ] `device_bus_bringup.py`/`test_device_bus_bringup_bench.py` are
      either rewritten against `Preamble` (103's boot-sequencing
      replacement for `DeviceBus`) with equivalent bringup diagnostics, OR
      retired with a documented reason in this ticket's completion notes
      (e.g. "no equivalent diagnostic need exists post-103 because X") —
      a ticket-time call per architecture-update.md's own flagged
      uncertainty, not pre-decided in the plan.
- [ ] Both rewritten scripts are exercised at least once against real
      hardware as part of this ticket's own verification (not deferred
      entirely to ticket 007) — a smoke-level run, distinct from ticket
      007's sustained soak run.

## Testing

- **Existing tests to run**: none directly (these are bench scripts, not
  library code with their own pytest suite) — but any shared helper code
  extracted for both scripts should get unit coverage.
- **New tests to write**: if `rig_dev.py`/`rig_soak.py` share a helper
  module (e.g. a common connect/arm/send-loop function), add unit
  coverage for that helper in isolation (mocked transport), matching the
  project's existing bench-script testing conventions.
- **Verification command**: a real bench session per Acceptance Criteria
  (`.claude/rules/hardware-bench-testing.md` — robot on the stand, wheels
  off the ground); no `pytest` gate substitutes for this.

## Implementation Plan

**Approach**: Read the existing `rig_dev.py`/`rig_soak.py`/
`device_bus_bringup.py` first to understand what operator-facing behavior
must be preserved (connection flow, logging format, CLI flags) versus
what is purely wire-surface plumbing that must change. Reuse
`NezhaProtocol`/`serial_conn.py` from tickets 001/003 rather than
reimplementing envelope construction or ack matching in the scripts
themselves.

**Files to create/modify**:
- `tests/bench/rig_dev.py` — rewritten.
- `tests/bench/rig_soak.py` — rewritten.
- `tests/bench/device_bus_bringup.py`,
  `tests/unit/test_device_bus_bringup_bench.py` — rewritten or retired.

**Testing plan**: covered above.

**Documentation updates**: update any `tests/bench/README.md` (or
equivalent) describing script usage/flags to match the new invocation;
record the bringup-script disposition decision (rewritten vs. retired) in
this ticket's completion notes.

## SUC-016: Bench script family rewritten to the binary twist/stop plane

Parent: `single-loop-firmware-p3-p7-continuation.md` (P6).

- **Actor**: Bench operator running `rig_dev.py`/`rig_soak.py`.
- **Preconditions**: Scripts target the retired pre-103 wire surface and
  `DeviceBus` bringup image.
- **Main Flow**: Rewrite onto the binary plane; retire/rewrite the
  bringup script.
- **Postconditions**: No bench tool targets deleted wire arms or the
  deleted `DeviceBus`.
- **Acceptance Criteria**: see above.
