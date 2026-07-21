---
id: "008"
title: "New tlm_log.py: stream telemetry frames to CSV"
status: open
use-cases: [SUC-047]
depends-on: ["007"]
github-issue: ""
issue: ""
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

# New tlm_log.py: stream telemetry frames to CSV

## Description

New tool: `src/tests/bench/tlm_log.py`. Per sprint.md's Architecture
Decision 2 ("frame-is-the-dataset"), this is the sole
dataset-construction path this program now has — with on-chip
measurement rings deleted (ticket 002) and every-cycle emission in place
(ticket 005), the host-side CSV log is what reconstructs any time window
for future analysis, including sprint 117's estimator work, which
depends on this tool's output existing and being correct. Narrowly
scoped: stream frames, write rows. No analysis logic belongs here (that
is explicitly a future sprint's job, per sprint.md's module boundary for
this file).

## Acceptance Criteria

- [ ] `src/tests/bench/tlm_log.py` connects to a robot (serial, matching
      this repo's existing bench-script connection pattern — see
      `twist_drive.py`/`rig_soak.py` for the established convention) and
      streams `Telemetry` frames via `protocol.py` (ticket 007).
- [ ] One CSV row per frame; columns cover every field: `now`, `seq`,
      `mode`, `flags` (raw, plus decoded convenience columns for at
      least the presence/fault/event bits used in the hardware gate),
      `ack_corr`/`ack_err`, both `EncoderReading`s' `position`/
      `velocity`/`time`, the `OtosReading`'s `x`/`y`/`heading`/`v_x`/
      `v_y`/`omega`/`time` (blank/NaN when `otos_present` is clear),
      `pose` (x/y/theta), `twist` (v_x/omega — differential drive, no
      `v_y` component on the wire twist), unpacked `line`/`color`
      per-channel values.
- [ ] Command-line invocable for a bench session (matches the existing
      `src/tests/bench/` scripts' own CLI convention — check
      `twist_drive.py`/`rig_soak.py` for the pattern already
      established, don't invent a new one).
- [ ] A short synthetic/sim-backed test confirms the CSV row shape and
      column count are stable (e.g., feed it a few `SimHarness`-emitted
      frames and check the written CSV's header + row count) — full
      hardware-session capture is ticket 010's own acceptance criterion,
      not this one's.

## Implementation Plan

**Approach**: Follow the existing `src/tests/bench/` script conventions
for connection setup/CLI args (grep `twist_drive.py`/`rig_soak.py`
first — do not invent a new connection pattern for one tool). Keep the
row-assembly function pure/testable (frame-in, row-dict-out) separate
from the CLI/file-I/O wrapper, so the synthetic test doesn't need a real
or simulated serial connection.

**Files to create**: `src/tests/bench/tlm_log.py`.

**Files to modify**: none (purely additive).

**Testing plan**: A small `src/tests/bench/`-local or
`src/tests/unit/`-local test (match wherever this repo's other bench
scripts' own tests live, if any exist — otherwise colocate a minimal
one) constructing a few frames (hand-built or via ticket 006's
`wire_test_codec` helpers) and asserting the CSV row-assembly function
produces the expected columns/values. `uv run python -m pytest` on
whatever path this lands in, green.

**Documentation updates**: none required beyond the tool's own
docstring/CLI `--help` text.
