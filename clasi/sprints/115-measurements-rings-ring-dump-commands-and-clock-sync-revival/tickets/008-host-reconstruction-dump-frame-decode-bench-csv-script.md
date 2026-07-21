---
id: 008
title: 'Host reconstruction: dump-frame decode + bench CSV script'
status: open
use-cases:
- SUC-115-002
depends-on:
- '007'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host reconstruction: dump-frame decode + bench CSV script

## Description

Depends on ticket 007 (the wire shape must exist). Adds host-side support
to receive ticket 007's dump frames and reconstruct them into a CSV.
Satisfies this sprint's own bench gate: "Sim first: dump rings from a sim
run → CSV; then stand: spin wheels, dump, plausible timestamped records."

Per this project's existing convention (diagnostic/test tooling lives
under `src/tests/`, never `src/host/robot_radio/`), this ticket splits
cleanly: the low-level WIRE DECODE is host-package library code (same
category as `read_binary_tlm_frames()`); the ORCHESTRATION/CSV script is
test/diagnostic tooling.

## Implementation Plan

- **Approach — protocol decode** (`src/host/robot_radio/robot/protocol.py`):
  add a decoder for ticket 007's new `ReplyEnvelope` dump arm, mirroring
  `read_binary_tlm_frames()`/`read_pending_binary_tlm_frames()`'s existing
  shape (same underlying frame-drain loop, `SerialConnection`, new
  dispatch branch for the new oneof arm — likely a new
  `_binary_dump_queue`-style path alongside the existing
  `_binary_tlm_queue`, following that module's own established pattern).
  Add a method that drains frames for one dump request until the
  terminator, returning the accumulated records (or raising/timing out if
  the terminator never arrives within a bounded window — dumps must not
  hang forever).
- **Approach — bench script** (new file under `src/tests/bench/`, e.g.
  `ring_dump.py`): sends a `RingDump` command for a given ring selector,
  drains frames via the new protocol.py method, reconstructs them into a
  CSV (one row per record, columns = the record's own fields — `stamp`,
  `v_x`, `v_y`, `omega`, `x`, `y`, `heading` for pose rings; `stamp`,
  `velocity`, `position` for encoder rings). Runnable against BOTH a
  sim-mode connection and real serial (the same script, different
  transport — per architecture-108's own precedent that
  `planner.tour.run_tour()`/`sim_loop.py` already present a
  transport-agnostic interface; this script should not need
  transport-specific branches beyond however connection setup already
  works elsewhere in `src/tests/bench/`).
- **Files to modify**: `src/host/robot_radio/robot/protocol.py`.
- **Files to create**: `src/tests/bench/` dump/reconstruction script
  (exact name at implementer's discretion, e.g. `ring_dump.py`).
- **Testing plan**: this ticket's own acceptance criteria ARE the
  sprint's sim-then-stand gate — see Acceptance Criteria below.
- **Documentation updates**: a brief usage note in the new script's own
  module docstring (how to invoke it, what CSV columns to expect) —
  no separate doc file required.

## Acceptance Criteria

- [ ] `protocol.py` gains a decode/drain function for the new dump reply
      arm, structurally mirroring `read_binary_tlm_frames()`/
      `read_pending_binary_tlm_frames()`.
- [ ] The new bench script lives under `src/tests/bench/`, not
      `src/host/robot_radio/`.
- [ ] **Sim run**: dump each of the five rings via the new script,
      reconstruct to CSV, confirm plausible records (empty CSV for
      `external`; plausible monotonically-increasing `stamp` values for
      the other four, once 005/006 have populated them in the same sim
      session).
- [ ] **Stand run**: spin wheels on the robot (on the stand, per
      `.claude/rules/hardware-bench-testing.md`), dump via the new script
      over serial, reconstruct to CSV, confirm plausible climbing encoder
      positions and nonzero velocities.
- [ ] A dump request that never receives a terminator (e.g. malformed
      firmware reply) times out with a clear error rather than hanging
      the script forever.

## Testing

- **Existing tests to run**: full `uv run python -m pytest` host/sim
  suite.
- **New tests to write**: a host-side unit test for the new decode
  function (feed it a synthetic sequence of dump frames + terminator,
  confirm correct reconstruction and correct done/count handling) in
  addition to the sim/stand exercises in Acceptance Criteria.
- **Verification command**: `uv run pytest`
