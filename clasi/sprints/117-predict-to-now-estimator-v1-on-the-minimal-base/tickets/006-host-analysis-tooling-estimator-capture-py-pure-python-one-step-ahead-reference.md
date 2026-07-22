---
id: '006'
title: "Host analysis tooling — estimator_capture.py + pure-Python one_step_ahead reference"
status: open
use-cases:
- SUC-061
depends-on:
- '002'
github-issue: ''
issue: predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host analysis tooling — estimator_capture.py + pure-Python one_step_ahead reference

## Description

Build the two pieces of host-side tooling the stakeholder's validation
methodology (and ticket 007's notebook) needs: a capture script that
drives a varied motion-pattern set while logging the TLM stream, and a
pure-Python, independently-testable reference implementation of the same
zero-order-hold one-step-ahead prediction math ticket 002 builds in C++
— NOT a wrapper calling into the C++ estimator, a genuinely separate
reimplementation, so the notebook's cross-check (and ticket 005's stretch
replay harness) is a real independent check, not the estimator agreeing
with itself.

`src/tests/bench/estimator_capture.py` reuses sprint 115's `tlm_log.py`
(`stream_to_csv()`/`FrameSource` protocol) as its logging backend — it
only adds the MOVE-pattern-driving loop on top, following
`twist_drive.py`'s/`move_protocol_bench.py`'s existing scripted-command
pattern. It must work against BOTH a real serial/relay connection AND
`SimLoop` (sim-first capture, per this project's "one Sim object"
convention and this sprint's own sim-then-bench sequencing).

This ticket depends on 002 only so the Python reference's ZOH formula is
written against the SAME, already-decided math (units, extrapolation
equations) ticket 002 ships in C++ — avoiding two divergent
"predict-to-now" formulas in the tree. It has no dependency on 003/004
(the config/wiring tickets) since it operates entirely on TLM-log CSV
columns sprint 115 already emits.

## Acceptance Criteria

- [ ] `src/tests/bench/estimator_capture.py`: drives a scripted MOVE-
      pattern sequence (both directions, steps, reversals, pivots;
      straights and turns) while capturing the TLM stream to CSV via
      `tlm_log.py`'s existing `stream_to_csv()`/`FrameSource` machinery
      (not a reimplementation of frame-to-row logic). Works against a
      real `NezhaProtocol` connection (serial or `--relay`) AND a
      `SimLoop` instance, mirroring `FrameSource`'s existing duck-typed
      protocol.
- [ ] `src/tests/tools/one_step_ahead.py`: pure functions, no I/O — given
      a sequence of timestamped per-stream readings (wheel position/
      velocity/time; body heading/omega/time), computes leave-one-out
      one-step-ahead ZOH predictions and residuals per the SAME formula
      ticket 002 ships (`distance = basis.position + basis.velocity ×
      age`; `heading = basis.heading + basis.omega × age`).
- [ ] `src/tests/unit/test_one_step_ahead.py`: pytest-collected unit
      tests covering the ZOH prediction math (matches ticket 002's C++
      formula on hand-computed fixtures), staleness/edge cases (single-
      sample stream, non-monotonic timestamps rejected or documented),
      and the leave-one-out walk's own bookkeeping (excludes exactly one
      sample per step, walks the whole stream).
- [ ] `src/tests/DESIGN.md` updated in place: `bench/`'s file listing
      gains `estimator_capture.py`; `tools/`'s description gains
      `one_step_ahead.py`; `unit/`'s listing gains
      `test_one_step_ahead.py`.

## Implementation Plan

**Approach.** `estimator_capture.py` is a thin orchestration layer:
issue a scripted sequence of `move_twist()`/`move_wheels()` calls (varied
speeds/directions/durations covering steady/ramp/reversal/pivot phases),
call `tlm_log.py`'s `stream_to_csv()` (or its constituent pieces) to
capture concurrently — reuse, don't reimplement, the frame-to-row/CSV
logic sprint 115 already built and tested. `one_step_ahead.py` is pure
math: no pandas/numpy dependency required (plain Python, matching
`clock_sync.py`'s own no-external-dependency precedent) unless the
notebook ticket (007) already depends on pandas, in which case align — implementer's
call, documented either way.

**Files to create:**
- `src/tests/bench/estimator_capture.py`.
- `src/tests/tools/one_step_ahead.py`.
- `src/tests/unit/test_one_step_ahead.py`.

**Files to modify:**
- `src/tests/DESIGN.md` — direct edits per Acceptance Criteria.

**Documentation updates:** `src/tests/DESIGN.md`.

## Testing

- **Existing tests to run**: `src/tests/bench/tlm_log.py`'s own test
  coverage (if any — confirm `stream_to_csv()`'s contract is unchanged),
  full `uv run python -m pytest`.
- **New tests to write**: `test_one_step_ahead.py` as described above.
- **Verification command**: `uv run python -m pytest src/tests/unit/test_one_step_ahead.py`.
