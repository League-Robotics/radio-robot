---
id: '006'
title: "Host analysis tooling \u2014 estimator_capture.py + pure-Python one_step_ahead\
  \ reference"
status: done
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

- [x] `src/tests/bench/estimator_capture.py`: drives a scripted MOVE-
      pattern sequence (both directions, steps, reversals, pivots;
      straights and turns) while capturing the TLM stream to CSV via
      `tlm_log.py`'s existing `stream_to_csv()`/`FrameSource` machinery
      (not a reimplementation of frame-to-row logic). Works against a
      real `NezhaProtocol` connection (serial or `--relay`) AND a
      `SimLoop` instance, mirroring `FrameSource`'s existing duck-typed
      protocol.
- [x] `src/tests/tools/one_step_ahead.py`: pure functions, no I/O — given
      a sequence of timestamped per-stream readings (wheel position/
      velocity/time; body heading/omega/time), computes leave-one-out
      one-step-ahead ZOH predictions and residuals per the SAME formula
      ticket 002 ships (`distance = basis.position + basis.velocity ×
      age`; `heading = basis.heading + basis.omega × age`).
- [x] `src/tests/unit/test_one_step_ahead.py`: pytest-collected unit
      tests covering the ZOH prediction math (matches ticket 002's C++
      formula on hand-computed fixtures), staleness/edge cases (single-
      sample stream, non-monotonic timestamps rejected or documented),
      and the leave-one-out walk's own bookkeeping (excludes exactly one
      sample per step, walks the whole stream).
- [x] `src/tests/DESIGN.md` updated in place: `bench/`'s file listing
      gains `estimator_capture.py`; `tools/`'s description gains
      `one_step_ahead.py`; `unit/`'s listing gains
      `test_one_step_ahead.py`.

## Completion Notes (2026-07-22)

- `src/tests/bench/estimator_capture.py`: `MoveSegment`-based scripted
  pattern (`DEFAULT_PATTERN`, 8 legs — forward step, reversal, both-
  direction pivots, then 4 chained short legs mixing straights/a turn),
  driven via `drive_pattern()`/`capture_with_pattern()`. The read side
  reuses `tlm_log.stream_to_csv()` UNMODIFIED, run on a background thread
  concurrently with the driving loop on the calling thread (both
  `SerialConnection`/`NezhaProtocol` and `SimLoop` already run their own
  background reader/tick thread feeding a thread-safe queue, so this is
  safe against either). The write side (`_drive_segment()`) dispatches on
  `hasattr(source, "move_twist")` (real `NezhaProtocol`) vs.
  `hasattr(source, "twist")` (`SimLoop`) — see the module's own header for
  why there is no single shared method: **`SimLoop.move()` is stale,
  pre-116-001 dead code** that builds an `envelope_pb2.Move(distance=,
  delta_heading=, v_max=, ...)` call against fields the CURRENT `Move`
  message no longer has (verified directly — constructing it raises
  `ValueError: Protocol message Move has no "delta_heading" field.`); this
  module never calls it, driving both sources through their own bounded
  TWIST entry point instead (`move_twist()`/`twist()`, both TIME-stop,
  `replace=True`).
- `src/tests/tools/one_step_ahead.py`: `one_step_ahead_walk()` (generic
  leave-one-out ZOH walk over parallel `times`/`positions`/`velocities`),
  `rms()`/`Phase`/`group_rms_by_phase()` (AC's "RMS grouping helpers by
  pattern phase"), plus `wheel_stream_from_rows()`/
  `heading_stream_from_rows()` CSV-row convenience extractors matching
  `tlm_log.CSV_FIELDNAMES` column names 1:1 — no pandas/numpy dependency
  (matches `clock_sync.py`'s own no-external-dependency precedent; ticket
  007's notebook does not exist yet, so there was no established pandas
  dependency to align with either way).
  **Unit-conversion gotcha caught during implementation**: `tlm_log.py`'s
  `pose_theta`/`twist_omega` CSV columns are NOT radians —
  `protocol.TLMFrame.from_pb2()` wire-scales them to compact integers
  (centidegrees / milliradians-per-second) before `frame_to_row()` ever
  sees them. `heading_stream_from_rows()` converts back to radians/rad-
  per-second (matching ticket 002's own C++ formula units) rather than
  importing `protocol.py`'s private `_ANGLE_SCALE` — recomputed from
  first principles (`degrees(1.0) * 100`) since it is fixed unit-
  conversion math, not a project tuning value.
- `src/tests/unit/test_one_step_ahead.py`: 27 tests, all passing — ZOH
  formula against hand-computed fixtures matching
  `app_state_estimator_harness.cpp`'s own scenario numbers (wheel
  distance, negative velocity, rotating heading), leave-one-out
  bookkeeping (N samples → N-1 residuals; an adversarial fixture proving
  sample k's own velocity is never used to predict itself), edge cases
  (empty/single-sample streams, mismatched lengths, non-monotonic
  timestamps → `ValueError`, zero-age step), `rms()`, `group_rms_by_
  phase()` (inclusive boundaries, empty-bucket omission, overlapping
  windows), and both CSV-row extractors (including the unit-conversion
  round trip).
- **Real sim capture run performed as part of this ticket's own
  verification** (`uv run python src/tests/bench/estimator_capture.py
  --sim`): produced a 134-row CSV across the 8-segment pattern (~9s).
  Discovered along the way that a freshly-constructed `SimLoop` is
  UNCONFIGURED (sprint 114's fail-closed configuration-completeness gate
  — `handleMove()` refuses every Move with `ERR_NOT_CONFIGURED` until
  `configure_from_robot()` has run) — a first capture attempt with no
  `configure_from_robot()` call produced an all-zero, perfectly flat
  trace (fire-and-poll `move_twist()`/`twist()` never surfaced the
  rejection). Fixed by configuring from `data/robots/tovez_nocal.json`
  (the same fixture `test_sim_loop.py`/`test_tour_closure_gate.py`
  already use), matching `SimLoop`'s own `track_width` to that config's
  trackwidth. Running `one_step_ahead.py`'s reference walk over the
  resulting real CSV produced sensible, non-trivial numbers: wheel
  one-step-ahead RMS ~1.6-1.8mm (max ~10.9mm during the reversal/pivot
  transients), body-heading RMS ~0.003rad (max ~0.031rad) — the same
  order of magnitude ticket 005's independent C++ sim-system harness
  measured for its own one-cycle-ahead checks. `group_rms_by_phase()`
  correctly bucketed a first-half/second-half split of the same walk.
- No production/`src/firm/` or other host-package code changed — new
  test-tree files plus the `src/tests/DESIGN.md` edits only, per the
  ticket's own Implementation Plan.

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
