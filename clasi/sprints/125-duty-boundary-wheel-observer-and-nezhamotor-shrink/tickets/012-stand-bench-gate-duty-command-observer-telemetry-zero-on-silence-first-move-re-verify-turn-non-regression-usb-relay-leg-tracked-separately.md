---
id: '012'
title: 'Stand bench gate: duty command, observer telemetry, zero-on-silence, first-MOVE
  re-verify, turn non-regression (USB); relay leg tracked separately'
status: open
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-005
- SUC-009
depends-on:
- '001'
- '008'
- '009'
- '010'
- '011'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Stand bench gate: duty command, observer telemetry, zero-on-silence, first-MOVE re-verify, turn non-regression (USB); relay leg tracked separately

## Description

The sprint's own acceptance gate per `.claude/rules/hardware-bench-
testing.md`, run on the stand over USB (a relay dongle is not connected
this session — the relay leg is tracked separately below and stays
`[relay-dongle-required]`, unverified, not silently implied by a USB
pass). Hardware IS reachable as of 2026-07-26 (v5 flashed, `wire_truth.py`
clean, `move_protocol_bench.py` mostly clean) — this ticket can and must
actually run, not merely be written and left unexecuted.

**Every criterion below requires POSITIVE evidence, not just the absence
of a fault** — the 124 lesson: a bench gate reported PASS for a
fault-free session having observed zero telemetry frames, and the same
vacuous shape was then found two levels deeper (inside `wire_truth.py`'s
own report). "No `ERR_*` observed" is never sufficient on its own; pair
it with a positive, counted observation (a nonzero frame count, a
measured encoder delta, an observed ack, a captured divergent
raw-vs-observed pair).

## Acceptance Criteria

- [ ] **Duty command completion**: a `move_wheels`-shaped duty command
      starts and stops the physical wheel on command, BOTH directions.
      Positive evidence: a nonzero, direction-correct encoder delta
      captured for each direction (report the actual delta values, not
      just PASS/FAIL).
- [ ] **Observer telemetry**: raw and observed values are both visible in
      telemetry, per wheel, every frame, across the whole gate run.
      Positive evidence: report the actual frame count observed (must be
      nonzero and consistent with the run duration at the expected rate —
      catches the exact 124 vacuous-PASS shape) AND at least one captured
      frame pair where raw and observed values visibly diverge (e.g.
      during a commanded transient), not merely two populated-but-never-
      diverging fields.
- [ ] **Zero-on-silence, live**: after a `STOP` or completed `Move` with
      an empty queue, the wheel duty reads exactly 0 for the remainder of
      the observation window. Positive evidence: report the number of
      post-stop frames observed and confirm all read duty 0 (not "no
      motion observed" alone — capture the actual telemetry values).
- [ ] **`NezhaMotor` line count**: confirmed at or near the ~200-line
      target on the ACTUAL built firmware for this bench run (not just a
      source-tree grep done earlier in the sprint — confirm the binary
      under test matches).
- [ ] **First-MOVE re-verification (ticket 001's fix, under the FINAL
      duty-based loop)**: `move_protocol_bench.py`'s
      `scenario_distance_stop` acks and executes on the first `MOVE`
      after a fresh connect, 5/5 runs. Positive evidence: 5 observed
      acks + 5 observed nonzero encoder deltas (this re-verifies the fix
      still holds after tickets 002-011 rewrote the loop it lives in —
      ticket 001's own bench verification ran against the PRE-duty tree).
- [ ] **Turn non-regression** (Design Rationale Decision 9): a `TURN` at
      the session's own 0.5 rad baseline target stays within a stated
      delta (e.g. ±15 percentage points) of the confirmed ~0.70 rad /
      +41% baseline measured 2026-07-26. This is explicitly a
      non-regression check, NOT a tolerance-pass requirement — report the
      actual measured value and the delta from baseline, and state
      plainly that landing outside the original ±25% bench tolerance is
      expected and NOT a 125 failure (126's problem to close).
- [ ] **[relay-dongle-required, tracked separately, NOT required for 125's
      own close]**: the duty-command-completion and observer-telemetry
      criteria above, re-run over the radio relay data plane. If no
      dongle is available before this sprint closes, state that
      explicitly in this ticket's completion notes (matching 124's own
      disclosed-unverified precedent) — do not mark this criterion done
      on a USB pass.

## Testing

- **Existing tests to run**: `src/tests/bench/move_protocol_bench.py`,
  `src/tests/bench/wire_truth.py` (or its promoted successor), a `TURN`-
  specific bench script (reuse whatever measured the 0.70 rad baseline
  this session, so the comparison is apples-to-apples).
- **New tests to write**: none new — this ticket runs and reports against
  existing bench scripts plus this ticket's own explicit reporting
  requirements (frame counts, deltas, ack counts) layered on top of
  whatever those scripts already print.
- **Verification command**: run each bench script against
  `/dev/cu.usbmodem2121102` (or the session's actual port) and record
  actual output in this ticket's completion notes — PASS/FAIL alone is
  not sufficient closure evidence for this ticket.
