---
id: '001'
title: 'encpose: feed the dead-reckoner on every frame, gate only the trace append'
status: open
use-cases:
- SUC-073
depends-on: []
github-issue: ''
issue: encpose-active-gate-freezes-dead-reckoner-before-motion-ends.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# encpose: feed the dead-reckoner on every frame, gate only the trace append

## Description

Display-only bug, smallest and lowest-risk ticket in this sprint. After a
single managed 360 deg turn, telemetry shows `pose theta +359.7` /
`otos theta -0.1` (both correct) but `encpose theta +349.1` — 10.6 deg short —
with `enc L -401 R +401` (correct totals on screen).

Root cause is already pinned (issue
`encpose-active-gate-freezes-dead-reckoner-before-motion-ends.md`):
`TraceModel.feed()` (`src/host/robot_radio/testgui/traces.py`) returns
immediately on `frame.active is False` (the idle-trace-growth guard), so
`EncoderDeadReckoner` never ingests the tail of the motion — the taper end, the
final cycle between the last active frame and rest, and the plant coast
(~+-11 mm of wheel travel here). Reproduced headlessly to the decimal:
replaying the run's frames through `EncoderDeadReckoner(128)` gives +359.4 deg
fed EVERY frame and +349.1 deg fed only `active=True` frames — exactly what the
GUI does. Real motion, counted by firmware odometry and OTOS/truth, invisible
to encpose; it accumulates across a tour and reads as a phantom sensor
disagreement.

No control consumes `encpose` — this is display/telemetry fidelity only. No
firmware or wire change.

## Approach

In `TraceModel.feed()`:

1. Advance the encoder dead-reckoner — `EncoderDeadReckoner.update(*frame.enc)`
   and `self.last_encpose` — on EVERY frame carrying `enc`, regardless of
   `frame.active`. The motion tail is real travel the O(1) integrator must not
   miss.
2. Apply the `active` / `_TRACE_IDLE_EPSILON_CM` gates ONLY to the trace-point
   APPEND (`_append_if_moved()`), exactly as today, so the idle-trace-growth
   problem the `active is False` early-return was added to solve stays solved
   (only the polylines grow; the reckoner is O(1) state).

Restructure the current single `if frame.active is False: return` early-return
so it no longer starves the integrator. Concretely, split "advance the
integrator" from "append a trace point": run `_dead_reckoner.update()` /
`last_encpose` unconditionally on enc-bearing frames, and gate the append
inside (or at the call to) `_feed_encpose()` — including keeping the
`encpose_baseline`/`encoder_yaw` bookkeeping correct on the frames where no
point is appended. Equivalent alternative allowed by the issue: keep
integrating until both wheel velocities read zero.

Consider whether `otos`/`fused` traces need the same integrator-vs-append
split: they diff an already-absolute firmware pose against a baseline (not an
incremental integrator), so they do NOT fall behind the way the encoder
reckoner does — but confirm the append-gating for them stays consistent (idle
growth still prevented). Do not change their baseline semantics.

Naming/units per `.claude/rules/coding-standards.md` and
`.claude/rules/naming-and-style.md` — no units in identifiers; Python unit tags
in `# [unit]` trailing comments where a new quantity is introduced.

## Files to modify

- `src/host/robot_radio/testgui/traces.py` — `TraceModel.feed()` (and, if the
  gating is pushed down, `_feed_encpose()`).
- `src/tests/testgui/test_traces.py` — new unit coverage (below).
- `src/host/robot_radio/DESIGN.md` — a short note on the integrator-vs-append
  split if the encoder dead-reckoner behavior is described there (edited
  directly on the canonical doc; not overlaid — see sprint.md Design Overlay).

## Acceptance Criteria

- [ ] `TraceModel.feed()` advances `EncoderDeadReckoner.update()` /
      `last_encpose` on every frame carrying `enc`, including frames with
      `active is False` (the motion tail).
- [ ] The trace-point append remains gated by `active` / the idle epsilon: a
      genuinely idle connection does not grow the encoder polyline without
      bound (the idle-growth guard the early-return provided is preserved).
- [ ] `docs/code_review/2026-07-22-turn-execution-review-scripts/encpose_check.py`
      shows all-frames == GUI-fed after the fix (the +359.4 vs +349.1 gap on a
      360 deg turn is closed); on a managed 360 deg turn `encpose` reads within
      ~1 deg of firmware `pose`.
- [ ] A `test_traces.py` case feeds a synthetic frame sequence with a motion
      tail after `active` drops to False and asserts BOTH: the reckoner ingested
      the tail (final `encpose`/`last_encpose` reflects the full wheel travel),
      AND the trace list did not grow while idle.
- [ ] No firmware/wire change; no control consumer of `encpose` is affected.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/testgui/test_traces.py`
  and the broader `uv run python -m pytest` sim gate (no regressions);
  `test_gui_button_acceptance.py` if the encoder trace/avatar is exercised
  there.
- **New tests to write**: the `test_traces.py` motion-tail case above; optional
  re-run of `encpose_check.py` as a manual/replay confirmation.
- **Verification command**: `uv run python -m pytest src/tests/testgui/test_traces.py`
