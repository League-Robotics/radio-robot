---
id: '006'
title: Re-validate motion-shape acceptance bar against the configured plant + stakeholder
  bench checklist
status: open
use-cases: [SUC-006]
depends-on: ['005']
github-issue: ''
issue: deadband-compensation-small-commands-must-produce-real-motion.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Re-validate motion-shape acceptance bar against the configured plant + stakeholder bench checklist

## Description

Re-validate the stakeholder's motion-shape acceptance bar against the sim's
*actually configured* plant (`vel_kp=0.002` from `tovez_nocal.json`, not the
pre-sprint-113 hardcoded `0.003` the existing traces were tuned against),
with ticket 005's deadband fix in place. Re-baseline any existing regression
threshold that assumed the old value. Produce a clearly-labeled,
stakeholder-run (not agent-executed) bench checklist covering everything no
agent in this sprint can verify directly.

## Context

Sprint 113 made the sim read `data/robots/*.json`, but every existing
motion-shape trace/threshold in the regression suite was validated back when
the sim silently ran a hardcoded `vel_kp=0.003`. Now that tickets 001-003
have removed every hardcoded fallback and the sim genuinely runs
`vel_kp=0.002` (the configured value), those traces need to be re-checked
against reality — a threshold that happened to pass against `0.003`'s
dynamics is not evidence it holds against `0.002`'s.

Separately, no agent in this sprint has hardware access.
`.claude/rules/hardware-bench-testing.md` requires a stand exercise for any
firmware sprint touching the HAL, motor control, sensing, or the command
protocol — this sprint touches all three (the config gate, the
persisted-tuning flash store, and the deadband fix). The bench checklist
this ticket produces is that gate's deliverable, explicitly not something
this ticket (or any ticket in this sprint) executes itself.

## Approach

1. **Re-run and re-baseline**: run `test_tour_closure_gate.py`,
   `behavior_lock_harness.cpp`, `test_turn_error_characterization.py`, and
   any other trace-shape-asserting test, against a sim configured from
   `tovez_nocal.json` (the now-default, post-ticket-001/002/003 behavior —
   no special setup needed if those tickets landed correctly; if any test
   still needs an explicit `configure_from_robot()` call to pick up
   `vel_kp=0.002`, add it). For each threshold that fails purely because it
   was tuned against `0.003`, re-derive it against `0.002` and document the
   change inline (old value, new value, why) — do not silently widen a
   tolerance without explanation.

2. **Verify the stakeholder's exact shape bar** on at least one
   straight-line and one turn scenario:
   - Wheel-speed trace is a clean trapezoid: smooth ramp-up, hold at max,
     smooth ramp-to-zero.
   - No oscillations anywhere in the trace.
   - No bumps (discontinuities/spikes) at the end of the move.
   - A straight's trace never goes below zero.
   - A turn's trace has exactly one wheel entirely below zero (the mirror
     wheel) — not both, not neither, not a partial dip.

3. **Produce the bench checklist** as a new file (check for precedent —
   search for prior sprints' bench checklist files/locations before picking
   a new one; suggest `docs/bench-checklists/sprint-114-config-and-deadband.md`
   if no existing convention is found) containing, clearly labeled
   **"STAKEHOLDER-RUN — NOT AGENT-EXECUTED"**:
   - The standing `hardware-bench-testing.md` gate items (sensors alive,
     wheels drive with encoders incrementing, round-trip over the real
     link).
   - **This sprint's specific additions**:
     (a) confirm an unconfigured real device (if reachable — e.g. a bench
     rig that hasn't been pointed at a robot JSON) refuses motion and the
     wire reply is `ERR_NOT_CONFIGURED`;
     (b) confirm a live-tuned gain (e.g. push a `heading_kp` change via
     `DEV M <n> CFG`/`SET`) survives a power cycle unchanged, then reflash
     the robot and confirm the same tune is gone (persisted store wiped on
     version mismatch) — this is `Config::PersistedTuning`'s only real
     verification, ticket 004 could not test it;
     (c) drive a move whose terminal correction is known to fall inside the
     historical ~15 mm/s dead zone (e.g. a small residual heading error)
     and visually/telemetrically confirm the wheel actually creeps to
     completion instead of holding flat;
     (d) capture a real wheel-speed trace (via TLM/STREAM) for a straight
     and a turn and eyeball it against the same shape bar step 2 verified
     in sim.

## Files to Touch

- Existing sim regression test files (re-baseline thresholds, add config
  where needed) — enumerate exact files during implementation, expect
  `test_tour_closure_gate.py`, `behavior_lock_harness.cpp`,
  `test_turn_error_characterization.py` at minimum.
- New bench checklist file (location TBD per existing precedent — search
  for prior sprints' bench checklist files before creating a new pattern).

## Acceptance Criteria

- [ ] Every existing motion-shape regression test passes against the sim
      configured from `tovez_nocal.json` (`vel_kp=0.002`), with any
      re-baselined threshold documented inline (old value, new value, why).
- [ ] A captured straight-line wheel-speed trace matches the full shape bar
      (trapezoid, no oscillation, no end bumps, never below zero).
- [ ] A captured turn wheel-speed trace matches the full shape bar
      (trapezoid, no oscillation, no end bumps, exactly one wheel entirely
      below zero).
- [ ] A stakeholder-run bench checklist exists, is clearly labeled as not
      agent-executed, and covers: the standing hardware-bench-testing.md
      gate, the config-gate's real-hardware refusal behavior, the
      persisted-tuning power-cycle/reflash-wipe behavior, the deadband
      fix's real-plant behavior, and a real wheel-speed trace capture.
- [ ] The checklist references exact commands/verbs to run (not vague
      prose) so the stakeholder can execute it without re-deriving them.

## Testing

- **Existing tests to run**: full sim regression suite, focused on
  motion-shape assertions.
- **New tests to write**: none required beyond re-baselining existing
  ones — this ticket is verification-and-documentation, not new production
  code.
- **Verification command**: `uv run python -m pytest` (full suite — this is
  the sprint's closing ticket, the full gate should run clean).
