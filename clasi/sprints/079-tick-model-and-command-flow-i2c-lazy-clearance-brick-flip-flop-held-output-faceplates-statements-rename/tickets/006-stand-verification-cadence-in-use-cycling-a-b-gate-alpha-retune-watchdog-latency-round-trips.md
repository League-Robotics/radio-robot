---
id: '006'
title: 'Stand verification: cadence, in-use cycling, A/B gate, alpha retune, watchdog
  latency, round-trips'
status: open
use-cases: [SUC-001, SUC-002, SUC-003, SUC-004, SUC-009]
depends-on: ['001', '002', '003', '004', '005']
github-issue: ''
issue:
- i2c-bus-lazy-clearance-timers.md
- tick-model-command-flow-and-the-command-board-design-sketch.md
- rename-wire-lines-to-statements.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Stand verification: cadence, in-use cycling, A/B gate, alpha retune, watchdog latency, round-trips

## Description

Deploy the fully-wired sprint to the robot on the stand and run the
verification sketch from `clasi/issues/tick-model-command-flow-and-the-
command-board-design-sketch.md` ("Verification sketch") and
`clasi/issues/i2c-bus-lazy-clearance-timers.md` ("Acceptance gate — stand
A/B required"). This ticket is the sprint's acceptance gate — success
criteria in `sprint.md` are only satisfied once this ticket's checks pass
on real hardware. Per `.claude/rules/hardware-bench-testing.md`: the robot
is on the stand, wheels free, safe to drive.

**Checks** (each is a discrete, recorded pass/fail, not just "ran without
crashing"):

1. **Encoder cadence + evenness**: with 2 ports in use, poll
   `DEV M <n> STATE` at a fast, fixed interval and measure the per-motor
   sample period. Expect ~11-13 ms (~80-90 Hz), matching the design
   sketch's cadence table. Compare against today's pre-sprint baseline
   (~10 ms nominal, but blocking — capture a `git stash`/prior-tag build if
   a direct A/B baseline is wanted, otherwise cite the sketch's own
   documented "today" column).
2. **In-use-port cycling**: address only 2 of 4 ports; confirm (via
   `DBG I2CLOG` or equivalent) zero bus traffic to the other 2 ports for an
   extended period; then address a 3rd port and confirm it joins the
   cycle from that point on (sticky, no auto-deactivation of the first 2).
3. **Reversal/armor still holds**: command a cruising motor to reverse;
   confirm (via `DEV M <n> STATE`'s `applied=`/timing) the zero-write +
   ~100 ms dwell + ramp-from-zero sequence from 078 is unchanged at the new
   cadence.
4. **Watchdog fire latency**: stop sending statements; measure time from
   last statement to `EVT dev_watchdog` / motors visibly neutral. Expect
   materially better than the pre-sprint ~32 ms worst case, within the
   sketch's ~1 cm-of-motion accepted bound (decision 2 — no escape hatch
   is being added, just measuring the new bound).
5. **Statement round-trips**: serial round-trip is the **required** gate
   (`PING`, `DEV M`, `DEV DT` verbs, replies correct). Radio round-trip is
   **best-effort** — check `mbdeploy list` at execution time; if no relay
   is connected, note that explicitly and do not block the gate on it.
6. **Lazy-timer A/B (the i2c-bus issue's required acceptance gate)**: run
   with and without deliberate settle-window traffic (e.g. an injected
   read to another device, or a scripted stray transaction) and compare
   latch rate, diagnosed from `TLM`/`DEV STATE` encoder-constancy — **not**
   from `EVT` (per `docs/knowledge/2026-07-04-encoder-wedge.md`'s
   diagnosis method). Record the result either way; a positive finding
   (settle-window traffic increases latch rate) blocks sprint close until
   resolved, per the issue's explicit acceptance gate.
7. **Shared-0x10 clobber check**: intentionally abandon a collect (e.g. by
   observing/forcing the HAL to move on past a settle window under load)
   and confirm the next request's readback is not corrupted — the
   structural argument is in `architecture-update.md`'s Migration Concerns;
   this ticket is where it gets a real hardware observation.
8. **`vel_filt_alpha` retune**: bench-tune via step responses at the new
   cadence (per `main.cpp`'s existing `initDefaultMotorConfigs()` comment on
   the `alpha=0` silent-failure precedent); confirm the result holds within
   `pid_hold_speed`-style tolerance bands (see `tests/bench/` scripts for
   the existing tolerance convention). Record the new value(s) and update
   `initDefaultMotorConfigs()`'s bench-placeholder default if the retuned
   value differs meaningfully from today's `0.3`.

## Acceptance Criteria

- [ ] Cadence measurement recorded: per-motor sample period with 2 ports in
      use is within (or better than) the design sketch's ~11-13 ms band.
- [ ] In-use-port cycling confirmed: idle ports generate zero bus traffic;
      a newly-addressed port joins the cycle without disturbing existing
      ones.
- [ ] Reversal/dwell behavior confirmed unchanged at the new cadence
      (078's armor still holds).
- [ ] Watchdog fire latency measured and recorded; within the accepted
      bound.
- [ ] Serial round-trip confirmed working end-to-end (required). Radio
      round-trip confirmed if a relay is connected (`mbdeploy list`
      checked first); if not connected, explicitly noted as skipped, not
      silently omitted.
- [ ] Lazy-timer A/B run; result recorded (pass or a filed follow-up issue
      if it fails).
- [ ] Shared-0x10 clobber check run; result recorded.
- [ ] `vel_filt_alpha` retuned via step response; new value(s) recorded and
      applied to `main.cpp`'s bench-placeholder defaults if changed.
- [ ] 078's standstill-guard constants (`kRestVelocity`/`kRestTicksRequired`)
      watched for spurious/missed hard-reset dispatches during the above —
      if evidence of a problem appears, file a follow-up issue rather than
      silently retuning them in this ticket (per architecture-update.md
      Open Question 2, out of this sprint's scope to change without bench
      evidence).
- [ ] All results are written into this ticket file (or a linked bench
      report) before it is marked done — a stand pass with no recorded
      numbers does not satisfy this ticket.

## Implementation Plan

**Approach**: this ticket is verification, not new source changes (beyond
the `vel_filt_alpha` default update and any follow-up-issue filing) —
deploy, run each of the 8 checks in order (cheapest/lowest-risk first:
cadence and round-trips before the A/B and reversal tests that need more
setup), record results directly in this file.

**Steps**:
1. `mbdeploy probe` then `mbdeploy deploy --build` (per
   `.claude/rules/hardware-bench-testing.md`).
2. `mbdeploy list` — confirm serial path; note whether a relay is present
   for the radio round-trip (best-effort).
3. Run checks 1-5 (cadence, in-use cycling, reversal, watchdog, round-trips)
   via the serial link and `tests/bench/` scripts where they already exist
   (e.g. `dev_exercise.py`, `velocity_chart.py`, `wedge_latch_matrix.py`),
   extending them only as needed for the new cadence/in-use assertions.
4. Run check 6 (lazy-timer A/B) per
   `docs/knowledge/2026-07-04-encoder-wedge.md`'s diagnosis method.
5. Run check 7 (shared-0x10 clobber) — likely needs a small, throwaway
   bench script or a `DBG I2CLOG` inspection around a deliberately-abandoned
   collect; this is the one check that may need new bench tooling.
6. Run check 8 (`vel_filt_alpha` retune) via step-response bench passes,
   comparing against `pid_hold_speed`-style tolerances.
7. Record every result in this file; file follow-up issues for anything
   that fails or needs future work (standstill-guard retuning, OTOS/line/
   color HAL-schedule integration, etc. — per architecture-update.md's Open
   Questions).

**Files to modify** (verification tooling only, not the redesigned
subsystems themselves):
- `source/main.cpp` — `initDefaultMotorConfigs()`'s `vel_filt_alpha`
  default, if retuning changes it.
- `tests/bench/*.py` — extend existing scripts as needed for the new
  cadence/in-use/A-B assertions; a new small script for the shared-0x10
  clobber check if none of the existing ones fit.
- `docs/knowledge/2026-07-04-encoder-wedge.md` — production-guidance status
  line update if the lazy-timer A/B and the flip-flop wiring together
  change its "pending"/"not yet in production firmware" language (mirrors
  078's own ticket-005-gated update to this same file).

**Testing plan**: this ticket **is** the testing plan — a hardware stand
pass with 8 recorded checks. `uv run python -m pytest` should still be run
once beforehand to confirm the full host suite is green before spending
stand time (catching any regression from tickets 001-005 cheaply first).

**Documentation updates**: `docs/knowledge/2026-07-04-encoder-wedge.md`
(above, if applicable); record final cadence/latency/A-B numbers in this
ticket file as the durable record of what was measured.
