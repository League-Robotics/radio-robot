---
id: '007'
title: 'Bench verification on the stand: D/T/TURN/RT motion accuracy and G spot-check'
status: open
use-cases:
- SUC-002
- SUC-003
- SUC-005
- SUC-006
depends-on:
- '006'
github-issue: ''
issue:
- planner-motion-planning-via-vendored-ruckig.md
- rt-open-loop-overshoot-under-synchronous-update.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench verification on the stand: D/T/TURN/RT motion accuracy and G spot-check

## Description

This is the sprint's REAL acceptance gate. Sim tests (ticket 006) cannot
close this sprint on their own: the sim plant already masks the confirmed
`D`/`T` reverse-spin symptom (architecture-update.md Grounding), and cannot
fully validate real-world turn accuracy either (086/087's `TURN`/`RT`
tolerance bars were bench-tuned against real slip/stiction, not sim's
idealized physics). Per `.claude/rules/hardware-bench-testing.md`, this
sprint (touching the Planner's motion-generation core) requires exactly
this gate. The robot is mounted on a stand with wheels off the ground —
safe to drive freely.

## Implementation Plan

**Approach**:
1. **Identify the robot, not the relay.** Run `mbdeploy list` and confirm
   the ROLE column before touching anything — never blind-flash
   (`.clasi/knowledge/verify-microbit-before-flashing.md`). Two
   micro:bits may be connected (robot + relay dongle); target the
   confirmed robot device explicitly.
2. **Deploy the sprint's final firmware** (tickets 001-006 all landed) to
   the confirmed robot. Check `.clasi/knowledge/` for the current
   known-good deploy recipe before assuming `mbdeploy deploy --build`
   works cleanly (prior sessions found a venv gap requiring `just
   build-clean` + `mbdeploy deploy <full-UID> --hex MICROBIT.hex` as a
   fallback).
3. **Safety first.** Before driving anything: widen the DEV serial-silence
   watchdog for the session (`DEV WD <large-ms>`) and feed it as needed
   during longer exercises. Wrap the ENTIRE exercise in a try/finally (or
   equivalent script structure) that ALWAYS sends `DEV STOP` and restores
   the watchdog to its default on exit or exception — motors must never be
   left running or the watchdog left widened.
4. **`D`/`T` no-reverse and no-overshoot** (SUC-002/SUC-003): issue
   `D 200 200 1000` and `T 200 200 1000` (the exact commands from the
   original hardware-confirmed bug report) over serial. Capture
   encoder/telemetry (`TLM`/`STREAM`) through and past `EVT done`.
   Confirm: (a) NO reverse encoder motion after `EVT done` (the confirmed
   bug was ~16 mm for `D`, ~23 mm for `T`); (b) peak commanded/measured
   wheel speed does not exceed the commanded 200 mm/s by more than the
   existing ratio-governor/PID tolerance (the confirmed bug was a ~292
   mm/s overshoot on a commanded 200).
5. **`TURN`/`RT` no-reverse AND accuracy re-verification** (SUC-005 — the
   stakeholder's explicitly accepted added risk): issue a `TURN <heading>`
   (a ~90° turn) and an `RT <relAngle>` command. Confirm: (a) NO reverse
   encoder motion after `EVT done`; (b) heading/rotation accuracy measured
   against the SAME numeric tolerance bars 086/087 established — pull the
   exact bar from those sprints' own artifacts and
   `tests/sim/unit/test_motion_commands_arc_turn.py`'s existing assertions
   (architecture-update.md Open Question 6 — do not re-derive or guess the
   number here). A regression on this bar is a sprint-blocking failure,
   not a footnote.
6. **`G` spot-check, not full re-verification** (SUC-006): issue one `G
   <x> <y> <speed>` command, confirm it still dispatches and settles
   (reaches the target region, emits a completion event) — this is a
   smoke check that the untouched `VelocityRamp`/`pursueSteer()` path
   still works end-to-end on real hardware, not a full accuracy
   re-verification (`G`'s own accuracy was not touched by this sprint).
7. **Characterize on-target Ruckig solve time** (architecture-update.md
   Open Question 4 — this ticket, not ticket 001, is where this is
   actually measurable: it needs live hardware AND a real goal being
   solved, which only exists once tickets 003-005 land). Use existing
   `DBG`/bench instrumentation to capture a rough per-solve timing number
   for at least one linear (`D`) and one rotational (`TURN`/`RT`) solve.
   Record whether this leaves adequate headroom within the control loop's
   own period, and whether it changes the feasibility assessment for a
   future per-tick `GOTO_GOAL` solve (explicitly out of THIS sprint's
   scope, but the number matters for planning the follow-on sprint).
8. **Radio relay round-trip**: exercise at least one motion verb over the
   radio relay path in addition to direct serial, confirming the fix holds
   over that transport too (matching sprint 088's own bench ticket
   precedent) — not required to be exhaustive, but not serial-only either.
9. **Write the bench log.** Capture pass/fail per verb, the measured
   accuracy numbers against their historical bars, and the solve-time
   characterization in a bench log file in the sprint directory.
10. **[Revision 2, post-stakeholder-design-discussion] Completion-mode
    criterion** (architecture-update.md Decision 10) — every bench `D`/
    `TURN`/`RT` must complete via ITS OWN stop condition, never the
    `STOP_TIME` safety net. For each of `D 200 200 1000`, `TURN <heading>`,
    `RT <relAngle>`: capture the `EVT done` reason field and confirm it is
    the goal's OWN completion reason (`reason=dist` for `D`, the
    heading/rotation equivalent for `TURN`/`RT`), not a `STOP_TIME`-net
    completion. A `STOP_TIME`-net completion on any of these is a
    SPRINT-BLOCKING failure — it is the exact stall-short symptom the
    divergence-triggered replan (Decision 10) exists to close, not a
    footnote. Also confirm terminal position/heading error is within the
    existing bar (the same numbers already captured in steps 4-5 above)
    even under whatever real tracking lag the bench run exhibits — i.e.
    this criterion is evidence the divergence replan actually kept the plan
    honest against real plant lag, not just that the sim's idealized plant
    never diverged.
11. **[Revision 2] Terminal-chatter characterization** (architecture-
    update.md Open Question 7). Near the end of at least one `D` and one
    `TURN`/`RT` run, observe whether a near-rest divergence replan repeats
    (chatter) rather than converging cleanly. Record: does chatter occur;
    how many replans fire in the terminal region; whether it self-resolves
    via the rate limiter or persists. This is a CHARACTERIZATION pass, not a
    fix — per this document's own anti-speculative-generality discipline,
    do NOT pre-build a mitigation (completion tolerance / replan cap /
    minimum-velocity floor) in this ticket; record findings and, if
    terminal chatter is observed and judged a problem, raise a follow-on
    issue rather than patching mid-bench.

**Files to create/modify**: a new bench checklist/log file in the sprint
directory (e.g. `bench-verification-log.md`); a `tests/bench/` CLI helper
only if an existing one doesn't cleanly cover a needed verb/measurement
(per `tests/CLAUDE.md`, these are HITL Python tools, not pytest-collected).

**Testing plan**: this ticket IS the test (HITL, not pytest-automated). Run
the full sim gate (ticket 006's consolidated suite) immediately before the
bench pass as a sanity check — it does not substitute for the bench pass
itself.

**Documentation updates**: the bench log itself. If the bench run surfaces
a new defect (e.g. the accuracy bar IS regressed, or a genuine footprint/
solve-time problem), raise a new issue rather than silently patching
mid-bench, and follow the exception protocol if it blocks this ticket's
own acceptance.

## Acceptance Criteria

- [ ] Robot vs. relay identified via `mbdeploy list`'s ROLE column before
      any flash; firmware deployed to the confirmed robot device.
- [ ] DEV serial-silence watchdog widened for the session and fed/restored
      correctly; `DEV STOP` (and watchdog restore) executed in a
      `finally`-equivalent for the entire exercise — motors never left
      running.
- [ ] `D 200 200 1000`: NO reverse encoder motion measured after `EVT
      done`; peak commanded/measured wheel speed within the existing
      ratio-governor/PID tolerance of the commanded 200 mm/s (no
      292-vs-200-style overshoot).
- [ ] `T 200 200 1000`: NO reverse encoder motion measured after `EVT
      done`.
- [ ] `TURN <heading>` (~90°): NO reverse encoder motion after `EVT done`
      AND heading accuracy within 086/087's existing tolerance bar (bar
      value pulled from those sprints' artifacts/existing test
      assertions, not guessed).
- [ ] `RT <relAngle>`: NO reverse encoder motion after `EVT done` AND
      rotation accuracy within 086/087's existing tolerance bar.
- [ ] `G <x> <y> <speed>`: dispatches and settles (reaches target region,
      emits completion) — smoke-checked, not fully re-verified.
- [ ] On-target Ruckig solve time characterized for at least one linear
      and one rotational solve; headroom within the control loop period
      assessed and recorded.
- [ ] At least one motion verb exercised over the radio relay path in
      addition to direct serial.
- [ ] Written bench log committed to the sprint directory, distinguishing
      verified results from any stand-limited or deferred items.
- [ ] `uv run pytest tests/sim` green (ticket 006's consolidated suite) at
      the last firmware ticket before the bench pass, re-confirmed at
      close.
- [ ] **[Revision 2]** `D`/`TURN`/`RT` each complete via their OWN stop
      condition (`EVT done reason=dist` / heading / rotation), never via
      the `STOP_TIME` safety net — verified from captured telemetry, not
      assumed.
- [ ] **[Revision 2]** Terminal near-rest replan-chatter behavior is
      characterized (present/absent, frequency, self-resolving or not) and
      recorded in the bench log; no mitigation is pre-built.

## Testing

- **Existing tests to run**: `uv run pytest` (full sim gate, as a
  pre-bench sanity check).
- **New tests to write**: none (HITL bench pass, not pytest content);
  optionally a `tests/bench/` CLI helper if needed for the exercise
  itself, or to characterize solve time. **[Revision 2]** The
  completion-mode criterion (`EVT done` reason field) and the
  terminal-chatter characterization are captured from the SAME telemetry
  streams already being recorded for steps 4-5 above — no separate
  instrumentation pass needed.
- **Verification command**: N/A (HITL) — the deliverable is the bench log,
  cross-checked against a green `uv run pytest`.
