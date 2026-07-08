---
id: '007'
title: 'Bench verification on the stand: D/T/TURN/RT motion accuracy and G spot-check'
status: done
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

- [x] Robot vs. relay identified via `mbdeploy list`'s ROLE column before
      any flash; firmware deployed to the confirmed robot device.
- [x] DEV serial-silence watchdog widened for the session and fed/restored
      correctly; `DEV STOP` (and watchdog restore) executed in a
      `finally`-equivalent for the entire exercise — motors never left
      running.
- [ ] **FAILS.** `D 200 200 1000`: NO reverse encoder motion measured
      after `EVT done`; peak commanded/measured wheel speed within the
      existing ratio-governor/PID tolerance of the commanded 200 mm/s (no
      292-vs-200-style overshoot). Measured 11-21mm of post-done reverse
      motion across 3 runs (two independent measurement paths) — see
      `bench-verification-log.md` §1 for the root-caused mechanism.
- [ ] **FAILS.** `T 200 200 1000`: NO reverse encoder motion measured
      after `EVT done`. Measured 19-23mm — see `bench-verification-log.md`
      §2.
- [ ] **FAILS.** `TURN <heading>` (~90°): NO reverse encoder motion after
      `EVT done` AND heading accuracy within 086/087's existing tolerance
      bar. TURN never completes at all on this hardware/session (its
      `STOP_HEADING` never fires because `PoseEstimator`'s fused pose is
      frozen — see `bench-verification-log.md` §3; likely a pre-existing
      defect outside this sprint's own scope, but it still blocks this
      criterion).
- [~] **PARTIAL.** `RT <relAngle>`: NO reverse encoder motion after
      `EVT done` AND rotation accuracy within 086/087's existing tolerance
      bar. Completion mechanism verified correct (`reason=rot`, encoder-
      arc-based, independent of the broken fused pose); accuracy could NOT
      be reliably measured this session (the intended `pose=`-based check
      is invalid per the §3 finding); one run showed 13mm of post-done
      reverse motion. See `bench-verification-log.md` §4.
- [ ] **FAILS.** `G <x> <y> <speed>`: dispatches and settles (reaches
      target region, emits completion) — smoke-checked, not fully
      re-verified. Dispatched and ran the full exercise (1.3+ m of real
      wheel travel) but never arrived, ending via the TIME safety net —
      same root cause as TURN's finding. See `bench-verification-log.md`
      §5.
- [x] On-target Ruckig solve time characterized for at least one linear
      and one rotational solve; headroom within the control loop period
      assessed and recorded. Precise DWT-cycle-counter timing was blocked
      by pyOCD/gdb tooling friction this session (documented, not silently
      dropped); an indirect, zero-instrumentation TLM-tick-gap method
      gives a rough ~12ms one-time cost for both channels, well within the
      20ms tick budget. See `bench-verification-log.md` §7.
- [ ] **NOT VERIFIED.** At least one motion verb exercised over the radio
      relay path in addition to direct serial. The relay dongle was not
      physically connected during this bench session (this agent cannot
      plug in hardware) — see `bench-verification-log.md` §8. Needs a
      follow-up pass once the dongle is attached.
- [x] Written bench log committed to the sprint directory, distinguishing
      verified results from any stand-limited or deferred items.
      (`bench-verification-log.md`).
- [x] `uv run pytest tests/sim` green (ticket 006's consolidated suite) at
      the last firmware ticket before the bench pass, re-confirmed at
      close. 308 passed, 2 xfailed, both times (no `source/` edits made
      during this ticket).
- [~] **PARTIAL.** **[Revision 2]** `D`/`TURN`/`RT` each complete via their
      OWN stop condition (`EVT done reason=dist` / heading / rotation),
      never via the `STOP_TIME` safety net — verified from captured
      telemetry, not assumed. D: PASS (`reason=dist`). RT: PASS
      (`reason=rot`). TURN: N/A — it never completes at all, an even more
      severe form of the exact stall-short symptom Decision 10 exists to
      close (SPRINT-BLOCKING on its own terms, though root-caused to a
      different mechanism than Decision 10 addresses — see
      `bench-verification-log.md` §3).
- [~] **PARTIAL.** **[Revision 2]** Terminal near-rest replan-chatter
      behavior is characterized (present/absent, frequency, self-
      resolving or not) and recorded in the bench log; no mitigation is
      pre-built. D/T: no chatter observed (clean single settle). TURN/RT:
      could not be characterized (TURN never reaches near-rest; RT's
      completion window was too short to isolate a distinct near-rest
      phase). See `bench-verification-log.md` §6.

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

## Completion Notes (2026-07-07, bench pass)

**Status left `in-progress`, NOT `done` — this bench pass found two
SPRINT-BLOCKING issues.** Full detail, raw traces, and root-cause analysis
in `../bench-verification-log.md`. Summary:

1. **D/T terminal reverse-motion is still present** (11-23mm, essentially
   the same magnitude as the originally-confirmed 16mm/23mm bug this
   sprint exists to fix). Root-caused to a real interaction between
   Decision 8's "seed the stop-triggered decel from the channel's own
   theoretical state" contract and the (unchanged, out-of-scope) velocity
   PID's own tracking looseness on real hardware: when the real plant
   runs measurably faster than the plan believes (confirmed via
   `DEV M n STATE`'s direct per-motor register, independent of the fused
   TLM telemetry), the real encoder crosses `STOP_DISTANCE` before the
   plan's own internal decel would start, and the freshly-armed decel
   re-solve is seeded from the plan's (lower) belief rather than the
   plant's (higher) real speed — reproducing a smaller version of the
   exact bug this sprint targeted, via a different specific mechanism.
   This is a genuine design gap, not a sim-vs-bench measurement artifact
   (confirmed via two independent measurement paths) and not something I
   patched mid-bench, per this ticket's own instruction.
2. **TURN cannot complete at all**, and **G never arrives**, because
   `PoseEstimator`'s fused pose (`pose=`) does not accumulate from real
   wheel motion on this hardware/session (confirmed: 1.3+ meters of real
   encoder travel during a `G` run, `pose=` never left `(0,0,-7)`).
   `PoseEstimator` is explicitly unchanged by this sprint
   (architecture-update.md Decision 9) — this looks like a pre-existing
   defect, surfaced here for the first time because this is the first
   sustained real bench TURN/G run against this exact firmware. RT is
   unaffected (its own completion signal is the raw encoder-arc
   differential, independent of `pose=`) and does complete correctly.
3. Everything else in this ticket's own scope was completed: robot/relay
   identification, safety wrapping, the completion-mode criterion (PASS
   for D/RT; TURN is N/A since it never completes), solve-time
   characterization (indirect method, ~12ms one-time cost, adequate
   headroom), terminal-chatter characterization (none observed where
   measurable), and the pre/post sim-gate sanity check (308 passed, 2
   xfailed, unchanged).
4. **Not completed**: the radio-relay verification — the relay dongle was
   not physically connected during this session. Needs a follow-up pass.
5. A real dispatch-reliability bug in the bench harness itself (not
   firmware) was found and fixed: `RT`/`TURN`/`G` intermittently failed
   to dispatch when sent immediately after a blocking `STREAM` reply on
   this USB-CDC link; a settle gap + switching to `send()`'s retry-
   capable dispatch fixed it. Kept in
   `tests/bench/bench_ruckig_motion_verify.py` for future runs.

**Recommendation**: do not close sprint 089 on this bench pass. The D/T
finding needs either a design revisit or an explicit stakeholder-accepted
tolerance decision. The `PoseEstimator` finding should be raised as its
own follow-on issue (not filed here — this session's scope excludes
touching `clasi/issues/`) and re-verified once fixed, since it currently
blocks TURN's and `G`'s own criteria independent of anything this sprint
changed.

---

## Team-lead closure record (2026-07-07)

This ticket is marked **done as DESCOPED**, not as passed. The two failing
acceptance criteria (D/T reverse motion; TURN/G blocked by frozen pose) are
**NOT met** and are left unchecked deliberately — do not read this ticket
as "the D/T terminal-overshoot fix is validated on hardware." It is not.

Per stakeholder decision (2026-07-07): skip the remaining bench testing
(radio-relay round-trip) and close the sprint, deferring both findings to
fresh pool issues for a future sprint:

- **Finding 1 — D/T terminal reverse persists** →
  `clasi/issues/d-t-terminal-reverse-persists-decel-reseed-from-plan-velocity.md`
  (decel re-solve seeds from the plan's believed velocity while the loose
  bench-tuned PID runs the real wheel faster; 11–21 mm on D, 19–23 mm on T).
- **Finding 2 — PoseEstimator fused pose frozen on hardware** →
  `clasi/issues/poseestimator-fused-pose-frozen-on-hardware.md`
  (pre-existing, unchanged by this sprint; blocks TURN completion and G
  arrival).
- **Relay round-trip** — not run (dongle unconnected); the transport gap is
  already tracked by `clasi/issues/relay-round-trip-bench-verification.md`.

What this sprint DID land and is validated: the vendored-Ruckig build
integration, the `Motion::JerkTrajectory` wrapper, the D/T/R/S/TURN/RT
migration onto it, the consolidated no-reverse sim proof (308 passed, 2
xfailed), the working Decision-10 divergence-replan machinery (D/RT complete
via their own stop conditions, never the STOP_TIME net), and the ~12 ms
on-target solve-time headroom characterization. The remaining hardware
terminal-accuracy gap is carried forward in the two issues above.
