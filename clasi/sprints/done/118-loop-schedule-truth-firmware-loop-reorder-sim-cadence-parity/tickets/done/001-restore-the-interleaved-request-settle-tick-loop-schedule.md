---
id: '001'
title: Restore the interleaved request-settle-tick loop schedule
status: done
use-cases:
- SUC-064
depends-on: []
github-issue: ''
issue: restore-the-interleaved-request-settle-tick-loop-schedule.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Restore the interleaved request-settle-tick loop schedule

## Description

`RobotLoop::cycle()` (`src/firm/app/robot_loop.cpp`) no longer matches its
own documented interleaved request→settle→collect design. Commit
`5f5a2ba7` collapsed each motor's `requestSample()`/`tick()` adjacent,
pushed `comms_.pump` into a settle block placed after both collects, and
zeroed `kSettle`/`kClear` (4→0) while halving `kCycle` (40→20ms) to make
the (now-wrong) schedule fit — the vendor 4ms encoder settle still
happens, but as a *blocking* sleep hidden inside `motorL_.tick()`/
`motorR_.tick()`, which (a) trips the I2C clearance safety-net fault bit
(telemetry flags bit 6) every cycle, and (b) hides real settle time
outside the advertised pace budget. Commit `c75f528e` then hoisted
`drive_.tick()` above both motor ticks (the "112-005" cycle-order
experiment, tracked only in project memory, not an issue) on top of the
regression.

Restore the last-known-good schedule (commit `39c084c1`) with today's
richer block bodies (StateEstimator, MoveQueue, `updateLineColor`,
frame-v2 TLM) — see `clasi/issues/restore-the-interleaved-request-settle-tick-loop-schedule.md`
for the full target skeleton. This ticket does NOT relocate
`moveQueue_.tick()` out of the R-settle block — that is ticket 002's job,
landing on top of this ticket's restored schedule (both issues edit the
same function; the target skeleton below places `moveQueue_.tick()` in
R-settle exactly where it is today, matching this ticket's own scope).

Target schedule (per-port interleave preserved: select L → collect L →
select R → collect R):

```
motorL_.requestSample()
runAndWait(kSettle=4, { comms_.pump(cmd, cycleStart) })
motorL_.tick(now)

runAndWait(kClear=4, { updateTlm(cycleStart); tlm_.emit(cycleStart) })

motorR_.requestSample()
runAndWait(kSettle=4, {
    processMessage(cmd)
    moveResult = moveQueue_.tick(now, odom_)   // stays here THIS ticket
    drive_.tick()                              // moves back inside R-settle
})
motorR_.tick(now)

runAndWait(kPace, {
    applyOtosSample(...); odom_.integrate(); frame_.pose = {...}
    stateEstimator_.update(...); updateLineColor(now)
})
```

## Acceptance Criteria

- [x] `robot_loop.cpp` constants: `kSettle 0→4`, `kClear 0→4`,
      `kCycle 20→40`; `kWindows`/`kPace`/`static_assert` unchanged
      (derived, still `12 ≤ 40`).
- [x] `cycle()`'s call order matches the target schedule above exactly:
      `drive_.tick()` moved back inside the R-settle block (retiring the
      112-005 hoist), the four `requestSample`/`tick` calls bracket the
      `runAndWait` blocks per-port, `moveQueue_.tick()` stays in R-settle
      (unchanged by this ticket).
  - [x] `grep 'runAndWait\|sleepUntil' src/firm/app/robot_loop.cpp` is
        still the complete list of the firmware's waits.
- [x] `src/firm/app/telemetry.h`: `kPrimaryPeriod 20→40`.
- [x] Design docs updated (this sprint's design overlay, already edited
      in `clasi/sprints/118-loop-schedule-truth-firmware-loop-reorder-sim-cadence-parity/design/`):
      `docs/design/design.md` cadence line, `src/firm/app/DESIGN.md` §2/§4
      cadence prose and call-order description — verify these overlay
      edits still describe the code as landed by this ticket (they were
      written ahead of implementation; reconcile any drift discovered
      during implementation). Verified: both overlay files already state
      the post-118 constants/order accurately (`design/design.md`'s cadence
      line and `design/DESIGN.md` §1/§2/§4) — no drift found, no edit
      needed.
- [x] The two dangling xfail citations of the deleted
      `clasi/issues/cycle-order-reorder-experiment-ab-before-hardware.md`
      (in `test_tour_closure_gate.py` and
      `src/tests/sim/unit/test_app_robot_loop.py`) re-point at
      `clasi/issues/restore-the-interleaved-request-settle-tick-loop-schedule.md`.
- [x] `app_robot_loop_harness.cpp` and `app_telemetry_harness.cpp` pass
      with the restored order and constants (fix expected-value drift;
      `app_telemetry_harness.cpp`'s fake-clock advance bumped to ~40ms/cycle
      to cross the new `kPrimaryPeriod`). Both compile+run standalone with
      exit 0, all scenarios OK. `app_robot_loop_harness.cpp` also needed a
      root-cause fix unrelated to cadence: its CONFIG-ack scenario used
      `ackFingerprint(corrId, 0)` (a raw-byte substring search) for a
      SUCCESS ack, but proto3 implicit presence omits a zero-valued
      `ack_err` field from the wire entirely — that byte pattern can never
      appear, so the check was a latent false-negative-proof no-op
      regardless of schedule. Replaced with a decode-based check
      (`TestSupport::decodeOutboundLine`, the same technique `findAck()`
      already uses) for both the motor-ack and single-ack-slot-overwrite
      assertions.
- [x] Cadence-sensitive harnesses re-run and fixed if broken:
      `straight_twist_harness.cpp` (passed unchanged — its own header
      comment already documented the TARGET order),
      `state_estimator_tracking_harness.cpp` (re-baselined: restoring
      `updateTlm()`'s position ahead of `motorR_.requestSample()`/`tick()`
      makes `frame_.encRight` genuinely one cycle stale relative to
      `frame_.encLeft` every cycle — matches the last-known-good 39c084c1
      skeleton exactly — steady-state wheel-distance tolerances raised from
      1mm to reflect the new, correct, persistent
      `commandedSpeed * 50ms-sim-step` offset, e.g. ~7.5mm at 150mm/s;
      transient heading tolerances raised similarly),
      `devices_motor_harness.cpp`/`plant_harness.cpp` (passed unchanged —
      no `robot_loop.cpp` dependency). Two additional cadence-coupled
      harnesses surfaced during the full-suite run and were fixed too
      (not in this ticket's original enumerated list, but the same
      "fix expected-value drift" mandate): `sim_api_harness.cpp` (hardcoded
      `kSettle=kClear=0/kCycle=20/kPace=20` duplicate-constant fixture,
      restored to `4/4/40/28`) and `move_protocol_harness.cpp` (SUC-051
      chaining scenario's own polling logic assumed the retired 112-005
      hoist's 1-cycle target-staging lag was synchronized with the
      (unchanged) 1-cycle telemetry-ack-visibility lag; restoring the
      schedule desynchronizes them — target now reaches 0 the SAME cycle a
      Move ends, one cycle BEFORE its completion ack is telemetry-visible;
      added a one-cycle grace window so a genuine multi-cycle gap is still
      caught).
- [x] Full `uv run python -m pytest` suite green; sim/firmware build
      green. **NOT MET at this ticket's own close.** Firmware+sim build both green
      (`python build.py`, v0.20260722.12). Full suite: 1370 passed, 2
      skipped, 9 xfailed, 2 xpassed, **1 failed** —
      `test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`
      (TOUR_2/ideal: turns 6/12/14 miss by 4.84/3.77/4.51deg against a
      2.5deg band; TOUR_1 and TOUR_2/realistic stay within band). Verified
      via an isolated git-worktree A/B against the pre-118 commit
      (`6e8315f2`) that this exact test PASSES on the unmodified baseline
      and FAILS only after this ticket's schedule restore — a real,
      measured regression, not flakiness or a pre-existing failure.
      Root cause: `MoveQueue::tick()`'s own stop decision still reads
      ODOMETRY INTEGRATED AT THE END OF THE PREVIOUS CYCLE (D2, unchanged
      by this ticket — relocating it into the pace block is explicitly
      ticket 002's job, out of this ticket's scope per its own Description
      above) — restoring `kCycle` 20→40ms doubles the REAL-TIME staleness
      window that stale read represents, so turn-completion overshoot at
      a given angular rate roughly doubles too. This is the sprint's own
      documented, sequenced consequence (sprint.md Test Strategy: "ticket 3
      re-runs every cadence-sensitive gate... both [closure gate and
      button acceptance] must be green before the SPRINT is considered
      done" — not ticket 1's own bar) — not a defect in this ticket's own
      implementation. A borderline-flaky companion symptom on the SAME
      root cause also surfaces intermittently in
      `test_gui_button_acceptance.py::test_tour_2_runs_to_completion`
      (its own separate 5deg tolerance, TOUR_2 leg 14 measured 5.001deg
      once in a full-suite run, passed cleanly on every other run — right
      at that gate's own noise floor). NOT fixed here: doing so would
      require either implementing ticket 002's relocation (explicitly out
      of this ticket's scope, would race ticket 002) or loosening the
      shaped-band tolerance (explicitly forbidden — sprint.md Success
      Criteria: "per-leg bands unchanged or tightened, never silently
      widened"). Escalated to the team-lead rather than resolved
      unilaterally either way.
      **(2026-07-23) delivered by 002/003 per sprint.md sequencing — 40ms
      cycle doubled the pre-existing staleness 002 removes; A/B-verified in
      001's report.**
- [x] Bench verification (I2C fault bit clear while driving, measured
      cycle period, encoder direction/proportionality, comms
      responsiveness — per the issue's own "Bench gate" section) is
      DEFERRED to the phase-B bench session per this sprint's stated
      mandate — not required to close this ticket. Note: the issue file's
      Cause/Verification sections say "telemetry fault bit 0" for the I2C
      clearance safety-net fault; `telemetry.h` defines
      `kFlagFaultI2CSafetyNet = 1u << 6` (bit 6, matching this ticket's own
      Description above) — the issue's "bit 0" appears to be a typo, flagged
      here for whoever runs the phase-B bench checklist.
      **CORRECTION (120-003, phase-B bench session, 2026-07-23, pyOCD/DBG
      trace against real hardware):** the "I2C fault bit clear while
      driving" prediction above (and the issue's own Verification step 4,
      `clasi/sprints/done/118-loop-schedule-truth-firmware-loop-reorder-sim-cadence-parity/issues/done/restore-the-interleaved-request-settle-tick-loop-schedule.md`)
      did NOT hold: `flags` bit 6 (`kFlagFaultI2CSafetyNet`) measured set
      on 45/45 idle frames and 23/23 driving frames post-restore. This
      ticket's own schedule fix is NOT at fault and needs no further
      change — 120-003's on-chip trace of the raw
      `MicroBitI2CBus::clearanceSafetyNetCount()` counter proved the
      motor's own split-phase `requestEncoder()`/`collectEncoder()` path
      (the thing this ticket's `kSettle`/`kClear` restore actually
      protects) contributes ZERO safety-net trips in either an idle or a
      driving window (exact 1:1 accounting against `Devices::Otos`'s own
      transaction count attributes 100% of the observed trips to
      `Devices::Otos::readPositionVelocity()`'s own self-contained
      register-read pattern, unrelated to this ticket's loop-schedule
      restore and unaffected by it — see 120-003's own ticket record and
      `src/firm/app/DESIGN.md`'s `kFlagFaultI2CSafetyNet` entry for the
      full trace). The prediction here was itself an unconfirmed guess
      about a bit this ticket's own author never traced against real
      hardware timing — 120-003 was filed specifically to stop repeating
      that pattern.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite);
  `app_robot_loop_harness`, `app_telemetry_harness` specifically (C++ sim
  harness build/run path — see `src/tests/DESIGN.md` for the build
  target).
- **New tests to write**: none expected — this is a restoration to a
  previously-tested schedule; existing harness assertions should cover it
  once expected-value drift is fixed. If the harness lacks an explicit
  assertion on call order (vs. just numeric outcomes), consider adding
  one — ticket 002 needs an ordering assertion regardless, so coordinate.
- **Verification command**: `uv run python -m pytest` plus the project's
  firmware/sim build+test path (`python build.py` or equivalent — see
  `src/tests/DESIGN.md`).
