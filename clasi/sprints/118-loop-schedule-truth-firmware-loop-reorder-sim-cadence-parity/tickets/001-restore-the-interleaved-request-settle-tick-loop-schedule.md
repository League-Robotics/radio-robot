---
id: '001'
title: Restore the interleaved request-settle-tick loop schedule
status: open
use-cases: [SUC-064]
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

- [ ] `robot_loop.cpp` constants: `kSettle 0→4`, `kClear 0→4`,
      `kCycle 20→40`; `kWindows`/`kPace`/`static_assert` unchanged
      (derived, still `12 ≤ 40`).
- [ ] `cycle()`'s call order matches the target schedule above exactly:
      `drive_.tick()` moved back inside the R-settle block (retiring the
      112-005 hoist), the four `requestSample`/`tick` calls bracket the
      `runAndWait` blocks per-port, `moveQueue_.tick()` stays in R-settle
      (unchanged by this ticket).
  - [ ] `grep 'runAndWait\|sleepUntil' src/firm/app/robot_loop.cpp` is
        still the complete list of the firmware's waits.
- [ ] `src/firm/app/telemetry.h`: `kPrimaryPeriod 20→40`.
- [ ] Design docs updated (this sprint's design overlay, already edited
      in `clasi/sprints/118-loop-schedule-truth-firmware-loop-reorder-sim-cadence-parity/design/`):
      `docs/design/design.md` cadence line, `src/firm/app/DESIGN.md` §2/§4
      cadence prose and call-order description — verify these overlay
      edits still describe the code as landed by this ticket (they were
      written ahead of implementation; reconcile any drift discovered
      during implementation).
- [ ] The two dangling xfail citations of the deleted
      `clasi/issues/cycle-order-reorder-experiment-ab-before-hardware.md`
      (in `test_tour_closure_gate.py` and
      `src/tests/sim/unit/test_app_robot_loop.py`) re-point at
      `clasi/issues/restore-the-interleaved-request-settle-tick-loop-schedule.md`.
- [ ] `app_robot_loop_harness.cpp` and `app_telemetry_harness.cpp` pass
      with the restored order and constants (fix expected-value drift;
      `app_telemetry_harness.cpp`'s fake-clock advance bumped to ~40ms/cycle
      to cross the new `kPrimaryPeriod`).
- [ ] Cadence-sensitive harnesses re-run and fixed if broken:
      `straight_twist_harness.cpp`, `state_estimator_tracking_harness.cpp`,
      `devices_motor_harness.cpp`, `plant_harness.cpp`.
- [ ] Full `uv run python -m pytest` suite green; sim/firmware build green.
- [ ] Bench verification (I2C fault bit clear while driving, measured
      cycle period, encoder direction/proportionality, comms
      responsiveness — per the issue's own "Bench gate" section) is
      DEFERRED to the phase-B bench session per this sprint's stated
      mandate — not required to close this ticket.

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
