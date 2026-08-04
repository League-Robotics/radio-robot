---
id: '001'
title: 'Stop-path safety: derived idle arbitration and commanded-stop re-assertion'
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: A-stop-path-runaway-single-stop-does-not-land.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Stop-path safety: derived idle arbitration and commanded-stop re-assertion

## Description

**This is a safety defect and it is first for that reason.** It is also a
*measurement* defect: until it is fixed, every "overshoot" number ticket 002
produces contains a tail that is a runaway rather than a control error. Do not
start 002 before this lands.

Measured on `vevov`, 16/16 reproductions: a host that issues a stop **once** and
then goes quiet gets **936 mm of continued travel with no decay**, still going
when the capture ended. `estop()` failed **5 of 6** attempts. Only repetition
stops the wheels.

The Nezha brick physically latches its last commanded speed and does not reset
on an nRF52 reset — only on power loss. So a lost zero write is not a transient
glitch, it is permanent. Both existing defences gate on the **encoder**
(`NezhaMotor::writeRawDuty()`'s re-issue, and `Drive::tick()`'s stop
re-assertion window whose `wheelsMoving` half is `|velocity()| > kRestVelocity`),
so both are disarmed by the same condition: a wheel that reports at rest.

Above them, the ownership handoff leaves a gap nothing covers:

```cpp
// App::Drive::update()
const bool owned = commandActive_;        // sampled BEFORE the expiry test
if (commandActive_ && expired) {
  commandActive_ = false;
  targetLeft_ = targetRight_ = 0.0f;      // publishes ONE zero pair
  ...
}
if (!owned) return;                       // ...and never publishes again
```

`Motion::Planner::update()` runs unconditionally but republishes only *its own*
idea of the command. **Nothing states "no one is driving, so the speed is
zero"** on the cycles in between. Every individual link was defensible in
isolation; the defect lived in what none of them was responsible for.

### The architectural resolution — read this before writing the guard

The issue was filed rather than merged because the guard "writes `cmdVelocity`
from the loop, violating the one-writer rule," and it named "an explicit idle
owner" as the proper home. **Do not build an idle owner.** Sprint architecture
Decision 1 resolves this differently, and the reasoning is load-bearing:

- The literal one-writer rule was **already false on master**.
  `RobotLoop::handleEstop()` writes `state_.wheelLeft/Right.cmdVelocity = 0.0f`
  from the loop today (`src/firm/app/robot_loop.cpp`, ~line 377) with a comment
  explaining why that is correct. The honest invariant was never "one writer" —
  it was "one decider plus a safety override," merely undocumented.
- An idle owner must be **acquired and released**, which adds a third handoff
  edge — and a handoff gap is precisely the defect being fixed. It also opens a
  new unowned window between Drive releasing and Idle acquiring.
- Decisively: an owner that must be *told* to take over cannot cover the failure
  mode that matters, a decider that has silently **stopped publishing**.
  Idleness must be **derived**, never announced.

So implement the guard as a **named `RobotLoop` safety-arbitration step** with a
stated monotone contract — it may write **only** `0.0f`, never a nonzero. That
contract is what makes a loop-level write legitimate: it cannot originate
motion, cannot fight a decider for control, and cannot produce a value no
decider asked for.

The invariant to document at `RobotLoop::publishWheels()` (which currently
carries the now-stale "exactly one writer per cycle" comment) and in
`src/firm/app/drive.h`:

> `cmdVelocity` has exactly one **decider** per cycle — `Motion::Planner::
> update()` or `App::Drive::update()` — and exactly one **safety arbiter**,
> `App::RobotLoop`, whose writes are restricted to zero, which runs after every
> decider and before actuation, and which supersedes all deciders. No other
> writer exists.

Bring `handleEstop()`'s existing write explicitly under that rule rather than
leaving it an unexplained special case.

### The two code changes

Both come from `clasi/issues/attachments/wheel-controller-2026-08-03/firmware-changes.patch`
(hunks A and B — that file also carries ticket 002's unrelated controller work;
take only A and B here). Both can only ever *remove* motion. Neither touches a
nonzero command.

**(a) Unowned-motion guard** — `App::RobotLoop::cycle()`, immediately ahead of
`drive_.tick(state_)`, i.e. the last thing before actuation:

```cpp
if (!planner_.active() && !drive_.owns()) {
  state_.wheelLeft.cmdVelocity = 0.0f;
  state_.wheelRight.cmdVelocity = 0.0f;
}
```

Lift this into a named private method with a doc comment stating the monotone
contract. `drive_.owns()` (`drive.h`, `bool owns() const { return
commandActive_; }`) and `planner_.active()` (`planner.h`) are both existing
public ownership queries — no new interface, and no new edge between the two
deciders.

**(b) Arm the stop re-assertion on every stop, not just `estop()`** —
`App::Drive::tick()`, after `alreadyQuiet` is computed, before the `return`:

```cpp
if (commandedStop && !alreadyQuiet) stopEnforceCountdown_ = kStopEnforceTicks;
```

The window already exists (`stopEnforceCountdown_`, `kStopEnforceTicks = 30`,
~1.5 s) but is armed only by `estop()`, and its other half trusts the encoder.
Arming it on the nonzero→zero transition of the **commanded duty pair** makes
re-assertion depend on what was commanded rather than on what the encoder claims
happened.

(a) addresses the silent/expired case; (b) addresses the lost single write.
**Both are needed** — (a) alone would still lose a stop whose one write was
dropped, and (b) alone would still let an expired command run on.

### Documentation that is now wrong

- `NezhaProtocol.wheels()` (`src/host/robot_radio/robot/protocol.py`) claims
  "a wheel command is always time-bounded, so a dead host can never mean a
  runaway." Measured, as shipped, a dead host means exactly a runaway. The
  duration expires correctly in `Drive`; the expiry does not reach the motor.
- `.claude/rules/playfield-testing.md` records `estop()` at 2.9 cm / 0.10 s and
  instructs every geofence, Ctrl-C handler and halt path to call it. On `vevov`,
  as shipped, a single `estop()` stopped nothing in 5 of 6 attempts. Note that
  a halt path calling it **once** was unverified before this fix.

Correct both. Do not overclaim: after this ticket the fix is verified in sim and
by construction, and `tovez` hardware verification is ticket 004's job.

### Build traps — these have already cost real time

- **A plain `just build` compiles the DBG channel OUT.** Use `--robot-debug`
  (`uv run python build.py --clean --robot-debug`) for anything that will be
  flashed and driven from the gate.
- **Adding a member to `Drive` in `drive.h` needs `just build-clean`.** An
  incremental build links stale objects against the old class layout, and the
  encoders then read a manufactured zero that looks **exactly like a dead bus**.
  This ticket does not add a `Drive` member, but 002 does — if you touch
  `drive.h` at all here, build clean.

## Acceptance Criteria

- [x] `RobotLoop` has a **named** safety-arbitration step, called immediately
      before `drive_.tick(state_)`, whose doc comment states the monotone
      contract: it may write only `0.0f` to `cmdVelocity`, never a nonzero.
- [x] The step derives idleness from `!planner_.active() && !drive_.owns()`.
      No idle-owner subsystem is introduced, and no ownership is acquired or
      released to make it work.
- [x] `Drive::tick()` arms `stopEnforceCountdown_` on the commanded nonzero→zero
      transition of the duty pair, not only from `estop()`, and the arming
      condition reads **no** measured velocity.
- [x] The revised ownership invariant is documented at
      `RobotLoop::publishWheels()` (replacing the stale "exactly one writer"
      comment) and in `drive.h`, and `handleEstop()`'s existing loop-level zero
      write is explicitly identified as an instance of the same rule.
- [x] `NezhaProtocol.wheels()`'s "a dead host can never mean a runaway"
      docstring claim is corrected.
- [x] `.claude/rules/playfield-testing.md`'s single-`estop()` claim is corrected,
      noting that a halt path calling it once was unverified before this fix.
- [x] No nonzero `cmdVelocity` value anywhere in the change originates from
      `RobotLoop`.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim
  src/tests/unit` — the firmware loop and Drive coverage. A full-collection run
  is not required for this ticket (see the sprint's Test Strategy: mid-sprint
  green is not required, but the **end** of the sprint must match the baseline
  set by identity, not by count).
- **New tests to write**:
  - A sim test that drives a `wheels()` command to expiry with a silent host and
    asserts `cmdVelocity` is zero on **every** cycle after expiry, not just the
    expiry cycle. This is the regression that would have caught the defect.
  - A sim test that neither decider owning motion at boot (planner inactive,
    Drive never armed) yields zero `cmdVelocity` before the first actuation.
  - A test asserting the stop re-assertion arms on the commanded transition
    while the encoder reports at rest — the exact condition that disarmed both
    prior defences.
  - A test that the arbitration step never writes a nonzero value.
- **Verification command**: `uv run python -m pytest src/tests/sim src/tests/unit`
- **Not in this ticket**: hardware verification on `tovez`. That is ticket 004,
  which measures this fix and 002's together on the stand.
