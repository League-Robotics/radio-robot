---
id: '002'
title: Commanded-zero through Stage B + Stage B freshness gates + NezhaMotor genuine
  freshness
status: done
use-cases:
- SUC-131-002
depends-on:
- '001'
github-issue: ''
issue: A-commanded-zero-leaks-through-stage-b.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Commanded-zero through Stage B + Stage B freshness gates + NezhaMotor genuine freshness

## Description

`correctedCommand()` (Stage A) already guards `desired == 0.0f` and never
offsets it. `Drive::tick()` (`drive.cpp:299-313`) adds Stage B's PID term
UNCONDITIONALLY — with gains on, a retained integrator after a normal
WHEELS expiry keeps duty != 0 at rest, `commandedStop`/`alreadyQuiet`
require duty `== 0.0f` exactly, and `writeShapedDuty()` boosts any nonzero
sub-deadband duty up to the 3% floor. A parked robot can creep or buzz on
encoder-quantization noise — the same silent-stall class the deadband
boost was built to kill, running in reverse. Give commanded-zero the same
explicit treatment through Stage B that Stage A already has: when the
(post-floor) commanded speed for a wheel is exactly 0.0f, Stage B's
contribution must be forced to zero for that wheel that tick, and its
integrator must FREEZE (not reset — matching this file's existing
anti-windup convention), not silently wind down or up.

Separately: Stage B reads `state.wheelLeft/Right.velocity`
(`drive.cpp:299-306`) with none of Stage C's fresh/connected/frozen gates
(`drive.cpp:323-328`), and a failed encoder collect currently manufactures
`velocity = 0` — so Stage B will wind against a wheel it cannot see, and
any future gain sweep run on a glitching bus is silently contaminated.
Gate Stage B's error/integration on the SAME `freshLeft`/`freshRight`
conjunct Stage C already computes.

**Mandatory pairing — do not land the Stage B freshness gates without the
`NezhaMotor` fix below.** `NezhaMotor::tick()` stamps `lastTickUs_`
unconditionally, even when `collectEncoder()`'s collect fails — so
`EncoderReading.age`/`sampleTime()` reads FRESH on a disconnected bus,
`motor.h:103-109` promises a `lastFreshUs_` that does not exist, and Stage
C currently survives only via its separate `connected` conjunct. If Stage
B's new freshness gate is wired to the same dishonest timestamp, the new
gate inherits the lie on day one and is no better than no gate at all. Add
a genuine `lastFreshUs_` (advances only on a successful collect) and make
`sampleTime()` return it. Both changes land in this one ticket.

## Acceptance Criteria

- [x] With nonzero Stage B gains (`kp`/`ki`/`pidMax` > 0) and a pre-wound
      `pidIntegralLeft_`/`Right_`, a commanded speed of exactly 0.0f for a
      wheel produces a Stage B contribution of exactly 0.0 and a written
      duty of exactly 0.0 for that wheel, every tick, regardless of the
      integrator's retained value.
- [x] The integrator is FROZEN (unchanged), not reset to 0, while
      commanded speed is exactly zero — verified by re-commanding a
      nonzero speed afterward and confirming the pre-zero integral value
      is still present.
- [x] Stage B's error/integration for a wheel is gated on that wheel's
      `freshLeft`/`freshRight` (the same computation Stage C already
      uses) — a simulated stale/disconnected/frozen wheel does not have
      Stage B wind against it.
- [x] `Devices::NezhaMotor` has a `lastFreshUs_` member, updated only when
      the split-phase collect (`collectEncoder()`) succeeds this tick;
      `sampleTime()` returns it instead of `lastTickUs_`.
- [x] New test: a simulated failed encoder collect (`connected_` false)
      leaves `sampleTime()`'s returned value unchanged from the last
      successful collect, across one or more failing ticks.
      `velocity_`'s own diffing behavior (independently characterized as
      correct) is UNCHANGED by this fix — this ticket touches only the
      freshness timestamp, not the velocity computation.
- [x] No regression: a healthy, connected wheel with Stage B gains at
      their shipped default (0, disabling the stage) shows bit-identical
      behavior to before this ticket.
- [ ] Full sim suite stays green.

## Testing

- **Existing tests to run**: `App::Drive` Stage A/B/C harness,
  `Devices::NezhaMotor` unit tests, full `src/tests/sim` suite.
- **New tests to write**:
  - Firmware/sim test: commanded-zero-through-Stage-B guard with a
    pre-wound integrator and nonzero gains.
  - Firmware/sim test: integrator freeze (not reset) at commanded-zero,
    confirmed by resuming a nonzero command afterward.
  - Firmware/sim test: Stage B does not integrate against a stale/
    disconnected/frozen wheel.
  - Firmware test: `NezhaMotor::sampleTime()` under a simulated failed
    collect holds the last successful timestamp.
- **Verification command**: `uv run python -m pytest src/tests/sim`; the
  firmware-side harness per its own build target.

## Implementation Plan

**Approach**: In `Drive::tick()`, move the `freshLeft`/`freshRight`
computation (currently just before Stage C) earlier, ahead of Stage B, and
use it plus a `speedLeft/Right == 0.0f` check to gate each wheel's call
into `fastPid()`: return 0 directly (skip the call) when commanded is
exactly zero for that wheel; when not fresh, freeze rather than integrate
(same shape as the existing `steady` gate's freeze-not-reset behavior). In
`nezha_motor.h`/`.cpp`, add `uint64_t lastFreshUs_` alongside the existing
`lastTickUs_`; set it in `tick()`'s step 1 only inside the branch where
`collectEncoder()`'s `connected_` comes back true; change the `sampleTime()`
accessor to return it.

**Files to modify**:
- `src/firm/app/drive.cpp` — `tick()` (Stage B gating), possibly
  `fastPid()`'s signature/call site to accept a `fresh` flag.
- `src/firm/devices/nezha_motor.h` — add `lastFreshUs_` member.
- `src/firm/devices/nezha_motor.cpp` — set it only on a successful
  collect; update `sampleTime()`.
- `src/firm/devices/motor.h` — update `sampleTime()`'s doc comment to say
  the promised contract is now implemented (no longer a forward
  reference).
- Existing `App::Drive`/`NezhaMotor` test harnesses.

**Testing plan**: as listed above.

**Documentation updates**: `drive.h`'s `ControlGains` doc comment gets a
note about the commanded-zero guard and freshness gating; `motor.h`'s
`sampleTime()` doc comment updated per the Files section above.

## Completion Notes

**Implemented** (one commit, per the ticket's own mandatory pairing):

- `src/firm/app/drive.cpp` (`tick()`): moved the `freshLeft`/`freshRight`
  computation (the exact conjunct Stage C's `adaptBias()` already used:
  `connected && !wheelFrozenLeft/Right && sampleAge(...) <= kMaxSampleAge`)
  ahead of Stage B, unchanged in formula — reused, not duplicated, by
  Stage C below it. Stage B's two `fastPid()` calls are now
  `(speedLeft == 0.0f) ? 0.0f : fastPid(..., steadyLeft && freshLeft)` (and
  the mirror for right) — a commanded-zero wheel skips `fastPid()`
  entirely (integrator untouched, output forced to exactly 0.0), and a
  commanded-nonzero wheel now also requires `fresh` (folded into the same
  `steady` parameter fastPid() already gated on) before its integrator may
  accumulate, so a stale/disconnected/frozen reading freezes it exactly
  like the pre-existing `!steady` case instead of winding against
  `nezha_motor.cpp`'s manufactured zero velocity.
- `src/firm/devices/nezha_motor.h`/`.cpp`: added `uint64_t lastFreshUs_ = 0`
  alongside `lastTickUs_`; `tick()` now sets it (`if (connected_) {
  lastFreshUs_ = nowUs; }`) immediately after `collectEncoder()`'s call —
  `connected_` reflects both halves of the split-phase transaction at that
  point — leaving `lastTickUs_`, `lastPosition_`, and the velocity diff
  computation completely untouched (same lines, same order, no logic
  changed). `sampleTime()` now returns `lastFreshUs_` instead of
  `lastTickUs_`.
- `src/firm/devices/motor.h`: `sampleTime()`'s doc comment updated from a
  forward reference ("NezhaMotor's lastFreshUs_ is the reference
  implementation" — a field that did not yet exist) to a statement that
  the contract is now genuinely implemented.
- `src/firm/app/drive.h`: `ControlGains`' doc comment, the Stage B
  paragraph in the file-header algorithm description, and `fastPid()`'s
  own private-method doc comment all updated to describe the
  commanded-zero skip and the freshness-folded-into-`steady` gate.
- Tests: `src/tests/sim/unit/devices_motor_harness.cpp` — one new
  scenario, `scenarioSampleTimeHoldsLastSuccessfulCollectAcrossFailedCollects`,
  scripting a NAK'd `collectEncoder()` read (`connected_` false) across
  TWO consecutive ticks and confirming `sampleTime()` holds the prior
  successful collect's timestamp both times, then resumes advancing once
  a collect succeeds again; the pre-existing
  `scenarioSampleTimeReflectsMostRecentTick`'s doc comment was updated
  (assertions unchanged) to note it now covers the healthy-path case only.
  `src/tests/sim/unit/app_drive_harness.cpp` — two new scenarios:
  `scenarioCommandedZeroForcesStageBToZeroAndFreezesIntegrator` (winds the
  integrator to a known value via 10 ticks of persistent error, exactly
  reproducing `scenarioFastPidSteadyGateAndAntiWindup`'s own arithmetic
  so the integrator predictably pins at `pidMax`; then commands zero for
  20 ticks asserting exactly-0.0 pid/duty every tick; then re-commands
  the original nonzero speed/error for one tick and asserts the result is
  at least the pre-zero pid value — which only holds if the integrator
  was frozen, not reset: a reset integrator would produce a far smaller
  resumed value, so this is a genuine regression guard, verified by
  reasoning through `fastPid()`'s own clamp/anti-windup arithmetic by
  hand) and
  `scenarioStaleDisconnectedOrFrozenWheelFreezesStageBIntegrator` (three
  independent sub-scenarios — disconnected, stale sample age, and
  `Health::wheelFrozenLeft` — each proving the p-term-only value stays
  IDENTICAL across 20 ticks of persistent error, i.e. no integral growth).

**Verified**:

- Compiled and ran both harnesses directly (not just through pytest) to
  read the per-scenario PASS/FAIL output: `devices_motor_harness` (13
  scenarios, all pass, including both `sampleTime()` scenarios back to
  back) and `app_drive_harness` (16 scenarios, all pass, including both
  new 131-002 scenarios and all of 001's own preserved scenarios).
- `uv run python -m pytest src/tests/sim/unit/test_app_drive.py
  src/tests/sim/unit/test_devices_motor.py -v` — 2 passed.
- `uv run python -m pytest src/tests/sim/unit/test_app_odometry.py` — 1
  passed (extra confidence pass, not required by the ticket — App::Drive
  and NezhaMotor are the two components this ticket touches; Odometry is
  a nearby, dependency-adjacent module).
- No-regression criterion: verified by re-running EVERY pre-existing
  `App::Drive` scenario unmodified (`scenarioBiasConvergesUnder...`,
  `scenarioBiasClampHolds...`, `scenarioBumplessTransfer...`,
  `scenarioFastPidSteadyGateAndAntiWindup`, `scenarioWheelsAndMove...`,
  and all four of 001's own scenarios) — all still pass with no changes
  to their own assertions, confirming the relocated `freshLeft`/`freshRight`
  computation and the new commanded-zero/`fresh` gate are no-ops for every
  scenario that was already setting `connected = true` with a fresh
  `sampleTime` (which every existing nonzero-command scenario already
  did, since Stage C's `adaptBias()` needed it too).
- Confirmed by `grep` that `Devices::Motor::sampleTime()`'s only other
  overrides in `src/firm` are `MotorArmor` (`motor_armor.h:88`, a pure
  passthrough to the wrapped motor — unaffected) and `BoardMotor`
  (`board_motor.h:45`, a different concrete leaf, out of this sprint's
  scope per the team-lead's own "leave alone" instruction — untouched).
  `App::RobotLoop::publishWheels()` (`robot_loop.cpp:384`) reads
  `motor.sampleTime()` generically through the `Devices::Motor` interface
  and needed no change.

**Not verified / explicitly deferred**:

- **Full `src/tests/sim` suite**: per the team-lead's own explicit
  instruction for this ticket ("Do not run the full suite yourself... I
  will run the full suite at the ticket boundary"), the full suite was
  NOT run by this ticket. The "Full sim suite stays green" acceptance
  criterion above is left UNCHECKED for that reason — it is not something
  this ticket confirmed directly, only inferred from the targeted runs
  above plus the no-regression check against every affected existing
  scenario. The baseline to beat, per the team-lead, is 001's own
  measured **460 passed, 1 xfailed, 2 xpassed, 0 failed** (the 1
  xfailed/2 xpassed are the pre-existing, unrelated
  `B-app-robot-loop-harness-never-compiled.md` rows, not touched here).
- **Hardware/bench acceptance**: not attempted. `tovez` has been wedged
  since 2026-08-01 per the team-lead's own instruction and
  `.claude/rules/hardware-bench-testing.md`; sim-tier acceptance is this
  sprint's declared, approved position. No bench result is claimed or
  implied anywhere above.
- A full ARM cross-build (`just build`) was not separately invoked — the
  host-build sim-tier compiles (both harnesses above) build the exact
  changed files (`drive.cpp`, `nezha_motor.cpp`/`.h`, `motor.h`) against
  the same `src/firm/devices`/`src/firm/app` headers the ARM build
  compiles, and this ticket's diff has no `#ifdef HOST_BUILD`/vendor-SDK
  touch (matching 001's own reasoning for the same judgment call).

**Note for ticket 003** (the speed floor, same file, `drive.cpp`): the
new commanded-zero check in Stage B (`speedLeft == 0.0f`) reads
`speedLeft`/`speedRight` — the ALREADY-floored commanded speed
(`applySpeedFloor(state.wheelLeft.cmdVelocity)`, computed earlier in
`tick()`), not the raw `state.wheelLeft.cmdVelocity`. This means Stage
B's commanded-zero guard is keyed on "the floored speed is exactly
zero," which today is identical to "the raw command is exactly zero"
(since `applySpeedFloor()` never floors an exact 0.0f — it has its own
`if (commanded == 0.0f) return 0.0f;` guard, matching
`correctedCommand()`'s own guard). If ticket 003's common-mode-only
speed-floor change alters what `speedLeft`/`Right` can be at a nominally
commanded-zero wheel (e.g. a floor applied to a component the current
`applySpeedFloor()` doesn't touch), re-verify that this ticket's
`speedLeft == 0.0f` / `speedRight == 0.0f` check in `tick()` (Stage B
gating, right above the `fastPid()` calls) still means exactly "this
wheel was commanded to stop," not "this wheel's post-floor magnitude
happens to be zero for some other reason." The two guards
(`correctedCommand()`'s Stage A guard and this ticket's Stage B guard)
must keep agreeing on what "commanded zero" means, or a future speed-
floor change could silently reintroduce the exact defect this ticket
fixes on one stage while leaving the other stage's notion of "zero"
out of sync.
