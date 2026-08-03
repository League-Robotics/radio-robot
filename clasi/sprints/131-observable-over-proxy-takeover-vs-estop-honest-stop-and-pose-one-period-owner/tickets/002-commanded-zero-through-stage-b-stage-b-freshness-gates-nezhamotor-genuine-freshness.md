---
id: '002'
title: Commanded-zero through Stage B + Stage B freshness gates + NezhaMotor genuine
  freshness
status: open
use-cases: [SUC-131-002]
depends-on: ['001']
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

- [ ] With nonzero Stage B gains (`kp`/`ki`/`pidMax` > 0) and a pre-wound
      `pidIntegralLeft_`/`Right_`, a commanded speed of exactly 0.0f for a
      wheel produces a Stage B contribution of exactly 0.0 and a written
      duty of exactly 0.0 for that wheel, every tick, regardless of the
      integrator's retained value.
- [ ] The integrator is FROZEN (unchanged), not reset to 0, while
      commanded speed is exactly zero — verified by re-commanding a
      nonzero speed afterward and confirming the pre-zero integral value
      is still present.
- [ ] Stage B's error/integration for a wheel is gated on that wheel's
      `freshLeft`/`freshRight` (the same computation Stage C already
      uses) — a simulated stale/disconnected/frozen wheel does not have
      Stage B wind against it.
- [ ] `Devices::NezhaMotor` has a `lastFreshUs_` member, updated only when
      the split-phase collect (`collectEncoder()`) succeeds this tick;
      `sampleTime()` returns it instead of `lastTickUs_`.
- [ ] New test: a simulated failed encoder collect (`connected_` false)
      leaves `sampleTime()`'s returned value unchanged from the last
      successful collect, across one or more failing ticks.
      `velocity_`'s own diffing behavior (independently characterized as
      correct) is UNCHANGED by this fix — this ticket touches only the
      freshness timestamp, not the velocity computation.
- [ ] No regression: a healthy, connected wheel with Stage B gains at
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
