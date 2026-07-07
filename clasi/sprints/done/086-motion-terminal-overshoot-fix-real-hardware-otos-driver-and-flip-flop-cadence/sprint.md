---
id: 086
title: Motion terminal-overshoot fix, real-hardware OTOS driver, and flip-flop cadence
status: done
branch: sprint/086-motion-terminal-overshoot-fix-real-hardware-otos-driver-and-flip-flop-cadence
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
- SUC-008
- SUC-009
issues:
- motion-turn-drive-terminal-overshoot.md
- nezha-hardware-otos-driver-for-new-source-tree.md
- flip-flop-cadence-below-design-target.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 086: Motion terminal-overshoot fix, real-hardware OTOS driver, and flip-flop cadence

## Goals

Resolve all three currently-open pool issues in one sprint, grouped by issue
and sequenced by priority/risk:

1. **Motion terminal-overshoot fix** (the priority) -- turns and drives
   overshoot through zero into a sustained reverse spin at completion,
   wrecking multi-leg tours. Root-caused already (see the issue); fix order
   is stakeholder-mandated: motor velocity loop first, decel anticipation
   second.
2. **Real-hardware OTOS driver** -- `NezhaHardware` has no `Hal::Odometer`
   leaf; real hardware has been sim-only for fused pose since sprint 082's
   approved deferral.
3. **Flip-flop cadence** -- measured ~44-52 Hz vs. a ~80-90 Hz design
   estimate; not urgent, not a correctness bug; measure before deciding
   whether/how to act.

## Problem

See each linked issue for full detail:

- `clasi/issues/motion-turn-drive-terminal-overshoot.md`
- `clasi/issues/nezha-hardware-otos-driver-for-new-source-tree.md`
- `clasi/issues/flip-flop-cadence-below-design-target.md`

## Solution

See `architecture-update.md` for the full seven-step design. Summary per
issue:

1. **Motion overshoot**: (A1) fix `Hal::MotorVelocityPid`'s zero-crossing
   deadband/`Hal::Motor::armoredWrite()`'s reversal-dwell interaction so a
   legitimate braking correction near a commanded-zero target is neither
   held off long enough to worsen overshoot nor allowed to defeat the
   dwell's protection against a genuine unrequested reversal (two named
   invariants, Design Rationale 1) -- with a mandatory armor-regression bar
   and a stand HITL pass; then (A2) extend the `pursueSteer()`-style
   terminal decel-cap pattern (already used by GOTO) to `DISTANCE`/`TURN`/
   `ROTATION` via a new shared "remaining-to-stop" query in `Motion::
   stop_condition`, so the wheel is already near zero when the stop fires.
   Verification bar: per-leg geometry vs. sim ground truth (not
   endpoint-only), a rendered Tour 1/2 trace, and a stand HITL pass.
2. **OTOS driver**: a new `Hal::Odometer` leaf (working name
   `Hal::OtosOdometer`, `source/hal/otos/`) drives the SparkFun OTOS I2C
   register protocol, applying the host-side lever-arm mounting-offset
   compensation (ported verbatim from `source_old/hal/capability/
   OtosLeverArm.h` -- the OTOS `REG_OFFSET` register is unwritable on this
   hardware, confirmed, do not re-derive). The offset + linear/angular
   scalar are boot-config-baked from `data/robots/*.json` (already present
   in the schema and in `tovez.json`, just not yet consumed by
   `scripts/gen_boot_config.py`), not a new live `SET`/wire surface
   (Design Rationale 4). `NezhaHardware` gains one new member and a
   one-line `odometer()` override; `dev_loop.cpp`/`otos_commands.cpp`
   need zero changes (the seam is already fully generic).
3. **Flip-flop cadence**: measure per-phase timing on the stand via
   `pyOCD`/`gdb` against `I2CBus`'s already-compiled-in per-device
   counters/transaction log (zero firmware change), test the
   double-counted-clearance hypothesis, then either apply a narrow,
   re-measured, non-regressed optimization or correct the design doc's
   cadence estimate -- either outcome is an acceptable close.

## Success Criteria

- Motion: per-leg tour geometry and rendered traces confirm Tour 1/Tour 2
  drive the intended figure (not a tangle); no post-stop reverse-spin
  residual beyond a tight tolerance; every pre-existing motor-armor test
  still passes plus new zero-crossing regression tests; stand HITL pass
  confirms turns/drives/STOP/armor on real hardware.
- OTOS: `NezhaHardware::odometer()` is live on real hardware; all seven
  OTOS wire verbs ack `OK` (no more `ERR nodev`); OTOS position/velocity
  reads are plausible and changing on the stand; hardware-bench gate's
  OTOS-alive check passes.
- Flip-flop: a documented, data-backed per-phase timing breakdown exists;
  the double-counting hypothesis is confirmed or refuted; either a measured
  cadence improvement with proven non-regression of the 079-006 TWIM-stall
  fix and the reversal-latch armor, or an honestly corrected design
  estimate.

## Scope

### In Scope

- `source/subsystems/planner.{h,cpp}`, `source/motion/stop_condition.{h,cpp}`,
  `source/hal/velocity_pid.{h,cpp}`, `source/hal/capability/motor.h` (motion
  overshoot fix).
- A new `Hal::Odometer` leaf for the real OTOS sensor, its lever-arm math,
  boot-config plumbing (`scripts/gen_boot_config.py`, `source/config/
  boot_config.{h,cpp}`), and `Subsystems::NezhaHardware` wiring (OTOS
  driver).
- Read-only (and possibly narrowly-extended) use of `source/com/i2c_bus.*`
  for flip-flop cadence measurement, plus a design-doc correction if that is
  the chosen outcome.
- New/extended tests across `tests/sim/unit/`, `tests/sim/system/` (new),
  `tests/testgui/`.
- Stand HITL verification for both the motion fix and the OTOS driver
  (`.claude/rules/hardware-bench-testing.md`).

### Out of Scope

- The wire/message-schema plane for motion (`msg::PlannerCommand`/
  `StopCondition`) and for OTOS (`msg::OdometerCommand`/`OdometerConfig`) --
  both are reused as-is.
- `dev_loop.cpp`, `source/commands/otos_commands.{h,cpp}` -- confirmed
  already fully generic/live-resolving; no change needed (Grounding fact 4).
- Reviving the stale pre-rebuild `DBG OTOS`/`DBG OTOS BENCH` wire family
  (Grounding fact 7) -- not resurrected.
- A live/wire-settable OTOS mounting-offset surface -- boot-config-baked
  only this sprint (Design Rationale 4); revisit only as an explicit future
  scope decision.
- Unifying `pursueSteer()`'s `STOP_POSITION` geometry with the new shared
  "remaining-to-stop" query (Open Question 2) -- left as two well-understood
  paths, not forced into one this sprint.
- Overlapping other-device (line/color) traffic into flip-flop settle
  windows (design Case 5) -- out of scope per the issue itself.

## Test Strategy

- **Motion overshoot**: sim-level regression harness first (proves the bug
  quantitatively before any fix lands), then unit-level armor
  re-verification (`tests/sim/unit/test_motor_policy.py`, extended) for
  every fix, then a new system-level per-leg geometry + rendered-trace test
  (`tests/sim/system/`, new) replacing the endpoint-only tour assertion,
  then a stand HITL pass. Endpoint-distance-only tour tests are banned per
  the issue's own mandate.
- **OTOS driver**: host-testable unit tests for the lever-arm math and the
  leaf's register sequencing against a scripted `I2CBus` fake (mirroring
  `NezhaMotor`'s own test precedent), then a stand HITL pass for the parts
  that cannot be sim-verified (a real chip, real mounting).
- **Flip-flop cadence**: a measurement session (not a pytest suite) via
  `pyOCD`/`gdb`, per `.claude/rules/debugging.md`; if an optimization
  ticket lands, its own regression bar is the full 079-006 TWIM-stall
  test/soak suite plus the motor-armor tests, re-run unmodified.

## Architecture Notes

See `architecture-update.md` for the full grounding, responsibilities,
module table, diagrams, design rationale (5 decisions), and open questions
(5). Highlights:

- The two-phase motion fix order (motor loop, then Planner anticipation) is
  stakeholder-mandated AND independently justified: the motor loop is the
  actual root cause; Planner anticipation only reduces how hard a problem
  the motor loop has to solve.
- The `Hal::Motor::armoredWrite()` zero-crossing fix must preserve two named
  invariants (Design Rationale 1) regardless of implementation mechanism --
  the implementing ticket has real design freedom within that bound.
- The OTOS driver's entire scope is a new leaf + boot-config plumbing;
  `dev_loop.cpp`/`otos_commands.cpp` are confirmed unaffected.
- Flip-flop measurement defaults to a zero-firmware-change `pyOCD`/`gdb`
  read of existing instrumentation; a new wire verb is a fallback, not the
  starting plan.

## GitHub Issues

None linked yet -- all three source items are `clasi/issues/*.md` pool
issues, not GitHub issues.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan (pre-authorized: "run to completion, I'll test on master")

## Tickets

Stakeholder approval recorded; all 9 ticket files are created under
`tickets/`.

| # | Title | Depends On | Issue |
|---|-------|------------|-------|
| [001](tickets/001-regression-harness-reproduce-turn-drive-terminal-reverse-spin-overshoot.md) | Regression harness: reproduce turn/drive terminal reverse-spin overshoot | -- | motion-turn-drive-terminal-overshoot.md |
| [002](tickets/002-motor-velocity-loop-armor-fix-root-fix-phase-1-armor-regression-re-verification.md) | Motor velocity-loop / armor fix (root fix, phase 1) + armor regression re-verification | 001 | motion-turn-drive-terminal-overshoot.md |
| [003](tickets/003-planner-terminal-decel-coast-anticipation-phase-2.md) | Planner terminal decel/coast anticipation (phase 2) | 002 | motion-turn-drive-terminal-overshoot.md |
| [004](tickets/004-per-leg-geometry-rendered-trace-verification-and-stand-hitl-pass.md) | Per-leg geometry + rendered-trace verification and stand HITL pass | 002, 003 | motion-turn-drive-terminal-overshoot.md |
| [005](tickets/005-otos-lever-arm-math-port-boot-config-surface.md) | OTOS lever-arm math port + boot-config surface | -- | nezha-hardware-otos-driver-for-new-source-tree.md |
| [006](tickets/006-real-hal-odometer-otos-leaf-nezhahardware-wiring.md) | Real `Hal::Odometer` (OTOS) leaf + `NezhaHardware` wiring | 005 | nezha-hardware-otos-driver-for-new-source-tree.md |
| [007](tickets/007-stand-hitl-verification-of-the-otos-driver.md) | Stand HITL verification of the OTOS driver | 006 | nezha-hardware-otos-driver-for-new-source-tree.md |
| [008](tickets/008-measure-per-phase-flip-flop-bus-timing-on-the-stand.md) | Measure per-phase flip-flop bus timing on the stand | -- | flip-flop-cadence-below-design-target.md |
| [009](tickets/009-act-on-the-measurement-targeted-optimization-or-design-doc-correction.md) | Act on the measurement: targeted optimization or design-doc correction | 008 | flip-flop-cadence-below-design-target.md |

Tickets execute serially in the order listed. Issue-groups are mutually
independent (no cross-issue dependency); ordered by stakeholder-set
priority (motion first) and urgency (flip-flop last, "not urgent" per its
own issue). Each issue-group's last ticket (004, 007, 009 respectively)
retires its issue on completion (`completes_issue: true` on every ticket
in each group; the issue auto-archives once every referencing ticket is
marked done).
