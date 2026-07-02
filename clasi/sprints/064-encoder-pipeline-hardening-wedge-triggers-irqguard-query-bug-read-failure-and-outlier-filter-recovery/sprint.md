---
id: '064'
title: 'Encoder pipeline hardening: wedge triggers, IRQGUARD query bug, read-failure
  and outlier-filter recovery'
status: ticketed
branch: sprint/064-encoder-pipeline-hardening-wedge-triggers-irqguard-query-bug-read-failure-and-outlier-filter-recovery
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
issues:
- dbg-irqguard-query-disables-guard.md
- encoder-reset-while-moving-latches-readback.md
- encoder-integrity-i2c-failures-and-outlier-filter-recovery.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 064: Encoder pipeline hardening: wedge triggers, IRQGUARD query bug, read-failure and outlier-filter recovery

## Goals

Eliminate the two isolated encoder-wedge triggers, fix the guard-disabling
diagnostic regression, make the detector actually see wedge episodes, and
harden the encoder read pipeline against I2C failures and permanent
outlier-filter freezes.

## Problem

The 2026-07-02 stand session (docs/knowledge/2026-07-01-encoder-wedge-
boundary-latch-flavor.md) isolated two independent, amplitude-dependent
triggers that latch the Nezha encoder readback: (1) full-speed reversal
PWM transients, and (2) atomic encoder resets fired while wheels rotate.
The IRQ guard does not protect against either — and a 051-008 ArgSchema
regression makes a bare `DBG IRQGUARD` query silently DISABLE that guard.
The enc_wedged detector missed all ~18 observed episodes (target==0 reset +
arming grace), so the EKF wedge gating never engages, and `wheel_wedged` is
absent from TLM. Separately (CR-02/CR-03), encoder I2C reads ignore failure
(fabricating position jumps) and the outlier filter can freeze permanently
because its streak-based rebaseline was lost in the sprint-060 cutover.

## Solution

- Slew-limit |ΔPWM| per write in `Motor::setSpeed` (stop exempt) — kills the
  reversal-transient trigger at the source.
- Defer/soften `resetEncoders()` when the drivetrain is not at rest
  (software-only rebaseline while moving; hardware re-prime only at rest).
- Fix `DBG IRQGUARD` so a bare query reports without mutating state; audit
  sibling ArgSchema handlers with defaulted optional args.
- Detector: count identical raw reads regardless of target/arming grace;
  emit `wheel_wedged` in TLM; auto re-prime at idle on detection.
- Check I2C return codes in all encoder read paths (hold-last-value on
  failure, like `readSpeedRaw`); restore the reject-streak rebaseline in
  `Drive::_runOutlierFilter`.

## Success Criteria

- Simulation suite green (2 known-baseline failures allowed, no new ones).
- New unit/sim tests cover: query-does-not-mutate IRQGUARD; ΔPWM slew cap;
  reset-deferred-while-moving; detector fires on frozen-from-start wheel;
  read-failure hold-last; reject-streak rebaseline recovery.
- ARM firmware builds clean (`--clean`).
- Stakeholder HITL check (deferred): the 5-arm slam matrix arms 1/2/3/5 run
  clean post-fix on the stand.

## Scope

### In Scope

Firmware: `source/hal/real/Motor.{h,cpp}`, `source/control/MotorController.*`,
`source/subsystems/drive/Drive.*`, `source/robot/Robot.cpp` (resetEncoders
path), `source/commands/DebugCommands.cpp`, `source/robot/RobotTelemetry.cpp`
(TLM field), plus matching sim-tier tests.

### Out of Scope

Stop/watchdog architecture (sprint 065); sim OTOS fidelity and host cleanups
(sprint 066); hardware validation runs (stakeholder, post-merge).

## Test Strategy

Simulation-tier pytest for all behavior changes (sim build exercises the same
control/Drive/MotorController code); pure-logic unit tests where the sim ABI
does not reach (Motor slew cap via sim motor or firmware-logic test). Full
default suite (`uv run --with pytest python -m pytest -q`) must be green
before close. HITL slam-matrix validation explicitly deferred to stakeholder.

## Architecture Notes

- The wedge detector and odometry gating (033-005d/e) remain; their blind
  spots are being removed, not redesigned.
- TLM gains a `wedge=` (or extends `enc=`) field — additive, host parsers
  tolerate unknown fields.
- ΔPWM slew cap default must not visibly change BVC-profiled motion (profiles
  never step more than the cap in normal operation); pick cap ≥ observed
  profiled per-tick deltas.

## GitHub Issues

(none)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan (auto-approve session)

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Fix ArgSchema query-mutates-state bug (DBG IRQGUARD, RF, OL, OA) | — |
| 002 | Add \|ΔPWM\| slew cap to Motor::setSpeed | — |
| 003 | Software-only encoder rebaseline when drivetrain is not at rest | — |
| 004 | Remove wedge-detector blind spots, add TLM wedge= field, auto re-prime at idle | 003 |
| 005 | Hold-last-value on I2C encoder read failure (Motor + SimMotor fault injection) | — |
| 006 | Restore outlier-filter reject-streak rebaseline and idle refresh in Drive | — |

Tickets execute serially in the order listed.
