---
id: '003'
title: Shrink Devices::Motor/NezhaMotor to duty-only protocol + bus hygiene + dwell/deadband
  + clamp
status: in-progress
use-cases:
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Shrink Devices::Motor/NezhaMotor to duty-only protocol + bus hygiene + dwell/deadband + clamp

## Description

Shrink `Devices::Motor`/`Devices::NezhaMotor` (`src/firm/devices/{motor.h,
nezha_motor.{h,cpp}}`) per the base-explicit-loop-sketch's KEEP/MOVE/DELETE
inventory (issue item table; sprint architecture Step 3). KEEP: split-phase
0x46 protocol, `hardReset()`, `connected()`/failure-hold, bus/write
hygiene (`fwdSign`, clamp ±100%, integer-% quantization, write-on-change,
NAK retry, write-rate throttle), the slew cap (UNMODIFIED — see ticket
010, do not touch it here), reversal dwell + output deadband
(`writeShapedDuty()`), `wheelTravelCalib`. DELETE: `setVelocity()`, the
embedded `MotorVelocityPid`, the kff mapping, the freshness gate, glitch
rejection, the EMA/least-squares velocity-estimator pair (`velEstMode_`
etc.), duty-boxcar smoothing (`dutyAvgWindow_`/`setDutyAvg()`). Also trim
`Devices::MotorConfig` (`device_config.h`): remove `velGains`,
`velFiltAlpha`, `velDeadband` (dead fields once nothing reads them);
KEEP `slewRate`/`outputDeadband`/`reversalDwell`/`wheelTravelCalib`/
`port`/`fwdSign`/`polled`. Target ~200 lines for `nezha_motor.cpp` (from
885). See sprint architecture Step 3 (`Devices::Motor`/`NezhaMotor`) and
Migration Concerns for the expected CI-red-then-green window this opens
(closes when ticket 004's observer replaces the deleted conditioning
logic).

## Acceptance Criteria

- [ ] `nezha_motor.cpp` measured at or near the ~200-line target;
      `grep -rn "MotorVelocityPid\|setVelocity\|velFiltAlpha\|dutyAvgWindow"
      src/firm/devices/` returns zero hits.
- [ ] `Devices::Motor`'s interface no longer declares `setVelocity()`/
      `velocityTarget()`/`gains()`/`applyGains()`'s gain-bearing surface
      (narrow `applyGains()` to `travelCalib` only, or remove it pending
      ticket 008's CONFIG-routing split — state which in the PR).
  - [ ] The slew cap's own code path (`writeRawDuty()`'s slew section) is
      untouched — this ticket must not pre-empt ticket 010's decision.
- [ ] `Devices::MotorConfig` no longer declares `velGains`/`velFiltAlpha`/
      `velDeadband`.

## Testing

- **Existing tests to run**: `devices_motor_harness.cpp`/
  `devices_motor_armor_harness.cpp` (expect PID/velocity-estimator-
  specific scenarios to go red or be deleted here — confirm protocol/
  dwell/deadband/clamp scenarios stay green).
- **New tests to write**: a duty-passthrough scenario confirming
  `setDuty()`→`tick()` writes exactly the given duty through dwell/
  deadband shaping, replacing whatever PID-chase scenarios are deleted.
- **Verification command**: `uv run pytest` plus the sim `ctest` suite.
