---
id: '002'
title: Per-port fwd_sign wheel-direction fix
status: open
use-cases: [SUC-001]
depends-on: []
github-issue: ''
issue: tovez-drive-motor-reversed-fwd-sign.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Per-port fwd_sign wheel-direction fix

## Description

The two drive-pair motors are mirror-mounted, but `scripts/gen_boot_config.py`
bakes `fwd_sign = +1` onto every port (`FWD_SIGN = 1` applied uniformly,
`gen_boot_config.py:71-74,258-266`). `msg::MotorConfig::fwd_sign` itself
and `NezhaMotor`'s consumption of it are already correct — the defect is
entirely that the generator's input is wrong. The generator's own
neighbor function, `travel_calib_for_ports()` (`gen_boot_config.py:176-194`),
already demonstrates the exact per-port, JSON-sourced-with-placeholder-
fallback pattern this fix needs.

## Implementation Plan

**Approach**: Add `fwd_sign_for_ports(cfg)` to `scripts/gen_boot_config.py`,
mirroring `travel_calib_for_ports()`'s exact shape: read
`calibration.fwd_sign_left` / `calibration.fwd_sign_right` from the robot
JSON when present, fall back to the existing `FWD_SIGN = 1` placeholder for
every other port. Wire the per-port values into `defaultMotorConfigs()`'s
codegen so each `out[i].setFwdSign(...)` call uses the per-port value
instead of the blanket `FWD_SIGN` literal (matching how `calib_lines` is
already built per-port). On the stand, determine empirically which port
(1 or 2) is mirror-mounted, set that port's `fwd_sign` to `-1` in
`data/robots/tovez_nocal.json` and `data/robots/tovez.json`, regenerate
`boot_config.cpp`, and verify with a straight `D` command that both
encoders increment positive.

**Files to create/modify**: `scripts/gen_boot_config.py` (new function +
codegen change), `data/robots/tovez_nocal.json` + `data/robots/tovez.json`
(new `calibration.fwd_sign_left`/`fwd_sign_right` fields),
`source/config/boot_config.cpp` (regenerated only — never hand-edited).

**Testing plan**: a Python unit test for `fwd_sign_for_ports()`'s mapping
logic (JSON field present → per-port value; JSON field absent → placeholder
for every port). On-stand verification (`D +<d> +<d>`, both wheels
forward, both encoders positive) is this ticket's own bench spot-check;
the full bench sign-off is ticket 009.

**Documentation updates**: none required — the generator's own module
docstring already documents this mapping pattern class.

## Acceptance Criteria

- [ ] `scripts/gen_boot_config.py` bakes per-port `fwd_sign` from
      `data/robots/tovez*.json`'s new `calibration.fwd_sign_left`/
      `fwd_sign_right` fields, mirroring `travel_calib_for_ports()`'s
      pattern (JSON value when present, `FWD_SIGN=1` placeholder
      otherwise).
- [ ] The physically-reversed drive port is identified on the stand and
      its JSON `fwd_sign` is set to `-1`.
- [ ] `source/config/boot_config.cpp` is regenerated (never hand-edited)
      and reflects the new per-port signs.
- [ ] On the stand: `D +<d> +<d>` (and straight `S`/`T`) drives both
      wheels forward with both encoders incrementing positive.
- [ ] A spin/turn command (opposite-signed L/R) is unaffected — still
      produces opposite wheel motion.
- [ ] The fix survives a clean rebuild (re-running `gen_boot_config.py`
      reproduces the same correct output).

## Testing

- **Existing tests to run**: full `uv run python -m pytest`.
- **New tests to write**: a Python test for `fwd_sign_for_ports()`'s
  mapping logic (present/absent JSON field cases).
- **Verification command**: `uv run python -m pytest` plus an on-stand
  spot-check (`D` command, read encoders).
