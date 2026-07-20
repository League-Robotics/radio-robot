---
id: '003'
title: 'Eliminate nezha_motor.h''s write-shaping ship defaults: reversalDwell/outputDeadband
  become required config'
status: open
use-cases: [SUC-002, SUC-004]
depends-on: ['002']
github-issue: ''
issue: config-as-truth-completion-no-defaults-fail-closed-version-erase.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Eliminate nezha_motor.h's write-shaping ship defaults: reversalDwell/outputDeadband become required config

## Description

Delete `nezha_motor.h`'s `kDefaultReversalDwell`/`kDefaultOutputDeadband`
ship-default substitution. `Devices::MotorConfig::reversalDwell`/
`outputDeadband` collapse from `Opt<float>` to plain required `float`,
matching every other field in that struct. Depends on ticket 002 because it
needs `gen_boot_config.py` to always emit real values for these two fields —
today it deliberately leaves them unset (`.has == false`) on every build.

## Context

`nezha_motor.cpp`'s constructor currently does:

```cpp
reversalDwell_ = config.reversalDwell.has ? config.reversalDwell.val : kDefaultReversalDwell;
outputDeadband_ = config.outputDeadband.has ? config.outputDeadband.val : kDefaultOutputDeadband;
```

with `kDefaultReversalDwell = 100.0f` (ms) and `kDefaultOutputDeadband =
0.03f` (duty), and `gen_boot_config.py`'s own generated comment says this is
deliberate: "reversal_dwell / output_deadband are left unset (.has == false)
on purpose." This is precisely the class of hidden code-side default the
config-as-truth issue targets, and — separately — it is also the exact value
ticket 005's deadband-compensation fix needs to be a real, config-sourced
number rather than a private implementation constant, since the fix boosts
*to* `outputDeadband_`.

## Approach

1. **`src/firm/devices/device_config.h`**: change `Opt<float> reversalDwell
   = {};` and `Opt<float> outputDeadband = {};` to plain `float
   reversalDwell = 0.0f;` / `float outputDeadband = 0.0f;` (matching the
   shape of every other field in `MotorConfig` — `wheelTravelCalib`,
   `velGains`, `slewRate`, etc. — none of which use `Opt<float>`).

2. **`src/firm/devices/nezha_motor.h`**: delete `static constexpr float
   kDefaultReversalDwell = 100.0f;` and `static constexpr float
   kDefaultOutputDeadband = 0.03f;`.

3. **`src/firm/devices/nezha_motor.cpp`** constructor: replace the two
   `.has ? .val : kDefault*` lines with plain `reversalDwell_ =
   config.reversalDwell; outputDeadband_ = config.outputDeadband;`.

4. **Add the two new JSON keys**: `control.output_deadband` (duty fraction,
   e.g. `0.03`) and `control.reversal_dwell_ms` (ms, e.g. `100.0`) to
   `data/robots/robot_config.schema.json`'s `control` object and to all
   three robot JSONs, seeded with exactly `0.03`/`100.0` (today's values —
   value-preserving).

5. **`src/scripts/gen_boot_config.py`**: add `output_deadband_for_config(cfg)`/
   `reversal_dwell_for_config(cfg)` (matching `arrive_dwell_for_config()`'s
   shape) reading the two new required keys (hard-fail if absent, per
   ticket 002's established convention); in `defaultMotorConfigs()`'s
   generated output, always call `out[i].setReversalDwell(...)`/
   `out[i].setOutputDeadband(...)` unconditionally — delete the "left unset
   on purpose" comment and behavior entirely.

6. **`src/firm/main.cpp`**'s `toDeviceMotorConfig()`: currently does
   `cfg.reversalDwell.has = src.reversal_dwell.has; cfg.reversalDwell.val =
   src.reversal_dwell.val;` (and the `outputDeadband` equivalent). Since
   `Devices::MotorConfig`'s fields are now plain `float`, this becomes a
   plain value copy: `cfg.reversalDwell = src.reversal_dwell.val;` (reading
   the wire `Opt<T>`'s `.val` unconditionally, since ticket 002 + this
   ticket guarantee `gen_boot_config.py` always sets `.has = true`). Do
   **not** change the wire `msg::MotorConfig` proto schema itself unless
   truly necessary — prefer reading `.val` and ignoring `.has` at this one
   call site over a wire-format change.

7. **`src/tests/sim/support/bench_test_config.cpp`** (ticket 001's new
   file): `benchTestMotorConfig()` must set `reversalDwell`/`outputDeadband`
   explicitly to `100.0f`/`0.03f` (today's values). Ticket 001's own copy of
   `makeMotorConfig()`'s body never set these two fields (production
   `NezhaMotor` substituted the ship defaults itself at the time, which was
   correct then) — after *this* ticket removes that substitution, leaving
   them unset would silently give every migrated test zero write-shaping
   instead of the historical 100ms/0.03. Check whether ticket 001 already
   landed without these two lines and add them here if so.

## Files to Touch

- `src/firm/devices/device_config.h`
- `src/firm/devices/nezha_motor.h`, `.cpp`
- `src/firm/main.cpp` (`toDeviceMotorConfig()`)
- `data/robots/robot_config.schema.json`, `tovez_nocal.json`, `tovez.json`,
  `togov.json`
- `src/scripts/gen_boot_config.py`
- `src/tests/sim/support/bench_test_config.cpp` (ticket 001's new file — add
  the two explicit field sets if not already present)

## Acceptance Criteria

- [ ] `Devices::MotorConfig::reversalDwell`/`outputDeadband` are plain
      `float`, not `Opt<float>`.
- [ ] `grep -n "kDefaultReversalDwell\|kDefaultOutputDeadband"
      src/firm/devices/nezha_motor.h src/firm/devices/nezha_motor.cpp` finds
      nothing.
- [ ] `gen_boot_config.py` always emits explicit `setReversalDwell()`/
      `setOutputDeadband()` calls with real, JSON-sourced values; omitting
      `control.output_deadband`/`control.reversal_dwell_ms` from any robot
      JSON fails the build (per ticket 002's established convention).
- [ ] A rebuild against the currently-active profile produces byte-identical
      `writeShapedDuty()` behavior to pre-ticket (100ms dwell, 0.03
      deadband) — regression, not yet a behavior change (ticket 005 changes
      behavior).
- [ ] The ~40 migrated sim test harnesses (ticket 001) still pass —
      `benchTestMotorConfig()` explicitly sets both fields to the historical
      values.

## Testing

- **Existing tests to run**: full `src/tests/sim` suite (write-shaping
  behavior must be unchanged); `test_sim_boot_config_parity.py`.
- **New tests to write**: a `NezhaMotor` unit test constructing with an
  explicit `reversalDwell=0.0f`/`outputDeadband=0.0f` `MotorConfig` (the new
  all-zero default) and confirming `writeShapedDuty()` behaves as documented
  ("0 skips the dwell transition entirely," matching the sim's own
  pre-existing documented configuration) — this proves the collapse from
  `Opt<float>` didn't silently change the *meaning* of an explicit zero.
- **Verification command**: `uv run python -m pytest src/tests/sim -v`, then
  full suite.
