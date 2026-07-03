---
id: '004'
title: Surface remaining sim-error knobs through SIMSET/SIMGET (encoder, OTOS)
status: open
use-cases:
- SUC-002
- SUC-005
- SUC-006
depends-on:
- '003'
github-issue: ''
issue: sim-error-model-runtime-settable-hardware-fit.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Surface remaining sim-error knobs through SIMSET/SIMGET (encoder, OTOS)

## Description

Six existing-but-hidden knobs are already fully implemented in
`PhysicsWorld`/`SimOdometer` — write-only setters exist, no getters and no
wire row. This is issue-1's entire TestGUI-knob-exposure ask, confirmed by
direct read of both headers, folded into the `SIMSET`/`SIMGET` surface ticket
003 just built rather than plumbed through the legacy ctypes path first.

Per-wheel encoder-report error (`source/hal/sim/PhysicsWorld.h`):
- `setEncoderScaleError(int side, float err)` (line 154) — no getter.
- `setEncoderSlip(int side, float fraction)` (line 158) — no getter.
- `setEncoderNoise(int side, float sigmaMm)` (line 141) — no getter.

OTOS sensor error (`source/hal/sim/SimOdometer.h`):
- `setLinearScaleError(float err)` / `setAngularScaleError(float err)`
  (lines 144-145) — no getters.
- `setDriftPerTickMm(float mm)` / `setDriftPerTickRad(float rad)`
  (lines 139-140) — no getters. These are PER-TICK internally; the wire
  keys are specified PER-SECOND (`otosLinDriftMmS`, `otosYawDriftDegS`),
  matching issue-1's plumbing guidance, so `SimCommands` must convert using
  `RobotConfig::controlPeriodMs` (`source/types/Config.h:167`) both on
  `SIMSET` (seconds → per-tick) and `SIMGET` (per-tick → seconds).
- `setLinearNoiseSigma(float sigma)` / `setYawNoiseSigma(float sigma)`
  (lines 121-122) — no getters (these are already reachable write-only via
  the existing `sim_set_otos_linear_noise`/`sim_set_otos_yaw_noise` ctypes
  functions, `tests/_infra/sim/sim_api.cpp:681,684`, but never readable and
  never wire-reachable).

## Acceptance Criteria

- [ ] `source/hal/sim/PhysicsWorld.h`: new const getters
      `encoderScaleErrL()`/`encoderScaleErrR()` (mirroring
      `setEncoderScaleError`), `encoderSlipL()`/`encoderSlipR()` (mirroring
      `setEncoderSlip`), `encoderNoiseL()`/`encoderNoiseR()` (mirroring
      `setEncoderNoise`) — the last pair is not explicitly named in
      `architecture-update.md`'s Step 5 getter list, but SUC-002's
      acceptance criterion ("Each of the six per-wheel encoder-report keys
      is `SIMSET`/`SIMGET`-able") requires it; follow the same
      mirror-the-setter pattern as the other four.
- [ ] `source/hal/sim/SimOdometer.h`: new const getters `linearScaleError()`,
      `angularScaleError()`, `driftPerTickMm()`, `driftPerTickRad()`,
      `linearNoiseSigma()`, `yawNoiseSigma()` — six getters mirroring the
      six existing setters listed above.
- [ ] `source/commands/SimCommands.cpp`'s `kSimRegistry[]` gains rows for:
      `encScaleErrL`/`encScaleErrR`, `encSlipL`/`encSlipR`,
      `encNoiseL`/`encNoiseR` (all → the new `PhysicsWorld` setter/getter
      pairs above; side 0/1 selects L/R, matching the existing
      `setEncoderScaleError`/`setEncoderSlip`/`setEncoderNoise`
      `(side, value)` signature), `otosLinScaleErr`/`otosAngScaleErr` (→
      `SimOdometer::setLinearScaleError`/`setAngularScaleError` and new
      getters), `otosLinNoise`/`otosYawNoise` (→
      `setLinearNoiseSigma`/`setYawNoiseSigma` and new getters).
- [ ] `otosLinDriftMmS`/`otosYawDriftDegS` rows: `SIMSET` converts the
      wire's per-second value to `SimOdometer`'s internal per-tick value
      using `RobotConfig::controlPeriodMs` before calling
      `setDriftPerTickMm`/`setDriftPerTickRad`; `SIMGET` converts back
      (per-tick → per-second) when reading `driftPerTickMm()`/
      `driftPerTickRad()`. Document the exact conversion formula
      (`per_second = per_tick * (1000.0f / controlPeriodMs)`, or the
      inverse) directly in `SimCommands.cpp` next to the two rows.
- [ ] Each of the six per-wheel encoder-report keys and each of the six OTOS
      error keys is `SIMSET`/`SIMGET`-able — new sim tests, one per group
      (or a combined parametrized test), per SUC-002 and SUC-005's
      acceptance criteria:
      - Setting `encScaleErrL` (only) produces a visible left/right encoder
        divergence in TLM `enc=` without moving `encpose=`/`otos=`/`pose=`
        away from the true trajectory an unaffected run would show.
      - Setting `otosLinScaleErr` alone changes `otos=`'s reported distance
        relative to the plant's true pose without perturbing `encpose=`.
- [ ] `SIMGET` (bare, no args) now dumps ALL registered keys from tickets 003
      and 004 combined — extend `test_sim_commands_registry.py` (ticket 003)
      or add a follow-on assertion confirming the full key set is present.
- [ ] Full default suite green: `uv run python -m pytest`.

## Testing

- **Existing tests to run**: `test_sim_commands_registry.py` (ticket 003);
  any existing encoder-scale-error / OTOS-noise tests (058-001 lineage);
  full default suite.
- **New tests to write**:
  - Per-wheel encoder-report-error `SIMSET`/`SIMGET` round-trip + behavioral
    test (as described above).
  - OTOS-error `SIMSET`/`SIMGET` round-trip + behavioral test, including the
    per-second/per-tick drift-conversion round-trip
    (`SIMSET otosLinDriftMmS=<v>` then `SIMGET otosLinDriftMmS` returns a
    value matching `<v>` within floating-point tolerance, accounting for the
    per-tick quantization).
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Pure additive-getter + registry-row work — no new error-model
math, since every setter already exists and is already exercised by existing
ctypes-based tests. Follow ticket 003's established `kSimRegistry[]` row
shape exactly. The one non-trivial piece is the per-second/per-tick drift
unit conversion, isolated entirely inside `SimCommands`'s two drift-row
handlers (or a tiny local helper) — `SimOdometer` itself is not changed
beyond adding getters.

**Files to modify**:
- `source/hal/sim/PhysicsWorld.h` — six new const getters.
- `source/hal/sim/SimOdometer.h` — six new const getters.
- `source/commands/SimCommands.cpp` — twelve new `kSimRegistry[]` rows (six
  encoder, six OTOS, two of which need the drift unit conversion).
- `docs/protocol-v2.md` §15 — extend the `SIMSET`/`SIMGET` key list with the
  twelve new rows.

**Testing plan**:
- Parametrized or per-key `SIMSET`/`SIMGET` round-trip tests for all twelve
  new keys.
- Behavioral tests confirming each error dimension is isolated (encoder
  error doesn't perturb `otos=`/true pose; OTOS error doesn't perturb
  `enc=`/true pose) — reuses the existing TLM field set, no new telemetry
  needed.
- Drift unit-conversion round-trip test.
- Full `uv run python -m pytest`.

**Documentation updates**: `docs/protocol-v2.md` §15 — extend with the
twelve new keys and the drift unit-conversion note.
