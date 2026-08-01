---
status: pending
---

# Continuous duty-per-speed calibration

## Description

`duty_per_speed` converts a commanded wheel speed [mm/s] into motor duty. It is
a per-robot config value today, and on tovez it is wrong by ~1.9x — the robot
runs at 35% of commanded speed.

Stop treating it as configuration. Compile in one generic, measured default and
let the robot correct it per wheel while it drives, from information the
closed-loop controller already computes and currently discards.

Stakeholder decisions (2026-07-31):

- The learned value is **RAM-only**, relearned each boot. Nothing is persisted.
- The unmanaged `WHEELS` path **gets closed-loop trim**; today it is open-loop
  end to end with zero feedback.
- The generic default is **measured by a duty sweep on the stand**, not derived.

## Cause

Hand-maintaining the constant has not worked, for three compounding reasons:

1. **Three constants claim to be the plant gain and disagree by up to 2.34x** —
   `duty_per_speed` (0.00187325, implying 534 mm/s at full duty), `vel_kff`
   (0.0008, implying 1250), and `vel_kff`'s own derivation note (1/650). Bench
   measurement implies a fourth value, ~1/282.
2. **`wheel_gain`/`wheel_intercept` was fitted against `duty_per_speed`, which
   was fitted against it.** `_wheel_correction_note` in each robot JSON warns
   about exactly this coupling. On tovez the correction now points the *wrong
   way* — gain 1.47 encodes "the plant over-delivers" while it actually
   under-delivers, so it compounds the error instead of correcting it.
3. Nothing forces either to be revisited when the plant drifts.

Full measurement and the error decomposition:
`duty-per-speed-and-wheel-gain-disagree-with-the-plant.md`.

## Proposed fix

The mechanism largely exists. `src/motion/planner/wheel_trim.h:22-24` already
names the destination:

> *"A residual scale correction belongs in Drive's own measured gain, where it
> is per-wheel and per-direction; not here."*

`Motion::WheelTrim` is live, is per-wheel, engages its integrator **only in
`MovePhase::Hold`** (steady state — the correct gate for adaptation), and
exposes `integral()` marked *"observability"* (`wheel_trim.h:76`). That integral
in steady state **is** the fractional error in `dutyPerSpeed`. Nothing reads it.

Note that "add feedforward" and "update `dutyPerSpeed` from the PID" are one
lever, not two: `duty = speed x dutyPerSpeed` *is* the feedforward.

### 1. Measure the default

Set `wheel_gain_* = 1.0` / `wheel_intercept_* = 0.0` in `data/robots/*.json`
first, or the measurement inherits the mismatch it is meant to replace.

New `src/tests/bench/duty_sweep.py` (closest precedent:
`src/tests/bench/velocity_step_response.py`). With corrections at identity and a
known compiled `dutyPerSpeed`, commanding speed `s` produces a known duty
`s * dps` — so a speed ladder is a duty ladder. Sweep ~0.10-0.60 duty both
directions, dwell to steady state, fit `speed = m*duty + b` per wheel; the new
default is `1/m`. Report `b` (deadband intercept) and the L/R spread — this
finally quantifies the wheel mismatch as a measurement rather than an inference.

### 2. Remove the key from configuration

Compiled constant in `src/firm/app/drive.h`, documented as a starting estimate
the robot refines — not a calibration to maintain. Keep `setDutyPerSpeed()`; it
becomes the adaptation entry point.

Chain to update: `data/robots/{tovez,togov,tovez_nocal}.json` (key +
`_drive_calibration_note`); `robot_config.schema.json:569-578` (`control` is
`additionalProperties: false`, so a leftover key hard-fails);
`config/robot_config.py:254-256` (`ControlConfig`);
`src/scripts/gen_boot_config.py:525-552` (`_require()`s the key — **the build
aborts until this is updated**); `src/firm/config/boot_config.{h,cpp}`
(regenerate, never hand-edit); `src/firm/main.cpp:277-286`;
`calibration/sim_boot_config.py:131-182`, `src/sim/sim_ctypes.cpp:536-556`,
`src/sim/sim_harness.h:143-157`; `src/scripts/config_sync_allowlist.json:17-18`;
`src/tests/sim/support/bench_test_config.cpp:74`,
`src/tests/sim/unit/app_drive_harness.cpp`.

### 3. Close the loop on `WHEELS`

`robot_loop.cpp:262-280` -> `drive_.command()` -> `drive_.tick()` -> duty, with
no feedback anywhere.

**Reuse `Motion::WheelTrim`; do not write a new controller.** Sprint 128 deleted
one generation (`WheelVelocityPid`) and parked another (`WheelPid`/`stageDuty()`)
precisely because three coexisted — a fourth would undo that. `App::Drive` gains
a `WheelTrim` pair applied **only while Drive owns the command** (existing
`owned` gate, `drive.cpp:54-55`), so the planner path is untouched and
corrections never double-apply.

Raise at review: this places a motion-library class inside the firm base. The
alternative — a base-side duplicate — is worse.

Also fix `src/firm/app/drive.h:14-15`, now actively wrong: it claims closed-loop
control lives in "Motion::Planner's own duty stage," which is parked and whose
output was never read.

### 4. Adapt from the trim integral

Add `velocityTrim` [mm/s] to `Types::RobotState::Wheel`
(`src/firm/types/robot_state.h`), written by whoever ran the trim (planner
`stageTrim()`, or Drive on the `WHEELS` path), so one adapter serves both paths.
128-014 promoted `cmdVelocity` across the same boundary.

Per wheel, in `App::Drive`:

```
frac    = trim / target              // fractional scale error
dps_new = dps * (1 + lambda * frac)
```

Bumpless transfer — bleed the integral so the duty written does not step:

```
trim_new = (target + trim) * dps_old / dps_new - target
```

Gates (all must hold): `phase == Hold` or Drive-owned steady state;
`|target| > 60 mm/s` (above the deadband, where the map is least linear); no
wheel-frozen or fault flag set.

Safety — non-negotiable for an adaptive gain in the actuation path:

- clamp `dps` to **[0.5x, 2.0x] of the compiled default**
- `lambda` small (~0.02), rate-limited to one update per N cycles
- reset to the compiled default on `estop()`

**Observability ships with the feature, not after it.** Publish live
`dutyPerSpeedLeft/Right` and both trim integrals in telemetry and surface them
in the TestGUI. An adaptive gain nobody can watch is a runaway nobody can
diagnose — see `otos-frozen-at-a-constant-on-tovez` for what a live-but-wrong
value costs when nothing displays it.

## Verification

1. **Unit** — extend `src/motion/planner/tests/wheel_trim_test.cpp` and
   `src/tests/sim/unit/test_app_drive.py`: a known plant-gain error converges
   `dps` to truth; the clamp holds against a divergent error; duty is continuous
   across an update.
2. **Sim** — a straight leg with deliberately mismatched L/R plant gains
   converges to two different `dps` values and drives straight. This is the case
   that failed repeatedly on the bench; the sim must reproduce it, per the
   standing SIM-equals-bench rule.
3. **Bench, on the stand** — re-run `duty_sweep.py` after adaptation; the
   learned value must match the measured line.
4. **Bench, acceptance spec** — the agreed +500 criteria: cruise 150 mm/s
   +/-10%, ripple <= +/-10 mm/s, |vL-vR| <= 10 mm/s, elapsed ~4 s, encoders
   500 +/- 15 mm, heading <= 3 deg, and camera-measured travel 500 +/- 25 mm —
   the encoders are not allowed to be right on their own.

## Out of scope

Flash persistence and host write-back (both deliberately deferred); deleting the
`wheel_gain`/`wheel_intercept` mechanism (set to identity here, left inert).

## Related

- [[duty-per-speed-and-wheel-gain-disagree-with-the-plant]] — the measurement
  and error decomposition this issue acts on
- [[testgui-unmanaged-drive-lease-expiry-and-terminal-pivot]] — the L/R mismatch that
  per-wheel adaptation should absorb
- [[estop-did-not-stop-write-on-change-vs-latching-brick]] — critical, unrelated
  but must land before any further bench driving
- [[otos-frozen-at-a-constant-on-tovez]] — precedent for why a live value with
  no display is expensive
