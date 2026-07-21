---
id: '007'
title: Teach SimPlant per-port motor mount orientation (fwd_sign) so sim ground-truth
  pose matches a mirrored-motor robot config
status: done
use-cases:
- SUC-007
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Teach SimPlant per-port motor mount orientation (fwd_sign) so sim ground-truth pose matches a mirrored-motor robot config

## Description

Teach `TestSim::SimPlant` each motor port's `fwd_sign` (the mount-orientation
correction firmware already applies), and apply it **only** at the boundary
where `SimPlant` feeds wheel positions into `OtosPlant` (ground-truth pose).
`TestSim::WheelPlant`'s own physics and the wire-level encoder-readback path
stay byte-for-byte unchanged — this fix touches nothing that currently
passes, only the one place a per-port sign convention was silently assumed
to be uniform.

## Context

**Sprint 114 Revision 2** (see `sprint.md`'s own Revision 2 note and Design
Rationale Decision 7 for the full writeup). Ticket 001's Revision 1 fix
(`Devices::Motor::reconfigure()`) made `SimHarness::configureMotor()` reach a
genuinely working motor for the first time — which means
`tovez_nocal.json`'s real, hardware-verified asymmetric `fwd_sign`
(`fwd_sign_left: 1` / `fwd_sign_right: -1`, issue 088-002) now reaches the
simulated motor for the first time too. That surfaced a **pre-existing,
orthogonal** gap: `TestSim::WheelPlant`/`SimPlant` have no notion of a
mirror-mounted motor, so a commanded straight twist drives the two
`WheelPlant`s in opposite physical directions and the simulated robot spins
in place instead of translating — even though firmware's own encoder
decode (which applies the same `fwd_sign`) self-consistently believes it
drove straight. Every other sim harness in this tree uses
`bench_test_config.cpp`'s symmetric `fwdSign=+1/+1` stand-in, which is why
this never surfaced before ticket 001.

Ticket 001's programmer parked this by xfailing
`test_distance_encoder_and_otos_match_truth` and
`test_heading_encoder_and_otos_match_truth` in
`src/tests/sim/test_motor_primitive.py`, with a full root-cause writeup in
that file's own "pytest entry points" section comment — read it before
starting.

**Why this is not a firmware bug and needs no `src/firm/` change.**
Firmware's own `NezhaMotor::position()`/`velocity()` already correctly undo
`fwd_sign` on the encoder-read side (`nezha_motor.cpp` line ~291:
`pos = (raw/10) * wheelTravelCalib * fwdSign`), mirroring the write side's
`effective = fwdSign * written`. Firmware's internal accounting
(`position()`, `velocity()`, `appliedDuty()`) is entirely in a consistent
"logical, forward-positive" frame end to end — that round trip was never
broken. The break is downstream, in test-only ground-truth code:
`SimPlant::tick()` feeds `left_.position()`/`right_.position()` — each
`WheelPlant`'s own **physical** (wire/motor-shaft-frame) position, driven
directly from the wire-parsed `effective` duty in `handleMotorWrite()` —
straight into `OtosPlant::step()`, which requires both wheel position
deltas to already share one "vehicle forward positive" convention
(`BodyKinematics::forward()`, `otos_plant.cpp`). Nothing today converts a
mirrored port's physical position into that shared convention before the
two are combined. `WheelPlant`'s own physics and the wire-level encoder
simulation (`SimPlant::handleMotorRead()`) must **not** change — they
already correctly reproduce what a real chip's raw encoder would report for
a physically-mirrored motor, and changing them would re-introduce the same
bug at the wire-decode step instead of fixing it.

## Approach

1. **`src/sim/sim_plant.h`/`.cpp`**: add
   `void setFwdSign(int port, int sign);` (port 1=left/2=right, matching
   every other per-port knob's convention — `setDisconnected()`,
   `setEncScaleErr()`, etc.). Store `leftFwdSign_`/`rightFwdSign_`, both
   defaulting to `+1` — a genuine no-op for any caller that never calls
   this, which is every existing symmetric-`fwd_sign` harness today.

2. **`SimPlant::tick()`**: change
   ```cpp
   otos_.step(left_.position(), right_.position(), dt);
   ```
   to
   ```cpp
   otos_.step(leftFwdSign_ * left_.position(), rightFwdSign_ * right_.position(), dt);
   ```
   `left_.position()`/`right_.position()` themselves are **not** touched —
   they still feed `handleMotorRead()`'s wire-level encoder simulation
   exactly as before. Only the values handed to `OtosPlant` are corrected
   into the shared logical/vehicle-forward convention.

3. **`SimPlant::setTruePose()`**: apply the same correction to the
   `otos_.reset(...)` call's wheel-baseline args, so the delta baseline
   `OtosPlant::reset()` captures is in the same corrected frame `step()`
   uses on the next call:
   ```cpp
   otos_.reset(x, y, heading, leftFwdSign_ * left_.position(), rightFwdSign_ * right_.position());
   ```
   An inconsistent baseline here would inject a phantom one-cycle jump on
   the very next `tick()` — exactly the failure mode `setTruePose()`'s own
   existing header comment already warns about for the position/OTOS-reset
   coupling. Do not introduce a new version of that bug.

4. **`src/sim/sim_harness.h`**, `configureMotor()`: after
   `armorX_.reconfigure(cfg)` succeeds, add
   `plant_.setFwdSign(port, cfg.fwdSign);`. This is the same call site
   `sim_ctypes.cpp`'s `sim_configure_motor()` already routes through
   (`harness->configureMotor(port, cfg)`), so both the C++ direct-harness
   path and the ctypes/TestGUI robot-select path pick up the fix with this
   one change.

5. **`src/tests/sim/test_motor_primitive.py`**: remove both
   `@pytest.mark.xfail(...)` decorators on
   `test_distance_encoder_and_otos_match_truth` and
   `test_heading_encoder_and_otos_match_truth`, and trim the long
   root-cause comment block above them to a short pointer at this ticket
   (the comment's diagnostic content can be summarized, not deleted
   outright — it documents a real, non-obvious gap future readers will
   want the "why" for). Both tests must pass unmodified otherwise — the
   tests themselves were always correct; only the plant was wrong.

6. **New direct regression test**: a targeted C++ case (new file under
   `src/tests/sim/plant/`, or extend an existing plant-level harness if one
   already exercises `SimPlant` directly) that: calls `setFwdSign(1, 1)` /
   `setFwdSign(2, -1)` directly on a `SimPlant`, drives both `WheelPlant`s
   with the same wire-level (`handleMotorWrite`) duty magnitude/sign a real
   mirrored pair would receive for straight travel, and asserts the
   resulting `OtosPlant` ground truth **translates** (nonzero `x()`,
   `heading()` within a tight tolerance of 0) rather than **spins**
   (nonzero `heading()`, `x()` near 0). This is the direct, minimal
   regression test for the bug — independent of the full
   firmware+TestGUI/ctypes pipeline `test_motor_primitive.py` exercises,
   and should fail fast and legibly if this fix ever regresses.

## Files to Touch

- `src/sim/sim_plant.h`, `.cpp` (`setFwdSign()`, `tick()`, `setTruePose()`)
- `src/sim/sim_harness.h` (`configureMotor()` — one new call)
- `src/tests/sim/test_motor_primitive.py` (remove the two `xfail` markers,
  trim the justifying comment)
- New (or extended) test file under `src/tests/sim/plant/` — direct
  `SimPlant` mount-orientation regression case

## Acceptance Criteria

- [x] `SimPlant::setFwdSign()` exists, defaults to `+1` per port, and is a
      genuine no-op for any caller that never invokes it — every existing
      symmetric-`fwd_sign` harness (all 9 via `bench_test_config.cpp`'s
      `fwdSign=+1/+1`) produces byte-for-byte unchanged ground truth.
- [x] `SimHarness::configureMotor()` (and therefore `sim_ctypes.cpp`'s
      `sim_configure_motor()`) propagates `cfg.fwdSign` to
      `SimPlant::setFwdSign()`.
- [x] A straight twist (`v_x` nonzero, `omega=0`) against
      `tovez_nocal.json`'s real `fwd_sign` (+1 left / -1 right) produces
      ground-truth OTOS translation (nonzero displacement, heading change
      near zero) — not a spin.
- [x] `test_distance_encoder_and_otos_match_truth` and
      `test_heading_encoder_and_otos_match_truth` pass with their `xfail`
      markers removed.
- [x] `WheelPlant::position()`/`velocity()`/`reportedPosition()` and every
      existing fault-injection-knob behavior are byte-for-byte unchanged —
      this fix touches only the `OtosPlant`-feeding boundary in `SimPlant`,
      never `WheelPlant`'s own physics or the wire-level encoder path.
- [x] No `src/firm/` file is touched — this is a test/sim-infrastructure
      fix; firmware's own `fwd_sign` encode/decode round trip was already
      correct (see Context above).

## Testing

- **Existing tests to run**: `test_motor_primitive.py` (both
  previously-xfailed cases must now pass unmarked), full `src/tests/sim`
  suite (regression check that the correction is a genuine no-op under
  every existing symmetric-`fwd_sign` harness).
- **New tests to write**: the direct `SimPlant` mount-orientation
  regression case (Approach step 6).
- **Verification command**: `uv run python -m pytest src/tests/sim -v -s`
  (targeted), then full suite `uv run python -m pytest` before marking
  done.
