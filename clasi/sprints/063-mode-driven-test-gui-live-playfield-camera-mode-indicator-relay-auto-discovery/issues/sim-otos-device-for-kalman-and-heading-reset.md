---
status: in-progress
sprint: '063'
tickets:
- 063-006
---

# Sim Mode needs a functional simulated OTOS (test the Kalman filter + heading reset)

## Problem

We cannot test the EKF/OTOS fusion or the "Set Robot @ 0,0" heading reset in the
simulator, because the sim has no functional OTOS device from the command/fusion
surface's point of view:

- **`OZ`/`OR`/`OI`/`OV` return `ERR nodev oz`** in the sim/host build — the OTOS
  command handlers' context (`OtosCtx.otos`) is not wired to the sim's `SimOdometer`.
  So the exact command the heading reset relies on (`OZ` → `setPositionRaw(0,0,0)`)
  can't even be exercised in sim.
- **`DBG OTOS` reports `otos=0,0,0 … valid=0`** even after a real turn (it reads a
  standalone bench-OTOS model that needs `sim_bench_otos_*` injection, separate from
  the `SimOdometer` that actually feeds the EKF).
- The `SimOdometer` that does feed the EKF returns an **injected pose by default**
  (not the tracked plant truth) and `SI`/`setPose` re-baselines it, so **`SI` alone
  holds heading at 0** in sim — i.e. the sim does NOT reproduce the hardware behavior
  where the OTOS retains an absolute heading and `correctEKF` drags the fused heading
  back after `SI`.

Net: the OTOS-correction path in `Odometry::correctEKF` — where the heading-reset bug
lives — is untested in the sim, and Sim mode in the Test GUI doesn't behave like
hardware for pose/heading reset.

## Goal — make Sim mode behave like hardware for OTOS fusion

1. **Wire the OTOS command surface to the sim OTOS.** `OZ`, `OR`, `OI`, `OV` must
   operate on the `SimOdometer` (or the active sim OTOS) in the sim/host build — no
   more `nodev`. `OZ` must zero the sim OTOS position + heading (`setPositionRaw(0,0,0)`).
2. **Model the OTOS as an independent absolute-heading source with retained state.**
   The sim OTOS should feed **valid** observations (position + heading) into the EKF,
   tracking the true plant pose (with optional noise/bias), and retain its own absolute
   reference — so `SI 0 0 0` alone does NOT hold heading (it drifts back toward the
   OTOS), exactly as on hardware. Zeroing it via `OZ` re-references it to the current
   orientation.
3. **Enable it in Sim mode by default** (or via the GUI Sim transport) so the Test GUI
   Sim mode exercises the same fusion path as hardware, and "Set Robot @ 0,0"
   (`ZERO enc` + `OZ` + `SI 0 0 0`) resets heading to 0 and holds.
4. **Regression test** in `tests/simulation`:
   - turn to a non-zero heading; `SI 0 0 0` alone → fused heading drifts back toward the
     OTOS (reproduces the bug);
   - `ZERO enc` + `OZ` + `SI 0 0 0` → fused heading resets to 0 and **holds** across many
     ticks (verifies the fix);
   - `OZ`/`OR`/`OI`/`OV` return `OK` in sim (not `nodev`).

## Guidance / findings (for planning)

- Sim OTOS lives at `source/hal/sim/SimOdometer.{h,cpp}`; wired via
  `source/hal/sim/SimHardware.h` `otos()` → `_odom`. `readTransformed` returns the
  injected pose by default; `enableSimModel(true)` switches to the accumulated
  plant-tracked pose. Error setters exist (noise/bias) and default to no-op (perfect).
- The OTOS command context (`OtosCtx`) is constructed where Robot registers commands
  (see `source/commands/OtosCommands.cpp` `otosReady()` → `c->otos` null in sim). Find
  where `OtosCtx.otos` is set for the real build and wire the sim equivalent.
- `Robot::otosCorrect` (`source/robot/Robot.cpp` ~200-295) already fuses OTOS via
  `estimate.addOtosObservation` → `Odometry::correctEKF` when `poseOk`. Confirm the sim
  path sets `state.actual.otos.valid` and feeds real (tracked) headings, not a constant.
- The SimTransport (`host/robot_radio/testgui/transport.py`) applies a field-error
  profile on connect (`_apply_field_profile`) and may need to enable the sim OTOS model.
- Keep golden-TLM / existing sim fusion tests passing (SimOdometer notes behavior
  preservation against the retired MockOtosSensor — don't regress those).

## Acceptance (behavioural)
- In Sim mode, `OZ`/`OR`/`OI`/`OV` succeed (no `nodev`).
- Without `OZ`: after a turn, `SI 0 0 0` alone leaves the fused heading drifting back to
  the OTOS reading (bug reproduced in sim).
- With `ZERO enc` + `OZ` + `SI 0 0 0`: fused heading resets to 0 and holds.
- Test GUI Sim mode: "Set Robot @ 0,0" makes the avatar heading go to 0 and stay 0.
- `tests/simulation` regression test covers the above; full suite green.
