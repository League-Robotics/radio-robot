---
id: "002"
title: "MotorController and Drive coverage: PI+FF inner loop, ZOH velocity, wedge-detector, RatioPidController audit"
status: open
use-cases: ["SUC-002"]
depends-on: ["001"]
github-issue: ""
issue: ""
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 045-002: MotorController and Drive coverage: PI+FF inner loop, ZOH velocity, wedge-detector, RatioPidController audit

## Description

`source/control/MotorController.cpp` has 127 uncovered lines (59% coverage). The
existing `test_motor_controller.py` has 4 tests covering only: PWM nonzero at
target speed, encoder grows, integral windup clamped, stop zeroes PWM. Uncovered
paths include:

- Per-wheel ZOH velocity differentiation (the `refreshedWheel == 1` vs `== 2` branch
  in `controlTick` — only one branch may be exercised by current tests).
- Wedge-detector: `_stuckCountL/R` increment loop, `kWedgeThreshold` fire, `EVT enc_wedged`
  emission via `_evtFn/_evtCtx`, latch re-arm when encoder moves again.
- Arming grace (`_hasMovedL/R`): wedge detector should NOT fire before the wheel has
  moved at least once since the command started.
- `startDrive` (streaming re-seed, distinct from `startDriveClean`).
- `resetIntegrators` and `updateVelGains` code paths.
- `getVelocitySourceFlags` and `getEncoderPositions` accessors.
- `wheelWedgedL()`/`wheelWedgedR()` latch state after EVT fires.
- `source/subsystems/drive/Drive.cpp` (30 uncovered / 53%): Drive-level command-dispatch
  paths not exercised by current tests.

`source/control/RatioPidController.cpp` has 0% coverage and 23 uncovered lines.
Per `MotorController.h` note N13/030-010, `RatioPidController::update()` was removed
from `controlTick` — the class is dead code in the live control loop. This ticket
must confirm that finding and either:
- (a) If truly dead code: add a note to the CODAL-only exclusion set in `coverage.sh`
  (RatioPidController is simulatable-architecture dead code, exclude from denominator).
- (b) If somehow reachable: write a test exercising it via `Sim`.

Do NOT write a fake/dummy test that instantiates `RatioPidController` in isolation
just to hit lines — if it's dead code in the firmware, document it as such.

## Acceptance Criteria

- [ ] New file `tests/simulation/unit/test_motor_controller_coverage.py` created.
- [ ] Wedge-detector EVT: test sends motion command, holds encoder constant via `sim_set_enc_l`/`sim_set_enc_r` for `kWedgeThreshold` ticks (10 consecutive), and observes `EVT enc_wedged` in the async events buffer (`sim_get_async_evts`).
- [ ] Arming grace: test confirms no `EVT enc_wedged` fires when encoder is held constant before the wheel has moved (command just started, zero travel since start).
- [ ] Latch re-arm: test confirms `EVT enc_wedged` fires once per episode; after encoder moves again, a second stuck episode produces a second EVT.
- [ ] ZOH path: test exercises `controlTick` with `refreshedWheel=1` (left) and `refreshedWheel=2` (right) separately (via sim tick with alternating encoder updates or direct tick manipulation if available).
- [ ] `startDriveClean` vs `startDrive` distinction: streaming command (`S`) exercises `startDrive`; bounded command (`D`) exercises `startDriveClean`. Both paths covered.
- [ ] `updateVelGains`: test issues `SET vel.kP=0.1` (or similar) and confirms no crash / the gains are accepted.
- [ ] Drive.cpp paths: test exercises Drive-level command dispatch (at minimum `D`, `T`, and a cancellation path).
- [ ] RatioPidController audit: programmer confirms dead code in `MotorController.cpp` (grep for any call to `_ratioPid` or similar); result documented in ticket closing comment. If dead, update `coverage.sh` exclusion set comment. If alive, write a test.
- [ ] All existing tests still pass: `uv run --with pytest python -m pytest tests/simulation -q` exits 0.
- [ ] Golden-TLM, field-pin, vendor grep gates all green.

## Implementation Plan

### Approach

Create `tests/simulation/unit/test_motor_controller_coverage.py` using the `sim`
fixture. Tests use `sim.send_command(...)`, `sim._lib.sim_tick(sim._h, ...)`,
`sim._lib.sim_set_enc_l(...)`, `sim._lib.sim_set_enc_r(...)`, and
`sim._lib.sim_get_async_evts(...)` to control and observe the C++ MotorController.

For the wedge-detector test: command `S 200 200 9000` (disable watchdog first with
`SET sTimeout=30000`), then in a loop call `sim_tick` and `sim_set_enc_l(0)` /
`sim_set_enc_r(0)` to hold encoders at zero while keeping the command live. After
10+ ticks, check `sim_get_async_evts` for `EVT enc_wedged`.

For the ZOH path: standard `sim_tick` exercises the normal alternating-wheel path.
If the sim always calls `controlTick` with `refreshedWheel=0` or always `1`, check
`sim_api.cpp` for the exact pattern. The sim's `Robot::controlCollectSplitPhase()`
determines which refreshedWheel value is passed — programmer reads this to understand
the test setup needed.

### Files to create

- `tests/simulation/unit/test_motor_controller_coverage.py`

### Files to read (for implementation)

- `source/control/MotorController.cpp` — full file; understand wedge threshold, latch flags, arming grace
- `source/control/MotorController.h` — `kWedgeThreshold = 10`
- `tests/_infra/sim/sim_api.cpp` — `sim_get_async_evts`, `sim_set_enc_l/r` signatures
- `tests/simulation/unit/test_motor_controller.py` — existing tests to avoid duplication
- `source/control/RatioPidController.h` / `MotorController.cpp` — grep for any live use

### Testing plan

- Run the full simulation tier after writing each test group to keep the suite green.
- Verify `EVT enc_wedged` appears in `sim_get_async_evts` output (not just an OK reply).

### Documentation updates

- If RatioPidController is confirmed dead code, update the CODAL-only exclusion set
  note in `coverage.sh` (comment explaining the exclusion).
