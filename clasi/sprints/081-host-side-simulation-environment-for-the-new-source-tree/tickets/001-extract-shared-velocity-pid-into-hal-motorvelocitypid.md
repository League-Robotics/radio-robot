---
id: '001'
title: Extract shared velocity PID into Hal::MotorVelocityPid
status: in-progress
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: host-side-simulation-environment-for-the-new-tree-design-write-up.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Extract shared velocity PID into Hal::MotorVelocityPid

## Description

`NezhaMotor::runVelocityPid()` (`source/hal/nezha/nezha_motor.cpp:291`,
declared `source/hal/nezha/nezha_motor.h:146`) embeds the velocity control
law directly inside the Nezha leaf. The upcoming simulated motor
(`Hal::SimMotor`, ticket 003) must run the **identical** control law, not a
re-derived approximation — this is the design write-up's own highest-flagged
correction ("the sim must run the real PID, not a re-derived approximation").
This ticket extracts that control law, byte-for-byte, into a pure,
host-clean class — `Hal::MotorVelocityPid` — with no I2C/CODAL dependency,
and refactors `NezhaMotor` to call it. This is a **behavior-preserving
refactor only**: no gain, anti-windup shape, or output-domain change.

This ticket has no dependency on any other ticket in this sprint and can
start immediately.

## Acceptance Criteria

- [x] `Hal::MotorVelocityPid` (`source/hal/velocity_pid.{h,cpp}`) exists,
      compiles standalone with no `#include "MicroBit.h"` and no I2C
      dependency, and reproduces `runVelocityPid()`'s exact math: the
      `iOld`-ordered output (`spSign*ff + kp*err + iOld`, output computed
      from the PRE-update integral), the anti-windup back-calculation
      against `+/- i_max`, the `min_duty`-as-integrator-freeze-deadband
      threshold on `|target|`, and the `dt<=0 -> kNominalDt` (~24 ms)
      fallback.
- [x] `NezhaMotor` owns a `Hal::MotorVelocityPid pid_` member (replacing the
      bare `integral_` float) and calls `pid_.compute(...)` from `tick()`'s
      VELOCITY case where it used to call `runVelocityPid(...)` directly.
      `tick()`'s 5-step call order (`processResetIfPending` ->
      encoder-sample -> `updateWedgeDetector` -> mode dispatch ->
      `updateRestTracking`) is otherwise byte-for-byte unchanged.
- [x] `runVelocityPid()`'s old declaration/definition is deleted (no
      unreferenced duplicate left behind, matching the project's own
      "duplicated-decision" anti-pattern discipline already applied to
      `readEncoderSettle()` in sprint 079).
- [ ] **Bench step-response comparison (hardware-bench-testing gate,
      `.claude/rules/hardware-bench-testing.md` — required, not optional):**
      deploy pre- and post-extraction builds (`mbdeploy deploy --build`),
      command an identical velocity step on the stand, and confirm rise
      time, overshoot, and settle time match within measurement noise. This
      ticket is not done until this is confirmed on real hardware.
      **NOT YET DONE — deliberately deferred to the team-lead per this
      ticket's dispatch instructions (physical-hardware step withheld from
      the implementing agent as a supervised, separate step). See the
      programmer's closing report for a ready-to-run procedure.**
- [x] A new standalone-compiled test (following the existing
      `tests/sim/unit/*_harness.cpp` ad hoc-compile convention — see
      `test_motor_policy.py`'s pattern, no CMake) exercises
      `MotorVelocityPid::compute()` in isolation: a velocity step converges
      without oscillation blow-up, anti-windup clamps the integral under a
      saturating target, and the `dt<=0` path substitutes `kNominalDt`
      rather than dividing by zero or NaN-ing.
- [x] Existing `tests/sim/unit/*` harnesses (`motor_policy`, `drivetrain`,
      `i2c_bus_clearance`, `nezha_flipflop`, `dev_command_outbox`) and
      `uv run python -m pytest` still pass with no regression.
- [x] No unit-suffixed identifier is introduced (`.claude/rules/coding-standards.md`);
      `compute(target, measured, dt)` names quantities, `dt`'s unit lives in
      a `// [s]` trailing comment tag, matching the existing convention.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/sim tests/unit`;
  the existing `tests/sim/unit/*_harness.cpp` compiled harnesses (motor
  policy, drivetrain, i2c bus clearance, Nezha flip-flop, dev command
  outbox) must all still pass unchanged.
- **New tests to write**: `tests/sim/unit/velocity_pid_harness.cpp` +
  `tests/sim/unit/test_velocity_pid.py` (mirrors `test_motor_policy.py`'s
  compile-and-run pattern) — step response, anti-windup saturation, and
  `dt<=0` fallback assertions.
- **Verification command**: `uv run python -m pytest tests/sim -q`, plus the
  hardware bench step-response comparison described above (not
  pytest-automatable — a manual HITL comparison per
  `.claude/rules/hardware-bench-testing.md`).

## Implementation Plan

**Approach:**

1. Create `source/hal/velocity_pid.h` declaring
   `class Hal::MotorVelocityPid` with one public method,
   `float compute(float target, float measured, float dt, const msg::Gains& gains, float minDuty);`
   (gains/`minDuty` passed per call rather than cached inside the class, so
   `NezhaMotor`'s own `config_` stays the single source of truth for
   calibration — no second copy of `MotorConfig` data). Internal state:
   one `float integral_ = 0.0f;` member, plus the ported
   `kNominalDt` constant.
2. Port `runVelocityPid()`'s body verbatim into
   `source/hal/velocity_pid.cpp`'s `compute()` — same variable names where
   reasonable, same order of operations, same comments explaining the
   `iOld`-ordering and the documented divergence from `source_old`'s
   `ReInit()` stale-D quirk (carry the existing code comment forward, it is
   still accurate and still load-bearing context for a future reader).
3. Refactor `source/hal/nezha/nezha_motor.h`: remove `float integral_;` and
   the `float runVelocityPid(...)` private declaration; add
   `#include "hal/velocity_pid.h"` and a `Hal::MotorVelocityPid pid_;`
   member.
4. Refactor `source/hal/nezha/nezha_motor.cpp`: in `tick()`'s `Mode::VELOCITY`
   case, replace `runVelocityPid(velocityTarget_, filteredVelocity_, dt)`
   with `pid_.compute(velocityTarget_, filteredVelocity_, dt, config_.vel_gains, config_.min_duty)`.
   Delete the old `runVelocityPid()` definition.
5. Write the new standalone harness (`tests/sim/unit/velocity_pid_harness.cpp`)
   exercising `MotorVelocityPid::compute()` directly — no HAL, no CODAL,
   matching `motor_policy_harness.cpp`'s existing shape.
6. `mbdeploy probe` to confirm the robot is connected and mounted on its
   stand (wheels off the ground — safe to spin freely per
   `.claude/rules/hardware-bench-testing.md`); `mbdeploy deploy --build`;
   run a velocity step-response comparison before committing (capture a
   baseline off the CURRENT tree first, then repeat identically after the
   refactor) — document the comparison (rise time / overshoot / settle) in
   the ticket's closing notes.

**Files to create:**
- `source/hal/velocity_pid.h`
- `source/hal/velocity_pid.cpp`
- `tests/sim/unit/velocity_pid_harness.cpp`
- `tests/sim/unit/test_velocity_pid.py`

**Files to modify:**
- `source/hal/nezha/nezha_motor.h`
- `source/hal/nezha/nezha_motor.cpp`

**Testing plan:** see "Testing" section above.

**Documentation updates:** none required beyond code comments — no wire
protocol, architecture-doc, or config-schema change. If the bench
comparison surfaces anything worth recording for future maintainers (e.g.
measured rise-time/overshoot numbers), consider a short note in
`docs/knowledge/` following the project's existing hard-won-knowledge
convention (`.clasi/knowledge/`) — optional, not a hard requirement of this
ticket.
