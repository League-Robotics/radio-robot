---
id: '003'
title: Add a noise-only EKF setNoise() path; wire ekfRHead through it
status: open
use-cases:
- SUC-002
depends-on: []
github-issue: ''
issue: set-config-not-propagated-to-planner.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add a noise-only EKF setNoise() path; wire ekfRHead through it

## Description

EKF sensor-fusion noise is a third, distinct failure mode from the Planner
and Drive shadow-cache bugs. `PhysicalStateEstimate::initEKF()` →
`EKFTiny::init()` is called exactly once, from `Drive`'s constructor. No
live-update path exists for the one currently-exposed noise key
(`ekfRHead`/`ekfROtosTheta`), annotated or not.

Naively "fixing" this by re-invoking `initEKF()` on every relevant `SET`
would be a regression in its own right: `EKFTiny::init()`'s documented
contract (`EKFTiny.cpp:43-69`) is "set noise parameters AND reset state and
covariance" — it zeroes `_ekf.x[]`/`_ekf.P[]`. A live re-init on a runtime
`SET ekfRHead=<x>` would teleport the robot's actual, in-flight fused pose
back to the origin mid-mission — worse than the current bug of the value
simply not propagating.

This ticket adds a new, narrow `setNoise()` method to `EKFTiny` that
updates `_Q`'s diagonal and the cached measurement-noise scalars
(`_rOtosXy`/`_rOtosV`/`_rEncV`) WITHOUT touching `x[]`/`P[]`/the
rejection-streak counters, threads it through `Odometry` and
`PhysicalStateEstimate`, and wires `Drive::configure()` to call it whenever
a `"drive"`-annotated key is SET (requiring `ekfRHead`'s registry row to
gain the `"drive"` annotation it currently lacks).

The `setNoise()` signature is designed to also accept the seven
not-yet-registered EKF noise fields (`ekfQxy`, `ekfQtheta`, `ekfROtosXy`,
`ekfQv`, `ekfQomega`, `ekfROtosV`, `ekfREncV`) even though they are not
SET-able today, so that a future sprint (069 is the likely candidate,
per the sim-to-hardware fitting workflow) can expose them via the registry
with no further plumbing changes to `EKFTiny`/`Odometry`/
`PhysicalStateEstimate` — only new registry rows would be needed.

See `architecture-update.md` Step 4-5 item 3 and Design Rationale
Decision 3 for the full design (the alternative — reusing `init()` with a
"first call only" guard flag — was rejected as adding hidden statefulness
to a class whose value is being small and stateless-feeling; a future
caller that legitimately wants a full reset would need a second method
anyway).

## Acceptance Criteria

- [ ] `source/state/EKFTiny.h`/`.cpp`: new method `setNoise(q_xy, q_theta,
      q_v, q_omega, r_otos_xy, r_otos_v, r_enc_v)` that updates `_Q`'s
      diagonal and `_rOtosXy`/`_rOtosV`/`_rEncV` exactly as `init()` does,
      but does NOT touch `_ekf.x[]`, `_ekf.P[]`, or any rejection-streak
      counters.
- [ ] `EKFTiny::init()` itself is unchanged in behavior — its
      state/covariance reset remains, now explicitly documented (comment)
      as boot-only / not safe to call mid-mission.
- [ ] `source/control/Odometry.h`/`.cpp`: new `setNoise(...)` method
      forwarding to `_ekf.setNoise(...)`, additionally refreshing
      `Odometry`'s own cached `_rOtosTheta` (read by `correctEKF()`).
- [ ] `source/state/PhysicalStateEstimate.h`/`.cpp`: new `setNoise(...)`
      method forwarding to `_odometry.setNoise(...)`.
- [ ] `source/subsystems/drive/Drive.cpp`, `Drive::configure()`: gains a
      call to `_est.setNoise(_robCfg.ekfQxy, _robCfg.ekfQtheta,
      _robCfg.ekfQv, _robCfg.ekfQomega, _robCfg.ekfROtosXy,
      _robCfg.ekfROtosV, _robCfg.ekfREncV, _robCfg.ekfROtosTheta)`, sourced
      from the live `_robCfg` (already reflects the just-committed SET) —
      not from the `cfg` parameter passed into `configure()`.
- [ ] `source/robot/ConfigRegistry.cpp`: `CFG_F("ekfRHead", ekfROtosTheta)`
      changed to `CFG_F_SS("ekfRHead", ekfROtosTheta, "drive")`, routing
      through the existing `driveChanged` → `Drive::configure()` path. No
      other registry row changes.
- [ ] `SET ekfRHead=<x>` changes how strongly a subsequent OTOS heading
      disagreement is corrected (observable via a deliberately-injected
      heading disagreement in sim).
- [ ] Immediately after `SET ekfRHead=<x>`, the fused pose/velocity read
      back identically to their pre-SET values — no reset-to-origin
      regression from reusing `init()`'s state-resetting path. (Verify with
      a robot that has driven to a non-trivial, non-origin pose before the
      SET.)
- [ ] Full default sim/unit test suite green.

## Testing

- **Existing tests to run**: any existing EKF/OTOS-fusion sim tests; full
  default suite via `uv run python -m pytest`.
- **New tests to write**:
  - A sim test that drives the robot to a non-trivial pose, sends
    `SET ekfRHead=<x>`, and asserts the fused pose/velocity are unchanged
    immediately after the SET (proves no state/covariance reset occurred).
  - A sim test that injects a deliberate OTOS heading disagreement, SETs
    `ekfRHead` to a different value, and confirms the correction weighting
    changes (proves the noise update is actually live).
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Add a noise-only sibling to `EKFTiny::init()` that touches
only the noise-related internal state, and thread it through the two
forwarding layers (`Odometry`, `PhysicalStateEstimate`) up to
`Drive::configure()`, which already fires on every `"drive"`-annotated
`SET` and already holds a live `_robCfg` reference. This mirrors the
project's existing idiom for "update a subset of a subsystem's internal
state without disturbing the rest" — `MotorController::updateVelGains()`
is the precedent (pushes gains without touching the PID's running
integrator state).

**Files to modify**:
- `source/state/EKFTiny.h`/`.cpp` — new `setNoise(...)` method.
- `source/control/Odometry.h`/`.cpp` — new `setNoise(...)` forwarding
  method; refresh cached `_rOtosTheta`.
- `source/state/PhysicalStateEstimate.h`/`.cpp` — new `setNoise(...)`
  forwarding method.
- `source/subsystems/drive/Drive.cpp` — `configure()` gains the
  `_est.setNoise(...)` call.
- `source/robot/ConfigRegistry.cpp` — `ekfRHead`'s registry row:
  `CFG_F` → `CFG_F_SS(..., "drive")`.

**Testing plan**:
- New sim test: drive to a non-origin pose, `SET ekfRHead=<x>`, assert
  fused pose/velocity unchanged immediately after (no reset).
- New sim test: inject an OTOS heading disagreement, vary `ekfRHead`
  between two values, and confirm the correction magnitude differs.
- Run the full default suite (`uv run python -m pytest`) and confirm no
  regressions to existing EKF/OTOS fusion tests.

**Documentation updates**: none — `architecture-update.md` already
documents this change in full (Step 4-5 item 3, Design Rationale
Decision 3). No wire-protocol change (the `ekfRHead` key itself is
unchanged; only its internal registry annotation changes, which is
invisible on the wire). No `RobotConfig` schema change.
