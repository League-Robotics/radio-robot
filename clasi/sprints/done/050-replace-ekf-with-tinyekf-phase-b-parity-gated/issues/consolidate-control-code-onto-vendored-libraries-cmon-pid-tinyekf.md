---
status: in-progress
sprint: '050'
tickets:
- 049-001
- 049-002
- 049-003
- 049-004
- 049-005
- 050-001
- 050-002
- 050-003
- 050-004
- 050-005
- 050-006
---

# Consolidate control code onto vendored libraries (cmon-pid + TinyEKF)

## Context

The firmware has accreted multiple hand-rolled feedback/estimation implementations:
two PID-ish controllers (one of them dead code) and a bespoke 5-state EKF. The goal
is to **collapse the PID story to a single controller sourced from a vetted external
library**, and to **replace the hand-rolled EKF with a vetted external library** as
well — trading hand-maintained math for documented, tested, reusable code.

Decisions locked in with the stakeholder:
- **PID** → vendor **cmon-pid** (https://github.com/corraid/cmon-pid, BSD-2-Clause).
- **EKF** → vendor **TinyEKF** (https://github.com/simondlevy/TinyEKF, MIT).
- **Delete** the dead `RatioPidController` **and** its `pid.*` config keys outright.

## What's actually there today

| Item | File | Status |
|---|---|---|
| `VelocityController` (PI + feed-forward, per-wheel) | [source/control/VelocityController.h](source/control/VelocityController.h) | **Live**, 2–4 instances via `MotorController` |
| `RatioPidController` (full P+I+D) | [source/control/RatioPidController.h](source/control/RatioPidController.h) | **Dead** — `update()` never called; only `pid.*` config keys survive |
| 5-state `EKF` (hand-unrolled matrices, no Eigen) | [source/state/EKF.h](source/state/EKF.h) | **Live**, primary fusion path, heavily tested |

Everything else mapped during research (`BodyVelocityController` profiler, differential/
mecanum kinematics, complementary filters, EMA, `StopCondition`/`HaltController`,
wedge detector) is **out of scope** — none are PIDs and none are being replaced.

Hard constraints any vendored code must satisfy: no heap, no STL, no exceptions/RTTI,
`float`-friendly (Cortex-M4F is single-precision; `double` is soft-emulated), must
compile in **both** the ARM firmware build and the host-sim build, and must stay
vendor-confined (no CODAL deps) — enforced by `tests/simulation/unit/test_vendor_confinement.py`.

## Feasibility findings (fact-checked against the actual headers)

**cmon-pid** — header-only C++, BSD-2, no STL/heap/virtuals → fits confinement. Caveats:
- Float type is **hard-coded `double`** throughout. On the M4 FPU this is costly.
  → Vendor our own copy and mechanically convert `double`→`float` in the header
  (it's ~9 KB, header-only; the anti-windup wrappers are already templated).
- **No feedforward**; `Update(error)` takes error only. So cmon-pid replaces the
  **PI/PID core**, not all of `VelocityController`. It exposes `ParallelPid(h, Kp, Ki,
  Kd, Tf)` (direct gains + derivative low-pass) wrapped in `clamping_t<>` or
  `backcalculation_t<>` for anti-windup — a clean match for our gains.

**TinyEKF** — header-only C, MIT, static allocation, `float` by default → fits confinement.
Caveat (the important one): TinyEKF provides **only** the bare predict/update linear
algebra (`ekf_initialize` / `ekf_predict(fx,F,Q)` / `ekf_update(z,hx,H,R)`, struct exposes
`x[]` and `P[]`). It has **no** Mahalanobis gating, **no** P-inflation gate-recovery,
**no** sequential scalar / heterogeneous-channel updates (`EKF_M` is one compile-time
constant; updates are batched). Our EKF's hard-won robustness — χ² gating per channel,
D3 gate-recovery, wedge-aware omega suppression, sequential 1-DOF updates — is exactly
what TinyEKF lacks. So replacement means **keeping our motion model, measurement models,
gating, and recovery as a thin layer on top of TinyEKF's core**, deleting only the
hand-unrolled matrix arithmetic. This is the higher-risk half of the work.

## Plan

Run as **two CLASI sprints** (this is a CLASI repo). Phase A is low-risk and lands first;
Phase B is gated on parity with the existing EKF test suite.

### Phase A — PID consolidation onto cmon-pid

1. **Vendor** cmon-pid into `libraries/cmon-pid/` (`cmon-pid.h` + `LICENSE`), as a
   `float`-typedef variant. Wire its include dir into both build paths: root
   [CMakeLists.txt](CMakeLists.txt) and [tests/_infra/sim/CMakeLists.txt](tests/_infra/sim/CMakeLists.txt).
2. **Refactor** [source/control/VelocityController.cpp](source/control/VelocityController.cpp)
   to compose a cmon-pid controller (`backcalculation_t<pid_bwe>` fed via `ParallelPid`)
   for the integral/derivative/anti-windup core, keeping the thin wrapper that owns
   feed-forward on `|setpoint|`, sign handling, deadband (`minWheelMms`), and the
   ±100 PWM clamp. Map `velKp/velKi/velIMax/velKaw` onto cmon-pid config; `velKff`
   stays in the wrapper.
3. **Delete** [source/control/RatioPidController.h](source/control/RatioPidController.h)
   and `.cpp`, and remove it from the sim build's source list.
4. **Delete the `pid.*` config keys** (`ratioPidKp/Ki/Kd/Max`) from
   [source/types/Config.h](source/types/Config.h) and the SET/GET `pid.*` handling in
   [source/robot/ConfigRegistry.cpp](source/robot/ConfigRegistry.cpp); update/remove
   `tests/simulation/unit/test_ratio_pid.py` and any host tooling that touches `pid.*`.
5. **Validate**: `test_velocity_controller.py`, `test_motor_controller.py`,
   `test_body_velocity_controller.py`, `test_vendor_confinement.py`.

### Phase B — EKF replacement onto TinyEKF (parity-gated)

1. **Vendor** TinyEKF into `libraries/tinyekf/` (`tinyekf.h` + `LICENSE`); set `EKF_N=5`
   and `EKF_M` to the max channel width (2). Wire into both build paths.
2. **Rebuild** the EKF as a layer over `ekf_t`: keep our arc-segment motion model
   (`fx`,`F`), the three update channels (position M=2, heading M=1, velocity M=1×2 done
   as gated per-channel calls), Mahalanobis χ² gating (compute innovation `y` and
   `S = H P Hᵀ + R` ourselves before calling `ekf_update`), D3 gate-recovery via direct
   `P` writes, and wedge omega suppression. Delete only the hand-unrolled matrix internals.
3. **Parity gate**: keep the old EKF in place and make the TinyEKF-backed one pass the
   existing [tests/simulation/unit/test_ekf.py](tests/simulation/unit/test_ekf.py) (large
   suite) at parity. Only then swap [source/state/PhysicalStateEstimate.cpp](source/state/PhysicalStateEstimate.cpp)
   and [source/control/Odometry.cpp](source/control/Odometry.cpp) over and delete the old
   [source/state/EKF.cpp](source/state/EKF.cpp) internals.

## Critical files

- **PID**: `source/control/VelocityController.{h,cpp}`, `RatioPidController.{h,cpp}` (delete),
  `source/types/Config.h`, `source/robot/ConfigRegistry.cpp`,
  `tests/simulation/unit/test_ratio_pid.py`, `test_velocity_controller.py`
- **EKF**: `source/state/EKF.{h,cpp}`, `source/state/PhysicalStateEstimate.{h,cpp}`,
  `source/control/Odometry.{h,cpp}`, `tests/simulation/unit/test_ekf.py`
- **Build / vendor**: `CMakeLists.txt`, `tests/_infra/sim/CMakeLists.txt`, `build.py`,
  `libraries/cmon-pid/`, `libraries/tinyekf/`

## Verification

1. `python build.py --clean` — must build **both** firmware (`build/MICROBIT.hex`) and the
   host sim (`libfirmware_host`) with no source-level `#ifdef` divergence.
2. `pytest tests/simulation/unit` — full suite green, with focus on `test_ekf.py`,
   `test_velocity_controller.py`, `test_motor_controller.py`, `test_body_velocity_controller.py`,
   `test_vendor_confinement.py`.
3. Bench/floor validation per memory notes (clean flash + decode `MICROBIT.hex` to confirm
   the build isn't stale; match `active_robot.json` to the physical bot) before trusting
   on-robot behavior.

## Open implementation decision (not blocking the plan)

cmon-pid `double`→`float`: recommend vendoring the float variant to match the FPU. If a
bench A/B shows the loop-rate cost of `double` is negligible, the unmodified upstream
header can be kept instead (less divergence from upstream).
