---
id: '001'
title: Create PhysicalStateEstimate wrapping Odometry+EKF; move EKF to source/state/
status: open
use-cases:
- SUC-001
- SUC-002
- SUC-005
depends-on: []
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Create PhysicalStateEstimate wrapping Odometry+EKF; move EKF to source/state/

## Description

Introduce the `PhysicalStateEstimate` class as a thin composition wrapper around
the existing `Odometry` object (which in turn owns `EKF`). Simultaneously move
`EKF.{h,cpp}` from `source/control/` to `source/state/` so the state-estimation
modules share a directory. Leave all call sites pointing at `robot.odometry.*` for
now — repointing happens in T3.

This ticket is the scaffolding step: after it, `PhysicalStateEstimate` exists and
compiles, `EKF` lives in `source/state/`, and the sim + firmware build are both
green. The `Robot` struct continues to expose `odometry` as its member name
(renaming to `estimate` happens in T3 to keep each ticket green at the call sites).

### Step-by-step plan

**1. Move EKF**

- `git mv source/control/EKF.h source/state/EKF.h`
- `git mv source/control/EKF.cpp source/state/EKF.cpp`
- Create shim at `source/control/EKF.h`:
  ```cpp
  // Phase C migration shim — EKF moved to source/state/. Delete in Phase F.
  #pragma once
  #include "../state/EKF.h"
  ```
- `source/control/EKF.cpp` is DELETED (not shimmed); `source/state/EKF.cpp`
  is the compiled file.

**2. Update CMake source lists**

- `tests/_infra/sim/CMakeLists.txt`: add a glob for `source/state/`:
  ```cmake
  file(GLOB STATE_SOURCES "${REPO_ROOT}/source/state/*.cpp")
  ```
  and append `${STATE_SOURCES}` to the source list (next to CONTROL_SOURCES etc.).
- Firmware `build.py` (or `CMakeLists.txt` for ARM): similarly add `source/state/`
  to the firmware source glob.
- Confirm that `source/control/*.cpp` glob no longer picks up `EKF.cpp` (it won't —
  the file is deleted from that directory).

**3. Create source/state/PhysicalStateEstimate.h**

```cpp
#pragma once
#include <stdint.h>
#include "Odometry.h"         // pulls in EKF.h, RobotState.h transitively

// PhysicalStateEstimate — the single fused-belief object for the robot's
// physical state (Phase C, Sprint 041). Wraps Odometry by composition.
//
// Observations in: addOdometryObservation, addOtosObservation, resetPose.
// Belief out:      getPose, getVelocity.
//
// HardwareState back-compat: each observation method mirrors the fused pose
// back into HardwareState fields (poseX/Y/poseHrad/fusedV/fusedOmega) so
// existing readers (buildTlmFrame, getPoseFloat) work unchanged until Phase F.
//
// Dependency rule: this header includes no CommandTypes.h, Commandable,
// MicroBit.h, or Protocol.h. (Odometry.h still pulls CommandTypes.h until T2
// strips Commandable from Odometry — the grep-gate baseline update is timed
// to T2, not here.)
class PhysicalStateEstimate {
public:
    PhysicalStateEstimate();

    // --- Observations in ---

    // Encoder dead-reckoning + EKF predict (= Odometry::predict, verbatim).
    void addOdometryObservation(HardwareState& s, float trackwidthMm,
                                float rotationalSlip, uint32_t now_ms);

    // OTOS EKF correction (= Odometry::correctEKF, verbatim).
    void addOtosObservation(HardwareState& s,
                            float x_otos, float y_otos,
                            float theta_otos_rad,
                            float v_otos_mmps, float omega_otos_rads);

    // External camera re-anchor / SI verb (= Odometry::setPose, verbatim).
    void resetPose(HardwareState& s,
                   int32_t x_mm, int32_t y_mm, int32_t h_cdeg);

    // --- Belief out ---

    // Read current fused pose (integer mm + centidegrees).
    static void getPose(const HardwareState& s,
                        int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg);

    // Read fused velocity (mm/s, rad/s) from HardwareState back-compat fields.
    static void getVelocity(const HardwareState& s,
                            float& v_mmps, float& omega_rads);

    // --- Initialisation / wiring ---
    void initEKF(float q_xy, float q_theta, float q_v, float q_omega,
                 float r_otos_xy, float r_otos_v, float r_enc_v,
                 float r_otos_theta);

    // Bind IOdometer* and HardwareState* for the OTOS command context.
    // (Passed through to _odometry.setCtx(); also stored for OtosCommands
    // wiring in T2.)
    void setCtx(IOdometer* otos, const HardwareState* hwState = nullptr);

    // --- Forwarded accessors (used by RobotTelemetry, LoopTickOnce, etc.) ---
    uint32_t otosRejectedCount() const;
    int      ekfRejectCount()    const;
    float    ekfPDiag(int idx)   const;
    float    lastEncV()          const;
    float    lastEncOmega()      const;

    bool     encOmegaHealthy()        const;
    void     setEncOmegaHealthy(bool healthy);

    bool     wedgeActive()            const;
    void     setWedgeActive(bool active);

    void     rebaselinePrev(float encL, float encR);

    // --- Access to the wrapped Odometry (for OtosCommands context in T2) ---
    Odometry& odometry() { return _odometry; }

private:
    Odometry _odometry;
};
```

**4. Create source/state/PhysicalStateEstimate.cpp**

Each method is a one-line delegation to `_odometry`:

```cpp
#include "PhysicalStateEstimate.h"

PhysicalStateEstimate::PhysicalStateEstimate() {}

void PhysicalStateEstimate::addOdometryObservation(
        HardwareState& s, float trackwidthMm,
        float rotationalSlip, uint32_t now_ms) {
    _odometry.predict(s, trackwidthMm, rotationalSlip, now_ms);
}

void PhysicalStateEstimate::addOtosObservation(
        HardwareState& s,
        float x_otos, float y_otos, float theta_otos_rad,
        float v_otos_mmps, float omega_otos_rads) {
    _odometry.correctEKF(s, x_otos, y_otos, theta_otos_rad,
                         v_otos_mmps, omega_otos_rads);
}

void PhysicalStateEstimate::resetPose(
        HardwareState& s, int32_t x_mm, int32_t y_mm, int32_t h_cdeg) {
    _odometry.setPose(s, x_mm, y_mm, h_cdeg);
}

void PhysicalStateEstimate::getPose(const HardwareState& s,
        int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg) {
    Odometry::getPose(s, x_mm, y_mm, h_cdeg);
}

void PhysicalStateEstimate::getVelocity(const HardwareState& s,
        float& v_mmps, float& omega_rads) {
    v_mmps      = s.fusedV;
    omega_rads  = s.fusedOmega;
}

void PhysicalStateEstimate::initEKF(
        float q_xy, float q_theta, float q_v, float q_omega,
        float r_otos_xy, float r_otos_v, float r_enc_v, float r_otos_theta) {
    _odometry.initEKF(q_xy, q_theta, q_v, q_omega,
                      r_otos_xy, r_otos_v, r_enc_v, r_otos_theta);
}

void PhysicalStateEstimate::setCtx(IOdometer* otos,
                                   const HardwareState* hwState) {
    _odometry.setCtx(otos, hwState);
}

uint32_t PhysicalStateEstimate::otosRejectedCount() const {
    return _odometry.otosRejectedCount();
}
int PhysicalStateEstimate::ekfRejectCount() const {
    return _odometry.ekfRejectCount();
}
float PhysicalStateEstimate::ekfPDiag(int idx) const {
    return _odometry.ekfPDiag(idx);
}
float PhysicalStateEstimate::lastEncV() const {
    return _odometry.lastEncV();
}
float PhysicalStateEstimate::lastEncOmega() const {
    return _odometry.lastEncOmega();
}
bool PhysicalStateEstimate::encOmegaHealthy() const {
    return _odometry.encOmegaHealthy();
}
void PhysicalStateEstimate::setEncOmegaHealthy(bool healthy) {
    _odometry.setEncOmegaHealthy(healthy);
}
bool PhysicalStateEstimate::wedgeActive() const {
    return _odometry.wedgeActive();
}
void PhysicalStateEstimate::setWedgeActive(bool active) {
    _odometry.setWedgeActive(active);
}
void PhysicalStateEstimate::rebaselinePrev(float encL, float encR) {
    _odometry.rebaselinePrev(encL, encR);
}
```

**5. Audit all robot.odometry.* external call sites**

Before declaring done, grep for `robot.odometry\|robot->odometry\|\.odometry\.` in:
- `source/control/LoopTickOnce.cpp`
- `source/robot/Robot.cpp`
- `source/robot/RobotTelemetry.cpp`
- `source/app/SystemCommands.cpp`
- `tests/_infra/sim/sim_api.cpp`

For each accessor found, confirm it is present in the forwarding list above. Add
any missing forwarders. (This is OQ-2 from the architecture doc — do it now.)

**6. Compile-verify (do NOT wire into Robot yet)**

`PhysicalStateEstimate` compiles in isolation. The existing test suite passes
unchanged because `Robot` still uses `odometry` directly — no call sites changed.

Verify:
```
python3 build.py --fw-only   # ARM gate
uv run --with pytest python -m pytest -q
```

Golden-TLM, field-pin, vendor-confinement all pass because no runtime paths changed.

## Acceptance Criteria

- [ ] `source/state/EKF.h` and `source/state/EKF.cpp` exist with verbatim content.
- [ ] `source/control/EKF.h` is a shim (`#include "../state/EKF.h"`); `source/control/EKF.cpp` is deleted.
- [ ] `source/state/PhysicalStateEstimate.{h,cpp}` exist and compile cleanly.
- [ ] `source/state/PhysicalStateEstimate.h` includes no `CommandTypes.h`, `Commandable`, `MicroBit.h`, or `Protocol.h` directly. (Transitive pull via `Odometry.h` is acceptable until T2.)
- [ ] All forwarded accessors present; external `robot.odometry.*` call-site audit complete.
- [ ] CMake source list for sim and firmware includes `source/state/`.
- [ ] `python3 build.py --fw-only` → 0 errors; then `git checkout -- source/robot/DefaultConfig.cpp`.
- [ ] `uv run --with pytest python -m pytest -q` → ≥ 1997 passed, 0 errors.
- [ ] Golden-TLM canary byte-exact; field-pin diff empty; vendor-confinement gate passes.

## Testing

- **Existing tests to run**: full simulation tier — `uv run --with pytest python -m pytest -q`
- **Behavior-preservation fences** (must stay green untouched): `test_ekf*.py`, `test_otos_fusion.py`, `test_estimator_isolation.py`, `test_estimator_command_paths.py`, `test_observation_models.py`, `test_incident_scenarios.py`, `test_watchdog_exemption.py`
- **New tests to write**: none — this ticket only adds files that are not yet wired into runtime paths; the suite already covers all estimator behavior
- **Verification command**: `uv run --with pytest python -m pytest -q` (≥ 1997 passed, 0 errors)
