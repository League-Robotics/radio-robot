---
id: '001'
title: "PhysicsWorld \u2014 ground-truth plant with canonical midpoint-arc integration"
status: done
use-cases:
- SUC-001
- SUC-005
- SUC-006
depends-on: []
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# PhysicsWorld — ground-truth plant with canonical midpoint-arc integration

## Description

Create `source/io/sim/PhysicsWorld.{h,cpp}` — the single source of ground truth
for the simulated chassis. This ticket is additive: no existing files are changed.
The new class compiles but is not yet wired into `sim_api.cpp` (wiring happens in T2).

`PhysicsWorld` owns:
- True chassis pose: `truePoseX`, `truePoseY`, `truePoseH`.
- True per-wheel travel: `trueEncLMm`, `trueEncRMm`.
- True per-wheel velocity: `trueVelLMms`, `trueVelRMms`.
- True auxiliary sensor values: `lineRaw[4]`, `colorRGBC`, `port[4]` (zero-initialized).
- Dynamics parameters: `trackwidthMm`, `nominalMaxMms`, `rotationalSlip`.
- Per-wheel offset factors: `offsetFactorL`, `offsetFactorR` (default 1.0).

### Two input modes

**Evolve mode:** `setActuators(pwmL, pwmR)` stores the commands; `update(dt_ms)`
advances the chassis one step.

**Truth-injection mode (for isolation tests):**
- `setTruePose(float x, float y, float h)`
- `setTrueWheelTravel(float encL, float encR)`
- `setTrueVelocity(float velL, float velR)`
- `setTrueSensorValues(...)` (line/color/port raw values)
- `reset()` — zeros all state.

### Integration formula — two structurally separate sub-steps

```cpp
void PhysicsWorld::update(uint32_t dt_ms) {
    if (dt_ms == 0) return;
    float dt_s = static_cast<float>(dt_ms) / 1000.0f;

    // Sub-step A: encoder accumulation.
    // CRITICAL: This expression MUST match MockMotor::integrate exactly
    // so the golden-TLM canary passes bit-for-bit. Do NOT refactor.
    float velL = (_pwmL / 100.0f) * _nominalMaxMms * _offsetFactorL;
    float velR = (_pwmR / 100.0f) * _nominalMaxMms * _offsetFactorR;
    _trueEncLMm += velL * dt_s;
    _trueEncRMm += velR * dt_s;
    _trueVelLMms = velL;
    _trueVelRMms = velR;

    // Sub-step B: chassis pose integration (NOT on TLM path; clean formula OK).
    float dL   = velL * dt_s;
    float dR   = velR * dt_s;
    float slip = effectiveSlip(_rotationalSlip);
    float dTh  = ((dR - dL) / _trackwidthMm) * slip;
    float hMid = _truePoseH + dTh * 0.5f;
    _truePoseX += (dL + dR) * 0.5f * cosf(hMid);
    _truePoseY += (dL + dR) * 0.5f * sinf(hMid);
    _truePoseH += dTh;
}
```

`effectiveSlip` is the same free function used by `Odometry::predict`
(defined in `control/Odometry.cpp` or extracted as a header-only inline).
Copy or share as appropriate; do not change the formula.

### Slip configuration

`setSlip(float straight, float turnExtra)` — matches the `MockMotor` API so
`sim_api.cpp` can forward `sim_set_motor_slip` calls here in T2.

`setOffsetFactor(int side, float f)` — 0=left, 1=right, 2=both.

### Read accessors

All const accessors for truth values:
- `truePoseX()`, `truePoseY()`, `truePoseH()`
- `trueEncLMm()`, `trueEncRMm()`
- `trueVelLMms()`, `trueVelRMms()`
- `lineRaw(int ch)`, `colorRGBC()`, `port(int ch)`

## Acceptance Criteria

- [x] `source/io/sim/PhysicsWorld.h` and `PhysicsWorld.cpp` exist and compile
      cleanly in the host sim build (HOST_BUILD, no CODAL dependency).
- [x] `update(dt_ms)` sub-step A formula is identical (bit-for-bit) to
      `MockMotor::integrate` for zero-slip, zero-noise, offset-factor-1.0 inputs.
      Verified by unit test (see Testing below).
- [x] `setTruePose` / `setTrueWheelTravel` / `setTrueVelocity` set their
      respective truth fields directly; `update()` after set does not overwrite them
      (only the actuator path overwrites on the next `update()` call).
- [x] `reset()` zeros all state.
- [x] `setSlip(straight, turnExtra)` and `setOffsetFactor(side, factor)` configure
      the dynamics parameters correctly.
- [x] No heap allocation; all members are value types or primitive arrays.
- [x] Host sim builds clean: `cmake --build tests/_infra/sim/build` succeeds.
- [x] All existing simulation tests pass: `uv run --with pytest python -m pytest -q`
      ≥ 1957 passed, 0 errors. (No sim_api.cpp change yet; existing tests are
      unaffected because PhysicsWorld is not yet wired in.)

## Implementation Plan

### Approach

Pure additive: create two new files. No existing file changes.

### Files to Create

- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/io/sim/PhysicsWorld.h`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/io/sim/PhysicsWorld.cpp`

Add `PhysicsWorld.cpp` to the sim glob in `tests/_infra/sim/CMakeLists.txt` if the
glob does not already pick it up (check: `file(GLOB SIM_SOURCES ".../*.cpp")`).

### Testing Plan

**No existing tests change.** The new class is not yet wired into `sim_api.cpp`.

**Verification test** (write as `tests/simulation/unit/test_physics_world_basic.py`
or inline in a C++ unit test compiled into the sim lib — programmer's choice):

1. Construct `PhysicsWorld` with `trackwidthMm=150, nominalMaxMms=400`.
2. Call `setActuators(50, 50)` (forward at 50% speed) + `update(24)` (24 ms step).
3. Compute expected encoder: `velL = velR = (50/100.0) * 400 = 200 mm/s`;
   `encL = encR = 200 * 0.024 = 4.8 mm`.
4. Assert `trueEncLMm() == 4.8f` within epsilon (`1e-5f`).
5. Assert `truePoseX() > 0` (moved forward) and `truePoseH() ≈ 0` (straight).
6. Assert `truePoseX()` matches the expected arc-integration value within `1e-3f`.

Additionally, run the host sim build + full suite to confirm zero regressions.

### Documentation Updates

None required — internal sim component.
