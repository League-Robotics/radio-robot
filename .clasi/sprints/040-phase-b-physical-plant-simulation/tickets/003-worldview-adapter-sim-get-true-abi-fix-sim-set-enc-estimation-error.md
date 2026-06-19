---
id: '003'
title: WorldView adapter, sim_get_true_* ABI, fix sim_set_enc, estimation_error
status: done
use-cases:
- SUC-003
- SUC-004
depends-on:
- 040-002
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# WorldView adapter, sim_get_true_* ABI, fix sim_set_enc, estimation_error

## Description

Create `WorldView` and wire it into `SimHandle`. Add new C ABI entry points for
truth access and estimation error. Fix `sim_set_enc_l/r` (the lying bug). Wire
`sim_get_exact_pose_*` as formal aliases. Add Python wrappers in `firmware.py`.

### WorldView (source/io/sim/WorldView.{h,cpp})

`WorldView` is constructed with `const PhysicsWorld&` and `const HardwareState&`.
It provides:
- `truePoseX()`, `truePoseY()`, `truePoseH()` — reads from `PhysicsWorld`.
- `estimationErrorXY()` — Euclidean distance: `sqrt((truePoseX - s.inputs.poseX)^2 + (truePoseY - s.inputs.poseY)^2)`.
- `estimationErrorH()` — heading error wrapped to `[-π, π]`.

`SimHandle` owns `WorldView _worldView` constructed from `{hal.plant(), robot.state.inputs}`.
`WorldView` holds references (not copies) so it always sees the current state.

### New C ABI entry points in sim_api.cpp

```c
// True pose (plant truth, not EKF estimate)
float sim_get_true_pose_x(void* h);
float sim_get_true_pose_y(void* h);
float sim_get_true_pose_h(void* h);

// Set true pose directly (for isolation tests)
void  sim_set_true_pose(void* h, float x, float y, float h_rad);
void  sim_set_true_wheel_travel(void* h, float enc_l_mm, float enc_r_mm);
void  sim_set_true_velocity(void* h, float vel_l_mms, float vel_r_mms);

// Reset all error layers to no-op (perfect sensors)
void  sim_set_perfect(void* h);

// Estimation error: EKF estimate vs. plant truth
float sim_get_estimation_error_xy(void* h);   // mm
float sim_get_estimation_error_h(void* h);    // rad
```

**Formal alias — update `sim_get_exact_pose_*` implementations:**
Change `sim_get_exact_pose_x/y/h` to call `_worldView.truePoseX/Y/H()` instead
of reading `hal.plant()` directly. The behavior is identical but goes through the
canonical accessor.

**Fix `sim_set_enc_l/r`:**
Currently these functions write to `state.inputs.encL/R` only, which is overwritten
on the next tick (the lying bug). Fix:
```c
void sim_set_enc_l(void* h, float mm) {
    SimHandle* s = static_cast<SimHandle*>(h);
    s->hal.plant().setTrueWheelTravel(mm, s->hal.plant().trueEncRMm());
    s->robot.state.inputs.encLMm = mm;  // keep state in sync
}
void sim_set_enc_r(void* h, float mm) {
    SimHandle* s = static_cast<SimHandle*>(h);
    s->hal.plant().setTrueWheelTravel(s->hal.plant().trueEncLMm(), mm);
    s->robot.state.inputs.encRMm = mm;
}
```

**`sim_set_perfect(void* h)`:** Calls all error-reset methods on the observation
models: `hal.simMotorL().setFrozen(false)`, `hal.simOdometer().setReadFailure(false)`,
`hal.simLineSensor().setFrozen(false)`, `hal.simColorSensor().setFrozen(false)`.
Resets any noise or drift parameters to zero.

### firmware.py additions

Add Python wrappers to `Sim._setup_types()` and public methods:
```python
def get_true_pose(self) -> tuple[float, float, float]:
    """Return (x_mm, y_mm, h_rad) from PhysicsWorld ground truth."""

def set_true_pose(self, x: float, y: float, h_rad: float) -> None:
    """Set ground truth pose directly (bypasses physics integration)."""

def set_true_wheel_travel(self, enc_l_mm: float, enc_r_mm: float) -> None:
    """Set true wheel travel accumulators directly."""

def estimation_error(self) -> tuple[float, float]:
    """Return (xy_mm, h_rad) estimation error: EKF pose vs. plant truth."""

def set_perfect(self) -> None:
    """Reset all observation-model error layers to no-op (perfect sensors)."""
```

## Acceptance Criteria

- [x] `WorldView.h` / `WorldView.cpp` created; `SimHandle` owns `WorldView _worldView`.
- [x] `sim_get_true_pose_x/y/h` return `PhysicsWorld` true pose.
- [x] `sim_set_true_pose` sets `PhysicsWorld` truth directly; next `sim_tick` does not overwrite it.
- [x] `sim_get_exact_pose_x/y/h` now formally alias `WorldView::truePoseX/Y/H()`.
- [x] **`sim_set_enc_l/r` fix:** after `sim_set_enc_l(h, 500.0)` and one `sim_tick`,
      `sim_get_enc_l(h)` still reflects the injected value (not back to 0). Verified: enc_l = 500.000.
- [x] `sim_get_estimation_error_xy()` returns 0.0 when robot has not moved (EKF == truth).
- [x] `sim_set_perfect()` resets all error layers (no observable test change; tested in T5).
- [x] `firmware.py` `Sim` class has `get_true_pose()`, `set_true_pose()`,
      `set_true_wheel_travel()`, `estimation_error()`, `set_perfect()` methods.
- [x] `uv run --with pytest python -m pytest -q` ≥ 1957 passed, 0 errors. Result: 1964 passed.
- [x] All canaries green (golden-TLM, field-pin, vendor grep).

## Implementation Plan

### Approach

1. Create `WorldView.h` / `.cpp`.
2. Add `WorldView _worldView` to `SimHandle` (initialized in constructor).
3. Add new C ABI functions to `sim_api.cpp`.
4. Fix `sim_set_enc_l/r`.
5. Add Python wrappers to `firmware.py`.
6. Run full suite.

### Files to Create

- `source/io/sim/WorldView.h` / `WorldView.cpp`

### Files to Modify

- `tests/_infra/sim/sim_api.cpp` — `SimHandle` struct; new entry points; `sim_set_enc_l/r` fix.
- `tests/_infra/sim/firmware.py` — `_setup_types()` additions; new `Sim` methods.

### Testing Plan

```
uv run --with pytest python -m pytest -q
```

Spot-check the enc-fix manually with a quick Python snippet if needed:
```python
with Sim() as s:
    s.send_command("SET sTimeout=60000")
    s._lib.sim_set_enc_l(s._h, ctypes.c_float(500.0))
    s._lib.sim_tick(s._h, ctypes.c_uint32(24))
    enc = s._lib.sim_get_enc_l(s._h)
    assert abs(enc - 500.0) < 10.0, f"enc_l = {enc}"  # ~500 mm after 1 tick at 0 PWM
```

### Documentation Updates

None required — internal sim component.
