---
id: '005'
title: 'Fusion-validation test: assert encoder and fused estimates diverge under OTOS
  offset'
status: done
use-cases:
- SUC-047-001
- SUC-047-004
- SUC-047-005
depends-on:
- '004'
github-issue: ''
issue: robot-state-object-proposed-structure-for-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fusion-validation test: assert encoder and fused estimates diverge under OTOS offset

## Description

Add a new pytest in `tests/simulation/unit/test_fusion_validation.py` that
proves the encoder-only dead-reckoning estimate is genuinely independent of
EKF fusion. The test drives the sim straight, injects an OTOS position offset,
enables fusion for several ticks, then asserts that:

- `sim_get_enc_pose_x/y/h()` (encoder path) has NOT been pulled toward the
  OTOS offset.
- `sim_get_fused_pose_x/y/h()` (EKF fused path) HAS been influenced by the
  OTOS correction.
- `sim_get_otos_pose_x/y/h()` (raw optical path) reflects the injected offset.

This directly validates the headline feature of Sprint 047: that all three
estimates are retained side by side and the encoder estimate is never
overwritten by fusion.

A second test verifies the `EST` dump output via `sim_command(h, "DBG EST", ...)`:
confirms that all three `EST enc/otos/fuse` lines appear in the reply with
non-empty values.

## Files to Create

- `tests/simulation/unit/test_fusion_validation.py` — new pytest module

## Files to Modify

- `tests/simulation/unit/firmware.py` (or `tests/_infra/sim/firmware.py`) — add Python
  ctypes bindings for the new ABI functions added in ticket 002:
  ```python
  lib.sim_get_enc_pose_x.restype   = ctypes.c_float
  lib.sim_get_enc_pose_y.restype   = ctypes.c_float
  lib.sim_get_enc_pose_h.restype   = ctypes.c_float
  lib.sim_get_otos_pose_x.restype  = ctypes.c_float
  lib.sim_get_otos_pose_y.restype  = ctypes.c_float
  lib.sim_get_otos_pose_h.restype  = ctypes.c_float
  lib.sim_get_fused_pose_x.restype = ctypes.c_float
  lib.sim_get_fused_pose_y.restype = ctypes.c_float
  lib.sim_get_fused_pose_h.restype = ctypes.c_float
  ```
  And add corresponding wrapper methods to the `Sim` class (e.g. `enc_pose_x()`,
  `otos_pose_x()`, `fused_pose_x()`, etc.).

## Test Design

```python
# tests/simulation/unit/test_fusion_validation.py

def test_encoder_not_overwritten_by_fusion(sim):
    """Encoder estimate must not be pulled toward OTOS offset after fusion."""
    # 1. Drive straight for 50 ticks with no OTOS — encoder integrates cleanly.
    for t in range(50):
        sim.tick(t * 10)

    enc_x_before = sim.enc_pose_x()

    # 2. Inject a large OTOS X offset (200 mm) without enabling fusion.
    sim.set_otos_pose(enc_x_before + 200.0, 0.0, 0.0)

    # 3. Enable OTOS fusion and tick for 20 more steps.
    sim.set_otos_fusion(True)
    for t in range(50, 70):
        sim.tick(t * 10)

    enc_x_after  = sim.enc_pose_x()
    fused_x_after = sim.fused_pose_x()
    otos_x_after  = sim.otos_pose_x()

    # Encoder must NOT have moved toward the OTOS offset.
    assert abs(enc_x_after - enc_x_before) < 5.0, (
        f"Encoder pose was corrupted by fusion: before={enc_x_before:.1f}, after={enc_x_after:.1f}"
    )

    # Fused pose MUST have been pulled toward the OTOS offset.
    assert fused_x_after > enc_x_before + 50.0, (
        f"Fused pose not influenced by OTOS: fused={fused_x_after:.1f}, enc={enc_x_before:.1f}"
    )

    # OTOS pose must reflect the injected offset.
    assert abs(otos_x_after - (enc_x_before + 200.0)) < 10.0, (
        f"OTOS pose not reflecting injection: otos={otos_x_after:.1f}"
    )

    # All three must be distinct.
    assert enc_x_after != fused_x_after, "Encoder and fused must differ after OTOS injection"
    assert enc_x_after != otos_x_after,  "Encoder and optical must differ after OTOS injection"


def test_est_dump_emits_three_lines(sim):
    """DBG EST command must emit three EST lines covering enc, otos, fuse."""
    reply = sim.command("DBG EST")
    assert "EST enc"  in reply, f"Missing EST enc line in: {reply!r}"
    assert "EST otos" in reply, f"Missing EST otos line in: {reply!r}"
    assert "EST fuse" in reply, f"Missing EST fuse line in: {reply!r}"
    # Each line must contain the key fields.
    for label in ("enc", "otos", "fuse"):
        line = next(l for l in reply.splitlines() if f"EST {label}" in l)
        assert "x=" in line and "y=" in line and "h=" in line, \
            f"EST {label} line missing pose fields: {line!r}"
        assert "age=" in line and "v=" in line, \
            f"EST {label} line missing freshness fields: {line!r}"
```

## Acceptance Criteria

- [x] `tests/simulation/unit/test_fusion_validation.py` exists and is collected by pytest.
- [x] `test_encoder_not_overwritten_by_fusion` passes: encoder pose is not pulled toward the OTOS-injected offset after fusion runs.
- [x] `test_est_dump_emits_three_lines` passes: `DBG EST` reply contains all three `EST enc/otos/fuse` lines with `x=`, `y=`, `h=`, `age=`, `v=` fields.
- [x] All three `sim_get_*_pose_*` ABI wrapper methods are added to the `Sim` Python class (or equivalent ctypes binding).
- [x] **Full sim suite green**: `uv run --with pytest python -m pytest tests/simulation/ -q` — 2230 passed, 2 known pre-existing failures only.
- [x] **Differential build compiles clean** (`python build.py --clean`): zero errors.
- [x] **Mecanum build compiles clean**: `cmake -S tests/_infra/sim -B tests/_infra/sim/build_mecanum -DROBOT_DRIVETRAIN=mecanum && cmake --build tests/_infra/sim/build_mecanum`: zero errors.

### Implementation Notes

**Real harness API used**: `Sim` class from `tests/_infra/sim/firmware.py`; fixture `sim` from `tests/conftest.py`.
New wrapper methods added: `get_enc_pose()`, `get_optical_pose()`, `get_fused_pose()` (each returns `(x_mm, y_mm, h_rad)` tuple).

**Scenario for `test_encoder_not_overwritten_by_fusion`**:
- Drive `VW 200 0` for 2000 ms → encoder accumulates ~200 mm in X.
- Stop with `X`, settle 2 ticks.
- Inject OTOS at `enc_x_before + 200 mm` (persistent via `set_otos_pose`).
- Enable fusion (`set_otos_fusion(True)`); tick 25 × 24 ms.
- After 10 consecutive gate rejections the EKF fires P-inflation (K≈1) and fused snaps to OTOS.
- Thresholds: enc drift < 5 mm (stopped robot, any larger drift is a regression); fused pull > 50 mm (actual snap is ~200 mm); optical error < 10 mm.

**Scenario for `test_est_dump_emits_three_lines`**:
- Tick 24 ms to populate estimates, then send `DBG EST`.
- Assert labels `EST enc`, `EST otos`, `EST fuse` present; all 8 fields `x=, y=, h=, vx=, vy=, w=, age=, v=` present on each line.

## Implementation Plan

1. Add ctypes bindings for the nine new ABI functions to `firmware.py` (or wherever the `Sim` ctypes wrappers live). Add convenience wrapper methods to the `Sim` class.
2. Write `test_fusion_validation.py` with the two tests above.
3. Run `uv run --with pytest python -m pytest tests/simulation/unit/test_fusion_validation.py -v` and iterate until both tests pass.
4. Run full sim suite to confirm no regressions.

## Testing Plan

- **This ticket IS the testing**: the two new tests directly validate SUC-047-005 (divergence) and SUC-047-001 (EST dump).
- **Regression gate**: full sim suite must remain green.
- **Build gate**: both firmware variants must compile clean.

## Documentation Updates

Architecture update section F (sim_api ABI) and section C (dump surface) describe the tested functionality. No new architecture docs needed.
