"""
test_067_002_drive_drvcfg_shadow_ekf_predict.py — Regression tests for ticket
067-002: Drive's `_drvCfg` shadow-cache for trackwidth and OTOS lag.

Background (see clasi/sprints/067-set-to-planner-config-propagation-fix/
architecture-update.md Step 4-5 item 2 and Design Rationale Decision 2):

  `Drive::tickUpdate()`'s STEP 4 (EKF predict) and STEP 5 (OTOS correction)
  read `trackwidthMm`/`lagOtosMs` through a ternary that *intends* a live
  fallback to `_robCfg` (the live `RobotConfig` reference):

      _drvCfg.get_trackwidth() > 0.0f ? _drvCfg.get_trackwidth() : _robCfg.trackwidthMm
      _drvCfg.get_lag_otos() > 0      ? _drvCfg.get_lag_otos()   : _robCfg.lagOtosMs

  `_drvCfg` (a `msg::DrivetrainConfig` snapshot) is only refreshed when a
  `"drive"`-annotated key is SET. Neither `tw` nor `lag.otos` is
  `"drive"`-annotated, so once boot's initial `configure()` call populates a
  positive `_drvCfg` value, the `>0.0f`/`>0` fallback guard can never fire
  again -- `SET tw=<x>` / `SET lag.otos=<x>` alone never reach these two read
  sites, even though the very next line already reads `rotationalSlip`
  directly and correctly from the same live `_robCfg`.

  Fix: both ternaries now read `_robCfg.trackwidthMm` / `_robCfg.lagOtosMs`
  directly, matching their `rotationalSlip` neighbor. `_drvCfg` itself is
  unchanged and still used elsewhere (e.g. `drivetrain_type`).

Isolation strategy
-------------------
Both tests inject raw plant/sensor state directly (`sim_set_enc_l/r`,
`sim_set_otos_pose`) rather than driving through Planner/BVC motion commands.
This bypasses Drive::tickAction()'s *separate*, still-`_drvCfg`-shadowed
TWIST inverse-kinematics read site (out of this ticket's scope -- see
architecture-update.md's audit, which names only the tickUpdate() EKF-predict
site) and isolates exactly the two read sites this ticket fixes.
"""
import pytest

from firmware import Sim


def _ekf_predict_heading(tw_value: int, enc_r_mm: float = 40.0) -> float:
    """Isolate tickUpdate() STEP 4: inject an encoder differential directly
    Injection magnitude: 40 mm in one 24 ms tick (~1667 mm/s) — deliberately
    below Odometry::predict's per-tick physical-step clamp (2000 mm/s, the
    bench-wedge release-jump defense, 2026-07-03); the previous 200 mm
    single-tick injection is now correctly rejected as unphysical.
    into the plant (bypassing all kinematics), tick once, and return the
    resulting fused heading.

    dTheta = ((dR - dL) / trackwidthMm) * effectiveSlip(rotationalSlip)

    With dR-dL and rotationalSlip held fixed across measurements,
    heading * trackwidth is invariant if (and only if) the live SET value is
    genuinely read each tick.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        reply = s.send_command(f"SET tw={tw_value}")
        assert "OK" in reply.upper(), f"SET tw={tw_value} -> unexpected reply {reply!r}"

        s._lib.sim_set_enc_l(s._h, 0.0)
        s._lib.sim_set_enc_r(s._h, enc_r_mm)
        s.tick_for(24, step_ms=24)
        return float(s._lib.sim_get_pose_h(s._h))


def test_set_tw_alone_changes_ekf_predict_trackwidth():
    """SET tw=<x> alone (not bundled with any "drive"-annotated key) must
    change the trackwidth Drive's EKF-predict step (tickUpdate STEP 4) uses
    on the very next tick.

    Pre-067-002: both headings below would be identical (frozen at the
    boot-default trackwidth, 128mm) regardless of the SET tw value.
    """
    h_small = _ekf_predict_heading(64)
    h_large = _ekf_predict_heading(256)

    assert h_small != pytest.approx(h_large, rel=0.01), (
        f"fused heading must differ between tw=64 ({h_small:.4f} rad) and "
        f"tw=256 ({h_large:.4f} rad) -- if both fell back to a stale/boot "
        f"value, they would be identical (the pre-067-002 bug)."
    )
    # heading * trackwidth == (dR-dL) * effectiveSlip(rotationalSlip), a
    # constant independent of tw -- proof the *live* tw value is used, not
    # just "some" value that happens to differ.
    assert h_small * 64 == pytest.approx(h_large * 256, rel=0.02), (
        f"heading*trackwidth should be invariant across tw values if Drive "
        f"reads the live tw on each tick: 64*{h_small:.4f}={h_small * 64:.2f} "
        f"vs 256*{h_large:.4f}={h_large * 256:.2f}"
    )


def _otos_fusion_pulled_x(lag_value: int, injected_x: float = 500.0,
                          window_ms: int = 1000) -> float:
    """Isolate tickUpdate() STEP 5: initialize OTOS, inject a distinct OTOS
    pose, and return the fused x after a fixed tick window.

    Wheel encoders are left untouched (both 0), so pure dead-reckoning holds
    fused x at 0 -- any nonzero fused x after the window can only have come
    from the OTOS-lag-gated correction firing.
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s._lib.sim_begin_otos(s._h)
        reply = s.send_command(f"SET lag.otos={lag_value}")
        assert "OK" in reply.upper(), f"SET lag.otos={lag_value} -> unexpected reply {reply!r}"

        s._lib.sim_set_otos_pose(s._h, injected_x, 0.0, 0.0)
        s.tick_for(window_ms, step_ms=24)
        return float(s._lib.sim_get_pose_x(s._h))


def test_set_lag_otos_alone_changes_otos_fusion_gate():
    """SET lag.otos=<x> alone must change the OTOS-lag compensation Drive's
    EKF-predict step (tickUpdate STEP 5) uses on the very next tick.

    Pre-067-002: lag.otos=5000 could not suppress fusion (frozen at the
    boot-default 10ms) -- OTOS fusion fired almost immediately regardless of
    the SET value, so fused_x_long_lag below would equal ~500 instead of ~0.
    """
    fused_x_short_lag = _otos_fusion_pulled_x(lag_value=5)
    fused_x_long_lag = _otos_fusion_pulled_x(lag_value=5000)

    assert fused_x_short_lag == pytest.approx(500.0, abs=5.0), (
        f"lag.otos=5 should let OTOS fusion pull fused x to ~500mm within "
        f"a 1s window, got {fused_x_short_lag:.2f}"
    )
    assert fused_x_long_lag == pytest.approx(0.0, abs=5.0), (
        f"lag.otos=5000 should suppress OTOS fusion for the full 1s window "
        f"(fused x should stay ~0, pure dead-reckoning with no wheel "
        f"motion), got {fused_x_long_lag:.2f}"
    )
