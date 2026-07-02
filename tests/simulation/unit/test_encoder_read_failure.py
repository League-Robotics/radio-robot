"""
test_encoder_read_failure.py — 064-005 regression tests.

CR-03 (clasi/issues/encoder-integrity-i2c-failures-and-outlier-filter-recovery.md):
Motor::collectEncoder()/readEncoderAtomic()/readEncoderMmFSettle()/
requestEncoder() (source/hal/real/Motor.cpp) never checked the I2C write/read
return codes; on failure the response buffer stayed {0,0,0,0}, so the computed
position became `0 - _encOffset` — a jump to a large, arbitrary value. The fix
holds the last known-good value (Motor::_lastGoodRawEnc) instead.

Motor.cpp is excluded from HOST_BUILD (MicroBit.h dependency), so its new
I2C-status-check lines are not directly reachable from pytest — see the
ticket's "Known testability gap" note. What IS testable here is the
**consuming pipeline's** contract: SimMotor gains a parallel
setReadFailure(bool) fault-injection model (mirroring SimOdometer::
setReadFailure / sim_set_otos_read_failure); while injected, SimMotor holds
its last cached position instead of promoting a fresh plant read — exactly
the behaviour the real Motor's fix produces. These tests drive the full
pipeline (Drive::_runOutlierFilter -> MotorController::controlTick ->
Odometry/EKF) through an injected failure and assert the fused pose does not
jump, matching the issue's own stated acceptance criterion.

Note on interaction with ticket 064-004 (wedge detector): a held-last value
during the failure window looks "frozen" to the per-wheel wedge counter —
that is CORRECT, not a bug. A genuinely failing I2C bus SHOULD read as
wedge-suspect rather than fabricate motion; these tests do not assert
anything about the wedge detector, only about the position/pose pipeline.
"""
import ctypes
import math

import pytest

# Drive wheel side indices for sim_set_motor_read_failure (matches SimMotor::
# Side and the sim_set_motor_slip/sim_set_encoder_noise side convention:
# 0=left, 1=right, other=both).
SIDE_LEFT = 0
SIDE_RIGHT = 1
SIDE_BOTH = 2


def _enc(sim) -> tuple[float, float]:
    """Return (encL, encR) — Drive's outlier-filtered encoder cache (mm)."""
    l = float(sim._lib.sim_get_enc_l(sim._h))
    r = float(sim._lib.sim_get_enc_r(sim._h))
    return (l, r)


# ---------------------------------------------------------------------------
# Held-last-value: no fabricated pose jump during an injected failure
# ---------------------------------------------------------------------------

def test_encoder_read_failure_holds_encoder_no_pose_jump(sim):
    """Injecting a read failure on one wheel must hold that wheel's filtered
    encoder reading constant (not fabricate a jump), and the fused pose must
    not jump beyond ordinary per-tick travel while the failure is active."""
    sim.set_perfect()

    sim.send_command("VW 200 0")           # straight forward
    sim.tick_for(600, step_ms=24)          # get up to speed, establish baseline

    enc_l0, enc_r0 = _enc(sim)
    px, py, _ = sim.get_pose()

    sim.set_motor_read_failure(SIDE_LEFT, True)

    max_step = 0.0
    for _ in range(5):                     # 5 ticks (~120 ms) under failure
        sim.tick_for(24, step_ms=24)
        nx, ny, _ = sim.get_pose()
        step = math.hypot(nx - px, ny - py)
        max_step = max(max_step, step)
        px, py = nx, ny

    enc_l1, enc_r1 = _enc(sim)

    # The failed (left) wheel's filtered reading is held — not fabricated.
    assert enc_l1 == pytest.approx(enc_l0, abs=0.5), (
        f"left encoder advanced {enc_l0:.2f}->{enc_l1:.2f} mm during an "
        f"injected read failure — should hold the last known-good value."
    )
    # The healthy (right) wheel keeps advancing normally — the failure is
    # scoped to the one wheel, not the whole pipeline.
    assert (enc_r1 - enc_r0) > 5.0, (
        f"right encoder barely moved ({enc_r0:.2f}->{enc_r1:.2f}) while only "
        f"the left wheel's read was failing."
    )
    # No single-tick pose step is anywhere near the magnitude a fabricated
    # `0 - _encOffset` jump would produce (the historical bug this ticket
    # fixes) — a generous bound well above ordinary per-tick travel at this
    # speed (~5 mm/tick) but far below a fabricated-offset jump.
    assert max_step < 50.0, (
        f"pose stepped {max_step:.1f} mm in a single tick during the "
        f"injected read failure — held-last-value must not fabricate a jump."
    )

    sim.set_motor_read_failure(SIDE_LEFT, False)


def test_encoder_read_failure_clear_resumes_tracking(sim):
    """After clearing the read failure, the failed wheel's filtered encoder
    resumes advancing and the fused pose keeps tracking the commanded move."""
    sim.set_perfect()

    sim.send_command("VW 200 0")
    sim.tick_for(600, step_ms=24)

    sim.set_motor_read_failure(SIDE_LEFT, True)
    sim.tick_for(120, step_ms=24)          # 5 ticks under failure

    enc_l_during, _ = _enc(sim)
    x_during, y_during, _ = sim.get_pose()

    sim.set_motor_read_failure(SIDE_LEFT, False)
    sim.tick_for(600, step_ms=24)          # give it time to resume tracking

    enc_l_after, _ = _enc(sim)
    x_after, y_after, _ = sim.get_pose()

    assert (enc_l_after - enc_l_during) > 20.0, (
        f"left encoder did not resume advancing after clearing the read "
        f"failure: {enc_l_during:.2f} -> {enc_l_after:.2f} mm"
    )
    assert math.hypot(x_after - x_during, y_after - y_during) > 20.0, (
        f"fused pose did not resume advancing after clearing the read "
        f"failure: ({x_during:.1f},{y_during:.1f}) -> "
        f"({x_after:.1f},{y_after:.1f})"
    )


# ---------------------------------------------------------------------------
# C-ABI side convention: 0=left, 1=right, other=both
# ---------------------------------------------------------------------------

def test_motor_read_failure_side_convention(sim):
    """sim_set_motor_read_failure's side param: 0=left, 1=right, other=both —
    matching sim_set_motor_slip/sim_set_encoder_noise."""
    sim.set_perfect()
    sim.send_command("VW 200 0")
    sim.tick_for(240, step_ms=24)

    # side=0: left only fails; right keeps advancing.
    sim.set_motor_read_failure(SIDE_LEFT, True)
    l0, r0 = _enc(sim)
    sim.tick_for(120, step_ms=24)
    l1, r1 = _enc(sim)
    assert l1 == pytest.approx(l0, abs=0.5), "left held during side=0 failure"
    assert (r1 - r0) > 5.0, "right kept advancing during side=0 (left-only) failure"
    sim.set_motor_read_failure(SIDE_LEFT, False)
    sim.tick_for(24, step_ms=24)

    # side=1: right only fails; left keeps advancing.
    sim.set_motor_read_failure(SIDE_RIGHT, True)
    l0, r0 = _enc(sim)
    sim.tick_for(120, step_ms=24)
    l1, r1 = _enc(sim)
    assert r1 == pytest.approx(r0, abs=0.5), "right held during side=1 failure"
    assert (l1 - l0) > 5.0, "left kept advancing during side=1 (right-only) failure"
    sim.set_motor_read_failure(SIDE_RIGHT, False)
    sim.tick_for(24, step_ms=24)

    # side=2 ("other"): both fail.
    sim.set_motor_read_failure(SIDE_BOTH, True)
    l0, r0 = _enc(sim)
    sim.tick_for(120, step_ms=24)
    l1, r1 = _enc(sim)
    assert l1 == pytest.approx(l0, abs=0.5), "left held during side=2 (both) failure"
    assert r1 == pytest.approx(r0, abs=0.5), "right held during side=2 (both) failure"
    sim.set_motor_read_failure(SIDE_BOTH, False)
