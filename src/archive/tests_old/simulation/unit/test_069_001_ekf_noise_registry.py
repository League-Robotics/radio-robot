"""test_069_001_ekf_noise_registry.py — seven EKF noise `RobotConfig` fields
gain `SET`/`GET` registry rows (sprint 069, ticket 001; closes 067's own Open
Question 5).

Sprint 067 (ticket 067-003) already built the noise-only `setNoise()` push
all the way from `ConfigRegistry` through `Drive::configure()` to `EKFTiny`
for all eight EKF process/measurement-noise fields, but only wired one of
them (`ekfRHead` -> `ekfROtosTheta`) to a registry row. This file proves the
same two properties `test_067_003_ekf_setnoise.py` proved for `ekfRHead`,
now for the seven newly-registered keys:

  1. test_set_get_roundtrip -- each of the seven keys is SET/GET-able and
     the value round-trips through the wire (proves the registry row exists
     and offsetof() targets the right field).
  2. test_set_does_not_reset_fused_pose -- SET-ting any of the seven keys
     mid-mission must NOT disturb the fused pose/velocity/covariance (proves
     Drive::configure() still routes through the noise-only setNoise() path,
     not a reset-capable init() path, for these keys too).
  3. test_ekfrotosxy_changes_position_correction_strength -- varying
     ekfROtosXy (OTOS position measurement noise) between a low and a high
     value must change how strongly a subsequent OTOS position disagreement
     is corrected (proves the noise update actually reaches EKFTiny's live
     _rOtosXy state consumed by updatePosition(), not just RobotConfig
     storage) -- mirrors 067-003's analogous heading-channel test, applied
     to the position channel via a newly-registered key.

No new C++ behavior is exercised here that 067-003 didn't already build;
this is wire-reachability coverage only (ConfigRegistry.cpp's kRegistry[]
additions).
"""
import pytest

from firmware import Sim


# ---------------------------------------------------------------------------
# The seven newly-registered keys and a non-default test value for each
# (firmware defaults, for reference, from source/types/Config.h:
#   ekfQxy=200.0  ekfQtheta=0.5  ekfROtosXy=50.0
#   ekfQv=5000.0  ekfQomega=1.0  ekfROtosV=200.0  ekfREncV=100.0).
# ---------------------------------------------------------------------------
NEW_NOISE_KEYS = [
    ("ekfQxy", 500.0),
    ("ekfQtheta", 2.0),
    ("ekfQv", 10000.0),
    ("ekfQomega", 3.0),
    ("ekfROtosXy", 150.0),
    ("ekfROtosV", 500.0),
    ("ekfREncV", 300.0),
]


# ---------------------------------------------------------------------------
# Test 1: SET/GET round-trip for all seven keys.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("key,value", NEW_NOISE_KEYS)
def test_set_get_roundtrip(sim, key, value):
    """SET <key>=<value> then GET <key> must round-trip the value.

    Proves the registry row exists (a still-unregistered key would reply
    `ERR badkey`) and offsetof() targets the correct RobotConfig field.
    """
    set_reply = sim.send_command(f"SET {key}={value}")
    assert "OK" in set_reply.upper(), f"SET {key}={value} failed: {set_reply!r}"
    assert "badkey" not in set_reply.lower(), (
        f"SET {key} rejected as unknown key: {set_reply!r}"
    )

    get_reply = sim.send_command(f"GET {key}")
    assert "badkey" not in get_reply.lower(), (
        f"GET {key} rejected as unknown key: {get_reply!r}"
    )
    # Registry values print with %.3f formatting; compare on the formatted value.
    expected = f"{value:.3f}"
    assert expected in get_reply, (
        f"GET {key} did not reflect the SET: expected {expected!r} in {get_reply!r}"
    )


# ---------------------------------------------------------------------------
# Test 2: SET of any of the seven keys does not reset the fused pose/velocity/
# covariance. Mirrors test_067_003_ekf_setnoise.py's
# test_set_ekfrhead_does_not_reset_fused_pose, parametrized across the new keys.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("key,value", NEW_NOISE_KEYS)
def test_set_does_not_reset_fused_pose(sim, key, value):
    """Drive to a non-origin pose, SET one new noise key, and read back
    immediately -- the fused pose, velocity, and full EKF covariance
    diagonal must be bit-for-bit unchanged (no tick elapses between the
    "before" read and the SET), proving Drive::configure()'s EKF-noise push
    does not touch _ekf.x[]/_ekf.P[] for these keys either.
    """
    sim.send_command("SET sTimeout=60000")

    # Drive an asymmetric D so the robot ends at a non-trivial, non-origin
    # pose -- same pattern as 067-003.
    r = sim.send_command("D 100 250 300")
    assert "OK" in r.upper(), f"D command failed: {r!r}"
    sim.tick_for(3000, step_ms=24)

    x_before, y_before, h_before = sim.get_fused_pose()
    v_before = sim.get_fused_v()
    omega_before = sim.get_fused_omega()
    p_before = [sim.get_ekf_p_diag(i) for i in range(5)]

    # Sanity: the robot actually reached a non-trivial pose.
    assert abs(x_before) > 20.0 or abs(y_before) > 20.0 or abs(h_before) > 0.1, (
        f"D command did not reach a non-trivial pose: "
        f"({x_before:.2f}, {y_before:.2f}, {h_before:.4f})"
    )

    set_reply = sim.send_command(f"SET {key}={value}")
    assert "OK" in set_reply.upper(), f"SET {key}={value} failed: {set_reply!r}"

    # Confirm the registry value actually moved (sanity on the SET itself).
    get_reply = sim.send_command(f"GET {key}")
    assert f"{value:.3f}" in get_reply, (
        f"GET {key} did not reflect the SET: {get_reply!r}"
    )

    # Immediately (no further ticks) re-read the fused belief -- must be
    # unchanged.
    x_after, y_after, h_after = sim.get_fused_pose()
    v_after = sim.get_fused_v()
    omega_after = sim.get_fused_omega()
    p_after = [sim.get_ekf_p_diag(i) for i in range(5)]

    assert x_after == x_before, (
        f"fused x reset by SET {key}: {x_before!r} -> {x_after!r}"
    )
    assert y_after == y_before, (
        f"fused y reset by SET {key}: {y_before!r} -> {y_after!r}"
    )
    assert h_after == h_before, (
        f"fused heading reset by SET {key}: {h_before!r} -> {h_after!r}"
    )
    assert v_after == v_before, (
        f"fused v reset by SET {key}: {v_before!r} -> {v_after!r}"
    )
    assert omega_after == omega_before, (
        f"fused omega reset by SET {key}: {omega_before!r} -> {omega_after!r}"
    )
    for i in range(5):
        assert p_after[i] == p_before[i], (
            f"EKF covariance P[{i}][{i}] reset by SET {key}: "
            f"{p_before[i]!r} -> {p_after[i]!r}"
        )

    # Belt-and-suspenders: tick a little further and confirm the fused pose
    # continues from where it was rather than teleporting toward the origin.
    sim.tick_for(200, step_ms=24)
    x_later, y_later, h_later = sim.get_fused_pose()
    dist_from_pre_set_pose = ((x_later - x_before) ** 2 + (y_later - y_before) ** 2) ** 0.5
    assert dist_from_pre_set_pose < 20.0, (
        f"fused pose drifted far from its pre-SET value after a few more "
        f"ticks -- looks like a delayed reset-to-origin: "
        f"pre-SET=({x_before:.2f}, {y_before:.2f}), "
        f"post-tick=({x_later:.2f}, {y_later:.2f}), "
        f"distance={dist_from_pre_set_pose:.2f} mm"
    )


# ---------------------------------------------------------------------------
# Test 3: SET ekfROtosXy changes the live position-correction strength.
# Mirrors test_067_003_ekf_setnoise.py's heading-channel fusion-behavior
# test, applied to the position channel via a newly-registered key.
# ---------------------------------------------------------------------------

def _position_correction_after_one_tick(ekf_r_otos_xy: float) -> float:
    """Fresh Sim(): SET ekfROtosXy, inject a fixed OTOS x-position
    disagreement, enable fusion, tick exactly one control step, and return
    the magnitude the fused x position moved toward the injected OTOS x.

    A fresh Sim() per value mirrors 067-003's own isolation pattern -- no
    state must carry over between the low-R and high-R measurements.
    """
    with Sim() as sim:
        sim.send_command("SET sTimeout=60000")
        sim.send_command(f"SET ekfROtosXy={ekf_r_otos_xy}")

        # Let P[0][0] (x-position covariance) grow off its zero-at-boot floor
        # via ordinary predict-step process noise so the Kalman gain is
        # non-degenerate (K = P/(P+R) is 0 regardless of R when P=0).
        sim.tick_for(500, step_ms=24)

        enc_x, enc_y, enc_h = sim.get_enc_pose()
        x_before = sim.get_fused_pose()[0]

        # Inject an OTOS reading that agrees on y and heading but disagrees
        # on x by a fixed, deliberate 15 mm -- comfortably inside the
        # chi-square 2-DOF gate (5.99) for every R tested below, so every
        # run performs a real (non-rejected) update.
        otos_x = enc_x + 15.0
        sim.set_otos_pose(otos_x, enc_y, enc_h)
        sim.set_otos_fusion(True)

        sim.tick_for(24, step_ms=24)  # exactly one control tick

        x_after = sim.get_fused_pose()[0]
        rejects = sim.get_ekf_rej_count()

    assert rejects == 0, (
        f"position update was gate-rejected at ekfROtosXy={ekf_r_otos_xy} "
        f"(rej_count={rejects}) -- test setup assumption violated"
    )
    return abs(x_after - x_before)


def test_ekfrotosxy_changes_position_correction_strength():
    """A smaller ekfROtosXy (less measurement noise assumed) must produce a
    markedly LARGER single-tick position correction toward a fixed OTOS
    disagreement than a larger ekfROtosXy (more measurement noise assumed).

    This is the live-consumer half of the fix: it proves Drive::configure()'s
    SET-triggered setNoise() call actually reaches EKFTiny's measurement-noise
    state consumed by updatePosition(), not just RobotConfig storage.
    """
    correction_low_r = _position_correction_after_one_tick(5.0)
    correction_high_r = _position_correction_after_one_tick(5000.0)

    assert correction_low_r > 5.0, (
        f"low-R correction too small to be a meaningful measurement: "
        f"{correction_low_r:.4f} mm"
    )
    assert correction_low_r > correction_high_r * 2.0, (
        f"SET ekfROtosXy did not change position-correction strength: "
        f"low-R (5.0) correction={correction_low_r:.4f} mm, "
        f"high-R (5000.0) correction={correction_high_r:.4f} mm "
        f"(expected low-R correction to be markedly larger)"
    )
