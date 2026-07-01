"""test_sim_otos_heading_reset.py — regression test for ticket 063-006.

Verifies the sim OTOS reproduces the hardware heading-reset bug (SI alone
drifts back to the stale OTOS heading) and its fix (ZERO enc + OZ + SI holds
heading at 0), using the sim OTOS's own re-referenced accumulator.

Uses the ``sim`` fixture (tests/conftest.py) and the new ``begin_otos()``
harness hook (SimOdometer::begin(), C ABI ``sim_begin_otos`` in
tests/_infra/sim/sim_api.cpp) rather than ``set_field_profile``/
``set_otos_fusion`` alone, so the fixture stays free of turn-slip/noise side
effects except where a test explicitly opts into ``set_otos_fusion`` for the
per-tick EKF correction path.
"""
from __future__ import annotations


def _turn_and_stop(sim, omega_mrad=300, turn_ms=1000):
    """Turn in place to a non-zero heading, then stop.

    Uses "X" (stop / soft stop, MotionCommands.cpp) rather than "S" -- "S" is
    "set wheel speeds" (takes args) and replies ERR badarg with none, leaving
    the robot still spinning, which was caught during development of this
    test (see .clasi/knowledge notes for ticket 063-006).
    """
    sim.send_command(f"VW 0 {omega_mrad}")
    sim.tick_for(turn_ms)
    sim.send_command("X")
    sim.tick_for(300)


def test_otos_commands_ok_not_nodev(sim):
    """OZ/OI/OR/OV all return OK (never nodev) once begin_otos() has run."""
    sim.begin_otos()
    for verb in ("OI", "OZ", "OR"):
        reply = sim.send_command(verb)
        assert "nodev" not in reply, f"{verb} returned nodev: {reply!r}"
        assert "OK" in reply
    reply = sim.send_command("OV 0 0 0")
    assert "nodev" not in reply
    assert "OK" in reply


def test_oz_zeroes_otos_accumulator(sim):
    """OZ re-references the accumulator (_odomX/_odomY/_odomH), not just the
    raw-register shadow -- after OZ, sim.get_otos_pose() reads (0, 0, 0) even
    though the robot was previously at a non-zero heading."""
    sim.begin_otos()
    sim.enable_otos_model()
    _turn_and_stop(sim)
    x, y, h = sim.get_otos_pose()
    assert abs(h) > 0.05, "test setup: expected a non-zero heading before OZ"

    sim.send_command("OZ")
    x2, y2, h2 = sim.get_otos_pose()
    assert x2 == 0.0 and y2 == 0.0 and h2 == 0.0, (
        f"OZ must zero the sim OTOS accumulator, got ({x2}, {y2}, {h2})"
    )


def test_si_alone_drifts_back_to_otos_heading(sim):
    """Reproduces the hardware bug: SI without OZ does not hold heading."""
    sim.begin_otos()
    sim.set_otos_fusion(True)   # marks initialised + enables per-tick fusion
    sim.enable_otos_model()
    _turn_and_stop(sim)
    _, _, otos_h = sim.get_otos_pose()
    assert abs(otos_h) > 0.05

    sim.send_command("SI 0 0 0")
    for _ in range(20):
        sim.tick_for(50)
    _, _, fused_h = sim.get_fused_pose()
    assert abs(fused_h - otos_h) < 0.02, (
        f"Expected fused heading to drift back toward stale OTOS heading "
        f"{otos_h:.4f} without OZ, got {fused_h:.4f}"
    )


def test_zero_oz_si_resets_and_holds_heading(sim):
    """Verifies the fix: ZERO enc + OZ + SI resets heading to 0 and holds."""
    sim.begin_otos()
    sim.set_otos_fusion(True)
    sim.enable_otos_model()
    _turn_and_stop(sim)

    sim.send_command("ZERO enc")
    sim.send_command("OZ")
    sim.send_command("SI 0 0 0")

    for _ in range(60):   # ~3s -- long enough to catch any residual drift-back
        sim.tick_for(50)
    _, _, fused_h = sim.get_fused_pose()
    assert abs(fused_h) < 0.02, (
        f"Expected fused heading to hold at 0 after ZERO+OZ+SI, got {fused_h:.4f}"
    )
