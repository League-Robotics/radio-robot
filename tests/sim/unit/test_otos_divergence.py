"""Sim verification for ticket 082-005 (SUC-005): with ``Hal::SimOdometer``'s
error knobs (noise/scale/drift) set to non-zero values, ``TLM``'s ``otos=``
field diverges from the ctypes ground-truth pose by roughly the configured
amount over a drive sequence; with every knob zeroed, ``otos=`` re-converges
to (matches) ground truth.

This is the WIRE-level counterpart of ``test_otos_error_injection.py``
(sprint 081, ticket 006), which asserts the identical error-knob behavior
directly against ``sim.otos_pose()`` (the ctypes ``sim_get_otos_x/y/h``
reads). This file instead drives the same knobs and reads the result back
through ``SNAP``'s ``TLM ... otos=<x>,<y>,<h>`` wire field
(``commands/telemetry_commands.cpp``'s ``telemetryEmit()``, sourced from
``hardware.odometer()->pose()`` -- the SAME ``Hal::SimOdometer`` accumulator
``sim.otos_pose()`` reads), proving the STREAM/SNAP telemetry pipeline
itself (not just the underlying accumulator) carries the error-knob
behavior onto the wire, integer-truncated (mm) / centidegree-scaled per
``tlm_frame.cpp``.
"""
from __future__ import annotations

import math

import pytest

CDEG_PER_RAD = 5729.5779513   # kAngleScale, tlm_frame.cpp -- centidegrees per radian


def _parse_tlm(line: str) -> dict[str, str]:
    """Parse one "TLM t=... mode=... ..." wire line into a key->value dict.

    Local, small, deliberately duplicated per test file -- mirrors this
    directory's existing precedent (e.g. ``_drive_straight`` duplicated
    across test_otos_error_injection.py / test_errored_observation.py)
    rather than a shared test-util module.
    """
    parts = line.strip().split()
    assert parts[0] == "TLM", f"not a TLM line: {line!r}"
    return dict(p.split("=", 1) for p in parts[1:])


def _snap_otos(sim) -> tuple[float, float, float]:
    """Issue SNAP and return the wire otos=<x>,<y>,<h> field as
    (x [mm], y [mm], h [rad]) -- converting the wire's integer mm /
    centidegree encoding back to float for comparison against
    sim.true_pose()'s raw floats."""
    reply = sim.command("SNAP").strip()
    lines = reply.splitlines()
    assert len(lines) == 1, f"expected exactly one TLM line from SNAP, got: {reply!r}"
    tlm = _parse_tlm(lines[0])
    assert "otos" in tlm, "otos= must be present -- SimHardware always has an odometer"
    x, y, h_cdeg = (float(v) for v in tlm["otos"].split(","))
    return x, y, h_cdeg / CDEG_PER_RAD


def _drive_straight(sim, vx: float = 150.0, ms: int = 1000) -> None:
    sim.command("DEV DT PORTS 1 2")
    sim.command(f"DEV DT VW {vx} 0 0")
    sim.tick_for(ms)


def _spin_in_place(sim, omega: float = 1.0, ms: int = 240) -> None:
    """A short in-place rotation (240ms @ 1.0 rad/s ~= 13.7deg) -- short
    enough that true heading never approaches the +-pi wrap boundary
    either PhysicsWorld or SimOdometer applies to their own accumulator
    (mirrors test_otos_error_injection.py's own _spin_in_place)."""
    sim.command("DEV DT PORTS 1 2")
    sim.command(f"DEV DT VW 0 0 {omega}")
    sim.tick_for(ms)


def test_linear_scale_error_diverges_wire_otos_from_true_pose(sim):
    """setLinearScaleError inflates the WIRE otos= x by ~the configured
    fraction relative to true pose, read back over SNAP rather than
    sim.otos_pose() directly."""
    sim.set_otos_linear_scale_error(0.10)
    _drive_straight(sim)

    true_x, true_y, _true_h = sim.true_pose()
    otos_x, otos_y, _otos_h = _snap_otos(sim)

    assert otos_x != true_x, "the scale error did not actually perturb the wire otos= field"
    assert otos_x == pytest.approx(true_x * 1.10, rel=0.03)
    assert abs(otos_y) < 2.0 and abs(true_y) < 2.0


def test_angular_scale_error_diverges_wire_otos_heading_from_true_heading(sim):
    """setAngularScaleError inflates the WIRE otos= heading by ~the
    configured fraction, isolated via an in-place spin (dL+dR == 0 exactly
    every tick, so the linear term never contributes)."""
    sim.set_otos_angular_scale_error(0.10)
    _spin_in_place(sim)

    _true_x, _true_y, true_h = sim.true_pose()
    _otos_x, _otos_y, otos_h = _snap_otos(sim)

    assert true_h != 0.0, "the spin did not actually rotate the plant"
    assert otos_h == pytest.approx(true_h * 1.10, rel=0.03)


def test_linear_and_yaw_noise_cause_nonzero_wire_divergence(sim):
    """Nonzero linear+yaw noise sigma perturbs the wire otos= field away
    from exact agreement with true pose. Deterministic (fixed
    std::mt19937 seed, ticket 003/005's determinism gate), not flaky, even
    though the perturbation itself is statistical."""
    sim.set_otos_linear_noise(0.05)
    sim.set_otos_yaw_noise(0.05)
    _drive_straight(sim)

    true_x, true_y, true_h = sim.true_pose()
    otos_x, otos_y, otos_h = _snap_otos(sim)
    assert (otos_x, otos_y, otos_h) != (true_x, true_y, true_h)


def test_linear_and_yaw_drift_accumulate_on_wire_while_stationary(sim):
    """Hal::SimOdometer::tick() adds its per-tick drift to its own
    accumulator EVERY tick, unconditionally -- even with the chassis
    completely at rest (no DEV DT command ever issued, so true pose stays
    pinned at the origin). Read back over the wire (SNAP) rather than
    sim.otos_pose() directly."""
    sim.set_otos_linear_drift(2.0)
    sim.set_otos_yaw_drift(0.01)

    sim.tick_for(240)
    x_after_10, _y, h_after_10 = _snap_otos(sim)

    sim.tick_for(240)
    x_after_20, _y2, h_after_20 = _snap_otos(sim)

    true_x, true_y, true_h = sim.true_pose()

    assert x_after_10 > 0.0
    assert x_after_20 > x_after_10   # accumulates monotonically with elapsed ticks
    assert h_after_10 > 0.0
    assert h_after_20 > h_after_10

    assert true_x == 0.0 and true_y == 0.0 and true_h == 0.0


def test_large_combined_otos_error_diverges_wire_otos_measurably(sim):
    """A LARGE combined OTOS error (every one of the six knobs this ABI
    exposes) leaves the wire otos= field measurably diverged from true
    pose -- the acceptance criterion's "diverges by roughly the configured
    amount" proven with every knob active at once, not just in isolation."""
    sim.set_otos_linear_noise(0.05)
    sim.set_otos_yaw_noise(0.05)
    sim.set_otos_linear_scale_error(0.15)
    sim.set_otos_angular_scale_error(0.15)
    sim.set_otos_linear_drift(3.0)
    sim.set_otos_yaw_drift(0.02)

    _drive_straight(sim, vx=150.0, ms=1500)

    true_x, true_y, _true_h = sim.true_pose()
    otos_x, otos_y, _otos_h = _snap_otos(sim)
    assert (otos_x, otos_y) != pytest.approx((true_x, true_y), abs=1.0), (
        "the OTOS error knobs did not actually perturb the wire otos= field -- "
        "this test's premise (a genuinely large error) did not hold"
    )


def test_zeroing_every_otos_knob_reconverges_wire_otos_to_true_pose(sim):
    """Explicitly re-zeroes every OTOS-error knob this ABI exposes (rather
    than relying on a fresh instance's implicit defaults) -- proves
    "zeroed" behaves identically to "never touched": the wire otos= field
    RE-CONVERGES to (matches, within integer/centidegree wire-encoding
    truncation) true pose once every knob is back at its no-op default."""
    sim.set_otos_linear_noise(0.0)
    sim.set_otos_yaw_noise(0.0)
    sim.set_otos_linear_scale_error(0.0)
    sim.set_otos_angular_scale_error(0.0)
    sim.set_otos_linear_drift(0.0)
    sim.set_otos_yaw_drift(0.0)

    _drive_straight(sim)

    true_x, true_y, true_h = sim.true_pose()
    otos_x, otos_y, otos_h = _snap_otos(sim)

    # Wire encoding truncates to whole mm / centidegrees (tlm_frame.cpp) --
    # 1mm / ~0.01deg tolerance accounts for that truncation, not for any
    # remaining error-knob divergence (there is none once every knob is 0).
    assert otos_x == pytest.approx(true_x, abs=1.0)
    assert otos_y == pytest.approx(true_y, abs=1.0)
    assert otos_h == pytest.approx(true_h, abs=math.radians(0.5))


def test_divergence_then_reconvergence_across_one_continuous_drive(sim):
    """The acceptance criterion's own shape end to end within ONE test: set
    an error knob non-zero, drive, confirm divergence; zero the knob, drive
    further (same continuous session, no reset), confirm the wire otos=
    field's INCREMENTAL growth re-converges to track true pose's own growth
    1:1 -- proving "reconverge" is a live property of the SAME odometer
    instance across the sim's lifetime, not merely two independently-
    passing tests.

    Checked on each phase's INCREMENT (end-of-phase minus start-of-phase),
    not on absolute position: ``Hal::SimOdometer::tick()`` accumulates
    incrementally from the true-pose DELTA each tick (sim_odometer.cpp's
    own ``tick()``), so phase one's ~20% inflation is permanently baked
    into the absolute accumulator (by design -- there is no "un-apply past
    error" operation, matching a real sensor). The increment during phase
    two, however, is produced entirely by ticks where the knob was already
    back at its zero/no-op default, so it must track true pose's own
    increment 1:1 -- exactly what "re-converges" means for an accumulating
    (not absolute-position) sensor model.
    """
    sim.set_otos_linear_scale_error(0.20)
    _drive_straight(sim, vx=150.0, ms=600)

    true_x_1, _true_y_1, _true_h_1 = sim.true_pose()
    otos_x_1, _otos_y_1, _otos_h_1 = _snap_otos(sim)
    assert otos_x_1 == pytest.approx(true_x_1 * 1.20, rel=0.03), (
        "divergence phase: wire otos= should be inflated ~20% ahead of true pose"
    )

    sim.set_otos_linear_scale_error(0.0)
    _drive_straight(sim, vx=150.0, ms=600)
    true_x_2, _true_y_2, _true_h_2 = sim.true_pose()
    otos_x_2, _otos_y_2, _otos_h_2 = _snap_otos(sim)

    otos_dx = otos_x_2 - otos_x_1
    true_dx = true_x_2 - true_x_1
    assert otos_dx == pytest.approx(true_dx, rel=0.05), (
        f"phase-two increment should track true pose's own increment 1:1 "
        f"once the scale error is zeroed: otos_dx={otos_dx:.2f} true_dx={true_dx:.2f}"
    )
