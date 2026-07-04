"""
test_sim_otos_lever_arm.py — ticket 066-001 (CR-07/CR-08).

Verifies the two structural sim-OTOS fidelity gaps this ticket closes:

  (a) SimOdometer now SAMPLES PhysicsWorld's true centre pose each tick
      (instead of re-integrating commanded wheel speeds), so a chassis-truth
      slip configured via ``sim_set_motor_slip`` (in the range
      ``Odometry.h::effectiveSlip`` treats as real slip, [0.5, 1.0]) now shows
      up as a genuine disagreement between the encoder-only dead-reckoning
      estimate and the OTOS/plant-truth estimate — matching real hardware,
      where the OTOS independently tracks ground truth instead of
      re-deriving the encoders' own belief.

  (b) SimOdometer::readTransformed() now round-trips the accumulated centre
      estimate through ``centreToSensor()``/``sensorToCentre()``
      (source/hal/capability/OtosLeverArm.h) — the SAME shared math
      ``OtosSensor::readTransformed()`` uses on hardware — so the host-side
      lever-arm compensation a past hardware regression (``db11b7c``, 433 mm
      of phantom translation on a pure spin) broke now has sim coverage.
      test_otos_lever_arm_regression_canary below exercises the shared
      header's pure functions directly to prove a mismatched-heading bug
      (the db11b7c failure mode) would produce exactly this kind of phantom
      translation, and that the correct (same-instant-heading) round trip
      used by SimOdometer/OtosSensor does not.
"""
from __future__ import annotations

import ctypes
import math
import pathlib
import subprocess

import pytest


# ---------------------------------------------------------------------------
# (a) Pure spin with a configured lever arm -> zero phantom translation
# ---------------------------------------------------------------------------

def test_pure_spin_with_lever_arm_zero_phantom_translation(sim):
    """Pure spin, nonzero odomOffX/odomOffY -> OTOS-derived centre stays ~0.

    Exercises the live IOdometer::readTransformed() round trip (via
    sim.get_optical_pose(), Robot::otosCorrect()'s captured reading) rather
    than calling centreToSensor()/sensorToCentre() directly — this is the
    same code path OtosSensor::readTransformed() runs on hardware.  Runs long
    enough to cross the (-pi, pi] wrap boundary (CR-15 item 1, also resolved
    by this ticket) to confirm the lever-arm compensation keeps holding once
    _truePoseH/_odomH wrap.
    """
    sim.send_command("SET sTimeout=60000")
    sim.set_perfect()
    sim.enable_otos_model()
    sim.set_otos_fusion(True)

    # A deliberately large mounting offset (tovez's real value is smaller,
    # ~-47.7/3.5mm) so any phantom translation would be unmistakable.
    sim.send_command("SET odomOffX=60")
    sim.send_command("SET odomOffY=-30")

    sim.send_command("VW 0 1500")           # spin CCW
    sim.tick_for(3500, step_ms=24)           # crosses the +-pi wrap boundary

    ox, oy, oh = sim.get_optical_pose()
    tx, ty, th = sim.get_true_pose()

    assert math.hypot(ox, oy) < 2.0, (
        f"phantom translation after a pure spin with a configured lever arm: "
        f"({ox:.3f}, {oy:.3f}) mm — the centreToSensor()/sensorToCentre() "
        f"round trip should cancel exactly for a stationary chassis centre."
    )
    # Confirm it actually spun past the wrap boundary (otherwise the
    # zero-translation assertion above is trivially true).
    assert abs(th) > 2.0, f"plant did not rotate enough to test the wrap: true h={th}"
    # OTOS heading tracks plant truth (perfect sensor, no noise configured).
    assert oh == pytest.approx(th, abs=0.05), f"OTOS h={oh} vs true h={th}"


# ---------------------------------------------------------------------------
# (b) Turn with chassis-truth slip -> encoder/OTOS disagree like hardware
# ---------------------------------------------------------------------------

def test_turn_with_slip_otos_matches_truth_encoder_diverges(sim):
    """Chassis-truth slip: OTOS tracks plant truth; encoder dead-reckoning
    does not; the fused estimate (with OTOS fusion enabled) tracks OTOS.

    sim_set_motor_slip's (side=2) "straight" argument lands _rotationalSlip
    in the [0.5, 1.0] range effectiveSlip() treats as real chassis-truth slip
    (see Odometry.h), so PhysicsWorld's sub-step B (chassis pose integration,
    what SimOdometer now samples) diverges from sub-step A' (the REPORTED
    encoder accumulator Odometry::predict() dead-reckons from) — exactly the
    "OTOS is an independent ground-truth sensor, encoders are not" gap CR-07
    identified as having zero sim reachability before this ticket.
    """
    sim.send_command("SET sTimeout=60000")
    sim.set_perfect()
    sim.enable_otos_model()
    sim.set_otos_fusion(True)

    # Chassis-truth slip in the effective real-slip range (not the negative
    # slip_turn_extra set_field_profile() uses, which effectiveSlip() clamps
    # to 1.0 / no chassis-truth effect — see architecture-update.md's
    # existing-test-impact analysis).
    sim._lib.sim_set_motor_slip(
        sim._h, ctypes.c_int(2), ctypes.c_float(0.7), ctypes.c_float(0.0))

    sim.send_command("VW 150 250")          # forward + turn: a genuine arc
    sim.tick_for(2000, step_ms=24)

    tx, ty, th = sim.get_true_pose()
    ox, oy, oh = sim.get_optical_pose()
    ex, ey, eh = sim.get_enc_pose()
    fx, fy, fh = sim.get_fused_pose()

    dist_otos_true = math.hypot(ox - tx, oy - ty)
    dist_enc_true  = math.hypot(ex - tx, ey - ty)
    dist_fused_otos = math.hypot(fx - ox, fy - oy)
    dist_fused_enc  = math.hypot(fx - ex, fy - ey)

    # OTOS samples plant truth directly (no noise configured) -> tight match.
    assert dist_otos_true < 2.0, (
        f"OTOS pose diverged from plant truth under slip: "
        f"otos=({ox:.2f},{oy:.2f}) true=({tx:.2f},{ty:.2f}) dist={dist_otos_true:.2f}mm"
    )
    # Encoder dead-reckoning integrates the REPORTED (slipped) accumulator and
    # so must clearly disagree with plant truth -- the exact hardware bug
    # class (encoder/OTOS disagreement under slip) that had zero sim coverage.
    assert dist_enc_true > 50.0, (
        f"encoder pose did not diverge from plant truth under configured "
        f"slip: enc=({ex:.2f},{ey:.2f}) true=({tx:.2f},{ty:.2f}) "
        f"dist={dist_enc_true:.2f}mm (expected > 50mm)"
    )
    # With fusion enabled, the fused estimate tracks OTOS, not the encoder.
    assert dist_fused_otos < dist_fused_enc, (
        f"fused pose should track OTOS (ground truth) under slip, not the "
        f"encoder accumulator: dist(fused,otos)={dist_fused_otos:.2f}mm "
        f"dist(fused,enc)={dist_fused_enc:.2f}mm"
    )
    assert dist_fused_otos < 20.0, (
        f"fused pose did not track OTOS closely under slip: "
        f"dist(fused,otos)={dist_fused_otos:.2f}mm"
    )


# ---------------------------------------------------------------------------
# (c) OtosLeverArm.h regression canary — direct math-level proof
# ---------------------------------------------------------------------------
#
# The sim-ABI test above (a) proves the round trip is exercised end to end
# and cancels correctly TODAY.  This harness proves WHY that matters: it
# calls the shared centreToSensor()/sensorToCentre() functions directly (the
# same functions OtosSensor.cpp and SimOdometer.cpp both call) and shows
# that (1) a same-instant-heading round trip is an exact no-op for a
# nonzero lever arm, and (2) a mismatched/stale heading between the two
# calls -- the db11b7c failure mode (readTransformed used a lagging fused
# heading instead of the same-instant OTOS heading) -- leaves a clear
# phantom translation, proportional to the offset and the heading error.

_TESTS_DIR = pathlib.Path(__file__).resolve().parents[2]   # tests/
_REPO_ROOT = _TESTS_DIR.parent                              # repo root
_SRC = _REPO_ROOT / "source"

_HARNESS = r"""
#include "hal/capability/OtosLeverArm.h"
#include <cstdio>
#include <cmath>

int main() {
    int failures = 0;

    // --- 1. Same-instant-heading round trip is an exact no-op ---------------
    {
        float offX = 60.0f, offY = -30.0f;
        bool ok = true;
        for (float h = -3.0f; h <= 3.0f; h += 0.37f) {
            float centreX = 123.0f, centreY = -45.0f;
            float sensorX = 0.0f, sensorY = 0.0f;
            centreToSensor(centreX, centreY, h, offX, offY, sensorX, sensorY);
            float outX = 0.0f, outY = 0.0f;
            sensorToCentre(sensorX, sensorY, h, offX, offY, outX, outY);
            if (std::fabs(outX - centreX) > 1e-3f || std::fabs(outY - centreY) > 1e-3f) {
                ok = false;
            }
        }
        if (!ok) { printf("FAIL roundtrip\n"); ++failures; }
        else     { printf("PASS roundtrip\n"); }
    }

    // --- 2. Mismatched (stale) heading between the two calls leaves a -------
    //        clear phantom translation -- the db11b7c failure mode.
    {
        float offX = 60.0f, offY = -30.0f;
        float centreX = 0.0f, centreY = 0.0f;   // pure spin: centre stays put
        float trueHeading  = 1.2f;               // same-instant heading (correct)
        float staleHeading = 0.3f;                // lagging heading (the bug)
        float sensorX = 0.0f, sensorY = 0.0f;
        centreToSensor(centreX, centreY, trueHeading, offX, offY, sensorX, sensorY);
        float outX = 0.0f, outY = 0.0f;
        sensorToCentre(sensorX, sensorY, staleHeading, offX, offY, outX, outY);
        float phantom = std::sqrt(outX * outX + outY * outY);
        if (!(phantom > 10.0f)) {
            printf("FAIL canary phantom=%.4f\n", phantom);
            ++failures;
        } else {
            printf("PASS canary phantom=%.4f\n", phantom);
        }
    }

    printf("DONE failures=%d\n", failures);
    return failures == 0 ? 0 : 1;
}
"""


@pytest.fixture(scope="module")
def lever_arm_harness(tmp_path_factory):
    """Compile + run the standalone OtosLeverArm.h harness once for the module."""
    workdir = tmp_path_factory.mktemp("otos_lever_arm")
    src = workdir / "harness.cpp"
    src.write_text(_HARNESS)
    exe = workdir / "harness"

    cmd = [
        "c++", "-std=c++11",
        str(src),
        "-I", str(_SRC),
        "-o", str(exe),
    ]
    build = subprocess.run(cmd, capture_output=True, text=True)
    if build.returncode != 0:
        pytest.fail(f"OtosLeverArm.h harness failed to compile:\n{build.stderr}")

    run = subprocess.run([str(exe)], capture_output=True, text=True)
    return run


def _results(run):
    return {
        line.split()[1]: line.split()[0]
        for line in run.stdout.splitlines()
        if line.startswith(("PASS ", "FAIL "))
    }


def test_otos_lever_arm_roundtrip_is_identity(lever_arm_harness):
    """Correct (same-instant-heading) round trip cancels exactly, any heading."""
    assert lever_arm_harness.returncode == 0, (
        f"harness stdout:\n{lever_arm_harness.stdout}\nstderr:\n{lever_arm_harness.stderr}"
    )
    assert _results(lever_arm_harness).get("roundtrip") == "PASS"


def test_otos_lever_arm_regression_canary(lever_arm_harness):
    """A stale/mismatched heading (db11b7c's failure mode) leaves a clear
    phantom translation -- proving the shared header, and any test that
    exercises it, WOULD catch a repeat of that regression."""
    assert _results(lever_arm_harness).get("canary") == "PASS"
