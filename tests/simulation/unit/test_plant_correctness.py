"""
test_plant_correctness.py — plant-only isolation tests (ticket 040-005, Test 1).

Verifies that the PhysicsWorld plant produces the expected kinematic trajectory
under setActuators+update, WITHOUT any sensor error involved.  This is the
"plant-only" row of the §7 verification matrix: it isolates ground-truth
chassis dynamics from every observation model.

Two harnesses are used, both exercising the SAME PhysicsWorld:

  1. A standalone C++ harness (compiled per-module, linking PhysicsWorld.cpp
     directly) drives setActuators(pwmL, pwmR)+update(dt) with the EXACT pwm
     values from the ticket — the only way to inject raw actuator PWM into the
     plant without the velocity controller in the loop.  It asserts the
     ticket's worked examples (straight drive → x ≈ 200 mm; spot turn → h ≈ π/2).

  2. The `sim` fixture (firmware Sim ctypes wrapper) exercises the back-compat
     ABI: sim_set_true_pose truth injection persists through a zero-PWM tick,
     and sim_set_enc_l/r correctly sets true wheel travel (the regression test
     for the historical "lying sim_set_enc" bug — 040-003 fix).

No sensor model, no slip (clean plant), no field profile.
"""
import math
import pathlib
import subprocess

import pytest

from firmware import Sim

_TESTS_DIR = pathlib.Path(__file__).resolve().parents[2]   # tests/
_REPO_ROOT = _TESTS_DIR.parent                             # repo root
_SRC = _REPO_ROOT / "source"

_INCLUDE_DIRS = [
    _SRC,
    _SRC / "hal",
    _SRC / "hal" / "capability",
    _SRC / "hal" / "real",
    _SRC / "control",
    _SRC / "robot",
    _SRC / "types",
    _SRC / "app",
    _REPO_ROOT / "libraries" / "tinyekf",  # 050-005: EKFTiny.h includes <tinyekf.h>
]

# ---------------------------------------------------------------------------
# Standalone C++ harness: drives PhysicsWorld with raw actuator PWM and prints
# PASS/FAIL lines the test parses.  Uses the ticket's exact pwm/duration values.
#
# Plant config matches the SIM runtime: trackwidth 128 mm (DefaultConfig),
# nominalMaxMms 400 mm/s (PhysicsWorld::kNominalMaxMms).
# ---------------------------------------------------------------------------
_HARNESS = r"""
#include "hal/sim/PhysicsWorld.h"
#include <cstdint>
#include <cstdio>
#include <cmath>

int main() {
    int failures = 0;
    const float TW  = 128.0f;     // DefaultConfig trackwidthMm (SIM runtime)
    const float NOM = 400.0f;     // PhysicsWorld::kNominalMaxMms

    // --- A. Straight drive: pwmL=pwmR=50 for 1000 ms → x ≈ 200 mm ---------
    {
        PhysicsWorld w;
        w.setTrackwidth(TW);
        w.setNominalMaxMms(NOM);
        w.setActuators(50, 50);
        // 1000 ms in 24 ms steps (matches the sim tick granularity).
        for (int t = 0; t < 1000; t += 24) {
            uint32_t dt = (1000 - t) < 24 ? (uint32_t)(1000 - t) : 24u;
            w.update(dt);
        }
        // vel = (50/100)*400 = 200 mm/s ; x ≈ 200 mm over 1 s.
        bool ok = (w.truePoseX() > 0.0f) &&
                  (std::fabs(w.truePoseX() - 200.0f) < 1.0f) &&
                  (std::fabs(w.truePoseY()) < 1e-3f) &&
                  (std::fabs(w.truePoseH()) < 1e-5f);
        if (!ok) {
            printf("FAIL straight x=%.4g y=%.4g h=%.4g\n",
                   w.truePoseX(), w.truePoseY(), w.truePoseH());
            ++failures;
        } else {
            printf("PASS straight\n");
        }
    }

    // --- B. Spot turn: pwmL=-50, pwmR=50 for 500 ms → h ≈ π/2 -------------
    {
        PhysicsWorld w;
        w.setTrackwidth(TW);
        w.setNominalMaxMms(NOM);
        w.setActuators(-50, 50);
        for (int t = 0; t < 500; t += 24) {
            uint32_t dt = (500 - t) < 24 ? (uint32_t)(500 - t) : 24u;
            w.update(dt);
        }
        // dL = -100, dR = +100 over 0.5 s ; dTheta = (dR-dL)/TW = 200/128 ≈ 1.5625 rad.
        // ≈ π/2 (1.5708).  Tolerance 0.05 rad covers the 128 vs π-exact mismatch.
        float h = w.truePoseH();
        bool ok = (std::fabs(h - (float)M_PI / 2.0f) < 0.05f);
        if (!ok) {
            printf("FAIL spotturn h=%.5g pi2=%.5g\n", h, (float)M_PI / 2.0f);
            ++failures;
        } else {
            printf("PASS spotturn\n");
        }
    }

    // --- C. Truth injection persists through a zero-PWM update -----------
    {
        PhysicsWorld w;
        w.setTruePose(0.0f, 0.0f, 0.0f);
        w.setActuators(0, 0);
        w.update(24);    // zero PWM: pose must stay at origin (no velocity)
        bool ok = (w.truePoseX() == 0.0f) && (w.truePoseY() == 0.0f) &&
                  (w.truePoseH() == 0.0f);
        if (!ok) { printf("FAIL idle_origin\n"); ++failures; }
        else     { printf("PASS idle_origin\n"); }
    }

    printf("DONE failures=%d\n", failures);
    return failures == 0 ? 0 : 1;
}
"""


@pytest.fixture(scope="module")
def plant_harness(tmp_path_factory):
    """Compile + run the standalone PhysicsWorld actuator harness once."""
    workdir = tmp_path_factory.mktemp("plant_correctness")
    src = workdir / "harness.cpp"
    src.write_text(_HARNESS)
    exe = workdir / "harness"

    cmd = [
        "c++", "-std=c++11", "-DHOST_BUILD=1",
        str(src),
        str(_SRC / "hal" / "sim" / "PhysicsWorld.cpp"),
    ]
    for d in _INCLUDE_DIRS:
        cmd += ["-I", str(d)]
    cmd += ["-o", str(exe)]

    build = subprocess.run(cmd, capture_output=True, text=True)
    if build.returncode != 0:
        pytest.fail(f"plant harness failed to compile:\n{build.stderr}")

    return subprocess.run([str(exe)], capture_output=True, text=True)


def _results(run):
    return {
        line.split()[1]: line.split()[0]
        for line in run.stdout.splitlines()
        if line.startswith(("PASS ", "FAIL "))
    }


# ---------------------------------------------------------------------------
# Plant-only via raw actuator PWM (standalone harness)
# ---------------------------------------------------------------------------

def test_straight_drive_reaches_target(plant_harness):
    """pwmL=pwmR=50 for 1000 ms → true_pose_x ≈ 200 mm, y ≈ 0, h ≈ 0."""
    assert plant_harness.returncode == 0, (
        f"harness stdout:\n{plant_harness.stdout}\nstderr:\n{plant_harness.stderr}"
    )
    assert _results(plant_harness).get("straight") == "PASS"


def test_spot_turn_quarter(plant_harness):
    """pwmL=-50, pwmR=50 for 500 ms → true_pose_h ≈ π/2 (quarter turn)."""
    assert _results(plant_harness).get("spotturn") == "PASS"


def test_idle_at_zero_pwm_does_not_move(plant_harness):
    """At zero PWM the plant stays at the origin (no velocity)."""
    assert _results(plant_harness).get("idle_origin") == "PASS"


# ---------------------------------------------------------------------------
# Truth injection via the back-compat ABI (sim fixture)
# ---------------------------------------------------------------------------

def test_set_true_pose_injection_persists(sim):
    """sim_set_true_pose injects ground truth that survives zero-PWM ticks.

    The plant integrates 0 velocity at PWM=0, so an injected pose must be
    returned unchanged by sim_get_true_pose_* on the next tick.
    """
    sim.set_true_pose(123.0, -45.0, 1.5)
    # Read back immediately — no tick between set and read.
    x, y, h = sim.get_true_pose()
    assert x == pytest.approx(123.0, abs=1e-3)
    assert y == pytest.approx(-45.0, abs=1e-3)
    assert h == pytest.approx(1.5, abs=1e-3)

    # Five idle ticks (no motion command → zero PWM) must not clobber it.
    sim.tick_for(120, step_ms=24)
    x2, y2, h2 = sim.get_true_pose()
    assert x2 == pytest.approx(123.0, abs=1e-3), f"pose_x drifted to {x2}"
    assert y2 == pytest.approx(-45.0, abs=1e-3), f"pose_y drifted to {y2}"
    assert h2 == pytest.approx(1.5, abs=1e-3), f"pose_h drifted to {h2}"


def test_set_true_wheel_travel_injection_persists(sim):
    """sim_set_true_wheel_travel sets the true accumulators (ground truth)."""
    sim.set_true_wheel_travel(7.0, 9.0)
    l, r = sim.get_true_wheel_travel()
    assert l == pytest.approx(7.0, abs=1e-3)
    assert r == pytest.approx(9.0, abs=1e-3)

    # Idle ticks leave the true travel untouched (zero PWM adds 0).
    sim.tick_for(120, step_ms=24)
    l2, r2 = sim.get_true_wheel_travel()
    assert l2 == pytest.approx(7.0, abs=1e-3), f"true enc_l drifted to {l2}"
    assert r2 == pytest.approx(9.0, abs=1e-3), f"true enc_r drifted to {r2}"


def test_sim_set_enc_l_does_not_reset_to_zero(sim):
    """sim_set_enc_l(500) survives 5 zero-PWM ticks (the 'lying enc' regression).

    HISTORY (040-003): sim_set_enc_l once wrote only state.inputs.encLMm, which
    the next tick overwrote with the value promoted from positionMm() — the
    injected value vanished.  The fix sets BOTH the true wheel travel AND the
    reported accumulator in the plant, so the value persists through ticks.
    """
    sim.send_command("SET sTimeout=60000")
    # Inject 500 mm on the left encoder.
    sim._lib.sim_set_enc_l(sim._h, 500.0)

    # Five idle ticks at PWM=0 — the injected value must NOT reset to 0.
    sim.tick_for(120, step_ms=24)

    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    true_enc_l = sim.get_true_wheel_travel()[0]
    assert enc_l == pytest.approx(500.0, abs=1.0), (
        f"sim_set_enc_l(500) reset to {enc_l} after 5 ticks — the 'lying enc' "
        f"bug regressed (injected value overwritten by the integrated value)."
    )
    assert true_enc_l == pytest.approx(500.0, abs=1.0), (
        f"true wheel travel = {true_enc_l}, expected 500 — sim_set_enc_l did "
        f"not flow into the plant ground truth."
    )


def test_sim_set_enc_r_does_not_reset_to_zero(sim):
    """sim_set_enc_r(500) survives 5 zero-PWM ticks (right-side regression)."""
    sim.send_command("SET sTimeout=60000")
    sim._lib.sim_set_enc_r(sim._h, 500.0)
    sim.tick_for(120, step_ms=24)
    enc_r = float(sim._lib.sim_get_enc_r(sim._h))
    true_enc_r = sim.get_true_wheel_travel()[1]
    assert enc_r == pytest.approx(500.0, abs=1.0), (
        f"sim_set_enc_r(500) reset to {enc_r} after 5 ticks (regression)."
    )
    assert true_enc_r == pytest.approx(500.0, abs=1.0)


def test_straight_drive_advances_true_pose_via_command(sim):
    """A real VW drive advances the TRUE plant pose forward (end-to-end plant).

    Drives VW 200 0 (200 mm/s forward, no rotation) and asserts the plant's
    true pose moved forward with ~zero lateral / heading drift — the plant
    integrates the commanded straight motion correctly.
    """
    sim.send_command("SET sTimeout=60000")
    sim.send_command("VW 200 0")
    sim.tick_for(1500, step_ms=24)
    x, y, h = sim.get_true_pose()
    assert x > 100.0, f"true_pose_x={x} — plant did not advance under VW 200 0"
    assert abs(y) < 5.0, f"true_pose_y={y} — unexpected lateral drift"
    assert abs(math.atan2(math.sin(h), math.cos(h))) < 0.05, (
        f"true_pose_h={h} — unexpected heading drift on a straight drive"
    )
