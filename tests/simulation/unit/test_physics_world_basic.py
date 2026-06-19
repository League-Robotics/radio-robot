"""
test_physics_world_basic.py — isolation test for PhysicsWorld (ticket 040-001).

PhysicsWorld is the new single-source-of-ground-truth plant (source/io/sim/).
It is NOT yet wired into the sim ABI (that happens in T2), so it cannot be
exercised through the `firmware.Sim` ctypes wrapper yet.  Instead this test
compiles a tiny self-contained C++ harness that links PhysicsWorld.cpp directly,
runs it, and asserts the ground-truth values.

What it pins:
  1. The encoder sub-step (sub-step A) is BIT-FOR-BIT identical to the golden
     MockMotor::integrate path (zero slip / zero noise / offset 1.0).  The harness
     computes the same reference expression in float and asserts exact equality of
     the raw 32-bit float bit patterns — not an epsilon compare.
  2. update() after setActuators advances the chassis forward (truePoseX > 0,
     truePoseH ~ 0 for a straight drive) and matches the midpoint-arc value.
  3. setTrue* injectors set their fields; a following update with zero PWM does
     not clobber injected pose / sensor truth.
  4. reset() zeros all state.
"""
import pathlib
import subprocess
import sys

import pytest

_TESTS_DIR = pathlib.Path(__file__).resolve().parents[2]   # tests/
_REPO_ROOT = _TESTS_DIR.parent                             # repo root
_SRC = _REPO_ROOT / "source"

_INCLUDE_DIRS = [
    _SRC,
    _SRC / "io",
    _SRC / "io" / "capability",
    _SRC / "io" / "real",
    _SRC / "control",
    _SRC / "robot",
    _SRC / "types",
    _SRC / "app",
]

# C++ harness: drives PhysicsWorld and prints PASS/FAIL lines the test parses.
_HARNESS = r"""
#include "io/sim/PhysicsWorld.h"
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>

// Reference encoder accumulation — the EXACT golden MockMotor::integrate path
// (zero slip, zero noise, offset 1.0).  Used to assert sub-step A bit-exactness.
static float refEnc(int8_t pwm, float nominalMaxMms, float offset,
                    uint32_t dt_ms, float prior) {
    float vel = (pwm / 100.0f) * nominalMaxMms * offset;
    float noisy = vel * (1.0f - 0.0f) + 0.0f;   // golden: slip=0, noise=0
    return prior + noisy * (static_cast<float>(dt_ms) / 1000.0f);
}

static bool bitEqual(float a, float b) {
    uint32_t ua, ub;
    std::memcpy(&ua, &a, 4);
    std::memcpy(&ub, &b, 4);
    return ua == ub;
}

int main() {
    int failures = 0;

    // --- 1. Bit-exact encoder accumulation across several steps -------------
    {
        PhysicsWorld w;
        w.setTrackwidth(150.0f);
        w.setNominalMaxMms(400.0f);
        w.setActuators(50, 50);

        float refL = 0.0f, refR = 0.0f;
        for (int i = 0; i < 10; ++i) {
            w.update(24);
            refL = refEnc(50, 400.0f, 1.0f, 24, refL);
            refR = refEnc(50, 400.0f, 1.0f, 24, refR);
        }
        if (!bitEqual(w.trueEncLMm(), refL) || !bitEqual(w.trueEncRMm(), refR)) {
            printf("FAIL bitexact encL=%.9g ref=%.9g encR=%.9g ref=%.9g\n",
                   w.trueEncLMm(), refL, w.trueEncRMm(), refR);
            ++failures;
        } else {
            printf("PASS bitexact\n");
        }
    }

    // --- 2. Single-step values match the ticket's worked example ------------
    {
        PhysicsWorld w;
        w.setTrackwidth(150.0f);
        w.setNominalMaxMms(400.0f);
        w.setActuators(50, 50);
        w.update(24);
        // velL = velR = (50/100)*400 = 200 mm/s ; enc = 200 * 0.024 = 4.8 mm
        if (std::fabs(w.trueEncLMm() - 4.8f) > 1e-5f ||
            std::fabs(w.trueEncRMm() - 4.8f) > 1e-5f) {
            printf("FAIL enc4p8 L=%.9g R=%.9g\n", w.trueEncLMm(), w.trueEncRMm());
            ++failures;
        } else {
            printf("PASS enc4p8\n");
        }
        if (!(w.trueVelLMms() == 200.0f && w.trueVelRMms() == 200.0f)) {
            printf("FAIL vel200 L=%.9g R=%.9g\n", w.trueVelLMms(), w.trueVelRMms());
            ++failures;
        } else {
            printf("PASS vel200\n");
        }
        // Straight drive: moved forward, heading ~ 0, X ~ dCenter = 4.8 mm.
        if (!(w.truePoseX() > 0.0f) || std::fabs(w.truePoseH()) > 1e-6f) {
            printf("FAIL straight X=%.9g H=%.9g\n", w.truePoseX(), w.truePoseH());
            ++failures;
        } else {
            printf("PASS straight\n");
        }
        if (std::fabs(w.truePoseX() - 4.8f) > 1e-3f) {
            printf("FAIL posex X=%.9g\n", w.truePoseX());
            ++failures;
        } else {
            printf("PASS posex\n");
        }
    }

    // --- 3. Truth injection is not clobbered by a zero-PWM update -----------
    {
        PhysicsWorld w;
        w.setTruePose(123.0f, -45.0f, 1.5f);
        w.setTrueWheelTravel(7.0f, 9.0f);
        w.setTrueVelocity(11.0f, 13.0f);
        uint16_t line[4] = {10, 20, 30, 40};
        uint16_t port[4] = {1, 2, 3, 4};
        w.setTrueSensorValues(line, 100, 200, 300, 400, port);
        // Zero PWM: encoder/velocity accumulate by 0; pose advances by 0.
        w.setActuators(0, 0);
        w.update(24);
        bool ok = (w.truePoseX() == 123.0f) && (w.truePoseY() == -45.0f) &&
                  (w.truePoseH() == 1.5f) &&
                  (w.trueEncLMm() == 7.0f) && (w.trueEncRMm() == 9.0f) &&
                  (w.trueVelLMms() == 0.0f) && (w.trueVelRMms() == 0.0f) &&
                  (w.lineRaw(0) == 10) && (w.lineRaw(3) == 40) &&
                  (w.port(0) == 1) && (w.port(3) == 4);
        uint16_t r, g, b, c;
        w.colorRGBC(r, g, b, c);
        ok = ok && (r == 100) && (g == 200) && (b == 300) && (c == 400);
        if (!ok) {
            printf("FAIL inject pose=(%.3g,%.3g,%.3g) enc=(%.3g,%.3g) vel=(%.3g,%.3g)\n",
                   w.truePoseX(), w.truePoseY(), w.truePoseH(),
                   w.trueEncLMm(), w.trueEncRMm(), w.trueVelLMms(), w.trueVelRMms());
            ++failures;
        } else {
            printf("PASS inject\n");
        }
    }

    // --- 4. reset() zeros all state -----------------------------------------
    {
        PhysicsWorld w;
        w.setActuators(80, 20);
        w.update(50);
        uint16_t line[4] = {1, 2, 3, 4};
        uint16_t port[4] = {5, 6, 7, 8};
        w.setTrueSensorValues(line, 9, 10, 11, 12, port);
        w.reset();
        uint16_t r, g, b, c;
        w.colorRGBC(r, g, b, c);
        bool ok = (w.truePoseX() == 0.0f) && (w.truePoseY() == 0.0f) &&
                  (w.truePoseH() == 0.0f) &&
                  (w.trueEncLMm() == 0.0f) && (w.trueEncRMm() == 0.0f) &&
                  (w.trueVelLMms() == 0.0f) && (w.trueVelRMms() == 0.0f) &&
                  (w.lineRaw(0) == 0) && (w.port(0) == 0) &&
                  (r == 0) && (g == 0) && (b == 0) && (c == 0);
        if (!ok) { printf("FAIL reset\n"); ++failures; }
        else     { printf("PASS reset\n"); }
    }

    // --- 5. Slip lives at the body-rotation step, not the encoder -----------
    {
        // With turn slip configured, a pure spin (L=-50, R=+50) must leave the
        // encoder TRUE travel unaffected (slip not applied in sub-step A) while
        // reducing the body heading (slip applied in sub-step B).
        PhysicsWorld noSlip, withSlip;
        noSlip.setTrackwidth(150.0f);
        noSlip.setNominalMaxMms(400.0f);
        noSlip.setActuators(-50, 50);
        withSlip.setTrackwidth(150.0f);
        withSlip.setNominalMaxMms(400.0f);
        withSlip.setActuators(-50, 50);
        withSlip.setSlip(0.7f, 0.0f);   // effectiveSlip(0.7) = 0.7
        noSlip.update(100);
        withSlip.update(100);
        // Encoders identical (slip not on the encoder path).
        bool encOk = bitEqual(noSlip.trueEncRMm(), withSlip.trueEncRMm()) &&
                     bitEqual(noSlip.trueEncLMm(), withSlip.trueEncLMm());
        // Heading reduced by the slip factor (0.7) in sub-step B.
        bool headOk = std::fabs(withSlip.truePoseH() - noSlip.truePoseH() * 0.7f) < 1e-4f;
        if (!encOk || !headOk) {
            printf("FAIL slip encOk=%d headOk=%d noSlipH=%.6g slipH=%.6g\n",
                   encOk, headOk, noSlip.truePoseH(), withSlip.truePoseH());
            ++failures;
        } else {
            printf("PASS slip\n");
        }
    }

    printf("DONE failures=%d\n", failures);
    return failures == 0 ? 0 : 1;
}
"""


@pytest.fixture(scope="module")
def physics_world_harness(tmp_path_factory):
    """Compile + run the standalone PhysicsWorld harness once for the module."""
    workdir = tmp_path_factory.mktemp("physics_world")
    src = workdir / "harness.cpp"
    src.write_text(_HARNESS)
    exe = workdir / "harness"

    cmd = [
        "c++", "-std=c++11", "-DHOST_BUILD=1",
        str(src),
        str(_SRC / "io" / "sim" / "PhysicsWorld.cpp"),
    ]
    for d in _INCLUDE_DIRS:
        cmd += ["-I", str(d)]
    cmd += ["-o", str(exe)]

    build = subprocess.run(cmd, capture_output=True, text=True)
    if build.returncode != 0:
        pytest.fail(f"PhysicsWorld harness failed to compile:\n{build.stderr}")

    run = subprocess.run([str(exe)], capture_output=True, text=True)
    return run


def _results(run):
    return {
        line.split()[1]: line.split()[0]
        for line in run.stdout.splitlines()
        if line.startswith(("PASS ", "FAIL "))
    }


def test_harness_runs_clean(physics_world_harness):
    """The whole harness returns 0 (no FAIL lines)."""
    assert physics_world_harness.returncode == 0, (
        f"harness stdout:\n{physics_world_harness.stdout}\n"
        f"stderr:\n{physics_world_harness.stderr}"
    )
    assert "DONE failures=0" in physics_world_harness.stdout


def test_encoder_bit_exact(physics_world_harness):
    """Sub-step A encoder accumulation is bit-for-bit MockMotor::integrate."""
    assert _results(physics_world_harness).get("bitexact") == "PASS"


def test_encoder_worked_example(physics_world_harness):
    """setActuators(50,50)+update(24) → 4.8 mm true travel, 200 mm/s velocity."""
    res = _results(physics_world_harness)
    assert res.get("enc4p8") == "PASS"
    assert res.get("vel200") == "PASS"


def test_straight_drive_pose(physics_world_harness):
    """Straight drive advances X forward with ~zero heading; X ~ midpoint arc."""
    res = _results(physics_world_harness)
    assert res.get("straight") == "PASS"
    assert res.get("posex") == "PASS"


def test_truth_injection_not_clobbered(physics_world_harness):
    """setTrue* injectors persist through a zero-PWM update()."""
    assert _results(physics_world_harness).get("inject") == "PASS"


def test_reset_zeros_state(physics_world_harness):
    """reset() zeros pose / travel / velocity / sensor truth."""
    assert _results(physics_world_harness).get("reset") == "PASS"


def test_slip_at_body_rotation_step(physics_world_harness):
    """Slip reduces body heading but leaves true encoder travel untouched."""
    assert _results(physics_world_harness).get("slip") == "PASS"
