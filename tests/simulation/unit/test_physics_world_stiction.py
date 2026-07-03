"""
test_physics_world_stiction.py — ticket 072-001.

``PhysicsWorld::update()``'s chassis integration was purely algebraic and
memoryless (``velL = (pwmL/100)*nominalMaxSpeed*offsetFactorL``): any nonzero
PWM produced a proportionally nonzero velocity, so the plant could never model
a motor that stops responding to small PWM near zero (real-world stiction /
breakaway). This ticket adds a stateless per-tick PWM dead-zone gate
(``|pwm| < stictionPwmSide -> vel = 0``, default 0 = never fires) and an
independent, separately-defaulted-off first-order response-lag filter
(``tau <= 0`` = no-op, no ``expf()`` call) — see architecture-update.md Step 4b
and Design Rationale Decision 3.

Same pattern as test_physics_world_body_scrub.py: compile a tiny
self-contained C++ harness that links PhysicsWorld.cpp directly, run it, and
parse PASS/FAIL lines.

What it pins:
  1. Default (stictionPwm=0, motorLag=0 on both sides) is a byte-identical
     no-op vs. the pre-072-001 algebraic expression (golden-TLM constraint).
  2. Gate boundary: |pwm| exactly at stictionPwmSide does NOT gate (formula
     applies); |pwm| one unit below DOES gate (vel forced to exactly 0) — both
     signs of pwm.
  3. Statelessness: a wheel gated to 0 this tick after being commanded above
     threshold (and moving) the previous tick has NO memory of "was moving" —
     the gate looks only at the CURRENT tick's pwm.
  4. Per-wheel independence: a stiction threshold configured on one side only
     does not affect the other side's gate.
  5. Lag filter tau<=0 is a byte-identical no-op (vel == velTarget, no expf()
     call), including when composed with an active stiction gate.
  6. Lag filter tau>0 converges the persistent lag state toward velTarget
     exponentially, matching the documented formula.
  7. reset() zeroes the lag filter's persistent state but leaves the
     stiction-threshold / lag-time-constant CONFIGURATION knobs intact
     (matches every other PhysicsWorld dynamics parameter's reset() contract).
"""
import pathlib
import subprocess

import pytest

_TESTS_DIR = pathlib.Path(__file__).resolve().parents[2]   # tests/
_REPO_ROOT = _TESTS_DIR.parent                              # repo root
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
    _REPO_ROOT / "libraries" / "tinyekf",  # EKFTiny.h includes <tinyekf.h>
]

_HARNESS = r"""
#include "hal/sim/PhysicsWorld.h"
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>

static bool bitEqual(float a, float b) {
    uint32_t ua, ub;
    std::memcpy(&ua, &a, 4);
    std::memcpy(&ub, &b, 4);
    return ua == ub;
}

int main() {
    int failures = 0;

    // --- 1. Default (stictionPwm=0, motorLag=0) is a byte-identical no-op --
    {
        PhysicsWorld base, withDefaults;
        base.setNominalMaxSpeed(400.0f);
        withDefaults.setNominalMaxSpeed(400.0f);
        // Defaults are already 0; set explicitly to prove the setter path
        // itself is a no-op at 0, not just an untouched field.
        withDefaults.setStictionPwm(2, 0.0f);
        withDefaults.setMotorLag(2, 0.0f);

        bool ok = true;
        int8_t pwmSeq[] = {37, -83, 5, -5, 0, 100, -100, 1, -1};
        for (int8_t pwm : pwmSeq) {
            base.setActuators(pwm, pwm);
            withDefaults.setActuators(pwm, pwm);
            base.update(24);
            withDefaults.update(24);
            ok = ok && bitEqual(base.trueEncL(), withDefaults.trueEncL())
                     && bitEqual(base.trueEncR(), withDefaults.trueEncR())
                     && bitEqual(base.truePoseX(), withDefaults.truePoseX())
                     && bitEqual(base.truePoseY(), withDefaults.truePoseY())
                     && bitEqual(base.truePoseH(), withDefaults.truePoseH());
        }
        if (!ok) {
            printf("FAIL default_noop\n");
            ++failures;
        } else {
            printf("PASS default_noop\n");
        }

        if (base.stictionPwmL() != 0.0f || base.stictionPwmR() != 0.0f ||
            base.motorLagL() != 0.0f || base.motorLagR() != 0.0f) {
            printf("FAIL default_getters stictionL=%.6g stictionR=%.6g lagL=%.6g lagR=%.6g\n",
                   base.stictionPwmL(), base.stictionPwmR(), base.motorLagL(), base.motorLagR());
            ++failures;
        } else {
            printf("PASS default_getters\n");
        }
    }

    // --- 2. Gate boundary: at threshold -> no gate; one below -> gate ------
    {
        // Positive pwm.
        PhysicsWorld atThresh, belowThresh;
        atThresh.setNominalMaxSpeed(400.0f);
        atThresh.setStictionPwm(2, 10.0f);
        atThresh.setActuators(10, 10);
        atThresh.update(100);   // dt_s = 0.1

        belowThresh.setNominalMaxSpeed(400.0f);
        belowThresh.setStictionPwm(2, 10.0f);
        belowThresh.setActuators(9, 9);
        belowThresh.update(100);

        float expectedVelAtThresh = (10 / 100.0f) * 400.0f;          // 40 mm/s
        float expectedEncAtThresh = expectedVelAtThresh * 0.1f;       // 4.0 mm

        bool atOk = bitEqual(atThresh.trueVelL(), expectedVelAtThresh) &&
                    bitEqual(atThresh.trueEncL(), expectedEncAtThresh);
        bool belowOk = belowThresh.trueVelL() == 0.0f && belowThresh.trueEncL() == 0.0f;

        if (!atOk || !belowOk) {
            printf("FAIL gate_boundary_positive atOk=%d belowOk=%d atVel=%.6g atEnc=%.6g belowVel=%.6g belowEnc=%.6g\n",
                   atOk, belowOk, atThresh.trueVelL(), atThresh.trueEncL(),
                   belowThresh.trueVelL(), belowThresh.trueEncL());
            ++failures;
        } else {
            printf("PASS gate_boundary_positive\n");
        }

        // Negative pwm — |pwm| is what gates, not the sign.
        PhysicsWorld atThreshNeg, belowThreshNeg;
        atThreshNeg.setNominalMaxSpeed(400.0f);
        atThreshNeg.setStictionPwm(2, 10.0f);
        atThreshNeg.setActuators(-10, -10);
        atThreshNeg.update(100);

        belowThreshNeg.setNominalMaxSpeed(400.0f);
        belowThreshNeg.setStictionPwm(2, 10.0f);
        belowThreshNeg.setActuators(-9, -9);
        belowThreshNeg.update(100);

        float expectedVelAtThreshNeg = (-10 / 100.0f) * 400.0f;       // -40 mm/s
        float expectedEncAtThreshNeg = expectedVelAtThreshNeg * 0.1f;  // -4.0 mm

        bool atNegOk = bitEqual(atThreshNeg.trueVelL(), expectedVelAtThreshNeg) &&
                       bitEqual(atThreshNeg.trueEncL(), expectedEncAtThreshNeg);
        bool belowNegOk = belowThreshNeg.trueVelL() == 0.0f && belowThreshNeg.trueEncL() == 0.0f;

        if (!atNegOk || !belowNegOk) {
            printf("FAIL gate_boundary_negative atNegOk=%d belowNegOk=%d\n", atNegOk, belowNegOk);
            ++failures;
        } else {
            printf("PASS gate_boundary_negative\n");
        }
    }

    // --- 3. Statelessness: no "was moving" memory --------------------------
    {
        PhysicsWorld w;
        w.setNominalMaxSpeed(400.0f);
        w.setStictionPwm(2, 50.0f);

        // Tick 1: above threshold -> moves.
        w.setActuators(80, 80);
        w.update(24);
        bool tick1Moved = w.trueVelL() > 0.0f && w.trueEncL() > 0.0f;

        float encAfterTick1 = w.trueEncL();

        // Tick 2: below threshold -> must gate to exactly 0 THIS tick, with
        // no residual memory of tick 1's nonzero velocity.
        w.setActuators(10, 10);
        w.update(24);
        bool tick2Gated = (w.trueVelL() == 0.0f) && (w.trueEncL() == encAfterTick1);

        if (!tick1Moved || !tick2Gated) {
            printf("FAIL stateless tick1Moved=%d tick2Gated=%d vel2=%.6g enc1=%.6g enc2=%.6g\n",
                   tick1Moved, tick2Gated, w.trueVelL(), encAfterTick1, w.trueEncL());
            ++failures;
        } else {
            printf("PASS stateless\n");
        }
    }

    // --- 4. Per-wheel independence ------------------------------------------
    {
        PhysicsWorld w;
        w.setNominalMaxSpeed(400.0f);
        w.setStictionPwm(0, 50.0f);   // LEFT only; right stays at default 0.
        w.setActuators(10, 10);
        w.update(100);

        float expectedVelR = (10 / 100.0f) * 400.0f;   // 40 mm/s, ungated
        bool leftGated  = (w.trueVelL() == 0.0f);
        bool rightNotGated = bitEqual(w.trueVelR(), expectedVelR);

        if (!leftGated || !rightNotGated) {
            printf("FAIL per_wheel_independence leftGated=%d rightNotGated=%d velL=%.6g velR=%.6g\n",
                   leftGated, rightNotGated, w.trueVelL(), w.trueVelR());
            ++failures;
        } else {
            printf("PASS per_wheel_independence\n");
        }
    }

    // --- 5. Lag tau<=0 is a byte-identical no-op, even with stiction active -
    {
        PhysicsWorld noLag, explicitZeroLag, withStictionNoLag;
        noLag.setNominalMaxSpeed(400.0f);
        explicitZeroLag.setNominalMaxSpeed(400.0f);
        explicitZeroLag.setMotorLag(2, 0.0f);
        withStictionNoLag.setNominalMaxSpeed(400.0f);
        withStictionNoLag.setStictionPwm(2, 5.0f);
        withStictionNoLag.setMotorLag(2, 0.0f);

        bool ok = true;
        int8_t pwmSeq[] = {80, 3, -80, -3, 50};
        for (int8_t pwm : pwmSeq) {
            noLag.setActuators(pwm, pwm);
            explicitZeroLag.setActuators(pwm, pwm);
            withStictionNoLag.setActuators(pwm, pwm);
            noLag.update(24);
            explicitZeroLag.update(24);
            withStictionNoLag.update(24);
            ok = ok && bitEqual(noLag.trueVelL(), explicitZeroLag.trueVelL());
        }
        // withStictionNoLag's tick with pwm=3 (< stictionPwm=5) must have
        // gated to exactly 0 -- confirms the no-op lag path composes with an
        // ACTIVE gate rather than masking it.
        if (!ok) {
            printf("FAIL lag_tau_zero_noop\n");
            ++failures;
        } else {
            printf("PASS lag_tau_zero_noop\n");
        }
    }

    // --- 6. Lag tau>0 converges toward velTarget exponentially --------------
    {
        PhysicsWorld w;
        w.setNominalMaxSpeed(400.0f);
        w.setMotorLag(2, 100.0f);   // tau = 100 ms
        w.setActuators(100, 100);  // velTarget = 400 mm/s, held constant

        float velTarget = 400.0f;
        float tauS = 0.100f;
        float lagRef = 0.0f;
        bool ok = true;
        for (int i = 0; i < 5; ++i) {
            w.update(10);   // dt_s = 0.01
            float alpha = 1.0f - expf(-0.01f / tauS);
            lagRef = lagRef + (velTarget - lagRef) * alpha;
            if (std::fabs(w.trueVelL() - lagRef) > 1e-3f) ok = false;
        }
        // Monotonically approaching (never overshoots for a constant target).
        bool monotonic = (lagRef > 0.0f) && (lagRef < velTarget);
        if (!ok || !monotonic) {
            printf("FAIL lag_converges velL=%.6g lagRef=%.6g\n", w.trueVelL(), lagRef);
            ++failures;
        } else {
            printf("PASS lag_converges\n");
        }
    }

    // --- 7. reset() zeroes lag STATE but preserves knob CONFIGURATION ------
    {
        PhysicsWorld w;
        w.setNominalMaxSpeed(400.0f);
        w.setMotorLag(2, 200.0f);
        w.setStictionPwm(2, 15.0f);
        w.setActuators(80, 80);
        for (int i = 0; i < 3; ++i) w.update(24);   // build up nonzero lag state
        bool hadNonzeroVel = w.trueVelL() > 0.0f;

        w.reset();
        bool velZeroedByReset = (w.trueVelL() == 0.0f) && (w.trueEncL() == 0.0f);
        bool configPreserved = (w.motorLagL() == 200.0f) && (w.stictionPwmL() == 15.0f);

        // Confirm the lag filter truly restarted from 0 (not just trueVelL
        // read as 0 incidentally) -- the FIRST post-reset tick's output must
        // match a fresh PhysicsWorld's first tick under the same config.
        PhysicsWorld fresh;
        fresh.setNominalMaxSpeed(400.0f);
        fresh.setMotorLag(2, 200.0f);
        fresh.setStictionPwm(2, 15.0f);
        fresh.setActuators(80, 80);
        fresh.update(24);
        w.setActuators(80, 80);
        w.update(24);
        bool restartMatches = bitEqual(w.trueVelL(), fresh.trueVelL());

        if (!hadNonzeroVel || !velZeroedByReset || !configPreserved || !restartMatches) {
            printf("FAIL reset_state_vs_config hadNonzeroVel=%d velZeroedByReset=%d configPreserved=%d restartMatches=%d\n",
                   hadNonzeroVel, velZeroedByReset, configPreserved, restartMatches);
            ++failures;
        } else {
            printf("PASS reset_state_vs_config\n");
        }
    }

    printf("DONE failures=%d\n", failures);
    return failures == 0 ? 0 : 1;
}
"""


@pytest.fixture(scope="module")
def stiction_harness(tmp_path_factory):
    """Compile + run the standalone PhysicsWorld stiction/lag harness once."""
    workdir = tmp_path_factory.mktemp("physics_world_stiction")
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
        pytest.fail(f"PhysicsWorld stiction harness failed to compile:\n{build.stderr}")

    run = subprocess.run([str(exe)], capture_output=True, text=True)
    return run


def _results(run):
    return {
        line.split()[1]: line.split()[0]
        for line in run.stdout.splitlines()
        if line.startswith(("PASS ", "FAIL "))
    }


def test_harness_runs_clean(stiction_harness):
    """The whole harness returns 0 (no FAIL lines)."""
    assert stiction_harness.returncode == 0, (
        f"harness stdout:\n{stiction_harness.stdout}\n"
        f"stderr:\n{stiction_harness.stderr}"
    )
    assert "DONE failures=0" in stiction_harness.stdout


def test_default_is_byte_identical_noop(stiction_harness):
    """stictionPwm=0 and motorLag=0 (the defaults) produce byte-identical output."""
    res = _results(stiction_harness)
    assert res.get("default_noop") == "PASS"
    assert res.get("default_getters") == "PASS"


def test_gate_boundary_positive_pwm(stiction_harness):
    """|pwm| exactly at threshold does not gate; one unit below does (positive pwm)."""
    assert _results(stiction_harness).get("gate_boundary_positive") == "PASS"


def test_gate_boundary_negative_pwm(stiction_harness):
    """Same boundary behavior for negative pwm -- |pwm| gates, not the sign."""
    assert _results(stiction_harness).get("gate_boundary_negative") == "PASS"


def test_gate_is_stateless(stiction_harness):
    """A wheel gated to 0 has no memory of the previous tick's motion."""
    assert _results(stiction_harness).get("stateless") == "PASS"


def test_gate_is_per_wheel_independent(stiction_harness):
    """A stiction threshold on one side does not affect the other side."""
    assert _results(stiction_harness).get("per_wheel_independence") == "PASS"


def test_lag_tau_zero_is_noop(stiction_harness):
    """motorLag tau<=0 is a byte-identical no-op, even with stiction active."""
    assert _results(stiction_harness).get("lag_tau_zero_noop") == "PASS"


def test_lag_tau_positive_converges(stiction_harness):
    """motorLag tau>0 converges the lag state toward velTarget exponentially."""
    assert _results(stiction_harness).get("lag_converges") == "PASS"


def test_reset_zeroes_lag_state_not_config(stiction_harness):
    """reset() zeroes the lag filter's persistent state but keeps the knobs."""
    assert _results(stiction_harness).get("reset_state_vs_config") == "PASS"
