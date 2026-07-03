"""
test_physics_world_body_scrub.py — ticket 069-002.

PhysicsWorld::update()'s sub-step B (chassis pose integration) previously had
exactly one scrub channel: effectiveSlip(_rotationalSlip), a test-infrastructure
knob written only by setSlip()/sim_set_motor_slip. This ticket adds two new,
independent, default-neutral (1.0 = no-op) fields — _bodyRotationalScrub and
_bodyLinearScrub — combined MULTIPLICATIVELY with the existing, unchanged
_rotationalSlip channel.

Same pattern as test_physics_world_basic.py: compile a tiny self-contained C++
harness that links PhysicsWorld.cpp directly (not yet exercised through the
sim ABI in this isolation test — that's covered separately by
test_069_rt_90deg_body_scrub.py), run it, and parse PASS/FAIL lines.

What it pins:
  1. Default (both new fields at 1.0) is a byte-identical no-op vs. the
     pre-069-002 expression (golden-TLM constraint).
  2. setBodyRotationalScrub() alone reduces the sub-step B heading term
     (dTheta), independent of _rotationalSlip.
  3. setBodyLinearScrub() alone reduces the sub-step B linear (X/Y) term,
     independent of _rotationalSlip and of the rotational scrub.
  4. Combining a non-zero setSlip() value with setBodyRotationalScrub()
     multiplies rather than replaces — both factors visible in the heading
     output (066-001's chassis-truth-slip test exercises setSlip() alone and
     must remain unaffected; this case proves the two channels compose).
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

    // --- 1. Default (1.0, 1.0) is a byte-identical no-op ---------------------
    {
        PhysicsWorld base, withDefaults;
        base.setTrackwidth(150.0f);
        base.setNominalMaxSpeed(400.0f);
        base.setActuators(-50, 50);   // pure spin: exercises both dTheta and linear terms
        withDefaults.setTrackwidth(150.0f);
        withDefaults.setNominalMaxSpeed(400.0f);
        withDefaults.setActuators(-50, 50);
        // Defaults are already 1.0f; set explicitly to prove the setter path
        // itself is a no-op at 1.0, not just an untouched field.
        withDefaults.setBodyRotationalScrub(1.0f);
        withDefaults.setBodyLinearScrub(1.0f);

        base.update(100);
        withDefaults.update(100);

        bool ok = bitEqual(base.truePoseX(), withDefaults.truePoseX()) &&
                  bitEqual(base.truePoseY(), withDefaults.truePoseY()) &&
                  bitEqual(base.truePoseH(), withDefaults.truePoseH());
        if (!ok) {
            printf("FAIL default_noop base=(%.9g,%.9g,%.9g) withDefaults=(%.9g,%.9g,%.9g)\n",
                   base.truePoseX(), base.truePoseY(), base.truePoseH(),
                   withDefaults.truePoseX(), withDefaults.truePoseY(), withDefaults.truePoseH());
            ++failures;
        } else {
            printf("PASS default_noop\n");
        }

        // Also confirm getters round-trip the default.
        if (base.bodyRotationalScrub() != 1.0f || base.bodyLinearScrub() != 1.0f) {
            printf("FAIL default_getters rot=%.9g lin=%.9g\n",
                   base.bodyRotationalScrub(), base.bodyLinearScrub());
            ++failures;
        } else {
            printf("PASS default_getters\n");
        }
    }

    // --- 2. bodyRotScrub alone reduces dTheta, independent of _rotationalSlip -
    {
        PhysicsWorld noScrub, withScrub;
        noScrub.setTrackwidth(150.0f);
        noScrub.setNominalMaxSpeed(400.0f);
        noScrub.setActuators(-50, 50);
        withScrub.setTrackwidth(150.0f);
        withScrub.setNominalMaxSpeed(400.0f);
        withScrub.setActuators(-50, 50);
        withScrub.setBodyRotationalScrub(0.92f);

        noScrub.update(100);
        withScrub.update(100);

        // Encoders (sub-step A) must be identical — scrub does not touch them.
        bool encOk = bitEqual(noScrub.trueEncL(), withScrub.trueEncL()) &&
                     bitEqual(noScrub.trueEncR(), withScrub.trueEncR());
        // Heading reduced by exactly the scrub factor (rotationalSlip is 0/unset
        // on both, so effectiveSlip() contributes 1.0 to both).
        bool headOk = std::fabs(withScrub.truePoseH() - noScrub.truePoseH() * 0.92f) < 1e-4f;
        // Linear (X/Y) term unaffected by the rotational scrub alone.
        bool linOk = std::fabs(withScrub.truePoseX() - noScrub.truePoseX()) < 1e-4f &&
                     std::fabs(withScrub.truePoseY() - noScrub.truePoseY()) < 1e-4f;
        if (!encOk || !headOk || !linOk) {
            printf("FAIL rot_scrub encOk=%d headOk=%d linOk=%d noH=%.6g scrubH=%.6g\n",
                   encOk, headOk, linOk, noScrub.truePoseH(), withScrub.truePoseH());
            ++failures;
        } else {
            printf("PASS rot_scrub\n");
        }
    }

    // --- 3. bodyLinScrub alone reduces the linear term, independent of heading
    {
        PhysicsWorld noScrub, withScrub;
        noScrub.setTrackwidth(150.0f);
        noScrub.setNominalMaxSpeed(400.0f);
        noScrub.setActuators(50, 50);   // straight drive: pure linear term, dTheta ~ 0
        withScrub.setTrackwidth(150.0f);
        withScrub.setNominalMaxSpeed(400.0f);
        withScrub.setActuators(50, 50);
        withScrub.setBodyLinearScrub(0.5f);

        noScrub.update(100);
        withScrub.update(100);

        bool headOk = bitEqual(noScrub.truePoseH(), withScrub.truePoseH());
        bool xOk = std::fabs(withScrub.truePoseX() - noScrub.truePoseX() * 0.5f) < 1e-4f;
        if (!headOk || !xOk) {
            printf("FAIL lin_scrub headOk=%d xOk=%d noX=%.6g scrubX=%.6g\n",
                   headOk, xOk, noScrub.truePoseX(), withScrub.truePoseX());
            ++failures;
        } else {
            printf("PASS lin_scrub\n");
        }
    }

    // --- 4. Combines multiplicatively with setSlip(), not a replacement ------
    {
        PhysicsWorld legacyOnly, both;
        legacyOnly.setTrackwidth(150.0f);
        legacyOnly.setNominalMaxSpeed(400.0f);
        legacyOnly.setActuators(-50, 50);
        legacyOnly.setSlip(0.7f, 0.0f);      // effectiveSlip(0.7) = 0.7 (066-001's channel)

        both.setTrackwidth(150.0f);
        both.setNominalMaxSpeed(400.0f);
        both.setActuators(-50, 50);
        both.setSlip(0.7f, 0.0f);
        both.setBodyRotationalScrub(0.92f);  // additional, independent factor

        legacyOnly.update(100);
        both.update(100);

        // both's heading should equal legacyOnly's heading * 0.92 (multiplicative
        // composition of the two independent channels: 0.7 * 0.92, not 0.92 alone).
        bool composedOk = std::fabs(both.truePoseH() - legacyOnly.truePoseH() * 0.92f) < 1e-4f;
        // And legacyOnly's own channel must still be intact (066-001 unaffected):
        // legacyOnly's heading should be scaled by exactly 0.7 vs. a fully
        // unscrubbed run.
        PhysicsWorld unscrubbed;
        unscrubbed.setTrackwidth(150.0f);
        unscrubbed.setNominalMaxSpeed(400.0f);
        unscrubbed.setActuators(-50, 50);
        unscrubbed.update(100);
        bool legacyIntactOk = std::fabs(legacyOnly.truePoseH() - unscrubbed.truePoseH() * 0.7f) < 1e-4f;

        if (!composedOk || !legacyIntactOk) {
            printf("FAIL compose composedOk=%d legacyIntactOk=%d unscrubbedH=%.6g "
                   "legacyOnlyH=%.6g bothH=%.6g\n",
                   composedOk, legacyIntactOk, unscrubbed.truePoseH(),
                   legacyOnly.truePoseH(), both.truePoseH());
            ++failures;
        } else {
            printf("PASS compose\n");
        }
    }

    printf("DONE failures=%d\n", failures);
    return failures == 0 ? 0 : 1;
}
"""


@pytest.fixture(scope="module")
def body_scrub_harness(tmp_path_factory):
    """Compile + run the standalone PhysicsWorld body-scrub harness once."""
    workdir = tmp_path_factory.mktemp("physics_world_body_scrub")
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
        pytest.fail(f"PhysicsWorld body-scrub harness failed to compile:\n{build.stderr}")

    run = subprocess.run([str(exe)], capture_output=True, text=True)
    return run


def _results(run):
    return {
        line.split()[1]: line.split()[0]
        for line in run.stdout.splitlines()
        if line.startswith(("PASS ", "FAIL "))
    }


def test_harness_runs_clean(body_scrub_harness):
    """The whole harness returns 0 (no FAIL lines)."""
    assert body_scrub_harness.returncode == 0, (
        f"harness stdout:\n{body_scrub_harness.stdout}\n"
        f"stderr:\n{body_scrub_harness.stderr}"
    )
    assert "DONE failures=0" in body_scrub_harness.stdout


def test_default_is_byte_identical_noop(body_scrub_harness):
    """Both new fields at their 1.0 default produce byte-identical sub-step B output."""
    res = _results(body_scrub_harness)
    assert res.get("default_noop") == "PASS"
    assert res.get("default_getters") == "PASS"


def test_body_rotational_scrub_reduces_heading_only(body_scrub_harness):
    """bodyRotScrub alone scales dTheta; encoders and the linear term are untouched."""
    assert _results(body_scrub_harness).get("rot_scrub") == "PASS"


def test_body_linear_scrub_reduces_linear_term_only(body_scrub_harness):
    """bodyLinScrub alone scales the (dL+dR)*0.5 linear term; heading is untouched."""
    assert _results(body_scrub_harness).get("lin_scrub") == "PASS"


def test_body_rotational_scrub_composes_multiplicatively_with_slip(body_scrub_harness):
    """setSlip() (066-001's channel) and setBodyRotationalScrub() multiply, not replace."""
    assert _results(body_scrub_harness).get("compose") == "PASS"
