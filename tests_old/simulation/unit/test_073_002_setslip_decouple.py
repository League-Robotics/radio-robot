"""
test_073_002_setslip_decouple.py — ticket 073-002 (sim plant scrub reconciliation).

Two independent, file-disjoint fixes land in this ticket (see
clasi/sprints/073-.../architecture-update.md Step 5 "Ticket 002",
Design Rationale Decisions 2 and 3):

  1. `SimHandle`'s constructor (`tests/_infra/sim/sim_api.cpp`) now seeds
     `PhysicsWorld::_bodyRotationalScrub` from the loaded `RobotConfig.
     rotationalSlip` (via `effectiveSlip()`), mirroring the existing
     trackwidth-seed line — a fresh, zero-configuration `Sim()` genuinely
     scrubs by the factor `Planner::beginRotation()`'s arc inflation already
     assumes it does.
  2. `PhysicsWorld::setSlip(straight, turnExtra)` no longer folds `turnExtra`
     into `_rotationalSlip` (the body-truth channel) — it derives
     `_rotationalSlip = straight` only, dropping the accidental coupling
     that let the TestGUI's `slip_turn_extra` control (an encoder-report-only
     knob) perturb body truth.

`test_setslip_decouple` below pins fix 2 directly (a standalone PhysicsWorld
harness, same compile-and-run pattern as `test_physics_world_basic.py` /
`test_physics_world_body_scrub.py`), independent of any end-to-end sim/angle
assertion. `test_simhandle_seeds_body_rot_scrub_from_default_config` pins fix
1's construction-time wiring in isolation (via `SIMGET bodyRotScrub` on a
freshly-constructed `sim` fixture, before any `SIMSET`/`SET` call), so a
construction-wiring bug can be told apart from a formula bug before Ticket
004's combined RT angle-sweep test runs.
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
#include <cstdio>

int main() {
    int failures = 0;

    // --- setSlip(0.0, <nonzero turnExtra>) must NOT perturb body truth ------
    // (073-002 Decision 2: turnExtra is encoder-report-only; the TestGUI's
    // slip_turn_extra control is the only current caller of a nonzero
    // turnExtra, and must no longer be able to reach _rotationalSlip.)
    {
        PhysicsWorld pw;
        pw.setSlip(0.0f, 0.5f);
        bool ok = (pw.rotationalSlip() == 0.0f);
        if (!ok) {
            printf("FAIL setslip_zero_straight_nonzero_turn rotationalSlip=%.9g (expected 0.0)\n",
                   pw.rotationalSlip());
            ++failures;
        } else {
            printf("PASS setslip_zero_straight_nonzero_turn\n");
        }
    }

    // --- Same, with a NEGATIVE turnExtra -------------------------------------
    // firmware.py::set_field_profile() negates slip_turn_extra
    // (sim_set_motor_slip(side=2, straight=0.0, turn_extra=-slip_turn_extra)) --
    // before this ticket, a large negative turnExtra could push
    // _rotationalSlip negative, silently neutralized only by effectiveSlip()'s
    // <=0 clamp rather than the channel being structurally unreachable.
    {
        PhysicsWorld pw;
        pw.setSlip(0.0f, -0.26f);
        bool ok = (pw.rotationalSlip() == 0.0f);
        if (!ok) {
            printf("FAIL setslip_zero_straight_negative_turn rotationalSlip=%.9g (expected 0.0)\n",
                   pw.rotationalSlip());
            ++failures;
        } else {
            printf("PASS setslip_zero_straight_negative_turn\n");
        }
    }

    // --- straight alone still drives _rotationalSlip -------------------------
    // (arithmetic non-effect for every current genuine-body-truth caller,
    // which always passes turnExtra=0.0: test_sim_otos_lever_arm.py,
    // test_physics_world_basic.py, test_physics_world_body_scrub.py.)
    {
        PhysicsWorld pw;
        pw.setSlip(0.7f, 0.0f);
        bool ok = (pw.rotationalSlip() == 0.7f);
        if (!ok) {
            printf("FAIL setslip_straight_only_unaffected rotationalSlip=%.9g (expected 0.7)\n",
                   pw.rotationalSlip());
            ++failures;
        } else {
            printf("PASS setslip_straight_only_unaffected\n");
        }
    }

    printf("DONE failures=%d\n", failures);
    return failures == 0 ? 0 : 1;
}
"""


@pytest.fixture(scope="module")
def setslip_decouple_harness(tmp_path_factory):
    """Compile + run the standalone PhysicsWorld::setSlip() decoupling harness once."""
    workdir = tmp_path_factory.mktemp("setslip_decouple")
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
        pytest.fail(f"setSlip decoupling harness failed to compile:\n{build.stderr}")

    run = subprocess.run([str(exe)], capture_output=True, text=True)
    return run


def _results(run):
    return {
        line.split()[1]: line.split()[0]
        for line in run.stdout.splitlines()
        if line.startswith(("PASS ", "FAIL "))
    }


def test_harness_runs_clean(setslip_decouple_harness):
    """The whole harness returns 0 (no FAIL lines)."""
    assert setslip_decouple_harness.returncode == 0, (
        f"harness stdout:\n{setslip_decouple_harness.stdout}\n"
        f"stderr:\n{setslip_decouple_harness.stderr}"
    )
    assert "DONE failures=0" in setslip_decouple_harness.stdout


def test_setslip_zero_straight_nonzero_turn_extra_yields_zero_rotational_slip(
    setslip_decouple_harness,
):
    """setSlip(0.0, <nonzero>) -> rotationalSlip() == 0.0 (the core acceptance point)."""
    assert (
        _results(setslip_decouple_harness).get("setslip_zero_straight_nonzero_turn")
        == "PASS"
    )


def test_setslip_zero_straight_negative_turn_extra_yields_zero_rotational_slip(
    setslip_decouple_harness,
):
    """Same, for a NEGATIVE turnExtra (the TestGUI's negated slip_turn_extra convention)."""
    assert (
        _results(setslip_decouple_harness).get("setslip_zero_straight_negative_turn")
        == "PASS"
    )


def test_setslip_straight_only_still_drives_rotational_slip(setslip_decouple_harness):
    """turnExtra=0.0 (every current genuine-body-truth caller) is arithmetically unaffected."""
    assert (
        _results(setslip_decouple_harness).get("setslip_straight_only_unaffected")
        == "PASS"
    )


# ---------------------------------------------------------------------------
# SimHandle-construction seed (fix 1): isolates the construction-wiring path
# from the setSlip() formula pinned above, via the real SIMGET wire surface.
# ---------------------------------------------------------------------------

def test_simhandle_seeds_body_rot_scrub_from_default_rotational_slip(sim):
    """A fresh, zero-configuration Sim() seeds bodyRotScrub from
    RobotConfig.rotationalSlip (0.92 default, data/robots/*.json) BEFORE any
    SIMSET/SET call -- effectiveSlip(0.92) == 0.92 (pass-through range), so
    SIMGET bodyRotScrub should read 0.920 on a brand-new Sim.
    """
    reply = sim.send_command("SIMGET bodyRotScrub")
    assert reply.startswith("SIMCFG"), f"SIMGET bodyRotScrub -> {reply!r}"
    assert "bodyRotScrub=0.920" in reply, (
        f"expected the construction-time seed to equal effectiveSlip(0.92) == "
        f"0.92; got {reply!r}"
    )


def test_simhandle_seed_is_overridable_not_a_floor(sim):
    """The construction-time seed is a DEFAULT, not a lock -- SIMSET after
    construction still wins (069's existing setter contract, unaffected).
    """
    reply = sim.send_command("SIMSET bodyRotScrub=1.0")
    assert reply.upper().startswith("OK"), f"SIMSET bodyRotScrub=1.0 -> {reply!r}"

    reply = sim.send_command("SIMGET bodyRotScrub")
    assert reply.startswith("SIMCFG"), f"SIMGET bodyRotScrub -> {reply!r}"
    assert "bodyRotScrub=1.000" in reply, f"override should stick; got {reply!r}"
