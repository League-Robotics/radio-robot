"""src/tests/sim/system/test_sim_boot_config_parity.py -- ticket 113-007's
own headline proof: sprint 113's stated goal is "a headless
robot_radio.io.sim_loop.SimLoop run and the TestGUI's Sim transport both run
the *same* configuration a real robot reflash would get from the same
[JSON] file." Every prior ticket (001-006) built one piece of the delivery
mechanism; THIS file is the test that directly asserts the mechanism closes
the gap -- SUC-001/SUC-002 ("golden parity") and SUC-003 ("robot-switch
fully replaces, not merges").

Rewritten 115-009 (gut S1's own test-sweep/green-bar ticket): the
`msg::PlannerConfig`/`sim_configure_planner()`/`sim_read_planner_config()`
half of this file's original golden-parity coverage (SUC-001's own
`PlannerConfig` half, and SUC-004's `model_tau_lin`/`model_tau_ang` check)
is DELETED, not ported -- `msg::PlannerConfig` itself, and the
`SimLoop.read_planner_config()` readback this file drove, went with
`Motion::Executor`/`App::Pilot` (115-003). Only the MOTOR half of the
Tier-2 golden-parity mechanism (`sim_configure_motor()`/
`sim_read_motor_config()` -- unaffected by the gut) survives, plus a
motor-config-driven re-derivation of the robot-switch replace-not-merge
proof (SUC-003), which is a property of `configure_from_robot()` itself,
independent of which config fields happen to carry it.

Golden-parity design (SUC-001/SUC-002, motor half only)
--------------------------------------------------------
For each of ``tovez_nocal.json`` and ``tovez.json``:

  (a) Compute the EXPECTED ``MotorConfig`` values (``vel_filt_alpha``/
      ``fwd_sign``) by calling ``gen_boot_config.py``'s own mapping
      functions DIRECTLY (``vel_gains_for_config()``, ``fwd_sign_for_ports()``)
      -- deliberately NOT via ``robot_radio.calibration.sim_boot_config``'s
      ``motor_boot_config_for()`` (ticket 004's own wrapper). This is
      sprint.md ticket 007's own explicit design constraint: asserting
      against ticket 004's wrapper would let a bug in that wrapper's own
      plumbing hide behind testing itself -- this file re-derives
      "expected" independently, using only the same generator functions
      ``gen_boot_config.py``'s ``generate()`` itself calls to bake a real
      robot's ``boot_config.cpp``.
  (b) Construct a HEADLESS ``SimLoop`` (``start_tick_thread=False``, ticket
      009's own deterministic-stepping precedent -- no ``SimTransport``, no
      Qt), call ``configure_from_robot()`` with the SAME file's
      ``RobotConfig`` -- this drives the FULL pipeline
      (``SimLoop.configure_from_robot()`` -> Tier 1 wire push +
      ``sim_boot_config.py``'s Tier-2 motor mapping -> ``sim_configure_motor()``
      ctypes -> ``SimHarness::configureMotor()``), then read back the LIVE
      config via ``sim_read_motor_config()`` (``SimLoop.read_motor_config()``).
  (c) Assert every Tier-2 field matches (a) field-for-field, with a small
      float tolerance for the double(Python)->float32(wire struct)->
      double(Python) round trip every field takes through the ctypes
      boundary.

Run with::

    uv run python -m pytest src/tests/sim/system/test_sim_boot_config_parity.py -v -s

Requires the compiled ``src/sim/build/libfirmware_host.{dylib,so}``
(``python build.py`` or ``cmake --build src/sim/build``) -- skips cleanly if
not present.
"""
from __future__ import annotations

import json
import pathlib
import sys

import pytest

# src/tests/sim/system/test_sim_boot_config_parity.py -> system -> sim ->
# tests -> src -> repo root = FOUR hops from __file__ (same convention
# test_sim_configure_from_robot.py's own header already establishes).
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"
_SCRIPTS_DIR = _REPO_ROOT / "src" / "scripts"

_TOVEZ_NOCAL_JSON = _ROBOTS_DIR / "tovez_nocal.json"
_TOVEZ_JSON = _ROBOTS_DIR / "tovez.json"

if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

import gen_boot_config as gbc  # noqa: E402  (path must be set up before this import)

_LIB_NAME = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
_SIM_LIB_PATH = _REPO_ROOT / "src" / "sim" / "build" / _LIB_NAME

pytestmark = pytest.mark.skipif(
    not _SIM_LIB_PATH.exists(),
    reason="sim lib not built -- cmake --build src/sim/build (or `python build.py`)",
)

_TRACK_WIDTH = 128.0  # [mm] matches both tovez.json AND tovez_nocal.json's own geometry.trackwidth

# Float tolerance for the double(Python)->float32(C struct)->double(Python)
# round trip every field takes through configureMotor()/motorConfig() and
# the ctypes marshaling on both sides -- observed drift is ~1e-6..1e-8 in
# practice (float32 rounding only); 1e-4 stays generously inside that while
# still catching a genuinely wrong value (a real mismatch is a different
# NUMBER, not a rounding-sized delta).
_APPROX = dict(rel=1e-4, abs=1e-4)


def _make_loop():
    """A bare, headless ``SimLoop`` -- deterministic manual stepping
    (``start_tick_thread=False``), no ``SimTransport``, no Qt. Mirrors
    ``test_sim_configure_from_robot.py``'s own ``_make_loop()`` helper."""
    from robot_radio.io.sim_loop import SimLoop

    loop = SimLoop(track_width=_TRACK_WIDTH, lib_path=_SIM_LIB_PATH)
    loop.connect(start_tick_thread=False)
    return loop


def _expected_motor_config(cfg: dict, port: int) -> "dict[str, float | int]":
    """``{"vel_filt_alpha": ..., "fwd_sign": ...}`` for *port* (1=left,
    2=right), computed directly from gen_boot_config.py's
    ``vel_gains_for_config()``/``fwd_sign_for_ports()`` -- see module
    docstring for why this is a separate call site from
    ``sim_boot_config.motor_boot_config_for()``."""
    *_gains, filt_alpha = gbc.vel_gains_for_config(cfg)
    fwd_signs = gbc.fwd_sign_for_ports(cfg)
    return {"vel_filt_alpha": filt_alpha, "fwd_sign": fwd_signs[port - 1]}


# ---------------------------------------------------------------------------
# Golden parity (SUC-001/SUC-002, motor half)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("robot_json", [_TOVEZ_NOCAL_JSON, _TOVEZ_JSON], ids=lambda p: p.stem)
@pytest.mark.parametrize("port", [1, 2], ids=["left", "right"])
def test_golden_parity_motor_config(robot_json, port):
    """A headless SimLoop's live per-motor vel_filt_alpha/fwd_sign, after
    configure_from_robot(), matches gen_boot_config.py's own directly-computed
    values for both drive-pair ports (1=left, 2=right)."""
    from robot_radio.config.robot_config import load_robot_config

    raw_cfg = json.loads(robot_json.read_text())
    expected = _expected_motor_config(raw_cfg, port)

    robot_config = load_robot_config(robot_json)
    loop = _make_loop()
    try:
        loop.configure_from_robot(robot_config)
        actual = loop.read_motor_config(port)
    finally:
        loop.disconnect()

    assert actual["fwd_sign"] == expected["fwd_sign"], (
        f"{robot_json.name} port {port}: fwd_sign mismatch -- "
        f"gen_boot_config.py says {expected['fwd_sign']!r}, sim readback says "
        f"{actual['fwd_sign']!r}"
    )
    assert actual["vel_filt_alpha"] == pytest.approx(expected["vel_filt_alpha"], **_APPROX), (
        f"{robot_json.name} port {port}: vel_filt_alpha mismatch -- "
        f"gen_boot_config.py says {expected['vel_filt_alpha']!r}, sim readback "
        f"says {actual['vel_filt_alpha']!r}"
    )


# ---------------------------------------------------------------------------
# Robot-switch (SUC-003): a mid-session profile switch fully REPLACES config,
# not merges it. Re-derived against the surviving MOTOR config (115-009 --
# the original discriminating fields, heading_kp/distance_kp, were
# PlannerConfig fields deleted by 115-003); the replace-not-merge property
# itself is a property of configure_from_robot(), independent of which
# config fields happen to carry it.
# ---------------------------------------------------------------------------

def test_robot_switch_replaces_not_merges():
    """Connect once against tovez_nocal.json, read back motor config, call
    configure_from_robot() again with tovez.json, read back again -- the
    second reading must reflect tovez.json's OWN values, not a merge of
    both. vel_filt_alpha (1.0 nocal vs 0.3 tovez, both ports) is the
    discriminating field -- it differs meaningfully between the two
    profiles' own JSON, so an unreplaced leftover from the FIRST
    configure_from_robot() call would be caught."""
    from robot_radio.config.robot_config import load_robot_config

    nocal_cfg = json.loads(_TOVEZ_NOCAL_JSON.read_text())
    tovez_cfg = json.loads(_TOVEZ_JSON.read_text())
    expected_nocal = _expected_motor_config(nocal_cfg, port=1)
    expected_tovez = _expected_motor_config(tovez_cfg, port=1)

    # The two profiles must actually diverge on this field for this test to
    # prove anything -- guard against a future edit to either JSON silently
    # collapsing the discrimination this test relies on.
    assert expected_nocal["vel_filt_alpha"] != pytest.approx(expected_tovez["vel_filt_alpha"])

    robot_config_nocal = load_robot_config(_TOVEZ_NOCAL_JSON)
    robot_config_tovez = load_robot_config(_TOVEZ_JSON)

    loop = _make_loop()
    try:
        loop.configure_from_robot(robot_config_nocal)
        first = loop.read_motor_config(1)
        assert first["vel_filt_alpha"] == pytest.approx(expected_nocal["vel_filt_alpha"], **_APPROX)

        loop.configure_from_robot(robot_config_tovez)
        second = loop.read_motor_config(1)
    finally:
        loop.disconnect()

    assert second["vel_filt_alpha"] == pytest.approx(expected_tovez["vel_filt_alpha"], **_APPROX), (
        f"post-switch vel_filt_alpha mismatch -- expected tovez.json's own "
        f"value {expected_tovez['vel_filt_alpha']!r} (a full replace), got "
        f"{second['vel_filt_alpha']!r} (possible leftover from the first "
        f"configure_from_robot(tovez_nocal) call -- a merge, not a replace)"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
