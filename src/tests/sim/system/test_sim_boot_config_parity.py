"""src/tests/sim/system/test_sim_boot_config_parity.py -- ticket 113-007's
own headline proof: sprint 113's stated goal is "a headless
robot_radio.io.sim_loop.SimLoop run and the TestGUI's Sim transport both run
the *same* configuration a real robot reflash would get from the same
[JSON] file." Every prior ticket (001-006) built one piece of the delivery
mechanism; THIS file is the test that directly asserts the mechanism closes
the gap -- SUC-001/SUC-002 ("golden parity"), SUC-003 ("robot-switch fully
replaces, not merges"), and SUC-004 (model_tau_lin/model_tau_ang are
JSON-driven).

Golden-parity design (SUC-001/SUC-002)
---------------------------------------
For each of ``tovez_nocal.json`` and ``tovez.json``:

  (a) Compute the EXPECTED ``PlannerConfig``/``MotorConfig`` values by
      calling ``gen_boot_config.py``'s own mapping functions DIRECTLY
      (``heading_gains_for_config()``, ``model_tau_for_config()``, etc.) --
      deliberately NOT via ``robot_radio.calibration.sim_boot_config``'s
      ``planner_boot_config_for()``/``motor_boot_config_for()`` (ticket
      004's own wrapper). This is sprint.md ticket 007's own explicit
      design constraint: asserting against ticket 004's wrapper would let a
      bug in that wrapper's own plumbing hide behind testing itself -- this
      file re-derives "expected" independently, using only the same
      generator functions ``gen_boot_config.py``'s ``generate()`` itself
      calls to bake a real robot's ``boot_config.cpp``.
  (b) Construct a HEADLESS ``SimLoop`` (``start_tick_thread=False``, ticket
      009's own deterministic-stepping precedent -- no ``SimTransport``, no
      Qt), call ``configure_from_robot()`` with the SAME file's
      ``RobotConfig`` -- this drives the FULL pipeline
      (``SimLoop.configure_from_robot()`` -> Tier 1 wire push +
      ``sim_boot_config.py``'s Tier-2 mapping -> ``sim_configure_planner()``/
      ``sim_configure_motor()`` ctypes -> ``SimHarness::configurePlanner()``/
      ``configureMotor()``), then read back the LIVE config via this
      ticket's own new ``sim_read_planner_config()``/``sim_read_motor_config()``
      ctypes exports (``SimLoop.read_planner_config()``/``read_motor_config()``
      -- added by this ticket; no Python-reachable readback of the live sim
      config existed before it, only ticket 002's own C++-level
      ``SimHarness::plannerConfig()``/``motorConfig()`` accessors, exercised
      only by that ticket's own C++ harness test).
  (c) Assert every Tier-2 field matches (a) field-for-field, with a small
      float tolerance for the double(Python)->float32(wire struct)->
      double(Python) round trip every field takes through the ctypes
      boundary.

Run with::

    uv run python -m pytest src/tests/sim/system/test_sim_boot_config_parity.py -v -s

Requires the compiled ``src/sim/build/libfirmware_host.{dylib,so}``
(``python build.py`` or ``cmake --build src/sim/build``) -- skips cleanly if
not present. This ticket ADDED two new ctypes exports
(``sim_read_planner_config``/``sim_read_motor_config``, ``sim_ctypes.cpp``)
that a stale prebuilt lib would not have -- rebuild if this file's tests
fail with an ``AttributeError`` on those symbols.
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
# round trip every field takes through configurePlanner()/plannerConfig()
# and the ctypes marshaling on both sides -- observed drift is ~1e-6..1e-8
# in practice (float32 rounding only); 1e-4 stays generously inside that
# while still catching a genuinely wrong value (a real mismatch is a
# different NUMBER, not a rounding-sized delta).
_APPROX = dict(rel=1e-4, abs=1e-4)


def _make_loop():
    """A bare, headless ``SimLoop`` -- deterministic manual stepping
    (``start_tick_thread=False``), no ``SimTransport``, no Qt. Mirrors
    ``test_sim_configure_from_robot.py``'s own ``_make_loop()`` helper."""
    from robot_radio.io.sim_loop import SimLoop

    loop = SimLoop(track_width=_TRACK_WIDTH, lib_path=_SIM_LIB_PATH)
    loop.connect(start_tick_thread=False)
    return loop


def _heading_source_wire_value(cfg: dict) -> int:
    """Resolve ``gen_boot_config.py``'s own ``heading_source_for_config()``
    C++ enumerator-literal STRING (e.g.
    ``"msg::HeadingSourceMode::HEADING_SOURCE_AUTO"``) to its wire int value,
    via the same generated ``planner_pb2.HeadingSourceMode`` descriptor the
    real wire/ctypes path uses. Deliberately a SEPARATE, independently
    written two-line resolver from ``sim_boot_config._heading_source_wire_value()``
    (ticket 004's own copy) -- this file asserts against gen_boot_config.py
    DIRECTLY (see module docstring); duplicating this trivial name->int
    lookup here, rather than importing ticket 004's copy, means a bug in
    THAT copy cannot hide behind this test reusing it."""
    from robot_radio.robot.pb2 import planner_pb2

    literal = gbc.heading_source_for_config(cfg)  # e.g. "msg::HeadingSourceMode::HEADING_SOURCE_AUTO"
    member_name = literal.rsplit("::", 1)[-1]
    return planner_pb2.HeadingSourceMode.Value(member_name)


def _expected_planner_config(cfg: dict) -> "dict[str, float | int]":
    """Every Tier-2 msg::PlannerConfig scalar this sprint covers, computed
    DIRECTLY from gen_boot_config.py's own functions -- see module
    docstring for why this must NOT go through sim_boot_config.py's
    planner_boot_config_for() wrapper. Field set and shape mirror that
    wrapper's own (ticket 004's acceptance criteria enumerate the same 21
    fields) but this is an independent call site."""
    out: "dict[str, float | int]" = {}
    (out["a_max"], out["a_decel"], out["v_body_max"], out["j_max"],
     out["yaw_jerk_max"]) = gbc.motion_limits_for_config(cfg)
    out["yaw_rate_max"], out["yaw_acc_max"] = gbc.profile_rot_limits_for_config(cfg)
    out["min_speed"] = gbc.min_speed_for_config(cfg)
    out["heading_kp"], out["heading_kd"] = gbc.heading_gains_for_config(cfg)
    out["arrive_dwell"] = gbc.arrive_dwell_for_config(cfg)
    out["heading_source"] = _heading_source_wire_value(cfg)
    out["heading_dwell_tol"], out["heading_dwell_rate"] = gbc.heading_dwell_for_config(cfg)
    (out["heading_lead_bias"], out["plan_lead"],
     out["terminal_lead"]) = gbc.lead_compensation_for_config(cfg)
    out["actuation_lag"] = gbc.actuation_lag_for_config(cfg)
    out["distance_kp"], out["distance_tol"] = gbc.distance_gains_for_config(cfg)
    out["model_tau_lin"], out["model_tau_ang"] = gbc.model_tau_for_config(cfg)
    return out


def _expected_motor_config(cfg: dict, port: int) -> "dict[str, float | int]":
    """``{"vel_filt_alpha": ..., "fwd_sign": ...}`` for *port* (1=left,
    2=right), computed directly from gen_boot_config.py's
    ``vel_gains_for_config()``/``fwd_sign_for_ports()`` -- see
    ``_expected_planner_config()``'s own docstring for why this is a
    separate call site from ``sim_boot_config.motor_boot_config_for()``."""
    *_gains, filt_alpha = gbc.vel_gains_for_config(cfg)
    fwd_signs = gbc.fwd_sign_for_ports(cfg)
    return {"vel_filt_alpha": filt_alpha, "fwd_sign": fwd_signs[port - 1]}


# ---------------------------------------------------------------------------
# Golden parity (SUC-001/SUC-002)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("robot_json", [_TOVEZ_NOCAL_JSON, _TOVEZ_JSON], ids=lambda p: p.stem)
def test_golden_parity_planner_config(robot_json):
    """A headless SimLoop's live PlannerConfig, after configure_from_robot(),
    matches gen_boot_config.py's own directly-computed values field-for-field
    -- the sprint's headline proof, for both the neutral/no-cal profile and
    the historically bench-tuned profile (two profiles whose control blocks
    diverge on several fields -- see test_robot_switch_replaces_not_merges
    below -- so this is not a single-profile coincidence)."""
    from robot_radio.config.robot_config import load_robot_config

    raw_cfg = json.loads(robot_json.read_text())
    expected = _expected_planner_config(raw_cfg)

    robot_config = load_robot_config(robot_json)
    loop = _make_loop()
    try:
        loop.configure_from_robot(robot_config)
        actual = loop.read_planner_config()
    finally:
        loop.disconnect()

    assert set(actual) == set(expected), (
        f"field-set mismatch between sim readback and gen_boot_config.py's "
        f"own field list: only-in-actual={set(actual) - set(expected)} "
        f"only-in-expected={set(expected) - set(actual)}"
    )
    for field, expected_value in expected.items():
        assert actual[field] == pytest.approx(expected_value, **_APPROX), (
            f"{robot_json.name}: PlannerConfig.{field} mismatch -- "
            f"gen_boot_config.py says {expected_value!r}, sim readback says "
            f"{actual[field]!r} after configure_from_robot()"
        )


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
# not merges it.
# ---------------------------------------------------------------------------

def test_robot_switch_replaces_not_merges():
    """Connect once against tovez_nocal.json, read back config, call
    configure_from_robot() again with tovez.json, read back again -- the
    second reading must reflect tovez.json's OWN values, not a merge of
    both. heading_kp (2.5 nocal vs 6.0 tovez) and distance_kp (2.5 nocal vs
    the 8.0 firmware-default fallback tovez.json itself doesn't override)
    are used as the discriminating fields -- both differ meaningfully
    between the two profiles' own JSON, so an unreplaced leftover from the
    FIRST configure_from_robot() call would be caught."""
    from robot_radio.config.robot_config import load_robot_config

    nocal_cfg = json.loads(_TOVEZ_NOCAL_JSON.read_text())
    tovez_cfg = json.loads(_TOVEZ_JSON.read_text())
    expected_nocal = _expected_planner_config(nocal_cfg)
    expected_tovez = _expected_planner_config(tovez_cfg)

    # The two profiles must actually diverge on these fields for this test
    # to prove anything -- guard against a future edit to either JSON
    # silently collapsing the discrimination this test relies on.
    assert expected_nocal["heading_kp"] != pytest.approx(expected_tovez["heading_kp"])
    assert expected_nocal["distance_kp"] != pytest.approx(expected_tovez["distance_kp"])

    robot_config_nocal = load_robot_config(_TOVEZ_NOCAL_JSON)
    robot_config_tovez = load_robot_config(_TOVEZ_JSON)

    loop = _make_loop()
    try:
        loop.configure_from_robot(robot_config_nocal)
        first = loop.read_planner_config()
        assert first["heading_kp"] == pytest.approx(expected_nocal["heading_kp"], **_APPROX)
        assert first["distance_kp"] == pytest.approx(expected_nocal["distance_kp"], **_APPROX)

        loop.configure_from_robot(robot_config_tovez)
        second = loop.read_planner_config()
    finally:
        loop.disconnect()

    for field, expected_value in expected_tovez.items():
        assert second[field] == pytest.approx(expected_value, **_APPROX), (
            f"post-switch PlannerConfig.{field} mismatch -- expected "
            f"tovez.json's own value {expected_value!r} (a full replace), got "
            f"{second[field]!r} (possible leftover from the first "
            f"configure_from_robot(tovez_nocal) call -- a merge, not a replace)"
        )


# ---------------------------------------------------------------------------
# model_tau_lin/model_tau_ang (SUC-004) -- called out explicitly per the
# ticket's own acceptance criterion, in addition to being covered inside
# test_golden_parity_planner_config above.
# ---------------------------------------------------------------------------

def test_model_tau_parity_tovez_nocal():
    """The sim's live modelTauLin_/modelTauAng_ (App::Pilot's own
    reference-model plant-lag time constants), after configure_from_robot(),
    match tovez_nocal.json's real control.model_tau_lin/control.model_tau_ang
    (0.1/0.08) -- SUC-004's own literal acceptance value."""
    from robot_radio.config.robot_config import load_robot_config

    raw_cfg = json.loads(_TOVEZ_NOCAL_JSON.read_text())
    assert raw_cfg["control"]["model_tau_lin"] == 0.1
    assert raw_cfg["control"]["model_tau_ang"] == 0.08

    robot_config = load_robot_config(_TOVEZ_NOCAL_JSON)
    loop = _make_loop()
    try:
        loop.configure_from_robot(robot_config)
        actual = loop.read_planner_config()
    finally:
        loop.disconnect()

    assert actual["model_tau_lin"] == pytest.approx(0.1, **_APPROX)
    assert actual["model_tau_ang"] == pytest.approx(0.08, **_APPROX)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
