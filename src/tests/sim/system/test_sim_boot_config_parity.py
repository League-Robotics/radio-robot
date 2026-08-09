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
`Motion::Executor`/`Core::Pilot` (115-003). Only the MOTOR half of the
Tier-2 golden-parity mechanism (`sim_configure_motor()`/
`sim_read_motor_config()` -- unaffected by the gut) survives.

Rewritten AGAIN 125-003 (Devices::MotorConfig::velFiltAlpha deleted --
sprint.md Decision 2, "protection vs. control": the EMA velocity estimator
it fed was measurement conditioning, deleted outright pending ticket 004's
Core::WheelObserver, not relocated with a config surface of its own yet).
`vel_filt_alpha` was this file's ONLY discriminating field for BOTH the
golden-parity check and the robot-switch replace-not-merge proof (SUC-003)
-- `fwd_sign` is identical across every port on both `tovez_nocal.json` and
`tovez.json` (verified: `[1, -1, 1, 1]` on both), so it cannot stand in as a
replacement discriminator. `test_robot_switch_replaces_not_merges` is
therefore DELETED, not adapted -- exactly the precedent this file's own
history already set for `PlannerConfig`'s `heading_kp`/`distance_kp`
discriminators above. Golden-parity (`test_golden_parity_motor_config`)
survives, narrowed to `fwd_sign` only. If ticket 004's Core::WheelObserver
(or a later ticket) introduces its own live Tier-2 config surface, THAT
ticket should add its own golden-parity/robot-switch coverage here rather
than resurrecting `vel_filt_alpha`'s.

Golden-parity design (SUC-001/SUC-002, fwd_sign only)
--------------------------------------------------------
For each of ``tovez_nocal.json`` and ``tovez.json``:

  (a) Compute the EXPECTED ``fwd_sign`` by calling ``gen_boot_config.py``'s
      own mapping function DIRECTLY (``fwd_sign_for_ports()``) --
      deliberately NOT via ``robot_radio.calibration.sim_boot_config``'s
      ``motor_boot_config_for()`` (ticket 004's own wrapper). This is
      sprint.md ticket 007's own explicit design constraint: asserting
      against ticket 004's wrapper would let a bug in that wrapper's own
      plumbing hide behind testing itself -- this file re-derives
      "expected" independently, using only the same generator function
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
  (c) Assert fwd_sign matches (a).

Run with::

    uv run python -m pytest src/tests/sim/system/test_sim_boot_config_parity.py -v -s

Requires the compiled ``src/firm/platform/host/build/libfirmware_host.{dylib,so}``
(``python build.py`` or ``cmake --build src/firm/platform/host/build``) -- skips cleanly if
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
_SIM_LIB_PATH = _REPO_ROOT / "src" / "firm" / "platform" / "host" / "build" / _LIB_NAME

pytestmark = pytest.mark.skipif(
    not _SIM_LIB_PATH.exists(),
    reason="sim lib not built -- cmake --build src/firm/platform/host/build (or `python build.py`)",
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


def _expected_motor_config(cfg: dict, port: int) -> "dict[str, int]":
    """``{"fwd_sign": ...}`` for *port* (1=left, 2=right), computed directly
    from gen_boot_config.py's ``fwd_sign_for_ports()`` -- see module
    docstring for why this is a separate call site from
    ``sim_boot_config.motor_boot_config_for()``."""
    fwd_signs = gbc.fwd_sign_for_ports(cfg)
    return {"fwd_sign": fwd_signs[port - 1]}


# ---------------------------------------------------------------------------
# Golden parity (SUC-001/SUC-002, fwd_sign only -- see module docstring for
# why vel_filt_alpha's own half of this coverage was retired, 125-003)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("robot_json", [_TOVEZ_NOCAL_JSON, _TOVEZ_JSON], ids=lambda p: p.stem)
@pytest.mark.parametrize("port", [1, 2], ids=["left", "right"])
def test_golden_parity_motor_config(robot_json, port):
    """A headless SimLoop's live per-motor fwd_sign, after
    configure_from_robot(), matches gen_boot_config.py's own directly-computed
    value for both drive-pair ports (1=left, 2=right)."""
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


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
