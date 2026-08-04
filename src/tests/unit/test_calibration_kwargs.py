"""src/tests/unit/test_calibration_kwargs.py — ticket 113-003, retargeted
132-014 (migrate host NezhaProtocol + calibration/push.py + TestGUI onto the
new config surface).

``calibration_commands()`` (``robot_radio.calibration.push``) used to do two
things at once: decide WHICH fields to push from a ``RobotConfig``, and
format them as text ``SET key=value`` command strings. Ticket 113-005 needs
the field-selection half alone (so ``SimLoop`` can call
``NezhaProtocol.set_config(**kwargs)`` directly, without a text round trip)
-- this ticket extracts that half into ``calibration_kwargs()``.

Covers:
  1. ``calibration_kwargs()`` returns the ml/mr/pid.* field set, sourced
     from ``config.motors``/``config.wheel_control`` (132-020's grouped
     shape, replacing the retired ``config.control``/``config.calibration``
     flat sections). ``tw``/``rotSlip`` are DROPPED (132-014, not migrated
     -- see ``calibration_kwargs()``'s own docstring: GEOMETRY is boot-only
     AND one of the sim's own justified divergences).
  2. ``OI``/``OL``/``OA`` (OTOS) never appear in ``calibration_kwargs()``'s
     output -- they are not flat ``SET key=value`` verbs (see
     ``otos_kwargs()`` instead, a separate selector).
  3. ``calibration_commands()``'s output against the two REAL shipped robot
     profiles: ``tovez.json``/``tovez_nocal.json``. 132-014 KNOWN GAP: both
     profiles currently pin to the SAME snapshot -- the JSON files are still
     in the OLD 13-section shape (ticket 017 reshapes them), so
     ``config.motors``/``config.wheel_control`` read their proto3 zero
     defaults for BOTH profiles today, and ``ml``/``mr`` fall back to the
     identical wheel-diameter-derived default (both profiles share the same
     ``wheel_diameter_mm``). This is expected, accepted mid-sprint state
     (sprint.md Design Rationale Decision 2), not a test bug -- ticket 017
     re-differentiates these two snapshots.
"""
from __future__ import annotations

import types
from pathlib import Path

from robot_radio.calibration.push import calibration_commands, calibration_kwargs
from robot_radio.config.robot_config import load_robot_config

_REPO_ROOT = Path(__file__).resolve().parents[3]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"


def _cfg(*, motors=None, wheel_control=None, wheel_diameter_mm=80.77, robot_name="r"):
    return types.SimpleNamespace(
        robot_name=robot_name,
        motors=motors if motors is not None else types.SimpleNamespace(),
        wheel_control=wheel_control if wheel_control is not None else types.SimpleNamespace(),
        wheels=types.SimpleNamespace(wheel_diameter_mm=wheel_diameter_mm),
    )


# ---------------------------------------------------------------------------
# 1. calibration_kwargs() field-set coverage.
# ---------------------------------------------------------------------------


def test_calibration_kwargs_covers_the_ml_mr_pid_field_set() -> None:
    """A fully-populated config yields exactly ml/mr/pid.kp/ki/kff/iMax/kaw
    -- tw/rotSlip are DROPPED (132-014, see this module's own docstring),
    not carried over from the pre-refactor field set."""
    cfg = _cfg(
        motors=types.SimpleNamespace(
            travel_calib_left=0.5, travel_calib_right=0.51),
        wheel_control=types.SimpleNamespace(
            pid_kp=0.002, pid_ki=0.0, pid_kaff=0.0, pid_i_max=0.0, pid_max=0.0),
    )

    kwargs = calibration_kwargs(cfg)

    assert set(kwargs) == {
        "ml", "mr", "pid.kp", "pid.ki", "pid.kff", "pid.iMax", "pid.kaw",
    }
    assert kwargs["ml"] == 0.5
    assert kwargs["mr"] == 0.51
    assert kwargs["pid.kp"] == 0.002


def test_calibration_kwargs_falls_back_to_wheel_diameter_when_travel_calib_is_zero() -> None:
    """0.0 (the generated Motors group's own proto3 default -- never a
    valid calibrated value, robot_config.proto's own (min)=0.0001 bound)
    falls back to the wheel-diameter-derived default, the same "uncalibrated
    -> geometric default" contract the pre-refactor None-sentinel had."""
    cfg = _cfg(wheel_diameter_mm=80.0)

    kwargs = calibration_kwargs(cfg)

    import math
    expected = math.pi * 80.0 / 360.0
    assert kwargs["ml"] == expected
    assert kwargs["mr"] == expected


def test_calibration_kwargs_omits_pid_keys_when_wheel_control_is_none() -> None:
    """No wheel_control section at all -> none of the pid.* keys are
    present, but ml/mr (which don't depend on wheel_control) still are."""
    cfg = _cfg()
    cfg.wheel_control = None

    kwargs = calibration_kwargs(cfg)

    assert set(kwargs) == {"ml", "mr"}


def test_calibration_kwargs_never_includes_otos_keys() -> None:
    """OI/OL/OA are not flat SET key=value verbs -- calibration_kwargs()
    must never produce them (OTOS fields live on a SEPARATE selector,
    otos_kwargs(), reading a different ConfigGroupTarget)."""
    cfg = _cfg()

    kwargs = calibration_kwargs(cfg)

    assert not {"OI", "OL", "OA"} & set(kwargs)
    assert all(not k.startswith("O") for k in kwargs)


# ---------------------------------------------------------------------------
# 2. calibration_commands() output -- snapshot pins against both real
#    shipped profiles. 132-014: both currently produce the SAME snapshot --
#    see this module's own header comment for why (expected, ticket-017-
#    blocked, not a test bug).
# ---------------------------------------------------------------------------


_EXPECTED_COMMANDS_PRE_017 = [
    ("SET ml=0.704851", 200),
    ("SET mr=0.704851", 200),
    ("SET pid.kp=0", 200),
    ("SET pid.ki=0", 200),
    ("SET pid.kff=0", 200),
    ("SET pid.iMax=0", 200),
    ("SET pid.kaw=0", 200),
    ("OI", 500),
    ("OL 0", 200),
    ("OA 0", 200),
]


def test_calibration_commands_tovez_json_snapshot() -> None:
    cfg = load_robot_config(_ROBOTS_DIR / "tovez.json")

    cmds = calibration_commands(cfg)

    assert cmds == _EXPECTED_COMMANDS_PRE_017


def test_calibration_commands_tovez_nocal_json_snapshot() -> None:
    cfg = load_robot_config(_ROBOTS_DIR / "tovez_nocal.json")

    cmds = calibration_commands(cfg)

    assert cmds == _EXPECTED_COMMANDS_PRE_017


def test_calibration_commands_is_calibration_kwargs_formatted_plus_otos() -> None:
    """calibration_commands() must be exactly calibration_kwargs()'s items,
    formatted, in the same order, with the OI/OL/OA suffix -- the "thin
    wrapper" acceptance criterion, asserted structurally rather than by
    re-pinning a third snapshot."""
    for name in ("tovez.json", "tovez_nocal.json"):
        cfg = load_robot_config(_ROBOTS_DIR / name)
        kwargs = calibration_kwargs(cfg)
        cmds = calibration_commands(cfg)

        set_cmds = [c for c in cmds if c[0].startswith("SET ")]
        assert len(set_cmds) == len(kwargs)
        for (cmd, timeout), key in zip(set_cmds, kwargs):
            assert cmd.startswith(f"SET {key}=")
            assert timeout == 200

        tail = [c for c, _t in cmds if not c.startswith("SET ")]
        assert tail[0] == "OI"
        assert tail[1].startswith("OL ")
        assert tail[2].startswith("OA ")
