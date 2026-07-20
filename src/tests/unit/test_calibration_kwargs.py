"""src/tests/unit/test_calibration_kwargs.py — ticket 113-003.

``calibration_commands()`` (``robot_radio.calibration.push``) used to do two
things at once: decide WHICH fields to push from a ``RobotConfig``, and
format them as text ``SET key=value`` command strings. Ticket 113-005 needs
the field-selection half alone (so ``SimLoop`` can call
``NezhaProtocol.set_config(**kwargs)`` directly, without a text round trip)
-- this ticket extracts that half into ``calibration_kwargs()``.

This module is Qt-free and sim-lib-free (pure function coverage only) --
collected under ``src/tests/unit/`` per ``pyproject.toml``'s ``testpaths``.

Covers:
  1. ``calibration_kwargs()`` returns the same field set the pre-113-003
     ``calibration_commands()`` text list implied, for a fully-populated
     config.
  2. ``minSpeed``/``distanceKp``/``arriveDwell`` (113-003's new wire-key
     coverage) are present only when the source config carries a value,
     mirroring ``headingKp``'s own "push only when present" rule -- proven
     against the two REAL shipped robot profiles: ``tovez.json`` (no
     ``min_speed``/``distance_kp`` override -> both absent; ``arrive_dwell``
     present) and ``tovez_nocal.json`` (all three present).
  3. ``OI``/``OL``/``OA`` (OTOS) never appear in ``calibration_kwargs()``'s
     output -- they are not flat ``SET key=value`` verbs.
  4. ``calibration_commands()``'s output is unchanged by the refactor: a
     snapshot-style pin of its exact ``(command, read_timeout)`` list for
     both real shipped profiles.
"""
from __future__ import annotations

import types
from pathlib import Path

from robot_radio.calibration.push import calibration_commands, calibration_kwargs
from robot_radio.config.robot_config import load_robot_config

_REPO_ROOT = Path(__file__).resolve().parents[3]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"


def _cfg(*, calibration=None, trackwidth=128, control=None, robot_name="r"):
    return types.SimpleNamespace(
        robot_name=robot_name,
        calibration=calibration if calibration is not None else types.SimpleNamespace(),
        geometry=types.SimpleNamespace(trackwidth=trackwidth),
        wheels=types.SimpleNamespace(wheel_diameter_mm=80.77),
        control=control,
    )


# ---------------------------------------------------------------------------
# 1. calibration_kwargs() field-set coverage.
# ---------------------------------------------------------------------------


def test_calibration_kwargs_covers_the_pre_refactor_field_set() -> None:
    """A fully-populated config yields exactly the field set the
    pre-113-003 calibration_commands() text list implied: ml, mr, tw,
    rotSlip, pid.kp/ki/kff/iMax/kaw, headingKp, headingKd -- plus the three
    113-003 additions (minSpeed, distanceKp, arriveDwell)."""
    cfg = _cfg(
        calibration=types.SimpleNamespace(
            mm_per_wheel_deg_left=0.5, mm_per_wheel_deg_right=0.51,
            rotational_slip=0.85,
        ),
        control=types.SimpleNamespace(
            vel_kp=0.002, vel_ki=0.0, vel_kff=0.0, vel_imax=0.0, vel_kaw=0.0,
            heading_kp=1.0, heading_kd=0.0,
            min_speed=16.0, distance_kp=2.5, arrive_dwell=0.15,
        ),
    )

    kwargs = calibration_kwargs(cfg)

    assert set(kwargs) == {
        "ml", "mr", "tw", "rotSlip",
        "pid.kp", "pid.ki", "pid.kff", "pid.iMax", "pid.kaw",
        "headingKp", "headingKd",
        "minSpeed", "distanceKp", "arriveDwell",
    }
    assert kwargs["ml"] == 0.5
    assert kwargs["mr"] == 0.51
    assert kwargs["tw"] == 128
    assert kwargs["rotSlip"] == 0.85
    assert kwargs["minSpeed"] == 16.0
    assert kwargs["distanceKp"] == 2.5
    assert kwargs["arriveDwell"] == 0.15


def test_calibration_kwargs_omits_control_keys_when_control_is_none() -> None:
    """No control section at all -> none of the pid.*/headingK*/minSpeed/
    distanceKp/arriveDwell keys are present (ControlConfig's documented
    contract: None -> firmware boot default kept), but ml/mr/tw/rotSlip
    (which don't depend on control) still are."""
    cfg = _cfg()

    kwargs = calibration_kwargs(cfg)

    assert set(kwargs) == {"ml", "mr", "tw", "rotSlip"}


def test_calibration_kwargs_never_includes_otos_keys() -> None:
    """OI/OL/OA are not flat SET key=value verbs -- calibration_kwargs()
    must never produce them, even when otos_linear_scale/otos_angular_scale
    are set on the config's calibration section."""
    cfg = _cfg(calibration=types.SimpleNamespace(
        otos_linear_scale=1.027, otos_angular_scale=0.987))

    kwargs = calibration_kwargs(cfg)

    assert not {"OI", "OL", "OA"} & set(kwargs)
    assert all(not k.startswith("O") for k in kwargs)


# ---------------------------------------------------------------------------
# 2. minSpeed/distanceKp/arriveDwell presence -- real shipped profiles.
# ---------------------------------------------------------------------------


def test_tovez_nocal_json_carries_all_three_new_planner_keys() -> None:
    """tovez_nocal.json's control section sets min_speed/distance_kp/
    arrive_dwell explicitly -- all three must be pushed."""
    cfg = load_robot_config(_ROBOTS_DIR / "tovez_nocal.json")

    kwargs = calibration_kwargs(cfg)

    assert kwargs["minSpeed"] == 16.0
    assert kwargs["distanceKp"] == 2.5
    assert kwargs["arriveDwell"] == 0.15


def test_tovez_json_omits_min_speed_and_distance_kp_but_keeps_arrive_dwell() -> None:
    """tovez.json's control section has no min_speed/distance_kp override
    at all (absent -> None -> omitted, ControlConfig's "push only when
    present" contract) but DOES carry arrive_dwell -- proving the "absent
    when not set" half of 113-003's acceptance criteria against a real
    fixture, not just a synthetic one."""
    cfg = load_robot_config(_ROBOTS_DIR / "tovez.json")

    kwargs = calibration_kwargs(cfg)

    assert "minSpeed" not in kwargs
    assert "distanceKp" not in kwargs
    assert kwargs["arriveDwell"] == 0.15


# ---------------------------------------------------------------------------
# 3. calibration_commands() output is unchanged by the refactor -- snapshot
#    pins against both real shipped profiles (verified against the real
#    pre-113-003 implementation's own output before the refactor landed).
# ---------------------------------------------------------------------------


def test_calibration_commands_tovez_json_snapshot() -> None:
    cfg = load_robot_config(_ROBOTS_DIR / "tovez.json")

    cmds = calibration_commands(cfg)

    assert cmds == [
        ("SET ml=0.716500", 200),
        ("SET mr=0.707700", 200),
        ("SET tw=128", 200),
        ("SET rotSlip=0.92", 200),
        ("SET pid.kp=0.0016", 200),
        ("SET pid.ki=0.005", 200),
        ("SET pid.kff=0.0008", 200),
        ("SET pid.iMax=0.3", 200),
        ("SET pid.kaw=20", 200),
        ("SET headingKp=6", 200),
        ("SET headingKd=0", 200),
        ("SET arriveDwell=0.15", 200),
        ("OI", 500),
        ("OL 67", 200),
        ("OA -13", 200),
    ]


def test_calibration_commands_tovez_nocal_json_snapshot() -> None:
    cfg = load_robot_config(_ROBOTS_DIR / "tovez_nocal.json")

    cmds = calibration_commands(cfg)

    assert cmds == [
        ("SET ml=0.704851", 200),
        ("SET mr=0.704851", 200),
        ("SET tw=128", 200),
        ("SET rotSlip=1", 200),
        ("SET pid.kp=0.002", 200),
        ("SET pid.ki=0", 200),
        ("SET pid.kff=0.002", 200),
        ("SET pid.iMax=0", 200),
        ("SET pid.kaw=0", 200),
        ("SET headingKp=2.5", 200),
        ("SET headingKd=0", 200),
        ("SET minSpeed=16", 200),
        ("SET distanceKp=2.5", 200),
        ("SET arriveDwell=0.15", 200),
        ("OI", 500),
        ("OL 0", 200),
        ("OA 0", 200),
    ]


def test_calibration_commands_is_calibration_kwargs_formatted_plus_otos() -> None:
    """calibration_commands() must be exactly calibration_kwargs()'s items,
    formatted, in the same order, with the unchanged OI/OL/OA suffix --
    the "thin wrapper" acceptance criterion, asserted structurally rather
    than by re-pinning a third snapshot."""
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
