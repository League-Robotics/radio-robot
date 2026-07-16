"""src/tests/unit/test_robot_config.py — 104-005 (persistent OTOS-untrusted marker).

Covers `geometry.otos_untrusted` on `RobotConfig`: it round-trips through
load (JSON -> pydantic) for a profile that sets it, defaults to False for a
profile that omits it, and the two shipped rig profiles (`tovez.json`,
`tovez_nocal.json`) actually carry it set True on disk — the persisted fact
this ticket exists to record (see
`clasi/sprints/104-host-realignment-and-full-bench-gate/issues/
rig-persistent-otos-distrust.md`).

Collected under src/tests/unit/ (a host-tooling check, not sim/bench/playfield-
scoped — see tests/CLAUDE.md); pyproject.toml's testpaths includes
tests/unit.
"""

import json
from pathlib import Path

from robot_radio.config.robot_config import RobotConfig, load_robot_config

_REPO_ROOT = Path(__file__).resolve().parents[3]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"

_MINIMAL_IDENTITY = {"robot_name": "test-bot", "uid": "test-bot"}


def test_otos_untrusted_defaults_false_when_omitted():
    """A profile with no geometry.otos_untrusted key at all loads with the
    field defaulting to False — existing profiles that predate this field
    stay valid without modification."""
    cfg = RobotConfig.model_validate({"identity": _MINIMAL_IDENTITY})
    assert cfg.geometry.otos_untrusted is False


def test_otos_untrusted_round_trips_true():
    """A profile that explicitly sets geometry.otos_untrusted=true loads
    that value through JSON (model_validate_json), matching how
    load_robot_config() reads real files from disk."""
    raw = json.dumps({
        "identity": _MINIMAL_IDENTITY,
        "geometry": {"otos_untrusted": True},
    })
    cfg = RobotConfig.model_validate_json(raw)
    assert cfg.geometry.otos_untrusted is True


def test_otos_untrusted_round_trips_false_explicit():
    """Explicitly-set False is preserved too (not conflated with the
    omitted/default case by model_dump)."""
    raw = json.dumps({
        "identity": _MINIMAL_IDENTITY,
        "geometry": {"otos_untrusted": False},
    })
    cfg = RobotConfig.model_validate_json(raw)
    assert cfg.geometry.otos_untrusted is False
    assert cfg.model_dump()["geometry"]["otos_untrusted"] is False


def test_tovez_profile_marks_otos_untrusted():
    """The rig's actual profile (tovez.json — the one active_robot.json
    points at and match_robot_by_id() resolves to for device_announcement_
    name='tovez', per 093's active-pointer switch) carries the persisted
    flag, so no manual per-session SET is needed to record the bench rig's
    OTOS-decoupled-from-wheels fact."""
    cfg = load_robot_config(_ROBOTS_DIR / "tovez.json")
    assert cfg.geometry.otos_untrusted is True


def test_tovez_nocal_profile_marks_otos_untrusted():
    """tovez_nocal.json is the calibration-stripped variant of the same
    physical robot (same connection.serial_last_6) — it carries the same
    mounting fact so the flag survives even if active_robot.json is ever
    pointed back at it."""
    cfg = load_robot_config(_ROBOTS_DIR / "tovez_nocal.json")
    assert cfg.geometry.otos_untrusted is True


def test_togov_profile_unaffected():
    """togov.json (a different physical robot, mecanum drivetrain) is out
    of scope for this ticket and keeps the default (unset -> False)."""
    cfg = load_robot_config(_ROBOTS_DIR / "togov.json")
    assert cfg.geometry.otos_untrusted is False
