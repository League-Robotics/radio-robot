"""src/host/robot_radio/calibration/sim_boot_config.py -- ticket 113-004.

Tier-2 (boot-only) ``Devices::MotorConfig`` scalar mapping helper: computes
the SAME field values ``gen_boot_config.py`` bakes into a real robot's
``boot_config.cpp`` at build time, but from an already-loaded host
``RobotConfig`` (or a raw robot-JSON dict) at *sim-open* time -- see sprint
113's own Design Rationale Decision 2 ("Reuse gen_boot_config.py's
functions, don't re-derive the mapping").

115-003 (gut-to-minimal-firmware S1 motion-stack excision) deleted this
module's ``msg::PlannerConfig`` half (``planner_boot_config_for()`` /
``_heading_source_wire_value()``) wholesale -- ``msg::PlannerConfig``
itself, and every ``gen_boot_config.py`` mapping function it called, went
with the deleted ``App::Pilot``/``Motion::Executor`` subsystems (ticket
003's proto surgery). ``motor_boot_config_for()`` below is the sole
survivor.

Runtime dependency: this module imports ``src/scripts/gen_boot_config.py``'s
pure ``cfg: dict -> value`` mapping functions directly, via the exact
``sys.path`` shim ``src/tests/sim/unit/test_gen_boot_config_planner.py``
already established -- NOT by reimplementing any JSON->value decision. This
takes a runtime dependency on ``gen_boot_config.py`` staying import-safe
(pure functions, no argv/stdout side effects at import time -- true today).
None of ``gen_boot_config.py``'s own mapping logic is touched here; every
Tier-2 field this module returns is computed by CALLING one of its
functions, never by re-expressing the same JSON->value decision a second
time (the exact bug class this sprint exists to close -- see sprint.md's
Problem section and Design Rationale Decision 2).

Why a RobotConfig->dict conversion is needed at all: ``gen_boot_config.py``'s
functions read a raw ``cfg: dict`` (``cfg.get("control", {})``,
``cfg.get("calibration", {})``, straight out of ``json.load()``).
``RobotConfig``'s ``ControlConfig``/``CalibrationConfig`` pydantic sub-models
are the host's own typed view of the identical JSON keys; 113-003/113-004
extended both models so every key ``gen_boot_config.py`` reads is a declared
field (pydantic silently DROPS undeclared keys at parse time, so an
incomplete model would silently diverge from the JSON on exactly the fields
this sprint cares about -- see ``robot_config.py``'s own 113-004 comment on
``ControlConfig``/``CalibrationConfig``). ``_as_cfg_dict()`` below
reconstructs the raw-dict shape via ``model_dump()`` so the SAME
``gen_boot_config.py`` functions run unmodified against either source.
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

# src/host/robot_radio/calibration/sim_boot_config.py -> calibration ->
# robot_radio -> host -> src -> repo root = FOUR hops from __file__, the
# identical pattern src/tests/sim/unit/test_gen_boot_config_planner.py's own
# _REPO_ROOT/_SCRIPTS_DIR shim already established (that file's own header
# is the precedent this ticket reuses -- sprint 113 Design Rationale
# Decision 2).
_REPO_ROOT = Path(__file__).resolve().parents[4]
_SCRIPTS_DIR = _REPO_ROOT / "src" / "scripts"

if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

import gen_boot_config as gbc  # noqa: E402  (path must be set up before this import)


def _as_cfg_dict(config: Any) -> dict:
    """Normalize *config* into the raw ``cfg: dict`` shape ``gen_boot_config.py``'s
    functions expect: ``{"control": {...}, "calibration": {...}}``.

    ``gen_boot_config.py``'s mapping functions only ever read
    ``cfg.get("control", {})`` and (``fwd_sign_for_ports()`` alone)
    ``cfg.get("calibration", {})`` -- no other top-level key. A raw dict
    (e.g. ``json.loads(path.read_text())``, or a bare ``{}``/partial dict
    for the no-config fallback case) is passed through unchanged. A
    ``RobotConfig`` (or any duck-typed object exposing ``.control``/
    ``.calibration`` pydantic sub-models) has each sub-model dumped to a
    plain dict via ``model_dump()`` -- this reproduces the identical dict
    shape a raw ``json.load()`` of the same robot config file would present
    at the same key path, since both models now declare every key
    ``gen_boot_config.py`` reads (113-004).
    """
    if isinstance(config, dict):
        return config

    control = getattr(config, "control", None)
    calibration = getattr(config, "calibration", None)
    return {
        "control": control.model_dump() if control is not None else {},
        "calibration": calibration.model_dump() if calibration is not None else {},
    }


# planner_boot_config_for() / _heading_source_wire_value() -- DELETED
# (115-003, gut-to-minimal-firmware S1 motion-stack excision). Both mapped
# host RobotConfig/JSON onto Tier-2 msg::PlannerConfig boot scalars (motion
# limits, heading/distance PD gains, heading_source, lead compensation,
# model tau) by calling gen_boot_config.py's own mapping functions -- EVERY
# one of which (motion_limits_for_config/profile_rot_limits_for_config/
# min_speed_for_config/heading_gains_for_config/arrive_dwell_for_config/
# heading_source_for_config/heading_dwell_for_config/
# lead_compensation_for_config/actuation_lag_for_config/
# distance_gains_for_config/model_tau_for_config) was deleted wholesale by
# ticket 003 alongside msg::PlannerConfig itself (planner.proto, deleted in
# the same ticket) and the App::Pilot/Motion::Executor subsystems that read
# it. There is no msg::PlannerConfig left to boot-initialize in the S1
# minimal firmware and no telemetry_pb2 (or other) type that now serves
# this role -- confirmed per this ticket's own acceptance criterion; the
# dead code path is removed rather than left calling ten now-nonexistent
# gen_boot_config.py functions. motor_boot_config_for() below is
# UNCHANGED -- it depends only on vel_gains_for_config()/
# fwd_sign_for_ports(), both still live.


def motor_boot_config_for(config: Any, port: int) -> "dict[str, float | int]":
    """Return ``{"vel_filt_alpha": ..., "fwd_sign": ...}`` for *port*
    (1=left, 2=right, per ``gen_boot_config.py``'s own ``LEFT_PORT``/
    ``RIGHT_PORT``), computed from *config* by calling
    ``gen_boot_config.py``'s existing ``vel_gains_for_config()`` (the
    ``filt`` element) and ``fwd_sign_for_ports()`` (indexed by port).
    """
    cfg = _as_cfg_dict(config)

    *_gains, filt_alpha = gbc.vel_gains_for_config(cfg)
    fwd_signs = gbc.fwd_sign_for_ports(cfg)

    return {
        "vel_filt_alpha": filt_alpha,
        "fwd_sign": fwd_signs[port - 1],
    }
