"""src/host/robot_radio/calibration/sim_boot_config.py -- ticket 113-004.

Tier-2 (boot-only) ``msg::PlannerConfig`` / ``Devices::MotorConfig`` scalar
mapping helper: computes the SAME field values ``gen_boot_config.py`` bakes
into a real robot's ``boot_config.cpp`` at build time, but from an
already-loaded host ``RobotConfig`` (or a raw robot-JSON dict) at
*sim-open* time -- see sprint 113's own Design Rationale Decision 2 ("Reuse
gen_boot_config.py's functions, don't re-derive the mapping").

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


def _heading_source_wire_value(cfg: dict) -> int:
    """Return ``heading_source`` as its wire int enum value
    (``msg::HeadingSourceMode`` / ``planner_pb2.HeadingSourceMode``).

    ``gen_boot_config.py``'s own ``heading_source_for_config()`` returns the
    C++ enumerator LITERAL STRING it bakes into generated source (e.g.
    ``"msg::HeadingSourceMode::HEADING_SOURCE_AUTO"``) -- exactly right for
    a ``.cpp`` file, not for a wire/ctypes int. Rather than hand-copying a
    second name->int table (the AUTO=0/FORCE_OTOS=1/FORCE_ENCODER=2
    assignment gen_boot_config.py's own ``_HEADING_SOURCE_WIRE_NAMES``
    dict implicitly encodes via string selection), this resolves the
    literal's trailing enumerator name through the SAME generated protobuf
    enum descriptor the real wire/ctypes path already uses
    (``planner_pb2.HeadingSourceMode``, generated from
    ``src/protos/planner.proto``) -- one source of truth for the
    name<->int mapping, `gen_boot_config.py`'s own function for the
    string selection.
    """
    from robot_radio.robot.pb2 import planner_pb2

    literal = gbc.heading_source_for_config(cfg)  # e.g. "msg::HeadingSourceMode::HEADING_SOURCE_AUTO"
    member_name = literal.rsplit("::", 1)[-1]  # "HEADING_SOURCE_AUTO"
    return planner_pb2.HeadingSourceMode.Value(member_name)


def planner_boot_config_for(config: Any) -> "dict[str, float | int]":
    """Return every Tier-2 (boot-only) ``msg::PlannerConfig`` scalar this
    sprint covers, computed from *config* (a ``RobotConfig`` or a raw robot
    JSON dict) by calling ``gen_boot_config.py``'s existing mapping
    functions -- never by re-deriving any of them.

    Sprint 114 (config-as-truth completion): ``a_max``/``a_decel``/
    ``v_body_max``/``j_max``/``yaw_jerk_max`` gained a real per-robot JSON
    mapping function (``motion_limits_for_config()``) -- before this ticket
    ``gen_boot_config.py`` had none at all for these five (its own
    ``generate()`` referenced its module DEFAULT constants directly), so
    this module mirrored that by reading the same constants directly. Now
    it calls the real mapping like every other field here, never re-deriving
    it.
    """
    cfg = _as_cfg_dict(config)

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
