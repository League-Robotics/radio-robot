"""src/host/robot_radio/calibration/sim_boot_config.py -- ticket 113-004.

Tier-2 (boot-only) ``Hal::MotorConfig`` scalar mapping helper: computes
the SAME field values ``gen_boot_config.py`` bakes into a real robot's
``boot_config.cpp`` at build time, but from an already-loaded host
``RobotConfig`` (or a raw robot-JSON dict) at *sim-open* time -- see sprint
113's own Design Rationale Decision 2 ("Reuse gen_boot_config.py's
functions, don't re-derive the mapping").

115-003 (gut-to-minimal-firmware S1 motion-stack excision) deleted this
module's ``msg::PlannerConfig`` half (``planner_boot_config_for()`` /
``_heading_source_wire_value()``) wholesale -- ``msg::PlannerConfig``
itself, and every ``gen_boot_config.py`` mapping function it called, went
with the deleted ``Core::Pilot``/``Motion::Executor`` subsystems (ticket
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

import math
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
    (e.g. ``json.loads(path.read_text())`` of a robot JSON, still in the
    OLD 13-section shape -- ticket 017 has not reshaped the files yet) is
    passed through unchanged; this remains the ONLY input shape this
    function still handles.

    132-014 (retargeting off ``RobotConfig``'s retired ``.control``/
    ``.calibration`` sub-models, removed when ``RobotConfig`` adopted
    ``robot_config.proto``'s consumer-grouped shape, 132-020): a
    ``RobotConfig``/duck-typed object is NO LONGER accepted here -- it has
    no ``.control``/``.calibration`` attributes to dump any more (both are
    now individually-typed, pydantic-validated fields spread across
    ``.motors``/``.wheel_control``/``.drive``/``.geometry``/etc., not one
    JSON-mirroring blob each). Every caller that used to pass a
    ``RobotConfig`` through this function now reads its own needed fields
    DIRECTLY off the grouped object instead -- see
    ``motor_boot_config_for()``/``drive_boot_config_for()``/
    ``drivetrain_boot_config_for()``'s own doc comments for the per-function
    split (grouped-object fast path vs. this function's raw-dict path).
    Passing a non-dict, non-grouped-shaped object here now raises
    ``AttributeError`` (``.get`` on the wrong type) -- an honest failure,
    not the old function's silent ``{}`` fallback that used to mask the
    130-020 shape change as an empty (and therefore ``_require()``-raising)
    config.
    """
    if isinstance(config, dict):
        return config
    raise TypeError(
        f"_as_cfg_dict() expects a raw robot-JSON dict (still the OLD "
        f"13-section shape, pending ticket 017's reshape) -- got "
        f"{type(config).__name__}. A RobotConfig/grouped-shape object must "
        f"be read directly off its own .motors/.drive/.geometry groups "
        f"(132-014) -- see this module's own header comment."
    )


def _is_grouped_robot_config(config: Any) -> bool:
    """True if *config* already exposes ``robot_config.proto``'s
    consumer-grouped attributes directly (``.motors``/``.drive`` -- 132-020
    adopted this shape into ``RobotConfig`` itself) rather than needing
    ``gen_boot_config.py``'s raw-dict path-mapping functions at all: once
    the source already carries typed ``Motors``/``Drive``/``Geometry``
    group objects, re-deriving the SAME values through a dict-path lookup
    would be needless indirection, not "reuse gen_boot_config.py's mapping,
    don't re-derive it" (sprint 113's own Design Rationale Decision 2) --
    that decision's whole point was to avoid a SECOND, independently-drifting
    expression of the JSON-path-to-value mapping; reading an already-typed
    field directly is not a second expression of anything.
    """
    return hasattr(config, "motors") and hasattr(config, "drive")


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
# the same ticket) and the Core::Pilot/Motion::Executor subsystems that read
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
    ``RIGHT_PORT``).

    132-014: two paths, selected by ``_is_grouped_robot_config()``. A
    ``RobotConfig``/grouped-shape *config* (``.motors``/``.drive`` -- the
    common case, 132-020) reads ``config.motors.vel_filt_alpha``/
    ``fwd_sign_left``/``fwd_sign_right`` DIRECTLY -- ``Motors.vel_filt_alpha``
    is a SINGLE value shared by both bound motors (matching
    ``vel_gains_for_config()``'s own "applied to BOTH bound motors" shape),
    ``fwd_sign_left``/``fwd_sign_right`` are per-side. A raw dict (still the
    OLD 13-section JSON shape, e.g. from ``json.loads()``) still goes
    through ``gen_boot_config.py``'s ``vel_gains_for_config()``/
    ``fwd_sign_for_ports()`` (unchanged from before this ticket) -- the
    JSON files themselves are not reshaped until ticket 017, so this path
    still reads their current key layout.
    """
    if _is_grouped_robot_config(config):
        motors = config.motors
        # THIS robot's own port binding, via the generator's single
        # definition -- not the LEFT_PORT/RIGHT_PORT defaults. `motors` is
        # labelled by WHEEL, and which port each wheel sits on is per-robot
        # wiring: tovez is port 1 = RIGHT. Reading the labels at the default
        # ports handed port 1 the LEFT wheel's fwd_sign, so a configure_from_
        # robot() push disagreed with the same robot's baked boot_config.cpp
        # (caught by test_sim_boot_config_parity.py's golden-parity check).
        left_port, right_port = gbc.drive_ports(
            {"motors": {"left_port": getattr(motors, "left_port", 0),
                        "right_port": getattr(motors, "right_port", 0)}}
        )
        if port == left_port:
            fwd_sign = motors.fwd_sign_left
        elif port == right_port:
            fwd_sign = motors.fwd_sign_right
        else:
            # Every other port (no drive-pair mount) -- the SAME placeholder
            # gen_boot_config.py's own fwd_sign_for_ports() uses for a port
            # outside the drive pair.
            fwd_sign = gbc.FWD_SIGN
        return {
            "vel_filt_alpha": float(motors.vel_filt_alpha),
            "fwd_sign": int(fwd_sign),
        }

    cfg = _as_cfg_dict(config)
    *_gains, filt_alpha = gbc.vel_gains_for_config(cfg)
    fwd_signs = gbc.fwd_sign_for_ports(cfg)

    return {
        "vel_filt_alpha": filt_alpha,
        "fwd_sign": fwd_signs[port - 1],
    }


def drive_boot_config_for(config: Any) -> "dict[str, float]":
    """Return ``Core::DifferentialDrive``'s own boot calibration -- the duty-per-speed
    pair, the eight commanded->actual wheel-correction values, and the crawl
    amplitude -- for ``sim_configure_drive()``, the sim-side counterpart of
    ``main.cpp``'s own ``setDutyPerSpeed()``/``setWheelCorrection()``/
    ``setCrawlPulse()`` boot seam (which reads the identical
    ``control.duty_per_speed_*``/``wheel_gain_*``/``wheel_intercept_*``/
    ``crawl_pulse`` JSON keys via ``Config::defaultDriveConfig()``).

    Before this function (and its ctypes call site,
    ``SimLoop.configure_from_robot()``) existed, NOTHING in the sim path ever
    installed a drive calibration: ``TestSim::SimHarness``'s composition root
    constructs ``Core::DifferentialDrive`` and never calls any of the three setters, so
    the sim's own Drive sat permanently uncalibrated (``calibrated_ ==
    false``, ``tick()`` returns without writing a duty). What made it move
    anyway was an accident: ``Core::Configurator::applyMotorConfigPatch()``
    used to route the ``pid.kff`` wire key into ``setDutyPerSpeed()``, so the
    connect-time calibration push doubled as the sim's only calibration
    install -- with the WRONG quantity (``control.vel_kff``, the velocity
    PID's feedforward gain, 0.0008 for tovez, against a real duty scale of
    0.00187325) and with the wheel correction never applied at all. Wheels
    ran at ~43% of the commanded speed and every tour turn leg timed out.
    That routing is gone (configurator.cpp); this is its honest replacement.

    Returns the duty-per-speed pair and the crawl amplitude ONLY. The eighth
    boot value main.cpp also installs -- ``setWheelCorrection()``'s measured
    commanded->actual line (``control.wheel_gain_*``/``wheel_intercept_*``) --
    is deliberately NOT installed in the sim: it is a LINEARIZATION of one
    physical drivetrain's gearbox (``measured = gain*commanded + intercept``,
    gain ~1.47 for tovez, docs/design/wheel-speed-command-mapping.md), and
    ``TestSim::WheelPlant`` is a plain first-order linear plant with none of
    the nonlinearity it corrects for. Installing it against that plant does
    not cancel anything -- it just divides every commanded speed by its own
    gain (measured: 200 mm/s commanded, 124 mm/s actual) and bends a straight
    leg, since the left and right gains deliberately differ. An identity
    correction (``Drive``'s own default, gain 1 / intercept 0) is the
    faithful choice for a plant that needs no linearization.

    Both remaining values are REQUIRED -- ``gen_boot_config.py``'s own
    fail-closed posture for the same keys, reached by calling ITS mapping
    function (``drive_config_for_config()``) rather than re-expressing the
    JSON->value decision here for the raw-dict path.

    132-014: a ``RobotConfig``/grouped-shape *config* (``.motors``/
    ``.drive``, 132-020) reads ``config.drive.duty_per_speed_left/right``/
    ``crawl_pulse`` DIRECTLY -- these are exactly ``Drive`` group fields
    now, no dict-path derivation needed. The eight ``wheel_gain_*``/
    ``wheel_intercept_*`` Stage-A fields ALSO live on ``config.drive`` but
    are deliberately NOT read here either, same reason as always (see this
    docstring's own linearization note above) -- this function's return
    shape is unchanged by the retarget, still duty-per-speed pair + crawl
    pulse only.
    """
    if _is_grouped_robot_config(config):
        drive = config.drive
        return {
            "duty_per_speed_left": float(drive.duty_per_speed_left),
            "duty_per_speed_right": float(drive.duty_per_speed_right),
            "crawl_pulse": float(drive.crawl_pulse),
        }

    cfg = _as_cfg_dict(config)
    duty_left, duty_right, crawl = gbc.drive_config_for_config(cfg)
    return {
        "duty_per_speed_left": duty_left,
        "duty_per_speed_right": duty_right,
        "crawl_pulse": crawl,
    }


def drivetrain_boot_config_for(config: Any) -> "dict[str, float]":
    """Return ``{"rot_gain_pos", "rot_offset_pos", "rot_gain_neg",
    "rot_offset_neg"}`` (offsets in RADIANS) for
    ``sim_configure_drivetrain()`` (125-007,
    adjacent-sim-plant-rotation-calibration-for-angle-stop-move-overshoot.md)
    -- the sim-side counterpart of ``main.cpp``'s own boot seam, which reads
    the identical four ``calibration.rotation_gain``/``rotation_offset_deg``/
    ``rotation_gain_neg``/``rotation_offset_deg_neg`` JSON keys via
    ``gen_boot_config.py``'s ``rotation_calibration_for_config()`` and calls
    ``Core::RobotLoop::setRotationCalibration()`` once at boot.

    Before this function (and its ctypes call site,
    ``SimLoop.configure_from_robot()``) existed, nothing in the sim path
    ever called ``setRotationCalibration()`` -- the sim's own
    ``Core::RobotLoop`` kept the identity default (gain 1, offset 0)
    permanently, so editing a robot JSON's rotation calibration fields was a
    silent no-op for ``square_tour.py --sim``.

    Degrees->radians conversion happens HERE (mirroring ``main.cpp``'s own
    ``kDegToRad`` conversion at its boot seam) so the ctypes export itself
    stays a pure passthrough -- both paths below hand this function values
    in DEGREES (the JSON's own unit, and ``robot_config.proto``'s own
    ``Geometry.rotation_offset``/``rotation_offset_neg`` -- ``// [deg]``).

    132-014: a ``RobotConfig``/grouped-shape *config* (``.motors``/
    ``.drive``, 132-020) reads ``config.geometry.rotation_gain_pos``/
    ``rotation_offset``/``rotation_gain_neg``/``rotation_offset_neg``
    DIRECTLY -- the exact same four quantities
    ``rotation_calibration_for_config()`` derives from the raw dict, now
    typed ``Geometry`` group fields.
    """
    if _is_grouped_robot_config(config):
        geometry = config.geometry
        return {
            "rot_gain_pos": float(geometry.rotation_gain_pos),
            "rot_offset_pos": math.radians(geometry.rotation_offset),
            "rot_gain_neg": float(geometry.rotation_gain_neg),
            "rot_offset_neg": math.radians(geometry.rotation_offset_neg),
        }

    cfg = _as_cfg_dict(config)
    gain_pos, offset_pos_deg, gain_neg, offset_neg_deg = gbc.rotation_calibration_for_config(cfg)
    return {
        "rot_gain_pos": gain_pos,
        "rot_offset_pos": math.radians(offset_pos_deg),
        "rot_gain_neg": gain_neg,
        "rot_offset_neg": math.radians(offset_neg_deg),
    }
