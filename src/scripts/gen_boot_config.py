#!/usr/bin/env python3
"""Generate src/firm/config/boot_config.cpp from the active robot JSON config.

Run:  python3 scripts/gen_boot_config.py

The message-based subsystems tree boots Subsystems::NezhaHardware from an array
of msg::MotorConfig and configures Subsystems::Drivetrain from a
msg::DrivetrainConfig. This generator bakes those boot defaults from the active
robot config (via data/robots/active_robot.json or the ROBOT_CONFIG env var) so
main.cpp never hardcodes calibration — it just calls
Config::defaultMotorConfigs() / Config::defaultDrivetrainConfig().

This is the message-tree sibling of scripts/gen_default_config.py (which bakes
the OLD source/robot/RobotConfig struct). It is deliberately separate: the two
target different C++ types and the new NezhaMotor velocity PID operates on a
different plant scale than the old RobotConfig velocity loop.

Config-as-truth (sprint 114) — no source-side behavioral defaults
-------------------------------------------------------------------
Every BEHAVIORAL value this generator bakes now comes from the active robot
JSON's `control`/`calibration`/`geometry` blocks, with NO Python-side
fallback: a robot JSON missing a required key fails the build loudly
(`MissingRobotConfigKeyError`, caught by `main()` as a `sys.exit(1)` naming
the key and the JSON path) instead of silently substituting a bench
placeholder. Before this ticket, ~29 module-level `*_DEFAULT` constants
supplied that placeholder whenever a key was absent — a deliberate design
choice documented in `src/firm/config/DESIGN.md` §3 ("missing/bad robot
JSON degrades to bench defaults, not a build failure"), reversed here per
the stakeholder's own instruction (2026-07-20): a build must refuse to
produce a firmware image with an incomplete calibration, not guess one.

Every field mapping below is a `*_for_config(cfg)` function reading one or
more `cfg["control"][...]`/`cfg["calibration"][...]`/`cfg["geometry"][...]`
keys via `_require()`:
  * `vel_gains_for_config()` — control.vel_kp/vel_ki/vel_kff/vel_imax/
    vel_kaw/vel_filt (the velocity PID, in the NezhaMotor duty [-1,1] plant
    scale — control._vel_gains_domain documents this; NOT the old
    RobotConfig PWM-percent scale, kp ~ 0.3).
  * `trackwidth_for_config()` — geometry.trackwidth -> DrivetrainConfig.trackwidth.
  * `otos_boot_config_values()` — geometry.odometry_offset_mm.{x,y,yaw_rad}
    and calibration.otos_linear_scale/otos_angular_scale (086-005) ->
    OtosBootConfig; boot-time-baked only, never a live SET/wire surface (see
    OtosBootConfig's own doc comment, src/firm/config/boot_config.h).
  * `heading_gains_for_config()` / `heading_source_for_config()` /
    `heading_dwell_for_config()` / `lead_compensation_for_config()` —
    control.heading_kp/heading_kd (098-001), control.heading_source
    ("auto"/"otos"/"encoder", 109-005), control.heading_dwell_tol_deg/
    heading_dwell_rate_dps, control.heading_lead_bias/plan_lead/
    terminal_lead (109-010).
  * `profile_rot_limits_for_config()` — control.yaw_rate_max [deg/s] /
    max_rot_accel_dps2 [deg/s^2] (100-014, converted to rad here).
  * `min_speed_for_config()` / `arrive_dwell_for_config()` /
    `actuation_lag_for_config()` / `distance_gains_for_config()` /
    `model_tau_for_config()` — control.min_speed (100-007), control.
    arrive_dwell (100-001), control.actuation_lag (112-002), control.
    distance_kp/distance_tol (112-003), control.model_tau_lin/model_tau_ang
    (113-001).
  * `motion_limits_for_config()` — control.a_max/a_decel/v_body_max/j_max/
    yaw_jerk_max (098-001's other five motion-limit fields; before sprint
    114 these had NO per-robot JSON mapping at all — generate() referenced
    the module DEFAULT constants directly).

Structural, compile-time, exempt (NOT behavioral tunables, NOT migrated —
see sprint 114's Architecture Boundary list): `K_MOTOR_COUNT` (array sizing,
tracks main.cpp's static_assert), `LEFT_PORT`/`RIGHT_PORT` (the drive-pair
wiring fact) and `polled_for_ports()` (the I2C flip-flop poll-schedule
membership — a firmware-scheduling fact, never per-robot calibration), and
`TRAVEL_CALIB_PLACEHOLDER`/`FWD_SIGN` — the documented placeholder for the
two motor ports the shipped drivetrain does not actually drive (ports 3/4 on
a 2-wheel differential robot; provably inert, excluded from
`polled_for_ports()`'s schedule). `travel_calib_for_ports()`/
`fwd_sign_for_ports()` still fall back to these placeholders when the robot
JSON omits `calibration.mm_per_wheel_deg_left/right` /
`calibration.fwd_sign_left/right` for the DRIVE-PAIR ports too — unchanged
by this ticket (out of its explicit scope; see sprint 114 ticket 002's own
Approach step 1).
"""

import json
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
OUT_FILE  = REPO_ROOT / "src" / "firm" / "config" / "boot_config.cpp"

# --- Structural constants (compile-time, exempt from config-as-truth) ------
# See this module's own docstring "Structural, compile-time, exempt" section
# and sprint 114's Architecture Boundary list for why these five stay Python
# constants instead of required robot-JSON keys.

# Ports 1..kMotorCount; matches Subsystems::NezhaHardware::kMotorCount, asserted
# in main.cpp. Keep in sync if the port count ever changes.
K_MOTOR_COUNT = 4

# The drive-pair port binding (the robot's normal drive pair). The coupled bench
# rig re-binds at runtime via `DEV DT PORTS 3 4`. An unseeded (zero) port would
# address motor(0), which NezhaHardware::motor() clamps to port 4 — silently
# wrong, not a crash — so these are always seeded.
LEFT_PORT  = 1
RIGHT_PORT = 2

# fwd_sign placeholder for any port OTHER than LEFT_PORT/RIGHT_PORT (088-002)
# -- the two motor ports the shipped 2-wheel differential drivetrain does not
# actually drive. Provably inert: polled_for_ports() excludes them from the
# I2C flip-flop schedule, so no live control path ever reads them. The
# DRIVE-PAIR ports' own fwd_sign comes from calibration.fwd_sign_left/right
# when the robot JSON supplies it (fwd_sign_for_ports() below) -- this
# placeholder is also its own fallback when the JSON omits the drive pair's
# values too, unchanged by sprint 114 (out of ticket 002's explicit scope).
FWD_SIGN = 1

# mm/deg placeholder, same shape/scope as FWD_SIGN above (the legacy
# firmware's ml/mr default; docs/protocol-v2.md's Named Key Table).
TRAVEL_CALIB_PLACEHOLDER = 0.487


# ---------------------------------------------------------------------------
# Config resolution (mirrors scripts/gen_default_config.py so both generators
# read the same active robot config).
# ---------------------------------------------------------------------------

def _load_json(path: Path):
    return json.loads(path.read_text())


def load_robot_config():
    """Return (config_dict, source_path_str) or ({}, '(firmware defaults)')."""
    env_path = os.environ.get("ROBOT_CONFIG")
    if env_path:
        p = Path(env_path)
        if not p.is_absolute():
            p = REPO_ROOT / p
        try:
            return _load_json(p), str(p)
        except Exception as e:
            print(f"gen_boot_config: ROBOT_CONFIG={p} unreadable: {e}", file=sys.stderr)

    active = REPO_ROOT / "data" / "robots" / "active_robot.json"
    if active.exists():
        try:
            data = _load_json(active)
        except Exception as e:
            print(f"gen_boot_config: {active} unreadable: {e}", file=sys.stderr)
            return {}, "(firmware defaults)"

        if "identity" in data or "schema_version" in data:
            return data, str(active)

        if "path" in data:
            target = REPO_ROOT / data["path"]
            try:
                return _load_json(target), str(target)
            except Exception as e:
                print(f"gen_boot_config: {target} unreadable: {e}", file=sys.stderr)

    print("gen_boot_config: no robot config found -- every behavioral key is "
          "required (sprint 114 config-as-truth); the build will fail on the "
          "first missing key", file=sys.stderr)
    return {}, "(firmware defaults)"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _get(d, *keys, default=None):
    """Safely traverse a chain of dict keys; return default if any is missing."""
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return default if cur is None else cur


class MissingRobotConfigKeyError(RuntimeError):
    """Raised by a ``*_for_config()`` mapping when a required robot-JSON key
    is absent (or explicitly null). Sprint 114 (config-as-truth completion):
    every BEHAVIORAL field this generator bakes must come from the active
    robot JSON -- there is no longer a source-side Python fallback for any
    of them (see this module's own docstring and sprint 114's Architecture
    Boundary list). Structural/placeholder fields (K_MOTOR_COUNT, LEFT_PORT/
    RIGHT_PORT, TRAVEL_CALIB_PLACEHOLDER, FWD_SIGN, polled_for_ports()) are
    NOT affected -- they stay compile-time constants, per that same list.

    Carries just the dotted key path at the point it is first raised, so a
    bare unit test calling a ``*_for_config()`` function directly (e.g.
    ``heading_gains_for_config({})``) gets a self-contained message with no
    source-path context needed. ``generate()`` catches this and calls
    ``with_source()`` to attach the resolved JSON path once one is known, so
    the end-to-end generator run (``main()``) reports both the key and the
    file -- this ticket's own acceptance criterion.
    """

    def __init__(self, key_path: str, source_path: str | None = None):
        self.key_path = key_path
        self.source_path = source_path
        super().__init__(self._message())

    def _message(self) -> str:
        where = self.source_path if self.source_path is not None else "the active robot config"
        return (
            f"gen_boot_config: required key '{self.key_path}' missing from {where} "
            "-- config-as-truth (sprint 114): this field has no source-side "
            "default; add it to the robot JSON."
        )

    def with_source(self, source_path: str) -> "MissingRobotConfigKeyError":
        """Return a copy of this error with the JSON source path attached."""
        return MissingRobotConfigKeyError(self.key_path, source_path)


def _require(cfg: dict, *keys):
    """Traverse a chain of dict keys; raise MissingRobotConfigKeyError if any
    is missing or explicitly null. Mirrors _get()'s traversal shape, but
    with no ``default`` -- every caller of this helper is a field sprint 114
    made required; a robot JSON that omits it is an incomplete build, not a
    silently-degraded one."""
    cur = cfg
    for k in keys:
        if not isinstance(cur, dict) or k not in cur or cur[k] is None:
            raise MissingRobotConfigKeyError(".".join(str(k) for k in keys))
        cur = cur[k]
    return cur


def _f(v) -> str:
    """Format a Python float as a C++ float literal."""
    s = f"{float(v):.7g}"
    if "." not in s and "e" not in s:
        s += ".0"
    return s + "f"


# ---------------------------------------------------------------------------
# Code generation
# ---------------------------------------------------------------------------

def travel_calib_for_ports(cfg: dict):
    """Return a list of kMotorCount mm/deg values, one per port (1..N).

    The left/right drive-pair ports take calibration.mm_per_wheel_deg_left/right
    when the robot JSON supplies them; every other port (and the pair, when the
    JSON omits them) uses the placeholder.
    """
    cal = cfg.get("calibration", {}) or {}
    left  = _get(cal, "mm_per_wheel_deg_left")
    right = _get(cal, "mm_per_wheel_deg_right")
    out = []
    for port in range(1, K_MOTOR_COUNT + 1):
        if port == LEFT_PORT and left is not None:
            out.append(float(left))
        elif port == RIGHT_PORT and right is not None:
            out.append(float(right))
        else:
            out.append(TRAVEL_CALIB_PLACEHOLDER)
    return out


def polled_for_ports():
    """Return a list of kMotorCount `polled` bools, one per port (1..N).

    091-002: the I2C flip-flop poll-schedule membership fact -- which ports
    Subsystems::NezhaHardware's brick flip-flop sequencer samples/dispatches
    each tick(). True for the drive-pair ports (LEFT_PORT/RIGHT_PORT);
    false for every other port, mirroring travel_calib_for_ports()'s/
    fwd_sign_for_ports()'s own LEFT_PORT/RIGHT_PORT-vs-"every other port"
    specialization pattern exactly -- unlike those two, there is no robot-JSON
    override: poll membership is a firmware-scheduling fact, not a
    per-robot calibration value, so this is the same for every robot.
    """
    return [port in (LEFT_PORT, RIGHT_PORT) for port in range(1, K_MOTOR_COUNT + 1)]


def fwd_sign_for_ports(cfg: dict):
    """Return a list of kMotorCount fwd_sign values, one per port (1..N).

    Mirrors travel_calib_for_ports()'s exact shape: the left/right drive-pair
    ports take calibration.fwd_sign_left/right when the robot JSON supplies
    them; every other port (and the pair, when the JSON omits them) uses the
    FWD_SIGN placeholder.

    Unlike travel_calib, the drive pair is mirror-mounted (088-002 —
    clasi/issues/tovez-drive-motor-reversed-fwd-sign.md), so left and right
    are EXPECTED to differ in sign -- a straight-drive command with equal
    L/R targets must spin the two wheels in opposite raw-command directions
    to travel the same physical direction.
    """
    cal = cfg.get("calibration", {}) or {}
    left  = _get(cal, "fwd_sign_left")
    right = _get(cal, "fwd_sign_right")
    out = []
    for port in range(1, K_MOTOR_COUNT + 1):
        if port == LEFT_PORT and left is not None:
            out.append(int(left))
        elif port == RIGHT_PORT and right is not None:
            out.append(int(right))
        else:
            out.append(FWD_SIGN)
    return out


def otos_boot_config_values(cfg: dict):
    """Return (offsetX, offsetY, offsetYaw, linearScale, angularScale) for the
    OtosBootConfig struct (086-005), reading geometry.odometry_offset_mm's
    x/y/yaw_rad and calibration.otos_linear_scale/otos_angular_scale.

    All five are REQUIRED as of sprint 114 (config-as-truth completion) --
    a robot JSON missing any of them fails the generator loudly rather than
    silently substituting the old identity defaults (zero offset, 1.0 scale).
    """
    offset_x   = _require(cfg, "geometry", "odometry_offset_mm", "x")
    offset_y   = _require(cfg, "geometry", "odometry_offset_mm", "y")
    offset_yaw = _require(cfg, "geometry", "odometry_offset_mm", "yaw_rad")
    linear_scale  = _require(cfg, "calibration", "otos_linear_scale")
    angular_scale = _require(cfg, "calibration", "otos_angular_scale")
    return (float(offset_x), float(offset_y), float(offset_yaw),
            float(linear_scale), float(angular_scale))


def vel_gains_for_config(cfg: dict):
    """Return (kp, ki, kff, i_max, kaw, filt_alpha) for the velocity PID.

    Read from the robot JSON's ``control`` block -- ALL SIX keys are
    REQUIRED as of sprint 114 (config-as-truth completion; previously fell
    back to bench-tuned firmware defaults when absent). NOTE: these keys
    must be expressed in the NEW NezhaMotor duty [-1,1] plant scale
    (kp ~ 0.002, kff ~ 0.0015), NOT the old RobotConfig PWM-percent scale
    (kp ~ 0.3) — the robot JSON's ``control._vel_gains_domain`` marker
    documents this.
    """
    kp   = _require(cfg, "control", "vel_kp")
    ki   = _require(cfg, "control", "vel_ki")
    kff  = _require(cfg, "control", "vel_kff")
    imax = _require(cfg, "control", "vel_imax")
    kaw  = _require(cfg, "control", "vel_kaw")
    filt = _require(cfg, "control", "vel_filt")
    return float(kp), float(ki), float(kff), float(imax), float(kaw), float(filt)


def heading_gains_for_config(cfg: dict):
    """Return (heading_kp, heading_kd) for the outer heading-loop PD
    (098-001, architecture-update.md M1/M2). Both keys REQUIRED as of
    sprint 114 (config-as-truth completion) -- previously fell back to a
    conservative firmware default (Kp=6.0/Kd=0.0, 112-004) when absent.
    """
    kp = _require(cfg, "control", "heading_kp")
    kd = _require(cfg, "control", "heading_kd")
    return float(kp), float(kd)


_HEADING_SOURCE_WIRE_NAMES = {
    "auto": "msg::HeadingSourceMode::HEADING_SOURCE_AUTO",
    "otos": "msg::HeadingSourceMode::HEADING_SOURCE_FORCE_OTOS",
    "encoder": "msg::HeadingSourceMode::HEADING_SOURCE_FORCE_ENCODER",
}


def heading_source_for_config(cfg: dict) -> str:
    """Return the C++ msg::HeadingSourceMode enumerator literal for the robot
    JSON's control.heading_source key (case-insensitive "auto"/"otos"/
    "encoder", 109-005). REQUIRED as of sprint 114 (config-as-truth
    completion) -- previously fell back to "auto" when absent. An
    unrecognized-but-PRESENT string still resolves to AUTO -- that is a
    value-validation question, not a missing-key one, and stays out of this
    ticket's scope."""
    raw = str(_require(cfg, "control", "heading_source")).strip().lower()
    return _HEADING_SOURCE_WIRE_NAMES.get(raw, _HEADING_SOURCE_WIRE_NAMES["auto"])


def heading_dwell_for_config(cfg: dict):
    """Return (heading_dwell_tol, heading_dwell_rate) in [rad]/[rad/s].

    Sprint 114 (config-as-truth completion): NEWLY wired to the robot JSON's
    ``control.heading_dwell_tol_deg``/``control.heading_dwell_rate_dps``
    (both required) -- before this ticket these two were hardcoded and never
    read from cfg at all, the one field pair in this generator with no JSON
    path whatsoever."""
    tol_deg  = _require(cfg, "control", "heading_dwell_tol_deg")
    rate_dps = _require(cfg, "control", "heading_dwell_rate_dps")
    return math.radians(float(tol_deg)), math.radians(float(rate_dps))


def lead_compensation_for_config(cfg: dict):
    """Return (heading_lead_bias, plan_lead, terminal_lead) in [s] --
    109-010's three independently-tunable lead-compensation Δt's. All three
    REQUIRED as of sprint 114 (config-as-truth completion); see
    data/robots/tovez_nocal.json's control block (or git blame on this
    function's pre-sprint-114 fallback constants) for the fitted-value
    derivation."""
    heading_lead_bias = _require(cfg, "control", "heading_lead_bias")
    plan_lead = _require(cfg, "control", "plan_lead")
    terminal_lead = _require(cfg, "control", "terminal_lead")
    return float(heading_lead_bias), float(plan_lead), float(terminal_lead)


def min_speed_for_config(cfg: dict):
    """Return min_speed (PlannerConfig field 10, 100-007) -- Drive::
    tracker's own pivot-mode gate (`fabsf(ref.v) < limits.minSpeed`) needs a
    small positive threshold, never left unset. REQUIRED as of sprint 114
    (config-as-truth completion)."""
    return float(_require(cfg, "control", "min_speed"))


def profile_rot_limits_for_config(cfg: dict):
    """Return (yaw_rate_max, yaw_acc_max) in [rad/s] / [rad/s^2] for the
    rotational master-profile ceiling (PlannerConfig fields 4-5, 100-014).
    Reads ``control.yaw_rate_max`` [deg/s] and ``control.max_rot_accel_dps2``
    [deg/s^2], converted to radians -- BOTH REQUIRED as of sprint 114
    (config-as-truth completion; previously each independently fell back to
    a firmware default when absent)."""
    yr = _require(cfg, "control", "yaw_rate_max")        # [deg/s]
    ya = _require(cfg, "control", "max_rot_accel_dps2")  # [deg/s^2]
    return math.radians(float(yr)), math.radians(float(ya))


def arrive_dwell_for_config(cfg: dict):
    """Return arrive_dwell (msg::PlannerConfig field 31, 100-001 -- the sole
    survivor of the original 17-field Drive::Limits/tracker/policy span, see
    planner.proto's own header comment for the 111-004 field accounting).
    REQUIRED as of sprint 114 (config-as-truth completion)."""
    return float(_require(cfg, "control", "arrive_dwell"))


def actuation_lag_for_config(cfg: dict):
    """Return actuation_lag (msg::PlannerConfig field 38, 112-002) -- App::
    Drive's own model feedforward gain (Drive::tick() adds
    actuation_lag * a onto each wheel's velocity target). REQUIRED as of
    sprint 114 (config-as-truth completion)."""
    return float(_require(cfg, "control", "actuation_lag"))


def distance_gains_for_config(cfg: dict):
    """Return (distance_kp, distance_tol) for msg::PlannerConfig fields
    39/40 (112-003) -- App::Pilot's own bounded linear position-feedback
    trim and Motion::Executor's own linear completion tolerance. Both
    REQUIRED as of sprint 114 (config-as-truth completion)."""
    distance_kp = _require(cfg, "control", "distance_kp")
    distance_tol = _require(cfg, "control", "distance_tol")
    return float(distance_kp), float(distance_tol)


def model_tau_for_config(cfg: dict):
    """Return (model_tau_lin, model_tau_ang) for msg::PlannerConfig fields
    41/42 (113-001) -- App::Pilot's own two-stage model-reference feedback
    plant-lag time constants (pilot.h's modelTauLin_/modelTauAng_). Both
    REQUIRED as of sprint 114 (config-as-truth completion)."""
    model_tau_lin = _require(cfg, "control", "model_tau_lin")
    model_tau_ang = _require(cfg, "control", "model_tau_ang")
    return float(model_tau_lin), float(model_tau_ang)


def motion_limits_for_config(cfg: dict):
    """Return (a_max, a_decel, v_body_max, j_max, yaw_jerk_max) for
    msg::PlannerConfig's five non-rotational motion-limit fields (098-001 --
    moved verbatim from main.cpp's old hand-written defaultMotionConfig()).

    ALL FIVE are REQUIRED as of sprint 114 (config-as-truth completion) --
    previously these were the only PlannerConfig fields with NO per-robot
    JSON mapping at all (generate() referenced the module-level DEFAULT
    constants directly, unconditionally). Every shipped robot JSON now
    carries control.a_max/a_decel/v_body_max/j_max/yaw_jerk_max seeded with
    the same numeric values main.cpp used to hardcode (value-preserving
    migration)."""
    a_max        = _require(cfg, "control", "a_max")         # [mm/s^2]
    a_decel      = _require(cfg, "control", "a_decel")       # [mm/s^2]
    v_body_max   = _require(cfg, "control", "v_body_max")    # [mm/s]
    j_max        = _require(cfg, "control", "j_max")         # [mm/s^3]
    yaw_jerk_max = _require(cfg, "control", "yaw_jerk_max")  # [rad/s^3]
    return (float(a_max), float(a_decel), float(v_body_max),
            float(j_max), float(yaw_jerk_max))


def trackwidth_for_config(cfg: dict) -> float:
    """Return geometry.trackwidth [mm] -> DrivetrainConfig.trackwidth.
    REQUIRED as of sprint 114 (config-as-truth completion) -- previously
    fell back to a 128.0mm placeholder when absent."""
    return float(_require(cfg, "geometry", "trackwidth"))


def generate(cfg: dict, source_path: str) -> str:
    try:
        trackwidth   = trackwidth_for_config(cfg)
        vel_kp, vel_ki, vel_kff, vel_imax, vel_kaw, vel_filt = vel_gains_for_config(cfg)
        travel_calib = travel_calib_for_ports(cfg)
        fwd_sign     = fwd_sign_for_ports(cfg)
        polled       = polled_for_ports()
        (otos_offset_x, otos_offset_y, otos_offset_yaw,
         otos_linear_scale, otos_angular_scale) = otos_boot_config_values(cfg)
        heading_kp, heading_kd = heading_gains_for_config(cfg)
        heading_source_wire = heading_source_for_config(cfg)
        heading_dwell_tol, heading_dwell_rate = heading_dwell_for_config(cfg)
        heading_lead_bias, plan_lead, terminal_lead = lead_compensation_for_config(cfg)
        yaw_rate_max, yaw_acc_max = profile_rot_limits_for_config(cfg)
        min_speed = min_speed_for_config(cfg)
        arrive_dwell = arrive_dwell_for_config(cfg)
        actuation_lag = actuation_lag_for_config(cfg)
        distance_kp, distance_tol = distance_gains_for_config(cfg)
        model_tau_lin, model_tau_ang = model_tau_for_config(cfg)
        a_max, a_decel, v_body_max, j_max, yaw_jerk_max = motion_limits_for_config(cfg)
    except MissingRobotConfigKeyError as e:
        raise e.with_source(source_path) from e

    calib_lines = "\n".join(
        f"    out[{i}].setTravelCalib({_f(v)});   // [mm/deg] port {i + 1}"
        for i, v in enumerate(travel_calib)
    )

    fwd_sign_lines = "\n".join(
        f"    out[{i}].setFwdSign({v});   // port {i + 1}"
        for i, v in enumerate(fwd_sign)
    )

    polled_lines = "\n".join(
        f"    out[{i}].setPolled({'true' if v else 'false'});   // port {i + 1}"
        for i, v in enumerate(polled)
    )

    return f"""\
// AUTO-GENERATED — do not edit by hand.
// Regenerated by scripts/gen_boot_config.py before each firmware build.
// Source: {source_path}
//
// The whole file is the robot's boot configuration: the per-port
// msg::MotorConfig defaults and the msg::DrivetrainConfig default, with
// per-robot calibration baked in from the robot JSON above. main.cpp calls
// these; it never hardcodes calibration. See src/firm/config/boot_config.h.

#include "config/boot_config.h"

namespace Config {{

void defaultMotorConfigs(msg::MotorConfig* out) {{
    // Velocity PID gains — baked from the robot JSON's control.vel_* keys
    // (093: now in the NezhaMotor duty [-1,1] plant scale, see the JSON's
    // control._vel_gains_domain marker), falling back to bench-tuned firmware
    // defaults when absent. Live-correctable per motor via `DEV M <n> CFG`.
    msg::Gains velGains;
    velGains.kp = {_f(vel_kp)};
    velGains.ki = {_f(vel_ki)};
    velGains.kff = {_f(vel_kff)};
    velGains.i_max = {_f(vel_imax)};
    velGains.kaw = {_f(vel_kaw)};   // anti-windup back-calculation (velocity_pid.cpp; 0 = off)

    // reversal_dwell / output_deadband are left unset (.has == false) on
    // purpose — Hal::Motor::configure() applies the real ship defaults (100 ms
    // / 0.03) whenever a config arrives unset; that is the one place those
    // defaults live.
    for (uint32_t i = 0; i < kMotorConfigCount; ++i) {{
        out[i] = msg::MotorConfig();
        out[i].setPort(i + 1);
        out[i].setVelGains(velGains);
        // EMA coeff — from control.vel_filt (fallback default); a=0 would pin
        // reported velocity at 0 forever regardless of real motion.
        out[i].setVelFiltAlpha({_f(vel_filt)});
    }}

    // Per-port forward-sign — baked from the robot JSON's calibration.
    // fwd_sign_{{left,right}} for the drive-pair ports
    // (ports {LEFT_PORT}/{RIGHT_PORT}); other ports use the bench placeholder
    // ({FWD_SIGN}). The drive pair is mirror-mounted, so left/right are
    // expected to differ in sign (088-002 —
    // clasi/issues/tovez-drive-motor-reversed-fwd-sign.md).
{fwd_sign_lines}

    // Per-port encoder travel calibration — baked from the robot JSON's
    // calibration.mm_per_wheel_deg_{{left,right}} for the drive-pair ports
    // (ports {LEFT_PORT}/{RIGHT_PORT}); other ports use the bench placeholder.
{calib_lines}

    // Per-port I2C flip-flop poll-schedule membership (091-002) — true for
    // the drive-pair ports ({LEFT_PORT}/{RIGHT_PORT}), false otherwise. Not
    // robot-JSON-configurable (a firmware-scheduling fact, not per-robot
    // calibration); live-adjustable via `DEV M <n> CFG polled=true` for a
    // bench rig's own non-drive-pair port (docs/protocol-v2.md §16).
{polled_lines}
}}

msg::DrivetrainConfig defaultDrivetrainConfig() {{
    msg::DrivetrainConfig cfg;
    cfg.setTrackwidth({_f(trackwidth)});   // [mm] baked from robot geometry
    // The drive-pair port binding lives in DrivetrainConfig (the robot's
    // normal drive pair); the coupled bench rig re-binds via `DEV DT PORTS`.
    cfg.setLeftPort({LEFT_PORT});
    cfg.setRightPort({RIGHT_PORT});
    return cfg;
}}

OtosBootConfig defaultOtosBootConfig() {{
    // 086-005 — additive to defaultMotorConfigs()/defaultDrivetrainConfig()
    // above; no existing mapping touched. Baked from the robot JSON's
    // geometry.odometry_offset_mm (x/y/yaw_rad) and calibration.
    // otos_linear_scale/otos_angular_scale where present; identity defaults
    // (zero offset, 1.0 scale) otherwise. Boot-time-baked only -- see
    // OtosBootConfig's own doc comment (src/firm/config/boot_config.h) for why
    // this is never a live SET/wire surface.
    OtosBootConfig cfg;
    cfg.offsetX = {_f(otos_offset_x)};        // [mm]
    cfg.offsetY = {_f(otos_offset_y)};        // [mm]
    cfg.offsetYaw = {_f(otos_offset_yaw)};    // [rad]
    cfg.linearScale = {_f(otos_linear_scale)};
    cfg.angularScale = {_f(otos_angular_scale)};
    return cfg;
}}

msg::PlannerConfig defaultPlannerConfig() {{
    // 098-001 — the motion-limit fields below are moved verbatim from
    // main.cpp's hand-written defaultMotionConfig() (same numeric values,
    // same units — not renumbered or retuned by this move), the one
    // PlannerConfig boot path that lived OUTSIDE this generator until now.
    // heading_kp/heading_kd are the new outer heading-loop PD gains
    // (architecture-update.md M1/M2), baked from the robot JSON's
    // control.heading_kp/heading_kd, falling back to conservative firmware
    // starting defaults when absent. min_speed is NO LONGER left unset
    // (100-007, THE CUTOVER) -- see MIN_SPEED_DEFAULT's own comment above
    // for why 0.0f silently broke pivot-mode detection the moment
    // source/drive/tracker.cpp became this field's first live reader.
    // arrive_tol/turn_in_place_gate, which used to be left unset here, were
    // removed as dead wire fields in 111-004 -- they no longer exist to be
    // set at all.
    //
    // arrive_dwell (100-001 — Drive::Limits' wire/config source,
    // architecture-update.md M1/Decision 2) is the sole survivor of the
    // original fields-15-31 span; baked from the robot JSON's control.*
    // keys via arrive_dwell_for_config(), falling back to
    // ARRIVE_DWELL_DEFAULT when absent. Its 16 dead siblings
    // (v_wheel_max..arrive_vel_tol) were removed in 111-004 -- see this
    // message's own header comment in planner.proto for the full
    // per-field accounting.
    msg::PlannerConfig cfg;
    cfg.setAMax({_f(a_max)});               // [mm/s^2]
    cfg.setADecel({_f(a_decel)});             // [mm/s^2]
    cfg.setVBodyMax({_f(v_body_max)});           // [mm/s]
    cfg.setYawRateMax({_f(yaw_rate_max)});         // [rad/s] (control.yaw_rate_max [deg/s])
    cfg.setYawAccMax({_f(yaw_acc_max)});          // [rad/s^2] (control.max_rot_accel_dps2 [deg/s^2])
    cfg.setJMax({_f(j_max)});                // [mm/s^3] ~6x a_max -- ~0.16s jerk-limited edges
    cfg.setYawJerkMax({_f(yaw_jerk_max)});         // [rad/s^3] ~5x yaw_acc_max -- ~0.2s
    cfg.setHeadingKp({_f(heading_kp)});              // [1/s] outer heading-loop proportional gain
    cfg.setHeadingKd({_f(heading_kd)});              // dimensionless outer heading-loop derivative gain
    cfg.setMinSpeed({_f(min_speed)});               // [mm/s] Drive:: tracker pivot-mode threshold (100-007)
    cfg.setArriveDwell({_f(arrive_dwell)});             // [s]

    // 109-005: App::HeadingSource per-robot policy override + the heading-
    // dwell completion gate. heading_source baked from the robot JSON's
    // control.heading_source ("auto"/"otos"/"encoder"); the dwell
    // tolerance/rate are firmware defaults today (no robot-JSON key yet --
    // see heading_dwell_for_config()'s own comment).
    cfg.setHeadingSource({heading_source_wire});
    cfg.setHeadingDwellTol({_f(heading_dwell_tol)});        // [rad]
    cfg.setHeadingDwellRate({_f(heading_dwell_rate)});       // [rad/s]

    // 109-010: three independently-tunable lead-compensation Δt's, fitted
    // from the rate-sweep regression -- see planner.proto's own field
    // comments and src/firm/motion/DESIGN.md's "Turn-error characterization"
    // entry for the full derivation.
    cfg.setHeadingLeadBias({_f(heading_lead_bias)});        // [s] locus 1
    cfg.setPlanLead({_f(plan_lead)});                // [s] locus 2
    cfg.setTerminalLead({_f(terminal_lead)});           // [s] locus 3

    // 112-002: App::Drive's own model feedforward gain (Drive::tick() adds
    // actuation_lag * a onto each wheel's velocity target). Motion::
    // kDeadTime's own bench-derived value (120-140ms) by default -- see
    // ACTUATION_LAG_DEFAULT's own comment above.
    cfg.setActuationLag({_f(actuation_lag)});           // [s]

    // 112-003: App::Pilot's own bounded linear position-feedback trim --
    // distance_kp is the trim's gain, distance_tol is Motion::Executor's
    // own unified completion rule's linear tolerance (112-004 wired this
    // live, replacing the hardcoded Motion::kDistanceSettleEpsilonMm
    // constant it repurposes the role of). See DISTANCE_KP_DEFAULT/
    // DISTANCE_TOL_DEFAULT's own comment above for the deadband-inequality
    // derivation AND 112-004's own closed-loop-convergence retune.
    cfg.setDistanceKp({_f(distance_kp)});              // [1/s]
    cfg.setDistanceTol({_f(distance_tol)});             // [mm]

    // 113-001: App::Pilot's own two-stage model-reference feedback plant-lag
    // time constants (pilot.h's modelTauLin_/modelTauAng_) -- previously
    // hardcoded with no config path at all. See MODEL_TAU_LIN_DEFAULT/
    // MODEL_TAU_ANG_DEFAULT's own comment above.
    cfg.setModelTauLin({_f(model_tau_lin)});            // [s]
    cfg.setModelTauAng({_f(model_tau_ang)});            // [s]
    return cfg;
}}

}}  // namespace Config
"""


def _display_path(source_path: str) -> str:
    """Repo-relative path so the committed file is stable across checkouts."""
    try:
        return str(Path(source_path).resolve().relative_to(REPO_ROOT))
    except (ValueError, OSError):
        return source_path   # sentinel like "(firmware defaults)", or outside the repo


def main():
    cfg, source_path = load_robot_config()
    display_path = _display_path(source_path)
    try:
        content = generate(cfg, display_path)
    except MissingRobotConfigKeyError as e:
        # Config-as-truth (sprint 114): fail the build loudly, naming the
        # missing key and the JSON path -- never emit a placeholder file.
        print(str(e), file=sys.stderr)
        sys.exit(1)
    OUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    OUT_FILE.write_text(content)
    print(f"gen_boot_config: wrote {OUT_FILE.relative_to(REPO_ROOT)}", file=sys.stderr)


if __name__ == "__main__":
    main()
