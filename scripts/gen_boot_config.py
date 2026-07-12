#!/usr/bin/env python3
"""Generate source/config/boot_config.cpp from the active robot JSON config.

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

What is baked from the robot JSON vs. held as a firmware default
----------------------------------------------------------------
Baked from JSON when present (matching semantics, so no behaviour surprise):
  * geometry.trackwidth               -> DrivetrainConfig.trackwidth
  * calibration.mm_per_wheel_deg_left  -> the left-port motor's travel_calib
  * calibration.mm_per_wheel_deg_right -> the right-port motor's travel_calib
  * calibration.fwd_sign_left  -> the left-port motor's fwd_sign
  * calibration.fwd_sign_right -> the right-port motor's fwd_sign
    (088-002 — the drive pair is mirror-mounted, so these are EXPECTED to
    differ in sign between the two ports, unlike travel_calib; see
    fwd_sign_for_ports() and clasi/issues/tovez-drive-motor-reversed-fwd-sign.md)
  * geometry.odometry_offset_mm (x/y/yaw_rad)         -> OtosBootConfig.offsetX/offsetY/offsetYaw
  * calibration.otos_linear_scale/otos_angular_scale  -> OtosBootConfig.linearScale/angularScale
    (086-005 — additive to the mappings above; see otos_boot_config_values()
    and OtosBootConfig's own doc comment in source/config/boot_config.h for why
    this is boot-time-baked only, never a live SET/wire surface)

Held as bench-tuned firmware DEFAULTS below and NOT read from the old-tree JSON
`control.*` keys — those describe the old RobotConfig velocity loop and are in a
different unit/plant scale (kp ~ 0.3, not ~ 0.002); mapping them onto the new
MotorConfig would silently break the velocity loop:
  * the velocity PID gains (kp/ki/kff/i_max)
  * vel_filt_alpha (the EMA coefficient — a value of 0 pins reported velocity at
    0 forever regardless of real motion; sprint 077-007 bench story)
  * the drive-pair port binding, the FWD_SIGN=1 placeholder for any port the
    JSON doesn't cover, and the mm/deg PLACEHOLDER
  * the per-port `polled` I2C flip-flop poll-schedule membership (091-002):
    true for the drive-pair ports, false otherwise -- a firmware-scheduling
    fact, never robot-JSON-configurable; see polled_for_ports()

When no robot config is found, everything falls back to these same firmware
defaults so the build always succeeds.
"""

import json
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT_FILE  = REPO_ROOT / "source" / "config" / "boot_config.cpp"

# --- Bench-tuned firmware defaults (NOT from the robot JSON) ----------------
# Ports 1..kMotorCount; matches Subsystems::NezhaHardware::kMotorCount, asserted
# in main.cpp. Keep in sync if the port count ever changes.
K_MOTOR_COUNT = 4

# Velocity PID gains, bench-tuned on the stand (Tovez, ports 1/3, targets
# 120/150/-100 mm/s): converges within ~1.5 s, small (~10%) overshoot, holds
# within the dev_exercise.py / pid_hold_speed.py tolerance bands (sprint
# 077-007). Live-correctable per motor via `DEV M <n> CFG`.
VEL_KP    = 0.0022
VEL_KI    = 0.0018
VEL_KFF   = 0.0038
VEL_IMAX  = 0.3
VEL_KAW   = 0.0   # anti-windup back-calculation gain (0 = off)

# EMA coefficient in NezhaMotor::tick()'s
# `filteredVelocity_ = a*rawVel + (1-a)*filteredVelocity_`. a=0 pins reported
# velocity at 0 forever regardless of real motion (077-007 silent-failure gap);
# 0.3 was bench-confirmed to produce real, converging vel= readings.
VEL_FILT_ALPHA = 0.3

# fwd_sign multiplies BOTH the drive command and the encoder reading. +1 is the
# bench placeholder for any port the robot JSON doesn't cover (calibration.
# fwd_sign_left/fwd_sign_right, 088-002); a specific motor's real sense can
# still be corrected live via `DEV M <n> CFG`.
FWD_SIGN = 1

# mm/deg placeholder used for any motor whose travel calibration is not supplied
# by the robot JSON (the legacy firmware's ml/mr default; docs/protocol-v2.md's
# Named Key Table). Live-correctable via `DEV M <n> CFG`.
TRAVEL_CALIB_PLACEHOLDER = 0.487

# The drive-pair port binding (the robot's normal drive pair). The coupled bench
# rig re-binds at runtime via `DEV DT PORTS 3 4`. An unseeded (zero) port would
# address motor(0), which NezhaHardware::motor() clamps to port 4 — silently
# wrong, not a crash — so these are always seeded.
LEFT_PORT  = 1
RIGHT_PORT = 2

# Trackwidth placeholder [mm] when the robot JSON does not supply geometry.
TRACKWIDTH_DEFAULT = 128.0

# OTOS lever-arm mounting offset defaults (086-005) — zero offset is the
# identity case (LeverArm::sensorToCentre()/centreToSensor() are no-ops when
# offsetX == offsetY == 0, source/hal/lever_arm.h), i.e. "no config = no
# correction", matching every other placeholder default in this file.
OTOS_OFFSET_X_DEFAULT   = 0.0   # [mm]
OTOS_OFFSET_Y_DEFAULT   = 0.0   # [mm]
OTOS_OFFSET_YAW_DEFAULT = 0.0   # [rad]

# OTOS linear/angular scale multiplier defaults (086-005). 1.0 == no
# correction (the OTOS chip's own scaleToInt8()-style conversion, applied
# once at Hal::OtosOdometer::begin() — ticket 086-006 — maps a 1.0 multiplier
# to register scalar 0, i.e. an unmodified chip reading).
OTOS_LINEAR_SCALE_DEFAULT  = 1.0
OTOS_ANGULAR_SCALE_DEFAULT = 1.0


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

    print("gen_boot_config: no robot config found — using firmware defaults", file=sys.stderr)
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
    x/y/yaw_rad and calibration.otos_linear_scale/otos_angular_scale, falling
    back to the identity defaults above when either is absent from the robot
    JSON (matching every other mapping's fall-back-to-firmware-default
    behavior in this file).
    """
    offset_x   = _get(cfg, "geometry", "odometry_offset_mm", "x",
                       default=OTOS_OFFSET_X_DEFAULT)
    offset_y   = _get(cfg, "geometry", "odometry_offset_mm", "y",
                       default=OTOS_OFFSET_Y_DEFAULT)
    offset_yaw = _get(cfg, "geometry", "odometry_offset_mm", "yaw_rad",
                       default=OTOS_OFFSET_YAW_DEFAULT)
    linear_scale  = _get(cfg, "calibration", "otos_linear_scale",
                         default=OTOS_LINEAR_SCALE_DEFAULT)
    angular_scale = _get(cfg, "calibration", "otos_angular_scale",
                         default=OTOS_ANGULAR_SCALE_DEFAULT)
    return (float(offset_x), float(offset_y), float(offset_yaw),
            float(linear_scale), float(angular_scale))


def vel_gains_for_config(cfg: dict):
    """Return (kp, ki, kff, i_max, filt_alpha) for the velocity PID.

    Read from the robot JSON's ``control`` block when present, falling back to
    the bench-tuned firmware defaults above. NOTE: these keys must be expressed
    in the NEW NezhaMotor duty [-1,1] plant scale (kp ~ 0.002, kff ~ 0.0015),
    NOT the old RobotConfig PWM-percent scale (kp ~ 0.3) — the robot JSON's
    ``control._vel_gains_domain`` marker documents this. A JSON still carrying
    old-scale values would silently break the loop, so a robot config that has
    not been migrated should simply omit these keys and inherit the defaults.
    """
    ctrl = cfg.get("control", {}) or {}
    kp   = _get(ctrl, "vel_kp",   default=VEL_KP)
    ki   = _get(ctrl, "vel_ki",   default=VEL_KI)
    kff  = _get(ctrl, "vel_kff",  default=VEL_KFF)
    imax = _get(ctrl, "vel_imax", default=VEL_IMAX)
    kaw  = _get(ctrl, "vel_kaw",  default=VEL_KAW)
    filt = _get(ctrl, "vel_filt", default=VEL_FILT_ALPHA)
    return float(kp), float(ki), float(kff), float(imax), float(kaw), float(filt)


def generate(cfg: dict, source_path: str) -> str:
    trackwidth   = _get(cfg, "geometry", "trackwidth", default=TRACKWIDTH_DEFAULT)
    vel_kp, vel_ki, vel_kff, vel_imax, vel_kaw, vel_filt = vel_gains_for_config(cfg)
    travel_calib = travel_calib_for_ports(cfg)
    fwd_sign     = fwd_sign_for_ports(cfg)
    polled       = polled_for_ports()
    (otos_offset_x, otos_offset_y, otos_offset_yaw,
     otos_linear_scale, otos_angular_scale) = otos_boot_config_values(cfg)

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
// these; it never hardcodes calibration. See source/config/boot_config.h.

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
    // OtosBootConfig's own doc comment (source/config/boot_config.h) for why
    // this is never a live SET/wire surface.
    OtosBootConfig cfg;
    cfg.offsetX = {_f(otos_offset_x)};        // [mm]
    cfg.offsetY = {_f(otos_offset_y)};        // [mm]
    cfg.offsetYaw = {_f(otos_offset_yaw)};    // [rad]
    cfg.linearScale = {_f(otos_linear_scale)};
    cfg.angularScale = {_f(otos_angular_scale)};
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
    content = generate(cfg, _display_path(source_path))
    OUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    OUT_FILE.write_text(content)
    print(f"gen_boot_config: wrote {OUT_FILE.relative_to(REPO_ROOT)}", file=sys.stderr)


if __name__ == "__main__":
    main()
