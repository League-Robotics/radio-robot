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
    and OtosBootConfig's own doc comment in src/firm/config/boot_config.h for why
    this is boot-time-baked only, never a live SET/wire surface)
  * control.heading_kp/control.heading_kd  -> PlannerConfig.heading_kp/heading_kd
    (098-001 — the outer heading-loop PD gains, per-robot tunable; see
    heading_gains_for_config() and architecture-update.md M1/M2. Also new
    this ticket: PlannerConfig's seven motion-limit fields (a_max/a_decel/
    v_body_max/yaw_rate_max/yaw_acc_max/j_max/yaw_jerk_max) are now baked
    HERE via defaultPlannerConfig(), moved verbatim off main.cpp's old
    hand-written defaultMotionConfig() — same bench-tuned firmware-default
    values as before, just no longer outside this generator's governance;
    they are not yet a robot-JSON-configurable mapping)

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
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
OUT_FILE  = REPO_ROOT / "src" / "firm" / "config" / "boot_config.cpp"

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

# Motion-limit defaults for msg::PlannerConfig (098-001 — moved verbatim from
# main.cpp's hand-written defaultMotionConfig(), the one PlannerConfig boot
# path that lived OUTSIDE this generator until now; see architecture-
# update.md M2). Same numeric values, same units — not renumbered or
# retuned by this move.
A_MAX_DEFAULT        = 800.0    # [mm/s^2]
A_DECEL_DEFAULT      = 800.0    # [mm/s^2]
V_BODY_MAX_DEFAULT   = 1000.0   # [mm/s]
YAW_RATE_MAX_DEFAULT = 6.0      # [rad/s]
YAW_ACC_MAX_DEFAULT  = 20.0     # [rad/s^2]
J_MAX_DEFAULT        = 5000.0   # [mm/s^3] ~6x a_max -- ~0.16s jerk-limited edges
YAW_JERK_MAX_DEFAULT = 100.0    # [rad/s^3] ~5x yaw_acc_max -- ~0.2s

# Outer heading-loop PD gain defaults (098-001 — sprint 098's new cascade,
# architecture-update.md M1/M2, Decision 2). Conservative STARTING values,
# not yet bench-tuned — heading_kp on the order of a few /s sits roughly a
# decade below the inner wheel-velocity loop's ~1-4 Hz corner
# (motion_control.ipynb); heading_kd starts at 0 (pure P, derivative off).
# Ticket 003 iterates both against tests/bench/turn_sweep.py --relay --both
# on the real plant.
HEADING_KP_DEFAULT = 3.0    # [1/s]
HEADING_KD_DEFAULT = 0.0    # dimensionless

# Drive::Limits/tracker/policy defaults for msg::PlannerConfig's fields
# 15-31 (100-001 -- motion-stack-v2 M1, architecture-update.md Decision 2).
# source/drive/ (landing tickets 002+) is self-contained and does NOT read
# DrivetrainConfig.v_wheel_max/steer_headroom (fields 10/11) -- these are
# genuinely NEW, separate fields, not a rename of the old ones. Every value
# below is a CONSERVATIVE STARTING firmware default (not yet bench-tuned),
# EXCEPT V_WHEEL_MAX_DEFAULT: 350.0 here is the generic firmware fallback
# for an uncharacterized robot (e.g. togov, per architecture-update.md Open
# Question 6); tovez.json overrides it with its own bench-MEASURED plateau
# (620.0), the same "generic firmware default vs. per-robot bench value"
# split heading_kp/heading_kd already established. See
# data/robots/tovez.json's own `_drive_limits_note` for the full per-field
# derivation and every documented ambiguity (trim_omega_max's arc-vs-pivot
# value, replan_err_pos/handoff_tol_pos's along-vs-cross approximation,
# handoff_tol_v's velocity-coupling-slope reading, steer_headroom/
# wheel_step_max's unanchored placeholders) -- ticket 001's completion
# notes carry the same derivation for reviewers who do not have the JSON
# open.
V_WHEEL_MAX_DEFAULT       = 350.0    # [mm/s]
STEER_HEADROOM_DEFAULT    = 20.0     # [mm/s]
WHEEL_STEP_MAX_DEFAULT    = 150.0    # [mm/s]
TRACK_K_S_DEFAULT         = 2.0      # [1/s]
TRACK_K_THETA_DEFAULT     = 6.0      # [1/s]
TRACK_K_CROSS_DEFAULT     = 1.5e-5   # [rad/mm^2]
TRIM_V_MAX_DEFAULT        = 120.0    # [mm/s]
TRIM_OMEGA_MAX_DEFAULT    = 2.0      # [rad/s] pivot-mode value; see _drive_limits_note
REPLAN_ERR_POS_DEFAULT    = 40.0     # [mm]
REPLAN_ERR_THETA_DEFAULT  = 0.15     # [rad]
REPLAN_HOLD_DEFAULT       = 0.2      # [s]
REPLAN_MIN_PERIOD_DEFAULT = 0.3      # [s]
REPLAN_MAX_DEFAULT        = 3.0      # dimensionless
HANDOFF_TOL_POS_DEFAULT   = 40.0     # [mm]
HANDOFF_TOL_V_DEFAULT     = 0.14     # [s] velocity-coupling slope; see _drive_limits_note
ARRIVE_VEL_TOL_DEFAULT    = 15.0     # [mm/s]
ARRIVE_DWELL_DEFAULT      = 0.15     # [s]

# MIN_SPEED_DEFAULT (100-007, THE CUTOVER): min_speed (PlannerConfig field
# 10) predates this sprint and was NEVER populated by this generator --
# "left unset (0.0f default)... main.cpp's old function never set them
# either" (see defaultPlannerConfig()'s own comment, below, kept verbatim
# for arrive_tol/turn_in_place_gate, which stay genuinely unused). That was
# harmless while nothing read it; ticket 100-007 makes it load-bearing for
# the first time -- source/drive/tracker.cpp's own pivot-mode gate is
# `fabsf(ref.v) < limits.minSpeed`, so a min_speed of EXACTLY 0.0 can never
# be true even for a genuine pivot (whose ref.v is the LITERAL 0.0f
# motion_plan.cpp's own isPivot_ branch sets), silently routing every pivot
# through the arc-mode trim law instead of the pivot-mode one. A small,
# conservative positive threshold (order of magnitude below any real
# cruise speed, matching policy.cpp's own kArriveTolVel=15.0f terminal
# velocity-tolerance scale) fixes this without narrowing arc-mode's own
# operating range. Starting value, not yet bench-tuned -- same posture as
# every other field 15-31 default above (M11 re-tunes against the real
# plant); overridable via a future robot JSON `control.min_speed` key,
# mirroring every other field's own override mechanism, once one exists.
MIN_SPEED_DEFAULT         = 10.0     # [mm/s]


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


def heading_gains_for_config(cfg: dict):
    """Return (heading_kp, heading_kd) for the outer heading-loop PD.

    Mirrors vel_gains_for_config()'s exact shape: read from the robot JSON's
    ``control`` block when present, falling back to the conservative firmware
    defaults above when either key is absent — an unmigrated robot JSON
    simply inherits Kp=3.0/Kd=0.0 (today's fallback discipline, same as every
    other mapping in this file).
    """
    ctrl = cfg.get("control", {}) or {}
    kp = _get(ctrl, "heading_kp", default=HEADING_KP_DEFAULT)
    kd = _get(ctrl, "heading_kd", default=HEADING_KD_DEFAULT)
    return float(kp), float(kd)


def min_speed_for_config(cfg: dict):
    """Return min_speed (PlannerConfig field 10) -- see MIN_SPEED_DEFAULT's
    own comment above for why this is no longer left at 0.0f. Read from the
    robot JSON's ``control.min_speed`` when present (mirroring every other
    mapping's fall-back discipline), else MIN_SPEED_DEFAULT."""
    ctrl = cfg.get("control", {}) or {}
    return float(_get(ctrl, "min_speed", default=MIN_SPEED_DEFAULT))


def profile_rot_limits_for_config(cfg: dict):
    """Return (yaw_rate_max, yaw_acc_max) in [rad/s] / [rad/s^2] for the
    rotational master-profile ceiling (PlannerConfig fields 4-5).

    Mirrors heading_gains_for_config()'s exact shape: read from the robot
    JSON's ``control`` block when present — ``control.yaw_rate_max`` [deg/s]
    and ``control.max_rot_accel_dps2`` [deg/s^2], converted to radians here —
    falling back to the rad-valued firmware defaults above when absent. Before
    this mapping (ticket 100-014) the generator emitted the hardcoded 6.0
    rad/s / 20.0 rad/s^2 defaults unconditionally, silently ignoring the
    robot JSON's own (much lower) pivot-speed intent and driving pivots at
    ~500 mm/s at the wheels -- unstable overshoot on the latent real plant.
    """
    ctrl = cfg.get("control", {}) or {}
    yr = _get(ctrl, "yaw_rate_max", default=None)        # [deg/s]
    ya = _get(ctrl, "max_rot_accel_dps2", default=None)  # [deg/s^2]
    yaw_rate_max = math.radians(float(yr)) if yr is not None else YAW_RATE_MAX_DEFAULT
    yaw_acc_max = math.radians(float(ya)) if ya is not None else YAW_ACC_MAX_DEFAULT
    return yaw_rate_max, yaw_acc_max


def drive_limits_for_config(cfg: dict):
    """Return the 17-tuple of msg::PlannerConfig fields 15-31 (Drive::Limits'
    wire/config source, 100-001), in field-number order.

    Mirrors heading_gains_for_config()'s exact shape: read from the robot
    JSON's ``control`` block when present, falling back to the conservative
    firmware defaults above when absent -- an unmigrated robot JSON (or
    togov, per architecture-update.md Open Question 6) simply inherits the
    firmware defaults, same fall-back discipline as every other mapping in
    this file.
    """
    ctrl = cfg.get("control", {}) or {}
    v_wheel_max       = _get(ctrl, "v_wheel_max",       default=V_WHEEL_MAX_DEFAULT)
    steer_headroom    = _get(ctrl, "steer_headroom",    default=STEER_HEADROOM_DEFAULT)
    wheel_step_max    = _get(ctrl, "wheel_step_max",    default=WHEEL_STEP_MAX_DEFAULT)
    track_k_s         = _get(ctrl, "track_k_s",         default=TRACK_K_S_DEFAULT)
    track_k_theta     = _get(ctrl, "track_k_theta",     default=TRACK_K_THETA_DEFAULT)
    track_k_cross     = _get(ctrl, "track_k_cross",     default=TRACK_K_CROSS_DEFAULT)
    trim_v_max        = _get(ctrl, "trim_v_max",        default=TRIM_V_MAX_DEFAULT)
    trim_omega_max    = _get(ctrl, "trim_omega_max",    default=TRIM_OMEGA_MAX_DEFAULT)
    replan_err_pos    = _get(ctrl, "replan_err_pos",    default=REPLAN_ERR_POS_DEFAULT)
    replan_err_theta  = _get(ctrl, "replan_err_theta",  default=REPLAN_ERR_THETA_DEFAULT)
    replan_hold       = _get(ctrl, "replan_hold",       default=REPLAN_HOLD_DEFAULT)
    replan_min_period = _get(ctrl, "replan_min_period", default=REPLAN_MIN_PERIOD_DEFAULT)
    replan_max        = _get(ctrl, "replan_max",        default=REPLAN_MAX_DEFAULT)
    handoff_tol_pos   = _get(ctrl, "handoff_tol_pos",   default=HANDOFF_TOL_POS_DEFAULT)
    handoff_tol_v     = _get(ctrl, "handoff_tol_v",     default=HANDOFF_TOL_V_DEFAULT)
    arrive_vel_tol    = _get(ctrl, "arrive_vel_tol",    default=ARRIVE_VEL_TOL_DEFAULT)
    arrive_dwell      = _get(ctrl, "arrive_dwell",      default=ARRIVE_DWELL_DEFAULT)
    return (
        float(v_wheel_max), float(steer_headroom), float(wheel_step_max),
        float(track_k_s), float(track_k_theta), float(track_k_cross),
        float(trim_v_max), float(trim_omega_max),
        float(replan_err_pos), float(replan_err_theta),
        float(replan_hold), float(replan_min_period), float(replan_max),
        float(handoff_tol_pos), float(handoff_tol_v),
        float(arrive_vel_tol), float(arrive_dwell),
    )


def generate(cfg: dict, source_path: str) -> str:
    trackwidth   = _get(cfg, "geometry", "trackwidth", default=TRACKWIDTH_DEFAULT)
    vel_kp, vel_ki, vel_kff, vel_imax, vel_kaw, vel_filt = vel_gains_for_config(cfg)
    travel_calib = travel_calib_for_ports(cfg)
    fwd_sign     = fwd_sign_for_ports(cfg)
    polled       = polled_for_ports()
    (otos_offset_x, otos_offset_y, otos_offset_yaw,
     otos_linear_scale, otos_angular_scale) = otos_boot_config_values(cfg)
    heading_kp, heading_kd = heading_gains_for_config(cfg)
    yaw_rate_max, yaw_acc_max = profile_rot_limits_for_config(cfg)
    min_speed = min_speed_for_config(cfg)
    (v_wheel_max, steer_headroom, wheel_step_max,
     track_k_s, track_k_theta, track_k_cross,
     trim_v_max, trim_omega_max,
     replan_err_pos, replan_err_theta,
     replan_hold, replan_min_period, replan_max,
     handoff_tol_pos, handoff_tol_v,
     arrive_vel_tol, arrive_dwell) = drive_limits_for_config(cfg)

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
    // starting defaults when absent. arrive_tol/turn_in_place_gate are left
    // unset (0.0f default) -- unchanged behavior, main.cpp's old function
    // never set them either (neither has a live consumer). min_speed is NO
    // LONGER left unset (100-007, THE CUTOVER) -- see MIN_SPEED_DEFAULT's
    // own comment above for why 0.0f silently broke pivot-mode detection
    // the moment source/drive/tracker.cpp became this field's first live
    // reader.
    //
    // Fields 15-31 (100-001 — Drive::Limits' wire/config source,
    // architecture-update.md M1/Decision 2): baked from the robot JSON's
    // control.* keys via drive_limits_for_config(), falling back to
    // conservative firmware starting defaults when absent — see this
    // generator's own field-default constants and data/robots/tovez.json's
    // `_drive_limits_note` for the full per-field derivation.
    msg::PlannerConfig cfg;
    cfg.setAMax({_f(A_MAX_DEFAULT)});               // [mm/s^2]
    cfg.setADecel({_f(A_DECEL_DEFAULT)});             // [mm/s^2]
    cfg.setVBodyMax({_f(V_BODY_MAX_DEFAULT)});           // [mm/s]
    cfg.setYawRateMax({_f(yaw_rate_max)});         // [rad/s] (control.yaw_rate_max [deg/s])
    cfg.setYawAccMax({_f(yaw_acc_max)});          // [rad/s^2] (control.max_rot_accel_dps2 [deg/s^2])
    cfg.setJMax({_f(J_MAX_DEFAULT)});                // [mm/s^3] ~6x a_max -- ~0.16s jerk-limited edges
    cfg.setYawJerkMax({_f(YAW_JERK_MAX_DEFAULT)});         // [rad/s^3] ~5x yaw_acc_max -- ~0.2s
    cfg.setHeadingKp({_f(heading_kp)});              // [1/s] outer heading-loop proportional gain
    cfg.setHeadingKd({_f(heading_kd)});              // dimensionless outer heading-loop derivative gain
    cfg.setMinSpeed({_f(min_speed)});               // [mm/s] Drive:: tracker pivot-mode threshold (100-007)
    cfg.setVWheelMax({_f(v_wheel_max)});              // [mm/s]
    cfg.setSteerHeadroom({_f(steer_headroom)});          // [mm/s]
    cfg.setWheelStepMax({_f(wheel_step_max)});          // [mm/s]
    cfg.setTrackKS({_f(track_k_s)});                // [1/s]
    cfg.setTrackKTheta({_f(track_k_theta)});            // [1/s]
    cfg.setTrackKCross({_f(track_k_cross)});            // [rad/mm^2]
    cfg.setTrimVMax({_f(trim_v_max)});               // [mm/s]
    cfg.setTrimOmegaMax({_f(trim_omega_max)});           // [rad/s]
    cfg.setReplanErrPos({_f(replan_err_pos)});           // [mm]
    cfg.setReplanErrTheta({_f(replan_err_theta)});         // [rad]
    cfg.setReplanHold({_f(replan_hold)});              // [s]
    cfg.setReplanMinPeriod({_f(replan_min_period)});        // [s]
    cfg.setReplanMax({_f(replan_max)});               // dimensionless
    cfg.setHandoffTolPos({_f(handoff_tol_pos)});          // [mm]
    cfg.setHandoffTolV({_f(handoff_tol_v)});            // [s]
    cfg.setArriveVelTol({_f(arrive_vel_tol)});           // [mm/s]
    cfg.setArriveDwell({_f(arrive_dwell)});             // [s]
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
