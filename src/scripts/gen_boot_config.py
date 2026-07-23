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
  * `output_deadband_for_config()` / `reversal_dwell_for_config()` —
    control.output_deadband [-1,1] / control.reversal_dwell_ms [ms] (sprint
    114 ticket 003) — Devices::NezhaMotor::writeShapedDuty()'s output-
    deadband floor and reversal-dwell hold; previously left unset (.has ==
    false) on purpose, ship-defaulted (0.03 / 100.0) inside NezhaMotor's own
    constructor.
  * `trackwidth_for_config()` — geometry.trackwidth -> DrivetrainConfig.trackwidth.
  * `otos_boot_config_values()` — geometry.odometry_offset_mm.{x,y,yaw_rad}
    and calibration.otos_linear_scale/otos_angular_scale (086-005) ->
    OtosBootConfig; boot-time-baked only, never a live SET/wire surface (see
    OtosBootConfig's own doc comment, src/firm/config/boot_config.h).

115-003 (gut-to-minimal-firmware S1, motion-stack excision) removed
`defaultPlannerConfig()` and its planner-field helper functions
(`heading_gains_for_config()`, `heading_source_for_config()`,
`heading_dwell_for_config()`, `lead_compensation_for_config()`,
`profile_rot_limits_for_config()`, `min_speed_for_config()`,
`arrive_dwell_for_config()`, `actuation_lag_for_config()`,
`distance_gains_for_config()`, `model_tau_for_config()`,
`motion_limits_for_config()`) wholesale, alongside `msg::PlannerConfig`
itself (planner.proto, deleted in the same ticket) -- nothing in the S1
minimal firmware boots a planner config. The robot JSON's `control.*` keys
those functions read (heading_kp/heading_kd/heading_source/
heading_dwell_tol_deg/heading_dwell_rate_dps/heading_lead_bias/plan_lead/
terminal_lead/yaw_rate_max/max_rot_accel_dps2/min_speed/arrive_dwell/
actuation_lag/distance_kp/distance_tol/model_tau_lin/model_tau_ang/
v_body_max) are STILL unread by this generator as of that ticket; existing
robot JSON files may still carry them harmlessly (dead data, not a build
error).

`a_max`/`a_decel`/`j_max`/`yaw_jerk_max` READ AGAIN (decel-into-the-goal
campaign) -- the four exceptions to the paragraph above. Orphaned by
115-003 alongside every other `motion_limits_for_config()` field, they are
the four of that list this campaign's `shaper_config_for_config()` (below)
reads back into a NEW consumer (`Config::ShaperBootConfig` ->
`Motion::VelocityShaper` / `App::MoveQueue`, not the deleted planner) --
see that function's own docstring. `alpha_max`/`alpha_decel` are genuinely
new fields this campaign added to the schema/every robot JSON (a_max/
a_decel's own angular sibling; no `msg::PlannerConfig` predecessor existed
for either) -- `yaw_jerk_max` already existed as `j_max`'s own angular
sibling, so no new angular jerk field was needed.

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
    ``vel_gains_for_config({})``) gets a self-contained message with no
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


def _u32(v) -> str:
    """Format a Python number as a C++ uint32_t literal (rounded to the
    nearest integer -- staleness_ms's own JSON value is a plain number,
    e.g. 60.0, that EstimatorBootConfig::staleness stores as uint32_t)."""
    return f"{int(round(float(v)))}u"


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


def output_deadband_for_config(cfg: dict):
    """Return control.output_deadband (duty fraction [-1,1]) -- Devices::
    NezhaMotor::writeShapedDuty()'s output-deadband floor (folded from the
    old MotorArmor base) and MotorArmor's own wedge-suspect motion-gate
    threshold. REQUIRED as of sprint 114 ticket 003 (config-as-truth
    completion) -- previously left unset (.has == false) on purpose, with
    NezhaMotor's own kDefaultOutputDeadband (0.03) substituted in the
    constructor whenever a config arrived unset; that substitution is gone,
    so every robot JSON must now carry a real value."""
    return float(_require(cfg, "control", "output_deadband"))


def reversal_dwell_for_config(cfg: dict):
    """Return control.reversal_dwell_ms [ms] -- Devices::NezhaMotor::
    writeShapedDuty()'s reversal-dwell hold time (folded from the old
    MotorArmor base). REQUIRED as of sprint 114 ticket 003 (config-as-truth
    completion) -- previously left unset (.has == false) on purpose, with
    NezhaMotor's own kDefaultReversalDwell (100.0) substituted in the
    constructor whenever a config arrived unset; that substitution is gone,
    so every robot JSON must now carry a real value."""
    return float(_require(cfg, "control", "reversal_dwell_ms"))


def trackwidth_for_config(cfg: dict) -> float:
    """Return geometry.trackwidth [mm] -> DrivetrainConfig.trackwidth.
    REQUIRED as of sprint 114 (config-as-truth completion) -- previously
    fell back to a 128.0mm placeholder when absent."""
    return float(_require(cfg, "geometry", "trackwidth"))


def estimator_config_for_config(cfg: dict):
    """Return (heading_otos, omega_otos, staleness, stop_lead) for the
    EstimatorBootConfig struct (117, predict-to-now estimator v1;
    stop_lead added by the turn-prediction campaign) --
    App::StateEstimator's fail-closed boot-time fusion-weight defaults plus
    App::MoveQueue's own fail-closed boot-time anticipation lead.

    REQUIRED as of ticket 003 (heading_otos/omega_otos/staleness) and the
    turn-prediction campaign (stop_lead) -- the SAME fail-closed discipline
    sprint 114 established for output_deadband_for_config()/
    reversal_dwell_for_config() above: a robot JSON missing any of the four
    ``estimator.*`` keys fails codegen loudly rather than silently
    defaulting to encoder-only/no-anticipation. Per the stakeholder's
    encoder-only-v1 decision, weight_heading_otos/weight_omega_otos are
    committed 0.0 in every robot JSON this sprint; staleness_ms/
    stop_lead_ms each carry a reasoned per-robot placeholder (see each
    robot JSON's own inline comment).
    """
    heading_otos = _require(cfg, "estimator", "weight_heading_otos")
    omega_otos = _require(cfg, "estimator", "weight_omega_otos")
    staleness = _require(cfg, "estimator", "staleness_ms")
    stop_lead = _require(cfg, "estimator", "stop_lead_ms")
    return float(heading_otos), float(omega_otos), float(staleness), float(stop_lead)


def shaper_config_for_config(cfg: dict):
    """Return (a_max, a_decel, alpha_max, alpha_decel, j_max, yaw_jerk_max)
    for Config::ShaperBootConfig (decel-into-the-goal campaign) --
    Motion::VelocityShaper's own accel/decel/jerk magnitude ceilings,
    consumed by App::MoveQueue to taper the commanded speed toward each
    Move's own stop threshold instead of holding a constant speed until
    Motion::StopCondition fires.

    a_max/a_decel/j_max/yaw_jerk_max are READ AGAIN here -- this module's
    own docstring explains why all four were dead ("unread") data since
    115-003's motion-stack excision and why this campaign resurrects them
    into a DIFFERENT consumer than the deleted planner. alpha_max/
    alpha_decel are new fields (a_max/a_decel's own angular sibling) --
    yaw_jerk_max already existed as j_max's own angular sibling, so no new
    field was needed there.

    All six REQUIRED, same fail-closed posture as every other field this
    generator bakes (sprint 114 config-as-truth, extended here) -- a robot
    JSON missing any one of them fails codegen loudly rather than shipping
    a boot image where App::ShaperLimits silently disables shaping on that
    axis (see App::ShaperLimits's own "0 == disabled" doc comment,
    app/move_queue.h) -- a build should refuse an incomplete shaping
    calibration the same way it already refuses an incomplete velocity-PID
    or OTOS calibration, not silently ship an unshaped robot.
    """
    a_max = _require(cfg, "control", "a_max")
    a_decel = _require(cfg, "control", "a_decel")
    alpha_max = _require(cfg, "control", "alpha_max")
    alpha_decel = _require(cfg, "control", "alpha_decel")
    j_max = _require(cfg, "control", "j_max")
    yaw_jerk_max = _require(cfg, "control", "yaw_jerk_max")
    return (float(a_max), float(a_decel), float(alpha_max), float(alpha_decel),
            float(j_max), float(yaw_jerk_max))


def generate(cfg: dict, source_path: str) -> str:
    try:
        trackwidth   = trackwidth_for_config(cfg)
        vel_kp, vel_ki, vel_kff, vel_imax, vel_kaw, vel_filt = vel_gains_for_config(cfg)
        output_deadband = output_deadband_for_config(cfg)
        reversal_dwell = reversal_dwell_for_config(cfg)
        travel_calib = travel_calib_for_ports(cfg)
        fwd_sign     = fwd_sign_for_ports(cfg)
        polled       = polled_for_ports()
        (otos_offset_x, otos_offset_y, otos_offset_yaw,
         otos_linear_scale, otos_angular_scale) = otos_boot_config_values(cfg)
        (estimator_heading_otos, estimator_omega_otos,
         estimator_staleness, estimator_stop_lead) = estimator_config_for_config(cfg)
        (shaper_a_max, shaper_a_decel, shaper_alpha_max, shaper_alpha_decel,
         shaper_j_max, shaper_yaw_jerk_max) = shaper_config_for_config(cfg)
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

    for (uint32_t i = 0; i < kMotorConfigCount; ++i) {{
        out[i] = msg::MotorConfig();
        out[i].setPort(i + 1);
        out[i].setVelGains(velGains);
        // EMA coeff — from control.vel_filt (fallback default); a=0 would pin
        // reported velocity at 0 forever regardless of real motion.
        out[i].setVelFiltAlpha({_f(vel_filt)});
        // Write-shaping floor/hold — baked from the robot JSON's
        // control.output_deadband/control.reversal_dwell_ms (sprint 114
        // ticket 003, config-as-truth completion). REQUIRED as of this
        // ticket: Devices::NezhaMotor no longer substitutes a ship default
        // when these arrive unset, so every build must emit a real value.
        out[i].setOutputDeadband({_f(output_deadband)});   // [-1,1] fraction
        out[i].setReversalDwell({_f(reversal_dwell)});   // [ms]
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

EstimatorBootConfig defaultEstimatorConfig() {{
    // 117 (predict-to-now estimator v1) — fail-closed baked from the robot
    // JSON's estimator.weight_heading_otos/weight_omega_otos/staleness_ms
    // (data/robots/robot_config.schema.json). Encoder-only v1 (stakeholder
    // decision): both blend weights are committed 0.0 in every robot JSON
    // this sprint -- see that JSON's own inline comment for the
    // staleness_ms reasoning. NOT a live SET/wire surface itself -- see
    // EstimatorBootConfig's own doc comment (src/firm/config/boot_config.h)
    // for the separate, volatile EstimatorConfigPatch live-tuning path.
    // stop_lead_ms (turn-prediction campaign) -- App::MoveQueue's own
    // fail-closed boot-time anticipation lead; see that JSON's own inline
    // comment for the derivation.
    EstimatorBootConfig cfg;
    cfg.headingOtos = {_f(estimator_heading_otos)};
    cfg.omegaOtos = {_f(estimator_omega_otos)};
    cfg.staleness = {_u32(estimator_staleness)};   // [ms]
    cfg.stopLead = {_u32(estimator_stop_lead)};    // [ms]
    return cfg;
}}

ShaperBootConfig defaultShaperConfig() {{
    // Decel-into-the-goal campaign -- fail-closed baked from the robot
    // JSON's control.a_max/a_decel/alpha_max/alpha_decel/j_max/
    // yaw_jerk_max (data/robots/robot_config.schema.json). a_max/a_decel/
    // j_max/yaw_jerk_max are the deleted msg::PlannerConfig's own former
    // fields, orphaned by 115-003 and read again here into a NEW consumer
    // (Motion::VelocityShaper); alpha_max/alpha_decel are new (a_max/
    // a_decel's own angular sibling -- yaw_jerk_max already covered the
    // angular jerk slot). NOT a live SET/wire surface itself -- see
    // App::MoveQueue's own setShaperLimits()/EstimatorConfigPatch's
    // a_max/a_decel/alpha_max/alpha_decel/j_max/yaw_jerk_max fields
    // (config.proto) for the separate, volatile live-tuning path (mirrors
    // OtosBootConfig/EstimatorBootConfig's own "boot bake vs. live
    // ConfigPatch" split).
    ShaperBootConfig cfg;
    cfg.aMax = {_f(shaper_a_max)};                  // [mm/s^2]
    cfg.aDecel = {_f(shaper_a_decel)};               // [mm/s^2]
    cfg.alphaMax = {_f(shaper_alpha_max)};           // [rad/s^2]
    cfg.alphaDecel = {_f(shaper_alpha_decel)};       // [rad/s^2]
    cfg.jMax = {_f(shaper_j_max)};                   // [mm/s^3]
    cfg.yawJerkMax = {_f(shaper_yaw_jerk_max)};      // [rad/s^3]
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
