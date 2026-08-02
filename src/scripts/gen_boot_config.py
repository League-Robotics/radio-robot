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
reads back into `Config::ShaperBootConfig`. That consumer (a
velocity-shaping stage this campaign added) was itself deleted as dead
code in sprint 128 ticket 014 -- `Config::ShaperBootConfig` is currently
unread by anything (`Motion::Planner` uses its own hand-baked
`Motion::PlannerLimits`, not this struct); kept declared since removing
it is a schema change outside that ticket's scope. See that function's
own docstring. `alpha_max`/`alpha_decel` are genuinely
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

`planner_config_for_config()` (129-009, config consolidation) — the
`Motion::PlannerLimits` tuning surface `main.cpp` used to assemble as C++
literals (profile ceilings, loop timing, settle/rest), baked from the
robot JSON's own `planner` block into `Config::PlannerBootConfig`/
`Config::defaultPlannerLimits()`. All 18 raw keys are REQUIRED
(fail-closed, same posture as every other mapping this module documents
above). Distinct from, and NOT a replacement for,
`shaper_config_for_config()`/`Config::ShaperBootConfig` above: that struct
is dead (unread by anything, superseded by `Motion::Planner`'s own former
hand-baked limits) and reads the UNRELATED legacy `control.a_max`/
`a_decel`/... keys; `planner_config_for_config()` reads the NEW
`planner.*` keys instead and IS live (main.cpp constructs its
`Motion::PlannerLimits` from this function's output).

130-009 (planner-honesty-pass-...limits-reduction.md item 3) cut 11 of
the original 29 raw keys — the M4 duty-stage gains (`vel_kp`/`vel_ki`/
`vel_i_max`/`vel_i_accel_gate`/`duty_floor`, plus the derived `velKff`/
`velKaff`) and the settle-confirm pair (`require_settle`/`settle_window`)
— and 5 more that fed PlannerLimits' now-dead planner-side trim gains
(`trim_kp`/`trim_ki`/`trim_i_max`/`trim_max`, plus the derived
`trimKaff`) — see `planner_config_for_config()`'s own docstring for the
per-field rationale. `plant_gain`/`plant_tau` are no longer read by this
function at all (both of their derived consumers are cut); either key may
still be present in a robot JSON as recorded measured data.
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


def rotational_slip_for_config(cfg: dict) -> float:
    """Return calibration.rotational_slip -> DrivetrainConfig.rotational_slip.

    The scrub factor: a differential robot skids its wheels sideways through
    a turn, so it rotates LESS than ideal kinematics (omega = (vR-vL)/b)
    predicts for a given wheel differential. This is the ratio of actual to
    ideal rotation, so the EFFECTIVE track is trackwidth / rotational_slip
    (main.cpp does that division; trackwidth itself stays the physically
    measured wheel separation and must not be bent to absorb scrub).

    Domain is `{0} u [0.5, 1.0]` (config.proto): 0 is the "uncalibrated"
    sentinel and means apply no correction, NOT divide-by-zero.
    """
    return float(_require(cfg, "calibration", "rotational_slip"))


def rotation_calibration_for_config(cfg: dict):
    """Return (gain_pos, offset_pos_deg, gain_neg, offset_neg_deg).

    The measured affine turn response, `actual = gain*commanded + offset`,
    per direction of rotation. RobotLoop inverts it so an ANGLE-stopped move
    lands on the requested angle. `_pos` is positive commanded omega.

    Offsets stay in DEGREES here (the unit the robot JSON and a human use);
    main.cpp converts to radians at the seam.
    """
    cal = cfg["calibration"]
    return (float(_require(cfg, "calibration", "rotation_gain")),
            float(_require(cfg, "calibration", "rotation_offset_deg")),
            float(_require(cfg, "calibration", "rotation_gain_neg")),
            float(_require(cfg, "calibration", "rotation_offset_deg_neg")))


def estimator_config_for_config(cfg: dict):
    """Return (heading_otos, omega_otos, staleness) for the
    EstimatorBootConfig struct (117, predict-to-now estimator v1) --
    App::StateEstimator's fail-closed boot-time fusion-weight defaults.

    REQUIRED as of ticket 003 -- the SAME fail-closed discipline sprint 114
    established for output_deadband_for_config()/reversal_dwell_for_config()
    above: a robot JSON missing any of the three ``estimator.*`` keys fails
    codegen loudly rather than silently defaulting to encoder-only. Per the
    stakeholder's encoder-only-v1 decision, weight_heading_otos/
    weight_omega_otos are committed 0.0 in every robot JSON this sprint;
    staleness_ms carries a reasoned per-robot placeholder (see each robot
    JSON's own inline comment).

    The turn-prediction campaign's own former fourth field (a boot-time
    anticipation-lead constant) is DELETED (118 ticket 004,
    land-at-zero-completion-delete-stop-lead.md) -- the anticipation
    mechanism it fed no longer exists (see
    docs/design/history/land-at-zero-margin-derivation.md for the
    land-at-zero completion predicate that replaced it, itself deleted as
    dead code in sprint 128 ticket 014), so this generator no longer reads
    (or requires) that key.
    """
    heading_otos = _require(cfg, "estimator", "weight_heading_otos")
    omega_otos = _require(cfg, "estimator", "weight_omega_otos")
    staleness = _require(cfg, "estimator", "staleness_ms")
    return float(heading_otos), float(omega_otos), float(staleness)


def shaper_config_for_config(cfg: dict):
    """Return (a_max, a_decel, alpha_max, alpha_decel, j_max, yaw_jerk_max)
    for Config::ShaperBootConfig (decel-into-the-goal campaign) --
    accel/decel/jerk magnitude ceilings originally consumed by a
    velocity-shaping stage this campaign added. That consumer was deleted
    as dead code in sprint 128 ticket 014 (zero callers, superseded by
    Motion::Planner's own hand-baked PlannerLimits) -- Config::
    ShaperBootConfig is currently unread by anything; kept declared since
    removing it is a boot-config schema change outside that ticket's
    scope.

    a_max/a_decel/j_max/yaw_jerk_max are READ AGAIN here -- this module's
    own docstring explains why all four were dead ("unread") data since
    115-003's motion-stack excision and why this campaign resurrected them
    into a (now also deleted) consumer. alpha_max/alpha_decel are new
    fields (a_max/a_decel's own angular sibling) -- yaw_jerk_max already
    existed as j_max's own angular sibling, so no new field was needed
    there.

    All six REQUIRED, same fail-closed posture as every other field this
    generator bakes (sprint 114 config-as-truth, extended here) -- a robot
    JSON missing any one of them fails codegen loudly rather than shipping
    an incomplete calibration silently, the same way it already refuses an
    incomplete velocity-PID or OTOS calibration.
    """
    a_max = _require(cfg, "control", "a_max")
    a_decel = _require(cfg, "control", "a_decel")
    alpha_max = _require(cfg, "control", "alpha_max")
    alpha_decel = _require(cfg, "control", "alpha_decel")
    j_max = _require(cfg, "control", "j_max")
    yaw_jerk_max = _require(cfg, "control", "yaw_jerk_max")
    return (float(a_max), float(a_decel), float(alpha_max), float(alpha_decel),
            float(j_max), float(yaw_jerk_max))


def wheel_correction_for_config(cfg: dict):
    """Return the 8 commanded->actual correction values in the order
    (gain, intercept) x (left, right) x (accel, decel) for
    Config::DriveBootConfig.

    measured = gain*commanded + intercept, per wheel per direction of
    approach, from docs/design/wheel-speed-command-mapping.md. App::Drive
    inverts it to seed the feedforward: command = (desired-intercept)/gain.

    The correction is defined RELATIVE to the duty_per_speed constant it was
    measured against, so both must come from the same characterization run.
    gain 1 / intercept 0 is the identity (an uncalibrated robot); a gain of
    0 or less is meaningless and aborts.

    All REQUIRED -- same fail-closed posture as every other baked field."""
    out = []
    for wheel in ("left", "right"):
        for direction in ("accel", "decel"):
            gain = float(_require(cfg, "control", f"wheel_gain_{wheel}_{direction}"))
            icpt = float(_require(cfg, "control", f"wheel_intercept_{wheel}_{direction}"))
            if gain <= 0.0:
                raise SystemExit(
                    f"control.wheel_gain_{wheel}_{direction} must be > 0 (got {gain})")
            out.append((gain, icpt))
    return out


def drive_config_for_config(cfg: dict):
    """Return (duty_per_speed_left, duty_per_speed_right, crawl_pulse) for
    Config::DriveBootConfig (command-ingestion-ring-buffered-comms-
    subsystem-routing-two-stops.md §6) -- App::Drive's open-loop wheel
    calibration and its crawl-shaper amplitude.

    These were HARD-CODED in C++ before this change: the duty-per-speed pair
    as member initializers on App::Drive itself, the crawl amplitude as a
    bare setCrawlPulse() call in main.cpp. That made one robot's gearboxes,
    on one battery, measured on one evening, the compiled-in default every
    other robot silently inherited -- and changing it meant editing a class
    definition and reflashing. The `kff` wire key was not an escape hatch
    either: it sets BOTH wheels to one value, so a single config push
    flattens the measured ~10% L/R asymmetry with no way to restore it short
    of a rebuild.

    All three REQUIRED, same fail-closed posture as every other field this
    generator bakes: a robot JSON missing any of them fails codegen loudly
    rather than shipping a boot image whose wheel calibration came from a
    different robot. App::Drive itself now carries NO calibration defaults
    at all -- an unconfigured Drive refuses to drive (drive.h), the same
    posture RobotLoop's `configured_` gate already takes for motion
    commands.
    """
    duty_left = _require(cfg, "control", "duty_per_speed_left")
    duty_right = _require(cfg, "control", "duty_per_speed_right")
    crawl = _require(cfg, "control", "crawl_pulse")
    return (float(duty_left), float(duty_right), float(crawl))


def wheel_controller_config_for_config(cfg: dict) -> dict:
    """Return a dict of every Config::WheelControllerBootConfig field
    (boot_config.h), keyed by its C++ field name, read from the robot
    JSON's `control.wheel_*`/`control.wheel_pid_*`/`control.wheel_deficit_*`
    keys (130-004, wheel-speed-controller-moves-into-drive.md Phase 2).

    App::Drive's unified three-timescale wheel-speed controller: Stage
    B's wire-tunable fast-PID gains (wheel_pid_kp/ki/i_max/kaff/max) and
    Stage C/deficit-flag's generated-constant bounds (wheel_v_min/
    wheel_bias_max/wheel_tau_adapt/wheel_a_steady/wheel_deficit_
    threshold/wheel_deficit_window_ms).

    All 11 REQUIRED, same fail-closed posture as every other field this
    generator bakes: a robot JSON missing any one of them fails codegen
    loudly. A robot JSON is free to set every one of these to 0 (both
    stages inert) -- see WheelControllerBootConfig's own doc comment
    (boot_config.h) for why 0 is a safe, meaningful disabled state here,
    not a placeholder standing in for a value the code requires nonzero.
    """
    return {
        "vMin": float(_require(cfg, "control", "wheel_v_min")),
        "biasMax": float(_require(cfg, "control", "wheel_bias_max")),
        "tauAdapt": float(_require(cfg, "control", "wheel_tau_adapt")),
        "aSteady": float(_require(cfg, "control", "wheel_a_steady")),
        "deficitThreshold": float(_require(cfg, "control", "wheel_deficit_threshold")),
        "deficitWindow": float(_require(cfg, "control", "wheel_deficit_window_ms")),
        "kp": float(_require(cfg, "control", "wheel_pid_kp")),
        "ki": float(_require(cfg, "control", "wheel_pid_ki")),
        "iMax": float(_require(cfg, "control", "wheel_pid_i_max")),
        "kaff": float(_require(cfg, "control", "wheel_pid_kaff")),
        "pidMax": float(_require(cfg, "control", "wheel_pid_max")),
    }


def planner_config_for_config(cfg: dict) -> dict:
    """Return a dict of every Config::PlannerBootConfig field (boot_config.h),
    keyed by its C++ field name, read from the robot JSON's `planner` block
    (129-009, config consolidation; field set reduced 130-009).

    Before 129-009 every one of these values was a C++ literal assembled
    directly in main.cpp's Motion::PlannerLimits construction -- this is
    the move to config-as-truth (sprint 114) that block itself called out
    as still owed ("A planner-domain config surface can supersede these
    constants later").

    All 18 raw keys are REQUIRED, same fail-closed posture as every other
    field this generator bakes: a robot JSON missing the `planner` block
    (or any key inside it) fails codegen loudly rather than a robot
    inheriting another robot's plant measurements.

    130-009 (planner-honesty-pass-...limits-reduction.md item 3): cut the
    11 raw keys that fed PlannerLimits' now-deleted M4 duty-stage/settle-
    confirm fields (vel_kp/vel_ki/vel_i_max/vel_i_accel_gate/duty_floor/
    require_settle/settle_window) and the 5 that fed the planner-side
    trim gains WheelTrim left dead (trim_kp/trim_ki/trim_i_max/trim_max,
    plus the derived trim_kaff) -- see PlannerBootConfig's own doc comment
    (boot_config.h) for why each is gone. `plant_gain` is no longer read
    here (its only consumers, velKff/velKaff, are both cut); `plant_tau`
    is ALSO no longer read (its only consumer, trimKaff, is cut too) --
    both keys may still be present in a robot JSON as recorded measured
    data, simply unread by this function now.
    """
    return {
        "vMax": float(_require(cfg, "planner", "v_max")),
        "aMax": float(_require(cfg, "planner", "a_max")),
        "aDecel": float(_require(cfg, "planner", "a_decel")),
        "omegaMax": float(_require(cfg, "planner", "omega_max")),
        "alphaMax": float(_require(cfg, "planner", "alpha_max")),
        "alphaDecel": float(_require(cfg, "planner", "alpha_decel")),
        "jerkMax": float(_require(cfg, "planner", "jerk_max")),
        "yawJerkMax": float(_require(cfg, "planner", "yaw_jerk_max")),

        "controlPeriod": float(_require(cfg, "planner", "control_period")),
        "actuationDelay": float(_require(cfg, "planner", "actuation_delay")),

        "settleRestVelocity": float(_require(cfg, "planner", "settle_rest_velocity")),
        "settleRestOmega": float(_require(cfg, "planner", "settle_rest_omega")),
        "settleEpsilonLinear": float(_require(cfg, "planner", "settle_epsilon_linear")),
        "settleEpsilonAngular": float(_require(cfg, "planner", "settle_epsilon_angular")),
        "headingHoldGain": float(_require(cfg, "planner", "heading_hold_gain")),

        "decelPlanFraction": float(_require(cfg, "planner", "decel_plan_fraction")),
    }


def profile_name_for_source(source_path: str) -> str:
    """The calibration-profile identifier `ID:` reports (sprint 124
    architecture Decision 4): the active robot JSON's own filename stem
    (e.g. "tovez_nocal" for "data/robots/tovez_nocal.json"), or
    "unconfigured" for the "(firmware defaults)" no-JSON-found sentinel
    `load_robot_config()` returns. See boot_config.h's own
    `kRobotProfileName` doc comment."""
    if source_path == "(firmware defaults)":
        return "unconfigured"
    return Path(source_path).stem


def drivetrain_type_for_config(cfg: dict) -> str:
    """The drivetrain-type identifier `ID:` reports (sprint 124
    architecture Decision 4): ``identity.drivetrain_type``
    (`data/robots/robot_config.schema.json`'s own enum, `["differential",
    "mecanum"]`), defaulting to `"differential"` per that schema's own
    documented default when the key is absent (e.g. `tovez_nocal.json`,
    which never sets it) -- mirrors the schema's own default exactly, not
    an independently-chosen one. This is the schema's own compile-time
    drivetrain variant, NOT derived from any wire-level `DrivetrainConfig`
    field (`half_track` in particular is never baked by
    `defaultDrivetrainConfig()` -- it stays at its wire default-member-
    initializer 0.0f for every profile, so it cannot distinguish
    drivetrain kind; an earlier draft of this ticket's own `main.cpp`
    change read `half_track` for this purpose, always got `differential`
    regardless of the profile, and is corrected here to read the JSON
    field the schema itself designates as authoritative for this
    question). See boot_config.h's own `kDrivetrainType` doc comment."""
    return str(_get(cfg, "identity", "drivetrain_type", default="differential"))


def generate(cfg: dict, source_path: str) -> str:
    try:
        trackwidth   = trackwidth_for_config(cfg)
        rot_slip     = rotational_slip_for_config(cfg)
        rot_cal      = rotation_calibration_for_config(cfg)
        vel_kp, vel_ki, vel_kff, vel_imax, vel_kaw, vel_filt = vel_gains_for_config(cfg)
        output_deadband = output_deadband_for_config(cfg)
        reversal_dwell = reversal_dwell_for_config(cfg)
        travel_calib = travel_calib_for_ports(cfg)
        fwd_sign     = fwd_sign_for_ports(cfg)
        polled       = polled_for_ports()
        (otos_offset_x, otos_offset_y, otos_offset_yaw,
         otos_linear_scale, otos_angular_scale) = otos_boot_config_values(cfg)
        (estimator_heading_otos, estimator_omega_otos,
         estimator_staleness) = estimator_config_for_config(cfg)
        (shaper_a_max, shaper_a_decel, shaper_alpha_max, shaper_alpha_decel,
         shaper_j_max, shaper_yaw_jerk_max) = shaper_config_for_config(cfg)
        (drive_duty_per_speed_left, drive_duty_per_speed_right,
         drive_crawl_pulse) = drive_config_for_config(cfg)
        wheel_corr = wheel_correction_for_config(cfg)
        wheel_controller = wheel_controller_config_for_config(cfg)
        planner = planner_config_for_config(cfg)
    except MissingRobotConfigKeyError as e:
        raise e.with_source(source_path) from e

    profile_name = profile_name_for_source(source_path)
    drivetrain_type = drivetrain_type_for_config(cfg)

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

// kRobotProfileName — see boot_config.h's own doc comment (sprint 124
// architecture Decision 4, `ID:`'s calibration-profile field). Baked from
// this generator's own source robot JSON path, above.
const char kRobotProfileName[] = "{profile_name}";

// kDrivetrainType — see boot_config.h's own doc comment (sprint 124
// architecture Decision 4, `ID:`'s drivetrain-type field). Baked from
// identity.drivetrain_type (robot JSON), defaulting to "differential"
// per the schema's own documented default.
const char kDrivetrainType[] = "{drivetrain_type}";

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
    cfg.setRotationalSlip({_f(rot_slip)});   // scrub: actual/ideal rotation, 0 = uncalibrated
    // Turn calibration: actual = gain*commanded + offset[deg], per direction.
    cfg.setRotationGainPos({_f(rot_cal[0])});
    cfg.setRotationOffset({_f(rot_cal[1])});
    cfg.setRotationGainNeg({_f(rot_cal[2])});
    cfg.setRotationOffsetNeg({_f(rot_cal[3])});
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
    EstimatorBootConfig cfg;
    cfg.headingOtos = {_f(estimator_heading_otos)};
    cfg.omegaOtos = {_f(estimator_omega_otos)};
    cfg.staleness = {_u32(estimator_staleness)};   // [ms]
    return cfg;
}}

ShaperBootConfig defaultShaperConfig() {{
    // Decel-into-the-goal campaign -- fail-closed baked from the robot
    // JSON's control.a_max/a_decel/alpha_max/alpha_decel/j_max/
    // yaw_jerk_max (data/robots/robot_config.schema.json). a_max/a_decel/
    // j_max/yaw_jerk_max are the deleted msg::PlannerConfig's own former
    // fields, orphaned by 115-003 and read again here into a velocity-
    // shaping consumer that has SINCE ALSO been deleted, as dead code, in
    // sprint 128 ticket 014 (zero callers, superseded by Motion::Planner's
    // own hand-baked PlannerLimits) -- this whole struct is currently
    // unread by anything; alpha_max/alpha_decel are new (a_max/a_decel's
    // own angular sibling -- yaw_jerk_max already covered the angular
    // jerk slot). NOT a live SET/wire surface itself -- see
    // EstimatorConfigPatch's a_max/a_decel/alpha_max/alpha_decel/j_max/
    // yaw_jerk_max fields (config.proto) for the separate, volatile
    // live-tuning path (mirrors OtosBootConfig/EstimatorBootConfig's own
    // "boot bake vs. live ConfigPatch" split).
    ShaperBootConfig cfg;
    cfg.aMax = {_f(shaper_a_max)};                  // [mm/s^2]
    cfg.aDecel = {_f(shaper_a_decel)};               // [mm/s^2]
    cfg.alphaMax = {_f(shaper_alpha_max)};           // [rad/s^2]
    cfg.alphaDecel = {_f(shaper_alpha_decel)};       // [rad/s^2]
    cfg.jMax = {_f(shaper_j_max)};                   // [mm/s^3]
    cfg.yawJerkMax = {_f(shaper_yaw_jerk_max)};      // [rad/s^3]
    return cfg;
}}

DriveBootConfig defaultDriveConfig() {{
    // command-ingestion-ring-buffered-comms-subsystem-routing-two-stops.md
    // §6 -- fail-closed baked from the robot JSON's
    // control.duty_per_speed_left/duty_per_speed_right/crawl_pulse
    // (data/robots/robot_config.schema.json). These were hard-coded in C++
    // before that change (the duty pair as App::Drive member initializers,
    // the crawl amplitude as a bare main.cpp setCrawlPulse() call); see
    // gen_boot_config.py's drive_config_for_config() for why that was
    // wrong. NOT a live SET/wire surface itself -- the `kff` CONFIG key
    // still retargets the duty scale at runtime, but it sets BOTH wheels
    // to one value, which is exactly why the per-wheel split has to be
    // baked here rather than left to it.
    DriveBootConfig cfg;
    cfg.dutyPerSpeedLeft = {_f(drive_duty_per_speed_left)};    // [duty/(mm/s)]
    cfg.dutyPerSpeedRight = {_f(drive_duty_per_speed_right)};  // [duty/(mm/s)]
    cfg.crawlPulse = {_f(drive_crawl_pulse)};                  // [-1,1]; 0 = off
    cfg.gainLeftAccel = {_f(wheel_corr[0][0])};
    cfg.interceptLeftAccel = {_f(wheel_corr[0][1])};   // [mm/s]
    cfg.gainLeftDecel = {_f(wheel_corr[1][0])};
    cfg.interceptLeftDecel = {_f(wheel_corr[1][1])};   // [mm/s]
    cfg.gainRightAccel = {_f(wheel_corr[2][0])};
    cfg.interceptRightAccel = {_f(wheel_corr[2][1])};   // [mm/s]
    cfg.gainRightDecel = {_f(wheel_corr[3][0])};
    cfg.interceptRightDecel = {_f(wheel_corr[3][1])};   // [mm/s]
    return cfg;
}}

WheelControllerBootConfig defaultWheelControllerConfig() {{
    // 130-004 (wheel-speed-controller-moves-into-drive.md Phase 2) --
    // fail-closed baked from the robot JSON's control.wheel_*/
    // control.wheel_pid_*/control.wheel_deficit_* keys
    // (data/robots/robot_config.schema.json). See
    // WheelControllerBootConfig's own doc comment (config/boot_config.h)
    // for the full field-for-field mapping and why 0 is a safe,
    // meaningful "this stage is disabled" value here.
    WheelControllerBootConfig cfg;
    cfg.vMin = {_f(wheel_controller["vMin"])};                        // [mm/s]
    cfg.biasMax = {_f(wheel_controller["biasMax"])};                  // [mm/s]
    cfg.tauAdapt = {_f(wheel_controller["tauAdapt"])};                // [s]
    cfg.aSteady = {_f(wheel_controller["aSteady"])};                  // [mm/s^2]
    cfg.deficitThreshold = {_f(wheel_controller["deficitThreshold"])};  // [mm/s]
    cfg.deficitWindow = {_f(wheel_controller["deficitWindow"])};      // [ms]
    cfg.kp = {_f(wheel_controller["kp"])};        // [1]
    cfg.ki = {_f(wheel_controller["ki"])};        // [1/s]
    cfg.iMax = {_f(wheel_controller["iMax"])};    // [mm/s]
    cfg.kaff = {_f(wheel_controller["kaff"])};    // [s]
    cfg.pidMax = {_f(wheel_controller["pidMax"])};  // [mm/s]
    return cfg;
}}

PlannerBootConfig defaultPlannerLimits() {{
    // 129-009 (config consolidation), field set reduced 130-009 -- fail-
    // closed baked from the robot JSON's `planner` block
    // (data/robots/robot_config.schema.json). See PlannerBootConfig's own
    // doc comment (src/firm/config/boot_config.h) for the current field
    // list and what was cut.
    PlannerBootConfig cfg;
    cfg.vMax = {_f(planner["vMax"])};              // [mm/s]
    cfg.aMax = {_f(planner["aMax"])};              // [mm/s^2]
    cfg.aDecel = {_f(planner["aDecel"])};          // [mm/s^2]
    cfg.omegaMax = {_f(planner["omegaMax"])};      // [rad/s]
    cfg.alphaMax = {_f(planner["alphaMax"])};      // [rad/s^2]
    cfg.alphaDecel = {_f(planner["alphaDecel"])};  // [rad/s^2]
    cfg.jerkMax = {_f(planner["jerkMax"])};        // [mm/s^3]
    cfg.yawJerkMax = {_f(planner["yawJerkMax"])};  // [rad/s^3]

    cfg.controlPeriod = {_f(planner["controlPeriod"])};    // [ms]
    cfg.actuationDelay = {_f(planner["actuationDelay"])};  // [ms]

    cfg.settleRestVelocity = {_f(planner["settleRestVelocity"])};    // [mm/s]
    cfg.settleRestOmega = {_f(planner["settleRestOmega"])};          // [rad/s]
    cfg.settleEpsilonLinear = {_f(planner["settleEpsilonLinear"])};  // [mm]
    cfg.settleEpsilonAngular = {_f(planner["settleEpsilonAngular"])};  // [rad]
    cfg.headingHoldGain = {_f(planner["headingHoldGain"])};  // [1/s]

    cfg.decelPlanFraction = {_f(planner["decelPlanFraction"])};  // [1]
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
