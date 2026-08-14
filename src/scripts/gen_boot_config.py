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
    114 ticket 003) — Hardware::NezhaMotor::writeShapedDuty()'s output-
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

`a_max`/`a_decel`/`j_max`/`yaw_jerk_max` -- formerly READ AGAIN here
(decel-into-the-goal campaign), the four exceptions to the paragraph
above: orphaned by 115-003 alongside every other
`motion_limits_for_config()` field, that campaign's `shaper_config_for_
config()` read them back a second time into `Config::ShaperBootConfig`.
That consumer (a velocity-shaping stage the campaign added) was deleted
as dead code in sprint 128 ticket 014, leaving `Config::ShaperBootConfig`
itself unread by anything (`Motion::Planner` uses its own hand-baked
`Motion::PlannerLimits`, not this struct) -- 132-015 (dead-code sweep)
deleted `shaper_config_for_config()`/`Config::ShaperBootConfig` outright,
along with the mirroring `msg::Planner.shaper_*` schema fields
(`robot_config.proto`, now `reserved`). These four `control.*` keys are
DEAD DATA again as of that ticket, exactly as 115-003 originally left
them -- an existing robot JSON may still carry them harmlessly.

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
above). Distinct from the now-deleted `shaper_config_for_config()`/
`Config::ShaperBootConfig` (132-015, see this module's own note above):
that struct was dead (unread by anything, superseded by
`Motion::Planner`'s own former hand-baked limits) and read the UNRELATED
legacy `control.a_max`/`a_decel`/... keys; `planner_config_for_config()`
reads the NEW `planner.*` keys instead and IS live (main.cpp constructs
its `Motion::PlannerLimits` from this function's output).

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

132-005 (sprint 132, "configuration discipline" — retarget baking) adds
seven new no-argument ``default*Group()`` functions — one per
``msg::ConfigGroupTarget`` (Geometry/Motors/Drive/WheelControl/Planner/Otos/
Estimator, `src/firm/messages/robot_config.h`, generated by ticket 002) —
alongside, not instead of, every function documented above. Each reads the
SAME `_require()`/`_get()` call sites already described in this docstring —
same JSON path, same fail-closed strictness — and writes the result into
the new generated `msg::Geometry`/`msg::Motors`/`msg::Drive`/
`msg::WheelControl`/`msg::Planner`/`msg::Otos`/`msg::Estimator` structs
instead of (or, for Motors' `travel_calib_left/right`/`fwd_sign_left/right`,
in addition to) the old hand-declared `boot_config.h` ones.

This is ADDITIVE, not a replacement, for exactly one sprint:
`boot_wiring.cpp`/`boot_calibration.cpp`/`main.cpp` still call the
pre-existing `default*Config()`/`default*BootConfig()` functions every
boot (confirmed by grep — none of those three files is in this ticket's
scope), so deleting them here would break HOST_BUILD/the ARM build today.
Ticket 006 ("Configurator owns Config::Robot") is what retargets
`RobotGraph`'s composition root onto `Configurator::loadBaked()` — which
calls these new `default*Group()` functions — at which point the old
family has no callers left and a later cleanup ticket can delete it.
Until then both families are baked from one robot JSON into two different
C++ shapes; this is a scoped, one-sprint duplication, not a permanent one.

The bridge itself — reading the CURRENT 13-section/`control`-dumping-ground
robot JSON shape into `Config::Robot`'s END-STATE consumer-grouped struct
layout — is a real, permanent step in the migration this generator has
always performed (`_require(cfg, "control", "vel_kp")`-style dotted-path
lookups), not a "keep the tree green between tickets" shim invented for
this sprint: ticket 017's JSON reshape does not remove this bridge, it
moves the JSON's OWN section boundaries to match `Config::Robot`'s
grouping, which only changes what dotted path each `_require()` call
reads, not whether the bridge exists.
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


def drive_ports(cfg):
    """(left_port, right_port) for THIS robot, defaulting to 1/2.

    Which physical motor port carries the LEFT wheel is per-robot wiring, not
    a universal constant. tovez is wired the other way round (port 1 is its
    RIGHT wheel), and hardcoding 1/2 made its firmware call the right wheel
    "left" -- which negates every heading derived from
    omega = (vR - vL) / b while leaving forward motion looking fine, so
    nothing surfaced it. Note this must move TOGETHER with the per-wheel
    calibrations (travel_calib_*, fwd_sign_*, wheel_gain_*): swapping only the
    values, with the ports fixed, inverts BOTH motors instead of relabelling
    them -- which reverses forward as well as rotation (measured 2026-08-13:
    a commanded +150 mm leg travelled -152 mm).

    PUBLIC on purpose: this is the ONE definition of the binding, and the
    host shares it (robot_radio.calibration.sim_boot_config's
    motor_boot_config_for()) rather than keeping a second copy that can
    skew. Absent OR zero means "not stated" -- proto3's uint32 default is 0,
    so a grouped RobotConfig for a robot whose JSON omits these keys arrives
    here as 0, and that must mean the historical 1/2, not port zero.
    """
    motors = cfg.get("motors", {}) or {}
    lp = motors.get("left_port") or LEFT_PORT
    rp = motors.get("right_port") or RIGHT_PORT
    return int(lp), int(rp)

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


def _i32(v) -> str:
    """Format a Python number as a C++ int32_t literal (rounded to the
    nearest integer -- a count-valued JSON field, e.g. planner.
    align_max_nudges, is a plain JSON number that the C++ side stores as
    int32_t)."""
    return f"{int(round(float(v)))}"


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

    The left/right drive-pair ports take motors.travel_calib_left/right
    (132-017 JSON reshape retarget -- was calibration.mm_per_wheel_deg_
    left/right before the grouped-shape migration; same soft-presence
    semantics, same placeholder fallback) when the robot JSON supplies
    them; every other port (and the pair, when the JSON omits them) uses
    the placeholder.
    """
    motors = cfg.get("motors", {}) or {}
    lp, rp = drive_ports(cfg)
    left  = _get(motors, "travel_calib_left")
    right = _get(motors, "travel_calib_right")
    out = []
    for port in range(1, K_MOTOR_COUNT + 1):
        if port == lp and left is not None:
            out.append(float(left))
        elif port == rp and right is not None:
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

    Mirrors travel_calib_for_ports()'s exact shape: the left/right
    drive-pair ports take motors.fwd_sign_left/right (132-017 JSON reshape
    retarget -- was calibration.fwd_sign_left/right before the
    grouped-shape migration) when the robot JSON supplies them; every
    other port (and the pair, when the JSON omits them) uses the
    FWD_SIGN placeholder.

    Unlike travel_calib, the drive pair is mirror-mounted (088-002 —
    clasi/issues/tovez-drive-motor-reversed-fwd-sign.md), so left and right
    are EXPECTED to differ in sign -- a straight-drive command with equal
    L/R targets must spin the two wheels in opposite raw-command directions
    to travel the same physical direction.
    """
    motors = cfg.get("motors", {}) or {}
    lp, rp = drive_ports(cfg)
    left  = _get(motors, "fwd_sign_left")
    right = _get(motors, "fwd_sign_right")
    out = []
    for port in range(1, K_MOTOR_COUNT + 1):
        if port == lp and left is not None:
            out.append(int(left))
        elif port == rp and right is not None:
            out.append(int(right))
        else:
            out.append(FWD_SIGN)
    return out


def otos_boot_config_values(cfg: dict):
    """Return (offsetX, offsetY, offsetYaw, linearScale, angularScale) for the
    OtosBootConfig struct (086-005), reading otos.offset_x/offset_y/
    offset_yaw and otos.linear_scale/angular_scale (132-017 JSON reshape
    retarget -- was geometry.odometry_offset_mm.{x,y,yaw_rad} +
    calibration.otos_linear_scale/otos_angular_scale before the
    grouped-shape migration; robot_config.proto's own header checklist
    groups these under Otos, not Geometry).

    All five are REQUIRED as of sprint 114 (config-as-truth completion) --
    a robot JSON missing any of them fails the generator loudly rather than
    silently substituting the old identity defaults (zero offset, 1.0 scale).
    """
    offset_x   = _require(cfg, "otos", "offset_x")
    offset_y   = _require(cfg, "otos", "offset_y")
    offset_yaw = _require(cfg, "otos", "offset_yaw")
    linear_scale  = _require(cfg, "otos", "linear_scale")
    angular_scale = _require(cfg, "otos", "angular_scale")
    return (float(offset_x), float(offset_y), float(offset_yaw),
            float(linear_scale), float(angular_scale))


def vel_gains_for_config(cfg: dict):
    """Return (kp, ki, kff, i_max, kaw, filt_alpha) for the velocity PID.

    Read from the robot JSON's ``motors`` block (132-017 JSON reshape
    retarget -- was ``control`` before the grouped-shape migration) -- ALL
    SIX keys are REQUIRED as of sprint 114 (config-as-truth completion;
    previously fell back to bench-tuned firmware defaults when absent).
    NOTE: these keys must be expressed in the NEW NezhaMotor duty [-1,1]
    plant scale (kp ~ 0.002, kff ~ 0.0015), NOT the old RobotConfig
    PWM-percent scale (kp ~ 0.3) — the robot JSON's own
    ``motors._vel_gains_domain`` marker documents this.
    """
    kp   = _require(cfg, "motors", "vel_kp")
    ki   = _require(cfg, "motors", "vel_ki")
    kff  = _require(cfg, "motors", "vel_kff")
    imax = _require(cfg, "motors", "vel_i_max")
    kaw  = _require(cfg, "motors", "vel_kaw")
    filt = _require(cfg, "motors", "vel_filt_alpha")
    return float(kp), float(ki), float(kff), float(imax), float(kaw), float(filt)


def output_deadband_for_config(cfg: dict):
    """Return motors.output_deadband (duty fraction [-1,1]) -- Devices::
    NezhaMotor::writeShapedDuty()'s output-deadband floor (folded from the
    old MotorArmor base) and MotorArmor's own wedge-suspect motion-gate
    threshold. 132-017 JSON reshape retarget -- was control.output_deadband
    before the grouped-shape migration. REQUIRED as of sprint 114 ticket 003
    (config-as-truth completion) -- previously left unset (.has == false) on
    purpose, with NezhaMotor's own kDefaultOutputDeadband (0.03) substituted
    in the constructor whenever a config arrived unset; that substitution is
    gone, so every robot JSON must now carry a real value."""
    return float(_require(cfg, "motors", "output_deadband"))


def reversal_dwell_for_config(cfg: dict):
    """Return motors.reversal_dwell [ms] -- Hardware::NezhaMotor::
    writeShapedDuty()'s reversal-dwell hold time (folded from the old
    MotorArmor base). 132-017 JSON reshape retarget -- was
    control.reversal_dwell_ms before the grouped-shape migration (the
    JSON key's own "_ms" unit suffix is dropped per coding-standards.md
    "no units in ANY identifier"; unit stays in this function's own "[ms]"
    comment tag). REQUIRED as of sprint 114 ticket 003 (config-as-truth
    completion) -- previously left unset (.has == false) on purpose, with
    NezhaMotor's own kDefaultReversalDwell (100.0) substituted in the
    constructor whenever a config arrived unset; that substitution is gone,
    so every robot JSON must now carry a real value."""
    return float(_require(cfg, "motors", "reversal_dwell"))


def trackwidth_for_config(cfg: dict) -> float:
    """Return geometry.trackwidth [mm] -> DrivetrainConfig.trackwidth.
    Unchanged JSON path across the 132-017 reshape (trackwidth already
    lived under `geometry` in the old shape too). REQUIRED as of sprint
    114 (config-as-truth completion) -- previously fell back to a 128.0mm
    placeholder when absent."""
    return float(_require(cfg, "geometry", "trackwidth"))


def rotational_slip_for_config(cfg: dict) -> float:
    """Return geometry.rotational_slip -> DrivetrainConfig.rotational_slip.
    132-017 JSON reshape retarget -- was calibration.rotational_slip
    before the grouped-shape migration.

    The scrub factor: a differential robot skids its wheels sideways through
    a turn, so it rotates LESS than ideal kinematics (omega = (vR-vL)/b)
    predicts for a given wheel differential. This is the ratio of actual to
    ideal rotation, so the EFFECTIVE track is trackwidth / rotational_slip
    (main.cpp does that division; trackwidth itself stays the physically
    measured wheel separation and must not be bent to absorb scrub).

    Domain is `{0} u [0.5, 1.0]` (robot_config.proto): 0 is the
    "uncalibrated" sentinel and means apply no correction, NOT
    divide-by-zero.
    """
    return float(_require(cfg, "geometry", "rotational_slip"))


def rotation_calibration_for_config(cfg: dict):
    """Return (gain_pos, offset_pos_deg, gain_neg, offset_neg_deg).

    132-017 JSON reshape retarget -- was calibration.rotation_gain/
    rotation_offset_deg/rotation_gain_neg/rotation_offset_deg_neg before
    the grouped-shape migration; now geometry.rotation_gain_pos/
    rotation_offset/rotation_gain_neg/rotation_offset_neg, matching
    robot_config.proto's own Geometry message field names exactly.

    The measured affine turn response, `actual = gain*commanded + offset`,
    per direction of rotation. RobotLoop inverts it so an ANGLE-stopped move
    lands on the requested angle. `_pos` is positive commanded omega.

    Offsets stay in DEGREES here (the unit the robot JSON and a human use);
    main.cpp converts to radians at the seam.
    """
    return (float(_require(cfg, "geometry", "rotation_gain_pos")),
            float(_require(cfg, "geometry", "rotation_offset")),
            float(_require(cfg, "geometry", "rotation_gain_neg")),
            float(_require(cfg, "geometry", "rotation_offset_neg")))


def estimator_config_for_config(cfg: dict):
    """Return (heading_otos, omega_otos, staleness) for the
    EstimatorBootConfig struct (117, predict-to-now estimator v1) --
    Core::StateEstimator's fail-closed boot-time fusion-weight defaults.

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
    # 132-017 JSON reshape retarget: staleness (not staleness_ms) -- the
    # JSON key's own "_ms" unit suffix is dropped, matching
    # robot_config.proto's Estimator.staleness field name exactly; unit
    # stays in this function's own docstring/comment tags.
    staleness = _require(cfg, "estimator", "staleness")
    return float(heading_otos), float(omega_otos), float(staleness)


def wheel_correction_for_config(cfg: dict):
    """Return the 8 commanded->actual correction values in the order
    (gain, intercept) x (left, right) x (accel, decel) for
    Config::DriveBootConfig.

    measured = gain*commanded + intercept, per wheel per direction of
    approach, from docs/design/wheel-speed-command-mapping.md. Core::DifferentialDrive
    inverts it to seed the feedforward: command = (desired-intercept)/gain.

    The correction is defined RELATIVE to the duty_per_speed constant it was
    measured against, so both must come from the same characterization run.
    gain 1 / intercept 0 is the identity (an uncalibrated robot); a gain of
    0 or less is meaningless and aborts.

    132-017 JSON reshape retarget: reads drive.wheel_gain_*/
    wheel_intercept_* -- was control.wheel_gain_*/wheel_intercept_* before
    the grouped-shape migration.

    All REQUIRED -- same fail-closed posture as every other baked field."""
    out = []
    for wheel in ("left", "right"):
        for direction in ("accel", "decel"):
            gain = float(_require(cfg, "drive", f"wheel_gain_{wheel}_{direction}"))
            icpt = float(_require(cfg, "drive", f"wheel_intercept_{wheel}_{direction}"))
            if gain <= 0.0:
                raise SystemExit(
                    f"drive.wheel_gain_{wheel}_{direction} must be > 0 (got {gain})")
            out.append((gain, icpt))
    return out


def drive_config_for_config(cfg: dict):
    """Return (duty_per_speed_left, duty_per_speed_right, crawl_pulse) for
    Config::DriveBootConfig (command-ingestion-ring-buffered-comms-
    subsystem-routing-two-stops.md §6) -- Core::DifferentialDrive's open-loop wheel
    calibration and its crawl-shaper amplitude.

    These were HARD-CODED in C++ before this change: the duty-per-speed pair
    as member initializers on Core::DifferentialDrive itself, the crawl amplitude as a
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
    different robot. Core::DifferentialDrive itself now carries NO calibration defaults
    at all -- an unconfigured Drive refuses to drive (drive.h), the same
    posture RobotLoop's `configured_` gate already takes for motion
    commands.

    132-017 JSON reshape retarget: reads drive.duty_per_speed_left/right/
    crawl_pulse -- was control.duty_per_speed_left/right/crawl_pulse
    before the grouped-shape migration.
    """
    duty_left = _require(cfg, "drive", "duty_per_speed_left")
    duty_right = _require(cfg, "drive", "duty_per_speed_right")
    crawl = _require(cfg, "drive", "crawl_pulse")
    return (float(duty_left), float(duty_right), float(crawl))


def wheel_controller_config_for_config(cfg: dict) -> dict:
    """Return a dict of every Config::WheelControllerBootConfig field
    (boot_config.h), keyed by its C++ field name, read from the robot
    JSON's `wheel_control.*` keys (130-004, wheel-speed-controller-moves-
    into-drive.md Phase 2). 132-017 JSON reshape retarget: was
    `control.wheel_*`/`control.wheel_pid_*`/`control.wheel_deficit_*`
    before the grouped-shape migration -- the `wheel_`/`wheel_pid_`/
    `wheel_deficit_` prefixes are dropped since the section itself is now
    named `wheel_control` (matching robot_config.proto's WheelControl
    message field names exactly).

    Core::DifferentialDrive's unified three-timescale wheel-speed controller: Stage
    B's wire-tunable fast-PID gains (pid_kp/ki/i_max/kaff/max, plus
    133-002's pos_err_max) and Stage C/deficit-flag's generated-constant
    bounds (v_min/bias_max/tau_adapt/a_steady/deficit_threshold/
    deficit_window).

    pos_err_max (133-002) is the BAKE half of the same field's runtime
    wire path (Configurator's existing WHEEL_CONTROL machinery reaches it
    through the generated codec, no new arm needed) -- both halves read
    THIS file, per .claude/rules/configuration-discipline.md invariant 2.

    All 12 REQUIRED, same fail-closed posture as every other field this
    generator bakes: a robot JSON missing any one of them fails codegen
    loudly. A robot JSON is free to set every one of these to 0 (both
    stages inert) -- see WheelControllerBootConfig's own doc comment
    (boot_config.h) for why 0 is a safe, meaningful disabled state here,
    not a placeholder standing in for a value the code requires nonzero.
    """
    return {
        "vMin": float(_require(cfg, "wheel_control", "v_min")),
        "biasMax": float(_require(cfg, "wheel_control", "bias_max")),
        "tauAdapt": float(_require(cfg, "wheel_control", "tau_adapt")),
        "aSteady": float(_require(cfg, "wheel_control", "a_steady")),
        "deficitThreshold": float(_require(cfg, "wheel_control", "deficit_threshold")),
        "deficitWindow": float(_require(cfg, "wheel_control", "deficit_window")),
        "kp": float(_require(cfg, "wheel_control", "pid_kp")),
        "ki": float(_require(cfg, "wheel_control", "pid_ki")),
        "iMax": float(_require(cfg, "wheel_control", "pid_i_max")),
        "kaff": float(_require(cfg, "wheel_control", "pid_kaff")),
        "pidMax": float(_require(cfg, "wheel_control", "pid_max")),
        "posErrMax": float(_require(cfg, "wheel_control", "pos_err_max")),
        # Stall detection (2026-08-08). _require, not a defaulted get: a robot
        # JSON that omits these bakes a robot with NO stall protection, and
        # config-as-truth says that must fail loudly at build time rather than
        # ship silently disabled. Setting stall_window to 0 in the JSON is the
        # supported way to turn the detector off deliberately.
        "stallSpeed": float(_require(cfg, "wheel_control", "stall_speed")),
        "stallDemand": float(_require(cfg, "wheel_control", "stall_demand")),
        "stallWindow": float(_require(cfg, "wheel_control", "stall_window")),
    }


def planner_config_for_config(cfg: dict) -> dict:
    """Return a dict of every Config::PlannerBootConfig field (boot_config.h),
    keyed by its C++ field name, read from the robot JSON's `planner`
    block (129-009, config consolidation; field set reduced 130-009) PLUS
    the six shaper-ceiling fields, which 132-017 (JSON reshape ticket,
    stakeholder-sanctioned mid-sprint Planner-split) moved to their OWN
    top-level `planner_shaper` JSON section (see robot_config.proto's
    PlannerShaper message header comment for why). This function still
    returns ONE flat dict spanning both JSON sections, unchanged shape --
    its two callers (defaultPlannerLimits(), the boot-only 16-field
    PlannerBootConfig struct below, and defaultPlannerGroup()/
    defaultPlannerShaperGroup(), Config::Robot's own SPLIT groups) need no
    code changes from this retarget.

    Before 129-009 every one of these values was a C++ literal assembled
    directly in main.cpp's Motion::PlannerLimits construction -- this is
    the move to config-as-truth (sprint 114) that block itself called out
    as still owed ("A planner-domain config surface can supersede these
    constants later").

    All 16 raw keys are REQUIRED, same fail-closed posture as every other
    field this generator bakes: a robot JSON missing either the `planner`
    or `planner_shaper` block (or any key inside either) fails codegen
    loudly rather than a robot inheriting another robot's plant
    measurements.

    130-009 (planner-honesty-pass-...limits-reduction.md item 3): cut the
    11 raw keys that fed PlannerLimits' now-deleted M4 duty-stage/settle-
    confirm fields (vel_kp/vel_ki/vel_i_max/vel_i_accel_gate/duty_floor/
    require_settle/settle_window) and the 5 that fed the planner-side
    trim gains WheelTrim left dead (trim_kp/trim_ki/trim_i_max/trim_max,
    plus the derived trim_kaff) -- see PlannerBootConfig's own doc comment
    (boot_config.h) for why each is gone. `plant_gain` is no longer read
    here (its only consumers, velKff/velKaff, are both cut); `plant_tau`
    is ALSO no longer read (its only consumer, trimKaff, is cut too) --
    both keys may still be present in a robot JSON (132-017: relocated to
    `planner._plant_gain`/`_plant_tau`, underscore-prefixed -- recorded
    measured data with no schema field, not silently dropped by the
    reshape), simply unread by this function now.
    """
    return {
        "vMax": float(_require(cfg, "planner", "v_max")),
        "aMax": float(_require(cfg, "planner_shaper", "a_max")),
        "aDecel": float(_require(cfg, "planner_shaper", "a_decel")),
        "omegaMax": float(_require(cfg, "planner", "omega_max")),
        "alphaMax": float(_require(cfg, "planner_shaper", "alpha_max")),
        "alphaDecel": float(_require(cfg, "planner_shaper", "alpha_decel")),
        "jerkMax": float(_require(cfg, "planner_shaper", "jerk_max")),
        "yawJerkMax": float(_require(cfg, "planner_shaper", "yaw_jerk_max")),

        "controlPeriod": float(_require(cfg, "planner", "control_period")),
        "actuationDelay": float(_require(cfg, "planner", "actuation_delay")),

        "settleRestVelocity": float(_require(cfg, "planner", "settle_rest_velocity")),
        "settleRestOmega": float(_require(cfg, "planner", "settle_rest_omega")),
        "settleEpsilonLinear": float(_require(cfg, "planner", "settle_epsilon_linear")),
        "settleEpsilonAngular": float(_require(cfg, "planner", "settle_epsilon_angular")),
        "headingHoldGain": float(_require(cfg, "planner", "heading_hold_gain")),

        "decelPlanFraction": float(_require(cfg, "planner", "decel_plan_fraction")),

        # Terminal fine-align (134-003). alignTol is [rad] -- the robot JSON
        # is where the report's 1.0 DEGREE operating point is converted, once
        # (0.017453 rad); nothing downstream re-converts. alignMaxNudges is a
        # plain count, carried as an int32 so PlannerLimits::Landing keeps its
        # uniform 4-byte stride.
        "alignTol": float(_require(cfg, "planner", "align_tol")),
        "alignMaxNudges": int(_require(cfg, "planner", "align_max_nudges")),
    }


def navigator_config_for_config(cfg: dict) -> dict:
    """Return a dict of every Motion::NavigatorLimits field this schema
    carries (robot_config.proto's Navigator message, 135-004), read from
    the robot JSON's `navigator` block. `track_width` is deliberately NOT
    read/returned here -- NavigatorLimits::trackWidth is sourced from
    Config::Robot::effectiveTrackWidth() at Core::configureNavigator() time
    (config/robot.h), not duplicated as a second, independently-tunable
    copy of the trackwidth/rotational_slip pair Geometry already owns
    (configuration-discipline.md: "one file, one truth").

    All 8 REQUIRED, same fail-closed posture as every other group this
    generator bakes -- a robot JSON missing the `navigator` block, or any
    key inside it, fails codegen loudly rather than a robot silently
    inheriting a C++ struct default it never actually configured.
    """
    return {
        "speed": float(_require(cfg, "navigator", "speed")),
        "maxWheelStep": float(_require(cfg, "navigator", "max_wheel_step")),
        "behindAngle": float(_require(cfg, "navigator", "behind_angle")),
        "turnFirstAngle": float(_require(cfg, "navigator", "turn_first_angle")),
        "approachRadius": float(_require(cfg, "navigator", "approach_radius")),
        "approachSpeed": float(_require(cfg, "navigator", "approach_speed")),
        "defaultArrivalTolerance": float(_require(cfg, "navigator", "default_arrival_tolerance")),
        "yawSign": float(_require(cfg, "navigator", "yaw_sign")),
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


def radio_channel_for_config(cfg: dict) -> int:
    """``connection.radio_channel`` (robot_config.proto's Connection group,
    host-only) -- the nRF frequency band ``main.cpp`` passes to
    ``Radio::begin()``. Defaults to 0 (the historical hard-coded value and
    the RadioRelay default) when absent, so every pre-existing robot JSON
    keeps its behavior without an edit. Clamped to the nRF band range
    [0, 83] (``Radio::begin()``'s own documented domain) rather than
    trusting the file blindly -- a robot silently deaf on a nonsense band
    looks exactly like a dead radio."""
    channel = int(_get(cfg, "connection", "radio_channel", default=0))
    return min(83, max(0, channel))


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
        (drive_duty_per_speed_left, drive_duty_per_speed_right,
         drive_crawl_pulse) = drive_config_for_config(cfg)
        wheel_corr = wheel_correction_for_config(cfg)
        wheel_controller = wheel_controller_config_for_config(cfg)
        planner = planner_config_for_config(cfg)
        navigator = navigator_config_for_config(cfg)
    except MissingRobotConfigKeyError as e:
        raise e.with_source(source_path) from e

    profile_name = profile_name_for_source(source_path)
    drivetrain_type = drivetrain_type_for_config(cfg)
    radio_channel = radio_channel_for_config(cfg)

    # Config::Robot's Motors group (132-005) only has room for the drive
    # pair's own left/right values -- no per-port array, no placeholder
    # ports 3/4 (those are structural hardware-wiring facts, not part of
    # this schema; see robot_config.proto's own header checklist). Reuse
    # the SAME travel_calib/fwd_sign lists computed above (same _get() call
    # sites, same placeholder fallback) rather than re-deriving them.
    #
    # Indexed by THIS robot's own port binding (drive_ports), not by the
    # LEFT_PORT/RIGHT_PORT defaults. The group's fields are labelled by
    # WHEEL, and that is how they are consumed: boot_wiring.cpp binds
    # `motorL_` to `drivetrainConfig.left_port`, and configurator.cpp's
    # live MOTORS push calls `configureMotor(motorL_, config, isLeft=true)`
    # -> `config.motors.travel_calib_left`. Indexing at the DEFAULT ports
    # here transposed both pairs on any robot wired the other way round
    # (tovez): the baked group reported port 1's calibration as "left"
    # while defaultMotorConfigs() correctly put the left wheel's on port 2,
    # so a live MOTORS push applied each wheel the OTHER wheel's travel
    # calib. Round-tripping the JSON labels exactly is the invariant here
    # (configuration-discipline.md: a pushed config and a rebuilt image
    # cannot disagree).
    drive_left_index, drive_right_index = drive_ports(cfg)
    motors_travel_calib_left = travel_calib[drive_left_index - 1]
    motors_travel_calib_right = travel_calib[drive_right_index - 1]
    motors_fwd_sign_left = fwd_sign[drive_left_index - 1]
    motors_fwd_sign_right = fwd_sign[drive_right_index - 1]

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

    # Per-robot drive-pair binding (see drive_ports): which physical port
    # carries the LEFT wheel is wiring, not a constant.
    DRIVE_LEFT_PORT, DRIVE_RIGHT_PORT = drive_ports(cfg)

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

// kRadioChannel — see boot_config.h's own doc comment. Baked from
// connection.radio_channel (robot JSON), defaulting to 0 (the historical
// hard-coded Radio::begin() value) when absent.
const int kRadioChannel = {radio_channel};  // [nRF frequency band]

void defaultMotorConfigs(msg::MotorConfig* out) {{
    // Velocity PID gains — baked from the robot JSON's motors.vel_* keys
    // (132-017 JSON reshape retarget -- was control.vel_* before the
    // grouped-shape migration; 093: now in the NezhaMotor duty [-1,1]
    // plant scale, see the JSON's motors._vel_gains_domain marker),
    // falling back to bench-tuned firmware defaults when absent.
    // Live-correctable per motor via `DEV M <n> CFG`.
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
        // EMA coeff — from motors.vel_filt_alpha (fallback default); a=0
        // would pin reported velocity at 0 forever regardless of real
        // motion.
        out[i].setVelFiltAlpha({_f(vel_filt)});
        // Write-shaping floor/hold — baked from the robot JSON's
        // motors.output_deadband/motors.reversal_dwell (132-017 JSON
        // reshape retarget -- was control.output_deadband/control.
        // reversal_dwell_ms before the grouped-shape migration; sprint 114
        // ticket 003, config-as-truth completion). REQUIRED as of that
        // ticket: Hardware::NezhaMotor no longer substitutes a ship default
        // when these arrive unset, so every build must emit a real value.
        out[i].setOutputDeadband({_f(output_deadband)});   // [-1,1] fraction
        out[i].setReversalDwell({_f(reversal_dwell)});   // [ms]
    }}

    // Per-port forward-sign — baked from the robot JSON's motors.
    // fwd_sign_{{left,right}} (132-017 JSON reshape retarget -- was
    // calibration.fwd_sign_{{left,right}} before the grouped-shape
    // migration) for the drive-pair ports (ports {LEFT_PORT}/{RIGHT_PORT});
    // other ports use the bench placeholder ({FWD_SIGN}). The drive pair is
    // mirror-mounted, so left/right are expected to differ in sign
    // (088-002 — clasi/issues/tovez-drive-motor-reversed-fwd-sign.md).
{fwd_sign_lines}

    // Per-port encoder travel calibration — baked from the robot JSON's
    // motors.travel_calib_{{left,right}} (132-017 JSON reshape retarget --
    // was calibration.mm_per_wheel_deg_{{left,right}} before the
    // grouped-shape migration) for the drive-pair ports (ports
    // {LEFT_PORT}/{RIGHT_PORT}); other ports use the bench placeholder.
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
    cfg.setLeftPort({DRIVE_LEFT_PORT});
    cfg.setRightPort({DRIVE_RIGHT_PORT});
    return cfg;
}}

OtosBootConfig defaultOtosBootConfig() {{
    // 086-005 — additive to defaultMotorConfigs()/defaultDrivetrainConfig()
    // above; no existing mapping touched. Baked from the robot JSON's
    // otos.offset_x/offset_y/offset_yaw and otos.linear_scale/
    // angular_scale (132-017 JSON reshape retarget -- was geometry.
    // odometry_offset_mm.{{x,y,yaw_rad}} + calibration.otos_linear_scale/
    // otos_angular_scale before the grouped-shape migration) where
    // present; identity defaults (zero offset, 1.0 scale) otherwise.
    // Boot-time-baked only -- see
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
    // JSON's estimator.weight_heading_otos/weight_omega_otos/staleness
    // (data/robots/robot_config.schema.json; 132-017 JSON reshape retarget
    // -- was estimator.staleness_ms before the grouped-shape migration).
    // Encoder-only v1 (stakeholder decision): both blend weights are
    // committed 0.0 in every robot JSON this sprint -- see that JSON's own
    // inline comment for the staleness reasoning. NOT a live SET/wire
    // surface itself -- see
    // EstimatorBootConfig's own doc comment (src/firm/config/boot_config.h)
    // for the separate, volatile EstimatorConfigPatch live-tuning path.
    EstimatorBootConfig cfg;
    cfg.headingOtos = {_f(estimator_heading_otos)};
    cfg.omegaOtos = {_f(estimator_omega_otos)};
    cfg.staleness = {_u32(estimator_staleness)};   // [ms]
    return cfg;
}}

// ShaperBootConfig/defaultShaperConfig(), DriveBootConfig/
// defaultDriveConfig(), WheelControllerBootConfig/
// defaultWheelControllerConfig() -- DELETED, 132-015 (dead-code sweep).
// See config/boot_config.h's own note at each struct's former declaration
// site for the full rationale (all three confirmed zero live consumers by
// a fresh grep; superseded by Config::defaultDriveGroup()/
// defaultWheelControlGroup(), Config::Robot's own generated groups,
// below). shaper_config_for_config() (this module) is deleted alongside
// its one-and-only two consumers (defaultShaperConfig() here and
// defaultPlannerGroup()'s own shaper_* fields below, robot_config.proto's
// Planner message now `reserved`s field numbers 17-22 instead of
// declaring them).

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

    cfg.alignTol = {_f(planner["alignTol"])};  // [rad]
    cfg.alignMaxNudges = {_i32(planner["alignMaxNudges"])};
    return cfg;
}}

// ---------------------------------------------------------------------------
// Config::Robot group defaults (132-005, sprint 132 "configuration
// discipline: one owned object..." -- retarget baking).
//
// Seven new no-argument functions, one per msg::ConfigGroupTarget
// (src/firm/messages/robot_config.h, generated by ticket 002 from
// src/protos/robot_config.proto): the SAME robot-JSON `_require()`/`_get()`
// call sites as the functions above, retargeted onto the NEW generated
// msg::Geometry/Motors/Drive/WheelControl/Planner/Otos/Estimator group
// structs instead of the old hand-declared boot_config.h ones.
//
// ADDITIVE for this ticket, not a replacement -- see gen_boot_config.py's
// own module docstring for why the functions above stay untouched (ticket
// 006, "Configurator owns Config::Robot", is what retargets RobotGraph's
// composition root onto Configurator::loadBaked(), which is the first
// thing to actually call these).
// ---------------------------------------------------------------------------

msg::Geometry defaultGeometryGroup() {{
    // geometry.trackwidth/rotational_slip/rotation_gain_pos/
    // rotation_offset/rotation_gain_neg/rotation_offset_neg (132-017 JSON
    // reshape retarget -- rotational_slip/rotation_* used to live under
    // `calibration` before the grouped-shape migration) -- see
    // trackwidth_for_config()/rotational_slip_for_config()/
    // rotation_calibration_for_config() above.
    msg::Geometry cfg;
    cfg.trackwidth = {_f(trackwidth)};                  // [mm]
    cfg.rotational_slip = {_f(rot_slip)};                // scrub: actual/ideal rotation, 0 = uncalibrated
    cfg.rotation_gain_pos = {_f(rot_cal[0])};
    cfg.rotation_offset = {_f(rot_cal[1])};              // [deg]
    cfg.rotation_gain_neg = {_f(rot_cal[2])};
    cfg.rotation_offset_neg = {_f(rot_cal[3])};          // [deg]
    return cfg;
}}

msg::Motors defaultMotorsGroup() {{
    // Drive-pair-only slice of travel_calib_for_ports()/fwd_sign_for_ports()
    // (this robot's OWN drive ports, left={DRIVE_LEFT_PORT} right={DRIVE_RIGHT_PORT} --
    // Config::Robot's schema has no per-port array, see this file's own
    // comment above) plus motors.vel_*/output_deadband/reversal_dwell
    // (132-017 JSON reshape retarget -- all lived under `control` before the
    // grouped-shape migration), shared by both bound motors
    // (vel_gains_for_config()/output_deadband_for_config()/
    // reversal_dwell_for_config() above).
    msg::Motors cfg;
    // The port binding itself, so this group round-trips the robot JSON
    // exactly (configuration-discipline.md invariant 2). Boot-only: the
    // live MOTORS push path (Core::configureMotor) applies travel_calib
    // only, and re-binding a running robot's drive ports is not a thing it
    // does. msg::DrivetrainConfig (defaultDrivetrainConfig(), above) is
    // what boot_wiring.cpp actually reads to bind motorL_/motorR_.
    cfg.left_port = {DRIVE_LEFT_PORT};
    cfg.right_port = {DRIVE_RIGHT_PORT};
    cfg.travel_calib_left = {_f(motors_travel_calib_left)};    // [mm/deg]
    cfg.travel_calib_right = {_f(motors_travel_calib_right)};  // [mm/deg]
    cfg.fwd_sign_left = {motors_fwd_sign_left};
    cfg.fwd_sign_right = {motors_fwd_sign_right};
    cfg.output_deadband = {_f(output_deadband)};         // [-1,1] fraction
    cfg.reversal_dwell = {_f(reversal_dwell)};            // [ms]
    cfg.vel_kp = {_f(vel_kp)};
    cfg.vel_ki = {_f(vel_ki)};
    cfg.vel_kff = {_f(vel_kff)};
    cfg.vel_i_max = {_f(vel_imax)};
    cfg.vel_kaw = {_f(vel_kaw)};                          // anti-windup back-calculation; 0 = off
    cfg.vel_filt_alpha = {_f(vel_filt)};
    return cfg;
}}

msg::Drive defaultDriveGroup() {{
    // drive.duty_per_speed_left/right/crawl_pulse (132-017 JSON reshape
    // retarget -- was control.duty_per_speed_left/right/crawl_pulse
    // before the grouped-shape migration; drive_config_for_config() above)
    // plus the Stage-A per-wheel commanded->actual correction (drive.
    // wheel_gain_*/wheel_intercept_*, wheel_correction_for_config() above)
    // -- this sprint's headline per-wheel drive calibration surface
    // (SUC-006).
    msg::Drive cfg;
    cfg.duty_per_speed_left = {_f(drive_duty_per_speed_left)};    // [duty/(mm/s)]
    cfg.duty_per_speed_right = {_f(drive_duty_per_speed_right)};  // [duty/(mm/s)]
    cfg.crawl_pulse = {_f(drive_crawl_pulse)};                    // [-1,1]; 0 = off
    cfg.wheel_gain_left_accel = {_f(wheel_corr[0][0])};
    cfg.wheel_intercept_left_accel = {_f(wheel_corr[0][1])};      // [mm/s]
    cfg.wheel_gain_left_decel = {_f(wheel_corr[1][0])};
    cfg.wheel_intercept_left_decel = {_f(wheel_corr[1][1])};      // [mm/s]
    cfg.wheel_gain_right_accel = {_f(wheel_corr[2][0])};
    cfg.wheel_intercept_right_accel = {_f(wheel_corr[2][1])};     // [mm/s]
    cfg.wheel_gain_right_decel = {_f(wheel_corr[3][0])};
    cfg.wheel_intercept_right_decel = {_f(wheel_corr[3][1])};     // [mm/s]
    return cfg;
}}

msg::WheelControl defaultWheelControlGroup() {{
    // wheel_control.v_min/bias_max/tau_adapt/a_steady/deficit_threshold/
    // deficit_window/pid_*/pos_err_max (132-017 JSON reshape retarget --
    // was control.wheel_*/wheel_pid_*/wheel_deficit_* before the
    // grouped-shape migration; pos_err_max added 133-002) --
    // wheel_controller_config_for_config() above.
    msg::WheelControl cfg;
    cfg.v_min = {_f(wheel_controller["vMin"])};                       // [mm/s]
    cfg.bias_max = {_f(wheel_controller["biasMax"])};                 // [mm/s]
    cfg.tau_adapt = {_f(wheel_controller["tauAdapt"])};                // [s]
    cfg.a_steady = {_f(wheel_controller["aSteady"])};                  // [mm/s^2]
    cfg.deficit_threshold = {_f(wheel_controller["deficitThreshold"])};  // [mm/s]
    cfg.deficit_window = {_f(wheel_controller["deficitWindow"])};      // [ms]
    cfg.pid_kp = {_f(wheel_controller["kp"])};        // [1]
    cfg.pid_ki = {_f(wheel_controller["ki"])};        // [1/s]
    cfg.pid_i_max = {_f(wheel_controller["iMax"])};    // [mm/s]
    cfg.pid_kaff = {_f(wheel_controller["kaff"])};    // [s]
    cfg.pid_max = {_f(wheel_controller["pidMax"])};    // [mm/s]
    cfg.pos_err_max = {_f(wheel_controller["posErrMax"])};  // [mm]
    cfg.stall_speed = {_f(wheel_controller["stallSpeed"])};  // [mm/s]
    cfg.stall_demand = {_f(wheel_controller["stallDemand"])};  // [mm/s]
    cfg.stall_window = {_f(wheel_controller["stallWindow"])};  // [ms]
    return cfg;
}}

msg::Planner defaultPlannerGroup() {{
    // planner.* (planner_config_for_config() above) -- the BOOT-ONLY
    // remainder. The six shaper-ceiling fields (a_max/a_decel/alpha_max/
    // alpha_decel/jerk_max/yaw_jerk_max) are SPLIT OUT, 132-017 (JSON
    // reshape ticket, stakeholder-sanctioned mid-sprint scope addition):
    // see defaultPlannerShaperGroup() immediately below, and robot_config.
    // proto's PlannerShaper message header comment for why. The formerly-
    // DEAD shaper_* fields (control.a_max/a_decel/alpha_max/alpha_decel/
    // j_max/yaw_jerk_max) are DELETED, 132-015 -- see robot_config.proto's
    // own Planner message (now `reserved 17 to 22`) and this module's own
    // note at defaultShaperConfig()'s former spot above. (Two DIFFERENT
    // things share the word "shaper" here: 132-015 deleted a DEAD
    // shaper_*-prefixed field set; 132-017 split a LIVE, un-prefixed
    // a_max/... field set into its own group -- see PlannerShaper's own
    // header comment for the distinction.)
    msg::Planner cfg;
    cfg.v_max = {_f(planner["vMax"])};                          // [mm/s]
    cfg.omega_max = {_f(planner["omegaMax"])};                  // [rad/s]
    cfg.control_period = {_f(planner["controlPeriod"])};        // [ms]
    cfg.actuation_delay = {_f(planner["actuationDelay"])};      // [ms]
    cfg.settle_rest_velocity = {_f(planner["settleRestVelocity"])};    // [mm/s]
    cfg.settle_rest_omega = {_f(planner["settleRestOmega"])};          // [rad/s]
    cfg.settle_epsilon_linear = {_f(planner["settleEpsilonLinear"])};  // [mm]
    cfg.settle_epsilon_angular = {_f(planner["settleEpsilonAngular"])};  // [rad]
    cfg.heading_hold_gain = {_f(planner["headingHoldGain"])};   // [1/s]
    cfg.decel_plan_fraction = {_f(planner["decelPlanFraction"])};  // [1]
    cfg.align_tol = {_f(planner["alignTol"])};  // [rad]
    cfg.align_max_nudges = {_i32(planner["alignMaxNudges"])};
    return cfg;
}}

msg::PlannerShaper defaultPlannerShaperGroup() {{
    // planner.a_max/a_decel/alpha_max/alpha_decel/jerk_max/yaw_jerk_max
    // (planner_config_for_config() above) -- the LIVE shaper-ceiling
    // group split out of Planner, 132-017. Same JSON source keys (the
    // `planner` section is not itself reshaped by this split -- only the
    // GENERATED group these values are baked into changes), read once by
    // planner_config_for_config() and reused here, not re-derived.
    msg::PlannerShaper cfg;
    cfg.a_max = {_f(planner["aMax"])};                // [mm/s^2]
    cfg.a_decel = {_f(planner["aDecel"])};            // [mm/s^2]
    cfg.alpha_max = {_f(planner["alphaMax"])};        // [rad/s^2]
    cfg.alpha_decel = {_f(planner["alphaDecel"])};    // [rad/s^2]
    cfg.jerk_max = {_f(planner["jerkMax"])};          // [mm/s^3]
    cfg.yaw_jerk_max = {_f(planner["yawJerkMax"])};   // [rad/s^3]
    return cfg;
}}

msg::Otos defaultOtosGroup() {{
    // geometry.odometry_offset_mm.{{x,y,yaw_rad}}, calibration.
    // otos_linear_scale/otos_angular_scale -- otos_boot_config_values()
    // above. These are the config MULTIPLIER domain (1.0 = no correction),
    // same as the robot JSON -- Core::configureOtos() (app/boot_calibration.
    // cpp) converts through Hardware::scaleToRegister() before reaching the
    // chip, the SAME conversion RealOtos::begin() applies to this baked
    // value at boot (132-010 closed the live/boot domain mismatch, trap 3,
    // sprint.md).
    msg::Otos cfg;
    cfg.offset_x = {_f(otos_offset_x)};          // [mm]
    cfg.offset_y = {_f(otos_offset_y)};          // [mm]
    cfg.offset_yaw = {_f(otos_offset_yaw)};      // [rad]
    cfg.linear_scale = {_f(otos_linear_scale)};
    cfg.angular_scale = {_f(otos_angular_scale)};
    return cfg;
}}

msg::Estimator defaultEstimatorGroup() {{
    // estimator.weight_heading_otos/weight_omega_otos/staleness --
    // estimator_config_for_config() above.
    msg::Estimator cfg;
    cfg.weight_heading_otos = {_f(estimator_heading_otos)};
    cfg.weight_omega_otos = {_f(estimator_omega_otos)};
    cfg.staleness = {_u32(estimator_staleness)};   // [ms]
    return cfg;
}}

msg::Navigator defaultNavigatorGroup() {{
    // navigator.* (navigator_config_for_config() above) -- Motion::
    // NavigatorLimits' own configuration group (135-004). track_width is
    // NOT baked here -- Core::configureNavigator() (app/boot_calibration.cpp)
    // sources it from Config::Robot::effectiveTrackWidth() instead, the
    // SAME derived value Drive/Odometry/PlannerLimits already use.
    msg::Navigator cfg;
    cfg.speed = {_f(navigator["speed"])};                              // [mm/s]
    cfg.max_wheel_step = {_f(navigator["maxWheelStep"])};              // [mm/s]
    cfg.behind_angle = {_f(navigator["behindAngle"])};                 // [rad]
    cfg.turn_first_angle = {_f(navigator["turnFirstAngle"])};          // [rad]
    cfg.approach_radius = {_f(navigator["approachRadius"])};           // [mm]
    cfg.approach_speed = {_f(navigator["approachSpeed"])};             // [mm/s]
    cfg.default_arrival_tolerance = {_f(navigator["defaultArrivalTolerance"])};  // [mm]
    cfg.yaw_sign = {_f(navigator["yawSign"])};
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
