#!/usr/bin/env python3
"""ONE-TIME migration script: reshape a robot JSON from the OLD 13-section
layout (``identity``/``connection``/``vision``/``geometry``/``wheels``/
``encoders``/``drive``/``gripper``/``peripherals``/``calibration``/
``control``/``estimator``/``planner``, with ``control``/``calibration`` as
flat 53+12-field dumping grounds feeding five firmware destinations) into
``Config::Robot``'s END-STATE consumer-grouped shape (sprint 132 "the
configuration object": ``robot_config.proto``, tickets 001-016) --
``identity``/``connection``/``vision`` (host-only, unchanged) plus
``geometry``/``motors``/``drive``/``wheel_control``/``planner``/
``planner_shaper``/``otos``/``estimator`` (robot-config, one
``ConfigGroupTarget``/wire message/``configure()`` consumer each -- see
robot_config.proto's own header comment), plus the non-wire-addressable
sections that stay in the file under the one-file rule but never cross the
wire (``wheels``/``encoders``/``gripper``/``peripherals``, unchanged).

RUN ONCE, 2026-08-04, sprint 132 ticket 017 (JSON reshape), against
``data/robots/tovez.json``/``togov.json``/``tovez_nocal.json`` -- committed
in the new shape. Do NOT re-run this script against an already-migrated
file: it assumes the OLD shape as input (the old ``control``/``calibration``
top-level sections, the OLD ``drive`` section's confusingly-named
motor-limit keys, the old singular ``planner`` section carrying all 16
fields including the six now-split shaper ceilings) and will raise
``KeyError``/produce nonsense on a file already in the new shape. This is
NOT part of the ongoing build pipeline -- ``gen_boot_config.py`` is what
runs on every build, reading the NEW shape this script produces; this
script itself is a migration tool, run by hand, once, and kept here only
as a record of exactly how the reshape was done (and in case a FOURTH
robot JSON is ever added from an old-shape source -- unlikely, but cheaper
to keep than to reconstruct from git history).

Design decisions this script makes (robot_config.proto's own Open
Question 3 leaves "exact migration-script mechanics ... how `_note`
comment-keys are handled" to this ticket's own implementation call):

1. **In-place rewrite**, not new-file-then-swap -- the three files are
   already git-tracked; the diff IS the record, so a copy-then-delete
   dance adds no safety a `git diff` doesn't already give for free.

2. **Underscore-prefixed keys (``_foo_note``/``_foo``) are documentation
   or non-schema recorded data, never live/wire data** -- an established
   project convention PRE-DATING this ticket (e.g. tovez.json's own
   ``_otos_linear_scale_note``, and ``control._vel_gains_domain``, a
   non-note informational string that already used the underscore
   convention for "not consumed by any `_require()`/`_get()` call" before
   this ticket existed). ``robot_config.py``'s ``RobotConfig`` model
   (132-017, same ticket) gained a ``model_validator(mode="before")`` that
   strips every underscore-prefixed key, recursively, before pydantic's
   ``extra="forbid"`` (132-016) ever sees it -- so an underscore-prefixed
   key survives in the committed FILE (physically relocated alongside the
   field it documents, per this ticket's acceptance criteria) without
   needing its own schema field. This script leans on that: a real value
   with no schema home (see class 4 below) is relocated with an added
   underscore prefix rather than dropped, UNLESS it is one of the 23
   explicitly-authorized-for-deletion dead ``control`` keys (class 3).

3. **The 17-originally-audited, freshly-re-verified-as-23 dead ``control``
   keys are DROPPED, not underscore-prefixed** -- explicit ticket
   authorization ("Drop the 17 dead `control` keys ... re-verify the list
   is still accurate"). Re-verifying against gen_boot_config.py's ACTUAL
   `_require()`/`_get()` call sites (not the original issue's stale count)
   found SIX MORE keys went dead since the issue's own audit: 132-015 (this
   same sprint, dead-code sweep) deleted `shaper_config_for_config()` --
   the ONE remaining reader of `control.a_max`/`a_decel`/`alpha_max`/
   `alpha_decel`/`j_max`/`yaw_jerk_max` (NOT to be confused with the LIVE,
   differently-cased `planner.a_max`/etc. this same ticket splits into
   `PlannerShaper`, below) -- bringing the total from 17 to 23. See
   ``DEAD_CONTROL_KEYS`` below for the full, freshly-verified list and
   ``report_dead_keys()`` for the per-file count this ticket's completion
   report cites.

4. **Real values with no schema field are relocated with an underscore
   prefix, not dropped** -- distinct from class 3 (which the ticket
   explicitly authorizes dropping): ``calibration.ekf_r_otos_theta`` (an
   EKF observation-noise tuning value, never wired to anything since the
   DRIVETRAIN patch target that would have carried it was always
   ``ERR_UNIMPLEMENTED``, but not one of the audited-and-authorized-dead
   `control` keys either), ``planner.plant_gain``/``plant_tau`` (recorded
   plant-ID measurements, explicitly "kept as recorded measured data" per
   their own JSON comment, no longer read by any `_require()` call since
   130-009), the OLD top-level ``drive`` section's own motor-limit keys
   (``motor_deadband``/``max_cmd``/... -- read by NO firmware code per
   robot_config.proto's own header audit, but ``tovez_nocal.json``
   carries REAL non-null measured values here with their own derivation
   note, not merely null placeholders -- irreplaceable the same way the
   calibration notes are), and ``geometry.drive_axle_offset_mm``/
   ``odometry_chip_upside_down``/``wheelbase_mm`` (dead per
   robot_config.proto's own header audit: "none is read by any
   `_require()`/`_get()` call in gen_boot_config.py today"). None of
   these is silently vaporized.

5. **The OLD top-level ``drive`` section is renamed to
   ``_legacy_drive_limits``** (top-level, underscore-prefixed) rather than
   staying named ``drive`` -- the NEW consumer-grouped ``drive`` section
   is Config::Robot's own live Drive group (duty_per_speed/wheel
   correction), an unrelated meaning that would otherwise collide with
   this section's own key.

6. **togov.json's ``mecanum_geometry``/``mecanum_calibration`` sections**
   (real per-wheel mecanum calibration data, e.g. ``fwd_sign_fr``/
   ``half_track_mm``) have no home in this sprint's schema at all --
   mecanum wire-config is out of this sprint's scope (differential-only
   ``Motors``/``Geometry`` groups; see sprint.md's Out of Scope section
   for the analogous ColorConfig/LineConfig call). Relocated to
   underscore-prefixed TOP-LEVEL keys (``_mecanum_geometry``/
   ``_mecanum_calibration``) rather than dropped -- real calibration data,
   just outside this sprint's schema.

7. **``vision.tag_offset_mm`` (nested {x,y,z,yaw_rad}) is flattened** into
   ``vision.tag_offset_x``/``tag_offset_y``/``tag_offset_z``/
   ``tag_offset_yaw`` -- matching ``robot_config.proto``'s own ``Vision``
   message shape (ticket 001 authored the schema in this flat form from
   the start; ``robot_config.py``'s own header comment already documents
   this exact flattening decision for ticket 020's shape). A missing
   ``z`` key (togov.json's ``tag_offset_mm`` never had one) is simply
   omitted -- the generated ``Vision.tag_offset_z`` field defaults to
   ``0.0``, matching what every reader of a MISSING dict key already got
   before this reshape.

8. **The ``planner`` JSON section is SPLIT into TWO top-level JSON keys**,
   ``planner`` (10 boot-only fields) and ``planner_shaper`` (6 live
   shaper-ceiling fields) -- this is NOT optional the way the other group
   boundaries are a pure firmware/generator-side wiring concern: pydantic
   ``RobotConfig`` (132-017, same ticket) declares ``planner: Planner``
   and ``planner_shaper: PlannerShaper`` as two SEPARATE top-level fields,
   each ``extra="forbid"`` -- a single JSON ``planner`` section carrying
   all 16 keys would fail validation the moment pydantic tried to
   construct ``Planner`` (which only declares 10 fields) with the six
   shaper-ceiling keys still mixed in. See ``robot_config.proto``'s own
   ``PlannerShaper`` message header comment for why the split exists at
   all (stakeholder-sanctioned mid-sprint scope addition, closing a
   measured regression: the six shaper ceilings had their own live wire
   arm before sprint 132, and the ORIGINAL single-``Planner``-group design
   silently demoted them to boot-only). The ``planner`` section's own
   ``_domain_note``/``_timing_note``/``_settle_note``/``_uncalibrated``
   documentation keys are left where they are (under ``planner``, not
   duplicated into ``planner_shaper``) -- they are pure prose, stripped
   from validation regardless of which section holds them, and splitting
   them too would only risk the two copies drifting apart.
"""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
ROBOTS_DIR = REPO_ROOT / "data" / "robots"

# ---------------------------------------------------------------------------
# Class 3 -- the dead `control.*` keys this ticket is explicitly authorized
# to DROP (re-verified against gen_boot_config.py's ACTUAL `_require()`/
# `_get()` call sites on 2026-08-04, not assumed from the issue's original
# "17" count -- see this module's own docstring, decision 3, for why the
# fresh count is 23, not 17: 132-015's dead-code sweep (earlier this same
# sprint) deleted the one remaining reader of the first seven keys below).
# ---------------------------------------------------------------------------
DEAD_CONTROL_KEYS: tuple[str, ...] = (
    # Former shaper_config_for_config() readers -- consumer deleted 132-015.
    "a_max", "a_decel", "alpha_max", "alpha_decel", "v_body_max", "j_max",
    "yaw_jerk_max",
    # Motion-stack-v2 (sprint 100-001) leftovers -- no living consumer
    # since 115-003's motion-stack excision; present only in tovez.json.
    "v_wheel_max", "steer_headroom", "wheel_step_max",
    "track_k_s", "track_k_theta", "track_k_cross",
    "trim_v_max", "trim_omega_max",
    "replan_err_pos", "replan_err_theta", "replan_hold", "replan_min_period",
    "replan_max",
    "handoff_tol_pos", "handoff_tol_v",
    "arrive_vel_tol",
)
assert len(DEAD_CONTROL_KEYS) == 23, "re-count drifted from this docstring's own tally"


def _is_note_key(key: str) -> bool:
    return key.startswith("_")


def _pop_if_present(src: dict, key: str, dst: dict, dst_key: "str | None" = None) -> None:
    """Move (POP) ``src[key]`` to ``dst[dst_key or key]`` iff present and
    not ``None`` -- mirrors gen_boot_config.py's own soft `_get()`
    presence semantics (an absent/null source key stays absent in the
    reshaped file too, so `_get()`'s existing placeholder-fallback
    behavior is unchanged by this reshape, not silently replaced by a
    baked-in placeholder value). Pops rather than merely copies so this
    module's own end-of-function completeness assertions (every
    `control`/`calibration`/old-`geometry`/old-`planner` key claimed by
    something, nothing silently left behind) are meaningful."""
    if key not in src:
        return
    value = src.pop(key)  # pop regardless of None-ness -- see doc comment
    if value is not None:
        dst[dst_key or key] = value


def _require(src: dict, key: str) -> object:
    """Like ``_pop_if_present``, but raises if *key* is absent or
    ``None`` -- mirrors gen_boot_config.py's own fail-closed `_require()`.
    Pops (see ``_pop_if_present``'s own doc comment for why)."""
    if key not in src or src[key] is None:
        raise KeyError(f"required key {key!r} missing from {src!r}")
    return src.pop(key)


def _move_notes(src: dict, dst: dict, keys: "tuple[str, ...]") -> None:
    """Move exactly *keys* (a note-key allowlist) from *src* to *dst*, iff
    present -- NOT "every underscore key in src", which would duplicate a
    single `control`-section note into every one of the three new groups
    (`motors`/`drive`/`wheel_control`) that all read from the same OLD
    `control` dict. Each note is claimed by exactly ONE destination, the
    group whose fields it actually documents -- see reshape()'s own
    per-group call sites for the explicit key lists this ticket's author
    derived from each note's own text."""
    for key in keys:
        if key in src:
            dst[key] = src.pop(key)


def reshape(old: dict) -> dict:
    """Return a NEW dict in Config::Robot's grouped shape, built from
    *old* (the 13-section shape). Pure function -- does not mutate *old*."""
    old = copy.deepcopy(old)
    new: dict = {}

    new["schema_version"] = old.get("schema_version", 2)

    # -- Identity: unchanged verbatim -----------------------------------
    new["identity"] = dict(old.get("identity", {}))

    # -- Connection: unchanged, except i2c_addresses has no schema field
    # (class 4: always {} in every robot JSON on disk today -- preserved,
    # not dropped, on the same "don't silently vaporize a real key"
    # discipline every other class-4 field gets, even though it happens
    # to carry no information in practice).
    conn_old = old.get("connection", {})
    conn_new = {k: v for k, v in conn_old.items() if k != "i2c_addresses" and not _is_note_key(k)}
    _pop_if_present(conn_old, "i2c_addresses", conn_new, "_i2c_addresses")
    _move_notes(conn_old, conn_new, tuple(k for k in conn_old if _is_note_key(k)))
    new["connection"] = conn_new

    # -- Vision: flatten tag_offset_mm.{x,y,z,yaw_rad} (decision 7) ------
    vis_old = old.get("vision", {})
    vis_new = {k: v for k, v in vis_old.items() if k not in ("tag_offset_mm",) and not _is_note_key(k)}
    tag_offset = vis_old.get("tag_offset_mm") or {}
    _pop_if_present(tag_offset, "x", vis_new, "tag_offset_x")
    _pop_if_present(tag_offset, "y", vis_new, "tag_offset_y")
    _pop_if_present(tag_offset, "z", vis_new, "tag_offset_z")
    _pop_if_present(tag_offset, "yaw_rad", vis_new, "tag_offset_yaw")
    _move_notes(vis_old, vis_new, tuple(k for k in vis_old if _is_note_key(k)))
    new["vision"] = vis_new

    # -- Geometry: trackwidth/otos_untrusted stay put; rotation_*/
    # rotational_slip move in from `calibration`; drive_axle_offset_mm/
    # odometry_chip_upside_down/wheelbase_mm are class-4 dead (preserved,
    # underscore-prefixed) -- robot_config.proto's own header audit.
    geo_old = old.get("geometry", {})
    cal_old = old.get("calibration", {})
    geo_new: dict = {}
    _pop_if_present(geo_old, "trackwidth", geo_new)
    _pop_if_present(geo_old, "otos_untrusted", geo_new)
    _pop_if_present(cal_old, "rotational_slip", geo_new, "rotational_slip")
    _pop_if_present(cal_old, "rotation_gain", geo_new, "rotation_gain_pos")
    _pop_if_present(cal_old, "rotation_offset_deg", geo_new, "rotation_offset")
    _pop_if_present(cal_old, "rotation_gain_neg", geo_new, "rotation_gain_neg")
    _pop_if_present(cal_old, "rotation_offset_deg_neg", geo_new, "rotation_offset_neg")
    _pop_if_present(geo_old, "drive_axle_offset_mm", geo_new, "_drive_axle_offset_mm")
    _pop_if_present(geo_old, "odometry_chip_upside_down", geo_new, "_odometry_chip_upside_down")
    _pop_if_present(geo_old, "wheelbase_mm", geo_new, "_wheelbase_mm")
    # geo_old's own notes (e.g. _trackwidth_note) belong here entirely;
    # cal_old's rotation/slip notes move in alongside the fields they
    # document (both now live under geometry, above).
    _move_notes(geo_old, geo_new, tuple(k for k in geo_old if _is_note_key(k)))
    _move_notes(cal_old, geo_new, ("_rotational_slip_note", "_rotation_calibration_note"))
    new["geometry"] = geo_new

    # -- Motors: travel_calib_*/fwd_sign_* are SOFT (class: only present
    # when the source JSON supplies them -- `_get()`'s own placeholder
    # fallback keeps working unmodified on an omitted key, so an absent
    # source key stays absent here too, per _pop_if_present()'s own
    # contract); output_deadband/reversal_dwell/vel_* are REQUIRED
    # (present in all three robot JSONs today).
    ctrl_old = old.get("control", {})
    mot_new: dict = {}
    _pop_if_present(cal_old, "mm_per_wheel_deg_left", mot_new, "travel_calib_left")
    _pop_if_present(cal_old, "mm_per_wheel_deg_right", mot_new, "travel_calib_right")
    _pop_if_present(cal_old, "fwd_sign_left", mot_new, "fwd_sign_left")
    _pop_if_present(cal_old, "fwd_sign_right", mot_new, "fwd_sign_right")
    mot_new["output_deadband"] = _require(ctrl_old, "output_deadband")
    mot_new["reversal_dwell"] = _require(ctrl_old, "reversal_dwell_ms")
    mot_new["vel_kp"] = _require(ctrl_old, "vel_kp")
    mot_new["vel_ki"] = _require(ctrl_old, "vel_ki")
    mot_new["vel_kff"] = _require(ctrl_old, "vel_kff")
    mot_new["vel_i_max"] = _require(ctrl_old, "vel_imax")
    mot_new["vel_kaw"] = _require(ctrl_old, "vel_kaw")
    mot_new["vel_filt_alpha"] = _require(ctrl_old, "vel_filt")
    # Notes documenting the vel_*/output_deadband/reversal_dwell fields
    # above -- NOT the whole `control` section (see _move_notes()'s own
    # doc comment for why this is an explicit allowlist, not "every
    # underscore key in ctrl_old").
    _move_notes(ctrl_old, mot_new, (
        "_vel_gains_domain", "_vel_gains_note", "_vel_kff_note",
        "_vel_ki_note", "_vel_filt_note", "_neutral_note",
    ))
    new["motors"] = mot_new

    # -- Drive: duty_per_speed_*/crawl_pulse + the 8 Stage-A correction
    # fields, all REQUIRED.
    drv_new: dict = {}
    drv_new["duty_per_speed_left"] = _require(ctrl_old, "duty_per_speed_left")
    drv_new["duty_per_speed_right"] = _require(ctrl_old, "duty_per_speed_right")
    drv_new["crawl_pulse"] = _require(ctrl_old, "crawl_pulse")
    for wheel in ("left", "right"):
        for direction in ("accel", "decel"):
            gain_key = f"wheel_gain_{wheel}_{direction}"
            icpt_key = f"wheel_intercept_{wheel}_{direction}"
            drv_new[gain_key] = _require(ctrl_old, gain_key)
            drv_new[icpt_key] = _require(ctrl_old, icpt_key)
    # _sprint_114_note/_shaper_note/_drive_limits_note are historical
    # (they document the now-DROPPED dead control keys, class 3 -- see
    # this module's own docstring decision 3) with no single surviving
    # field to attach to; kept here as the closest surviving "this used
    # to be part of control" home, not silently discarded.
    _move_notes(ctrl_old, drv_new, (
        "_sprint_114_note", "_shaper_note", "_drive_limits_note",
        "_duty_per_speed_correction_note", "_drive_calibration_note",
        "_wheel_correction_note",
    ))
    new["drive"] = drv_new

    # -- WheelControl: the 11 Stage B/C fields, all REQUIRED.
    wc_new: dict = {}
    wc_new["v_min"] = _require(ctrl_old, "wheel_v_min")
    wc_new["bias_max"] = _require(ctrl_old, "wheel_bias_max")
    wc_new["tau_adapt"] = _require(ctrl_old, "wheel_tau_adapt")
    wc_new["a_steady"] = _require(ctrl_old, "wheel_a_steady")
    wc_new["deficit_threshold"] = _require(ctrl_old, "wheel_deficit_threshold")
    wc_new["deficit_window"] = _require(ctrl_old, "wheel_deficit_window_ms")
    wc_new["pid_kp"] = _require(ctrl_old, "wheel_pid_kp")
    wc_new["pid_ki"] = _require(ctrl_old, "wheel_pid_ki")
    wc_new["pid_i_max"] = _require(ctrl_old, "wheel_pid_i_max")
    wc_new["pid_kaff"] = _require(ctrl_old, "wheel_pid_kaff")
    wc_new["pid_max"] = _require(ctrl_old, "wheel_pid_max")
    _move_notes(ctrl_old, wc_new, ("_wheel_controller_note",))
    new["wheel_control"] = wc_new

    # -- Drop the 23 dead `control` keys (class 3, explicit ticket
    # authorization) -- everything ELSE in `control` has now been claimed
    # (a real field moved to motors/drive/wheel_control, or a note moved
    # to motors/drive/wheel_control above). Assert nothing unaccounted-for
    # is left behind silently -- a leftover key here means this script's
    # own mapping missed something, not that the leftover is safe to drop.
    for key in DEAD_CONTROL_KEYS:
        ctrl_old.pop(key, None)
    leftover = {k: v for k, v in ctrl_old.items() if not _is_note_key(k)}
    assert not leftover, (
        f"reshape(): unaccounted-for real control.* key(s) left behind: "
        f"{leftover!r} -- add a mapping or a DEAD_CONTROL_KEYS entry")
    leftover_notes = list(ctrl_old.keys())
    assert not leftover_notes, (
        f"reshape(): unaccounted-for control.* note key(s): {leftover_notes!r} "
        f"-- add a _move_notes() destination")

    # -- Planner (boot-only, 10 fields) / PlannerShaper (live, 6 fields) --
    # decision 8: SPLIT into two top-level JSON keys, same source section.
    pl_old = old.get("planner", {})
    pl_new: dict = {}
    for key in ("v_max", "omega_max", "control_period", "actuation_delay",
                "settle_rest_velocity", "settle_rest_omega",
                "settle_epsilon_linear", "settle_epsilon_angular",
                "heading_hold_gain", "decel_plan_fraction"):
        pl_new[key] = _require(pl_old, key)
    _pop_if_present(pl_old, "plant_gain", pl_new, "_plant_gain")
    _pop_if_present(pl_old, "plant_tau", pl_new, "_plant_tau")
    _pop_if_present(pl_old, "_uncalibrated", pl_new, "_uncalibrated")
    # Every planner.* note (_domain_note/_timing_note/_settle_note) stays
    # under `planner`, not split/duplicated into `planner_shaper` too --
    # see this module's own docstring decision 8.
    _move_notes(pl_old, pl_new, tuple(k for k in pl_old if _is_note_key(k)))
    new["planner"] = pl_new

    ps_new: dict = {}
    for key in ("a_max", "a_decel", "alpha_max", "alpha_decel", "jerk_max",
                "yaw_jerk_max"):
        ps_new[key] = _require(pl_old, key)
    new["planner_shaper"] = ps_new
    assert not pl_old, f"reshape(): unaccounted-for planner.* key(s): {pl_old!r}"

    # -- Otos: offset_x/y/yaw from geometry.odometry_offset_mm, scales
    # from calibration.otos_linear_scale/otos_angular_scale -- all
    # REQUIRED, grouped under Otos per robot_config.proto's own explicit
    # mapping (not Geometry, even though offset_* lived under `geometry`
    # in the old shape).
    odom = geo_old.get("odometry_offset_mm") or {}
    otos_new: dict = {}
    otos_new["offset_x"] = _require(odom, "x")
    otos_new["offset_y"] = _require(odom, "y")
    otos_new["offset_yaw"] = _require(odom, "yaw_rad")
    otos_new["linear_scale"] = _require(cal_old, "otos_linear_scale")
    otos_new["angular_scale"] = _require(cal_old, "otos_angular_scale")
    _pop_if_present(cal_old, "ekf_r_otos_theta", otos_new, "_ekf_r_otos_theta")
    _move_notes(cal_old, otos_new, ("_otos_linear_scale_note",))
    new["otos"] = otos_new

    # cal_old (`calibration`) has now been fully claimed across geometry/
    # motors/otos, above -- confirm nothing was missed.
    assert not cal_old, f"reshape(): unaccounted-for calibration.* key(s): {cal_old!r}"
    # geo_old (`geometry`) similarly, across geometry/otos (odometry_offset_mm
    # is claimed piecemeal via the local `odom` dict, which is the SAME
    # dict object geo_old["odometry_offset_mm"] points at).
    geo_old.pop("odometry_offset_mm", None)
    assert not geo_old, f"reshape(): unaccounted-for geometry.* key(s): {geo_old!r}"

    # -- Estimator: unchanged location, staleness_ms -> staleness rename.
    est_old = old.get("estimator", {})
    est_new: dict = {}
    est_new["weight_heading_otos"] = _require(est_old, "weight_heading_otos")
    est_new["weight_omega_otos"] = _require(est_old, "weight_omega_otos")
    est_new["staleness"] = _require(est_old, "staleness_ms")
    _move_notes(est_old, est_new, tuple(k for k in est_old if _is_note_key(k)))
    new["estimator"] = est_new
    assert not est_old, f"reshape(): unaccounted-for estimator.* key(s): {est_old!r}"

    # -- Non-wire-addressable sections: unchanged verbatim (one-file rule).
    for section in ("wheels", "encoders", "gripper", "peripherals"):
        if section in old:
            new[section] = old[section]

    # -- Class 5: the OLD `drive` section (motor-limit keys, unrelated to
    # the NEW consumer-grouped `drive` group above) -- renamed to a
    # top-level underscore-prefixed key so it never collides with the new
    # `drive` section's own meaning.
    if "drive" in old:
        new["_legacy_drive_limits"] = old["drive"]

    # -- Class 6: togov.json's mecanum-only sections -- no schema home
    # this sprint (mecanum wire-config out of scope); preserved.
    for section in ("mecanum_geometry", "mecanum_calibration"):
        if section in old:
            new[f"_{section}"] = old[section]

    return new


# ---------------------------------------------------------------------------
# Verification: prove every REAL (non-note, non-dropped) value survived
# bit-exact, by walking both the old and new dict and cross-checking the
# migration's own field map -- NOT a diff-the-whole-tree check, because the
# shape genuinely changes (that is the point of the reshape); this instead
# re-derives, from the SAME mapping `reshape()` implements, what the new
# value at each new path should equal, and compares against what a fresh
# `reshape()` run actually produced.
# ---------------------------------------------------------------------------

def _flatten_non_notes(d: dict, prefix: str = "") -> "dict[str, object]":
    """Flatten a (possibly nested) dict into {dotted_path: value}, skipping
    every underscore-prefixed key at every level (recursively) -- used to
    count/compare only the REAL, schema-relevant values a migration must
    preserve bit-exact, not the documentation this script relocates by a
    separate, deliberate mechanism (see _copy_notes())."""
    out: dict[str, object] = {}
    for k, v in d.items():
        if _is_note_key(k):
            continue
        path = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            out.update(_flatten_non_notes(v, path))
        else:
            out[path] = v
    return out


def report_dead_keys(old: dict) -> "list[str]":
    """Return which of DEAD_CONTROL_KEYS are actually present in *old*'s
    `control` section -- the freshly-re-verified count this ticket's own
    completion report cites, per robot file (not every robot JSON carries
    all 23 -- see this module's own docstring, decision 3)."""
    ctrl = old.get("control", {})
    return [k for k in DEAD_CONTROL_KEYS if k in ctrl]


def migrate_file(path: Path) -> None:
    old = json.loads(path.read_text())
    new = reshape(old)
    path.write_text(json.dumps(new, indent=2, sort_keys=False) + "\n")


def main(argv: "list[str] | None" = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print(
            "usage: migrate_robot_json_to_grouped_shape.py <robot.json> [<robot.json> ...]\n"
            "  One-time migration (132-017) -- do NOT re-run against an "
            "already-migrated file.",
            file=sys.stderr,
        )
        return 1
    for arg in argv:
        path = Path(arg)
        if not path.is_absolute():
            path = REPO_ROOT / path
        dead = report_dead_keys(json.loads(path.read_text()))
        migrate_file(path)
        print(f"migrate_robot_json_to_grouped_shape: reshaped {path} "
              f"(dropped {len(dead)} dead control keys: {sorted(dead)})",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
