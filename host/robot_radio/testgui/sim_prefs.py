"""robot_radio.testgui.sim_prefs — Sim-mode injected error profile + persistence.

Qt-free. Backs the "Sim Errors" GUI panel (``__main__.py``) and
``SimTransport`` (``transport.py``): lets the operator configure the noise
the simulator injects into encoders and the OTOS, instead of hardcoded
module constants.

Persistence
-----------
The profile is persisted to ``data/testgui/sim_error_profile.json``, mirroring
the ``camera_prefs.py`` convention (see that module's docstring for the
``_PROJECT_ROOT`` four-``.parent`` chain rationale — this module sits at the
same ``host/robot_radio/testgui/`` depth).

Keys
----
083-001: the sprint-069 ``SIMSET`` wire protocol this module used to target
no longer exists — ``source/commands/`` has no ``SIMSET`` verb (confirmed by
reading every ``makeCmd``/``makeSchemaCmd`` registration; the current verb
set is ``PING``/``VER``/``HELP``/``ECHO``/``ID``/``STREAM``/``SNAP``/
``DEV M``/``DEV DT``/``DEV STATE``/``DEV STOP``/``DEV WD``).  Every key below
is now applied directly through a ctypes sim-connection setter (108-006:
``robot_radio.io.sim_loop.SimLoop`` -- see that module's own docstring for
the reconciliation from the deleted predecessor this comment originally
described), either via ``PROFILE_TO_SIM_SETTER`` (the 1:1-mapped keys) or
via a small number of keys ``transport.py``'s ``_apply_profile_to_sim()``
special-cases (documented under each key below and in that map's own
docstring).

``encoder_noise``
    Per-side encoder noise sigma, in millimetres. Default ``0.0``. Applied to
    both sides in ONE call, ``SimConnection.set_enc_noise(2, value)`` (side=2
    = both) — excluded from ``PROFILE_TO_SIM_SETTER`` (fans out specially).
``slip_turn_extra``
    Fractional encoder over-report during turns (turn-slip scrub model).
    Default ``0.0`` (ticket 073-003 — previously ``0.26``). **No ctypes ABI
    entry point backs this knob at all** in the sprint-081/082 ABI (no
    turn-rate-dependent slip knob is wired — see
    ``SimConnection.set_slip()``'s own docstring) — excluded from
    ``PROFILE_TO_SIM_SETTER``; ``_apply_profile_to_sim()`` skips applying it
    and logs a one-time ``[WARN]`` if it is set away from its neutral ``0.0``.
    See ``resolve_calibration_defaults()`` for the reconciliation this
    default is (historically) part of.
``otos_linear_noise``
    OTOS linear-position noise sigma, as a fraction of arc. Default ``0.05``.
    ``PROFILE_TO_SIM_SETTER`` key: ``set_otos_linear_noise``.
``otos_yaw_noise``
    OTOS yaw noise sigma, as a fraction. Default ``0.0``.
    ``PROFILE_TO_SIM_SETTER`` key: ``set_otos_yaw_noise``.

Additive/noise terms — ``0.0`` is a genuine no-op:

``enc_scale_err_l`` / ``enc_scale_err_r``
    Fractional per-side encoder over/under-report (0 = perfect). Each needs
    an explicit ``side`` argument (``SimConnection.set_enc_scale_error(0/1,
    value)``) — excluded from ``PROFILE_TO_SIM_SETTER`` (same reason as
    ``encoder_noise``) and applied by two explicit calls in
    ``_apply_profile_to_sim()``.
``otos_lin_scale_err`` / ``otos_ang_scale_err``
    Fractional OTOS linear/angular scale error (0 = perfect).
    ``PROFILE_TO_SIM_SETTER`` keys: ``set_otos_linear_scale_error`` /
    ``set_otos_angular_scale_error``.
``otos_lin_drift`` / ``otos_yaw_drift``
    OTOS linear/yaw drift, applied as a constant ADDITIVE term once per
    ``Hal::SimOdometer`` tick (``setLinearDriftPerTick()``/
    ``setYawDriftPerTick()`` — ``source/hal/sim/sim_odometer.{h,cpp}``) — NOT
    a one-shot bias and NOT a per-SECOND rate.  (The retired ``SIMSET`` wire
    keys ``otosLinDriftMmS``/``otosYawDriftDegS`` were a mm/s, deg/s rate,
    with ``SimCommands.cpp`` converting to per-tick internally; that
    conversion layer no longer exists, so this module and the GUI now deal
    in the SAME per-tick unit the ctypes setter takes directly — confirmed
    by reading ``physics_world.h``/``sim_odometer.h`` and
    ``sim_odometer.cpp``'s ``tick()``, which adds
    ``linearDriftPerTick_``/``yawDriftPerTick_`` to the accumulator exactly
    once per call, unconditional on elapsed time.)  ``otos_lin_drift`` is in
    millimetres PER TICK; ``otos_yaw_drift`` is in RADIANS per tick (not
    degrees — ``sim_set_otos_yaw_drift`` takes radians directly and there is
    no host-side unit conversion). ``PROFILE_TO_SIM_SETTER`` keys:
    ``set_otos_linear_drift`` / ``set_otos_yaw_drift``.

Multiplicative terms — ``1.0`` is the genuine no-op, NOT ``0.0`` (see
``PhysicsWorld``'s ``_bodyRotationalScrub``/``_bodyLinearScrub``/
``_offsetFactorL``/``_offsetFactorR`` field defaults, all ``1.0f``):

``body_rot_scrub`` / ``body_lin_scrub``
    Body-truth rotational/linear scrub factor, clamped to ``(0, 1]`` on the
    firmware side. ``PROFILE_TO_SIM_SETTER`` keys:
    ``set_body_rotational_scrub`` / ``set_body_linear_scrub``.
    ``DEFAULT_PROFILE["body_rot_scrub"]`` itself stays the neutral ``1.0``
    (a bare, no-calibration-lookup profile dict must remain a genuine
    no-op) — but ``load_sim_error_profile()``'s FALLBACK path (no persisted
    file, or a persisted file missing this key) resolves it from the active
    robot's calibration via ``resolve_calibration_defaults()`` instead,
    ticket 073-003.
``motor_offset_l`` / ``motor_offset_r``
    Per-side motor actuation offset factor (multiplies commanded velocity).
    **No ctypes ABI entry point backs this knob at all**
    (``Hal::PhysicsWorld::setOffsetFactor()`` is deliberately left unwrapped
    by ticket 081-004's ``sim_api.cpp`` — see
    ``SimConnection.set_motor_offset()``'s own docstring) — excluded from
    ``PROFILE_TO_SIM_SETTER``; ``_apply_profile_to_sim()`` skips applying
    either and logs a one-time ``[WARN]`` if either is set away from its
    neutral ``1.0``.

``trackwidth``
    The plant's trackwidth, in millimetres. Has NO safe zero default —
    ``PhysicsWorld::update()``'s sub-step B divides by it. Defaults to
    ``128.0``, matching ``PhysicsWorld::kDefaultTrackwidth`` (``source/hal/
    sim/physics_world.h`` — fixed from a stale 150.0 to the project's real
    128.0 during the 097-OOP wheelbase-consistency investigation: that
    constant is ALSO what ``tests/_infra/sim/sim_api.cpp``'s
    ``defaultSimDrivetrainConfig()`` seeds the firmware's OWN kinematics
    trackwidth from, so it is the sim's single point of truth for both
    sides). This keeps the plant's geometry matched to the firmware's
    kinematic calibration (a mismatch makes every encoder-arc turn land
    off-angle by the ratio) even for a caller that applies THIS knob
    without also pushing an equivalent ``SET tw=`` to the firmware (the
    TestGUI's own Connect flow already does that separately, via
    ``__main__.py``'s ``_push_robot_calibration()`` — this default is
    belt-and-suspenders for callers below that layer). This is NOT a
    sentinel meaning "don't touch" — every Apply unconditionally calls
    ``SimConnection.set_trackwidth(value)`` and overwrites the plant's
    trackwidth. ``PROFILE_TO_SIM_SETTER`` key: ``set_trackwidth``.
"""
from __future__ import annotations

import json
import logging
from collections.abc import Callable
from pathlib import Path

_log = logging.getLogger(__name__)

# host/robot_radio/testgui/sim_prefs.py -> repo root (same depth as
# host/robot_radio/testgui/camera_prefs.py's _PROJECT_ROOT).
_PROJECT_ROOT = Path(__file__).parent.parent.parent.parent
_PREFS_DIR = _PROJECT_ROOT / "data" / "testgui"
_PREFS_PATH = _PREFS_DIR / "sim_error_profile.json"

#: Default injected-error profile. ``slip_turn_extra``'s default was changed
#: by ticket 073-003 from the historical ``0.26`` to ``0.0``: the
#: encoder-report-only 0.26 combined with a neutral ``body_rot_scrub`` (see
#: below) under-rotated turns net ~14% out of the box; reconciling
#: body_rot_scrub against the active robot's real calibration (see
#: ``resolve_calibration_defaults()`` and ``load_sim_error_profile()``'s
#: fallback path) is now the factory default instead of a manual opt-in via
#: the "From Calibration" button. (083-001: ``slip_turn_extra`` additionally
#: has no ctypes ABI backing at all now — see the module docstring's "Keys"
#: section — so its default is moot for sim behavior either way; it is kept
#: only so a persisted profile file with this key still loads.)
#:
#: Every default below reproduces the historical no-op-until-opted-in
#: behavior: additive/noise terms default 0.0, multiplicative terms default
#: 1.0, and trackwidth defaults to the plant's real compiled-in trackwidth
#: (128.0mm) rather than an unsafe 0.0. See the module docstring's "Keys"
#: section for units/semantics/no-op rationale per key.
DEFAULT_PROFILE: dict = {
    # -- historical four --
    "encoder_noise": 0.0,
    # 073-003: was 0.26 -- see the module-level comment above this dict.
    "slip_turn_extra": 0.0,
    "otos_linear_noise": 0.05,
    "otos_yaw_noise": 0.0,
    # -- additive/noise terms (0.0 = no-op) --
    "enc_scale_err_l": 0.0,
    "enc_scale_err_r": 0.0,
    "otos_lin_scale_err": 0.0,
    "otos_ang_scale_err": 0.0,
    "otos_lin_drift": 0.0,
    "otos_yaw_drift": 0.0,
    # -- multiplicative terms (1.0 = no-op, NOT 0.0) --
    "body_rot_scrub": 1.0,
    "body_lin_scrub": 1.0,
    "motor_offset_l": 1.0,
    "motor_offset_r": 1.0,
    # -- no safe zero default; defaults to the firmware config's
    # trackwidthMm (DefaultConfig.cpp: 128.0 — what the sim seeds the plant
    # with at construction) so Apply-at-default is a genuine no-op rather
    # than a divide-by-zero sentinel. NOT kDefaultTrackwidthMm (150.0): the
    # plant never actually runs at that value, and applying it would inject
    # a plant-vs-calibration geometry error into every turn.
    "trackwidth": 128.0,
}

#: Maps every profile key that has a 1:1 ``SimConnection`` setter equivalent
#: to that setter's method name (e.g. ``"body_rot_scrub":
#: "set_body_rotational_scrub"``), called with the single profile value as
#: its only argument (``transport.py``'s ``_apply_profile_to_sim()``).
#:
#: Deliberately excludes five ``DEFAULT_PROFILE`` keys that do NOT fit that
#: shape — each documented in the module docstring's "Keys" section and
#: handled by explicit code in ``_apply_profile_to_sim()`` instead:
#:   - "encoder_noise": fans out to ONE call, both sides at once
#:     (``set_enc_noise(2, value)``).
#:   - "enc_scale_err_l" / "enc_scale_err_r": each need an explicit ``side``
#:     argument (``set_enc_scale_error(0/1, value)``).
#:   - "motor_offset_l" / "motor_offset_r": no ctypes ABI entry point backs
#:     this knob at all in the sprint-081/082 ABI — applying is skipped
#:     outright (with a one-time ``[WARN]`` if non-neutral).
#:   - "slip_turn_extra": likewise no ctypes ABI backing at all — same
#:     skip-and-warn treatment.
PROFILE_TO_SIM_SETTER: dict = {
    "otos_lin_scale_err": "set_otos_linear_scale_error",
    "otos_ang_scale_err": "set_otos_angular_scale_error",
    "otos_linear_noise": "set_otos_linear_noise",
    "otos_yaw_noise": "set_otos_yaw_noise",
    "otos_lin_drift": "set_otos_linear_drift",
    "otos_yaw_drift": "set_otos_yaw_drift",
    "body_rot_scrub": "set_body_rotational_scrub",
    "body_lin_scrub": "set_body_linear_scrub",
    "trackwidth": "set_trackwidth",
}


def resolve_calibration_defaults(
    log: Callable[[str], None] | None = None,
) -> tuple[float, float]:
    """Resolve ``(body_rot_scrub, trackwidth)`` from the active robot's
    calibration.

    Ticket 073-003: factored out of ``__main__.py``'s "From Calibration"
    button handler (``_on_sim_errors_from_cal()``, ticket 070-004), which
    this function now backs, mirroring its lookup EXACTLY:
    ``get_robot_config()`` -> ``cfg.calibration.rotational_slip`` /
    ``cfg.geometry.trackwidth``, each field independently falling back to
    its neutral value (``1.0`` for body_rot_scrub;
    ``DEFAULT_PROFILE["trackwidth"]`` for trackwidth) with a logged
    ``[WARN]`` when the config, or a specific field on it, is missing.
    Never raises.

    This is the SHARED resolver behind both the manual "From Calibration"
    button and ``load_sim_error_profile()``'s factory-default fallback for
    ``body_rot_scrub`` — a fresh TestGUI install (no persisted profile) now
    turns the commanded angle out of the box instead of requiring the
    operator to discover and click the button (Design Rationale Decision 4).

    Args:
        log: optional sink for the exact ``"[WARN] ..."`` line(s) the
            original button handler appended to the GUI's log pane
            (``_append_log``). Every fallback branch always logs via the
            module logger (``_log.warning``) regardless of ``log`` — this
            module stays Qt-free; passing a callable (e.g. ``_append_log``)
            is how a caller opts into ALSO surfacing the same message on a
            GUI widget without this module importing one.
    """
    # Local import (mirrors the original _on_sim_errors_from_cal()'s own
    # per-call import): re-resolves robot_radio.config.robot_config's
    # get_robot_config attribute fresh on every call, so a test (or a
    # future caller) that monkeypatches it at the SOURCE module -- the
    # existing, established patch point (e.g.
    # test_sim_errors_from_cal_button.py) -- is honored, rather than a
    # frozen reference captured once at this module's own import time.
    from robot_radio.config.robot_config import get_robot_config

    def _warn(msg: str) -> None:
        _log.warning(msg)
        if log is not None:
            log(f"[WARN] {msg}")

    cfg = get_robot_config()
    if cfg is None:
        rot_slip = 1.0
        tw = DEFAULT_PROFILE["trackwidth"]
        _warn(
            "From Calibration: no active robot config found — falling back "
            f"to neutral body_rot_scrub={rot_slip}, trackwidth={tw}"
        )
        return rot_slip, tw

    if cfg.calibration.rotational_slip is not None:
        rot_slip = cfg.calibration.rotational_slip
    else:
        rot_slip = 1.0
        _warn(
            "From Calibration: active robot config has no "
            f"calibration.rotational_slip — falling back to neutral "
            f"body_rot_scrub={rot_slip}"
        )

    if cfg.geometry.trackwidth is not None:
        tw = cfg.geometry.trackwidth
    else:
        tw = DEFAULT_PROFILE["trackwidth"]
        _warn(
            "From Calibration: active robot config has no "
            f"geometry.trackwidth — falling back to neutral "
            f"trackwidth={tw}"
        )

    return rot_slip, tw


def load_sim_error_profile() -> dict:
    """Return the persisted sim error profile merged over ``DEFAULT_PROFILE``.

    Never raises. Missing file, corrupt JSON, or a non-dict top level all
    fall back to a copy of ``DEFAULT_PROFILE`` whose ``body_rot_scrub`` has
    ALREADY been replaced (ticket 073-003) by
    ``resolve_calibration_defaults()``'s reconciled value — the active
    robot's ``calibration.rotational_slip`` (or the neutral ``1.0``, with a
    logged fallback, if no active robot config is found). Missing keys are
    defaulted; unknown keys are ignored; a key present but holding a
    non-numeric (or otherwise unconvertible) value falls back to that key's
    default rather than aborting the whole load.

    A persisted file that DOES carry an explicit ``body_rot_scrub`` key
    (every file ``save_sim_error_profile()`` writes does) always wins over
    the calibration-resolved value below — an operator's existing saved
    profile is not silently overridden.
    """
    profile = dict(DEFAULT_PROFILE)
    # 073-003: the fallback default for body_rot_scrub is now the active
    # robot's reconciled calibration, not the neutral 1.0 literal in
    # DEFAULT_PROFILE. Computed unconditionally so it takes effect for a
    # missing file AND for a persisted file that predates this key (both are
    # "the fallback path" per this function's contract); the merge loop
    # below overwrites it again if the persisted file has its own value.
    profile["body_rot_scrub"], _resolved_tw = resolve_calibration_defaults()
    del _resolved_tw  # trackwidth's own fallback is unaffected by this ticket.
    try:
        data = json.loads(_PREFS_PATH.read_text())
    except Exception:
        return profile
    if not isinstance(data, dict):
        return profile
    for key in DEFAULT_PROFILE:
        if key not in data:
            continue
        try:
            profile[key] = float(data[key])
        except (TypeError, ValueError):
            # Leave the default for this key in place.
            continue
    return profile


def save_sim_error_profile(profile: dict) -> None:
    """Persist ``profile`` (creates ``data/testgui/`` if needed).

    Only the known keys (``DEFAULT_PROFILE``'s keys — the historical four
    plus the full 069-007 knob set, now applied via ``PROFILE_TO_SIM_SETTER``
    per 083-001) are written, each coerced to
    ``float``; a key missing from ``profile`` falls back to
    ``DEFAULT_PROFILE``'s value for it, and a key with a non-numeric value
    falls back the same way. Best effort: logs a warning and returns on
    failure rather than raising, so a persistence error never breaks the Sim
    Errors panel's Apply flow.
    """
    try:
        out = {}
        for key, default in DEFAULT_PROFILE.items():
            try:
                out[key] = float(profile.get(key, default))
            except (TypeError, ValueError):
                out[key] = default
        _PREFS_DIR.mkdir(parents=True, exist_ok=True)
        _PREFS_PATH.write_text(json.dumps(out) + "\n")
    except Exception as exc:
        _log.warning("Failed to persist sim error profile: %s", exc)
