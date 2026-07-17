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
same ``src/host/robot_radio/testgui/`` depth).

Keys
----
083-001: the sprint-069 ``SIMSET`` wire protocol this module used to target
no longer exists — ``src/firm/commands/`` has no ``SIMSET`` verb (confirmed by
reading every ``makeCmd``/``makeSchemaCmd`` registration; the current verb
set is ``PING``/``VER``/``HELP``/``ECHO``/``ID``/``STREAM``/``SNAP``/
``DEV M``/``DEV DT``/``DEV STATE``/``DEV STOP``/``DEV WD``).

108-007: repointed a SECOND time, from the deleted ``SimConnection``
(sprint 081/082's ~40-symbol ctypes ABI) onto
``robot_radio.io.sim_loop.SimLoop`` (108-005/006's real, 19-symbol
``sim_ctypes.cpp`` ABI over ``TestSim::SimHarness``/``TestSim::SimPlant``).
The new ABI backed FAR fewer fault-condition knobs than the old one did --
``SimLoop`` exposed exactly four fault setters (``set_wheel_disconnected``/
``set_wheel_freeze``/``set_wheel_dropout_rate``/``set_otos_drift``), of
which only ``set_otos_drift`` mapped onto any key below (the
``otos_lin_drift``/``otos_yaw_drift`` pair, combined into one call).

109-002 added a FIFTH setter, ``set_enc_scale_err(port, fraction)``, giving
``enc_scale_err_l``/``enc_scale_err_r`` a real 1:1 mapping each (port
1=left, 2=right) -- see ``transport.py``'s
``SimTransport._apply_profile_to_sim()`` for the actual calls. Every OTHER
key in this module still has NO ``SimLoop`` setter backing it -- applying
is skipped outright and a ``[WARN]`` is logged if the profile carries a
non-neutral value for it. ``PROFILE_TO_SIM_SETTER`` (below) stays EMPTY:
even ``enc_scale_err_l/r``'s new mapping is a port-keyed call, not a bare
1:1 (key -> single-arg setter) shape this table's own contract expects, so
it (like ``otos_lin_drift``/``otos_yaw_drift`` before it) is handled as a
special case directly in ``SimTransport._apply_profile_to_sim()`` instead
of through this table. See that method's own docstring for the
authoritative, current mapping.

``encoder_noise``
    Per-side encoder noise sigma, in millimetres. Default ``0.0``. **No
    ``SimLoop`` setter backs this knob** -- applying is skipped, with a
    ``[WARN]`` logged if set away from its neutral ``0.0``.
``slip_turn_extra``
    Fractional encoder over-report during turns (turn-slip scrub model).
    Default ``0.0`` (ticket 073-003 — previously ``0.26``). **No ``SimLoop``
    setter backs this knob** (no turn-rate-dependent slip knob is wired
    into the current ABI either) -- applying is skipped, with a ``[WARN]``
    logged if set away from its neutral ``0.0``. See
    ``resolve_calibration_defaults()`` for the reconciliation this default
    is (historically) part of.
``otos_linear_noise``
    OTOS linear-position noise sigma, as a fraction of arc. Default ``0.05``.
    **No ``SimLoop`` setter backs this knob** -- applying is skipped, with a
    ``[WARN]`` logged if set away from its neutral ``0.05``.
``otos_yaw_noise``
    OTOS yaw noise sigma, as a fraction. Default ``0.0``. **No ``SimLoop``
    setter backs this knob** -- applying is skipped, with a ``[WARN]``
    logged if set away from its neutral ``0.0``.

Additive/noise terms — ``0.0`` is a genuine no-op:

``enc_scale_err_l`` / ``enc_scale_err_r``
    Fractional per-side encoder over/under-report (0 = perfect). 109-002:
    each maps 1:1 onto ``SimLoop.set_enc_scale_err(port, fraction)``
    (port 1=left, 2=right) -- see ``transport.py``'s
    ``SimTransport._apply_profile_to_sim()`` for the actual calls.
``otos_lin_scale_err`` / ``otos_ang_scale_err``
    Fractional OTOS linear/angular scale error (0 = perfect). **No
    ``SimLoop`` setter backs either knob** -- applying is skipped, with a
    ``[WARN]`` logged if either is set away from its neutral ``0.0``.
``otos_lin_drift`` / ``otos_yaw_drift``
    OTOS linear/yaw drift. THE ONE surviving mapping: combined into a
    single ``SimLoop.set_otos_drift(otos_lin_drift, 0.0, otos_yaw_drift)``
    call (``otos_lin_drift`` -> the ABI's ``x_drift`` term; ``y_drift`` is
    left at its neutral ``0.0`` -- the pre-108-007 profile shape has no
    separate x/y drift terms to split across). ``otos_lin_drift`` is in
    millimetres; ``otos_yaw_drift`` is in radians -- matching
    ``sim_ctypes.cpp``'s ``sim_set_otos_drift`` argument units directly, no
    host-side conversion. See ``transport.py``'s
    ``SimTransport._apply_profile_to_sim()`` for the actual call.

Multiplicative terms — ``1.0`` is the genuine no-op, NOT ``0.0``:

``body_rot_scrub`` / ``body_lin_scrub``
    Body-truth rotational/linear scrub factor. **No ``SimLoop`` setter backs
    either knob** -- applying is skipped, with a ``[WARN]`` logged if either
    is set away from its neutral ``1.0``.
    ``DEFAULT_PROFILE["body_rot_scrub"]`` itself stays the neutral ``1.0``
    (a bare, no-calibration-lookup profile dict must remain a genuine
    no-op) — but ``load_sim_error_profile()``'s FALLBACK path (no persisted
    file, or a persisted file missing this key) resolves it from the active
    robot's calibration via ``resolve_calibration_defaults()`` instead,
    ticket 073-003 (this reconciliation still runs; it just no longer has
    any live sim effect after 108-007's ABI narrowing).
``motor_offset_l`` / ``motor_offset_r``
    Per-side motor actuation offset factor (multiplies commanded velocity).
    **No ``SimLoop`` setter backs either knob** -- applying is skipped, with
    a ``[WARN]`` logged if either is set away from its neutral ``1.0``.

``trackwidth``
    The plant's trackwidth, in millimetres. Has NO safe zero default.
    Defaults to ``128.0``, matching ``PhysicsWorld::kDefaultTrackwidth``
    (``source/hal/sim/physics_world.h``). 108-007: ``SimLoop``'s
    ``track_width`` is fixed at CONSTRUCTION time (``sim_create()``'s own
    argument) -- there is no live setter for it any more. ``SimTransport.
    connect()`` reads this key from the persisted profile and passes it to
    ``SimLoop(track_width=...)``; a live Apply with a changed trackwidth
    logs an explicit "takes effect on next Connect" note (NOT the generic
    "not supported" warning, since it genuinely does take effect, just not
    live).
"""
from __future__ import annotations

import json
import logging
from collections.abc import Callable
from pathlib import Path

_log = logging.getLogger(__name__)

# src/host/robot_radio/testgui/sim_prefs.py -> repo root (same depth as
# src/host/robot_radio/testgui/camera_prefs.py's _PROJECT_ROOT). 109-002
# fix: FIVE hops from __file__ (testgui/robot_radio/host/src/repo-root),
# not four -- the "unify all source trees under src/" refactor (commit
# 575ef391) added one path-depth level without updating this constant, so
# it silently pointed at src/data/testgui/ instead of the real
# data/testgui/ (masked in every test here, which monkeypatches
# _PREFS_DIR/_PREFS_PATH directly rather than exercising this constant).
# See canvas.py's own _HERE/_SRC/_REPO (fixed for this identical off-by-one
# by ticket 107-004) and robot_config.py's _PROJECT_ROOT (109-002's other
# instance of the same bug).
_PROJECT_ROOT = Path(__file__).parent.parent.parent.parent.parent
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

#: Maps every profile key that has a 1:1 ``SimLoop`` setter equivalent to
#: that setter's method name, called with the single profile value as its
#: only argument (``transport.py``'s ``SimTransport._apply_profile_to_sim()``).
#:
#: 108-007: repointed onto ``robot_radio.io.sim_loop.SimLoop``'s far
#: narrower 19-symbol ABI (down from the deleted ``SimConnection``'s
#: ~40-symbol one) -- EMPTY as of this ticket. No remaining
#: ``DEFAULT_PROFILE`` key has a bare 1:1 (key -> single-arg setter) mapping
#: onto ``SimLoop``: the one surviving fault mapping
#: (``otos_lin_drift``/``otos_yaw_drift`` -> ``set_otos_drift(x, y,
#: heading)``) needs two keys combined into one three-argument call, so it
#: is handled as an explicit special case in ``_apply_profile_to_sim()``
#: instead (see that method's own docstring for the authoritative mapping
#: and every "not supported in this sim" key). Kept as a (currently empty)
#: dict, not deleted, so a future ``SimLoop`` fault setter that DOES land a
#: bare 1:1 shape has an obvious place to register without another
#: call-site change.
PROFILE_TO_SIM_SETTER: dict = {}


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
