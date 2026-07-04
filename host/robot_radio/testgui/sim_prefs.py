"""robot_radio.testgui.sim_prefs — Sim-mode injected error profile + persistence.

Qt-free. Backs the "Sim Errors" GUI panel (``__main__.py``) and
``SimTransport`` (``transport.py``): lets the operator configure the noise
the simulator injects into encoders and the OTOS, instead of the two
hardcoded module constants that previously governed this
(``_SIM_SLIP_TURN_EXTRA`` / ``_SIM_OTOS_LINEAR_NOISE``).

Persistence
-----------
The profile is persisted to ``data/testgui/sim_error_profile.json``, mirroring
the ``camera_prefs.py`` convention (see that module's docstring for the
``_PROJECT_ROOT`` four-``.parent`` chain rationale — this module sits at the
same ``host/robot_radio/testgui/`` depth).

Keys
----
The first four keys are the historical set, applied directly via dedicated
ctypes methods on the ``Sim`` object (``transport.py``'s
``_apply_profile_to_sim()``). ``slip_turn_extra`` has no ``SIMSET`` wire-key
equivalent — it drives the pre-existing ``_rotationalSlip`` test-infra
channel (``sim.set_field_profile()``) and is applied on a separate path from
every other key below (see ``PROFILE_TO_SIMSET_KEY``'s docstring).

``encoder_noise``
    Per-side encoder noise sigma, in millimetres. Default ``0.0``. Applied to
    both sides equally via the ``SIMSET`` keys ``encNoiseL``/``encNoiseR``
    (fans out to two wire keys — not in ``PROFILE_TO_SIMSET_KEY``).
``slip_turn_extra``
    Fractional encoder over-report during turns (turn-slip scrub model).
    Default ``0.0`` (ticket 073-003 — previously ``0.26``, the historical
    ``_SIM_SLIP_TURN_EXTRA`` value; combined with a neutral ``body_rot_scrub``
    it under-rotated turns net ~14% out of the box). No ``SIMSET`` key (see
    above); see ``resolve_calibration_defaults()`` for the reconciliation
    this default is now part of.
``otos_linear_noise``
    OTOS linear-position noise sigma, as a fraction of arc. Default ``0.05``
    (the historical ``_SIM_OTOS_LINEAR_NOISE`` value). ``SIMSET`` key
    ``otosLinNoise``.
``otos_yaw_noise``
    OTOS yaw noise sigma, as a fraction. Default ``0.0``. ``SIMSET`` key
    ``otosYawNoise``.

The remaining keys (ticket 069-007) are the full newly-surfaced ``SIMSET``
registry (tickets 069-002..004), each mapped 1:1 to its wire key by
``PROFILE_TO_SIMSET_KEY`` and sent in a single ``SIMSET k1=v1 k2=v2 …``
command by ``transport.py``'s ``_apply_profile_to_sim()``.

Additive/noise terms — ``0.0`` is a genuine no-op:

``enc_scale_err_l`` / ``enc_scale_err_r``
    Fractional per-side encoder over/under-report (0 = perfect). ``SIMSET``
    keys ``encScaleErrL`` / ``encScaleErrR``.
``otos_lin_scale_err`` / ``otos_ang_scale_err``
    Fractional OTOS linear/angular scale error (0 = perfect). ``SIMSET``
    keys ``otosLinScaleErr`` / ``otosAngScaleErr``.
``otos_lin_drift`` / ``otos_yaw_drift``
    OTOS linear/yaw drift rate, in mm/s and deg/s respectively (wire-level
    per-second units; ``SimCommands`` converts to per-tick internally).
    ``SIMSET`` keys ``otosLinDriftMmS`` / ``otosYawDriftDegS``.

Multiplicative terms — ``1.0`` is the genuine no-op, NOT ``0.0`` (see
``PhysicsWorld``'s ``_bodyRotationalScrub``/``_bodyLinearScrub``/
``_offsetFactorL``/``_offsetFactorR`` field defaults, all ``1.0f``):

``body_rot_scrub`` / ``body_lin_scrub``
    Body-truth rotational/linear scrub factor, clamped to ``(0, 1]`` on the
    firmware side. ``SIMSET`` keys ``bodyRotScrub`` / ``bodyLinScrub``.
    ``DEFAULT_PROFILE["body_rot_scrub"]`` itself stays the neutral ``1.0``
    (a bare, no-calibration-lookup profile dict must remain a genuine
    no-op) — but ``load_sim_error_profile()``'s FALLBACK path (no persisted
    file, or a persisted file missing this key) resolves it from the active
    robot's calibration via ``resolve_calibration_defaults()`` instead,
    ticket 073-003.
``motor_offset_l`` / ``motor_offset_r``
    Per-side motor actuation offset factor (multiplies commanded velocity).
    ``SIMSET`` keys ``motorOffsetL`` / ``motorOffsetR``.

``trackwidth``
    The plant's trackwidth, in millimetres. Has NO safe zero default —
    ``PhysicsWorld::update()``'s sub-step B divides by it. Defaults to
    ``128.0``: although ``PhysicsWorld::kDefaultTrackwidthMm`` is 150.0,
    the sim re-seeds the plant from the firmware config at construction
    (``sim_api.cpp``: ``hal.setTrackwidth(cfg.trackwidthMm)``,
    ``DefaultConfig.cpp``: 128.0), so 128.0 — not 150.0 — is the value that
    makes Apply-at-defaults a genuine no-op AND keeps the plant's geometry
    matched to the firmware's kinematic calibration (a mismatch makes every
    encoder-arc turn land off-angle by the ratio). This is NOT a sentinel
    meaning "don't touch" — every Apply unconditionally sends
    ``trackwidthMm=<value>`` and overwrites the plant's trackwidth (no
    silent 0-means-no-op special-casing inside ``SimCommands``, which would
    violate ticket 069-003's atomic apply-what-was-sent contract). ``SIMSET``
    key ``trackwidthMm``.
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

#: Default injected-error profile — matches the historical hardcoded
#: constant _SIM_OTOS_LINEAR_NOISE=0.05, plus the two previously-unused
#: knobs (encoder noise, OTOS yaw noise), both defaulted to 0.0 so existing
#: behavior for those three fields is unchanged until an operator opts in.
#:
#: ``slip_turn_extra`` is the exception: ticket 073-003 changed its default
#: from the historical ``_SIM_SLIP_TURN_EXTRA=0.26`` to ``0.0``. The
#: encoder-report-only 0.26 combined with a neutral ``body_rot_scrub`` (see
#: below) under-rotated turns net ~14% out of the box; reconciling
#: body_rot_scrub against the active robot's real calibration (see
#: ``resolve_calibration_defaults()`` and ``load_sim_error_profile()``'s
#: fallback path) is now the factory default instead of a manual opt-in via
#: the "From Calibration" button.
#:
#: 069-007 extends this with the full SIMSET registry (see the module
#: docstring's "Keys" section for units/semantics and no-op rationale per
#: key). Every default below reproduces today's no-op-until-opted-in
#: behavior: additive/noise terms default 0.0, multiplicative terms default
#: 1.0, and trackwidth defaults to the plant's real compiled-in trackwidth
#: (150.0mm) rather than an unsafe 0.0.
DEFAULT_PROFILE: dict = {
    # -- historical four --
    "encoder_noise": 0.0,
    # 073-003: was 0.26 -- see the module-level comment above this dict.
    "slip_turn_extra": 0.0,
    "otos_linear_noise": 0.05,
    "otos_yaw_noise": 0.0,
    # -- 069-007: additive/noise terms (0.0 = no-op) --
    "enc_scale_err_l": 0.0,
    "enc_scale_err_r": 0.0,
    "otos_lin_scale_err": 0.0,
    "otos_ang_scale_err": 0.0,
    "otos_lin_drift": 0.0,
    "otos_yaw_drift": 0.0,
    # -- 069-007: multiplicative terms (1.0 = no-op, NOT 0.0) --
    "body_rot_scrub": 1.0,
    "body_lin_scrub": 1.0,
    "motor_offset_l": 1.0,
    "motor_offset_r": 1.0,
    # -- 069-007: no safe zero default; defaults to the firmware config's
    # trackwidthMm (DefaultConfig.cpp: 128.0 — what the sim seeds the plant
    # with at construction) so Apply-at-default is a genuine no-op rather
    # than a divide-by-zero sentinel. NOT kDefaultTrackwidthMm (150.0): the
    # plant never actually runs at that value, and applying it would inject
    # a plant-vs-calibration geometry error into every turn.
    "trackwidth": 128.0,
}

#: Maps every profile key that has a 1:1 SIMSET wire-key equivalent to that
#: wire-key name (e.g. ``"enc_scale_err_l": "encScaleErrL"``), keyed exactly
#: to source/commands/SimCommands.cpp's kSimRegistry[] rows.
#:
#: Deliberately excludes two DEFAULT_PROFILE keys that do NOT have a 1:1
#: mapping:
#:   - "encoder_noise": fans out to TWO wire keys (encNoiseL AND
#:     encNoiseR, both set to the same value) — handled specially by
#:     transport.py's _apply_profile_to_sim(), not via this map.
#:   - "slip_turn_extra": has no SIMSET key at all — drives the legacy
#:     _rotationalSlip test-infra channel (sim.set_field_profile()) on a
#:     separate path, per Design Rationale Decision 4.
PROFILE_TO_SIMSET_KEY: dict = {
    "enc_scale_err_l": "encScaleErrL",
    "enc_scale_err_r": "encScaleErrR",
    "otos_lin_scale_err": "otosLinScaleErr",
    "otos_ang_scale_err": "otosAngScaleErr",
    "otos_linear_noise": "otosLinNoise",
    "otos_yaw_noise": "otosYawNoise",
    "otos_lin_drift": "otosLinDriftMmS",
    "otos_yaw_drift": "otosYawDriftDegS",
    "body_rot_scrub": "bodyRotScrub",
    "body_lin_scrub": "bodyLinScrub",
    "motor_offset_l": "motorOffsetL",
    "motor_offset_r": "motorOffsetR",
    "trackwidth": "trackwidthMm",
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
    plus the full 069-007 ``SIMSET`` knob set) are written, each coerced to
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
