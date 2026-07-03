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

``encoder_noise_mm``
    Per-side encoder noise sigma, in millimetres. Default ``0.0``. Applied to
    both sides equally via the ``SIMSET`` keys ``encNoiseL``/``encNoiseR``
    (fans out to two wire keys — not in ``PROFILE_TO_SIMSET_KEY``).
``slip_turn_extra``
    Fractional encoder over-report during turns (turn-slip scrub model).
    Default ``0.26`` (the historical ``_SIM_SLIP_TURN_EXTRA`` value). No
    ``SIMSET`` key (see above).
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
``otos_lin_drift_mms`` / ``otos_yaw_drift_degs``
    OTOS linear/yaw drift rate, in mm/s and deg/s respectively (wire-level
    per-second units; ``SimCommands`` converts to per-tick internally).
    ``SIMSET`` keys ``otosLinDriftMmS`` / ``otosYawDriftDegS``.

Multiplicative terms — ``1.0`` is the genuine no-op, NOT ``0.0`` (see
``PhysicsWorld``'s ``_bodyRotationalScrub``/``_bodyLinearScrub``/
``_offsetFactorL``/``_offsetFactorR`` field defaults, all ``1.0f``):

``body_rot_scrub`` / ``body_lin_scrub``
    Body-truth rotational/linear scrub factor, clamped to ``(0, 1]`` on the
    firmware side. ``SIMSET`` keys ``bodyRotScrub`` / ``bodyLinScrub``.
``motor_offset_l`` / ``motor_offset_r``
    Per-side motor actuation offset factor (multiplies commanded velocity).
    ``SIMSET`` keys ``motorOffsetL`` / ``motorOffsetR``.

``trackwidth_mm``
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
from pathlib import Path

_log = logging.getLogger(__name__)

# host/robot_radio/testgui/sim_prefs.py -> repo root (same depth as
# host/robot_radio/testgui/camera_prefs.py's _PROJECT_ROOT).
_PROJECT_ROOT = Path(__file__).parent.parent.parent.parent
_PREFS_DIR = _PROJECT_ROOT / "data" / "testgui"
_PREFS_PATH = _PREFS_DIR / "sim_error_profile.json"

#: Default injected-error profile — matches the historical hardcoded
#: constants (_SIM_SLIP_TURN_EXTRA=0.26, _SIM_OTOS_LINEAR_NOISE=0.05) plus
#: the two previously-unused knobs (encoder noise, OTOS yaw noise), both
#: defaulted to 0.0 so existing behavior is unchanged until an operator
#: opts in.
#:
#: 069-007 extends this with the full SIMSET registry (see the module
#: docstring's "Keys" section for units/semantics and no-op rationale per
#: key). Every default below reproduces today's no-op-until-opted-in
#: behavior: additive/noise terms default 0.0, multiplicative terms default
#: 1.0, and trackwidth_mm defaults to the plant's real compiled-in trackwidth
#: (150.0mm) rather than an unsafe 0.0.
DEFAULT_PROFILE: dict = {
    # -- historical four --
    "encoder_noise_mm": 0.0,
    "slip_turn_extra": 0.26,
    "otos_linear_noise": 0.05,
    "otos_yaw_noise": 0.0,
    # -- 069-007: additive/noise terms (0.0 = no-op) --
    "enc_scale_err_l": 0.0,
    "enc_scale_err_r": 0.0,
    "otos_lin_scale_err": 0.0,
    "otos_ang_scale_err": 0.0,
    "otos_lin_drift_mms": 0.0,
    "otos_yaw_drift_degs": 0.0,
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
    "trackwidth_mm": 128.0,
}

#: Maps every profile key that has a 1:1 SIMSET wire-key equivalent to that
#: wire-key name (e.g. ``"enc_scale_err_l": "encScaleErrL"``), keyed exactly
#: to source/commands/SimCommands.cpp's kSimRegistry[] rows.
#:
#: Deliberately excludes two DEFAULT_PROFILE keys that do NOT have a 1:1
#: mapping:
#:   - "encoder_noise_mm": fans out to TWO wire keys (encNoiseL AND
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
    "otos_lin_drift_mms": "otosLinDriftMmS",
    "otos_yaw_drift_degs": "otosYawDriftDegS",
    "body_rot_scrub": "bodyRotScrub",
    "body_lin_scrub": "bodyLinScrub",
    "motor_offset_l": "motorOffsetL",
    "motor_offset_r": "motorOffsetR",
    "trackwidth_mm": "trackwidthMm",
}


def load_sim_error_profile() -> dict:
    """Return the persisted sim error profile merged over ``DEFAULT_PROFILE``.

    Never raises. Missing file, corrupt JSON, or a non-dict top level all
    fall back to a copy of ``DEFAULT_PROFILE``. Missing keys are defaulted;
    unknown keys are ignored; a key present but holding a non-numeric (or
    otherwise unconvertible) value falls back to that key's default rather
    than aborting the whole load.
    """
    profile = dict(DEFAULT_PROFILE)
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
