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
``encoder_noise_mm``
    Per-side encoder noise sigma, in millimetres. Default ``0.0``.
``slip_turn_extra``
    Fractional encoder over-report during turns (turn-slip scrub model).
    Default ``0.26`` (the historical ``_SIM_SLIP_TURN_EXTRA`` value).
``otos_linear_noise``
    OTOS linear-position noise sigma, as a fraction of arc. Default ``0.05``
    (the historical ``_SIM_OTOS_LINEAR_NOISE`` value).
``otos_yaw_noise``
    OTOS yaw noise sigma, as a fraction. Default ``0.0``.
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
DEFAULT_PROFILE: dict = {
    "encoder_noise_mm": 0.0,
    "slip_turn_extra": 0.26,
    "otos_linear_noise": 0.05,
    "otos_yaw_noise": 0.0,
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

    Only the four known keys are written, each coerced to ``float``; a key
    missing from ``profile`` falls back to ``DEFAULT_PROFILE``'s value for
    it, and a key with a non-numeric value falls back the same way. Best
    effort: logs a warning and returns on failure rather than raising, so a
    persistence error never breaks the Sim Errors panel's Apply flow.
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
