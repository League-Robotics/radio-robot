"""robot_radio.testgui.camera_prefs — camera selection + persisted preference.

Qt-free. Shared by ``operations.py``, ``live_view.py``, and ``__main__.py`` so
all three camera-consuming code paths (one-shot "Refresh Playfield" grab,
the live-view worker, and the Sync-Pose daemon read) resolve to the same
aprilcam camera instead of picking it three different ways (see ticket
``063-008`` / issue ``testgui-playfield-oneshot-grab-and-camera-selection.md``
for the root-cause analysis).

Persistence
-----------
The selected camera name is persisted to ``data/testgui/camera_prefs.json``,
following the ``data/robots/active_robot.json`` pointer-file convention used
by :mod:`robot_radio.config.robot_config` (project-root-relative, computed
from ``_PROJECT_ROOT``). ``src/host/robot_radio/testgui/camera_prefs.py`` sits at
the same directory depth as ``src/host/robot_radio/config/robot_config.py``
(``src/host/robot_radio/<pkg>/<module>.py``), so the same four-``.parent`` chain
resolves to the repository root from either module.

Selection priority (:func:`select_camera`)
-------------------------------------------
1. The persisted preference, if it is present in the daemon's current camera
   list.
2. The first camera whose name contains ``fallback_contains`` (default
   ``"3"``, matching the historical ``_PLAYFIELD_CAMERA_INDEX = 3`` /
   "Arducam OV9782 USB Camera" heuristic in ``operations.py``).
3. The first available camera.
4. ``None`` if no cameras are available at all.

``list_cameras()`` vs ``enumerate_cameras()``
----------------------------------------------
``aprilcam.client.control.DaemonControl`` exposes two different camera
listings:

- ``list_cameras() -> list[str]`` — names of cameras the daemon has
  **already opened**.
- ``enumerate_cameras() -> list[CameraDevice]`` — probes *all* hardware
  (open or not), returning ``CameraDevice(index, name, slug, enum)``.

This module (and every call site wired to it: ``_capture_playfield_frame_and_calib``,
``_read_daemon_pose``, ``live_view._capture_and_emit``, and the GUI camera
combo in ``__main__.py``) uses ``list_cameras()``. Rationale: none of the
runtime capture paths in this codebase ever call ``open_camera()`` — the
daemon owns which cameras are open — so ``enumerate_cameras()`` would let the
operator "select" a camera the daemon cannot actually capture from yet,
silently breaking the pull-down. Using ``list_cameras()`` everywhere keeps
the combo's choices consistent with what the capture paths can actually use.
This choice is made on code-reading grounds (confirmed against the
``DaemonControl`` source in the ``AprilTags`` package); it was not re-verified
against a live daemon session, per the ticket's guidance not to block on that.
"""
from __future__ import annotations

import json
import logging
from pathlib import Path

_log = logging.getLogger(__name__)

# src/host/robot_radio/testgui/camera_prefs.py -> repo root (same depth as
# src/host/robot_radio/config/robot_config.py's _PROJECT_ROOT). 109-002 fix:
# FIVE hops from __file__, not four -- see robot_config.py's own
# _PROJECT_ROOT comment for the full off-by-one explanation (the "unify all
# source trees under src/" refactor, commit 575ef391).
_PROJECT_ROOT = Path(__file__).parent.parent.parent.parent.parent
_PREFS_DIR = _PROJECT_ROOT / "data" / "testgui"
_PREFS_PATH = _PREFS_DIR / "camera_prefs.json"

#: Fallback substring matching the historical _PLAYFIELD_CAMERA_INDEX = 3
#: behavior ("Arducam OV9782 USB Camera" is the current playfield camera).
DEFAULT_FALLBACK_CONTAINS = "3"


def select_camera(
    available: list[str],
    preferred: str | None,
    fallback_contains: str = DEFAULT_FALLBACK_CONTAINS,
) -> str | None:
    """Resolve the camera to use given what's available and the preference.

    Priority: ``preferred`` (if present in ``available``) > first entry in
    ``available`` whose name contains ``fallback_contains`` > first entry in
    ``available`` > ``None`` (if ``available`` is empty).

    Parameters
    ----------
    available:
        Camera names currently known to the daemon (typically from
        ``DaemonControl.list_cameras()``).
    preferred:
        The persisted preference (from :func:`load_camera_pref`), or
        ``None``.
    fallback_contains:
        Substring to match when ``preferred`` is absent or unset. Defaults
        to ``"3"``, matching the historical playfield-camera heuristic.

    Returns
    -------
    str | None
        The resolved camera name, or ``None`` if ``available`` is empty.
    """
    if not available:
        return None
    if preferred is not None and preferred in available:
        return preferred
    for name in available:
        if fallback_contains in name:
            return name
    return available[0]


def load_camera_pref() -> str | None:
    """Return the persisted camera name, or ``None`` if absent/invalid.

    Never raises — any I/O or parse error is treated as "no preference".
    """
    try:
        data = json.loads(_PREFS_PATH.read_text())
        name = data.get("camera_name")
        return str(name) if name else None
    except Exception:
        return None


def save_camera_pref(name: str) -> None:
    """Persist the selected camera name (creates ``data/testgui/`` if needed).

    Best-effort: logs a warning and returns on failure rather than raising,
    so a persistence error never breaks the camera-selection UI flow.
    """
    try:
        _PREFS_DIR.mkdir(parents=True, exist_ok=True)
        _PREFS_PATH.write_text(json.dumps({"camera_name": name}) + "\n")
    except Exception as exc:
        _log.warning("Failed to persist camera preference: %s", exc)
