---
id: '008'
title: 'Camera-selection: persisted pull-down for the playfield camera'
status: open
use-cases:
- SUC-010
depends-on: []
github-issue: ''
issue: testgui-playfield-oneshot-grab-and-camera-selection.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Camera-selection: persisted pull-down for the playfield camera

## Description

Three different code paths in the Test GUI pick the aprilcam camera three
different ways, confirmed by code reading
(`testgui-playfield-oneshot-grab-and-camera-selection.md`):

- `operations.py::_capture_playfield_frame_and_calib` (one-shot "Refresh
  Playfield" grab): calls `dc.list_cameras()` (names of currently-**open**
  daemon cameras only — see `aprilcam.client.control.DaemonControl.list_cameras`,
  which returns `list[str]` of already-opened camera names), then picks the
  first name containing the digit `3` (`_PLAYFIELD_CAMERA_INDEX = 3`), falling
  back to `cams[0]`.
- `operations.py::_read_daemon_pose`: calls `dc.list_cameras()` and
  unconditionally uses `cams[0]` — **no** index/name matching at all.
- `live_view.py::_capture_and_emit` (the live-view worker, ticket 003): calls
  `dc.list_cameras()` and unconditionally uses `cams[0]` — same gap.

If the daemon has more than one camera open (in the reported case: "Brio 501"
was `cams[0]`, not the calibrated Arducam playfield camera at index 3), the
live-view and pose-read paths may silently read the wrong, possibly
uncalibrated camera and produce garbage or no frame/pose — while "Refresh
Playfield" (which does the name-matching) keeps working. This divergence is
confusing and hard to diagnose from the symptoms alone.

**Important API note for the implementer** (confirmed by reading
`aprilcam.client.control.DaemonControl` in the `AprilTags` package): there are
two different camera listings in the daemon client:

- `list_cameras() -> list[str]` — names of cameras **already open** in the
  daemon. This is what all three existing call sites use today, and what the
  daemon-facing selection logic in this ticket should keep using for the
  *runtime* camera pick (the daemon owns which cameras are open; the GUI does
  not call `open_camera()` itself anywhere in this codebase).
- `enumerate_cameras() -> list[CameraDevice]` — probes *all* hardware, open or
  not, returning `CameraDevice(index, name, slug, enum)` where `index` is the
  **unstable** OS probe index (used to `open_camera(index)`) and `enum` is the
  **persistent, stable, user-facing** enumeration number. This is the richer,
  index+name view the stakeholder is referring to as "the daemon's persistent
  enumeration."

  Verify against `get_robot_api_guide()` / the live daemon at implementation
  time (the MCP session's enumeration may not exactly match what the
  `DaemonControl` instance in `testgui` sees — noted as an open question in
  the issue). If `enumerate_cameras()` is not reachable/appropriate from the
  `testgui`'s `DaemonControl` in practice, the pull-down MAY instead be
  populated from `list_cameras()` (open-camera names only) — pick whichever
  actually lists the calibrated Arducam camera by name in a live check, and
  note the choice in the ticket's implementation notes / docstring so it is
  not re-litigated.

## Stakeholder Decisions (binding)

- Add a `QComboBox` camera pull-down to the GUI listing cameras known to the
  aprilcam daemon (index 3 = "Arducam OV9782 USB Camera" is the current
  playfield camera and must remain the default fallback).
- The selection **persists across sessions** — a small JSON sidecar file.
  Follow the existing `data/robots/active_robot.json` pointer-file convention
  in `host/robot_radio/config/robot_config.py` (project-root-relative,
  `_PROJECT_ROOT`-anchored `Path`): use `data/testgui/camera_prefs.json` (new
  directory) so the persistence mechanism matches an established codebase
  pattern rather than inventing a new location.
- On startup: default to the persisted camera if it is still present in the
  daemon's camera list; otherwise fall back to the camera matching index 3
  ("Arducam OV9782 USB Camera" by name-contains-digit heuristic, matching
  today's `_PLAYFIELD_CAMERA_INDEX` behavior); otherwise fall back to the
  first available camera.
- **All three** camera-consuming code paths must resolve the camera through
  one shared helper, replacing their individual `cams[0]` / name-heuristic
  logic:
  - `operations.py::_capture_playfield_frame_and_calib`
  - `operations.py::_read_daemon_pose`
  - `live_view.py::_capture_and_emit`
- When the user changes the pull-down selection, immediately trigger a fresh
  one-shot grab (reuse the existing `OperationsPanel.trigger_live_grab()` /
  `_capture_playfield_frame_and_calib()` path) and update the playfield image,
  the same way "Refresh Playfield" does.

## Affected Code

- `host/robot_radio/testgui/operations.py` — `_capture_playfield_frame_and_calib`,
  `_read_daemon_pose`, `_PLAYFIELD_CAMERA_INDEX`.
- `host/robot_radio/testgui/live_view.py` — `_capture_and_emit`.
- `host/robot_radio/testgui/__main__.py` — add the `QComboBox` widget + wiring
  (near the existing "Refresh Playfield" control area in the operations
  panel, or in the right panel next to the mode indicator — implementer's
  call on placement; must be visible without scrolling).
- New module: `host/robot_radio/testgui/camera_prefs.py` (Qt-free) — shared
  camera-selection + persistence helper, importable by `operations.py`,
  `live_view.py`, and `__main__.py` without a circular import (all three
  currently import from `operations.py`; a new standalone module avoids
  `live_view.py` importing from `operations.py` for this concern, though it
  already does so for `_deskew_bgr_ndarray` — either is acceptable, but a
  dedicated module keeps the persistence/selection concern cohesive and
  independently testable).

## Acceptance Criteria

### Shared selection/persistence helper (`camera_prefs.py`, Qt-free)

- [ ] A pure function, e.g. `select_camera(available: list[str], preferred:
      str | None, fallback_contains: str = "3") -> str | None`, returns:
      the `preferred` name if present in `available`; else the first entry in
      `available` containing `fallback_contains`; else `available[0]`; else
      `None` if `available` is empty. Fully testable without any daemon or Qt.
- [ ] `load_camera_pref() -> str | None` reads the persisted camera name (or
      `None` if the sidecar file doesn't exist or is invalid — never raises).
- [ ] `save_camera_pref(name: str) -> None` writes the sidecar file
      (`data/testgui/camera_prefs.json`, creating the `data/testgui/`
      directory if needed).
- [ ] Round-trip: `save_camera_pref("X")` then `load_camera_pref()` returns
      `"X"` (headless test, using a temp path override / monkeypatched
      constant — do not write into the real repo `data/` dir from tests).
- [ ] The module is importable without PySide6 or aprilcam installed.

### Consistent camera resolution across all three call sites

- [ ] `_capture_playfield_frame_and_calib` resolves its camera via the shared
      helper (using `dc.list_cameras()` for `available` and the persisted
      preference), not its own inline name-matching loop.
- [ ] `_read_daemon_pose` resolves its camera via the same shared helper
      instead of unconditional `cams[0]`.
- [ ] `live_view.py::_capture_and_emit` resolves its camera via the same
      shared helper instead of unconditional `cams[0]`.
- [ ] Given the same `available` list and persisted preference, all three
      call sites pick the identical camera (verified by a shared-fixture test
      that calls the resolution logic used by each path with the same inputs).

### GUI pull-down

- [ ] A `QComboBox` (suggested `objectName="camera_combo"`) is present in the
      main window, listing the cameras the daemon reports (via `list_cameras()`
      or `enumerate_cameras()` per the implementation note above).
- [ ] On window build, the combo's current selection reflects
      `load_camera_pref()` (or the index-3/fallback default if nothing
      persisted), without requiring a daemon connection to construct the
      window (must degrade gracefully — empty/placeholder combo — when the
      daemon is unreachable at startup, matching existing "no crash without
      hardware" conventions elsewhere in this file).
- [ ] Changing the combo selection calls `save_camera_pref(new_name)` and
      triggers an immediate one-shot playfield grab
      (`trigger_live_grab()`/equivalent) using the newly selected camera.

### No regressions

- [ ] `_PLAYFIELD_CAMERA_INDEX = 3` remains the documented fallback constant
      (used by the shared helper's `fallback_contains` default), so existing
      single-camera setups behave exactly as before.
- [ ] Existing `tests/testgui/test_operations.py` and `tests/testgui/test_live_view.py`
      pass unchanged (update any test that asserted the old `cams[0]`/heuristic
      behavior to instead assert the new shared-helper behavior).
- [ ] All `tests/testgui/` tests pass headlessly with the aprilcam daemon
      mocked/faked (no hardware in CI).

## Implementation Plan

### Approach

1. Create `host/robot_radio/testgui/camera_prefs.py`:

   ```python
   """robot_radio.testgui.camera_prefs — camera selection + persisted preference.

   Qt-free. Shared by operations.py, live_view.py, and __main__.py so all
   three camera-consuming code paths (one-shot grab, live-view worker, pose
   read) resolve to the same aprilcam camera.
   """
   from __future__ import annotations
   import json
   import logging
   from pathlib import Path

   _log = logging.getLogger(__name__)

   _PROJECT_ROOT = Path(__file__).parent.parent.parent.parent  # host/robot_radio/testgui -> repo root
   _PREFS_DIR = _PROJECT_ROOT / "data" / "testgui"
   _PREFS_PATH = _PREFS_DIR / "camera_prefs.json"

   #: Fallback substring matching today's _PLAYFIELD_CAMERA_INDEX = 3 behavior.
   DEFAULT_FALLBACK_CONTAINS = "3"

   def select_camera(
       available: list[str],
       preferred: str | None,
       fallback_contains: str = DEFAULT_FALLBACK_CONTAINS,
   ) -> str | None:
       """Resolve the camera to use given what's available and the preference.

       Priority: preferred (if present in available) > first available camera
       whose name contains fallback_contains > first available camera > None.
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
       """Return the persisted camera name, or None if absent/invalid."""
       try:
           data = json.loads(_PREFS_PATH.read_text())
           name = data.get("camera_name")
           return str(name) if name else None
       except Exception:
           return None

   def save_camera_pref(name: str) -> None:
       """Persist the selected camera name (creates data/testgui/ if needed)."""
       try:
           _PREFS_DIR.mkdir(parents=True, exist_ok=True)
           _PREFS_PATH.write_text(json.dumps({"camera_name": name}) + "\n")
       except Exception as exc:
           _log.warning("Failed to persist camera preference: %s", exc)
   ```

   Verify the exact confirmed relative path from `camera_prefs.py` to the repo
   root matches `robot_config.py`'s `_PROJECT_ROOT` computation
   (`host/robot_radio/config/robot_config.py` uses `parent.parent.parent.parent`;
   `host/robot_radio/testgui/camera_prefs.py` is at the same directory depth,
   so the same `.parent` chain length applies — confirm with a quick check
   rather than assuming).

2. In `operations.py`, replace the inline name-matching loop in
   `_capture_playfield_frame_and_calib` and the unconditional `cams[0]` in
   `_read_daemon_pose` with calls to
   `camera_prefs.select_camera(cams, camera_prefs.load_camera_pref())`.

3. In `live_view.py::_capture_and_emit`, same replacement for `cam = cams[0]`.

4. In `__main__.py`, add the `camera_combo` `QComboBox`, populate it at window
   build time (best-effort daemon query, degrade to empty on failure — mirror
   the existing defensive patterns already used for daemon calls elsewhere in
   this file), and wire `currentTextChanged` to `save_camera_pref(...)` +
   `ops_ctrl.trigger_live_grab()`.

5. Confirm the daemon-listing choice (`list_cameras()` vs `enumerate_cameras()`)
   against `get_robot_api_guide()` or a live daemon session before finalizing
   the combo-population code — do not guess silently; the two APIs have
   different semantics (open-only vs. all-hardware) and picking the wrong one
   will silently limit what the operator can select.

### Files to create

- `host/robot_radio/testgui/camera_prefs.py`
- `tests/testgui/test_camera_prefs.py`

### Files to modify

- `host/robot_radio/testgui/operations.py`
- `host/robot_radio/testgui/live_view.py`
- `host/robot_radio/testgui/__main__.py`
- `tests/testgui/test_operations.py` (update any camera-selection assertions)
- `tests/testgui/test_live_view.py` (update any camera-selection assertions)

### Testing Plan

- **Existing tests to run**: `QT_QPA_PLATFORM=offscreen uv run python -m pytest
  tests/testgui/ -q`; `uv run python -m pytest tests/simulation -q`.
- **New tests to write**:
  - `tests/testgui/test_camera_prefs.py` (Qt-free, no daemon):
    - `select_camera` preferred-present, preferred-absent-falls-back-to-3,
      no-3-falls-back-to-first, empty-available-returns-None.
    - `save_camera_pref` / `load_camera_pref` round-trip using a monkeypatched
      `_PREFS_PATH` pointed at a `tmp_path` fixture (never touch the real
      repo `data/` directory from a test).
    - `load_camera_pref` returns `None` (not an exception) when the file is
      missing or contains invalid JSON.
  - Update `test_operations.py`: mock `dc.list_cameras()` to return multiple
    names (e.g. `["Brio 501", "Arducam OV9782 USB Camera"]`) and assert
    `_capture_playfield_frame_and_calib` / `_read_daemon_pose` both select the
    Arducam entry via the shared helper, not `cams[0]`.
  - Update `test_live_view.py`: same multi-camera assertion for
    `_capture_and_emit`.
  - New GUI test: `camera_combo` exists, lists mocked camera names, and
    changing selection calls `save_camera_pref` and triggers a grab (mock
    `trigger_live_grab`/`_capture_playfield_frame_and_calib` and assert it was
    called after a simulated combo change).
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run python -m pytest
  tests/testgui/ -q`

### Documentation updates

- `camera_prefs.py` module docstring (new file) — document the persistence
  location, the `select_camera` priority order, and why a shared module
  exists (three call sites, one behavior).
- Update `operations.py`'s module docstring section on `_PLAYFIELD_CAMERA_INDEX`
  to note it is now only the `fallback_contains` default, not the sole
  selection mechanism.
- Update `live_view.py`'s module docstring to mention camera resolution now
  goes through `camera_prefs.select_camera`.
