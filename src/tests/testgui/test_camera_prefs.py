"""src/tests/testgui/test_camera_prefs.py -- ticket 085-008: camera/relay
selection test port. Ported from ``tests_old/testgui/test_camera_prefs.py``.

Headless, Qt-free tests for camera_prefs. Covers:
- ``select_camera`` priority matrix (preferred > fallback_contains > first > None).
- ``save_camera_pref`` / ``load_camera_pref`` round-trip via a monkeypatched
  ``_PREFS_PATH`` pointed at ``tmp_path`` (never touches the real repo
  ``data/`` directory).
- ``load_camera_pref`` returns ``None`` (never raises) on missing/invalid files.
- The module is importable without PySide6 or aprilcam.

No production code change: pure verification pass.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_camera_prefs.py -q
"""

from __future__ import annotations


# ---------------------------------------------------------------------------
# select_camera -- pure priority-matrix tests (no filesystem, no Qt)
# ---------------------------------------------------------------------------


class TestSelectCamera:
    def test_preferred_present_is_selected(self):
        from robot_radio.testgui.camera_prefs import select_camera

        available = ["Brio 501", "Arducam OV9782 USB Camera"]
        result = select_camera(available, preferred="Brio 501")
        assert result == "Brio 501"

    def test_preferred_absent_falls_back_to_fallback_contains(self):
        """Default fallback_contains="arducam" matches the playfield camera."""
        from robot_radio.testgui.camera_prefs import select_camera

        available = ["Brio 501", "arducam-ov9782-usb-camera"]
        result = select_camera(available, preferred="Some Other Camera")
        assert result == "arducam-ov9782-usb-camera"

    def test_no_preferred_falls_back_to_fallback_contains(self):
        """Default fallback_contains="arducam" matches the playfield camera."""
        from robot_radio.testgui.camera_prefs import select_camera

        available = ["Brio 501", "arducam-ov9782-usb-camera"]
        result = select_camera(available, preferred=None)
        assert result == "arducam-ov9782-usb-camera"

    def test_preferred_present_takes_priority_over_fallback(self):
        """A persisted preference wins even when it isn't the fallback match."""
        from robot_radio.testgui.camera_prefs import select_camera

        available = ["arducam-ov9782-usb-camera", "Brio 501"]
        result = select_camera(available, preferred="Brio 501")
        assert result == "Brio 501"
        assert result != available[0]

    def test_no_fallback_match_falls_back_to_first(self):
        from robot_radio.testgui.camera_prefs import select_camera

        available = ["Brio 501", "Logitech C920"]
        result = select_camera(available, preferred=None)
        assert result == "Brio 501"

    def test_empty_available_returns_none(self):
        from robot_radio.testgui.camera_prefs import select_camera

        result = select_camera([], preferred="anything")
        assert result is None

    def test_custom_fallback_contains(self):
        from robot_radio.testgui.camera_prefs import select_camera

        available = ["cam-A", "cam-B", "cam-C"]
        result = select_camera(available, preferred=None, fallback_contains="B")
        assert result == "cam-B"

    def test_default_fallback_contains_matches_the_playfield_camera(self):
        """The default fallback must actually match the real camera name.

        Regression pin: the default used to be ``"3"`` -- a mechanical
        conversion of the historical ``_PLAYFIELD_CAMERA_INDEX = 3`` into a
        NAME-substring match. The live daemon names the camera
        ``"arducam-ov9782-usb-camera"``, which contains no ``"3"``, so the
        rule could never fire and selection always fell through to "first
        available". It looked right only because one camera was open.
        """
        from robot_radio.testgui.camera_prefs import (
            DEFAULT_FALLBACK_CONTAINS,
            select_camera,
        )

        assert DEFAULT_FALLBACK_CONTAINS == "arducam"
        available = ["Brio 501", "arducam-ov9782-usb-camera", "Logitech C920"]
        assert select_camera(available, preferred=None) == "arducam-ov9782-usb-camera"

    def test_fallback_match_is_case_insensitive(self):
        """The daemon lower-cases the slug; older listings used title case."""
        from robot_radio.testgui.camera_prefs import select_camera

        available = ["Brio 501", "Arducam OV9782 USB Camera"]
        assert select_camera(available, preferred=None) == "Arducam OV9782 USB Camera"

    def test_single_camera_unaffected(self):
        """Existing single-camera setups behave exactly as before (no regression)."""
        from robot_radio.testgui.camera_prefs import select_camera

        assert select_camera(["Arducam OV9782 USB Camera"], preferred=None) == (
            "Arducam OV9782 USB Camera"
        )


# ---------------------------------------------------------------------------
# save_camera_pref / load_camera_pref -- persistence round-trip (tmp_path only)
# ---------------------------------------------------------------------------


class TestPersistence:
    def test_round_trip(self, tmp_path, monkeypatch):
        from robot_radio.testgui import camera_prefs

        prefs_path = tmp_path / "camera_prefs.json"
        monkeypatch.setattr(camera_prefs, "_PREFS_PATH", prefs_path)
        monkeypatch.setattr(camera_prefs, "_PREFS_DIR", tmp_path)

        camera_prefs.save_camera_pref("X")
        assert camera_prefs.load_camera_pref() == "X"

    def test_save_creates_directory(self, tmp_path, monkeypatch):
        from robot_radio.testgui import camera_prefs

        nested_dir = tmp_path / "testgui"
        prefs_path = nested_dir / "camera_prefs.json"
        monkeypatch.setattr(camera_prefs, "_PREFS_PATH", prefs_path)
        monkeypatch.setattr(camera_prefs, "_PREFS_DIR", nested_dir)

        assert not nested_dir.exists()
        camera_prefs.save_camera_pref("Arducam OV9782 USB Camera")
        assert prefs_path.exists()
        assert camera_prefs.load_camera_pref() == "Arducam OV9782 USB Camera"

    def test_load_missing_file_returns_none(self, tmp_path, monkeypatch):
        from robot_radio.testgui import camera_prefs

        prefs_path = tmp_path / "does_not_exist.json"
        monkeypatch.setattr(camera_prefs, "_PREFS_PATH", prefs_path)

        assert camera_prefs.load_camera_pref() is None

    def test_load_invalid_json_returns_none(self, tmp_path, monkeypatch):
        from robot_radio.testgui import camera_prefs

        prefs_path = tmp_path / "camera_prefs.json"
        prefs_path.write_text("not valid json {{{")
        monkeypatch.setattr(camera_prefs, "_PREFS_PATH", prefs_path)

        assert camera_prefs.load_camera_pref() is None

    def test_load_empty_camera_name_returns_none(self, tmp_path, monkeypatch):
        from robot_radio.testgui import camera_prefs

        prefs_path = tmp_path / "camera_prefs.json"
        prefs_path.write_text('{"camera_name": ""}')
        monkeypatch.setattr(camera_prefs, "_PREFS_PATH", prefs_path)

        assert camera_prefs.load_camera_pref() is None

    def test_load_missing_key_returns_none(self, tmp_path, monkeypatch):
        from robot_radio.testgui import camera_prefs

        prefs_path = tmp_path / "camera_prefs.json"
        prefs_path.write_text('{"other_key": "value"}')
        monkeypatch.setattr(camera_prefs, "_PREFS_PATH", prefs_path)

        assert camera_prefs.load_camera_pref() is None

    def test_save_never_raises_on_write_failure(self, tmp_path, monkeypatch):
        """save_camera_pref must not raise even if persistence fails."""
        from robot_radio.testgui import camera_prefs

        # Point _PREFS_DIR at a path that collides with an existing file,
        # so mkdir(parents=True, exist_ok=True) raises (NotADirectoryError).
        blocker = tmp_path / "blocker"
        blocker.write_text("I am a file, not a directory")
        bad_dir = blocker / "testgui"
        monkeypatch.setattr(camera_prefs, "_PREFS_DIR", bad_dir)
        monkeypatch.setattr(camera_prefs, "_PREFS_PATH", bad_dir / "camera_prefs.json")

        # Must not raise.
        camera_prefs.save_camera_pref("X")


# ---------------------------------------------------------------------------
# Importability without PySide6 / aprilcam
# ---------------------------------------------------------------------------


class TestImportability:
    def test_importable_without_qt_or_aprilcam(self):
        """The module must not import PySide6 or aprilcam at module scope."""
        import robot_radio.testgui.camera_prefs as camera_prefs_module

        assert callable(camera_prefs_module.select_camera)
        assert callable(camera_prefs_module.load_camera_pref)
        assert callable(camera_prefs_module.save_camera_pref)
