"""tests/testgui/test_camera_combo.py -- ticket 085-008: camera/relay
selection test port. Ported from ``tests_old/testgui/test_camera_combo.py``.

Headless tests for the camera-selection pull-down added to the main window
(ticket 063-008). Covers:
- ``camera_combo`` (a ``QComboBox``) exists in the built window.
- Window construction never raises/blocks when the daemon is unreachable
  (empty combo instead).
- The combo is populated from the (mocked) daemon's ``list_cameras()`` and
  the initial selection reflects ``camera_prefs.load_camera_pref()`` (or the
  fallback heuristic when nothing is persisted).
- Changing the combo selection calls ``camera_prefs.save_camera_pref()`` and
  triggers ``OpsController.trigger_live_grab()``.

The aprilcam daemon is fully mocked in every test -- no hardware/daemon
required in CI.

No production code change: pure verification pass.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_camera_combo.py -q
"""

from __future__ import annotations

import sys
from unittest.mock import MagicMock, patch

import pytest


@pytest.fixture(scope="module")
def qapp():
    """Return (or create) the QApplication singleton for this module."""
    # 107-004: turn a missing `gui` dependency group into a clean skip, not
    # a hard collection/run error -- see test_tour1_geometry.py's module
    # docstring for the full rationale (tests/testgui/ re-added to
    # pyproject.toml's testpaths this ticket).
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]

    app = QApplication.instance() or QApplication(sys.argv)
    yield app


# ---------------------------------------------------------------------------
# camera_combo exists / degrades gracefully without a daemon
# ---------------------------------------------------------------------------


class TestCameraComboExists:
    def test_camera_combo_exists(self, qapp):
        """A QComboBox with objectName 'camera_combo' is present in the window."""
        from robot_radio.testgui.__main__ import _build_main_window
        from PySide6.QtWidgets import QComboBox  # type: ignore[import-untyped]

        with patch("aprilcam.config.Config") as MockConfig, \
             patch("aprilcam.client.control.DaemonControl") as MockDC:
            fake_dc = MagicMock()
            fake_dc.list_cameras.return_value = []
            MockConfig.load.return_value = MagicMock()
            MockDC.connect_default.return_value = fake_dc

            window, _ = _build_main_window()

        combo = window.findChild(QComboBox, "camera_combo")
        assert combo is not None, "Expected a QComboBox with objectName 'camera_combo'"
        window.close()

    def test_daemon_unreachable_does_not_raise_or_block(self, qapp):
        """Window construction must not crash when the daemon can't be reached.

        Degrades to an empty combo -- matches the "no crash without hardware"
        convention used elsewhere in this file (e.g. the Sim-mode fallback).
        """
        from robot_radio.testgui.__main__ import _build_main_window
        from PySide6.QtWidgets import QComboBox  # type: ignore[import-untyped]

        with patch("aprilcam.client.control.DaemonControl") as MockDC:
            MockDC.connect_default.side_effect = RuntimeError("daemon unreachable")

            window, _ = _build_main_window()  # must not raise

        combo = window.findChild(QComboBox, "camera_combo")
        assert combo is not None
        assert combo.count() == 0
        window.close()

    def test_aprilcam_not_installed_does_not_raise(self, qapp):
        """Window construction must not crash if aprilcam cannot be imported."""
        from robot_radio.testgui.__main__ import _build_main_window
        from PySide6.QtWidgets import QComboBox  # type: ignore[import-untyped]

        with patch.dict(sys.modules, {"aprilcam.config": None, "aprilcam.client.control": None}):
            window, _ = _build_main_window()  # must not raise

        combo = window.findChild(QComboBox, "camera_combo")
        assert combo is not None
        assert combo.count() == 0
        window.close()


# ---------------------------------------------------------------------------
# Population + initial selection reflects the persisted preference
# ---------------------------------------------------------------------------


class TestCameraComboPopulateAndPreference:
    _CAMS = ["Brio 501", "Arducam OV9782 USB Camera"]

    def test_populates_from_daemon_list_cameras(self, qapp):
        from robot_radio.testgui.__main__ import _build_main_window
        from PySide6.QtWidgets import QComboBox  # type: ignore[import-untyped]

        with patch("aprilcam.config.Config") as MockConfig, \
             patch("aprilcam.client.control.DaemonControl") as MockDC, \
             patch(
                 "robot_radio.testgui.camera_prefs.load_camera_pref",
                 return_value=None,
             ):
            fake_dc = MagicMock()
            fake_dc.list_cameras.return_value = list(self._CAMS)
            MockConfig.load.return_value = MagicMock()
            MockDC.connect_default.return_value = fake_dc

            window, _ = _build_main_window()

        combo = window.findChild(QComboBox, "camera_combo")
        assert combo is not None
        assert [combo.itemText(i) for i in range(combo.count())] == self._CAMS
        window.close()

    def test_initial_selection_reflects_persisted_preference(self, qapp):
        from robot_radio.testgui.__main__ import _build_main_window
        from PySide6.QtWidgets import QComboBox  # type: ignore[import-untyped]

        with patch("aprilcam.config.Config") as MockConfig, \
             patch("aprilcam.client.control.DaemonControl") as MockDC, \
             patch(
                 "robot_radio.testgui.camera_prefs.load_camera_pref",
                 return_value="Arducam OV9782 USB Camera",
             ):
            fake_dc = MagicMock()
            fake_dc.list_cameras.return_value = list(self._CAMS)
            MockConfig.load.return_value = MagicMock()
            MockDC.connect_default.return_value = fake_dc

            window, _ = _build_main_window()

        combo = window.findChild(QComboBox, "camera_combo")
        assert combo.currentText() == "Arducam OV9782 USB Camera"
        window.close()

    def test_no_persisted_pref_falls_back_to_index3_heuristic(self, qapp):
        """No persisted preference -> falls back to the "3"-heuristic camera."""
        from robot_radio.testgui.__main__ import _build_main_window
        from PySide6.QtWidgets import QComboBox  # type: ignore[import-untyped]

        cams = ["cam-1", "cam-3", "cam-5"]
        with patch("aprilcam.config.Config") as MockConfig, \
             patch("aprilcam.client.control.DaemonControl") as MockDC, \
             patch(
                 "robot_radio.testgui.camera_prefs.load_camera_pref",
                 return_value=None,
             ):
            fake_dc = MagicMock()
            fake_dc.list_cameras.return_value = list(cams)
            MockConfig.load.return_value = MagicMock()
            MockDC.connect_default.return_value = fake_dc

            window, _ = _build_main_window()

        combo = window.findChild(QComboBox, "camera_combo")
        assert combo.currentText() == "cam-3"
        window.close()

    def test_no_fallback_match_falls_back_to_first_camera(self, qapp):
        """No persisted preference and no fallback match -> first available camera."""
        from robot_radio.testgui.__main__ import _build_main_window
        from PySide6.QtWidgets import QComboBox  # type: ignore[import-untyped]

        cams = ["cam-a", "cam-b"]
        with patch("aprilcam.config.Config") as MockConfig, \
             patch("aprilcam.client.control.DaemonControl") as MockDC, \
             patch(
                 "robot_radio.testgui.camera_prefs.load_camera_pref",
                 return_value=None,
             ):
            fake_dc = MagicMock()
            fake_dc.list_cameras.return_value = list(cams)
            MockConfig.load.return_value = MagicMock()
            MockDC.connect_default.return_value = fake_dc

            window, _ = _build_main_window()

        combo = window.findChild(QComboBox, "camera_combo")
        assert combo.currentText() == "cam-a"
        window.close()


# ---------------------------------------------------------------------------
# Selection change persists the preference and triggers a fresh grab
# ---------------------------------------------------------------------------


class TestCameraComboSelectionChange:
    def test_change_persists_and_triggers_grab(self, qapp):
        """Changing camera_combo saves the new preference and triggers a grab."""
        from robot_radio.testgui.__main__ import _build_main_window
        from robot_radio.testgui.operations import OpsController
        from PySide6.QtWidgets import QComboBox  # type: ignore[import-untyped]

        cams = ["Brio 501", "Arducam OV9782 USB Camera"]
        saved: list[str] = []

        with patch("aprilcam.config.Config") as MockConfig, \
             patch("aprilcam.client.control.DaemonControl") as MockDC, \
             patch(
                 "robot_radio.testgui.camera_prefs.load_camera_pref",
                 return_value=None,
             ), \
             patch(
                 "robot_radio.testgui.camera_prefs.save_camera_pref",
                 side_effect=lambda name: saved.append(name),
             ), \
             patch.object(OpsController, "trigger_live_grab") as mock_trigger:
            fake_dc = MagicMock()
            fake_dc.list_cameras.return_value = list(cams)
            MockConfig.load.return_value = MagicMock()
            MockDC.connect_default.return_value = fake_dc

            window, _ = _build_main_window()
            combo = window.findChild(QComboBox, "camera_combo")
            assert combo is not None
            # Initial fallback selection is "Brio 501" (no persisted pref, no
            # camera name contains "3") -- switch to the Arducam entry so the
            # signal actually fires (Qt only emits on an actual value change).
            assert combo.currentText() == "Brio 501"

            combo.setCurrentText("Arducam OV9782 USB Camera")

        assert saved == ["Arducam OV9782 USB Camera"], (
            f"Expected save_camera_pref('Arducam OV9782 USB Camera'); got {saved!r}"
        )
        mock_trigger.assert_called_once()
        window.close()

    def test_empty_selection_does_not_save_or_trigger(self, qapp):
        """Guard: an empty combo (no cameras) must not call save/trigger."""
        from robot_radio.testgui.__main__ import _build_main_window
        from robot_radio.testgui.operations import OpsController
        from PySide6.QtWidgets import QComboBox  # type: ignore[import-untyped]

        with patch("aprilcam.config.Config") as MockConfig, \
             patch("aprilcam.client.control.DaemonControl") as MockDC, \
             patch(
                 "robot_radio.testgui.camera_prefs.save_camera_pref"
             ) as mock_save, \
             patch.object(OpsController, "trigger_live_grab") as mock_trigger:
            fake_dc = MagicMock()
            fake_dc.list_cameras.return_value = []
            MockConfig.load.return_value = MagicMock()
            MockDC.connect_default.return_value = fake_dc

            window, _ = _build_main_window()
            combo = window.findChild(QComboBox, "camera_combo")
            assert combo is not None
            assert combo.count() == 0

        mock_save.assert_not_called()
        mock_trigger.assert_not_called()
        window.close()
