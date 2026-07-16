"""src/tests/testgui/test_live_view.py -- ticket 085-007: live camera view
verification. Ported from ``tests_old/testgui/test_live_view.py``.

Tests:
- ``_deskew_bgr_ndarray`` is Qt-free and returns ndarray or None.
- ``_LiveViewWorker`` emits ``frame_ready`` with mocked daemon.
- Worker holds last known tag pose when tag 100 is absent (no snap to 0,0).
- ``CanvasController.set_avatar_pose`` positions the marker.
- ``CanvasController.restore_static_background`` resets origin and background.

No production code change: pure verification pass.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_live_view.py -v
"""

from __future__ import annotations

import math

import pytest


# ---------------------------------------------------------------------------
# QApplication fixture (module-scoped to share across all tests)
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def qapp():
    """Return (or create) the QApplication singleton for this module."""
    # 107-004: turn a missing `gui` dependency group into a clean skip, not
    # a hard collection/run error -- see test_tour1_geometry.py's module
    # docstring for the full rationale.
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]
    import sys
    app = QApplication.instance() or QApplication(sys.argv)
    yield app


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_fake_tag_frame(ox: float = 40.0, oy: float = 30.0,
                          fw: float = 80.0, fh: float = 60.0) -> object:
    """Build a minimal fake TagFrame for operations/deskew tests."""
    import numpy as np

    H = np.eye(3, dtype=float).tolist()

    class FakeTagFrame:
        homography = H
        playfield_corners = [[0, 0], [640, 0], [640, 480], [0, 480]]
        field_width_cm = fw
        field_height_cm = fh
        origin_x = ox
        origin_y = oy

        def by_id(self, tag_id: int):
            return None  # no tags by default

    return FakeTagFrame()


# ---------------------------------------------------------------------------
# _deskew_bgr_ndarray -- Qt-free deskew helper
# ---------------------------------------------------------------------------


class TestDeskewBgrNdarray:
    """_deskew_bgr_ndarray must be Qt-free and callable in a headless test."""

    def test_importable_without_qt(self):
        """Module-level import of _deskew_bgr_ndarray must not touch Qt."""
        from robot_radio.testgui.operations import _deskew_bgr_ndarray
        assert callable(_deskew_bgr_ndarray)

    def test_returns_tuple_or_none(self):
        """Returns (ndarray, float, float) on success or None if cv2 absent."""
        import numpy as np
        from robot_radio.testgui.operations import _deskew_bgr_ndarray

        tag_frame = _make_fake_tag_frame()
        raw = np.zeros((480, 640, 3), dtype=np.uint8)
        result = _deskew_bgr_ndarray(raw, tag_frame, ppc=2.0)

        # Either a valid 3-tuple or None (if cv2 not installed in test env).
        assert result is None or (
            isinstance(result, tuple) and len(result) == 3
        ), f"Expected (ndarray, float, float) or None; got {type(result)}"

    def test_returns_ndarray_not_pixmap(self):
        """_deskew_bgr_ndarray must return an ndarray, not a QPixmap."""
        import numpy as np
        from robot_radio.testgui.operations import _deskew_bgr_ndarray

        tag_frame = _make_fake_tag_frame()
        raw = np.zeros((480, 640, 3), dtype=np.uint8)
        result = _deskew_bgr_ndarray(raw, tag_frame, ppc=2.0)

        if result is not None:
            ndarray, ox, oy = result
            assert isinstance(ndarray, np.ndarray), (
                f"First element must be ndarray, got {type(ndarray)}"
            )
            assert isinstance(ox, float)
            assert isinstance(oy, float)

    def test_no_homography_returns_none(self):
        """Returns None when TagFrame has no homography."""
        import numpy as np
        from robot_radio.testgui.operations import _deskew_bgr_ndarray

        class NoHomographyFrame:
            homography = None
            field_width_cm = 80.0
            field_height_cm = 60.0
            origin_x = 40.0
            origin_y = 30.0
            playfield_corners = None

        raw = np.zeros((480, 640, 3), dtype=np.uint8)
        result = _deskew_bgr_ndarray(raw, NoHomographyFrame(), ppc=2.0)
        assert result is None

    def test_deskew_bgr_with_tag_frame_behavior_unchanged(self, qapp):
        """_deskew_bgr_with_tag_frame wraps _deskew_bgr_ndarray -- behavior unchanged.

        This ensures the refactor did not break the QPixmap-returning wrapper.

        NOTE (085-007 bug fix): the pre-rebuild file did not request the
        ``qapp`` fixture here, even though this is the one test in this class
        that constructs a real ``QPixmap`` (via ``_bgr_ndarray_to_pixmap``).
        Running this file standalone (`pytest src/tests/testgui/test_live_view.py`,
        exactly the invocation this file's own module docstring documents)
        crashed with a hard `Fatal Python error: Aborted` inside
        `QImage`/`QPixmap` construction -- Qt requires a `QApplication` to
        exist first, and nothing earlier in this file created one. The crash
        was latent/hidden in a full-suite run only because some
        alphabetically-earlier test file happened to construct a
        `QApplication` first. Fixed by requesting `qapp` like every other
        Qt-touching test in this file.
        """
        import numpy as np
        from robot_radio.testgui.operations import _deskew_bgr_with_tag_frame

        # FakeTagFrame with a valid homography so the function can succeed.
        class FakeTagFrame:
            homography = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            playfield_corners = [[0, 0], [640, 0], [640, 480], [0, 480]]
            field_width_cm = 80.0
            field_height_cm = 60.0
            origin_x = 40.0
            origin_y = 30.0

        raw = np.zeros((480, 640, 3), dtype=np.uint8)
        result = _deskew_bgr_with_tag_frame(raw, FakeTagFrame(), ppc=2.0)

        # Must return None or a (QPixmap, float, float) tuple.
        assert result is None or (
            isinstance(result, tuple) and len(result) == 3
        )


# ---------------------------------------------------------------------------
# _LiveViewWorker -- factory and signal emission
# ---------------------------------------------------------------------------


class TestLiveViewWorker:
    """build_live_view_worker() returns a QObject with frame_ready signal."""

    def test_module_importable_without_qt(self):
        """live_view module must be importable without PySide6 (deferred import)."""
        from robot_radio.testgui import live_view
        assert hasattr(live_view, "build_live_view_worker")

    def test_build_returns_qobject(self, qapp):
        """build_live_view_worker() returns a QObject."""
        from PySide6.QtCore import QObject  # type: ignore[import-untyped]
        from robot_radio.testgui.live_view import build_live_view_worker

        worker = build_live_view_worker()
        assert isinstance(worker, QObject)

    def test_worker_has_frame_ready_signal(self, qapp):
        """Worker must have a frame_ready signal."""
        from robot_radio.testgui.live_view import build_live_view_worker

        worker = build_live_view_worker()
        assert hasattr(worker, "frame_ready"), "Worker must have frame_ready signal"

    def test_worker_has_stop_slot(self, qapp):
        """Worker must have a stop() method."""
        from robot_radio.testgui.live_view import build_live_view_worker

        worker = build_live_view_worker()
        assert callable(getattr(worker, "stop", None)), "Worker must have stop() slot"

    def test_worker_has_last_tag_initialized_to_zero(self, qapp):
        """Worker._last_tag must be initialised to (0.0, 0.0, 0.0)."""
        from robot_radio.testgui.live_view import build_live_view_worker

        worker = build_live_view_worker()
        assert hasattr(worker, "_last_tag")
        assert worker._last_tag == (0.0, 0.0, 0.0)

    def test_stop_sets_stop_flag(self, qapp):
        """Calling stop() sets the internal _stop flag."""
        from robot_radio.testgui.live_view import build_live_view_worker

        worker = build_live_view_worker()
        assert not worker._stop
        worker.stop()
        assert worker._stop

    def test_capture_and_emit_with_mocked_daemon(self, qapp):
        """Worker._capture_and_emit() with a mocked daemon emits frame_ready."""
        import numpy as np
        from unittest.mock import MagicMock, patch
        from PySide6.QtCore import Qt  # type: ignore[import-untyped]
        from robot_radio.testgui.live_view import build_live_view_worker

        received: list = []
        worker = build_live_view_worker()
        worker.frame_ready.connect(
            lambda bgr, ox, oy, tx, ty, tyaw: received.append((ox, oy, tx, ty, tyaw)),
            Qt.ConnectionType.DirectConnection,
        )

        # Fake BGR frame.
        fake_bgr = np.zeros((60, 80, 3), dtype=np.uint8)

        # Fake TagFrame -- by_id(100) returns None (tag not visible).
        fake_tag_frame = MagicMock()
        fake_tag_frame.homography = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        fake_tag_frame.playfield_corners = [[0, 0], [80, 0], [80, 60], [0, 60]]
        fake_tag_frame.field_width_cm = 80.0
        fake_tag_frame.field_height_cm = 60.0
        fake_tag_frame.origin_x = 40.0
        fake_tag_frame.origin_y = 30.0
        fake_tag_frame.by_id.return_value = None  # tag 100 not present

        fake_dc = MagicMock()
        fake_dc.list_cameras.return_value = ["cam0"]
        fake_dc.capture_frame.return_value = fake_bgr
        fake_dc.get_tags.return_value = fake_tag_frame

        with patch("aprilcam.config.Config") as MockConfig, \
             patch("aprilcam.client.control.DaemonControl") as MockDC:
            MockConfig.load.return_value = MagicMock()
            MockDC.connect_default.return_value = fake_dc

            worker._capture_and_emit()

        # If cv2 is available the frame was emitted; if not, result was None.
        # Both outcomes are acceptable -- we just verify no crash.
        # If a frame was emitted, verify we got a 5-tuple.
        if received:
            assert len(received[0]) == 5, "Expected (ox, oy, tx, ty, tyaw)"

    def test_capture_and_emit_selects_camera_via_shared_helper_not_cams0(self, qapp):
        """_capture_and_emit must resolve the camera via camera_prefs.select_camera(),
        not an unconditional cams[0] (ticket 063-008).

        Mocks a multi-camera daemon list with the non-playfield camera first
        (mirroring the reported bug: "Brio 501" was cams[0], not the
        calibrated Arducam playfield camera) and a persisted preference for
        the Arducam camera, then asserts capture_frame/get_tags were called
        with the Arducam name, not cams[0].
        """
        import numpy as np
        from unittest.mock import MagicMock, patch
        from robot_radio.testgui.live_view import build_live_view_worker

        cams = ["Brio 501", "Arducam OV9782 USB Camera"]

        fake_bgr = np.zeros((60, 80, 3), dtype=np.uint8)
        fake_tag_frame = MagicMock()
        fake_tag_frame.homography = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        fake_tag_frame.playfield_corners = [[0, 0], [80, 0], [80, 60], [0, 60]]
        fake_tag_frame.field_width_cm = 80.0
        fake_tag_frame.field_height_cm = 60.0
        fake_tag_frame.origin_x = 40.0
        fake_tag_frame.origin_y = 30.0
        fake_tag_frame.by_id.return_value = None

        fake_dc = MagicMock()
        fake_dc.list_cameras.return_value = cams
        fake_dc.capture_frame.return_value = fake_bgr
        fake_dc.get_tags.return_value = fake_tag_frame

        worker = build_live_view_worker()

        with patch("aprilcam.config.Config") as MockConfig, \
             patch("aprilcam.client.control.DaemonControl") as MockDC, \
             patch(
                 "robot_radio.testgui.camera_prefs.load_camera_pref",
                 return_value="Arducam OV9782 USB Camera",
             ):
            MockConfig.load.return_value = MagicMock()
            MockDC.connect_default.return_value = fake_dc

            worker._capture_and_emit()

        fake_dc.capture_frame.assert_called_once_with("Arducam OV9782 USB Camera")
        fake_dc.get_tags.assert_called_once_with("Arducam OV9782 USB Camera")
        assert fake_dc.capture_frame.call_args[0][0] != cams[0]

    def test_worker_holds_last_tag_when_tag_missing(self, qapp):
        """When tag 100 is absent, worker emits last known pose (no snap to 0,0)."""
        import numpy as np
        from unittest.mock import MagicMock, patch
        from PySide6.QtCore import Qt  # type: ignore[import-untyped]
        from robot_radio.testgui.live_view import build_live_view_worker

        received: list = []
        worker = build_live_view_worker()
        worker.frame_ready.connect(
            lambda bgr, ox, oy, tx, ty, tyaw: received.append((tx, ty, tyaw)),
            Qt.ConnectionType.DirectConnection,
        )

        # Seed a known last pose directly.
        worker._last_tag = (12.0, 34.0, 0.5)

        fake_bgr = np.zeros((60, 80, 3), dtype=np.uint8)

        # Fake TagFrame -- by_id(100) returns None (tag not visible).
        fake_tag_frame = MagicMock()
        fake_tag_frame.homography = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        fake_tag_frame.playfield_corners = [[0, 0], [80, 0], [80, 60], [0, 60]]
        fake_tag_frame.field_width_cm = 80.0
        fake_tag_frame.field_height_cm = 60.0
        fake_tag_frame.origin_x = 40.0
        fake_tag_frame.origin_y = 30.0
        fake_tag_frame.by_id.return_value = None  # tag 100 absent

        fake_dc = MagicMock()
        fake_dc.list_cameras.return_value = ["cam0"]
        fake_dc.capture_frame.return_value = fake_bgr
        fake_dc.get_tags.return_value = fake_tag_frame

        with patch("aprilcam.config.Config") as MockConfig, \
             patch("aprilcam.client.control.DaemonControl") as MockDC:
            MockConfig.load.return_value = MagicMock()
            MockDC.connect_default.return_value = fake_dc

            worker._capture_and_emit()

        # If a frame was emitted (cv2 available), the tag pose must be the last known.
        if received:
            tx, ty, tyaw = received[0]
            assert tx == 12.0, f"Avatar must hold last known X=12.0, got {tx}"
            assert ty == 34.0, f"Avatar must hold last known Y=34.0, got {ty}"
            assert tyaw == 0.5, f"Avatar must hold last known yaw=0.5, got {tyaw}"

    def test_worker_updates_last_tag_when_tag_visible(self, qapp):
        """When tag 100 is visible with world_xy, worker updates _last_tag
        using rec.yaw (tag orientation) -- heading_rad (velocity course) is
        never consulted.
        """
        import numpy as np
        from unittest.mock import MagicMock, patch
        from PySide6.QtCore import Qt  # type: ignore[import-untyped]
        from robot_radio.testgui.live_view import build_live_view_worker

        received: list = []
        worker = build_live_view_worker()
        worker.frame_ready.connect(
            lambda bgr, ox, oy, tx, ty, tyaw: received.append((tx, ty, tyaw)),
            Qt.ConnectionType.DirectConnection,
        )

        fake_bgr = np.zeros((60, 80, 3), dtype=np.uint8)

        # Fake TagRecord for tag 100.
        fake_tag_rec = MagicMock()
        fake_tag_rec.world_xy = (55.0, 22.0)
        fake_tag_rec.heading_rad = 1.2  # velocity course -- must be ignored
        fake_tag_rec.yaw = 0.5  # tag orientation -- this is what must be used

        # Fake TagFrame -- by_id(100) returns the fake record.
        fake_tag_frame = MagicMock()
        fake_tag_frame.homography = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        fake_tag_frame.playfield_corners = [[0, 0], [80, 0], [80, 60], [0, 60]]
        fake_tag_frame.field_width_cm = 80.0
        fake_tag_frame.field_height_cm = 60.0
        fake_tag_frame.origin_x = 40.0
        fake_tag_frame.origin_y = 30.0
        fake_tag_frame.by_id.return_value = fake_tag_rec

        fake_dc = MagicMock()
        fake_dc.list_cameras.return_value = ["cam0"]
        fake_dc.capture_frame.return_value = fake_bgr
        fake_dc.get_tags.return_value = fake_tag_frame

        with patch("aprilcam.config.Config") as MockConfig, \
             patch("aprilcam.client.control.DaemonControl") as MockDC:
            MockConfig.load.return_value = MagicMock()
            MockDC.connect_default.return_value = fake_dc

            worker._capture_and_emit()

        # If a frame was emitted (cv2 available), verify tag pose was updated.
        if received:
            tx, ty, tyaw = received[0]
            assert tx == pytest.approx(55.0), f"Expected tx=55.0, got {tx}"
            assert ty == pytest.approx(22.0), f"Expected ty=22.0, got {ty}"
            assert tyaw == pytest.approx(0.5), (
                f"Expected tyaw=0.5 (rec.yaw), got {tyaw} -- heading_rad must be ignored"
            )
            # Also verify _last_tag was updated.
            assert worker._last_tag == pytest.approx((55.0, 22.0, 0.5))

    def test_avatar_yaw_ignores_heading_rad_uses_tag_yaw(self, qapp):
        """Avatar yaw must come from rec.yaw (tag orientation), never
        heading_rad (velocity course-over-ground) -- even when both are
        present and clearly different values.
        """
        import numpy as np
        from unittest.mock import MagicMock, patch
        from PySide6.QtCore import Qt  # type: ignore[import-untyped]
        from robot_radio.testgui.live_view import build_live_view_worker

        received: list = []
        worker = build_live_view_worker()
        worker.frame_ready.connect(
            lambda bgr, ox, oy, tx, ty, tyaw: received.append((tx, ty, tyaw)),
            Qt.ConnectionType.DirectConnection,
        )

        fake_bgr = np.zeros((60, 80, 3), dtype=np.uint8)

        fake_tag_rec = MagicMock()
        fake_tag_rec.world_xy = (1.0, 2.0)
        fake_tag_rec.yaw = 0.5
        fake_tag_rec.heading_rad = 2.0  # deliberately different -- must be ignored

        fake_tag_frame = MagicMock()
        fake_tag_frame.homography = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        fake_tag_frame.playfield_corners = [[0, 0], [80, 0], [80, 60], [0, 60]]
        fake_tag_frame.field_width_cm = 80.0
        fake_tag_frame.field_height_cm = 60.0
        fake_tag_frame.origin_x = 40.0
        fake_tag_frame.origin_y = 30.0
        fake_tag_frame.by_id.return_value = fake_tag_rec

        fake_dc = MagicMock()
        fake_dc.list_cameras.return_value = ["cam0"]
        fake_dc.capture_frame.return_value = fake_bgr
        fake_dc.get_tags.return_value = fake_tag_frame

        with patch("aprilcam.config.Config") as MockConfig, \
             patch("aprilcam.client.control.DaemonControl") as MockDC:
            MockConfig.load.return_value = MagicMock()
            MockDC.connect_default.return_value = fake_dc

            worker._capture_and_emit()

        if received:
            _, _, tyaw = received[0]
            assert tyaw == pytest.approx(0.5), (
                f"Expected tyaw=0.5 (rec.yaw); heading_rad=2.0 must be ignored, got {tyaw}"
            )

    def test_avatar_yaw_none_holds_last_known_not_heading_rad(self, qapp):
        """When rec.yaw is None (but heading_rad is set), the avatar must hold
        the last-known yaw -- it must NOT fall back to heading_rad.
        """
        import numpy as np
        from unittest.mock import MagicMock, patch
        from PySide6.QtCore import Qt  # type: ignore[import-untyped]
        from robot_radio.testgui.live_view import build_live_view_worker

        received: list = []
        worker = build_live_view_worker()
        worker.frame_ready.connect(
            lambda bgr, ox, oy, tx, ty, tyaw: received.append((tx, ty, tyaw)),
            Qt.ConnectionType.DirectConnection,
        )

        # Seed a known last yaw.
        worker._last_tag = (1.0, 2.0, 0.75)

        fake_bgr = np.zeros((60, 80, 3), dtype=np.uint8)

        fake_tag_rec = MagicMock()
        fake_tag_rec.world_xy = (9.0, 8.0)
        fake_tag_rec.yaw = None
        fake_tag_rec.heading_rad = 2.0  # must NOT be used as a fallback

        fake_tag_frame = MagicMock()
        fake_tag_frame.homography = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        fake_tag_frame.playfield_corners = [[0, 0], [80, 0], [80, 60], [0, 60]]
        fake_tag_frame.field_width_cm = 80.0
        fake_tag_frame.field_height_cm = 60.0
        fake_tag_frame.origin_x = 40.0
        fake_tag_frame.origin_y = 30.0
        fake_tag_frame.by_id.return_value = fake_tag_rec

        fake_dc = MagicMock()
        fake_dc.list_cameras.return_value = ["cam0"]
        fake_dc.capture_frame.return_value = fake_bgr
        fake_dc.get_tags.return_value = fake_tag_frame

        with patch("aprilcam.config.Config") as MockConfig, \
             patch("aprilcam.client.control.DaemonControl") as MockDC:
            MockConfig.load.return_value = MagicMock()
            MockDC.connect_default.return_value = fake_dc

            worker._capture_and_emit()

        if received:
            tx, ty, tyaw = received[0]
            assert tx == pytest.approx(9.0), f"Expected tx updated to 9.0, got {tx}"
            assert ty == pytest.approx(8.0), f"Expected ty updated to 8.0, got {ty}"
            assert tyaw == pytest.approx(0.75), (
                f"Expected last-known yaw=0.75 held (not heading_rad=2.0), got {tyaw}"
            )

    def test_worker_uses_by_id_not_tags_dict(self, qapp):
        """Worker must call by_id(100) on the TagFrame, not tags.get(100)."""
        import numpy as np
        from unittest.mock import MagicMock, patch
        from robot_radio.testgui.live_view import build_live_view_worker

        fake_bgr = np.zeros((60, 80, 3), dtype=np.uint8)

        # Fake TagFrame that tracks by_id calls.
        fake_tag_frame = MagicMock()
        fake_tag_frame.homography = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        fake_tag_frame.playfield_corners = [[0, 0], [80, 0], [80, 60], [0, 60]]
        fake_tag_frame.field_width_cm = 80.0
        fake_tag_frame.field_height_cm = 60.0
        fake_tag_frame.origin_x = 40.0
        fake_tag_frame.origin_y = 30.0
        fake_tag_frame.by_id.return_value = None

        fake_dc = MagicMock()
        fake_dc.list_cameras.return_value = ["cam0"]
        fake_dc.capture_frame.return_value = fake_bgr
        fake_dc.get_tags.return_value = fake_tag_frame

        worker = build_live_view_worker()

        with patch("aprilcam.config.Config") as MockConfig, \
             patch("aprilcam.client.control.DaemonControl") as MockDC:
            MockConfig.load.return_value = MagicMock()
            MockDC.connect_default.return_value = fake_dc

            worker._capture_and_emit()

        # by_id(100) must have been called (only if a frame was actually deskewed).
        # We use a permissive check: if by_id was called it must have been called
        # with 100.  It may not be called if cv2 is absent (deskew returns None).
        if fake_tag_frame.by_id.called:
            fake_tag_frame.by_id.assert_called_with(100)


# ---------------------------------------------------------------------------
# CanvasController additions
# ---------------------------------------------------------------------------


class TestSetAvatarPose:
    """CanvasController.set_avatar_pose positions the marker at world coordinates."""

    @pytest.fixture
    def canvas_ctrl(self, qapp):
        from robot_radio.testgui.traces import TraceModel
        from robot_radio.testgui.canvas import build_canvas

        tm = TraceModel()
        _, ctrl = build_canvas(tm)
        return ctrl

    def test_set_avatar_pose_does_not_raise(self, canvas_ctrl):
        """set_avatar_pose(x, y, yaw) must not raise."""
        canvas_ctrl.set_avatar_pose(10.0, 5.0, math.pi / 2)  # must not raise

    def test_set_avatar_pose_marker_visible(self, canvas_ctrl):
        """After set_avatar_pose the marker must be visible."""
        canvas_ctrl.set_avatar_pose(10.0, 5.0, math.pi / 4)
        assert canvas_ctrl._marker_group.isVisible(), (
            "Marker must be visible after set_avatar_pose"
        )

    def test_set_avatar_pose_rotation(self, canvas_ctrl):
        """set_avatar_pose applies rotation = 90 - degrees(yaw_rad)."""
        yaw = math.pi / 2  # 90 degrees -> rotation = 90 - 90 = 0
        canvas_ctrl.set_avatar_pose(0.0, 0.0, yaw)
        expected = 90.0 - math.degrees(yaw)
        actual = canvas_ctrl._marker_group.rotation()
        assert actual == pytest.approx(expected, abs=0.01), (
            f"rotation: expected {expected:.2f}, got {actual:.2f}"
        )

    def test_set_avatar_pose_position(self, canvas_ctrl):
        """set_avatar_pose positions the marker at the correct pixel coords."""
        from robot_radio.testgui.canvas import _PIXELS_PER_CM

        x_cm, y_cm = 10.0, 5.0
        canvas_ctrl.set_avatar_pose(x_cm, y_cm, 0.0)

        expected_px, expected_py = canvas_ctrl._world_to_px(x_cm, y_cm)
        pos = canvas_ctrl._marker_group.pos()
        assert pos.x() == pytest.approx(expected_px, abs=1.0), (
            f"Marker x: expected {expected_px:.1f}, got {pos.x():.1f}"
        )
        assert pos.y() == pytest.approx(expected_py, abs=1.0), (
            f"Marker y: expected {expected_py:.1f}, got {pos.y():.1f}"
        )

    def test_set_avatar_pose_does_not_read_trace_model(self, canvas_ctrl):
        """set_avatar_pose must not depend on trace_model.fused."""
        # Verify it works with an empty fused trace (no data fed).
        assert len(canvas_ctrl._trace_model.fused) == 0, (
            "Pre-condition: fused trace must be empty"
        )
        canvas_ctrl.set_avatar_pose(30.0, 20.0, 0.3)  # must not raise
        assert canvas_ctrl._marker_group.isVisible()


class TestRestoreStaticBackground:
    """CanvasController.restore_static_background resets origin and background."""

    @pytest.fixture
    def canvas_ctrl(self, qapp):
        from robot_radio.testgui.traces import TraceModel
        from robot_radio.testgui.canvas import build_canvas

        tm = TraceModel()
        _, ctrl = build_canvas(tm)
        return ctrl

    def test_restore_static_background_does_not_raise(self, canvas_ctrl):
        """restore_static_background() must not raise."""
        canvas_ctrl.restore_static_background()  # must not raise

    def test_restore_static_background_resets_origin_x(self, canvas_ctrl):
        """After restore, origin_x must equal field_w_cm / 2."""
        # First set a non-centre origin.
        from PySide6.QtGui import QPixmap, QColor  # type: ignore[import-untyped]
        pm = QPixmap(400, 300)
        pm.fill(QColor(100, 100, 100))
        canvas_ctrl.set_background(pm, origin_x=10.0, origin_y=8.0)
        assert canvas_ctrl._origin_x == pytest.approx(10.0)

        canvas_ctrl.restore_static_background()

        expected_ox = canvas_ctrl._field_w_cm / 2.0
        assert canvas_ctrl._origin_x == pytest.approx(expected_ox), (
            f"origin_x after restore: expected {expected_ox}, got {canvas_ctrl._origin_x}"
        )

    def test_restore_static_background_resets_origin_y(self, canvas_ctrl):
        """After restore, origin_y must equal field_h_cm / 2."""
        from PySide6.QtGui import QPixmap, QColor  # type: ignore[import-untyped]
        pm = QPixmap(400, 300)
        pm.fill(QColor(100, 100, 100))
        canvas_ctrl.set_background(pm, origin_x=10.0, origin_y=8.0)

        canvas_ctrl.restore_static_background()

        expected_oy = canvas_ctrl._field_h_cm / 2.0
        assert canvas_ctrl._origin_y == pytest.approx(expected_oy), (
            f"origin_y after restore: expected {expected_oy}, got {canvas_ctrl._origin_y}"
        )

    def test_restore_static_background_world_zero_maps_to_centre(self, canvas_ctrl):
        """After restore, world (0,0) must map to image centre."""
        canvas_ctrl.restore_static_background()
        px, py = canvas_ctrl._world_to_px(0.0, 0.0)
        from robot_radio.testgui.canvas import _load_calibration, _PIXELS_PER_CM
        fw, fh = _load_calibration()
        ppc = _PIXELS_PER_CM
        assert px == pytest.approx(ppc * fw / 2.0, abs=2.0)
        assert py == pytest.approx(ppc * fh / 2.0, abs=2.0)

    def test_restore_calls_refresh(self, qapp):
        """restore_static_background() must call refresh() internally."""
        from robot_radio.testgui.traces import TraceModel
        from robot_radio.testgui.canvas import build_canvas

        tm = TraceModel()
        _, ctrl = build_canvas(tm)

        refresh_calls = []
        original_refresh = ctrl.refresh

        def _spy(*args, **kwargs):
            refresh_calls.append(1)
            return original_refresh(*args, **kwargs)

        ctrl.refresh = _spy
        ctrl.restore_static_background()
        assert len(refresh_calls) >= 1, "restore_static_background must call refresh()"
