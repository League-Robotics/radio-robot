"""tests/testgui/test_set_origin.py -- ticket 085-004: Operations panel
verification (Sync-Pose, Zero-Encoders, Set-Origin, STREAM). Ported from
``tests_old/testgui/test_set_origin.py``.

The pre-rebuild file's inline reimplementation of ``_set_origin`` only
modelled a 3-step sequence (``ZERO enc``, ``OZ``, ``SI``). Production
(``host/robot_radio/testgui/__main__.py``, ``_set_origin``, ~line 1729) has
since grown a leading ``STOP`` (halt + cancel any in-flight Planner goal,
architecture-update.md Decision 1) and a Sim-only plant-teleport step
between ``STOP`` and ``ZERO enc`` -- a real 5-step sequence. This file's
inline reimplementation below mirrors that CURRENT sequence line-for-line
(the same "no import seam" answer ``test_goto.py``/``test_tour1_geometry.py``
use for other callables nested inside ``_build_main_window()``), and adds
the real-sim confirmation ticket 004 asks for: after actually driving away
from world origin, clicking "Set Robot @ 0,0" reads the fused pose back at
(0, 0, 0 deg) against the real compiled sprint-084 firmware/sim.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_set_origin.py -v
"""
from __future__ import annotations

import time

import pytest

from robot_radio.testgui.operations import (
    OpsController,
    build_setpose_command,
    is_sim_transport,
)
from robot_radio.testgui.transport import Transport, _sim_lib_path

_requires_sim_lib = pytest.mark.skipif(
    not _sim_lib_path().exists(),
    reason="sim lib not built -- run `just build-sim` first",
)


# ---------------------------------------------------------------------------
# Qt-free: build_setpose_command sanity (kept from the pre-rebuild file)
# ---------------------------------------------------------------------------


def test_build_setpose_command_origin() -> None:
    """build_setpose_command(0, 0, 0) returns an SI command string."""
    result = build_setpose_command(0.0, 0.0, 0.0)
    assert result.startswith("SI"), f"Expected SI command, got: {result!r}"
    assert len(result.split()) == 4, f"Expected 4 tokens in SI command, got: {result!r}"


def test_build_setpose_command_origin_exact() -> None:
    """build_setpose_command(0, 0, 0) returns exactly 'SI 0 0 0'."""
    assert build_setpose_command(0.0, 0.0, 0.0) == "SI 0 0 0"


# ---------------------------------------------------------------------------
# Fakes (mirrors test_operations.py's established pattern)
# ---------------------------------------------------------------------------


class _FakeTransport(Transport):
    """Records every command()/send() line; no real IO."""

    def __init__(self) -> None:
        super().__init__()
        self.commands_sent: list[str] = []

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def send(self, line: str) -> None:
        self.commands_sent.append(line)

    def command(self, line: str, read_timeout: int = 300) -> str:  # [ms]
        self.commands_sent.append(line)
        return ""


class SimTransport(_FakeTransport):  # noqa: N801 -- name is load-bearing: is_sim_transport()
    """Fake transport named exactly ``SimTransport``, so
    ``operations.is_sim_transport()``'s duck-type check (``type(t).__name__``)
    matches -- exercises ``_set_origin``'s Sim-only plant-teleport branch
    without needing the real ctypes sim.
    """

    def __init__(self) -> None:
        super().__init__()
        self.true_pose_calls: list[tuple[float, float, float]] = []

    def set_true_pose(self, x_cm: float, y_cm: float, yaw_rad: float) -> None:
        self.true_pose_calls.append((x_cm, y_cm, yaw_rad))
        self.commands_sent.append(f"TELEPORT {x_cm} {y_cm} {yaw_rad}")


class _ReplyTransport(_FakeTransport):
    """Like _FakeTransport, but command() returns a configurable reply."""

    def __init__(self, reply: str = "") -> None:
        super().__init__()
        self.reply = reply

    def command(self, line: str, read_timeout: int = 300) -> str:
        self.commands_sent.append(line)
        return self.reply


class _RaisingTransport(_FakeTransport):
    """command() always raises -- used to test failure-revert paths."""

    def command(self, line: str, read_timeout: int = 300) -> str:
        raise RuntimeError("boom")


class _FakeTraceModel:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def anchor(self, *_a) -> None:
        self.calls.append("anchor")

    def clear(self) -> None:
        self.calls.append("clear")


class _FakeCanvasCtrl:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def reset_avatar_to_center(self) -> None:
        self.calls.append("reset_avatar_to_center")

    def refresh(self, *_a) -> None:
        self.calls.append("refresh")


class _FakeButton:
    """Minimal QPushButton stand-in -- records every setter call."""

    def __init__(self) -> None:
        self.enabled: bool | None = None
        self.tooltip: str | None = None
        self.checked: bool | None = None
        self.text: str | None = None
        self.visible: bool | None = None

    def setEnabled(self, value: bool) -> None:
        self.enabled = value

    def setToolTip(self, value: str) -> None:
        self.tooltip = value

    def setChecked(self, value: bool) -> None:
        self.checked = value

    def setText(self, value: str) -> None:
        self.text = value

    def setVisible(self, value: bool) -> None:
        self.visible = value


def _make_controller(
    transport: "Transport | None", *, sync_btn=None, stream_btn=None
) -> tuple[OpsController, list[str]]:
    logs: list[str] = []
    controller = OpsController(
        transport_ref={"transport": transport},
        log_cb=logs.append,
        sync_btn=sync_btn,
        zero_btn=None,
        stop_btn=None,
        clear_btn=None,
        refresh_btn=None,
        stream_btn=stream_btn,
        origin_btn=None,
        transport_buttons=[],
    )
    return controller, logs


# ---------------------------------------------------------------------------
# _set_origin -- inline reimplementation mirroring __main__.py (~line 1729),
# the CURRENT 5-step sequence: STOP, sim-teleport, ZERO enc, OZ, SI 0 0 0.
# ---------------------------------------------------------------------------


def _set_origin(transport, trace_model, canvas_ctrl, append_log) -> None:
    """Line-for-line reimplementation of __main__.py's _set_origin()."""
    if transport is not None:
        transport.command("STOP", read_timeout=300)
        if is_sim_transport(transport):
            transport.set_true_pose(0.0, 0.0, 0.0)
        transport.command("ZERO enc", read_timeout=300)
        transport.command("OZ", read_timeout=300)
        si_cmd = build_setpose_command(0.0, 0.0, 0.0)
        transport.command(si_cmd, read_timeout=300)
    else:
        append_log("[WARN] Set Robot @ 0,0: no robot connected — display only")
    trace_model.anchor(0.0, 0.0, 0.0)
    trace_model.clear()
    canvas_ctrl.reset_avatar_to_center()
    canvas_ctrl.refresh()


def test_set_origin_non_sim_sends_stop_zero_oz_si_in_order() -> None:
    """A non-Sim transport gets exactly 4 wire commands, no plant teleport,
    in the order STOP, ZERO enc, OZ, SI 0 0 0."""
    transport = _FakeTransport()
    log: list[str] = []

    _set_origin(transport, _FakeTraceModel(), _FakeCanvasCtrl(), log.append)

    assert transport.commands_sent == ["STOP", "ZERO enc", "OZ", "SI 0 0 0"], (
        f"got: {transport.commands_sent}"
    )


def test_set_origin_sim_transport_teleports_plant_between_stop_and_zero_enc() -> None:
    """Sim mode inserts a plant teleport to (0,0,0) between STOP and ZERO enc."""
    transport = SimTransport()
    log: list[str] = []

    _set_origin(transport, _FakeTraceModel(), _FakeCanvasCtrl(), log.append)

    assert transport.commands_sent == [
        "STOP", "TELEPORT 0.0 0.0 0.0", "ZERO enc", "OZ", "SI 0 0 0",
    ], f"got: {transport.commands_sent}"
    assert transport.true_pose_calls == [(0.0, 0.0, 0.0)]


def test_set_origin_no_transport_skips_wire_commands_and_logs_warning() -> None:
    """_set_origin with no transport skips wire commands and logs a warning."""
    log: list[str] = []

    _set_origin(None, _FakeTraceModel(), _FakeCanvasCtrl(), log.append)

    assert any("no robot connected" in line for line in log), (
        "Expected a disconnected-state warning in the log"
    )


def test_set_origin_no_transport_display_reset_still_runs() -> None:
    """Even without transport, the display-reset methods are all called."""
    trace_model = _FakeTraceModel()
    canvas_ctrl = _FakeCanvasCtrl()

    _set_origin(None, trace_model, canvas_ctrl, lambda _l: None)

    assert trace_model.calls == ["anchor", "clear"]
    assert canvas_ctrl.calls == ["reset_avatar_to_center", "refresh"]


# ---------------------------------------------------------------------------
# Sync Pose disabled (with explanatory tooltip) in Sim mode -- controller-level,
# Qt-free.
# ---------------------------------------------------------------------------


def test_sync_pose_disabled_with_tooltip_when_transport_is_sim() -> None:
    sync_btn = _FakeButton()
    stream_btn = _FakeButton()
    transport = SimTransport()
    controller, _logs = _make_controller(transport, sync_btn=sync_btn, stream_btn=stream_btn)

    controller.set_connected(True, transport=transport)

    assert sync_btn.enabled is False
    assert sync_btn.tooltip is not None and "Sim mode" in sync_btn.tooltip


def test_sync_pose_untouched_by_set_connected_for_non_sim_transport() -> None:
    sync_btn = _FakeButton()
    stream_btn = _FakeButton()
    transport = _FakeTransport()
    controller, _logs = _make_controller(transport, sync_btn=sync_btn, stream_btn=stream_btn)

    controller.set_connected(True, transport=transport)

    # set_connected only touches sync_btn for the Sim-mode branch; a non-Sim
    # transport (and a sync_btn not itself in transport_buttons here) leaves
    # it completely untouched.
    assert sync_btn.enabled is None
    assert sync_btn.tooltip is None


# ---------------------------------------------------------------------------
# Zero Encoders -- sends ZERO enc, reply logged.
# ---------------------------------------------------------------------------


def test_on_zero_encoders_sends_zero_enc_and_logs_reply() -> None:
    transport = _ReplyTransport(reply="OK zero enc")
    controller, logs = _make_controller(transport)

    controller.on_zero_encoders()

    assert transport.commands_sent == ["ZERO enc"]
    assert any("OK zero enc" in line for line in logs)


def test_on_zero_encoders_not_connected_logs_warning() -> None:
    controller, logs = _make_controller(None)

    controller.on_zero_encoders()

    assert any("not connected" in line for line in logs)


# ---------------------------------------------------------------------------
# STREAM toggle -- STREAM 50 / STREAM 0, reverts on failed send, resets to
# off on disconnect.
# ---------------------------------------------------------------------------


def test_stream_toggled_on_sends_stream_50() -> None:
    transport = _ReplyTransport()
    stream_btn = _FakeButton()
    controller, _logs = _make_controller(transport, stream_btn=stream_btn)

    controller.on_stream_toggled(True)

    assert transport.commands_sent == ["STREAM 50"]
    assert stream_btn.text == "STREAM: on"


def test_stream_toggled_off_sends_stream_0() -> None:
    transport = _ReplyTransport()
    stream_btn = _FakeButton()
    controller, _logs = _make_controller(transport, stream_btn=stream_btn)

    controller.on_stream_toggled(False)

    assert transport.commands_sent == ["STREAM 0"]
    assert stream_btn.text == "STREAM: off"


def test_stream_toggled_reverts_visual_state_on_failed_send() -> None:
    transport = _RaisingTransport()
    stream_btn = _FakeButton()
    controller, logs = _make_controller(transport, stream_btn=stream_btn)

    controller.on_stream_toggled(True)

    assert stream_btn.checked is False, "a failed STREAM send must revert the toggle"
    assert any("STREAM toggle" in line for line in logs)


def test_set_connected_false_resets_stream_to_off() -> None:
    stream_btn = _FakeButton()
    stream_btn.checked = True
    stream_btn.text = "STREAM: on"
    controller, _logs = _make_controller(_FakeTransport(), stream_btn=stream_btn)

    controller.set_connected(False)

    assert stream_btn.checked is False
    assert stream_btn.text == "STREAM: off"


# ---------------------------------------------------------------------------
# Real end-to-end confirmation: Set-Origin's fused pose reads back at world
# origin, after actually driving away, against the real 084 sim.
# ---------------------------------------------------------------------------

_FIRMWARE_TRACKWIDTH = 128.0
#: Zero-error Sim Errors panel values (matches test_goto.py/test_tour1_geometry.py).
_ZERO_ERROR_SPINS: dict[str, float] = {
    "sim_err_encoder_mm": 0.0,
    "sim_err_enc_scale_l": 0.0,
    "sim_err_enc_scale_r": 0.0,
    "sim_err_slip_turn": 0.0,
    "sim_err_body_rot_scrub": 1.0,
    "sim_err_body_lin_scrub": 1.0,
    "sim_err_motor_offset_l": 1.0,
    "sim_err_motor_offset_r": 1.0,
    "sim_err_trackwidth": _FIRMWARE_TRACKWIDTH,
    "sim_err_otos_linear": 0.0,
    "sim_err_otos_yaw": 0.0,
    "sim_err_otos_lin_scale": 0.0,
    "sim_err_otos_ang_scale": 0.0,
    "sim_err_otos_lin_drift": 0.0,
    "sim_err_otos_yaw_drift": 0.0,
}


@pytest.fixture(scope="module")
def qapp():
    """QApplication for the module (offscreen platform set by conftest)."""
    import sys

    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]

    app = QApplication.instance() or QApplication(sys.argv)
    yield app


def _spin_events(qapp, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        qapp.processEvents()
        time.sleep(0.01)


def _connect_real_sim(qapp, monkeypatch, tmp_path):
    """Build the real GUI window and connect a real SimTransport.

    Mirrors test_goto.py's ``_connect_sim``/test_tour1_geometry.py's setup:
    pins the active robot to a literal, uncalibrated ("nocal") config,
    redirects sim_prefs persistence to tmp_path, and zeroes every Sim Errors
    panel knob via the real spinboxes + Apply for a deterministic run.
    """
    import json

    from PySide6.QtWidgets import (  # type: ignore[import-untyped]
        QComboBox,
        QDoubleSpinBox,
        QPushButton,
    )

    import robot_radio.testgui.__main__ as gui_main
    from robot_radio.config import robot_config as rc_mod
    from robot_radio.testgui import sim_prefs
    from robot_radio.testgui import transport as transport_mod

    baked_cfg = tmp_path / "baked_tovez_nocal.json"
    baked_cfg.write_text(json.dumps({
        "schema_version": 2,
        "identity": {"robot_name": "tovez-nocal-baked", "uid": "tovez-nocal-baked"},
        "connection": {"device_announcement_name": "tovez"},
        "geometry": {"trackwidth": 128},
        "calibration": {},
    }))
    monkeypatch.setenv("ROBOT_CONFIG", str(baked_cfg))
    rc_mod._reset_robot_config()

    monkeypatch.setattr(sim_prefs, "_PREFS_DIR", tmp_path)
    monkeypatch.setattr(sim_prefs, "_PREFS_PATH", tmp_path / "sim_error_profile.json")

    # The subclass must be named exactly "SimTransport": operations.
    # is_sim_transport() duck-checks type(t).__name__, and _set_origin's
    # plant-teleport branch hangs off that check (same requirement
    # test_goto.py/test_tour1_geometry.py document).
    _RealSimTransport = transport_mod.SimTransport
    created: list = []

    class SimTransport(_RealSimTransport):
        def __init__(self) -> None:
            super().__init__()
            created.append(self)

    monkeypatch.setattr(transport_mod, "SimTransport", SimTransport)

    window, _app = gui_main._build_main_window()

    combo = window.findChild(QComboBox, "transport_combo")
    assert combo is not None
    combo.setCurrentText("Sim")

    connect_btn = window.findChild(QPushButton, "connect_btn")
    assert connect_btn is not None
    connect_btn.click()
    _spin_events(qapp, 0.3)

    assert created, "Connect did not construct a SimTransport"
    transport = created[-1]
    assert transport._connected, "SimTransport failed to connect -- is the sim lib built?"

    for name, value in _ZERO_ERROR_SPINS.items():
        spin = window.findChild(QDoubleSpinBox, name)
        assert spin is not None, f"Sim Errors spinbox {name!r} not found"
        spin.setValue(value)
    apply_btn = window.findChild(QPushButton, "sim_errors_apply_btn")
    assert apply_btn is not None
    apply_btn.click()
    _spin_events(qapp, 0.3)

    transport.set_true_pose(0.0, 0.0, 0.0)
    _spin_events(qapp, 0.2)

    return window, transport


def _disconnect_real_sim(qapp, window) -> None:
    from PySide6.QtWidgets import QPushButton  # type: ignore[import-untyped]

    from robot_radio.config import robot_config as rc_mod

    disconnect_btn = window.findChild(QPushButton, "disconnect_btn")
    if disconnect_btn is not None and disconnect_btn.isEnabled():
        disconnect_btn.click()
        _spin_events(qapp, 0.3)
    window.hide()
    rc_mod._reset_robot_config()


def _wait_mode_idle(transport, fused: dict, timeout_s: float) -> bool:
    """Fire-and-forget SNAP-poll until fused["mode"] == 'I'.

    Mirrors ``_TourRunner._wait_for_idle``'s rationale: a SNAP reply is a
    corr-id-less TLM frame, so ``command("SNAP")`` never resolves --
    ``send("SNAP")`` + a spy on ``on_telemetry`` is the only reliable path.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            transport.send("SNAP")
        except Exception:  # noqa: BLE001
            return False
        time.sleep(0.05)
        if (fused.get("mode") or "").upper() == "I":
            return True
    return False


def _read_fresh_pose(transport, fused: dict, settle_s: float) -> "tuple[int, int, int] | None":
    """Poll SNAP for ``settle_s`` seconds and return the last observed fused pose."""
    deadline = time.monotonic() + settle_s
    while time.monotonic() < deadline:
        try:
            transport.send("SNAP")
        except Exception:  # noqa: BLE001
            break
        time.sleep(0.05)
    return fused.get("pose")


@_requires_sim_lib
def test_set_origin_button_resets_fused_pose_to_world_origin_against_real_sim(
    qapp, monkeypatch, tmp_path
) -> None:
    """After actually driving away from world origin (a straight leg + a
    turn, so the raw OTOS chip's own heading state is nonzero too -- not
    just the fused EKF pose), clicking "Set Robot @ 0,0" resets the
    firmware's fused pose back to (0, 0, 0 deg)."""
    from PySide6.QtWidgets import QPushButton

    window, transport = _connect_real_sim(qapp, monkeypatch, tmp_path)
    try:
        sync_btn = window.findChild(QPushButton, "ops_btn_sync_pose")
        assert sync_btn is not None
        assert not sync_btn.isEnabled(), (
            "Sync Pose must be disabled in Sim mode (no camera)"
        )

        fused: dict = {"pose": None, "mode": None}
        gui_tlm_cb = transport.on_telemetry

        def _tlm_spy(frame) -> None:
            if getattr(frame, "pose", None) is not None:
                fused["pose"] = frame.pose
            if getattr(frame, "mode", None) is not None:
                fused["mode"] = frame.mode
            if gui_tlm_cb is not None:
                gui_tlm_cb(frame)

        transport.on_telemetry = _tlm_spy

        transport.command("D 200 200 500", read_timeout=1000)
        assert _wait_mode_idle(transport, fused, 8.0), "D never reached mode=I"
        transport.command("RT 4500", read_timeout=1000)
        assert _wait_mode_idle(transport, fused, 8.0), "RT never reached mode=I"

        away_pose = fused["pose"]
        assert away_pose is not None, "no fused pose (TLM pose=) observed after driving"
        away_x, away_y, _away_h = away_pose
        away_dist = (away_x ** 2 + away_y ** 2) ** 0.5
        assert away_dist > 100, (
            f"sanity check failed -- robot did not actually move away from "
            f"origin (fused pose {away_pose})"
        )

        origin_btn = window.findChild(QPushButton, "ops_btn_set_origin")
        assert origin_btn is not None
        origin_btn.click()
        _spin_events(qapp, 0.3)

        final_pose = _read_fresh_pose(transport, fused, 1.0)
        assert final_pose is not None
        fx, fy, fh = final_pose
        dist = (fx ** 2 + fy ** 2) ** 0.5
        # Tolerance: SI/OZ set the pose directly (no convergence loop), but a
        # small residual (measured ~10-25 mm, ~1-3 deg) remains from the EKF's
        # continued OTOS fusion during the settle window after the plant
        # teleport -- the same order of magnitude as this sprint's other
        # real-sim tolerances (test_tour1_geometry.py: 300 mm; test_goto.py:
        # 150 mm slack), just much tighter since no physical drive is involved.
        assert dist <= 60, (
            f"Set-Origin's fused pose ended {dist:.1f} mm from world origin "
            f"(pose={final_pose}) -- expected ~(0,0,0)"
        )
        assert abs(fh) <= 600, (  # cdeg -> within 6 degrees
            f"Set-Origin's fused heading ended {fh / 100.0:.1f} deg from 0 "
            f"(pose={final_pose})"
        )
    finally:
        _disconnect_real_sim(qapp, window)
