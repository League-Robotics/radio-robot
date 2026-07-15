"""tests/testgui/test_calibration_push_on_connect.py -- ticket 085-005:
connect-time calibration push verification. Ported from
``tests_old/testgui/test_calibration_push_on_connect.py``.

Stakeholder contract (2026-07-03): opening a robot in the TestGUI must send
that robot's calibration values to the robot -- what DefaultConfig.cpp baked
in at compile time must not matter. In Sim mode the firmware bakes tovez's
calibration (rotationalSlip=0.92, trackwidth=128); selecting the
uncalibrated "tovez nocal" must therefore push neutral values over it
(``SET rotSlip=0``), so a nocal robot at zero sim errors runs geometry-pure.

**Real bug found and fixed by this ticket:** ``calibration_commands()``
(``host/robot_radio/calibration/push.py``) used to also push
``SET odomOffX=``/``odomOffY=``/``odomYaw=`` whenever
``config.geometry.odometry_offset_mm`` was non-zero -- which it is for BOTH
real robot configs (``data/robots/tovez.json`` and ``tovez_nocal.json``,
``x=-47.7``). ``config_commands.cpp``'s registered `SET` key table
(architecture-update.md (084) Decision 2's closed 15-key surface) does not
include those keys, so every Connect with either real config hit
``ERR badkey`` (silently tolerated by the push loop's own ``ERR``-counting,
but a real rejection nonetheless -- found during tickets 085-002/003's
manual runs, flagged for this ticket). Fixed by dropping the odom-offset
push entirely (Option (a) from this ticket's brief): the OTOS lever-arm has
no real hardware driver in this program, and OTOS pose is otherwise
configured entirely via ``OI``/``OL``/``OA``/``OV``, never `SET` -- so the
push was dead weight, not a feature to preserve. See
``calibration_commands()``'s own docstring for the full rationale.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_calibration_push_on_connect.py -v
"""
from __future__ import annotations

import json
import pathlib
import time
import types

import pytest

from robot_radio.testgui.transport import _sim_lib_path

_REPO = pathlib.Path(__file__).resolve().parents[2]

_requires_sim_lib = pytest.mark.skipif(
    not _sim_lib_path().exists(),
    reason="sim lib not built -- run `just build-sim` first",
)


# ---------------------------------------------------------------------------
# Qt-free: calibration_commands() pure-function coverage.
# ---------------------------------------------------------------------------


def _cfg(*, calibration=None, trackwidth=128, odometry_offset_mm=None, robot_name="r"):
    return types.SimpleNamespace(
        robot_name=robot_name,
        calibration=calibration if calibration is not None else types.SimpleNamespace(),
        geometry=types.SimpleNamespace(
            trackwidth=trackwidth, odometry_offset_mm=odometry_offset_mm
        ),
        wheels=types.SimpleNamespace(wheel_diameter_mm=80.77),
    )


def test_calibration_commands_excludes_odom_offset_keys_even_when_nonzero() -> None:
    """The fix under test: a non-zero odometry_offset_mm (both real robot
    configs have one, x=-47.7) must never produce a SET odomOff*/odomYaw*
    command -- those keys are unregistered and get ERR badkey."""
    from robot_radio.calibration.push import calibration_commands

    cfg = _cfg(
        odometry_offset_mm=types.SimpleNamespace(x=-47.7, y=3.5, yaw_rad=0.0)
    )

    cmds = calibration_commands(cfg)

    joined = " ".join(c for c, _t in cmds)
    assert "odomOff" not in joined and "odomYaw" not in joined, (
        f"calibration_commands must never push the unregistered odomOff*/"
        f"odomYaw SET keys (ERR badkey on the current firmware): {cmds}"
    )


def test_calibration_commands_nocal_pushes_rotslip_zero_sentinel() -> None:
    from robot_radio.calibration.push import calibration_commands

    cmds = calibration_commands(_cfg())

    assert ("SET rotSlip=0", 200) in cmds


def test_calibration_commands_calibrated_pushes_actual_rotslip() -> None:
    from robot_radio.calibration.push import calibration_commands

    cfg = _cfg(calibration=types.SimpleNamespace(rotational_slip=0.85))
    cmds = calibration_commands(cfg)

    assert ("SET rotSlip=0.85", 200) in cmds


def test_calibration_commands_always_includes_oi_ol_oa() -> None:
    from robot_radio.calibration.push import calibration_commands

    cmds = calibration_commands(_cfg())
    verbs = [c.split()[0] for c, _t in cmds]

    assert "OI" in verbs
    assert any(c.startswith("OL ") for c, _t in cmds)
    assert any(c.startswith("OA ") for c, _t in cmds)


# ---------------------------------------------------------------------------
# Push-loop resilience: NODEV-tolerant, ERR-tolerant -- inline
# reimplementation of __main__.py's _push_robot_calibration inner loop
# (~line 1616-1638; the transport-is-None/cfg-is-None guards above it are
# not this function's concern and are not reimplemented here).
# ---------------------------------------------------------------------------


class _ScriptedReplyTransport:
    """Records every command() line; returns a scripted reply per line prefix."""

    def __init__(self, reply_map: "dict[str, str] | None" = None) -> None:
        self.reply_map = reply_map or {}
        self.commands_sent: list[str] = []

    def command(self, line: str, read_timeout: int = 200) -> str:  # [ms]
        self.commands_sent.append(line)
        for prefix, reply in self.reply_map.items():
            if line.startswith(prefix):
                return reply
        return "OK"


def _push_calibration_loop(transport, cfg, append_log):
    """Line-for-line reimplementation of __main__.py's push loop."""
    from robot_radio.calibration.push import calibration_commands

    cmds = calibration_commands(cfg)
    n_bad = 0
    n_nodev = 0
    for cmd, read_timeout in cmds:
        reply = transport.command(cmd, read_timeout=read_timeout)
        upper = (reply or "").upper()
        if "NODEV" in upper:
            n_nodev += 1
        elif "ERR" in upper:
            n_bad += 1
            append_log(f"[CAL] {cmd!r} rejected: {(reply or '').strip()}")
    append_log(
        f"[CAL] pushed {len(cmds) - n_bad - n_nodev}/{len(cmds)} "
        f"calibration values from robot '{cfg.robot_name}'"
        + (f" ({n_nodev} device cmds skipped: no device)" if n_nodev else "")
        + (f" ({n_bad} REJECTED)" if n_bad else "")
    )
    return cmds, n_bad, n_nodev


def test_push_loop_tolerates_nodev_reply_and_continues_all_commands() -> None:
    """A NODEV reply on OI (no OTOS device -- normal for real hardware
    without the sim's model) must not abort the loop -- every remaining
    command is still sent, and NODEV is not counted as a rejection."""
    cfg = _cfg(robot_name="tovez nocal")
    transport = _ScriptedReplyTransport({"OI": "ERR nodev"})
    log: list[str] = []

    cmds, n_bad, n_nodev = _push_calibration_loop(transport, cfg, log.append)

    assert transport.commands_sent == [c for c, _t in cmds], (
        "every command must still be sent even after a NODEV reply"
    )
    assert n_nodev == 1
    assert n_bad == 0
    assert any("device cmds skipped: no device" in line for line in log)
    assert not any("REJECTED" in line for line in log)


def test_push_loop_logs_and_continues_past_a_genuine_err_reply() -> None:
    """A genuine (non-NODEV) ERR reply is counted, logged, and does not
    abort the remaining commands."""
    cfg = _cfg(robot_name="tovez nocal")
    transport = _ScriptedReplyTransport({"SET tw=": "ERR badval tw=0"})
    log: list[str] = []

    cmds, n_bad, n_nodev = _push_calibration_loop(transport, cfg, log.append)

    assert transport.commands_sent == [c for c, _t in cmds]
    assert n_bad == 1
    assert any("rejected" in line for line in log)
    assert any("REJECTED" in line for line in log)


# ---------------------------------------------------------------------------
# Real GUI + real ctypes firmware sim.
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def qapp():
    # 107-004: turn a missing `gui` dependency group into a clean skip, not
    # a hard collection/run error -- see test_tour1_geometry.py's module
    # docstring for the full rationale (tests/testgui/ re-added to
    # pyproject.toml's testpaths this ticket).
    pytest.importorskip("PySide6")
    import sys

    from PySide6.QtWidgets import QApplication  # type: ignore[import-untyped]

    app = QApplication.instance() or QApplication(sys.argv)
    yield app


def _spin_events(qapp, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        qapp.processEvents()
        time.sleep(0.01)


def _connect_gui_with_config(qapp, monkeypatch, tmp_path, cfg):
    """Pin *cfg* (a dict, written to a temp file, or a real Path) as the
    active robot, build the GUI, click Connect. Returns (window, transport).
    Caller must disconnect + hide.
    """
    from PySide6.QtWidgets import QComboBox, QPushButton  # type: ignore[import-untyped]

    import robot_radio.testgui.__main__ as gui_main
    from robot_radio.config import robot_config as rc_mod
    from robot_radio.testgui import sim_prefs
    from robot_radio.testgui import transport as transport_mod

    if isinstance(cfg, dict):
        cfg_path = tmp_path / "active.json"
        cfg_path.write_text(json.dumps(cfg))
    else:
        cfg_path = cfg

    monkeypatch.setenv("ROBOT_CONFIG", str(cfg_path))
    rc_mod._reset_robot_config()

    monkeypatch.setattr(sim_prefs, "_PREFS_DIR", tmp_path)
    monkeypatch.setattr(
        sim_prefs, "_PREFS_PATH", tmp_path / "sim_error_profile.json"
    )

    _RealSimTransport = transport_mod.SimTransport
    created: list = []

    class SimTransport(_RealSimTransport):  # noqa: N801 -- name-checked by is_sim_transport
        def __init__(self) -> None:
            super().__init__()
            created.append(self)

    monkeypatch.setattr(transport_mod, "SimTransport", SimTransport)

    window, _app = gui_main._build_main_window()
    combo = window.findChild(QComboBox, "transport_combo")
    combo.setCurrentText("Sim")
    connect_btn = window.findChild(QPushButton, "connect_btn")
    connect_btn.click()

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        qapp.processEvents()
        time.sleep(0.01)
        if created and created[-1]._connected:
            break

    assert created and created[-1]._connected, "SimTransport failed to connect"
    return window, created[-1]


def _teardown(qapp, window) -> None:
    from PySide6.QtWidgets import QPushButton  # type: ignore[import-untyped]

    from robot_radio.config import robot_config as rc_mod

    btn = window.findChild(QPushButton, "disconnect_btn")
    if btn is not None and btn.isEnabled():
        btn.click()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            qapp.processEvents()
            time.sleep(0.01)
    window.hide()
    rc_mod._reset_robot_config()


def _log_text(window) -> str:
    from PySide6.QtWidgets import QPlainTextEdit  # type: ignore[import-untyped]

    pane = window.findChild(QPlainTextEdit, "log_pane")
    assert pane is not None, "log_pane not found"
    return pane.toPlainText()


def _nocal_config() -> dict:
    return {
        "schema_version": 2,
        "identity": {"robot_name": "tovez nocal", "uid": "tovez-nocal"},
        "connection": {"device_announcement_name": "tovez"},
        "geometry": {"trackwidth": 128},
        "wheels": {"wheel_diameter_mm": 80.77},
    }


@_requires_sim_lib
def test_connect_pushes_nocal_neutral_calibration_into_firmware(
    qapp, monkeypatch, tmp_path
) -> None:
    """Connect with an uncalibrated active robot -> the firmware's baked
    rotationalSlip=0.92 is overwritten with the neutral sentinel 0."""
    window, transport = _connect_gui_with_config(
        qapp, monkeypatch, tmp_path, _nocal_config()
    )
    try:
        reply = transport.command("GET rotSlip", read_timeout=500)
        assert "rotSlip=0.000" in reply, (
            f"nocal connect must neutralize the baked rotationalSlip; "
            f"firmware reports {reply.strip()!r}"
        )
        assert "tw=128" in transport.command("GET tw", read_timeout=500)
    finally:
        _teardown(qapp, window)


@_requires_sim_lib
def test_connect_pushes_calibrated_values_into_firmware(
    qapp, monkeypatch, tmp_path
) -> None:
    """Connect with a calibrated active robot -> its values land verbatim
    (here deliberately different from the baked 0.92 to prove overwrite)."""
    cfg = _nocal_config()
    cfg["identity"] = {"robot_name": "tovez-custom", "uid": "tovez-custom"}
    cfg["calibration"] = {"rotational_slip": 0.85}
    window, transport = _connect_gui_with_config(qapp, monkeypatch, tmp_path, cfg)
    try:
        reply = transport.command("GET rotSlip", read_timeout=500)
        assert "rotSlip=0.850" in reply, (
            f"connect must push the active robot's calibration; firmware "
            f"reports {reply.strip()!r}"
        )
    finally:
        _teardown(qapp, window)


@_requires_sim_lib
def test_connect_with_real_tovez_nocal_config_does_not_hit_badkey_on_odom_offset(
    qapp, monkeypatch, tmp_path
) -> None:
    """Regression for this ticket's finding: the REAL
    ``data/robots/tovez_nocal.json`` carries a non-zero
    ``geometry.odometry_offset_mm`` (x=-47.7) -- before the fix, Connect's
    calibration push sent ``SET odomOffX=-47.700`` and got ``ERR badkey``
    from the current firmware/sim."""
    real_cfg_path = _REPO / "data" / "robots" / "tovez_nocal.json"
    assert real_cfg_path.exists(), f"missing {real_cfg_path}"

    window, transport = _connect_gui_with_config(
        qapp, monkeypatch, tmp_path, real_cfg_path
    )
    try:
        log_text = _log_text(window)
        assert "badkey" not in log_text.lower(), (
            f"calibration push must not hit ERR badkey on Connect:\n{log_text}"
        )
        assert "REJECTED" not in log_text, (
            f"calibration push had rejected commands:\n{log_text}"
        )
        assert "rotSlip=0.000" in transport.command("GET rotSlip", read_timeout=500)
    finally:
        _teardown(qapp, window)


@_requires_sim_lib
def test_robot_combo_change_while_connected_repushes_and_overwrites(
    qapp, monkeypatch, tmp_path
) -> None:
    """Switching the Robot combo while connected re-triggers the calibration
    push and overwrites the previously-active robot's firmware values.

    Exercises the REAL robot_combo / list_robots() / set_active_robot()
    wiring against the real data/robots/{tovez_nocal,tovez}.json files (both
    carry the same non-zero odometry_offset_mm this ticket's fix addresses).
    ``ROBOT_CONFIG`` is deliberately left UNSET here -- it would override
    robot_combo selection entirely, per get_robot_config()'s own documented
    resolution order (env var wins over the active_robot.json pointer).
    Temporarily rewrites (and restores in `finally`) the real
    data/robots/active_robot.json pointer file.
    """
    from PySide6.QtWidgets import QComboBox, QPushButton  # type: ignore[import-untyped]

    import robot_radio.testgui.__main__ as gui_main
    from robot_radio.config import robot_config as rc_mod
    from robot_radio.testgui import sim_prefs
    from robot_radio.testgui import transport as transport_mod

    active_pointer = _REPO / "data" / "robots" / "active_robot.json"
    original_bytes = active_pointer.read_bytes() if active_pointer.exists() else None

    monkeypatch.delenv("ROBOT_CONFIG", raising=False)
    monkeypatch.setattr(sim_prefs, "_PREFS_DIR", tmp_path)
    monkeypatch.setattr(
        sim_prefs, "_PREFS_PATH", tmp_path / "sim_error_profile.json"
    )

    _RealSimTransport = transport_mod.SimTransport
    created: list = []

    class SimTransport(_RealSimTransport):  # noqa: N801 -- name-checked by is_sim_transport
        def __init__(self) -> None:
            super().__init__()
            created.append(self)

    monkeypatch.setattr(transport_mod, "SimTransport", SimTransport)

    rc_mod._reset_robot_config()
    window, _app = gui_main._build_main_window()
    try:
        robot_combo = window.findChild(QComboBox, "robot_combo")
        assert robot_combo is not None

        nocal_idx = robot_combo.findText("tovez nocal")
        cal_idx = robot_combo.findText("tovez")
        assert nocal_idx >= 0 and cal_idx >= 0, (
            f"expected both 'tovez nocal' and 'tovez' in the combo, got "
            f"{[robot_combo.itemText(i) for i in range(robot_combo.count())]}"
        )

        robot_combo.setCurrentIndex(nocal_idx)
        _spin_events(qapp, 0.1)

        transport_combo = window.findChild(QComboBox, "transport_combo")
        assert transport_combo is not None
        transport_combo.setCurrentText("Sim")
        connect_btn = window.findChild(QPushButton, "connect_btn")
        assert connect_btn is not None
        connect_btn.click()
        _spin_events(qapp, 0.3)

        assert created, "Connect did not construct a SimTransport"
        transport = created[-1]
        assert transport._connected, "SimTransport failed to connect"

        assert "rotSlip=0.000" in transport.command("GET rotSlip", read_timeout=500), (
            "connecting with 'tovez nocal' active must push the neutral sentinel"
        )

        robot_combo.setCurrentIndex(cal_idx)
        _spin_events(qapp, 0.3)

        assert "rotSlip=0.920" in transport.command("GET rotSlip", read_timeout=500), (
            "switching to the calibrated 'tovez' robot while connected must "
            "re-push and overwrite the firmware's rotSlip"
        )

        log_text = _log_text(window)
        assert "badkey" not in log_text.lower(), (
            f"robot-combo re-push must not hit ERR badkey:\n{log_text}"
        )
    finally:
        disconnect_btn = window.findChild(QPushButton, "disconnect_btn")
        if disconnect_btn is not None and disconnect_btn.isEnabled():
            disconnect_btn.click()
            _spin_events(qapp, 0.3)
        window.hide()
        rc_mod._reset_robot_config()
        if original_bytes is not None:
            active_pointer.write_bytes(original_bytes)
        else:
            active_pointer.unlink(missing_ok=True)
