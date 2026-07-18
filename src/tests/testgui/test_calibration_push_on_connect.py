"""src/tests/testgui/test_calibration_push_on_connect.py -- ticket 085-005:
connect-time calibration push verification. Ported from
``tests_old/testgui/test_calibration_push_on_connect.py``.

Stakeholder contract (2026-07-03): opening a robot in the TestGUI must send
that robot's calibration values to the robot -- what DefaultConfig.cpp baked
in at compile time must not matter. In Sim mode the firmware bakes tovez's
calibration (rotationalSlip=0.92, trackwidth=128); selecting the
uncalibrated "tovez nocal" must therefore push neutral values over it
(``SET rotSlip=0``), so a nocal robot at zero sim errors runs geometry-pure.

**Real bug found and fixed by this ticket:** ``calibration_commands()``
(``src/host/robot_radio/calibration/push.py``) used to also push
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

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_calibration_push_on_connect.py -v
"""
from __future__ import annotations

import json
import pathlib
import time
import types

import pytest

from robot_radio.testgui.transport import _sim_lib_path

_REPO = pathlib.Path(__file__).resolve().parents[3]

_requires_sim_lib = pytest.mark.skipif(
    not _sim_lib_path().exists(),
    reason="sim lib not built -- run `just build-sim` first",
)

# 109-002: UN-SKIPPED. 108-007 left SimTransport.send()/command() routing
# no SET/GET at all; 109-002 gave SimTransport a real config path -- typed
# ConfigDelta patches constructed via the SAME NezhaProtocol.config()
# hardware transports use, injected via SimLoop.inject_command() (see
# transport.py's _SimConfigConn/_handle_config_set()/_handle_config_get()).
#
# Architecture Revision 1 (sprint.md, this ticket's sprint) found a THIRD,
# deeper fact along the way: RobotLoop::handleConfig() only ever applied
# MotorConfigPatch (pid.*/ml/mr) -- every other ConfigDelta patch kind
# (DrivetrainConfigPatch, PlannerConfigPatch, the watchdog arm) replies
# ERR_UNIMPLEMENTED unconditionally, a documented scope boundary
# (src/firm/app/DESIGN.md §3), not something 109-002 could fix (it is
# host-only scope). Concretely: `rotSlip`/`tw` (DrivetrainConfigPatch) have
# NO firmware consumer on any transport this sprint, so "GET rotSlip
# reflects a pushed value" can never legitimately pass again -- the four
# tests below were revised (not simply un-skipped) per Architecture
# Revision 1's explicit direction: retarget the round-trip assertion onto
# `ml` (MotorConfigPatch.travel_calib -- a key calibration_commands()
# ALSO pushes, and one with a real firmware consumer), which preserves
# each test's actual intent ("Connect pushes this robot's calibration into
# firmware, overwriting whatever was there") without asserting something
# structurally impossible. `rotSlip`/`tw` still get exercised -- each test
# now asserts they get the honest, immediate "unsupported" error (no wire
# round trip, no silent no-op, no fabricated value), which is itself
# meaningful coverage of Architecture Revision 1's own decision.


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


def test_calibration_commands_pushes_oi_ol_oa_unconditionally() -> None:
    """109-004 RESTORES the OI/OL/OA push (dropped 2026-07-16 when these
    verbs had no path over the binary wire at all -- see this module's own
    docstring / calibration_commands()'s own docstring for the full
    restoration rationale). All three are pushed unconditionally -- OI
    (chip init) always, and OL/OA with the SAME "uncalibrated -> neutral
    sentinel" discipline rotSlip already uses: a bare _cfg() with no
    otos_linear_scale/otos_angular_scale calibration still pushes the 1.0
    (no-correction) default, encoded as ``OL 0``/``OA 0``, not omitted."""
    from robot_radio.calibration.push import calibration_commands

    cmds = calibration_commands(_cfg())

    assert ("OI", 500) in cmds
    assert ("OL 0", 200) in cmds
    assert ("OA 0", 200) in cmds
    verbs = [c.split()[0] for c, _t in cmds]
    assert verbs.index("OI") < verbs.index("OL") < verbs.index("OA")


def test_calibration_commands_pushes_pid_and_heading_gains_when_present() -> None:
    """Stakeholder 2026-07-18: the control gains live in the robot JSON and
    must ride the same connect-time push as the geometry calibration --
    ``control.vel_*`` -> ``SET pid.*`` (MotorConfigPatch, both motors) and
    ``control.heading_*`` -> ``SET headingKp/headingKd``
    (PlannerConfigPatch). Values formatted ``:g`` like rotSlip."""
    from robot_radio.calibration.push import calibration_commands

    cfg = _cfg()
    cfg.control = types.SimpleNamespace(
        vel_kp=0.002, vel_ki=0.0, vel_kff=0.0, vel_imax=0.0, vel_kaw=0.0,
        heading_kp=1.0, heading_kd=0.0)

    cmds = calibration_commands(cfg)

    for expected in (
        "SET pid.kp=0.002", "SET pid.ki=0", "SET pid.kff=0",
        "SET pid.iMax=0", "SET pid.kaw=0",
        "SET headingKp=1", "SET headingKd=0",
    ):
        assert (expected, 200) in cmds, f"missing {expected!r} in {cmds}"


def test_calibration_commands_omits_pid_gains_when_config_has_none() -> None:
    """A config with no ``control`` section (or all-None fields) pushes no
    gain keys at all -- ``ControlConfig``'s documented contract is
    "None -> the firmware boot default is kept", NOT a zero-sentinel push
    like rotSlip's."""
    from robot_radio.calibration.push import calibration_commands

    cmds = calibration_commands(_cfg())

    joined = " ".join(c for c, _t in cmds)
    assert "pid." not in joined and "headingK" not in joined, cmds


def test_real_tovez_nocal_json_pushes_neutral_gains_via_real_model() -> None:
    """End-to-end through the REAL pydantic model: data/robots/
    tovez_nocal.json carries the neutral baseline (stakeholder 2026-07-18:
    ki/kd = 0, vanilla kp; kff = 0.002 = 1/500 -- the vanilla inverse-plant
    slope, kept non-zero because it IS the open-loop law the PID checkbox's
    disabled state drives, see nezha_motor.h's dispatch bullet) and
    ``load_robot_config()`` + ``calibration_commands()`` actually read it
    from there. This is the test that catches a JSON key the model silently
    drops (heading_kp/heading_kd were not ControlConfig fields before this
    change)."""
    from robot_radio.calibration.push import calibration_commands
    from robot_radio.config.robot_config import load_robot_config

    cfg_path = _REPO / "data" / "robots" / "tovez_nocal.json"
    assert cfg_path.exists(), f"missing {cfg_path}"
    cfg = load_robot_config(cfg_path)

    cmds = calibration_commands(cfg)

    for expected in (
        "SET pid.kp=0.002", "SET pid.ki=0", "SET pid.kff=0.002",
        "SET pid.iMax=0", "SET pid.kaw=0",
        "SET headingKp=1", "SET headingKd=0",
    ):
        assert (expected, 200) in cmds, f"missing {expected!r} in {cmds}"


def test_calibration_commands_pushes_encoded_otos_scale() -> None:
    """OL/OA carry the chip's RAW int8 register scalar (scale_to_int8()),
    not the raw multiplier -- e.g. otos_linear_scale=1.027 -> ``OL 27``."""
    from robot_radio.calibration.push import calibration_commands

    cfg = _cfg(calibration=types.SimpleNamespace(
        otos_linear_scale=1.027, otos_angular_scale=0.987))
    cmds = calibration_commands(cfg)

    assert ("OI", 500) in cmds
    assert ("OL 27", 200) in cmds
    assert ("OA -13", 200) in cmds
    # OI precedes OL/OA (chip init must run before the scale writes).
    verbs = [c.split()[0] for c, _t in cmds]
    assert verbs.index("OI") < verbs.index("OL")
    assert verbs.index("OI") < verbs.index("OA")


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
    """A NODEV reply on any command must not abort the loop -- every remaining
    command is still sent, and NODEV is not counted as a rejection. (109-004:
    OI/OL/OA are pushed again and have a real firmware consumer now, so they
    no longer produce NODEV on their own -- this scripts the NODEV onto a
    still-sent SET command instead, to keep exercising the loop's
    resilience.)"""
    cfg = _cfg(robot_name="tovez nocal")
    transport = _ScriptedReplyTransport({"SET rotSlip": "ERR nodev"})
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
    # docstring for the full rationale (src/tests/testgui/ re-added to
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


def _expected_ml(wheel_diameter_mm: float = 80.77) -> str:
    """The default ``ml``/``mr`` wheel-travel-calib ``calibration_commands()``
    pushes when a config carries no ``mm_per_wheel_deg_left/right`` override
    (``push.py``'s own ``default_wheel_travel_calib = math.pi*wd/360.0``),
    formatted the SAME way ``SimTransport._handle_config_set()`` echoes it
    back (``protocol._format_config_value()``'s ``:.6g``) -- so tests assert
    against the real formatting path, never a hand-duplicated one."""
    import math

    from robot_radio.robot import protocol

    return protocol._format_config_value(math.pi * wheel_diameter_mm / 360.0)


_UNSUPPORTED_ERR_PREFIX = "ERR unsupported"


@_requires_sim_lib
def test_connect_pushes_nocal_neutral_calibration_into_firmware(
    qapp, monkeypatch, tmp_path
) -> None:
    """Connect with an uncalibrated active robot.

    109-002 retarget (Architecture Revision 1): ``rotSlip``/``tw``
    (DrivetrainConfigPatch) have no live firmware consumer on ANY transport
    this sprint (RobotLoop::handleConfig only applies MotorConfigPatch) --
    asserting they "reflect a pushed value" is no longer legal. Both now
    assert the honest, immediate "unsupported" error instead (no wire round
    trip attempted, no silent no-op). ``ml`` (MotorConfigPatch.travel_calib,
    a key calibration_commands() ALSO pushes and one with a real firmware
    consumer) carries the "connect pushes calibration into firmware" intent
    that ``rotSlip`` used to: the nocal config has no
    ``mm_per_wheel_deg_left`` override, so its push lands the wheel-diameter
    -derived default.
    """
    window, transport = _connect_gui_with_config(
        qapp, monkeypatch, tmp_path, _nocal_config()
    )
    try:
        rot_reply = transport.command("GET rotSlip", read_timeout=500)
        assert rot_reply.startswith(_UNSUPPORTED_ERR_PREFIX), (
            f"rotSlip has no firmware consumer this sprint -- GET must be an "
            f"honest unsupported error, not a fabricated value: {rot_reply!r}"
        )
        tw_reply = transport.command("GET tw", read_timeout=500)
        assert tw_reply.startswith(_UNSUPPORTED_ERR_PREFIX), (
            f"tw has no firmware consumer this sprint -- GET must be an "
            f"honest unsupported error, not a fabricated value: {tw_reply!r}"
        )
        ml_reply = transport.command("GET ml", read_timeout=500)
        assert f"ml={_expected_ml()}" in ml_reply, (
            f"nocal connect must push the wheel-diameter-derived default "
            f"travel calib via the real MotorConfigPatch consumer; firmware "
            f"reports {ml_reply.strip()!r}"
        )
    finally:
        _teardown(qapp, window)


@_requires_sim_lib
def test_connect_pushes_calibrated_values_into_firmware(
    qapp, monkeypatch, tmp_path
) -> None:
    """Connect with a calibrated active robot -> its values land verbatim.

    109-002 retarget: was ``rotational_slip=0.85`` (no firmware consumer,
    see module docstring); now ``mm_per_wheel_deg_left`` (MotorConfigPatch,
    a real consumer), deliberately different from the nocal default to
    prove overwrite.
    """
    cfg = _nocal_config()
    cfg["identity"] = {"robot_name": "tovez-custom", "uid": "tovez-custom"}
    cfg["calibration"] = {"mm_per_wheel_deg_left": 0.5}
    window, transport = _connect_gui_with_config(qapp, monkeypatch, tmp_path, cfg)
    try:
        reply = transport.command("GET ml", read_timeout=500)
        assert "ml=0.5" in reply, (
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
    from the current firmware/sim.

    109-002: the blanket "no REJECTED at all" assertion no longer holds --
    ``rotSlip``/``tw`` NOW legitimately get rejected every Connect (the
    honest unsupported-key error, Architecture Revision 1), which is
    correct, documented behavior, not a regression. What still must never
    happen is a REJECTED entry for anything OTHER than those two known-
    unsupported keys (in particular, no ``badkey`` at all -- the odom-offset
    bug this test guards against).
    """
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
        rejected_lines = [
            line for line in log_text.splitlines() if "rejected:" in line.lower()
        ]
        unexpected_rejections = [
            line for line in rejected_lines
            if "rotSlip" not in line and "tw=" not in line
        ]
        assert not unexpected_rejections, (
            f"only rotSlip/tw (known-unsupported this sprint) may be "
            f"rejected; found other rejections:\n{unexpected_rejections}"
        )
        rot_reply = transport.command("GET rotSlip", read_timeout=500)
        assert rot_reply.startswith(_UNSUPPORTED_ERR_PREFIX), (
            f"rotSlip has no firmware consumer this sprint: {rot_reply!r}"
        )
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

        # 109-002 retarget (Architecture Revision 1): rotSlip has no
        # firmware consumer this sprint -- ml (MotorConfigPatch.
        # travel_calib, a real consumer) carries the "combo switch
        # re-pushes and overwrites" intent instead. tovez_nocal.json has no
        # mm_per_wheel_deg_left override (wheel-diameter-derived default,
        # see _expected_ml()); tovez.json's calibration carries
        # mm_per_wheel_deg_left=0.7165 -- a genuinely different value,
        # proving the re-push overwrote it.
        assert f"ml={_expected_ml()}" in transport.command(
            "GET ml", read_timeout=500
        ), "connecting with 'tovez nocal' active must push its default travel calib"

        robot_combo.setCurrentIndex(cal_idx)
        _spin_events(qapp, 0.3)

        assert "ml=0.7165" in transport.command("GET ml", read_timeout=500), (
            "switching to the calibrated 'tovez' robot while connected must "
            "re-push and overwrite the firmware's ml"
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
