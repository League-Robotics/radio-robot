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


def _cfg(*, motors=None, wheel_control=None, otos=None, robot_name="r"):
    """132-014: retargeted onto robot_config.proto's grouped shape
    (config.motors/config.wheel_control/config.otos) -- config.calibration/
    config.geometry.odometry_offset_mm (the pre-020 flat sections this
    helper used to build) no longer exist on RobotConfig."""
    return types.SimpleNamespace(
        robot_name=robot_name,
        motors=motors if motors is not None else types.SimpleNamespace(),
        wheel_control=wheel_control if wheel_control is not None else types.SimpleNamespace(),
        otos=otos if otos is not None else types.SimpleNamespace(),
        wheels=types.SimpleNamespace(wheel_diameter_mm=80.77),
    )


def test_calibration_commands_excludes_odom_offset_keys_even_when_nonzero() -> None:
    """The fix under test (still holds, 132-014): calibration_commands()
    reads only calibration_kwargs() (ml/mr/pid.*) + otos_kwargs()
    (offset_x/y/yaw, linear_scale/angular_scale) -- config.geometry.
    odometry_offset_mm is not, and never was, one of either selector's own
    field names, so it can never leak into a SET odomOff*/odomYaw* command
    (those keys were never a registered wire key on any surface)."""
    from robot_radio.calibration.push import calibration_commands

    cmds = calibration_commands(_cfg())

    joined = " ".join(c for c, _t in cmds)
    assert "odomOff" not in joined and "odomYaw" not in joined, (
        f"calibration_commands must never push the unregistered odomOff*/"
        f"odomYaw SET keys (ERR badkey on the current firmware): {cmds}"
    )


# test_calibration_commands_nocal_pushes_rotslip_zero_sentinel() /
# test_calibration_commands_calibrated_pushes_actual_rotslip() -- DELETED,
# 132-014: rotSlip (GEOMETRY.rotational_slip) is dropped from
# calibration_kwargs()'s own selection (GEOMETRY is boot-only AND one of
# the sim's own justified BootOverrides divergences -- see that function's
# docstring). There is no live wire target left for calibration_commands()
# to conditionally push it onto any more; the "uncalibrated -> neutral
# sentinel" property these two tests proved no longer has anything to
# prove. Not ported as always-fail assertions -- simply removed, matching
# this project's own residual-reference sweep policy for other
# no-longer-selected fields (e.g. 115-003's headingKp/headingKd removal in
# this exact file's git history).


def test_calibration_commands_pushes_oi_ol_oa_unconditionally() -> None:
    """109-004 RESTORES the OI/OL/OA push (dropped 2026-07-16 when these
    verbs had no path over the binary wire at all). OI (chip init) is
    ALWAYS appended (132-014: an inert legacy token on the current wire --
    see calibration_commands()'s own docstring). OL/OA are appended
    whenever config.otos carries linear_scale/angular_scale -- unlike
    rotSlip's old "always sentinel" discipline, otos_kwargs() pushes
    whatever value is actually there (132-014, no synthetic 1.0
    fallback -- see that function's own docstring), formatted as the
    MULTIPLIER directly now, not a scale_to_int8()-encoded register."""
    from robot_radio.calibration.push import calibration_commands

    cfg = _cfg(otos=types.SimpleNamespace(linear_scale=1.0, angular_scale=1.0))
    cmds = calibration_commands(cfg)

    assert ("OI", 500) in cmds
    assert ("OL 1", 200) in cmds
    assert ("OA 1", 200) in cmds
    verbs = [c.split()[0] for c, _t in cmds]
    assert verbs.index("OI") < verbs.index("OL") < verbs.index("OA")


def test_calibration_commands_pushes_pid_gains_when_present() -> None:
    """Stakeholder 2026-07-18: the control gains live in the robot JSON and
    must ride the same connect-time push as the wheel-travel calibration.
    Values formatted ``:g``.

    130-005 (wheel-speed-controller-moves-into-drive.md Phase 3): ``pid.*``
    targets App::Drive's unified wheel-speed controller. 132-014 (patch-
    surface retirement, host migration): the SOURCE moved from
    ``config.control.wheel_pid_*`` to ``config.wheel_control.pid_*``
    (robot_config.proto's WheelControl group, 132-020's grouped
    RobotConfig shape) -- unchanged consumer/field-name mapping otherwise
    (``pid.kff``/``pid.kaw`` still carry Stage B's ``kaff``/``pidMax``, now
    spelled ``pid_kaff``/``pid_max``)."""
    from robot_radio.calibration.push import calibration_commands

    cfg = _cfg(wheel_control=types.SimpleNamespace(
        pid_kp=0.002, pid_ki=0.0, pid_kaff=0.0, pid_i_max=0.0, pid_max=0.0))

    cmds = calibration_commands(cfg)

    for expected in (
        "SET pid.kp=0.002", "SET pid.ki=0", "SET pid.kff=0",
        "SET pid.iMax=0", "SET pid.kaw=0",
    ):
        assert (expected, 200) in cmds, f"missing {expected!r} in {cmds}"


def test_calibration_commands_omits_pid_gains_when_config_has_none() -> None:
    """A config with no ``wheel_control`` section at all pushes no gain
    keys -- unlike a REAL RobotConfig (whose ``.wheel_control`` is never
    None, 132-020's root model), a duck-typed test double CAN omit it, and
    that omission must still degrade gracefully (no exception, no keys)."""
    from robot_radio.calibration.push import calibration_commands

    cfg = _cfg()
    cfg.wheel_control = None

    cmds = calibration_commands(cfg)

    joined = " ".join(c for c, _t in cmds)
    assert "pid." not in joined, cmds


def test_real_tovez_nocal_json_pushes_neutral_gains_via_real_model() -> None:
    """End-to-end through the REAL pydantic model: data/robots/
    tovez_nocal.json and ``load_robot_config()`` + ``calibration_commands()``
    actually read it from there. This is the test that catches a JSON key
    the model silently drops (heading_kp/heading_kd were not ControlConfig
    fields before this change).

    UPDATE (113-003, 2026-07-20): ``heading_kp``/``distance_kp``/
    ``distance_tol``/``actuation_lag``/``model_tau_lin``/``model_tau_ang``
    in ``tovez_nocal.json`` are no longer neutral placeholders -- per the
    JSON's own ``_neutral_note``, they are now the SIM-VALIDATED motion
    values (config-as-truth), replacing the old broken code defaults
    (``heading_kp=6``, ``distance_kp=8``, ``actuation_lag=0.13``).

    UPDATE (115-003, gut-to-minimal-firmware S1 motion-stack excision):
    113-003's own ``minSpeed``/``distanceKp``/``arriveDwell``/``headingKp``/
    ``headingKd`` pushes (``PlannerConfigPatch`` wire keys) are DELETED, not
    ported -- ``PlannerConfigPatch`` itself, and the ``App::Pilot`` that
    applied it, are gone; none of these five keys are valid ``set_config()``
    wire keys any more (see ``calibration_kwargs()``'s own docstring).

    UPDATE (130-005, wheel-speed-controller-moves-into-drive.md Phase 3):
    ``pid.*`` is REPOINTED from ``control.vel_*`` onto
    ``control.wheel_pid_*`` (App::Drive's unified wheel-speed controller).
    ``tovez_nocal.json`` ships every ``wheel_pid_*`` field at 0.0 (ticket
    130-004's own decision: the no-calibration profile ships Stage B fully
    inert, pending ticket 006's bench tuning on hardware) -- every
    ``pid.*`` line below now reads ``0``, not the old ``vel_*`` values
    (``pid.kff`` used to read ``0.002`` from ``control.vel_kff``; it now
    reads Stage B's ``kaff``, which is also 0 in this profile)."""
    from robot_radio.calibration.push import calibration_commands
    from robot_radio.config.robot_config import load_robot_config

    cfg_path = _REPO / "data" / "robots" / "tovez_nocal.json"
    assert cfg_path.exists(), f"missing {cfg_path}"
    cfg = load_robot_config(cfg_path)

    cmds = calibration_commands(cfg)

    for expected in (
        "SET pid.kp=0", "SET pid.ki=0", "SET pid.kff=0",
        "SET pid.iMax=0", "SET pid.kaw=0",
    ):
        assert (expected, 200) in cmds, f"missing {expected!r} in {cmds}"

    joined = " ".join(c for c, _t in cmds)
    for deleted_key in ("headingKp", "headingKd", "minSpeed", "distanceKp", "arriveDwell"):
        assert deleted_key not in joined, (
            f"{deleted_key} must NOT be pushed -- PlannerConfigPatch was "
            f"deleted wholesale (115-003): {cmds}"
        )


def test_calibration_commands_pushes_the_multiplier_directly_not_int8_encoded() -> None:
    """132-014: OL/OA now carry the config MULTIPLIER directly (1.0 = no
    correction) -- the live wire push (set_config_field(OTOS,
    "linear_scale"/"angular_scale", value)) is applied through
    Devices::scaleToRegister() FIRMWARE-side now (132-010, trap 3 closed),
    so the host no longer pre-encodes via scale_to_int8() the way the
    pre-132 OL/OA text verbs did -- e.g. otos.linear_scale=1.027 ->
    ``OL 1.027``, not the old register-domain ``OL 27``."""
    from robot_radio.calibration.push import calibration_commands

    cfg = _cfg(otos=types.SimpleNamespace(linear_scale=1.027, angular_scale=0.987))
    cmds = calibration_commands(cfg)

    assert ("OI", 500) in cmds
    assert ("OL 1.027", 200) in cmds
    assert ("OA 0.987", 200) in cmds
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
    transport = _ScriptedReplyTransport({"SET ml": "ERR nodev"})
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
    transport = _ScriptedReplyTransport({"SET ml=": "ERR badval ml=0"})
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
    see module docstring); now ``travel_calib_left`` (MOTORS, a real
    consumer -- 132-014: moved from the retired ``calibration.
    mm_per_wheel_deg_left`` JSON path to ``motors.travel_calib_left``,
    132-020's grouped shape), deliberately different from the nocal default
    to prove overwrite.
    """
    cfg = _nocal_config()
    cfg["identity"] = {"robot_name": "tovez-custom", "uid": "tovez-custom"}
    cfg["motors"] = {"travel_calib_left": 0.5}
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

    109-002: the blanket "no REJECTED at all" assertion no longer holds.
    132-014 changes WHICH keys are the known, POSSIBLE rejections (the
    assertion below tolerates but does not require them): ``calibration_
    kwargs()`` no longer selects ``tw``/``rotSlip`` at all (GEOMETRY is
    boot-only AND a sim divergence -- see that function's own docstring),
    so they no longer appear in the push loop's own SET sequence at all
    (unrelated to whether they'd be accepted). ``OI`` has no live retarget
    (see ``binary_bridge.py``) -- always a known-tolerated rejection.
    ``OL``/``OA`` used to be a 132-014 KNOWN GAP (``config.otos.
    linear_scale``/``angular_scale`` read their proto3 zero default,
    below ``robot_config.proto``'s own ``(min) = 0.0001`` bound, so
    firmware honestly rejected them with ``ERR_RANGE``) -- 132-017's JSON
    reshape gave ``tovez_nocal.json`` real ``otos.linear_scale``/
    ``angular_scale`` values (1.0/1.0, within bounds), so OL/OA are
    typically ACCEPTED now; still allow-listed here (not asserted-absent)
    since a rejection for either is not itself a symptom of the odom-
    offset bug this test guards against. What still must never happen is
    a REJECTED entry for anything OTHER than OI/OL/OA (in particular, no
    ``badkey`` at all).
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
        _known_rejected_prefixes = ("[CAL] 'OI'", "[CAL] 'OL", "[CAL] 'OA")
        unexpected_rejections = [
            line for line in rejected_lines
            if not line.startswith(_known_rejected_prefixes)
        ]
        assert not unexpected_rejections, (
            f"only OI/OL/OA (known-rejected this sprint, see this test's "
            f"own docstring) may be rejected; found other rejections:\n"
            f"{unexpected_rejections}"
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
        # re-pushes" intent instead.
        assert f"ml={_expected_ml()}" in transport.command(
            "GET ml", read_timeout=500
        ), "connecting with 'tovez nocal' active must push its default travel calib"

        # 132-014 KNOWN GAP, RESOLVED 132-017: before the JSON reshape,
        # tovez.json's calibration.mm_per_wheel_deg_left=0.7165 lived at a
        # JSON path config.motors (the new grouped shape) did not read
        # yet, so BOTH profiles resolved ml to the SAME wheel-diameter
        # fallback -- this assertion could only prove the re-push
        # pipeline worked end to end, not that it overwrote a DIFFERENT
        # value. 132-017's JSON reshape gave tovez.json a real
        # motors.travel_calib_left (0.7165), now DISTINCT from
        # tovez_nocal's wheel-diameter-derived fallback (_expected_ml()) --
        # this is the STRONGER proof the original test name promised: the
        # combo switch genuinely overwrites firmware with a different
        # value, not merely re-sends the same one.
        log_before = _log_text(window)
        pushed_count_before = log_before.count("[CAL] pushed")

        robot_combo.setCurrentIndex(cal_idx)
        _spin_events(qapp, 0.3)

        assert "ml=0.7165" in transport.command("GET ml", read_timeout=500), (
            "switching to the calibrated 'tovez' robot while connected must "
            "re-push its own real travel_calib_left (0.7165), distinct from "
            "nocal's wheel-diameter fallback"
        )

        log_text = _log_text(window)
        assert log_text.count("[CAL] pushed") > pushed_count_before, (
            f"robot-combo switch must re-trigger a calibration push:\n{log_text}"
        )
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


# ---------------------------------------------------------------------------
# wire-testgui-live-push-of-estimator-stop-lead fix, RETARGETED 132-014:
# the nine estimator/shaper fields no longer ride one binary-only
# EstimatorConfigPatch ConfigDelta arm (config.proto, deleted 132-013) --
# they now live on TWO robot_config.proto groups (config.estimator.*/
# config.planner_shaper.*, see push.estimator_kwargs()'s own docstring for
# the full field-by-field disposition) and are pushed via
# set_config_field() per field. FIXED, 132-017 (JSON reshape ticket,
# stakeholder-sanctioned mid-sprint scope addition): PLANNER_SHAPER is now
# LIVE, split out of the boot-only PLANNER group -- these tests cover the
# selection function (unchanged shape, new sources) and the connect-time
# push it feeds (__main__.py's _push_estimator_config()), now asserting a
# real apply for the six shaper fields and an honest rejection for the
# three ESTIMATOR ones -- the SAME "Architecture Revision retarget"
# pattern this file already applies to rotSlip/tw.
#
# 118 ticket 004 (land-at-zero-completion-delete-stop-lead.md): a former
# fourth ``estimator.*`` field (a boot-time/live-tunable time-lead
# anticipation constant) is DELETED -- the completion mechanism it fed no
# longer exists (see App::MoveQueue::tick()'s own doc comment for the
# land-at-zero predicate that replaces it), so estimator_kwargs() still
# selects nine fields, not ten.
# ---------------------------------------------------------------------------


def _estimator_cfg(*, estimator=None, planner_shaper=None, robot_name="r"):
    return types.SimpleNamespace(
        robot_name=robot_name,
        estimator=estimator if estimator is not None else types.SimpleNamespace(),
        planner_shaper=planner_shaper if planner_shaper is not None else types.SimpleNamespace(),
    )


def test_estimator_kwargs_selects_estimator_and_shaper_fields_when_present() -> None:
    from robot_radio.calibration.push import estimator_kwargs

    cfg = _estimator_cfg(
        estimator=types.SimpleNamespace(
            weight_heading_otos=0.0, weight_omega_otos=0.0,
            staleness=60.0,
        ),
        planner_shaper=types.SimpleNamespace(
            a_max=800.0, a_decel=800.0, alpha_max=7.0, alpha_decel=7.0,
            jerk_max=5000.0, yaw_jerk_max=100.0,
        ),
    )

    kwargs = estimator_kwargs(cfg)

    assert kwargs == {
        "weight_heading_otos": 0.0, "weight_omega_otos": 0.0,
        "staleness": 60.0,
        "a_max": 800.0, "a_decel": 800.0, "alpha_max": 7.0, "alpha_decel": 7.0,
        "jerk_max": 5000.0, "yaw_jerk_max": 100.0,
    }


def test_estimator_kwargs_omits_none_fields() -> None:
    """A config with only SOME fields set (e.g. staleness alone) selects
    only those -- mirrors calibration_kwargs()'s own contract ('None ->
    nothing selected')."""
    from robot_radio.calibration.push import estimator_kwargs

    cfg = _estimator_cfg(
        estimator=types.SimpleNamespace(
            weight_heading_otos=None, weight_omega_otos=None,
            staleness=60.0,
        ),
        planner_shaper=types.SimpleNamespace(
            a_max=None, a_decel=None, alpha_max=None, alpha_decel=None,
            jerk_max=None, yaw_jerk_max=None,
        ),
    )

    assert estimator_kwargs(cfg) == {"staleness": 60.0}


def test_estimator_kwargs_empty_when_config_has_neither_section() -> None:
    from robot_radio.calibration.push import estimator_kwargs

    assert estimator_kwargs(_estimator_cfg()) == {}
    assert estimator_kwargs(types.SimpleNamespace(robot_name="bare")) == {}


def test_estimator_kwargs_real_tovez_nocal_json_via_real_model() -> None:
    """End-to-end through the REAL pydantic model -- data/robots/
    tovez_nocal.json read via RobotConfig.estimator/RobotConfig.
    planner_shaper (132-020's grouped shape, planner_shaper split out by
    132-017)."""
    from robot_radio.calibration.push import estimator_kwargs
    from robot_radio.config.robot_config import load_robot_config

    cfg_path = _REPO / "data" / "robots" / "tovez_nocal.json"
    cfg = load_robot_config(cfg_path)

    kwargs = estimator_kwargs(cfg)

    for key in (
        "weight_heading_otos", "weight_omega_otos", "staleness",
        "a_max", "a_decel", "alpha_max", "alpha_decel", "jerk_max", "yaw_jerk_max",
    ):
        assert key in kwargs, f"missing {key!r} in {kwargs}"


@_requires_sim_lib
def test_connect_pushes_estimator_config_and_reports_the_honest_rejection(
    qapp, monkeypatch, tmp_path
) -> None:
    """132-014 retarget, FIXED 132-017 (JSON reshape ticket,
    stakeholder-sanctioned mid-sprint scope addition): Connect (Sim) with
    the real tovez_nocal.json active still attempts all nine
    estimator/shaper fields -- PLANNER_SHAPER (the six shaper-ceiling
    fields) is now LIVE, split out of the boot-only PLANNER group because
    it carries its own re-appliable setter, so 6/9 applied, 3/9 rejected
    (matching __main__.py's own _push_estimator_config() log line format)
    is the CORRECT, documented outcome -- closing the 132-014-era "0/9
    applied" regression (itself already a fix over the ORIGINAL trap:
    EstimatorConfigPatch acking OK while landing nowhere). A caller
    reading this log now sees a real apply for the shaper fields and an
    honest, named rejection for ESTIMATOR alone."""
    cfg_path = _REPO / "data" / "robots" / "tovez_nocal.json"
    window, transport = _connect_gui_with_config(qapp, monkeypatch, tmp_path, cfg_path)
    try:
        log_text = _log_text(window)
        assert "pushed 6/9 estimator/shaper fields" in log_text, (
            f"estimator/shaper push did not report the expected 6/9 apply "
            f"(PLANNER_SHAPER is live as of 132-017):\n{log_text}"
        )
        assert "3 rejected" in log_text, log_text
    finally:
        _teardown(qapp, window)
