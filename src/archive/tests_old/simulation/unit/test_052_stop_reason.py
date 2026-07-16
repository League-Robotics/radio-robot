"""
test_052_stop_reason.py — Tests for stop-reason reporting (052-002).

Verifies that MotionCommand::emitEvt appends "reason=<token>" to EVT done
strings when a stop condition fires, and that Superstructure::evaluateSafety
emits "EVT safety_stop reason=watchdog" when the watchdog fires.

Tests use the C++ firmware sim (build_lib fixture) for end-to-end coverage,
plus the Python MotionCommand mirror for fast unit coverage of each Kind.

Acceptance criteria:
  - T 200 200 1000 (TIME stop) emits EVT done T reason=time
  - D 200 200 300 (DISTANCE stop) emits EVT done D reason=dist
  - ROTATION stop → reason=rot
  - HEADING stop → reason=heading (covered via presence in EVT)
  - POSITION stop → reason=pos
  - LINE_ANY stop → reason=line
  - COLOR stop → reason=color
  - SENSOR stop (line0) → reason=line0
  - Watchdog → EVT safety_stop reason=watchdog
  - corr_id + reason both present in correct order (#id before reason=)
  - Cancel: reason= absent from EVT cancelled
"""

import ctypes
import re

import pytest
from firmware import Sim


TICK_STEP_MS = 24


def _tick_collect(s: Sim, n: int) -> str:
    """Tick n times, accumulating async events."""
    evts = ""
    for _ in range(n):
        s._lib.sim_tick(s._h, ctypes.c_uint32(s._t))
        s._t += TICK_STEP_MS
        evts += s.get_async_evts()
    return evts


# ---------------------------------------------------------------------------
# TIME stop → reason=time
# ---------------------------------------------------------------------------

def test_time_stop_reason(build_lib):
    """T 200 200 500 emits EVT done T reason=time when the TIME stop fires."""
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.get_async_evts()  # drain
        s.send_command("T 200 200 500")  # 500 ms TIME stop
        evts = _tick_collect(s, 100)  # ~2.4 s, generous
        assert "EVT done T" in evts, f"No EVT done T in: {evts!r}"
        assert "reason=time" in evts, (
            f"TIME stop did not emit reason=time; evts={evts!r}"
        )


def test_time_stop_reason_with_corr_id(build_lib):
    """T with #id emits EVT done T #id reason=time — corr_id before reason=.

    Note: corrId must be all-digits (protocol rule: '#' followed by digits only).
    """
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.get_async_evts()
        s.send_command("T 200 200 500 #42")
        evts = _tick_collect(s, 100)
        assert "EVT done T #42 reason=time" in evts, (
            f"Expected 'EVT done T #42 reason=time'; evts={evts!r}"
        )


# ---------------------------------------------------------------------------
# DISTANCE stop → reason=dist
# ---------------------------------------------------------------------------

def test_distance_stop_reason(build_lib):
    """D 200 200 200 emits EVT done D reason=dist when the DISTANCE stop fires."""
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.get_async_evts()
        s.send_command("D 200 200 200")
        evts = _tick_collect(s, 200)  # generous window
        assert "EVT done D" in evts, f"No EVT done D in: {evts!r}"
        assert "reason=dist" in evts, (
            f"DISTANCE stop did not emit reason=dist; evts={evts!r}"
        )


# ---------------------------------------------------------------------------
# VW stop=t → reason=time (additive stop on VW)
# ---------------------------------------------------------------------------

def test_vw_stop_time_reason(build_lib):
    """VW 200 0 stop=t:300 emits reason=time when the TIME stop fires."""
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.get_async_evts()
        s.send_command("VW 200 0 stop=t:300")
        evts = _tick_collect(s, 100)
        # VW uses 'EVT done VW' label when a stop condition fires.
        assert "reason=time" in evts, (
            f"VW stop=t did not emit reason=time; evts={evts!r}"
        )


# ---------------------------------------------------------------------------
# DISTANCE stop on VW → reason=dist
# ---------------------------------------------------------------------------

def test_vw_stop_distance_reason(build_lib):
    """VW 200 0 stop=d:150 emits reason=dist when the DISTANCE stop fires."""
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.get_async_evts()
        s.send_command("VW 200 0 stop=d:150")
        evts = _tick_collect(s, 200)
        assert "reason=dist" in evts, (
            f"VW stop=d did not emit reason=dist; evts={evts!r}"
        )


# ---------------------------------------------------------------------------
# ROTATION stop → reason=rot  (via RT command)
# ---------------------------------------------------------------------------

def test_rotation_stop_reason(build_lib):
    """RT 9000 emits reason=rot when the ROTATION stop fires."""
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.get_async_evts()
        s.send_command("RT 9000")
        evts = _tick_collect(s, 200)
        # RT fires ROTATION stop when encoder arc reaches target.
        assert "EVT" in evts, f"No EVT emitted; evts={evts!r}"
        assert "reason=rot" in evts, (
            f"ROTATION stop did not emit reason=rot; evts={evts!r}"
        )


# ---------------------------------------------------------------------------
# LINE_ANY stop → reason=line
# ---------------------------------------------------------------------------

def test_line_any_stop_reason(build_lib):
    """VW 200 0 stop=line:ge:512 emits reason=line when the LINE_ANY stop fires."""
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.init_line_sensor()
        s.set_line_values(0, 0, 0, 0)
        _tick_collect(s, 3)
        s.get_async_evts()

        s.send_command("VW 200 0 stop=line:ge:512")
        _tick_collect(s, 5)
        s.set_line_values(700, 0, 0, 0)
        evts = _tick_collect(s, 80)

        assert "reason=line" in evts, (
            f"LINE_ANY stop did not emit reason=line; evts={evts!r}"
        )


# ---------------------------------------------------------------------------
# SENSOR stop (line0) → reason=line0
# ---------------------------------------------------------------------------

def test_sensor_stop_reason_line0(build_lib):
    """T ... stop=sensor:line0:ge:500 emits reason=line0 on SENSOR stop."""
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.init_line_sensor()
        s.set_line_values(0, 0, 0, 0)
        _tick_collect(s, 3)
        s.get_async_evts()

        s.send_command("T 200 200 9000 stop=sensor:line0:ge:500")
        _tick_collect(s, 5)
        s.set_line_values(800, 0, 0, 0)
        evts = _tick_collect(s, 80)

        assert "reason=line0" in evts, (
            f"SENSOR stop (line0) did not emit reason=line0; evts={evts!r}"
        )


# ---------------------------------------------------------------------------
# COLOR stop → reason=color
# ---------------------------------------------------------------------------

def test_color_stop_reason(build_lib):
    """VW stop=color fires with reason=color."""
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.init_color_sensor()
        s.set_color_rgbc(255, 0, 0, 255)  # pure red
        _tick_collect(s, 3)
        s.get_async_evts()

        # Target hue 0 (red), generous threshold
        r = s.send_command("VW 200 0 stop=color:0:1.0:1.0:0.5")
        assert "ERR" not in r.upper(), f"stop=color rejected: {r!r}"

        evts = _tick_collect(s, 80)
        assert "reason=color" in evts, (
            f"COLOR stop did not emit reason=color; evts={evts!r}"
        )


# ---------------------------------------------------------------------------
# Cancel: reason= absent from EVT cancelled
# ---------------------------------------------------------------------------

def test_cancel_no_reason(build_lib):
    """EVT cancelled does NOT include reason= (cancel is not a stop-condition fire)."""
    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.get_async_evts()
        s.send_command("T 200 200 9000")   # long drive
        _tick_collect(s, 3)
        # Cancel the command.
        x_reply = s.send_command("X")
        # EVT cancelled should appear in the synchronous X reply.
        assert "EVT cancelled" in x_reply, (
            f"Expected 'EVT cancelled' in X reply; got {x_reply!r}"
        )
        # No reason= should be appended to EVT cancelled.
        assert "reason=" not in x_reply, (
            f"EVT cancelled should not include reason=; got {x_reply!r}"
        )


# ---------------------------------------------------------------------------
# Watchdog → EVT safety_stop reason=watchdog
# ---------------------------------------------------------------------------

def test_watchdog_emits_reason_watchdog(build_lib):
    """Watchdog timeout emits 'EVT safety_stop reason=watchdog'."""
    with Sim() as s:
        # Short watchdog (100 ms) and open-ended VW.
        s.send_command("SET sTimeout=100")
        s.get_async_evts()
        s.send_command("VW 200 0")   # open-ended, no time stop
        # Tick enough for the watchdog to fire (~100 ms / 24 ms per tick = ~5).
        evts = _tick_collect(s, 30)  # ~720 ms, generous
        assert "EVT safety_stop" in evts, (
            f"Watchdog did not fire; evts={evts!r}"
        )
        assert "reason=watchdog" in evts, (
            f"Watchdog EVT missing reason=watchdog; evts={evts!r}"
        )


# ---------------------------------------------------------------------------
# Python-mirror unit tests (no C++ needed)
# ---------------------------------------------------------------------------

from test_motion_command import (
    MotionCommand, HardwareState, MockBVC, make_mc, make_bvc, _reason_token
)


class TestReasonTokenMapping:
    """Unit tests for the _reason_token() helper (Python mirror)."""

    def test_none_kind_returns_empty(self):
        assert _reason_token('NONE') == ''

    def test_time_kind(self):
        assert _reason_token('TIME') == 'time'

    def test_distance_kind(self):
        assert _reason_token('DISTANCE') == 'dist'

    def test_rotation_kind(self):
        assert _reason_token('ROTATION') == 'rot'

    def test_heading_kind(self):
        assert _reason_token('HEADING') == 'heading'

    def test_position_kind(self):
        assert _reason_token('POSITION') == 'pos'

    def test_line_any_kind(self):
        assert _reason_token('LINE_ANY') == 'line'

    def test_color_kind(self):
        assert _reason_token('COLOR') == 'color'

    def test_sensor_line0(self):
        assert _reason_token('SENSOR', 0) == 'line0'

    def test_sensor_line1(self):
        assert _reason_token('SENSOR', 1) == 'line1'

    def test_sensor_line2(self):
        assert _reason_token('SENSOR', 2) == 'line2'

    def test_sensor_line3(self):
        assert _reason_token('SENSOR', 3) == 'line3'

    def test_sensor_colorR(self):
        assert _reason_token('SENSOR', 4) == 'colorR'

    def test_sensor_colorG(self):
        assert _reason_token('SENSOR', 5) == 'colorG'

    def test_sensor_colorB(self):
        assert _reason_token('SENSOR', 6) == 'colorB'

    def test_sensor_colorC(self):
        assert _reason_token('SENSOR', 7) == 'colorC'

    def test_sensor_analogIn0(self):
        assert _reason_token('SENSOR', 8) == 'analogIn0'

    def test_sensor_analogIn3(self):
        assert _reason_token('SENSOR', 11) == 'analogIn3'


class TestMotionCommandReasonInEvt:
    """Python-mirror tests: EVT done includes correct reason= token."""

    def test_time_stop_appends_reason_time(self):
        """TIME stop fires → 'EVT done reason=time'."""
        mc  = make_mc()
        bvc = make_bvc(at_target=True)

        mc.configure(200.0, 0.0, bvc)
        mc.add_stop({'kind': 'TIME', 'a': 100.0})
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(), now_ms=0)

        mc.tick(HardwareState(), now_ms=100, dt_s=0.01)  # TIME fires
        mc.tick(HardwareState(), now_ms=110, dt_s=0.01)  # BVC at target → done

        assert mc.active() is False
        assert mc.emitted_evts[-1] == 'EVT done reason=time'

    def test_distance_stop_appends_reason_dist(self):
        """DISTANCE stop fires → 'EVT done reason=dist'."""
        mc  = make_mc()
        bvc = make_bvc(at_target=True)

        mc.configure(200.0, 0.0, bvc)
        mc.add_stop({'kind': 'DISTANCE', 'a': 200.0})
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(encLMm=0.0, encRMm=0.0), now_ms=0)

        mc.tick(HardwareState(encLMm=200.0, encRMm=200.0), now_ms=1000, dt_s=0.01)
        mc.tick(HardwareState(encLMm=200.0, encRMm=200.0), now_ms=1010, dt_s=0.01)

        assert mc.active() is False
        assert mc.emitted_evts[-1] == 'EVT done reason=dist'

    def test_corr_id_before_reason(self):
        """corr_id appears before reason=: 'EVT done #id reason=time'."""
        mc  = make_mc()
        bvc = make_bvc(at_target=True)

        mc.configure(200.0, 0.0, bvc)
        mc.set_reply_sink(None, None, 'test42')
        mc.add_stop({'kind': 'TIME', 'a': 100.0})
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(), now_ms=0)

        mc.tick(HardwareState(), now_ms=100, dt_s=0.01)
        mc.tick(HardwareState(), now_ms=110, dt_s=0.01)

        assert mc.emitted_evts[-1] == 'EVT done #test42 reason=time'

    def test_cancel_no_reason(self):
        """Cancel does not append reason= (not a stop-condition fire)."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(200.0, 0.0, bvc)
        mc.start(HardwareState(), now_ms=0)
        mc.cancel()

        assert mc.emitted_evts[-1] == 'EVT cancelled'
        assert 'reason=' not in mc.emitted_evts[-1]

    def test_cancel_with_corr_id_no_reason(self):
        """Cancel with corr_id: 'EVT cancelled #id' without reason=."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(200.0, 0.0, bvc)
        mc.set_reply_sink(None, None, 'myid')
        mc.start(HardwareState(), now_ms=0)
        mc.cancel()

        assert mc.emitted_evts[-1] == 'EVT cancelled #myid'
        assert 'reason=' not in mc.emitted_evts[-1]

    def test_heading_stop_appends_reason_heading(self):
        """HEADING stop fires → reason=heading."""
        mc  = make_mc()
        bvc = make_bvc(at_target=True)

        mc.configure(0.0, 0.5, bvc)
        # Heading stop: target delta ~0, eps large (fires immediately at heading0).
        mc.add_stop({'kind': 'HEADING', 'a': 0.0, 'b': 3.14})
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(poseHrad=0.0), now_ms=0)

        mc.tick(HardwareState(poseHrad=0.0), now_ms=10, dt_s=0.01)
        mc.tick(HardwareState(poseHrad=0.0), now_ms=20, dt_s=0.01)

        # HEADING fired → reason=heading in EVT.
        assert any('reason=heading' in e for e in mc.emitted_evts), (
            f"Expected reason=heading; emitted_evts={mc.emitted_evts!r}"
        )

    def test_position_stop_appends_reason_pos(self):
        """POSITION stop fires → reason=pos."""
        mc  = make_mc()
        bvc = make_bvc(at_target=True)

        mc.configure(200.0, 0.0, bvc)
        # POSITION stop: target (0,0), radius 1000 mm (fires immediately at origin).
        mc.add_stop({'kind': 'POSITION', 'a': 0.0, 'ax': 0.0, 'b': 1000.0})
        mc.set_stop_style('SOFT')
        mc.start(HardwareState(poseX=0.0, poseY=0.0), now_ms=0)

        mc.tick(HardwareState(poseX=0.0, poseY=0.0), now_ms=10, dt_s=0.01)
        mc.tick(HardwareState(poseX=0.0, poseY=0.0), now_ms=20, dt_s=0.01)

        assert any('reason=pos' in e for e in mc.emitted_evts), (
            f"Expected reason=pos; emitted_evts={mc.emitted_evts!r}"
        )

    def test_no_stop_no_reason(self):
        """Open-ended command (no stops, externally cancelled) → no reason= in cancel."""
        mc  = make_mc()
        bvc = make_bvc()

        mc.configure(200.0, 0.0, bvc)
        mc.start(HardwareState(), now_ms=0)

        # Tick without any stop condition.
        mc.tick(HardwareState(), now_ms=10, dt_s=0.01)
        assert mc.active() is True
        assert mc.emitted_evts == []

        mc.cancel()
        assert mc.emitted_evts == ['EVT cancelled']
