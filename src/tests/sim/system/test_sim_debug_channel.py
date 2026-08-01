"""src/tests/sim/system/test_sim_debug_channel.py -- 129-003 (DBG debug
message channel, bench/Sim only; issue 05-dbg-debug-message-channel-for-
bench-and-sim.md).

Proves the FULL real C++ plumbing end to end through the compiled sim
library, not a Python stub: ``App::setDebugSink()`` (wired by
``TestSim::SimHarness``'s own constructor, ``src/sim/sim_harness.h``) ->
``App::debugf()`` (``src/firm/app/debug.h``/``.cpp``) -> ``App::Comms::
sendDebug()`` (``comms.h``/``.cpp``, a real cleartext ``"DBG:<message>"``
wire line via ``sendReliable()``) -> ``SimHarness::drainReliable()`` ->
``sim_drain_debug()`` (``sim_ctypes.cpp``) -> ``SimLoop._drain_debug_into_
queue()`` -> ``SimLoop.drain_debug_lines()``/``drain_pending_debug()``.

``sim_test_emit_debug()`` (``sim_ctypes.cpp``) is a TEST-ONLY escape hatch
that calls ``App::debugf("%s", msg)`` directly against the handle's own
installed sink -- this ticket lands the DBG CHANNEL itself, ahead of
tickets 006 (duty sweep)/007 (adaptive calibration)'s actual ``debugf()``
call sites, so there is no real diagnostic producing DBG lines yet to
exercise this path against.

Deterministic manual stepping (``connect(start_tick_thread=False)``,
mirroring ``test_sim_wire_loopback.py``'s own ``_make_loop()`` -- no
background tick thread, no timing races).

Requires the compiled ``src/sim/build/libfirmware_host.{dylib,so}``
(``just build-sim``) -- skips cleanly if not present (same convention as
every other ``src/tests/sim/system/`` file in this tier).
"""
from __future__ import annotations

import pathlib
import sys

import pytest

# src/tests/sim/system/test_sim_debug_channel.py -> system -> sim -> tests ->
# src -> repo root = FOUR hops from __file__ (same convention as this
# tier's sibling files, e.g. test_sim_wire_loopback.py).
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]

_LIB_NAME = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
_SIM_LIB_PATH = _REPO_ROOT / "src" / "sim" / "build" / _LIB_NAME

pytestmark = pytest.mark.skipif(
    not _SIM_LIB_PATH.exists(),
    reason="sim lib not built -- just build-sim (or cmake --build src/sim/build)",
)


def _make_loop():
    """A bare, headless SimLoop -- deterministic manual stepping, no
    SimTransport, no Qt. Mirrors this tier's own established _make_loop()
    helper (test_sim_wire_loopback.py, test_sim_configure_from_robot.py)."""
    from robot_radio.io.sim_loop import SimLoop

    loop = SimLoop(lib_path=_SIM_LIB_PATH)
    loop.connect(start_tick_thread=False)
    return loop


def test_debugf_output_reaches_drain_debug_lines_via_manual_drain():
    """The ticket's own acceptance wording: "Sim ... builds: debugf() lines
    arrive host-side". drain_pending_debug() is the manual-step counterpart
    of the tick thread's own per-iteration drain (drain_pending_tlm()'s own
    established shape) -- does the sim_drain_debug() fetch itself, then
    returns the queue contents."""
    loop = _make_loop()
    try:
        loop._lib.sim_test_emit_debug(loop._handle, b"hello from debugf 42")

        lines = loop.drain_pending_debug()

        assert lines == ["DBG:hello from debugf 42"]
    finally:
        loop.disconnect()


def test_drain_debug_lines_is_a_pure_queue_read_populated_by_the_manual_drain():
    """drain_debug_lines() itself never touches the ctypes handle (mirrors
    read_pending_binary_tlm_frames()'s own contract) -- calling
    drain_pending_debug() once, then drain_debug_lines() again, must not
    re-fetch or duplicate anything."""
    loop = _make_loop()
    try:
        loop._lib.sim_test_emit_debug(loop._handle, b"first")

        first_read = loop.drain_pending_debug()
        second_read = loop.drain_debug_lines()  # queue already empty -- no ctypes touch needed

        assert first_read == ["DBG:first"]
        assert second_read == []
    finally:
        loop.disconnect()


def test_multiple_debug_lines_arrive_in_order():
    loop = _make_loop()
    try:
        for i in range(5):
            loop._lib.sim_test_emit_debug(loop._handle, f"line {i}".encode())

        lines = loop.drain_pending_debug()

        assert lines == [f"DBG:line {i}" for i in range(5)]
    finally:
        loop.disconnect()


def test_debug_channel_does_not_disturb_telemetry_drain():
    """A DBG emission interleaved with normal stepping must not perturb the
    existing TLM drain path -- the two channels ride separate cleartext/
    binary wire planes and separate ctypes drain calls, but this proves
    they genuinely coexist through one real SimHarness session.

    A freshly-booted, unconfigured, idle SimHarness emits NO periodic
    telemetry at all (comms.h's own STATUS doc comment: "telemetry is also
    silent while the robot is parked") -- ``TLM:NOW`` (the same cleartext
    control token a human at a serial terminal would type, comms.cpp's own
    dispatchLine() TLM-interception branch) is injected as a plain ASCII
    line (not COBS-framed -- TLM's inbound surface is intercepted BEFORE
    the binary/cleartext registry dispatch) to force one frame per check,
    exactly the escape hatch that comment describes. NO trailing ``\n`` --
    ``FakeTransport::enqueueInbound()``'s own contract is "one complete
    line, the delimiter already stripped" (fake_transport.h), matching
    every other ``inject_command()`` caller in this tier (e.g.
    ``test_sim_wire_loopback.py``'s own ``wire_line`` builders)."""
    loop = _make_loop()
    try:
        loop.inject_command(b"TLM:NOW")
        loop.step(1)
        tlm_before = loop.drain_pending_tlm()
        assert tlm_before, "no telemetry frame decoded for an explicit TLM:NOW request"

        loop._lib.sim_test_emit_debug(loop._handle, b"mid-session diagnostic")

        loop.inject_command(b"TLM:NOW")
        loop.step(1)
        tlm_after = loop.drain_pending_tlm()
        assert tlm_after, "telemetry stopped answering TLM:NOW after a DBG emission"

        debug_lines = loop.drain_pending_debug()
        assert debug_lines == ["DBG:mid-session diagnostic"]
    finally:
        loop.disconnect()


def test_on_debug_callback_receives_the_line_and_a_raising_callback_does_not_raise():
    """SimLoop.on_debug mirrors SerialConnection.on_debug's own immediate-
    delivery/exception-proof contract (io/serial_conn.py) -- see
    _drain_debug_into_queue()'s own doc comment."""
    received: list[str] = []

    def _on_debug(line: str) -> None:
        received.append(line)
        raise RuntimeError("a buggy on_debug handler must not propagate")

    loop = _make_loop()
    loop.on_debug = _on_debug
    try:
        loop._lib.sim_test_emit_debug(loop._handle, b"callback test")

        loop._drain_debug_into_queue()  # must not raise despite the callback's own bug

        assert received == ["DBG:callback test"]
    finally:
        loop.disconnect()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
