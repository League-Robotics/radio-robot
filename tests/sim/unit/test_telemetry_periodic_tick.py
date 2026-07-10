"""Sim verification for ticket 096-002 (SUC-002): the loop-owned periodic-
emission mechanism (``tickTelemetry()``, ``source/commands/
telemetry_commands.{h,cpp}``) that sprint 093's loop rewrite deleted,
restored via a peer call from BOTH ``source/main.cpp``'s bare ``for(;;)``
loop and ``tests/_infra/sim/sim_api.cpp``'s ``sim_tick()`` -- the same
"hardware and sim call the identical function" invariant ``Rt::MainLoop::
tick()`` already establishes for motion (that ``source/main.cpp``'s call
site compiles and links is proven by the ARM firmware build, not by this
file, which can only exercise the sim side).

Exercises STREAM/SNAP (``source/commands/telemetry_commands.cpp``,
``telemetryCommands()``, re-registered in ``Rt::CommandRouter::
buildTable()`` by this same ticket) end to end through the compiled
``libfirmware_host``, reading ``tickTelemetry()``'s periodic output via
``sim.peek_reply_store()`` (096-002, test-only ABI --
``sim_peek_reply_store()``) -- neither ``sim.command()`` nor
``sim.command_on()`` can be used to observe this output directly, since both
RESET (clear) the target channel's ReplyStore before routing anything,
which would silently wipe out whatever ``tickTelemetry()`` had already
accumulated across the preceding ``tick_for()`` calls before a test got to
read it.

Two acceptance-criteria groups (096-002's own):
  (a) ``STREAM <ms>`` followed by >= 200ms of ticking yields >= 3 periodic
      ``TLM ...`` frames with strictly increasing ``seq=``.
  (b) ``STREAM 0`` stops periodic emission; ``SNAP`` still works standalone
      (one-shot, unaffected by an active periodic stream).

Per Open Question 5 (architecture-update.md): STREAM's own dispatch reply
(the "OK stream period=..." ACK) carries NO concatenated first frame
anymore -- the old same-reply immediate-emission optimization is
deliberately not reproduced; the first periodic frame now arrives one pass
later, via ``tickTelemetry()``'s own ``!bb.telemetryHasLastEmit`` trigger.
Asserted directly below (``test_stream_ack_reply_carries_no_immediate_frame``).
"""
from __future__ import annotations

from firmware import CHANNEL_SERIAL


def _parse_tlm_lines(text: str) -> list[dict[str, str]]:
    """Parse zero or more "TLM t=... mode=... ..." wire lines (newline
    separated -- ReplyStore::append()'s own convention, sim_api.cpp) into a
    list of key->value dicts, in the order they were appended."""
    frames = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        assert parts[0] == "TLM", f"not a TLM line: {line!r}"
        frames.append(dict(p.split("=", 1) for p in parts[1:]))
    return frames


def test_stream_ack_reply_carries_no_immediate_frame(sim):
    """096-002 Open Question 5: STREAM's own ACK reply is exactly one line
    -- no concatenated immediate TLM frame (the pre-093 same-reply
    optimization is deliberately not reproduced by the new loop-owned
    tick)."""
    reply = sim.command("STREAM 50").strip()
    lines = reply.splitlines()
    assert lines == ["OK stream period=50"], (
        f"STREAM's ACK reply should carry no concatenated frame, got: {reply!r}"
    )


def test_stream_periodic_emission_monotonic_seq_over_200ms(sim):
    """STREAM 50 armed, then >= 200ms of ticking (tick_for()'s default 24ms
    step) must yield >= 3 periodic TLM frames on the SERIAL sync store, with
    strictly increasing seq= -- tickTelemetry()'s own per-pass elapsed check
    (bb.telemetryLastEmitMs/bb.telemetryHasLastEmit)."""
    sim.command("STREAM 50")
    assert sim.peek_reply_store(CHANNEL_SERIAL) == "", (
        "no periodic frame should exist yet -- tickTelemetry() has not run "
        "a single pass since STREAM armed the period"
    )

    sim.tick_for(240)   # [ms] >= 200ms of ticking, 10 x 24ms passes

    frames = _parse_tlm_lines(sim.peek_reply_store(CHANNEL_SERIAL))
    assert len(frames) >= 3, f"expected >= 3 periodic frames, got {len(frames)}: {frames}"

    seqs = [int(f["seq"]) for f in frames]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs), (
        f"seq= must strictly increase across periodic frames, got {seqs}"
    )


def test_stream_zero_stops_periodic_emission(sim):
    """STREAM 0 must stop periodic emission -- no further frames appended to
    the SERIAL sync store across a subsequent run of ticking, even though
    the channel binding (bb.telemetryChannel) and the last-emit bookkeeping
    are left untouched by STREAM 0 itself (only bb.telemetryPeriod is
    zeroed -- tickTelemetry()'s own `if (bb.telemetryPeriod == 0) return;`
    guard is what actually stops it)."""
    sim.command("STREAM 50")
    sim.tick_for(240)
    frames_before = _parse_tlm_lines(sim.peek_reply_store(CHANNEL_SERIAL))
    assert len(frames_before) >= 3, (
        "sanity: periodic emission must be active before disabling it"
    )

    sim.command("STREAM 0")   # resets both sync stores as a side effect of routing
    assert sim.peek_reply_store(CHANNEL_SERIAL) == "", (
        "sim.command()'s own store reset should have cleared the store"
    )

    sim.tick_for(240)
    assert sim.peek_reply_store(CHANNEL_SERIAL) == "", (
        "STREAM 0 must prevent any further periodic frame from being emitted"
    )


def test_snap_still_works_standalone_while_stream_is_active(sim):
    """SNAP is a one-shot, dispatched on its OWN reply channel -- unaffected
    by an active periodic STREAM (096-002 acceptance criterion). Exercised
    with STREAM 50 armed and periodic frames already emitted, so a clean
    SNAP reply proves the two paths do not interfere with each other."""
    sim.command("STREAM 50")
    sim.tick_for(120)   # a couple of periodic frames land on the SERIAL store

    reply = sim.command("SNAP").strip()   # resets the store first, then SNAP's own one-shot reply
    lines = reply.splitlines()
    assert len(lines) == 1, f"expected exactly one TLM line from SNAP, got: {reply!r}"
    frame = _parse_tlm_lines(reply)[0]
    assert "seq" in frame and "t" in frame and "mode" in frame
