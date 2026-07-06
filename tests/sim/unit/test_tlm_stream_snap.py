"""Sim verification for ticket 082-005 (SUC-005): the STREAM/SNAP wire
surface (``commands/telemetry_commands.{h,cpp}``), exercised end to end
through the compiled ``libfirmware_host`` -- as opposed to
``test_tlm_frame.py``'s harness, which exercises only the pure
``Telemetry::buildTlmFrame()`` formatter with synthetic ``TlmFrameInput``
values and no ``DevLoop``/``Hardware``/``Drivetrain``/``PoseEstimator``
wiring at all.

Four acceptance-criteria groups:
  (a) shape -- every documented field present when its source exists.
  (b) otos= omission -- NOT exercised here (see that test's own docstring):
      ``Subsystems::SimHardware::odometer()`` always returns a real
      ``Hal::SimOdometer*`` (sim_hardware.h), so there is no way to reach
      the ``hardware.odometer() == nullptr`` branch through this ABI. The
      omission-vs-zero-fill proof lives at the pure-formatter level in
      ``test_tlm_frame.py``'s ``scenarioOtosOmittedNotZeroFilled`` (082-004)
      -- this file instead confirms the PRESENT half of Decision 7 (otos=
      IS emitted when an odometer exists), which is the half this ABI can
      actually exercise.
  (c) seq= -- shared by STREAM and SNAP, monotonically increasing.
  (d) STREAM <ms> clamp -- STREAM 10 -> OK stream period=20.

mode= at rest ('I', including across DEV DT drives that never engage
Subsystems::Planner) is covered by its own test functions below rather than
a separate file -- it is a TLM wire field like any other in this shape
family, per the ticket's own "Files to create" list naming only three
sim-level files. As of 084-005, mode= is derived exclusively from
Planner::state().mode (see that ticket's own doc comment on the tests
below); the full I/S/T/D/G vocabulary over the actual Planner-mediated
motion verbs (S/T/D/R/TURN/RT/G) is covered end to end in
tests/sim/unit/test_mode_machine.py.
"""
from __future__ import annotations


def _parse_tlm(line: str) -> dict[str, str]:
    """Parse one "TLM t=... mode=... ..." wire line into a key->value dict.

    Local, small, deliberately duplicated per test file -- mirrors this
    directory's existing precedent (e.g. ``_drive_straight`` duplicated
    across test_otos_error_injection.py / test_errored_observation.py)
    rather than a shared test-util module.
    """
    parts = line.strip().split()
    assert parts[0] == "TLM", f"not a TLM line: {line!r}"
    return dict(p.split("=", 1) for p in parts[1:])


def _snap(sim) -> dict[str, str]:
    """Issue SNAP and parse its reply, asserting it is exactly one line
    (true whenever STREAM has never been enabled in the same test, since
    then periodMs stays 0 and only SNAP itself ever advances the shared
    seq= counter/emits a frame)."""
    reply = sim.command("SNAP").strip()
    lines = reply.splitlines()
    assert len(lines) == 1, f"expected exactly one TLM line from SNAP, got: {reply!r}"
    return _parse_tlm(lines[0])


# ---------------------------------------------------------------------------
# (a) Shape: every documented field present when its source exists.
# ---------------------------------------------------------------------------

def test_snap_frame_carries_every_documented_field_at_rest(sim):
    """At rest (no DEV DT command ever issued), a SNAP frame still carries
    every documented field -- enc=/vel=/pose=/encpose=/otos=/twist= -- since
    every one of their sources (Hardware's motor pair, PoseEstimator,
    SimHardware's odometer) exists from sim construction, per Decision 7's
    field-sourcing table (telemetry_commands.h)."""
    tlm = _snap(sim)

    for mandatory in ("t", "mode", "seq"):
        assert mandatory in tlm, f"{mandatory}= is mandatory and must always be present"

    for optional in ("enc", "vel", "pose", "encpose", "otos", "twist"):
        assert optional in tlm, (
            f"{optional}= should be present -- its source (Hardware/"
            f"PoseEstimator/SimHardware's odometer) exists in the sim"
        )

    # enc=/vel= are 2-tuples; pose=/encpose=/otos= are 3-tuples; twist= is
    # a 2-tuple -- confirms field SHAPE, not just key presence.
    assert len(tlm["enc"].split(",")) == 2
    assert len(tlm["vel"].split(",")) == 2
    assert len(tlm["pose"].split(",")) == 3
    assert len(tlm["encpose"].split(",")) == 3
    assert len(tlm["otos"].split(",")) == 3
    assert len(tlm["twist"].split(",")) == 2


def test_snap_frame_carries_every_documented_field_while_driving(sim):
    """Same shape assertion mid-drive -- proves the field set doesn't
    silently shrink once motion/nonzero values are involved (e.g. a field
    formatter choking on a non-zero value some initial all-zero-state test
    wouldn't catch)."""
    sim.command("DEV DT PORTS 1 2")
    sim.command("DEV DT VW 120 0 0.3")
    sim.tick_for(240)

    tlm = _snap(sim)
    for optional in ("enc", "vel", "pose", "encpose", "otos", "twist"):
        assert optional in tlm, f"{optional}= should still be present while driving"


# ---------------------------------------------------------------------------
# (c) seq=: shared by STREAM and SNAP, monotonically increasing.
# ---------------------------------------------------------------------------

def test_snap_seq_increments_by_exactly_one_per_call_with_stream_disabled(sim):
    """With STREAM never enabled (periodMs stays at its 0/disabled default
    -- TelemetryState's own field default), only SNAP itself advances the
    shared seq= counter, so three back-to-back SNAP calls must read
    0, 1, 2 exactly -- the tightest possible monotonicity proof."""
    seqs = [int(_snap(sim)["seq"]) for _ in range(3)]
    assert seqs == [0, 1, 2], f"expected a clean 0,1,2 sequence, got {seqs}"


def test_stream_and_snap_share_one_monotonically_increasing_seq_counter(sim):
    """STREAM and SNAP are dispatched by two structurally different
    handlers (handleStream()/handleSnap(), telemetry_commands.cpp) that
    both call the SAME telemetryEmit(), advancing the SAME
    TelemetryState::seq field -- proven here by interleaving SNAP calls
    with a STREAM <ms> command and confirming the counter never resets or
    goes backwards across the switch.

    STREAM's own reply is itself observable proof of the shared counter:
    since no channel has ever emitted a frame yet when STREAM first fires
    (TelemetryState::hasLastEmit starts false, and SNAP's handler never
    touches hasLastEmit/lastEmitMs -- only the periodic-emission step in
    dev_loop.cpp does), the very first STREAM command immediately emits
    one frame in its OWN reply (mirrors dev_loop.h's own doc comment: "the
    very first pass after a channel issues STREAM emits immediately") --
    continuing the SAME seq= sequence SNAP had already advanced, not
    restarting it at 0.
    """
    first = int(_snap(sim)["seq"])
    second = int(_snap(sim)["seq"])
    assert second == first + 1

    stream_reply = sim.command("STREAM 50").strip()
    lines = stream_reply.splitlines()
    assert lines[0] == "OK stream period=50"
    assert len(lines) == 2, (
        f"STREAM's own reply should carry exactly one immediate TLM frame "
        f"(no channel had emitted yet): {stream_reply!r}"
    )
    stream_emitted_tlm = _parse_tlm(lines[1])
    stream_seq = int(stream_emitted_tlm["seq"])
    assert stream_seq == second + 1, (
        "STREAM's own immediate emission must continue the SAME counter "
        "SNAP was already advancing, not restart at 0"
    )

    # Advance time (periodic emissions may fire during this window, further
    # advancing the internal counter even though their individual frames
    # are not separately observed here) then confirm the NEXT observable
    # frame (another SNAP) reads a seq strictly greater than STREAM's own
    # -- proving continuity across the periodic-emission path too, not
    # just across the two synchronous command handlers.
    sim.tick_for(240)
    after_tick_seq = int(_snap(sim)["seq"])
    assert after_tick_seq > stream_seq, (
        "seq= must keep monotonically increasing across ticks with STREAM "
        "enabled, never reset or go backwards"
    )


# ---------------------------------------------------------------------------
# (d) STREAM <ms> clamp: STREAM 10 -> OK stream period=20.
# ---------------------------------------------------------------------------

def test_stream_period_clamps_below_20ms_floor(sim):
    """STREAM 10 (below docs/protocol-v2.md §8's documented 20ms floor)
    must be ACCEPTED and silently clamped to 20, not rejected -- the exact
    behavior telemetry_commands.cpp's handleStream() implements
    (kStreamFloorMs)."""
    reply = sim.command("STREAM 10").strip()
    lines = reply.splitlines()
    assert lines[0] == "OK stream period=20", (
        f"STREAM 10 should clamp to period=20, got: {lines[0]!r}"
    )


def test_stream_period_at_or_above_floor_passes_through_unclamped(sim):
    """Sanity companion to the clamp test above: a period already at or
    above the 20ms floor passes through UNCHANGED (proves the clamp is a
    floor, not an unconditional override to 20)."""
    reply = sim.command("STREAM 100").strip()
    lines = reply.splitlines()
    assert lines[0] == "OK stream period=100"


# ---------------------------------------------------------------------------
# mode=: 'I' at rest -- confirmed over the sim's actual wire surface (SNAP).
#
# 084-005 update (Decision 6): mode= is now derived EXCLUSIVELY from
# Subsystems::Planner::state().mode (docs/protocol-v2.md §8; architecture-
# update.md (084) Decision 6), not `drivetrain.active() ? 'S' : 'I'` (082's
# original, minimal source, extended by this ticket). `DEV DT VW`/`WHEELS`
# command Subsystems::Drivetrain directly through DevLoopState's own outbox
# and never stage a msg::PlannerCommand at all, so mode= now correctly reads
# 'I' throughout a `DEV DT VW`/`WHEELS` drive -- the wheels really are
# spinning (confirmed below via true_velocity()), but that authority is
# DEV's bench-diagnostic path, not a `Planner`-mediated production drive, so
# it is invisible to mode= by design. This is a deliberate consequence of
# consolidating mode= onto a single source of truth, not a regression: the
# three tests below used to assert 'S' here (082's `drivetrain.active()`-
# based source could not tell DEV DT's authority apart from Planner's); they
# now assert the corrected 'I' behavior and confirm the wheels are still
# genuinely moving despite it. Full I/S/T/D/G coverage for the actual
# Planner-mediated verb families lives in test_mode_machine.py.
# ---------------------------------------------------------------------------

def test_mode_is_idle_at_rest(sim):
    """Fresh sim, no DEV DT command ever issued -- mode= must read 'I'."""
    assert _snap(sim)["mode"] == "I"


def test_mode_stays_idle_during_dev_dt_vw_drive_since_planner_not_engaged(sim):
    """An active `DEV DT VW` body-twist drive never engages `Planner`, so
    mode= (084-005: Planner-exclusive source) reads 'I' throughout -- even
    though the wheels are genuinely spinning under DEV's own authority."""
    sim.command("DEV DT PORTS 1 2")
    sim.command("DEV DT VW 100 0 0")
    assert _snap(sim)["mode"] == "I"

    sim.tick_for(240)
    vel_l, vel_r = sim.true_velocity()
    assert vel_l > 10.0 and vel_r > 10.0, (
        f"expected DEV DT VW to actually be driving the wheels, got vel=({vel_l}, {vel_r})"
    )
    assert _snap(sim)["mode"] == "I"


def test_mode_stays_idle_during_dev_dt_wheels_drive_since_planner_not_engaged(sim):
    """Same as above for the `DEV DT WHEELS` per-wheel-velocity verb --
    ticket's acceptance criterion names both VW and WHEELS explicitly."""
    sim.command("DEV DT PORTS 1 2")
    sim.command("DEV DT WHEELS 80 80")
    assert _snap(sim)["mode"] == "I"

    sim.tick_for(240)
    vel_l, vel_r = sim.true_velocity()
    assert vel_l > 10.0 and vel_r > 10.0, (
        f"expected DEV DT WHEELS to actually be driving the wheels, got vel=({vel_l}, {vel_r})"
    )
    assert _snap(sim)["mode"] == "I"


def test_mode_stays_idle_across_a_dev_dt_stop_bookend(sim):
    """Bookend on "at rest": mode= must still read 'I' on both sides of a
    `DEV DT VW` drive plus its `DEV DT STOP` -- none of these three commands
    ever stage a msg::PlannerCommand, so `Planner::hasActiveCommand()` (and
    therefore mode=) never leaves 'I' across the whole sequence."""
    sim.command("DEV DT PORTS 1 2")
    sim.command("DEV DT VW 100 0 0")
    assert _snap(sim)["mode"] == "I"

    sim.command("DEV DT STOP")
    assert _snap(sim)["mode"] == "I"
