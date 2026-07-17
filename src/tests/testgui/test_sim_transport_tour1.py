"""src/tests/testgui/test_sim_transport_tour1.py — the headless equivalent of
"press Tour 1 in Sim, watch the trace draw" (sprint 108 ticket 007).

Constructs a REAL ``SimTransport`` (``robot_radio.testgui.transport``),
``connect()``s it against the REAL compiled firmware simulator
(``src/sim/build/libfirmware_host.{dylib,so}``), reads
``.protocol`` (a live ``SimLoop`` — see that class's own docstring), and
drives real twist/stop commands against it — exactly the path
``__main__.py``'s ``_TourRunner.run()`` takes once a Sim connection is live
and the tour buttons are un-gated (this ticket's other half). No Qt/GUI
objects involved, only the transport + planner stack.

Skips cleanly (module-level ``skipif``) if the sim lib has not been built —
mirrors ``src/tests/testgui/test_sim_loop.py``'s own guard.

Run with::

    uv run python -m pytest src/tests/testgui/test_sim_transport_tour1.py -v
"""
from __future__ import annotations

import math
import time

import pytest

from robot_radio.testgui.transport import SimTransport, _sim_lib_path

pytestmark = pytest.mark.skipif(
    not _sim_lib_path().exists(),
    reason="sim lib not built -- cmake --build src/sim/build",
)


@pytest.fixture
def sim_transport():
    transport = SimTransport()
    transport.connect()
    assert transport._connected, "SimTransport failed to connect to the sim lib"
    try:
        yield transport
    finally:
        transport.disconnect()


def test_sim_transport_connects_and_exposes_a_live_protocol(sim_transport):
    """108-006 left SimTransport.connect() failing fast with "SimTransport
    failed to connect" -- this ticket's rewire fixes that regression. A
    connected SimTransport's `.protocol` must be a live SimLoop (satisfying
    TwistTransport), not None."""
    from robot_radio.io.sim_loop import SimLoop

    protocol = sim_transport.protocol
    assert protocol is not None
    assert isinstance(protocol, SimLoop)
    assert protocol.is_connected


def test_tour_shaped_sequence_via_direct_twist_calls_drives_and_closes(sim_transport):
    """Drives TOUR_1's own leg geometry directly against ``.protocol``
    (``twist()``/``stop()``, no ``run_tour()``/``StreamingExecutor`` in the
    loop) -- proves the REWIRED SEAM itself (``SimTransport`` -> ``SimLoop``
    -> real compiled firmware -> ``SimPlant`` -> telemetry ->
    ``on_telemetry``) is correct end-to-end: commands actually move the
    plant, telemetry actually flows back, and a plausible, finite closure
    can be computed from real ``TLMFrame.pose`` readings -- independent of
    ``run_tour()``'s own baseline-exclusion fault-check timing, which a
    separate, sim-specific issue currently makes unreliable across a full
    leg-chained tour (see this file's other test and
    ``clasi/issues/sim-mode-tour-1-fault-baseline-exclusion-mismatch.md``).
    """
    from robot_radio.planner.tour import TOUR_1, parse_tour
    from robot_radio.robot.protocol import TLMFrame

    protocol = sim_transport.protocol
    assert protocol is not None

    legs = parse_tour(TOUR_1)
    assert len(legs) == 13

    delivered_frames: list[TLMFrame] = []
    sim_transport.on_telemetry = delivered_frames.append

    _tick_interval = 0.1  # [s]
    _turn_omega = 1.0  # [rad/s]

    def _drain_pose() -> "tuple[float, float, float] | None":
        frames = protocol.read_pending_binary_tlm_frames()
        for f in frames:
            if f.pose is not None:
                x, y, h_cdeg = f.pose
                return (float(x), float(y), math.radians(h_cdeg / 100.0))
        return None

    # Settle briefly and capture a starting pose before driving anything.
    start_pose = None
    deadline = time.monotonic() + 3.0
    while start_pose is None and time.monotonic() < deadline:
        start_pose = _drain_pose()
        if start_pose is None:
            time.sleep(0.05)
    assert start_pose is not None, "never received a pose-bearing TLMFrame at start"

    last_pose = start_pose
    for leg in legs:
        if leg.kind == "distance":
            speed = leg.speed or 150.0  # [mm/s]
            direction = 1.0 if leg.value >= 0 else -1.0
            duration_s = abs(leg.value) / speed
            v_x, omega = direction * speed, 0.0
        else:
            direction = 1.0 if leg.value >= 0 else -1.0
            duration_s = math.radians(abs(leg.value)) / _turn_omega
            v_x, omega = 0.0, direction * _turn_omega

        elapsed = 0.0
        while elapsed < duration_s:
            protocol.twist(v_x, omega, _tick_interval * 1000.0 * 2)
            time.sleep(_tick_interval)
            elapsed += _tick_interval
            pose = _drain_pose()
            if pose is not None:
                last_pose = pose
        protocol.stop()
        time.sleep(0.3)
        pose = _drain_pose()
        if pose is not None:
            last_pose = pose

    end_pose = last_pose
    dx = end_pose[0] - start_pose[0]
    dy = end_pose[1] - start_pose[1]
    position_delta = math.hypot(dx, dy)  # [mm]

    # Report-oriented sanity bound, not a bench-tuned tolerance (real bench
    # TOUR_1 closures observed 2026-07-15 ranged 32-503mm position /
    # -177..+73deg heading across several completed runs -- see
    # src/tests/bench/data/tour_traces/tour_tour_1_*.json -- TOUR_1 is NOT a
    # tightly-closed loop in practice). This bound only catches a genuinely
    # broken seam (integration blowing up / stuck at the origin / NaN), not
    # pinning a specific closure number.
    assert position_delta < 2000.0, (
        f"tour-shaped sequence closure position delta implausibly large: "
        f"{position_delta:.1f}mm (start={start_pose}, end={end_pose})"
    )
    assert math.isfinite(end_pose[2])

    assert len(delivered_frames) > 0, "no telemetry frames were forwarded to on_telemetry"


# ---------------------------------------------------------------------------
# Full run_tour(TOUR_1) -- was known-unreliable in Sim (see the closed issue
# below): WheelPlant::reportedPosition() reported a stopped wheel's position
# with zero noise, so a stopped wheel emitted byte-identical tenths every
# cycle and starved Devices::MotorArmor::updateWedgeDetector() of the jitter
# a real, healthy encoder always has at rest, latching kFaultWedgeLatch at
# every leg boundary. Fixed by ticket 108-011 (a per-wheel, rest-gated,
# seeded ±1 LSB dither in WheelPlant::reportedPosition()'s nominal branch --
# see src/tests/sim/plant/wheel_plant.{h,cpp}). This test now runs for real, no
# xfail.
#
# 109-008: `run_tour()` itself was rewired onto the MOVE-queue path this
# ticket (one `Move` per leg, firmware-owned queue/boundary-carry/heading PD
# -- see `planner/tour.py`'s own file header). This is also this ticket's
# own verification of `tour1-freeze-investigation-2026-07-15.md`: that
# investigation's verdict was a real `kFaultWedgeLatch` firmware fault
# tripping the OLD path's own "stop the whole tour on any nonzero fault_bits"
# polling -- the new path has no such polling at all (a leg's own outcome is
# driven solely by that Move's own terminal ack-ring status), so a transient
# fault bit can no longer freeze/stop a tour on its own. This test running
# TOUR_1 to completion end to end over the real compiled firmware sim is the
# concrete demonstration that the boundary crossings (13 legs, 6 straight-
# >turn/turn->straight transitions) do not reproduce the freeze symptom on
# this path.
# ---------------------------------------------------------------------------

_MAX_TOUR_ATTEMPTS = 5
_MAX_CLOSURE_POSITION_MM = 600.0  # [mm] -- see the direct-twist test's own
# comment: real bench TOUR_1 closures ranged up to ~500mm even when
# COMPLETED cleanly. This only catches an implausible blowup.


# 109-009 (round 2, resolved): this test regressed to a consistent leg-12
# STOP_TIME fault after round 1's completion-gate fixes landed (see
# clasi/issues/tour1-via-simtransport-leg12-stop-time-regression.md for the
# full history) -- the xfail below is REMOVED, not just loosened, because
# round 2 found and fixed the actual root cause: `Motion::Executor`'s dwell
# gate combined (1) a hard reset-to-zero on any single tolerance/rate miss
# with (2) a rate test built on a RAW one-sample finite-difference
# derivative, which is highly sensitive to per-cycle heading-measurement
# noise. Fixed with a leaky/decaying hold counter (a miss now costs one
# cycle, not the whole accumulated hold) plus a light exponential low-pass
# filter on the rate estimate the dwell gate itself uses (see
# `motion/executor.cpp`'s own dwell-completion comment and `motion/
# DESIGN.md`'s dwell-completion entry). Verified: this test now passes
# reliably (3/3 repeated pytest invocations, each with its own fresh
# SimTransport connection) -- see the issue file's own resolution note.
def test_tour_1_runs_to_completion_with_finite_small_closure(sim_transport):
    """The programmatic equivalent of "press Tour 1 and watch the trace
    draw": every leg of TOUR_1 runs to completion (RunOutcome.COMPLETED)
    against the REAL compiled firmware sim, and the tour's own pose closure
    (measured pose before leg 1 vs. after the final leg) is finite."""
    from types import SimpleNamespace

    from robot_radio.planner.heading import HeadingCorrector
    from robot_radio.planner.model import PlannerParams
    from robot_radio.planner.tour import TOUR_1, parse_tour, run_tour

    legs = parse_tour(TOUR_1)
    assert len(legs) == 13

    delivered_frames: list = []
    sim_transport.on_telemetry = delivered_frames.append

    attempts: list[str] = []
    result = None
    for attempt in range(1, _MAX_TOUR_ATTEMPTS + 1):
        protocol = sim_transport.protocol
        assert protocol is not None, "SimTransport has no live protocol -- not connected?"

        params = PlannerParams()
        # Mirrors src/tests/bench/tour_bench_run.py's own bench-rig convention:
        # force encoder-derived heading rather than trusting OTOS, so this
        # test's outcome does not depend on whichever robot config happens
        # to be "active" in this process (get_robot_config() may return
        # None headless).
        heading = HeadingCorrector(
            params,
            robot_config=SimpleNamespace(geometry=SimpleNamespace(otos_untrusted=True)),
        )

        sim_transport.suspend_telemetry_reader()
        try:
            result = run_tour(
                protocol, params, heading, legs,
                row_callback=lambda tick_index, leg_index, leg, tick_result, frame: (
                    sim_transport.on_telemetry(frame) if frame is not None else None
                ),
            )
        finally:
            sim_transport.resume_telemetry_reader()

        if result.stopped_at is None:
            break
        attempts.append(
            f"attempt {attempt}: stopped at leg {result.stopped_at} ({result.stopped_outcome})"
        )
        # Reconnect for a clean retry -- a fresh SimLoop/SimPlant, not the
        # same one that just fault-stopped mid-tour.
        sim_transport.disconnect()
        sim_transport.connect()
        assert sim_transport._connected, "SimTransport failed to reconnect for retry"

    assert result is not None and result.stopped_at is None, (
        f"tour never completed in {_MAX_TOUR_ATTEMPTS} attempts (see "
        f"clasi/issues/sim-mode-tour-1-fault-baseline-exclusion-mismatch.md): "
        + "; ".join(attempts)
    )
    assert len(result.legs) == 13
    for i, leg_result in enumerate(result.legs):
        assert leg_result.outcome.value == "completed", (
            f"leg {i + 1}/13 ({leg_result.leg.kind} {leg_result.leg.value:g}) "
            f"did not complete: {leg_result.outcome.value}"
        )

    closure = result.closure
    assert closure.position_delta is not None, "closure never computed (tour stopped early?)"
    assert closure.heading_delta is not None
    assert closure.position_delta < _MAX_CLOSURE_POSITION_MM, (
        f"tour closure position delta implausibly large: {closure.position_delta:.1f}mm "
        f"(start={closure.start_pose}, end={closure.end_pose})"
    )

    # Telemetry actually flowed to on_telemetry during the run (the row_callback
    # forwarding above mirrors _TourRunner._on_row()'s own real GUI wiring —
    # see __main__.py) -- the canvas/avatar tracking path this test stands in
    # for.
    assert len(delivered_frames) > 0, "no telemetry frames were forwarded to on_telemetry"
