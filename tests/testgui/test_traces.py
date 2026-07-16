"""tests/testgui/test_traces.py — end-to-end trace-accumulation tests
against a connected SimTransport (ticket 083-003).

``protocol.py``'s ``TLMFrame`` and ``traces.py``'s ``TraceModel.feed()``
already match the sprint-082 ``TLM`` format (``encpose=``/``otos=``/``pose=``
as absolute ``(x_mm, y_mm, heading_cdeg)`` triples) -- this module makes no
changes to ``traces.py``'s transform math (``_tw``/``_rw``/baseline
handling). It only verifies, end-to-end against a real (simulated) firmware,
that ``TraceModel`` actually accumulates plausible forward-motion traces --
closing the loop the architecture doc's Step 1 investigation opened but did
not itself verify.

Drives a connected ``SimTransport`` (ticket 083-001) directly -- bypassing
``KeyboardDriver`` entirely -- via ``transport.send("S 200 200")`` (097:
``DEV DT VW``/``DEV DT PORTS`` have no binary arm and never will -- the
legacy ``DEV`` debug command family was retired along with the rest of the
text plane; ``binary_bridge.translate_command()`` translates ``S`` into a
binary ``CommandEnvelope{drive: DrivetrainCommand{wheels}}``, the same
per-wheel-speed drive a real TestGUI session's S row Send button issues).

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_traces.py -q

Requires the compiled ``tests/_infra/sim/build/libfirmware_host.{dylib,so}``
(``just build-sim``) -- every test here skips cleanly if it is not present.

This module is not yet wired into ``pyproject.toml``'s ``testpaths`` (ticket
083-004's job, which also adds this directory's own fixtures/conftest) -- run
it directly, per ticket 083-003's Testing section.
"""
from __future__ import annotations

import time

import pytest

from robot_radio.testgui.transport import SimTransport, _sim_lib_path
from robot_radio.testgui.traces import TraceModel

pytestmark = pytest.mark.skipif(
    not _sim_lib_path().exists(),
    reason="sim lib not built -- run `just build-sim` first",
)

# Bounded wait for the tick-thread to deliver enough telemetry/truth samples.
# Generous relative to every observed run (SimTransport streams TLM at the
# sim's ~50 ms period and truth at ~5 Hz) so a slow CI box never flakes; a
# real hang/regression still fails the test rather than blocking forever.
_WAIT_TIMEOUT_S = 5.0
_POLL_INTERVAL_S = 0.02
# Minimum accumulated trace points before the "grows" assertions are trusted
# -- more than the single anchor/baseline point every _feed_* helper appends
# on its first call (see traces.py's TraceModel.feed() baseline handling).
_MIN_TRACE_POINTS = 5


def _wait_until(predicate, timeout_s: float = _WAIT_TIMEOUT_S) -> bool:
    """Poll ``predicate`` until it is truthy or ``timeout_s`` elapses."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(_POLL_INTERVAL_S)
    return predicate()


@pytest.fixture
def transport():
    """A connected SimTransport; disconnected on teardown even on failure."""
    t = SimTransport()
    t.on_log = lambda _s: None
    t.connect()
    assert t._connected, "SimTransport failed to connect -- is the sim lib built?"
    try:
        yield t
    finally:
        t.disconnect()


# ---------------------------------------------------------------------------
# (a) encoder / otos / fused traces all grow from on_telemetry -> TraceModel.feed()
# ---------------------------------------------------------------------------

def test_encoder_trace_grows_with_forward_drive_via_dead_reckoning(
    transport: SimTransport,
) -> None:
    """Driving the sim forward via binary-translated ``S`` and feeding the
    resulting ``TLMFrame``s into a ``TraceModel`` grows the ``encoder``
    trace with plausible forward-motion (+x, ~0 y) points.

    097: un-xfailed for ``encoder`` specifically. ``encpose`` still has NO
    wire representation in telemetry.proto (096-001's permanent trim,
    cited in ``TLMFrame.from_pb2()``'s own docstring) -- but
    ``TraceModel.feed()`` now dead-reckons an equivalent pose host-side via
    ``EncoderDeadReckoner``, integrated from ``frame.enc`` (cumulative
    per-wheel distance, always present), so the ``encoder`` trace grows
    regardless. See ``test_otos_fused_traces_still_flat_pending_098``
    below for why ``otos``/``fused`` do NOT (a genuinely separate,
    firmware-level gap this dead-reckoning fallback cannot paper over).
    """
    model = TraceModel()
    transport.on_telemetry = model.feed

    transport.send("S 200 200")

    assert _wait_until(lambda: len(model.encoder) >= _MIN_TRACE_POINTS), (
        f"encoder trace only reached {len(model.encoder)} points within "
        f"{_WAIT_TIMEOUT_S}s"
    )

    first_x, first_y = model.encoder[0]
    last_x, last_y = model.encoder[-1]
    # Anchor is (0, 0, 0) (feed() auto-anchors on first call, and the
    # sim starts at the origin) -- the first point is the zeroed
    # baseline, so it should sit at (approximately) the origin.
    assert abs(first_x) < 5.0 and abs(first_y) < 5.0, (
        f"encoder trace's first point {model.encoder[0]} is not near the origin"
    )
    # Driving straight forward (v_x=200 mm/s, v_y=0, omega=0) must grow
    # +x substantially and leave y close to zero.
    assert last_x > first_x + 1.0, (
        f"encoder trace did not move forward in x: {model.encoder}"
    )
    assert abs(last_y) < 5.0, (
        f"encoder trace drifted laterally during a straight drive: {model.encoder}"
    )


@pytest.mark.xfail(
    reason="not a wire/transport gap (097's own scope) -- "
           "Subsystems::PoseEstimator::tick() (source/subsystems/"
           "pose_estimator.cpp), the only producer of bb.fusedPose/otos "
           "state, is never called anywhere in source/ -- confirmed by "
           "grep (no call site exists) and by direct binary-telemetry "
           "probing (has_pose=True but pose stays (0,0); has_otos=False "
           "always) -- matching the (now-deleted) sim ctypes backend's own "
           "module docstring: 'there is no EKF/fusion loop anywhere in source/ this "
           "sprint'. Unlike encoder (097: now host-side dead-reckoned "
           "from frame.enc, see test_encoder_trace_grows_..._dead_"
           "reckoning above), otos/fused have no equivalent host-side "
           "fallback -- there is no raw sensor field to dead-reckon them "
           "from; they genuinely need sprint 098's fused-pose wiring.",
    strict=False,
)
def test_otos_fused_traces_still_flat_pending_098(transport: SimTransport) -> None:
    """``otos``/``fused`` traces do NOT grow yet -- still pending sprint 098.

    Companion to ``test_encoder_trace_grows_with_forward_drive_via_dead_
    reckoning`` above: same drive, but asserting the OPPOSITE for the two
    traces that genuinely need firmware fusion (no host-side fallback is
    possible for either, unlike encoder).
    """
    model = TraceModel()
    transport.on_telemetry = model.feed

    transport.send("S 200 200")

    assert _wait_until(lambda: len(model.otos) >= _MIN_TRACE_POINTS), (
        f"otos trace only reached {len(model.otos)} points within "
        f"{_WAIT_TIMEOUT_S}s"
    )
    assert _wait_until(lambda: len(model.fused) >= _MIN_TRACE_POINTS), (
        f"fused trace only reached {len(model.fused)} points within "
        f"{_WAIT_TIMEOUT_S}s"
    )

    for name, points in (
        ("otos", model.otos),
        ("fused", model.fused),
    ):
        first_x, first_y = points[0]
        last_x, last_y = points[-1]
        assert abs(first_x) < 5.0 and abs(first_y) < 5.0, (
            f"{name} trace's first point {points[0]} is not near the origin"
        )
        assert last_x > first_x + 1.0, (
            f"{name} trace did not move forward in x: {points}"
        )
        assert abs(last_y) < 5.0, (
            f"{name} trace drifted laterally during a straight drive: {points}"
        )


# ---------------------------------------------------------------------------
# (b) camera trace grows in step via on_truth -> feed_truth()
# ---------------------------------------------------------------------------

def test_camera_trace_grows_in_step_with_ground_truth(transport: SimTransport) -> None:
    """Feeding ground-truth poses (``SimTransport``'s ``on_truth`` callback,
    sourced from ``conn.get_true_pose()``) via ``feed_truth()`` grows the
    ``camera`` trace in step with a short straight drive, alongside the
    telemetry-fed traces.
    """
    model = TraceModel()
    transport.on_telemetry = model.feed

    def _on_truth(pose) -> None:
        if pose is not None:
            model.feed_truth(*pose)

    transport.on_truth = _on_truth

    transport.send("S 200 200")

    assert _wait_until(lambda: len(model.camera) >= 3), (
        f"camera trace only reached {len(model.camera)} points within "
        f"{_WAIT_TIMEOUT_S}s; camera={model.camera!r}"
    )
    # The telemetry-fed traces should be growing in step (not stalled while
    # only the camera trace advances, or vice versa).
    assert _wait_until(lambda: len(model.fused) >= 3), (
        f"fused trace only reached {len(model.fused)} points within "
        f"{_WAIT_TIMEOUT_S}s"
    )

    # feed_truth() appends the raw (x_cm, y_cm) truth pose directly -- unlike
    # the encoder/otos/fused traces it is NOT anchor-relative (see traces.py's
    # feed_truth() docstring), so the first sample already reflects however
    # far the robot travelled before the first ~5 Hz truth poll landed. The
    # meaningful assertion is growth (+x) and negligible lateral drift across
    # every sample, not proximity of the first sample to the origin.
    first_x, first_y = model.camera[0]
    last_x, last_y = model.camera[-1]
    assert last_x > first_x + 1.0, (
        f"camera trace did not move forward in x: {model.camera}"
    )
    assert all(abs(y) < 5.0 for _x, y in model.camera), (
        f"camera trace drifted laterally during a straight drive: {model.camera}"
    )
