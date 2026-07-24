"""src/tests/testgui/test_traces.py — end-to-end trace-accumulation tests
against a connected SimTransport (ticket 083-003).

``protocol.py``'s ``TLMFrame`` and ``traces.py``'s ``TraceModel.feed()``
already match the sprint-082 ``TLM`` format (``encpose=``/``otos=``/``pose=``
as absolute ``(x_mm, y_mm, heading_cdeg)`` triples) -- this module makes no
changes to ``traces.py``'s transform math (``_tw``/``_rw``/baseline
handling). It only verifies, end-to-end against a real (simulated) firmware,
that ``TraceModel`` actually accumulates plausible forward-motion traces --
closing the loop the architecture doc's Step 1 investigation opened but did
not itself verify.

Drives a connected ``SimTransport`` directly -- bypassing ``KeyboardDriver``
entirely -- via ``transport.protocol.twist()`` (108-007: ``SimLoop`` has no
generic wire/config-channel simulation surface at all, so ``send()``/
``command()`` text verbs no longer route to anything on Sim; a single
``twist(v_x, omega, duration)`` call arms the firmware's deadman for
``duration`` ms and the robot drives at that commanded twist until it
expires -- see ``src/firm/app/robot_loop.cpp``'s ``deadman_.arm()`` call --
so one call with a generous duration covers this file's whole wait window).

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest src/tests/testgui/test_traces.py -q

Most tests here require the compiled ``src/sim/build/libfirmware_host.
{dylib,so}`` (``just build-sim``) and are marked ``@_needs_sim_lib`` so they
skip cleanly if it is not present. 121-001's
``test_encoder_dead_reckoner_ingests_motion_tail_after_active_drops`` is the
one exception -- a pure synthetic-``TLMFrame`` test of ``TraceModel.feed()``'s
integrator-vs-append gating with no ``SimTransport``/sim-lib dependency at
all -- so it is NOT decorated with that marker and always runs.
"""
from __future__ import annotations

import math
import time

import pytest

from robot_radio.robot.protocol import TLMFrame
from robot_radio.testgui.transport import SimTransport, _sim_lib_path
from robot_radio.testgui.traces import EncoderDeadReckoner, TraceModel

# Applied per-test (not module-wide via `pytestmark`) so that
# test_encoder_dead_reckoner_ingests_motion_tail_after_active_drops below
# (121-001) -- a pure synthetic-frame test with no SimTransport/sim-lib
# dependency at all -- still runs on a box that hasn't built the sim lib.
_needs_sim_lib = pytest.mark.skipif(
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

@_needs_sim_lib
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

    protocol = transport.protocol
    assert protocol is not None
    protocol.twist(200.0, 0.0, 6000.0)

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


@_needs_sim_lib
@pytest.mark.xfail(
    reason="not a wire/transport gap (097's own scope) -- "
           "Subsystems::PoseEstimator::tick() (src/firm/subsystems/"
           "pose_estimator.cpp), the only producer of bb.fusedPose/otos "
           "state, is never called anywhere in src/firm/ -- confirmed by "
           "grep (no call site exists) and by direct binary-telemetry "
           "probing (has_pose=True but pose stays (0,0); has_otos=False "
           "always) -- matching the (now-deleted) sim ctypes backend's own "
           "module docstring: 'there is no EKF/fusion loop anywhere in src/firm/ this "
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

    protocol = transport.protocol
    assert protocol is not None
    protocol.twist(200.0, 0.0, 6000.0)

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

@_needs_sim_lib
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

    protocol = transport.protocol
    assert protocol is not None
    protocol.twist(200.0, 0.0, 6000.0)

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


# ---------------------------------------------------------------------------
# (c) 121-001: the encoder dead-reckoner ingests the motion tail even
#     while frame.active is False; only the trace-point APPEND is gated.
# ---------------------------------------------------------------------------

def test_encoder_dead_reckoner_ingests_motion_tail_after_active_drops() -> None:
    """Synthetic frame sequence, no SimTransport/sim-lib needed.

    Feeds ``TraceModel.feed()`` a differential-drive arc (both wheels
    advancing, right faster than left -- real translation AND heading
    change every step, unlike a pure in-place pivot) split into an
    "active" phase and a "tail" phase where ``frame.active`` has already
    dropped to ``False`` but ``frame.enc`` keeps advancing by the same
    per-step amount -- exactly the taper/coast tail described in
    ``encpose-active-gate-freezes-dead-reckoner-before-motion-ends.md``.

    Asserts BOTH halves of the 121-001 fix:
      (a) the integrator ingests the tail -- ``model.last_encpose`` must
          equal an independent reference ``EncoderDeadReckoner`` fed the
          IDENTICAL frame sequence unconditionally (the "all-frames"
          reckoner from the source issue's repro), not frozen at whatever
          it was after the last active frame.
      (b) the idle-growth guard the active gate exists for is preserved --
          the ``encoder`` trace's point count must NOT grow across the
          tail frames, even though the underlying enc values keep
          changing by far more than ``_TRACE_IDLE_EPSILON_CM`` per step.
    """
    left_step = 10.0    # [mm] per-step left-wheel travel
    right_step = 20.0   # [mm] per-step right-wheel travel (curving arc)
    active_steps = 12
    tail_steps = 4

    model = TraceModel(trackwidth=128.0)
    reference = EncoderDeadReckoner(trackwidth=128.0)

    def _feed_step(k: int, active: bool) -> tuple[int, int, int]:
        enc = (left_step * k, right_step * k)  # [mm] cumulative (left, right)
        model.feed(TLMFrame(t=k, enc=enc, active=active))
        return reference.update(*enc)

    # k=0 establishes both the reckoner's zero point and TraceModel's own
    # encpose baseline/anchor -- mirrors every other trace's "first
    # reading is the baseline" convention.
    ref_last = _feed_step(0, active=True)

    for k in range(1, active_steps + 1):
        ref_last = _feed_step(k, active=True)

    points_after_active = len(model.encoder)
    assert points_after_active > 1, (
        "encoder trace did not grow during the active phase -- test setup "
        "is not exercising the append path at all"
    )

    # The tail: frame.active is False, but enc keeps climbing by the same
    # per-step amount -- real wheel travel a finished-but-coasting motion
    # still produces for a few more frames.
    for k in range(active_steps + 1, active_steps + tail_steps + 1):
        ref_last = _feed_step(k, active=False)

    # (b) idle-growth guard preserved: no new points appended while idle,
    # regardless of how far enc has actually moved.
    assert len(model.encoder) == points_after_active, (
        f"encoder trace grew from {points_after_active} to "
        f"{len(model.encoder)} points while frame.active was False"
    )

    # (a) integrator fidelity: last_encpose must reflect the FULL wheel
    # travel (active phase + tail), matching a reckoner fed every frame
    # unconditionally -- not just the active-gated subset. Both reckoners
    # run the identical deterministic update() math over the identical
    # input sequence, so this is an exact match, not an approximation.
    assert model.last_encpose == ref_last, (
        f"model.last_encpose {model.last_encpose} does not match the "
        f"all-frames reference reckoner's final reading {ref_last} -- the "
        f"motion tail was not ingested"
    )
    # Sanity: the tail actually mattered (theta strictly grew across it) --
    # guards against a degenerate test that would pass even with the old,
    # buggy early-return.
    active_only = EncoderDeadReckoner(trackwidth=128.0)
    active_only_last = None
    for k in range(0, active_steps + 1):
        active_only_last = active_only.update(left_step * k, right_step * k)
    assert ref_last[2] > active_only_last[2], (
        "tail frames contributed no additional heading -- test is degenerate"
    )
