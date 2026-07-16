"""src/tests/testgui/test_trace_idle_dedup.py — pure-logic tests for
``TraceModel``'s idle-jitter trace-growth guard (``_append_if_moved()`` /
``_TRACE_IDLE_EPSILON_CM``, added alongside the TestGUI Sim command-surface
fix).

No Qt, no compiled sim lib, no live transport -- feeds synthetic
``TLMFrame``s directly into a bare ``TraceModel``, exercising exactly the
bug this guard fixes: Sim mode's tick thread streams telemetry forever once
connected, and ticket 108-011's rest-encoder dither means an idle sim's
encoder-derived pose micro-jitters by a fraction of a millimetre every
frame -- without the guard, every such frame appends a new trace point and
the trace grows without bound even though nothing is actually moving.
"""
from __future__ import annotations

from robot_radio.robot.protocol import TLMFrame
from robot_radio.testgui.traces import TraceModel, _TRACE_IDLE_EPSILON_CM


def _frame(encpose: tuple[int, int, int]) -> TLMFrame:
    return TLMFrame(encpose=encpose)


def test_repeated_identical_frames_do_not_grow_encoder_trace() -> None:
    """A genuinely idle connection re-delivers the SAME pose frame after
    frame (no motion at all) -- the baseline-diff is exactly zero every
    time, which must never re-append. This is the degenerate case of the
    idle-jitter guard: real rest-dither aliases in and out of the integer
    wire mm resolution (see this module's own docstring), but a truly
    unmoving frame is the clearest possible "must not grow" input."""
    model = TraceModel()

    model.feed(_frame((0, 0, 0)))
    assert len(model.encoder) == 1

    for _ in range(50):
        model.feed(_frame((0, 0, 0)))

    assert len(model.encoder) == 1, (
        f"repeated identical frames grew the encoder trace to "
        f"{len(model.encoder)} points; expected it to stay at the single "
        f"baseline point"
    )


def test_repeated_identical_frames_do_not_grow_otos_or_fused_traces() -> None:
    """Same guard applies to the ``otos``/``fused`` trace helpers."""
    model = TraceModel()

    model.feed(TLMFrame(otos=(0, 0, 0), pose=(0, 0, 0)))
    assert len(model.otos) == 1
    assert len(model.fused) == 1

    for _ in range(50):
        model.feed(TLMFrame(otos=(0, 0, 0), pose=(0, 0, 0)))

    assert len(model.otos) == 1
    assert len(model.fused) == 1


def test_append_if_moved_drops_sub_epsilon_deltas() -> None:
    """Direct unit test of the ``_append_if_moved()`` guard itself (the
    exact quantity the module docstring's dither-amplitude reasoning is
    about) -- a fraction-of-a-millimetre wiggle, well under
    ``_TRACE_IDLE_EPSILON_CM``, must not append; a step past it must."""
    trace: list[tuple[float, float]] = []

    TraceModel._append_if_moved(trace, (0.0, 0.0))
    assert trace == [(0.0, 0.0)]

    # +-0.01cm (0.1mm) jitter around the last point -- 5x below the 0.05cm
    # threshold in magnitude.
    TraceModel._append_if_moved(trace, (0.01, 0.0))
    TraceModel._append_if_moved(trace, (-0.01, 0.0))
    TraceModel._append_if_moved(trace, (0.005, -0.005))
    assert trace == [(0.0, 0.0)], (
        f"sub-epsilon deltas should not append; got {trace}"
    )

    # A step clearly past the threshold appends.
    TraceModel._append_if_moved(trace, (0.5, 0.0))
    assert trace == [(0.0, 0.0), (0.5, 0.0)]


def test_real_motion_still_appends_past_epsilon() -> None:
    """A displacement clearly larger than the epsilon threshold must still
    append -- the guard filters idle noise only, not real motion."""
    model = TraceModel()

    model.feed(_frame((0, 0, 0)))
    assert len(model.encoder) == 1

    # 5mm >> the 0.5mm epsilon -- a real drive-commanded step.
    model.feed(_frame((50, 0, 0)))
    assert len(model.encoder) == 2, (
        "a displacement well past the idle epsilon should append a new point"
    )

    last_x, _last_y = model.encoder[-1]
    first_x, _first_y = model.encoder[0]
    assert last_x > first_x, "the appended point should reflect the forward move"


def test_epsilon_boundary_value() -> None:
    """Sanity check the module constant itself: comfortably above the
    dither amplitude (+-0.1mm) and comfortably below a real per-tick
    displacement at any commanded speed."""
    assert 0.0 < _TRACE_IDLE_EPSILON_CM < 1.0


# ---------------------------------------------------------------------------
# frame.active gating (OOP sim-motor-state fix) — the stronger, authoritative
# gate on top of the epsilon dead-band above: an explicit active=False must
# freeze every trace list outright, not just filter sub-epsilon jitter.
# ---------------------------------------------------------------------------


def test_active_false_frame_does_not_append_to_any_trace() -> None:
    """A frame explicitly reporting ``active=False`` (motion finished) must
    not append to ANY trace list, even carrying a real, past-epsilon
    displacement -- the motion-state gate is authoritative over the
    epsilon dead-band, which only filters small jitter."""
    model = TraceModel()
    model.feed(TLMFrame(encpose=(0, 0, 0), otos=(0, 0, 0), pose=(0, 0, 0), active=True))
    assert len(model.encoder) == 1
    assert len(model.otos) == 1
    assert len(model.fused) == 1

    # A frame carrying a real (5mm, well past epsilon) displacement, but
    # tagged active=False, must be dropped outright.
    model.feed(TLMFrame(encpose=(50, 0, 0), otos=(50, 0, 0), pose=(50, 0, 0), active=False))
    assert len(model.encoder) == 1, "active=False must freeze the encoder trace"
    assert len(model.otos) == 1, "active=False must freeze the otos trace"
    assert len(model.fused) == 1, "active=False must freeze the fused trace"


def test_active_true_frame_appends_normally() -> None:
    """A frame explicitly reporting ``active=True`` behaves exactly like the
    pre-fix (no ``active`` field) behavior -- normal epsilon-gated append."""
    model = TraceModel()
    model.feed(TLMFrame(encpose=(0, 0, 0), active=True))
    assert len(model.encoder) == 1

    model.feed(TLMFrame(encpose=(50, 0, 0), active=True))
    assert len(model.encoder) == 2, "active=True with real displacement should append"


def test_active_none_frame_keeps_prior_epsilon_gated_behavior() -> None:
    """``active is None`` (older/pre-fault frames that never set the field)
    must NOT be treated as a confirmed idle -- it falls through to the
    pre-existing epsilon dead-band, unaffected by this fix. This is exactly
    ``_frame()``'s frame shape used throughout the rest of this module, so
    every test above it continues to hold."""
    model = TraceModel()
    model.feed(_frame((0, 0, 0)))
    assert len(model.encoder) == 1

    model.feed(_frame((50, 0, 0)))
    assert len(model.encoder) == 2, (
        "active=None (unknown) must still allow a real displacement to append"
    )


def test_point_count_freezes_after_motion_completes() -> None:
    """End-to-end shape of the reported bug: a burst of active=True motion
    frames followed by a run of active=False idle frames (mirroring a
    finished SEG/TURN in Sim mode) must leave the trace's point count
    frozen at whatever it reached during the active burst -- this is the
    "clear stop" the fix delivers."""
    model = TraceModel()

    for i in range(5):
        model.feed(TLMFrame(encpose=(i * 20, 0, 0), active=True))
    count_at_stop = len(model.encoder)
    assert count_at_stop > 1

    for i in range(50):
        # Idle frames still carry a (tiny, rest-dither-shaped) changing
        # encpose -- the exact "point count climbs, resets, keeps counting"
        # symptom this fix targets -- but are tagged active=False.
        model.feed(TLMFrame(encpose=(100 + i, 0, 0), active=False))

    assert len(model.encoder) == count_at_stop, (
        f"trace kept growing after active went False: "
        f"{count_at_stop} -> {len(model.encoder)}"
    )
