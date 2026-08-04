"""src/tests/unit/test_estop_tail_measurement.py -- 133-003: synthetic-data
unit tests for `src/tests/bench/estop_unlosable_bench.py`'s pure tail
measurement, no hardware required.

The property under test is the one the ticket calls out by name: travel is
measured from the **commanded-zero transition**, not from the baseline
frame. An earlier harness anchored on the baseline (captured during the
leading settle window, before the leg even started) and therefore charged
the whole commanded leg to the tail, inflating every figure by roughly
150 mm. That is a pure arithmetic bug over a frame list, so it is testable
without a robot -- and it is worth testing, because the inflated number
looks entirely plausible.

`src/tests/bench/` is "HITL CLI tools, not pytest-collected"
(`src/tests/CLAUDE.md`), so this loads the bench script by file path via
importlib, mirroring `test_wheel_controller_ab_bench.py`'s established
precedent.
"""
from __future__ import annotations

import importlib.util
import pathlib
import sys
import types

import pytest

_BENCH_SCRIPT = (pathlib.Path(__file__).resolve().parents[1]
                 / "bench" / "estop_unlosable_bench.py")


def _load_bench_module():
    spec = importlib.util.spec_from_file_location("estop_unlosable_bench",
                                                  _BENCH_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # Register BEFORE exec -- the same dataclass/`from __future__ import
    # annotations` interaction test_wheel_controller_ab_bench.py documents.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def bench():
    return _load_bench_module()


def _reading(position: float):
    """A stand-in for `EncoderReading` -- the capture only ever reads
    `.position` off it."""
    return types.SimpleNamespace(position=position)


def _frame(position_left: float, position_right: float, *,
           active: "bool | None" = None):
    return types.SimpleNamespace(enc_left=_reading(position_left),
                                 enc_right=_reading(position_right),
                                 active=active)


def _capture(bench, rows):
    """Build a TailCapture from `(recvTime, positionLeft, positionRight
    [, active])` rows, bypassing the wire entirely."""
    capture = bench.TailCapture()
    for row in rows:
        received, left, right = row[0], row[1], row[2]
        active = row[3] if len(row) > 3 else None
        capture.frames.append((received, _frame(left, right, active=active)))
    return capture


# ---------------------------------------------------------------------------
# The anchor -- which frame travel is measured FROM
# ---------------------------------------------------------------------------

def test_anchor_is_the_last_frame_at_or_before_the_commanded_zero_instant(bench):
    # A leg from t=0 to t=1.0 covering 150 mm, then a tail.
    capture = _capture(bench, [
        (0.0, 0.0, 0.0),
        (0.5, 75.0, 75.0),
        (1.0, 150.0, 150.0),
        (1.5, 155.0, 155.0),
    ])
    anchor_at, positions = capture.anchor(1.02)
    assert anchor_at == 1.0
    assert positions == (150.0, 150.0)


def test_anchor_falls_back_to_the_first_frame_when_none_precedes(bench):
    """The transition can legitimately precede every frame in the capture
    (a halt sent before the first push arrived). Falling back to the first
    frame under-counts slightly; returning None would discard the trial."""
    capture = _capture(bench, [(1.0, 150.0, 150.0), (1.5, 155.0, 155.0)])
    anchor_at, positions = capture.anchor(0.4)
    assert anchor_at == 1.0
    assert positions == (150.0, 150.0)


def test_anchor_is_none_when_no_frame_carries_positions(bench):
    capture = bench.TailCapture()
    capture.frames.append(
        (0.0, types.SimpleNamespace(enc_left=None, enc_right=None, active=True)))
    assert capture.anchor(0.0) is None


# ---------------------------------------------------------------------------
# The measurement bug this harness carries a fix for
# ---------------------------------------------------------------------------

def test_travel_is_measured_from_the_transition_not_the_baseline_frame(bench):
    """THE regression this file exists for.

    A 150 mm leg followed by a 5 mm coast. Anchored correctly, the tail is
    5 mm. Anchored on the baseline frame -- which is what the earlier
    harness did -- it reads 155 mm, and 150 mm of perfectly ordinary
    commanded travel gets reported as a stop-path failure.
    """
    capture = _capture(bench, [
        (0.0, 0.0, 0.0),      # baseline, captured during the settle window
        (0.5, 75.0, 75.0),
        (1.0, 150.0, 150.0),  # commanded zero lands here
        (1.5, 155.0, 155.0),
        (2.0, 155.0, 155.0),
    ])
    commanded_zero_at = 1.0

    _, anchor_positions = capture.anchor(commanded_zero_at)
    travel = capture.travel_after(anchor_positions)
    assert travel == pytest.approx((5.0, 5.0))

    # And the wrong way round, stated explicitly so the difference is a
    # documented number rather than an implied one.
    baseline_travel = capture.travel_after((0.0, 0.0))
    assert baseline_travel == pytest.approx((155.0, 155.0))
    assert baseline_travel[0] - travel[0] == pytest.approx(150.0)


def test_travel_reports_each_wheel_signed_and_independently(bench):
    """An L/R split in the tail is a real finding (one wheel's zero write
    landed, the other's did not), so the two are never collapsed."""
    capture = _capture(bench, [
        (1.0, 150.0, 150.0),
        (2.0, 152.0, 940.0),   # right wheel ran away; left coasted 2 mm
    ])
    _, anchor_positions = capture.anchor(1.0)
    assert capture.travel_after(anchor_positions) == pytest.approx((2.0, 790.0))


# ---------------------------------------------------------------------------
# The `silent` tail's anchor -- the robot's own transition
# ---------------------------------------------------------------------------

def test_active_fell_finds_the_true_to_false_transition(bench):
    capture = _capture(bench, [
        (0.0, 0.0, 0.0, False),     # pre-leg idle -- must NOT count
        (0.5, 75.0, 75.0, True),
        (1.0, 150.0, 150.0, True),
        (1.5, 155.0, 155.0, False),  # this is the transition
        (2.0, 155.0, 155.0, False),
    ])
    assert capture.active_fell() == 1.5


def test_active_fell_is_none_when_the_leg_was_never_observed_active(bench):
    """Not a detail: it means the drive command never took, and the caller
    must fall back to the commanded deadline AND say so rather than
    silently anchoring somewhere else."""
    capture = _capture(bench, [
        (0.0, 0.0, 0.0, False),
        (0.5, 0.0, 0.0, False),
    ])
    assert capture.active_fell() is None


def test_active_fell_ignores_frames_that_never_decoded_the_flag(bench):
    capture = _capture(bench, [
        (0.5, 75.0, 75.0, True),
        (1.0, 150.0, 150.0, None),   # flags absent -- carries no opinion
        (1.5, 155.0, 155.0, False),
    ])
    assert capture.active_fell() == 1.5


# ---------------------------------------------------------------------------
# Settle time
# ---------------------------------------------------------------------------

def test_settled_at_reports_time_to_the_last_real_position_change(bench):
    capture = _capture(bench, [
        (1.0, 150.0, 150.0),
        (1.2, 153.0, 153.0),
        (1.4, 155.0, 155.0),   # last real change
        (2.0, 155.0, 155.0),
        (3.0, 155.0, 155.0),
    ])
    assert capture.settled_at(1.0) == pytest.approx(0.4)


def test_settled_at_ignores_sub_millimetre_encoder_dither(bench):
    """An encoder at rest dithers well under a millimetre. Counting that as
    motion would make every tail read as never settling, which would make
    the number useless rather than conservative."""
    capture = _capture(bench, [
        (1.0, 150.0, 150.0),
        (1.4, 155.0, 155.0),
        (2.0, 155.1, 154.9),
        (3.0, 155.0, 155.05),
    ])
    assert capture.settled_at(1.0) == pytest.approx(0.4)


def test_settled_at_is_zero_when_the_wheels_never_moved_again(bench):
    capture = _capture(bench, [
        (1.0, 150.0, 150.0),
        (2.0, 150.0, 150.0),
    ])
    assert capture.settled_at(1.0) == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# The tail catalogue itself
# ---------------------------------------------------------------------------

def test_all_four_tails_are_declared(bench):
    """The ticket names exactly these four. `stream` runs last on purpose --
    it is the control case (see the module docstring)."""
    assert bench.TAIL_NAMES == ("silent", "estop", "wheels0", "stream")
    assert bench.TAIL_NAMES[-1] == "stream"
