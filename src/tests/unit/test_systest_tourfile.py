"""Unit tests for src/tests/system/tourfile.py -- the tour-file grammar."""

import math
import sys
from pathlib import Path

import pytest

_SYSTEM_DIR = Path(__file__).resolve().parents[1] / "system"
sys.path.insert(0, str(_SYSTEM_DIR))

from tourfile import (  # noqa: E402
    CamfixStep, DbgStep, DwellStep, ExpectStep, MoveStep, SendStep, StopStep,
    Tour, TourParseError, parse_tour_file, parse_tour_text,
)

_TOURS_DIR = _SYSTEM_DIR / "tours"


def test_twist_distance_leg():
    tour = parse_tour_text("TWIST vx=150 dist=350 timeout=9", name="t")
    (step,) = tour.steps
    assert isinstance(step, MoveStep)
    assert step.variant == "twist"
    assert step.v_x == 150.0
    assert step.stop_kind == "dist"
    assert step.stop_value == 350.0
    assert step.timeout == 9.0


def test_twist_angle_converts_degrees():
    tour = parse_tour_text("TWIST omega=45 angle=90", name="t")
    (step,) = tour.steps
    assert step.omega == pytest.approx(math.radians(45.0))
    assert step.stop_kind == "angle"
    assert step.stop_value == pytest.approx(math.radians(90.0))
    # default timeout: 3x expected (2 s) = 6 s
    assert step.timeout == pytest.approx(6.0)


def test_timeout_floor():
    tour = parse_tour_text("TWIST vx=400 dist=40", name="t")
    (step,) = tour.steps
    assert step.timeout == 2.0  # 3 * 0.1 s < floor


def test_wheels_leg():
    tour = parse_tour_text("WHEELS left=100 right=-100 time=1.5", name="t")
    (step,) = tour.steps
    assert step.variant == "wheels"
    assert step.v_left == 100.0 and step.v_right == -100.0
    assert step.stop_kind == "time" and step.stop_value == 1.5


def test_exactly_one_stop_condition():
    with pytest.raises(TourParseError, match="exactly one stop condition"):
        parse_tour_text("TWIST vx=150 dist=100 time=2", name="t")
    with pytest.raises(TourParseError, match="exactly one stop condition"):
        parse_tour_text("TWIST vx=150", name="t")


def test_unknown_directive_rejected():
    with pytest.raises(TourParseError, match="unknown directive"):
        parse_tour_text("DRIVE fast", name="t")


def test_unknown_key_rejected():
    with pytest.raises(TourParseError, match="unknown key"):
        parse_tour_text("TWIST vx=150 dist=100 warp=9", name="t")


def test_stop_dwell_and_bare_stop():
    tour = parse_tour_text("STOP dwell=1.0\nTWIST vx=1 time=1\nSTOP", name="t")
    assert isinstance(tour.steps[0], StopStep)
    assert tour.steps[0].dwell == 1.0
    assert isinstance(tour.steps[2], StopStep)
    assert tour.steps[2].dwell == 0.0


def test_mark_is_dbg_sugar():
    tour = parse_tour_text("MARK leg1a\nTWIST vx=1 time=1", name="t")
    step = tour.steps[0]
    assert isinstance(step, DbgStep)
    assert step.text == "mark leg1a"


def test_dbg_passthrough():
    tour = parse_tour_text("DBG wedge left 1500\nTWIST vx=1 time=1", name="t")
    assert tour.steps[0].text == "wedge left 1500"


def test_send_verb():
    tour = parse_tour_text("SEND STATUS\nTWIST vx=1 time=1", name="t")
    step = tour.steps[0]
    assert isinstance(step, SendStep)
    assert step.verb == "STATUS" and step.data == ""


def test_expect_quoted_query_with_timeout():
    tour = parse_tour_text(
        "EXPECT '.type==\"status\" and .payload.ready' timeout=1.5\n"
        "TWIST vx=1 time=1", name="t")
    step = tour.steps[0]
    assert isinstance(step, ExpectStep)
    assert step.query == '.type=="status" and .payload.ready'
    assert step.timeout == 1.5


def test_camfix_required_and_units():
    tour = parse_tour_text("CAMFIX x=0 y=0 radius=50 heading=90 tol=2",
                           name="t")
    step = tour.steps[0]
    assert isinstance(step, CamfixStep)
    assert step.radius == 50.0
    assert step.heading == pytest.approx(math.radians(90.0))
    assert step.tol == pytest.approx(math.radians(2.0))
    with pytest.raises(TourParseError, match="CAMFIX needs radius"):
        parse_tour_text("CAMFIX x=0 y=0", name="t")


def test_comments_and_blanks_ignored():
    tour = parse_tour_text(
        "# a comment\n\nTWIST vx=150 dist=100  # trailing comment\n",
        name="t")
    assert len(tour.steps) == 1


def test_empty_tour_rejected():
    with pytest.raises(TourParseError, match="no steps"):
        parse_tour_text("# only a comment\n", name="t")


@pytest.mark.parametrize("name", ["square", "square_cw", "circle",
                                  "fault_wedge"])
def test_shipped_tours_parse(name):
    tour = parse_tour_file(_TOURS_DIR / f"{name}.tour")
    assert isinstance(tour, Tour)
    assert tour.steps
    # every shipped tour ends at rest or with a fix, never mid-move
    assert not isinstance(tour.steps[-1], MoveStep)
