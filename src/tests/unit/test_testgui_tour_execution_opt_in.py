"""The TestGUI's rest-to-rest opt-in and its turn-angle unwrap.

Both live in ``robot_radio.testgui.commands``, which imports no PySide6 (see
that module's own docstring), so they are testable without a display server
or a Qt application instance -- which is the point of putting them there
rather than inside ``__main__.py``'s GUI-construction closure.

Scope note: the GUI's per-segment REPORTING is exercised end to end by
``src/tests/testgui/test_gui_button_acceptance.py`` against the compiled
firmware simulator. What is pinned here is the part that has a right answer
independent of any plant: which execution a tour dispatches, and how an
achieved turn angle is resolved onto the correct whole-turn branch.
"""

from __future__ import annotations

import pytest

from robot_radio.planner.tour import PIPELINED_EXECUTION, SQUARE_EXECUTION
from robot_radio.testgui.commands import tour_execution, unwrap_angle_toward


# ---------------------------------------------------------------------------
# Which execution a tour button dispatches
# ---------------------------------------------------------------------------


def test_pipelined_tours_are_unchanged_when_the_box_is_unchecked() -> None:
    """Default behaviour is the historical one -- the checkbox is opt-in.

    Tour 1 / Tour 2 have always run the one-leg lookahead. An operator who
    never touches the new control must get exactly the tour they got before
    it existed; a silent promotion would change what every previously
    captured Tour 1 result means.
    """
    assert tour_execution("Tour 1") == PIPELINED_EXECUTION
    assert tour_execution("Tour 2") == PIPELINED_EXECUTION
    assert tour_execution("Tour 1").sequential is False


@pytest.mark.parametrize("name", ["Tour 1", "Tour 2"])
def test_rest_to_rest_promotes_a_pipelined_tour_onto_the_measured_execution(
    name: str,
) -> None:
    """Checked, a pipelined tour runs the sequencing sprint 134 measured.

    Not merely "sequential=True": the settle and the pivot rate are part of
    what was measured (ticket 134-004, median 6.3 mm closure). A promotion
    that kept `PlannerParams.omega_max` would pivot at 1.2 rad/s, near the
    wheels' breakaway dead zone, and arc -- 18.0 mm vs 6.1 mm closure,
    measured. So the whole execution is adopted, not the flag.
    """
    execution = tour_execution(name, rest_to_rest=True)
    assert execution == SQUARE_EXECUTION
    assert execution.sequential is True
    assert execution.settle == pytest.approx(1.2)  # [s]
    assert execution.omega_max == pytest.approx(2.4)  # [rad/s]


@pytest.mark.parametrize("rest_to_rest", [False, True])
def test_square_is_never_altered_by_the_checkbox(rest_to_rest: bool) -> None:
    """"Square" runs its own measured execution whatever the box says.

    Its sequencing is the one with hardware numbers behind it; the control
    exists to bring the OTHER tours to it, and must not become a way to
    change it.
    """
    assert tour_execution("Square", rest_to_rest=rest_to_rest) == SQUARE_EXECUTION


# ---------------------------------------------------------------------------
# Turn-angle unwrap
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "raw, command, expected",  # [deg] [deg] [deg]
    [
        # Ordinary near-miss: already on the right branch, left alone.
        (89.6, 90.0, 89.6),
        (-88.2, -90.0, -88.2),
        # A full revolution reads as ~0 in a difference of absolute
        # headings -- the whole reason this helper exists.
        (-10.0, 360.0, 350.0),
        (2.5, 360.0, 362.5),
        # A leg straddling the +/-180 seam reads with the wrong sign.
        (-179.0, 90.0, 181.0),
        (179.0, -90.0, -181.0),
        # In-leg drift on a straight leg unwraps toward zero.
        (359.7, 0.0, -0.3),
        (0.0, 0.0, 0.0),
    ],
)
def test_unwrap_angle_toward_picks_the_branch_nearest_the_command(
    raw: float, command: float, expected: float
) -> None:
    assert unwrap_angle_toward(raw, command) == pytest.approx(expected)


def test_unwrap_angle_toward_never_moves_by_a_partial_turn() -> None:
    """The correction is always a whole number of revolutions.

    That is what makes the helper safe to apply to a MEASURED angle: it can
    resolve which branch the reading is on, but it can never quietly edit
    the reading toward the command and flatter the result.
    """
    for raw in (-540.0, -37.5, 0.0, 12.25, 400.0):
        for command in (-360.0, -90.0, 0.0, 90.0, 360.0):
            delta = unwrap_angle_toward(raw, command) - raw
            assert delta % 360.0 == pytest.approx(0.0, abs=1e-9)
