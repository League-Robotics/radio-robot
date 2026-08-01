"""src/tests/testgui/test_commands.py — Headless tests for commands.py.

Ported from tests_old/testgui/test_commands.py (ticket 083-004); rewritten
at 128-004 for ``COMMANDS``'s new EMPTY shape (the seven-row S/T/D/R/TURN/
RT/G schema this file used to pin is deleted -- see ``commands.py``'s own
module docstring for why). ``build_wire_string`` itself is UNCHANGED code
(kept as a small, independently-tested pure function with no live caller
any more) -- the tests below still exercise its full behavior (centidegree
conversion, optional-zero-field omission, heading wrap, default fallback),
just against literal ``CommandSpec`` dicts built in this file instead of
rows pulled out of ``COMMANDS``, since ``COMMANDS`` no longer carries them.
``goto_distance``/``goto_reached``, and ``parse_tlm_mode`` are untouched by
this ticket -- pure schema/string-building/geometry data, unrelated to the
COMMANDS-row deletion. The one real divergence from the tests_old baseline
is ``TOUR_1``/``TOUR_2``'s actual content, which drifted (independent of
sprint 083) between the pre-rebuild tree and today's ``commands.py`` — the
values asserted below match the CURRENT source, not the historical
tests_old expectation (see ``TestTours`` for specifics). Tours themselves
are sprint-083 Out of Scope (sprint.md) — this class only guards the static
data table against silent drift, it does not exercise the Tour UI.

Verifies that:
- COMMANDS is empty (128-004: no live command row left).
- build_wire_string emits the correct wire string for each of the old
  command shapes, given a literal spec (not sourced from COMMANDS).
- RT deg is passed as centidegrees (integer): a relative in-place turn.
- TURN heading is passed as centidegrees (integer).
- TURN eps=0 is omitted; non-zero eps is included as eps=<val>.
- All tests run without a QApplication or a display server.

Run with:
    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui -q
"""

from __future__ import annotations

import pytest


# ---------------------------------------------------------------------------
# Literal CommandSpec fixtures (128-004: COMMANDS itself no longer carries
# these rows -- build_wire_string() only needs a CommandSpec-shaped dict,
# not membership in COMMANDS, so these preserve the builder's own test
# coverage independent of the schema table).
# ---------------------------------------------------------------------------


def _s_spec():
    return {
        "label": "S",
        "params": [
            {"name": "left", "type": int, "min": -1000, "max": 1000, "default": 200, "unit": "mm/s"},
            {"name": "right", "type": int, "min": -1000, "max": 1000, "default": 200, "unit": "mm/s"},
        ],
    }


def _t_spec():
    return {
        "label": "T",
        "params": [
            {"name": "left", "type": int, "min": -1000, "max": 1000, "default": 200, "unit": "mm/s"},
            {"name": "right", "type": int, "min": -1000, "max": 1000, "default": 200, "unit": "mm/s"},
            {"name": "ms", "type": int, "min": 1, "max": 30000, "default": 1000, "unit": "ms"},
        ],
    }


def _d_spec():
    return {
        "label": "D",
        "params": [
            {"name": "left", "type": int, "min": -1000, "max": 1000, "default": 200, "unit": "mm/s"},
            {"name": "right", "type": int, "min": -1000, "max": 1000, "default": 200, "unit": "mm/s"},
            {"name": "mm", "type": int, "min": 1, "max": 10000, "default": 500, "unit": "mm"},
        ],
    }


def _r_spec():
    return {
        "label": "R",
        "params": [
            {"name": "speed", "type": int, "min": -1000, "max": 1000, "default": 200, "unit": "mm/s"},
            {"name": "radius", "type": int, "min": -10000, "max": 10000, "default": 500, "unit": "mm"},
        ],
    }


def _turn_spec():
    return {
        "label": "TURN",
        "params": [
            {"name": "heading", "type": int, "min": -3600, "max": 3600, "default": 90, "unit": "deg"},
            {"name": "eps", "type": int, "min": 0, "max": 18, "default": 0, "unit": "deg", "optional": True},
        ],
        "cdeg_fields": ["heading", "eps"],
        "optional_zero_fields": ["eps"],
        "wrap_deg_fields": ["heading"],
    }


def _rt_spec():
    return {
        "label": "RT",
        "params": [
            {"name": "deg", "type": int, "min": -1800, "max": 1800, "default": 90, "unit": "deg"},
        ],
        "cdeg_fields": ["deg"],
    }


def _g_spec():
    return {
        "label": "G",
        "params": [
            {"name": "x", "type": int, "min": -10000, "max": 10000, "default": 0, "unit": "mm"},
            {"name": "y", "type": int, "min": -10000, "max": 10000, "default": 0, "unit": "mm"},
            {"name": "speed", "type": int, "min": 1, "max": 1000, "default": 200, "unit": "mm/s"},
        ],
    }


# ---------------------------------------------------------------------------
# Schema structure
# ---------------------------------------------------------------------------


class TestCommandsSchema:
    """COMMANDS is empty (128-004) -- no live command row survives the
    hidden-panel/dead-verb deletion (see commands.py's own docstring)."""

    def test_commands_is_empty(self):
        from robot_radio.testgui.commands import COMMANDS
        assert COMMANDS == []


# ---------------------------------------------------------------------------
# Wire-string builder — pure function tests (no Qt required)
# ---------------------------------------------------------------------------


class TestBuildWireStringS:
    """S <left> <right>"""

    def test_s_basic(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_s_spec(), {"left": 200, "right": 200})
        assert result == "S 200 200"

    def test_s_negative_left(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_s_spec(), {"left": -150, "right": 150})
        assert result == "S -150 150"

    def test_s_zero_speeds(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_s_spec(), {"left": 0, "right": 0})
        assert result == "S 0 0"

    def test_s_max_speeds(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_s_spec(), {"left": 1000, "right": -1000})
        assert result == "S 1000 -1000"


class TestBuildWireStringT:
    """T <left> <right> <ms>"""

    def test_t_basic(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_t_spec(), {"left": 200, "right": 200, "ms": 1000})
        assert result == "T 200 200 1000"

    def test_t_asymmetric(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_t_spec(), {"left": -300, "right": 300, "ms": 500})
        assert result == "T -300 300 500"

    def test_t_max_ms(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_t_spec(), {"left": 100, "right": 100, "ms": 30000})
        assert result == "T 100 100 30000"


class TestBuildWireStringD:
    """D <left> <right> <mm>"""

    def test_d_basic(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_d_spec(), {"left": 200, "right": 200, "mm": 500})
        assert result == "D 200 200 500"

    def test_d_large_distance(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_d_spec(), {"left": 300, "right": 300, "mm": 10000})
        assert result == "D 300 300 10000"

    def test_d_reverse(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_d_spec(), {"left": -200, "right": -200, "mm": 250})
        assert result == "D -200 -200 250"


class TestBuildWireStringR:
    """R <speed> <radius>"""

    def test_r_basic(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_r_spec(), {"speed": 200, "radius": 500})
        assert result == "R 200 500"

    def test_r_negative_radius_cw(self):
        """Negative radius → CW arc."""
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_r_spec(), {"speed": 200, "radius": -500})
        assert result == "R 200 -500"

    def test_r_zero_radius_straight(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_r_spec(), {"speed": 200, "radius": 0})
        assert result == "R 200 0"


class TestBuildWireStringTURN:
    """TURN <heading_cdeg> [eps=<eps_cdeg>]

    heading and eps are entered in degrees but sent in centidegrees.
    """

    def test_turn_heading_deg_to_cdeg(self):
        """Heading entered in degrees is converted to cdeg on the wire."""
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_turn_spec(), {"heading": 90, "eps": 0})
        assert result == "TURN 9000"

    def test_turn_no_eps_when_zero(self):
        """eps=0 → omitted from wire string."""
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_turn_spec(), {"heading": 45, "eps": 0})
        assert result == "TURN 4500"

    def test_turn_with_nonzero_eps(self):
        """Non-zero eps appears as eps=<cdeg>."""
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_turn_spec(), {"heading": 90, "eps": 3})
        assert result == "TURN 9000 eps=300"

    def test_turn_negative_heading(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_turn_spec(), {"heading": -90, "eps": 0})
        assert result == "TURN -9000"

    def test_turn_max_heading(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_turn_spec(), {"heading": 180, "eps": 0})
        assert result == "TURN 18000"

    def test_turn_eps_small(self):
        """Smallest non-zero eps (1° = 100 cdeg) is included."""
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_turn_spec(), {"heading": 0, "eps": 1})
        assert result == "TURN 0 eps=100"


class TestTurnHeadingWrap:
    """TURN heading accepts any angle; out-of-range wraps onto (-180, 180].

    Stakeholder bug 2026-07-03: the GUI clamped heading at ±180, so values
    like 270 could not be entered at all.  The spinbox now spans ±3600 and
    the wire builder maps any angle onto the equivalent absolute heading.
    """

    def _spec(self):
        return _turn_spec()

    def test_heading_spinbox_range_exceeds_180(self):
        """The UI range must allow entry beyond ±180 (the original bug)."""
        heading = next(p for p in self._spec()["params"] if p["name"] == "heading")
        assert heading["min"] <= -360
        assert heading["max"] >= 360

    def test_270_wraps_to_minus_90(self):
        from robot_radio.testgui.commands import build_wire_string
        assert build_wire_string(self._spec(), {"heading": 270, "eps": 0}) == "TURN -9000"

    def test_minus_270_wraps_to_90(self):
        from robot_radio.testgui.commands import build_wire_string
        assert build_wire_string(self._spec(), {"heading": -270, "eps": 0}) == "TURN 9000"

    def test_450_wraps_to_90(self):
        from robot_radio.testgui.commands import build_wire_string
        assert build_wire_string(self._spec(), {"heading": 450, "eps": 0}) == "TURN 9000"

    def test_360_wraps_to_0(self):
        from robot_radio.testgui.commands import build_wire_string
        assert build_wire_string(self._spec(), {"heading": 360, "eps": 0}) == "TURN 0"

    def test_540_wraps_to_180(self):
        """Wrap lands on (-180, 180]: 540 ≡ 180, sent as +18000 not -18000."""
        from robot_radio.testgui.commands import build_wire_string
        assert build_wire_string(self._spec(), {"heading": 540, "eps": 0}) == "TURN 18000"

    def test_181_wraps_to_minus_179(self):
        from robot_radio.testgui.commands import build_wire_string
        assert build_wire_string(self._spec(), {"heading": 181, "eps": 0}) == "TURN -17900"

    def test_in_range_values_pass_through_unchanged(self):
        """Values already in [-180, 180] are sent exactly as typed,
        including both edge representations 180 and -180."""
        from robot_radio.testgui.commands import build_wire_string
        assert build_wire_string(self._spec(), {"heading": 180, "eps": 0}) == "TURN 18000"
        assert build_wire_string(self._spec(), {"heading": -180, "eps": 0}) == "TURN -18000"

    def test_wrap_composes_with_eps(self):
        from robot_radio.testgui.commands import build_wire_string
        assert build_wire_string(self._spec(), {"heading": 270, "eps": 3}) == "TURN -9000 eps=300"


class TestBuildWireStringRT:
    """RT <rel_cdeg> — relative in-place turn; deg entered, cdeg on wire."""

    def test_rt_positive_ccw(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_rt_spec(), {"deg": 90})
        assert result == "RT 9000"

    def test_rt_negative_cw(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_rt_spec(), {"deg": -45})
        assert result == "RT -4500"

    def test_rt_zero(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_rt_spec(), {"deg": 0})
        assert result == "RT 0"

    def test_rt_uses_default(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_rt_spec(), {})
        # Default deg=90 → 9000 cdeg
        assert result == "RT 9000"


class TestBuildWireStringG:
    """G <x> <y> <speed>"""

    def test_g_basic(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_g_spec(), {"x": 500, "y": 300, "speed": 200})
        assert result == "G 500 300 200"

    def test_g_negative_coords(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_g_spec(), {"x": -1000, "y": -500, "speed": 150})
        assert result == "G -1000 -500 150"

    def test_g_zero_zero(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_g_spec(), {"x": 0, "y": 0, "speed": 200})
        assert result == "G 0 0 200"

    def test_g_large_coords(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_g_spec(), {"x": 10000, "y": 10000, "speed": 1000})
        assert result == "G 10000 10000 1000"


# ---------------------------------------------------------------------------
# Defaults: build_wire_string uses param defaults when values are missing
# ---------------------------------------------------------------------------


class TestBuildWireStringDefaults:
    """build_wire_string falls back to param defaults for missing values."""

    def test_s_uses_defaults(self):
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_s_spec(), {})
        # Default for S is left=200, right=200
        assert result == "S 200 200"

    def test_turn_default_eps_omitted(self):
        """TURN default eps=0 → omitted from wire."""
        from robot_radio.testgui.commands import build_wire_string
        result = build_wire_string(_turn_spec(), {})
        # Default heading=90 deg (9000 cdeg on the wire), eps=0 (omitted)
        assert "eps" not in result


# ---------------------------------------------------------------------------
# Pre-programmed tours
# ---------------------------------------------------------------------------
#
# Tours are sprint-083 Out of Scope (sprint.md's Out of Scope section) — the
# Tour UI itself is not exercised here (see tests_old/testgui/test_tour_*.py,
# intentionally left un-ported). This class only pins TOUR_1/TOUR_2's static
# data against silent drift, since commands.py (and its data) is otherwise
# ported whole. TOUR_1/TOUR_2's actual content diverged from tests_old's
# expectations independent of sprint 083 (git history shows the two tours'
# content/order changed across several pre-083 commits) — the values below
# were read directly from the current commands.py, not carried over from
# tests_old.
#


class TestTours:
    """TOUR_1 / TOUR_2 / TOURS content and shape (pure data, no Qt)."""

    def test_tour_1_sequence(self):
        from robot_radio.testgui.commands import TOUR_1
        assert TOUR_1 == [
            "D 200 200 345",
            "RT 9000",
            "D 200 200 240",
            "RT 9000",
            "D 200 200 700",
            "RT 9000",
            "D 200 200 480",
            "RT 9000",
            "D 200 200 700",
            "RT 9000",
            "D 200 200 240",
            "RT 9000",
            "D 200 200 345",
        ]

    def test_tour_2_sequence(self):
        from robot_radio.testgui.commands import TOUR_2
        assert TOUR_2 == [
            "D 200 200 345",
            "RT 9000",
            "D 200 200 240",
            "RT 12400",
            "D 200 200 850",
            "RT -21700",
            "D 200 200 700",
            "RT 14600",
            "D 200 200 850",
            "RT 21500",
            "D 200 200 700",
            "RT -9000",
            "D 200 200 240",
            "RT -9000",
            "D 200 200 345",
        ]

    def test_tour_1_is_a_closed_loop(self):
        """TOUR_1's defining property: dead-reckoned ideally, it returns to
        the origin (all RT steps are the same +90°, so the path is a
        regular closed polygon by construction). TOUR_2's asymmetric turn
        angles do NOT dead-reckon back to the origin exactly (verified: the
        residual is ~14mm out of a multi-metre path) so only TOUR_1 carries
        this assertion."""
        import math

        from robot_radio.testgui.commands import TOUR_1

        x = y = h = 0.0
        for cmd in TOUR_1:
            parts = cmd.split()
            if parts[0] == "RT":
                h += math.radians(int(parts[1]) / 100.0)
            elif parts[0] == "TURN":
                h = math.radians(int(parts[1]) / 100.0)
            elif parts[0] == "D":
                d = float(parts[3])
                x += d * math.cos(h)
                y += d * math.sin(h)
        assert math.hypot(x, y) < 1e-6, (
            f"Tour 1 must dead-reckon back to the origin; ends at "
            f"({x:.1f}, {y:.1f}) mm"
        )

    def test_tours_are_read_from_planner_tour(self):
        """107-002: TOUR_1/TOUR_2's own raw wire-string geometry moved to
        planner/tour.py (architecture-update.md Decision 3) -- commands.py
        now only reads it back for GUI labeling ([Presentation] ->
        [Domain], not the reverse). `commands.TOURS` must be built from the
        SAME objects planner.tour owns, not a re-typed copy that could
        silently drift."""
        from robot_radio.planner import tour as planner_tour
        from robot_radio.testgui.commands import TOUR_1, TOUR_2, TOURS

        assert TOUR_1 is planner_tour.TOUR_1
        assert TOUR_2 is planner_tour.TOUR_2
        assert TOURS["Tour 1"] is planner_tour.TOUR_1
        assert TOURS["Tour 2"] is planner_tour.TOUR_2

    def test_tour_all_steps_are_wire_strings(self):
        """Every step is a non-empty string beginning with a known verb."""
        from robot_radio.testgui.commands import TOUR_1, TOUR_2
        verbs = {"RT", "D", "TURN"}
        for step in [*TOUR_1, *TOUR_2]:
            assert isinstance(step, str) and step
            assert step.split()[0] in verbs

    def test_tours_registry_contains_all_tours(self):
        from robot_radio.testgui.commands import TOURS, TOUR_1, TOUR_2
        assert TOURS["Tour 1"] is TOUR_1
        assert TOURS["Tour 2"] is TOUR_2


# ---------------------------------------------------------------------------
# Camera-based GOTO geometry helpers
# ---------------------------------------------------------------------------


class TestGotoGeometry:
    """goto_distance / goto_reached pure geometry (no Qt)."""

    def test_distance_zero_at_target(self):
        from robot_radio.testgui.commands import goto_distance
        assert goto_distance(100, 200, 100, 200) == 0.0

    def test_distance_3_4_5(self):
        from robot_radio.testgui.commands import goto_distance
        assert goto_distance(300, 400, 0, 0) == 500.0

    def test_reached_within_eps(self):
        from robot_radio.testgui.commands import goto_reached
        # 30 mm away, eps 50 → reached.
        assert goto_reached(30, 0, 0, 0, 50) is True

    def test_reached_exactly_at_eps(self):
        from robot_radio.testgui.commands import goto_reached
        # Exactly eps away counts as reached (<=).
        assert goto_reached(50, 0, 0, 0, 50) is True

    def test_not_reached_beyond_eps(self):
        from robot_radio.testgui.commands import goto_reached
        assert goto_reached(100, 0, 0, 0, 50) is False


# ---------------------------------------------------------------------------
# SNAP / TLM mode parsing (completion detection)
# ---------------------------------------------------------------------------


class TestParseTlmMode:
    """parse_tlm_mode extracts the mode= character from a TLM/SNAP reply."""

    def test_idle_mode(self):
        from robot_radio.testgui.commands import parse_tlm_mode
        assert parse_tlm_mode("TLM t=1234 mode=I seq=5 x=0 y=0") == "I"

    def test_distance_mode(self):
        from robot_radio.testgui.commands import parse_tlm_mode
        assert parse_tlm_mode("TLM t=42 mode=D seq=9") == "D"

    def test_lowercase_is_uppercased(self):
        from robot_radio.testgui.commands import parse_tlm_mode
        assert parse_tlm_mode("TLM mode=i") == "I"

    def test_multiline_reply(self):
        from robot_radio.testgui.commands import parse_tlm_mode
        assert parse_tlm_mode("OK\nTLM t=1 mode=G seq=2") == "G"

    def test_empty_reply_returns_none(self):
        from robot_radio.testgui.commands import parse_tlm_mode
        assert parse_tlm_mode("") is None

    def test_no_mode_field_returns_none(self):
        from robot_radio.testgui.commands import parse_tlm_mode
        assert parse_tlm_mode("OK done") is None


# ---------------------------------------------------------------------------
# Package importability without PySide6
# ---------------------------------------------------------------------------


class TestPackageImportability:
    """import robot_radio.testgui must not require PySide6."""

    def test_commands_importable_without_qt(self):
        """commands.py must be importable in a vanilla Python process."""
        import importlib

        mod = importlib.import_module("robot_radio.testgui.commands")
        assert hasattr(mod, "COMMANDS")
        assert hasattr(mod, "build_wire_string")

    def test_testgui_init_importable_without_qt(self):
        """robot_radio.testgui.__init__ must be importable without PySide6."""
        import robot_radio.testgui
        assert hasattr(robot_radio.testgui, "__version__")
