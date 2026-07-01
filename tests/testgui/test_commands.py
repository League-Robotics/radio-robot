"""tests/testgui/test_commands.py — Headless tests for commands.py.

Verifies that:
- COMMANDS contains seven entries for S, T, D, R, TURN, RT, G.
- build_wire_string emits the correct wire string for each command.
- RT deg is passed as centidegrees (integer): a relative in-place turn.
- TURN heading is passed as centidegrees (integer).
- TURN eps=0 is omitted; non-zero eps is included as eps=<val>.
- All tests run without a QApplication or a display server.

Run with:
    QT_QPA_PLATFORM=offscreen uv run --with pytest python -m pytest tests/testgui -q
"""

from __future__ import annotations

import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _labels():
    from robot_radio.testgui.commands import COMMANDS
    return [spec["label"] for spec in COMMANDS]


# ---------------------------------------------------------------------------
# Schema structure
# ---------------------------------------------------------------------------


class TestCommandsSchema:
    """COMMANDS list shape and content."""

    def test_commands_has_seven_entries(self):
        from robot_radio.testgui.commands import COMMANDS
        assert len(COMMANDS) == 7, f"Expected 7 commands, got {len(COMMANDS)}"

    def test_commands_labels(self):
        expected = ["S", "T", "D", "R", "TURN", "RT", "G"]
        assert _labels() == expected

    def test_each_spec_has_label(self):
        from robot_radio.testgui.commands import COMMANDS
        for spec in COMMANDS:
            assert "label" in spec
            assert isinstance(spec["label"], str)

    def test_each_spec_has_params(self):
        from robot_radio.testgui.commands import COMMANDS
        for spec in COMMANDS:
            assert "params" in spec
            assert isinstance(spec["params"], list)
            assert len(spec["params"]) >= 1

    def test_each_param_has_required_keys(self):
        from robot_radio.testgui.commands import COMMANDS
        required_keys = {"name", "type", "min", "max", "default"}
        for spec in COMMANDS:
            for param in spec["params"]:
                missing = required_keys - set(param.keys())
                assert not missing, (
                    f"Param {param.get('name')!r} in {spec['label']} "
                    f"is missing keys: {missing}"
                )

    def test_s_has_two_params(self):
        from robot_radio.testgui.commands import COMMANDS
        s_spec = next(s for s in COMMANDS if s["label"] == "S")
        assert len(s_spec["params"]) == 2
        names = [p["name"] for p in s_spec["params"]]
        assert names == ["left", "right"]

    def test_t_has_three_params(self):
        from robot_radio.testgui.commands import COMMANDS
        t_spec = next(s for s in COMMANDS if s["label"] == "T")
        assert len(t_spec["params"]) == 3
        names = [p["name"] for p in t_spec["params"]]
        assert names == ["left", "right", "ms"]

    def test_d_has_three_params(self):
        from robot_radio.testgui.commands import COMMANDS
        d_spec = next(s for s in COMMANDS if s["label"] == "D")
        assert len(d_spec["params"]) == 3
        names = [p["name"] for p in d_spec["params"]]
        assert names == ["left", "right", "mm"]

    def test_r_has_two_params(self):
        from robot_radio.testgui.commands import COMMANDS
        r_spec = next(s for s in COMMANDS if s["label"] == "R")
        assert len(r_spec["params"]) == 2
        names = [p["name"] for p in r_spec["params"]]
        assert names == ["speed", "radius"]

    def test_turn_has_two_params(self):
        from robot_radio.testgui.commands import COMMANDS
        turn_spec = next(s for s in COMMANDS if s["label"] == "TURN")
        assert len(turn_spec["params"]) == 2
        names = [p["name"] for p in turn_spec["params"]]
        assert names == ["heading", "eps"]

    def test_rt_has_one_param(self):
        from robot_radio.testgui.commands import COMMANDS
        rt_spec = next(s for s in COMMANDS if s["label"] == "RT")
        assert len(rt_spec["params"]) == 1
        names = [p["name"] for p in rt_spec["params"]]
        assert names == ["deg"]

    def test_g_has_three_params(self):
        from robot_radio.testgui.commands import COMMANDS
        g_spec = next(s for s in COMMANDS if s["label"] == "G")
        assert len(g_spec["params"]) == 3
        names = [p["name"] for p in g_spec["params"]]
        assert names == ["x", "y", "speed"]


# ---------------------------------------------------------------------------
# Wire-string builder — pure function tests (no Qt required)
# ---------------------------------------------------------------------------


class TestBuildWireStringS:
    """S <left> <right>"""

    def test_s_basic(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "S")
        result = build_wire_string(spec, {"left": 200, "right": 200})
        assert result == "S 200 200"

    def test_s_negative_left(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "S")
        result = build_wire_string(spec, {"left": -150, "right": 150})
        assert result == "S -150 150"

    def test_s_zero_speeds(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "S")
        result = build_wire_string(spec, {"left": 0, "right": 0})
        assert result == "S 0 0"

    def test_s_max_speeds(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "S")
        result = build_wire_string(spec, {"left": 1000, "right": -1000})
        assert result == "S 1000 -1000"


class TestBuildWireStringT:
    """T <left> <right> <ms>"""

    def test_t_basic(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "T")
        result = build_wire_string(spec, {"left": 200, "right": 200, "ms": 1000})
        assert result == "T 200 200 1000"

    def test_t_asymmetric(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "T")
        result = build_wire_string(spec, {"left": -300, "right": 300, "ms": 500})
        assert result == "T -300 300 500"

    def test_t_max_ms(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "T")
        result = build_wire_string(spec, {"left": 100, "right": 100, "ms": 30000})
        assert result == "T 100 100 30000"


class TestBuildWireStringD:
    """D <left> <right> <mm>"""

    def test_d_basic(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "D")
        result = build_wire_string(spec, {"left": 200, "right": 200, "mm": 500})
        assert result == "D 200 200 500"

    def test_d_large_distance(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "D")
        result = build_wire_string(spec, {"left": 300, "right": 300, "mm": 10000})
        assert result == "D 300 300 10000"

    def test_d_reverse(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "D")
        result = build_wire_string(spec, {"left": -200, "right": -200, "mm": 250})
        assert result == "D -200 -200 250"


class TestBuildWireStringR:
    """R <speed> <radius>"""

    def test_r_basic(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "R")
        result = build_wire_string(spec, {"speed": 200, "radius": 500})
        assert result == "R 200 500"

    def test_r_negative_radius_cw(self):
        """Negative radius → CW arc."""
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "R")
        result = build_wire_string(spec, {"speed": 200, "radius": -500})
        assert result == "R 200 -500"

    def test_r_zero_radius_straight(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "R")
        result = build_wire_string(spec, {"speed": 200, "radius": 0})
        assert result == "R 200 0"


class TestBuildWireStringTURN:
    """TURN <heading_cdeg> [eps=<eps_cdeg>]

    heading and eps are in centidegrees on the wire.
    """

    def test_turn_heading_cdeg(self):
        """Heading value in cdeg goes through as-is."""
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "TURN")
        result = build_wire_string(spec, {"heading": 9000, "eps": 0})
        assert result == "TURN 9000"

    def test_turn_no_eps_when_zero(self):
        """eps=0 → omitted from wire string."""
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "TURN")
        result = build_wire_string(spec, {"heading": 4500, "eps": 0})
        assert result == "TURN 4500"

    def test_turn_with_nonzero_eps(self):
        """Non-zero eps appears as eps=<cdeg>."""
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "TURN")
        result = build_wire_string(spec, {"heading": 9000, "eps": 300})
        assert result == "TURN 9000 eps=300"

    def test_turn_negative_heading(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "TURN")
        result = build_wire_string(spec, {"heading": -9000, "eps": 0})
        assert result == "TURN -9000"

    def test_turn_max_heading(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "TURN")
        result = build_wire_string(spec, {"heading": 18000, "eps": 0})
        assert result == "TURN 18000"

    def test_turn_eps_small(self):
        """Small eps value (10 cdeg = 0.1°) is included."""
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "TURN")
        result = build_wire_string(spec, {"heading": 0, "eps": 10})
        assert result == "TURN 0 eps=10"


class TestBuildWireStringRT:
    """RT <rel_cdeg> — relative in-place turn; deg entered, cdeg on wire."""

    def test_rt_positive_ccw(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "RT")
        result = build_wire_string(spec, {"deg": 90})
        assert result == "RT 9000"

    def test_rt_negative_cw(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "RT")
        result = build_wire_string(spec, {"deg": -45})
        assert result == "RT -4500"

    def test_rt_zero(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "RT")
        result = build_wire_string(spec, {"deg": 0})
        assert result == "RT 0"

    def test_rt_uses_default(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "RT")
        result = build_wire_string(spec, {})
        # Default deg=90 → 9000 cdeg
        assert result == "RT 9000"


class TestBuildWireStringG:
    """G <x> <y> <speed>"""

    def test_g_basic(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "G")
        result = build_wire_string(spec, {"x": 500, "y": 300, "speed": 200})
        assert result == "G 500 300 200"

    def test_g_negative_coords(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "G")
        result = build_wire_string(spec, {"x": -1000, "y": -500, "speed": 150})
        assert result == "G -1000 -500 150"

    def test_g_zero_zero(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "G")
        result = build_wire_string(spec, {"x": 0, "y": 0, "speed": 200})
        assert result == "G 0 0 200"

    def test_g_large_coords(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "G")
        result = build_wire_string(spec, {"x": 10000, "y": 10000, "speed": 1000})
        assert result == "G 10000 10000 1000"


# ---------------------------------------------------------------------------
# Defaults: build_wire_string uses param defaults when values are missing
# ---------------------------------------------------------------------------


class TestBuildWireStringDefaults:
    """build_wire_string falls back to param defaults for missing values."""

    def test_s_uses_defaults(self):
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "S")
        result = build_wire_string(spec, {})
        # Default for S is left=200, right=200
        assert result == "S 200 200"

    def test_turn_default_eps_omitted(self):
        """TURN default eps=0 → omitted from wire."""
        from robot_radio.testgui.commands import COMMANDS, build_wire_string
        spec = next(s for s in COMMANDS if s["label"] == "TURN")
        result = build_wire_string(spec, {})
        # Default heading=9000, eps=0 (omitted)
        assert "eps" not in result


# ---------------------------------------------------------------------------
# Pre-programmed tours
# ---------------------------------------------------------------------------


class TestTours:
    """TOUR_1 / TOURS content and shape (pure data, no Qt)."""

    def test_tour_1_sequence(self):
        from robot_radio.testgui.commands import TOUR_1
        assert TOUR_1 == [
            "RT 4500",
            "D 200 200 420",
            "TURN 18000",
            "D 200 200 700",
            "RT 9000",
            "D 200 200 500",
            "RT 9000",
            "D 200 200 700",
            "RT 9000",
            "D 200 200 500",
        ]

    def test_tour_1_all_steps_are_wire_strings(self):
        """Every step is a non-empty string beginning with a known verb."""
        from robot_radio.testgui.commands import TOUR_1
        verbs = {"RT", "D", "TURN"}
        for step in TOUR_1:
            assert isinstance(step, str) and step
            assert step.split()[0] in verbs

    def test_tours_registry_contains_tour_1(self):
        from robot_radio.testgui.commands import TOURS, TOUR_1
        assert TOURS["Tour 1"] is TOUR_1


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
        import sys

        # Ensure the module is not already cached under a Qt-contaminated state.
        mod = importlib.import_module("robot_radio.testgui.commands")
        assert hasattr(mod, "COMMANDS")
        assert hasattr(mod, "build_wire_string")

    def test_testgui_init_importable_without_qt(self):
        """robot_radio.testgui.__init__ must be importable without PySide6."""
        import robot_radio.testgui
        assert hasattr(robot_radio.testgui, "__version__")
