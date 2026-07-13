"""tests/unit/test_device_bus_bringup_bench.py -- DB-009.

Unit tests for the PURE (no I/O, no hardware) helper logic in
``tests/bench/device_bus_bringup.py`` -- the DeviceBus bring-up bench-gate
script (device-bus-tickets.md's DB-009). Covers: the reply-line parser
(``parse_kv``), the ring-stamp-delta/loss-rate statistics helpers, and each
gate's pass/fail threshold function.

``tests/bench/`` is explicitly "HITL CLI tools, not pytest-collected, and
have no conftest of their own" (tests/conftest.py's own header comment), so
this test loads ``device_bus_bringup.py`` directly by file path via
``importlib`` rather than a package import -- it does not turn tests/bench/
into a package or add it to sys.path for anything else. Fully offline: no
serial port, no hardware, no ``robot_radio.io.serial_conn`` import even
occurs at module scope of the loaded script beyond the top-level `from
robot_radio.io.serial_conn import SerialConnection` line, which only
requires the package to be importABLE (robot_radio is installed editable,
pyproject.toml's ``[tool.hatch.build.targets.wheel]``), never a real port to
be opened.
"""

from __future__ import annotations

import importlib.util
import pathlib

import pytest

_BENCH_SCRIPT = (
    pathlib.Path(__file__).resolve().parents[1] / "bench" / "device_bus_bringup.py"
)


def _load_bench_module():
    spec = importlib.util.spec_from_file_location("device_bus_bringup", _BENCH_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def bringup():
    return _load_bench_module()


# ---------------------------------------------------------------------------
# parse_kv
# ---------------------------------------------------------------------------

class TestParseKv:
    def test_ok_with_fields_and_corr_id(self, bringup):
        line = "OK pos=12.345 vel=-1.000 applied=0.030 t=1234567 valid=1 conn=1 wedged=0 #7"
        parsed = bringup.parse_kv(line)
        assert parsed["tag"] == "OK"
        assert parsed["corr_id"] == "7"
        assert parsed["kv"]["pos"] == "12.345"
        assert parsed["kv"]["vel"] == "-1.000"
        assert parsed["kv"]["wedged"] == "0"
        assert parsed["tokens"] == []

    def test_ok_bare_token_no_kv(self, bringup):
        parsed = bringup.parse_kv("OK pong")
        assert parsed["tag"] == "OK"
        assert parsed["tokens"] == ["pong"]
        assert parsed["kv"] == {}
        assert parsed["corr_id"] is None

    def test_ok_no_fields_at_all(self, bringup):
        parsed = bringup.parse_kv("OK")
        assert parsed["tag"] == "OK"
        assert parsed["tokens"] == []
        assert parsed["kv"] == {}

    def test_err_reply(self, bringup):
        parsed = bringup.parse_kv("ERR badport #3")
        assert parsed["tag"] == "ERR"
        assert parsed["tokens"] == ["badport"]
        assert parsed["corr_id"] == "3"

    def test_unrecognized_tag_returns_none(self, bringup):
        assert bringup.parse_kv("garbage line") is None
        assert bringup.parse_kv("# relay comment") is None

    def test_empty_and_none(self, bringup):
        assert bringup.parse_kv("") is None
        assert bringup.parse_kv(None) is None

    def test_kv_float_and_int_helpers(self, bringup):
        parsed = bringup.parse_kv("OK pos=12.5 glitch=3 valid=1")
        assert bringup.kv_float(parsed, "pos") == 12.5
        assert bringup.kv_int(parsed, "glitch") == 3
        assert bringup.kv_float(parsed, "missing") is None
        assert bringup.kv_int(None, "pos") is None

    def test_kv_helpers_tolerate_unparsable_value(self, bringup):
        parsed = bringup.parse_kv("OK pos=nope")
        assert bringup.kv_float(parsed, "pos") is None


# ---------------------------------------------------------------------------
# compute_deltas / summarize_deltas / loss_rate
# ---------------------------------------------------------------------------

class TestStats:
    def test_compute_deltas_newest_first(self, bringup):
        # Ring ages 0 (newest) .. 4 (oldest), stamps descending.
        stamps = [1_050_000, 1_034_000, 1_018_000, 1_002_000, 986_000]
        deltas = bringup.compute_deltas(stamps)
        assert deltas == [16_000, 16_000, 16_000, 16_000]

    def test_compute_deltas_short_input(self, bringup):
        assert bringup.compute_deltas([]) == []
        assert bringup.compute_deltas([100]) == []

    def test_summarize_deltas(self, bringup):
        stats = bringup.summarize_deltas([10, 20, 30])
        assert stats["n"] == 3
        assert stats["min_us"] == 10
        assert stats["max_us"] == 30
        assert stats["mean_us"] == pytest.approx(20.0)
        assert stats["stdev_us"] > 0

    def test_summarize_deltas_empty(self, bringup):
        assert bringup.summarize_deltas([]) is None

    def test_summarize_deltas_single(self, bringup):
        stats = bringup.summarize_deltas([42])
        assert stats["stdev_us"] == 0.0

    def test_loss_rate(self, bringup):
        assert bringup.loss_rate(60, 60) == pytest.approx(0.0)
        assert bringup.loss_rate(60, 57) == pytest.approx(0.05)
        assert bringup.loss_rate(60, 0) == pytest.approx(1.0)

    def test_loss_rate_zero_sent(self, bringup):
        assert bringup.loss_rate(0, 0) == 0.0


# ---------------------------------------------------------------------------
# Gate pass/fail threshold functions
# ---------------------------------------------------------------------------

class TestGateThresholds:
    def test_pipelining_gate_pass_clean_run(self, bringup):
        assert bringup.pipelining_gate_pass(
            glitch1_before=0, glitch1_after=0,
            glitch2_before=0, glitch2_after=1,
            pos_delta1=40.0, pos_delta2=38.0,
            wedge_seen=False,
        )

    def test_pipelining_gate_fails_on_wedge(self, bringup):
        assert not bringup.pipelining_gate_pass(
            glitch1_before=0, glitch1_after=0,
            glitch2_before=0, glitch2_after=0,
            pos_delta1=40.0, pos_delta2=38.0,
            wedge_seen=True,
        )

    def test_pipelining_gate_fails_on_glitch_growth(self, bringup):
        assert not bringup.pipelining_gate_pass(
            glitch1_before=0, glitch1_after=10,
            glitch2_before=0, glitch2_after=0,
            pos_delta1=40.0, pos_delta2=38.0,
            wedge_seen=False,
            max_glitch_growth=2,
        )

    def test_pipelining_gate_fails_on_no_movement(self, bringup):
        assert not bringup.pipelining_gate_pass(
            glitch1_before=0, glitch1_after=0,
            glitch2_before=0, glitch2_after=0,
            pos_delta1=0.5, pos_delta2=38.0,
            wedge_seen=False,
            min_pos_delta=5.0,
        )

    def test_pipelining_gate_fails_on_missing_data(self, bringup):
        assert not bringup.pipelining_gate_pass(
            glitch1_before=None, glitch1_after=0,
            glitch2_before=0, glitch2_after=0,
            pos_delta1=40.0, pos_delta2=38.0,
            wedge_seen=False,
        )

    def test_reversal_gate_pass_clean_run(self, bringup):
        assert bringup.reversal_gate_pass(
            wedge_seen_during=False, wedged_after=0,
            glitch_before=0, glitch_after=1,
        )

    def test_reversal_gate_tolerates_transient_wedge(self, bringup):
        # A transient latch DURING a hard reversal is expected armor
        # behavior (project's own encoder-wedge-boundary-latch finding) --
        # only a wedge that persists AFTER the sequence fails the gate.
        assert bringup.reversal_gate_pass(
            wedge_seen_during=True, wedged_after=0,
            glitch_before=0, glitch_after=1,
        )

    def test_reversal_gate_fails_on_persistent_wedge(self, bringup):
        assert not bringup.reversal_gate_pass(
            wedge_seen_during=True, wedged_after=1,
            glitch_before=0, glitch_after=1,
        )

    def test_reversal_gate_fails_on_excess_glitch_growth(self, bringup):
        assert not bringup.reversal_gate_pass(
            wedge_seen_during=False, wedged_after=0,
            glitch_before=0, glitch_after=10,
            max_glitch_growth=3,
        )

    def test_loss_gate_pass(self, bringup):
        assert bringup.loss_gate_pass(0.0)
        assert bringup.loss_gate_pass(0.05, max_loss=0.05)
        assert not bringup.loss_gate_pass(0.10, max_loss=0.05)

    def test_flash_ram_gate_pass_within_budget(self, bringup):
        assert bringup.flash_ram_gate_pass(140_000, 120_000, flash_budget=372_736)

    def test_flash_ram_gate_fails_over_flash_budget(self, bringup):
        assert not bringup.flash_ram_gate_pass(400_000, 120_000, flash_budget=372_736)

    def test_flash_ram_gate_ignores_ram(self, bringup):
        # RAM sits ~98% full by design on this target -- never gates.
        assert bringup.flash_ram_gate_pass(140_000, 122_800, flash_budget=372_736,
                                             ram_budget=122_816)
