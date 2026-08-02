"""src/tests/unit/test_duty_sweep_population.py -- sprint 130 ticket 001's own
testing plan: synthetic-data unit tests for `src/tests/bench/duty_sweep.py`'s
fitting and population-analysis math, no hardware required.

`src/tests/bench/` is "HITL CLI tools, not pytest-collected" (`tests/
CLAUDE.md`), so this test loads `duty_sweep.py` directly by file path via
`importlib`, mirroring `test_tlm_log.py`'s established precedent for testing a
bench script's pure logic in isolation.
"""
from __future__ import annotations

import importlib.util
import pathlib

import pytest

_BENCH_SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "bench" / "duty_sweep.py"


def _load_bench_module():
    spec = importlib.util.spec_from_file_location("duty_sweep", _BENCH_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def duty_sweep():
    return _load_bench_module()


# ---------------------------------------------------------------------------
# fit_line
# ---------------------------------------------------------------------------

def test_fit_line_recovers_exact_affine_line(duty_sweep):
    gain, offset = 850.0, -12.0
    pts = [(x, gain * x + offset) for x in (0.10, 0.15, 0.20, 0.25, 0.30)]
    m, b = duty_sweep.fit_line(pts)
    assert m == pytest.approx(gain, abs=1e-6)
    assert b == pytest.approx(offset, abs=1e-6)


def test_fit_line_least_squares_over_noisy_points(duty_sweep):
    # Symmetric perturbation around an exact line -- the least-squares fit
    # should still recover the underlying gain/offset closely.
    gain, offset = 800.0, 5.0
    xs = [0.10, 0.15, 0.20, 0.25, 0.30]
    noise = [3.0, -3.0, 2.0, -2.0, 0.0]
    pts = [(x, gain * x + offset + n) for x, n in zip(xs, noise)]
    m, b = duty_sweep.fit_line(pts)
    assert m == pytest.approx(gain, abs=15.0)
    assert b == pytest.approx(offset, abs=3.0)


# ---------------------------------------------------------------------------
# population_spread
# ---------------------------------------------------------------------------

def test_population_spread_uniform_values(duty_sweep):
    mean, envelope, stdev = duty_sweep.population_spread([10.0, 10.0, 10.0])
    assert mean == pytest.approx(10.0)
    assert envelope == pytest.approx(0.0)
    assert stdev == pytest.approx(0.0)


def test_population_spread_reports_max_deviation_envelope(duty_sweep):
    # mean = 10, deviations = [-8, -2, +10] -> envelope = 10
    mean, envelope, stdev = duty_sweep.population_spread([2.0, 8.0, 20.0])
    assert mean == pytest.approx(10.0)
    assert envelope == pytest.approx(10.0)
    assert stdev > 0.0


# ---------------------------------------------------------------------------
# parallel_lines_verdict
# ---------------------------------------------------------------------------

def test_parallel_lines_verdict_confirms_intercept_dominated(duty_sweep):
    # Same slope, different intercepts -- textbook "parallel lines": spread
    # should NOT grow across the fit window.
    fits = [(850.0, 0.0), (850.0, -20.0), (850.0, 15.0), (850.0, 5.0)]
    verdict, lo_spread, hi_spread, growth = duty_sweep.parallel_lines_verdict(
        fits, duty_lo=0.10, duty_hi=0.30)
    assert verdict == "intercept-dominated (parallel)"
    assert growth == pytest.approx(0.0, abs=1e-9)
    assert lo_spread == pytest.approx(hi_spread, abs=1e-6)


def test_parallel_lines_verdict_flags_fanned_slopes(duty_sweep):
    # Very different slopes, same intercept -- lines fan out with duty.
    fits = [(600.0, 0.0), (850.0, 0.0), (1100.0, 0.0), (1350.0, 0.0)]
    verdict, lo_spread, hi_spread, growth = duty_sweep.parallel_lines_verdict(
        fits, duty_lo=0.10, duty_hi=0.30)
    assert verdict == "slope-dominated (fanned)"
    assert hi_spread > lo_spread
    assert growth > 0.35


# ---------------------------------------------------------------------------
# bias_max_from_offsets / v_min_from_breakaway / map_gain_intercept
# ---------------------------------------------------------------------------

def test_bias_max_from_offsets_is_the_offset_envelope(duty_sweep):
    offsets = [0.0, -20.0, 15.0, 5.0]  # mean = 0.0, envelope = 20.0
    assert duty_sweep.bias_max_from_offsets(offsets) == pytest.approx(20.0)


def test_v_min_from_breakaway_uses_worst_case_duty(duty_sweep):
    breakaways = {"a": 0.10, "b": 0.164, "c": 0.102}
    mean_gain = 845.7
    v_min = duty_sweep.v_min_from_breakaway(breakaways.values(), mean_gain)
    assert v_min == pytest.approx(0.164 * 845.7)


def test_map_gain_intercept_identity_when_matching_population_mean(duty_sweep):
    # A motor whose own gain equals the population mean and whose offset is
    # zero needs no correction: map gain 1, map intercept 0.
    map_gain, map_intercept = duty_sweep.map_gain_intercept(
        gain=845.7, offset=0.0, mean_gain=845.7)
    assert map_gain == pytest.approx(1.0)
    assert map_intercept == pytest.approx(0.0)


def test_map_gain_intercept_scales_for_an_above_average_motor(duty_sweep):
    # A motor that delivers MORE speed per duty than the population mean
    # needs map_gain < 1 to bring it back toward the shared dutyPerSpeed.
    map_gain, map_intercept = duty_sweep.map_gain_intercept(
        gain=900.0, offset=10.0, mean_gain=850.0)
    assert map_gain == pytest.approx(850.0 / 900.0)
    assert map_intercept == pytest.approx(-(850.0 / 900.0) * 10.0)
