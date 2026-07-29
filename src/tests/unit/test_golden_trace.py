"""Tests for src/tests/tools/golden_trace.py.

The point of this suite is that the tripwire must be PROVEN to trip. Sprint
123 shipped a wire bug precisely because a check that could not fail was
mistaken for a check that passed, and sprint 124's bench gate reported PASS
for a session that observed zero telemetry frames. So several tests below
deliberately perturb a trace and assert the comparison NOTICES.
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from golden_trace import (  # noqa: E402
    Comparison, PlotSpec, band_mask, compare, distance_to_polyline,
    load_golden, render_png, save_golden, suggest_tolerance,
)


def _square(side: float = 1000.0, per_leg: int = 50):
    """A closed square path, the shape the system test actually drives."""
    legs = [
        (np.linspace(0, side, per_leg), np.zeros(per_leg)),
        (np.full(per_leg, side), np.linspace(0, side, per_leg)),
        (np.linspace(side, 0, per_leg), np.full(per_leg, side)),
        (np.zeros(per_leg), np.linspace(side, 0, per_leg)),
    ]
    return np.concatenate([l[0] for l in legs]), np.concatenate([l[1] for l in legs])


def _spec(side: float = 1000.0) -> PlotSpec:
    pad = side * 0.2
    return PlotSpec(-pad, side + pad, -pad, side + pad, width=200, height=200)


# --- geometry ------------------------------------------------------------

def test_distance_to_polyline_is_zero_on_the_line():
    gx, gy = np.array([0.0, 10.0]), np.array([0.0, 0.0])
    d = distance_to_polyline(np.array([0.0, 5.0, 10.0]), np.zeros(3), gx, gy)
    assert np.allclose(d, 0.0)


def test_distance_to_polyline_measures_perpendicular_offset():
    gx, gy = np.array([0.0, 10.0]), np.array([0.0, 0.0])
    d = distance_to_polyline(np.array([5.0]), np.array([3.0]), gx, gy)
    assert d[0] == pytest.approx(3.0)


def test_distance_clamps_past_the_segment_ends():
    """Past an endpoint the distance is to the endpoint, not to the infinite
    line -- otherwise a run that stops short would score as still on track."""
    gx, gy = np.array([0.0, 10.0]), np.array([0.0, 0.0])
    d = distance_to_polyline(np.array([13.0]), np.array([4.0]), gx, gy)
    assert d[0] == pytest.approx(5.0)  # 3-4-5 from the (10, 0) endpoint


def test_degenerate_zero_length_segment_is_point_distance():
    """A stopped robot emits repeated identical samples, and this test has
    planned stops -- so zero-length segments are normal, not exceptional."""
    gx, gy = np.array([2.0, 2.0, 2.0]), np.array([2.0, 2.0, 2.0])
    d = distance_to_polyline(np.array([2.0, 5.0]), np.array([2.0, 6.0]), gx, gy)
    assert d[0] == pytest.approx(0.0)
    assert d[1] == pytest.approx(5.0)


# --- the gate ------------------------------------------------------------

def test_identical_run_passes_with_zero_deviation():
    gx, gy = _square()
    result = compare(gx, gy, gx, gy, _spec(), tolerance=5.0)
    assert result.passed()
    assert result.outside_count == 0
    assert result.max_deviation == pytest.approx(0.0)
    assert result.outside_pixels == 0


def test_small_jitter_inside_tolerance_passes():
    gx, gy = _square()
    rng = np.random.default_rng(1234)
    nx = gx + rng.uniform(-1.0, 1.0, gx.size)
    ny = gy + rng.uniform(-1.0, 1.0, gy.size)
    result = compare(nx, ny, gx, gy, _spec(), tolerance=5.0)
    assert result.passed()
    assert result.max_deviation < 5.0


def test_a_real_excursion_is_caught():
    """THE tripwire: bulge one leg well outside the band and require failure.

    The offset must be PERPENDICULAR to the leg being perturbed. Samples
    10:40 sit on the first (horizontal) leg, so displacing y moves them off
    the path; displacing y on a vertical leg would slide them ALONG it and
    correctly measure zero deviation.
    """
    gx, gy = _square()
    nx, ny = gx.copy(), gy.copy()
    ny[10:40] += 40.0
    result = compare(nx, ny, gx, gy, _spec(), tolerance=5.0)
    assert not result.passed()
    assert result.outside_count > 0
    assert result.max_deviation == pytest.approx(40.0)
    assert result.outside_pixels > 0


def test_displacement_along_a_leg_is_not_an_excursion():
    """Sliding samples along the path is not going off course -- it is a
    timing/pacing difference. The geometric gate must not confuse the two."""
    gx, gy = _square()
    nx, ny = gx.copy(), gy.copy()
    ny[60:90] += 40.0          # leg 2 is vertical, so this slides along it
    result = compare(nx, ny, gx, gy, _spec(), tolerance=5.0)
    assert result.passed()
    assert result.max_deviation == pytest.approx(0.0)


def test_tolerance_governs_the_verdict():
    """The same run passes or fails purely on the band width -- which is what
    makes silently widening a tolerance dangerous."""
    gx, gy = _square()
    nx, ny = gx.copy(), gy + 12.0
    spec = _spec()
    assert not compare(nx, ny, gx, gy, spec, tolerance=5.0).passed()
    assert compare(nx, ny, gx, gy, spec, tolerance=20.0).passed()


def test_empty_run_raises_rather_than_passing_vacuously():
    """A run that produced no samples must never score as clean -- this is the
    exact shape of sprint 124's vacuous-PASS defect."""
    gx, gy = _square()
    with pytest.raises(ValueError, match="empty"):
        compare(np.array([]), np.array([]), gx, gy, _spec(), tolerance=5.0)


def test_truncated_run_is_reported_as_uncovered_not_as_failure():
    """Half a square is not 'off course' -- it is short. The failure metric
    stays clean while the coverage metric flags it, which is exactly why a
    plain XOR (which sums the two) is the wrong operator."""
    gx, gy = _square()
    half = gx.size // 2
    result = compare(gx[:half], gy[:half], gx, gy, _spec(), tolerance=5.0)
    assert result.passed()
    assert result.outside_count == 0
    assert result.uncovered_pixels > 0


# --- raster --------------------------------------------------------------

def test_band_mask_widens_with_tolerance():
    gx, gy = _square()
    spec = _spec()
    narrow = np.count_nonzero(band_mask(gx, gy, spec, 2.0))
    wide = np.count_nonzero(band_mask(gx, gy, spec, 20.0))
    assert 0 < narrow < wide


def test_band_mask_is_resolution_independent_in_data_units():
    """Doubling the raster must not change what counts as in-band -- the whole
    reason the gate is analytic rather than a pixel op."""
    gx, gy = _square()
    coarse = PlotSpec(-200, 1200, -200, 1200, width=100, height=100)
    fine = PlotSpec(-200, 1200, -200, 1200, width=400, height=400)
    nx, ny = gx.copy(), gy + 12.0
    a = compare(nx, ny, gx, gy, coarse, tolerance=5.0)
    b = compare(nx, ny, gx, gy, fine, tolerance=5.0)
    assert a.outside_count == b.outside_count
    assert a.max_deviation == pytest.approx(b.max_deviation)


def test_plot_spec_rejects_degenerate_limits():
    with pytest.raises(ValueError):
        PlotSpec(0.0, 0.0, 0.0, 1.0)
    with pytest.raises(ValueError):
        PlotSpec(0.0, 1.0, 0.0, 1.0, width=1)


# --- tolerance derivation ------------------------------------------------

def test_suggest_tolerance_reflects_observed_spread():
    """Derive the band from measured run-to-run variation rather than from a
    number chosen to make the first run pass."""
    gx, gy = _square()
    rng = np.random.default_rng(7)
    runs = [(gx + rng.uniform(-2, 2, gx.size), gy + rng.uniform(-2, 2, gy.size))
            for _ in range(5)]
    tolerance = suggest_tolerance(runs, gx, gy, margin=1.5)
    assert tolerance > 0.0
    for rx, ry in runs:
        assert compare(rx, ry, gx, gy, _spec(), tolerance).passed()


# --- persistence ---------------------------------------------------------

def test_golden_round_trips_through_disk(tmp_path):
    gx, gy = _square()
    spec = _spec()
    save_golden(tmp_path, "xy_path", gx, gy, spec, tolerance=5.0)
    lx, ly, lspec, ltol = load_golden(tmp_path, "xy_path")
    assert np.allclose(lx, gx)
    assert np.allclose(ly, gy)
    assert lspec == spec           # the pinned geometry survives, unmodified
    assert ltol == pytest.approx(5.0)


def test_render_png_writes_an_image(tmp_path):
    gx, gy = _square()
    out = tmp_path / "sub" / "xy_path.png"
    render_png(out, gx, gy, _spec(), tolerance=5.0, label="xy path")
    assert out.exists() and out.stat().st_size > 0


def test_summary_states_the_verdict():
    gx, gy = _square()
    good = compare(gx, gy, gx, gy, _spec(), tolerance=5.0).summary()
    assert good.startswith("[PASS]")
    nx, ny = gx.copy(), gy + 50.0
    assert compare(nx, ny, gx, gy, _spec(), tolerance=5.0).summary().startswith("[FAIL]")
