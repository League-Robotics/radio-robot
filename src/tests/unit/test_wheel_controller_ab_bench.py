"""src/tests/unit/test_wheel_controller_ab_bench.py -- sprint 130 ticket 006's
own testing plan: synthetic-data unit tests for
`src/tests/bench/wheel_controller_ab_bench.py`'s pure +500-spec scoring
logic, no hardware required.

`src/tests/bench/` is "HITL CLI tools, not pytest-collected" (`tests/
CLAUDE.md`), so this test loads the bench script directly by file path via
`importlib`, mirroring `test_duty_sweep_population.py`'s established
precedent for testing a bench script's pure logic in isolation.
"""
from __future__ import annotations

import importlib.util
import pathlib
import sys

import pytest

_BENCH_SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "bench" / "wheel_controller_ab_bench.py"


def _load_bench_module():
    spec = importlib.util.spec_from_file_location("wheel_controller_ab_bench", _BENCH_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # Register in sys.modules BEFORE exec: the bench script's @dataclass
    # Sample uses `from __future__ import annotations` (string annotations),
    # and dataclasses' own ClassVar detection resolves those strings via
    # sys.modules[cls.__module__] at class-definition time -- an unregistered
    # module name makes that lookup return None and crash. Same fix
    # importlib's own recipes document for this exact case.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def bench():
    return _load_bench_module()


def _synthetic_plus500(bench, *, target=150.0, rise_s=0.25, cruise_ripple=0.0,
                       lr_split=0.0, dt=0.02, ease=60.0, floor=90.0):
    """Build a clean, idealized +500 trace: linear ramp to `target` over
    `rise_s`, flat plateau (with optional alternating +/-`cruise_ripple`
    oscillation and a constant `lr_split` offset) until the last `ease` mm,
    then a linear taper to `floor`, ending at exactly 500mm. Distance
    integrated from velocity via simple Euler steps at `dt`.

    `cruise_ripple` alternates sign every sample (a frame-to-frame
    oscillation, matching what `ripple_mm_s()`'s own peak-to-peak
    definition actually measures) -- a CONSTANT offset would shift the
    whole trace without ever showing up as ripple, which is not what this
    knob is for."""
    samples = []
    t = 0.0
    dist = 0.0
    i = 0
    while dist < 500.0 and t < 30.0:
        remaining = 500.0 - dist
        if t < rise_s:
            base = target * (t / rise_s)
        elif remaining > ease:
            base = target
        else:
            base = max(floor, target * remaining / ease)
        sign = 1.0 if i % 2 == 0 else -1.0
        vl = base + sign * cruise_ripple
        vr = base - lr_split
        samples.append(bench.Sample(t=t, vl=vl, vr=vr, remaining=remaining))
        dist += ((vl + vr) / 2.0) * dt
        t += dt
        i += 1
    return samples


# ---------------------------------------------------------------------------
# rise_time_s
# ---------------------------------------------------------------------------

def test_rise_time_matches_the_ramp_duration(bench):
    samples = _synthetic_plus500(bench, rise_s=0.25)
    rise = bench.rise_time_s(samples, target=150.0)
    # 0.9*target crossing happens at 90% of the way up a linear ramp.
    assert rise == pytest.approx(0.25 * 0.9, abs=0.03)


def test_rise_time_none_when_never_reached(bench):
    samples = [bench.Sample(t=0.0, vl=10.0, vr=10.0, remaining=490.0)]
    assert bench.rise_time_s(samples, target=150.0) is None


def test_rise_time_needs_both_wheels_over_threshold(bench):
    samples = [
        bench.Sample(t=0.0, vl=0.0, vr=0.0, remaining=500.0),
        bench.Sample(t=0.1, vl=200.0, vr=10.0, remaining=490.0),   # only left crossed
        bench.Sample(t=0.2, vl=200.0, vr=140.0, remaining=470.0),  # both now crossed
    ]
    rise = bench.rise_time_s(samples, target=150.0)
    assert rise == pytest.approx(0.2, abs=1e-9)


# ---------------------------------------------------------------------------
# cruise_samples / taper_samples
# ---------------------------------------------------------------------------

def test_cruise_and_taper_partition_on_the_ease_threshold(bench):
    samples = _synthetic_plus500(bench)
    cruise = bench.cruise_samples(samples, ease=60.0)
    taper = bench.taper_samples(samples, ease=60.0)
    assert all(s.remaining > 60.0 for s in cruise)
    assert all(0.0 < s.remaining <= 60.0 for s in taper)
    assert len(cruise) + len(taper) <= len(samples)  # the final (remaining<=0) sample belongs to neither


# ---------------------------------------------------------------------------
# ripple_mm_s / max_lr_split
# ---------------------------------------------------------------------------

def test_ripple_is_small_on_a_perfectly_flat_plateau(bench):
    # Not exactly 0.0: the plateau window starts at the exact rise-time
    # SAMPLE (90%-of-target crossing, matching velocity_step_response.py's
    # own frac=0.9 rise-time convention), whose own value is ~90% of
    # target by construction -- one sample's worth of ramp increment
    # (target*dt/rise_s here) is expected, real ripple, not injected noise.
    # The bound that matters is the spec's own <=10mm/s, which this clears
    # comfortably.
    samples = _synthetic_plus500(bench, cruise_ripple=0.0)
    score = bench.score_plus500(samples)
    assert score["ripple_l"] < 10.0
    assert score["ripple_r"] < 10.0
    assert score["ripple_ok"] is True


def test_ripple_reports_the_injected_peak_to_peak_spread(bench):
    vals = [150.0, 155.0, 148.0, 152.0]
    assert bench.ripple_mm_s(vals) == pytest.approx(7.0)


def test_max_lr_split_reports_the_injected_offset(bench):
    samples = [
        bench.Sample(t=0.0, vl=150.0, vr=150.0, remaining=400.0),
        bench.Sample(t=0.1, vl=150.0, vr=138.0, remaining=390.0),
        bench.Sample(t=0.2, vl=150.0, vr=145.0, remaining=380.0),
    ]
    assert bench.max_lr_split(samples) == pytest.approx(12.0, abs=1e-9)


# ---------------------------------------------------------------------------
# taper_neither_hits_zero
# ---------------------------------------------------------------------------

def test_taper_ok_on_a_clean_symmetric_taper(bench):
    samples = _synthetic_plus500(bench)
    taper = bench.taper_samples(samples)
    assert bench.taper_neither_hits_zero(taper) is True


def test_taper_flags_one_wheel_stopped_while_the_other_drives(bench):
    taper = [
        bench.Sample(t=0.0, vl=90.0, vr=90.0, remaining=50.0),
        bench.Sample(t=0.1, vl=0.0, vr=85.0, remaining=30.0),   # left stalled, right still driving
        bench.Sample(t=0.2, vl=0.0, vr=0.0, remaining=0.0),
    ]
    assert bench.taper_neither_hits_zero(taper) is False


def test_taper_ok_when_both_wheels_land_at_rest_together(bench):
    taper = [
        bench.Sample(t=0.0, vl=90.0, vr=90.0, remaining=50.0),
        bench.Sample(t=0.1, vl=3.0, vr=2.0, remaining=1.0),
        bench.Sample(t=0.2, vl=0.0, vr=0.0, remaining=0.0),
    ]
    assert bench.taper_neither_hits_zero(taper) is True


# ---------------------------------------------------------------------------
# score_plus500 -- integration of the above into the ticket's own criteria
# ---------------------------------------------------------------------------

def test_score_plus500_passes_every_criterion_on_an_idealized_trace(bench):
    samples = _synthetic_plus500(bench, rise_s=0.25, cruise_ripple=0.0, lr_split=0.0)
    score = bench.score_plus500(samples)
    assert score["rise_ok"] is True
    assert score["ripple_ok"] is True
    assert score["split_ok"] is True
    assert score["taper_ok"] is True
    assert score["n_plateau"] > 0
    assert score["n_taper"] > 0


def test_score_plus500_fails_ripple_when_injected_ripple_exceeds_the_bound(bench):
    samples = _synthetic_plus500(bench, cruise_ripple=15.0)
    score = bench.score_plus500(samples)
    assert score["ripple_ok"] is False
    # Alternating +/-15 -> peak-to-peak 30.
    assert score["ripple_l"] == pytest.approx(30.0, abs=0.5)


def test_score_plus500_fails_split_when_injected_split_exceeds_the_bound(bench):
    # A split large enough to fail the >10mm/s bound but small enough that
    # BOTH wheels still cross the 90%-of-target rise threshold (target=150,
    # so vr=base-lr_split must still reach >=135 -- lr_split=12 clears
    # that; lr_split=25 would never let vr cross 135 at all, which is a
    # real but DIFFERENT failure mode -- see the rise-time test above).
    samples = _synthetic_plus500(bench, lr_split=12.0)
    score = bench.score_plus500(samples)
    assert score["split_ok"] is False
    assert score["max_split"] == pytest.approx(12.0, abs=0.5)


def test_score_plus500_fails_rise_when_the_ramp_is_slow(bench):
    samples = _synthetic_plus500(bench, rise_s=1.5)
    score = bench.score_plus500(samples)
    assert score["rise_ok"] is False
    assert score["rise_time_s"] > 0.3
