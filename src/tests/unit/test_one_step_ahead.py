"""src/tests/unit/test_one_step_ahead.py -- sprint 117 ticket 006.

Unit tests for ``src/tests/tools/one_step_ahead.py`` -- the pure-Python,
independently-testable reference implementation of ``App::StateEstimator``'s
zero-order-hold (ZOH) one-step-ahead prediction math (ticket 002,
``src/firm/app/state_estimator.{h,cpp}``). Fully offline -- no serial port,
no hardware, no sim lib.

Test plan (mirrors this ticket's own Acceptance Criteria):
  1. ZOH prediction math matches ticket 002's C++ formula on hand-computed
     fixtures (including the exact numbers ``app_state_estimator_harness.
     cpp``'s own scenarios 1/3 use, so the two independent implementations
     are checked against the SAME hand-derived arithmetic).
  2. Leave-one-out walk bookkeeping: an N-sample stream produces exactly
     N-1 residuals; each residual's own prediction uses ONLY the
     immediately preceding sample's basis, never the sample being
     predicted (proven by a case where using the predicted sample's own
     velocity would give a different answer).
  3. Edge cases: empty stream, single-sample stream (empty walk, not an
     error), mismatched-length inputs (``ValueError``), non-monotonic
     timestamps (``ValueError``), and a zero-age (repeated-timestamp) step.
  4. ``rms()``: empty sequence -> 0.0; a known fixture.
  5. ``group_rms_by_phase()``: correct bucketing (inclusive boundaries),
     empty-bucket phases omitted from the result.
  6. ``wheel_stream_from_rows()``/``heading_stream_from_rows()``: correct
     extraction from CSV-row-shaped dicts (``tlm_log.CSV_FIELDNAMES``
     column names), including the ``pose_theta``[cdeg]/``twist_omega``
     [mrad/s] -> [rad]/[rad/s] unit conversion ``heading_stream_from_rows()``
     performs, and blank-cell skipping.

Collected under ``src/tests/unit/`` (a pure-Python/tooling check, not
sim/bench/playfield-scoped -- see ``tests/CLAUDE.md``); ``pyproject.toml``'s
``testpaths`` includes ``tests/unit`` so ``uv run python -m pytest`` collects
it. Imports ``one_step_ahead`` via a local ``sys.path`` shim onto
``src/tests/tools/`` -- the SAME flat-import-shim pattern
``test_gen_boot_config_otos.py``/``test_pose_fix_convergence_pure.py`` already
use for a cross-directory `src/tests/*` import.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

# src/tests/unit/test_one_step_ahead.py -> unit -> tests -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[3]
_TOOLS_DIR = _REPO_ROOT / "src" / "tests" / "tools"

if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

import one_step_ahead as osa  # noqa: E402  (path must be set up before this import)


# ---------------------------------------------------------------------------
# 1. ZOH prediction math -- hand-computed fixtures matching ticket 002's own
#    C++ scenarios (app_state_estimator_harness.cpp).
# ---------------------------------------------------------------------------

class TestZohFormula:
    def test_wheel_distance_matches_hand_computed_fixture(self):
        """Mirrors app_state_estimator_harness.cpp's own scenarioWheelZoh
        Extrapolation: basis distance=100mm at t=1000ms, velocity=50mm/s;
        the ACTUAL reading 2s later is 205mm -- predicted = 100 + 50*2.0 =
        200mm, residual = 205 - 200 = 5mm."""
        walk = osa.one_step_ahead_walk(times=[1000.0, 3000.0], positions=[100.0, 205.0],
                                       velocities=[50.0, 60.0])
        assert len(walk) == 1
        r = walk[0]
        assert r.time == 3000.0
        assert r.predicted == pytest.approx(200.0, abs=1e-6)
        assert r.actual == 205.0
        assert r.residual == pytest.approx(5.0, abs=1e-6)

    def test_negative_velocity_matches_hand_computed_fixture(self):
        """Mirrors app_state_estimator_harness.cpp's rightAt1500 fixture:
        basis distance=-40mm at t=1000ms, velocity=-20mm/s; predicted at
        t=1500ms (age 0.5s) = -40 + (-20*0.5) = -50mm."""
        walk = osa.one_step_ahead_walk(times=[1000.0, 1500.0], positions=[-40.0, -51.0],
                                       velocities=[-20.0, -15.0])
        assert len(walk) == 1
        r = walk[0]
        assert r.predicted == pytest.approx(-50.0, abs=1e-6)
        assert r.actual == -51.0
        assert r.residual == pytest.approx(-1.0, abs=1e-6)

    def test_heading_formula_matches_hand_computed_fixture(self):
        """Same generic formula, applied to a heading/omega stream --
        mirrors app_state_estimator_harness.cpp's scenarioBodyZohRotating
        shape: basis heading=pi/2 at t=0, omega=1.0rad/s; predicted at
        t=1000ms (age 1.0s) = pi/2 + 1.0*1.0."""
        kPi = math.pi
        walk = osa.one_step_ahead_walk(times=[0.0, 1000.0], positions=[kPi / 2.0, kPi / 2.0 + 1.05],
                                       velocities=[1.0, 1.0])
        assert len(walk) == 1
        r = walk[0]
        assert r.predicted == pytest.approx(kPi / 2.0 + 1.0, abs=1e-6)
        assert r.residual == pytest.approx(0.05, abs=1e-6)

    def test_zero_age_step_predicts_exactly_the_basis_position(self):
        """A repeated timestamp (age == 0) is allowed (mirrors
        state_estimator.h's own non-strict ">= basisTime" precondition) --
        predicted == the basis's own position exactly, velocity irrelevant."""
        walk = osa.one_step_ahead_walk(times=[500.0, 500.0], positions=[10.0, 12.0],
                                       velocities=[999.0, 0.0])
        assert len(walk) == 1
        assert walk[0].predicted == pytest.approx(10.0, abs=1e-9)
        assert walk[0].residual == pytest.approx(2.0, abs=1e-9)


# ---------------------------------------------------------------------------
# 2. Leave-one-out walk bookkeeping.
# ---------------------------------------------------------------------------

class TestLeaveOneOutBookkeeping:
    def test_n_samples_produce_exactly_n_minus_1_residuals(self):
        times = [0.0, 1000.0, 2000.0, 3000.0, 4000.0]
        positions = [0.0, 10.0, 25.0, 30.0, 50.0]
        velocities = [10.0, 15.0, 5.0, 20.0, 0.0]
        walk = osa.one_step_ahead_walk(times, positions, velocities)
        assert len(walk) == len(times) - 1
        assert [r.time for r in walk] == times[1:]

    def test_prediction_uses_only_the_preceding_sample_never_its_own(self):
        """A deliberately adversarial fixture: if the walk's own prediction
        for sample k ever used sample k's OWN velocity (leaking the sample
        being predicted into its own basis), the predicted value would come
        out very different from what this test asserts -- k-1's velocity
        (10) predicts 0 + 10*1.0 = 10 (the actual value planted at index 1);
        k's own velocity (9999) would predict something wildly different if
        it were (wrongly) used instead."""
        walk = osa.one_step_ahead_walk(times=[0.0, 1000.0], positions=[0.0, 10.0],
                                       velocities=[10.0, 9999.0])
        assert walk[0].predicted == pytest.approx(10.0, abs=1e-6)
        assert walk[0].residual == pytest.approx(0.0, abs=1e-6)

    def test_walks_the_whole_stream_not_just_the_first_step(self):
        """Every step's own basis is THAT step's immediately preceding
        sample, not always sample 0 -- proven by a stream whose velocity
        changes every step (a bug that always re-used sample 0's basis
        would fail this)."""
        times = [0.0, 1000.0, 2000.0, 3000.0]
        positions = [0.0, 100.0, 100.0, 300.0]
        velocities = [100.0, 0.0, 200.0, 0.0]
        walk = osa.one_step_ahead_walk(times, positions, velocities)
        assert len(walk) == 3
        # step 1: basis (t=0, pos=0, vel=100) -> predicted 0+100*1=100
        assert walk[0].predicted == pytest.approx(100.0, abs=1e-6)
        # step 2: basis (t=1000, pos=100, vel=0) -> predicted 100+0*1=100
        assert walk[1].predicted == pytest.approx(100.0, abs=1e-6)
        # step 3: basis (t=2000, pos=100, vel=200) -> predicted 100+200*1=300
        assert walk[2].predicted == pytest.approx(300.0, abs=1e-6)


# ---------------------------------------------------------------------------
# 3. Edge cases.
# ---------------------------------------------------------------------------

class TestEdgeCases:
    def test_empty_stream_produces_empty_walk(self):
        assert osa.one_step_ahead_walk([], [], []) == []

    def test_single_sample_stream_produces_empty_walk(self):
        """Documented, not an error -- there is nothing to leave out of a
        single-sample stream."""
        assert osa.one_step_ahead_walk([1000.0], [5.0], [0.0]) == []

    def test_mismatched_lengths_raise_value_error(self):
        with pytest.raises(ValueError):
            osa.one_step_ahead_walk(times=[0.0, 1.0], positions=[0.0], velocities=[0.0, 0.0])

    def test_non_monotonic_timestamps_raise_value_error(self):
        with pytest.raises(ValueError, match="non-monotonic"):
            osa.one_step_ahead_walk(times=[1000.0, 500.0, 2000.0], positions=[0.0, 0.0, 0.0],
                                    velocities=[0.0, 0.0, 0.0])

    def test_non_monotonic_error_names_the_offending_index(self):
        with pytest.raises(ValueError, match=r"index 2"):
            osa.one_step_ahead_walk(times=[0.0, 1000.0, 500.0], positions=[0.0, 0.0, 0.0],
                                    velocities=[0.0, 0.0, 0.0])


# ---------------------------------------------------------------------------
# 4. rms().
# ---------------------------------------------------------------------------

class TestRms:
    def test_empty_sequence_is_zero(self):
        assert osa.rms([]) == 0.0

    def test_known_fixture(self):
        # rms([3, 4]) == sqrt((9+16)/2) == sqrt(12.5)
        assert osa.rms([3.0, 4.0]) == pytest.approx(math.sqrt(12.5), abs=1e-9)

    def test_all_zero_is_zero(self):
        assert osa.rms([0.0, 0.0, 0.0]) == 0.0


# ---------------------------------------------------------------------------
# 5. group_rms_by_phase().
# ---------------------------------------------------------------------------

class TestGroupRmsByPhase:
    def _walk(self) -> "list[osa.Residual]":
        # Three residuals with known, hand-picked errors at times 100/200/500.
        return [
            osa.Residual(time=100.0, predicted=0.0, actual=3.0, residual=3.0),
            osa.Residual(time=200.0, predicted=0.0, actual=4.0, residual=4.0),
            osa.Residual(time=500.0, predicted=0.0, actual=10.0, residual=10.0),
        ]

    def test_buckets_by_time_window(self):
        phases = [osa.Phase("early", 0.0, 250.0), osa.Phase("late", 250.0, 1000.0)]
        grouped = osa.group_rms_by_phase(self._walk(), phases)
        assert grouped["early"] == pytest.approx(math.sqrt((9.0 + 16.0) / 2.0), abs=1e-9)
        assert grouped["late"] == pytest.approx(10.0, abs=1e-9)

    def test_boundary_is_inclusive_both_ends(self):
        # A phase window whose end EXACTLY equals a residual's own time
        # includes that residual.
        phases = [osa.Phase("p", 0.0, 200.0)]
        grouped = osa.group_rms_by_phase(self._walk(), phases)
        assert grouped["p"] == pytest.approx(math.sqrt((9.0 + 16.0) / 2.0), abs=1e-9)

    def test_empty_bucket_phase_is_omitted_not_zero(self):
        phases = [osa.Phase("nothing_here", 900.0, 1000.0)]
        grouped = osa.group_rms_by_phase(self._walk(), phases)
        assert "nothing_here" not in grouped

    def test_overlapping_phases_each_count_the_shared_residual(self):
        phases = [osa.Phase("a", 0.0, 300.0), osa.Phase("b", 150.0, 600.0)]
        grouped = osa.group_rms_by_phase(self._walk(), phases)
        # residual at t=200 falls in BOTH windows.
        assert grouped["a"] == pytest.approx(math.sqrt((9.0 + 16.0) / 2.0), abs=1e-9)
        assert grouped["b"] == pytest.approx(math.sqrt((16.0 + 100.0) / 2.0), abs=1e-9)


# ---------------------------------------------------------------------------
# 6. CSV-row extraction helpers.
# ---------------------------------------------------------------------------

class TestWheelStreamFromRows:
    def test_extracts_left_side(self):
        rows = [
            {"enc_left_position": "10.0", "enc_left_velocity": "1.0", "enc_left_time": "100"},
            {"enc_left_position": "20.0", "enc_left_velocity": "2.0", "enc_left_time": "200"},
        ]
        times, positions, velocities = osa.wheel_stream_from_rows(rows, side="left")
        assert times == [100.0, 200.0]
        assert positions == [10.0, 20.0]
        assert velocities == [1.0, 2.0]

    def test_extracts_right_side(self):
        rows = [{"enc_right_position": "5.0", "enc_right_velocity": "0.5", "enc_right_time": "50"}]
        times, positions, velocities = osa.wheel_stream_from_rows(rows, side="right")
        assert times == [50.0]
        assert positions == [5.0]
        assert velocities == [0.5]

    def test_invalid_side_raises_value_error(self):
        with pytest.raises(ValueError):
            osa.wheel_stream_from_rows([], side="front")

    def test_skips_rows_with_blank_cells(self):
        rows = [
            {"enc_left_position": "", "enc_left_velocity": "", "enc_left_time": ""},
            {"enc_left_position": "10.0", "enc_left_velocity": "1.0", "enc_left_time": "100"},
        ]
        times, positions, velocities = osa.wheel_stream_from_rows(rows, side="left")
        assert times == [100.0]


class TestHeadingStreamFromRows:
    def test_converts_cdeg_and_mradps_to_radians(self):
        # 90.00deg == 9000 cdeg == pi/2 rad; 1000 mrad/s == 1.0 rad/s.
        rows = [{"now": "1000", "pose_theta": "9000", "twist_omega": "1000"}]
        times, headings, omegas = osa.heading_stream_from_rows(rows)
        assert times == [1000.0]
        assert headings[0] == pytest.approx(math.pi / 2.0, abs=1e-6)
        assert omegas[0] == pytest.approx(1.0, abs=1e-9)

    def test_zero_is_zero(self):
        rows = [{"now": "0", "pose_theta": "0", "twist_omega": "0"}]
        _times, headings, omegas = osa.heading_stream_from_rows(rows)
        assert headings == [0.0]
        assert omegas == [0.0]

    def test_skips_rows_with_blank_cells(self):
        rows = [
            {"now": "0", "pose_theta": "", "twist_omega": ""},
            {"now": "1000", "pose_theta": "9000", "twist_omega": "1000"},
        ]
        times, _headings, _omegas = osa.heading_stream_from_rows(rows)
        assert times == [1000.0]

    def test_round_trips_through_one_step_ahead_walk(self):
        """End-to-end: extracted heading/omega streams feed straight into
        one_step_ahead_walk() with no further conversion -- a captured CSV's
        pose_theta/twist_omega columns produce the SAME radian-domain
        residual a hand-computed heading fixture would."""
        rows = [
            {"now": "0", "pose_theta": "0", "twist_omega": "1000"},        # 0rad, 1.0rad/s
            {"now": "1000", "pose_theta": "10000", "twist_omega": "1000"},  # 100deg == 1.745rad
        ]
        times, headings, omegas = osa.heading_stream_from_rows(rows)
        walk = osa.one_step_ahead_walk(times, headings, omegas)
        assert len(walk) == 1
        # basis heading=0, omega=1.0rad/s, age=1.0s -> predicted=1.0rad
        assert walk[0].predicted == pytest.approx(1.0, abs=1e-6)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
