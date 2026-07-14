"""test_sim_realtime.py — timing tests for SimConnection real_time/speed_factor.

Tests (037-002):
  1. test_simconn_default_is_fast     — real_time=False runs well under sim time
  2. test_simconn_realtime_pacing     — real_time=True paces to wall-clock
  3. test_simconn_speed_factor        — speed_factor=4.0 runs at 4× real time
  4. test_make_target_sim_realtime    — make_target("sim", real_time=True) wires flag

Bench/production paths are inherently wall-clock; the real_time flag is a
no-op there (they don't use SimConnection's ticking).

All real-time assertions are marked ``pytest.mark.slow`` and skipped in CI
(CI=true/1) to keep the default run fast.
"""

from __future__ import annotations

import os
import time

import pytest

# Skip real-time wall-clock tests in CI — they take real seconds.
_IN_CI = os.environ.get("CI", "").lower() in ("true", "1", "yes")
_slow = pytest.mark.skipif(_IN_CI, reason="slow real-time test, skipped in CI")


# --------------------------------------------------------------------------- #
# Test 1 — default (real_time=False) runs at full CPU speed                   #
# --------------------------------------------------------------------------- #

class TestSimConnDefaultIsFast:
    """SimConnection with default real_time=False must be much faster than sim time."""

    def test_simconn_default_is_fast(self):
        """real_time=False for 200 ms sim time finishes well under 1.0 s wall time."""
        from robot_radio.io.sim_conn import SimConnection

        conn = SimConnection(real_time=False)
        result = conn.connect(skip_ping=True)
        assert "error" not in result, f"connect failed: {result}"

        start = time.monotonic()
        conn.tick(200)
        elapsed = time.monotonic() - start

        conn.disconnect()

        # 200 ms of sim time at full CPU should be done in well under 1 s.
        assert elapsed < 1.0, (
            f"real_time=False took {elapsed:.3f}s for 200ms sim — unexpectedly slow"
        )

    def test_simconn_real_time_attribute_default_false(self):
        """SimConnection() default: _real_time=False, _speed_factor=1.0."""
        from robot_radio.io.sim_conn import SimConnection

        conn = SimConnection()
        assert conn._real_time is False
        assert conn._speed_factor == 1.0

    def test_simconn_real_time_attribute_set(self):
        """SimConnection(real_time=True, speed_factor=2.0) stores the values."""
        from robot_radio.io.sim_conn import SimConnection

        conn = SimConnection(real_time=True, speed_factor=2.0)
        assert conn._real_time is True
        assert conn._speed_factor == 2.0


# --------------------------------------------------------------------------- #
# Test 2 — real_time=True paces to wall-clock                                 #
# --------------------------------------------------------------------------- #

class TestSimConnRealtimePacing:
    """SimConnection(real_time=True) must take at least 95% of sim time."""

    @_slow
    def test_simconn_realtime_pacing(self):
        """real_time=True, 200 ms sim → wall time ≥ 190 ms."""
        from robot_radio.io.sim_conn import SimConnection

        conn = SimConnection(real_time=True)
        result = conn.connect(skip_ping=True)
        assert "error" not in result, f"connect failed: {result}"

        sim_ms = 200
        start = time.monotonic()
        conn.tick(sim_ms)
        elapsed_ms = (time.monotonic() - start) * 1000

        conn.disconnect()

        assert elapsed_ms >= sim_ms * 0.95, (
            f"real_time=True took {elapsed_ms:.1f}ms wall for {sim_ms}ms sim "
            f"(expected ≥ {sim_ms * 0.95:.1f}ms)"
        )


# --------------------------------------------------------------------------- #
# Test 3 — speed_factor scales wall-clock time                                #
# --------------------------------------------------------------------------- #

class TestSimConnSpeedFactor:
    """speed_factor=4.0 must be ~4× faster than real time."""

    @_slow
    def test_simconn_speed_factor(self):
        """real_time=True, speed_factor=4.0, 200 ms sim → wall time ≥ 45 ms."""
        from robot_radio.io.sim_conn import SimConnection

        conn = SimConnection(real_time=True, speed_factor=4.0)
        result = conn.connect(skip_ping=True)
        assert "error" not in result, f"connect failed: {result}"

        sim_ms = 200
        expected_wall_ms = sim_ms / 4.0  # = 50 ms
        start = time.monotonic()
        conn.tick(sim_ms)
        elapsed_ms = (time.monotonic() - start) * 1000

        conn.disconnect()

        # Lower bound: at least 90% of expected wall time.
        lower_bound = expected_wall_ms * 0.90
        assert elapsed_ms >= lower_bound, (
            f"speed_factor=4.0 took {elapsed_ms:.1f}ms wall for {sim_ms}ms sim "
            f"(expected ≥ {lower_bound:.1f}ms)"
        )
        # Upper bound: no more than the full real-time duration (sanity check).
        assert elapsed_ms < sim_ms, (
            f"speed_factor=4.0 should be faster than real time, "
            f"but took {elapsed_ms:.1f}ms for {sim_ms}ms sim"
        )


# --------------------------------------------------------------------------- #
# Test 4 — make_target("sim", real_time=True) wires the flag                  #
# --------------------------------------------------------------------------- #

class TestMakeTargetSimRealtime:
    """make_target("sim", real_time=True) must set conn._real_time=True."""

    def test_make_target_sim_realtime(self):
        """make_target("sim", real_time=True).conn._real_time is True."""
        from robot_radio.testkit import make_target

        tr = make_target("sim", real_time=True)
        try:
            assert tr.conn._real_time is True, (
                f"conn._real_time should be True, got {tr.conn._real_time!r}"
            )
            assert tr.real_time is True, (
                f"tr.real_time should be True, got {tr.real_time!r}"
            )
        finally:
            tr.conn.disconnect()

    def test_make_target_sim_realtime_speed_factor(self):
        """make_target("sim", real_time=True) with default speed_factor=1.0."""
        from robot_radio.io.sim_conn import SimConnection

        conn = SimConnection(real_time=True)
        assert conn._speed_factor == 1.0

    def test_make_target_sim_default_not_realtime(self):
        """make_target("sim") (default) has real_time=False."""
        from robot_radio.testkit import make_target

        tr = make_target("sim")
        try:
            assert tr.conn._real_time is False
            assert tr.real_time is False
        finally:
            tr.conn.disconnect()
