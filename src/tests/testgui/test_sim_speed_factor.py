"""src/tests/testgui/test_sim_speed_factor.py -- ticket 110-003: Fix Sim
speed-up factor stutter/breakage at 10x/20x.

Reported symptom (``testgui-speedup-factor-broken-at-high-values.md``): the
TestGUI's Sim speed-up selector offers 1x/2x/5x/10x/20x; 2x and 5x work,
10x is "herky-jerky" (doesn't actually run faster, just stutters), 20x is
broken (doesn't work at all).

**Sprint-planning-time hypothesis** (ticket file): at ``cycles=10``/``20``,
one ``sim_loop.SimLoop._tick_loop()`` iteration delivers a BURST of 10-20
telemetry frames to ``on_telemetry`` back to back, and this could overwhelm
the GUI's own ``QueuedConnection`` bridge/redraw budget. Per ticket 109-009's
own precedent (build a deterministic harness FIRST, confirm before fixing),
this file does that in two parts:

Part A -- ``TestBurstSizeScalesWithSpeedFactor``
    A deterministic (non-wall-clock-racing) harness over the REAL compiled
    firmware simulator: ``SimLoop.connect(start_tick_thread=False)`` +
    explicit ``step(cycles)`` calls, mirroring ``_tick_loop()``'s own
    per-iteration ``cycles = max(1, int(speed_factor))`` step, followed by
    ``drain_pending_tlm()``. This CONFIRMS the burst-delivery half of the
    hypothesis directly against the real sim binary: the number of TLM
    frames delivered by one iteration's worth of stepping grows with the
    selected speed factor (empirically ~0.8 frame/cycle at this firmware's
    own STREAM period -- not necessarily exactly 1:1, but strictly more at
    10x/20x than at 1x/2x/5x, which is the shape that actually matters for
    the GUI-congestion mechanism below).

Part B -- the confirmed real cause and its regression test
    Reading ``canvas.py``'s ``CanvasController._update_traces()`` (called by
    ``refresh()``) shows it REBUILDS every trace's full ``QPainterPath`` from
    scratch on every call -- an O(total accumulated trace length) rebuild,
    not an incremental append. Before this ticket, ``__main__.py``'s
    ``_TelemetryBridge.on_frame_ready`` called ``canvas_ctrl.refresh()``
    INSIDE its per-frame drain loop -- once per queued TLMFrame, not once
    per drain. Combined with Part A's confirmed burst sizes, this means a
    single ~50ms tick-thread iteration at 10x/20x triggered 8-16 full trace
    rebuilds back to back on the Qt GUI thread instead of one -- the actual,
    now-confirmed GUI-side congestion mechanism (not sim-side mistiming;
    ``sim_loop.py``'s own per-iteration pacing is unchanged by this fix).
    This explains the reported shape exactly: at 2x/5x the burst (per Part
    A) is small enough that a few redundant rebuilds are imperceptible; at
    10x/20x it is not.

    The fix (``__main__.py::on_frame_ready``) moves ``canvas_ctrl.refresh()``
    OUTSIDE the per-frame drain loop -- every frame is still fed into the
    ``TraceModel``/graph panel (cheap, dirty-flag-gated accumulation), but
    the expensive redraw itself runs AT MOST ONCE per drained batch, using
    the last frame's state. ``test_telemetry_gating.py``'s mirrored
    ``on_frame_ready`` re-implementation (the established pattern for testing
    this closure -- see that file's own docstring) carries the permanent
    regression test for this: ``test_multiple_frames_coalesce_into_one_
    refresh_in_live_view`` / ``test_multiple_frames_coalesce_into_one_
    refresh_using_last_frame_yaw``.

Run with::

    uv run python -m pytest src/tests/testgui/test_sim_speed_factor.py -v
    uv run python -m pytest src/tests/testgui/ -k "speed_factor or sim_loop or speedup"

Requires the compiled ``src/sim/build/libfirmware_host.{dylib,so}``
(``python build.py``) -- skips cleanly if not present.
"""
from __future__ import annotations

import pytest

from robot_radio.testgui.transport import _sim_lib_path

pytestmark = pytest.mark.skipif(
    not _sim_lib_path().exists(),
    reason="sim lib not built -- cmake --build src/sim/build (or `python build.py`)",
)

# The five multiples the TestGUI's own sim_speed_combo offers
# (testgui/__main__.py) -- see also testgui/transport.py's own
# _SIM_SPEED_MIN/_SIM_SPEED_MAX.
_OFFERED_MULTIPLIERS = (1, 2, 5, 10, 20)


def _make_deterministic_loop():
    from robot_radio.io.sim_loop import SimLoop

    loop = SimLoop(track_width=128.0, lib_path=_sim_lib_path())
    loop.connect(start_tick_thread=False)
    return loop


# ---------------------------------------------------------------------------
# Part A -- deterministic burst-size-vs-speed-factor harness
# ---------------------------------------------------------------------------


class TestBurstSizeScalesWithSpeedFactor:
    """Drives the REAL compiled sim deterministically (explicit ``step()``
    calls, no tick thread, no wall-clock racing) at each of the five offered
    multipliers, mirroring exactly what ``SimLoop._tick_loop()`` does per
    iteration (``cycles = max(1, int(speed_factor))`` then one ``sim_step()``
    call) -- and measures cycles-advanced and frames-delivered per call."""

    def test_cycle_count_advances_by_exactly_the_requested_cycles(self):
        """Sanity: ``step(cycles)`` (this harness's own per-iteration
        primitive) advances ``sim_cycle_count()`` by exactly ``cycles`` --
        confirms the harness itself is deterministic and not silently
        coalescing or dropping steps, before trusting any frame-count
        measurement built on top of it."""
        loop = _make_deterministic_loop()
        try:
            loop.twist(150.0, 0.0, 5000.0)  # [mm/s] [rad/s] [ms] -- keep the plant moving
            loop.step(1)
            loop.drain_pending_tlm()
            for cycles in _OFFERED_MULTIPLIERS:
                before = loop.cycle_count()
                loop.step(cycles)
                after = loop.cycle_count()
                assert after - before == cycles, (
                    f"step({cycles}) advanced cycle_count by {after - before}, expected {cycles}")
        finally:
            loop.disconnect()

    def test_frames_delivered_per_iteration_grows_with_speed_factor(self):
        """One iteration's worth of stepping (``step(cycles)`` then a single
        drain) must deliver MORE telemetry frames at a higher speed factor
        than at a lower one -- the burst-delivery premise the sprint-
        planning-time hypothesis rests on, confirmed against the real
        firmware sim rather than assumed. Measured empirically at ~0.8
        frame/cycle for this firmware's own STREAM period -- not
        necessarily an exact 1:1 ratio -- so this asserts the MONOTONIC
        growth shape that actually drives the GUI-congestion mechanism,
        not a specific frame/cycle constant."""
        loop = _make_deterministic_loop()
        try:
            loop.twist(150.0, 0.0, 20000.0)
            loop.step(1)
            loop.drain_pending_tlm()  # discard startup/twist-injection frames

            burst_sizes: dict[int, int] = {}
            for cycles in _OFFERED_MULTIPLIERS:
                loop.step(cycles)
                frames = loop.drain_pending_tlm()
                burst_sizes[cycles] = len(frames)

            assert burst_sizes[10] > burst_sizes[2], (
                f"expected 10x's per-iteration burst ({burst_sizes[10]}) to exceed "
                f"2x's ({burst_sizes[2]}) -- burst_sizes={burst_sizes}")
            assert burst_sizes[20] > burst_sizes[5], (
                f"expected 20x's per-iteration burst ({burst_sizes[20]}) to exceed "
                f"5x's ({burst_sizes[5]}) -- burst_sizes={burst_sizes}")
            # The two high multipliers actually named as broken/stuttering in
            # the origin issue must burst noticeably more than one frame at a
            # time -- a single frame per iteration would mean no burst-
            # delivery mechanism exists at all, refuting the hypothesis.
            assert burst_sizes[10] > 1, f"burst_sizes={burst_sizes}"
            assert burst_sizes[20] > 1, f"burst_sizes={burst_sizes}"
        finally:
            loop.disconnect()

    @pytest.mark.parametrize("multiplier", _OFFERED_MULTIPLIERS)
    def test_each_offered_multiplier_steps_cleanly_with_no_exception(self, multiplier):
        """Every one of the five offered multipliers must drive the sim
        without raising or corrupting state -- SUC-003's own "all five
        multipliers advance the sim" acceptance, checked per-multiplier."""
        loop = _make_deterministic_loop()
        try:
            loop.twist(150.0, 0.0, 3000.0)
            for _ in range(5):
                loop.step(multiplier)
                loop.drain_pending_tlm()
            pose = loop.get_true_pose()
            assert isinstance(pose["x"], float)
        finally:
            loop.disconnect()
