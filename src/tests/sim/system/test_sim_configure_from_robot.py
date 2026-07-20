"""src/tests/sim/system/test_sim_configure_from_robot.py -- ticket 113-005's
own acceptance proof: a HEADLESS ``robot_radio.io.sim_loop.SimLoop`` caller
(no ``SimTransport``, no Qt, no ``robot_radio.testgui`` anywhere in this
file's own imports) can configure the sim from a robot's own JSON-derived
``RobotConfig`` via ``SimLoop.configure_from_robot()`` -- sprint 113's
SUC-002 ("Headless SimLoop gets the same config as the TestGUI").

Covers:

1. ``configure_from_robot()`` succeeds against a bare, headless ``SimLoop``
   (``start_tick_thread=False``, ticket 009's own deterministic-stepping
   pattern) loaded with ``data/robots/tovez_nocal.json`` -- no exception,
   and no NEW ``robot_radio.testgui`` module enters ``sys.modules`` as a
   side effect of the call (a dynamic, not just import-grep, proof of
   SUC-002's "no dependency on TestGUI/Qt code path" acceptance criterion).
2. The Tier-1 push actually reaches and is APPLIED by
   ``RobotLoop::handleConfig()`` -- proven by draining the sim's own ack
   ring after ``configure_from_robot()`` and observing at least one OK ack,
   not merely that the host-side call returned without raising.
3. ``configure_from_robot()`` measurably changes the sim's own plant
   response to an identical twist: ``tovez_nocal.json``'s
   ``control.vel_kp`` (0.002) differs from ``SimHarness::makeMotorConfig()``'s
   own hardcoded stand-in gain (0.003, ``sim_harness.h``) -- a direct,
   deterministic before/after comparison of two freshly-connected
   ``SimLoop``s (one left at the sim's own defaults, one configured from
   ``tovez_nocal.json``) driven with the SAME twist for the SAME number of
   cycles must diverge, or ``configure_from_robot()`` is not actually
   reaching the firmware's own gains.

Run with::

    uv run python -m pytest src/tests/sim/system/test_sim_configure_from_robot.py -v -s

Requires the compiled ``src/sim/build/libfirmware_host.{dylib,so}``
(``python build.py`` or ``cmake --build src/sim/build``) -- skips cleanly if
not present.
"""
from __future__ import annotations

import pathlib
import sys

import pytest

# src/tests/sim/system/test_sim_configure_from_robot.py -> system -> sim ->
# tests -> src -> repo root = FOUR hops from __file__ (the same convention
# src/host/robot_radio/calibration/sim_boot_config.py's own header
# establishes for its own four-hop path).
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"

_LIB_NAME = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
_SIM_LIB_PATH = _REPO_ROOT / "src" / "sim" / "build" / _LIB_NAME

pytestmark = pytest.mark.skipif(
    not _SIM_LIB_PATH.exists(),
    reason="sim lib not built -- cmake --build src/sim/build (or `python build.py`)",
)

_TRACK_WIDTH = 128.0  # [mm] matches tovez_nocal.json's own geometry.trackwidth


def _make_loop():
    """A bare, headless ``SimLoop`` -- deterministic manual stepping
    (``start_tick_thread=False``, ticket 009's own precedent), no
    ``SimTransport``, no Qt. Deferred import (matching this test's own
    "no testgui import anywhere in this file" proof -- see module
    docstring point 1) even though ``robot_radio.io.sim_loop`` itself has
    no testgui dependency; keeping the import local to this helper is
    simply this file's own convention, not a requirement."""
    from robot_radio.io.sim_loop import SimLoop

    loop = SimLoop(track_width=_TRACK_WIDTH, lib_path=_SIM_LIB_PATH)
    loop.connect(start_tick_thread=False)
    return loop


def test_configure_from_robot_succeeds_headless_no_testgui_import():
    """Constructing a bare SimLoop (no SimTransport, no Qt) and calling
    configure_from_robot() with a tovez_nocal.json-loaded RobotConfig
    succeeds -- SUC-002's own acceptance criterion, proven concretely (not
    just by import-grep): no exception, and no NEW robot_radio.testgui
    module enters sys.modules as a side effect of THIS call."""
    from robot_radio.config.robot_config import load_robot_config

    config = load_robot_config(_ROBOTS_DIR / "tovez_nocal.json")
    loop = _make_loop()
    try:
        modules_before = set(sys.modules)
        loop.configure_from_robot(config)
        modules_after = set(sys.modules)

        new_testgui_modules = {
            name for name in (modules_after - modules_before)
            if name == "robot_radio.testgui" or name.startswith("robot_radio.testgui.")
        }
        assert not new_testgui_modules, (
            "configure_from_robot() pulled in robot_radio.testgui module(s) "
            f"it must have zero dependency on: {sorted(new_testgui_modules)}"
        )
    finally:
        loop.disconnect()


def test_configure_from_robot_tier1_push_is_acked_by_firmware():
    """The Tier-1 ConfigDelta push configure_from_robot() sends is not just
    "sent without raising" -- it reaches RobotLoop::handleConfig() and gets
    ACKed. Steps a few cycles after the call (manual/deterministic mode) so
    the injected ConfigDelta command(s) are processed and their acks ride
    back out on a subsequent Telemetry push, then drains and asserts at
    least one OK ack landed."""
    from robot_radio.config.robot_config import load_robot_config

    config = load_robot_config(_ROBOTS_DIR / "tovez_nocal.json")
    loop = _make_loop()
    try:
        loop.configure_from_robot(config)

        acks = []
        for _ in range(5):
            loop.step(1)
            for frame in loop.drain_pending_tlm():
                acks.extend(frame.acks or [])
            if any(ack.ok for ack in acks):
                break

        assert acks, "no acks observed at all after configure_from_robot()'s Tier-1 push"
        assert any(ack.ok for ack in acks), (
            f"configure_from_robot()'s Tier-1 ConfigDelta push was never OK-acked: {acks}"
        )
    finally:
        loop.disconnect()


def test_configure_from_robot_tovez_nocal_changes_measurable_drive_behavior():
    """tovez_nocal.json's control.vel_kp (0.002) differs from SimHarness's
    own hardcoded stand-in motor gain (0.003, sim_harness.h's
    makeMotorConfig()) -- a Tier-1 MotorConfigPatch field with a real,
    already-proven firmware consumer. Driving the SAME twist for the SAME
    number of cycles on two freshly-connected SimLoops -- one left at the
    sim's own hardcoded defaults, one configured via
    configure_from_robot(tovez_nocal_config) -- must produce a DIFFERENT
    plant response; identical responses would mean configure_from_robot()
    is not actually reaching the firmware's own velocity-PID gains."""
    from robot_radio.config.robot_config import load_robot_config

    config = load_robot_config(_ROBOTS_DIR / "tovez_nocal.json")
    assert config.control is not None
    assert config.control.vel_kp == pytest.approx(0.002)

    baseline = _make_loop()
    configured = _make_loop()
    try:
        configured.configure_from_robot(config)
        # Let RobotLoop actually consume+apply the injected ConfigDelta(s)
        # before commanding a twist -- the config command sits in
        # FakeTransport's inbound queue until the next sim_step().
        baseline.step(1)
        configured.step(1)

        for loop in (baseline, configured):
            loop.twist(v_x=200.0, omega=0.0, duration=2000)  # [mm/s] [rad/s] [ms]
            loop.step(10)

        pose_baseline = baseline.get_true_pose()
        pose_configured = configured.get_true_pose()

        assert pose_baseline["x"] != pytest.approx(pose_configured["x"], abs=1e-4), (
            "configure_from_robot()'s Tier-1 push (vel_kp=0.002 vs the sim's own "
            "hardcoded 0.003) produced no observable difference in plant response "
            f"after 10 cycles: baseline={pose_baseline!r} configured={pose_configured!r}"
        )
    finally:
        baseline.disconnect()
        configured.disconnect()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
