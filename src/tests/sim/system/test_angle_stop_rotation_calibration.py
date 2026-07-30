"""src/tests/sim/system/test_angle_stop_rotation_calibration.py -- 125-007's
own regression test (adjacent-sim-plant-rotation-calibration-for-angle-stop-
move-overshoot.md): a single 90 deg ANGLE-stop MOVE, measured against sim
plant truth, must land close to the requested angle -- so this ticket's
defect (a ~13 deg constant overshoot, `square_tour.py --sim` closing ~288mm
short instead of the required <=60mm) cannot silently reappear.

Root cause (this ticket's own investigation, not assumed): `TestSim::
WheelPlant`'s first-order duty->velocity model (`kDefaultTau = 0.13s`,
matching the bench-characterized actuation-lag range) means the wheels keep
coasting for a beat after an ANGLE stop condition fires, the same physical
mechanism real hardware's own actuation lag produces -- a genuine plant
artifact, not a stop-condition bug (`App::RobotLoop::handleMove()`'s ANGLE
threshold math is unmodified by this ticket). What WAS broken: nothing in
the sim path ever called `App::RobotLoop::setRotationCalibration()` at all
-- `SimLoop.configure_from_robot()`'s Tier 2 never touched turn calibration,
so a robot JSON's own `calibration.rotation_gain`/`rotation_offset_deg`
values were a silent no-op for the sim, identical to real hardware's own
boot seam (`main.cpp`) in every way except that it was simply never wired
up. This ticket added that wiring (`sim_configure_drivetrain()`,
`sim_ctypes.cpp`/`sim_boot_config.py`) and fit `data/robots/tovez_nocal.json`'s
rotation_gain/rotation_offset_deg to the sim plant's own measured overshoot
at omega=2.0 rad/s (`square_tour.py`'s own `runTurnMove()` rate) -- see that
JSON's own `_rotation_calibration_note` for the full measurement.

Run with::

    uv run python -m pytest src/tests/sim/system/test_angle_stop_rotation_calibration.py -v -s

Requires the compiled ``src/sim/build/libfirmware_host.{dylib,so}``
(``just build-sim`` or ``cmake --build src/sim/build``) -- skips cleanly if
not present, mirroring every other system/ sim test's own convention.
"""
from __future__ import annotations

import math
import pathlib
import sys

import pytest

# src/tests/sim/system/test_angle_stop_rotation_calibration.py -> system ->
# sim -> tests -> src -> repo root = FOUR hops from __file__, the same
# convention every other system/ sim test's own header establishes.
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"

_LIB_NAME = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
_SIM_LIB_PATH = _REPO_ROOT / "src" / "sim" / "build" / _LIB_NAME

pytestmark = pytest.mark.skipif(
    not _SIM_LIB_PATH.exists(),
    reason="sim lib not built -- cmake --build src/sim/build (or `just build-sim`)",
)

_TRACK_WIDTH = 128.0  # [mm] matches tovez_nocal.json's own geometry.trackwidth
_CYCLE_S = 0.04       # [s] one SimLoop.step() -- App::RobotLoop::kCycle
_OMEGA = 2.0          # [rad/s] matches square_tour.py's runTurnMove()
_STOP_DEG = 90.0      # [deg] every square_tour.py corner is exactly this

# Generous relative to the fitted residual (~0.2 deg measured, see the robot
# JSON's own note) but tight relative to the UNCORRECTED defect (~13 deg) --
# this bound exists to catch the calibration silently regressing to a no-op
# (e.g. the sim_configure_drivetrain() wiring breaking, or the JSON reverting
# to identity), not to hold the sim plant to a tolerance tighter than this
# ticket ever measured or claimed.
_MAX_OVERSHOOT_DEG = 5.0


def _make_loop():
    """A bare, headless ``SimLoop`` -- deterministic manual stepping,
    mirroring test_sim_configure_from_robot.py's own ``_make_loop()``."""
    from robot_radio.io.sim_loop import SimLoop

    loop = SimLoop(track_width=_TRACK_WIDTH, lib_path=_SIM_LIB_PATH)
    loop.connect(start_tick_thread=False)
    return loop


def _run_angle_stop_move(loop, stop_deg: float, omega: float) -> float:
    """Command one ANGLE-stopped move_twist() (stop_angle=`stop_deg`,
    omega=`omega`) and return the plant's own true heading DELTA in degrees
    (``SimLoop.get_true_pose()`` -- no odometry/estimator in the loop at
    all, matching square_tour.py's own sim ground truth)."""
    from robot_radio.io.sim_config import SimConfigConn
    from robot_radio.robot.protocol import NezhaProtocol

    proto = NezhaProtocol(SimConfigConn(loop))
    rad = math.radians(stop_deg)
    timeout = abs(rad / omega) * 1000.0 * 3.0 + 3000.0  # [ms]

    pose0 = loop.get_true_pose()
    proto.move_twist(0.0, 0.0, omega, stop_angle=rad, timeout=timeout, move_id=1)

    active_seen = False
    deadline_cycles = int((timeout / 1000.0 + 2.0) / _CYCLE_S)
    for _ in range(deadline_cycles):
        loop.step(1)
        loop._drain_tlm_into_queue()  # noqa: SLF001 -- mirrors square_tour.py's own SimBackend
        frames = loop.read_pending_binary_tlm_frames()
        completed = False
        for f in frames:
            if f.active:
                active_seen = True
            elif active_seen:
                completed = True
        if completed:
            break

    # Settle, mirroring square_tour.py's own SEGMENT_REST.
    for _ in range(int(1.0 / _CYCLE_S)):
        loop.step(1)
        loop._drain_tlm_into_queue()  # noqa: SLF001
        loop.read_pending_binary_tlm_frames()

    pose1 = loop.get_true_pose()
    return math.degrees(pose1["h"] - pose0["h"])


def test_angle_stop_lands_close_to_target_with_tovez_nocal_calibration():
    """The regression this ticket exists to prevent: a 90 deg ANGLE-stop
    MOVE against `tovez_nocal.json`'s own (now-calibrated) rotation
    constants must land within `_MAX_OVERSHOOT_DEG` of 90 -- not the ~13 deg
    the identity/uncalibrated defect produced."""
    from robot_radio.config.robot_config import load_robot_config

    config = load_robot_config(_ROBOTS_DIR / "tovez_nocal.json")
    loop = _make_loop()
    try:
        loop.configure_from_robot(config)
        loop.step(1)  # let the injected ConfigDelta(s) + drivetrain load land

        actual = _run_angle_stop_move(loop, _STOP_DEG, _OMEGA)
    finally:
        loop.disconnect()

    overshoot = actual - _STOP_DEG
    assert abs(overshoot) <= _MAX_OVERSHOOT_DEG, (
        f"ANGLE-stop MOVE landed at {actual:.2f} deg for a {_STOP_DEG:.0f} deg "
        f"target (overshoot {overshoot:+.2f} deg, bound +/-{_MAX_OVERSHOOT_DEG:.0f} deg) -- "
        f"either tovez_nocal.json's calibration.rotation_gain/rotation_offset_deg "
        f"regressed toward identity, or SimLoop.configure_from_robot()'s "
        f"drivetrain Tier-2 load (sim_configure_drivetrain()) stopped taking effect."
    )


def test_angle_stop_overshoots_without_rotation_calibration():
    """Negative control: WITHOUT any rotation calibration applied (a bare,
    configured-but-otherwise-default SimLoop, never given a drivetrain Tier-2
    load), the SAME 90 deg ANGLE-stop MOVE overshoots by roughly the
    magnitude this ticket measured (~13 deg) -- proving the positive test
    above is actually exercising the calibration path, not passing for some
    unrelated reason (e.g. a stop-condition bound loose enough to swallow
    any overshoot)."""
    from robot_radio.config.robot_config import load_robot_config

    config = load_robot_config(_ROBOTS_DIR / "tovez_nocal.json")
    loop = _make_loop()
    try:
        # Tier 1 (ConfigDelta) + Tier 2 motor load only -- deliberately skip
        # configure_from_robot() (which would apply the calibrated
        # rotation constants) and instead push just enough for the
        # config-completeness gate to accept a Move at all, mirroring
        # configure_from_robot()'s own Tier-1 push plus the motor Tier-2
        # load it already does, minus the NEW drivetrain Tier-2 call this
        # ticket added.
        import ctypes

        from robot_radio.calibration.push import calibration_kwargs
        from robot_radio.calibration.sim_boot_config import motor_boot_config_for
        from robot_radio.io.sim_config import SimConfigConn
        from robot_radio.robot.protocol import NezhaProtocol

        config_proto = NezhaProtocol(SimConfigConn(loop))  # type: ignore[arg-type]
        config_proto.set_config(**calibration_kwargs(config))
        for port in (1, 2):
            motor_cfg = motor_boot_config_for(config, port)
            loop._lib.sim_configure_motor(  # noqa: SLF001
                loop._handle, ctypes.c_int(port),  # noqa: SLF001
                ctypes.c_float(motor_cfg["vel_filt_alpha"]),
                ctypes.c_int(motor_cfg["fwd_sign"]))
        loop.step(1)

        actual = _run_angle_stop_move(loop, _STOP_DEG, _OMEGA)
    finally:
        loop.disconnect()

    overshoot = actual - _STOP_DEG
    assert overshoot > _MAX_OVERSHOOT_DEG, (
        f"expected the UNCALIBRATED plant to overshoot a {_STOP_DEG:.0f} deg "
        f"target by more than {_MAX_OVERSHOOT_DEG:.0f} deg (this ticket measured "
        f"~13 deg), but got {actual:.2f} deg (overshoot {overshoot:+.2f} deg) -- "
        f"the WheelPlant coast-down artifact this ticket calibrates against may "
        f"have changed; if so, re-measure and refit tovez_nocal.json's rotation "
        f"constants (see this file's own module docstring)."
    )
