"""src/tests/testgui/test_otos_calibration_convergence.py -- ticket 109-007
(sim-honors-otos-calibration.md): SUC-005's second acceptance criterion,
end to end through the SAME transport-layer mechanism a TestGUI operator
drives.

1. Connect a real ``SimLoop`` (108-005/006's ctypes ABI), synchronously
   stepped (``connect(start_tick_thread=False)``) so this test is fully
   deterministic -- no tick-thread race, no wall-clock wait.
2. Set a nonzero raw OTOS linear scale error (``set_otos_raw_scale_err()``,
   109-007) against a known true pose (``set_true_pose()``).
3. Confirm the firmware's own decoded OTOS reading (the primary TLM frame's
   ``otos=`` field, ``App::Odometry``'s ``frame.hasOtos``/``frame.otos`` --
   ``Devices::Otos::pose()`` read back over the real wire) diverges from
   truth by the injected fraction -- SimPlant's OTOS burst-read response is
   `truth * rawError`, exactly as sim_plant.h documents.
4. Push the COMPENSATING ``OtosConfigPatch`` via ``_SimConfigConn``/
   ``NezhaProtocol.otos_config()`` -- the exact mechanism
   ``SimTransport._handle_otos_patch()`` (transport.py) uses for the
   TestGUI's ``OL``/``OA`` verbs and ticket 004's own live calibration path
   -- and confirm the NEXT decoded OTOS reading converges back to truth.

This is a lower-level (no Qt, no GUI) but MORE literal exercise of "applies
OtosConfigPatch (ticket 004)" than sim_fidelity_harness.cpp's C++-level
scenario 3 (which drives ``Devices::Otos::setLinearScalar()`` directly) --
this test goes through the actual wire envelope/ack round trip the TestGUI
itself uses.

Run with::

    uv run pytest src/tests/testgui/test_otos_calibration_convergence.py -v

Requires the compiled ``src/sim/build/libfirmware_host.{dylib,so}``
(``python build.py``) -- skips cleanly if not present.
"""
from __future__ import annotations

import pytest

from robot_radio.testgui.transport import _SimConfigConn, _sim_lib_path

_TRACK_WIDTH = 128.0  # [mm]
_TRUE_X = 1000.0      # [mm]
_RAW_ERROR_LINEAR = 0.05  # 5% over-report -- a plausible mis-calibration

# Steps enough sim cycles (50ms each) to clear Otos::tick()'s own
# kReadPeriod (20ms/1 cycle) rate limit and land at least one fresh burst.
_SETTLE_CYCLES = 3


def _compensating_register(raw_error: float) -> float:
    """The SAME conversion ``push.py``'s ``scale_to_int8()``/
    ``Devices::Otos::scaleToRegister()`` perform: a scale multiplier ->
    the chip's raw int8 register value (0.1%-per-LSB)."""
    scale = 1.0 / (1.0 + raw_error)
    return round((scale - 1.0) / 0.001)


@pytest.fixture
def sim_loop():
    lib_path = _sim_lib_path()
    if not lib_path.exists():
        pytest.skip(f"sim lib not built -- run `python build.py` (missing {lib_path})")

    from robot_radio.io.sim_loop import SimLoop

    loop = SimLoop(track_width=_TRACK_WIDTH, lib_path=lib_path)
    loop.connect(start_tick_thread=False)
    try:
        yield loop
    finally:
        loop.disconnect()


def _step_and_drain(loop) -> list:
    """Step past the rate-limit window and return every frame drained.

    Manual-mode note: with no tick thread running (this fixture's own
    ``connect(start_tick_thread=False)``), nothing steps the sim in the
    background -- ``_SimConfigConn.poll_ack()``'s own polling loop would
    spin forever waiting for a reply that never arrives unless THIS
    (the calling thread) explicitly steps first. Every caller below steps
    before draining/polling for exactly this reason.
    """
    loop.step(_SETTLE_CYCLES)
    return loop.drain_pending_tlm()


def _latest_otos_reading(loop) -> tuple[int, int, int] | None:
    """Step past the rate-limit window and return the last frame's
    ``otos=`` reading, or ``None`` if no frame carried one."""
    frames = _step_and_drain(loop)
    otos_frames = [f for f in frames if f.otos is not None]
    return otos_frames[-1].otos if otos_frames else None


def _find_ack(frames: list, corr_id: int):
    for frame in frames:
        if frame.ack is not None and frame.ack.corr_id == corr_id:
            return frame.ack
    return None


def test_otos_calibration_push_converges_pose_via_the_real_config_path(sim_loop) -> None:
    """Uncalibrated raw scale error diverges the firmware's decoded OTOS
    pose from truth; pushing the compensating OtosConfigPatch (the SAME
    mechanism SimTransport's OL/OA verbs use) converges it back."""
    from robot_radio.robot.protocol import NezhaProtocol

    sim_loop.set_true_pose(_TRUE_X, 0.0, 0.0)
    sim_loop.set_otos_raw_scale_err(_RAW_ERROR_LINEAR, 0.0)

    reading = _latest_otos_reading(sim_loop)
    assert reading is not None, "expected at least one TLM frame carrying an otos= reading"
    uncalibrated_x = reading[0]  # [mm]

    assert uncalibrated_x == pytest.approx(_TRUE_X * (1.0 + _RAW_ERROR_LINEAR), rel=0.05), (
        f"uncalibrated OTOS x should read truth*rawError (~{_TRUE_X * (1.0 + _RAW_ERROR_LINEAR):.0f}mm), "
        f"got {uncalibrated_x}mm"
    )
    assert abs(uncalibrated_x - _TRUE_X) > 20.0, (
        "uncalibrated pose must have MEASURABLY diverged from truth "
        "(the test would be vacuous otherwise)"
    )

    # Push the compensating OtosConfigPatch -- the exact direct-patch-send
    # mechanism SimTransport._handle_otos_patch()/ticket 002 established,
    # reused verbatim here.
    conn = _SimConfigConn(sim_loop)
    proto = NezhaProtocol(conn)  # type: ignore[arg-type]
    corr_id = proto.otos_config(linear_scale=_compensating_register(_RAW_ERROR_LINEAR))

    frames = _step_and_drain(sim_loop)
    ack = _find_ack(frames, corr_id)
    assert ack is not None, "OtosConfigPatch ack never arrived"
    assert ack.ok, f"OtosConfigPatch was NAK'd: err_code={ack.err_code}"

    reading = _latest_otos_reading(sim_loop)
    assert reading is not None, "expected a fresh otos= reading after calibration"
    calibrated_x = reading[0]  # [mm]

    assert calibrated_x == pytest.approx(_TRUE_X, abs=_TRUE_X * 0.02), (
        f"calibrated OTOS x should converge back to truth (~{_TRUE_X:.0f}mm), got {calibrated_x}mm"
    )
