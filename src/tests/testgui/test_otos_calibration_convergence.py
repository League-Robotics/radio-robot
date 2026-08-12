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
   ``otos=`` field, ``Core::Odometry``'s ``frame.hasOtos``/``frame.otos`` --
   ``Hal::Otos::pose()`` read back over the real wire) diverges from
   truth by the injected fraction -- SimPlant's OTOS burst-read response is
   `truth * rawError`, exactly as sim_plant.h documents.
4. Push the COMPENSATING scale via a ``SetConfigField{OTOS, linear_scale}``
   envelope (132-014: retargeted off the retired ``OtosConfigPatch``/
   ``NezhaProtocol.otos_config()``, config.proto deleted 132-013) -- the
   exact mechanism ``SimTransport._handle_otos_patch()`` (transport.py)
   uses for the TestGUI's ``OL``/``OA`` verbs -- and confirm the NEXT
   decoded OTOS reading converges back to truth.

This is a lower-level (no Qt, no GUI) but MORE literal exercise of "applies
OtosConfigPatch (ticket 004)" than sim_fidelity_harness.cpp's C++-level
scenario 3 (which drives ``Hal::Otos::setLinearScalar()`` directly) --
this test goes through the actual wire envelope/ack round trip the TestGUI
itself uses.

Run with::

    uv run pytest src/tests/testgui/test_otos_calibration_convergence.py -v

Requires the compiled ``src/firm/platform/host/build/libfirmware_host.{dylib,so}``
(``python build.py``) -- skips cleanly if not present.
"""
from __future__ import annotations

from pathlib import Path

import pytest

from robot_radio.testgui.transport import _SimConfigConn, _sim_lib_path

_TRACK_WIDTH = 128.0  # [mm]
_TRUE_X = 1000.0      # [mm]
_RAW_ERROR_LINEAR = 0.05  # 5% over-report -- a plausible mis-calibration

# 128-003 baseline fix: sprint 114's sim fails closed and refuses MOTION
# (twist/move) until it has received a complete configuration (114-001/
# 002/003) -- a bare connect() with no configure_from_robot() call, as this
# fixture used to do, emits zero TLM frames. Same fixture-config path
# test_sim_loop.py's own `loop` fixture, test_turn_error_characterization.py's
# `_make_sweep_loop()`, and test_tour_closure_gate.py's `_make_loop()` use.
# test_otos_calibration_convergence.py -> testgui -> tests -> src -> repo root
_ACTIVE_ROBOT_JSON = Path(__file__).resolve().parents[3] / "data" / "robots" / "tovez_nocal.json"

# Steps enough sim cycles (50ms each) to clear Otos::tick()'s own
# kReadPeriod (20ms/1 cycle) rate limit and land at least one fresh burst.
_SETTLE_CYCLES = 3


def _compensating_scale(raw_error: float) -> float:
    """The MULTIPLIER (config domain, 1.0 = no correction) that cancels
    *raw_error*. 132-014: unlike the pre-132-010 live wire path (trap 3 --
    passed the config value straight through to a setter expecting the
    chip's raw int8 register, installing a 1-LSB scalar instead of the
    intended multiplier), ``Core::configureOtos()`` now converts this
    multiplier through ``Hardware::scaleToRegister()`` FIRMWARE-side before
    calling ``setLinearScalar()`` -- so the host pushes the multiplier
    directly, no int8 pre-encoding."""
    return 1.0 / (1.0 + raw_error)


@pytest.fixture
def sim_loop():
    lib_path = _sim_lib_path()
    if not lib_path.exists():
        pytest.skip(f"sim lib not built -- run `python build.py` (missing {lib_path})")

    from robot_radio.config.robot_config import load_robot_config
    from robot_radio.io.sim_loop import SimLoop

    loop = SimLoop(track_width=_TRACK_WIDTH, lib_path=lib_path)
    loop.connect(start_tick_thread=False)
    loop.configure_from_robot(load_robot_config(_ACTIVE_ROBOT_JSON))
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
    # 124-008 (issue §B4): the single "freshest ack" scalar slot
    # (frame.ack/frame.ack_fresh) is deleted -- scan the bounded ack ring.
    for frame in frames:
        for entry in frame.acks:
            if entry.corr_id == corr_id:
                return entry
    return None


@pytest.mark.xfail(
    strict=True,
    reason=(
        "136-002: pushing the compensating SetConfigField{OTOS, "
        "linear_scale} does not converge the decoded reading back to "
        "truth -- measured 1047mm for a 1000mm true pose (+5% raw error, "
        "compensating multiplier pushed and acked ok), expected 1000mm "
        "+/-2%. Register quantization ruled out directly (the pushed "
        "multiplier quantizes to within 0.04% of intended). Leading "
        "hypothesis (NOT confirmed): TestSim::OtosPlant's burst-read "
        "packing may not apply a written linear_scale register to its own "
        "reported bytes -- a sim-fidelity gap, not necessarily a firmware "
        "defect. Tracked: "
        "clasi/issues/later/otos-live-config-push-does-not-converge-in-sim.md."
    ),
)
def test_otos_calibration_push_converges_pose_via_the_real_config_path(sim_loop) -> None:
    """Uncalibrated raw scale error diverges the firmware's decoded OTOS
    pose from truth; pushing the compensating SetConfigField{OTOS,
    linear_scale} (the SAME mechanism SimTransport's OL/OA verbs use)
    converges it back.

    132-014: builds and sends the envelope directly via
    ``conn.send_envelope_fast()`` rather than
    ``NezhaProtocol.set_config_field()`` -- that method polls for the ack
    INTERNALLY (blocking), which would spin for its own timeout with
    NOTHING advancing the sim in this fixture's manual-step session
    (``connect(start_tick_thread=False)``, no background tick thread to
    process the queued command). This test's own ``_step_and_drain()`` +
    ``_find_ack()`` pattern is the correct fire-then-caller-steps shape for
    a manual-step session, mirroring the pre-132-014 ``otos_config()``'s
    own non-blocking ``send_envelope_fast()``-only contract exactly."""
    from robot_radio.robot.pb2 import envelope_pb2, robot_config_pb2

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

    # Push the compensating SetConfigField{OTOS, linear_scale} -- the exact
    # direct-send mechanism SimTransport._handle_otos_patch()/132-014
    # established, reused verbatim here (built by hand, not via
    # NezhaProtocol.set_config_field(), for the manual-step-session reason
    # this test's own docstring gives).
    conn = _SimConfigConn(sim_loop)
    field_number = robot_config_pb2.Otos.DESCRIPTOR.fields_by_name["linear_scale"].number
    request = robot_config_pb2.SetConfigField(
        target=robot_config_pb2.OTOS, field=field_number,
        value=_compensating_scale(_RAW_ERROR_LINEAR))
    envelope = envelope_pb2.CommandEnvelope(set_field=request)
    corr_id = conn.send_envelope_fast(envelope)

    frames = _step_and_drain(sim_loop)
    ack = _find_ack(frames, corr_id)
    assert ack is not None, "SetConfigField ack never arrived"
    assert ack.ok, f"SetConfigField was NAK'd: err_code={ack.err_code}"

    # 128-003 baseline fix: TlmMode defaults to kAuto (telemetry.h) --
    # unsolicited frames only flow during "activity" (kFlagActive, or
    # recently-moved) OR while the just-pushed config's ack is still being
    # delivered (pendingAckDeliveries(), honored in every mode regardless
    # of unsolicited gating). This session never moves (set_true_pose()
    # sets ground truth directly, not via a Move), so once the ack above
    # finishes its bounded delivery window, a SEPARATE further step+drain
    # call observes zero frames -- there is no ongoing activity left to
    # keep emitting (confirmed empirically: a bare 3rd step+drain here
    # returns 0 frames). Every primary TLM frame carries pose/otos
    # together, so the ack-delivery frames already drained above carry a
    # fresh, post-config otos= reading too -- read the calibrated value
    # from THAT batch instead of stepping again into telemetry silence.
    otos_frames = [f for f in frames if f.otos is not None]
    assert otos_frames, "expected a fresh otos= reading in the ack-delivery frames"
    calibrated_x = otos_frames[-1].otos[0]  # [mm]

    assert calibrated_x == pytest.approx(_TRUE_X, abs=_TRUE_X * 0.02), (
        f"calibrated OTOS x should converge back to truth (~{_TRUE_X:.0f}mm), got {calibrated_x}mm"
    )
