"""tests/testgui/test_error_divergence.py — end-to-end error-divergence test
(ticket 083-004): the sprint's headline Success Criteria, automated.

sprint.md's Success Criteria states: "injecting a slip/encoder-error profile
visibly separates the encoder trace from truth". This module is the
automated, scripted proof of that claim — it drives the fully reconciled
stack (SimTransport + TraceModel, tickets 083-001/002/003 together, not in
isolation) exactly the way an operator would from the GUI:

1. Connect a real ``SimTransport`` (ctypes sim, ticket 083-001).
2. Drive forward with ``S 200 200`` (097: ``DEV DT VW``/``DEV DT PORTS`` --
   the wire strings ``KeyboardDriver`` used pre-097 -- have no binary arm
   and never will; the legacy ``DEV`` debug command family was retired
   along with the rest of the text plane. ``S`` is translated to a binary
   ``CommandEnvelope{drive: DrivetrainCommand{wheels}}`` and commands
   ``Drivetrain`` directly, the same as ``DEV DT VW`` did).
3. Apply a nonzero ``enc_scale_err_l`` error profile via
   ``SimTransport.apply_error_profile()`` (ticket 083-001).
4. Tick the sim forward in real time and feed telemetry into a
   ``TraceModel`` (ticket 083-003).
5. Assert the ``encoder`` trace has measurably diverged from the ``camera``
   (ground-truth) trace by an amount consistent with the injected error.

Why ``enc_scale_err_l`` (not ``encoder_noise``)
------------------------------------------------
``enc_scale_err_l`` is a per-side encoder over/under-*report* (see
``sim_prefs.py``'s module docstring) — it does not touch the physical plant
at all (``PhysicsWorld`` still drives dead straight for a symmetric ``VW``
command), only what the firmware's own encoder-based dead-reckoning
(``TLMFrame.encpose``) believes happened. Driving straight with the left
encoder over-reporting distance makes the firmware compute a *curving*
encoder-only pose while the simulated plant's true motion (and therefore
the camera/ground-truth trace) stays straight — this is precisely "the
encoder trace visibly separates from truth", made concrete and reproducible
without any physical noise/randomness in the assertion. ``encoder_noise``
(pure per-sample sigma) would also separate the traces but through
zero-mean noise, which needs more samples/time to become a *statistically*
reliable, non-flaky assertion; the scale-error path diverges monotonically
and is therefore the more robust automated check.

Threshold calibration
----------------------
Measured empirically against this same stack (``just build-sim`` lib,
2026-07-05): driving ``DEV DT VW 200 0 0`` for ~1.5s wall-clock with
``enc_scale_err_l=0.25`` produces an encoder-vs-camera divergence of ~9cm,
growing to ~15cm by 2s and ~30cm by 3s. The SAME drive with NO error applied
(defaults only) stays under ~1.5cm of divergence throughout (residual is
TLM/truth sampling-time misalignment, not a real pose difference). The
``_DIVERGENCE_THRESHOLD_CM`` below (5cm) sits comfortably above the
no-error noise floor and comfortably below the ~1.5s-in signal, so this
assertion is not on a knife's edge in either direction.

Run with::

    QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui/test_error_divergence.py -q

Requires the compiled ``tests/_infra/sim/build/libfirmware_host.{dylib,so}``
(``just build-sim``) — skips cleanly if not present.
"""
from __future__ import annotations

import math
import time

import pytest

from robot_radio.testgui.transport import SimTransport, _sim_lib_path
from robot_radio.testgui.traces import TraceModel

pytestmark = pytest.mark.skip(
    reason="108-007: enc_scale_err_l has no robot_radio.io.sim_loop.SimLoop "
           "setter at all in the current 19-symbol sim_ctypes.cpp ABI (narrowed "
           "from the deleted ~40-symbol SimConnection one) -- see "
           "clasi/issues/sim-transport-command-set-get-not-supported.md and "
           "sim_prefs.py's own module docstring for the full mapping.",
)

_WAIT_TIMEOUT_S = 10.0
_POLL_INTERVAL_S = 0.02
_MIN_TRACE_POINTS = 10
_MIN_CAMERA_POINTS = 3

# Large enough that the resulting divergence is unmistakable well within
# _DRIVE_DURATION_S, without being so large the plant behaves nonsensically
# (a single wheel "reporting" 2x its true travel, say, is already a gross
# miscalibration -- 0.25 is a moderate, plausible real-world encoder fault).
_ENC_SCALE_ERR_L = 0.25

# How long to let the sim run (wall-clock, after both traces have reached
# their minimum point counts) before comparing endpoints -- the divergence
# grows with distance/time, so a fixed settle window makes the final
# comparison deterministic rather than racing the tick-thread.
_DRIVE_SETTLE_S = 1.5

# See "Threshold calibration" in the module docstring: no-error divergence
# measured under ~1.5cm; injected-error divergence measured ~9cm at the same
# elapsed drive time. 5cm sits well clear of both.
_DIVERGENCE_THRESHOLD_CM = 5.0


def _wait_until(predicate, timeout_s: float = _WAIT_TIMEOUT_S) -> bool:
    """Poll ``predicate`` until it is truthy or ``timeout_s`` elapses."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(_POLL_INTERVAL_S)
    return predicate()


@pytest.fixture
def transport():
    """A connected SimTransport; disconnected on teardown even on failure."""
    t = SimTransport()
    t.on_log = lambda _s: None
    t.connect()
    assert t._connected, "SimTransport failed to connect -- is the sim lib built?"
    try:
        yield t
    finally:
        t.disconnect()


def test_enc_scale_err_separates_encoder_trace_from_camera_truth(
    transport: SimTransport,
) -> None:
    """Injecting ``enc_scale_err_l`` visibly separates the ``encoder`` trace
    from the ``camera`` (ground-truth) trace during a straight-forward drive.

    This is sprint 083's headline Success Criteria
    ("injecting a slip/encoder-error profile visibly separates the encoder
    trace from truth"), automated end-to-end against the real reconciled
    stack (081/082 sim + 083-001/002/003's SimTransport/drive/traces).

    097: un-xfailed. ``encpose`` (the firmware-computed dead-reckoned pose
    this test's ``model.encoder`` trace used to read) still has no wire
    representation on the binary plane (096-001's permanent trim) -- but
    ``TraceModel.feed()`` now synthesizes an equivalent pose host-side via
    ``EncoderDeadReckoner``, integrated from ``frame.enc`` (cumulative
    per-wheel distance, which DOES carry the injected ``enc_scale_err_l``
    over-report -- see ``traces.py``). ``model.encoder`` grows and diverges
    from ``model.camera`` exactly as this test expects.
    """
    model = TraceModel()
    transport.on_telemetry = model.feed

    def _on_truth(pose) -> None:
        if pose is not None:
            model.feed_truth(*pose)

    transport.on_truth = _on_truth

    # Apply the error profile before driving so it is live from the first
    # tick -- apply_error_profile() only needs the one nonzero key; every
    # other profile field falls back to sim_prefs.DEFAULT_PROFILE's neutral
    # value (transport.py's _apply_profile_to_sim() does `profile.get(key,
    # defaults[key])` per field).
    transport.apply_error_profile({"enc_scale_err_l": _ENC_SCALE_ERR_L})

    # 097: DEV DT VW/PORTS have no binary arm (see module docstring) --
    # drive forward via the binary-translated S verb instead.
    transport.send("S 200 200")

    assert _wait_until(lambda: len(model.encoder) >= _MIN_TRACE_POINTS), (
        f"encoder trace only reached {len(model.encoder)} points within "
        f"{_WAIT_TIMEOUT_S}s"
    )
    assert _wait_until(lambda: len(model.camera) >= _MIN_CAMERA_POINTS), (
        f"camera trace only reached {len(model.camera)} points within "
        f"{_WAIT_TIMEOUT_S}s"
    )

    # Let the divergence accumulate for a bit longer -- the effect grows
    # with distance travelled / time, not instantaneously (see "Threshold
    # calibration" above).
    time.sleep(_DRIVE_SETTLE_S)

    enc_x, enc_y = model.encoder[-1]
    cam_x, cam_y = model.camera[-1]
    divergence_cm = math.hypot(enc_x - cam_x, enc_y - cam_y)

    assert divergence_cm > _DIVERGENCE_THRESHOLD_CM, (
        f"encoder trace ({enc_x:.1f}, {enc_y:.1f}) cm did not measurably "
        f"diverge from camera ground truth ({cam_x:.1f}, {cam_y:.1f}) cm "
        f"beyond the {_DIVERGENCE_THRESHOLD_CM}cm threshold; "
        f"divergence={divergence_cm:.2f}cm "
        f"(encoder points={len(model.encoder)}, camera points={len(model.camera)})"
    )

    # Sanity check on direction: both traces should still have moved
    # forward overall (the injected error separates them, it does not stop
    # the robot) -- guards against a vacuous pass where both traces are
    # degenerate/empty-ish.
    assert enc_x > 1.0, f"encoder trace did not move forward in x: {model.encoder}"
    assert cam_x > 1.0, f"camera trace did not move forward in x: {model.camera}"
