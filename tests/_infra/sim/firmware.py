"""firmware.py — ctypes loader and Sim class for the host simulation
library (sprint 081, ticket 005), wrapping ticket 004's C ABI
(tests/_infra/sim/sim_api.cpp) with Python test-friendly ergonomics.

Loads ``libfirmware_host.dylib`` (macOS) / ``.so`` (Linux) — built by
``just build-sim`` (tests/_infra/sim/CMakeLists.txt) — and exposes every
``sim_*`` entry point through the ``Sim`` context manager below.

Usage::

    from firmware import Sim

    with Sim() as sim:
        sim.command("DEV WD 60000")     # widen the serial-silence watchdog
        sim.command("DEV M 1 VEL 120")
        sim.tick_for(3000)              # [ms]
        vel_l, _ = sim.vel()

See tests/sim/conftest.py's ``sim``/``build_lib`` fixtures for the pytest
integration, and this ticket's closing notes (clasi/sprints/081-.../
tickets/done/005-....md) for the full 40-symbol ABI this class wraps.
"""
from __future__ import annotations

import ctypes
import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve().parent


def _lib_name() -> str:
    return "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"


_DEFAULT_LIB_PATH = _HERE / "build" / _lib_name()

# One [-1,1] float32 reply buffer size -- matches sim_api.cpp's ReplyStore
# capacity (kReplyBufSize) so multi-line replies (e.g. DEV STATE's 5 lines)
# are never silently truncated.
_REPLY_BUF_SIZE = 2048

# Default tick step -- matches the design's ~24 ms control-period increment
# convention (see docs/protocol-v2.md and the 001/003/004 tickets' own use
# of this value).
_DEFAULT_STEP = 24   # [ms]

# Subsystems::Channel's own enum values (source/subsystems/wire_command.h:
# `enum class Channel : uint8_t { NONE, SERIAL, RADIO };`) -- mirrored here
# (and in host/robot_radio/io/sim_conn.py) so callers of command_on() can
# select a channel without reaching into the C++ enum directly (088-006).
CHANNEL_SERIAL = 1
CHANNEL_RADIO = 2


class Sim:
    """Context manager owning one ``SimHandle`` (sim_api.cpp).

    ``sim_destroy`` is called exactly once per instance: either by
    ``__exit__`` (``with Sim() as sim: ...``) or by an explicit ``close()``
    call. A second ``close()`` call is a no-op (idempotent), so callers that
    mix an explicit ``close()`` with a ``with`` block (or a try/finally) do
    not double-free the underlying ``SimHandle``.
    """

    def __init__(self, lib_path: str | pathlib.Path | None = None) -> None:
        self._lib_path = pathlib.Path(lib_path) if lib_path else _DEFAULT_LIB_PATH
        if not self._lib_path.exists():
            raise FileNotFoundError(
                f"libfirmware_host not found at {self._lib_path} -- build it "
                f"first: just build-sim"
            )
        self._lib = ctypes.CDLL(str(self._lib_path))
        self._setup_types()
        self._h = self._lib.sim_create()
        if not self._h:
            raise RuntimeError("sim_create() returned NULL")
        self._now: int = 0   # [ms] mirrors SimHandle::lastTickNow_

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def __enter__(self) -> "Sim":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def close(self) -> None:
        """Destroy the underlying SimHandle. Idempotent -- safe to call
        more than once (a second call is a no-op), so sim_destroy is
        still invoked exactly once per live SimHandle."""
        if self._h is not None:
            self._lib.sim_destroy(self._h)
            self._h = None

    # ------------------------------------------------------------------
    # Time advance
    # ------------------------------------------------------------------

    def tick_for(self, total: int, step: int = _DEFAULT_STEP) -> None:  # [ms] [ms]
        """Advance the sim by ``total`` ms, in ``step`` ms increments.

        Every ``sim_tick()`` call this makes advances ``now`` by EXACTLY
        ``step`` ms -- never a shorter final remainder. A shorter last step
        would hand ``Hal::SimMotor::tick()`` an irregular dt one tick after
        a run of uniform ``step``-sized ones; since that method's velocity
        filter divides THIS tick's own elapsed time into a position delta
        produced by the PREVIOUS tick's plant advance (the documented
        one-tick sample latency, sim_hardware.h's file header), an
        irregular dt scales the reading by the ratio of the two intervals
        and produces a spurious spike -- reproducible directly against
        sim_api.cpp: a run of uniform 24 ms ticks holding steady at a
        converged ~120 mm/s jumps to ~192 mm/s the instant a single
        non-uniform (e.g. 8 ms) tick follows. Matching the project's own
        "~24 ms increment convention" (every tick the same size) sidesteps
        this rather than working around it here. A ``total`` that is not
        an exact multiple of ``step`` therefore advances by
        ``(total // step) * step`` ms, silently dropping the remainder --
        the same behavior tests_old/_infra/sim/firmware.py's tick_for()
        had (its own dt=min(step, remaining) shape defers, rather than
        drops, a short final step into the NEXT call instead, but never
        ticks a partial step either).
        """
        steps = total // step
        for _ in range(steps):
            self._now += step
            self._lib.sim_tick(self._h, ctypes.c_uint32(self._now))

    # ------------------------------------------------------------------
    # Command dispatch
    # ------------------------------------------------------------------

    def command(self, line: str) -> str:
        """Send one command line synchronously; return the decoded reply.

        Routes the command then replays Rt::MainLoop::tick() at the SAME
        ``now`` as the most recent tick_for() call (sim_api.cpp's dt=0
        synchronous-command trick) -- see sim_command()'s own doc comment.
        """
        buf = ctypes.create_string_buffer(_REPLY_BUF_SIZE)
        n = self._lib.sim_command(self._h, line.encode(), buf, _REPLY_BUF_SIZE)
        if n <= 0:
            return ""
        return buf.raw[:n].decode(errors="replace")

    def command_on(self, line: str, channel: int) -> str:
        """Like command(), but selects the reply channel explicitly (088-006).

        ``channel`` is CHANNEL_SERIAL or CHANNEL_RADIO (module constants,
        mirroring Subsystems::Channel). Routes the command with that
        returnPath and reads the reply back from the MATCHING channel's
        sim-side ReplyStore (sim_command_on() -- sim_api.cpp) -- proves a
        command dispatches/replies correctly on a specific channel, which
        command() (always SERIAL) cannot.
        """
        buf = ctypes.create_string_buffer(_REPLY_BUF_SIZE)
        n = self._lib.sim_command_on(self._h, line.encode(), ctypes.c_int(channel),
                                      buf, _REPLY_BUF_SIZE)
        if n <= 0:
            return ""
        return buf.raw[:n].decode(errors="replace")

    def route_no_tick(self, line: str, channel: int = CHANNEL_SERIAL) -> str:
        """Like command_on(), but skips the trailing Rt::MainLoop::tick()
        replay (095-007, test-only -- sim_route_no_tick()). Lets a test peek
        bb.segmentIn/bb.replaceIn's raw just-ADMITTED Drive::Goal (via
        peek_segment_in()/peek_replace_in() below) BEFORE Drivetrain::tick()
        drains it into its own ring_."""
        buf = ctypes.create_string_buffer(_REPLY_BUF_SIZE)
        n = self._lib.sim_route_no_tick(self._h, line.encode(), ctypes.c_int(channel),
                                         buf, _REPLY_BUF_SIZE)
        if n <= 0:
            return ""
        return buf.raw[:n].decode(errors="replace")

    # (100-007, THE CUTOVER) Drive::Goal's own declared field order
    # (source/drive/drivetrain.h) -- retyped from Motion::Segment's 12-field
    # +stream shape.
    _GOAL_FIELDS = ("arc_length", "delta_heading", "exit_speed")

    # (100-008) Rt::MoverRequest's own declared field order (source/runtime/
    # commands.h) -- replaceIn's own retype away from Drive::Goal (MOVER is
    # the `replace` arm's exclusive meaning now -- see blackboard.h/
    # commands.h's doc comments). `v`/`omega` are MoverRequest.target's
    # v_x/omega components (v_y is always 0.0f, no holonomic drivetrain).
    _MOVER_FIELDS = ("v", "omega", "deadman")

    def peek_segment_in(self, idx: int = 0) -> dict | None:
        """Non-destructive read of bb.segmentIn[idx] (100-007, THE CUTOVER
        -- sim_peek_segment_in()). Returns a dict keyed by Drive::Goal's own
        field names (arc_length/delta_heading/exit_speed), or None if no
        Goal is queued at that position."""
        out3 = (ctypes.c_float * 3)()
        present_out = ctypes.c_int()
        self._lib.sim_peek_segment_in(self._h, ctypes.c_int(idx), out3,
                                       ctypes.byref(present_out))
        if not present_out.value:
            return None
        return dict(zip(self._GOAL_FIELDS, (float(v) for v in out3)))

    def peek_replace_in(self) -> dict | None:
        """Non-destructive read of bb.replaceIn (100-008 -- retyped to
        Rt::MoverRequest, sim_peek_replace_in()). Returns a dict keyed by
        MoverRequest's own field names (v/omega/deadman), or None if no
        MoverRequest is pending."""
        out3 = (ctypes.c_float * 3)()
        present_out = ctypes.c_int()
        self._lib.sim_peek_replace_in(self._h, out3, ctypes.byref(present_out))
        if not present_out.value:
            return None
        return dict(zip(self._MOVER_FIELDS, (float(v) for v in out3)))

    def chain_tail(self) -> dict:
        """bb.chainTail (100-007, THE CUTOVER, test-only -- sim_get_chain_tail()):
        the predicted world state at the end of everything currently
        admitted -- x/y/heading/exit_speed/kappa."""
        x, y, h, exit_speed, kappa = (ctypes.c_float() for _ in range(5))
        self._lib.sim_get_chain_tail(self._h, ctypes.byref(x), ctypes.byref(y),
                                      ctypes.byref(h), ctypes.byref(exit_speed),
                                      ctypes.byref(kappa))
        return {"x": x.value, "y": y.value, "h": h.value,
                "exit_speed": exit_speed.value, "kappa": kappa.value}

    def last_event(self) -> dict:
        """bb.lastEvent (100-007, THE CUTOVER, test-only --
        sim_get_last_event()): the most recent msg::EventNotify the adapter
        populated on an ABORT_* -- seg_seq/status (Drive::Status ordinal)/
        e_final_pos/e_final_theta."""
        seg_seq = ctypes.c_uint32()
        status = ctypes.c_int()
        e_final_pos, e_final_theta = ctypes.c_float(), ctypes.c_float()
        self._lib.sim_get_last_event(self._h, ctypes.byref(seg_seq), ctypes.byref(status),
                                      ctypes.byref(e_final_pos), ctypes.byref(e_final_theta))
        return {"seg_seq": seg_seq.value, "status": status.value,
                "e_final_pos": e_final_pos.value, "e_final_theta": e_final_theta.value}

    def reply_store_len(self, channel: int) -> int:
        """Read a channel's CURRENT reply-store length without draining or
        routing anything (088-006, test-only -- sim_get_reply_store_len()).

        Lets a test call command_on() on one channel and then confirm the
        OTHER channel's store is still empty, proving CommandRouter's two
        reply channels are backed by genuinely distinct ReplyStore
        instances rather than one shared sink.
        """
        return int(self._lib.sim_get_reply_store_len(self._h, ctypes.c_int(channel)))

    def peek_reply_store(self, channel: int) -> str:
        """Non-destructive read of a channel's CURRENT ReplyStore content
        (096-002, test-only -- sim_peek_reply_store()). Companion to
        reply_store_len(): lets a test drain the periodic TLM frames
        tickTelemetry() accumulates into a channel's sync store across a run
        of tick_for() calls -- command()/command_on() cannot be used for
        this since both reset (clear) every store before routing anything,
        which would wipe out whatever tickTelemetry() had already
        accumulated before a test got to read it.
        """
        buf = ctypes.create_string_buffer(_REPLY_BUF_SIZE)
        n = self._lib.sim_peek_reply_store(self._h, ctypes.c_int(channel), buf, _REPLY_BUF_SIZE)
        if n <= 0:
            return ""
        return buf.raw[:n].decode(errors="replace")

    def drain_reply_store(self, channel: int) -> str:
        """DESTRUCTIVE read of one channel's CURRENT ReplyStore content (097,
        test-only -- sim_drain_reply_store()): returns exactly what
        peek_reply_store() would, then resets (clears) THAT ONE channel's
        store. Companion to peek_reply_store(): a test collecting several
        tick_for() passes' worth of periodic frames (100-009: a MotionTrace +
        Telemetry pair per pass, once StreamControl.trace is armed) drains
        between reads so ReplyStore's fixed-size, no-wraparound buffer
        (sim_api.cpp's own ReplyStore struct) never silently overflows and
        freezes mid-run.
        """
        buf = ctypes.create_string_buffer(_REPLY_BUF_SIZE)
        n = self._lib.sim_drain_reply_store(self._h, ctypes.c_int(channel), buf, _REPLY_BUF_SIZE)
        if n <= 0:
            return ""
        return buf.raw[:n].decode(errors="replace")

    def post_segment(self, arc_length: float, delta_heading: float = 0.0,
                      exit_speed: float = 0.0) -> bool:
        """Post one Drive::Goal directly to bb.segmentIn (100-007, THE
        CUTOVER -- sim_post_segment()), BYPASSING wire admission entirely
        (no admit() check, no bb.chainTail advance -- use command()/
        command_on() with a real `segment`/`replace` CommandEnvelope to
        exercise admission). Angle arg (delta_heading) is RADIANS --
        Drive::Goal's own native unit (source/drive/drivetrain.h). Returns
        True if segmentIn accepted it (False only if segmentIn is already
        at its 8-slot cap)."""
        accepted = self._lib.sim_post_segment(
            self._h,
            ctypes.c_float(arc_length), ctypes.c_float(delta_heading),
            ctypes.c_float(exit_speed),
        )
        return bool(accepted)

    def get_async_evts(self) -> str:
        """Drain and return any loop-originated async EVT lines (e.g. the
        watchdog-fire ``EVT dev_watchdog``) accumulated since the last call."""
        buf = ctypes.create_string_buffer(_REPLY_BUF_SIZE)
        n = self._lib.sim_get_async_evts(self._h, buf, _REPLY_BUF_SIZE)
        if n <= 0:
            return ""
        return buf.raw[:n].decode(errors="replace")

    # ------------------------------------------------------------------
    # Ground-truth reads (Hal::PhysicsWorld's TRUE, unslipped/unerrored
    # accumulators -- sim_api.cpp's "Ground truth" ABI group).
    # ------------------------------------------------------------------

    def true_pose(self) -> tuple[float, float, float]:
        """Return (x, y, h) -- true chassis pose. [mm] [mm] [rad]"""
        lib, h = self._lib, self._h
        return (float(lib.sim_get_true_pose_x(h)),
                float(lib.sim_get_true_pose_y(h)),
                float(lib.sim_get_true_pose_h(h)))

    def exact_pose(self) -> tuple[float, float, float]:
        """Legacy alias for true_pose() -- sim_api.cpp's sim_get_exact_pose_*
        entry points, kept as a second read of the identical data."""
        lib, h = self._lib, self._h
        return (float(lib.sim_get_exact_pose_x(h)),
                float(lib.sim_get_exact_pose_y(h)),
                float(lib.sim_get_exact_pose_h(h)))

    def true_wheel_travel(self) -> tuple[float, float]:
        """Return (enc_l, enc_r) -- true (unslipped) per-wheel travel. [mm]"""
        lib, h = self._lib, self._h
        return (float(lib.sim_get_true_enc_l(h)), float(lib.sim_get_true_enc_r(h)))

    def true_velocity(self) -> tuple[float, float]:
        """Return (vel_l, vel_r) -- true per-wheel velocity. [mm/s]"""
        lib, h = self._lib, self._h
        return (float(lib.sim_get_true_vel_l(h)), float(lib.sim_get_true_vel_r(h)))

    def set_true_wheel_travel(self, enc_l: float, enc_r: float) -> None:  # [mm] [mm]
        self._lib.sim_set_true_wheel_travel(self._h, ctypes.c_float(enc_l), ctypes.c_float(enc_r))

    def set_true_pose(self, x: float, y: float, heading: float) -> None:  # [mm] [mm] [rad]
        self._lib.sim_set_true_pose(self._h, ctypes.c_float(x), ctypes.c_float(y),
                                     ctypes.c_float(heading))

    # ------------------------------------------------------------------
    # Errored-observation reads (sim_api.cpp's "Errored observation" ABI
    # group -- the reported/filtered values a real firmware consumer sees).
    # ------------------------------------------------------------------

    def enc(self) -> tuple[float, float]:
        """Return (enc_l, enc_r) -- REPORTED (errored) per-wheel travel. [mm]"""
        lib, h = self._lib, self._h
        return (float(lib.sim_get_enc_l(h)), float(lib.sim_get_enc_r(h)))

    def vel(self) -> tuple[float, float]:
        """Return (vel_l, vel_r) -- the two default plant-bound SimMotors'
        own filtered velocity() (port 1=LEFT, port 2=RIGHT). [mm/s]"""
        lib, h = self._lib, self._h
        return (float(lib.sim_get_vel_l(h)), float(lib.sim_get_vel_r(h)))

    def active(self) -> bool:
        """Return bb.drivetrain.busy directly (097-008, TEST-ONLY) --
        motion in progress, the SAME value Telemetry::tick() copies into
        TlmFrameInput.active/msg::Telemetry.active. A zero-cost peek,
        bypassing the telemetry wire entirely -- see sim_api.cpp's own
        sim_get_active() doc comment for why a per-iteration precision
        loop (test_pivot_completes_promptly_single_peaked) needs this
        instead of a binary `stream` read."""
        return bool(self._lib.sim_get_active(self._h))

    def pwm(self) -> tuple[float, float]:
        """Return (pwm_l, pwm_r) -- the plant's raw commanded actuator value,
        [-100, 100]."""
        lib, h = self._lib, self._h
        return (float(lib.sim_get_pwm_l(h)), float(lib.sim_get_pwm_r(h)))

    def otos_pose(self) -> tuple[float, float, float]:
        """Return (x, y, h) -- SimOdometer's accumulated OTOS pose. [mm] [mm] [rad]"""
        lib, h = self._lib, self._h
        return (float(lib.sim_get_otos_x(h)), float(lib.sim_get_otos_y(h)),
                float(lib.sim_get_otos_h(h)))

    def enc_pose(self) -> tuple[float, float, float]:
        """Return (x, y, h) -- bb.encoderPose.pose, a direct zero-cost peek
        (099-008, TEST-ONLY -- sim_get_enc_pose_x/y/h). PoseEstimator's pure
        dead-reckoning accumulator is never wire-visible (encpose= was
        trimmed from Telemetry, 096-001); this is the only way a test can
        prove a delayed camera-fix leaves it untouched. [mm] [mm] [rad]"""
        lib, h = self._lib, self._h
        return (float(lib.sim_get_enc_pose_x(h)), float(lib.sim_get_enc_pose_y(h)),
                float(lib.sim_get_enc_pose_h(h)))

    # ------------------------------------------------------------------
    # Error-knob setters (sim_api.cpp's 14 canonical call sites into
    # hal/sim/sim_setters.h). side: 0=left, 1=right, 2=both, matching every
    # setter's own C-side convention.
    # ------------------------------------------------------------------

    def set_enc_scale_error(self, side: int, err: float) -> None:
        self._lib.sim_set_enc_scale_error(self._h, ctypes.c_int(side), ctypes.c_float(err))

    def set_enc_slip(self, side: int, fraction: float) -> None:
        self._lib.sim_set_enc_slip(self._h, ctypes.c_int(side), ctypes.c_float(fraction))

    def set_enc_noise(self, side: int, sigma: float) -> None:  # [mm]
        self._lib.sim_set_enc_noise(self._h, ctypes.c_int(side), ctypes.c_float(sigma))

    def set_stiction(self, side: int, pwm: float) -> None:
        self._lib.sim_set_stiction(self._h, ctypes.c_int(side), ctypes.c_float(pwm))

    def set_motor_lag(self, side: int, tau: float) -> None:  # [ms]
        self._lib.sim_set_motor_lag(self._h, ctypes.c_int(side), ctypes.c_float(tau))

    def set_nominal_max_speed(self, speed: float) -> None:  # [mm/s]
        self._lib.sim_set_nominal_max_speed(self._h, ctypes.c_float(speed))

    def set_coulomb_friction(self, side: int, decel: float) -> None:  # [mm/s^2]
        self._lib.sim_set_coulomb_friction(self._h, ctypes.c_int(side), ctypes.c_float(decel))

    def set_trackwidth(self, trackwidth: float) -> None:  # [mm]
        self._lib.sim_set_trackwidth(self._h, ctypes.c_float(trackwidth))

    def set_body_rotational_scrub(self, scrub: float) -> None:
        self._lib.sim_set_body_rotational_scrub(self._h, ctypes.c_float(scrub))

    def set_body_linear_scrub(self, scrub: float) -> None:
        self._lib.sim_set_body_linear_scrub(self._h, ctypes.c_float(scrub))

    def set_otos_linear_noise(self, sigma: float) -> None:  # [mm]
        self._lib.sim_set_otos_linear_noise(self._h, ctypes.c_float(sigma))

    def set_otos_yaw_noise(self, sigma: float) -> None:  # [rad]
        self._lib.sim_set_otos_yaw_noise(self._h, ctypes.c_float(sigma))

    def set_otos_linear_scale_error(self, err: float) -> None:
        self._lib.sim_set_otos_linear_scale_error(self._h, ctypes.c_float(err))

    def set_otos_angular_scale_error(self, err: float) -> None:
        self._lib.sim_set_otos_angular_scale_error(self._h, ctypes.c_float(err))

    def set_otos_linear_drift(self, drift: float) -> None:  # [mm]
        self._lib.sim_set_otos_linear_drift(self._h, ctypes.c_float(drift))

    def set_otos_yaw_drift(self, drift: float) -> None:  # [rad]
        self._lib.sim_set_otos_yaw_drift(self._h, ctypes.c_float(drift))

    # ------------------------------------------------------------------
    # Internal: argtypes / restype declarations for every sim_* symbol.
    # ------------------------------------------------------------------

    def _setup_types(self) -> None:
        lib = self._lib

        lib.sim_create.argtypes = []
        lib.sim_create.restype = ctypes.c_void_p

        lib.sim_destroy.argtypes = [ctypes.c_void_p]
        lib.sim_destroy.restype = None

        lib.sim_tick.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        lib.sim_tick.restype = None

        lib.sim_command.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
        lib.sim_command.restype = ctypes.c_int

        # sim_command_on (088-006) -- sim_command's argtypes plus one c_int
        # channel selector (CHANNEL_SERIAL/CHANNEL_RADIO, above).
        lib.sim_command_on.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
        lib.sim_command_on.restype = ctypes.c_int

        # sim_get_reply_store_len (088-006, test-only) -- non-draining peek
        # at one channel's ReplyStore length.
        lib.sim_get_reply_store_len.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.sim_get_reply_store_len.restype = ctypes.c_int

        # sim_peek_reply_store (096-002, test-only) -- non-draining read of
        # one channel's ReplyStore CONTENT (companion to
        # sim_get_reply_store_len() above).
        lib.sim_peek_reply_store.argtypes = [
            ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
        lib.sim_peek_reply_store.restype = ctypes.c_int

        # sim_drain_reply_store (097, test-only) -- sim_peek_reply_store's
        # argtypes exactly (same signature; DESTRUCTIVE instead of peek).
        lib.sim_drain_reply_store.argtypes = [
            ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
        lib.sim_drain_reply_store.restype = ctypes.c_int

        # sim_route_no_tick (095-007, test-only) -- sim_command_on()'s
        # argtypes exactly (same signature, different behavior).
        lib.sim_route_no_tick.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
        lib.sim_route_no_tick.restype = ctypes.c_int

        # sim_peek_segment_in / sim_peek_replace_in (100-007, THE CUTOVER) --
        # non-destructive Drive::Goal reads.
        lib.sim_peek_segment_in.argtypes = [
            ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_int)]
        lib.sim_peek_segment_in.restype = None

        lib.sim_peek_replace_in.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_int)]
        lib.sim_peek_replace_in.restype = None

        lib.sim_get_async_evts.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        lib.sim_get_async_evts.restype = ctypes.c_int

        # sim_post_segment (100-007, THE CUTOVER) -- direct bb.segmentIn
        # Drive::Goal producer.
        lib.sim_post_segment.argtypes = [ctypes.c_void_p] + [ctypes.c_float] * 3
        lib.sim_post_segment.restype = ctypes.c_int

        # sim_get_chain_tail / sim_get_last_event (100-007, THE CUTOVER,
        # test-only) -- bb.chainTail/bb.lastEvent peeks.
        lib.sim_get_chain_tail.argtypes = [ctypes.c_void_p] + [ctypes.POINTER(ctypes.c_float)] * 5
        lib.sim_get_chain_tail.restype = None

        lib.sim_get_last_event.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
        lib.sim_get_last_event.restype = None

        # Ground truth (12) -- all no-arg float getters, plus two setters.
        for name in (
            "sim_get_true_pose_x", "sim_get_true_pose_y", "sim_get_true_pose_h",
            "sim_get_exact_pose_x", "sim_get_exact_pose_y", "sim_get_exact_pose_h",
            "sim_get_true_enc_l", "sim_get_true_enc_r",
            "sim_get_true_vel_l", "sim_get_true_vel_r",
        ):
            fn = getattr(lib, name)
            fn.argtypes = [ctypes.c_void_p]
            fn.restype = ctypes.c_float

        lib.sim_set_true_wheel_travel.argtypes = [
            ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
        lib.sim_set_true_wheel_travel.restype = None

        lib.sim_set_true_pose.argtypes = [
            ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        lib.sim_set_true_pose.restype = None

        # Errored observation (9) -- all no-arg float getters.
        # sim_get_enc_pose_x/y/h (099-008, TEST-ONLY) share this same
        # no-arg-float-getter shape -- see their own sim_api.cpp doc comment.
        for name in (
            "sim_get_enc_l", "sim_get_enc_r",
            "sim_get_vel_l", "sim_get_vel_r",
            "sim_get_pwm_l", "sim_get_pwm_r",
            "sim_get_otos_x", "sim_get_otos_y", "sim_get_otos_h",
            "sim_get_enc_pose_x", "sim_get_enc_pose_y", "sim_get_enc_pose_h",
        ):
            fn = getattr(lib, name)
            fn.argtypes = [ctypes.c_void_p]
            fn.restype = ctypes.c_float

        # sim_get_active (097-008, TEST-ONLY) -- bb.drivetrain.busy direct peek.
        lib.sim_get_active.argtypes = [ctypes.c_void_p]
        lib.sim_get_active.restype = ctypes.c_int

        # Error-knob setters (14).
        for name in (
            "sim_set_enc_scale_error", "sim_set_enc_slip", "sim_set_enc_noise",
            "sim_set_stiction", "sim_set_motor_lag", "sim_set_coulomb_friction",
        ):
            fn = getattr(lib, name)
            fn.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_float]
            fn.restype = None

        lib.sim_set_trackwidth.argtypes = [ctypes.c_void_p, ctypes.c_float]
        lib.sim_set_trackwidth.restype = None

        lib.sim_set_nominal_max_speed.argtypes = [ctypes.c_void_p, ctypes.c_float]
        lib.sim_set_nominal_max_speed.restype = None

        for name in ("sim_set_body_rotational_scrub", "sim_set_body_linear_scrub"):
            fn = getattr(lib, name)
            fn.argtypes = [ctypes.c_void_p, ctypes.c_float]
            fn.restype = None

        for name in (
            "sim_set_otos_linear_noise", "sim_set_otos_yaw_noise",
            "sim_set_otos_linear_scale_error", "sim_set_otos_angular_scale_error",
            "sim_set_otos_linear_drift", "sim_set_otos_yaw_drift",
        ):
            fn = getattr(lib, name)
            fn.argtypes = [ctypes.c_void_p, ctypes.c_float]
            fn.restype = None
