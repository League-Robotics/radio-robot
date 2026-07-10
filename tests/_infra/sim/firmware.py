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
        bb.segmentIn/bb.replaceIn's raw just-posted Motion::Segment (via
        peek_segment_in()/peek_replace_in() below) BEFORE Drivetrain::tick()
        drains it into its own ring_/executor_."""
        buf = ctypes.create_string_buffer(_REPLY_BUF_SIZE)
        n = self._lib.sim_route_no_tick(self._h, line.encode(), ctypes.c_int(channel),
                                         buf, _REPLY_BUF_SIZE)
        if n <= 0:
            return ""
        return buf.raw[:n].decode(errors="replace")

    _SEGMENT_FIELDS = (
        "distance", "direction", "final_heading", "speed_max", "accel_max", "jerk_max",
        "yaw_rate_max", "yaw_accel_max", "yaw_jerk_max", "time", "v", "omega",
    )

    def peek_segment_in(self, idx: int = 0) -> dict | None:
        """Non-destructive read of bb.segmentIn[idx] (095-007, test-only --
        sim_peek_segment_in()). Returns a dict keyed by Motion::Segment's
        own field names (Motion::Segment's OWN spelling, e.g. `final_heading`
        not `finalHeading`, to match the wire message's field names 1:1 for
        a direct field-by-field translation check) plus `stream`, or None if
        no segment is queued at that position."""
        out12 = (ctypes.c_float * 12)()
        stream_out = ctypes.c_int()
        present_out = ctypes.c_int()
        self._lib.sim_peek_segment_in(self._h, ctypes.c_int(idx), out12,
                                       ctypes.byref(stream_out), ctypes.byref(present_out))
        if not present_out.value:
            return None
        result = dict(zip(self._SEGMENT_FIELDS, (float(v) for v in out12)))
        result["stream"] = bool(stream_out.value)
        return result

    def peek_replace_in(self) -> dict | None:
        """Non-destructive read of bb.replaceIn (095-007, test-only --
        sim_peek_replace_in()). Same shape as peek_segment_in()."""
        out12 = (ctypes.c_float * 12)()
        stream_out = ctypes.c_int()
        present_out = ctypes.c_int()
        self._lib.sim_peek_replace_in(self._h, out12,
                                       ctypes.byref(stream_out), ctypes.byref(present_out))
        if not present_out.value:
            return None
        result = dict(zip(self._SEGMENT_FIELDS, (float(v) for v in out12)))
        result["stream"] = bool(stream_out.value)
        return result

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

    def post_segment(self, distance: float, direction: float, final_heading: float,
                      speed_max: float = 0.0, accel_max: float = 0.0, jerk_max: float = 0.0,
                      yaw_rate_max: float = 0.0, yaw_accel_max: float = 0.0,
                      yaw_jerk_max: float = 0.0) -> bool:
        """Post one Motion::Segment directly to bb.segmentIn (094-005,
        test-only -- sim_post_segment()), bypassing the wire entirely ahead
        of 094-006's `MOVE` verb. Angle args (direction/final_heading) are
        RADIANS -- Motion::Segment's own native unit (segment.h), not the
        wire's eventual centidegrees. Returns True if segmentIn accepted it
        (False only if segmentIn is already at its 8-slot cap)."""
        accepted = self._lib.sim_post_segment(
            self._h,
            ctypes.c_float(distance), ctypes.c_float(direction), ctypes.c_float(final_heading),
            ctypes.c_float(speed_max), ctypes.c_float(accel_max), ctypes.c_float(jerk_max),
            ctypes.c_float(yaw_rate_max), ctypes.c_float(yaw_accel_max), ctypes.c_float(yaw_jerk_max),
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

        # sim_route_no_tick (095-007, test-only) -- sim_command_on()'s
        # argtypes exactly (same signature, different behavior).
        lib.sim_route_no_tick.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
        lib.sim_route_no_tick.restype = ctypes.c_int

        # sim_peek_segment_in / sim_peek_replace_in (095-007, test-only) --
        # non-destructive Motion::Segment reads.
        lib.sim_peek_segment_in.argtypes = [
            ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
        lib.sim_peek_segment_in.restype = None

        lib.sim_peek_replace_in.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
        lib.sim_peek_replace_in.restype = None

        lib.sim_get_async_evts.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        lib.sim_get_async_evts.restype = ctypes.c_int

        # sim_post_segment (094-005, test-only) -- direct bb.segmentIn producer.
        lib.sim_post_segment.argtypes = [ctypes.c_void_p] + [ctypes.c_float] * 9
        lib.sim_post_segment.restype = ctypes.c_int

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
        for name in (
            "sim_get_enc_l", "sim_get_enc_r",
            "sim_get_vel_l", "sim_get_vel_r",
            "sim_get_pwm_l", "sim_get_pwm_r",
            "sim_get_otos_x", "sim_get_otos_y", "sim_get_otos_h",
        ):
            fn = getattr(lib, name)
            fn.argtypes = [ctypes.c_void_p]
            fn.restype = ctypes.c_float

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
