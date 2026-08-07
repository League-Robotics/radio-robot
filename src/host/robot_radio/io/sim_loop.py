"""robot_radio.io.sim_loop -- SimLoop: a TwistTransport-shaped Python object
over ticket 108-005's ``sim_ctypes.cpp`` C ABI.

Sprint 108 ticket 006 (Stage 3 part b of
``clasi/issues/plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md``).
Deletes and replaces the dead ``robot_radio.io.sim_conn.SimConnection`` --
that module bound a ~40-symbol ABI (``Hal::PhysicsWorld``/``Hal::SimOdometer``)
from a subsystem graph deleted by the same greenfield rebuild that
introduced this sprint's own ``TestSim::SimHarness``/``TestSim::SimPlant``
composition (``src/sim/sim_harness.h``/``sim_plant.h``);
``SimConnection.connect()`` has unconditionally returned "Sim library not
found" since commit ``72d8be7e`` (the library it targeted was never
rebuilt against the current tree). This module targets the NEW, real
19-symbol ABI (``src/sim/sim_ctypes.cpp``) instead -- see that
file's own header comment for the full export list and hook contract this
module binds.

Two very different shapes, on purpose
--------------------------------------
``SimConnection`` was a *drop-in ``SerialConnection`` substitute*: it
answered ``send()``/``send_envelope()``/``connect()`` the same way a real
serial port did, so ``NezhaProtocol`` could wrap it transparently and the
WHOLE binary command/config channel (twist/stop/config, SET/GET, ...) was
simulated. The current firmware graph's sim harness does not simulate a
generic wire channel at all -- ``TestSim::SimHarness`` exposes exactly
three surfaces: command injection (``injectTwist``/``injectStop``/raw
``injectCommand``), telemetry drain (``drainRawTelemetry``), and fault
knobs/hooks on the composed ``SimPlant``. There is no ``config`` or
``set_config_binary()`` simulation surface to wrap -- so ``SimLoop`` does
NOT attempt to be a ``NezhaProtocol``-compatible ``SerialConnection``
substitute. Instead it implements ``planner/executor.py``'s
``TwistTransport`` structural protocol DIRECTLY (``twist()``/``stop()``/
``read_pending_binary_tlm_frames()`` -- verified against that module's own
``TwistTransport`` class, not assumed) -- the exact, already-proven
consumer interface ``planner/tour.py``'s ``run_tour()`` drives a real
``NezhaProtocol`` through today (see ``testgui/transport.py``'s
``_HardwareTransport.protocol`` property). A ``SimLoop`` instance can be
handed anywhere a ``NezhaProtocol`` currently is, with no adapter.

Threading model
----------------
Mirrors ``testgui/transport.py``'s ``SimTransport._tick_loop`` pattern: the
raw ``ctypes`` handle (a ``TestSim::SimHarness*``) is NOT thread-safe for
concurrent access, so exactly ONE thread ever touches it. ``connect()``
starts a background wall-clock tick thread (unless
``start_tick_thread=False``, for callers -- e.g. ticket 009's synchronous
register-level tests -- that want single-threaded, fully deterministic
``step()``/hook calls with no background thread in play at all). While the
tick thread is running:

  - ``twist()``/``stop()``/``inject_command()`` enqueue a fire-and-forget
    action the tick thread executes on its next iteration, returning
    immediately with an assigned ``corr_id`` -- the SAME fire-and-poll
    contract ``NezhaProtocol.twist()``/``.stop()`` document (this module's
    own outcome, if any, rides the next drained telemetry frame's ack
    ring, exactly like the real wire).
  - Fault-condition setters, ``get_true_pose()``, and hook registration
    (``set_read_hook()``/``set_write_hook()``, including the
    ``read_hook()``/``write_hook()`` context managers' clear-on-exit) are
    round-tripped SYNCHRONOUSLY onto the tick thread
    (``_call_on_tick_thread()``) -- unlike twist/stop, a caller reading
    these values needs them to reflect a specific, already-applied state,
    not "eventually applied", and hook registration additionally must
    never reassign ``SimPlant::readHook_``/``writeHook_`` concurrently with
    the tick thread's own ``sim_step()`` mid-invocation of the previous
    hook (the raw ctypes handle is NOT thread-safe -- see this section's
    opening sentence -- so registration is no exception to it).
  - Every tick, the tick thread drains ``sim_drain_tlm()`` into
    ``TLMFrame`` objects (dearmoring/parsing the raw ``*B<base64>`` wire
    text with the exact same ``pb2`` codec a real robot's replies go
    through -- see ``robot_radio.robot.protocol.TLMFrame.from_pb2()``),
    pushes them onto a bounded internal queue ``read_pending_binary_tlm_
    frames()`` drains, and -- unless ``suspend_telemetry_reader()`` is in
    effect -- also delivers each one to ``on_telemetry`` immediately.

If ``start_tick_thread=False``, every one of the above happens
synchronously on the CALLING thread instead -- there is no queue, no
corr_id fire-and-forget delay, and the caller owns pacing (calling
``step(cycles)`` explicitly). This is the shape a register-level hook test
wants: inject a twist, ``step()`` exactly N cycles, inspect exactly what
the hook observed, deterministically.

``suspend_telemetry_reader()``/``resume_telemetry_reader()``
--------------------------------------------------------------
Mirrors ``_HardwareTransport.suspend_telemetry_reader()``'s own rationale
(``testgui/transport.py``): a caller that becomes the sole consumer of
telemetry for a bounded window (e.g. a tour driving ``run_tour()`` directly
against this object) calls ``suspend_telemetry_reader()`` first so
``on_telemetry`` stops firing a SECOND, competing consumer of the same
frames during that window, then ``resume_telemetry_reader()`` in a
``finally``. Unlike the hardware transport, there is no possibility of
STARVING the other consumer here -- the internal queue
``read_pending_binary_tlm_frames()`` drains is filled unconditionally every
tick regardless of suspension -- this toggle exists purely to stop a
second delivery path (``on_telemetry``, e.g. a GUI canvas/log pane) from
double-rendering the same frames a tour is already narrating itself.

Hook wrappers
--------------
``set_read_hook(cb)``/``set_write_hook(cb)`` wrap ``sim_ctypes.cpp``'s raw
``ctypes.CFUNCTYPE`` hook registration (``sim_set_read_hook``/
``sim_set_write_hook``) with a friendlier Python surface: ``cb(addr, buf)``
receives the wire address and a mutable ``ctypes`` array view (a read hook
fills it and returns 1/HANDLED; a write hook reads it and returns
0/PASS or 1/HANDLED). ``pass_through(addr, buf, length, write)`` calls
``sim_default_read()``/``sim_default_write()`` -- the un-hooked default
handler -- so a hook that wants "observe, then behave normally" can call it
and return its result. The ``ctypes.CFUNCTYPE`` object built for each
registered callback is kept alive on ``self`` (``_read_hook_c``/
``_write_hook_c``) for as long as it is registered -- ``ctypes`` holds no
reference of its own, and a garbage-collected trampoline crashes the
process the next time the firmware touches that wire address.
``read_hook()``/``write_hook()`` context managers register on ``__enter__``
and clear (``cb=None``) on ``__exit__``, for a caller that wants scoped
registration without a manual try/finally. The actual registration/clear
call is round-tripped onto the tick thread -- see "Threading model" above.
"""

from __future__ import annotations

import ctypes
import logging
import pathlib
import queue
import sys
import threading
import time
from contextlib import contextmanager
from typing import TYPE_CHECKING, Any, Callable, Iterator

from robot_radio.io.wire_codec import decode_frame, encode_frame

if TYPE_CHECKING:
    from robot_radio.config.robot_config import RobotConfig
    from robot_radio.robot.pb2 import envelope_pb2
    from robot_radio.robot.protocol import TLMFrame

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Lib path resolution -- same convention the deleted predecessor used
# (io/ -> ../../../../src/sim/build), and the same one
# testgui/transport.py's own _sim_lib_path() independently resolves.
# ---------------------------------------------------------------------------

_LIB_NAME = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
_HERE = pathlib.Path(__file__).parent
_DEFAULT_LIB_PATH = (_HERE / "../../../../src/sim/build" / _LIB_NAME).resolve()

# One sim cycle == 32ms of sim/firmware time (TestSim::SimHarness::kCycleDtUs,
# sim_harness.h -- itself derived from firmware's own App::RobotLoop::kCycle,
# robot_loop.h; 118 ticket 003, sim-cycle-must-match-firmware-period.md;
# 130-007 raised kCycle 40ms -> 50ms, one control period everywhere; lowered
# again 50ms -> 32ms on 2026-08-07 against the loop's measured work floor).
# Real-time (1x) tick-thread pacing advances one cycle per wall-clock tick by
# default. This module constant is only the FALLBACK default (used before a
# lib is ever loaded, e.g. as SimLoop.__init__'s initial value) -- connect()
# overwrites `self._cycle_duration_s` with the value actually compiled into
# the LOADED library (`sim_cycle_dt_us()`) so a caller pointed at a stale or
# differently-built .dylib/.so still paces against what that binary really
# runs, not what this source tree currently says.
_CYCLE_DURATION_S = 0.032  # [s]

# Telemetry queue: bounded, drop-oldest -- mirrors SerialConnection's own
# _binary_tlm_queue policy (never let an un-drained queue grow unbounded).
_TLM_QUEUE_MAXSIZE = 512

# Ground-truth pose delivered every Nth tick -- ~6.25 Hz at 1x speed and the
# 40ms cycle period (118 ticket 003; was ~5 Hz at the pre-118 50ms cycle),
# in the same ballpark as testgui/transport.py's own
# _SIM_TRUTH_EVERY_N_TICKS/hardware truth-poll rate (that module computes
# its own N dynamically off ITS tick duration to target ~5 Hz exactly; this
# constant is not recomputed to match -- a UI truth-delivery rate, not a
# gated cadence assumption).
_TRUTH_EVERY_N_TICKS = 4

# set_speed_factor() clamp range -- matches testgui/transport.py's own
# _SIM_SPEED_MIN/_SIM_SPEED_MAX (1x..20x fast-forward).
_SPEED_FACTOR_MIN = 1
_SPEED_FACTOR_MAX = 20

# Motor-state-aware tick cadence (OOP sim-motor-state fix). While the plant
# reports ``active`` (TLMFrame.active -- bb.drivetrain.busy) TRUE, the tick
# thread runs at the usual full real-time rate (_CYCLE_DURATION_S per cycle,
# scaled by _speed_factor). Once ``active`` goes FALSE (a motion finished),
# the thread drops to a slow HEARTBEAT: step + drain/deliver one frame every
# _IDLE_HEARTBEAT_INTERVAL_S instead of every _CYCLE_DURATION_S, so an idle
# connection stops flooding the UI/consumers with full-rate telemetry the way
# a real, idle robot would not. _IDLE_POLL_INTERVAL_S is the cmd-queue poll
# granularity used WHILE waiting out a heartbeat interval -- draining the cmd
# queue at this fine grain (not the coarse heartbeat interval) is what makes
# an incoming twist/stop feel immediate rather than laggy by up to 2s. See
# _tick_loop()'s own docstring for the full state machine.
_IDLE_HEARTBEAT_INTERVAL_S = 2.0  # [s] idle step+telemetry cadence
_IDLE_POLL_INTERVAL_S = 0.05      # [s] cmd-queue poll granularity while idle
# Grace window: after the last command OR the last `active=True` frame, keep
# stepping at FULL rate for this long before dropping to the idle heartbeat.
# Two reasons this must be > a few cycles: (1) the plant takes a cycle or two
# after a twist is injected to actually report `active=True` back — throttling
# on the instantaneous `active is False` reading deadlocks (never steps, so
# never observes the plant wake up); (2) a tour's inter-leg settle (~1.0s, no
# twists sent) must keep simulating deceleration at full rate, not freeze.
_IDLE_GRACE_S = 1.5  # [s]

# Generous scratch buffer for sim_drain_tlm()'s snprintf-style fill --
# "a handful of KB comfortably covers a burst of frames from one step()
# call" (sim_ctypes.cpp's own doc comment). Retried once, sized exactly, if
# a single drain call ever needs more.
_TLM_DRAIN_BUFFER = 16384

# Same idea as _TLM_DRAIN_BUFFER, sized down: DBG lines (129-003) are rare
# (one App::debugf() call per diagnostic, not a per-cycle telemetry push)
# and individually short (app/debug.cpp's own kDebugMsgMaxBytes=200 bound),
# so a much smaller scratch buffer comfortably covers a burst. Retried once,
# sized exactly, if a single drain call ever needs more (same convention as
# _TLM_DRAIN_BUFFER above).
_DEBUG_DRAIN_BUFFER = 4096

# Bounded debug-line queue: drop-oldest on overflow, mirroring
# SerialConnection's own _text_queue policy for the same reason -- a DBG
# line is a diagnostic, not a value a caller can afford to block forever
# waiting to read.
_DEBUG_QUEUE_MAXSIZE = 256

_SimHookFn = ctypes.CFUNCTYPE(
    ctypes.c_int, ctypes.c_void_p, ctypes.c_uint16,
    ctypes.POINTER(ctypes.c_uint8), ctypes.c_int)

# Lazily-imported/cached pb2 module -- see sim_ctypes.cpp's own header and
# the deleted predecessor's _get_envelope_pb2() docstring: no circular-
# import hazard for this module specifically, but deferring keeps a bare
# `import robot_radio.io.sim_loop` a lightweight, ctypes-only operation for
# a caller (e.g. a register-level hook test) that never touches telemetry
# decoding at all.
_envelope_pb2_module = None


def _get_envelope_pb2():
    global _envelope_pb2_module
    if _envelope_pb2_module is None:
        from robot_radio.robot.pb2 import envelope_pb2 as _mod
        _envelope_pb2_module = _mod
    return _envelope_pb2_module


def _decode_reply_frame(line: bytes, pb2_mod) -> "envelope_pb2.ReplyEnvelope | None":
    """Split ``line``'s own ``<COMMAND>':'`` prefix off, COBS+CRC-decode the
    rest (124-005 -- 123-002/003 had no prefix to strip; before that, a
    ``*B<base64>`` armored text line), and parse it as a
    ``pb2.ReplyEnvelope``. Returns ``None`` on any malformed/corrupt input
    (no ``':'`` at all, bad COBS, a CRC mismatch -- scoped over the parsed
    command, matching how ``Comms::sendReply()``/``Telemetry::
    emitSecondary()`` actually framed it -- or bad protobuf bytes),
    mirroring ``SerialConnection._handle_binary_reply()``'s own tolerance
    for a single corrupted binary reply -- never raises."""
    command, sep, frame = line.partition(b":")
    if not sep:
        return None
    payload = decode_frame(frame, command=command)
    if payload is None:
        return None
    try:
        return pb2_mod.ReplyEnvelope.FromString(payload)
    except Exception:
        return None


def _bind_ctypes(lib: ctypes.CDLL) -> None:
    """Set argtypes/restypes for every one of sim_ctypes.cpp's 19 exports.

    A thin, exhaustive transcription of sim_ctypes.cpp's own header comment
    -- no logic, just the C signature -> ctypes shape mapping ctypes needs
    to marshal arguments correctly (without this, ctypes assumes every
    argument/return value is a plain ``int``, which silently corrupts every
    float/pointer call on 64-bit platforms).
    """
    lib.sim_create.argtypes = [ctypes.c_float]
    lib.sim_create.restype = ctypes.c_void_p

    lib.sim_destroy.argtypes = [ctypes.c_void_p]
    lib.sim_destroy.restype = None

    lib.sim_booted.argtypes = [ctypes.c_void_p]
    lib.sim_booted.restype = ctypes.c_int

    lib.sim_cycle_count.argtypes = [ctypes.c_void_p]
    lib.sim_cycle_count.restype = ctypes.c_int

    lib.sim_step.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.sim_step.restype = None

    lib.sim_inject_twist.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint32]
    lib.sim_inject_twist.restype = None

    lib.sim_inject_stop.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.sim_inject_stop.restype = None
    lib.sim_inject_wheels.argtypes = [ctypes.c_void_p, ctypes.c_float,
                                      ctypes.c_float, ctypes.c_float,
                                      ctypes.c_uint32]
    lib.sim_inject_wheels.restype = None

    # `frame` is a `<COMMAND>':'<COBS+CRC bytes>` wire LINE (124-005; was a
    # bare COBS+CRC command body 123-002-124-004, an already-armored
    # `*B<base64>` line pre-123) -- build the COBS half with
    # `robot_radio.io.wire_codec.encode_frame()`, PREFIXED with the ASCII
    # command name and ':'. `len` is an EXPLICIT length, passed separately
    # (`inject_command()` below) rather than relying on `c_char_p`'s own
    # NUL-termination: COBS is keyed on 0x0A now, not 0x00 (wire_runtime.h
    # item 8), so the line may legitimately contain an embedded 0x00 byte
    # that a strlen()-based recovery would truncate at -- exactly the
    # pre-124-005 assumption this signature replaces (see
    # `SimHarness::injectCommand()`, src/sim/sim_harness.h, for the C++-side
    # fix to the same trap).
    lib.sim_inject_command.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.sim_inject_command.restype = None

    # 123-002/003/124-005: `buf` now receives raw, memcpy'd bytes -- every
    # captured outbound wire LINE this drain call, each followed by exactly
    # one '\n' (0x0A) delimiter (sim_ctypes.cpp's own doc comment) -- NOT a
    # snprintf("%s")-style NUL-terminated string. Read exactly the returned
    # byte count (or `buflen`, whichever is smaller) via the buffer's `.raw`
    # attribute, never `.value` (which would stop at the FIRST embedded
    # 0x00 -- i.e. after the first line only, and 124-005 lines may
    # legitimately contain one).
    lib.sim_drain_tlm.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.sim_drain_tlm.restype = ctypes.c_int

    # sim_drain_debug/sim_test_emit_debug (129-003, bench/Sim-only DBG
    # channel): sim_drain_debug() shares sim_drain_tlm()'s exact
    # snprintf()-style return-value/truncation contract (sim_ctypes.cpp's
    # own doc comment) but the captured bytes are already plain cleartext
    # "DBG:<message>\n" lines, never COBS/CRC-framed -- read via `.raw`
    # nonetheless (not `.value`) purely for symmetry with the tlm drain
    # above; a DBG line built from debugf()'s own kDebugMsgMaxBytes-bounded
    # vsnprintf() output cannot itself contain an embedded NUL, but there
    # is no reason to rely on that when `.raw[:n]` is exactly as cheap.
    lib.sim_drain_debug.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.sim_drain_debug.restype = ctypes.c_int
    lib.sim_test_emit_debug.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.sim_test_emit_debug.restype = None

    lib.sim_true_x.argtypes = [ctypes.c_void_p]
    lib.sim_true_x.restype = ctypes.c_float
    lib.sim_true_y.argtypes = [ctypes.c_void_p]
    lib.sim_true_y.restype = ctypes.c_float
    lib.sim_true_h.argtypes = [ctypes.c_void_p]
    lib.sim_true_h.restype = ctypes.c_float

    lib.sim_set_true_pose.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
    lib.sim_set_true_pose.restype = None

    lib.sim_set_wheel_disconnected.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    lib.sim_set_wheel_disconnected.restype = None
    lib.sim_set_wheel_freeze.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    lib.sim_set_wheel_freeze.restype = None
    lib.sim_set_wheel_dropout_rate.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_float]
    lib.sim_set_wheel_dropout_rate.restype = None
    lib.sim_set_otos_drift.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
    lib.sim_set_otos_drift.restype = None
    lib.sim_set_enc_scale_err.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_float]
    lib.sim_set_enc_scale_err.restype = None
    lib.sim_set_otos_raw_scale_err.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
    lib.sim_set_otos_raw_scale_err.restype = None
    lib.sim_set_enc_tick_quant.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_float]
    lib.sim_set_enc_tick_quant.restype = None
    lib.sim_set_enc_slip.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.c_float, ctypes.c_float]
    lib.sim_set_enc_slip.restype = None

    # Tier-2 config-load surface (113-002/113-005): SimHarness::
    # configureMotor()'s one-shot runtime load, for per-motor vel_filt/
    # fwd_sign with no live Tier-1 wire arm -- see sim_ctypes.cpp's own
    # header comment (Tier-2 config-load surface section) for the full
    # field list/order and SimLoop.configure_from_robot()'s own docstring
    # for how this is called. `sim_configure_planner()`/
    # `sim_read_planner_config()` -- DELETED (115-003, gut S1 motion-stack
    # excision): `msg::PlannerConfig` and its `SimHarness::configurePlanner()`
    # one-shot loader went with `Motion::Executor`/`App::Pilot`.
    lib.sim_configure_motor.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.c_float, ctypes.c_int]
    lib.sim_configure_motor.restype = None

    lib.sim_read_motor_config.argtypes = [
        ctypes.c_void_p, ctypes.c_int,
        ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_int)]
    lib.sim_read_motor_config.restype = None

    # 125-007: Tier-2 boot-only turn-calibration load -- App::RobotLoop::
    # setRotationCalibration() passthrough, offsets in RADIANS (see
    # sim_ctypes.cpp's own doc comment on sim_configure_drivetrain()).
    lib.sim_configure_drivetrain.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
    lib.sim_configure_drivetrain.restype = None

    # Tier-2 boot-only App::Drive calibration -- setDutyPerSpeed()/
    # setCrawlPulse() passthrough (see sim_ctypes.cpp's own doc comment on
    # sim_configure_drive(), and drive_boot_config_for() for why the wheel
    # correction main.cpp also installs is not mirrored into the sim).
    lib.sim_configure_drive.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
    lib.sim_configure_drive.restype = None

    lib.sim_set_read_hook.argtypes = [ctypes.c_void_p, _SimHookFn, ctypes.c_void_p]
    lib.sim_set_read_hook.restype = None
    lib.sim_set_write_hook.argtypes = [ctypes.c_void_p, _SimHookFn, ctypes.c_void_p]
    lib.sim_set_write_hook.restype = None

    lib.sim_default_read.argtypes = [
        ctypes.c_void_p, ctypes.c_uint16, ctypes.POINTER(ctypes.c_uint8), ctypes.c_int]
    lib.sim_default_read.restype = ctypes.c_int
    lib.sim_default_write.argtypes = [
        ctypes.c_void_p, ctypes.c_uint16, ctypes.POINTER(ctypes.c_uint8), ctypes.c_int]
    lib.sim_default_write.restype = ctypes.c_int

    lib.sim_firmware_version.argtypes = []
    lib.sim_firmware_version.restype = ctypes.c_char_p

    # sim_cycle_dt_us -- 118 ticket 003: the loaded library's own compiled-in
    # cycle period, [us] -- see connect()'s own use of this (sets
    # self._cycle_duration_s from it rather than trusting the module-level
    # _CYCLE_DURATION_S fallback to still match a possibly-stale-built lib).
    lib.sim_cycle_dt_us.argtypes = []
    lib.sim_cycle_dt_us.restype = ctypes.c_int

    # Commanded per-wheel velocity (velocity-PID setpoint) read live from the
    # firmware -- Path B for the commanded-vs-actual graph (cmd_vel is not on
    # the primary wire frame; see sim_ctypes.cpp's own note).
    lib.sim_cmd_vel_left.argtypes = [ctypes.c_void_p]
    lib.sim_cmd_vel_left.restype = ctypes.c_float
    lib.sim_cmd_vel_right.argtypes = [ctypes.c_void_p]
    lib.sim_cmd_vel_right.restype = ctypes.c_float

    # Velocity-PID enable/disable on both live NezhaMotors (TestGUI "PID"
    # checkbox) -- same direct-firmware-object surface as sim_cmd_vel_*.
    lib.sim_set_pid_enabled.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.sim_set_pid_enabled.restype = None


HookCallback = Callable[[int, "ctypes.Array[ctypes.c_uint8]"], int]


class SimLoop:
    """TwistTransport-shaped Python object over sim_ctypes.cpp's C ABI.

    See this module's own docstring for the threading model and the
    reconciliation from the deleted ``SimConnection``. Satisfies
    ``planner.executor.TwistTransport`` (``twist()``/``stop()``/
    ``read_pending_binary_tlm_frames()``) directly -- a real instance needs
    no adapter anywhere ``planner/tour.py``'s ``run_tour()`` accepts a
    transport.
    """

    def __init__(self, track_width: float = 0.0,
                 lib_path: "str | pathlib.Path | None" = None) -> None:
        self._track_width = track_width
        self._lib_path = pathlib.Path(lib_path) if lib_path else _DEFAULT_LIB_PATH
        self._lib: ctypes.CDLL | None = None
        # Real-time tick-thread pacing target -- module-level _CYCLE_DURATION_S
        # until connect() overwrites it with the LOADED library's own compiled-in
        # value (sim_cycle_dt_us(), 118 ticket 003 -- see that constant's own
        # doc comment for why the live value wins over this fallback).
        self._cycle_duration_s: float = _CYCLE_DURATION_S
        self._handle: ctypes.c_void_p | None = None

        self.on_telemetry: "Callable[[TLMFrame], None] | None" = None
        self.on_truth: "Callable[[tuple[float, float, float]], None] | None" = None
        # 129-003: optional immediate-delivery callback for DBG: lines,
        # mirroring on_telemetry's own shape -- called from the tick
        # thread, wrapped in try/except so a raising callback can never
        # kill it (SerialConnection.on_debug's own exception-proof
        # contract, io/serial_conn.py). Most callers instead poll
        # drain_debug_lines(); this exists for a caller that wants
        # immediate delivery the way on_telemetry already offers for TLM.
        self.on_debug: "Callable[[str], None] | None" = None
        # Non-DBG cleartext observer (system-test recorder): today's
        # sim_drain_debug() C export filters drainReliable() down to DBG:
        # lines only, so with a stock library this never fires; a library
        # built with that filter removed delivers READY/STATUS/PONG/DEVICE
        # replies here. Same exception-proof dispatch contract as on_debug.
        self.on_cleartext: "Callable[[str], None] | None" = None

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._telemetry_suspended = threading.Event()
        # Queue of plain 0-arg callables -- the tick thread is the only
        # consumer/executor, matching testgui/transport.py's SimTransport
        # convention (see that module's _drain_cmd_queue docstring).
        self._cmd_queue: "queue.Queue[Callable[[], None]]" = queue.Queue()
        self._tlm_queue: "queue.Queue[TLMFrame]" = queue.Queue(maxsize=_TLM_QUEUE_MAXSIZE)
        # 129-003 (bench/Sim-only DBG channel): bounded, drop-oldest,
        # mirroring _tlm_queue's own policy -- see drain_debug_lines().
        self._debug_queue: "queue.Queue[str]" = queue.Queue(maxsize=_DEBUG_QUEUE_MAXSIZE)

        self._corr_lock = threading.Lock()
        self._corr_id = 0
        self._speed_factor = 1

        # Motor-state-aware tick cadence (see module-level
        # _IDLE_HEARTBEAT_INTERVAL_S doc comment and _tick_loop()'s own
        # docstring). ``None`` == "no frame drained yet -- unknown", treated
        # as full-rate (the safe default: never silently throttle before
        # we've actually heard the plant say it's idle). Updated from the
        # latest drained TLMFrame's ``.active`` field; a frame with
        # ``active is None`` (older/pre-fault frame) leaves the last known
        # state alone.
        self._active: "bool | None" = None

        # Kept alive for as long as a hook is registered -- ctypes holds no
        # reference of its own to a CFUNCTYPE-wrapped callback (see module
        # docstring's "Hook wrappers" section).
        self._read_hook_c: Any = None
        self._write_hook_c: Any = None
        self._read_hook_py: HookCallback | None = None
        self._write_hook_py: HookCallback | None = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._handle is not None

    def connect(self, start_tick_thread: bool = True) -> None:
        """Load the sim lib, create a ``SimHandle`` (booted), and optionally
        start the background tick thread. Idempotent -- a no-op if already
        connected. Raises ``FileNotFoundError`` if the lib has not been
        built (``cmake --build`` in ``src/sim/build``)."""
        if self.is_connected:
            return
        if not self._lib_path.exists():
            raise FileNotFoundError(
                f"sim lib not found at {self._lib_path} -- build it: "
                f"cmake -S src/sim -B src/sim/build && "
                f"cmake --build src/sim/build")

        self._lib = ctypes.CDLL(str(self._lib_path))
        _bind_ctypes(self._lib)
        # 118 ticket 003: pace against what THIS loaded binary actually runs,
        # not the module-level fallback -- see _cycle_duration_s's own
        # __init__ comment.
        self._cycle_duration_s = self._lib.sim_cycle_dt_us() / 1e6
        self._handle = self._lib.sim_create(ctypes.c_float(self._track_width))

        self._stop_event.clear()
        if start_tick_thread:
            self._thread = threading.Thread(
                target=self._tick_loop, name="sim-loop-tick-thread", daemon=True)
            self._thread.start()

    def disconnect(self) -> None:
        """Stop the tick thread (if running) and destroy the sim handle.
        Safe to call whether or not connected; never raises."""
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            if self._thread is not threading.current_thread():
                self._thread.join(timeout=3.0)
        self._thread = None

        if self._handle is not None and self._lib is not None:
            try:
                self._lib.sim_destroy(self._handle)
            except Exception:
                pass
        self._handle = None
        self._lib = None
        self._read_hook_c = None
        self._write_hook_c = None

    def firmware_version(self) -> str:
        """Version string compiled into the LOADED sim library (not the source
        tree) -- lets the GUI show which built binary is actually running."""
        self._require_connected()
        raw = self._lib.sim_firmware_version()
        return raw.decode() if raw else "?"

    # ------------------------------------------------------------------
    # Configure from robot (113-005) -- "the sim configures on open"
    # ------------------------------------------------------------------

    def configure_from_robot(self, config: "RobotConfig") -> None:
        """Configure the running sim from *config* (a
        ``robot_radio.config.robot_config.RobotConfig``, or any duck-typed
        object with the same attribute structure) -- THREE tiers close the
        gap between what a real serial boot bakes in and what a bare Sim
        session would otherwise run with:

        - **Tier 1** (the live per-field SET wire plane -- fields BOTH real
          hardware and this sim already apply identically via
          ``Configurator::applyField()``): builds a ``NezhaProtocol``
          wrapping a ``SimConfigConn`` over this ``SimLoop`` and calls
          ``set_config(**calibration_kwargs(config))`` -- ticket 003's
          extracted field-selection function. 132-014: ``calibration_kwargs()``
          now selects ml/mr/pid.* only (MOTORS.travel_calib_*/
          WHEEL_CONTROL.pid_*) -- ``tw``/``rotSlip`` are deliberately NOT
          selected any more (see that function's own docstring: GEOMETRY is
          boot-only AND one of this sim's own justified ``BootOverrides``
          divergences, trackWidth -- a live push must never reach it).
          Reuses the EXACT envelope-building/wire-key vocabulary hardware
          transports use (109-002 Architecture Revision 1's "one mechanism,
          not a Sim-specific fork") -- no Tier-1 field selection is
          reimplemented here.
        - **Tier 2** (the boot-only fields with no live wire arm):
          calls ``motor_boot_config_for(config, port)`` (ticket 004's reuse
          of ``gen_boot_config.py``'s own mapping functions) and passes the
          result to the ``sim_configure_motor()`` ctypes export (ticket
          002) -- ``SimHarness::configureMotor()``'s one-shot runtime-load
          surface. (The planner half of this tier --
          ``planner_boot_config_for()``/``sim_configure_planner()`` --
          was DELETED, 115-003, gut S1 motion-stack excision: nothing in
          the S1 minimal firmware reads a boot-loaded ``msg::PlannerConfig``
          any more.) Also calls ``drivetrain_boot_config_for(config)``
          (125-007) and passes the result to the ``sim_configure_drivetrain()``
          ctypes export -- ``App::RobotLoop::setRotationCalibration()``'s
          own one-shot runtime-load surface, the sim-side counterpart of
          ``main.cpp``'s real-hardware boot seam. Before 125-007 this tier
          never touched turn calibration at all, so the sim's own
          ``App::RobotLoop`` stayed at the identity default (gain 1, offset
          0) regardless of a robot JSON's own
          ``calibration.rotation_gain``/``rotation_offset_deg`` values.
        - **Tier 3** (119 ticket 001,
          kill-the-silent-off-shaping-config-boundary.md, RETARGETED
          132-014): ``App::StateEstimator``'s fusion weights plus
          ``Motion::VelocityShaper``'s accel/jerk ceilings
          (``push.estimator_kwargs(config)``, the SAME field selection the
          TestGUI's own connect-time ``_push_estimator_config()`` uses),
          pushed one ``set_config_field()`` round trip per field over the
          SAME ``config_proto``/``SimConfigConn`` Tier 1 already built (one
          config connection, not a second one) -- the old single
          ``EstimatorConfigPatch`` envelope (``estimator_config(**kwargs)``)
          this method used to build no longer exists (``config.proto``
          deleted, 132-013); ``robot_config.proto`` spreads the same nine
          fields across TWO ``ConfigGroupTarget``s now
          (``push.ESTIMATOR_FIELDS``/``push.PLANNER_SHAPER_FIELDS`` name
          the split).

          ``ESTIMATOR`` is a HONEST DEAD END: it decodes but
          ``install(ESTIMATOR)`` permanently returns ``ERR_UNIMPLEMENTED``
          (``App::StateEstimator`` was already deleted as dead code before
          this sprint, sprint 128 ticket 016 -- configurator.h's own
          re-appliability table). ``PLANNER_SHAPER`` -- FIXED, 132-017
          (JSON reshape ticket, stakeholder-sanctioned mid-sprint scope
          addition): the six shaper-ceiling fields this method pushes now
          land on their OWN ``ConfigGroupTarget``, split out of the
          boot-only ``PLANNER`` group specifically because they carry
          their own re-appliable setter (``Motion::Planner::
          applyShaperLimits()``) -- a live push now genuinely lands, the
          same capability this method had before sprint 132's schema
          unification (132-002 through 132-013) temporarily regressed it.
          See ``estimator_kwargs()``'s own docstring for the full
          per-field target-split rationale.

          A config carrying none of the nine estimator/shaper fields (rare
          -- every shipped robot JSON populates all nine) is a logged
          no-op, never a raised exception -- ``estimator_kwargs()``'s own
          "return ``{}``, caller must treat that as nothing to push"
          contract.

        Tier 1 runs first (the smaller, already-proven mechanism); Tier 2
        second; Tier 3 third. No tier's outcome depends on another's.

        Requires an active connection (``_require_connected()``, matching
        every other ``SimLoop`` method's precondition style). Every import
        this method needs is deferred to inside this method body (matching
        this module's own existing convention for ``envelope_pb2``/
        ``TLMFrame`` -- see the module docstring's "Lazily-imported/cached
        pb2 module" note) -- deliberately, so this method has NO import-time
        OR call-time dependency on ``robot_radio.testgui`` (or Qt): a
        headless caller (pytest fixture, diagnostic script) can call this
        without pulling in Qt at all (sprint 113's SUC-002).
        """
        self._require_connected()

        # ---- Tier 1: live per-field SET wire plane -------------------------
        from robot_radio.calibration.push import (
            ESTIMATOR_FIELDS, calibration_kwargs, estimator_kwargs)
        from robot_radio.io.sim_config import SimConfigConn
        from robot_radio.robot.pb2 import robot_config_pb2
        from robot_radio.robot.protocol import NezhaProtocol

        # Kept as its own local (not just wrapped straight into NezhaProtocol)
        # so Tier 3 below can reuse its poll_ack() -- the SAME single config
        # connection every tier of this method shares, not a second one.
        sim_config_conn = SimConfigConn(self)
        config_proto = NezhaProtocol(sim_config_conn)  # type: ignore[arg-type]
        config_proto.set_config(**calibration_kwargs(config))

        # ---- Tier 2: one-shot boot-config load surface (motor + drivetrain
        # rotation calibration -- the planner half was DELETED, 115-003, gut
        # S1 motion-stack excision) ------------------------------------------
        from robot_radio.calibration.sim_boot_config import (
            drive_boot_config_for, drivetrain_boot_config_for, motor_boot_config_for)

        for port in (1, 2):  # 1=left, 2=right -- same convention as every other port-keyed call
            motor_cfg = motor_boot_config_for(config, port)
            self._lib.sim_configure_motor(
                self._handle, ctypes.c_int(port),
                ctypes.c_float(motor_cfg["vel_filt_alpha"]),
                ctypes.c_int(motor_cfg["fwd_sign"]))

        # 125-007: turn calibration -- the sim's own App::RobotLoop otherwise
        # keeps the identity default (gain 1, offset 0) forever, regardless
        # of what a robot JSON's calibration.rotation_gain/
        # rotation_offset_deg say. See sim_configure_drivetrain()'s own
        # doc comment (sim_ctypes.cpp) for why this tier was missing.
        drivetrain_cfg = drivetrain_boot_config_for(config)
        self._lib.sim_configure_drivetrain(
            self._handle,
            ctypes.c_float(drivetrain_cfg["rot_gain_pos"]),
            ctypes.c_float(drivetrain_cfg["rot_offset_pos"]),
            ctypes.c_float(drivetrain_cfg["rot_gain_neg"]),
            ctypes.c_float(drivetrain_cfg["rot_offset_neg"]))

        # App::Drive's own boot calibration -- the sim's Drive is constructed
        # uncalibrated by TestSim::SimHarness and, without this call, refuses
        # to write a duty at all (drive.h's fail-closed gate). See
        # drive_boot_config_for()'s own docstring for what used to stand in
        # for it (the `pid.kff` wire key, carrying the wrong quantity) and
        # why that made every sim run drive at ~43% of the commanded speed.
        drive_cfg = drive_boot_config_for(config)
        self._lib.sim_configure_drive(
            self._handle,
            ctypes.c_float(drive_cfg["duty_per_speed_left"]),
            ctypes.c_float(drive_cfg["duty_per_speed_right"]),
            ctypes.c_float(drive_cfg["crawl_pulse"]))

        # ---- Tier 3: estimator/shaper fields -- 132-014 RETARGET: the old
        # single EstimatorConfigPatch envelope no longer exists; the same
        # nine fields now span TWO ConfigGroupTarget groups. FIXED, 132-017:
        # PLANNER_SHAPER (the six shaper-ceiling fields) is LIVE now, split
        # out of the boot-only PLANNER group -- see this method's own
        # docstring and estimator_kwargs()'s own docstring. ESTIMATOR
        # remains a permanent, honest dead end (ERR_UNIMPLEMENTED). Still
        # attempted and logged, never silently skipped -- "no silent
        # no-ops."
        est_kwargs = estimator_kwargs(config)
        if not est_kwargs:
            logger.info(
                "configure_from_robot(): no estimator/shaper fields on config -- "
                "push skipped")
            return

        applied: list[str] = []
        rejected: list[str] = []
        for field_name, value in est_kwargs.items():
            target = (robot_config_pb2.ESTIMATOR if field_name in ESTIMATOR_FIELDS
                      else robot_config_pb2.PLANNER_SHAPER)
            try:
                ack = config_proto.set_config_field(target, field_name, value)
            except Exception as exc:  # noqa: BLE001 -- log, don't raise out of a boot-config call
                logger.warning(
                    "configure_from_robot(): %s push failed to send: %s", field_name, exc)
                rejected.append(field_name)
                continue
            (applied if ack is not None else rejected).append(field_name)

        if rejected:
            logger.info(
                "configure_from_robot(): pushed %d/%d estimator/shaper fields (%s) -- "
                "%d rejected (%s), expected: ESTIMATOR has no live consumer "
                "(ERR_UNIMPLEMENTED) as of sprint 132's own re-appliability "
                "table -- PLANNER_SHAPER fields rejecting would be a "
                "regression, not expected (132-017)",
                len(applied), len(est_kwargs), sorted(applied), len(rejected), sorted(rejected))
        else:
            logger.info(
                "configure_from_robot(): pushed %d/%d estimator/shaper fields (%s)",
                len(applied), len(est_kwargs), sorted(applied))

    # ------------------------------------------------------------------
    # Tier-2 config-load readback (113-007) -- test-only diagnostic proving
    # what configure_from_robot() (or a direct sim_configure_motor() ctypes
    # call) actually landed. No production caller needs this -- it exists
    # for sprint 113's own golden-parity test (test_sim_boot_config_parity.py)
    # to compare against gen_boot_config.py's independently-computed
    # expected values, field-for-field. `read_planner_config()` -- DELETED
    # (115-003, gut S1 motion-stack excision): `msg::PlannerConfig` and its
    # `sim_read_planner_config()` ctypes export went with `Motion::Executor`/
    # `App::Pilot`.
    # ------------------------------------------------------------------

    def read_motor_config(self, port: int) -> "dict[str, float | int]":
        """Return ``{"vel_filt_alpha": ..., "fwd_sign": ...}`` last pushed to
        *port* (1=left, 2=right) via ``configureMotor()`` -- the same dict
        shape ``sim_boot_config.motor_boot_config_for()`` returns.
        Synchronous round-trip onto the tick thread when one is running
        (same rationale as ``get_true_pose()``): a caller must see whatever
        was already applied, not "eventually applied"."""
        self._require_connected()
        return self._call_on_tick_thread(lambda: self._read_motor_config(port))

    def _read_motor_config(self, port: int) -> "dict[str, float | int]":
        vel_filt_alpha = ctypes.c_float()
        fwd_sign = ctypes.c_int()
        self._lib.sim_read_motor_config(
            self._handle, ctypes.c_int(port),
            ctypes.byref(vel_filt_alpha), ctypes.byref(fwd_sign))
        return {"vel_filt_alpha": vel_filt_alpha.value, "fwd_sign": fwd_sign.value}

    # ------------------------------------------------------------------
    # TwistTransport protocol (planner/executor.py) -- twist()/stop()/
    # read_pending_binary_tlm_frames()
    # ------------------------------------------------------------------

    def twist(self, v_x: float, omega: float, duration: float) -> int:  # [mm/s] [rad/s] [ms]
        """Fire-and-poll, matching ``NezhaProtocol.twist()``'s own contract
        exactly: assigns and returns a ``corr_id`` immediately without
        waiting for the tick thread to actually inject it."""
        self._require_connected()
        corr_id = self._next_corr_id()
        self._run_or_enqueue(
            lambda: self._lib.sim_inject_twist(
                self._handle, ctypes.c_float(v_x), ctypes.c_float(omega),
                ctypes.c_float(duration), ctypes.c_uint32(corr_id)))
        return corr_id

    def stop(self) -> int:
        """Fire-and-poll. NOTE: ``sim_inject_stop`` (``sim_ctypes.cpp``) is
        retargeted at ESTOP, not the planned STOP -- see that function's own
        comment (command-ingestion-ring-buffered-comms-subsystem-routing-
        two-stops.md §2: "every existing caller of this entry point means
        'halt the drivetrain now'"). This method's NAME is unchanged for
        back-compat with existing ``TwistTransport`` callers; ``estop()``
        below is the same call under its own, unambiguous name."""
        self._require_connected()
        corr_id = self._next_corr_id()
        self._run_or_enqueue(
            lambda: self._lib.sim_inject_stop(self._handle, ctypes.c_uint32(corr_id)))
        return corr_id

    def estop(self) -> int:
        """``TwistTransport.estop()`` -- the SAME injection ``stop()``
        already performs (``sim_inject_stop`` sends ESTOP on the wire; see
        that method's own docstring), exposed under its real name so a
        caller migrating off the "stop() means halt-now" assumption has an
        unambiguous method to call."""
        return self.stop()

    def wheels(self, v_left: float, v_right: float, duration: float,  # [mm/s] x2 [ms]
               ) -> int:
        """The WHEELS teleop primitive -- per-wheel velocity held for a bounded
        `duration`, routed firmware-side straight to ``App::Drive`` after
        ``planner_.estop()``. No profile, no shaping, no planner stop condition.

        Deliberately the SAME signature and the same wire command as
        ``NezhaProtocol.wheels()``: a button's message must not depend on which
        transport is underneath it (stakeholder, 2026-07-31 -- "the button on
        SIM sends the same message as the button"). Without this, the shared
        unmanaged-drive routine had no Sim-side primitive to call and the two
        backends drifted into different commands for the same button, which is
        precisely the divergence the Sim exists to rule out.

        Bounded by construction: ``App::Drive`` arms it until
        ``commandDeadline_ = now + duration``, so a host that stops re-arming
        stops the robot within one lease.
        """
        self._require_connected()
        corr_id = self._next_corr_id()
        self._run_or_enqueue(
            lambda: self._lib.sim_inject_wheels(
                self._handle, ctypes.c_float(v_left), ctypes.c_float(v_right),
                ctypes.c_float(duration), ctypes.c_uint32(corr_id)))
        return corr_id

    def move(self, *, v_x: float = 0.0, v_y: float = 0.0, omega: float = 0.0,
             v_left: "float | None" = None, v_right: "float | None" = None,
             stop_time: "float | None" = None, stop_distance: "float | None" = None,
             stop_angle: "float | None" = None, timeout: float,
             replace: bool = True, id: "int | None" = None) -> int:
        """MOVE-queue command -- builds and injects
        ``CommandEnvelope{move: Move{...}}`` via ``inject_command()``'s
        generic escape hatch (the SAME mechanism ``_SimConfigConn.
        send_envelope_fast()`` uses for the config path, Architecture
        Revision 1's "one mechanism, not a Sim-specific fork") rather than a
        dedicated ``sim_inject_move`` ctypes symbol -- unlike
        ``twist()``/``stop()`` (hot teleop-path calls with their own fast
        ctypes entry points), Move is sent at most once per leg, so the
        extra Python-side envelope-build cost here is immaterial.

        Rebuilt (116-001 MOVE-protocol cutover;
        ``clasi/issues/testgui-motion-paths-dead-after-move-cutover.md``)
        against the CURRENT ``Move`` schema (``envelope.proto``) -- this is
        NOT the same shape the pre-115 sprint-109 arc-command ``Move`` used
        (bare top-level ``distance``/``delta_heading``/``v_max``/``omega``/
        ``time`` fields); that message was deleted wholesale (115-003, gut
        S1 motion-stack excision). A velocity variant -- ``MoveTwist{v_x,
        v_y, omega}`` (the default; leave ``v_left``/``v_right`` both
        ``None``) OR ``MoveWheels{v_left, v_right}`` (pass BOTH -- raises
        ``ValueError`` if only one is given) -- plus exactly ONE stop
        condition (``stop_time``/``stop_distance``/``stop_angle``, built via
        ``protocol._build_move_stop_kwargs()`` -- the SAME helper
        ``NezhaProtocol.move_twist()``/``move_wheels()`` use, reused rather
        than reimplemented so the "exactly one" validation lives in ONE
        place) plus a REQUIRED ``timeout`` safety backstop (``ValueError``
        if not ``> 0``, mirroring ``move_twist()``'s own host-side check).

        ``id`` becomes ``Move.id`` -- the key THIS Move's own LATER
        completion event echoes (``docs/protocol-v4.md`` section 7.2).
        Defaults to this instance's own ``_next_corr_id()`` counter when
        omitted (matching ``twist()``/``stop()``'s own auto-assignment) --
        every Move sent through this method therefore gets a distinct,
        incrementing id unless the caller overrides it.

        The envelope's own ``corr_id`` (the EARLIER enqueue ack's
        correlation key) is now assigned INDEPENDENTLY from ``id``/
        ``Move.id`` -- ``self._next_corr_id()`` again, a SEPARATE draw from
        the SAME counter ``id`` itself defaults from, so the two never
        collide within one session. This mirrors
        ``NezhaProtocol.move_twist()``/``move_wheels()``, whose envelope
        ``corr_id`` is auto-assigned by ``send_envelope_fast()``'s own
        connection-scoped counter, genuinely distinct from the caller's
        ``move_id`` (see that method's own docstring). Before this fix, this
        method set ``corr_id=move_id`` -- CommandEnvelope 116-001 cutover
        anomaly, `turn-prediction-campaign` diagnosis: RobotLoop::handleMove()
        acks the ENQUEUE outcome against the envelope's own ``corr_id``
        (``tlm_.ack(result.corrId, ...)``, ``move_queue.h``) SEPARATELY from
        the COMPLETION ack against ``Move.id`` (``tlm_.ack(moveResult.
        completion.moveId, 0)``) -- both ride the SAME single ack slot
        (``Telemetry.ack_corr``/``ack_err``). With ``corr_id == move_id``,
        the FIRST (enqueue) ack a poller drains already satisfies a
        ``frame.ack.corr_id == move_id`` match (``planner.tour``'s
        ``_drain_and_poll()``), so a caller waiting for the Move's own
        COMPLETION mistakenly accepts the near-instant enqueue ack instead
        -- and, since an ``ERR_FULL`` rejection is ALSO acked against the
        SAME (aliased) corr_id, a rejected Move reads as completed too. Real
        hardware never hit this (its own envelope ``corr_id`` was already
        independent) -- only the Sim path aliased the two.

        Returns the id used (``Move.id`` -- NOT the envelope's own
        ``corr_id``, which is fire-and-forgotten here exactly as
        ``NezhaProtocol.move()`` already documents its own equivalent
        return value asymmetry). Fire-and-poll, matching ``twist()``/
        ``stop()`` -- this call never blocks on a reply; a caller learns
        the outcome from telemetry's ack slot, same as a real robot (see
        ``docs/protocol-v4.md`` section 7).
        """
        self._require_connected()
        from robot_radio.robot.protocol import _build_move_stop_kwargs

        if timeout <= 0:
            raise ValueError(f"move(): timeout must be > 0, got {timeout!r}")
        stop_kwargs = _build_move_stop_kwargs(
            stop_time=stop_time, stop_distance=stop_distance, stop_angle=stop_angle)

        move_id = id if id is not None else self._next_corr_id()
        corr_id = self._next_corr_id()  # independent draw -- see this method's own doc comment
        pb2_mod = _get_envelope_pb2()

        if v_left is not None or v_right is not None:
            if v_left is None or v_right is None:
                raise ValueError(
                    "move(): v_left and v_right must both be given for a "
                    "wheels Move (got only one)")
            velocity_kwargs = {"wheels": pb2_mod.MoveWheels(v_left=v_left, v_right=v_right)}
        else:
            velocity_kwargs = {"twist": pb2_mod.MoveTwist(v_x=v_x, v_y=v_y, omega=omega)}

        envelope = pb2_mod.CommandEnvelope(
            corr_id=corr_id,
            move=pb2_mod.Move(
                timeout=timeout, replace=replace, id=move_id,
                **velocity_kwargs, **stop_kwargs))
        # "MOVE" unconditionally -- this method always builds a
        # CommandEnvelope{move: ...} (124-005, issue §1/§3: the wire line's
        # own leading prefix and the CRC's scope-extension argument).
        frame = encode_frame(envelope.SerializeToString(), command=b"MOVE")
        self.inject_command(b"MOVE:" + frame)
        return move_id

    def read_pending_binary_tlm_frames(self) -> "list[TLMFrame]":
        """Non-blocking drain of every currently-queued ``TLMFrame`` --
        the sim-side counterpart of ``NezhaProtocol.
        read_pending_binary_tlm_frames()``. Populated by the tick thread's
        own per-iteration ``sim_drain_tlm()`` drain (or, with no tick
        thread running, by whatever last called ``step()`` on the calling
        thread)."""
        frames: "list[TLMFrame]" = []
        try:
            while True:
                frames.append(self._tlm_queue.get_nowait())
        except queue.Empty:
            pass
        return frames

    def drain_debug_lines(self) -> "list[str]":
        """Non-blocking drain of every currently-queued ``DBG:`` line
        (129-003, bench/Sim-only debug channel -- ``App::debugf()``,
        ``app/debug.h``). Same shape as ``read_pending_binary_tlm_frames()``:
        a pure queue read, populated by the tick thread's own per-iteration
        ``sim_drain_debug()`` drain (or, with no tick thread running, by
        ``drain_pending_debug()``, this method's manual-mode counterpart --
        matching ``drain_pending_tlm()``'s own relationship to
        ``read_pending_binary_tlm_frames()``). Each returned string is the
        FULL decoded line, verb included (e.g. ``"DBG:v=1234"``), matching
        ``SerialConnection``'s own ``on_debug`` callback convention
        (``io/serial_conn.py``)."""
        lines: "list[str]" = []
        try:
            while True:
                lines.append(self._debug_queue.get_nowait())
        except queue.Empty:
            pass
        return lines

    def drain_pending_debug(self) -> "list[str]":
        """Manual-mode counterpart of the tick thread's own per-iteration
        debug drain -- see ``drain_pending_tlm()``'s own docstring; same
        shape, ``DBG:`` lines instead of ``TLMFrame`` objects."""
        self._require_connected()
        self._drain_debug_into_queue()
        return self.drain_debug_lines()

    # ------------------------------------------------------------------
    # Telemetry-reader suspend/resume (mirrors _HardwareTransport)
    # ------------------------------------------------------------------

    def suspend_telemetry_reader(self) -> None:
        """Stop delivering drained frames to ``on_telemetry`` -- see module
        docstring. Idempotent; safe regardless of tick-thread state."""
        self._telemetry_suspended.set()

    def resume_telemetry_reader(self) -> None:
        """Undo ``suspend_telemetry_reader()``. Idempotent."""
        self._telemetry_suspended.clear()

    # ------------------------------------------------------------------
    # Raw command injection escape hatch
    # ------------------------------------------------------------------

    def inject_command(self, frame: bytes) -> None:
        """Push an already-`<COMMAND>':'<COBS+CRC bytes>`-framed wire LINE
        (124-005; was a bare COBS+CRC command body 123-002-124-004, an
        already-armored ``*B...`` text line pre-123) straight onto the
        inbound FakeTransport -- ``sim_inject_command()``'s own escape hatch
        for a wire shape ``twist()``/``stop()`` don't cover. Build the COBS
        half with ``robot_radio.io.wire_codec.encode_frame()``, PREFIXED
        with the ASCII command name and ``':'``. Passes ``len(frame)``
        explicitly (never NUL-terminated recovery) -- see
        ``_bind_ctypes()``'s own doc comment for why."""
        self._require_connected()
        self._run_or_enqueue(
            lambda: self._lib.sim_inject_command(self._handle, frame, len(frame)))

    # ------------------------------------------------------------------
    # True pose
    # ------------------------------------------------------------------

    def get_true_pose(self) -> dict:
        """Ground-truth ``{"x": ..., "y": ..., "h": ...}`` in (mm, mm, rad)
        -- ``SimPlant``'s owned OTOS-plant ground truth, bypassing every
        drift/noise fault knob (``sim_true_x/y/h``). Synchronous
        round-trip onto the tick thread when one is running (see module
        docstring) so the read never races a concurrent ``step()``."""
        self._require_connected()
        return self._call_on_tick_thread(self._read_true_pose)

    def _read_true_pose(self) -> dict:
        return {
            "x": float(self._lib.sim_true_x(self._handle)),
            "y": float(self._lib.sim_true_y(self._handle)),
            "h": float(self._lib.sim_true_h(self._handle)),
        }

    def set_true_pose(self, x: float, y: float, heading: float) -> None:  # [mm] [mm] [rad]
        """Teleport the plant's ground-truth pose to ``(x, y, heading)`` --
        ``sim_set_true_pose()``'s own Python binding. Synchronous round-trip
        onto the tick thread when one is running (see module docstring's
        "Threading model" section, same rationale as ``get_true_pose()``):
        a caller that immediately reads the pose back afterward must see the
        teleport already applied, not "eventually applied" the way
        ``twist()``/``stop()`` are.

        Resets both ``WheelPlant`` positions to 0 in the same call
        (``SimPlant::setTruePose()``'s own C++ contract) -- see that
        method's own comment for why the OtosPlant re-baseline and the
        wheel-position resets must happen together."""
        self._require_connected()
        self._call_on_tick_thread(lambda: self._lib.sim_set_true_pose(
            self._handle, ctypes.c_float(x), ctypes.c_float(y), ctypes.c_float(heading)))

    # ------------------------------------------------------------------
    # Fault-condition setters (thin call-throughs, port: 1=left, 2=right)
    # ------------------------------------------------------------------

    def set_wheel_disconnected(self, port: int, disconnected: bool) -> None:
        self._require_connected()
        self._call_on_tick_thread(
            lambda: self._lib.sim_set_wheel_disconnected(
                self._handle, int(port), 1 if disconnected else 0))

    def set_wheel_freeze(self, port: int, freeze: bool) -> None:
        self._require_connected()
        self._call_on_tick_thread(
            lambda: self._lib.sim_set_wheel_freeze(
                self._handle, int(port), 1 if freeze else 0))

    def set_wheel_dropout_rate(self, port: int, fraction: float) -> None:  # [0,1]
        self._require_connected()
        self._call_on_tick_thread(
            lambda: self._lib.sim_set_wheel_dropout_rate(
                self._handle, int(port), ctypes.c_float(fraction)))

    def set_otos_drift(self, x_drift: float, y_drift: float,
                       heading_drift: float) -> None:  # [mm] [mm] [rad]
        self._require_connected()
        self._call_on_tick_thread(
            lambda: self._lib.sim_set_otos_drift(
                self._handle, ctypes.c_float(x_drift), ctypes.c_float(y_drift),
                ctypes.c_float(heading_drift)))

    def set_enc_scale_err(self, port: int, fraction: float) -> None:  # [fractional over/under-report]
        """109-002: fractional per-side encoder over/under-report knob --
        ``sim_ctypes.cpp``'s ``sim_set_enc_scale_err()``, added this ticket
        alongside the other three fault-condition setters above (``sim_plant.h``'s
        ``SimPlant::setEncScaleErr()`` -> ``WheelPlant::setScaleErr()``).
        port: 1=left, 2=right, matching every other port-keyed knob here."""
        self._require_connected()
        self._call_on_tick_thread(
            lambda: self._lib.sim_set_enc_scale_err(
                self._handle, int(port), ctypes.c_float(fraction)))

    def set_otos_raw_scale_err(self, linear_fraction: float,
                               angular_fraction: float) -> None:  # [fractional over/under-report, 0=perfect]
        """109-007: models a physically MIS-calibrated OTOS chip --
        ``sim_ctypes.cpp``'s ``sim_set_otos_raw_scale_err()`` ->
        ``SimPlant::setOtosRawScaleErr()`` -> ``OtosPlant::setRawScaleErr()``.
        A firmware-pushed OTOS calibration scalar (``OL``/``OA``, or a live
        ``OtosConfigPatch`` -- ticket 004's direct-patch-send mechanism)
        multiplies back against this fault knob (captured by
        ``SimPlant::handleOtosWrite()``'s new register-write path), so a
        correctly calibrated chip reports the true pose again -- see
        ``otos_plant.h``'s own header comment for the full net-effect
        contract. 0.0/0.0 (the default) is a genuine no-op."""
        self._require_connected()
        self._call_on_tick_thread(
            lambda: self._lib.sim_set_otos_raw_scale_err(
                self._handle, ctypes.c_float(linear_fraction),
                ctypes.c_float(angular_fraction)))

    def set_enc_tick_quant(self, port: int, tick_size: float) -> None:  # [mm]
        """109-007: per-wheel encoder tick-quantization knob -- rounds the
        reported position to the nearest multiple of ``tick_size`` [mm],
        modeling a real encoder's finite count resolution. 0.0 (the
        default) is a genuine no-op. port: 1=left, 2=right, matching every
        other port-keyed knob here."""
        self._require_connected()
        self._call_on_tick_thread(
            lambda: self._lib.sim_set_enc_tick_quant(
                self._handle, int(port), ctypes.c_float(tick_size)))

    def set_enc_slip(self, port: int, rate: float, magnitude: float) -> None:  # [0,1] [mm]
        """109-007: per-wheel encoder slip-event knob -- a deterministic
        accumulator (mirrors ``set_wheel_dropout_rate()``'s own design, no
        RNG) fires a slip event every time it crosses 1.0, injecting a
        PERMANENT signed ``magnitude`` [mm] offset into every future
        reported position -- models a wheel that slipped against the
        surface. ``rate``=0.0 (the default) never fires. port: 1=left,
        2=right."""
        self._require_connected()
        self._call_on_tick_thread(
            lambda: self._lib.sim_set_enc_slip(
                self._handle, int(port), ctypes.c_float(rate), ctypes.c_float(magnitude)))

    # set_lead_compensation() -- DELETED (115-003, gut S1 motion-stack
    # excision): `msg::PlannerConfig`'s lead-compensation fields and
    # `SimHarness::setLeadCompensation()`/`sim_set_lead_compensation()` went
    # with `Motion::Executor`/`App::Pilot`.

    def set_pid_enabled(self, enabled: bool) -> None:
        """Enable/disable the velocity PID on BOTH firmware motors
        (``sim_set_pid_enabled()`` -> ``NezhaMotor::setPidEnabled()``, both
        ports). Firmware default is enabled. With PID OFF, a velocity-staged
        command drives OPEN-LOOP: duty = ``Gains::kff`` * target with every
        feedback term bypassed -- twist/Move motion keeps moving at the
        feedforward-nominal speed, uncorrected (so a fault knob like
        ``set_enc_scale_err()`` visibly goes uncompensated). Synchronous
        round-trip onto the tick thread (same rationale as the fault
        setters above)."""
        self._require_connected()
        self._call_on_tick_thread(
            lambda: self._lib.sim_set_pid_enabled(
                self._handle, 1 if enabled else 0))

    # set_yaw_rate_max() -- DELETED (115-003, gut S1 motion-stack excision):
    # `PlannerConfig.yaw_rate_max` and `SimHarness::setYawRateMax()`/
    # `sim_set_yaw_rate_max()` went with `Motion::Executor`/`App::Pilot`.

    # ------------------------------------------------------------------
    # Manual stepping (no tick thread required -- ticket 009's shape)
    # ------------------------------------------------------------------

    def step(self, cycles: int = 1) -> None:
        """Advance the sim ``cycles`` cycles (40ms sim-time each) on the
        CALLING thread. Only safe to call directly when no tick thread is
        running (``connect(start_tick_thread=False)``) -- otherwise this
        races the tick thread's own ``sim_step()`` calls against the same
        unsynchronized handle."""
        self._require_connected()
        self._lib.sim_step(self._handle, int(cycles))

    def set_speed_factor(self, factor: int) -> None:
        """Set the sim's fast-forward multiple: the tick thread advances
        ``max(1, int(_speed_factor))`` sim cycles per wall-clock tick (see
        ``_tick_loop()``). Clamped to ``[_SPEED_FACTOR_MIN, _SPEED_FACTOR_MAX]``.

        Plain-attribute write, not round-tripped onto the tick thread: the
        tick thread reads ``self._speed_factor`` fresh every iteration, and a
        bare Python ``int`` attribute assignment is atomic under the GIL --
        no lock needed, same reasoning ``testgui/transport.py``'s
        ``SimTransport.set_speed_factor()`` already documented for its own
        direct write to this same attribute (this method now backs that
        call instead of the caller poking the attribute directly). Safe to
        call before ``connect()`` -- takes effect on the tick thread's next
        iteration once one exists.
        """
        self._speed_factor = max(_SPEED_FACTOR_MIN, min(_SPEED_FACTOR_MAX, int(factor)))

    def booted(self) -> bool:
        self._require_connected()
        return bool(self._lib.sim_booted(self._handle))

    def cycle_count(self) -> int:
        self._require_connected()
        return int(self._lib.sim_cycle_count(self._handle))

    def drain_pending_tlm(self) -> "list[TLMFrame]":
        """Manual-mode counterpart of the tick thread's own per-iteration
        drain -- decodes ``sim_drain_tlm()`` right now on the calling
        thread and pushes results onto the same internal queue
        ``read_pending_binary_tlm_frames()`` drains (so both stepping
        styles share one consumer-facing method)."""
        self._require_connected()
        self._drain_tlm_into_queue()
        return self.read_pending_binary_tlm_frames()

    # ------------------------------------------------------------------
    # Hook wrappers
    # ------------------------------------------------------------------

    def set_read_hook(self, cb: "HookCallback | None") -> None:
        """Register (or, with ``cb=None``, clear) a Python read hook.

        ``cb(addr, buf)`` receives the wire address and a mutable
        ``ctypes`` ``(c_uint8 * len)`` array view onto the SAME memory the
        firmware's I2C read targets -- fill it and return 1 (HANDLED), or
        return 0 (PASS, then call ``pass_through()`` first if you want the
        real bytes filled in before returning). See module docstring."""
        self._set_hook(is_write=False, cb=cb)

    def set_write_hook(self, cb: "HookCallback | None") -> None:
        """Register (or, with ``cb=None``, clear) a Python write hook.

        ``cb(addr, buf)`` receives the wire address and a ``ctypes``
        array view of the bytes the firmware just wrote -- return 1
        (HANDLED, e.g. to silently swallow the write) or 0 (PASS)."""
        self._set_hook(is_write=True, cb=cb)

    @contextmanager
    def read_hook(self, cb: "HookCallback") -> "Iterator[None]":
        """Context-managed ``set_read_hook()`` -- registers on entry,
        clears (``cb=None``) on exit, even if the body raises."""
        self.set_read_hook(cb)
        try:
            yield
        finally:
            self.set_read_hook(None)

    @contextmanager
    def write_hook(self, cb: "HookCallback") -> "Iterator[None]":
        """Context-managed ``set_write_hook()`` -- see ``read_hook()``."""
        self.set_write_hook(cb)
        try:
            yield
        finally:
            self.set_write_hook(None)

    def pass_through(self, addr: int, buf: "ctypes.Array[ctypes.c_uint8]",
                     length: int, write: bool) -> int:
        """Call the un-hooked default handler (``sim_default_read()``/
        ``sim_default_write()``) for ``addr`` -- what a hook that wants
        "observe or lightly perturb, but mostly pass through" calls to get
        the real response, then optionally mutates ``buf`` before
        returning its own result (1/HANDLED). Runs on whichever thread the
        hook itself is invoked from (the tick thread, or the calling
        thread in manual-step mode) -- never re-enqueued, since a hook
        callback is by definition already executing from inside a
        ``sim_step()`` call on the thread that owns the handle."""
        self._require_connected()
        ptr = ctypes.cast(buf, ctypes.POINTER(ctypes.c_uint8))
        if write:
            return int(self._lib.sim_default_write(self._handle, addr, ptr, length))
        return int(self._lib.sim_default_read(self._handle, addr, ptr, length))

    def _set_hook(self, is_write: bool, cb: "HookCallback | None") -> None:
        """Register (``cb`` given) or clear (``cb=None``) the read/write
        hook. Builds the ``ctypes.CFUNCTYPE`` trampoline (if any) on the
        CALLING thread -- that step never touches the sim handle, only
        Python/ctypes bookkeeping -- but the actual
        ``sim_set_read_hook()``/``sim_set_write_hook()`` call, which
        reassigns ``SimPlant::readHook_``/``writeHook_`` on the live handle,
        is round-tripped through ``_call_on_tick_thread()`` like every other
        mutator (module docstring's "Threading model" section) so it never
        races a concurrent ``sim_step()``. Symmetric for register AND clear
        -- the race is the same in both directions."""
        self._require_connected()

        if cb is None:
            c_cb = ctypes.cast(None, _SimHookFn)
        else:
            def _trampoline(_ctx, addr, data, length):
                try:
                    arr_type = ctypes.c_uint8 * length if length > 0 else ctypes.c_uint8 * 0
                    arr = (ctypes.cast(data, ctypes.POINTER(arr_type)).contents
                           if length > 0 else arr_type())
                    return int(cb(int(addr), arr))
                except Exception:
                    # A raising Python hook must never crash the sim -- PASS
                    # (0) so the default handler still answers the transaction.
                    return 0

            c_cb = _SimHookFn(_trampoline)

        def _register() -> None:
            register = self._lib.sim_set_write_hook if is_write else self._lib.sim_set_read_hook
            register(self._handle, c_cb, None)

        self._call_on_tick_thread(_register)

        if is_write:
            self._write_hook_c = c_cb if cb is not None else None
            self._write_hook_py = cb
        else:
            self._read_hook_c = c_cb if cb is not None else None
            self._read_hook_py = cb

    # ------------------------------------------------------------------
    # Internal: corr_id assignment
    # ------------------------------------------------------------------

    def _next_corr_id(self) -> int:
        with self._corr_lock:
            self._corr_id += 1
            return self._corr_id

    def _require_connected(self) -> None:
        if not self.is_connected:
            raise ConnectionError("SimLoop is not connected -- call connect() first")

    # ------------------------------------------------------------------
    # Internal: tick-thread routing
    # ------------------------------------------------------------------

    def _run_or_enqueue(self, fn: "Callable[[], None]") -> None:
        """Fire-and-forget: run ``fn`` now if no tick thread is alive,
        otherwise hand it to the tick thread's own queue."""
        if self._thread is not None and self._thread.is_alive():
            self._cmd_queue.put(fn)
        else:
            fn()

    def _call_on_tick_thread(self, fn: "Callable[[], Any]") -> Any:
        """Synchronous round trip: run ``fn`` now if no tick thread is
        alive, otherwise enqueue it and block for the result (bounded --
        never an infinite wait)."""
        if self._thread is None or not self._thread.is_alive():
            return fn()

        result: list = []
        done = threading.Event()

        def _wrapped() -> None:
            try:
                result.append(fn())
            except Exception as exc:  # noqa: BLE001 -- re-raised on the caller's thread below
                result.append(exc)
            finally:
                done.set()

        self._cmd_queue.put(_wrapped)
        if not done.wait(timeout=5.0):
            raise TimeoutError("SimLoop: tick thread did not process call within 5s")
        value = result[0] if result else None
        if isinstance(value, Exception):
            raise value
        return value

    # ------------------------------------------------------------------
    # Background tick thread
    # ------------------------------------------------------------------

    def _tick_loop(self) -> None:
        """Advance the sim, draining commands and telemetry each iteration,
        at a cadence that depends on the plant's own reported motor state
        (OOP sim-motor-state fix).

        State machine
        --------------
        Two speeds, chosen from ``self._active`` (last known
        ``TLMFrame.active``, updated by ``_drain_tlm_into_queue()``)::

            ACTIVE  (self._active in (True, None) -- moving, or unknown/
                     not-yet-heard-from-plant, which defaults to the safe
                     "don't throttle" side):
                One ``sim_step()`` + drain per iteration, exactly like
                before this fix -- full real-time pace, ``self._cycle_duration_s``
                per cycle, ``_speed_factor`` cycles per iteration.

            IDLE (self._active is False -- the plant confirmed the last
                  motion finished):
                Step/drain/deliver only once every
                ``_IDLE_HEARTBEAT_INTERVAL_S`` (~2s) instead of every
                ``self._cycle_duration_s`` (~40ms) -- a slow "I'm still here"
                heartbeat matching a real idle robot's cadence, instead of
                full-rate churn over a pose that (modulo sprint 108's
                intentional rest-dither) isn't changing.

        Regardless of which speed is active, ``_drain_cmd_queue()`` runs
        EVERY iteration -- including every ``_IDLE_POLL_INTERVAL_S`` (~50ms)
        poll tick spent waiting out a heartbeat interval -- so an incoming
        twist/stop/inject is picked up with no more than one poll tick of
        lag, never delayed by the ~2s heartbeat. The moment a command runs
        (``had_cmd``) this iteration steps at full rate regardless of the
        current ``self._active`` reading (the plant hasn't had a chance to
        report ``active=True`` back yet) -- this is what makes resuming
        motion feel immediate, not just "eventually catches up."
        """
        tick_count = 0
        last_heartbeat = time.monotonic()
        # Timestamp of the last "activity" (a command ran, or the plant last
        # reported active=True). The idle heartbeat only kicks in once this is
        # older than _IDLE_GRACE_S — see that constant's own comment for why an
        # instantaneous `active is False` check deadlocks / breaks tours.
        last_active_ts = time.monotonic()
        while not self._stop_event.is_set():
            t0 = time.monotonic()

            had_cmd = self._drain_cmd_queue()
            if had_cmd:
                last_active_ts = t0

            idle = (t0 - last_active_ts) > _IDLE_GRACE_S
            if idle:
                if t0 - last_heartbeat < _IDLE_HEARTBEAT_INTERVAL_S:
                    # Still within the heartbeat window -- poll the cmd
                    # queue again shortly rather than sleeping the full
                    # interval, so a fresh command isn't delayed by up to 2s.
                    self._stop_event.wait(timeout=_IDLE_POLL_INTERVAL_S)
                    continue
                last_heartbeat = t0
                cycles = 1
            else:
                cycles = max(1, int(self._speed_factor))

            try:
                self._lib.sim_step(self._handle, cycles)
            except Exception:
                break

            self._drain_tlm_into_queue()
            self._drain_debug_into_queue()
            # Keep the grace window fresh while the plant reports motion, so a
            # sustained drive/turn stays at full rate for its whole duration.
            if self._active is True:
                last_active_ts = time.monotonic()

            tick_count += 1
            # During an idle heartbeat, every step is already ~2s apart, so
            # always deliver truth on it (the modulo cadence below exists to
            # slow down FULL-RATE delivery, and would otherwise mean a
            # heartbeat step only "counts" 1-in-_TRUTH_EVERY_N_TICKS of the
            # time -- i.e. an up-to-8s-stale UI "I'm here" signal).
            if (idle or tick_count % _TRUTH_EVERY_N_TICKS == 0) and self.on_truth is not None:
                try:
                    pose = self._read_true_pose()
                    self.on_truth((pose["x"], pose["y"], pose["h"]))
                except Exception:
                    pass

            if idle:
                # Heartbeat pacing is handled by last_heartbeat above, not
                # by sleeping the full cycle duration here.
                continue

            # Pace ONE iteration to a single cycle's wall time, regardless of
            # how many sim cycles it stepped -- so speed_factor N steps N cycles
            # per one cycle's wall time = N x real-time (the previous `* cycles`
            # here paced N cycles to N cycles' wall time, i.e. always 1x, so
            # speed_factor did nothing). At a high enough speed_factor the
            # compute for N cycles exceeds one cycle's wall budget and this
            # sleeps 0 -- the sim then free-runs at full compute speed (what the
            # "fast tour" wants).
            elapsed = time.monotonic() - t0
            sleep_s = self._cycle_duration_s - elapsed
            if sleep_s > 0:
                self._stop_event.wait(timeout=sleep_s)

    def _drain_cmd_queue(self) -> bool:
        """Run every currently-queued command. Returns ``True`` if at least
        one command ran this call -- ``_tick_loop()`` uses this to resume
        full-rate stepping immediately on a fresh command, without waiting
        for the plant's own ``active`` flag to catch up (see that method's
        docstring)."""
        ran = False
        try:
            while True:
                fn = self._cmd_queue.get_nowait()
                ran = True
                try:
                    fn()
                except Exception:
                    pass
        except queue.Empty:
            pass
        return ran

    def _drain_tlm_into_queue(self) -> None:
        """One ``sim_drain_tlm()`` call, decoded into ``TLMFrame`` objects,
        pushed onto the bounded internal queue (drop-oldest on overflow --
        mirrors ``SerialConnection``'s own ``_binary_tlm_queue`` policy),
        and (unless suspended) delivered to ``on_telemetry``.

        Also updates ``self._active`` (OOP sim-motor-state fix) from the
        LAST frame decoded this call whose ``.active`` field is not
        ``None`` -- an explicit ``True``/``False`` always overwrites the
        previous reading; a frame that never sets the field (older/
        pre-fault frames) leaves the last known state alone rather than
        being treated as "idle" by default. See ``_tick_loop()``'s
        docstring for how this drives the tick cadence."""
        from robot_radio.robot.protocol import TLMFrame

        # 123-002/003/124-005: sim_drain_tlm() now fills `buf` with raw,
        # memcpy'd bytes -- every captured outbound wire LINE
        # (`<COMMAND>':'<COBS bytes>`), each followed by exactly one '\n'
        # (0x0A) delimiter (sim_ctypes.cpp's own doc comment) -- NOT a
        # NUL-terminated string. Read via `.raw[:n]` (never `.value`, which
        # would stop at the FIRST embedded 0x00 -- a 124-005 line may
        # legitimately contain one, COBS being keyed on 0x0A now) and split
        # on the SAME '\n' delimiter. This drain path only ever carries
        # binary lines (drainRawTelemetry() reads FakeTransport::sent(),
        # never sentReliable()'s cleartext-plane HELLO/PING/ID/VER
        # replies), so a plain b"\n"-split is correct here -- no
        # cleartext/binary demux ambiguity to resolve (contrast
        # ByteStreamDemuxer, used where cleartext and binary genuinely
        # interleave on the same stream, e.g. a real serial connection).
        buf = ctypes.create_string_buffer(_TLM_DRAIN_BUFFER)
        needed = self._lib.sim_drain_tlm(self._handle, buf, _TLM_DRAIN_BUFFER)
        if needed > _TLM_DRAIN_BUFFER:
            # Truncated -- retry once with an exactly-sized buffer (the
            # drain already advanced regardless, per sim_ctypes.cpp's own
            # snprintf-return-value-style convention, so this is a fresh
            # drain of whatever accumulated since, not a re-fetch of the
            # lost data).
            buf = ctypes.create_string_buffer(needed)
            needed = self._lib.sim_drain_tlm(self._handle, buf, needed)
        raw = buf.raw[:needed]
        if not raw:
            return

        pb2_mod = _get_envelope_pb2()
        suspended = self._telemetry_suspended.is_set()
        for line_bytes in raw.split(b"\n"):
            if not line_bytes:
                continue
            reply = _decode_reply_frame(line_bytes, pb2_mod)
            if reply is None or reply.WhichOneof("body") != "tlm":
                continue
            frame = TLMFrame.from_pb2(reply.tlm)

            # Path B (2026-07-17): commanded per-wheel velocity is NOT on the
            # wire (186-byte primary-frame budget). In sim we read it straight
            # from the firmware's live NezhaMotor::velocityTarget() via the
            # ctypes hook and stamp it onto the frame at full telemetry rate,
            # so TestGUI's commanded-vs-actual wheel-speed graph has data.
            if self._lib is not None and self._handle is not None:
                frame.cmd_vel = (
                    int(round(self._lib.sim_cmd_vel_left(self._handle))),
                    int(round(self._lib.sim_cmd_vel_right(self._handle))),
                )

            if frame.active is not None:
                self._active = bool(frame.active)

            if self._tlm_queue.full():
                try:
                    self._tlm_queue.get_nowait()
                except queue.Empty:
                    pass
            try:
                self._tlm_queue.put_nowait(frame)
            except queue.Full:
                pass

            if not suspended and self.on_telemetry is not None:
                try:
                    self.on_telemetry(frame)
                except Exception:
                    pass

    def _drain_debug_into_queue(self) -> None:
        """One ``sim_drain_debug()`` call (129-003, bench/Sim-only DBG
        channel), pushing each captured ``DBG:<message>`` line onto the
        bounded internal queue (drop-oldest on overflow, same policy as
        ``_drain_tlm_into_queue()``'s own ``_tlm_queue``) and, unless no
        callback is installed, delivering it to ``on_debug`` -- wrapped in
        its own try/except so a raising callback can never propagate out
        of the tick thread (the exact historical defect
        ``SerialConnection``'s own ``on_debug`` dispatch guards against,
        ``io/serial_conn.py``'s module docstring: "a `_log` NameError
        inside the host DBG handler killed the reader thread mid-
        session")."""
        buf = ctypes.create_string_buffer(_DEBUG_DRAIN_BUFFER)
        needed = self._lib.sim_drain_debug(self._handle, buf, _DEBUG_DRAIN_BUFFER)
        if needed > _DEBUG_DRAIN_BUFFER:
            # Truncated -- retry once with an exactly-sized buffer, same
            # convention as _drain_tlm_into_queue()'s own retry.
            buf = ctypes.create_string_buffer(needed)
            needed = self._lib.sim_drain_debug(self._handle, buf, needed)
        raw = buf.raw[:needed]
        if not raw:
            return

        for line_bytes in raw.split(b"\n"):
            if not line_bytes:
                continue
            text = line_bytes.decode("utf-8", "ignore")

            # Route non-DBG cleartext (READY/STATUS/PONG/...) to its own
            # observer instead of the DBG queue -- keeps
            # drain_debug_lines()'s DBG-only contract intact whether or not
            # the loaded library still filters at the C level.
            if not text.startswith("DBG:"):
                if self.on_cleartext is not None:
                    try:
                        self.on_cleartext(text)
                    except Exception:
                        pass
                continue

            if self._debug_queue.full():
                try:
                    self._debug_queue.get_nowait()
                except queue.Empty:
                    pass
            try:
                self._debug_queue.put_nowait(text)
            except queue.Full:
                pass

            if self.on_debug is not None:
                try:
                    self.on_debug(text)
                except Exception:
                    pass
