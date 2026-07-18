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
  - Fault-condition setters and ``get_true_pose()`` are round-tripped
    SYNCHRONOUSLY onto the tick thread (``_call_on_tick_thread()``) --
    unlike twist/stop, a caller reading these values needs them to reflect
    a specific, already-applied state, not "eventually applied".
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
registration without a manual try/finally.
"""

from __future__ import annotations

import base64
import ctypes
import pathlib
import queue
import sys
import threading
import time
from contextlib import contextmanager
from typing import TYPE_CHECKING, Any, Callable, Iterator

if TYPE_CHECKING:
    from robot_radio.robot.pb2 import envelope_pb2
    from robot_radio.robot.protocol import TLMFrame

# ---------------------------------------------------------------------------
# Lib path resolution -- same convention the deleted predecessor used
# (io/ -> ../../../../src/sim/build), and the same one
# testgui/transport.py's own _sim_lib_path() independently resolves.
# ---------------------------------------------------------------------------

_LIB_NAME = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
_HERE = pathlib.Path(__file__).parent
_DEFAULT_LIB_PATH = (_HERE / "../../../../src/sim/build" / _LIB_NAME).resolve()

# One sim cycle == 50ms of sim/firmware time (TestSim::SimHarness::kCycleDtUs,
# sim_harness.h). Real-time (1x) tick-thread pacing advances one cycle per
# wall-clock tick by default.
_CYCLE_DURATION_S = 0.050  # [s]

# Telemetry queue: bounded, drop-oldest -- mirrors SerialConnection's own
# _binary_tlm_queue policy (never let an un-drained queue grow unbounded).
_TLM_QUEUE_MAXSIZE = 512

# Ground-truth pose delivered every Nth tick (~5 Hz at 1x speed, matching
# testgui/transport.py's own _SIM_TRUTH_EVERY_N_TICKS/hardware truth-poll
# rate).
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


def _dearmor_reply(line: str, pb2_mod) -> "envelope_pb2.ReplyEnvelope | None":
    """Strip a ``*B`` armor prefix, base64-decode, and parse as a
    ``pb2.ReplyEnvelope``. Returns ``None`` on any malformed input, mirroring
    ``SerialConnection._handle_binary_reply()``'s own tolerance for a single
    corrupted binary reply -- never raises."""
    line = line.strip()
    if not line.startswith("*B"):
        return None
    try:
        raw = base64.b64decode(line[2:])
        return pb2_mod.ReplyEnvelope.FromString(raw)
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

    lib.sim_inject_command.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.sim_inject_command.restype = None

    lib.sim_drain_tlm.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.sim_drain_tlm.restype = ctypes.c_int

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
    lib.sim_set_lead_compensation.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
    lib.sim_set_lead_compensation.restype = None
    lib.sim_set_yaw_rate_max.argtypes = [ctypes.c_void_p, ctypes.c_float]
    lib.sim_set_yaw_rate_max.restype = None

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

    # Commanded per-wheel velocity (velocity-PID setpoint) read live from the
    # firmware -- Path B for the commanded-vs-actual graph (cmd_vel is not on
    # the primary wire frame; see sim_ctypes.cpp's own note).
    lib.sim_cmd_vel_left.argtypes = [ctypes.c_void_p]
    lib.sim_cmd_vel_left.restype = ctypes.c_float
    lib.sim_cmd_vel_right.argtypes = [ctypes.c_void_p]
    lib.sim_cmd_vel_right.restype = ctypes.c_float


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
        self._handle: ctypes.c_void_p | None = None

        self.on_telemetry: "Callable[[TLMFrame], None] | None" = None
        self.on_truth: "Callable[[tuple[float, float, float]], None] | None" = None

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._telemetry_suspended = threading.Event()
        # Queue of plain 0-arg callables -- the tick thread is the only
        # consumer/executor, matching testgui/transport.py's SimTransport
        # convention (see that module's _drain_cmd_queue docstring).
        self._cmd_queue: "queue.Queue[Callable[[], None]]" = queue.Queue()
        self._tlm_queue: "queue.Queue[TLMFrame]" = queue.Queue(maxsize=_TLM_QUEUE_MAXSIZE)

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
        """Fire-and-poll, matching ``NezhaProtocol.stop()``'s own contract."""
        self._require_connected()
        corr_id = self._next_corr_id()
        self._run_or_enqueue(
            lambda: self._lib.sim_inject_stop(self._handle, ctypes.c_uint32(corr_id)))
        return corr_id

    def move(self, *, distance: float = 0.0, delta_heading: float = 0.0,
             v_max: float = 0.0, omega: float = 0.0, time: float = 0.0,
             replace: bool = False, id: "int | None" = None) -> int:
        """MOVE-queue command (109-008 host adoption) -- builds and injects
        ``CommandEnvelope{move: Move{...}}`` via ``inject_command()``'s
        generic escape hatch (the SAME mechanism ``_SimConfigConn.
        send_envelope_fast()`` uses for the config path, Architecture
        Revision 1's "one mechanism, not a Sim-specific fork") rather than a
        dedicated ``sim_inject_move`` ctypes symbol -- unlike
        ``twist()``/``stop()`` (hot teleop-path calls with their own fast
        ctypes entry points), Move is sent at most once per tour LEG, so the
        extra Python-side envelope-build cost here is immaterial.

        ``id`` doubles as both the envelope's own ``corr_id`` (the enqueue
        ack's own correlation key) and ``Move.id`` (the LATER completion
        event's key, per ``envelope.proto``'s own ``Move.id`` doc comment)
        -- mirrors ``NezhaProtocol.move()``'s own convention exactly, so
        ``planner.tour.run_tour()`` can treat a ``NezhaProtocol`` and a
        ``SimLoop`` identically. Defaults to this instance's own
        ``_next_corr_id()`` counter when omitted (matching ``twist()``/
        ``stop()``'s own auto-assignment).

        Returns the id used. Fire-and-poll, matching ``twist()``/``stop()``
        -- this call never blocks on a reply; see ``planner.tour``'s own
        completion-event polling for how a caller learns the outcome.
        """
        self._require_connected()
        move_id = id if id is not None else self._next_corr_id()
        pb2_mod = _get_envelope_pb2()
        envelope = pb2_mod.CommandEnvelope(
            corr_id=move_id,
            move=pb2_mod.Move(distance=distance, delta_heading=delta_heading,
                              v_max=v_max, omega=omega, time=time,
                              replace=replace, id=move_id))
        armored = base64.b64encode(envelope.SerializeToString()).decode("ascii")
        self.inject_command(f"*B{armored}")
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

    def inject_command(self, armored_line: str) -> None:
        """Push an already-armored (``*B...``) line straight onto the
        inbound FakeTransport -- ``sim_inject_command()``'s own escape
        hatch for a wire shape ``twist()``/``stop()`` don't cover."""
        self._require_connected()
        encoded = armored_line.encode("ascii")
        self._run_or_enqueue(
            lambda: self._lib.sim_inject_command(self._handle, encoded))

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

    def set_lead_compensation(self, heading_lead_bias: float, plan_lead: float,
                              terminal_lead: float) -> None:  # [s] [s] [s]
        """109-010: rate-sweep characterization harness hook -- sets the
        three independently-tunable lead-compensation Δt's directly on the
        sim's own ``msg::PlannerConfig`` (``SimHarness::
        setLeadCompensation()``). No wire ``PlannerConfigPatch`` arm exists
        for these fields (boot-baked-default-only per this ticket's own
        scope) -- this sim-only ctypes path is how a test varies them
        against the compiled sim without a reflash/rebuild."""
        self._require_connected()
        self._call_on_tick_thread(
            lambda: self._lib.sim_set_lead_compensation(
                self._handle, ctypes.c_float(heading_lead_bias),
                ctypes.c_float(plan_lead), ctypes.c_float(terminal_lead)))

    def set_yaw_rate_max(self, yaw_rate_max: float) -> None:  # [rad/s]
        """109-010: rate-sweep characterization harness hook -- varies the
        pivot cruise-rate ceiling (``PlannerConfig.yaw_rate_max``) against
        the compiled sim without a reflash/rebuild (``SimHarness::
        setYawRateMax()``)."""
        self._require_connected()
        self._call_on_tick_thread(
            lambda: self._lib.sim_set_yaw_rate_max(self._handle, ctypes.c_float(yaw_rate_max)))

    # ------------------------------------------------------------------
    # Manual stepping (no tick thread required -- ticket 009's shape)
    # ------------------------------------------------------------------

    def step(self, cycles: int = 1) -> None:
        """Advance the sim ``cycles`` cycles (50ms sim-time each) on the
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
        self._require_connected()
        register = self._lib.sim_set_write_hook if is_write else self._lib.sim_set_read_hook

        if cb is None:
            register(self._handle, ctypes.cast(None, _SimHookFn), None)
            if is_write:
                self._write_hook_c = None
                self._write_hook_py = None
            else:
                self._read_hook_c = None
                self._read_hook_py = None
            return

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
        register(self._handle, c_cb, None)
        if is_write:
            self._write_hook_c = c_cb
            self._write_hook_py = cb
        else:
            self._read_hook_c = c_cb
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
                before this fix -- full real-time pace, ``_CYCLE_DURATION_S``
                per cycle, ``_speed_factor`` cycles per iteration.

            IDLE (self._active is False -- the plant confirmed the last
                  motion finished):
                Step/drain/deliver only once every
                ``_IDLE_HEARTBEAT_INTERVAL_S`` (~2s) instead of every
                ``_CYCLE_DURATION_S`` (~50ms) -- a slow "I'm still here"
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
            # per 50ms wall = N x real-time (the previous `* cycles` here paced N
            # cycles to N*50ms wall, i.e. always 1x, so speed_factor did nothing).
            # At a high enough speed_factor the compute for N cycles exceeds one
            # cycle's wall budget and this sleeps 0 -- the sim then free-runs at
            # full compute speed (what the "fast tour" wants).
            elapsed = time.monotonic() - t0
            sleep_s = _CYCLE_DURATION_S - elapsed
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

        buf = ctypes.create_string_buffer(_TLM_DRAIN_BUFFER)
        needed = self._lib.sim_drain_tlm(self._handle, buf, _TLM_DRAIN_BUFFER)
        if needed >= _TLM_DRAIN_BUFFER:
            # Truncated -- retry once with an exactly-sized buffer (the
            # drain already advanced regardless, per sim_ctypes.cpp's own
            # snprintf-return-value convention, so this is a fresh drain
            # of whatever accumulated since, not a re-fetch of the lost
            # data).
            buf = ctypes.create_string_buffer(needed + 1)
            self._lib.sim_drain_tlm(self._handle, buf, needed + 1)
        joined = buf.value.decode("utf-8", errors="replace")
        if not joined:
            return

        pb2_mod = _get_envelope_pb2()
        suspended = self._telemetry_suspended.is_set()
        for line in joined.split("\n"):
            if not line:
                continue
            reply = _dearmor_reply(line, pb2_mod)
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
