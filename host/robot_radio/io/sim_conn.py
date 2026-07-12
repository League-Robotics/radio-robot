"""SimConnection — SerialConnection-compatible sim backend.

Drop-in replacement for SerialConnection that drives libfirmware_host
(source/dev_loop.h's DevLoop + CommandProcessor, via ticket 081-004's
sim_api.cpp C ABI) over ctypes instead of a serial port.

Usage::

    from robot_radio.io.sim_conn import SimConnection

    conn = SimConnection()
    conn.connect()
    conn.send("DEV M 1 VEL 120", read_timeout=200)
    conn.tick(3000)
    df = conn.state_df()   # pandas DataFrame of time-series state

The sim backend advances wall-clock time explicitly: every call to
read_lines() advances the simulation by the requested duration in small
steps, collecting async EVTs and recording per-step state (velocities,
encoder positions, true/OTOS pose).  This makes time-series analysis
trivially easy.

Thread safety: not thread-safe.  Use from one thread.

--- Reconciliation against ticket 081-004's REAL ABI (sprint 081, ticket
005) ---

This module's ctypes bindings and injection helpers previously targeted a
~28-symbol contract inherited from the OLD (pre-077-greenfield-rebuild)
tree's Robot/MockHAL/EKF-fusion model. That model does not exist in this
dev-bench tree: source/subsystems/drivetrain.h has no odometry this sprint,
and there is no EKF/fusion loop anywhere in source/ to feed a "pose"
estimate or an injected OTOS reading into. This module has been reconciled
against the ACTUAL 40-symbol ABI ticket 004 exports
(tests/_infra/sim/sim_api.cpp) instead. Concretely:

  - "Fused pose" (the old sim_get_pose_x/y/h) has no equivalent -- dropped
    from state_log/_snapshot(). The closest available concepts are
    get_true_pose() (Hal::PhysicsWorld ground truth; sim_get_true_pose_*)
    and get_otos_pose() (Hal::SimOdometer's errored accumulator;
    sim_get_otos_*) -- both already exposed below. get_exact_pose() is
    kept as a synonym for get_true_pose() (sim_api.cpp's own legacy-alias
    naming: sim_get_exact_pose_x/y/h reads the identical true-pose data).
  - set_motor_offset() (PhysicsWorld::setOffsetFactor()) and
    set_otos_pose() (an injected-reading hook for the old EKF's Mahalanobis
    gate) have NO backing ctypes entry point in this ABI -- ticket 004's
    own closing notes call out setOffsetFactor() as "deliberately left
    unwrapped." Both raise NotImplementedError with a message pointing
    here, rather than failing with an opaque AttributeError, so a future
    caller (or the TestGUI/SimTransport revival referenced in the sprint's
    Open Question 5 -- explicitly out of THIS ticket's scope) knows
    exactly what is missing and why.
  - enable_otos_model() is now a documented no-op: the new Hal::SimOdometer
    always accumulates every pass (Subsystems::SimHardware::tick() calls
    its tick() unconditionally) -- there is no separate "enable" step to
    wire.
  - enable_otos_fusion() raises NotImplementedError -- there is no EKF/
    fusion loop in source/ this sprint to enable at all (not merely
    "disabled by default").
  - set_slip()'s `turn_extra` parameter has no backing knob: the new
    Hal::PhysicsWorld exposes only setEncoderSlip(side, fraction) over
    ctypes (sim_set_enc_slip) -- a flat per-wheel fraction, not the old
    model's separate straight/turn-rate-dependent split
    (PhysicsWorld::setSlip(straight, turnExtra) exists in physics_world.h
    but is not wired into sim_api.cpp's ABI). `turn_extra` is accepted for
    call-signature compatibility and warns (via warnings.warn) if nonzero,
    rather than silently discarding it.
  - set_enc()'s semantics changed: the old sim_set_enc_l/r pair (which set
    the REPORTED-only accumulator, distinct from ground truth) has no
    equivalent; the only wheel-travel injection point left is
    sim_set_true_wheel_travel (GROUND TRUTH). set_enc() now calls that --
    documented explicitly on the method itself, since silently redefining
    an existing method's semantics is exactly the kind of gap a future
    reviver needs to know about up front.
  - sim_set_motor_slip/sim_set_encoder_noise (single call, both wheels) map
    onto this ABI's more granular sim_set_enc_slip/sim_set_enc_noise
    (explicit `side` parameter: 0=left, 1=right, 2=both) -- both
    convenience wrappers below call the new entry points with side=2.
  - New ABI-only knobs with no OLD equivalent at all (stiction, motor lag,
    trackwidth, body rotational/linear scrub, OTOS scale error/drift, and
    the true-vs-reported read split) are exposed as new methods below,
    named to match tests/_infra/sim/firmware.py's Sim wrapper (ticket
    081-005) for consistency across the two Python ABI clients.
"""

from __future__ import annotations

import base64
import ctypes
import pathlib
import sys
import time
import warnings
from typing import Any, TYPE_CHECKING

if TYPE_CHECKING:
    # Type-checking only -- see _get_envelope_pb2() below for why the
    # runtime import is deferred past module-load time (this module's own
    # reason differs from serial_conn.py's -- no circular-import hazard
    # here -- but the lazy-import SHAPE is kept the same for consistency
    # with that module's binary-plane reference implementation).
    from robot_radio.robot.pb2 import envelope_pb2

_LIB_NAME = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
_HERE = pathlib.Path(__file__).parent
# Resolve the dylib relative to this file: host/robot_radio/io/ -> ../../.. = repo root
_DEFAULT_LIB = (_HERE / "../../../tests/_infra/sim/build" / _LIB_NAME).resolve()

# Default tick step: 24 ms matches tests/sim/conftest.py's fixture and is
# fine for the ~25 ms control period.  Smaller = smoother state log, more CPU.
_DEFAULT_TICK_DURATION = 24

# Subsystems::Channel's own enum values (source/subsystems/wire_command.h:
# `enum class Channel : uint8_t { NONE, SERIAL, RADIO };`) -- mirrored here
# so callers of send_on()/sim_command_on() can select a channel without
# reaching into the C++ enum directly (088-006).
CHANNEL_SERIAL = 1
CHANNEL_RADIO = 2

# Module-level cache for the lazily-imported envelope_pb2 module (see
# _get_envelope_pb2()'s docstring for why this cannot be a top-level import).
_envelope_pb2_module = None


def _get_envelope_pb2():
    """Lazily import and cache robot_radio.robot.pb2.envelope_pb2.

    Mirrors serial_conn.py's own ``_get_envelope_pb2()`` helper (095-002),
    deferred past module-load time for a DIFFERENT reason than that
    module's: serial_conn.py has a genuine circular-import hazard
    (robot_radio.robot's own __init__.py imports robot_radio.robot.protocol,
    which imports SerialConnection from THAT module). This module
    (robot_radio.io.sim_conn) has no such hazard -- nothing in
    robot_radio.robot's own import chain ever imports
    robot_radio.io.sim_conn -- but importing robot_radio.robot.pb2 still
    transitively imports the WHOLE robot_radio.robot package (Robot,
    NezhaProtocol, pyserial via robot_radio.io.serial_conn, ...), which the
    ctypes-only sim harness (tests/_infra/sim/, tests/sim/) has no other
    reason to pull in. Deferring the import to first use (rather than
    module load) keeps a bare ``import robot_radio.io.sim_conn`` a
    lightweight, ctypes-only operation for callers that never touch the
    binary plane -- the same practical benefit serial_conn.py's helper
    gets, via the same lazy-import shape, for a different root cause.
    """
    global _envelope_pb2_module
    if _envelope_pb2_module is None:
        from robot_radio.robot.pb2 import envelope_pb2 as _mod
        _envelope_pb2_module = _mod
    return _envelope_pb2_module


def _dearmor_reply(line: str, pb2_mod) -> "envelope_pb2.ReplyEnvelope | None":
    """Strip a ``*B`` armor prefix, base64-decode, and parse the result as a
    ``pb2.ReplyEnvelope``. Returns ``None`` on any malformed input (missing
    prefix, bad base64, bad protobuf bytes) instead of raising -- mirrors
    ``SerialConnection._handle_binary_reply()``'s own tolerance for a single
    corrupted binary reply (a decode failure there is swallowed and the line
    dropped, never crashes the caller)."""
    line = line.strip()
    if not line.startswith("*B"):
        return None
    try:
        raw = base64.b64decode(line[2:])
        return pb2_mod.ReplyEnvelope.FromString(raw)
    except Exception:
        return None


class SimConnection:
    """SerialConnection-compatible backend using libfirmware_host.

    Supports the same interface as SerialConnection:
      - connect() / disconnect()
      - send(message, read_timeout, stop_token) -> dict
      - send_fast(message) -> None
      - read_lines(duration, stop_token) -> list[str]
      - is_open property
      - mode property

    Extra sim-only interface:
      - tick(ms) -> list[str]
          Advance sim time, return EVT lines, record state.
      - state_log  -> list[dict]
          All state snapshots recorded during ticking.
      - state_df() -> pd.DataFrame
          state_log as a DataFrame (requires pandas).
      - clear_state_log()
          Reset the log.
    """

    def __init__(self, lib_path: str | pathlib.Path | None = None,
                 tick_step: int = _DEFAULT_TICK_DURATION,  # [ms]
                 real_time: bool = False,
                 speed_factor: float = 1.0) -> None:
        self._lib_path = pathlib.Path(lib_path) if lib_path else _DEFAULT_LIB
        self._tick_step = tick_step
        self._real_time = real_time
        self._speed_factor = speed_factor
        self._lib: ctypes.CDLL | None = None
        self._h: ctypes.c_void_p | None = None
        self._t: int = 0
        self._state_log: list[dict[str, float]] = []

    # ------------------------------------------------------------------
    # SerialConnection-compatible interface
    # ------------------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._h is not None

    @property
    def mode(self) -> str | None:
        return "sim" if self.is_open else None

    def connect(self, skip_ping: bool = False, **_: Any) -> dict[str, Any]:
        """Load the shared library, create a SimHandle, optionally PING.

        Parameters match SerialConnection.connect() so callers are portable.
        """
        if self.is_open:
            return {"status": "already_connected", "mode": "sim"}

        if not self._lib_path.exists():
            return {
                "error": f"Sim library not found at {self._lib_path}. "
                         f"Run: just build-sim",
                "lib_path": str(self._lib_path),
            }

        self._lib = ctypes.CDLL(str(self._lib_path))
        self._setup_types()
        self._h = self._lib.sim_create()
        self._t = 0
        self._state_log = []

        if not self._h:
            self._lib = None
            return {"error": "sim_create() returned NULL"}

        if not skip_ping:
            resp = self.send("PING", read_timeout=200)
            if not any("pong" in l for l in resp.get("responses", [])):
                return {"error": "PING failed — sim may not have initialised"}

        return {"status": "connected", "mode": "sim",
                "lib": str(self._lib_path)}

    def disconnect(self) -> dict[str, Any]:
        if not self.is_open:
            return {"status": "not_connected"}
        self._lib.sim_destroy(self._h)
        self._h = None
        self._lib = None
        return {"status": "disconnected", "ticks": self._t}

    def send(self, message: str,
             read_timeout: int = 500,  # [ms]
             stop_token: str | None = "OK") -> dict[str, Any]:
        """Send a command; advance sim for read_timeout collecting EVTs.

        Mirrors SerialConnection.send() return shape:
            {"sent": ..., "mode": "sim", "responses": [line, ...]}

        For non-blocking commands (PING, DEV M/DT, ...) the firmware
        replies OK immediately.  The stop_token="OK" default means
        _advance() exits as soon as OK is in the response — before ticking
        — so no sim time is consumed for fast commands. For long-running
        reads (EVT-wait loops via read_lines()) callers pass stop_token
        explicitly.
        """
        if not self.is_open:
            return {"error": "Not connected. Call connect() first."}

        sync = self._raw_command(message)
        lines: list[str] = [l for l in sync.strip().split("\n") if l.strip()] if sync else []

        # Collect additional EVTs by advancing time; stop early on stop_token.
        evts = self._advance(read_timeout, stop_token, existing_lines=lines)
        lines.extend(evts)

        return {"sent": message, "mode": "sim", "responses": lines}

    def send_fast(self, message: str) -> None:
        """Fire-and-forget: dispatch the command, consume no sim time."""
        if not self.is_open:
            raise ConnectionError("Not connected. Call connect() first.")
        self._raw_command(message)

    def send_on(self, message: str, channel: int,
                read_timeout: int = 500,  # [ms]
                stop_token: str | None = "OK") -> dict[str, Any]:
        """Like send(), but selects the reply channel explicitly (088-006).

        `channel` is CHANNEL_SERIAL or CHANNEL_RADIO (module constants,
        mirroring Subsystems::Channel). Routes the command with that
        returnPath and reads the reply back from the MATCHING channel's
        sim-side ReplyStore (sim_command_on() — tests/_infra/sim/
        sim_api.cpp) — proves a command dispatches/replies correctly on a
        specific channel, which the plain send()/SERIAL-only path cannot.
        """
        if not self.is_open:
            return {"error": "Not connected. Call connect() first."}

        sync = self._raw_command_on(message, channel)
        lines: list[str] = [l for l in sync.strip().split("\n") if l.strip()] if sync else []

        evts = self._advance(read_timeout, stop_token, existing_lines=lines)
        lines.extend(evts)

        return {"sent": message, "mode": "sim", "channel": channel, "responses": lines}

    def send_envelope(self, envelope: "envelope_pb2.CommandEnvelope",
                      read_timeout: int = 500,  # [ms] accepted for call-site
                                                 # parity with SerialConnection
                                                 # .send_envelope(); unused --
                                                 # see docstring.
                      channel: int = CHANNEL_SERIAL,
                      ) -> "envelope_pb2.ReplyEnvelope | None":
        """Send a binary ``pb2.CommandEnvelope`` through the sim's dt=0
        synchronous command channel; return its decoded ``pb2.ReplyEnvelope``.

        The sim-side counterpart of ``SerialConnection.send_envelope()``
        (the hardware binary-plane sender, ``robot_radio/io/serial_conn.py``
        -- read as the reference pattern this mirrors): serializes
        ``envelope``, base64-armors it as ``*B<base64>``, and dispatches it
        through ``_raw_command_on()`` -- the SAME ``sim_command_on()`` C
        entry point ``send()``/``send_on()`` already use for the text plane
        -- then dearmors and decodes the single synchronous reply line it
        returns.

        Simplification vs. the hardware version (documented per this
        ticket's own instruction to flag any simplification):
        ``SerialConnection.send_envelope()`` manages its own corr-id pool
        and blocks on a corr-id-keyed queue that a BACKGROUND READER THREAD
        fills, because over a real serial link commands and replies are
        genuinely asynchronous and can interleave with unrelated traffic
        (another in-flight request, a push frame, ...). The sim has neither
        a reader thread nor any interleaving: ``_raw_command_on()`` is a
        single, synchronous, in-process C call that returns only once
        ``sim_command_on()`` has already written THIS call's own (and only
        this call's own) reply into the target channel's ReplyStore --
        there is no other in-flight request whose reply could arrive first,
        so no corr-id-keyed queue is needed at all. Accordingly this method
        does **not** overwrite ``envelope.corr_id`` the way
        ``SerialConnection.send_envelope()`` does -- whatever ``corr_id``
        the caller set (0 if unset) is sent as-is and echoed back unchanged
        on the reply, matching every ``send()``/``dearmor()`` helper in
        ``tests/sim/unit/test_binary_channel.py``/``_binary_envelope.py``,
        which set ``corr_id`` explicitly on each envelope they build.
        ``read_timeout`` is accepted only for call-site portability with
        code written against ``SerialConnection``'s signature -- the sim
        call is already synchronous (it returns only once the one reply is
        available), so there is nothing to wait on and the argument is
        ignored.

        Args:
            envelope: A populated ``pb2.CommandEnvelope``. Its ``corr_id``
                is sent as-is (see above) -- NOT overwritten.
            read_timeout: Ignored -- accepted only for signature parity
                with ``SerialConnection.send_envelope()``.
            channel: ``CHANNEL_SERIAL`` (default) or ``CHANNEL_RADIO`` --
                selects which of the sim's two independent ReplyStores the
                reply is read back from (mirrors ``send_on()``'s own
                ``channel`` parameter, 088-006).

        Returns:
            The decoded ``pb2.ReplyEnvelope``, or ``None`` if the sim is
            not connected, produced no reply line at all, or produced a
            line that could not be dearmored/parsed.
        """
        del read_timeout  # unused -- see docstring
        if not self.is_open:
            return None

        pb2 = _get_envelope_pb2()
        armored = "*B" + base64.b64encode(envelope.SerializeToString()).decode("ascii")
        reply_line = self._raw_command_on(armored, channel)
        return _dearmor_reply(reply_line, pb2)

    def drain_binary_tlm(self, channel: int = CHANNEL_SERIAL,
                         ) -> list["envelope_pb2.ReplyEnvelope"]:
        """Destructively drain ``channel``'s ReplyStore of every unsolicited
        binary telemetry push frame (``ReplyEnvelope{tlm}``, always
        ``corr_id=0``) accumulated there since the last drain -- or since
        the last ``send()``/``send_on()``/``send_envelope()`` call on ANY
        channel, since ``sim_command_on()`` resets BOTH channels'
        ReplyStores as a side effect of routing (``sim_api.cpp``'s "Two
        reply-store instances" file-header note).

        The sim-side counterpart of ``SerialConnection.drain_binary_tlm()``:
        that method drains a bounded, drop-oldest queue a background reader
        thread fills continuously as frames arrive; this one drains the
        sim's own fixed-size ReplyStore via ``sim_drain_reply_store()`` --
        a NEW ``tests/_infra/sim/sim_api.cpp`` ABI entry point this ticket
        adds. Neither existing sim ABI accessor was enough on its own: the
        pre-existing ``sim_peek_reply_store()`` is non-destructive, so a
        caller that only ever peeks lets ``tickTelemetry()``'s periodic
        frames accumulate in the store until it silently overflows
        (``ReplyStore::append()``'s own "once full, every further append is
        a no-op" behavior -- see ``sim_api.cpp``); ``sim_command()``/
        ``sim_command_on()`` DO reset a store, but only as an incidental
        side effect of routing an unrelated command, and reset BOTH
        channels' stores unconditionally, which would also silently wipe
        out whatever the OTHER channel had pending. ``sim_drain_reply_store``
        resets only the ONE channel it drains, with no command routed at
        all -- see that entry point's own doc comment.

        Both this method and its hardware counterpart return the raw,
        decoded ``pb2.ReplyEnvelope`` -- callers build a ``TLMFrame``
        themselves via ``TLMFrame.from_pb2(reply.tlm)``
        (``robot_radio/robot/protocol.py``), matching
        ``SerialConnection.drain_binary_tlm()``'s own "raw envelope, caller
        parses" split.

        Call this FREQUENTLY once binary streaming is armed (a binary
        ``stream`` command / ``StreamControl{binary:true, period:...}``):
        each channel's ReplyStore is a small (2048-byte) FIXED buffer with
        no wraparound, so an undrained store freezes after roughly 10-14
        periodic frames and stops reflecting current state (see above).

        Only ``tlm``-body frames are ever returned -- any other body left
        behind in the store by a caller that mixed ``send()``/
        ``send_envelope()`` polling with tick()-only draining is silently
        excluded, matching ``SerialConnection.drain_binary_tlm()``'s own
        "only tlm bodies ever land here" contract.

        Args:
            channel: ``CHANNEL_SERIAL`` (default) or ``CHANNEL_RADIO``.

        Returns:
            A list of decoded ``pb2.ReplyEnvelope`` objects (body ``tlm``),
            in the order ``tickTelemetry()`` appended them; empty if the
            sim is not connected or none were pending.
        """
        if not self.is_open:
            return []

        pb2 = _get_envelope_pb2()
        raw = self._raw_drain_reply_store(channel)
        frames: list = []
        for line in raw.splitlines():
            line = line.strip()
            if not line:
                continue
            reply = _dearmor_reply(line, pb2)
            if reply is not None and reply.WhichOneof("body") == "tlm":
                frames.append(reply)
        return frames

    def read_lines(self, duration: int = 500,  # [ms]
                   stop_token: str | None = None) -> list[str]:
        """Tick the sim for duration, collecting and returning EVT lines."""
        if not self.is_open:
            return []
        return self._advance(duration, stop_token)

    def read_pending_lines(self) -> list[str]:
        """Non-blocking drain — always empty in sim (no buffered input concept).

        The sim has no equivalent of the serial TLM/EVT queues: all sim output
        is produced synchronously by _raw_command() or _advance().  Callers
        that expect a non-blocking poll get an empty list, which is correct —
        there is nothing waiting.
        """
        return []

    # ------------------------------------------------------------------
    # Sim-only interface
    # ------------------------------------------------------------------

    def tick(self, ms: int) -> list[str]:
        """Advance sim time by ms milliseconds; return EVT lines emitted."""
        if not self.is_open:
            return []
        return self._advance(ms, stop_token=None)

    # ------------------------------------------------------------------
    # Sim state injection helpers (no firmware command needed)
    # ------------------------------------------------------------------

    def set_motor_offset(self, side: int, factor: float) -> None:
        """No ctypes entry point in this ABI.

        Hal::PhysicsWorld::setOffsetFactor() exists in physics_world.h but
        is deliberately NOT wrapped by sim_api.cpp (ticket 081-004's own
        closing notes) -- there is no wire-free way to reach it yet. Raises
        NotImplementedError rather than silently no-op'ing; a future ticket
        that needs this knob adds the ctypes entry point first.
        """
        raise NotImplementedError(
            "set_motor_offset: no sim_set_motor_offset ABI entry point in "
            "this tree (Hal::PhysicsWorld::setOffsetFactor() is unwrapped "
            "by ticket 081-004's sim_api.cpp) — see this module's docstring."
        )

    def set_enc(self, left: float, right: float) -> None:  # [mm]
        """Inject GROUND-TRUTH wheel travel (semantics changed from the old
        ABI — see this module's docstring).

        The old sim_set_enc_l/r pair injected the REPORTED-only
        accumulator, distinct from ground truth. This ABI has no such
        reported-only setter; the only wheel-travel injection point is
        sim_set_true_wheel_travel (Hal::PhysicsWorld's TRUE accumulator).
        Prefer set_true_wheel_travel() directly in new code — this name is
        kept only for call-site compatibility with existing callers.
        """
        self.set_true_wheel_travel(left, right)

    def set_otos_pose(self, x: float, y: float, h_rad: float) -> None:
        """No ctypes entry point in this ABI.

        The old sim_set_otos_pose() fed a deliberately-bad reading into the
        firmware's EKF Mahalanobis gate — there is no EKF/fusion loop in
        source/ this sprint (see this module's docstring) for such a
        reading to feed. Raises NotImplementedError.
        """
        raise NotImplementedError(
            "set_otos_pose: no sim_set_otos_pose ABI entry point in this "
            "tree (no EKF/fusion loop exists to feed an injected reading "
            "into) — see this module's docstring."
        )

    def get_exact_pose(self) -> dict:
        """Return oracle ground-truth pose (Hal::PhysicsWorld true pose).

        Returns {"x": mm, "y": mm, "h": rad}. Synonym for get_true_pose() —
        sim_api.cpp's sim_get_exact_pose_x/y/h are legacy aliases reading
        the identical true-pose data as sim_get_true_pose_x/y/h.
        """
        return self.get_true_pose()

    def get_true_pose(self) -> dict:
        """Return (x, y, h) ground-truth chassis pose from Hal::PhysicsWorld.

        Returns {"x": mm, "y": mm, "h": rad}.
        """
        if not self.is_open:
            raise ConnectionError("Not connected")
        lib, h = self._lib, self._h
        return {"x": float(lib.sim_get_true_pose_x(h)),
                "y": float(lib.sim_get_true_pose_y(h)),
                "h": float(lib.sim_get_true_pose_h(h))}

    def get_true_wheel_travel(self) -> tuple[float, float]:
        """Return (enc_l, enc_r) true (unslipped) per-wheel travel. [mm]"""
        if not self.is_open:
            raise ConnectionError("Not connected")
        lib, h = self._lib, self._h
        return (float(lib.sim_get_true_enc_l(h)), float(lib.sim_get_true_enc_r(h)))

    def get_true_velocity(self) -> tuple[float, float]:
        """Return (vel_l, vel_r) true per-wheel velocity. [mm/s]"""
        if not self.is_open:
            raise ConnectionError("Not connected")
        lib, h = self._lib, self._h
        return (float(lib.sim_get_true_vel_l(h)), float(lib.sim_get_true_vel_r(h)))

    def set_true_wheel_travel(self, enc_l: float, enc_r: float) -> None:  # [mm] [mm]
        """Set the plant's TRUE per-wheel travel accumulators directly."""
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_true_wheel_travel(self._h, ctypes.c_float(enc_l),
                                            ctypes.c_float(enc_r))

    def set_true_pose(self, x: float, y: float, heading: float) -> None:  # [mm] [mm] [rad]
        """Set the plant's TRUE chassis pose directly."""
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_true_pose(self._h, ctypes.c_float(x), ctypes.c_float(y),
                                    ctypes.c_float(heading))

    def set_slip(self, straight: float = 0.005, turn_extra: float = 0.03) -> None:
        """Apply an encoder-slip fraction to both wheels.

        straight: fractional slip applied to the REPORTED encoder (e.g.
            0.005 = 0.5% under-report) via sim_set_enc_slip(side=2, ...).
        turn_extra: NO backing knob in this ABI (see this module's
            docstring) — accepted only for call-signature compatibility.
            Warns via warnings.warn if nonzero rather than silently
            discarding it.
        """
        if turn_extra:
            warnings.warn(
                "SimConnection.set_slip(): turn_extra has no ABI backing in "
                "this tree (no sim_set_slip/turn-rate-dependent knob is "
                "wired) and is being ignored — see this module's docstring.",
                stacklevel=2,
            )
        self.set_enc_slip(2, straight)

    def set_enc_slip(self, side: int, fraction: float) -> None:
        """Set the REPORTED encoder-slip fraction. side: 0=left, 1=right, 2=both."""
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_enc_slip(self._h, ctypes.c_int(side), ctypes.c_float(fraction))

    def set_enc_scale_error(self, side: int, err: float) -> None:
        """Set the REPORTED encoder scale error (fractional over/under-report)."""
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_enc_scale_error(self._h, ctypes.c_int(side), ctypes.c_float(err))

    def set_encoder_noise(self, sigma: float = 0.05) -> None:  # [mm]
        """Apply Gaussian encoder noise to both wheels.

        sigma: standard deviation of per-tick encoder noise in mm.
        """
        self.set_enc_noise(2, sigma)

    def set_enc_noise(self, side: int, sigma: float) -> None:  # [mm]
        """Set per-side encoder noise sigma. side: 0=left, 1=right, 2=both."""
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_enc_noise(self._h, ctypes.c_int(side), ctypes.c_float(sigma))

    def set_stiction(self, side: int, pwm: float) -> None:
        """Set the per-wheel PWM stiction/breakaway dead-zone threshold."""
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_stiction(self._h, ctypes.c_int(side), ctypes.c_float(pwm))

    def set_motor_lag(self, side: int, tau: float) -> None:  # [ms]
        """Set the per-wheel first-order motor-response time constant."""
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_motor_lag(self._h, ctypes.c_int(side), ctypes.c_float(tau))

    def set_trackwidth(self, trackwidth: float) -> None:  # [mm]
        """Set the plant's chassis trackwidth."""
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_trackwidth(self._h, ctypes.c_float(trackwidth))

    def set_body_rotational_scrub(self, scrub: float) -> None:
        """Set the plant's independent body-rotational scrub (1.0 = no-op)."""
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_body_rotational_scrub(self._h, ctypes.c_float(scrub))

    def set_body_linear_scrub(self, scrub: float) -> None:
        """Set the plant's independent body-linear scrub (1.0 = no-op)."""
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_body_linear_scrub(self._h, ctypes.c_float(scrub))

    def enable_otos_model(self) -> None:
        """No-op: Hal::SimOdometer always accumulates every pass in this
        tree (Subsystems::SimHardware::tick() calls it unconditionally) --
        there is no separate "enable" step to wire, unlike the old model.
        Kept as a no-op (not removed) so existing call sites do not need to
        change to keep working.
        """
        return None

    def enable_otos_fusion(self, on: bool = True) -> None:
        """No ctypes entry point in this ABI.

        There is no EKF/fusion loop anywhere in source/ this sprint (see
        this module's docstring) — not merely "disabled by default".
        Raises NotImplementedError.
        """
        raise NotImplementedError(
            "enable_otos_fusion: no EKF/fusion loop exists in this tree to "
            "enable — see this module's docstring."
        )

    def set_otos_noise(self, linear: float = 0.01, yaw: float = 0.025) -> None:
        """Set OTOS noise sigmas.

        linear: fractional standard deviation for linear-position noise.
        yaw: standard deviation for yaw noise, [rad].
        """
        self.set_otos_linear_noise(linear)
        self.set_otos_yaw_noise(yaw)

    def set_otos_linear_noise(self, sigma: float) -> None:  # [mm]
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_otos_linear_noise(self._h, ctypes.c_float(sigma))

    def set_otos_yaw_noise(self, sigma: float) -> None:  # [rad]
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_otos_yaw_noise(self._h, ctypes.c_float(sigma))

    def set_otos_linear_scale_error(self, err: float) -> None:
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_otos_linear_scale_error(self._h, ctypes.c_float(err))

    def set_otos_angular_scale_error(self, err: float) -> None:
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_otos_angular_scale_error(self._h, ctypes.c_float(err))

    def set_otos_linear_drift(self, drift: float) -> None:  # [mm]
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_otos_linear_drift(self._h, ctypes.c_float(drift))

    def set_otos_yaw_drift(self, drift: float) -> None:  # [rad]
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_otos_yaw_drift(self._h, ctypes.c_float(drift))

    def get_otos_pose(self) -> dict:
        """Return accumulated OTOS odometry pose (SimOdometer, errored).

        Returns {"x": mm, "y": mm, "h": rad}.
        """
        if not self.is_open:
            raise ConnectionError("Not connected")
        lib, h = self._lib, self._h
        return {"x": float(lib.sim_get_otos_x(h)),
                "y": float(lib.sim_get_otos_y(h)),
                "h": float(lib.sim_get_otos_h(h))}

    def clear_state_log(self) -> None:
        """Clear the accumulated state log."""
        self._state_log.clear()

    @property
    def state_log(self) -> list[dict[str, float]]:
        """Time-series state recorded during ticking.

        Each entry: {time, vel_l, vel_r, enc_l, enc_r, pwm_l, pwm_r,
        true_pose_x, true_pose_y, true_pose_h, otos_x, otos_y, otos_h}
        (``time`` in ms; poses in mm/rad). No "fused pose" field — see this
        module's docstring (no EKF/fusion loop in this tree).
        """
        return self._state_log

    def state_df(self):
        """Return state_log as a pandas DataFrame.

        Requires pandas to be installed.  Columns: see state_log's own doc.
        """
        import pandas as pd  # local import — not a hard dependency
        return pd.DataFrame(self._state_log)

    def get_state(self) -> dict[str, float]:
        """Return a single current-state snapshot (does not record to log)."""
        if not self.is_open:
            return {}
        return self._snapshot()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _raw_command(self, line: str) -> str:
        """Send one command to the C sim; return the synchronous reply.

        Buffer is 2048 bytes — matches sim_command()'s C-side ReplyStore
        capacity (kReplyBufSize in sim_api.cpp), so long synchronous replies
        are not silently truncated.
        """
        buf = ctypes.create_string_buffer(2048)
        n = self._lib.sim_command(self._h, line.encode(), buf, 2048)
        return buf.raw[:n].decode(errors="replace") if n > 0 else ""

    def _raw_command_on(self, line: str, channel: int) -> str:
        """Like _raw_command(), but selects the reply channel (088-006).

        Same 2048-byte buffer convention as _raw_command() — matches
        sim_command_on()'s C-side ReplyStore capacity.
        """
        buf = ctypes.create_string_buffer(2048)
        n = self._lib.sim_command_on(self._h, line.encode(), ctypes.c_int(channel), buf, 2048)
        return buf.raw[:n].decode(errors="replace") if n > 0 else ""

    def _raw_drain_reply_store(self, channel: int) -> str:
        """Destructively read and clear one channel's ReplyStore
        (sim_drain_reply_store() -- tests/_infra/sim/sim_api.cpp, added for
        drain_binary_tlm() above). Same 2048-byte buffer convention as
        _raw_command()/_raw_command_on().
        """
        buf = ctypes.create_string_buffer(2048)
        n = self._lib.sim_drain_reply_store(self._h, ctypes.c_int(channel), buf, 2048)
        return buf.raw[:n].decode(errors="replace") if n > 0 else ""

    def _get_evts(self) -> str:
        """Drain async EVT buffer from the C sim."""
        buf = ctypes.create_string_buffer(2048)
        n = self._lib.sim_get_async_evts(self._h, buf, 2048)
        return buf.raw[:n].decode(errors="replace") if n > 0 else ""

    def _advance(self, total: int, stop_token: str | None = None,  # [ms]
                 existing_lines: list[str] | None = None) -> list[str]:
        """Tick the sim for total ms, recording state each step.

        Returns EVT lines accumulated during the advance.  If stop_token
        is set, returns as soon as a line containing stop_token is seen.
        existing_lines is checked first; if stop_token already satisfied,
        returns immediately without ticking (fast path for OK commands).
        """
        lines: list[str] = []

        # Fast path: stop_token already satisfied by the sync reply.
        if stop_token and existing_lines and any(stop_token in l for l in existing_lines):
            return lines

        step = self._tick_step
        end_t = self._t + total

        while self._t < end_t:
            dt = min(step, end_t - self._t)
            self._t += dt
            self._lib.sim_tick(self._h, ctypes.c_uint32(self._t))
            self._state_log.append(self._snapshot())
            if self._real_time:
                time.sleep(dt / 1000.0 / self._speed_factor)

            evts = self._get_evts()
            if evts:
                for ln in evts.strip().split("\n"):
                    ln = ln.strip()
                    if ln:
                        lines.append(ln)
                if stop_token and any(stop_token in l for l in lines):
                    break

        return lines

    def _snapshot(self) -> dict[str, float]:
        """Read all sim state getters into a single dict.

        No "pose_x/y/h" (firmware fused/dead-reckoned estimate) — that
        concept has no backing ABI symbol in this tree (see this module's
        docstring); true_pose_x/y/h (ground truth) and otos_x/y/h (the
        errored OTOS accumulator) are the two available estimates instead.
        """
        lib, h = self._lib, self._h
        return {
            "time":         float(self._t),  # [ms]
            "vel_l":        float(lib.sim_get_vel_l(h)),
            "vel_r":        float(lib.sim_get_vel_r(h)),
            "enc_l":        float(lib.sim_get_enc_l(h)),
            "enc_r":        float(lib.sim_get_enc_r(h)),
            "pwm_l":        float(lib.sim_get_pwm_l(h)),
            "pwm_r":        float(lib.sim_get_pwm_r(h)),
            "true_pose_x":  float(lib.sim_get_true_pose_x(h)),
            "true_pose_y":  float(lib.sim_get_true_pose_y(h)),
            "true_pose_h":  float(lib.sim_get_true_pose_h(h)),
            "otos_x":       float(lib.sim_get_otos_x(h)),
            "otos_y":       float(lib.sim_get_otos_y(h)),
            "otos_h":       float(lib.sim_get_otos_h(h)),
        }

    def _setup_types(self) -> None:
        """Bind every sim_* symbol ticket 081-004 exports (40 total) —
        see tests/_infra/sim/sim_api.cpp for the authoritative signatures."""
        lib = self._lib

        # --- Lifecycle / loop (4) ---
        lib.sim_create.argtypes = []
        lib.sim_create.restype = ctypes.c_void_p

        lib.sim_destroy.argtypes = [ctypes.c_void_p]
        lib.sim_destroy.restype = None

        lib.sim_tick.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        lib.sim_tick.restype = None

        lib.sim_command.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
        lib.sim_command.restype = ctypes.c_int

        # sim_command_on (088-006) -- sim_command's argtypes plus one
        # c_int channel selector (CHANNEL_SERIAL/CHANNEL_RADIO, above).
        lib.sim_command_on.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
        lib.sim_command_on.restype = ctypes.c_int

        # --- Async (1) ---
        lib.sim_get_async_evts.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        lib.sim_get_async_evts.restype = ctypes.c_int

        # --- Ground truth (12) ---
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

        # --- Errored observation (9) ---
        for name in (
            "sim_get_enc_l", "sim_get_enc_r",
            "sim_get_vel_l", "sim_get_vel_r",
            "sim_get_pwm_l", "sim_get_pwm_r",
            "sim_get_otos_x", "sim_get_otos_y", "sim_get_otos_h",
        ):
            fn = getattr(lib, name)
            fn.argtypes = [ctypes.c_void_p]
            fn.restype = ctypes.c_float

        # --- Error-knob setters (14) ---
        for name in (
            "sim_set_enc_scale_error", "sim_set_enc_slip", "sim_set_enc_noise",
            "sim_set_stiction", "sim_set_motor_lag",
        ):
            fn = getattr(lib, name)
            fn.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_float]
            fn.restype = None

        lib.sim_set_trackwidth.argtypes = [ctypes.c_void_p, ctypes.c_float]
        lib.sim_set_trackwidth.restype = None

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

        # --- Binary reply-store drain (1, 097 addition -- SimConnection
        # binary transport) ---
        # sim_drain_reply_store: same argtypes as sim_peek_reply_store()
        # (this module does not bind that non-destructive sibling -- only
        # the destructive drain drain_binary_tlm() needs), but resets the
        # target channel's ReplyStore as it reads it. See
        # tests/_infra/sim/sim_api.cpp's own doc comment on this entry
        # point and drain_binary_tlm()'s docstring above for why the
        # destructive variant is needed here.
        lib.sim_drain_reply_store.argtypes = [
            ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
        lib.sim_drain_reply_store.restype = ctypes.c_int
