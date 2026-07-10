"""_legacy_tlm_text — frozen, private copy of the retired text-plane TLM
line parser, for the narrow set of consumers that CANNOT source a
``TLMFrame`` from the binary plane (097-003).

Why this file exists
---------------------
097-003 converts ``NezhaProtocol.stream()``/``.snap()`` to the binary plane
and deletes the module-level text-line-to-``TLMFrame`` parser and its
config-line counterpart those methods' internal consumers used to call
(architecture-update.md (097) Decision 3). Sweeping every internal consumer
onto binary-native
``TLMFrame`` delivery (``SerialConnection._binary_tlm_queue`` /
``TLMFrame.from_pb2()``) is the default and normal path -- see
``host/robot_radio/robot/nezha.py``, ``nezha_state.py``,
``host/robot_radio/testgui/transport.py``'s ``_HardwareTransport``,
``host/robot_radio/io/cli.py``, and ``tests/playfield/world_goto_chart.py``
for that sweep.

Four call sites cannot make that move, for two DIFFERENT structural
reasons, neither of which this ticket can resolve by editing host code:

1. **No ``SerialConnection`` in play at all** -- ``_binary_tlm_queue`` is a
   ``SerialConnection`` implementation detail. These consumers talk to the
   robot/simulator over a DIFFERENT transport that ``SerialConnection``
   never owns:
     - ``host/robot_radio/calibration/linear.py``,
       ``host/robot_radio/calibration/angular.py`` -- use
       ``robot_radio.calibration._conn_helpers.RelaySerial``/
       ``DirectSerial``, a raw pyserial wrapper chosen DELIBERATELY to get
       fine-grained timing control over the relay handshake/DTR reset
       (``_conn_helpers.py``'s own header comment) -- not
       ``SerialConnection``.
     - ``host/robot_radio/testgui/transport.py``'s ``SimTransport`` -- uses
       ``robot_radio.io.sim_conn.SimConnection``, a ctypes ABI directly
       wrapping the compiled sim library, again not ``SerialConnection``.

2. **The data itself is gone from the binary wire schema** --
   ``host/robot_radio/calibration/fit_sim_error_model.py``'s residual
   computation (``_residual_vector()``) structurally depends on
   ``TLMFrame.encpose`` (the encoder-only dead-reckoned pose) as one of
   its three pose-residual channels. ``telemetry.proto``'s ``Telemetry``
   message never carries ``encpose`` at all (096-001 Decision 6 trimmed it
   to fit the 186-byte envelope budget) -- ``TLMFrame.from_pb2()`` can
   NEVER populate it, on ANY transport, binary or otherwise. This is a
   permanent wire-schema gap, not a parsing or transport limitation.

For all four, the firmware's TEXT ``STREAM``/``SNAP`` handlers remain live
through this ticket (only a LATER ticket, 097-008, deletes them) -- so the
conservative, no-behavior-change choice is to keep these four consumers on
the text plane, using this frozen, private copy of the retired parser
(field-for-field identical to the function this ticket removed from
``host/robot_radio/robot/protocol.py``) rather than either (a) silently
dropping data (``encpose`` for the fit) or (b) hand-rolling a new,
UNTESTED binary armor/dearmor round trip outside ``SerialConnection``'s
already-proven machinery for a rarely-run calibration/dev-tooling path.

This is a DELIBERATE, FLAGGED exception (097-003 completion notes; see also
the ticket's own resolution section) -- not an oversight. Once these four
consumers' data sources retire their own text-plane replies (097-008 for
real hardware/the sim binary; a future encpose wire-schema restoration for
the fit), each needs its own dedicated follow-up, tracked as a separate
issue at that point. Do NOT import this module from any NEW call site --
it exists only to keep these four historical consumers alive without
resurrecting a general-purpose replacement for the retired parser.
"""

from __future__ import annotations

from robot_radio.robot.protocol import TLMFrame, parse_response


def parse_historical_tlm_line(line: str) -> "TLMFrame | None":
    """Parse a text ``TLM ...`` line into a ``TLMFrame``, or ``None`` if
    ``line`` is not a TLM line.

    Field-for-field identical to the module-level text-line parser 097-003
    deleted from ``host/robot_radio/robot/protocol.py`` -- see this
    module's own header for why it is frozen here instead of resurrected
    as a public, general-purpose function.
    """
    resp = parse_response(line)
    if resp is None or resp.tag != "TLM":
        return None

    frame = TLMFrame()
    kv = resp.kv

    if "t" in kv:
        try:
            frame.t = int(kv["t"])
        except ValueError:
            pass

    if "mode" in kv:
        frame.mode = kv["mode"]

    if "seq" in kv:
        try:
            frame.seq = int(kv["seq"])
        except ValueError:
            pass

    if "wedge" in kv:
        try:
            parts = kv["wedge"].split(",")
            if len(parts) == 2:
                frame.wedge = (int(parts[0]), int(parts[1]))
        except ValueError:
            pass

    if "enc" in kv:
        try:
            parts = kv["enc"].split(",")
            if len(parts) == 2:
                frame.enc = (int(parts[0]), int(parts[1]))
        except ValueError:
            pass

    if "pose" in kv:
        try:
            parts = kv["pose"].split(",")
            if len(parts) == 3:
                frame.pose = (int(parts[0]), int(parts[1]), int(parts[2]))
        except ValueError:
            pass

    if "encpose" in kv:
        try:
            parts = kv["encpose"].split(",")
            if len(parts) == 3:
                frame.encpose = (int(parts[0]), int(parts[1]), int(parts[2]))
        except ValueError:
            pass

    if "vel" in kv:
        try:
            parts = kv["vel"].split(",")
            if len(parts) == 2:
                # Differential: (vL_mmps, vR_mmps)
                frame.vel = (int(parts[0]), int(parts[1]))
            elif len(parts) == 4:
                # Mecanum: (vFR_mmps, vFL_mmps, vBR_mmps, vBL_mmps)
                frame.vel = (int(parts[0]), int(parts[1]),
                             int(parts[2]), int(parts[3]))
        except ValueError:
            pass

    if "cmd" in kv:
        try:
            parts = kv["cmd"].split(",")
            if len(parts) == 2:
                # Commanded per-wheel velocity (PID setpoint): (vL, vR) [mm/s]
                frame.cmd_vel = (int(parts[0]), int(parts[1]))
        except ValueError:
            pass

    if "twist" in kv:
        try:
            parts = kv["twist"].split(",")
            if len(parts) == 2:
                # Differential: (v_mmps, omega_mradps)
                frame.twist = (int(parts[0]), int(parts[1]))
            elif len(parts) == 3:
                # Mecanum: (vx_mmps, vy_mmps, omega_mradps)
                frame.twist = (int(parts[0]), int(parts[1]), int(parts[2]))
        except ValueError:
            pass

    if "otos" in kv:
        try:
            parts = kv["otos"].split(",")
            if len(parts) == 3:
                frame.otos = (int(parts[0]), int(parts[1]), int(parts[2]))
        except ValueError:
            pass

    if "line" in kv:
        try:
            parts = kv["line"].split(",")
            if len(parts) == 4:
                frame.line = (int(parts[0]), int(parts[1]),
                              int(parts[2]), int(parts[3]))
        except ValueError:
            pass

    if "color" in kv:
        try:
            parts = kv["color"].split(",")
            if len(parts) == 4:
                frame.color = (int(parts[0]), int(parts[1]),
                               int(parts[2]), int(parts[3]))
        except ValueError:
            pass

    if "ekf_rej" in kv:
        try:
            frame.ekf_rej = int(kv["ekf_rej"])
        except ValueError:
            pass

    if "otos_health" in kv:
        try:
            parts = kv["otos_health"].split(",")
            if len(parts) == 2:
                frame.otos_health = (int(parts[0]), bool(int(parts[1])))
        except ValueError:
            pass

    return frame
