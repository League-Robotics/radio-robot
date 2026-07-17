---
status: pending
sprint: '112'
---

# Relay connect handshake trips `kFaultCommsMalformed` before any application command

Found during 104-007's bench soak session (2026-07-15, robot v0.20260715.3, bench
stand), while characterizing the ack-ring delivery-gap issue
(`clasi/issues/ack-ring-intermittent-delivery-gap.md` — see its own "104-007
characterization" section, finding 4).

## Problem

`kFaultCommsMalformed` (bit 3, `source/app/telemetry.h`, wired by 104-004 —
`App::Comms::malformedCount() > 0`) is meant to signal a malformed/undecodable
inbound frame reaching the robot's line parser. It should stay clear as long as the
host only ever sends well-formed traffic (envelope.proto `*B<base64>` binary lines
or recognized text-plane lines).

Confirmed with an isolated test: fresh clean-boot firmware (immediately after
`mbdeploy deploy`), connect via the radio relay's `!GO` data plane ONLY — zero
`twist()`/`stop()`/`config()`/any application command sent — and the very first
telemetry frames already show `fault_bits` bit 3 set:

```
conn = SerialConnection(port=relay_port, mode="relay")
info = conn.connect()   # info["relay_info"]["entered_data_plane"] == True
proto = NezhaProtocol(conn)
time.sleep(1.0)
frames = proto.read_pending_binary_tlm_frames()
# frames[0].fault_bits == 0b1011  (bit 0 + bit 1 + bit 3 already set)
```

The SAME sequence over DIRECT USB (no relay) never shows bit 3 set at connect —
only the relay path exhibits this. It reproduced across multiple fresh-boot trials
during the 104-007 session.

## What does NOT show this problem

- Direct-USB connect/soak: `kFaultCommsMalformed` stayed clear for the entire
  240s official soak window (104-007's own gating evidence).
- The relay soak's own 240s window: no NEW fault bits appeared DURING the run —
  the bit is already latched by the time the soak's own baseline is captured, and
  never re-trips afterward. Same "fires once, never again" shape as
  `kFaultI2CSafetyNet`'s own documented boot-time-one-shot behavior
  (`source/app/telemetry.h`'s own doc comment, characterized by 103-010) — just
  triggered by relay-connect instead of firmware-boot.

## Impact

- 104-007's own soak-gate acceptance criterion ("`kFaultCommsMalformed` stays clear
  throughout both soak windows") is satisfied under the SAME "already-latched-at-
  baseline, never re-trips" standard the sprint already accepts for
  `kFaultI2CSafetyNet` — this finding did not block that ticket. But the underlying
  fact (something reaches the robot's line parser as malformed on every relay
  connect) is a real gap in "the host's own traffic is always well-formed," worth
  fixing rather than permanently living with a spurious latched bit on every relay
  session.
- Not yet root-caused: most likely candidates are (a) the relay's own
  `!ECHO OFF`/`!MODE RAW250`/`!GO` control-plane bytes momentarily leaking through
  to the robot before the relay fully commits to transparent pass-through, or (b) a
  partial/fragment line at the exact moment of the RAW250 mode transition that the
  robot's parser sees as one malformed line. Not isolated further — no wire-level
  byte capture was taken between the relay and the robot.

## Direction

- Instrument `App::Comms::malformedCount()`'s call site (or a targeted pyOCD/gdb
  session, `.claude/rules/debugging.md`) to capture the actual malformed bytes the
  robot's parser saw at the moment of the trip, correlated with the relay's own
  handshake timing.
- Alternatively, capture the raw byte stream on the relay-robot leg (a
  logic-analyzer or a second probe) across a fresh connect to see exactly what
  crosses the wire during `!GO`'s transition to RAW250 transparent mode.
- Consider whether `SerialConnection.connect()`'s relay handshake
  (`host/robot_radio/io/serial_conn.py`) should drain/discard a settle window on the
  robot side of the link before treating the connection as ready, if the artifact
  turns out to be a transition-timing race rather than genuine leaked control-plane
  bytes.
