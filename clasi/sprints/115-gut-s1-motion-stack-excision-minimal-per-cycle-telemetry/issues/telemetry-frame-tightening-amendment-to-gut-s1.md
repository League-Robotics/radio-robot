---
status: in-progress
sprint: '115'
tickets:
- 115-003
- 115-005
- 115-007
- 115-009
- 115-010
---

# Telemetry frame tightening (amendment to the gut's S1 stage)

## Description

Tighten and restructure the primary `Telemetry` wire frame as part of the minimal-firmware gut's S1 proto surgery. Stakeholder directives (Eric, 2026-07-21, two review passes over `src/firm/messages/telemetry.h`):

- **Per-source reading objects, timestamped.** New `EncoderReading{position, velocity, time}` — `enc_left`/`enc_right` become one each, and the separate `vel_left`/`vel_right` fields disappear (velocity rides inside the reading). New `OtosReading` carrying **everything the OTOS supplies**: position (x, y), heading, velocities (v_x, v_y, omega), and time — replacing the bare `Pose2D otos` (and finally putting the OTOS velocities, which the driver already reads and currently drops, on the wire).
- **One bit string.** The ack ring (depth 3) shrinks to a single ack slot; ALL booleans **and** the `fault_bits`/`event_bits` masks fold into a single `flags` field.
- Kill the oversized declarations (`uint32` queue depth etc. — `queue_depth`/`active_id`/`exec_state`/`heading_source` are already deleted by the gut itself, being executor-era reporting).
- **Add packed sensor words** (2026-07-21, second addition): `line` — the 4-channel line-sensor state, one byte per channel, in one `uint32`; `color` — the color sensor's 4 channels (RGBC), 8 bits each, in one `uint32`. This gives the line/color drivers (kept by the gut) a live consumer: the loop reads them at a **rate-limited, bus-safe cadence** (see firmware notes — never naive per-pass reads on the shared I2C bus).

**The frame is the dataset** (stakeholder decision, 2026-07-21 third pass): with a timestamped frame emitted **every loop iteration** (primary period = cycle period, 20 ms — this closes `kcycle-kprimaryperiod-mismatch.md`), on-chip measurement rings and a dump command are unnecessary — the host logs the stream and reconstructs any window from it. The TLM frame is "the newest timestamped sample from each source", i.e. the central latest-value store expressed on the wire, and the host-side log is the dataset for all analysis and future odometry work. All `time` fields are ms on the robot clock (same domain as `now`).

Old worst-case primary frame: **179 B** (7 B margin at ack-ring depth 3). The restructured frame lands around **~137 B worst case** — smaller than today while carrying strictly more signal (per-sample timestamps, OTOS velocities, and the packed line/color words). `gen_messages.py`'s size table gives the authoritative number at build.

## Cause

The frame accreted executor-era reporting (ack completion statuses, queue/exec fields), a per-field presence convention, and flat parallel arrays (`enc_*`/`vel_*` with no timestamps) that predate the measurement-centric rebuild. Nine standalone bools plus two bitmask fields cost ~26 B of tags and bytes where one flags field carries the same information; untimestamped readings can't feed prediction analysis.

## Proposed fix

### The restructured frame (replaces the gut issue's original telemetry.proto instruction)

```proto
// One wheel's encoder sample.
message EncoderReading {
  float  position = 1;  // [mm] accumulated
  float  velocity = 2;  // [mm/s] signed, measured
  uint32 time     = 3;  // [ms] robot clock at sample collect
}

// Everything the OTOS supplies in one burst.
message OtosReading {
  float  x       = 1;   // [mm]
  float  y       = 2;   // [mm]
  float  heading = 3;   // [rad]
  float  v_x     = 4;   // [mm/s]
  float  v_y     = 5;   // [mm/s]
  float  omega   = 6;   // [rad/s]
  uint32 time    = 7;   // [ms] robot clock at burst read
}

message Telemetry {
  uint32    now  = 1;   // [ms] robot clock at frame assembly
  uint32    seq  = 2;
  DriveMode mode = 3;
  // flags -- THE bit string: status + faults + events, one field.
  //   bit 0  -- otos_present    (OtosReading fresh this frame)
  //   bit 1  -- otos_connected
  //   bit 2  -- active          (motion in progress)
  //   bit 3  -- conn_left       (left motor bus connectivity)
  //   bit 4  -- conn_right
  //   bit 5  -- ack_fresh       (ack_corr/ack_err are a new ack this frame)
  //   bit 6  -- fault: I2C clearance safety-net trip (known benign boot one-shot)
  //   bit 7  -- fault: wedge latch detected
  //   bit 8  -- fault: I2C NAK/timeout
  //   bit 9  -- fault: malformed inbound frame seen
  //   bit 10 -- event: deadman expired (transition cycle)
  //   bit 11 -- event: boot ready (transition cycle)
  //   bit 12 -- event: config delta applied (transition cycle)
  //   bit 13 -- line_present   (line word fresh)
  //   bit 14 -- color_present  (color word fresh)
  //   bit 15 -- fault: MOVE timeout backstop fired (protocol set-point issue)
  //   bits 16+ reserved
  uint32 flags = 4 [(max) = 65535];
  // Single ack slot (was: repeated AckEntry x3). ack_err == 0 means OK;
  // nonzero is the ErrCode (envelope.proto) value. Valid iff flags bit 5.
  uint32 ack_corr = 5 [(max) = 65535];
  uint32 ack_err  = 6 [(max) = 7];
  // Sensors and encoders
  EncoderReading enc_left  = 7;
  EncoderReading enc_right = 8;
  OtosReading    otos      = 9;   // valid iff flags bit 0
  Pose2D         pose      = 10;  // encoder-odometry integrated pose, always present
  BodyTwist3     twist     = 11;  // body twist from measured wheel velocities, always present
  // Packed sensor words. Valid iff flags bits 13/14.
  uint32 line  = 12;  // 4 line channels, one byte each (ch1 in the low byte)
  uint32 color = 13;  // color sensor RGBC, 8 bits per channel (R in the low byte)
}
```

**Deleted entirely**: `AckEntry` + the depth-3 ring; the `AckStatus` enum (OK/ERR collapses into `ack_err` zero/nonzero; the six executor-completion statuses die with the executor); all five `has_*` bools and all four standalone status bools; standalone `fault_bits`/`event_bits` fields (folded into `flags`); flat `enc_left/enc_right/vel_left/vel_right` floats (replaced by the two `EncoderReading`s); bare `Pose2D otos` (replaced by `OtosReading`); `queue_depth/active_id/exec_state/heading_source` + their enums (per the gut issue).

**Renumbering (decided)**: clean renumber, all fields ≤ 15 → every tag 1 byte. The only consumer is this repo's host (`pb2` regenerates from the same protos, firmware+host co-deploy at the bench). Proto header comment records the compat break (frame v2, sprint-115 era).

**Ack-depth-1 tradeoff (stakeholder-accepted)**: commands acked within the same primary period overwrite; rare at bench rates, `wait_for_ack` timeout + retry covers it.

**`TelemetrySecondary` untouched** (slow-cadence diagnostics). Note: with velocity now inside `EncoderReading`, secondary's `ts_left/ts_right` sample-timestamp fields become redundant — prune them opportunistically if touched.

### Firmware changes (inside the gut-S1 pass)

- `src/protos/telemetry.proto` — rewrite per the spec above.
- `src/firm/app/telemetry.{h,cpp}` — `Frame` reshapes: two `EncoderReading`-shaped members (position/velocity/time from the accepted encoder samples), one `OtosReading` member, single `ackCorr/ackErr/ackFresh`, one `flags` assembly point ORing status + fault + event bits; encode paths follow the regenerated message. **Primary emission every loop iteration** (primary period = cycle period, 20 ms; fix both constants' stale "~25 Hz" labels).
- `src/firm/app/robot_loop.cpp` — `updateTlm()` stages the readings with their sample times (encoder: collect-time; OTOS: burst-read time; ms of the robot clock); flags assembly replaces the separate bool/bitmask staging; ack call sites pass err codes (0 = OK).
- **Line/color reads (new)**: the loop reads the line sensor (4 channels → one byte each) and color sensor (RGBC, scaled to 8 bits per channel) into the packed words. **Bus discipline is mandatory**: reads are rate-limited in-driver (the `Otos::readDue` pattern, ~TLM cadence) and scheduled in the kPace block, at most one of the two sensors per cycle (alternating) — never naive per-pass reads; the 098-004 regression (per-pass OTOS reads disrupting the motor flip-flop cadence and wrecking motion timing) is the standing precedent. The soak gate specifically watches motion timing for regressions after these reads land.
- Generated `messages/telemetry.h` + `wire.cpp` regenerate from the proto.

### Host changes (rides the same forced S1 `protocol.py` touch-up)

- `src/host/robot_radio/robot/protocol.py` — decode the nested readings (`frame.enc_left.position` etc.), single-ack gated on the flags bit, presence/status/fault/event exposed as properties derived from `flags` so downstream consumers keep working.
- `src/host/robot_radio/robot/nezha_state.py` / `robot_state.py` — adapter mapping readings + flags onto the existing attribute names TestGUI panels read.

## Verification

- `gen_messages.py` worst-case size table: confirm the measured frame size (~125 B expected vs 179 B); record in the proto comment.
- Tests: wire round-trip for `EncoderReading`/`OtosReading`/frame; flags semantics across all three groups (status/fault/event bits); reading stamps monotonic and consistent with `now`; single-ack overwrite; `wait_for_ack` happy + timeout paths.
- Hardware (part of the S1 gate, robot on stand): telemetry streams with per-wheel readings carrying sane times (~cycle-period apart), OTOS reading populating with velocities when present, `line`/`color` words showing plausible, changing channel values (per the bench sensors-alive gate), flags tracking presence/connectivity/faults/events, acks observed via the single slot; 10-min soak with drop rate at or better than the S0 baseline **and no motion-timing regression from the added sensor reads** (encoder tracking vs commanded speed unchanged from the pre-sensor-read runs).

## Related

- `gut-to-minimal-firmware-motion-stack-excision-move-protocol-minimal-telemetry.md` — the parent work; this issue is its S1 telemetry.proto spec and shares its hardware gate. Plan as one sprint.
- `protocol-set-point-the-minimal-firmware-s-complete-command-surface.md` — defines flags bit 15 (MOVE timeout fault) and the single-ack usage this frame carries.
- `kcycle-kprimaryperiod-mismatch.md` — resolved by the every-cycle cadence decision recorded here.
