# Protocol v5 Wire Specification

**Current wire truth.** This document describes the Nezha firmware
command/telemetry protocol **as shipped by sprint 124** ("Protocol v5,
`RobotState` blackboard, and radio bench gate", tickets 001-013) — the
one-line-per-packet grammar, `0x0A`-safe COBS framing, CRC scope extended
to the command name, the restored `ID`/`VER`/`PONG` reply-plane verbs, and
packed fixed-point telemetry that sprint's own atomic wire cutover
landed. It supersedes [`docs/protocol-v4.md`](protocol-v4.md) (kept, not
deleted, as the historical record of what shipped when — see that file's
own banner and this document's Appendix), which is now itself the
historical record of protocol v4 (sprint 116's MOVE-protocol command
surface, sprint 123's COBS+CRC binary framing) the way `docs/protocol-v2.md`/
`docs/protocol-v3.md` already were for v4.

Source of the contract this document transcribes:
[`protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md`](../clasi/sprints/124-protocol-v5-robotstate-blackboard-and-radio-bench-gate/issues/done/protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md)
for the framing-grammar/registry/telemetry-packing proposal, and
[`robot-state-blackboard-one-struct-for-all-shared-state-and-telemetry.md`](../clasi/sprints/124-protocol-v5-robotstate-blackboard-and-radio-bench-gate/issues/done/robot-state-blackboard-one-struct-for-all-shared-state-and-telemetry.md)
for the `RobotState`/`Telemetry`-projection side. Every claim below was
cross-checked against the actual shipped source
(`src/protos/commands.proto`, `src/protos/envelope.proto`,
`src/protos/telemetry.proto`, `src/protos/options.proto`,
`src/firm/app/comms.{h,cpp}`, `src/firm/app/robot_loop.cpp`,
`src/firm/app/telemetry.{h,cpp}`, `src/firm/types/robot_state.h`,
`src/firm/messages/wire_runtime.h`, `src/firm/messages/wire.h`,
`src/firm/main.cpp`, `src/host/robot_radio/io/wire_codec.py`,
`src/host/robot_radio/io/serial_conn.py`,
`src/host/robot_radio/io/wire_commands.py`,
`src/host/robot_radio/robot/protocol.py`) and its own test suite (the
cross-language golden vectors, ticket 004; the sim/loopback framing path,
ticket 006), not restated from either issue's proposal alone. Sections
whose content is unchanged from protocol v4 (the `Move` message shape,
the execution model, the `ConfigDelta` arm) are carried forward with the
same section numbers as `docs/protocol-v4.md` so an existing citation of
e.g. "protocol-v4 §7.2" still resolves to the equivalent v5 content at
"protocol-v5 §7.2" — call-site comments across the tree still cite the
number, updated to the `v5` filename.

---

## 1. Overview

The robot speaks **one uniform packet grammar in both directions, text or
binary**: `<COMMAND>[':' <data>]'\n'`. There is no separate "text safety
rump" mechanism any more (protocol v4's two-verb `HELLO`/`PING` rump,
demuxed from the binary plane by a `0x00`-vs-`\n` terminator heuristic) —
every verb, cleartext or binary, lives in one generated command registry
and is framed identically:

| Plane | Shape | Carries |
|---|---|---|
| Binary command verbs (host→robot) | `<VERB>':'<COBS+CRC frame>'\n'` | `MOVE`/`CONFIG`/`STOP` — `CommandEnvelope`'s three `cmd` oneof arms (§3) |
| Cleartext command verbs (host→robot) | `<VERB>'\n'`, no data | `HELLO` (identity request), `PING` (liveness), `ID` (configured-identity request), `VER` (build-version request) |
| Cleartext reply verbs (robot→host) | `<VERB>':'<data>'\n'` | `DEVICE:` (boot/HELLO-reply hardware identity, byte-frozen), `PONG:t=<ms>` (PING's reply), `ID:<fields>` (configured identity), `VER:<version>` (build version) |
| Binary reply verb (robot→host), every cycle | `TLM:<COBS+CRC frame>'\n'` | `ReplyEnvelope{corr_id=0, tlm: Telemetry}` — the SOLE per-command outcome path (§7) and the always-on telemetry push (§8) |

Which verb a line names, and whether its data is cleartext or binary, is
looked up in **one generated command registry**
(`src/protos/commands.proto` → `messages/commands.h`'s `kVerbTable[]` on
firmware, `io/wire_commands.py`'s `VERBS` on the host) — the SOLE
text/binary discriminator. Nothing about a line's own data bytes is ever
inspected to decide how it is read; this closes the "two independent
per-language heuristics, guessing about the same byte stream, that can
disagree" defect class sprint 123 shipped with (a `move_wheels` envelope
containing a literal `0x0A` byte failed 0/10 on hardware before 123-006's
narrow patch — this sprint's grammar closes the whole class, not just
that one byte value).

The command surface itself (§3-§6) is otherwise unchanged from protocol
v4: `CommandEnvelope.cmd` still carries exactly three arms —
`move`/`config`/`stop` — and every motion is still a bounded `Move`
(velocity variant + stop condition + required `timeout` backstop, queued
1-active + 4-pending). Protocol v5 is a **framing and reply-plane
rewrite**, not a command-surface rewrite.

---

## 2. Transport & framing

**Sprint 124 replaces sprint 123's `0x00`-delimited COBS+CRC frame (with
a separate `\r`?`\n`-terminated text rump sharing the same byte stream,
demuxed by `App::FrameKind`) with one uniform grammar, keyed on the SAME
byte the grammar's own terminator uses.** The command surface itself
(§3+) is unaffected — this is a framing-layer and reply-plane change
only.

### 2.1 Line grammar

```
<COMMAND>'\n'                          -- cleartext, no data (HELLO/PING/ID/VER)
<COMMAND>':'<data>'\n'                 -- cleartext, with data (DEVICE:/PONG:/ID:/VER: replies)
<COMMAND>':'<COBS(payload + CRC-16)>'\n'  -- binary, either direction (MOVE/CONFIG/STOP/TLM)
```

One shared byte stream per transport (serial CDC, bench 115200 baud; or
the radio relay, `RadioTransport`) carries every packet, unconditionally
split on `0x0A` (`'\n'`) — there is no per-transport heuristic left at
all (`Transport::readLine()` returns a plain `bool`, not a `FrameKind`).
This is unambiguous by construction, not by guessing: a COBS-encoded
frame body can never contain an embedded `0x0A` (§2.2's delimiter
parameterization makes `0x0A` — not `0x00` — the byte COBS excludes), and
a cleartext line never legitimately contains one either (`\n` always ends
it). `App::Comms::dispatchLine()` (`src/firm/app/comms.cpp`) parses the
`<COMMAND>` prefix from an already-`\n`-delimited line — the FIRST `':'`
ends the command; every later byte, including further `':'` bytes, is
data — then looks that name up in the registry (§2.4) and dispatches by
the lookup's own `binary` flag. A colon-less line is only ever a
candidate for one of the four no-data cleartext verbs; a single trailing
`\r\n`-style `\r` on such a line is stripped before the lookup (a raw
terminal artifact, never legitimate binary content, since every binary
verb always carries a `':'`-prefixed body, even an empty one). An
unrecognized `<COMMAND>` increments `malformedCount_` — the SAME counter
a COBS/CRC failure already increments — one fault-bit source
(`kFlagFaultCommsMalformed`), not two. `App::Comms::pump()` reads at most
one complete line per call from serial first, falling back to radio only
if serial had nothing.

**Relay control-plane tolerance (124-010).** A line whose first byte is
`#`/`!`/`?` — the radio-relay dongle's own control-plane sigils (a
status/comment reply, a dongle command, or the dongle's config query,
respectively) — is dropped BEFORE the registry lookup, uncounted (never
incrementing `malformedCount_`). This closes a false trip of
`kFaultCommsMalformed` on a clean relay connect, before any application
command: a fragment of the host↔dongle `!GO`-entry handshake traffic can
reach the robot's radio receiver before/at the moment the dongle commits
to transparent pass-through. No registered v5 verb name starts with any
of these three bytes, so this carve-out can never mask a genuine
malformed command. **Known measurement bias**: any inbound line the relay
dongle itself emits that happens to start with `#`/`!`/`?` is silently
absorbed rather than counted — this is deliberate (it is exactly the
noise class this ticket exists to tolerate) but means
`Comms::malformedCount()` is not a perfect count of "malformed frames
arriving on the wire" over the relay path specifically; it undercounts by
however many control-plane fragments leak through in a given session.
Direct USB never runs the dongle's handshake at all, so this bias does
not apply there.

### 2.2 COBS + CRC framing (binary verbs only)

- **Delimiter: `0x0A`, not `0x00`** (`App::kCobsDelimiter`, `comms.h`;
  `wire_codec.py`'s `encode_frame()`/`decode_frame()` hard-code
  `delimiter=0x0A`). `WireRuntime::cobsEncode()`/`cobsDecode()`
  (`wire_runtime.{h,cpp}`) gained a trailing `delimiter` byte parameter
  (default `0x00`, unchanged for any pre-124 caller) implemented as an
  XOR of every output byte (encode) / every input byte at the point of
  reading (decode) by `delimiter` on top of the existing `0x00`-keyed
  algorithm — sound because the pre-XOR output is guaranteed `0x00`-free
  by construction, and `b ^ delimiter == delimiter` iff `b == 0`, which
  never occurs, so the XOR-ed stream can never contain `delimiter`
  either. A COBS-encoded body under this delimiter is `0x0A`-free but MAY
  legitimately contain an embedded `0x00` byte — the inverse of protocol
  v4's guarantee.
- **CRC scope now covers `COMMAND ':' payload`, not payload alone**
  (`Comms::crcOverScope()`/`wire_codec.py`'s `_crc_over_scope()`): the
  schema-encoded payload's CRC-16 is computed over the parsed ASCII
  command-name bytes, then the `':'` separator, then the payload —
  `WireRuntime::crcInit()`/`crcUpdate()` (an incremental pair,
  `crcCompute(data, len) == crcUpdate(crcInit(), data, len)`) lets a
  caller fold two byte ranges that are not adjacent in memory (the parsed
  command name and the payload buffer) into one CRC without concatenating
  them first. This closes the "a bit-flip inside the command name lands
  on another valid verb and still dispatches with a passing CRC" gap
  protocol v4's payload-only CRC left open. The CRC *variant* itself is
  unchanged from v4: **CRC-16/CCITT-FALSE** — poly `0x1021`, init
  `0xFFFF`, no input/output reflection, no final XOR. Known-answer
  vector: `crcCompute("123456789", 9) == 0x29B1`.
- **Wire layout is otherwise unchanged from v4's CRC-then-COBS
  composition**: append the little-endian CRC-16 to the schema-encoded
  payload, THEN COBS-encode the combined bytes
  (`WireRuntime::cobsEncode()`); the transport appends the trailing
  `'\n'` delimiter itself — callers never include it. Decode is the exact
  reverse: COBS-decode, split off the trailing 2-byte CRC, verify it
  (scoped per the bullet above) against the leading payload bytes, and
  only then hand the payload to `msg::wire::decode()` to walk
  `CommandEnvelope`'s field table. Any step failing — malformed COBS, a
  CRC mismatch, or a malformed/truncated decode — increments
  `malformedCount_` and sends no reply (§7.4).
- **Host mirror**: `src/host/robot_radio/io/wire_codec.py` remains the
  ONE place the host encodes/decodes this framing; every producer/
  consumer (`io/serial_conn.py`, `io/sim_loop.py`, `io/cli.py`,
  `testgui/transport.py`, `robot/protocol.py`) imports from here.
- **Base64 remains retained, not removed, and still not the wire's
  armor** — unchanged from v4: `WireRuntime::base64Encode()`/
  `base64Decode()` exist only because `wire_differential_harness.cpp`
  (`src/tests/sim/unit/`) independently depends on the primitive for its
  own, unrelated debug-CLI wire encoding.

### 2.3 Size budget (measured, `gen_messages.py`-computed)

The 240-byte envelope budget itself is **unchanged from v4** — sprint
124's framing change (delimiter + CRC scope) does not alter COBS+CRC's
fixed ~4-byte overhead, and the ASCII `<COMMAND>':'` prefix lives OUTSIDE
the COBS-encoded region, so it does not affect this budget either (it
affects the whole-LINE scratch-buffer size instead — see below). Sprint
124 ticket 008 additionally introduces a NEW, tighter, explicit gate —
`kReplyEnvelopeMaxEncodedSize <= 130` bytes — driven by packed telemetry
encoding (§8), separate from and stricter than the 240-byte envelope
budget:

| Envelope | Worst-case arm | Total (worst arm + non-oneof bytes) |
|---|---|---|
| `CommandEnvelope` | `config`=49B, `stop`=2B, `move`=38B (worst=`config`) | **55 B** (unchanged from v4) |
| `ReplyEnvelope` | `ok`=19B, `err`=10B, **`tlm`=126B** (worst=`tlm`) | **130 B** (down from v4's 194 B — §8.3) |

`TelemetrySecondary` — a second, independently-framed diagnostic frame
that existed through protocol v4 — no longer exists (sprint 124 ticket
009; §8.4 below); there is only ever one outbound top-level wire message
now, `ReplyEnvelope`.

- **COBS-encoded region size (unchanged from v4)**: `kFramedMaxBytes` =
  200 bytes (`comms.h`) — the worst-case COBS-encoded length of
  `kMaxEnvelopeBytes` (194, the larger of the two per-direction budgets;
  note this is the pre-ticket-008 194 B figure the buffer was sized
  against, still comfortably covering the now-smaller 130 B `tlm` worst
  case) + 2-byte CRC = 196 payload bytes, `cobsEncodedMaxLength(196)` =
  197, rounded up to 200 for headroom.
- **Whole-line buffer size (NEW this sprint — accounts for the ASCII
  command prefix, which the COBS region above does not)**:
  `kMaxCommandPrefixBytes` = the longest registered verb name + 1 for the
  `':'` separator, computed at compile time from `messages/commands.h`'s
  own `kVerbTable[]` (currently 7 — `"DEVICE"`/`"CONFIG"`, both 6 bytes,
  are the longest registered names). `kMaxLineBytes = kFramedMaxBytes +
  kMaxCommandPrefixBytes` = 207 bytes — the longest possible
  `<COMMAND>':'<COBS+CRC bytes>` content a single `Transport::readLine()`/
  `Transport::send()` call ever needs to hold (the transport's own
  trailing `'\n'` is one further byte, not counted here, matching
  `kFramedMaxBytes`'s own convention). Replaces v4's `kArmoredBufSize`-
  descended, prefix-free buffer sizing.
- **Nesting depth cap**: 8 levels (`WireRuntime::kMaxNestingDepth`) —
  unchanged, a property of the schema, not the wire armor.

### 2.4 Command registry and the cleartext reply plane

**The closed v5 wire verb set — and which verbs carry binary vs.
cleartext data — is generated from ONE schema**,
[`src/protos/commands.proto`](../src/protos/commands.proto), processed by
`gen_messages.py` into `src/firm/messages/commands.h` (the `Verb` enum +
`kVerbTable[]`, firmware dispatch) and
`src/host/robot_radio/io/wire_commands.py` (`VERBS`/`VERB_BY_NAME`/
`BINARY_VERBS`/`CLEARTEXT_VERBS`, the host-consumable mirror) — the
single source firmware dispatch, the host codec, and this document are
all generated from or checked against. Adding a future verb means
editing exactly this one schema file; no parallel edit in firmware, host,
or docs is needed to keep the verb set consistent — closing the
firmware/host/doc three-way drift risk protocol v4's hand-maintained
two-verb rump invited.

| Verb | Direction | Binary? | Reply / effect |
|---|---|---|---|
| `HELLO` | host→robot | cleartext, no data | `DEVICE:NEZHA2:robot:<name>:<serial>` — byte-frozen hardware-identity banner, unchanged from v4 |
| `PING` | host→robot | cleartext, no data | `PONG:t=<ms>` — liveness probe + clock-sync activation (was `OK pong t=<ms>` in v4 — see below) |
| `ID` | host→robot | cleartext, no data | `ID:<drivetrain>:<profile>:<version>` — configured/calibration identity |
| `VER` | host→robot | cleartext, no data | `VER:<version>` — build identity |
| `DEVICE` | robot→host | cleartext | `HELLO`'s reply, and the always-emitted boot banner (`sendBanner()`, twice per boot: power-on and preamble-done) |
| `PONG` | robot→host | cleartext | `PING`'s reply |
| `MOVE` | host→robot | binary | `CommandEnvelope.cmd.move` (§3/§4) |
| `CONFIG` | host→robot | binary | `CommandEnvelope.cmd.config` (§3/§6) |
| `STOP` | host→robot | binary | `CommandEnvelope.cmd.stop` (§3/§5.6) |
| `TLM` | robot→host | binary | `ReplyEnvelope{tlm: Telemetry}` — the only reply arm with a live producer (§7.1/§8) |
| `OK` / `ERR` | robot→host | binary | `ReplyEnvelope`'s `ok`/`err` oneof arms — schema-declared, zero live producers today (unchanged from v4, §7.1) |

`Comms::dispatchCleartext()` (`comms.cpp`) is the inbound handler for the
four cleartext command verbs:

- **`HELLO` → `banner_`** (unchanged content/formatting from v4 —
  `formatBanner()` already conformed to the v5 grammar with no edit:
  command `DEVICE`, cleartext data after the first `':'`).
- **`PING` → `PONG:t=<ms>`** — `t` is the robot's own clock at
  reply-formatting time (unchanged activation semantics from v4's SUC-056
  — this activates the host's `ClockSync`, min-RTT offset + skew fit).
  Formatted via `std::snprintf(pong, sizeof(pong), "PONG:t=%lu", ...)` —
  integer-only, matching `newlib-nano`'s lack of `%f` support (not a
  workaround; `now` is already an integer). **The one behavioral break
  from v4**: the reply text itself changed from `"OK pong t=<ms>"` to
  `"PONG:t=<ms>"` — the reply plane now uses the SAME colon-joined
  grammar as every other verb, rather than a legacy space-joined shape. A
  reply verb echoing its own command verb (`PING`→`PONG`, `HELLO`→
  `DEVICE`) is unambiguous because link direction is known.
- **`ID` → `idLine_`** — a caller-owned string set once at `Comms`
  construction (architecture Decision 4: **configured/calibration
  identity** — drivetrain type + calibration-profile name/version —
  distinct from `DEVICE:`'s hardware identity, and a genuinely different
  axis: "am I talking to the robot I think, configured how I think").
  `main.cpp` builds it as `ID:<drivetrain>:<profile>:<version>` from two
  generated constants (`Config::kDrivetrainType`/
  `Config::kRobotProfileName`, baked from the robot JSON's own
  `identity.drivetrain_type`/filename stem) plus the same build-version
  string `VER:` reports. This verb did not exist in protocol v4.
- **`VER` → `"VER:" FIRMWARE_VERSION_STR`** — reads the existing
  generated build-version constant (`src/firm/types/version_generated.h`,
  the same source `close_sprint`'s version-bump cadence stamps) directly;
  zero new version-tracking infrastructure. This verb did not exist in
  protocol v4.

All four reply via `sendReliable()` (bounded-wait, must-not-drop —
unchanged policy from v4's `HELLO`/`PING` reply path). A cleartext verb
with no inbound handler (a stray `DEVICE`/`PONG`/`OK`/`ERR`/`TLM` sent
host→robot) increments `malformedCount_` exactly like an unrecognized
command name.

`Comms::sendReply()`'s outbound verb name (for both the reply's
`<COMMAND>':'` wire prefix and the CRC's scope-extension input, §2.2) is
derived INTERNALLY from `reply.body_kind` (`TLM`/`OK`/`ERR` map 1:1 onto
the registry's `Verb::TLM`/`OK`/`ERR`) — a caller never threads a second,
independently-spelled name string that could drift from the envelope's
own discriminant.

---

## 3. `CommandEnvelope` — command arms

`src/protos/envelope.proto`'s `CommandEnvelope.cmd` oneof, dispatched by
`RobotLoop::processMessage()`'s switch on `msg::CommandEnvelope::CmdKind`
— **field numbers, handler bindings, and reserved-list unchanged from
protocol v4**:

| Arm | Field # | Payload | Handler | Wire verb name (§2.4) |
|---|---|---|---|---|
| `config` | 6 | `ConfigDelta` (§6) | `RobotLoop::handleConfig()` | `CONFIG` |
| `stop` | 13 | `Stop{}` (zero fields) | `RobotLoop::handleStop()` | `STOP` |
| `move` | 21 | `Move` (§4) | `RobotLoop::handleMove()` | `MOVE` |

`corr_id` (field 1) is present on every `CommandEnvelope` and is echoed
back via the ack ring (§7.1), never a per-command `ReplyEnvelope`.
Reserved field numbers (2, 3, 4, 5, 7-12, 14-18, 19, 20) are unchanged
from v4 — this sprint added no new `CommandEnvelope` arm and removed
none. `ErrCode` (used by every ack — §7.3) is unchanged.

---

## 4. The `Move` message

**Unchanged from protocol v4** — sprint 124 is a framing/reply-plane
rewrite, not a command-surface rewrite:

```proto
message MoveTwist {
  float v_x   = 1;  // [mm/s] body forward
  float v_y   = 2;  // [mm/s] accepted-and-ignored on this differential build (wire-forward
                    //        for a future holonomic base)
  float omega = 3;  // [rad/s]
}
message MoveWheels {
  float v_left  = 1;  // [mm/s]
  float v_right = 2;  // [mm/s]
}
message Move {
  oneof velocity {              // exactly one of the two velocity variants
    MoveTwist  twist  = 1;
    MoveWheels wheels = 2;
  }
  oneof stop {                  // exactly one stop condition
    float time     = 3;  // [ms] elapsed since activation
    float distance = 4;  // [mm] |path arc length| since activation (encoder odometry)
    float angle    = 5;  // [rad] |heading change| since activation (encoder odometry)
  }
  float  timeout = 6;  // [ms] REQUIRED safety backstop; <=0 -> ERR_BADARG.
  bool   replace = 7;  // true: flush pending + preempt active, this MOVE starts now.
                        // false: enqueue behind the active command (ERR_FULL if 4 pending).
  uint32 id      = 8;  // echoed in this command's COMPLETION ack (enqueue ack echoes corr_id)
}
```

See [`docs/protocol-v4.md`](protocol-v4.md) §4.1-§4.3 for the full,
still-accurate shape-validation rules (config-completeness gate, the
`ERR_BADARG` checks), the AS-BUILT zero/negative stop-threshold clamp
behavior, and the stop-condition/timeout tie-break — none of it changed
by this sprint.

---

## 5. Execution model

**Unchanged from protocol v4** — the queue (1 active + 4 pending), the
per-cycle `MoveQueue::tick()` schedule, land-at-zero completion,
approach shaping (`Motion::VelocityShaper`), the "no completion ack for a
flushed-while-pending `Move`" AS-BUILT behavior, the no-deadman
structural safety property (SUC-053), velocity staging (`App::Drive`),
`STOP`'s effect, and the config-completeness gate are all identical to
[`docs/protocol-v4.md`](protocol-v4.md) §5.1-§5.7. This sprint's own
`RobotLoop`/`Telemetry` restructure (124-009) changed HOW state is
assembled and projected (§8 below), not this execution model.

**New this sprint, orthogonal to the execution model above: position
rebaseline.** `EncoderReading.position` (§8.1) accumulates monotonically
for the entire session in `RobotState`/`NezhaMotor::encOffset_` — a wide
(`int32_t` tenths-of-degree) raw accumulator that will not realistically
overflow (≈121 km of wheel travel before it could). The WIRE field is a
tighter, `sint32`/zigzag-packed `(abs_max) = 32000` (±32 m at 1 mm
scale), for wire-cost reasons, not a hardware limit — a multi-hour
characterization session can plausibly accumulate that much *net signed*
travel if direction isn't perfectly balanced. Each cycle, after a wheel's
position is read, if `|position| >= 30000` mm (a 2000 mm margin below the
32000 mm wire bound), `RobotLoop` calls the wheel's EXISTING, UNMODIFIED
`Devices::Motor::rebaseline()` — a pure software re-anchor (folds the
current position back into the device's own offset and zeroes the local
cache, issuing **zero I2C bus traffic**, the same model a vendor motor
controller uses: read raw, hold a software offset, report
`raw - offset`) — and increments that wheel's `positionEpoch` counter
(§8.1). This is never the staged, potentially-bus-touching
`Motor::resetPosition()` path (`MotorArmor::processResetIfPending()`),
which is off-limits for this policy. As a defensive fallback only (not
the expected path, since the 2000 mm margin dwarfs one cycle's worst-case
travel), the fixed-point ENCODE step additionally clamps `position` to
`±32000` rather than wrapping, and sets `kFlagFaultPositionClamped`
(§8.2 bit 17) when it does. A host observing a `positionEpoch` change
knows a rebase occurred even if it missed the exact boundary frame — it
may ignore the discontinuity (accepting at most one cycle's worth of
uncertainty at the boundary) or sum each epoch's final observed value
into a running total if it cares about total travel since connect.
`Devices::Motor`/`NezhaMotor`/`MotorArmor` are unmodified by this
policy — `RobotLoop` calls an existing primitive, it does not add one.

---

## 6. `ConfigDelta` arm

**Unchanged from protocol v4** — `ConfigDelta.patch`'s `drivetrain`/
`motor`/`otos` arms, their live-vs-`ERR_UNIMPLEMENTED` status, persistence,
and the "arriving mid-`Move` never disturbs the active `Move`" guarantee
are all identical to [`docs/protocol-v4.md`](protocol-v4.md) §6.

---

## 7. Responses

### 7.1 The ack ring — the ONLY per-command outcome path

There is no per-command `ReplyEnvelope`, and — **as of this sprint, no
single-scalar "freshest ack" slot either** (protocol v4's `ack_corr`/
`ack_err` fields 5/6 and `flags` bit 5 `kFlagAckFresh`, added sprint 120
alongside the ring, are DELETED outright: `telemetry.proto` fields 5/6
are `reserved`, not reused, and bit 5 is RESERVED). Every command's
outcome rides `Telemetry` (§8) inside the next push —
`App::Telemetry::ack(corrId, errCode)` pushes onto the bounded ack ring
ONLY now:

- **`acks`** (§8.1 field 14) — a bounded ring, depth 4 (`kAckRingDepth`),
  of packed `uint32_t` entries (`corr_id << 4 | err` — NOT `msg::AckEntry`,
  which is also deleted), oldest evicted first. This is the field
  `wait_for_ack()` (§9) scans, and, since this sprint, the field EVERY
  ack-observability consumer must scan — ring membership alone means
  "really acked," there is no separate freshness concept to check.
  Because the ring holds up to 4 recent acks rather than 1, a host
  reading telemetry at ~15 Hz against the firmware's ~25 Hz (40 ms) emit
  rate still observes a transient enqueue/STOP/CONFIG ack that would
  otherwise be overwritten before the next read.

Why the scalar slot was removed, not merely left additive: it duplicated
the ring's newest entry, and its own freshness bit added complexity for a
"is this genuinely new" question the ring's own membership already
answers unambiguously (a `corr_id` present in the ring was genuinely
pushed at some point — nothing "stale" about it the way a leftover
scalar value could be).

**AS-BUILT (unchanged from v4)**: `ReplyEnvelope`'s `ok` (`Ack`) and
`err` (`Error`) oneof arms remain declared in the schema with **zero live
producers** — `Comms::sendReply()` is called from exactly one call site
(`App::Telemetry::emitPrimary()`), always with `body_kind = TLM`. A wire
sniffer will never observe a `ReplyEnvelope{ok: ...}` or
`ReplyEnvelope{err: ...}` frame from this firmware.

### 7.2 Two kinds of ack ride the same ring

Unchanged from protocol v4:

1. **Enqueue/command ack** — `corr_id = CommandEnvelope.corr_id`, `err` =
   the `ErrCode` from dispatch (§7.3). Sent for every `move`/`config`/
   `stop` that reaches a handler.
2. **MOVE completion ack** — `corr_id = Move.id` (NOT the enqueue
   envelope's `corr_id`), sent on the exact cycle the active `Move` ends
   (§5).

Both push onto the SAME ring; the ring (depth 4) keeps all of them,
evicting only the OLDEST entry once full. **Every consumer must scan
`acks`** — this was already true since sprint 120/121-002 for any
consumer that wanted lossy-link-safe completion observability, and is now
the ONLY option, full stop, since the scalar slot no longer exists.
`wait_for_ack()` (§9) scans the ring.

The wire-visible-latency behavior from v4 (119 ticket 005 — telemetry
emit runs AFTER `processMessage()`'s own dispatch, so an enqueue/command
ack typically rides the SAME cycle's frame; a MOVE completion ack still
rides the NEXT cycle's frame, since `moveQueue_.tick()` runs after the
emit call) is unchanged.

### 7.3 AS-BUILT: the completion ack's `err` is always 0 — timeout is signaled by the flags bit

Unchanged from protocol v4 (`robot_loop.cpp`):

```cpp
tlm_.setLiveFlag(kFlagFaultMoveTimeout, moveTimedOut);
if (moveResult.completed) {
  tlm_.ack(moveResult.completion.moveId, 0);   // err is ALWAYS 0 here
}
```

The host distinguishes a stop-condition completion from a timeout
completion **only** via `flags` bit 15 (`kFlagFaultMoveTimeout`, §8.2) on
the same frame that carries the completion ack — never via a nonzero
`err`.

### 7.4 A malformed/undecodable frame gets no reply at all

Unchanged in spirit from protocol v4, generalized to the new grammar:
`Comms`'s binary-frame decode path (malformed COBS, a CRC mismatch, or
`msg::wire::decode()` failure — §2.2) **never replies synchronously** —
it increments `malformedCount_` and returns, leaving `Cmd.status` at
`kNone`. An unrecognized `<COMMAND>` name (§2.1) increments the SAME
counter, before any framing is even attempted — one fault-bit source
(`kFlagFaultCommsMalformed`, `comms.malformedCount() > 0`), covering both
"the command name doesn't exist" and "the command's data is malformed."
`ERR_DECODE` remains a schema-declared `ErrCode` value with **no live
wire producer** — no ack this firmware sends will ever carry it.

### 7.5 Error taxonomy (`envelope.proto`'s `ErrCode`)

**Unchanged from protocol v4** — see
[`docs/protocol-v4.md`](protocol-v4.md) §7.5 for the full table
(`ERR_NONE`/`ERR_UNKNOWN`/`ERR_BADARG`/`ERR_RANGE`/`ERR_FULL`/
`ERR_DECODE`/`ERR_UNIMPLEMENTED`/`ERR_OVERSIZE`/`ERR_NOT_CONFIGURED`) —
this sprint added no new code and changed no code's live-producer status.

---

## 8. Telemetry frame reference

Rides `ReplyEnvelope{corr_id=0, tlm: Telemetry}` (unsolicited, `corr_id`
always 0), emitted **every loop cycle** — primary period == cycle period,
~25 Hz / 40 ms (`App::Telemetry::kPrimaryPeriod`), unchanged from v4.
**There is only ever one telemetry frame now** — `msg::TelemetrySecondary`,
the slower ~5 Hz diagnostic frame protocol v4 §8.4 described, is DELETED
outright (sprint 124 ticket 009), not merely unused: it emitted nothing
but `now` in production (no firmware caller ever populated its
`cmd_vel`/`acc_*`/`glitch_*`/`ts_*` fields), and `RobotState ⊇ wire`
means there was no wire-visibility invariant obligating any of its fields
to survive — nothing did. "The frame is the dataset" is now unqualified:
one timestamped frame every iteration is the entire dataset-construction
path.

**Source: `RobotState`, not a hand-copied struct.** `App::RobotLoop`
publishes `Types::RobotState` (`src/firm/types/robot_state.h`) — a
dependency-free blackboard struct, the sole cross-subsystem AND
cross-tree (`src/firm`↔`src/motion`) data contract — once per cycle, each
section at its own coherence point. `Telemetry::update(const RobotState&)`
is the ONE method that reads it and stages the whole next wire frame:
`RobotLoop::cycle()` contains exactly one `tlm_.update(state)` call and
zero direct `tlm_.setFlag()` calls (grep-enforceable:
`grep setFlag src/firm/app/robot_loop.cpp` returns nothing) —
`kFlagFaultMoveTimeout`/`kFlagFaultShapingDisabled` are the one
documented exception, set via `tlm_.setLiveFlag()` after
`MoveQueue::tick()` runs, since their defining condition isn't known at
`update()` time.

### 8.1 `Telemetry` fields

| Field | # | Type | Always present? |
|---|---|---|---|
| `now` | 1 | `uint32` [ms] robot clock at frame assembly | always |
| `seq` | 2 | `uint32` | always — increments once per SENT primary frame, wraps at 128 |
| `mode` | 3 | `DriveMode` | always — `VELOCITY` iff `moveQueue_.active()`, else `IDLE` |
| `flags` | 4 | `uint32` bit-string — see §8.2 | always |
| ~~`ack_corr`/`ack_err`~~ | ~~5/6~~ | **DELETED** — `reserved`, not reused | — |
| `enc_left` / `enc_right` | 7 / 8 | `EncoderReading` — see below | always |
| `otos` | 9 | `OtosReading` — see below | valid iff `flags` bit 0 |
| `pose` | 10 | `Pose2D{x,y[mm], h[rad]}` — `sint32`/zigzag packed, `(scale)`, encoder-odometry integrated pose | always |
| `twist` | 11 | `BodyTwist3{v_x[mm/s], v_y, omega[rad/s]}` — `sint32`/zigzag packed, fused from both wheels' measured velocities | always (`v_y` always 0 on this differential build) |
| `line` | 12 | `uint32`, 4 packed 1-byte channels (ch1 low byte) | valid iff `flags` bit 13 |
| `color` | 13 | `uint32`, packed RGBC (R low byte) | valid iff `flags` bit 14 |
| `acks` | 14 | `repeated uint32`, packed, up to 4 — the ONLY ack-observability field (§7.1) | always present (may be an empty list) |
| `cycle_busy` | 15 | `uint32` [us] `cycleStart` → frame-staging instant, THIS cycle | always |
| `cycle_period` | 16 | `uint32` [us] this cycle's `cycleStart` minus the previous cycle's | always (0 on the first-ever cycle) |

**`EncoderReading`** (per wheel):

| Field | # | Type | Notes |
|---|---|---|---|
| `position` | 1 | `sint32`, `(scale)=1.0`, `(abs_max)=32000` | [mm] accumulated — see §5's rebaseline policy |
| `velocity` | 2 | `sint32`, `(scale)=0.1`, `(abs_max)=4000` | [mm/s] signed, measured |
| `age` | 3 | `uint32`, `(max)=255` | [ms] `now` minus this sample's own genuine collect time (`Devices::Motor::sampleTime()`) — NOT an absolute timestamp; renamed from v4's `time` field, which read absolute `[ms]` |
| `position_epoch` | 4 | `uint32`, `(max)=127` (sizing-only; real storage is a full 8-bit counter) | wraps; +1 each `RobotLoop`-triggered `rebaseline()` (§5) — NEW this sprint, no v4 equivalent |

**`OtosReading`** (one burst):

| Field | # | Type | Notes |
|---|---|---|---|
| `x` / `y` | 1/2 | `sint32`, `(scale)=1.0`, `(abs_max)=32000` | [mm] |
| `heading` | 3 | `sint32`, `(scale)=0.001`, `(abs_max)=3142` | [rad] |
| `v_x` / `v_y` | 4/5 | `sint32`, `(scale)=0.1`, `(abs_max)=4000` | [mm/s] |
| `omega` | 6 | `sint32`, `(scale)=0.01`, `(abs_max)=1000` | [rad/s] |
| `age` | 7 | `uint32`, `(max)=255` | [ms] same rename/rationale as `EncoderReading.age` |

Every `EncoderReading`/`OtosReading`/`Pose2D`/`BodyTwist3` field that was
a `float` (fixed32, flat 4 bytes) in protocol v4 is now a protobuf
`sint32` (zigzag varint) with a GENERATED `(scale)` conversion
(`options.proto` extension 50007) — the generator emits a
`pack<Field>(float) -> int32_t`/`unpack<Field>(int32_t) -> float` method
pair on each field's own generated struct; every caller (`App::Telemetry::
update()`) uses these, never hand-copied inline arithmetic. A
zigzag-mapped small-magnitude value costs far fewer bytes than a `float`'s
flat 4 (e.g. `0` costs 1 byte) — this shrink, together with `acks`'
packing (below), is what makes the ≤130 B `ReplyEnvelope` gate reachable.

`age`, not an absolute timestamp, is what makes independent per-sample
capture skew (`enc_left.age` and `enc_right.age` differing by roughly the
left/right settle-collect separation, never equal, never zero) wire-
representable at all — an absolute `[ms]` clock value grows for the whole
session and could never be packed small.

### 8.2 `flags` bit table

Bits 0-4 and 6-16 are **unchanged from protocol v4** (`kFlagOtosPresent`,
`kFlagOtosConnected`, `kFlagActive`, `kFlagConnLeft`/`kFlagConnRight`,
`kFlagFaultI2CSafetyNet`, `kFlagFaultWedgeLatch`, `kFlagFaultI2CNak`,
`kFlagFaultCommsMalformed`, `kFlagEventDeadmanExpired` (orphaned,
unchanged), `kFlagEventBootReady`, `kFlagEventConfigApplied`,
`kFlagLinePresent`, `kFlagColorPresent`, `kFlagFaultMoveTimeout`,
`kFlagFaultShapingDisabled`) — see
[`docs/protocol-v4.md`](protocol-v4.md) §8.2 for their full descriptions.
Two bits change:

| Bit | Constant | Meaning |
|---|---|---|
| 5 | *(reserved)* | **DELETED** — was `kFlagAckFresh` (protocol v4); deleted along with the scalar ack slot it gated (§7.1). Reading this bit is meaningless now. |
| 17 | `kFlagFaultPositionClamped` | **NEW.** A wheel's position was clamped to `EncoderReading.position`'s own `(abs_max)` at the encode step rather than allowed to wrap (§5's rebaseline policy, defensive fallback). Not the expected path — `RobotLoop`'s own per-cycle rebaseline trigger (2000 mm margin) should prevent this in normal operation; purely observable evidence the fallback engaged. |
| 18-31 | — | reserved |

### 8.3 Measured sizes

`gen_messages.py`-measured (`wire.h`'s own `kReplyEnvelopeMaxEncodedSize`
comment is the generator's own regenerated ground truth, re-measured on
every build):

- **Protocol v4 (pre-this-sprint)**: `Telemetry` standalone 188 B,
  wrapped as `ReplyEnvelope.body`'s `tlm` arm 194 B total.
- **Protocol v5 (this sprint, ticket 008)**: `ReplyEnvelope`'s `tlm` arm
  now measures **126 B**, for a **130 B total** (`ok`=19B, `err`=10B,
  non-oneof=4B) — this despite the ADDITIVE `position_epoch` field,
  because the `sint32`/zigzag+`(scale)` conversion (§8.1) and packed
  `acks` (a single `uint32` per ring entry instead of a nested `AckEntry`
  sub-message's own tag+length+two-field overhead) shrink every other
  field far more than `position_epoch` costs. This is the exact ticket
  008 gate (`kReplyEnvelopeMaxEncodedSize <= 130`) — confirm against
  `wire.h`'s own regenerated constant before assuming this number is
  still current.
- **`CommandEnvelope`**: unchanged, **55 B** total (§2.3).

### 8.4 `TelemetrySecondary` — deleted outright

`msg::TelemetrySecondary` — the message type, its `encode()` overload,
`kTelemetrySecondaryMaxEncodedSize`, its own independently-COBS+CRC-framed
wire arm, and the tie-break/alternation cadence machinery `Telemetry::emit()`
used to run to choose between it and the primary frame — no longer exist
anywhere in the tree (sprint 124 ticket 009). See
[`docs/protocol-v4.md`](protocol-v4.md) §8.4 for its historical field
list and rationale; nothing survives into protocol v5.

---

## 9. Host API examples (`src/host/robot_radio/robot/protocol.py`)

`NezhaProtocol` is fire-and-poll for every command — each call writes the
envelope and returns the assigned `corr_id` immediately; pair with
`wait_for_ack()` to confirm the outcome. Unchanged from protocol v4
except where the reply plane itself changed:

```python
from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

conn = SerialConnection(port="/dev/cu.usbmodem2121102")
conn.connect()
proto = NezhaProtocol(conn)

# Bounded twist MOVE: drive forward at 150 mm/s for up to 2000 ms of
# elapsed time (TIME stop condition), 5000 ms timeout backstop,
# replace=True (the default) preempts anything already running.
corr = proto.move_twist(150.0, 0.0, 0.0, stop_time=2000.0, timeout=5000.0)
ack = proto.wait_for_ack(corr, timeout=500)
assert ack is not None and ack.ok

# Bounded wheels MOVE: both wheels at 100 mm/s until 300 mm of path has
# been traveled (DISTANCE stop condition), enqueued behind whatever is
# currently active (replace=False).
proto.move_wheels(100.0, 100.0, stop_distance=300.0, timeout=4000.0,
                   replace=False)

# Panic stop.
proto.stop()

# Live-tune a motor gain (persisted across power-cycle, §6).
proto.config(**{"pid.kp": 0.02})

# Live-apply an OTOS calibration offset (also persisted, §6).
proto.otos_config(offset_x=-51.5, offset_y=0.0)

# Drain telemetry (§8) as parsed TLMFrame objects.
frames = proto.read_pending_binary_tlm_frames()
for f in frames:
    for entry in f.acks:  # the bounded ack ring -- the ONLY ack path now
        print(entry.corr_id, entry.ok, entry.err_code)
    if f.fault_move_timeout:
        print("a MOVE just timed out")
```

`stop_time`/`stop_distance`/`stop_angle` are mutually exclusive
keyword-only args, mirroring `Move.stop`'s own oneof — unchanged.

**`wait_for_ack(corr_id, timeout)`** scans `acks` (§7.1) across however
many `Telemetry` pushes arrive within `timeout`, returning the FIRST
matching entry (or `None`) — unchanged mechanism from v4, except that
scanning the ring is now the ONLY option (the scalar-slot fallback v4
still described for old readers no longer exists on the wire at all).

**Identity round-trip (NEW this sprint)**:

```python
conn.send_text("PING")            # -> "PONG:t=<ms>" (was "OK pong t=<ms>")
conn.send_text("ID")              # -> "ID:<drivetrain>:<profile>:<version>"
conn.send_text("VER")             # -> "VER:<version>"
```

**`tlm_log.py`** (`src/tests/bench/tlm_log.py`) remains the bench dataset
logger, unchanged in shape — one CSV row per frame, every `flags`-derived
boolean and every `EncoderReading`/`OtosReading` sub-field (now including
`position_epoch` and each reading's real `age`).

---

## 10. Worked wire examples

The `CommandEnvelope`/`Move` schema is byte-identical to protocol v4's
(§4) — the raw protobuf field encoding for a `move`/`stop`/`config`
command is UNCHANGED (`corr_id`, `move{...}`, etc. all still encode the
same way; see [`docs/protocol-v4.md`](protocol-v4.md) §10 for those exact
byte dumps). **What changed is the wire LINE the raw bytes are wrapped
in**: protocol v4 emitted `*B<base64(COBS(payload+CRC))>` on its own
line; protocol v5 emits `<VERB>':'<COBS(payload+CRC), 0x0A-keyed>'\n'` —
the same CRC-then-COBS composition, now `0x0A`-keyed and CRC-scoped over
`VERB ':'` too (§2.2), with an ASCII verb name prepended instead of a
`*B` sigil, and no base64 step at all. Rather than hand-transcribe a COBS
+CRC byte dump here (error-prone, and this document's own standing rule
is "generated directly from real bindings, never hand-computed" — see
this document's own header), the authoritative byte-exact reference is
the cross-language golden vector fixture (sprint 124 ticket 004,
`src/tests/`) that both the C++ and pytest suites assert against
byte-for-byte, plus the sim/loopback framing test (ticket 006) that
exercises the real encode→COBS→decode chain end to end off-hardware. A
`MOVE:`-prefixed line for the `MoveTwist`+TIME-stop example protocol v4
§10 shows in raw-byte form would read, schematically:

```
MOVE:<COBS-encoded(<26-byte raw CommandEnvelope> + <2-byte CRC-16 over "MOVE:" + those 26 bytes>), 0x0A-keyed>
```

---

## 11. Deliberately NOT in this protocol

Arc/segment moves, planned/time-optimal trajectory profiles (Ruckig-style,
uploaded or planned ahead of time with a known arrival time), heading
cascade, pose-fix injection, `GET`/`STREAM`/`ECHO`, plan dumps, ring dumps
— all reserved wire numbers, all recoverable from the
`pre-gut-motion-stack` tag if ever needed. The protocol is: **bounded
velocity commands in, timestamped measurements out.** There is also no
`HELP`/`SET`/`GET` text verb, and no free-form text command parser of any
kind — the closed four-cleartext-verb registry (§2.4) is the entire
cleartext surface, generated from the same one schema as the binary
verbs, not a separate legacy rump.

---

## 12. Verification

The bench protocol gate this document's contract is verified against
lives in sprint 124 ticket 013
(`clasi/sprints/124-protocol-v5-robotstate-blackboard-and-radio-bench-gate/tickets/done/013-radio-relay-standing-bench-gate-...md`)
— run over the **radio relay**, not USB, per stakeholder directive: the
`DEVICE:` banner at a fresh connect with zero `HELLO` polling,
`HELLO`→`DEVICE:`/`PING`→`PONG:`/`ID`→`ID:`/`VER`→`VER:` all answered,
`kFaultCommsMalformed` staying clear through a clean connect (ticket 010),
a `0x0A`-embedding `move_wheels` running 10/10, encoder positions
climbing in telemetry while a move runs, both enqueue and completion acks
observed via the ring, and a `wire_truth`-equivalent wire-quality
measurement against a stated loss budget (ticket 012) alongside the
equivalent USB measurement. The bench script catalogue lives in
[`src/tests/bench/`](../src/tests/bench/); see
[`.claude/rules/hardware-bench-testing.md`](../.claude/rules/hardware-bench-testing.md)
for the general stand-testing procedure. Cross-language golden vectors
(ticket 004) and the sim/loopback framing path (ticket 006) catch a
framing regression in CI before it ever reaches the bench.

---

## Appendix: superseded documents

- [`docs/protocol-v2.md`](protocol-v2.md) — the original text-only
  protocol (pre-097). Superseded by v3 for motion/config/telemetry, and
  by v4 (then v5) for everything still live in v3.
- [`docs/protocol-v3.md`](protocol-v3.md) — the post-097 binary-envelope
  + text-rump + `rogo` proxy protocol, frozen at sprint 097. Superseded
  by v4 (then v5).
- [`docs/protocol-v4.md`](protocol-v4.md) — the MOVE-protocol command
  surface (sprint 116) with COBS+CRC binary framing (sprint 123),
  `0x00`-delimited, with a separate `HELLO`/`PING` text rump and a
  single-scalar-plus-ring ack slot. Superseded by this document (sprint
  124's protocol v5 grammar/registry/telemetry-packing cutover) for
  every section it covers — its command-surface content (§3-§6) remains
  byte-accurate history for what shipped when, since this sprint did not
  change that surface.
