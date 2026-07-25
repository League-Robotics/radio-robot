---
status: pending
filed: 2026-07-25
filed_by: team-lead (stakeholder directive, protocol design session 2026-07-25)
related:
- cobs-crc-binary-framing-replace-base64-armor.md
- relay-handshake-trips-comms-malformed.md
- binary-plane-bursty-reply-loss-on-hardware-post-gut-regression.md
- telemetry-frame-is-the-robot-state.md
- tlm-rate-15-19hz-vs-50hz-nominal-serial.md
tickets: []
---

# Protocol v5: one line per packet, packed telemetry, state-derived frame assembly

Two parts, one atomic wire break:

- **Part A — framing.** ASCII command prefix, `\n`-delimited COBS, restored
  DEVICE announcement. Directive of 2026-07-25.
- **Part B — telemetry.** Fixed-point field packing, real per-sample
  timestamps, and one-line frame construction from a single robot-state
  object. Directive of 2026-07-25 (later the same session).

Both change the wire. Landing them together costs one cutover and one budget
re-derivation instead of two; Part B's packing is also what makes Part A's
per-packet command prefix free in the size budget.

# Part A — framing

## Stakeholder directive (2026-07-25)

> I want to restore the device announcement and the response to HELLO. The
> announcement looks like this: `DEVICE:NEZHA2:robot:<deviceName>:<serialNumber>`
>
> I'd like to have some commands be in cleartext, and some be in binary. I'd
> also really like each packet, even if it's binary or clear text, to go on a
> single line.
>
> HELLO, PING, ID must be cleartext.
>
> Let's say that every command is in ASCII cleartext, so the form of a packet
> is `COMMAND<data>\n`. For some commands the `<data>` will be cleartext, but
> currently all of those (HELLO, PING, ID, VER) have no data. If there is
> data, it is interpreted as either cleartext or binary according to the
> command. For binary, the data is `<COBS ZERO POS><data><CRC>`.

Follow-up decisions taken in the same session:

- Use `:` as the command/data separator, **not** required for the no-data
  verbs (HELLO, PING, ID, VER).
- Run COBS against `0x0A` rather than `0x00`, implemented by XOR-ing the
  existing encoder's output (see §2) — stakeholder accepted this once the
  cost was established as ~2 lines per side rather than a new algorithm.
- **The CRC covers the command name and separator**, not just the payload
  (§3). Stakeholder: *"Okay, do that. Add and extend the CRC to cover the
  command."*
- **The reply plane adopts the same grammar** as the command plane (§4).
  Stakeholder: *"Okay, make that change to just be consistent."*
- The radio's RAW250 mode means fragment size is **not** a constraint on the
  added prefix bytes (corrects an earlier 19-byte-fragment assumption).

## Problem: two framings, two guess-based demuxers, one missing banner

### The wire today (post-123)

A binary packet has no command name at all — the verb lives inside the
protobuf envelope:

```
COBS_0x00( <envelope bytes> <crc16 LE> )  0x00
```

CRC-16/CCITT-FALSE, little-endian, appended to the payload **before** COBS
(`Comms::sendReply()`, [src/firm/app/comms.cpp:166-202](../../src/firm/app/comms.cpp)),
so the CRC rides inside the encoded region and can never emit a literal
delimiter. The text plane is a separate shape on the same byte stream:
`HELLO`/`PING` in, `DEVICE:...`/`OK pong t=<ms>` out, `\r\n`-terminated.

### Why that is unstable

COBS guarantees `0x00`-freedom. It guarantees nothing about `0x0A`. A
`move_wheels` envelope containing a literal `0x0A` byte was being split and
corrupted at the terminator — proven 0/10 on hardware, fixed in `febfb450`
(123-006). The fix was **two different heuristics**, one per direction:

- Firmware inbound: an exact match against a literal table,
  `kTextCommands[] = {"HELLO","PING"}`
  ([src/firm/com/serial_port.cpp:17-30](../../src/firm/com/serial_port.cpp)).
  A `0x0A` ends a text line only if the bytes before it are exactly one of
  those two strings.
- Host inbound: a printable-ASCII content check, `_looks_like_text()` /
  `_TEXT_SAFE_BYTES`
  ([src/host/robot_radio/io/wire_codec.py:79-100](../../src/host/robot_radio/io/wire_codec.py)),
  because the replies (`DEVICE:`, `OK pong t=`, relay `#` comments) are not a
  fixed literal set.

Both are guesses about whether a `0x0A` is a terminator or content. Costs:

1. `readline()` is unsafe against this wire; every host read must go through
   `ByteStreamDemuxer`.
2. Adding any new text verb requires editing a firmware literal table, or the
   firmware silently reclassifies it as binary and counts it malformed.
3. `_looks_like_text()` carries its own documented near-miss (a short COBS
   frame's leading code byte can land on `0x09`, which is why tab is
   deliberately excluded from the alphabet) — the class of bug is contained,
   not eliminated.
4. Two independent recognizers that must stay behaviourally consistent across
   two languages, with no shared test vector forcing them to.

### And the announcement was gone

At the time this issue was opened, `formatBanner()` produced the byte-frozen
`DEVICE:NEZHA2:robot:<name>:<serial>` and `Comms` held it, but it was only
ever sent from the `HELLO` branch
([comms.cpp:92-94](../../src/firm/app/comms.cpp)) — **nothing emitted it at
boot**. The host expects one: `serial_conn.py` documents "the relay emits a
`DEVICE:` boot announcement" and only falls back to polling `HELLO` up to 10×
over a 2.5 s budget (`_HELLO_CLASSIFY_TIMEOUT_S`) when one does not arrive,
so every connect took that slow path.

`ID` and `VER` do not exist in firmware at all — `kTextCommands` is
`{"HELLO","PING"}`. The host has a vestigial `ID` reply-routing branch
([serial_conn.py:~896](../../src/host/robot_radio/io/serial_conn.py)) from
the pre-v4 protocol with nothing behind it.

#### Work in flight (uncommitted, observed 2026-07-25)

The announcement half is **already being restored** in the working tree,
outside this issue. Do not re-plan it; scope around it:

- `src/firm/com/banner.{h,cpp}` (new, untracked) — `formatBanner()` moved out
  of `main.cpp` into `com/`, where the ARM-only CODAL identity calls belong.
- `Comms::sendBanner()` (new, `comms.cpp:204-211`) — `sendReliable()` on both
  transports.
- `main.cpp:96-97` — power-on emission, immediately after `radio.begin()`.
- `robot_loop.cpp` `boot()` — a **second**, byte-identical emission after the
  preamble loop and before the first `cycle()`, so a host that opened its
  port mid-preamble still gets identity; and `comms_.pump()` now runs inside
  the boot loop so `HELLO`/`PING` answer while devices are still probing.

Net effect: the boot-emission and text-rump-during-boot gaps are closed. What
remains for this issue is `ID`/`VER`, the reply-plane grammar (§4), and the
framing change itself (§1-§3). Both of those emissions become `\n`-terminated
rather than `\r\n` under §7.

## The change

### 1. Uniform packet grammar

Every packet, text or binary, in both directions, is exactly one line:

```
<COMMAND>[':' <data>] '\n'
```

- `<COMMAND>` — uppercase ASCII from a closed set.
- No-data verbs (`HELLO`, `PING`, `ID`, `VER`) carry no `:` — the line is the
  bare command.
- With data, exactly **one** separator: the **first** `:` ends the command.
  Every later byte, including further `:` bytes, is data. This is what makes
  the parse unambiguous even though binary data can contain `0x3A`.
- Whether `<data>` is cleartext or binary is a property of the command, known
  before the data is parsed. Nothing about the data's own bytes is inspected
  to decide.
- `\n` (`0x0A`) is an **unconditional** terminator. No heuristics, either
  direction.

The existing announcement already satisfies this grammar unmodified:
`DEVICE:NEZHA2:robot:<name>:<serial>` parses as command `DEVICE`, cleartext
data `NEZHA2:robot:<name>:<serial>`. No format change, and the byte-freeze
holds.

### 2. COBS against `0x0A` instead of `0x00`

COBS is generic over its delimiter. The cheapest correct implementation keeps
the existing routine untouched and XORs:

```
encode:  frame = cobsEncode(payload) each byte ^ 0x0A
decode:  cobsDecode( wire bytes each ^ 0x0A )
```

**Why this is sound.** `cobsEncode()`'s output is guaranteed `0x00`-free — code
bytes are `>= 0x01` and block data bytes are non-zero by construction
([wire_runtime.h:197-233](../../src/firm/messages/wire_runtime.h)). For any
byte `b`, `b ^ 0x0A == 0x0A` iff `b == 0x00`, which never occurs. Therefore the
XOR-ed stream is `0x0A`-free. `0x00` becomes ordinary content requiring no
special handling.

**The malformed-input checks survive unchanged and get sharper.** A literal
`0x0A` arriving inside a frame body XORs back to `0x00`, which
`cobsDecode()` already rejects as "literal 0x00 code byte" / "literal 0x00
inside data block". That path is unreachable except under corruption — which
is exactly the case it should reject. No new error handling.

Verified round-trip against the existing encoder, including the adversarial
inputs:

```
payload           wire frame                                       0x0A?  rt
all 0x0A (8B)     01 00 00 00 00 00 00 00 00 41 78                 no     ok
all 0x00 (8B)     0b 0b 0b 0b 0b 0b 0b 0b 09 34 3b                 no     ok
0x00..0x0F        0b 18 0b 08 09 0e 0f 0c 0d 02 03 00 01 06 07 ... no     ok
```

Implementation note: prefer a delimiter parameter on
`WireRuntime::cobsEncode()`/`cobsDecode()` (defaulting to `0x00`) over an
XOR at every call site, so the delimiter constant lives in exactly one place
on each side. The XOR is the implementation *inside* those functions.

### 3. CRC scope — covers the command (DECIDED)

Keep CRC-then-COBS composition — non-negotiable, and unchanged from 123: the
CRC must be inside the encoded region or a CRC byte equal to the delimiter
terminates the frame early.

**The CRC's input extends to cover the command name and separator.** The wire
layout is unaffected — the command is already outside the COBS region, and
only the CRC's *input range* changes:

```
crc  = crc16( COMMAND ':' payload )        <-- input extended
line = COMMAND ':' COBS_0x0A( payload ‖ crc_LE ) '\n'
                              ^^^^^^^^^^^^^^^^^ wire layout unchanged
```

Rationale: without it, a bit flip inside the command name that happens to
land on another *valid* verb dispatches the wrong command with a passing CRC.
The alternative mitigation — choosing command names with pairwise Hamming
distance ≥ 2 — is a constraint on every future verb name and is not
checkable at build time. One extra `crcCompute()` pass over ~5 bytes is
cheaper and needs no discipline to maintain.

Consequence for the codec API: `encode_frame()`/`decode_frame()`
([wire_codec.py:214-255](../../src/host/robot_radio/io/wire_codec.py)) and
`Comms::sendReply()`/`decodeBinaryFrame()` no longer take payload alone —
they need the command bytes as a separate CRC-scope argument (the command is
not part of the COBS input). Keep that asymmetry explicit in the signature
rather than concatenating and slicing, so a caller cannot accidentally COBS
the command name.

A cleartext-data command carries no CRC at all — there is no COBS region to
put one in. Corruption on a cleartext line surfaces as a parse failure. This
is unchanged from today's text rump and is accepted: the cleartext verbs are
identity/liveness, not control.

### 4. The reply plane uses the same grammar (DECIDED)

The directive is written host→robot, but the grammar applies in both
directions. **Every line the firmware emits is `<VERB>[':' <data>]'\n'`.**

The text reply inventory is smaller than it looks. Post-123, `OK`/`ERR`/`TLM`
are protobuf `ReplyEnvelope` oneof arms on the binary plane; the only text
lines firmware actually emits are the two `sendReliable()` call sites
([comms.cpp:93, 105](../../src/firm/app/comms.cpp)) plus the new banner
emissions:

| Trigger | Today | Post-v5 | Note |
|---|---|---|---|
| boot / `HELLO` | `DEVICE:NEZHA2:robot:<name>:<serial>` | unchanged | already conforms — byte-freeze holds |
| `PING` | `OK pong t=<ms>` | `PONG:t=<ms>` | the one behavioural break |
| `ID` | — | `ID:<fields>` | new |
| `VER` | — | `VER:<version>` | new |

`OK pong t=<ms>` → `PONG:t=<ms>`: `OK` as a verb fails the grammar's own
principle, that the verb determines how the data is read — `OK`'s data means
whatever the request meant. On the text rump every request has a distinct
reply, so a verb per reply is both simpler and conformant. (`OK:pong t=<ms>`
would satisfy the letter of the grammar and not its point; rejected.)
A reply verb echoing its command verb (`ID`→`ID:`, `VER`→`VER:`) is
unambiguous because link direction is known — `HELLO`→`DEVICE:` already
breaks verb symmetry anyway.

Host-side, this exposes dead code rather than creating work. These branches
in `_handle_text_line()`
([serial_conn.py:~880-905](../../src/host/robot_radio/io/serial_conn.py))
are pre-v4 vestiges with **no firmware emitter** behind them:

- `text.startswith("TLM")` → `_tlm_queue` — telemetry is binary now.
- `text.startswith("EVT")` → `_evt_queue` — no emitter.
- `text.startswith(("OK","ERR","CFG","ID"))` corr-id routing, and
  `_CORR_ID_RE` — corr-id'd replies ride `ReplyEnvelope`. The `ID` case is
  the one that becomes live again, in its new `ID:` form.

Delete them as part of this change rather than porting them to the new
grammar; carrying dead text-plane routing forward is how the two planes got
confusable in the first place. Relay `#`-comment lines are *not* ours and
stay handled as-is.

### 5. Worked example

Command `MOVE`, a 9-byte envelope payload containing two `0x00` bytes and one
`0x0A` byte (i.e. the exact shape that broke on hardware in 123-006):

```
CRC input     4d 4f 56 45 3a 08 07 1a 00 22 03 00 41 0a     "MOVE:" + payload
crc16                                            0x03d9
payload‖crc   08 07 1a 00 22 03 00 41 0a d9 03              11 B
cobs (0x00)   04 08 07 1a 03 22 03 05 41 0a d9 03           12 B
^ 0x0A        0e 02 0d 10 09 28 09 0f 4b 00 d3 09           12 B, 0x0A-free
on the wire   4d 4f 56 45 3a  0e 02 0d 10 09 28 09 0f 4b 00 d3 09  0a
              M  O  V  E  :   <------------ frame ------------->   \n
```

Total 18 bytes for a 9-byte payload: 5 prefix + 11 payload‖crc + 1 COBS + 1
terminator.

Reading the COBS layer (pre-XOR): `04 | 08 07 1a` — three data bytes then a
zero was here; `03 | 22 03` — two data bytes then a zero was here;
`05 | 41 0a d9 03` — four data bytes, end of frame, no zero. Note the
original data's zeros are **gone**, overwritten in place by the offsets. A
decoder cannot treat the region after the leading code byte as opaque
payload; it must walk the chain to restore them.

### 6. Sizing — the prefix is affordable

- **COBS overhead is exactly one byte** for any payload under 254 bytes,
  regardless of how many zeros it contains (verified: a 200-byte payload
  encodes to 201 whether it holds 0, 5, 50, or 200 zeros). Code bytes occupy
  the positions the zeros vacated; they are substituted, not inserted.
- **Single-code-byte territory ends at 253 combined bytes**, not 254 —
  verified empirically: 253 zero-free bytes encode to 254 (one code byte),
  254 encode to **256** (two), because the encoder's `code == 0xFF` branch
  flushes a full block and opens a fresh one that then takes a trailing
  `0x01`. So `payload ≤ 251`. The 240-byte envelope budget
  (`kEnvelopeBudgetBytes`, [wire.h:53-78](../../src/firm/messages/wire.h))
  clears it.
- **Serial** has the hard ceiling: `setTxBufferSize(255)` is a `uint8_t` max,
  254 usable. Worst frame today is 194 payload + 2 CRC → 197 COBS + 1
  delimiter = 198. Adding `TLM:` (4 B) gives 202. 52 B of headroom.
- **Radio is not a constraint.** RAW250 framing, `MICROBIT_RADIO_MAX_PACKET_SIZE=250`,
  MTU 247 payload bytes per fragment
  ([src/firm/com/radio.h:8-14, 89](../../src/firm/com/radio.h)). The full
  198-byte frame is a single fragment today and stays single with the prefix.
- `kFramedMaxBytes` / `kMaxCrcPayloadBytes` (comms.h) need re-derivation
  against the new worst case (longest command name + separator).

### 7. What gets deleted

This is the payoff, and it should be visible in the diff as removal:

- `kTextCommands[]` and `isRecognizedTextCommand()`
  ([serial_port.cpp:17-30](../../src/firm/com/serial_port.cpp)) — gone.
  `SerialPort::readLine()` reads to `\n`, unconditionally.
- `_looks_like_text()` and `_TEXT_SAFE_BYTES`
  ([wire_codec.py:79-100](../../src/host/robot_radio/io/wire_codec.py)) —
  gone. `ByteStreamDemuxer` collapses to split-on-`\n`, and plain
  `readline()` becomes safe against this wire again.
- `App::FrameKind` (kText/kBinary) as a *transport-level* concept
  ([comms.h:48](../../src/firm/app/comms.h)) — the transport delivers a line;
  `Comms` decides text-vs-binary from the parsed command. `Transport::readLine()`
  likely reduces to a bool.
- `Radio::send()` (appends `0x00`) and `Radio::sendText()` (appends `\n`)
  converge on one terminator ([radio.cpp:83, 146](../../src/firm/com/radio.cpp)).
- `\r` handling. Text replies are `\r\n` today (`sendReliable()`); under one
  uniform rule `\r` is legal binary content, so strip-trailing-`\r` cannot be
  unconditional. Emit bare `\n` everywhere; strip `\r` only for cleartext
  commands, which the parsed prefix already identifies.

### 8. The announcement and the text verbs

**Partly done already — see "Work in flight" below.** What this issue still
owns:

- **`ID`** → `ID:<fields>`. New. Decide the field set; the host's dead `ID`
  routing branch is the only prior art and it is unspecified.
- **`VER`** → `VER:<version>`. New. Needs a runtime version source (open
  question 3).
- **`PING`** → `PONG:t=<ms>`, migrated per §4.
- All four commands are cleartext, no data, no `:` on the inbound side.
- Confirm the banner survives the grammar change untouched: `DEVICE` parses
  as the verb, `NEZHA2:robot:<name>:<serial>` as cleartext data on the first
  `:`. The byte-freeze holds and `formatBanner()` needs no edit.

---

# Part B — packed telemetry, built from state

## Stakeholder directive (2026-07-25, same session)

> Pack the telemetry and reduce its size. Make the telemetry constructable
> from the state. What should be happening in the main loop is that we create
> or update the state object, and then at the very end we send telemetry by
> construct — one line that updates the telemetry with the state object. All
> that stuff that's currently all over the main loop should be in that one
> function call, then you send telemetry.
>
> `enc_left.time` and `enc_right.time` should not be the same thing. That's
> terrible. They need to be independent, because that's the whole point of
> having two of them: so you can tell when those values were collected. Make
> them deltas from `now`.

## B1. The shape

Today `cycle()`'s pace block fills **two** parallel structs from the same
primary sources, field by field, then a third object (`Telemetry`'s internal
flag word) via ten scattered `setFlag()` calls:

```
motorL_/motorR_/odom_/otos_/line_/color_  ──┬─→ Motion::StateEstimator::Input  (14 fields, hand-copied)
                                            ├─→ App::Telemetry::Frame          (via assembleFrame(), 10 args)
                                            └─→ tlm_.setFlag() × 10
```

`assembleFrame()` already takes **ten arguments** because the primaries it
needs are scattered across the block's local scope. That argument list is the
symptom.

The target:

```
                    ┌──────────────┐
primary sources ──→ │  RobotState  │ ──→ stateEstimator_.update(state)
   (read once)      │  (floats)    │ ──→ tlm_.update(state)   ← the one line
                    └──────────────┘ ──→ tlm_.emit(now)
```

One struct, filled once, consumed three times. `Telemetry::update(const
RobotState&)` absorbs everything `assembleFrame()` does today **including all
ten `setFlag()` calls** — the flags are derived from state, not set alongside
it.

`Motion::StateEstimator::Input` (`src/motion/state_estimator.h:118-137`) is
already 90% of `RobotState`: the same encoder/pose/twist/otos fields, in
floats, with the same units. It is the natural seed for the type.

### RobotState is NOT the wire frame

This is the point that resolves the tension recorded in
`telemetry-frame-is-the-robot-state.md` ("make the telemetry frame exactly
equal to the robot state — don't fill up two objects"). Once the wire is
fixed-point, the frame **cannot** be the state:

- `RobotState` holds floats, in the units the odometry, the estimator, and
  the PID actually compute in. It is the truth.
- `msg::Telemetry` holds scaled integers, bounded, lossy. It is a
  **projection** of the truth for transmission.

So the resolution is not "one object" but "one *source* object and one
projection function." That still eliminates the duplication the earlier issue
was aimed at — the hand-copy from `Frame` to `Input` disappears, and there is
exactly one place where a state field becomes a wire field. `RobotState` also
stays float-typed and therefore stays usable by `src/motion`, which must not
depend on `messages/`.

### What leaves `cycle()`

Everything between `odom_.integrate()` and `tlm_.emit()` in the pace block
(`robot_loop.cpp:698-780ish`) collapses to: read the primaries into `state`,
then three calls. Specifically these go away as *loop* code and become
*projection* code: the ten-argument `assembleFrame()`, the ten `setFlag()`
calls, the fourteen-line `estimatorInput` hand-copy, `packLine()`/
`packColor()`, and the `cycleBusy`/`cyclePeriod` bookkeeping.

## B2. `enc_left.time` / `enc_right.time` — the real defect

The two timestamps are set to the same value in **both** consumers:

```cpp
// assembleFrame(), robot_loop.cpp:196,200
frame_.encLeft.time  = now;
frame_.encRight.time = now;

// estimatorInput, robot_loop.cpp:~752
estimatorInput.encLeftTime  = cycleStart;
estimatorInput.encRightTime = cycleStart;
```

The samples are genuinely **not** simultaneous. `cycle()` interleaves per port
by hardware necessity — the brick holds one pending encoder read
(`robot_loop.cpp:600-655`):

```
motorL_.requestSample()  →  kSettle (≥4ms)  →  motorL_.tick(nowMicros)
                         →  kClear  (≥4ms)
motorR_.requestSample()  →  kSettle (≥4ms)  →  motorR_.tick(nowMicros)
                         →  pace block → assembleFrame() → emit()
```

The left sample is collected **8–12 ms before** the right one, and both before
`now`. That skew is real, it is the exact quantity 119-005 rearranged the loop
to control, and the frame currently throws it away by stamping both with the
same number. Two fields carrying one value is worse than one field: it implies
a measurement that was never made.

### Blocking change: the device layer does not expose sample time

Neither interface has an accessor:

- `Devices::Motor` (`devices/motor.h:84-91`) — `tick(uint64_t nowUs)` receives
  the time and drops it; `position()`/`velocity()` return bare floats.
- `Devices::Otos` (`devices/otos.h:116-118`) — same, `tick(uint64_t nowUs)` /
  `pose()`.

Both already *have* the value internally. Each needs one accessor
(`sampleTime()` [us], the `nowUs` of the tick that produced the current
reading). Without it the timestamps cannot be made honest, and this is the
enabling change for the whole item — do it first.

### Deltas

Wire them as **age**, not absolute time: `age = now - sampleTime`, unsigned,
milliseconds, `(max) = 255`.

| | absolute (today) | age (proposed) |
|---|---|---|
| typical value | 3,600,000 | 8–20 |
| varint width | 4–5 B | 1 B |
| field cost | 5–6 B | 2 B |
| information | none (== `now`) | real capture skew |

Three fields (`enc_left`, `enc_right`, `otos`) × ~4 B saved, and they start
carrying signal. `(max) = 255` also makes a pathological value obvious rather
than silently wrapping.

Naming: `age`, not `age_ms` — units go in a `// [ms]` comment per
`.claude/rules/naming-and-style.md`.

## B3. Fixed-point field encoding

`sint32` with zigzag, **not** `int32`. A negative `int32` is sign-extended to
64 bits before varint encoding — 10 bytes for `-4000`. `wire.cpp:171-177`
already documents this gotcha correctly for the existing `int32` path.

| Field(s) | Scale | Range | `sint32` bound | varint | field |
|---|---|---|---|---|---|
| `position`, `pose.x/y`, `otos.x/y` | 1 mm | ±32 m | `(abs_max) = 32000` | 3 B | 4 B |
| `velocity`, `twist.v_x/v_y`, `otos.v_x/v_y` | 0.1 mm/s | ±400 mm/s | `(abs_max) = 4000` | 2 B | 3 B |
| `pose.h`, `otos.heading` | 1 mrad | ±π | `(abs_max) = 3142` | 2 B | 3 B |
| `twist.omega`, `otos.omega` | 0.01 rad/s | ±10 rad/s | `(abs_max) = 1000` | 2 B | 3 B |

1 mm and 0.1 mm/s are below what the hardware resolves, so this is lossless in
practice, and — unlike a float — the precision is **uniform** across the range.
(A 16-bit float, considered and rejected: protobuf has no 16-bit scalar at all,
and float16's 10-bit mantissa gives ~2 mm resolution at 4 m and ~16 mm at 32 m.
Fixed-point wins on both size and accuracy for bounded quantities.)

### Two generator changes required

1. **`ScalarType::kSint32`** — `WireRuntime::zigzagEncode32()`/`zigzagDecode32()`
   already exist, tested, and unused; `wire_runtime.h:102` says so explicitly.
   The table layer (`wire.cpp:50`) has no enumerator for them, so
   `gen_messages.py` cannot emit a zigzag field. Add the enumerator, the
   encode/decode branches, and the generator mapping.
2. **A `(scale)` field option** — `options.proto` has `(units)`, explicitly
   "informational only." Fixed-point needs the divisor to live in the schema
   so firmware and host generate the *same* constant. A scale mismatch between
   the projection function and the host decoder is the classic fixed-point
   bug: silent, and it looks like a calibration error. Put the number in one
   place and generate both sides.

## B4. Ack ring packing

`repeated AckEntry` costs a tag **and** a length prefix per element — 8 B × 4 =
32 B, the single largest line in the frame. `corr_id` needs 16 bits and `err`
needs 4:

```
repeated uint32 acks = 14 [(max_count) = 4];   // packed: corr_id<<4 | err
```

Packed repeated scalars are one tag + one length + four varints = 14 B. The
engine already declares `FieldKind::kRepeatedScalar` as "PACKED on the wire"
(`wire.cpp:43`), currently unreached by the schema — this would be its first
real use, so it needs test coverage, not just a schema edit.

`ack_corr`/`ack_err` (fields 5/6) then duplicate the ring's newest entry —
delete them, and with them flags bit 5 (`kFlagAckFresh`), whose only job was
to say whether that scalar pair was new. Ring membership already means "really
acked."

## B5. Size accounting

Worst case, per field, current vs proposed:

| Field | Now | Proposed | Δ |
|---|---:|---:|---:|
| `enc_left` | 18 | 12 | −6 |
| `enc_right` | 18 | 12 | −6 |
| `otos` | 38 | 25 | −13 |
| `pose` | 17 | 13 | −4 |
| `twist` | 17 | 11 | −6 |
| `acks` | 32 | 14 | −18 |
| `ack_corr` + `ack_err` | 6 | 0 | −6 |
| `seq` (bound to 16 bits, wraps) | 6 | 4 | −2 |
| `now`, `mode`, `flags`, `line`, `color`, `cycle_*` | 32 | 32 | 0 |
| **Telemetry payload** | **184** | **123** | **−61** |

| | Now | Proposed |
|---|---:|---:|
| Telemetry payload | 184 B | 123 B |
| ReplyEnvelope | 187 B | 125 B |
| On the wire (+CRC, COBS, delimiter) | 191 B | 129 B |
| At 25 Hz | 4775 B/s | 3225 B/s |
| Share of the 115200 link | 41 % | 28 % |

Add Part A's `TLM:` prefix (4 B) and the wire frame is 133 B — still a 30 %
reduction, and the prefix is paid for many times over. This is also the
headroom `tlm-rate-15-19hz-vs-50hz-nominal-serial.md` is asking for.

`kEnvelopeBudgetBytes`, `kReplyEnvelopeMaxEncodedSize`, `kMaxCrcPayloadBytes`
and `kFramedMaxBytes` all get re-derived by `gen_messages.py` from the new
schema — do not hand-edit them.

## B6. Fix the bound violations while the schema is open

Both are live today and both are in fields this change touches anyway (see
`telemetry-frame-inventory.md`, findings 2 and 3):

- `flags` declares `(max) = 65535` but `kFlagFaultShapingDisabled` is
  `1u << 16` = 65536. Widen the bound and fix the proto's bit documentation,
  which still stops at bit 15.
- `ack_err` / `AckEntry.err` declare `(max) = 7` but `ErrCode::ERR_NOT_CONFIGURED`
  is 8, and `robot_loop.cpp:270` acks exactly that. The packed ack word above
  needs 4 bits for `err` regardless — size the bound to the enum, not to 7.

Neither breaks at runtime because `validateBounds()` runs only on decode
(`wire.cpp:549/556/562`), never on encode. Both would fail a firmware-side
round-trip or differential test, which Part A's acceptance criteria introduce.

## Scope / touch points

Firmware:

- [src/firm/messages/wire_runtime.{h,cpp}](../../src/firm/messages/wire_runtime.h) —
  delimiter-parameterised `cobsEncode()`/`cobsDecode()`; §8 header comment is
  written entirely in terms of `0x00` and needs rewriting.
- [src/firm/app/comms.{h,cpp}](../../src/firm/app/comms.h) — command
  prefix parse/emit, CRC scope, `FrameKind` collapse, buffer re-derivation,
  new `ID`/`VER` handlers.
- [src/firm/com/serial_port.cpp](../../src/firm/com/serial_port.cpp) —
  recognizer deletion, terminator unification.
- [src/firm/com/radio.{h,cpp}](../../src/firm/com/radio.h) —
  `send()`/`sendText()` terminator convergence. Also: the comment at
  radio.cpp:135 says `kFramedMaxBytes (192)`; it is 200. Stale since 123-004.
- `src/firm/com/banner.{h,cpp}`, `robot_loop.cpp` `boot()`, `main.cpp:96-97` —
  **in flight, do not re-plan.** Confirm both banner emissions end up
  `\n`-terminated once `sendReliable()` drops `\r\n` (§7).
- [src/firm/main.cpp](../../src/firm/main.cpp) — confirm `VER` has a
  build-version source to read.

Host:

- [src/host/robot_radio/io/wire_codec.py](../../src/host/robot_radio/io/wire_codec.py) —
  the mirror. Delimiter change, recognizer deletion, and the
  `encode_frame()`/`decode_frame()` signature change the extended CRC scope
  forces (§3).
- [src/host/robot_radio/io/serial_conn.py](../../src/host/robot_radio/io/serial_conn.py) —
  `_banner_classify()` fast path; `PONG:` parse; new `ID:`/`VER:` handling;
  **deletion** of the dead `TLM`/`EVT`/`OK|ERR|CFG|ID` text branches and
  `_CORR_ID_RE` (§4).
- `io/{cli,sim_loop,sim_config}.py`, `testgui/transport.py`,
  `robot/protocol.py` — every binary-frame producer/consumer.
- `src/sim/sim_ctypes.cpp` and its Python counterpart.

Part B additionally:

- `src/protos/telemetry.proto` — `sint32` + `(scale)` on every geometric/
  kinematic field, `age` replacing `time`, packed `acks`, `ack_corr`/`ack_err`
  removed, bound fixes (B6).
- `src/protos/options.proto` — new `(scale)` option.
- `src/scripts/gen_messages.py` — `sint32`/zigzag emission, `(scale)`
  handling, re-derived size constants.
- `src/firm/messages/wire.cpp` — `ScalarType::kSint32` + encode/decode
  branches; first real `kRepeatedScalar` use.
- `src/firm/devices/motor.h`, `otos.h` (+ `nezha_motor.cpp`, `otos.cpp`,
  the sim/fake implementations) — `sampleTime()` accessor. **Do this first;
  everything else in B2 depends on it.**
- `src/firm/app/robot_loop.{h,cpp}` — `RobotState` assembly replaces
  `assembleFrame()`'s ten-arg signature, the ten `setFlag()` calls, and the
  `estimatorInput` hand-copy.
- `src/firm/app/telemetry.{h,cpp}` — `Frame`/`SecondaryFrame` replaced by
  `update(const RobotState&)`; flag derivation moves in; `packLine()`/
  `packColor()` move in from `robot_loop.cpp`'s anonymous namespace.
- `src/motion/state_estimator.h` — `Input` becomes (or is replaced by)
  `RobotState`. Watch the base↔motion boundary: a shared state struct
  crossing it is a design decision, and `RobotState` must stay float-typed
  and `messages/`-free for `src/motion` to consume it.
- `src/host/robot_radio/robot/robot_state.py`, `nezha_state.py` — host-side
  unscaling.

Tests:

- `src/tests/unit/test_wire_codec.py`, `test_host_wire_codec.py`,
  `test_serial_conn_binary_plane.py`, `test_protocol_binary_client.py`
- `src/tests/sim/unit/wire_runtime_harness.cpp`, `app_comms_harness.cpp`,
  `app_telemetry_harness.cpp`, `app_robot_loop_harness.cpp`
- `src/tests/sim/support/wire_test_codec.{h,cpp}`

Docs:

- `docs/design/design.md`, `src/firm/app/DESIGN.md`,
  `src/firm/messages/DESIGN.md`, and https://robots.jointheleague.org/
  (the protocol page is the published contract).

## Acceptance criteria

1. `crcCompute("123456789") == 0x29B1` still passes — the CRC variant is not
   changing.
2. New known-answer vectors for `0x0A`-delimited COBS, shared byte-for-byte
   between the C++ and Python suites: all-`0x0A` payload, all-`0x00` payload,
   `0x00..0xFF` sweep, empty payload.
3. Property test both sides: for random payloads up to 251 bytes, the encoded
   frame contains no `0x0A`, and decode round-trips.
4. Differential test: the same fixture vectors produce byte-identical frames
   from firmware and host codecs.
5. **CRC scope**: a vector proving the CRC covers the command — same payload
   under two different command names must produce two different CRCs, and a
   frame whose command byte is mutated in transit must fail verification
   rather than dispatch.
6. Grammar tests: data containing `:`; data containing `0x00`; a no-data verb
   with a stray trailing `:`; an unknown command (must count malformed, not
   crash); a truncated line.
7. Host reads the wire via plain `readline()` in at least one test path — the
   demonstration that the demuxer is no longer load-bearing.
8. **Bench gate**: the 123-006 hardware repro (a `move_wheels` envelope
   embedding a literal `0x0A`) executes 10/10, not 0/10.
9. **Bench gate**: `DEVICE:` announcement observed at boot on a fresh connect
   with no `HELLO` sent, and `connect()` completes without entering the
   `_HELLO_CLASSIFY_TIMEOUT_S` fallback.
10. `HELLO`→`DEVICE:`, `PING`→`PONG:`, `ID`→`ID:`, `VER`→`VER:` all answer on
    both transports, and every emitted line parses under the §1 grammar.
11. The dead host text branches (§4) are gone, and no test depends on them.

Part B:

12. **`sint32` round-trip**: negative values at each declared bound encode to
    the widths in B3 — specifically a negative velocity must cost 3 B, not 10.
    An explicit regression test against the `int32` sign-extension trap.
13. **Scale round-trip**: firmware-encoded → host-decoded reproduces the
    original float within the declared quantum (1 mm, 0.1 mm/s, 1 mrad,
    0.01 rad/s), across the full range including both bounds and zero.
14. **Timestamps are independent**: a sim run shows `enc_left.age` and
    `enc_right.age` differing by roughly the kClear+kSettle separation, and
    neither equal to zero. A test asserting they are *equal* is the bug, not
    the spec.
15. **One assembly point**: `cycle()` contains exactly one `tlm_.update(state)`
    call and zero `setFlag()` calls; every flag is derived inside
    `Telemetry::update()`. Enforceable by grep in review.
16. **Size gate**: regenerated `kReplyEnvelopeMaxEncodedSize` ≤ 130 B. If
    `gen_messages.py` reports more, the packing did not land as designed —
    investigate rather than raising the number.
17. Packed `acks` round-trips at ring depths 0-4, with `corr_id` at 65535 and
    `err` at 8 (the real `ErrCode` ceiling).
18. `flags` with bit 16 set, and an ack carrying `ERR_NOT_CONFIGURED`, both
    survive a firmware-side decode without `ERR_RANGE` (B6).
19. **Bench gate**: measured telemetry rate improves against
    `tlm-rate-15-19hz-vs-50hz-nominal-serial.md`'s recorded baseline.

## Open questions

1. **Atomic cutover?** 123 was atomic — no version byte, both sides change
   together, and the same argument applies (a mixed-version pair produces
   garbage either way, so a negotiation byte buys nothing). Confirm, and
   confirm the relay firmware needs no corresponding change.
2. **Command name registry** — where does the closed set live so firmware,
   host, and the published protocol doc cannot drift? A generated table from
   one source is the obvious answer given `gen_messages.py` already exists.
   This registry is also where "which verbs take binary data" belongs, since
   that is now the sole discriminator.
3. **`VER` content** — what does it report? Build version is in
   `codal.json`/the version bump commits; there is no obvious runtime
   accessor today.
4. **`ID` field set** — the host's dead `ID` branch expected
   `ID model=... #<corr>`. Corr-ids are a binary-plane concept now, so the
   trailing `#<corr>` should not come back. What does `ID:` actually carry
   that `DEVICE:` does not?
5. **Sequencing** — this lands on the firmware base, which sprints 124/125
   are steering toward freeze. Does it go before 124's duty-boundary
   migration, or after? The wire is base-frozen surface, so "before the
   freeze" is not optional, only "when". Note the in-flight banner work
   touches the same files.
6. **Where does `RobotState` live?** It is consumed by `src/motion`
   (`StateEstimator`) and by `src/firm/app` (`Telemetry`). The base↔motion
   boundary is supposed to be exactly one interface, `Motion::WheelSink`
   (CLAUDE.md, `src/motion/DESIGN.md`). A shared state struct is a **second**
   crossing and needs a stakeholder decision on which side owns it — not a
   free refactor.
7. **Does the secondary frame survive?** It currently emits nothing but `now`
   (`setSecondaryFrame()` has no firmware caller — see
   `telemetry-frame-inventory.md` finding 1). Under a state-derived
   projection it is either populated from `RobotState` for free, or it should
   be deleted and its cadence slot returned to the primary. Deciding this is
   cheaper now than after the schema is reworked around it.
8. **`(scale)` semantics** — does the generated code apply the scale (schema
   carries floats, wire carries ints, conversion is generated), or does the
   schema simply declare an integer field and `(scale)` stay documentation
   the projection function honours by hand? The first is safer and is what
   B3 assumes; the second is less generator work. Stakeholder call.
9. **`position` range vs. accumulation** — `EncoderReading.position` is
   documented "[mm] accumulated". ±32 m covers the playfield, but a long
   bench session driving back and forth accumulates monotonically unless the
   value is a true odometric position. Confirm which it is before pinning
   `(abs_max) = 32000`; if it accumulates, it needs a rebaseline policy, not
   a wider field.
