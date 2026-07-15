---
id: '004'
title: app/Comms and app/Deadman
status: done
use-cases:
- SUC-004
depends-on:
- '001'
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# app/Comms and app/Deadman

## Description

Build `source/app/comms.{h,cpp}` and `source/app/deadman.{h,cpp}`, the
first two modules of the new loop. `Comms` reproduces the `"*B"` armor/
dearmor framing sequence transcribed BEFORE `binary_channel.cpp` was
deleted (sprint 102's transcription note,
`clasi/sprints/done/102-.../notes/armor-wire-codec-transcription.md`) —
NOT the old per-oneof dispatch switch, which is genuine Elite-stack
orchestration this sprint's loop (ticket 008) replaces with its own
dispatch. `Deadman` is the ONE staleness rule that gates every actuation
source in the new design — no second ad hoc watchdog timer anywhere else
in `source/app/`.

Depends on ticket 001 (compiles against the pruned `msg::CommandEnvelope`/
`ReplyEnvelope` types).

## Acceptance Criteria

- [x] `Comms::pump(Cmd& out)` drains available serial/radio RX and decodes
      at most one `"*B"`-armored frame into `out` per call; never sleeps,
      never blocks.
- [x] Malformed armor (`line[1] != 'B'`) and malformed base64/protobuf
      decode failures are rejected cleanly — no crash, no partial/garbage
      state left in `out`.
- [x] Buffer sizing constants (`kMaxEnvelopeBytes`, `kArmoredBufSize`)
      transcribed per the note's own values/rationale, sized from
      `msg::wire::kCommandEnvelopeMaxEncodedSize`/
      `kReplyEnvelopeMaxEncodedSize` (ticket 001's regenerated constants,
      not the note's now-stale pre-prune numbers).
- [x] Base64 alphabet is standard RFC 4648 (`+/`) — matches the host's
      `base64.b64encode`/`b64decode` defaults; no `-_` url-safe variant
      anywhere.
- [x] `Comms::sendReply(const msg::ReplyEnvelope&, ...)` (or equivalent)
      encode+armor+send path exists and is what `Telemetry` (ticket 005)
      calls to emit frames.
- [x] `Deadman::arm(duration)`, `Deadman::disarm()`, `Deadman::expired()`
      exist; `expired()` is checked at most once per cycle by the loop
      (ticket 008), and `Deadman` itself never calls `Drive::stop()` or
      touches any other module — the loop does that.
- [x] `grep -rn` across `source/app/` confirms `Deadman` is the only
      staleness/timeout mechanism gating actuation (no second inline
      timer anywhere else in this sprint's new code).
- [x] Host-buildable (`HOST_BUILD`) unit coverage for `Comms`'s
      encode/decode round-trip, using the existing `WireRuntime`/
      `msg::wire` `HOST_BUILD` seam.

## Implementation Plan

**Approach**: `Comms` is a straight port of the transcription note's two
code blocks (`sendReply`/dearmor-and-decode), adapted from the deleted
`BinaryChannel`'s free-function style into a small class with a
`pump()`/`sendReply()` public surface, reading `com/serial_port.h`'s
actual `readLine()`/`sendReliable()` signatures and `com/radio.h`'s
`poll()`/`send()` signatures (both unchanged, confirmed during this
sprint's own planning) rather than assuming the note's prose. `Deadman` is
new, small (a handful of functions around one timer) — no transcription
source, write it directly against Step 3's boundary (arm/disarm/expired
only, no side effects of its own).

**Files to create/modify**:
- `source/app/comms.h`, `source/app/comms.cpp` (new)
- `source/app/deadman.h`, `source/app/deadman.cpp` (new)

**Testing plan**:
- Existing tests to run: none directly (new files); confirm
  `source/messages/wire.{h,cpp}`/`wire_runtime.{h,cpp}`'s own tests
  (ticket 001's rewritten wire suite) still pass, since `Comms` depends on
  them unchanged.
- New tests to write: a `HOST_BUILD` round-trip test —
  encode a `Twist` command with `Comms::sendReply`-equivalent tooling (or
  a small test-only encode helper), feed the resulting armored line back
  through `Comms::pump()`, confirm the decoded `Cmd` matches; a malformed-
  line rejection test (bad prefix, truncated base64, corrupt protobuf
  bytes) confirming no crash and `out` unchanged. A `Deadman` timer test:
  `arm(100)` then check `expired()` false before 100ms (scripted clock)
  and true after; `disarm()` then confirm `expired()` stays false
  regardless of elapsed time.
- Verification command: `uv run python -m pytest tests/sim/unit/ -k "comms or deadman"`
  (once the corresponding test files exist) plus a host-side C++ compile/
  run of the new harness.

**Documentation updates**: none beyond inline header comments documenting
the armor/dearmor sequence's provenance (cite the transcription note, per
this project's own "cite the source, don't silently re-derive" pattern
used elsewhere in this sprint's planning).

## Completion Notes

**Files created**: `source/app/comms.h`, `source/app/comms.cpp`,
`source/app/deadman.h`, `source/app/deadman.cpp`,
`tests/sim/unit/app_comms_harness.cpp`, `tests/sim/unit/test_app_comms.py`,
`tests/sim/unit/app_deadman_harness.cpp`,
`tests/sim/unit/test_app_deadman.py`.

**Transport seam design** (left open by the ticket, resolved during
planning): `App::Transport` is a plain virtual base class (not an `#ifdef
HOST_BUILD` fork), matching `Devices::MotorArmor`'s precedent —
`readLine()`/`send()`/`sendReliable()`. Two concrete ARM adapters
(`SerialTransport`, `RadioTransport`) are declared in `comms.h` and defined
in `comms.cpp`, both guarded `#ifndef HOST_BUILD`; `comms.h` only
forward-declares `class SerialPort;`/`class Radio;` (also guarded), never
`#include`s their headers, so `comms.h`/`comms.cpp` under `HOST_BUILD`
never see `MicroBit.h`. `RadioTransport::send()` and `sendReliable()` both
delegate to the same `Radio::send()` — `Radio` has only one send path. The
`HOST_BUILD` test harness defines its own `FakeTransport` locally (not in
`comms.h`) — a scripted line queue plus `send()`/`sendReliable()` call
logs.

**Buffer sizing arithmetic**: `kMaxEnvelopeBytes = max(kCommandEnvelopeMaxEncodedSize=115,
kReplyEnvelopeMaxEncodedSize=179) = 179`, computed by the constexpr
ternary itself (not hardcoded) so a future schema regeneration updates it
automatically. `kArmoredBufSize = 256`: `"*B"` (2) + `base64(179)` =
`ceil(179/3)*4 = 240` + NUL (1) = 243, rounded up to 256 with headroom —
matches `SerialPort`'s own 256-byte `_rxBuf` and stays under the ticket's
`<=~250B` outbound-line guidance (243 < 250).

**Deliberate deviation from the transcription note**: the transcribed
dearmor path calls `sendError(ErrCode::ERR_DECODE, ...)` on every
malformed-armor/base64/protobuf rejection. `Comms::decodeArmoredLine()`
does NOT reproduce those calls — it only increments `malformedCount_` and
returns, per this ticket's design decision that "ACKs ride the ack ring,
not per-command" (ticket 005's `Telemetry` surfaces `malformedCount()` as
a fault bit instead of a synchronous per-line reply). This overrides the
transcription note's own behavior and is documented inline in
`comms.cpp`'s `decodeArmoredLine()`.

**`sendReply()` broadcasts both transports**: `Comms::sendReply()` encodes
+ armors once, then sends the SAME line on both `serialLink_` and
`radioLink_` via the async/drop-on-full `Transport::send()` (never
`sendReliable()`) — telemetry is always-on and must never stall the loop
on serial backpressure, and SUC-005 requires primary/secondary frames on
both transports every cadence, not just back to whoever last spoke.

**`Deadman` negative-duration clamp**: `arm(duration)` clamps any
`duration` for which `duration > 0.0f` is false (covers both negative
values and NaN, since NaN comparisons are always false) to `0` —
immediate expiry — rather than inventing a general min/max bound the
ticket doesn't specify. The `[ms]->[us]` conversion multiplies while
still float-typed (`clamped * 1000.0f`) before the final `uint64_t` cast,
so sub-millisecond fractions of `duration` are not truncated before the
unit conversion. A never-armed `Deadman` reads `expired() == false` (no
window has ever been opened); `disarm()` cancels unconditionally,
regardless of subsequent clock advance.

**Verification evidence**:
- `uv run python -m pytest tests/sim/unit/ -v` — 335 passed (all
  pre-existing `tests/sim/unit/` suites plus the two new ones:
  `test_app_comms.py::test_app_comms_harness_compiles_and_passes`,
  `test_app_deadman.py::test_app_deadman_harness_compiles_and_passes`).
- `just build` — succeeds; `source/app/comms.cpp` and
  `source/app/deadman.cpp` compile clean for the real ARM target (confirmed
  via `CMakeFiles/MICROBIT.dir/source/app/{comms,deadman}.cpp.obj` present
  after the build) and link into `MICROBIT.hex` (v0.20260714.9 at build
  time).
- `grep -rn "expired\|Deadline\|deadline\|watchdog\|timeout" source/app/`:
  ```
  source/app/deadman.cpp:19:  deadlineMicros_ = clock_.nowMicros() + deltaMicros;
  source/app/deadman.cpp:25:bool Deadman::expired() const {
  source/app/deadman.cpp:27:  return clock_.nowMicros() >= deadlineMicros_;
  source/app/deadman.h:3:// 3 "Deadman" boundary: inside -- arm(duration)/disarm()/expired() and the
  source/app/deadman.h:6:// other module)). No second ad hoc watchdog timer belongs anywhere else in
  source/app/deadman.h:24:  // clamping is invented here). Every call sets a FRESH deadline from now,
  source/app/deadman.h:35:  bool expired() const;
  source/app/deadman.h:40:  uint64_t deadlineMicros_ = 0;  // [us]
  ```
  All matches are confined to `deadman.{h,cpp}` — `Deadman` is the only
  staleness/timeout construct in `source/app/` (only `comms.{h,cpp}` and
  `deadman.{h,cpp}` exist there at this point in the sprint).
