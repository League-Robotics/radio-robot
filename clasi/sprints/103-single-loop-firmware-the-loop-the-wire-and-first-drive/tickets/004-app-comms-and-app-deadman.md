---
id: '004'
title: app/Comms and app/Deadman
status: open
use-cases: [SUC-004]
depends-on: ['001']
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

- [ ] `Comms::pump(Cmd& out)` drains available serial/radio RX and decodes
      at most one `"*B"`-armored frame into `out` per call; never sleeps,
      never blocks.
- [ ] Malformed armor (`line[1] != 'B'`) and malformed base64/protobuf
      decode failures are rejected cleanly — no crash, no partial/garbage
      state left in `out`.
- [ ] Buffer sizing constants (`kMaxEnvelopeBytes`, `kArmoredBufSize`)
      transcribed per the note's own values/rationale, sized from
      `msg::wire::kCommandEnvelopeMaxEncodedSize`/
      `kReplyEnvelopeMaxEncodedSize` (ticket 001's regenerated constants,
      not the note's now-stale pre-prune numbers).
- [ ] Base64 alphabet is standard RFC 4648 (`+/`) — matches the host's
      `base64.b64encode`/`b64decode` defaults; no `-_` url-safe variant
      anywhere.
- [ ] `Comms::sendReply(const msg::ReplyEnvelope&, ...)` (or equivalent)
      encode+armor+send path exists and is what `Telemetry` (ticket 005)
      calls to emit frames.
- [ ] `Deadman::arm(duration)`, `Deadman::disarm()`, `Deadman::expired()`
      exist; `expired()` is checked at most once per cycle by the loop
      (ticket 008), and `Deadman` itself never calls `Drive::stop()` or
      touches any other module — the loop does that.
- [ ] `grep -rn` across `source/app/` confirms `Deadman` is the only
      staleness/timeout mechanism gating actuation (no second inline
      timer anywhere else in this sprint's new code).
- [ ] Host-buildable (`HOST_BUILD`) unit coverage for `Comms`'s
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
