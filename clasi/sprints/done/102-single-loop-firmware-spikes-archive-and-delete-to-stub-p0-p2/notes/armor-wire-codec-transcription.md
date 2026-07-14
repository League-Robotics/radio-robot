# Armor codec transcription — for sprint 103's `Comms`

Transcribed per ticket 005's acceptance criterion 1, BEFORE
`source/commands/binary_channel.cpp` was deleted (P2, this sprint). This is
NOT new code and nothing here needs to be recovered from git history — the
underlying primitives it calls (`msg::wire::encode`/`decode`,
`WireRuntime::base64Encode`/`base64Decode`) live in `source/messages/wire.{h,cpp}`
and `source/messages/wire_runtime.{h,cpp}`, which are **KEPT** by this sprint
(messages/ is not deleted — protos are not pruned this sprint). What
disappears with `binary_channel.cpp` is only the **orchestration**: the
buffer sizing and the armor/dearmor call sequence that wraps those kept
primitives. Sprint 103's `Comms` needs to reproduce this sequence, not the
codec itself.

Source: `source/commands/binary_channel.cpp` (commit history, this branch,
just before deletion) — `namespace BinaryChannel`, lines ~46-127 (armor
sizing + encode/armor) and ~771-806 (dearmor + decode). Reproduced verbatim
below, trimmed of unrelated per-arm dispatch (that part — the oneof switch
per `CommandEnvelope.cmd_kind` — is genuine Elite-stack orchestration tied
to `Rt::Blackboard`/`Rt::CommandRouter` and is NOT part of what sprint 103
needs; only the framing wrapper is).

## Buffer sizing

```cpp
// kMaxEnvelopeBytes -- the larger of the two generated per-direction
// budgets (both 168B as of this ticket; wire.h's own static_asserts keep
// either one from silently exceeding the 186B envelope cap on a future
// schema change) -- one raw-byte scratch buffer, reused sequentially for
// the incoming decode and the outgoing encode within a single handle()
// call (never overlapping).
constexpr size_t kMaxEnvelopeBytes =
    (msg::wire::kCommandEnvelopeMaxEncodedSize > msg::wire::kReplyEnvelopeMaxEncodedSize)
        ? msg::wire::kCommandEnvelopeMaxEncodedSize
        : msg::wire::kReplyEnvelopeMaxEncodedSize;

// kArmoredBufSize -- "*B" (2) + base64(kMaxEnvelopeBytes) (ceil(168/3)*4 =
// 224) + NUL (1) = 227, rounded up with headroom; matches whatever
// transport line-buffer budget the new Comms module uses (was 256B, sized
// to Subsystems::CommunicatorToCommandProcessorCommand::line in the deleted
// stack).
constexpr size_t kArmoredBufSize = 256;
```

## Encode + armor (outbound: ReplyEnvelope -> `*B<base64>` line)

```cpp
// sendReply -- encode+armor+send one ReplyEnvelope. Every exit path should
// funnel through one function like this, so "always reply exactly once,
// always armored" is enforced structurally rather than repeated at every
// call site.
void sendReply(const msg::ReplyEnvelope& reply, ReplyFn replyFn, void* replyCtx) {
  uint8_t rawBuf[kMaxEnvelopeBytes];
  const uint16_t n = msg::wire::encode(reply, rawBuf, static_cast<uint16_t>(sizeof(rawBuf)));
  if (n == 0) {
    // Unreachable in practice: kMaxEnvelopeBytes is sized from the SAME
    // generated kCommandEnvelopeMaxEncodedSize/kReplyEnvelopeMaxEncodedSize
    // constants encode() itself is budgeted against (wire.h's own
    // static_asserts), so every ReplyEnvelope this file ever builds fits.
    // No reply is sent rather than emitting a malformed/truncated line.
    return;
  }

  char armored[kArmoredBufSize];
  armored[0] = '*';
  armored[1] = 'B';
  size_t b64Len = 0;
  if (!WireRuntime::base64Encode(rawBuf, n, armored + 2, sizeof(armored) - 3, &b64Len)) {
    return;  // same unreachable-in-practice sizing argument as above
  }
  armored[2 + b64Len] = '\0';
  replyFn(armored, replyCtx);
}
```

## Dearmor + decode (inbound: `*B<base64>` line -> CommandEnvelope)

```cpp
// Callers only ever reach here with line[0] == '*' (the caller's own
// dispatch branch on the leading byte) -- line[1] != 'B' is still a real
// possibility (a malformed/future-armor line) and must be rejected
// cleanly, not assumed away.
if (line[0] != '*' || line[1] != 'B') {
  sendError(msg::ErrCode::ERR_DECODE, 0, 0, replyFn, replyCtx);
  return;
}

const char* b64 = line + 2;
size_t b64Len = std::strlen(b64);
while (b64Len > 0 && (b64[b64Len - 1] == '\r' || b64[b64Len - 1] == '\n' ||
                      b64[b64Len - 1] == ' ' || b64[b64Len - 1] == '\t')) {
  --b64Len;
}

uint8_t rawBuf[kMaxEnvelopeBytes];
size_t rawLen = 0;
if (!WireRuntime::base64Decode(b64, b64Len, rawBuf, sizeof(rawBuf), &rawLen)) {
  sendError(msg::ErrCode::ERR_DECODE, 0, 0, replyFn, replyCtx);
  return;
}

// --- Decode: walk the generated field table, validating bounds inline. ---
msg::CommandEnvelope env;
const msg::wire::Result r = msg::wire::decode(env, rawBuf, static_cast<uint16_t>(rawLen));
if (!r.ok) {
  // env.corr_id may or may not have been populated before the failing
  // field, depending on wire order -- best effort (0 if never reached),
  // matching every real protobuf encoder's field-ascending emission
  // order in practice (corr_id is field 1).
  sendError(r.code, r.field, env.corr_id, replyFn, replyCtx);
  return;
}
```

## What sprint 103 needs to rebuild vs. what it can call directly

- **Call directly, unchanged** (KEPT this sprint): `msg::wire::encode()`,
  `msg::wire::decode()` (`source/messages/wire.{h,cpp}`),
  `WireRuntime::base64Encode()`/`base64Decode()`
  (`source/messages/wire_runtime.{h,cpp}`). These are schema-driven/
  hand-written byte-level codecs with no dependency on the deleted
  `Rt::Blackboard`/`Rt::CommandRouter`/`Subsystems::*` types — sprint 103's
  `Comms` can `#include` them exactly as-is.
- **Reproduce the shape of, not copy verbatim** (deleted with
  `binary_channel.cpp`): the buffer sizing constants, the armor prefix
  (`"*B"` + base64 + NUL) and dearmor (strip `"*B"`, trim trailing
  whitespace, base64-decode) sequences above, and the "always exactly one
  reply, always armored" discipline `sendReply()` enforced structurally.
  The per-oneof-arm dispatch switch (`handle()`'s `switch (env.cmd_kind)`)
  is NOT reproduced here — it is genuine Elite-stack orchestration
  (`Rt::Blackboard` posts, `Drive::Drivetrain` admission, etc.) that
  sprint 103's new loop replaces with its own dispatch, not a framing
  concern.
- **Base64 alphabet pin**: standard RFC 4648 (`+/`), NOT url-safe (`-_`) —
  see `wire_runtime.h`'s own file header. The host side
  (`host/robot_radio/io/serial_conn.py`) uses Python stdlib
  `base64.b64encode`/`b64decode`'s default alphabet, which matches. Do not
  change this on one side without the other — there is no negotiation, no
  version byte.
