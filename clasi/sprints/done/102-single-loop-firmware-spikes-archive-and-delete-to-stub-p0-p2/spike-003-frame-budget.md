---
ticket: '003'
status: done
---

# Spike 003 results: wire-frame budget dry run

## What was measured

A dry-run draft of the P4 pruned wire protocol (issue's "Wire protocol"
section) was built on scratch branch **`scratch/102-003-frame-budget`**
(commit **`10985ec1d46737090c00f6b9f7b33f1fa2de9ed0`**, branched from this
sprint's HEAD at the time, `7aaea0a5`). It is **not merged** into this
sprint's branch or into `master`'s `protos/` — verified: `git diff --stat
protos/ source/messages/` against this sprint's own branch is empty after
returning from the scratch branch.

The draft:

- **`protos/envelope.proto`**: `CommandEnvelope` pruned to exactly
  `corr_id` + `oneof cmd { Twist twist; ConfigDelta config; Stop stop; }`
  (`Twist{v_x, omega, duration}` is new; `ConfigDelta`/`Stop` unchanged
  from the pre-102 schema). Every other pre-102 arm (drive, segment,
  replace, pose_fix, otos, ping, echo, get, stream, id, hello, ver, help,
  plan_dump) is deleted, not commented out. `ReplyEnvelope` narrowed to
  `ok`/`err`/`tlm` (id/echo/helptext/cfg/evt/plan/trace all die with their
  triggering command arms).
- **`protos/telemetry.proto`**: `Telemetry` gains an ack ring
  (`repeated AckEntry acks`, `AckEntry{corr_id, status:AckStatus,
  err_code:uint32}`) and two fault/event bitmask fields (`fault_bits`,
  `event_bits`, bit layout left as a P4 decision). `acc_left/acc_right`,
  `glitch_left/glitch_right`, `ts_left/ts_right`, and
  `has_cmd_vel/cmd_vel_left/cmd_vel_right` move out to a new
  `TelemetrySecondary` message, per the issue's instruction. `active`,
  `conn_left`, `conn_right` were kept in the primary frame (not in the
  issue's explicit trim list, and load-bearing for
  `hardware-bench-testing.md`'s "sensors alive" gate at full rate).

`scripts/gen_messages.py` ran clean (no warnings) against the draft and
regenerated `source/messages/{envelope,telemetry,layout_checks,wire}.{h,cpp}`
on the scratch branch only.

## Verdict: the static_asserts (real compile, not just eyeballing the numbers)

Generated `source/messages/wire.h`:

```cpp
//   CommandEnvelope: twist=17B, config=109B, stop=2B (worst=config=109B) + non-oneof=6B => total=115B
//   ReplyEnvelope: ok=19B, err=10B, tlm=173B (worst=tlm=173B) + non-oneof=6B => total=179B
constexpr uint16_t kCommandEnvelopeMaxEncodedSize = 115;
constexpr uint16_t kReplyEnvelopeMaxEncodedSize = 179;
static_assert(kCommandEnvelopeMaxEncodedSize <= 186, ...);  // PASS
static_assert(kReplyEnvelopeMaxEncodedSize <= 186, ...);    // PASS
```

Verified by an actual compile, not just reading the header: `c++ -std=c++20
-Wall -Wextra -I source -c source/messages/wire.h/.cpp/wire_runtime.cpp/
layout_checks.cpp` all compiled clean (Apple clang 21.0.0) — the
static_asserts were evaluated by the real compiler and passed. A hand-built
round-trip program (`msg::wire::decode()`/`encode()` against the real
generated tables — `CommandEnvelope{twist}`, `CommandEnvelope{stop}`,
`ReplyEnvelope{tlm}` with a populated 3-entry ack ring) also compiled and
ran, all checks passing, confirming the new ack-ring repeated-message field
walks correctly through the generated encoder, not just that it fits the
static size budget on paper.

## Numbers

| Frame | Worst case | Ceiling | Margin |
|---|---|---|---|
| **CommandEnvelope** (twist/config/stop) | **115 B** | 186 B | 71 B (was 168 B pre-102 — freed 53 B by dropping 13 arms) |
| **ReplyEnvelope{tlm}** ("main frame", ack ring depth=3) | **179 B** | 186 B | **7 B** |
| **TelemetrySecondary** (slower diagnostic frame, standalone) | **52 B** (54 B if wrapped as one more envelope arm) | 186 B | 132+ B |

### Ack-ring depth vs. budget (measured, not estimated — one `gen_messages.py` run per row)

| Ring depth | `tlm` arm | `ReplyEnvelope` total | Fits 186 B? |
|---|---|---|---|
| 2 | 157 B | 163 B | yes (23 B margin) |
| **3** | **173 B** | **179 B** | **yes (7 B margin) — chosen** |
| 4 | 189 B | 195 B | **no — 9 B over** |
| 5 | 205 B | 211 B | no |
| 6 | 221 B | 227 B | no |
| 8 | 253 B | 259 B | no |

Each ring entry costs exactly 16 B (`AckEntry` payload 14 B: `corr_id`
6 B + `status` 2 B + `err_code` 6 B, wrapped in a 1-byte tag + 1-byte
length since a repeated *message* field is never packed — every entry pays
its own tag+len, unlike a packed scalar array).

## Verdict vs. the issue's stated target

The issue's "Wire protocol" section asks for ring depth **4-8**. The
measured result is that **depth=3 is the maximum that fits** the current
primary-frame field set (`now`/`mode`/`seq`/`enc`/`vel`/`pose`/`otos`+
`otos_connected`/`twist`/`active`/`conn_left`/`conn_right`/`fault_bits`/
`event_bits`) under the 186 B ceiling — **this is a genuine "does not fit
at the target depth" result**, per the ticket's own accepted outcome shape
("a 'does not fit, here's the max that does' verdict is a SUCCESS").

**The tradeoff, measured**: depth=4 needs ~9 B trimmed from the primary
frame. One concrete trim was measured directly (not estimated): dropping
`active`/`conn_left`/`conn_right` (9 B total — 3 bools at the unavoidable
2-byte tag width, since this schema has 22 primary-frame fields against
only 15 cheap 1-byte-tag field numbers) from the primary frame and moving
them to `TelemetrySecondary` lands ring depth=4 at **exactly 186 B, zero
margin** (`tlm=180B`, total=186B — confirmed by a real `gen_messages.py`
run, not arithmetic). Zero margin is fragile — any future field addition
to the primary frame would immediately blow the budget again — so this
spike does **not** recommend shipping that combination as-is.

## Recommendation for sprint 103 (P4)

1. **Ship ring depth=3** with the field set recorded above (7 B margin) —
   the safe, verified choice. This is below the issue's stated 4-8 target;
   flag that gap explicitly to the stakeholder/architecture-update rather
   than silently splitting the difference.
2. If depth=4+ is required, the primary frame needs a real trim, not just
   the bare-minimum 9 B: candidates already visible from this spike are
   `active`/`conn_left`/`conn_right` (9 B, cheap but tight) and/or trimming
   `otos`/`otos_connected` out of the always-on cadence (raw OTOS is only
   refreshed ~1/3 of cycles by the perception round-robin per the issue's
   own main-loop sketch, so sending it every telemetry frame is arguably
   already wasteful) — that alone frees ~21 B, enough for depth=5 with
   real margin. This spike does not decide between these; it hands sprint
   103 the exact byte costs to decide with.
3. `TelemetrySecondary`'s own framing (how it rides the wire — a second
   `*B`-armored line, a new `ReplyEnvelope` oneof arm, or something else)
   is an open P4 implementation decision; this spike measured it wrapped
   as a hypothetical extra `ReplyEnvelope` arm (54 B) purely to get a
   real generator-computed number, not as a design recommendation.
4. Whether `ReplyEnvelope` survives at all as a distinct top-level type
   (vs. `Telemetry` becoming its own unwrapped top-level push frame, since
   the design is "telemetry-only return path" with no more per-command
   reply) is also an open P4 decision, out of this spike's scope.

## Known gap for sprint 103: the existing wire-codec test harness does not compile against the pruned schema

`tests/sim/unit/wire_codec_harness.cpp` (exercised by
`tests/sim/unit/test_wire_codec.py`) is written against the pre-102 arm set
(`drive`/`segment`/etc.) and fails to compile against this draft schema
(20+ errors, e.g. `no member named 'DRIVE' in msg::CommandEnvelope::CmdKind`)
— expected and out of this spike's scope (the ticket's own testing plan:
"the wire.h static_asserts ARE the test... no new pytest needed"). Instead,
a throwaway round-trip program was hand-built directly against
`msg::wire::decode()`/`encode()` (not committed anywhere — scratch-local
only) to prove the pruned+extended codec actually round-trips real bytes,
not just that the header's static_asserts evaluate true:
- `CommandEnvelope{twist}` and `CommandEnvelope{stop}` hand-encoded via
  `WireRuntime` primitives, decoded via `msg::wire::decode()`, every field
  verified to round-trip.
- `ReplyEnvelope{tlm}` with a populated 3-entry ack ring encoded via
  `msg::wire::encode()`: succeeded, returned 85 B (well under the 179 B
  worst case, since not every optional field was populated), confirming
  the generated encoder correctly walks the new repeated-message ack-ring
  field.

**Sprint 103 will need to rewrite `wire_codec_harness.cpp` (and
`test_wire_differential.py`/`test_wire_fuzz.py`, which share its shape)
against the real P4 schema** — this is inherent P4 scope (the harness
exercises the arms that exist), not a defect introduced by this spike.
Flagging it here so it is not a surprise mid-sprint-103.

## Scratch branch for sprint 103 to pick up

- Branch: `scratch/102-003-frame-budget`
- Commit: `10985ec1d46737090c00f6b9f7b33f1fa2de9ed0`
- Contents: `protos/envelope.proto`, `protos/telemetry.proto` (both
  rewritten, not incrementally diffed against the pre-102 originals — see
  the commit message), and the matching regenerated
  `source/messages/{envelope,telemetry,layout_checks,wire}.{h,cpp}`.
- **Not merged anywhere.** Sprint 103 should treat it as a reference/
  starting point to re-derive its own real P4 schema from, verifying its
  own field list against the architecture-update.md that sprint writes
  (this draft made several judgment calls — e.g. keeping
  `active`/`conn_left`/`conn_right` in the primary frame, keeping
  `ReplyEnvelope` as a wrapper type, the exact `fault_bits`/`event_bits`
  bit layout being undefined — that are P4's to confirm or revise, not
  inherited silently).

## No hardware used

This spike was schema/codegen/host-compile only — no hardware involved, no
`mbdeploy`/bench step, per the ticket's own acceptance criterion.
