---
status: pending
---

# Message-Processor Dispatch for the Binary Command Plane

## Context

`BinaryChannel::handle()` (source/commands/binary_channel.cpp) grew into a ~400-line
central switch during sprint 096 — every binary arm decoded and translated in one
file. Eric's governing principle: **adding a message type = add an enum value + a new
message processor; existing dispatch code never changes** (open/closed). Today adding
one command touches envelope.proto + regen + the binary_channel switch + host
protocol.py. This plan removes the hand-edited firmware dispatch site entirely:
framing at the top, a message-number → MessageProcessor map in the middle, and
processors owned/declared by their subsystems, posting onto Blackboard queues.

## Stakeholder decisions (2026-07-10)

1. **Keep the envelope on the wire.** Wire bytes identical — host pb2 clients
   (byte-driven, verified serial_conn.py:657-708) and the differential/fuzz suites
   are untouched. The protobuf oneof tag IS the message number; firmware *peeks*
   `{corr_id, arm field number, payload slice}` and routes the slice without a
   central envelope decode.
2. **Own sprint, after 097 closes** (097 is in-flight in a parallel session and
   explicitly scopes dispatch out). The POSE/OTOS binary arms already earmarked for
   098 become this sprint's capstone — the first proof of open/closed.
3. **Thin `Subsystems::Telemetry` subsystem** owning both the STREAM processor
   (unpack side) and the periodic binary emit (pack side); the loop calls
   `telemetry.tick()`. (Design agent proposed a free-function bundle; overridden by
   stakeholder decision.)

## Target architecture

```
"*B<base64>" line
  └─ Framing::dearmor/armor            [NEW source/messages/framing.{h,cpp}; COBS-swappable]
       └─ raw envelope bytes
            └─ msg::wire::peek()        [NEW, generated] → {corrId, armField, payload slice}
                 └─ Rt::MessageRouter::dispatch — table_[armField] → MessageProcessor*
                      └─ processor->handle(payload, len, bb, reply)
                           ├─ msg::wire::decodeMessage(<ArmType>&, slice)   [NEW, generated]
                           ├─ bb.<queue>.post(...)      // post-and-reply ONLY
                           └─ reply.ack(bb)/error()/send(env) → Rt::emitEnvelope → armor
```

**Open/closed test (acceptance for the capstone):** adding an arm = proto oneof entry
+ regen + one processor class + one `add()` line in its owner — zero diffs to
`message_router.*`, `binary_channel.cpp`, or generator dispatch logic.

## Design

### Framing module — `source/messages/framing.{h,cpp}` (NEW)
`armor()`/`dearmor()` for `*B<base64>` lines (whitespace-trim absorbed), one
`kArmoredBufSize = 256` replacing the two duplicated TU-local copies
(binary_channel.cpp:52, telemetry_commands.cpp:22). Codec-agnostic; a future COBS
swap touches only this module. Companion funnel `Rt::emitEnvelope(reply, replyFn,
replyCtx)` in `source/runtime/binary_reply.{h,cpp}` = encode + armor + emit —
replaces the duplicated funnels at binary_channel.cpp:83-104 and
telemetry_commands.cpp:36-66.

### Generated codec additions — `scripts/gen_messages.py` → wire.{h,cpp}
`decode()`/`encode()` untouched (differential/fuzz gates stay valid). Three additions:
- **`Result peek(PeekResult&, buf, len)`**: top-level scan only — decodes corr_id,
  records the last recognized oneof arm's `{fd->number, buf+pos, payloadLen}` (the
  slice is already isolated at wire.cpp:603-615 before recursion; peek just doesn't
  recurse), skips unknown fields like decode() does. No arm present → `armField 0`.
- **Per-arm `decodeMessage(<ArmType>&, buf, len)` overloads** (11 today, generated
  from the oneof so new arms auto-gain them): memset + `decodeInto(&out,
  kTable_<T>, buf, len, depth=1)`. Bounds/(req) validation already runs per-message
  inside `decodeInto` (wire.cpp:287,587-600,678-683) — processors inherit it, and
  error `Result{code, field}` stays byte-identical to today's nested propagation.
- **`msg::wire::cmdfield::k*` constants** — the registry key vocabulary. Keys are
  **proto field numbers, not CmdKind ordinals** (field 5 reserved: drive=2, segment=3,
  replace=4, config=6, pose=7, otos=8, ping=9, echo=10, get=11, stream=12, stop=13,
  id=14). Field numbers are what error replies already report.

### MessageProcessor — `source/runtime/message_processor.h` (NEW)
Pure-virtual interface (stakeholder's model is class-shaped; the registry holds
object pointers either way; ~4B vptr × ~10 processors is invisible next to
base64+decode):
```cpp
class MessageProcessor {
 public:
  virtual uint16_t fieldNumber() const = 0;             // msg::wire::cmdfield::k*
  virtual void handle(const uint8_t* payload, uint16_t len,
                      Blackboard& bb, BinaryReply& reply) = 0;
 protected:
  ~MessageProcessor() = default;                        // never deleted polymorphically
};
struct ProcessorList { MessageProcessor* const* items; uint8_t count; };  // no heap
```
**Ordered-tick discipline (architecture decision, reverses 087's SUC-006
"pointerless dispatch"):** `handle()` may ONLY post to Blackboard queues/cells and
reply — never mutate tick-owned subsystem state; processors hold NO owner
back-reference. Ownership = declaration + lifetime; the Blackboard stays the only
data path.

### BinaryReply — `source/runtime/binary_reply.{h,cpp}` (NEW)
Wraps `{corrId, replyFn, replyCtx, channel}`; `ack(bb)` (today's sendAck verbatim:
q = segmentIn.size()+drivetrain.queue, rem, t=0), `error(code, field)`,
`send(ReplyEnvelope&)` (stamps corr_id). `replied_` flag enforces exactly-once
(host-build assert). `channel()` replaces handleStream's routerCtx cast.

### MessageRouter — `source/runtime/message_router.{h,cpp}` (NEW), member of CommandRouter
Direct-indexed `MessageProcessor* table_[32]` (128B BSS; static_assert vs
`cmdfield::kMaxArmFieldNumber`). `add()`/`addAll()` at setup (duplicate = wiring
bug, host-build assert). `dispatch()` owns every non-processor reply, preserving
today's semantics with **no stub registrants**:
- peek fails → `ERR(code, field)`; no arm (`armField 0`) → `ERR_UNKNOWN(0)`
- declared-but-unregistered (pose=7/otos=8 until capstone) → `ERR_UNIMPLEMENTED(armField)`
- processor never replied → `ERR_UNKNOWN(armField)` backstop + host assert

`CommandRouter` gains only `MessageRouter messages_` + accessor;
`command_processor.cpp`'s `*B` branch is untouched; `BinaryChannel::handle` shrinks
to dearmor + `router->messages().dispatch(...)` (~40 lines final).

### Processor inventory (bodies move verbatim from binary_channel.cpp — commit 294feec5 staged them as per-arm helpers)

| Arm (field) | Processor | Owner / declaration |
|---|---|---|
| drive(2), segment(3), replace(4), stop(13) | Drive/Segment/Replace/StopProcessor | `Subsystems::Drivetrain` member aggregate, `source/subsystems/drivetrain_processors.{h,cpp}`; `toSegment` moves here |
| config(6), get(11) | Config/ConfigGetProcessor | `Rt::Configurator`, `source/runtime/config_processors.{h,cpp}` |
| stream(12) + periodic emit | StreamProcessor + pack side | **NEW `Subsystems::Telemetry`** (`source/subsystems/telemetry.{h,cpp}`): owns the processor AND `tick()` (absorbs `tickTelemetry`/`telemetryEmitBinary`); loop calls `telemetry.tick(bb, router, now)` |
| ping(9), echo(10), id(14) | Ping/Echo/IdProcessor | stateless bundle `source/commands/system_processors.{h,cpp}` + free `systemMessageProcessors()` (mirrors text `systemCommands()` precedent) |
| pose(7), otos(8) | capstone tickets | transcribed from retained pose/otos_commands.cpp per 097's Decision 6 |

Wiring (main.cpp ~line 141 and sim_api.cpp ~line 287, after all subsystems exist):
```cpp
router.messages().addAll(drivetrain.messageProcessors());
router.messages().addAll(configurator.messageProcessors());
router.messages().addAll(telemetry.messageProcessors());
router.messages().addAll(systemMessageProcessors());
```

### Rider: Configurator wired into hardware main() — behavioral fix
Today binary CONFIG on real hardware posts into `bb.configIn` which **nothing
drains** (main.cpp:125-139 wires no Configurator; acks never mean "applied").
Config ticket adds `PoseEstimator` + `Rt::Configurator` statics, replaces the manual
`bb.drivetrainConfig` seed with `configurator.publish(bb)` (sim precedent
sim_api.cpp:277), and drains one delta per loop pass (`pending()`/`applyOne()` —
087's CPU-budget shape). After 097 M7 deletes text config, `ConfigProcessor` is the
ONLY config path — this rider is what makes it real on hardware.

## Migration strategy

Registry lands **behind the existing switch**: after dearmor, peek; registered arm →
new path; unregistered → legacy full `decode()` + switch fallback (kept verbatim).
Each family ticket registers processors and deletes its dead switch cases; the final
ticket deletes the fallback. Every intermediate commit keeps
`tests/sim/unit/test_binary_channel.py` green **byte-identical**.

## Ticket breakdown (dependency order)

- **T1 — Framing + emitEnvelope funnel**: new modules; rewire binary_channel +
  telemetryEmitBinary; delete duplicated constants. Armor round-trip tests.
- **T2 — Generator: peek + decodeMessage + cmdfield**: gen_messages.py; regenerate;
  assert decode()/encode() byte-unchanged via git diff. Peek/equivalence tests
  (peek+decodeMessage must reproduce decode()'s arm struct/Result/corrId on the
  differential corpus).
- **T3 — Runtime core wired empty** (deps T1,T2): MessageProcessor/BinaryReply/
  MessageRouter; CommandRouter member; handle() = peek→registered?→dispatch:legacy
  fallback; empty registration seams in both roots. Router unit harness (duplicate
  add, unregistered→ERR_UNIMPLEMENTED, no-arm→ERR_UNKNOWN, exactly-once backstop).
- **T4 — Motion family → Drivetrain** (deps T3): four processors + toSegment move;
  delete switch cases. ERR_FULL overflow case pinned.
- **T5 — System + Telemetry subsystem** (deps T3, parallel T4): system bundle;
  new `Subsystems::Telemetry` absorbing tickTelemetry + STREAM; loop call sites
  updated in main.cpp/sim (MainLoop untouched — telemetry tick stays a main-loop
  line, matching current shape).
- **T6 — Config family + hardware Configurator rider** (deps T3): processors move;
  main.cpp wiring; sim keeps to-exhaustion drainConfig. New sim assert: config ack ↔
  applied state via the real fold.
- **T7 — Kill the switch + gates** (deps T4-T6): delete fallback; registry-
  completeness test driven by the host pb2 oneof descriptor (auto-gains future
  arms; pose/otos pinned ERR_UNIMPLEMENTED(7/8)); flash/RAM delta report;
  architecture-update.md records the SUC-006 reversal + envelope-quarantine note.
- **T8 — Capstone: POSE + OTOS processors** (deps T7): transcribe from retained
  pose/otos_commands.cpp; post to poseResetIn/otosCommandIn/otosSetPoseIn (consumer
  wiring on hardware follows the T6 pattern — verify PoseEstimator/odometer drain
  exists per-target and scope accordingly at sprint planning). **Acceptance: zero
  diffs to message_router.*, binary_channel.cpp, or generator dispatch logic.**
- **HITL bench gate** (per .claude/rules/hardware-bench-testing.md): binary
  drive/segment/stop on the stand, PING/ID over the real link, and the newly-live
  hardware CONFIG→GET fold round-trip.

## Key risks

- **R1 SUC-006 reversal** — recorded as an architecture decision; the
  post-and-reply contract preserves what SUC-006 actually protected.
- **R2 Two adversarial-input-only dispatch divergences** (multi-arm envelope with
  malformed first arm; corr_id after malformed arm — now reported correctly, strictly
  better). No field-ascending encoder (any real client) produces either; pin with
  tests + doc notes in T3.
- **R3 097 collision** — hard-sequenced: branch only after 097 closes (overlap:
  telemetry_commands.cpp, command_router.cpp, binary_channel.cpp, main.cpp).
- **R4 Hardware behavior change** (Configurator rider): CONFIG acks start meaning
  "applied"; bench gate exercises it.
- **R5 Footprint**: +128B BSS registry, ~1-2KB flash (vtables/peek/decodeMessage)
  offset by switch + duplicate-funnel deletion; exact deltas reported in T7.

## Verification

1. Every ticket: `just build-sim` + `uv run python -m pytest tests/sim tests/unit`
   green; `test_binary_channel.py` byte-identical (reply bytes asserted, not just
   status).
2. T2: differential corpus equivalence (peek+decodeMessage ≡ decode) + regenerated
   decode()/encode() byte-unchanged (git diff).
3. T7: pb2-descriptor-driven completeness sweep — every declared arm answers, with
   pose/otos pinned to ERR_UNIMPLEMENTED(7/8) until T8 flips them to live.
4. ARM build (`just build`) each ticket; flash/RAM delta report at T7.
5. HITL bench gate on the stand at sprint end (drive/encoders/config fold over the
   real serial link, binary plane end-to-end).

## Process routing

Sprint 096 closed; 097 planning is in-flight in a parallel session. On approval:
file this plan as a `clasi/issues/` item, then dispatch the sprint-planner to
formalize it as the next sprint **after 097** (insert before/merged-with the
POSE/OTOS 098 earmark per stakeholder decision), with T8's zero-diff acceptance
criterion written into the capstone tickets. Implementation is dispatched
per-ticket to programmer agents per the CLASI process; the staged per-arm helpers
(commit 294feec5) are the movable bodies.
