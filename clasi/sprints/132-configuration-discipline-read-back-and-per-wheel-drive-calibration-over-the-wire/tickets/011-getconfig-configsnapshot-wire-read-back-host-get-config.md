---
id: '011'
title: GetConfig/ConfigSnapshot wire read-back + host get_config()
status: done
use-cases:
- SUC-003
depends-on:
- '007'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# GetConfig/ConfigSnapshot wire read-back + host get_config()

## Description

Add the `GetConfig`/`ConfigSnapshot` wire round trip, per-`ConfigTarget`
(the 240 B envelope cannot carry the whole object at once — see
sprint.md Design Rationale Decision 3). Firmware: `robot_loop.cpp`'s
`processMessage()` gains a `GetConfig` branch that reads
`Configurator::config()`'s relevant group and replies with a
`ConfigSnapshot` carrying that group's current values (reusing the same
generated group codec's encode path `applyGroup` used to decode). Host:
`NezhaProtocol.get_config(target)` sends `GetConfig` and returns the
parsed `ConfigSnapshot`.

## Acceptance Criteria

- [x] A `GetConfig(target)` request returns a `ConfigSnapshot` carrying
      that group's current values from `Configurator::config()`.
- [x] `get_config(target)` round-trips a value just pushed to that same
      target via `applyGroup` (push then get shows the pushed value, not
      a stale baked one) — verified in sim.
- [x] `NezhaProtocol.get_config(target)` exists on the host and returns a
      typed result (using the generated pydantic group model from ticket
      002, not a raw dict).
- [x] Compiles under `HOST_BUILD`.

## Testing

- **Existing tests to run**: unaffected — new capability, no existing
  call site changes.
- **New tests to write**: the push-then-get round-trip test above, for
  at least `DRIVE` and one boot-only target (confirming GET still works
  for boot-only groups even though SET is rejected — read-back is not
  gated by re-appliability, only writes are).
- **Verification command**: `uv run python -m pytest
  <configurator/protocol test paths> -q`.

## Implementation Plan

**Approach**: Add `GetConfig`/`ConfigSnapshot` handling in
`robot_loop.cpp`'s `processMessage()`, encoding `config_`'s relevant
group via the same generated encode path `install`'s wire equivalent
would use. Host side: a `get_config()` method on `NezhaProtocol`
mirroring the shape of its existing request/reply methods (e.g.
`PING`/`PONG`'s round-trip pattern).

**Files to modify**: `src/firm/app/robot_loop.cpp`,
`src/firm/app/configurator.{h,cpp}` (if `config()` needs a per-group
accessor rather than just the whole struct),
`src/host/robot_radio/robot/protocol.py`.

**Testing plan**: as above.

**Documentation updates**: `docs/protocol-v5.md` gets a short addendum
documenting `GetConfig`/`ConfigSnapshot` as part of the CONFIG binary
arm — confirm this is expected under this project's protocol-doc
maintenance convention before skipping it.

## Completion Notes

**Wire shape.** `GetConfig{target}` (`CommandEnvelope.cmd.get_config`,
field 24 — the next fresh number after `wheels`(22)/`estop`(23)) and
`ConfigSnapshot{target, body}` (`ReplyEnvelope.body.cfg`, field 12 — the
first free number after `reserved 5 to 11`). Two new `commands.proto`
verbs: `GET_CONFIG`(19)/`CFG`(20), both `(binary)=true`. `get_config`'s
arm name is load-bearing (host derives the wire prefix from
`WhichOneof("cmd").upper()`); `cfg`'s is a naming choice only
(`Comms::sendReply()`'s `bodyKindToVerb()` is a hand-written switch, not
string-derived).

**Reply budget.** Adding `cfg` (`ConfigSnapshot`'s ~220 B `body` capacity)
made it `ReplyEnvelope`'s new worst-case oneof arm — 228 B, ahead of
`tlm`'s 188 B — bringing `kReplyEnvelopeMaxEncodedSize` to 232 B, 8 B
under the 240 B envelope budget (measured by `gen_messages.py`'s own size
report after regeneration). That growth cascaded into `comms.h`'s
hand-maintained buffer constants, which have their own `static_assert`
safety net that fired immediately: `kFramedMaxBytes` 200→238,
`kMaxLineBytes` 207→249. **Flagged, not silently absorbed**:
`kMaxLineBytes` (249) now sits only 1 byte below
`Com::SerialPort::kTxBufferCapacity` (250) — not a buffer-overflow risk,
but a real, concrete headroom risk one layer down (a worst-case `CFG`
reply competing with still-draining serial traffic can be silently
dropped under `Comms::sendReply()`'s async, drop-on-full `send()`
policy). Documented in both `comms.h` (inline, at `kMaxLineBytes`) and
`docs/protocol-v5.md` §6.1; not fixed here — shrinking
`ConfigSnapshot.body`'s `(max_count)` or raising `kTxBufferCapacity` are
both decisions outside this ticket's own wiring scope.

**Firmware.** `gen_messages.py` gained an encode-direction mirror of
132-008's `decode(<Group>&, ...)` family — `msg::wire::encode(<Group>&,
...)`, one overload per robot-config group, reusing the existing
`encodeInto()` engine `encode(ReplyEnvelope&, ...)` already uses.
`Configurator::encodeSnapshot(target, out)` (new) dispatches to the
matching group's `encode()` call; deliberately NOT gated by
`isLiveConfigurable()` — read-back works for every `ConfigGroupTarget`,
including boot-only ones (`GEOMETRY`/`PLANNER`), since only writes are
re-appliability-gated. `RobotLoop::handleGetConfig()` (new) calls it and
replies via `Comms::sendReply()` — the SECOND live call site that method
has ever had (the first is `Telemetry::emitPrimary()`'s unsolicited
`tlm` push); an unrecognized target replies `{err: ERR_BADARG}` instead
of dropping silently.

**Host.** `NezhaProtocol.get_config(target)` sends via the BLOCKING
`send_envelope()`/`_send_envelope()` path (not the ack-ring fire-and-poll
every other CONFIG method uses) and parses `reply.cfg.body` into the
target group's GENERATED pydantic model (`robot_config_generated.<Group>`,
ticket 002's own model) via the real compiled `robot_config_pb2.<Group>`
message — generic field copy by descriptor, not a per-group hand-written
mapping. Returns `None` on an unknown target, timeout, not-connected, or
an `err` reply.

**Read-back vs. the file.** Not proven end-to-end here (that is ticket
018's own acceptance criterion — "config() serialized diffs clean against
the robot JSON" requires the JSON reshape, ticket 017, to have landed
first, and requires iterating every `ConfigGroupTarget` host-side and
assembling the full picture). What THIS ticket proves, directly: (1)
`Config::Robot` already holds RAW file-shaped values (`Config::Robot`'s
own header comment, unchanged by this ticket) — so once the file reshapes,
`get_config()`'s per-group result is already file-shaped and diffable
field-for-field; (2) the read-back path itself is honest end-to-end,
verified both at the `Configurator` API level (push via `applyGroup()`,
read back via `encodeSnapshot()`, decode, compare — DRIVE, ESTIMATOR,
GEOMETRY) and at the host parsing level (a scripted `ConfigSnapshot` reply
parses into the correct typed model, one test per group).

**Deliberately left for a later ticket**: `SetConfigGroup`/`SetConfigField`
remain declared in `robot_config.proto` but unwired into `CommandEnvelope`
(tickets 012/013) — GET landing does not depend on SET landing first, per
the ticket's own testing note that read-back is not gated by
re-appliability.
