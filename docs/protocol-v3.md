# Protocol v3 Wire Specification

Version 3 of the Nezha firmware command/telemetry protocol: a
schema-driven **binary envelope command plane** (`*B<base64(protobuf)>`),
a deliberately tiny **hand-typeable text safety rump** (`STOP`/`PING`/
`HELLO`), and a host-side **`rogo` translator proxy** that speaks the old
protocol-v2 text grammar to legacy clients while talking binary-only to
the robot. This document describes the wire surface as it exists after
sprint 097 ("Protocol v3 Sprint 3: host completion and text retirement")
tickets 004/006/007/008 landed.

`docs/protocol-v2.md` is **superseded** by this document for every family
097 gutted (motion, config, telemetry) — see the banner at the top of
that file. `docs/protocol-v2.md` §11 (OTOS/port I/O) and §16
(Development commands) still accurately describe those two families'
*text grammar*, but that grammar has been off the wire (unregistered,
never called from `Rt::CommandRouter::buildTable()`) since before sprint
097 even started — see §8 below.

---

## 1. Overview

The robot understands exactly three kinds of line on its serial/radio
command channel, discriminated by the first character(s):

| Line shape | Meaning | Handled by |
|---|---|---|
| `*B<base64>` | Binary `CommandEnvelope` | `source/commands/binary_channel.cpp` |
| `STOP` / `PING` / `HELLO` (+ optional `#<id>`) | Text safety rump | `source/commands/system_commands.cpp` / `motion_commands.cpp` |
| anything else | Unrecognized | `CommandProcessor::dispatchTable()`'s no-match fallback → `ERR unknown` |

`CommandProcessor::process()` branches on `line[0] == '*'` **before**
tokenizing (`source/commands/command_processor.cpp:428`) — base64 must
never be uppercased or whitespace-tokenized the way a text verb line is.
Every other line goes through the same `parseTokens()`/`dispatchTable()`
path protocol v2 always used; with the motion/config/telemetry families
deleted outright (not merely unregistered), only `STOP`/`PING`/`HELLO`
still match anything in the table (§6).

For everything the rump doesn't cover, the **primary compatibility path**
is the `rogo proxy` PTY bridge (§7), not the binary wire directly — a
legacy text client (TestGUI, calibration scripts, gamepad teleop) is
expected to open the proxy's published device path exactly as it always
opened a real serial port.

---

## 2. Envelope framing

```
*B<base64(CommandEnvelope bytes)>\n     -- host -> robot
*B<base64(ReplyEnvelope bytes)>\n       -- robot -> host
```

- **Base64 alphabet: standard (`+/`), NOT url-safe (`-_`)**, RFC 4648 `=`
  padding. Pinned once, both sides must agree:
  `source/messages/wire_runtime.h`'s file header vs.
  `host/robot_radio/io/serial_conn.py`'s `send_envelope()` (Python stdlib
  `base64.b64encode`/`b64decode`, whose default alphabet is this same
  standard one). There is no negotiation and no version byte — whichever
  alphabet a build encodes/decodes with *is* the wire format.
- **Dearmor** (`BinaryChannel::handle()`, `source/commands/binary_channel.cpp:496-519`):
  callers only reach this function once `CommandProcessor::process()` has
  already seen `line[0] == '*'`; `line[1] != 'B'` is still rejected
  (`ERR_DECODE`, field 0, corr_id 0) as a malformed/future-armor line.
  Trailing `\r`/`\n`/space/tab is trimmed off the base64 payload before
  `WireRuntime::base64Decode()` runs.
- **Decode**: `msg::wire::decode()` (generated, `source/messages/wire.{h,cpp}`)
  walks `CommandEnvelope`'s generated field table, validating every
  `(min)`/`(max)`/`(abs_max)`/`(req)` bound **inline** during the same pass
  — see `protos/options.proto` for what each option means and the
  per-message `.proto` files cited in §3 for what each field's own bound
  actually is (this document cites, it does not restate). Unknown field
  numbers are skipped, not rejected — forward-compatible with a future
  schema declaring a field number an older build doesn't recognize.
  Malformed/truncated bytes return `ERR_DECODE`.
- **Armor** (reply path): `msg::wire::encode()` emits only the currently-set
  `body` oneof arm plus `corr_id` (proto3 implicit presence — a zero
  `corr_id` is omitted, exactly like a real protobuf encoder), then
  `WireRuntime::base64Encode()` re-wraps it as `*B<base64>\0`.
- **Per-arm byte cap: 186 bytes.** `kCommandEnvelopeMaxEncodedSize = 168`,
  `kReplyEnvelopeMaxEncodedSize = 171`, each statically asserted `<= 186`
  (`source/messages/wire.h:43-59`). Both constants are the **worst case
  across mutually exclusive oneof arms** (max, not sum), recomputed by
  `scripts/gen_messages.py` on every build — a schema change that pushes
  either total past 186B fails the build (`static_assert`), never a
  silently truncated wire line at runtime.
- **Armored buffer: 256 bytes** (`kArmoredBufSize`,
  `source/commands/binary_channel.cpp:48-53`) — sized to match
  `Subsystems::CommunicatorToCommandProcessorCommand::line`'s own 256-byte
  budget, so an armored reply always fits the same transport channel a
  request arrived on.
- **Reply-exactly-once contract**: every `handle<Arm>()` helper in
  `binary_channel.cpp` funnels through one `sendReply()`/`sendError()`
  pair; no bare (unarmored) text ever leaves `BinaryChannel::handle()` —
  a binary client only ever sees `*B<base64>` lines back.
- **Nesting depth bound**: length-delimited recursion is capped at 8
  levels (`WireRuntime::kMaxNestingDepth`) — this schema's deepest actual
  chain (`CommandEnvelope → DrivetrainCommand → WheelTargets → repeated
  WheelTarget`) is 3.

---

## 3. `CommandEnvelope` — command arms

`protos/envelope.proto`'s `CommandEnvelope.cmd` oneof, dispatched by
`BinaryChannel::handle()`'s switch (`source/commands/binary_channel.cpp:538-582`).
Every `(min)`/`(max)`/`(abs_max)`/`(req)` bound is declared as a field
option in the cited `.proto` file, transcribed from the matching
pre-097 text-handler constant (095 Decision 5's "transcribe, never
re-derive" discipline) — read the `.proto` file for the exact numbers;
this table names the message, the Blackboard destination, and anything
not obvious from the field list alone.

| Arm (field #) | Payload message | Blackboard / Configurator path | Notes |
|---|---|---|---|
| `drive` (2) | `DrivetrainCommand` (`drivetrain.proto`) | `b.driveIn.post(cmd)` — posted verbatim, no translation | `oneof control`: `twist`/`wheels`/`neutral`/`pose`, plus `seed`/`standby` side-channel bools |
| `segment` (3) | `MotionSegment` (`motion.proto`) | `b.segmentIn.post(toSegment(src))` → `ERR_FULL` if the queue is full | Field-by-field copy into `Motion::Segment`'s own native units (mm, rad, mm/s, …) — the MOVE-equivalent; every bound cites its `motion_commands.cpp` source constant in the `.proto` file's own comments |
| `replace` (4) | `MotionSegment` | `b.replaceIn.post(...)` — a `Mailbox`, latest-wins, cannot fail | The MOVER-equivalent (streaming/deadman teleop primitive) |
| *(reserved 5)* | — | — | `PlannerCommand` (`motion`, the R/TURN/G-equivalent) is **reserved, not declared** — its 327B worst case alone exceeds the 186B cap; a future sprint declares it with a new, deliberately-bounded payload type once `Subsystems::Planner` un-parks |
| `config` (6) | `ConfigDelta` (`envelope.proto`/`config.proto`), `oneof patch`: `drivetrain`/`motor`/`planner`/`watchdog` | `drivetrain`/`motor`/`planner` patches → one field-masked `Rt::ConfigDelta` posted to `b.configIn` (the Configurator folds + applies it); `watchdog` (`sTimeout`) posts its `uint32` window **directly to `b.streamWatchdogWindowIn`**, bypassing the Configurator entirely — it is not one of the Configurator's four fold targets | `MotorConfigPatch.side` disambiguates `travel_calib` only; any present `kp`/`ki`/`kff`/`i_max`/`kaw` applies to **both** bound motors unconditionally (two separate `ConfigDelta` posts) |
| `pose` (7) | `SetPose` (`drivetrain.proto`) | **declared only** — `BinaryChannel` replies `Error{ERR_UNIMPLEMENTED, field=7}` | Reserved for sprint 098; see §8 |
| `otos` (8) | `OdometerCommand` (`odometer.proto`) | **declared only** — `Error{ERR_UNIMPLEMENTED, field=8}` | Reserved for sprint 098; see §8 |
| `ping` (9) | `Ping{}` (zero fields) | none | Reply `Ack{t=Types::systemClockNow()}` — the one `Ack` producer that ever sets `t`, for clock-sync parity with text `PING`'s `OK pong t=<ms>` |
| `echo` (10) | `Echo{payload}` (≤64 bytes) | none | Reply `body.echo` mirrors the payload verbatim |
| `get` (11) | `ConfigGet{target}` | Reads `bb.drivetrainConfig` / `bb.motorConfig[]` / `bb.plannerConfig` / `bb.streamWatchdogWindow` directly — a snapshot read, no queue | `target` is `optional` + `(req)=true`; a missing `target` is rejected by the generated decoder before dispatch ever reaches the handler (`ERR_BADARG`, field=1) |
| `stream` (12) | `StreamControl{binary, period}` | Sets `bb.telemetryPeriod` (floored to 20ms if nonzero, per `kStreamFloorMs`), `bb.telemetryChannel` (`router.currentChannel()`), `bb.telemetryBinary` | No "immediate first frame" concatenated into the ack (unlike the old text `STREAM`) — the first periodic frame arrives one `tickTelemetry()` pass later, uniformly |
| `stop` (13) | `Stop{}` (zero fields) | Builds `msg::DrivetrainCommand{NEUTRAL=BRAKE}` inline, posts to `b.driveIn` | Byte-identical construction to the text rump's own `STOP` handler (§6) — "cannot be malformed" by design, never derived from a caller-supplied field |
| `id` (14) | `DeviceId{}` (empty request) | none | Reply `body.id` = `model`/`name`/`serial`/`fw_version`/`proto_version`, sourced from the same `deviceIdentity()` helper the text rump's `HELLO`/boot banner use |

---

## 4. `ReplyEnvelope` — reply arms

| Arm | Payload message | Produced when |
|---|---|---|
| `ok` | `Ack{q, rem, t}` | Success for `drive`/`segment`/`replace`/`stop`/`config`/`stream`. `q` = `b.segmentIn.size() + b.drivetrain.queue`; `rem` = the live plan's remaining translation (mm); `t` stays 0 except on `ping` |
| `err` | `Error{code, field}` | Any rejection — see §5 |
| `tlm` | `Telemetry` (`telemetry.proto`) | `tickTelemetry()`'s periodic push once `StreamControl.period > 0`, on whichever channel bound it last — **unsolicited** (`corr_id = 0`), same "0 for an unsolicited reply" convention `envelope.proto`'s own doc comment states |
| `cfg` | `ConfigSnapshot` | Reply to `get` |
| `id` | `DeviceId` | Reply to `id` |
| `echo` | `Echo` | Reply to `echo` |
| `evt` | `EventNotify{}` (zero fields) | **Declared only — zero producers today.** `CommandProcessor::emitEvent()` (`command_processor.cpp:336-364`) is still the one place in the tree that assembles `"EVT ..."` wire text, but nothing calls it on the binary path (or anywhere else, currently). See §7's EVT-synthesis discussion for how the `rogo` proxy fills this gap **host-side** without firmware changes. |

`Telemetry`'s own field set (`telemetry.proto`) is a curated union of the
old periodic STREAM/SNAP frame and the old one-shot `TLM` verb's
bench-diagnostic fields — every field traces 1:1 to one of those two
retired text surfaces, **except** `encpose` (dropped in the 096-001 trim
to fit the 186-byte budget — see `telemetry.proto`'s own header comment).
Line-sensor and color-sensor fields (`line=`/`color=`) are not on the
binary wire, and were never on **this rebuilt tree's** text `TLM`/
`STREAM` wire either — the line/color HAL leaves are declared
(`source/hal/capability/{line_sensor,color_sensor}.h`) but never
instantiated or ticked in `source/`, so `source/telemetry/tlm_frame.cpp`
never had a `line=`/`color=` field to emit in the first place (see
`clasi/issues/restore-line-and-color-sensors-as-ticked-blackboard-devices.md`).
Per-wheel wedge diagnostics exist structurally
(`DrivetrainState.wheel_wedged`, `drivetrain.proto`) but were likewise
never surfaced as a `wedge=` `Telemetry` token. `docs/protocol-v2.md` §8's
`TLM Frame Format` table documents an older, `source_old/`-era wire
format that did carry all three (plus `otos_health=`/`ekf_rej=`) —
historical, not something this rebuilt tree's text plane ever exposed
before 097 gutted it (see §10's cross-reference table and
`.claude/rules/hardware-bench-testing.md`'s current sensor list).

---

## 5. Error taxonomy (`envelope.proto`'s `ErrCode`)

| Code | Meaning |
|---|---|
| `ERR_NONE` | (unused as a wire value; zero-default) |
| `ERR_UNKNOWN` | No such oneof arm, or an unknown enum target |
| `ERR_BADARG` | Malformed argument (mirrors text `badarg`) |
| `ERR_RANGE` | A `(min)`/`(max)`/`(abs_max)` bound was violated (mirrors text `range`) |
| `ERR_FULL` | Destination queue full (mirrors text `full`) |
| `ERR_DECODE` | Malformed wire bytes — bad base64, or a protobuf decode failure |
| `ERR_UNIMPLEMENTED` | A declared-only arm with no live consumer yet (`pose`/`otos` today) |
| `ERR_OVERSIZE` | An encoded reply would exceed the envelope cap |

`Error.field` names the `CommandEnvelope` field number that failed
validation (0 if not field-specific, e.g. `ERR_UNKNOWN`/`ERR_DECODE`); a
host maps it back to a field name via the same schema
(`CommandEnvelope.DESCRIPTOR.fields_by_number`, exactly what
`legacy_render.field_name_for_error()` does for the proxy, §7).

---

## 6. Text safety rump — `STOP` / `PING` / `HELLO`

The firmware's entire text command table (`Rt::CommandRouter::buildTable()`,
`source/runtime/command_router.cpp:27-34`) is now:

```cpp
std::vector<CommandDescriptor> all = systemCommands(router);   // PING, HELLO
std::vector<CommandDescriptor> motion = motionCommands(router); // STOP only
std::vector<CommandDescriptor> telemetry = telemetryCommands(router); // registers ZERO commands
```

- **`systemCommands()`** (`source/commands/system_commands.cpp:123-129`)
  registers exactly `PING` and `HELLO`.
- **`motionCommands()`** (`source/commands/motion_commands.cpp:82-87`)
  registers exactly `STOP` — every other verb that file ever held
  (`S`/`D`/`T`/`R`/`TURN`/`RT`/`G`/`MOVE`/`MOVER`, `QLEN`, the shared
  stop-clause grammar, `StreamingDriveWatchdog`) was **deleted outright**
  by ticket 097-006, and the one-shot `TLM` verb (`handleTlm()`) was
  deleted by ticket 097-008 — not merely unregistered, the parser/handler
  functions no longer exist anywhere in the file.
- **`telemetryCommands()`** (`source/commands/telemetry_commands.h:61-64`)
  registers **zero** commands post-097-008 — `STREAM`/`SNAP`'s text
  handlers and `telemetryEmit()` were deleted outright; the function is
  kept only as a stable no-op entry point so `command_router.cpp` didn't
  need touching.
- The now-deleted `config_commands.{h,cpp}` (text `SET`/`GET`) is gone as
  a *file*, not merely unregistered (ticket 097-007).

That leaves **three** hand-typeable verbs, byte-identical in behavior to
their pre-097 text-plane selves:

- **`STOP`** — `handleStop()` (`motion_commands.cpp:64-74`) posts
  `msg::DrivetrainCommand{NEUTRAL=BRAKE}` to `b.driveIn` and replies
  `OK stop`. This is the deliberate **safety affordance**: a human with a
  bare serial terminal (`screen`, `minicom`) and no host program, no
  protobuf tooling, and no base64 encoder can *always* halt the robot by
  typing five characters and Enter. It is byte-identical to the binary
  `stop` arm's own construction (§3) — the same "cannot be malformed"
  design, just reachable without the binary plane at all.
- **`PING`** — clock-sync probe, `OK pong t=<ms>` (same
  `Types::systemClockNow()` source the binary `ping` arm's `Ack.t` uses).
- **`HELLO`** — re-emits the `DEVICE:NEZHA2:robot:<name>:<serial>`
  identity banner (`formatDeviceAnnouncement()`, shared with the boot-time
  announcement and the binary `id` reply's fields). This is the
  **connect-handshake verb**: a host or a human can confirm they're
  talking to a live Nezha2 firmware and learn its identity without
  needing the binary plane at all — the same role it has always played,
  now doubling as the one text-plane path a client can probe before it
  knows whether the firmware even understands base64 lines.

Anything else sent as plain text — `ECHO`, `VER`, `HELP`, `ID`,
`S`/`D`/`T`/`R`/`TURN`/`RT`/`G`, `MOVE`/`MOVER`, `QLEN`,
`SET`/`GET`/`STREAM`/`SNAP`/`TLM` — no longer matches anything in
`buildTable()` and falls through to `CommandProcessor::dispatchTable()`'s
own no-match branch (`command_processor.cpp:103-107`): `ERR unknown`,
the identical code path a genuinely unrecognized verb has always hit.

---

## 7. The `rogo` translator proxy — the primary text-compatibility path

Everything the three-verb rump above doesn't cover is meant to go through
a standing host-side bridge, **not** through hand-built binary envelopes:
`rogo proxy` (`host/robot_radio/io/proxy.py`'s `ProtocolBridge`, ticket
097-004). It is the primary text-compatibility story for legacy clients —
see `clasi/issues/rogo-translator-proxy-text-v2-binary-bridge-on-a-pty.md`
for the full implementation spec this section summarizes.

### 7.1 Architecture

```
legacy client (pyserial / SerialConnection, unchanged code)
      │  text-v2 lines                 ~/.rogo/robot-pty (symlink -> PTY slave)
      ▼
┌─ rogo proxy ────────────────────────────────────────────────┐
│ pty-reader thread: line-split -> route:                     │
│   local (HELLO/HELP/!*/unknown) -- fake-ack / typed ERR      │
│   binary (everything BINARY_DISPATCH covers) -- send_envelope│
│   -> legacy_render -> write PTY                              │
│ tlm-pump thread: read_binary_tlm -> EvtWatcher(active)       │
│   -> synthesized EVT done / text TLM lines -> write PTY      │
└──────────────┬────────────────────────────────────────────────┘
               │  *B<base64> only          one SerialConnection
               ▼
        robot (serial or radio relay) -- binary-only firmware
```

### 7.2 Starting it

```bash
rogo proxy [--port <device-or-relay>] [--link ~/.rogo/robot-pty] \
           [--watch-period 50] [--no-evt] [-v]
```

`cmd_proxy()` (`host/robot_radio/io/cli.py:1606-1644`) opens the real
robot connection (`_make_robot(args)`), constructs one `ProtocolBridge`,
calls `bridge.start()`, prints the PTY slave path and the published
symlink, then blocks (`bridge.run_forever()`) until SIGINT/SIGTERM.

### 7.3 Transport: a PTY, not a socket

`ProtocolBridge.start()` calls `os.openpty()`, `tty.setraw()`s the slave,
sets the master fd non-blocking, and publishes a **stable symlink**
(default **`~/.rogo/robot-pty`**, override with `--link`) pointing at the
PTY slave device path. A legacy client opens that symlink **exactly like
a real serial port** — `serial.Serial(path)` / `SerialConnection(path)` —
with **zero code changes**. This was a deliberate stakeholder revision
(2026-07-10) away from the ticket's originally-planned `AF_UNIX` socket:
every legacy consumer already opens its port as a plain device path, so a
socket would have forced a code change into every one of them, recreating
the exact migration problem the proxy exists to avoid.

**Single-client contract**: exactly one client is expected to have the
PTY slave open at a time — documented (module docstring, `--help`), not
policed. A second concurrent client would interleave reads/writes,
undefined for this bridge's purposes. The routing core
(`_handle_client_line`/`_EvtWatcher`) is transport-agnostic, so an
additive `AF_UNIX` listener remains cheap to add later if multi-client
need materializes.

Upstream (proxy → robot) is binary-only over one real `SerialConnection`
— the same transport (serial or radio relay) any other `rogo`/
`robot_radio` tool uses.

### 7.4 Verb routing

Every line the pty-reader thread reads is split into `(VERB, positional,
kv)` (`legacy_verbs.tokenize_send_line()`, mirroring the firmware's own
`parseTokens()`/`parseKV()`) and a trailing `#<digits>` corr-id
(`legacy_verbs.split_corr_id()`), then routed by
`ProtocolBridge._handle_client_line()`:

| Client sends | Route | Rendered reply |
|---|---|---|
| `S`, `D`, `T`, `RT`, `MOVE`, `MOVER`, `ECHO`, `PING`, `STOP`, `ID`, `VER` | `legacy_verbs.BINARY_DISPATCH[verb]` builds a `CommandEnvelope`; one blocking `send_envelope()` round trip | `legacy_render`'s per-verb `OK`/`ID`/`ERR` line, transcribed byte-for-byte from the deleted text handlers' own `snprintf` formats |
| `HELLO` | **local** — answered from a `DeviceId` cached at proxy startup (retried once live if still empty) | `DEVICE:NEZHA2:robot:<name>:<serial>` |
| `HELP` | **local** | short proxy help text (firmware `HELP` is gutted) |
| `SET k=v ...` | binary fan-out — one `ConfigDelta` per distinct target, via `NezhaProtocol.set_config()`; unknown key → local `ERR badkey <k>` before any wire traffic | `OK set <k=v ...> [#id]` |
| `GET [keys]` | binary fan-out/fan-in — one `ConfigGet` per distinct target, merged; unknown key → local `ERR badkey <k>` | one `CFG k=v ... [#id]` line, `kAllKeys` order, firmware-exact per-key int/fixed-3-decimal formatting |
| `STREAM <n>` | binary `{stream: StreamControl{binary=true, period=n}}`; sets the client-stream flag the tlm-pump thread reads | `OK stream period=<0 or max(20,n)>` |
| `SNAP` | binary arm-wait-disarm-restore (never blindly cancels a client's own in-progress `STREAM`) | exactly one bare `TLM ...` line |
| `TLM` (one-shot) | same arm-wait-disarm-restore, rendered as the bench-diagnostic body | `OK tlm enc=... vel=... cmd=... acc=... active=... conn=... glitch=... ts=... now=... [#id]` |
| `+` (keepalive) | forwarded via `conn.send_fast("+")` (feeds the firmware's own liveness signal) | none |
| `!MODE`/`!CG`/`!P`/`!ECHO`/`!GO`/`?` (relay-control lines) | **local** swallow | `# ok` |
| `*B...` (a binary-native client on the proxy port) | **local**, never forwarded | `ERR unsupported proxy-is-text-only` — use the real port for binary tools |
| `QLEN`, `G`, `R`, `TURN`, `GRIP`, any `DEV *`, any other unrecognized verb | **local** typed error — no binary arm exists for any of these; the proxy cannot manufacture capability the firmware doesn't have | `ERR unsupported <verb>` |
| `SI`, `ZERO`, `OI`, `OZ`, `OR`, `OP`, `OV`, `OL`, `OA` | **local** typed error today (`_POSE_OTOS_BINARY = False`) — the binary `pose`/`otos` arms exist in the schema but reply `ERR_UNIMPLEMENTED` (§3); flip that flag once sprint 098 lands them | `ERR unsupported <verb>` |

`Error{code, field}` replies map to text via `ERR_CODE_TEXT`
(`legacy_render.py`): `UNKNOWN→unknown`, `BADARG→badarg`, `RANGE→range`,
`FULL→full`, `DECODE→badarg`, `UNIMPLEMENTED→unsupported`,
`OVERSIZE→unsupported`.

### 7.5 EVT synthesis

Legacy calibration scripts (`calibration/linear.py`/`angular.py`) block
on `EVT done D`/`EVT done T`. Current firmware emits **no EVT at all**
(§4) — the proxy synthesizes it, entirely host-side, via
`_EvtWatcher` (`io/proxy.py`), a pure state machine fed by the tlm-pump
thread's `Telemetry.active` samples:

- **`IDLE` → `WAIT_BUSY`**: an `Ack` for `T`/`D`/`RT`/`MOVE` just landed
  (`EVT_ARMING_VERBS`). If the client has no `STREAM` of its own armed,
  the pump thread arms an **internal-only** upstream stream at
  `--watch-period` (default 50 ms) — its frames feed the watcher only,
  never forwarded to the PTY.
- **`WAIT_BUSY` → `BUSY`**: `Telemetry.active` observed `True`.
  `WAIT_BUSY` has a 2s cap: if it expires while still waiting (a short
  segment can finish between two telemetry frames), the watcher emits
  anyway — late beats missing.
- **`BUSY` → `IDLE`**: `Telemetry.active` observed `False` → emits
  `EVT done <VERB> [#id] reason=idle`.
- **`STOP`** clears any pending watch **silently** (matches the v2 spec:
  `STOP` itself emits no event). A new motion verb's `Ack` **supersedes**
  whatever was pending, also silently.
- **Gap, not a regression**: `EVT safety_stop` is not synthesizable (no
  binary watchdog-stop signal exists to watch for) — firmware emitted no
  EVT at all before this proxy existed either.

### 7.6 What the proxy cannot restore

- **No binary arm exists** for `R`/`TURN`/`G`/`QLEN`/`GRIP` — the proxy
  returns a typed `ERR unsupported <verb>` for each; it cannot
  manufacture capability the firmware doesn't have. (`GRIP` in particular
  never had *any* firmware command handler, text or binary — `gripper.proto`
  declares a message shape but no `source/commands/*` file ever
  registered a `GRIP` verb.)
- **`SI`/`ZERO`/OTOS verbs** (`OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/`OA`) are
  gated behind `_POSE_OTOS_BINARY = False` today — the schema's `pose`/
  `otos` arms exist but reply `ERR_UNIMPLEMENTED` (§3/§8) until sprint 098.
- **`encpose`/line/color/`otos_health`/`ekf_rej` are not on the binary
  telemetry wire at all** (`telemetry.proto`'s field set, §4) — no
  transport fix restores them; any consumer reading those tokens off a
  proxied `TLM`/`STREAM` line is broken by the schema itself, not by the
  proxy. Line/color were never on **this rebuilt tree's** `TLM`/`STREAM`
  text wire either — `source/telemetry/tlm_frame.{h,cpp}` never emitted
  `line=`/`color=` in the first place (the line/color sensor HAL leaves
  are declared but never instantiated or ticked in `source/` — see
  `clasi/issues/restore-line-and-color-sensors-as-ticked-blackboard-devices.md`).
  `docs/protocol-v2.md` §8's `TLM Frame Format` table documents an older,
  pre-rebuild wire format (`source_old/`-era) that did carry `line=`/
  `color=`/`wedge=`/`otos_health=`/`ekf_rej=` — historical, not something
  this rebuilt tree's text plane ever exposed before 097 gutted it.

---

## 8. Off the wire entirely — not gutted, not proxied

Two families are a **different category** from everything §6/§7 cover.
The gutted-but-proxied families (`S`/`D`/`T`/`RT`/`MOVE`/`MOVER`/`ECHO`/
`VER`/`SET`/`GET`/`STREAM`/`SNAP`/one-shot `TLM`) each had a **live text
handler that 097 deleted outright**, replaced on the wire by a binary
arm, **and** covered by the proxy's translation table. `QLEN`/`R`/`TURN`/
`G` had dormant (unregistered) handler code that 097-006 deleted
alongside the live ones, and the proxy answers all four with a typed
error (§7.6) since no binary arm exists for any of them.

**OTOS/pose/DEV are different again**: their firmware source was **never
touched by sprint 097 at all**, and their `CommandDescriptor` tables were
**already unregistered before 097 started** — `command_router.cpp`'s own
`buildTable()` comment says so explicitly: *"Binary `config`/`get` ...
is the only live config-plane path, same as the still-unregistered
`dev`/`pose`/`otos` text families."*

- **`source/commands/otos_commands.{h,cpp}`** (OI/OZ/OR/OP/OV/OL/OA) and
  **`source/commands/pose_commands.{h,cpp}`** (SI/ZERO) — both files
  exist, `otosCommands()`/`poseCommands()` both build valid
  `CommandDescriptor` tables, but **neither function is called anywhere**
  except its own definition (confirmed by grep — no call site outside
  the two `.cpp` files themselves). They are kept as the **transcription
  reference** for sprint 098's binary `pose`/`otos` `CommandEnvelope`
  arms (envelope.proto fields 7/8, §3) — 098 owns porting their field
  shapes and Blackboard-queue targets (`bb.poseResetIn`/
  `bb.otosSetPoseIn`/`bb.otosCommandIn`/`bb.motorResetIn[]`, per each
  header's own doc comment) into `BinaryChannel`.
- **`source/commands/dev_commands.{h,cpp}`** (`DEV M`/`DEV DT`/`DEV
  STATE`/`DEV STOP`/`DEV WD`) — same "exists, never registered" status,
  for a **different reason**: `DEV` is a `ROBOT_DEV_BUILD`-gated
  bench-diagnostic family for raw per-port motor control, bypassing
  `Drivetrain` entirely. It never had, and does not need, a binary
  counterpart — no sprint has ever proposed a `dev` `CommandEnvelope` arm.

Both families are proxied the same way `QLEN`/`R`/`TURN`/`G` are (a local
typed `ERR unsupported <verb>`, §7.6) — but that is a **proxy-side**
statement about client compatibility, not a firmware-side statement about
what 097 did. Firmware-side, OTOS/pose/DEV are simply dead source on disk
today, unrelated to this sprint's gut.

---

## 9. Accepted breakage window

The following host tools point their serial connection **directly at the
robot**, not at the `rogo proxy`, and are **currently broken** against
this firmware — they send text verbs the firmware no longer understands
at all (§6) and have not yet been rewired to open `~/.rogo/robot-pty`
instead of a real port:

- **TestGUI**'s manual command panel (`host/robot_radio/testgui/
  commands.py`'s `COMMANDS` table) and its hardcoded `"STREAM 50"` on
  every connect.
- **`host/robot_radio/io/robot_mcp.py`**'s calibration push
  (`push_calibration()` falls through to raw text `SET`).
- **`host/robot_radio/calibration/linear.py`/`angular.py`** — raw text
  `D`/`T`/`STREAM`/`SNAP` over `RelaySerial`/`DirectSerial`.
- **`host/robot_radio/io/cli.py`**'s `cmd_turn`'s default (non-
  `--open-loop`) path and `_push_calibration()` (`rogo sync-cal`).
- **`host/calibrate_verify.py`** — raw text `SET`/`GET`.
- **`tests/bench/gamepad_teleop.py`** — raw text `MOVER`.
- **`tests/bench/dtr_drive_demo.py`/`random_segment_demo.py`** — raw text
  `MOVE`.

This is a **deliberate, accepted cost** of sprint 097's decision to gut
the firmware text plane unconditionally rather than wait for every
consumer to migrate first (`architecture-update-r2.md` Decision 9): the
proxy exists precisely so these tools *can* keep working, by repointing
their `--port`/serial-path argument at `~/.rogo/robot-pty` — but that
repointing is **not done yet** for any of them. Tracked by
`clasi/issues/realign-host-tooling-to-gutted-four-verb-wire-surface.md`,
which now explicitly owns migrating each of the tools above. Do not
imply continuity that doesn't exist: as of this document, every tool in
this list fails the moment it sends its first text verb to the real
robot.

---

## 10. Cross-reference: `docs/protocol-v2.md` section → v3 status

| v2 section | Status under v3 |
|---|---|
| §2 Grammar, §3 Response Taxonomy, §4 Error Codes, §5 `#id` Correlation | Still describes the **text rump**'s own grammar (§6 above) exactly — `STOP`/`PING`/`HELLO` use the identical tokenizer, corr-id extraction, and `OK`/`ERR` reply shapes |
| §6 Liveness/Identity (`PING`/`ECHO`/`ID`/`DEVICE:`/`HELLO`/`VER`/`HELP`) | `PING`/`HELLO` live (rump); `ECHO`/`ID`/`VER`/`HELP` deleted from the firmware text plane — binary `ping`/`echo`/`id` arms exist (§3); proxied (§7.4) |
| §7 Config (`SET`/`GET`) | Firmware text family deleted (`config_commands.{h,cpp}` file removed); binary `config`/`get` arms exist (§3); proxied (§7.4) |
| §8 Telemetry (`STREAM`/`SNAP`/`TLM` frame) | Firmware text handlers deleted; binary `stream` arm + `tlm` reply exist (§3/§4); proxied (§7.4) |
| §9 Time Synchronisation | Unchanged in spirit — binary `ping`'s `Ack.t` carries the same clock-sync role text `PING`'s `t=` did |
| §10 Motion Commands (`S`/`T`/`D`/`R`/`TURN`/`RT`/`G`/`VW`/`RF`/`STOP`/`GRIP`/`SI`) | `STOP` lives (rump); `S`/`T`/`D`/`RT`/`MOVE`/`MOVER` deleted, binary `drive`/`segment`/`replace`/`stop` arms exist (§3), proxied (§7.4); `R`/`TURN`/`G`/`GRIP` had no binary arm before or after — proxy returns typed `ERR unsupported` (§7.6); `VW`/`RF` were already off the wire pre-097 (not covered by this document — see the realign issue); `SI` is off-the-wire (§8) |
| §11 OTOS/Port I/O (`OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/`OA`/`P`/`PA`) | Off the wire entirely, untouched by 097 — §8 |
| §12 Buffer/Framing Note | Superseded by §2 above for the binary plane; still accurate for the text rump's own line length |
| §13 Verification Examples | Stale for every deleted verb; see `.claude/rules/hardware-bench-testing.md` and `tests/bench/` for current bench sequences |
| §14 Debug Commands (`DBG ...`) | Not covered by this document — unaffected by 097; verify separately against current source before relying on it |
| §15 Sim parameters | Unaffected by 097 (ctypes-only, not a wire concern) |
| §16 Development Commands (`DEV ...`) | Off the wire entirely, untouched by 097 — §8 |

---

## Appendix: quick reference

- **Binary CLI**: `rogo binary <arm> ...` (see `rogo --help`) sends a
  single hand-built `CommandEnvelope` directly, bypassing the proxy —
  useful for probing the firmware's own binary behavior.
- **Proxy CLI**: `rogo proxy` (§7.2) — the PTY bridge for legacy text
  clients.
- **Hardware bench verification**: follow
  `.claude/rules/hardware-bench-testing.md` for the sensor/encoder/
  round-trip gate; the proxy's own bench verification sequence lives in
  `clasi/issues/rogo-translator-proxy-text-v2-binary-bridge-on-a-pty.md`'s
  "Verification" section (PTY symlink + identity banner at startup,
  `screen ~/.rogo/robot-pty` smoke test, motion + EVT, telemetry,
  flagship unmodified-legacy-client run via `calibration/linear.py
  --port ~/.rogo/robot-pty --direct`, gamepad teleop, client churn,
  relay-upstream variant).
