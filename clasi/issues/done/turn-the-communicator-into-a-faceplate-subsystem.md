---
status: done
---

# Turn the Communicator into a Faceplate Subsystem

## Context

The Communicator ([source/com/communicator.h](source/com/communicator.h)) was copied into the greenfield tree in sprint 077 as legacy-style infrastructure (global namespace, `_member` prefixes, `serial()`/`radio()` pass-through accessors). Stakeholder direction (2026-07-04): make it a subsystem with a faceplate like everything else. It has **no command-in channel** — it is a *source* of commands — and its `tick()` **produces** commands. The line buffers, currently separate stack arrays in `main.cpp` threaded through `pollComms()`, are internalized so the statically-constructed Communicator contains everything.

**Radio framing note** (stakeholder-reviewed): the new-tree Radio ([radio.h:13-26](source/com/radio.h#L13-L26)) speaks the RadioRelay RAW250 fragment framing (§5: `[SEQ][FLAGS][LEN][payload≤247]`, START/MORE/END, reassembly on END), so multi-fragment messages exist at the driver level. But nothing in this refactor relies on large reassembly: the internal line buffer is **256 bytes, byte-identical to today's behavior**. The protocol principle going forward is that long replies (e.g. a future v2 GET dump) are sent in **multiple parts**, not reassembled monoliths — a sender-side design rule for the ticket that introduces GET, out of scope here.

## Design

New `Subsystems::Communicator` at `source/subsystems/communicator.{h,cpp}` (old `source/com/communicator.{h,cpp}` deleted). It keeps owning `SerialPort` + `Radio` by value (those stay untouched at `source/com/` as infrastructure leaves) and internalizes a **single shared line buffer** `line_[256]` — serial and radio command lines are the same format (the !GO data plane carries plain lines both ways), so one buffer, one line at a time.

Faceplate channels:
- **config** — `configure(const msg::CommunicatorConfig&)`: `radio_channel` (0..35, clamped via `radiochan::clamp()`; proto zero-default == `radiochan::kDefault` == 0). After `begin()`, a changed channel retunes live via `Radio::setChannel()`. No baud field — nothing drives `setBaud` today; it stays a `SerialPort` primitive.
- **command-in** — deliberately absent (no `apply()`); documented on the class.
- **command-out** — returned from `tick(now)` as a plain edge struct (precedent: `DrivetrainToMotorCommand`, [drivetrain.h:47-50](source/subsystems/drivetrain.h#L47-L50)): **one parsable line plus its return path**:
  ```cpp
  enum class Channel : uint8_t { NONE, SERIAL, RADIO };

  struct CommunicatorToCommandProcessorCommand {
    const char* line;    // nullptr when no complete line this tick
    Channel returnPath;  // where the reply to this line must be sent
  };
  ```
  At most ONE line per tick: serial is checked first; a radio message not taken this tick stays latched in `Radio::_msg` until the next poll ([radio.h:20-24](source/com/radio.h#L20-L24)), and the loop runs ~kHz vs ≤12 radio msg/s — no loss, no starvation. The pointer aliases the internal buffer, valid until the next `tick()` (safe: `CommandProcessor::process()` copies the line before parsing).
- **observation** — `state()` → `msg::CommunicatorState`: `radio_channel` + received-line counters per channel (DEV bench visibility).
- `capabilities()` → `msg::CommunicatorCapabilities`: `serial`/`radio` bools.
- **Primitive sends** — `sendSerial(const char*)` / `sendRadio(const char*)` (reply adapters build on these; keeps today's `send()` semantics, not `sendReliable()`). The `serial()`/`radio()` accessors are **removed** — that's the point of internalizing.
- `begin()` — hardware bring-up on the configured channel; only ONE Communicator may `begin()` (Radio's datagram ISR is a static singleton — document, no code change).

New code uses current style: 2-space Google layout, trailing-underscore members, lowerCamelCase functions, `// [ms]` tag on `tick(now)`.

## Steps

1. **`protos/communicator.proto`** (new): `CommunicatorConfig { uint32 radio_channel = 1; }`, `CommunicatorState { uint32 radio_channel; uint32 serial_lines; uint32 radio_lines; }`, `CommunicatorCapabilities { bool serial; bool radio; }`. Header comment states there is intentionally no `CommunicatorCommand`. No `(units)` options — all fields dimensionless.
2. **`scripts/gen_messages.py`**: add `"CommunicatorConfig"` to `_SETTER_TYPES` (~line 36); add six `_INVENTORY_MAP` rows for the new fields. Run `python3 scripts/gen_messages.py` → generated `source/messages/communicator.h` (never hand-edited; it will carry unused `get_*` accessors like every generated header until [clasi/issues/remove-generated-get-accessors.md](clasi/issues/remove-generated-get-accessors.md) lands — not a blocker).
3. **`source/subsystems/communicator.h`** (new): `Channel` enum + edge struct + class per Design above.
4. **`source/subsystems/communicator.cpp`** (new): port constructor/begin from old communicator.cpp + poll logic from main's `pollComms()`. `tick()` = try `serial_.readLine(line_, ...)` → `{line_, SERIAL}`; else try `radio_.poll(line_, ...)` → `{line_, RADIO}`; else `{nullptr, NONE}`; bump the matching counter. `configure()` clamps and live-retunes. Includes `"com/radio_channel.h"` (main.cpp drops it).
5. **`source/main.cpp`** — rewire BOTH `#if ROBOT_DEV_BUILD` branches:
   - Include `"subsystems/communicator.h"`; drop `"radio_channel.h"`.
   - Adapters cast ctx to `Subsystems::Communicator*` → `sendSerial`/`sendRadio`.
   - Construction: `static Subsystems::Communicator comm(uBit.serial, uBit.radio, uBit.messageBus);` then `configure(msg::CommunicatorConfig{})` + `begin()`.
   - Delete `pollComms()` and the two stack line buffers; loop body becomes:
     ```cpp
     Subsystems::CommunicatorToCommandProcessorCommand in = comm.tick(now);
     if (in.line) {
         watchdog.feed(now);   // any line, either channel, regardless of dispatch outcome
         cmd.process(in.line,
                     in.returnPath == Subsystems::Channel::RADIO ? radioReply : serialReply,
                     &comm);
     }
     ```
     (`#else` branch: same shape, no watchdog.) `setSerialReply(serialReply, &comm)` and the `replyEvt` ctx likewise become `&comm`. Update the file-header comment describing `pollComms()`.
   - Watchdog stays in the main/dev layer — no comms coupling added.
6. **Delete** `source/com/communicator.{h,cpp}`. No build edits needed (CMake globs `source/**/*.cpp` at configure time) — run `just build-clean` once after the file moves to force a re-glob.

## Files

| Action | File |
|---|---|
| create | `protos/communicator.proto`, `source/subsystems/communicator.{h,cpp}` |
| generated | `source/messages/communicator.h` (+ optional `docs/design/message-inventory.md` via `--emit-inventory`) |
| modify | `scripts/gen_messages.py`, `source/main.cpp` |
| delete | `source/com/communicator.h`, `source/com/communicator.cpp` |

Pattern references: [source/subsystems/drivetrain.h](source/subsystems/drivetrain.h) (faceplate + edge type), [protos/drivetrain.proto](protos/drivetrain.proto) (proto style).

## Risks

- **Edge pointer lifetime**: consumers must dispatch before the next `tick()` — documented on the struct; safe with today's only consumer.
- **One line per tick** (was: one per channel per iteration): when both channels have a line in the same iteration, the radio line waits one loop pass (~ms). Latched in `Radio::_msg`, so nothing is dropped; negligible at real message rates.
- **`#else` branch bit-rot**: must be compiled, not just edited (see verification).
- Do not re-add `serial()`/`radio()` accessors "for convenience"; a future need becomes a new primitive.
- `state()`/`capabilities()` initially uncalled — faceplate completeness; optional follow-up: a `DEV COMM STATE` command to surface the counters.
- Radio messages longer than 255 bytes clip exactly as today (unchanged behavior; long replies are a sender-side multi-part concern for the future GET ticket).

## Verification

1. **Codegen**: `python3 scripts/gen_messages.py` — `source/messages/communicator.h` appears with `setRadioChannel`; no diffs in other generated headers.
2. **Build both forks**: `just build-clean`; then temporarily set `"ROBOT_DEV_BUILD": 0` in `codal.json`, build again (proves the `#else` branch), revert. `grep -rn 'com/communicator' source/` (excluding source_old) → empty.
3. **HITL on the stand** (robot mounted, wheels free — per hardware-bench-testing rule): `mbdeploy probe` → `mbdeploy deploy --build`, then:
   - Serial: `PING` → `OK PING`; `ECHO hello` round-trips (`sendSerial` path).
   - Watchdog: `DEV M 1 VEL 120` spins the wheel, `DEV M 1 STATE` shows climbing position/converging vel; go silent >1 s → exactly one `EVT dev_watchdog` + motor neutralizes (proves `feed()` moved intact).
   - Radio: through the RadioRelay (channel 0, group 10, `!GO` data plane): `PING` replies over radio while serial `PING` still replies on serial — per-origin reply routing (the edge's `returnPath`) survived.
4. Run the repo test suite (`uv run python -m pytest`) before commit.
