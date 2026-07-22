---
root: ../../../docs/design/design.md
---

# Com (src/firm/com)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-16 · **Status:** stable

---

## 1. Purpose

`com/` is the ARM-only raw transport layer: USB CDC serial (`SerialPort`) and
the micro:bit radio (`Radio`), plus persisted radio-channel storage
(`radio_channel.h`). It owns the two physical byte pipes the robot talks
over and the framing each one needs to turn bytes into complete lines — and
nothing else. It does not know what a line means; that is `app/Comms`'s job
(see [../app/DESIGN.md](../app/DESIGN.md)). The seam exists because these two
transports are the only code in the firmware that must touch `MicroBit.h`
directly outside `devices/microbit_*` and `main.cpp` — isolating them here is
what keeps the rest of the tree `HOST_BUILD`-clean (the system doc's
firmware-tree overview, `docs/design/design.md` §5, "HOST_BUILD purity").

## 2. Orientation

`SerialPort` wraps the CODAL `NRF52Serial` ASYNC API into a line-buffered,
non-blocking reader (`readLine()`) plus two senders with different drop
policies (`send()`, `sendReliable()`). `Radio` wraps `MicroBitRadio`'s
datagram API into the RadioRelay RAW250 fragment protocol: it reassembles
inbound fragments in the datagram ISR and hands a completed line to the main
loop via `poll()`, and fragments an outbound line across one or more frames
in `send()`. `radio_channel.h` is unrelated to either transport's byte path —
it just persists which nRF frequency band the radio uses across reboots.

`app/comms.h`'s `SerialTransport`/`RadioTransport` are the only consumers:
thin adapters that implement `App::Transport` by forwarding to a `SerialPort&`
or `Radio&`. `com/` itself has no dependency on `app/`, `messages/`, or any
wire-schema type — it moves opaque NUL-terminated C strings, one line at a
time, in both directions.

## 3. Constraints and Invariants

- **`send()` vs `sendReliable()` is a deliberate drop-policy split, not
  redundant API surface.** `send()` is ASYNC and drop-on-full — used for the
  telemetry flood, where a lost frame is harmless and a stalled loop is not.
  `sendReliable()` bounded-waits (5 ms cap on serial) for TX-buffer room
  before handing off — used for replies/EVT, where silently dropping a line
  is a correctness bug. Routing a reply through `send()` reintroduces silent
  reply loss under buffer pressure; routing telemetry through
  `sendReliable()` reintroduces the stalled-loop failure mode the split
  exists to prevent (`docs/design/design.md` §5, "App modules are passive
  and bounded").
- **Neither transport ever blocks unboundedly or sleeps.** `readLine()`/
  `poll()` are non-blocking; `sendReliable()`'s wait is capped (5 ms serial,
  effectively bounded on radio by fragment count). A transport that blocks
  the calling cycle destroys the loop's timing budget exactly as an
  in-loop sleep would (`docs/design/design.md` §5).
- **Radio group is fixed at 10; only the channel (frequency band, 0–35) is
  configurable**, persisted in flash via `uBit.storage` (`radio_channel.h`).
  Group must match the RadioRelay's fixed group or the link never forms
  regardless of channel. The 0–35 range is not a hardware limit (the nRF
  supports bands up to 83) — it is chosen so the channel renders as one
  base-36 LED-matrix digit (`0`–`9`, then `A`–`Z`) for a glanceable boot
  display; widening the range past 35 breaks the single-digit display
  contract.
- **Re-tuning the radio channel drops the link immediately** — the relay
  stays on the old channel until it is separately retold. Any reply to the
  command that triggered a channel change must be sent BEFORE calling
  `Radio::setChannel()`, or the reply is lost with the old channel.
- **Only one `Radio` instance may call `begin()`.** `_instance` is a static
  singleton pointer the static ISR callback (`onData`) dereferences; a second
  `begin()` call silently redirects all interrupt-context reassembly to the
  new instance while the old one's buffers go stale.
- **Radio buffers exactly one completed message between ISR and `poll()`.**
  If a second message finishes reassembly before `poll()` drains the first,
  the newer message is dropped (not queued). This is acceptable because
  commands are processed far faster than they arrive; it stops being
  acceptable the moment anything depends on radio message ordering/completeness
  under back-to-back sends.
- **`SerialPort`'s CODAL TX buffer size is a `uint8_t`** — 255 is the actual
  max (a requested 1024 silently wraps to 0, i.e. no buffer at all). Replies
  must fit in a single line and must not be fired in a rapid burst; the
  firmware blocks or loses output when a burst outruns what 255 bytes can
  absorb. Do not "simplify" `begin()` by requesting a larger buffer size.
- **`Radio::send()` always appends a trailing `'\n'`**, mirroring
  `SerialPort::send()`'s `"\r\n"`. RAW250 START/END framing alone delimits a
  message on the wire, but after the relay's `!GO` handshake it becomes a
  transparent byte pipe with no per-message boundary of its own — without
  the embedded newline, consecutive robot→host messages (TLM frames,
  OK/ID/EVT replies) concatenate on the host side and its line reader can't
  split them. Removing the newline "since START/END already delimits it" is
  a trap that silently reintroduces host-side message loss.
- **`MICROBIT_RADIO_MAX_PACKET_SIZE` must be built as 250** (set in
  `codal.json`) to match the RadioRelay's on-air MAXLEN. A mismatch drops
  the relay's larger frames on receive rather than failing loudly.

## 4. Design

**Why two senders instead of one with a flag.** An enum/bool parameter on a
single `send()` would still leave every call site free to pick either policy
for either kind of message; splitting into two named methods makes the
policy part of the call site's own type-level contract (`Comms::sendReply()`
calls `sendReliable()`; the telemetry path calls `send()`), so the
distinction survives refactors instead of depending on someone remembering
to pass the right flag.

**Why RAW250 fragmentation lives here, not in `app/`.** The fragment/ frame
boundary is a property of the physical radio MTU (247 bytes of payload per
frame), which only `Radio` knows about. `app/Comms` only ever sees complete,
reassembled lines — it has no fragment-level concept at all. Reassembly runs
in ISR context (`onData`) because CODAL delivers `MICROBIT_RADIO_EVT_DATAGRAM`
that way; the ISR does the minimum (copy bytes, flip `_msgReady`) and defers
everything else to `poll()` on the main loop.

**Why the channel is stored separately from the group.** The group (10) is a
compile-time constant matching the relay fleet-wide; the channel is the one
per-robot, per-session knob an operator changes to avoid cross-talk between
robots sharing a room. Persisting only the channel (not the group) in flash
keeps the one mutable knob explicit and the fixed constant un-editable by
mistake.

**Why `sendReliable()`'s wait is a spin, not a sleep/yield.** The wait is
sub-millisecond in the normal case (host reading) and capped at 5 ms in the
worst case (host absent) — short enough that a busy-spin is cheaper and
simpler than threading a yield through a leaf transport class that must stay
usable from ISR-adjacent contexts. This mirrors `SerialPort::setBaud()`'s
drain/settle spins, which exist for the same reason (a bounded wait for
hardware to catch up, not a scheduling primitive).

## 5. Interfaces

### Exposes

- **`SerialPort` (USB CDC, 115200 baud default):** `begin()` once before use;
  `readLine(buf, len)` non-blocking, true when a `'\n'`-terminated line is
  ready (newline stripped, NUL-terminated); `send(msg)` async drop-on-full;
  `sendReliable(msg)` bounded-wait, effectively lossless when a reader is
  present; `sendf(fmt, ...)` formats into a 256-byte stack buffer and calls
  `send()`; `setBaud(baud)` drains + settles before retuning — the host must
  change its own baud to match without reopening the port (reopening pulses
  DTR, which resets the board).
- **`Radio` (micro:bit radio, RadioRelay RAW250 framing):** `begin(channel)`
  once before use, enables the radio at group 10; `setChannel(channel)`
  re-tunes at runtime (send any pending reply first — see §3);
  `channel()` reports the active band; `poll(buf, len)` non-blocking, true
  when one reassembled message is ready (NUL-terminated, newline NOT
  stripped); `send(msg)` fragments and transmits, appending `'\n'`.
- **`radiochan::load(storage)` / `save(storage, channel)`:** read/persist the
  channel in the micro:bit's flash-backed key-value store; `load()` falls
  back to `kDefault` (0) when unset or out of range; `save()` clamps to
  `[kMin, kMax]`. `radiochan::toChar(channel)` renders the active channel as
  one base-36 LED-matrix character for the boot display.

### Consumes

- **CODAL / codal-microbit-v2 vendor SDK:** `NRF52Serial`, `MicroBitRadio`,
  `PacketBuffer`, `MicroBitStorage`, `MessageBus`,
  `system_timer_current_time_us()`. See
  `.claude/rules/coding-standards.md`'s "external/vendor function names
  are excluded" clause — vendor names are exempt from project naming
  rules.
- **`App::Transport` (from `app/`):** `com/` is consumed BY `app/comms.h`'s
  `SerialTransport`/`RadioTransport` adapters, not the other way around;
  `com/` has no include on `app/`. See [../app/DESIGN.md](../app/DESIGN.md).

## 6. Open Questions / Known Limitations

- `Radio`'s single-message buffer (§3) has no back-pressure signal to the
  sender — a dropped message is simply invisible on both ends. If a future
  use case needs guaranteed in-order delivery of back-to-back radio
  commands, this buffering model needs revisiting (likely a small ring
  rather than a single slot).
- `SerialPort::sendReliable()`'s 5 ms cap and `setBaud()`'s drain/settle caps
  (20 ms / 4 ms) are empirically chosen, not derived from a documented worst
  case; if TX buffer sizing or baud rate changes, revisit whether the caps
  still hold.
