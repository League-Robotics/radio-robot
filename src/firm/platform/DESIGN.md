# platform/ — board and runtime primitives

`Platform::` is the bottom layer of the firmware: **bus I/O and timing, and
nothing device-specific.** It is what a second compute target has to supply
before any of the rest of `src/firm` can run on it.

## What lives here

```
platform/
  microbit/     the micro:bit (nRF52833/CODAL) implementations
  host/         the HOST platform: the sim (was src/sim/)
```

`microbit/` holds `MicroBitI2CBus`, `MicroBitClock`, `MicroBitSleeper` —
the ARM implementations of `Hal::I2CBus`/`Hal::Clock`/`Hal::Sleeper` — plus
(136-005, "dissolve `com/` into `Hal::Transport` + `platform/microbit/`")
the two `Hal::Transport` implementations and the boot-identity/banner
helpers:

```
microbit/
  microbit_i2c_bus.{h,cpp}      Platform::MicroBitI2CBus     — Hal::I2CBus
  microbit_clock.{h,cpp}        Platform::MicroBitClock/MicroBitSleeper
  microbit_serial_port.{h,cpp}  Platform::MicroBitSerialPort — Hal::Transport,
                                 USB CDC serial (115200 baud)
  microbit_radio_link.{h,cpp}   Platform::MicroBitRadioLink  — Hal::Transport,
                                 the micro:bit radio (RadioRelay RAW250 framing)
  microbit_banner.{h,cpp}       Platform::formatBanner()/formatIdLine() —
                                 the two frozen wire-identity strings
  microbit_boot_identity.{h,cpp} Platform::showBootIdentity() — LED-matrix
                                 boot display (relocated out of main.cpp)
```

This is the only subtree in `src/firm` (besides `main.cpp` itself) that
includes `MicroBit.h` — isolating every CODAL-touching file here is what
keeps the rest of the tree `HOST_BUILD`-clean (`docs/design/design.md` §5,
"HOST_BUILD purity"). `com/` used to hold the two transports separately
(a directory of its own, ARM-only, with no `Hal::` interface underneath
it); dissolving it here means `Platform::MicroBitSerialPort`/
`MicroBitRadioLink` implement `Hal::Transport` directly instead of routing
through a `Core::SerialTransport`/`RadioTransport` adapter pair, the same
shape every other `Platform::` leaf already had for `Hal::I2CBus`/
`Hal::Clock`/`Hal::Sleeper`.

`host/` holds `TestSim::SimPlant` (an `I2CBus` that answers wire
transactions out of a physics model), `TestSim::SimClock`/`SimSleeper`, the
`sim_ctypes.cpp` C ABI, and `sim_harness.h`. It is a platform in exactly
the sense this directory means: **real firmware, substituted primitives.**
`Core::composeRobot()` is called identically by `src/firm/main.cpp` and by
`TestSim::SimHarness`; the only difference between them is which
`Hal::I2CBus`/`Hal::Clock`/`Hal::Sleeper` the caller constructs and passes
in — `TestSim::SimHarness` passes `TestSupport::FakeTransport` (an
in-memory `Hal::Transport` double) where `main.cpp` passes the two real
`Platform::MicroBit*` transports above.

## Transport invariants (folded in from the retired `com/DESIGN.md`)

- **Both transports are binary-clean, uniformly `\n`-terminated.**
  `readLine()`/`send()`/`sendReliable()` never distinguish text from binary
  at this layer — `Core::Comms` decides that one layer up, from the parsed
  `<COMMAND>` prefix. This is safe because COBS is keyed on 0x0A
  (`wire_runtime.h` item 8): a binary line's own bytes never contain a
  literal 0x0A, so `\n` is a genuine, unconditional terminator in both
  directions.
- **`send()` vs `sendReliable()` is a drop-policy split by call-site
  cadence, not by content kind.** `send()` is ASYNC/drop-on-full (the
  telemetry flood, every binary reply) — a lost frame is harmless, a
  stalled loop is not. `sendReliable()` bounded-waits (5 ms cap on serial)
  — used only for the rare HELLO/PING/ID/VER cleartext replies, where
  silently dropping would defeat the safety rump's purpose.
  `MicroBitRadioLink::sendReliable()` simply forwards to `send()` — RAW250
  fragmentation has no separate bounded-wait path to offer.
  Neither transport ever blocks unboundedly or sleeps.
- **Radio group is fixed at 10; only the channel (frequency band, 0–35) is
  configurable**, baked from the robot JSON's `connection.radio_channel`
  (`Config::kRadioChannel`, `config/boot_config.h`) — no wire verb retunes
  it live. Group must match the RadioRelay's fixed group or the link never
  forms regardless of channel.
- **Re-tuning the radio channel drops the link immediately** — send any
  reply BEFORE calling `MicroBitRadioLink::setChannel()`.
- **Only one `MicroBitRadioLink` instance may call `begin()`** — `_instance`
  is a static singleton pointer the static ISR callback (`onData`)
  dereferences.
- **Radio buffers exactly one completed message** between the ISR and
  `readLine()` — a second message finishing reassembly before `readLine()`
  drains the first is dropped, not queued.
- **`MicroBitSerialPort`'s CODAL TX buffer size is a `uint8_t`** — 255 is
  the actual max (a requested 1024 silently wraps to 0, i.e. no buffer at
  all). Do not "simplify" `begin()` by requesting a larger buffer size.
- **`MICROBIT_RADIO_MAX_PACKET_SIZE` must be built as 250** (`codal.json`)
  to match the RadioRelay's on-air MAXLEN.

## Rules

1. **Platform knows nothing about Hardware.** No `#include` of a device
   leaf, no knowledge of what is on the bus. `Hal::I2CBus` (implemented
   here by `Platform::MicroBitI2CBus`) deals in addresses and byte
   buffers.
2. **A platform-intrinsic *device*** — hardware physically part of one
   compute board and unable to exist anywhere else, such as the micro:bit's
   onboard compass — belongs in `platform/<target>/hardware/`, not here and
   not in `hardware/generic/`. Nothing occupies that slot today.
3. **`platform/host/` is excluded from the ARM firmware build.** It lives
   under `src/firm` so the whole firmware travels together on a `git
   subtree split`, which puts it inside the root `CMakeLists.txt`'s
   recursive source glob — a `list(FILTER ... EXCLUDE REGEX
   "/platform/host/")` there is what keeps host-only C++ (and any nested
   CMake build directory) out of the image. Removing that filter breaks the
   ARM build, loudly.

## New surface not yet designed

UART, GPIO, and analog/digital pin interfaces have no counterpart here yet
— every device in the tree today is I2C. They should be shaped by the first
port that actually needs them (a Raspberry Pi target, or the WiFi module's
AT-command UART), not designed speculatively now.
