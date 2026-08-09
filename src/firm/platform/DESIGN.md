# platform/ — board and runtime primitives

`Platform::` is the bottom layer of the firmware: **bus I/O and timing, and
nothing device-specific.** It is what a second compute target has to supply
before any of the rest of `src/firm` can run on it.

## What lives here

```
platform/
  i2c_bus.h     Platform::I2CBus     — pure interface
  clock.h       Platform::Clock / Platform::Sleeper — pure interfaces
  microbit/     the micro:bit (nRF52833/CODAL) implementations
  host/         the HOST platform: the sim (was src/sim/)
```

`microbit/` holds `MicroBitI2CBus`, `MicroBitClock`, `MicroBitSleeper` —
the only files in this tree that include `MicroBit.h`.

`host/` holds `TestSim::SimPlant` (an `I2CBus` that answers wire
transactions out of a physics model), `TestSim::SimClock`/`SimSleeper`, the
`sim_ctypes.cpp` C ABI, and `sim_harness.h`. It is a platform in exactly
the sense this directory means: **real firmware, substituted primitives.**
`Core::composeRobot()` is called identically by `src/firm/main.cpp` and by
`TestSim::SimHarness`; the only difference between them is which
`Platform::I2CBus`/`Platform::Clock`/`Platform::Sleeper` the caller
constructs and passes in.

## Rules

1. **Platform knows nothing about Hardware.** No `#include` of a device
   leaf, no knowledge of what is on the bus. `Platform::I2CBus` deals in
   addresses and byte buffers.
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
