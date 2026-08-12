---
id: '005'
title: Dissolve com/ into Hal::Transport + platform/microbit/, and de-junk main.cpp
status: open
use-cases: ["SUC-002", "SUC-003"]
depends-on: ["004"]
github-issue: ''
issue:
- firmware-layering-cleanup-interfaces-to-hal-impls-to-platform-microbit.md
- main-cpp-holds-code-that-does-not-belong-in-main.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Dissolve com/ into Hal::Transport + platform/microbit/, and de-junk main.cpp

## Description

Phase 3 of the layering-cleanup issue, combined with the `main.cpp`
issue's relocations — both land files in `platform/microbit/`, so they
ride together per sprint.md's Architecture (Design Rationale: "`com/`'s
two transports move into `platform/microbit/`").

**Part A — dissolve `com/`.** Extract `Hal::Transport` into a new
`hal/transport.h`, moved verbatim from `core/comms.h`'s existing
`#ifndef HOST_BUILD` block (readLine/send/sendReliable — the interface
already exists, it just needs to move and lose its build guard). Move the
two concrete pipes into `platform/microbit/`, implementing `Hal::Transport`
directly:

- `com/serial_port.{h,cpp}` → `platform/microbit/microbit_serial_port.
  {h,cpp}`, `Platform::MicroBitSerialPort : public Hal::Transport`.
  `readLine`/`send`/`sendReliable` become `override` — signatures already
  match.
- `com/radio.{h,cpp}` → `platform/microbit/microbit_radio_link.{h,cpp}`,
  `Platform::MicroBitRadioLink : public Hal::Transport`. **Not**
  `MicroBitRadio` — that's CODAL's own global type, held here by
  reference. `poll()` becomes `readLine()` (same signature);
  `sendReliable()` forwards to `send()`, as `Core::RadioTransport` already
  does today.
- `com/banner.{h,cpp}` → `platform/microbit/microbit_banner.{h,cpp}`,
  `Platform::formatBanner()`.

Then delete `com/` (including its `DESIGN.md`), drop the
`#ifndef HOST_BUILD` block from `core/comms.{h,cpp}`, rename
`Core::Transport` → `Hal::Transport` repo-wide, re-base
`TestSupport::FakeTransport`, and simplify `main.cpp` — the two adapter
objects (`Core::SerialTransport`, `Core::RadioTransport`) disappear since
the concrete pipes now implement `Hal::Transport` directly.

**Part B — de-junk `main.cpp`** (`main-cpp-holds-code-that-does-not-
belong-in-main.md`). Three relocations, landing in the same directory
Part A just populated:

1. `showBootIdentity()` (lines 68-124, a CODAL-bound `uBit.display`
   boot-time UI routine) → a new `platform/microbit/` file. `main()`
   still calls it, at the identical point in the boot sequence — identity
   before the buses, `display.disable()` after boot and before the first
   cycle. **This ordering is load-bearing** (the matrix refresh timer
   competes with the control loop's cycle budget) and must survive
   verbatim.
2. `versionTag()` (lines 46-66, pure string logic parsing
   `FIRMWARE_VERSION_STR`) → a host-buildable, testable home. It has never
   had a test because it was ARM-only; give it its first one. Where
   exactly it lands (`types/`, which already holds the version-generation
   seam, or a new small home) is this ticket's own design call — document
   the choice in Completion Notes.
3. The `ID:<drivetrain>:<profile>:<version>` line's `snprintf`
   construction (lines 148-156) → alongside `formatBanner()` in
   `platform/microbit/microbit_banner.{h,cpp}`, matching its sibling
   (which already lives outside `main`).

**Explicitly not in scope**: changing any string content. The
`DEVICE:NEZHA2:...` banner and `ID:...` line are frozen wire format
(`docs/protocol-v5.md`); the boot tag's day+build shape is a 2026-07-29
stakeholder directive. This is a relocation, not a redesign.

**Trap**: exclude `.claude/worktrees/rogo-revival/` from every grep. This
ticket's own explicit-list update must find and fix
`src/firm/platform/host/CMakeLists.txt`'s literal `core/comms.cpp`
reference (confirmed present during planning) plus any other explicit
`com/*.cpp` reference across the ~49-file build/test-source-list set.

## Acceptance Criteria

- [ ] `hal/transport.h` exists, declares `Hal::Transport`, moved verbatim
      from `core/comms.h`'s former `#ifndef HOST_BUILD` block.
- [ ] `platform/microbit/microbit_serial_port.{h,cpp}` exists:
      `Platform::MicroBitSerialPort : public Hal::Transport`, all three
      methods `override`.
- [ ] `platform/microbit/microbit_radio_link.{h,cpp}` exists:
      `Platform::MicroBitRadioLink : public Hal::Transport` (not
      `MicroBitRadio`); `poll()` renamed `readLine()`; `sendReliable()`
      forwards to `send()`.
- [ ] `platform/microbit/microbit_banner.{h,cpp}` exists:
      `Platform::formatBanner()` plus the relocated `ID:` line
      construction.
- [ ] `com/` directory and `com/DESIGN.md` deleted entirely.
- [ ] `Core::Transport` renamed `Hal::Transport` at every call site
      (`TestSupport::FakeTransport`, `Core::Comms`'s two named slots,
      `main.cpp`'s composition); `Core::SerialTransport`/
      `Core::RadioTransport` adapters removed (no longer needed).
- [ ] `showBootIdentity()` relocated to `platform/microbit/`; `main()`
      calls it at the identical boot-sequence point (verified against the
      pre-ticket ordering, not just "still called somewhere").
- [ ] `versionTag()` relocated to a host-buildable home; a new unit test
      covers a normal `major.date.build` string and the `"?"` fallback
      when the string lacks the two required dots.
- [ ] The `ID:` line construction lives in
      `platform/microbit/microbit_banner.*`; `main.cpp` calls it rather
      than building the string inline.
- [ ] No wire-visible string content changed anywhere in this ticket.
- [ ] `just build-sim` and ARM build both clean; `test_layer_isolation.py`
      still passes.
- [ ] Every explicit (non-glob) source list naming `com/*.cpp` or
      `core/comms.cpp` by path updated — `src/firm/platform/host/
      CMakeLists.txt`'s literal `core/comms.cpp` reference (confirmed
      during planning) re-verified and updated, plus a fresh sweep of the
      full ~49-file explicit-list set for any other `com/` reference.

## Testing

- **Existing tests to run**: `just build-sim`, `uv run python -m pytest
  src/tests/sim/unit/test_layer_isolation.py`, ARM build
  (`uv run python3 build.py`).
- **New tests to write**: `versionTag()`'s first-ever unit test (host-side,
  the two cases named above).
- **Verification command**: `just build-sim && uv run python -m pytest
  src/tests/sim/unit/test_layer_isolation.py`. Bench/hardware confirmation
  of the boot-display sequence and banner/`ID:` byte-identity is
  deliberately **deferred to ticket 009** — no full or hardware gate per
  ticket, per the sprint's Test Strategy.
