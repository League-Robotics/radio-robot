---
id: '005'
title: 'Device announcement: boot banner and HELLO verb'
status: open
use-cases: [SUC-004]
depends-on: ['003']
github-issue: ''
issue: robot-device-announcement-on-connect-and-hello.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Device announcement: boot banner and HELLO verb

## Description

The firmware emits no connect-time identity banner. The retired firmware
did this over serial (`DEVICE:NEZHA2:robot:<name>:<serial>`,
`source_old/commands/SystemCommands.cpp:84-94`), and the Python host still
parses and caches this exact line (`serial_conn.py`, `connection.py`,
`testgui/transport.py`, `config/devices.json`). Stakeholder decision
(2026-07-07): field 1 stays `NEZHA2` (matches `ID` and the host's cache),
field 2 is `robot` (mirrors the relay's `relay`). Emit the banner as the
first line on both serial and radio at boot, and re-emit it on a re-added
`HELLO` verb, on the channel it arrived on.

## Implementation Plan

**Approach**: Add a shared free function
`int formatDeviceAnnouncement(char* buf, int size)` to
`source/commands/system_commands.{h,cpp}`, using the same
`#ifdef HOST_BUILD` identity-source branch `handleId` already uses
(`HOST-SIM`/`0` vs `microbit_friendly_name()`/`microbit_serial_number()`),
formatting `DEVICE:NEZHA2:robot:<name>:<serial>`. Add a new `handleHello`
handler in the bare-reply style `handleId` already uses
(`replyFn(banner, replyCtx)`, no `OK`/`ERR` wrapper — `DEVICE:` is its own
reply taxonomy, like `ID`), and register `HELLO` in `systemCommands()`
(`handlerCtx = nullptr`, same as `PING`/`VER`/`ECHO`/`ID` — `HELLO` does
not need `CommandRouter` access, it uses the normal
`replyFn`/`replyCtx` mechanism already resolved to "the channel this
command arrived on"). In `main.cpp`, immediately after `comm.begin()`
(currently line ~106), format the banner into a stack buffer and call
`comm.sendSerial(banner)` then `comm.sendRadio(banner)`.

**Files to create/modify**: `source/commands/system_commands.{h,cpp}` (new
helper + `HELLO` handler + registration), `source/main.cpp` (boot
announcement call), `docs/protocol-v2.md` (re-add `HELLO` and document
the boot `DEVICE:` announcement — both currently listed as removed under
v2).

**Testing plan**: a sim test sending `HELLO` and asserting the
`DEVICE:NEZHA2:robot:...` reply shape (correct prefix/role tokens,
name/serial fields present). HITL bench check: banner on serial at
connect, `HELLO` re-request returns the banner over the radio/relay path.

**Documentation updates**: `docs/protocol-v2.md` only.
`docs/architecture.md`'s stale `Announcer`-class reference is explicitly
NOT touched here (out of scope, deferred to a future
`consolidate-architecture` pass).

## Acceptance Criteria

- [ ] `DEVICE:NEZHA2:robot:<name>:<serial>` is the first line out on
      serial at boot.
- [ ] `DEVICE:NEZHA2:robot:<name>:<serial>` is the first line out on radio
      at boot (radio is fire-and-forget — a missed boot radio banner
      because no relay was listening yet is not a failure).
- [ ] `HELLO` re-emits the same banner on the channel it arrived on.
- [ ] `<name>`/`<serial>` are exactly `microbit_friendly_name()`/
      `microbit_serial_number()` — the same pair `ID` already uses.
- [ ] The host's existing parsers (`serial_conn.py`, `connection.py`,
      `testgui/transport.py`) classify the robot as a direct/robot device
      from the banner unchanged — verify by inspection; no host-side code
      change should be needed.
- [ ] `docs/protocol-v2.md` re-adds `HELLO` and documents the boot
      announcement (both currently listed as removed under v2).
- [ ] HITL bench: banner appears on serial at connect; over the
      radio/relay path a `HELLO` re-request returns the banner.

## Testing

- **Existing tests to run**: `tests/sim/unit/test_protocol_roundtrips.py`
  (or wherever the liveness family is tested) plus the full suite.
- **New tests to write**: a sim test sending `HELLO` and asserting the
  `DEVICE:NEZHA2:robot:...` reply shape.
- **Verification command**: `uv run python -m pytest`; bench spot-check
  (serial boot banner, `HELLO` over the radio relay).
