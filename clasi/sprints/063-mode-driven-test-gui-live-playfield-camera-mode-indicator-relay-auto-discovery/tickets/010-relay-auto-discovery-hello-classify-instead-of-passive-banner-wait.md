---
id: '010'
title: 'Relay auto-discovery: HELLO-classify instead of passive banner wait'
status: done
use-cases: []
depends-on: []
github-issue: ''
issue: testgui-relay-discovery-passive-banner-fails.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Relay auto-discovery: HELLO-classify instead of passive banner wait

## Description

The Test GUI's relay auto-discovery (`transport.py::_relay_probe_banner`)
waited passively for a spontaneous `DEVICE:` boot-banner after opening the
port. Live-verified against the real relay (`/dev/cu.usbmodem2121402`): a
passive open + 2.5 s read sees nothing (this relay does not reset on open,
so no banner is ever emitted), while sending `HELLO\n` gets an immediate
`DEVICE:RADIOBRIDGE:relay:zavaz:4076631795` reply. This is the exact failure
mode documented in `.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md`
(correction of 2026-06-13, sprint 036-007): the robust classification method
is HELLO-classify (send `HELLO`, read the `DEVICE:` reply), which
`SerialConnection` already uses for the real connection handshake. Ticket
063-002's probe had re-introduced the passive-banner assumption, so
`find_relay_port` never found a real relay and the GUI logged "No relay
found on any serial port."

Fixed `_relay_probe_banner` to open the port (default DTR, unchanged),
`reset_input_buffer()`, send `HELLO\n` (re-sent every ~0.4 s within the
probe window in case the device is still mid-boot on the first write), and
read lines until one starts with `DEVICE:` or the deadline passes. Bumped
the default `timeout_s` from 1.2 to 2.0 to give a retry a real chance while
keeping multi-port scans snappy. `find_relay_port` itself is unchanged — it
still just substring-matches `RADIOBRIDGE` in whatever banner the probe
returns, so a robot answering `HELLO` with its own `DEVICE:` banner
continues to be correctly skipped.

## Acceptance Criteria

- [x] `_relay_probe_banner` opens the port with default DTR (unchanged),
      resets the input buffer, sends `HELLO\n`, and reads until a
      `DEVICE:` line arrives or the timeout expires — it no longer waits
      passively for a spontaneous banner.
- [x] `HELLO` is re-sent periodically (~every 0.4 s) within the probe
      window so a device that is still mid-boot on the first write is
      still classified correctly.
- [x] The port is always closed before `_relay_probe_banner` returns, in
      both the success and failure/timeout paths.
- [x] `find_relay_port`'s contract and behavior are unchanged (substring
      match on `RADIOBRIDGE`; a non-relay `DEVICE:` banner — e.g. a robot's
      own banner — causes the port to be skipped).
- [x] The docstrings for `_relay_probe_banner` (module docstring and
      function docstring) explain why the passive boot-banner strategy is
      wrong (no reset ⇒ no banner; even with a reset, boot time can exceed
      a short passive window) and cite
      `.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md`.
- [x] New tests cover: HELLO sent before any banner is returned (fake
      serial only replies after observing the HELLO write); a device that
      never replies causing a `None` return within the timeout; a
      non-relay `DEVICE:` banner reply, with `find_relay_port` confirmed to
      skip that port; an exception on `serial.Serial(...)` open causing
      `None` / the port being skipped; the port closed after both success
      and failure paths.
- [x] Live-verified against the real relay hardware
      (`/dev/cu.usbmodem2121402`): `find_relay_port` locates the relay
      port via the fixed probe.
- [x] `tests/testgui/` and `tests/simulation/` both pass with no
      regressions.

## Testing

- **Existing tests to run**:
  - `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -q`
  - `uv run python -m pytest tests/simulation -q`
- **New tests written**: `tests/testgui/test_relay_discovery.py` —
  `TestRelayProbeBannerHelloClassify`, using a fake `serial.Serial` that
  only replies with a `DEVICE:` banner after observing a `HELLO` write
  (never a spontaneous banner). Covers: relay banner returned + HELLO
  observed; port closed on success; silent device times out to `None` with
  the port closed; a robot's own `DEVICE:` banner is returned by the probe
  but correctly excluded by `find_relay_port` (no `RADIOBRIDGE`); an
  open-time exception (port busy) returns `None` and is skipped by
  `find_relay_port`; HELLO retried when the device is slow to "wake up"
  (simulated mid-boot).
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/test_relay_discovery.py -q`
- **Live verification**: `uv run python -c "from robot_radio.testgui.transport import _relay_probe_banner, find_relay_port, list_ports; ps=list_ports(); print('ports:', ps); print('found:', find_relay_port(ps, _relay_probe_banner))"` → `found: /dev/cu.usbmodem2121402`
