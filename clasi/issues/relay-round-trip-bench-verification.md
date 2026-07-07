---
status: pending
---

# Bench-verify the command surface over the radio-relay round-trip (not just direct serial)

## Context

Sprint 088's on-stand bench verification (ticket 088-009,
`tests/bench/motion_command_verify.py`) proved the full command surface functions
on real hardware — announcement, liveness, config, and motion verbs via encoders —
but **over direct USB serial only**. The relay round-trip (host → relay `!GO` data
plane → radio → robot) was deferred, not a stand limitation.

## Ask

Run the same functional pass over the radio relay and confirm round-trip:

- `uv run python tests/bench/motion_command_verify.py --port /dev/cu.usbmodem2121402`
  (the relay; `SerialConnection` auto-detects `mode=relay` and does the `!GO`
  handshake). Confirm the straight-drive verbs (D/T/S) and at least one more
  (RT) round-trip and drive the wheels with encoders responding.
- Confirm the `HELLO` re-announce and boot `DEVICE:` banner reach the host over the
  relay path (the announcement design specifically supports relay rediscovery).
- Expect radio-specific flakiness (the relay drops async / caps throughput ~12
  msg/s) — the tool's retry logic should absorb it; widen tolerances if needed.

## Acceptance

- Motion verbs (at least D/T/S/RT) verified functional over the relay with encoder
  response; HELLO banner received over the relay. Result appended to sprint 088's
  `bench-verification-log.md` or a fresh relay log.
