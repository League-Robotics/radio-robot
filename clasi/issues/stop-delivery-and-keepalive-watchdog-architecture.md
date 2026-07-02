---
status: pending
review: docs/code_review/2026-07-01-full-codebase-review.md
findings: CR-04, CR-05
severity: high
sprint: '065'
---

# STOP delivery is unreliable and the ambient keepalive defeats the motion watchdog

## Problem

**(a) Dropped STOP = unbounded manual-drive runaway (CR-04).** The TestGUI
KeyboardDriver drives with open-ended `VW` (no TIME stop by design) and on
key release sends `STOP` **once, fire-and-forget**
([drive.py:279-288](../../host/robot_radio/testgui/drive.py)) — no ack, no
retry. Direct USB intermittently drops 15–50 % of lines. If the STOP line is
dropped, the VW resend timer stops but the robot keeps driving at the last
commanded velocity **indefinitely**, because of (b). Window focus loss
suppresses the keyRelease event entirely, with the same result.

**(b) The watchdog only catches host-process death (CR-05).**
`SerialConnection` starts a keepalive daemon on connect that streams `+`
every ~150 ms whenever the port is open
([serial_conn.py:682-711](../../host/robot_radio/io/serial_conn.py)), and the
firmware watchdog resets on **any** inbound line
([Superstructure.cpp:122-158](../../source/superstructure/Superstructure.cpp)).
So every host program (CLI, TestGUI, bench scripts) silently disables the
last safety layer for open-ended motion (`S`/`VW`/`R`): a hung host script
with an open port = a robot that never stops. This is the same
"watchdog silenced by keepalives" mechanism from the June wild-spin
postmortem, now structural.

## Fix direction (layered; pick at least two)

- TestGUI: send STOP via the acked `command()` path and re-send until `OK`
  (STOP is idempotent); and/or turn the resend timer into a deadman that
  sends STOP for the next N ticks after key release instead of stopping.
- Make motion keepalive intentional, not ambient: firmware resets the
  *motion* watchdog only on `+` or motion commands, and the host sends `+`
  only while a motion source is actively driving (keepalive armed/disarmed by
  the layer that owns motion, not by `connect()`).
- Consider a firmware max-staleness cap on `VW`: auto soft-stop after N ms
  without a *fresh* VW, independent of `+`.

## Acceptance

- Sim/bench test: start VW, simulate a dropped STOP (suppress one send) —
  the robot must stop within a bounded time anyway.
- A hung host process holding the port open must no longer keep an
  open-ended motion alive past the watchdog window.
