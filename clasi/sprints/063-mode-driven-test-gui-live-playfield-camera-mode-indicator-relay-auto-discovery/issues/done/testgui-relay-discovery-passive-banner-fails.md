---
status: done
sprint: '063'
tickets:
- 063-010
---

# TestGUI: Relay auto-discovery fails — passive banner probe never sees the relay

## Symptom

With a relay plugged in (`/dev/cu.usbmodem2121402`, nothing holding the port),
clicking Connect in Relay mode repeatedly logs:

```
[INFO] Relay: scanning serial ports for relay...
[WARN] No relay found on any serial port
```

## Root cause (live-verified 2026-07-01)

`transport.py::_relay_probe_banner` is **purely passive**: it opens the port
and waits up to 1.2 s for a spontaneous `DEVICE:` boot banner, relying on the
open-time DTR pulse resetting the relay.

Live probe against the real relay:
- Passive open + 2.5 s read → **nothing** (open does not reset this relay; no
  boot banner is ever emitted).
- Sending `HELLO\n` → immediate `DEVICE:RADIOBRIDGE:relay:zavaz:4076631795`.

This is the exact failure mode documented in
`.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md` (correction of
2026-06-13, sprint 036-007): *"with no reset there is no boot banner"* — the
robust classification method is **HELLO-classify** (send `HELLO`, the relay
answers with its `DEVICE:` banner), which is what `SerialConnection` already
does. Ticket 063-002's probe re-introduced the passive-banner assumption.

Even when the DTR reset does occur, a micro:bit takes longer than 1.2 s to
boot, so the passive wait is also too short — the passive strategy is wrong on
both counts.

## Fix

In `host/robot_radio/testgui/transport.py::_relay_probe_banner`:
- After opening the port (default DTR, unchanged), **send `HELLO\n`** (flush),
  then read lines until a `DEVICE:` line arrives or the timeout expires.
- Re-send `HELLO` once or twice within the window (e.g. every ~0.4 s) in case
  the first write lands while the device is mid-boot.
- Keep everything else: passive-first is unnecessary; `find_relay_port`'s
  contract (`RADIOBRIDGE` substring match) is unchanged; the port is always
  closed before returning.
- `HELLO` is safe to send to a non-relay device (the robot's own USB answers
  `HELLO` with its own `DEVICE:` banner, which won't contain `RADIOBRIDGE`, so
  classification still works).

## Affected code

- `host/robot_radio/testgui/transport.py` — `_relay_probe_banner`.
- `tests/testgui/test_relay_discovery.py` — update/extend probe tests (the
  probe is injectable in `find_relay_port`, and the probe itself can be tested
  with a fake `serial.Serial`).
