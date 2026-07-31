---
status: done
sprint: '128'
tickets:
- 128-005
---

# testgui arrow-key drive is fully dead in both modes — port to the Move surface or delete it

**Source:** code review 2026-07-30, `05-testgui-testkit.md` CRITICAL §2.
**Priority:** P1 — no safety exposure (nothing moves at all), but it is ~500
lines of convincing-looking dead machinery that costs every reader who tries
to understand the GUI's motion paths.
**Goal served:** dead code that *looks* alive (deadman timers, keepalive
bookkeeping, drop-rate math) is the most expensive kind to audit. Deleting or
porting it removes a whole false lead from future bug hunts.

## What is wrong

`testgui/drive.py` (`KeyboardDriver`) drives the `DEV DT VW`/`DEV DT STOP`
wire family. On hardware that falls into the dead
`binary_bridge.translate_command()` stub; in Sim, `SimTransport._dispatch()`
has no `DEV` branch and logs "not supported in this sim". Pressing an arrow
key moves nothing, in either mode, and the 100 ms keepalive + five-resend
STOP deadman all run to completion around commands that go nowhere.

## What to do — decide, then do one of these

**Option A (port — if arrow-key teleop is still a wanted workflow).** Rebuild
`KeyboardDriver` on the live bounded-Move surface. The deadman problem the
old `DEV` family solved is now solved *by the protocol itself*: every Move
carries a mandatory timeout, so a keepalive is just a short re-issued Move
and a lost host stops the robot by construction.

```python
# On key-state change or 100ms keepalive tick, while any arrow is held:
#   short bounded move; replace=True preempts the previous one. If the host
#   dies, the 300ms timeout expires and the robot stops -- the deadman is
#   now the protocol's own bounded-Move contract, not a resend ritual.
transport.protocol.move_wheels(
    v_left, v_right,
    stop_time=250,      # [ms]
    timeout=300,        # [ms] deadman
    replace=True,
)
# On all-keys-released: transport.halt()   (see the Transport.halt() issue)
```

For Sim parity, `SimTransport` needs the same call shape via `SimLoop.move()`
— no `DEV` strings anywhere.

**Option B (delete — if teleop-by-arrow-key is not a supported workflow).**
Delete `testgui/drive.py`, its `attach()` call site, and the key-event
plumbing in `__main__.py`. The gamepad/preset-button paths remain the teleop
story.

Stakeholder pick required between A and B; do not leave the file as-is.

## Acceptance

- `grep -rn "DEV DT" src/host/robot_radio/` returns nothing (either option).
- Option A: headless button-acceptance test proving a key-press produces a
  `move_wheels` call with `replace=True` and a bounded `timeout`, and that
  key-release produces a halt; then a bench check on the stand — hold Up,
  wheels spin; release, wheels stop; yank the USB cable mid-hold, wheels
  stop within the Move timeout.
- Option B: the file and every reference are gone; the GUI builds and the
  existing acceptance suite passes.
