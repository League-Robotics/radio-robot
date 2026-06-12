---
date: 2026-06-12
tags: [relay, radio, serial, comms, protocol, rogo, robot_radio, bench, documentation, DTR, "!GO"]
related-tickets: ["032-001"]
---

# Talking to the robot through the relay (the `!GO` data plane) + where the docs are

## Where the documentation is (read this FIRST)

**All project documentation is at https://robots.jointheleague.org/.** For ANY
unknown about the hardware, the relay, the radio link, the wire protocol, firmware
commands, calibration, or the host tooling — WebFetch the docs site FIRST, before
reverse-engineering behavior or writing diagnostic code. (This doc exists partly
because that step was skipped and ~2 hours were burned re-deriving the documented
relay protocol below.)

## Problem

A host program could not talk to the robot over the radio relay. `rogo` and the
`robot_radio` library failed with `Error: No device found on
/dev/cu.usbmodem...  Is it powered on?` even though the robot was powered, flashed,
and fine.

## Symptoms

- `rogo opos` / `rogo enc` / `rogo send PING` → "No device found. Is it powered on?"
- Library `SerialConnection.connect()` sometimes reported `pinged=True` once, but
  every subsequent `send()` returned no reply (the corr-id reply queue never filled).
- Raw `PING\n` or `>PING\n` to the relay returned zero bytes.
- Robot micro:bit LED showed a **static heart** (this is the firmware's
  booted-and-running indicator, NOT a crash — a CODAL panic shows a sad face /
  scrolling error code).

## What Was Tried (and why it failed)

- One-shot `rogo` calls — each tears down/re-opens the port; preflight PING never
  got a reply because the relay was never put into its data plane.
- Hunting a macOS `Resource busy` port conflict — real but secondary (VS Code's
  serial-monitor plugin, `Code Helper (Plugin)` PID, held the port; found via
  `lsof -n | grep usbmodem`). Freeing it did not fix comms.
- Raw pyserial probes with `dtr=False`/HUPCL-disabled — got total silence;
  forcing `dtr=False` actually SUPPRESSES the relay.
- Reflashing / battery power-cycles / relay replug — none fixed it, because the
  firmware and hardware were never the problem.

## What Worked

Talk to the **relay** serial port (`/dev/cu.usbmodem...`; the robot's own USB is
flash-only/silent) using its control-plane → data-plane protocol:

1. Open with **DTR asserted** — pyserial DEFAULT: `serial.Serial(port, 115200,
   timeout=...)`. Do NOT set `dtr=False`/`rts=False`. Opening pulses DTR, resetting
   the relay; it announces `DEVICE:RADIOBRIDGE:relay:gozop:<id>`. Wait ~1 s.
2. Send **`!GO\n`** → relay replies `# entering data plane`.
3. Send **plain** commands, **no `>` prefix**: `HELLO` → `DEVICE:NEZHA2:robot:tovez:<id>`;
   `PING` → `OK pong t=<uptime_ms>`; `SNAP` → a full `TLM ...` frame.
4. Hold ONE connection open for the whole program. Telemetry: poll **SNAP**
   (request/reply, reliable); async **STREAM** frames are dropped by the bridge.

TLM units: `pose=x_mm,y_mm,h_centideg`, `twist=v_mmps,omega_mrad/s`, `enc=L_mm,R_mm`.
Working reference implementation: `tests/bench/bench_validation_032.py`.

## Why It Works

The current relay firmware ("gozop") has a **control plane** and a **data plane**.
On open it sits in the control plane and only forwards robot traffic after `!GO`.
In the data plane the relay is transparent and commands go to the robot verbatim
(no `>` prefix). DTR-asserted-on-open is what triggers the relay's reset+announce
handshake; suppressing DTR leaves it mute.

## Future Guidance

- **`rogo` / `robot_radio` are out of date vs the relay.** `SerialConnection`'s
  relay mode prefixes commands with `>` and never sends `!GO`, so it never enters
  the data plane and silently fails against the current relay firmware. Until the
  host library is updated to the `!GO` protocol, do bench comms with the raw
  `!GO`+plain-command pattern above. Reconciling `robot_radio` with the documented
  protocol is a real follow-up.
- For any comms/hardware unknown, **read https://robots.jointheleague.org/ first.**
- A static heart on the robot LED = alive; don't mistake it for a crash.
