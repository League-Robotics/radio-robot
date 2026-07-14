---
date: 2026-06-12
tags: [relay, radio, serial, comms, protocol, rogo, robot_radio, bench, documentation, DTR, "!GO"]
related-tickets: ["032-001"]
---

# Talking to the robot through the relay (the `!GO` data plane) + where the docs are

> **CORRECTION (2026-07-14, sprint 102-001, bench-verified — RETRACTS the
> async-STREAM-drop claim in "What Worked" step 4 below).** The claim "async
> STREAM frames are dropped by the bridge" is **RETRACTED** against CURRENT
> relay firmware ("gozop", `!GO` data-plane protocol) and CURRENT robot
> firmware (v0.20260714.2). Measured: binary `STREAM` armed at a 33 ms period
> (~30.3 Hz target), two sustained 240 s captures on the bench rig (robot
> "tovez"/2314287040, relay "zavaz"/4076631795), same period, run
> sequentially:
>   - **Direct USB** (fixed 115200 baud): 6430/6430 delivered (seq-span
>     match), **0.00% drop**, 0 malformed, sustained **26.79 fps**.
>   - **Through the relay** (`!GO` data plane): 6428/6430 delivered,
>     **0.031% drop** — exactly two isolated single-frame gaps (t≈59 s,
>     t≈110 s), classified uniform/sparse, not a burst. Sustained
>     **26.78 fps**. Root cause of the two drops: `source/com/radio.cpp:62-71`'s
>     single-slot RX reassembly mailbox silently drops a new message that
>     completes before the previous one is drained — a real, low-rate, expected
>     loss source, NOT a systematic "bridge drops async pushes" behavior.
>   - The two sustained rates are statistically indistinguishable
>     (26.79 vs 26.78 fps) — the relay is not the bottleneck at this rate;
>     both transports comfortably deliver whatever the firmware actually
>     emits (both landed at ~37.3 ms actual period despite a 33 ms armed
>     period — a firmware emission-pacing characteristic, not a link limit).
>
> **Verdict: PUSH telemetry (`STREAM`) is reliable through the current relay
> firmware.** The P4/P5 ack-ring telemetry design (sprint 103/104) may rely
> on a pushed stream through the relay; it does not need to fall back to
> host-paced `SNAP` polling for the common case — the design's own
> loss/redelivery tolerance already covers the rare single-frame radio-mailbox
> drop. **Recommended common telemetry cadence for BOTH transports: 25 Hz
> (40 ms period)** — the minimum of the two measured sustained rates
> (26.78 fps) with headroom, no baud change (serial stays at the fixed
> 115200 baud per the 2026-07-14 stakeholder decision). Full measurement
> writeup: `clasi/sprints/102-single-loop-firmware-spikes-archive-and-delete-to-stub-p0-p2/spike-001-relay-telemetry.md`.

> **CORRECTION (2026-06-13, sprint 036-007, bench-verified).** The claim below that
> "forcing `dtr=False` actually SUPPRESSES the relay … leaves it mute" is **FALSE**.
> Verified live: opening with `dtr=False` and spamming `HELLO` returns the `DEVICE:`
> banner on the first try, and plain `PING`/`SNAP` return `OK pong` / a full `TLM`
> frame. DTR's only role is **reset** (it resets any micro:bit on the open-time
> transition); you assert DTR to reset a relay into a clean *command plane* so you can
> configure it, not to "enable" comms. The real reason `rogo`/`robot_radio` failed
> with "No device found" was **announcement detection**: with no reset there is no
> boot banner, AND the reader thread dropped `DEVICE:` lines as noise — so the banner
> was never captured even though plain commands worked. Fixed in sprint 036-007:
> `SerialConnection` now HELLO-classifies (capturing the banner *before* the reader
> starts), and for a `RADIOBRIDGE` relay runs `!ECHO OFF` → `!MODE RAW250` → `!GO`,
> then talks plain. Also note: the robot's own USB is NOT always "flash-only/silent" —
> the `tovez` NEZHA2 answers `HELLO`/`PING`/`SNAP`/`VER`/`ID` directly on its USB.
> Radio-link note: relay and robot must share a channel; the robot's channel is
> boot-selected and queried/set via the robot `RF` command (group is fixed).

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
