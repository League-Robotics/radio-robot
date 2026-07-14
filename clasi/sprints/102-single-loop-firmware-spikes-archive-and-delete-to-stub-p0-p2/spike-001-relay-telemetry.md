---
ticket: '001'
status: done
---

# Spike 001 results: relay sustained-push telemetry

## What was measured

Binary `STREAM` telemetry armed at a **33 ms period (~30.3 Hz target)** against
**current firmware** (robot v0.20260714.2, tovez/NEZHA2, serial 2314287040),
CURRENT relay firmware ("gozop", relay "zavaz", serial 4076631795), no code
changes anywhere in the stack. Two sustained 240 s captures, same period, same
robot, same bench rig (wheels off the ground, motors neutralized throughout —
no motion commands were issued):

1. **Direct USB**, `/dev/cu.usbmodem2121102`, fixed 115200 baud (no baud
   switching, per the 2026-07-14 stakeholder decision dropping the baud-ceiling
   spike).
2. **Through the radio relay's `!GO` data plane**, `/dev/cu.usbmodem2121302`.
   `SerialConnection.connect()` performed the documented handshake
   automatically (DTR-asserted open → `DEVICE:RADIOBRIDGE:relay:zavaz:...`
   banner → `!ECHO OFF` → `!MODE RAW250` → `!GO` → `# entering data plane`),
   confirmed in the connect-info logged by the capture (`relay_info.
   entered_data_plane: true`, `relay_config: channel 0 group 10 mode RAW250
   power 7`).

Tool: `tests/bench/relay_telemetry_rate.py` (new, committed this ticket).
Method: arm `STREAM`, poll `NezhaProtocol.read_pending_binary_tlm_frames()`
every 1 s (well inside the 256-frame drop-oldest host queue's ~8.5 s buffer at
this rate), track the D10 `seq` counter for gap/drop accounting
(`tlm_drop_rate()`, uint16-wrap-safe), classify gap bursts vs. uniform sparse
loss, and count `*B` frame base64/protobuf decode failures via a script-side
(non-production) wrap of `SerialConnection._handle_binary_reply`. A sanity
`PING` was issued immediately after `connect()` on each path before arming
the stream (both succeeded), and each `connect()` was allowed to complete its
own internal HELLO-classify + readiness poll (which pulses DTR and resets the
micro:bit on open) before the script's own explicit ping — i.e. the stream was
armed only after the reset-on-open settled and round-trip commands were
already confirmed working.

Direct-USB session first, then the relay session, run sequentially (not
concurrently) to avoid two live command sessions confounding a single robot's
telemetry state.

## Results

| Path | Armed period | Delivered | Expected (seq span) | Drop rate | Longest gap | Malformed | Sustained rate |
|---|---|---|---|---|---|---|---|
| Direct USB (115200) | 33 ms (~30.3 Hz) | 6430 | 6430 | **0.00%** | 0 | 0 | **26.79 fps** |
| Relay (`!GO` data plane) | 33 ms (~30.3 Hz) | 6428 | 6430 | **0.031%** | 1 | 0 | **26.78 fps** |

Relay gap detail: exactly two isolated single-frame gaps in the full 240 s /
6430-frame-expected window, at t≈59.0 s and t≈110.0 s — classified
**uniform/sparse** (2 missing frames in 2 of 240 one-second windows, never
more than 1 in any window). This is not a burst and not a sustained-loss
pattern.

Both transports converge on the same actual delivered period (~37.3 ms,
≈26.8 Hz) despite being armed for a 33 ms (~30.3 Hz) period — direct USB
delivered its own emitted frames at **0% loss**, so the 30.3→26.8 Hz shortfall
is a **firmware telemetry-emission pacing characteristic** (the main loop
producing frames slightly slower than the armed STREAM period asks for), not
a transport ceiling on either path. Neither USB at 115200 nor the relay's
current radio link is the bottleneck at this rate — both comfortably deliver
whatever the firmware actually emits, with the relay only 0.03 points worse
than direct USB's perfect delivery.

## Interpretation: the two relay drops

`source/com/radio.cpp:62-71` implements a **single-slot RX reassembly
mailbox**: `if (self->_reasmActive && !self->_msgReady) { publish }` — if a
new message's `FLAG_END` arrives while the previous reassembled message is
still unconsumed (`_msgReady` still true), the new message is silently
dropped, not queued. Two isolated single-frame drops spread ~51s apart over a
6430-frame, 240s capture is exactly the loss signature this mechanism
predicts under normal jitter (an occasional double-message-arrives-before-
host-drains-previous race), not a bulk "bridge drops async pushes" failure
mode. This matches the "uniform/sparse" classification, not "burst."

## Verdict: confirm or retract the standing knowledge-note claim

**RETRACTED.** The 2026-06-12 knowledge note (`.clasi/knowledge/
2026-06-12-relay-go-data-plane-and-docs.md`, "What Worked" step 4) states
"async STREAM frames are dropped by the bridge" as the reason to prefer
polling `SNAP`. Against CURRENT relay firmware ("gozop") and CURRENT robot
firmware (v0.20260714.2), this is false: pushed `STREAM` frames survive the
relay's `!GO` data plane at **99.97% delivery** over a sustained 4-minute
window — not materially different from direct USB's 100%. The knowledge note
is updated in this commit with a dated 2026-07-14 correction block (the
document's established pattern — corrections are appended, not silently
rewritten, to preserve the debugging history) pointing at this file.

## Push vs. host-paced-poll recommendation for P4/P5 (sprint 103/104)

**Use PUSH (STREAM), not host-paced SNAP polling, as the return-path
strategy for the ack-ring telemetry design, on both transports.** The
measured loss rate (0.03% on the relay, 0% on direct USB) is well within what
an ack-ring design should already tolerate for re-delivery/gap-fill
robustness — no fallback to polling is needed for the common case. The
single-slot radio mailbox drop (radio.cpp:62-71) is a real, expected,
low-rate loss source the ack-ring's tolerance should continue to cover; it is
not a reason to poll instead of push.

## The three rate-setting numbers (ticket's required verdict)

1. **Relay-sustained rate: 26.78 fps** (0.031% drop over 240 s / 6430 expected
   frames).
2. **Direct-USB-sustained rate (fixed 115200 baud): 26.79 fps** (0.00% drop
   over 240 s / 6430 expected frames).
3. **Recommended common cadence for BOTH transports: 25 Hz (40 ms period)**
   — the minimum of the two measured sustained rates (26.78 fps) rounded down
   to a clean number with ~7% headroom below the demonstrated ceiling. This is
   *below* the 33 ms/~30.3 Hz period armed during this spike (which itself
   already ran at only ~26.8 fps actual due to firmware pacing, not link
   limits) — 40 ms gives the firmware's own emission loop more margin to hit
   the target consistently, on both transports, with no baud change anywhere
   in the stack (serial stays at the fixed 115200 baud, per the 2026-07-14
   stakeholder decision). This is the rate budget P4 (wire protocol) and P5
   (host) must design to.

## Open finding for sprint 103 (not itself an acceptance item — flagging only)

The ~30.3 Hz armed vs. ~26.8 Hz actual gap (an extra ~4.3ms per frame, firmware
- side) was not root-caused by this spike (out of scope — no firmware changes
permitted). Sprint 103's new single-loop main() should re-measure its own
telemetry-emission pacing once the loop is rewritten; this spike's numbers are
a pre-rewrite baseline, not a promise about the new loop's behavior.

## Hardware bench gate

Per `.claude/rules/hardware-bench-testing.md`: robot mounted on the stand,
wheels off the ground, motors neutralized throughout — no motion commands
were issued, only `PING`/`STREAM` telemetry arm/disarm. Confirmed alive
before and after both captures (`PING` → `OK pong`, `HELLO` →
`DEVICE:NEZHA2:robot:tovez:2314287040` on the robot's own USB; relay
confirmed as `DEVICE:RADIOBRIDGE:relay:zavaz:4076631795` with `ROLE` verified
before use, distinct from the robot's own USB port).

## No firmware or production host code modified

Confirmed: only `tests/bench/relay_telemetry_rate.py` (new file),
`.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md`, this results
note, and the ticket's own frontmatter/checklist changed on this branch for
ticket 001. No `source/` or `host/` production file touched.
