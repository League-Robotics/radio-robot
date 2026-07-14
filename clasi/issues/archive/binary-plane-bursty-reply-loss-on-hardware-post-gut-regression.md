---
status: obsolete
---

> **OBSOLETE (2026-07-14 stakeholder triage).** Superseded by the single-loop
> firmware rebuild (`clasi/issues/single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md`;
> review: `docs/code_review/2026-07-13-devices-drive-review.md`). Per-command replies are eliminated: ACKs/NACKs ride in the always-on ~30Hz telemetry frame's ack ring, repeated across frames, so single-frame loss cannot lose an ack — the loss-recovery design this issue asked for. The P0 relay/rate spikes measure stream delivery directly.

# Binary command plane: bursty reply loss on hardware (post-gut regression candidate)

## Evidence (2026-07-10 team-lead bench session, robot tovez on stand, direct USB serial, no relay)

On post-gut firmware **v0.20260710.5**, binary `CommandEnvelope` round trips
fail in **bursts** while the text rump stays healthy **on the same boot**:

- 60-ping binary burst (`send_envelope`, `read_timeout=400`): **16/60 lost
  (27%)**, including **8 consecutive** losses (indexes 22–29); firmware uptime
  monotonic across the whole burst — **no reboots**.
- 60 raw text `PING`s via `send_fast` (no corr-id, no retry) on the same boot:
  **1/60 lost (1.7%)** — the known pre-existing baseline.
- Loss is **bimodal across sessions**: some windows are 30/30 clean; others go
  bursty-deaf for hundreds of ms. Observed at idle, under binary streaming
  (28/30), and worst just after stream disarm (23/30).

Host layers are exonerated:

- The rogo proxy's TX envelopes were captured and decoded byte-correct
  (drive/segment/config/get/stream all match known-good direct sends).
- In-process `ProtocolBridge` handlers complete in <10 ms whenever the link
  cooperates (full GET fan-out 48 ms; SET/S/STOP/D all single-digit ms).

## Impact

The binary-only wire surface (everything post-097 except the PING/HELLO/STOP
text rump) is unreliable on real hardware:

- Motion acks vanish → `rogo proxy` renders `ERR unknown timeout` while the
  segment still executes; EVT synthesis never arms.
- `send_envelope` is single-attempt **by design** (no ERR-unknown retry —
  `serial_conn.py` docstring), so there is no loss recovery anywhere in the
  binary stack.
- The 097-004 bench-gate checkbox cannot fully pass, and **ticket 097-010's
  closure claims (bench gate / robot-drivable-at-every-commit) are blocked**
  until this is resolved or explicitly waived.

## Suspects for the firmware debugger

1. BinaryChannel reply encode/armor path sharing scratch buffers with the new
   pure-binary `tickTelemetry()` (097-008, commit `bce277c7`) — TX-side
   contention when telemetry emission overlaps a command reply.
2. Loop-yield/timing changes from the 006/007/008 gut. The sim suite passed
   (597 green) but the sim does not model DMA/IRQ serial timing — this class
   of regression is structurally invisible to the close gate
   (see `.clasi/knowledge/` loop-yield and IRQ-guard entries).
3. RX-side: inbound `*B` line drops under whatever state the burst windows
   correspond to (text lines survive the same windows, which argues TX-side
   or binary-dispatch-side, not raw serial RX).

## Scope note: loss recovery is needed regardless

Even after the burst regression is fixed, the pre-existing ~2–8% single-line
loss baseline means the binary plane needs SOME loss-recovery design —
bounded retry for idempotent envelopes at the `serial_conn` level, or
firmware-side reply prioritization. The text plane masked this with its
3-attempt retry; the binary plane currently has nothing. That design decision
is part of this issue's scope.

## Reproduce

Flash current HEAD firmware, then run a 60-ping `send_envelope` burst at
`read_timeout=400` against the robot and count `reply is None`. Compare with
60 `send_fast("PING")` text sends on the same boot (count pongs via an
`on_recv` hook).
