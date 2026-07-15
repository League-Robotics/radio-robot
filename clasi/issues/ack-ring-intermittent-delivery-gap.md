---
status: pending
---

# Ack-ring entries intermittently missing for individual commands (P4 single-loop firmware)

Found during 104-006 (bench script family rewritten to the binary twist/config/stop
plane) hardware verification, 2026-07-15, robot on the bench stand, direct USB
(`/dev/cu.usbmodem2121102`).

## Problem

`NezhaProtocol.wait_for_ack(corr_id)` — the P4 wire's only way to observe a
`twist`/`stop`/`config` command's outcome (its ack rides the depth-3 `Telemetry.acks`
ring inside a subsequent `Telemetry` push, per `envelope.proto`/`103-009` Decision 2) —
intermittently returns `None` for a specific `corr_id` even when:

- the command was well-separated in time from any other command (no back-to-back
  contention for the ring's 3 slots),
- the wait timeout was generous (tested up to 2000 ms, 4-10x the ~40 ms telemetry
  cadence),
- the SAME corr_id/command shape succeeds cleanly on other invocations (non-
  deterministic — rerunning an identical script produces different pass/fail patterns
  run to run).

Confirmed independent of any bench-script logic (104-006's own scripts were not
involved in isolating this — the following used only `SerialConnection`/
`NezhaProtocol` directly, the same primitives `twist_drive.py`, `rig_dev.py`, and
`rig_soak.py` all build on):

```
for i in range(5):
    corr = proto.twist(v_x=80.0, omega=0.0, duration=400.0)
    ack = proto.wait_for_ack(corr, timeout=2000)
    print(f"corr_id={corr} ack={ack}")
    time.sleep(2.0)
```
produced (one representative run):
```
corr_id=1 ack=AckEntry(corr_id=1, ok=True, err_code=0)
corr_id=2 ack=None
corr_id=3 ack=AckEntry(corr_id=3, ok=True, err_code=0)
corr_id=4 ack=None
corr_id=5 ack=None
```
Re-running the identical script produced a *different* pass/fail pattern (including one
run where even `corr_id=1`, the very first command after a fresh connect, missed).
Draining `drain_binary_tlm()` continuously and logging every `AckEntry` seen (rather
than only searching for one specific corr_id) shows the ack for a "missing" corr_id
sometimes DOES appear later — but only once the caller has already moved on to a
different `corr_id`'s own search window and stopped looking for it — which explains why
the miss is not simply "increase the timeout": in at least one isolated trial a corr_id
never appeared in ANY polled frame across a full 2000 ms window.

One anomalous trial also showed the underlying wheel *encoders* not moving during a
`twist()`'s watch window (not just the ack) — suggesting the loss may occasionally be
the command itself (inbound `*B` line), not only the outbound ack, though this was a
single observation and not independently isolated.

`config()`'s ack (`ok=False, err_code=ERR_UNIMPLEMENTED`, the expected outcome per
`source/main.cpp`'s `CmdKind::CONFIG` case) was confirmed to work correctly when tested
in isolation (2/3 trials acked cleanly, well-separated, matching the same intermittent
rate as `twist`/`stop`) — ruling out a `config`-arm-specific bug; the behavior is
uniform across all three `cmd` oneof arms.

## What does NOT show this problem

`tests/bench/rig_soak.py`'s continuous reissue loop (twist re-sent every ~150 ms,
draining telemetry every ~10-20 ms, no idle gaps) measured only **0.65% ack loss**
over a clean 25 s run (155 commands sent, 347 primary frames, **0.00% TLM frame drop
rate**, zero new fault bits, 97% encoder-responsiveness rate). This suggests the
intermittent-miss rate is much worse for *discrete, well-separated* single commands
(the pattern above, and `rig_dev.py`'s own single-shot smoke-test style) than for a
*continuous, tightly-polled* reissue loop — plausibly because a continuously-draining
poller has many more chances to catch a given ack before it ages out of the ring,
while a discrete "send one command, wait, move on" caller only gets one narrow window.

## Impact

- `tests/bench/rig_dev.py`'s smoke verification (104-006) typically passes 5-7 of its 8
  checks on a given run (varies run to run) — the misses are consistently individual
  `wait_for_ack()` calls, never `connect()`, encoder-movement, or secondary-telemetry
  checks, which are reliable every run.
- `tests/bench/rig_soak.py` does NOT gate on ack loss (see its own module docstring) —
  it gates on TLM drop rate (a genuinely reliable, near-zero metric) and encoder
  responsiveness instead, specifically because of this finding.
- Not yet root-caused: could be firmware-side (the depth-3 ring itself, or how/when
  `Telemetry::ack()` is called relative to `emitPrimary()`), or a real (if rare) `*B`
  line loss on the direct-USB link under the CURRENT single-loop firmware — untested
  whether the historical ~0.03% frame-drop figure (`relay_telemetry_rate.py`,
  pre-single-loop firmware) still holds for the P4 firmware's own binary armor/framing.

## Direction

- Root-cause investigation: instrument `Telemetry::ack()`/`emitPrimary()` (or a targeted
  pyOCD/gdb session, see `.claude/rules/debugging.md`) to determine whether the ring
  entry is ever actually written for a "missing" corr_id, vs. written-but-not-
  transmitted, vs. transmitted-but-lost on the wire.
- Ticket 007 (`clasi/sprints/104-host-realignment-and-full-bench-gate/tickets/
  007-p6-soak-gate-sustained-dual-transport-bench-runnable-verification.md`) — its own
  sustained dual-transport soak run is a natural place to gather a much larger sample
  (direct USB AND relay) and confirm/refine the loss-rate characterization above.
- Consider whether `docs/protocol-v2.md`/`envelope.proto`'s ack-ring depth (3) is
  adequate given telemetry cadence and typical command rates, or whether a corr_id
  should be re-broadcast for more than 3 frames.
