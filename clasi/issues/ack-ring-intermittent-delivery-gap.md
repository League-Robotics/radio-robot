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

## 104-007 characterization (2026-07-15, robot v0.20260715.3, bench stand)

Ticket 007's own sustained dual-transport soak run (`clasi/sprints/
104-host-realignment-and-full-bench-gate/tickets/done/
007-p6-soak-gate-sustained-dual-transport-bench-runnable-verification.md`) gathered
a much larger sample, direct USB AND relay, per this issue's own "Direction" section
above. Ad hoc scratch diagnostics (non-destructive, continuously-draining ack
matchers — unlike `wait_for_ack()`'s own bounded-then-give-up semantics — so a
"missed" ack could still be caught arbitrarily later in the same capture) were used;
not committed to the tree, but the exact patterns are reproducible from this
section's description. Four distinct findings, not one:

**1. The discrete/well-separated repro's "miss" is a DELAY, not a loss — and NOT a
ring-depth-eviction effect.** Reproduced the exact pattern in this issue's own
"Problem" section (twist, wait, sleep ~1-2s, repeat) over direct USB across three
trials (15/20/20 commands at 1.5-2s gaps): 20-30% of commands showed their ack
arrive **~1 full inter-command gap late** (e.g. sent at t=22.6s, ack not observed
until t=24.2s — a ~1.6s delay at a 1.5s gap), bundled into the SAME telemetry frame
as the NEXT command's own ack. Critically: **nothing evicts a ring entry in this
pattern** — no other command is acked in between to push it out (`Telemetry::ack()`,
`source/app/telemetry.cpp`, only shifts the ring when a 4th entry arrives) — so ring
depth 3 cannot be the cause here. TLM frame delivery itself was ALSO 0.000% dropped
throughout every trial (seq-gap accounting), proving the delay is not a
telemetry-transmission loss either. Encoder motion corroborated the delay is real at
the plant, not just cosmetic: the affected commands' encoders did not move until the
same late window the ack appeared in. One trial showed a ~1.6s stall affecting FOUR
consecutive commands in a row with near-identical latency each (1587-1606ms), then a
full recovery to normal (~100ms) latency on the next command — a single contiguous
stall event, not independent per-command coin-flips. Root cause not conclusively
identified (would need the wire-level/pyOCD instrumentation this issue's own
"Direction" section names — out of this verification-only ticket's scope), but the
evidence points at a HOST-side or serial-link stall (Python scheduling, pyserial
write latency, OS driver buffering) rather than firmware ring/cadence behavior. Over
the relay, the same discrete pattern (2 trials, 37 commands total) showed ZERO
delayed acks — cleaner than direct in this sample, though the N is small enough that
this is not proof the relay never exhibits it.

**2. Realistic continuous streaming (the rate 106 actually needs) is reliable with a
continuously-draining client.** `rig_soak.py`'s own 150ms reissue period (~6.7
commands/sec, matching "several twists/sec") over the OFFICIAL 240s soak windows
(clean-boot, see ticket 007's completion notes): **0.07% ack loss / 0.00% TLM drop
over direct USB; 2.17% ack loss / 0.48% TLM drop over the relay.** The relay is
measurably noisier than direct (a real, small amount of genuine frame loss on the
radio path, unlike direct's perfect record) but nowhere near the severity the
discrete-pattern repro suggested — because a continuously-draining consumer (any
loop that keeps calling `read_pending_binary_tlm_frames()`/`drain_binary_tlm()`
every pass, never stopping to wait-then-give-up on one corr_id) simply doesn't hit
finding 1's failure window: even a ~1.6s stall gets caught on the very next drain
pass once traffic resumes.

**3. True ring-depth eviction IS real, but only under an artificial zero-paced
burst no realistic caller produces.** Sent N `twist()` commands fully back-to-back
(no sleep between `send_envelope_fast()` calls at all — a ~9-10ms burst for 12
commands) with ZERO polling until after the whole burst, then drained continuously
for a 4s tail. Direct USB: 10/12 acks eventually appeared (in matched pairs roughly
72ms apart, i.e. one telemetry period each, consistent with ~2 commands processed
per firmware loop cycle-pair); the LAST 2/12 never appeared in any frame across the
4s tail — genuine, permanent ring-depth loss, matching the eviction mechanism this
issue's own "Direction" section hypothesized, but only reachable this way. Over the
relay, the SAME 12-command zero-paced burst lost ALL 12/12 permanently — the relay
hop has materially less burst-absorption capacity than direct USB. Neither rate
matches any actual caller in this tree today (`rig_soak.py` paces at 150ms;
`rig_dev.py`/`twist_drive.py` send one command and wait) — but it is a real ceiling
a naive un-paced 106 send loop (e.g. a bug that queues sends with no yield) could
hit, and it hits much harder over the relay.

**4. New, orthogonal finding — the relay CONNECT handshake itself trips
`kFaultCommsMalformed` once, before any application command is sent.** Confirmed
with an isolated test: fresh clean-boot firmware, connect via the relay's `!GO` data
plane ONLY (zero `twist()`/`stop()`/`config()` calls), and the very first telemetry
frames already show `fault_bits` bit 3 set. This is NOT the host's application
traffic tripping it (the AC 007 was gating on) — it is something in the relay's own
`!ECHO OFF`/`!MODE RAW250`/`!GO` handshake or the transition-to-transparent-mode
window that the robot's line parser sees as a malformed frame. Structurally
identical to `kFaultI2CSafetyNet`'s own documented boot-time one-shot (fires once,
never re-trips during the session) but triggered by relay-connect rather than
firmware-boot. Filed as its own new issue —
`clasi/issues/relay-handshake-trips-comms-malformed.md` — since it is not an
ack-ring/cadence problem and doesn't share this issue's root-cause space.

### Recommendation (the "emit cadence vs ring depth vs host polling" question)

**Host polling fix — primary recommendation.** Finding 1 (the practically-observed
failure mode for a discrete, wait-then-give-up caller) is not a ring or cadence
problem at all: a continuously-draining matcher (as `rig_soak.py` and this
characterization's own diagnostics use) catches a finding-1 delay every time,
because the ack is never actually evicted — it is just late. `rig_dev.py`'s current
`wait_for_ack_retrying()` (bounded timeout, give up, move to the next command) is
the one place in this tree still vulnerable to finding 1. Recommend: sprint 106
should NOT gate any closed-loop control decision on a bounded per-command
`wait_for_ack()` — either (a) don't depend on command acks for feedback at all (use
the continuously-flowing `pose`/`twist`/`enc` telemetry fields instead, which is the
natural fit for heading feedback anyway and sidesteps this whole failure class), or
(b) if a caller genuinely needs ack confirmation, track it with a continuously-
draining background matcher rather than a bounded-wait-then-abandon call.

**Ring depth increase — not recommended as a primary fix.** Finding 3 shows the
eviction mechanism is real, but only past an unrealistic zero-paced-burst threshold
neither `rig_soak.py` nor any planned 106 streaming pattern (several twists/sec,
paced) reaches. A deeper ring would raise that ceiling but does nothing for finding
1 (nothing is evicting there) and does not change the relay's much lower
burst-absorption capacity (finding 3's 0/12 vs 10/12) in relative terms.

**Emit cadence fix — secondary, not primary.** Ticket 007 measured the REAL primary
cadence at ~13.87 Hz (72ms), materially below both the 25 Hz design target and
103-010's own ~15.62 Hz figure (see ticket 007's completion notes for the full
measurement and the loop-cycle correlation). Raising cadence toward 25 Hz would
shrink the dwell/delay window somewhat and reduce how long a caller has to keep
polling to catch a finding-1 delay, but does not touch finding 1's root cause
(upstream of telemetry emission) and is a separate, larger change (main loop timing)
than this issue's own scope.

**Is twist streaming at 106's target rates reliable? Mostly yes, with one loud
caveat.** At realistic paced rates (finding 2), yes — 0.07%/2.17% ack loss direct/
relay, and genuinely reliable TLM delivery, is fine for several-twists/sec
streaming, PROVIDED the consumer drains continuously rather than using
`rig_dev.py`'s current discrete wait-then-give-up pattern. The loud part: do not let
106 ever send twist commands back-to-back with no pacing at all (finding 3) — this
is a real, sharp cliff, and it is much worse over the relay (100% loss in the tested
burst) than direct (83% survival). A paced sender (anything resembling
`rig_soak.py`'s 150ms reissue) does not come close to this cliff.
