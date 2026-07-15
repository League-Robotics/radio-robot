---
status: pending
---

# Secondary telemetry starved to 0 Hz by the 106-001 loop-cadence retarget

Found during 106-001 ("Firmware loop-cadence fix") bench verification, 2026-07-15,
robot on the bench stand, direct USB (`/dev/cu.usbmodem2121102`) and relay
(`/dev/cu.usbmodem2121302`), robot v0.20260715.12+106-001.

## Problem

106-001 retargeted `source/app/robot_loop.cpp`'s `kCycle` from an unachievable 16ms
to an honest ~40ms (matching `App::Telemetry::kPrimaryPeriod`, `telemetry.h`), and
fixed the schedule's own arithmetic so the three settle/clearance windows are
absorbed into that 40ms total (see 106-001's own completion notes). Real
bench-measured loop cadence post-fix is a rock-solid 52ms/19.27Hz (both transports,
zero seq gaps over ~30s captures) — GOOD, and the design intent behind Decision 1
(architecture-update.md, sprint 106).

Side effect, discovered during bench verification, not anticipated by
architecture-update.md's Decision 1 analysis: because the real loop period (52ms)
now sits ABOVE `kPrimaryPeriod` (40ms) on every single cycle, `Telemetry::emit()`'s
`primaryDue(now)` is true every call, and its own documented "primary checked
first, unconditionally sent when due -- secondary can never delay it" contract
(`telemetry.h`/`telemetry.cpp`'s `emit()`) means secondary NEVER gets checked at
all. This is not a new code path — `telemetry.h`'s own pre-existing comment
(103-009) already named this exact failure mode:

> Scheduling note: this internal gate assumes emit() is called more often than
> kPrimaryPeriod (the loop's own per-cycle rate, well under 40 ms per
> architecture-update.md's runAndWait design) -- a caller that invokes emit() at
> EXACTLY the primary period would starve the secondary frame ... Not a defect
> this ticket resolves -- flagged for ticket [103-]008's own loop-cadence choice.

103-008 established the original (never-achievable) 16ms `kCycle` sketch; 106-001
is the ticket that actually fixed/retargeted that constant, and is therefore the
"loop-cadence choice" the 103-009 comment was written to flag forward to.

## Evidence

Confirmed empirically, twice, with independent draining methods (`Rig.
read_secondary_tlm()` polling loop, and a direct `SerialConnection.
read_binary_secondary_tlm(3000)` blocking call): **0 secondary frames received
over a 3s window**, both times, direct USB, post-106-001 firmware.

Contrast with the PRE-106-001 measured baseline (ticket 104-007's own soak
capture, `clasi/issues/ack-ring-intermittent-delivery-gap.md`): **1123 secondary
samples over a 240.1s direct soak = 4.676 Hz**, close to `kSecondaryPeriod`'s own
5 Hz target. Secondary telemetry worked correctly before 106-001 (because the
pre-fix real loop period, ~36-72ms depending on measurement, was NOT reliably
above `kPrimaryPeriod`=40ms on every cycle, so `primaryDue()` was occasionally
false, giving `secondaryDue()` a turn) and does not work at all after it.

`glitch`/`acc`/`ts`/`cmd_vel` diagnostics (`TelemetrySecondary`, the only wire
carrier for these fields since 103-001's prune moved them off the primary frame)
are therefore currently **unreachable from a live binary telemetry stream** on
any 106-001+ firmware build.

## Impact

- `tests/bench/rig_dev.py`'s own smoke check "secondary telemetry (glitch/acc/ts
  diagnostics) received" now fails every run (was already occasionally flaky
  pre-106-001; is now unconditionally failing post-106-001).
- Any host consumer of `cmd_vel`/`acc`/`glitch`/`ts` (velocity-PID diagnostics,
  encoder-glitch monitoring) has no live data source until this is resolved.
- Does NOT affect primary telemetry (`enc`/`vel`/`pose`/`otos`/`twist`/`active`/
  `acks`/`fault_bits`/`event_bits`) — that is the frame 106-001 was chartered to
  keep byte-identical, and did (confirmed unchanged content, confirmed flowing
  cleanly at the new 52ms/19.27Hz cadence, zero seq gaps).

## Direction (not resolved by 106-001 — out of that ticket's own scope: "only
pacing constants ... change")

`Telemetry::emit()`'s "primary always wins a same-call tie" contract is a
deliberate, documented 103-009 design choice (secondary must never delay
primary) — simply flipping the priority is not a safe fix. Candidate directions
for a follow-up ticket:

1. **Alternate/round-robin priority** when both are simultaneously due, instead
   of primary unconditionally winning every tie — guarantees secondary a turn at
   least once every `kSecondaryPeriod` even if primary is due every cycle,
   at the cost of occasionally delaying one primary frame by one cycle period.
2. **Shrink `kCycle`** back below `kPrimaryPeriod` (restores the pre-106-001
   headroom that let secondary through) — but this reintroduces 105-004's own
   diagnosed primary-cadence doubling defect (loop period < kPrimaryPeriod means
   `primaryDue()` needs 2 cycles some of the time, and 106-001's whole point was
   eliminating that doubling). Not recommended without also solving 1.
3. **Move select secondary fields onto the primary frame** (or a lower-rate
   subset gated differently) if `cmd_vel`/`acc`/`glitch`/`ts` turn out to be
   needed at a cadence the current split can't deliver without starvation.
4. **Accept 0 Hz secondary as a permanent trade-off** if no live consumer
   actually needs it post-P4 (verify against `fit_sim_error_model.py` and any
   other named consumer in `protocol.py`'s own `TLMFrame` docstring before
   choosing this).

Whichever direction is chosen is an `App::Telemetry` architecture decision
(`emit()`'s own scheduling contract), not a pacing-constant tweak — scope it as
its own ticket.
