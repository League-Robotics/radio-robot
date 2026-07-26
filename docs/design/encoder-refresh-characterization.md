# Encoder refresh characterization — the "~80 ms register" theory is false

**Status:** MEASURED, 2026-07-26, on the stand. This document is the
authoritative record; code and other docs should reference it rather than
restate the old theory.
**Apparatus:** `src/tests/firmware/encoder_rate/` — a self-contained bench
firmware (own CODAL CMake project) that drives both wheels at constant
duty and tight-polls the raw Nezha V2 0x46 encoder registers, byte-level
protocol, no driver conditioning. Capture/analysis:
`analyze.py` + three recorded runs (`capture*.log`) in the same directory.

## The old theory, and where it came from

Project folklore held that the Nezha brick's 0x46 encoder register
"refreshes only every ~80 ms," making most loop-cadence samples stale.
The number was inferred once, during the freshness-gate bug fix (~4 of 5
collects returned the same raw count at a ~16 ms collect cadence), and
was never characterized directly. It then propagated into driver
comments, DESIGN docs, the base-loop sketch, and planner assumptions.

## What the hardware actually does

### 1. The register is live (no 80 ms latch)

Fresh value on **every single poll**, in every condition tested:

| Phase | Condition | Poll period | Result |
|---|---|---|---|
| A1 | LEFT alone, tight poll, 30% duty | ~16 ms | fresh every poll (1250/1251); interval stdev 0.03 ms |
| A2 | RIGHT alone, tight poll | ~16 ms | fresh every poll |
| B | L/R alternating | ~32 ms/wheel | fresh every poll, both wheels |
| C | LEFT slow poll | ~24-32 ms | fresh every poll |
| D | tight poll + duty write BEFORE each select | ~16-24 ms | fresh every poll |
| E | LEFT at 8% duty (slow wheel) | ~16 ms | fresh every poll (deltas ~16-22 tenths-deg) |

Count deltas are unimodal (61±3 tenths-deg per 16 ms at 30% duty) — no
latch quantization anywhere. An 80 ms latch would have shown ~5 polls
per change in phase A1; the measured value was exactly 1, always.
**The register updates continuously, or at least far faster than 16 ms.**

### 2. The real staleness mechanism: interposed-traffic sample invalidation

**Phase F: select-L → duty write (0x60, same value) → read returned
raw 0, 416 out of 416 times.** Any 0x10 transaction interposed between
the 0x46 select-write and its read destroys the pending sample — the
read returns zeros, which the production driver's glitch rejection
discards, manifesting exactly as "sample stale this cycle."

The pre-118 loop schedule did exactly this interposition (back-to-back
selects, then collects — the same select-latch misuse sprint 118's
interleaved schedule fixed). That era is where the "~4 of 5 stale ⇒
~80 ms refresh" inference was born. The staleness was real; the
attribution to a brick refresh timer was wrong.

### 3. Select-latch semantics (confirmed)

The brick holds ONE pending encoder select; the last select wins. After
select-L → select-R → read, the read returns the RIGHT counter — clean,
valid data for the selected port (phase G). Ordering discipline matters;
data integrity survives. This is the behavior the loop's clear/settle
slices exist to discipline.

## Implications

1. **Base loop** (main-repo territory): with the current (118)
   interleaved schedule — select → settle → collect → duty write →
   clear → next port — the select→read window stays clean, so encoder
   samples should be fresh **every 50 ms cycle**. Worth one confirmation
   from telemetry `sampleTime` deltas through the real loop. If any
   staleness remains, hunt for 0x10 traffic inside a select→read
   window — not for a refresh timer.
2. **Velocity-PID placement rationale**: the "PID cannot run faster than
   the loop anyway (encoder freshness ~80 ms bounds it)" argument is no
   longer valid as stated — the register could support a faster inner
   loop. The PID-to-motion relocation decision may still stand on its
   other merits (one estimate feeding one controller, tunable in
   motion_tests); the freshness bound just isn't one of them.
3. **Motion planner**: plan for fresh samples at the loop cadence. The
   fresh-sample-gated EMA stays correct (it keys on `sampleTime`, never
   an assumed period); stale-cadence noise emulation is a degraded-mode
   test, not the expected regime.
4. **Wedge caution unchanged**: this characterization used same-value,
   same-direction duty writes. The reversal-write-train 0x46 latch-up
   (`docs/knowledge/2026-07-04-encoder-wedge.md`) is a separate,
   still-real hazard.

## Reproducing

```
cd src/tests/firmware/encoder_rate && mkdir -p build && cd build && cmake .. && make -j8
just erase   # APPROTECT relocks after runs; mass-erase before each flash
pyocd flash -t nrf52833 ../MICROBIT.hex
uv run python ../analyze.py --port /dev/cu.usbmodem2121102
```

~80 s per run; motors stop and the display shows `E` when done. Rebuild
and reflash the ROBOT firmware afterward (`just build`, `just erase`,
`pyocd flash -t nrf52833 MICROBIT.hex`).
