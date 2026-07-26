# Encoder refresh characterization — RESULTS (2026-07-26)

**Bottom line: the ~80 ms encoder-refresh folklore is FALSE.** The Nezha
V2 brick's 0x46 encoder register is live at every polling rate tested
(fresh value on **every single poll** at ~16 ms, ~24 ms, and ~32 ms
periods, single-port and alternating). The staleness the robot loop
historically observed is a **loop-schedule artifact**, not a brick
refresh rate: any 0x10 transaction interposed between the 0x46
select-write and its read **invalidates the pending select — the read
returns 0**. The pre-118 loop schedule did exactly that interposition
(back-to-back selects, then collects), which is where the "~4 of 5
samples stale ⇒ ~80 ms" inference was born.

Setup: bench firmware in this directory (`app/main.cpp`), robot on the
stand, both wheels driven at constant duty, raw byte-level protocol
(driver conditioning deliberately bypassed). Three runs; raw captures in
`capture_run1.log` / `capture_run2.log` / `capture.log` (run 3).

## Question 1 — actual refresh period and jitter

| Phase | Condition | Poll period | Result |
|---|---|---|---|
| A1 | LEFT alone, tight poll, 30% duty | ~16 ms | fresh EVERY poll (1250/1251); interval stdev 0.03 ms |
| A2 | RIGHT alone, tight poll | ~16 ms | fresh EVERY poll |
| B | L/R alternating | ~32 ms/wheel | fresh EVERY poll, both wheels |
| C | LEFT slow poll | ~24-32 ms | fresh EVERY poll |
| E | LEFT at 8% duty (slow wheel) | ~16 ms | fresh EVERY poll (deltas ~16-22 tenths-deg) |

Count deltas are unimodal (e.g. 61±3 tenths-deg per 16 ms at 30% duty) —
no latch quantization anywhere. **The register updates continuously (or
at least far faster than 16 ms).** An 80 ms latch would have shown
~5 polls per change in A1; we saw exactly 1, always.

## Question 2 — are L and R on the same refresh clock?

Moot as originally posed: there is no observable refresh latch to
correlate. Phase G established the **select-latch semantics**: the brick
holds ONE pending encoder select, last select wins — after
select-L → select-R → read, the read returns the RIGHT counter (clean,
valid data). Ordering discipline matters; data integrity survives.

## Question 3 — does traffic perturb the readback? (the real finding)

**Phase F: select-L → duty write (0x60, same value) → read returns 0 —
416 out of 416 times.** An interposed 0x10 transaction between the 0x46
select and its read destroys the pending sample entirely (reads as raw
0, which the production driver's glitch rejection would discard —
manifesting exactly as "sample stale this cycle").

Phase D control: a duty write BEFORE the select (write → select → read)
is harmless — fresh every poll. Only the select→read window is fragile.

## Implications

1. **For the base loop (main-repo territory):** the current (118-fixed)
   interleaved schedule — select L → settle → collect L → duty write L →
   clear → select R → … — keeps the select→read window clean, so encoder
   samples should be fresh EVERY 50 ms cycle already. Worth confirming
   from telemetry `sampleTime` deltas on the real firmware; if any
   staleness remains, look for 0x10 traffic sneaking into the
   select→read window, not for a brick refresh timer.
2. **For the motion planner:** plan for fresh samples at the loop
   cadence (50 ms), not 80 ms. The fresh-sample-gated EMA stays correct
   (it keys on `sampleTime`, not an assumed period); the noise-tier
   staleness emulation (issue §7.1's N=2 cadence) drops to a
   degraded-mode test rather than the expected regime.
3. **Fix the folklore at its sources** when next touched:
   `src/firm/devices/DESIGN.md`, `nezha_motor.{h,cpp}` comments, and
   `docs/design/base-explicit-loop-sketch.md` items 4/7 all state
   "~80 ms register refresh" as fact. The correct statement: the
   register is live; the historical staleness was interposed-traffic
   sample invalidation under the pre-118 schedule.

## Reproducing

```
cd src/tests/firmware/encoder_rate && mkdir -p build && cd build && cmake .. && make -j8
just erase   # APPROTECT relocks after runs; mass-erase before each flash
pyocd flash -t nrf52833 ../MICROBIT.hex
uv run python ../analyze.py --port /dev/cu.usbmodem2121102
```

Phases in the current firmware: A1 (baseline tight poll), B (alternating),
D (duty write before select), E (8% duty), F (duty write INSIDE
select→read window), G (other-port select inside the window). ~80 s
total; motors stop and the display shows `E` when done. Remember to
rebuild and reflash the ROBOT firmware afterward (`just build`,
`just erase`, `pyocd flash -t nrf52833 MICROBIT.hex`).
