# Cooperative loop timing and achievable control frequencies

**Measured:** 2026-06-05, firmware `0.20260605.2`, 40-second steady-state window (1680
iterations), robot on stand, all three sensors active, no streaming, carriage running.

## Raw data

```
LOOP ctl=11421 cyc=23996 wrk=18172 loops=1680
     t0=7  t1=4  t2=14  t3=193  t4=2395  t5=4011  t6=70  t7=4
```

All values in **µs, averaged per loop iteration**.

| Field | Value | Description |
|---|---:|---|
| `ctl` | **11 421 µs** | control task: encoder read (8 ms vendor busy-wait) → PID → PWM write |
| `cyc` | **23 996 µs** | full loop period incl. idle sleep (~42 Hz) |
| `wrk` | **18 172 µs** | work per iteration excl. idle sleep |
| `loops` | 1680 | sample count |
| `t0` | 7 µs | comms-in |
| `t1` | 4 µs | drive-advance |
| `t2` | 14 µs | odometry-predict |
| `t3` | 193 µs | otos-correct (gated to 100 ms; amortized over ~100 iterations) |
| `t4` | **2 395 µs** | line-read (runs every iteration) |
| `t5` | **4 011 µs** | color-read (runs every iteration) |
| `t6` | 70 µs | ports-read |
| `t7` | 4 µs | telemetry-emit (off; rises to ~45 µs when streaming at 5 Hz) |

## What the numbers mean

### Control task (encoder → PID → PWM) — 11.4 ms

This is the loop metronome and its cost is almost entirely **hardware timing**.  The Nezha
V2 motor controller needs ~8 ms between writing the encoder-request command and reading the
response (4 ms pre-write settle + 4 ms post-write settle — vendor requirement; see
`source/hal/Motor.cpp` `readEncoderAtomic`).  That time cannot be safely shortened without
risking the read returning zeros or corrupting the bus.  The remaining ~3 ms is PID math +
PWM writes.

**Consequence:** the control loop can never run faster than ~85–90 Hz regardless of
software improvements.

### Line + color reads — 6.4 ms combined, every iteration

At the time of measurement both sensor reads run on *every* loop iteration (no cadence
gate).  They are by far the largest tunable cost.

- line-read: 2.4 ms (4 × single-byte I2C write+read at 0x1A)
- color-read: 4.0 ms (blocking `pollRGBC` or similar at 0x43)

These are perception sensors, not control sensors.  They do not need to update at the full
PID rate.

### Odometry-predict — 14 µs, pure math

Free.  Keep it every iteration so dead-reckoning updates at the full control rate.

### OTOS-correct — 193 µs (amortized)

The OTOS I2C read itself costs ~2 ms, but the task is internally gated to run only every
100 ms (10 Hz), so the per-iteration average is 193 µs.  Raising the gate to 20–50 Hz
would add ~20–100 µs/iteration — negligible.

### Telemetry-emit — 4 µs when off, ~45 µs at 5 Hz

Streaming at a moderate rate has near-zero impact on the loop.

## Achievable improvement

Gate line and color reads to ~50–100 ms instead of every iteration:

| Scenario | Work/iteration | Loop period | Effective PID rate |
|---|---:|---:|---:|
| Today (line+color every iteration) | 18.2 ms | 24.0 ms | **~42 Hz** |
| Line+color gated to 50 ms | ~11.8 ms | ~13.5 ms | **~74 Hz** |
| Line+color gated to 100 ms | ~11.8 ms | ~13.5 ms | **~74 Hz** |
| Theoretical ceiling (no sensor reads) | ~11.5 ms | ~13.1 ms | **~76 Hz** |

The practical ceiling is **~75–80 Hz**, set by the encoder read's mandatory 8 ms busy-wait.
Gating the sensor tasks is the highest-leverage single change.

## How to gate sensor task cadence

Each task in the `LoopScheduler` table has a `periodMs` field.  The OTOS-correct task
(index 3) already uses `cfg.lagOtosMs` (default 100 ms).  Setting `lagLineMs` and
`lagColorMs` to 50–100 ms via `SET lag.line=50 lag.color=50` (or changing defaults in
`Config.h`) would gate those tasks and approximately double the PID rate with no other
code changes.

## How to reproduce

```
# Send via serial monitor or rogo:
DBG LOOP RESET          # zero stats
# wait 30–60 s
DBG LOOP                # read the one-line dump
```

Output format:
```
LOOP ctl=<us> cyc=<us> wrk=<us> loops=<n> t0=<us> t1=<us> t2=<us> t3=<us> t4=<us> t5=<us> t6=<us> t7=<us>
```

Task index map: 0=comms-in, 1=drive-advance, 2=odometry-predict, 3=otos-correct,
4=line-read, 5=color-read, 6=ports-read, 7=telemetry-emit.
