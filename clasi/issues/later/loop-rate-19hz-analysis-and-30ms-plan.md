---
status: pending
filed: 2026-07-26
filed_by: team-lead (stakeholder-directed logical analysis, motion-planner checkout)
related:
- motion-planner-implementation.md
---

# Loop rate: why ~19 Hz instead of the designed 25 Hz, and how to reach 30 ms

**Self-contained** — the full analysis is inline; no other document is
required to act on this. Code references are for verification.

## 0. Framing: 40 Hz was never the design target

`RobotLoop::kCycle = 40` [ms] (`src/firm/app/robot_loop.h`, ~25 Hz) is
the shipped pace target. The schedule is four `runAndWait` blocks:

| Block | Gap | Purpose | Borrowed work inside |
|---|---|---|---|
| settle-L | `kSettle` = 4 ms | Nezha 0x46 select→read settle | `comms_.pump()` |
| clear | `kClear` = 4 ms | post-duty-write bus clearance | (empty since 119-005) |
| settle-R | `kSettle` = 4 ms | R select→read settle | `processMessage()` |
| pace | `kPace` = 40−12 = 28 ms | absorb the rest to hit kCycle | odom, OTOS, line/color, estimator, assembleFrame, emit, MoveQueue tick |

So the two questions are separate: (a) why the loop delivers ~19 Hz
instead of its designed 25 Hz; (b) what it takes to run at 30 ms
(33 Hz) or 25 ms (40 Hz).

## 1. Where the missing ~12 ms/cycle goes (40 ms design → ~52 ms actual)

### 1a. Fiber-sleep tick quantization — the dominant tax (~+8-12 ms)

Every `runAndWait` ends in `fiber_sleep()` (via
`Devices::MicroBitSleeper::sleepMillis`). CODAL wakes sleeping fibers on
its scheduler tick, and this target sets
`SCHEDULER_TICK_PERIOD_US: 4000` (`src/libraries/codal-microbit-v2/
target-locked.json`). A 4 ms sleep on a 4 ms tick grid takes 4-8 ms
depending on phase — average ~+2 ms overshoot per sleep, worst +4. The
loop sleeps **four times per cycle** → +8 ms typical, +16 worst.
40 + 8..13 ≈ 48-53 ms ≈ 19-20 Hz. **This alone explains the gap.**

Corroboration: the sprint-086 flip-flop measurement (44-52 Hz per motor
vs an 80-90 Hz design estimate — `clasi/sprints/done/086-*/`) is the
same mechanism at the old schedule's two-sleeps-per-half-cycle.

### 1b. Unpaced bus transactions between blocks (~+2-4 ms)

The selects, collects, and duty writes run OUTSIDE the `runAndWait`
gaps (`cycle()`'s own call sites): each is a real I2C transfer
(~0.3-1 ms), four to six per cycle. Pure addition on top of the 40.

### 1c. `sleepUntil` sleeps ≥1 ms even on overrun (small bug)

`src/firm/app/robot_loop.cpp` `sleepUntil()`:
`sleeper_.sleepMillis(remaining > 0 ? remaining : 1)` — when a block's
body already overran its gap, it still sleeps "1 ms", which under 1a's
quantization costs up to 4 ms. Overrun should `yield()`, not sleep.

### 1d. The measured 19 Hz may partly be DELIVERY, not the loop

Sprints 115/116 measured the same ~18.9-19 Hz **at the host** and
diagnosed it as serial bandwidth: the CRC16+COBS-framed
primary+secondary telemetry at 115200 baud (~11.5 bytes/ms), with
`SerialPort::send()` dropping WHOLE frames when the 255-byte TX buffer
lacks space (`docs/bench-checklists/sprint-116-move-protocol.md` ~L231,
`sprint-115-gut-s1.md` ~L405). Host-side inter-frame timing therefore
under-reports the loop rate whenever frames drop. **First action of any
fix: read `cycleBusy`/`cyclePeriod` (122-003 loop-timing telemetry
fields) to split loop time from delivery time.**

## 2. Structural floor with the current architecture

Irreducible per-cycle time given the Nezha vendor timing (measured
2026-07-26, `docs/design/encoder-refresh-characterization.md`: the
0x46 select→read settle and the ≥4 ms 0x10 transaction spacing are
real; the "~80 ms register refresh" is NOT):

- 3 × 4 ms vendor windows (settle-L, clear, settle-R): **12 ms**
- actual I2C transfers (selects/collects/duty writes): ~2-3 ms
- pace-block work: OTOS burst (write + 4 ms per-device clearance +
  12-byte read ≈ 4.5 ms on reading cycles, `Devices::RealOtos`,
  `kBusClearance = 4000` us), line-or-color read ~1 ms, estimator +
  odometry + assembleFrame + COBS ~1-2 ms, async sends ~0.5 ms:
  **~8-10 ms**

True busy time today ≈ **22-25 ms**. The OTOS clearance can overlap the
motor settle windows (different I2C device; the I2CBus clearance
deadlines are per-device) for ~4 ms back. Realistic floor:
**~18-22 ms (45-55 Hz)**. The brick's timing, not CPU, binds.

## 3. The plan to 30 ms (33 Hz), in order

1. **Measure**: capture `cyclePeriod`/`cycleBusy` from telemetry on the
   bench. Confirms 1a/1d split before touching anything.
2. **Kill sleep quantization** (biggest single win, ~6-12 ms back):
   set `SCHEDULER_TICK_PERIOD_US` to 1000 via the `codal.json` config
   block — or keep the tick and sleep `gap−1` then spin the last
   millisecond on `system_timer_current_time_us()`. Either way the
   loop should then actually deliver its designed 40 ms/25 Hz.
3. **Fix `sleepUntil`'s overrun floor**: `remaining == 0` → `yield()`.
4. **Drop `kCycle` 40 → 30**: budget = 12 ms windows + ~10 ms pace
   work + slack. Fits without restructuring.
5. **For 25 ms (40 Hz)**: additionally move the OTOS transaction pair
   inside a motor settle window so its 4 ms clearance overlaps a window
   that is being slept anyway.

### Traps that WILL bite below ~35 ms (do these with step 4)

- **`kMinWriteIntervalUs = 35000`** (`src/firm/devices/nezha_motor.cpp`,
  duty-write throttle, hand-derived as kCycle(40)−5 ms): at
  kCycle = 30 it silently skips every other duty write. Must be derived
  from kCycle, not a literal.
- **Serial bandwidth**: at 33 Hz the per-frame budget is
  11520 B/s ÷ 33 ≈ **345 bytes**; the current armored primary+secondary
  frame is at/past that (that IS the measured ~19 Hz delivery cap).
  Raising the loop rate without shrinking frames just drops more of
  them. Protocol v5's lean-primary/delete-secondary work (sprint 124's
  `protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md`,
  `robot-state-blackboard-...md`) is the scheduled fix; a baud raise
  (115200 → 230400/460800 over DAPLink CDC) is the alternative/adjunct.
  The radio path fragments at 250 B/packet and may need rate-division
  at higher loop rates.

## 4. Ownership / coordination

Everything in §3 is `src/firm` + codal-config territory — the main
environment (sprints 124-126), not the motion-planner checkout. Filed
from the parallel effort as analysis input. Motion-planner consequence
if the loop speeds up: `PlannerLimits.controlPeriod` is already a
parameter (50 ms default), and the discrete-exact profiler re-derives
everything from dt — no planner rework needed at 30 ms beyond passing
the new period.

## 5. MEASURED on tovez, 2026-08-07 (throwaway experiment, nothing committed)

Everything above §5 was analysis. These are hardware measurements from
a detached worktree, telemetry `cycle_period`/`cycle_busy` over direct
serial, idle and driving (both wheels 120 mm/s). Firmware reverted and
tovez restored to baseline afterwards.

**Method for measuring the floor: set `kCycle` BELOW it.** The trailing
`sleepUntil()` finds `remaining == 0`, yields, and the delivered period
becomes the loop's true work time. No estimation needed.

| variant | kSettle | kClear | `requestEncoder` preClear | kCycle | period median | busy median |
|---|---|---|---|---|---|---|
| baseline | 4 | 4 | 4000 | 50 | 50.00 | 21.27 / 22.27 drv |
| A | 4 | 4 | **0** | 50 | 50.00 | 21.27 / 22.26 drv |
| B (floor probe) | 4 | 4 | 0 | 20 | **22.22** | 21.95 |
| C | 4 | 4 | 0 | 32 | 32.00 (max 32.04) | 21.27 / 23.27 p95 |
| D | **0** | **0** | 0 | 20 | 20.00 | **17.26** |
| E | 4 | **0** | 0 | 20 | 20.00 | **17.27** |
| F (floor probe) | 4 | 0 | 0 | 12 | **17.70** | 17.62 |

### Three findings

1. **`requestEncoder`'s `preClear=4000` is dead weight.** Removing it
   (A vs baseline) changed nothing — period, busy, and encoder integrity
   all identical. It is provably redundant by construction too: `lastEnd`
   is PER-DEVICE (`devices_[idx].lastEnd`), and whichever 0x10 write
   preceded the select already set `readyAt = lastEnd + 4000` via its own
   `postClear`. `preClear` recomputes the same value; the `max()` is a
   no-op. Its `postClear=4000` IS load-bearing — it is the only thing
   making the paired `collectEncoder()` (which passes `0/0`) honour the
   select→read settle.

2. **The two 4 ms `kSettle` windows cost zero wall-clock; the 4 ms
   `kClear` window is pure padding worth 4.7 ms/cycle.** D vs E isolates
   it exactly: keeping both settles (E, 17.27) is indistinguishable from
   zeroing them (D, 17.26), because their wait simply reappears inside
   `waitForClearance`'s `fiber_sleep` — the settles are where that
   mandatory wait gets spent *usefully* (comms pump). `kClear` is
   different: the next 0x10 op after the duty write is next cycle's
   encoder select, 20-50 ms later, long past any clearance deadline. The
   window guards nothing.

3. **131-005's absolute-deadline pacer is confirmed on hardware** —
   first time. 50.00 ms median, min 49.97, max 50.06 (the 130-011 era
   delivered ~54 ms). At `kCycle=32` it holds 32.00 with max 32.04.

### The numbers that answer "what is the maximum loop rate"

- **Irreducible bus wait: 8 ms/cycle** (2 × 4 ms encoder select→read
  settle, one per motor). Not removable without restructuring.
- **Floor as shipped: 22.2 ms** (~45 Hz) median, ~28 ms worst case.
- **Floor with `kClear` removed: 17.7 ms** (~57 Hz) median, ~30 ms worst.
- **Safe `kCycle` today: 32 ms.** Verified rock-solid. With `kClear`
  removed, ~30 ms has comfortable margin. Either is a 1.6x rate
  increase over the current 50 ms.
- The tail, not the median, is what binds — worst-case cycles run
  ~28-30 ms in every configuration.
- Going below ~17 ms requires overlapping the OTOS transaction with a
  motor settle window (legal — deadlines are per-device). That is what
  `clasi/issues/spike-i2c-bus-owning-fiber.md` would buy, ~4 ms.

Encoder integrity (position advancing, nonzero velocity) held in EVERY
variant, including F where the duty-write throttle was down to 7 ms.
Note the health flags are useless as a signal here: `kFlagFaultI2CSafetyNet`
(bit 6) and `kFlagFaultWedgeLatch` (bit 7) are already latched at boot on
a good robot, and only a boolean (`count > 0`) is on the wire, not the
count — so a violated window cannot be read off a flag. It has to be
checked functionally.
