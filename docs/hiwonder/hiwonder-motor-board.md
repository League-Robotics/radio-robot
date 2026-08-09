# HiWonder 4-Channel Encoder Motor Board — Driver Documentation

The reference documentation for `Devices::MotorBoard`,
`Devices::HiwonderBoard`, and `Devices::BoardMotor`
(`src/firm/devices/motor_board.h`, `hiwonder_board.{h,cpp}`,
`board_motor.{h,cpp}`). The code's comments cite the section numbers
here; this file carries the evidence and the reasoning.

Everything below was characterized on the bench rig **vogop** (JGB37-520
motor pair, 2026-08-02) during the `hiwonder-spike` branch campaign.
**Every number is measured unless marked *vendor*.** The
characterization scripts and raw datasets live on that branch under
`hwspike/` — including the latch-test `L` verb in its characterization
firmware, the profile-tracking suite, and all plots. Source provenance —
which claims are vendor-corroborated, which are third-party, which are
ours alone — is itemized in [`provenance.md`](provenance.md); a
BSD-licensed copy of the PX4 `HiwonderEMM` driver, the only independent
implementation found, is vendored in
[`px4-hiwonder-emm/`](px4-hiwonder-emm/). **There is no vendor
datasheet**: Hiwonder's protocol documentation is the sample code in
their chassis tutorials (see provenance for URLs), and this file is
deliberately self-sufficient so the driver never depends on those pages
staying up.

---

## 1. The hardware

A coprocessor motor board: STM32-class MCU at **I2C address 0x34**, four
DC motor channels with quadrature encoders, running **its own onboard
closed-loop speed control**. The host never sees a PWM pin or an encoder
edge — it writes setpoints and reads totals over I2C.

### 1.1 Register map

Verified against Hiwonder's own tutorial code and the PX4 driver, and
exercised on the bench:

| reg | dir | format | meaning |
|---|---|---|---|
| `0x00` | R | u16 **LE** | battery ADC — **reads directly in millivolts** (8053 measured against a nominal 7.4 V pack) |
| `0x14` | W | u8 | motor type; **3 = JGB** (magnetic encoder, 44 pulses/rev, ratio 131 default — *vendor*; our JGB37-520s accept it) |
| `0x15` | W | u8 | encoder polarity, 0 or 1 (0 for our wiring) |
| `0x1F` | W | 4× i8 | open-loop PWM, **vendor range −100..100**, **non-latching** (§1.3) |
| `0x33` | W | 4× i8 | closed-loop speed, unit **pulses per 10 ms** (*vendor, verbatim*), usable range "generally around ±50" (*vendor*) |
| `0x3C` | R | 4× i32 **LE** | cumulative encoder totals; auto-incrementing multi-byte read works (one 16-byte read returns all four) |

### 1.2 Command scales — the mistake to not repeat

- **`0x33` full scale is 50, not 127.** The unit is pulses per 10 ms and
  the ceiling for these motors is ~±50. Commanding 127 saturates
  silently — measured 5736 c/s = 57 pulses/10 ms, flat at the physical
  ceiling. An early driver used ±127 and every "tracking" number it
  produced was actually a saturation measurement. The PX4 driver maps
  [−128, 127] with no guard and would saturate the same way.
- **`0x1F` full scale is 100, not 127.** 127 is out of range — the motor
  barely turned.

### 1.3 Command persistence (latch behavior), measured exhaustively

Measured with a dedicated latch-test verb (write once / burst-N, then
encoder-reads-only, then TOTAL bus silence, then reads again):

- **Speed (`0x33`) latches absolutely while the motor is moving.** A
  wheel at 3117 c/s held full speed through 1 s of reads-only AND 2 s of
  total bus silence — zero decay. A setpoint CHANGE also takes with a
  single write: one half-value write at speed moved 3127 → 1564 c/s and
  held through 2 s of silence.
- **Starting from rest needs a write burst, not one write.** A single
  write (and even two, seconds apart) never starts the motor — zero
  counts. Burst sweep at 10 ms spacing: 1 write never, 2–5 flaky
  (gear-mesh/stiction randomness), 10 usually, **20+ (~200 ms)
  reliably**. Interpretation: the board only keeps trying to break away
  while writes keep arriving; once moving, the setpoint is sticky.
- **PWM (`0x1F`) is non-latching** — full speed to stopped within 1 s of
  command silence. Continuous sending required, as the vendor states.
- **There is no watchdog.** Latch + no watchdog has a hard safety
  corollary: **a host that crashes mid-motion leaves the robot driving
  at its last speed forever, and "stop sending" is never a stop.**
  Every stop path must actively write zeros; the loop's
  bounded-Move/estop discipline is the only deadman.

### 1.4 Command policy (what `HiwonderBoard::exchange()` implements)

From §1.3, the write policy is **write-on-change, with two mandatory
exceptions**:

1. any channel commanded nonzero whose encoder did not advance since the
   previous exchange keeps being written every cycle (this is the
   from-rest start burst, self-sustaining until motion confirms, and it
   also covers a stalled wheel);
2. the PWM plane, when selected, is written every cycle.

A failed write leaves the change pending, so it retries next cycle. A
steady cruise costs zero command writes; the encoder read (~0.6 ms) is
the whole per-cycle bus bill.

### 1.5 The encoder tick, and why velocity must not be naively differenced

**The board updates its totals registers on its OWN ~9.56 ms tick** —
measured as 38 counts/tick over a 3975 c/s plateau, cross-checked as 323
updates / 3.09 s = 9.57 ms. NOT the 10.00 ms the speed unit implies, and
not synchronized to anything the host does. A host reading every 10 ms
beats against it: every ~220 ms one read interval spans two board
updates, and a naive Δcounts/Δt reports a +100% velocity spike of
physically impossible motion (+190,000 c/s² against a measured
achievable ~16,000). At 200 Hz reads the same physics shows up as
0/2×/0/2× alternation. Retiming the loop cannot fix it — the tick is
per-board (the Yahboom sibling measures exactly 10.00 ms) and
plausibly temperature-dependent; matching only lengthens the beat.

The estimator that works (validated against ground truth: 14/14 known
double-reads found, zero false positives through accel and decel ramps,
plateau velocity sd 792 → 100 c/s, residual = genuine count
quantization):

> On each fresh delta, compare its magnitude to the **windowed median**
> of recent per-tick magnitudes (ring of 9). If it exceeds **1.6×** the
> median AND is at least **15 counts**, it is one read that spanned two
> board ticks: **keep the counts, credit TWO tick periods** —
> `v = Δcounts / (n_ticks × kEncoderTick)`.

Each guard exists because its absence was observed to fail: the median
(not mean) because the mean is dragged by the outlier being hunted; the
15-count floor because near zero speed a tick is 1–3 counts and ±1 count
of quantization legitimately doubles a delta; the ring stores
`mag/n_ticks` so a split double cannot inflate the median it is compared
against next; the board's update count is the timebase (halving the
counts instead would corrupt anything integrating the stream — position
through a spike is a clean ±1-tick sawtooth, zero extra distance).

### 1.6 Plant behavior (for whatever plans motion on top)

- Commanding 0 is a **coast** (~390 ms, ~780 counts from a 4000 c/s
  plateau); an active reverse command of ~0.30 stops promptly without
  reversing.
- Spin-up time constant ≈ **145 ms** (position-deficit method, ±2 ms
  over 5 repeats).
- The onboard loop tracks a trapezoid down-ramp acceptably; with lead
  shaping `u = v_ref + tau·a_ref`, `tau_up = tau_dn = 150 ms`,
  delivered/commanded distance = 0.998 and the post-profile tail drops
  from 103 to 4 counts. (Lead shaping belongs in the motion layer, not
  this driver.)
- **Supply floor:** 6.4–8.4 V per the product spec. A pack at ~2.9 V
  produced a live I2C device with dead motors — "acks but won't move"
  means read register `0x00` before debugging anything else.
- Bus cost: one command write + one 16-byte totals read ≈ **0.8 ms per
  cycle at 400 kHz** (~0.6 ms on read-only cycles). The micro:bit
  default 100 kHz triples it — set 400 kHz.
- 100 Hz is the natural loop rate: it matches the board's ~10 ms sensor
  bandwidth. Faster reads add no information (§1.5).

---

## 2. Class design

```
Drive / RobotLoop            (unchanged)
     │  Devices::Motor        per-wheel interface (unchanged)
     ▼
Devices::BoardMotor           one channel of ANY MotorBoard as a Motor
     │  Devices::MotorBoard   board-level interface
     ▼
Devices::HiwonderBoard        the 0x34 register protocol + §1.4 policy
     │  Devices::I2CBus       existing pure bus interface
     ▼
MicroBitI2CBus / SimPlant     real or simulated wire
```

The `MotorBoard` seam exists because these coprocessor boards are
**whole-board** devices: one I2C frame commands all four channels, one
read returns all four totals. A per-motor driver would pay the bus cost
four times or need back-channel coordination. The board owns the
exchange; per-motor `BoardMotor` objects are cheap views. The
`exchangeOncePerCycle()` guard in the base makes the first
`BoardMotor::tick()` of a cycle pay the bus while the rest read the
cache.

Normalized `float` fractions ([-1..1]) cross the seam; each board owns
its native encoding. This property let identical characterization
firmware drive this board and the wildly different Yahboom quad
(int16 BE mm/s) unmodified — keep it.

`BoardMotor` details worth knowing:

- `setDuty(duty)` forwards `duty × fwdSign` to `stageSpeed()`. The
  "duty" the drivetrain computes IS the normalized fraction; Drive's
  calibrated velocity→duty map stays above this seam. Note the board's
  own closed loop makes duty↔speed much closer to linear than the Nezha
  plant — Nezha-tuned trim/PID gains are wrong here.
- `requestSample()` is a no-op — no split-phase encoder latch exists on
  this board.
- `resetPosition()`/`rebaseline()` are the same operation: fold the
  current total into a software offset. The board's counters are never
  reset (project convention: store wide, subtract an offset).
  `rebaseline()` must bring `position()` back inside the wire bound —
  `robot_loop::publishWheel` calls it and reads position right back.
- **The `begin()` trap:** `begin()` runs before the first bus exchange,
  so the board total is still zero. Capturing the offset there leaves
  the board's large cumulative total in the reported position — on the
  bench it pinned at the ±32000 mm wire bound within one session. A
  `zeroPending_` flag defers the zero to the first `tick()` with real
  data.

---

## 3. Wiring instructions (currently NOT wired in)

The classes are in the tree but nothing constructs them. State of the
build today:

- **Firmware:** the top-level `CMakeLists.txt` recursively globs
  `src/firm/**/*.cpp` (`RECURSIVE_FIND_FILE(SOURCE_FILES ...)`), so
  `hiwonder_board.cpp` and `board_motor.cpp` are **already compiled and
  linked** into the firmware image as dead code. No CMake change is
  needed for the firmware build; the cost is a little flash.
- **Sim/tests:** `src/firm/platform/host/CMakeLists.txt` lists sources explicitly, as
  do the per-test translation-unit lists in `src/tests/sim/unit/*.py`
  (grep `nezha_motor.cpp` for the pattern). The new files are in none
  of them — the sim is untouched.

To wire the board in as the drivetrain:

1. **Composition root** (`src/firm/main.cpp`): construct one board and
   one `BoardMotor` per wheel where the `NezhaMotor` leaves are built
   today —

   ```cpp
   static Devices::HiwonderBoard motorBoard(i2cBus);
   static Devices::BoardMotor leftMotor(motorBoard, 0, leftConfig);
   static Devices::BoardMotor rightMotor(motorBoard, 1, rightConfig);
   // motorBoard.init() alongside the other device init calls; false
   // means the board never acked -- treat as board-absent, same as any
   // other device.
   ```

   Nothing above `Devices::Motor` changes — `Drive`, `RobotLoop`,
   telemetry, and config keep their existing wiring.
2. **Bus speed:** `uBit.i2c.setFrequency(400000)` in the composition
   root (§1.6).
3. **Loop period:** consider 10 ms (§1.6). The driver is correct at any
   period; faster buys nothing.
4. **Robot config:** `wheelTravelCalib` is **mm per count** on this
   board. A Nezha-era mm/deg value is the wrong scale entirely and
   produces garbage positions until recalibrated. Measured plateau
   scale on vogop: ~5250–5350 counts/s at command 0.75 (~7000 c/s full
   scale). Zero the Nezha-tuned trim gains (§2, duty map note).
5. **Sim:** when a simulated variant is wanted, add the two `.cpp`s to
   `src/firm/platform/host/CMakeLists.txt`'s explicit list and implement the 0x34
   register protocol in `TestSim::SimPlant` — same wire-level approach
   as the other simulated devices.
6. **Safety check before first drive:** verify every stop path
   (`estop`, fence, Ctrl-C) actively writes zero speeds — §1.3's
   latch + no-watchdog corollary. Silence never stops this board.
7. **On-stand verification:** the standing bench gate applies
   (`.claude/rules/hardware-bench-testing.md`). Additionally verify the
   in-class velocity estimator with a steady plateau capture — the
   estimator's *rule* is validated on captured data (§1.5), but its
   in-class output should get its own plateau-sd check on the stand.

## 4. Known limitations

- `pwmMode_` is board-global because the register format is — closed-
  loop and PWM channels cannot mix on one board.
- No watchdog on the board (§1.3): host-side motion bounding is the only
  safety.
- `kEncoderTick = 9.56 ms` was measured on ONE board at bench
  temperature. It only scales the velocity estimate (a few percent error
  scales velocity by the same few percent); if that matters, measure per
  board (updates/second over a long plateau) and move it to the robot
  JSON.
- The JGB37-520 pair on vogop showed ~1% left/right scale mismatch —
  normal; handled by the existing per-wheel calibration.
- The start burst means a from-rest start has up to ~200 ms of
  command-write traffic before motion confirms (§1.4) — by design, not
  a bug.
