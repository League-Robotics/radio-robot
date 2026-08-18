---
status: pending
---

# Spike: move I2C into a bus-owning fiber — settle a 3-way open question

## Description

Stakeholder proposal (2026-08-07): put the Nezha's I2C discipline into its
own **bus-owning fiber**. The `I2CBus` interface would post read/write
requests carrying their own pre-guard and post-guard times; the fiber
services them on a fixed cycle (`select → guard → collect → perception
slot → publish`), servicing foreign requests only in its own slack.
Everything else runs in other fibers and is structurally incapable of
touching the bus. A robot/hardware class would own the bus interface and
declare its background processes to the core, which starts them.

Motivation: generalize the hardware abstraction. The Nezha's bus timing
currently dictates the shape of the whole main loop, which makes adding a
second robot family awkward. Related context in
`clasi/issues/kernel-packaging-host-sim-rigor-and-hardware-abstraction-program-plan.md`.

**This issue is scoped as a SPIKE, not a commitment** — because the repo
already contains three mutually incompatible answers to this exact
question, and four load-bearing facts are unmeasured.

### The three conflicting answers already in the repo

| Document | Answer | Status |
|---|---|---|
| `clasi/sprints/done/014-single-cooperative-main-loop-abandon-fibers/architecture-update.md:294-315` | Rejects "a third I2C-bus-owner fiber" as *fragile*; one loop makes bus atomicity structural | Shipped, live today |
| `clasi/issues/done/device-bus-fiber-owned-self-contained-device-subsystem.md` (400 lines) | **This proposal**, already designed to sprint-ready depth: fiber cycle sketch (:238-251), 4-rule concurrency contract (:278-291), `MeasurementRing` (:159-194), 6 bench gates (:331-345) | **Never implemented** |
| The kernel-packaging program plan | **Zero mentions of "fiber"** (`grep -ic fiber` → 0). Keeps ONE loop; moves the *schedule* into `RobotHardware::cycle(CycleServices&)` as a "literal transcription" of today's sequence | Proposed |

Sprint 014's rejection rested on a cost that **has since decayed**: an
8 ms *busy-wait* per encoder read, chosen so the scheduler could not
dispatch a competing I2C write mid-transaction. Clearance now **yields**
via `fiber_sleep()` (`microbit_i2c_bus.cpp:139-156`), so that objection is
obsolete — but the decision was never revisited. That is the gap this
spike closes.

## Cause

### The "encoders wedge if read too fast" premise is measurably false

`docs/design/encoder-refresh-characterization.md` (MEASURED 2026-07-26 on
the stand; apparatus `src/tests/firmware/encoder_rate/`) retired the
"~80 ms register refresh" folklore: the 0x46 register returned a fresh
value on **every single poll at ~16 ms across 6 phases** (1250/1251 in
phase A1), with no latch quantization. The real mechanisms:

| # | Mechanism | Measured |
|---|---|---|
| 1 | **Interposed-traffic sample invalidation** — any 0x10 transaction between a 0x46 select and its read destroys the pending sample | **416/416** (phase F) |
| 2 | **Reversal write train** latches the readback | 150 ms clean, 50 ms clean, **20 ms fails 12/12**; threshold in (20, 50] ms |
| 3 | **No clearance at all** → TWIM `waitForStop()` busy-spins toward its own ~10 s timeout, freezing the whole loop incl. serial | pyOCD backtrace, `docs/knowledge/2026-07-04-encoder-wedge.md:253-278` |

So the constraint is **≥4 ms spacing between 0x10 transactions, plus no
0x10 traffic inside a select→read window** — per-transaction spacing and
ordering, *not* a poll rate.

**This reframing favours the proposal.** The spacing half is already
enforced inside the bus object: `preClear`/`postClear` [us] on
`I2CBus::read/write` (`src/firm/devices/i2c_bus.h:36-41`), a lazy
per-device `readyAt` deadline, a non-spinning `fiber_sleep()` catch-up,
and a trip counter surfaced as telemetry fault bit 0. **The
per-transaction pre/post guard the proposal describes already exists —
build on it, do not rebuild it.** What the single loop uniquely provides
is **ordering**, which the bus does *not* enforce. A bus-owning fiber
would provide ordering structurally. That is the actual prize.

### The regression net does not cover what this would break

| Property | Covered? | Where |
|---|---|---|
| Loop **shape** (4 sleeps/cycle, 0 direct yields, trailing sleep == kCycle) | **Yes, strong** | `src/tests/sim/system/sim_api_harness.cpp:359-401` |
| Delivered **mean period** converges to kCycle under jitter | **Yes, strong** | `src/tests/sim/unit/app_robot_loop_pacing_harness.cpp` (`JitterySleeper`, 200 cycles) |
| `preClear`/`postClear` values | **No** — zero test hits repo-wide | — |
| I2C transaction **order** | **No** — mechanism exists unused | `src/tests/sim/unit/scripted_i2c_hook.h` |
| Wedge from a **timing cause** | **No** — every wedge test injects a frozen encoder by hand; tests the *detector* | `src/tests/sim/system/faults/fault_knobs_harness.cpp` |
| Anything timing-related in sim | **Cannot** — `SimPlant` discards `preClear`/`postClear` (`sim_plant.cpp:105-110`) and hardcodes `clearanceSafetyNetCount() → 0` (`sim_plant.h:98`) | — |

**If a refactor breaks the 4 ms spacing or interposes a write inside a
select window, every automated test still passes.** Additionally,
131-005's absolute-deadline pacing fix was **never bench-verified**
(`robot_loop.h:75-78`, `data/robots/tovez.json:114`) — the delivered
period today is unknown (50 or 54 ms), so the baseline any experiment
would be judged against does not yet exist.

## Proposed fix

Staged. Decisions already taken: **spike to decide** (not commit),
**tovez only** as the rig (gopiv has motors but no OTOS/line/color —
measured `otosPresent=0` — so it physically cannot exercise bus
contention), **build the safety net before touching the loop**.

### Stage A — build the safety net (no loop changes)

**A1. The I2C ordering test.** `src/tests/sim/unit/scripted_i2c_hook.h`
(174 lines) already FIFO-scripts exact per-call address expectations and
returns `kScriptMismatch = -100` on an unscripted/wrong-address call.
Write the test nobody has written: assert **no 0x10 transaction occurs
between a 0x46 select and its read**, for each motor, plus the full
per-cycle address sequence. Highest-leverage artifact here — the only
thing that can fail on the exact hazard the fiber is meant to kill.

**A2. Hardware baseline on tovez.** Record median `cycle_period`,
`cycle_busy`, `i2c_safety_net` (fault bit 0) and `wedge_latch` over a real
run: `src/tests/bench/planner_square_tour.py` plus
`relay_telemetry_rate.py:269-307` (already reports period/Hz, gates
nothing). Closes the 131-005 open follow-up as a side effect. **Note:**
first flash of tovez after the `DEVICE_BLE=0` change (commit `608cc885`)
needs a one-time CTRL-AP mass erase — expected, `mbdeploy` self-recovers.

**A3 (optional, larger).** Teach `SimPlant` to honour `preClear`/
`postClear` and to invalidate a pending sample on interposed 0x10
traffic, so sim can fail on a spacing/ordering violation. Turns A1 from
an ordering check into a genuine timing check. Own decision — real work,
and it overlaps the program plan's Phase 4 sim-rigor scope.

### Stage B — measure the four unknowns that gate the design

**B1. Does `NRF52I2C::waitForStop()` yield? Desk work, gates everything —
do it first.** Read `libraries/codal-nrf52/source/NRF52I2C.cpp`. The
pyOCD evidence says it *busy-spins* toward a ~10 s timeout and froze the
entire loop including serial. **If it does not yield, a stalled bus fiber
still takes the robot down and the fiber buys no isolation — abort the
experiment at this point.** No hardware required.

**B2.** Extend `src/tests/firmware/encoder_rate/` (the only apparatus
that has ever characterized this hardware; separate CODAL project,
requires a reflash cycle) to measure on tovez:
- **Other-address traffic inside a pending select→read window** — sprint
  014's own open question 1 (`architecture-update.md:357-363`), never
  answered. The load-bearing unknown for letting any non-motor device use
  the bus while a motor read is pending.
- **Two per-port selects pending simultaneously** — DeviceBus gate 1;
  only "one pending, last wins" is established
  (`encoder-refresh-characterization.md:55-61`).
- **`fiber_sleep(4)` latency histogram** — DeviceBus gate 2. With
  `SCHEDULER_TICK_PERIOD_US: 1000` (`codal.json`) it *should* be 4-5 ms,
  but that is inference from the CODAL tick model, not measurement.

### Stage C — the fiber spike (only if B1 passes)

Branch, never master. Behind a build flag so both paths coexist in one
firmware and can be A/B'd back-to-back on the same robot.

Reuse rather than reinvent:
- **Design**: `clasi/issues/done/device-bus-fiber-owned-self-contained-device-subsystem.md`
  — cycle sketch (:238-251), concurrency contract (:278-291), ring buffer
  (:159-194). Also read its **rejected** timer-ISR alternative (:293-315)
  so it is not re-proposed.
- **Seams that already exist for exactly this**: `Devices::Clock`/
  `Sleeper` (`devices/clock.h`, already documented as "the fiber-level
  time seam"); `MicroBitI2CBus::clear(addr7)` — a non-spinning readiness
  peek with **zero callers today**; the `inUse_` re-entrancy counter,
  which exists to catch the bug a second bus-touching fiber would
  introduce.

**Architectural commitment this breaks, consciously:**
`src/firm/app/DESIGN.md:766-772` states *"`grep 'runAndWait\|sleepUntil'
app/robot_loop.cpp` must remain the firmware's complete list of waits."*
A bus fiber creates a second, independent list. Either revise that
invariant deliberately or make the fiber's waits enumerable the same way
— do not let it erode silently. Same file at :841-849 argues the
converse position (fibers hide the bus schedule and make both hard
realtime problems undebuggable); that argument deserves an explicit
answer, not a bypass.

### Decision gate

| Verdict | Criteria |
|---|---|
| **Fiber wins** | Ordering hazard dies structurally (A1 passes by construction, not by scheduling); delivered period ≤ A2 baseline; `cycle_busy` **lower** (bus waits overlap compute); `i2c_safety_net == 0`; no wedge latch over a soak; encoder `sampleTime` advances every cycle |
| **Abort** | B1 says `waitForStop()` does not yield; or any wedge latch the baseline did not produce; or delivered period worse than baseline |

## Verification

- **A1**: `uv run python -m pytest src/tests/sim/unit/test_<new_ordering>.py`
- **Shape guards must stay green** — they are the "zero timing change"
  evidence: `src/tests/sim/system/test_sim_api.py`,
  `src/tests/sim/unit/test_app_robot_loop_pacing.py`
- **A2 / Stage C A-B**: same bench script before and after; compare
  median `cycle_period`, `cycle_busy`, and the fault bits. Soak with
  `src/tests/bench/move_soak.py` (already aborts on `wedge_latch` /
  `i2c_nak`).
- **Caveat**: the suite is **not currently all-green** — see
  `clasi/issues/sprint-135-pre-existing-test-failures-need-triage.md`,
  plus xfails at `src/tests/sim/unit/test_app_robot_loop.py:110` and in
  `test_angle_stop_rotation_calibration.py`. Record which failures are
  expected *before* using "tests pass" as a gate.

## Related

Adjacent defects and drift found while surveying this area. **Flag, do not
bundle** — each is its own change:

- **`cycleCount_` is stuck at 0, so the line sensor is NEVER ticked**
  (`robot_loop.h:327-337`; no `cycleCount_++` anywhere). Its 4 ms
  transaction is therefore **not** in today's measured period — any
  redesign that "fixes" the parity silently changes loop timing. Actual
  current cadence: OTOS 50 ms, color ~100 ms, line never. The program
  plan defers this deliberately for the same reason.
- **`src/firm/app/DESIGN.md:851-869` is materially stale**: claims `kPace`
  exists (deleted by 131-005), says `kCycle` is 40 (it is 50), and
  describes the trailing-block anchoring as the *opposite* of what the
  code now does. Anyone reading it to understand the loop will be misled.
- `lagLine` / `lagColor` are **not in any robot JSON** — not per-robot
  configurable despite `devices/device_config.h` implying otherwise.
- ~450 lines of dead `devices/` code (`hiwonder_board`, `motor_board`,
  `board_motor`) — zero callers; free deletion, and the program plan's
  Phase 6 wants the `hiwonder` name back.

Key references:
- `clasi/issues/kernel-packaging-host-sim-rigor-and-hardware-abstraction-program-plan.md`
- `clasi/issues/done/device-bus-fiber-owned-self-contained-device-subsystem.md`
- `clasi/sprints/done/014-single-cooperative-main-loop-abandon-fibers/architecture-update.md:294-315`
- `docs/design/encoder-refresh-characterization.md`
- `docs/knowledge/2026-07-04-encoder-wedge.md`
