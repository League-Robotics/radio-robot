---
id: "007"
title: "Stand HITL verification of the OTOS driver"
status: open
use-cases: [SUC-005, SUC-006, SUC-007]
depends-on: ["006"]
github-issue: ""
issue: nezha-hardware-otos-driver-for-new-source-tree.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Stand HITL verification of the OTOS driver

## Description

Final ticket in the OTOS driver set. Depends on ticket 006 (the leaf +
`NezhaHardware` wiring). Closes the parent issue
(`nezha-hardware-otos-driver-for-new-source-tree.md`).

**Hardware dependency (flagged explicitly, not discovered mid-execution):
this ticket requires the physical SparkFun OTOS sensor connected to the
robot under test.** If unavailable at execution time, tickets 005/006 can
still be complete and reviewed on their own merits (host-testable against a
scripted `I2CBus` fake) — only this ticket's HITL gate blocks on hardware
availability. If the sensor is unavailable, report that explicitly rather
than skipping the gate silently; do not claim this ticket is done without
it.

## Acceptance Criteria

- [ ] `NezhaHardware::odometer()` confirmed returning a non-null,
      OTOS-backed leaf on the deployed build.
- [ ] On the stand: OTOS position and velocity reads change plausibly
      (correct sign/magnitude) as the robot or a wheel is moved by hand.
- [ ] `TLM`'s `pose=`/`otos=` fields are live (not `ERR nodev`/absent) on
      real hardware.
- [ ] All seven OTOS wire verbs (`OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/`OA`) ack
      `OK` against the real robot, matching `docs/protocol-v2.md` §11's
      documented reply shapes.
- [ ] A pure spin-in-place on the stand produces bounded residual
      translation (the lever-arm compensation is working correctly with a
      same-instant heading) — not a large phantom offset (the historical
      ~433 mm `db11b7c` failure mode).
- [ ] `OL`/`OA` read back the values the leaf was configured with, matching
      `otos_commands.cpp`'s existing shadow-read contract.
- [ ] The hardware-bench gate's "OTOS alive" check
      (`.claude/rules/hardware-bench-testing.md`) passes.
- [ ] The parent issue is closeable.

## Implementation Plan

**Approach**: Deploy the ticket-006 build to the robot (`mbdeploy
deploy --build` or the project's current deploy path per
`.claude/rules/hardware-bench-testing.md`), open a serial/relay session,
and work through the acceptance criteria above in order: confirm liveness
first (verbs ack), then plausibility (values change correctly), then the
lever-arm-specific spin-in-place check last (the most sensitive check for a
regression of the `db11b7c` failure mode).

**Files to create/modify**: None expected — this is a verification-only
ticket. If the stand pass surfaces a genuine driver bug ticket 006's
sim-only testing couldn't catch (e.g., a real-hardware-only I2C timing
issue), fix it here and document the deviation explicitly, per the
architecture doc's own "verification may find something sim couldn't"
allowance.

**Testing plan**: The stand HITL pass itself, per the acceptance criteria
above, following `.claude/rules/hardware-bench-testing.md`'s standing
verification gate (sensors alive, this being an "OTOS alive" check
specifically).

**Documentation updates**: Close the parent issue file. Record the stand
session's observations (plausible values, spin-in-place residual, verb
acks) in this ticket's completion notes for traceability.

## 007 HITL-fix — real-hardware bug found and fixed

**This is exactly the "verification may find something sim couldn't"
deviation the Implementation Plan above allows for.** The team-lead's stand
HITL pass (pyOCD/gdb attached to the deployed ticket-006 build) found the
robot barely responsive over USB and dead over the radio relay. Root-caused
on real hardware, not from sim/theory:

**Root cause.** `dev_loop.cpp` calls `Hal::OtosOdometer::tick()`
unconditionally every main-loop pass (~470 Hz observed). Ticket 006's
`tick()` issued 4 I2C transactions per call (two `readXYH()` bursts — pos
then vel, each a register-select write + a 6-byte read), and — despite the
leaf's own header documenting an intent to reuse `I2CBus`'s
preClear/postClear clearance mechanism — every one of those 4 transactions
was issued with the default `(preClear=0, postClear=0)`, i.e. no clearance
at all. 4 out of 4 gdb halts during the stand campaign caught the main loop
parked inside vendor CODAL's `NRF52I2C::waitForStop()`
(`libraries/codal-nrf52/source/NRF52I2C.cpp`), called from
`I2CBus::read(address=0x2E, len=6, preClear=0, postClear=0)` <-
`OtosOdometer::readXYH` <- `OtosOdometer::tick`. When it stalls, the ENTIRE
main loop freezes — motors, comms, and radio alike — which is why the robot
was unresponsive. This is the exact same CODAL TWIM-errata stall class that
sprint 079-006 (`nezha_motor.cpp`) already root-caused and fixed on the
Nezha motor path; the OTOS leaf simply hadn't received the same treatment
yet.

**The fix** (`source/hal/otos/otos_odometer.{h,cpp}`), three parts, all
inside the OTOS leaf — no other files touched:

1. **I2C clearance on every OTOS bus transaction.** Added
   `static constexpr uint32_t kBusClearance = 4000; // [us]` (mirrors
   079-006's proven Nezha-path value exactly). Every `bus_.write()`/
   `bus_.read()` call in the leaf (`readReg8`, `writeReg8`,
   `readPositionVelocity`, `writeXYH`) now passes it: a register-select
   write carries `postClear=kBusClearance`, the read that follows it
   carries `preClear=kBusClearance`; standalone writes carry
   `postClear=kBusClearance`.
2. **Combined the pos+vel read into ONE 12-byte burst.** `kRegPositionXl`
   (0x20, 6 bytes) and `kRegVelocityXl` (0x26, 6 bytes) are contiguous
   registers (confirmed against `source_old/hal/real/OtosSensor.cpp` — no
   documented hardware reason they must stay separate; the two-method split
   there was an interface-design choice, not a bus constraint). Replaced
   `tick()`'s two separate `readXYH()` calls with one register-select write
   to `kRegPositionXl` + a single 12-byte auto-increment read
   (`OtosOdometer::readPositionVelocity()`), parsing bytes `[0..5]` as
   position and `[6..11]` as velocity. Halves the transaction count (4 -> 2
   per real read) and thus the per-tick clearance cost. `readXYH()` is
   deleted (would have been dead code — its only caller was `tick()`).
3. **Rate-limited `tick()`'s bus read.** Added
   `static constexpr uint32_t kReadPeriod = 20; // [ms]` (~50 Hz, ample for
   pose fusion) plus `lastReadMs_`/`hasRead_` members. A `tick()` call that
   arrives sooner than `kReadPeriod` since the last REAL bus read is now a
   no-op on the bus; it marks `cachedPose_.stamp.valid = false` so
   `Subsystems::PoseEstimator::tick()` skips fusing a non-fresh sample
   (avoids over-weighting the EKF by re-fusing the same reading every
   ~2 ms main-loop pass). `tick()`'s parameter was also renamed
   `nowMs` -> `now` (with `// [ms]` moved to a comment) — a pre-existing
   naming-convention violation in the file this change touched anyway (see
   `.claude/rules/coding-standards.md`).

**Tests updated** (`tests/sim/unit/otos_odometer_harness.cpp`,
`test_otos_odometer.py` unchanged — it just compiles+runs the harness):
replaced the `scriptXYH()` (6-byte) script helper with `scriptPosVel()`
(12-byte, matching the new combined burst) and updated every scenario that
scripted position+velocity reads (scenarios 4, 5, 6) to script one combined
read instead of two. Added scenario 8
(`scenarioTickRateLimitsBusReads`) asserting: a first `tick()` always reads
(bus traffic + `stamp.valid`); a second `tick()` inside `kReadPeriod`
issues **zero** further bus traffic and marks the sample stale; a third
`tick()` at/after the period boundary reads again (exactly one write + one
12-byte read). All 8 scenarios pass. Full host suite: `uv run python -m
pytest` — **620 passed**, matching ticket 006's baseline exactly (no
regressions).

**Files touched**: `source/hal/otos/otos_odometer.h`,
`source/hal/otos/otos_odometer.cpp`,
`tests/sim/unit/otos_odometer_harness.cpp`. `source/dev_loop.cpp` and
`source/commands/otos_commands.{h,cpp}` were NOT touched (confirmed via
`git diff --stat` — empty).

**Not done here** (team-lead owns this per the ticket's Do-Not list):
firmware build/flash and the actual stand re-verification against the
acceptance criteria above (sensors alive, encoders/verbs, spin-in-place
residual, radio recovery). The acceptance checkboxes above are left
unchecked pending that hardware pass. One thing worth the team-lead's
attention on the stand: this fix keeps `kBusClearance` at the same 4000us
value as the Nezha path, applied to a device whose reads are now
rate-limited to every 20 ms — bus load from this leaf alone should be
lighter than before (2 transactions every 20ms instead of 4 every ~2ms), but
worth watching I2C bus utilization if OTOS reads interleave with Nezha
flip-flop traffic on the shared bus during a real drive.
