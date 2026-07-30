# Craftsmanship + Correctness Review — `src/firm` (firmware base)

**Date:** 2026-07-30 · **Reviewer:** programmer-agent (review-only pass) ·
**Branch:** `sprint/127-host-side-path-planner-goto-and-path-following`
(WIP from a parallel session) · **Scope:** every `.cpp`/`.h` under
`src/firm` (~12.6k lines), read in full, plus the `Motion::WheelSink`
boundary from the firm side. Guidelines: `docs/code_review/GUIDELINES.md`.

## Verdict

**Merge-with-findings.** The great majority of this tree is genuinely
close to the "textbook" bar the guidelines set: `App::Telemetry`'s
single-assembly-point projection, `Devices::NezhaMotor`'s
never-latch-success-before-confirmed write path, `Devices::
MicroBitI2CBus`'s never-spin clearance wait, and `WireRuntime`'s
COBS+CRC codec are all disciplined, well-bounded, and paranoid about
the unhappy path in exactly the ways the guidelines ask for. The review
originally flagged one **CRITICAL** correctness defect sitting in the
composition root (`main.cpp` unconditionally shipping with a documented
"non-negotiable" hardware-errata guard turned OFF) — that finding is now
**RESOLVED by stakeholder decision** (2026-07-30): the guard stays off
deliberately, and the whole `irqGuard_` mechanism is being deleted, not
reinstated or gated. Finding 1 below is retained as the record of that
decision, not as an open defect. What remains open is a cluster of
**MAJOR** accretion/interface-bleed findings clustered around a separate
root cause: the firmware went through a real architecture change (the
on-robot `Motion::Planner` replacing `Motion::MoveQueue` as the loop's
motion decider) without walking back and updating either the
`Motion::WheelSink` boundary contract or the design docs that describe
it. None of this is exotic — every finding below has a concrete file:line
and a concrete failing scenario — but the `WheelSink` situation in
particular is exactly the "interface still standing, violated by its own
callers" pattern the guidelines call out as this codebase's most common
failure mode, and it should get a real go/no-go decision (declare
`RobotState.wheelLeft/wheelRight.cmdVelocity` the actual boundary and
delete `WheelSink`, or restore `WheelSink` as the real seam) rather than
staying in its current half-retired state.

---

## Findings

### 1. RESOLVED (stakeholder decision) — the I2C errata guard is deliberately off and is being deleted

**Category:** resolved

**Evidence:** `src/firm/main.cpp:201-213`:

```cpp
  // BENCH A/B, 2026-07-28 -- TEMPORARY, revert after the measurement.
  // ...
  // NOT a candidate for shipping as-is: the guard exists to suppress the
  // nRF52 TWIM errata (microbit_i2c_bus.h -- "under higher levels of
  // background interrupt load"), which is what produces encoder wedges.
  // Any run with this off must watch the wedge/bus-error counters, not
  // just the ping loss.
  bus.setIrqGuard(false);
```

`src/firm/devices/microbit_i2c_bus.h:136-142` documents `irqGuard_` as
"Default ON — non-negotiable (issue 'Armor stays intact')" and its whole
purpose as masking IRQs for the full I2C transaction to work around a
silicon errata that "is what produces encoder wedges." `main.cpp`'s own
comment agrees this is a temporary bench experiment ("NOT a candidate
for shipping as-is") — but the call is unconditional, with no `#ifdef`,
no config flag, and no reversion commit (`git log -S"setIrqGuard(false)"
-- src/firm/main.cpp` shows exactly one commit, `cc04ac84`, which added
it and never touched it again). **Every build of this firmware compiled
from this tree today boots with the errata workaround disabled.**

**Resolution (stakeholder decision, 2026-07-30):** "I think this
experiment succeeded, and we should have no IRQ guard. [...] Don't go
reverting this until we find a reason to turn it back on. If we're not
seeing any problems resulting from having it off, then we just remove
it. We just remove the whole system of having IRQ guards." The evidence
above is accurate and kept as history; the analysis that follows
corrects two things the original write-up got wrong about what the
guard does and what turning it off actually risks — see "Original
failing-scenario analysis, corrected" below.

**Original failing-scenario analysis, corrected:** the original write-up
argued that the guard being off "directly defeats the mechanism the rest
of the wedge-protection stack ... was built to compensate for," implying
`irqGuard_` was load-bearing for bus contention / encoder settling. It
is not, and the consequence was overstated on two counts:

1. **The guard was never about bus contention or encoder settling.**
   Three separate mechanisms live in `src/firm/devices/
   microbit_i2c_bus.cpp` and only one of them is the guard:
   - `waitForClearance()`'s per-device `preClear`/`postClear` timer
     (`write()` lines 61-65, `read()` lines 106-110) is what actually
     keeps traffic off the bus during the encoder settling period — and
     it deliberately runs BEFORE any masking ("never mask interrupts for
     a multi-ms wait").
   - The `inUse_` re-entrancy check-and-set masks only briefly, for a
     flag access.
   - `irqGuard_` masks the FULL transaction and is purely an nRF52 TWIM
     silicon-errata workaround (the STOPPED event failing to fire under
     background interrupt load, leaving `waitForStop()` spinning).
   Only the third of these is being deleted; `waitForClearance()` and the
   `inUse_` check are unaffected and keep doing what they always did.

2. **The unbounded-runaway chain the scenario implicitly invoked no
   longer exists.** That chain depended on the pre-rebuild `source/`
   tree's `D` distance-stop, which was deleted in the sprint 115
   gut-to-minimal rebuild. Under protocol v4/v5, every `MOVE` carries a
   mandatory `timeout` (`timeout <= 0` → `ERR_BADARG`), so a wedged
   encoder can no longer produce an unbounded run, and
   `Devices::MotorArmor::wedged()` still detects the wedge itself. The
   failure this finding worried about is detected and bounded, not
   silently compensated-for-then-defeated.

**The cost that tips the trade:** the guard costs roughly 7-8% inbound
command loss on direct USB (the nRF52 serial RX is DMA-driven, so bytes
arriving in a masked window are lost). Guard-off has now shipped for two
days across sprint 126's full camera-verified OTOS calibration campaign
(18 straight runs, 24 turns on the playfield) and all of sprint 127's
bench work, with no runaway observed. Paying a large, certain, permanent
command-loss cost to lower the odds of a fault that is now detected and
bounded is a bad trade — hence the decision to remove the guard rather
than gate or revert it.

Anyone tempted to reinstate the guard on the strength of the original
runaway report must first account for the mandatory `MOVE` timeout
backstop above — the mechanism that report's failure mode depended on no
longer exists.

**Design answer:** the whole `irqGuard_` mechanism is being deleted —
`setIrqGuard()`, `irqGuard()`, the `irqGuard_` member, the conditional
masking in `microbit_i2c_bus.cpp`, and the `main.cpp:213` call site —
while `waitForClearance()` and the `inUse_` re-entrancy check are
**kept**, unchanged. Tracked in
`clasi/issues/make-irq-guard-off-permanent-and-reconcile-the-docs.md`.
This document is not touching source; the deletion itself is later
sprint work.

One genuinely open sub-question, explicitly **not** a reason to keep the
guard: telemetry flags bit 7 (`kFlagFaultWedgeLatch`) reads set on the
live robot and deserves investigation on its own merits — with the
precedent that bit 6 in the same word is a documented always-on false
alarm whose "one-shot latch" story was falsified by direct on-chip
measurement (`src/firm/app/telemetry.h:76-122`). If bit 7 turns out to
be a similar false alarm, that is a telemetry-decode fix, not a reason
to reinstate `irqGuard_`.

---

### 2. MAJOR — `Motion::WheelSink` is a dead interface on the live motion path; the real base/motion actuation crossing is an undocumented `RobotState` field

**Category:** interface-bleed / accretion

**Evidence:** `src/firm/app/drive.h:125-132`:

```cpp
  // --- Motion::WheelSink (legacy boundary) ---
  // The velocity-sink interface Motion::MoveQueue drives. RobotLoop no
  // longer routes anything through it -- the live path is command()/tick()/
  // update() above -- but the interface is still implemented so a MoveQueue
  // -era harness keeps compiling. setDuty() stages targets with no deadline
  // and no ownership claim; nothing in the live loop calls either method.
  void setDuty(float left, float right) override;  // [mm/s] [mm/s] velocity targets
  void stop() override;                            // == estop()
```

Confirmed from the other side: `src/motion/planner/planner.cpp:1230-1233`
— `Motion::Planner::update()` (the actual on-robot motion decider,
wired into `main.cpp:434` and `robot_loop.cpp`'s `planner_` member)
writes straight into the blackboard —

```cpp
  const float stagedLeft = cmdLeft_ + trimLeft_;    // [mm/s]
  const float stagedRight = cmdRight_ + trimRight_;  // [mm/s]
  state.wheelLeft.cmdVelocity = stagedLeft;
  state.wheelRight.cmdVelocity = stagedRight;
```

— never calling `WheelSink::setDuty()`/`stop()` at all. `App::Drive`'s
own `Motion::WheelSink` overrides (`drive.cpp:31-36`) are, by the class's
own doc comment, called by nothing in the live loop. This is exactly the
guidelines' "interface still present, still exported, but violated by
its own callers" pattern (§1, §3): CLAUDE.md and `docs/design/design.md`
both describe `Motion::WheelSink` as *the* one narrow, named seam between
`src/firm` and `src/motion` ("the boundary interface pattern we use...
one narrow, named seam, defined by the lower layer, implemented by the
upper"). In the code as it stands, the actual crossing is an implicit
contract through two plain floats on `Types::RobotState`
(`wheelLeft.cmdVelocity`/`wheelRight.cmdVelocity`, `src/firm/types/
robot_state.h:124`), written by whichever of `Motion::Planner::update()`
or `App::Drive::update()` currently "owns" motion (`robot_loop.cpp:591-
599`'s ordering comment), and consumed by `RobotLoop::cycle()`'s single
`drive_.tick(state_.wheelLeft.cmdVelocity, ...)` call
(`robot_loop.cpp:499`) — not by `WheelSink` at all.

**Design answer:** this is a real design decision waiting to be made
explicit, not merely a cleanup: either (a) `Types::RobotState::Wheel::
cmdVelocity` is promoted to be the documented actuation boundary (in
which case `Motion::WheelSink`, `Drive`'s override of it, and every
design doc describing `WheelSink` as the seam should be deleted/rewritten
together), or (b) `Motion::Planner` is changed to drive `Drive` through
`WheelSink` as designed (in which case the current direct-to-blackboard
write is the accretion to remove). Leaving both in place — one dead, one
live, both still compiling — is the "two divergent paths, pick one"
condemnation the guidelines name explicitly.

---

### 3. MAJOR — design docs describe an architecture the code no longer has

**Category:** accretion

**Evidence:** `docs/design/design.md` §5 and `src/firm/app/DESIGN.md`
(lines 25-33, 392-397, 1113-1172) both describe `Motion::MoveQueue` as
the firmware's motion decider and `App::Drive::setDuty()`/`stop()` as the
live actuation surface Drive exists to provide, with `Motion::WheelSink`
as the one seam in active use. The code that actually ships
(`src/firm/app/robot_loop.h:39`, `main.cpp:434`, `src/motion/planner/
planner.h`) has moved wholesale to `Motion::Planner` — a different,
larger class (`move()`/`plannedStop()`/`estop()`/`tick()`/`update()`/
`applyVelGains()`/`applyShaperLimits()`, `src/motion/planner/planner.h:30-
70`) — and `App::Drive` has regained a full second lifecycle
(`command()`/`estop()`/`owns()`/`takeCompletion()`/`update()` for the
`WHEELS` teleop primitive, `drive.h:83-123`) that the design docs do not
mention at all. Per GUIDELINES §3, "fixes that don't update the design
docs" are themselves accretion: `docs/design/design.md`'s own text says
sprint 122 "moves `Motion::MoveQueue`/... out of `src/firm/app/`" and
that `App::Drive` implements `WheelSink` as its narrowed contract — both
statements are now false of the code in this tree.

**Design answer:** the `motion-planner` merge (commit `f6dbf598`, "planner:
full firmware integration -- onboard Motion::Planner replaces the interim
Drive velocity PID as the loop's motion decider") needs its own
architecture-update pass through `docs/design/design.md` §5 and
`src/firm/app/DESIGN.md`, folding in the `WheelSink` decision from
finding #2 at the same time — both documents currently describe a
retired shape.

---

### 4. MAJOR — `PlannerLimits` tuning is hardcoded in the composition root, bypassing the project's own config-as-truth convention

**Category:** accretion

**Evidence:** `src/firm/main.cpp:341-433` assembles the entire
`Motion::PlannerLimits` struct — `vMax`, `aMax`, `aDecel`, `omegaMax`,
`jerkMax`, `velKp`/`velKi`/`velKff`/`velIMax`, `trimKp`/`trimKi`/
`trimKaff`, `dutyFloor`, settle epsilons, etc. — as C++ literals in
`main()`, explicitly **not** read from `data/robots/*.json`/the boot
config the rest of this file otherwise sources everything from (contrast
`Config::defaultMotorConfigs`/`Config::defaultDriveConfig`/
`Config::defaultDrivetrainConfig`/`Config::defaultOtosBootConfig` a few
lines above, all reading real per-robot JSON). The comment at
`main.cpp:331-340` is explicit about why: the JSON's own `vel_gains`/
`shaper` block was tuned for the deleted `NezhaMotor` PID and "deployed
as-is... limit-cycled the real wheels at ~2-3 Hz," so the values below
are a plant-measured override that "must not reach this controller" —
but that override lives in source, not in a per-robot config file. This
directly contradicts `docs/design/design.md` §3's own stated project
convention: "**Config is fail-closed truth from `data/robots/*.json` —
no behavioral defaults baked into source**" (sprint 114's own rule,
restated as a global convention every subsystem doc may assume).

**Failing scenario:** any second robot built from this same firmware
image inherits robot-1's plant-measured gains (`kPlantGain = 1370.0f`,
`kPlantTau = 0.23f`, and every constant derived from them) regardless of
its own gearbox/wheel measurements — precisely the "one robot's gearboxes
[became] every robot's" failure this project has already hit once with
`Config::DriveBootConfig` (`drive.h:173-178`'s own doc comment references
that exact history).

**Design answer:** the comment already names the fix ("A planner-domain
config surface can supersede these constants later") — this finding is
that the supersession is not done, and the gap should be tracked as an
open issue rather than silently accepted as the shipping state, per
GUIDELINES §3's closing rule ("even when the diff is accepted for
schedule reasons, the gap gets named and filed as an issue").

---

### 5. MINOR — line sensor is permanently dead; a self-documented defect, but still a live production gap

**Category:** correctness

**Evidence:** `src/firm/app/robot_loop.h:171-181` (the header's own doc
comment):

```cpp
  // Parity picks line vs color in the pace block.
  //
  // KNOWN DEFECT, deliberately left alone by the command-ingestion rework:
  // nothing increments this. It has been stuck at 0 since the counter was
  // introduced, so `(cycleCount_ % 2) == 1` is permanently false and the
  // LINE sensor is never ticked -- only the color sensor is. ...
  uint32_t cycleCount_ = 0;
```

Confirmed at the call site, `robot_loop.cpp:531-533`:
`const bool tickedLine = (cycleCount_ % 2) == 1;` is always `false`
because `cycleCount_` is never mutated anywhere in this file. This is
already disclosed and deliberately deferred (the comment gives a
specific, defensible reason — a fix would add an I2C transaction every
other pace block and shift loop timing the motion tuning is calibrated
against), which is why this is MINOR rather than MAJOR: it is exactly
the "named the gap, filed it, didn't silently absorb it" behavior the
guidelines ask for. Flagged here only so it is visible in a review that
reads the whole tree rather than buried in one header's comment: **the
line sensor is 100% non-functional on this firmware today** — anything
downstream expecting `kFlagLinePresent`/`Frame.line` to ever go true will
wait forever.

**Design answer:** none needed beyond what the comment already proposes;
this entry exists so the gap is visible in a project-wide review, not
only in one file's local comment.

---

### 6. MINOR — I2C NAK/error counts are computed but structurally unreachable from telemetry

**Category:** correctness / explicitness

**Evidence:** `src/firm/devices/microbit_i2c_bus.cpp:313-320`
(`MicroBitI2CBus::record()`) genuinely tracks a per-device `errCount`/
`lastErr` on every transaction, and `reentryViolations_`
(`microbit_i2c_bus.cpp:74-82`) genuinely tracks bus re-entrancy. But
`src/firm/devices/i2c_bus.h:1-50` — the abstract `Devices::I2CBus`
interface `App::RobotLoop` actually holds (`robot_loop.h:145`,
`Devices::I2CBus& bus_`) — deliberately exposes only
`clearanceSafetyNetCount()`; `txnCount()`/`errCount()`/`lastErr()`/
`reentryViolations()` stay on the concrete `MicroBitI2CBus` class (by
the header's own explicit design note, `i2c_bus.h:5-9`). The result,
already half-admitted at `src/firm/app/telemetry.h:126-128`:

```cpp
//   bit 8  (kFlagFaultI2CNak)       -- I2C NAK/timeout. Declared, not yet
//                                    wired live (no per-transaction NAK
//                                    aggregate exists yet).
```

— is not quite accurate: the aggregate **does** exist
(`MicroBitI2CBus::errCount()`), it is simply unreachable through the
interface the one caller that could wire it (`RobotLoop`) is holding.
Per GUIDELINES §5, "errors must be loud or handled — never both absorbed
and unrecorded": a run of NAK'd duty writes or encoder reads is silently
retried (correctly — see the "what's good" section) but never surfaces
as a counted, wire-visible fault, and a genuine bus re-entrancy violation
is invisible outside a live debugger session for the same reason.

**Design answer:** either widen `Devices::I2CBus`'s interface with an
`errCount()`-style rollup (a single running total across all devices is
enough for a fault bit; the per-device breakdown can stay
`MicroBitI2CBus`-only) so `RobotLoop` can wire `kFlagFaultI2CNak`, or
retarget the flag's own doc comment to say plainly that it is
unreachable by construction, not merely "not yet wired."

---

### 7. MINOR — stale in-source doc comment for `RobotState::Wheel::cmdVelocity`'s writer

**Category:** style / explicitness

**Evidence:** `src/firm/types/robot_state.h:103-104`: "`cmdVelocity`
writer: `App::Drive::tick()`'s own staged target." This was accurate
before the `Motion::Planner` integration; today the writer is whichever
of `Motion::Planner::update()` (`planner.cpp:1232-1233`) or
`App::Drive::update()` (`drive.cpp:61-62`) currently owns motion — the
struct's own section-header convention ("documented with which
subsystem publishes it," line 37 of the same file) is violated by its
own field comment. Small, but this is the kind of doc-rot *inside*
source (not just in `docs/design/`) that compounds finding #3.

**Design answer:** update the comment to name both writers and the
ownership switch, mirroring the accurate treatment `Command.mode`/
`moveActive` already get two sections below it (lines 241-256).

---

### 8. NOTE — trailing whitespace

**Category:** style

**Evidence:** `src/firm/app/robot_loop.cpp:502-503` (inside the
`kSettle` `runAndWait` lambda) carry trailing whitespace after `{` and
`comms_.pump(...)`. Cosmetic only; called out because the project's
touched-code-conforms rule (`.claude/rules/coding-standards.md`) implies
this file is otherwise held to a high bar.

---

## What's good

This tree earned its "obsessive craftsmanship" bar in most places:

- **`App::Telemetry`** (`telemetry.h`/`.cpp`) is the single best example
  of GUIDELINES §1's "assembled once, from primary sources, immediately
  before it crosses the boundary" rule in the whole codebase: one
  `update(const Types::RobotState&)` call stages the entire outbound
  frame and every flag, with an explicit, categorized (State/Freshness/
  Event/Reserved) bit-layout comment block that names the exact defect
  class (comparing a Freshness bit across frames) a past sprint fell
  into. `setLiveFlag()` is a narrow, named, single-purpose escape hatch
  from that single-assembly rule, not a second path.
- **`Devices::NezhaMotor::writeRawDuty()`** (`nezha_motor.cpp:380-391`)
  is a textbook instance of the guidelines' own headline rule ("never
  record success before it happened"): `lastWrittenPct_`/
  `lastWriteTimeUs_` commit only on `status == kOk`, with a doc comment
  that states the exact failure mode (a NAK'd STOP silently suppressed
  forever by write-on-change) this project's own guidelines cite as the
  canonical example.
- **`Devices::MicroBitI2CBus::waitForClearance()`** never spins — a
  shortfall is always paid via `fiber_sleep()`, with the counter bump and
  the "never spin" reasoning stated inline (`microbit_i2c_bus.cpp:146-
  163`), directly answering GUIDELINES §5's concurrency concern.
- **`WireRuntime`** (`wire_runtime.h`/`.cpp`) is schema-agnostic, never
  allocates, and every decode function has an explicit, tested
  never-partial contract; the COBS delimiter-XOR mechanism and the
  CRC-scope composition are each justified with a short proof of
  correctness in the comment, not just an assertion that it works.
- **`App::Comms`**'s command ring (`comms.h`/`.cpp`) counts drops instead
  of silently absorbing them, distinguishes "malformed" from "dropped
  for backpressure" as two different telemetry-visible counters, and its
  handling of the 0x0A-embedding/relay-control-plane-sigil edge cases is
  narrowly scoped with an explicit non-masking argument for each.
- **Comment discipline** throughout is unusually good against GUIDELINES
  §6's own bar ("comments state constraints the code can't show, not
  narration") — nearly every non-trivial function states *why*, with a
  measurement or a sprint reference, not merely *what*.

Findings #1-#4 above cluster around one root cause (the `motion-planner`
architecture merge landed without a matching design-doc/interface
cleanup pass) rather than four unrelated problems — worth treating as
one conversation with the stakeholder about closing that gap, not four
separate tickets.
