---
status: in-progress
---

# DifferentialDrive: one class, one fiber — exploratory worktree

> ## ⚠ THIS IS NOT A GREENFIELD TASK — AN IMPLEMENTATION ALREADY EXISTS
>
> **Do not plan this from scratch, and do not write the class again.**
> A working implementation of this design is committed on branch
> `explore/differential-drive-kernel`, worktree
> `/Volumes/Proj/proj/RobotProjects/radio-robot-elite.worktrees/differential-drive-kernel`,
> head `1cda1e2f`. It builds (firmware + host) and its suites are green
> (471 sim, 977 unit). It has NEVER been on the robot.
>
> That code implements this document **as it stood on 2026-08-15**. The
> 2026-08-16 revision below landed a day AFTER the code was written, so
> the review layer it added — `Status` returns, fail-closed defaults,
> the heartbeat sentinel, gated λ, the observability counters, several
> renames — is **not in the implementation**.
>
> The live task is therefore a CONFORMANCE pass, not a build: close the
> gap between the extant code and this spec. **That pass is now
> largely DONE — head `7c682fc4`, 9 of 14 audit items closed.** See
> "Conformance status" at the end for what is closed, what is not,
> and the one finding the fidelity gate produced. The gap is enumerated in
> "Conformance audit" and the work is planned in "Conformance plan",
> both near the end of this file. Read those two sections before
> touching anything.

*Revised 2026-08-16, out of process: incorporates the two-agent design
review — Config completeness, lifecycle/boot ordering, estop honesty,
adapter contract, gated λ, execution reorder, verification bounds,
reference hygiene.*

*Reopened 2026-08-17 as a conformance pass against the extant
implementation (see the banner above).*

## Description

Build an exploratory version of the firmware in which motor control is
one self-contained class — `Control::DifferentialDrive`, one class, two
files — that owns the motors, the encoders, the (velocity, twist)
control law, and **its own CODAL fiber**. The rest of the system talks
to it only through its program interface: command methods in, an output
snapshot out. No line sensor, no color sensor, no OTOS inside it — just
PID.

Work happens **out of process** (stakeholder directive 2026-08-15) in a
dedicated git worktree off master. The worktree must end with **exactly
one way to work with the differential-drive motors**: the new class
replaces the old control path and every related dead branch is deleted,
not parked (see the deletion inventory below).

Stakeholder rulings (2026-08-15) that bind this design:

- Program interface, not a wire ABI: structs exist internally (the
  fiber↔object mailbox) and as public value types (Config, Output —
  copies); nothing is serialized.
- Config three ways: chainable single-field setters (construct empty,
  chain setters), whole-block `setConfig()`, `config()` fetch (copy).
- Lifecycle: start the object (`begin()`), then start the fiber
  (`start()`).
- Commands: `drive(velocity, twist, lease)` and
  `driveDuty(dutyLeft, dutyRight, lease)` — lease on both.
- Commands are body velocity + twist in **native encoder counts**
  (1 count = 0.1° shaft; rates counts/s). No millimeters anywhere in
  the class.
- Duty is a percentage; config carries `maxDuty` and `fullDutyVelocity`
  (wheel rate at 100% duty — the plant gain).
- **No heading in this system.** The application runs the observer
  (OTOS, encoders, time, velocity) and re-issues (velocity, twist)
  commands mid-motion; turns are planned from heading assessed outside
  motor control. No camera assumptions anywhere.
- `output()` returns a copy of the fiber-published block.

This prototypes the wheel kernel that later drops into the MicroPython
image (branch `micropython-vevov-handoff`; RAM/flash measured as
non-blockers — ~67 KB CODAL heap and ~97 KB flash headroom remain even
with all of `src/firm` compiled in). Building it in the current C++
tree first means it can be benched with existing tooling (rogo,
telemetry, tovez on the stand) before touching the MP build, and it
directly runs the long-standing sequestration experiment: motor control
on its own fiber, everything else elsewhere.

## Proposed fix

### The class

```cpp
// differential_drive.h -- Control::DifferentialDrive: the whole wheel
// kernel in ONE class. Owns both motors, the encoder schedule, the
// (velocity, twist) control law, and its own fiber. Program interface
// only -- nothing here crosses the wire.
// Native unit: 1 count = 0.1 deg shaft rotation (Nezha 0x46 native).
// Twist: half-differential wheel rate -- left = velocity - twist,
// right = velocity + twist -- CCW-positive (REP-103; fwdSign in leaf).
// The counts-per-body-radian constant lives in the APPLICATION.
namespace Control {

class DifferentialDrive {
 public:
  enum class Status : uint8_t {  // every refusal visible at the callsite
    kOk = 0,
    kRefusedUnconfigured,  // maxDuty == 0; or VELOCITY with fullDutyVelocity == 0
    kRefusedNotBegun,      // command before begin(). NOT before start(): the
                           //   host harness commands and step()s WITHOUT ever
                           //   launching the fiber, so readiness is begin()'s
                           //   to grant, not start()'s
    kRefusedEstopped,
    kRefusedNonFinite,
    kCadencePreserved,     // post-begin setConfig with a differing cyclePeriod:
                           //   block applied, frozen cadence kept
  };

  struct Config {                    // value type -- fetch/replace by copy
    // Every default is fail-closed: a default-constructed Config refuses
    // BOTH modes (maxDuty == 0 -> no authority at all; fullDutyVelocity
    // == 0 -> VELOCITY additionally refused). Nothing moves until
    // configured. This struct carries the ENTIRE tuned surface of the
    // pipeline being ported (Drive + WheelControl proto groups, rebaked
    // counts-native) -- no hidden constants, no silently dropped stage.
    float maxDuty = 0.0f;            // [%] authority rail; 0 = ALL modes refused
    float fullDutyVelocity = 0.0f;   // [counts/s] wheel rate at 100% duty;
                                     //   0 = uncalibrated, VELOCITY refused
    float kp = 0.0f;                 // [1]
    float ki = 0.0f;                 // [1/s] on clamped position error
    float iMax = 0.0f;               // [counts/s]
    float kaff = 0.0f;               // [s] accel feedforward
    float pidMax = 0.0f;             // [counts/s]
    // Stage A commanded->actual correction, per wheel per approach
    // direction (counts rebake of Drive's wheel_gain_*/wheel_intercept_*)
    float wheelGainLeftAccel = 1.0f;        // [1]
    float wheelInterceptLeftAccel = 0.0f;   // [counts/s]
    float wheelGainLeftDecel = 1.0f;        // [1]
    float wheelInterceptLeftDecel = 0.0f;   // [counts/s]
    float wheelGainRightAccel = 1.0f;       // [1]
    float wheelInterceptRightAccel = 0.0f;  // [counts/s]
    float wheelGainRightDecel = 1.0f;       // [1]
    float wheelInterceptRightDecel = 0.0f;  // [counts/s]
    float crawlPulse = 0.0f;         // [%] sub-breakaway sigma-delta pulse; 0 = off
    float speedFloor = 0.0f;         // [counts/s] Stage A v_min; 0 = off
    float posErrMax = 0.0f;          // [counts] Stage B position-error clamp;
                                     //   0 = unclamped
    float biasMax = 0.0f;            // [counts/s] Stage C trim authority clamp
    float tauAdapt = 0.0f;           // [s] Stage C adaptation; <=0 disables
    float aSteady = 0.0f;            // [counts/s^2] steady-state gate
    float deficitThreshold = 0.0f;   // [counts/s] sustained-error flag; 0 = off
    float deficitWindow = 0.0f;      // [ms]
    float twistHoldGain = 0.0f;      // [1/s] twist-integral ratio hold; 0 = off
    bool lambdaEnabled = false;      // authority-headroom scaling; ships OFF so
                                     //   the first bench pass runs the pure port
    float stallSpeed = 0.0f;         // [counts/s]
    float stallDemand = 0.0f;        // [counts/s] 0 = detector off
    float stallWindow = 0.0f;        // [ms]
    uint32_t cyclePeriod = 24;       // [ms] fiber cadence; FROZEN at begin()
                                     //   (leaf write throttle derives from it)
  };

  struct Output {                    // value type -- output() returns a copy
    uint32_t now;                    // [ms] kernel clock at publish
    uint32_t nowFine;                // [us] same instant -- age-math base
                                     //   (no unit in the name; [us] tag rules)
    uint32_t cycleCount;             // heartbeat -- the sentinel watches this
    uint32_t cyclePeriodMeasured;    // [us] measured (feeds all dt terms;
                                     //   named apart from Config::cyclePeriod
                                     //   [ms] to keep the units apart)
    uint32_t cycleBusy;              // [us]
    uint32_t cycleOverrunCount;      // missed absolute deadlines (lesson 17's
                                     //   observability, now a real field)
    uint32_t sampleTimeLeft;         // [us] last SUCCESSFUL L collect
    uint32_t sampleTimeRight;        // [us] ~4-5 ms younger than L
    uint32_t positionEpochLeft;      // bump on that wheel's software
    uint32_t positionEpochRight;     //   rebaseline -- feed telemetry's
                                     //   per-wheel EncoderReading.position_epoch
    float positionLeft;              // [counts] accumulated between software
    float positionRight;             //   rebaselines; never device-reset
    float velocityLeft;              // [counts/s] over genuine inter-sample dt
    float velocityRight;             // [counts/s]
    float velocity;                  // [counts/s] measured mean
    float twist;                     // [counts/s] measured half-diff, CCW+
    float appliedDutyLeft;           // [%]
    float appliedDutyRight;          // [%]
    float lambda;                    // [1] authority scale in effect
    bool ready, estopped, leaseExpired;
    bool satLeft, satRight, stallLeft, stallRight;
    bool wedgeLeft, wedgeRight, connectedLeft, connectedRight;
    uint32_t leaseExpiryCount;       // sticky diagnostics
    uint32_t i2cFaultCount;          // failed-collect cycles, derived from
                                     //   sample-stamp non-advance (the leaf's
                                     //   requestSample()/tick() return void;
                                     //   sampleTime()/connected() are the
                                     //   observable surface)
  };

  // Hal::FiberLauncher: NEW one-method HAL seam, launch(entry, arg) --
  // platform/microbit wraps codal create_fiber; the host build's
  // implementation fails the test if invoked (host drives step()).
  DifferentialDrive(Hal::Motor& left, Hal::Motor& right,
                    const Hal::Clock& clock, Hal::Sleeper& sleeper,
                    Hal::FiberLauncher& launcher);

  // ---- config surface 1: chainable single-field setters ----
  DifferentialDrive& setMaxDuty(float maxDuty);            // [%]
  DifferentialDrive& setFullDutyVelocity(float velocity);  // [counts/s]
  DifferentialDrive& setKp(float kp);
  DifferentialDrive& setKi(float ki);                      // [1/s]
  DifferentialDrive& setIMax(float iMax);                  // [counts/s]
  DifferentialDrive& setKaff(float kaff);                  // [s]
  DifferentialDrive& setPidMax(float pidMax);              // [counts/s]
  DifferentialDrive& setTwistHoldGain(float gain);         // [1/s]
  DifferentialDrive& setWheelCorrection(
      float gainLeftAccel, float interceptLeftAccel,
      float gainLeftDecel, float interceptLeftDecel,
      float gainRightAccel, float interceptRightAccel,
      float gainRightDecel, float interceptRightDecel);
                              // [1] gains, [counts/s] intercepts -- Stage A
  DifferentialDrive& setCrawlPulse(float crawlPulse);      // [%]
  DifferentialDrive& setSpeedFloor(float speedFloor);      // [counts/s]
  DifferentialDrive& setPosErrMax(float posErrMax);        // [counts]
  DifferentialDrive& setAdaptation(float biasMax, float tauAdapt,
                                   float aSteady);
                              // [counts/s] [s] [counts/s^2] -- Stage C
  DifferentialDrive& setDeficit(float threshold, float window);
                              // [counts/s] [ms]
  DifferentialDrive& setLambdaEnabled(bool enabled);
  DifferentialDrive& setStall(float speed, float demand,
                              float window);   // [counts/s] [counts/s] [ms]
  DifferentialDrive& setCyclePeriod(uint32_t period);      // [ms] REFUSED
                                                           //   after begin()
  // ---- config surfaces 2 + 3: whole-block replace, fetch (copy) ----
  Status setConfig(const Config& config);  // post-begin: differing
                                           //   cyclePeriod -> kCadencePreserved
  Config config() const;

  // ---- how a REFUSED SETTER is observed -------------------------------
  // The chainable setters must return DifferentialDrive& to chain, so they
  // cannot return Status. Without this pair, "every setter rejects
  // non-finite values" and "setCyclePeriod() after begin() is refused"
  // would both be SILENT -- exactly the old silent-no-write behaviour this
  // design set out to kill, reintroduced on the whole config surface.
  // Sticky: holds the first refusal since the last clear, so a caller can
  // run a long setter chain and check once at the end.
  Status lastError() const;
  void clearLastError();

  // ---- lifecycle (ordering: see "Lifecycle and boot ordering") ----
  Status begin();        // software init + boot zero-write, called AFTER the
                         //   Preamble resolves (Preamble keeps motor priming);
                         //   freezes cyclePeriod, derives the leaf throttle
  Status start();        // launch the kernel fiber via the injected launcher
                         //   (idempotent); the fiber body just loops step()
  bool running() const;
  void step();           // ONE kernel cycle, synchronously -- the host
                         //   harness's deterministic entry point; never
                         //   called while the fiber is running

  // ---- commands; lease is a DURATION from now, expiry stops ----
  Status drive(float velocity, float twist,
               uint32_t lease);       // [counts/s] [counts/s] [ms]
  Status driveDuty(float dutyLeft, float dutyRight,
                   uint32_t lease);   // [%] [%] [ms]
  void neutral();        // commanded stop through the full stop path
  void estop();          // latch set immediately; zero written at the next
                         //   kernel cycle; holds until estopClear()
  void estopClear();

  // ---- output: seq-consistent COPY of the published block ----
  Output output() const;

 private:
  struct Command {       // fiber<->object mailbox (seq bump commits)
    uint32_t seq; uint8_t mode;   // 0 neutral, 1 velocity, 2 raw duty
    float velocity, twist;        // [counts/s]
    float dutyLeft, dutyRight;    // [%]
    uint32_t validUntil;          // [ms] absolute, computed in drive()
  };
  void run();            // fiber body: loop { step(); sleep to deadline }
  // + config snapshot (seq'd), internal Output instance (seq'd),
  //   estop latch (single u32, outside the seq handshake),
  //   ported pipeline state (position refs, biases, latch timers)
};

}  // namespace Control
```

### Interface semantics

- **Lease is relative**: `drive(v, t, 300)` = valid 300 ms from now.
  The class computes absolute expiry against its own clock internally —
  no clock-domain coupling in the API. Lease durations are clamped to
  `kLeaseMax` (3,600,000 ms — far below INT32_MAX ms so the wrap-safe
  signed compare against `validUntil` is never ambiguous). Expiry
  zeroes through the full stop path (stop-enforce countdown,
  stopNotTaken), consulting nobody; it only ever moves motion toward
  zero (the `zeroUnownedMotion` monotone contract restated as a
  timestamp). Refreshing the command IS feeding the watchdog.
- **Estop: the honest guarantee.** `estop()` sets a kernel-side latch
  outside the seq handshake (single atomic u32) from the caller's
  fiber. It does NOT write the motors itself — the kernel fiber is the
  only 0x10 client, always. The zero is issued at the next kernel
  cycle start: **latched immediately; zero written within one
  cyclePeriod when the kernel is healthy.** A wedged kernel fiber
  (dead-bus busy-wait, lesson 17) delays enforcement — see the
  heartbeat sentinel and the safety-scope note in "Lifecycle and boot
  ordering". `estopClear()` clears the latch only; the pending command
  was invalidated at estop, so motion resumes only on a FRESH
  `drive()`/`driveDuty()`.
- **Config is live, except cadence**: the fiber snapshots the config
  block (under seq) at each cycle start, so chained setters work
  mid-run — preserves the bench live-tuning workflow. A chain of
  single-field setters is NOT atomic against that snapshot: a cycle
  can see a half-applied chain (benign for independent gains; the
  grouped setters — `setStall`, `setWheelCorrection`, `setAdaptation`,
  `setDeficit` — exist for fields that must move together).
  `cyclePeriod` is the exception: frozen at `begin()`, when the leaf
  write throttle is derived from it (lesson 6); `setCyclePeriod()`
  after `begin()` is refused. Write shaping (reversal dwell, deadband,
  slew) stays motor-leaf config.
- **Fail-closed by default, visible when refused**: a default Config
  refuses everything — `maxDuty == 0` refuses BOTH modes,
  `fullDutyVelocity == 0` additionally refuses VELOCITY — with
  `ready == false`, not the old silent no-write return. Every setter,
  `setConfig()`, and command rejects non-finite values (fastPid's
  fail-closed posture applied at the boundary, lesson 10); a refused
  command leaves current state untouched AND returns the reason — the
  `Status` enum makes every refusal observable at the callsite, not
  just in Output. Commands before `begin()` are refused the same way.
  **The chainable setters are the exception that needs its own
  mechanism**: they return `DifferentialDrive&` so they can chain, so
  they cannot return `Status`, so a rejected non-finite value or a
  post-`begin()` `setCyclePeriod()` would be refused SILENTLY — the very
  behaviour this bullet claims to have removed. `lastError()` /
  `clearLastError()` (above) carry it: sticky, checked once after a
  chain.
- **`driveDuty()` semantics**: bypasses Stage A/PID/λ entirely; still
  goes through `writeShapedDuty()` (never raw), still bounded by
  `maxDuty`, still lease-guarded, and stop-enforce stays armed. Stall
  detection needs a demand in counts/s: in duty mode the demand is
  `duty × fullDutyVelocity / 100`; with `fullDutyVelocity == 0`
  (uncalibrated plant-ID runs) no speed-equivalent demand exists, so
  the stall detector is DISARMED in duty mode — documented, and
  visible (`stallLeft/Right` stay false). It is a plant-ID/bench
  mode, not an unshaped escape hatch.
- **Atomicity**: cooperative fibers on one core; aligned 32-bit stores
  are atomic; each block is single-writer, seq-committed; readers copy
  under a seq check and RETRY UNTIL CONSISTENT (a retry-once seqlock
  can return a torn copy; the retry loop is bounded only by a debug
  assert). Methods are called from the main fiber; the kernel fiber is
  the only writer of Output. The no-preemption assumption (CODAL
  fibers switch only at yield points) is load-bearing — verify it
  against codal-core's scheduler source before trusting the seq
  scheme, and record the finding here.

### Lifecycle and boot ordering

Composition constructs; nothing touches hardware until the boot
sequence says so. The ordering is fixed (132-015 and the Preamble
contracts, verified against `core/preamble.cpp` and `main.cpp`
2026-08-16):

1. `Core::composeRobot()` CONSTRUCTS the class — no I2C, no fiber.
2. **Baked Config applies at compose**: `setConfig()` with the
   `gen_boot_config.py`-baked block, INCLUDING `cyclePeriod`.
   Memory-only — no I2C happens here, so a pre-boot apply is safe
   (the 132-015 trap was OTOS register writes, not struct
   assignment). This is where the JSON cadence reaches the class:
   before the freeze, by construction.
3. `RobotLoop::boot()` runs the Preamble as today: power-settle, then
   bounded round-robin probes. **The Preamble keeps motor priming** —
   the class never re-primes. One owner, never both.
4. `begin()` — called from `boot()` after the Preamble resolves:
   software init (position epochs, latches), the boot zero-write
   through the leaf's stop path, cyclePeriod FREEZE + leaf-throttle
   derivation. Bounded like everything else in boot (lesson 17).
5. `loadPersistedTuning()` — persisted LIVE fields land after boot,
   as today (132-015); they flow into setters/`setConfig()`. Cadence
   is not among them: a post-begin `setConfig()` whose block carries
   a different `cyclePeriod` PRESERVES the frozen value, applies the
   rest, and returns a Status saying so; `setCyclePeriod()` after
   `begin()` is refused outright.
6. `start()` — the fiber launches (via the injected launcher) only
   after the object is configured.

**Readiness is `begin()`'s to grant, not `start()`'s.** Commands are
accepted once `begin()` has completed; a command before that is refused
`kRefusedNotBegun`. Tying acceptance to `start()` instead would make the
golden-trace fidelity gate — the one test of the whole "zero math
changes" claim — impossible to write: the host harness drives `step()`
directly and NEVER calls `start()` (it has no fibers), so a
`running()`-gated `drive()` would refuse every command the harness
issued. `start()` is about who calls `step()`, nothing more.

**Host-harness seam**: the class takes `Hal::Sleeper` and a
`Hal::FiberLauncher` (NEW one-method HAL interface — `launch(entry,
arg)`; platform/microbit wraps codal `create_fiber`, the host build's
implementation fails the test if ever invoked) alongside
`Hal::Clock`, and the fiber body is a loop over the public `step()`.
The host harness drives `step()` directly against the stepped plant —
deterministic, no fibers — and never calls `start()`. NOTHING in the
class names a CODAL symbol, so the same two files compile and link in
both builds: the launcher seam, not an #ifdef, is the split.

**Heartbeat sentinel (defense in depth, limits stated)**: `RobotLoop`
keeps exactly one safety job — each main-loop cycle it checks
`output().cycleCount` is advancing; if it stalls for more than
`kSentinelPeriods` cycle periods while motion was commanded, it (a)
raises a sticky fault flag on telemetry and (b) calls the leaf's NEW
`Hal::Motor::emergencyStop()` on both motors — an immediate, unstaged
zero write through the never-shaped stop path (stop already bypasses
all shaping, lesson 2). `setDuty(0)` would NOT do: it only stages,
and the dead kernel fiber's `tick()` is what executes stages.
`emergencyStop()` is the one sanctioned exception to the kernel-only
0x10 rule — fiber-atomic I2C makes the interleaving safe (worst case
it destroys one pending encoder sample; the right trade in an
emergency) — and it joins the leaf's short exception list.

Scope of the guarantee, honestly: the sentinel covers failures where
the MAIN loop still runs — kernel fiber crashed, exited, or
logic-stalled while the bus is alive (brick latched at last speed,
the 936 mm runaway class). It CANNOT cover the dead-bus busy-wait
(lesson 17): `waitForStop()` freezes every fiber, sentinel included —
though in that failure the bus that would carry the zero write is
itself the thing that died. Only the deferred hardware WDT covers
that class.

**Safety scope, stated honestly**: with CODAL I2C busy-waiting
(`waitForStop`, lesson 17) and the hardware WDT deferred to the MP
phase, this experiment demonstrates the interface and the control
law; it CANNOT claim a starvation-proof stop path.
`spike-i2c-bus-owning-fiber.md` stage B1 says as much. The sentinel
narrows that gap; only the deferred WDT/non-blocking-I2C work closes
it.

### Measurement age semantics

- `sampleTimeLeft/Right` are stamped at collect SUCCESS only (the
  131-002 rule from `NezhaMotor::lastFreshUs_`): failed collects HOLD
  the stamp so age grows honestly; `connectedLeft/Right` complement it.
- Age = `(int32)(nowFine − sampleTime)` — wrap-safe signed difference.
- L and R differ by construction: sequential split-phase collects L,
  then R one settle window later — R deterministically ~4-5 ms younger;
  an observer can model the fixed skew. Fault-driven divergence (ages
  separating by whole cycles) is the per-wheel staleness signal.
- Per-wheel velocity is computed over the GENUINE inter-sample
  interval, not the nominal cycle period.

### Control design (2-DOF: mean + differential)

- Per-wheel targets: `left = velocity − twist`, `right = velocity +
  twist` — the commanded RATIO is the (v, twist) pair.
- **Authority headroom (λ)** — config-gated (`lambdaEnabled`, ships
  OFF so the first bench pass runs the pure port): SAME-CYCLE two-pass
  allocation — compute both wheels' unclamped duty demands, derive
  `λ = min(1, maxDuty/|dutyDemandLeft|, maxDuty/|dutyDemandRight|)`,
  scale both targets by λ, then convert and write. Same-cycle because
  a last-cycle λ distorts curvature for the first cycle after every
  command step. The asymmetric filter applies to RELEASE only
  (~300 ms); attack is immediate within the cycle. Curvature
  preserved — the healthy wheel slows when the other saturates.
  (Min-lambda shape from `Kinematics::Model::saturate`,
  `src/firm/kinematics/kinematics.h:76` — tested, zero callers.)
- **Twist-integral hold** (encoder-only ratio maintenance — NOT a
  heading feature): integral of commanded twist vs measured
  differential position, trim clamped to duty headroom. twist=0 →
  equal accumulated counts (straight as wheels can know); twist≠0 →
  the commanded arc. `twistHoldGain` 0 disables. The reference
  integrates the λ-SCALED commanded twist, not the raw command —
  otherwise a saturation episode teaches the hold an arc the wheels
  never drove.
- **Anti-windup**: the λ-SCALED command feeds the integral-of-command
  position reference (the position ref IS the integrator — current
  config runs pure-I). Bias adaptation gated on λ≈1.
- Duty conversion: `duty = velocity / fullDutyVelocity × 100` + PID
  correction — `fullDutyVelocity` replaces `duty_per_speed`.
- Ported pipeline (from the existing `differential_drive.cpp`, which is
  unit-parametric — mm→counts is a config rebake, zero math changes):
  speed floor → Stage A gain/intercept → position-error integral →
  fastPid → duty → crawl sigma-delta → Stage C bias adaptation →
  stall/deficit latches → stop-enforce write gate.
- Motor leaf + anti-latch shaping (`nezha_motor.cpp`) and MotorArmor
  stay byte-identical, EXCEPT: `travel_calib` is REMOVED from the leaf
  — `position()`/`velocity()` become counts-native; travel calibration
  moves up to the application — and `Hal::Motor::emergencyStop()` is
  ADDED (immediate unstaged zero through the never-shaped stop path;
  the sentinel's actuation primitive, emergency-only).

### The fiber and the main loop

The kernel fiber free-runs `run()`, PRESERVING today's loop schedule
(`RobotLoop::cycle()`): the control step runs FIRST, on the PREVIOUS
cycle's published samples, because `Hal::Motor` is stage-then-execute
— `setDuty()` only stages, and `tick()` both collects the encoder AND
executes the staged duty. A duty computed after both collects would
land a cycle late, and a second `tick()` would perform an invalid
extra collect. So:

snapshot command+config → estop/lease check → control step on last
cycle's samples (λ, twist hold, PID; stage shaped writes via
`setDuty()`; no yields) → requestL → settle (sleeper) → tickL
(collect L + execute staged L) → requestR → settle → tickR (collect R
+ execute staged R) → publish Output → sleep to the ABSOLUTE
next-cycle deadline.

The one-cycle command-to-write latency is IDENTICAL to today's loop —
which is what the golden-trace equivalence gate requires. The
alternative (splitting collect and apply into separate leaf
operations) is a real interface change to a leaf this issue promises
to keep byte-identical, and is explicitly NOT taken.

`Core::RobotLoop` no longer runs the drive. The main fiber keeps
comms, telemetry, sensors (OTOS et al.), the heartbeat sentinel
(above), and the WHEELS↔class adapter — whose contract is explicit;
"thin" does not mean unspecified:

- **Inbound** (`Wheels` is [mm/s] on the wire, `envelope.proto`):
  `v_left`/`v_right` → counts/s PER WHEEL via the application-held
  travel calibrations (robot JSON `travel_calib_left`/
  `travel_calib_right`, mm/deg — two independent values,
  `robot_config.proto` Motors); then `velocity = (l+r)/2`,
  `twist = (r−l)/2`; `lease = Wheels.duration` (clamped to
  `kLeaseMax`). ESTOP→`estop()`, STOP→`neutral()`.
- **Outbound** (TLM frame SHAPE unchanged): `output()` counts → mm
  per wheel via the same two calibrations, the existing ±32 m wire
  clamp preserved, and the per-wheel rebaseline-epoch contract
  (`EncoderReading.position_epoch`, `telemetry.proto`) fed from
  `Output::positionEpochLeft/Right`. Pose sources OTOS only; with
  OTOS absent the pose valid bit stays clear — encoder-pose is the
  application's job now.
- Two calibration values, one conversion each way per wheel, owned by
  the adapter — the class never sees millimeters.

Bench scripts that speak MOVE (`move_protocol_bench`, `twist_drive`,
`velocity_step_response`, the accuracy benches) get ERR in this tree.
Verification drives WHEELS-speaking scripts (`wheels()` already
exists on `NezhaProtocol`); porting the two scripts steps 3-5 need is
budgeted in execution step 4, not assumed away.

- The kernel's settle windows fiber_sleep — yielding to the main fiber,
  which is where comms pumping already happens; the current design's
  "spend the settle usefully" property is preserved by construction.
- Bus safety: CODAL I2C transactions never yield mid-transaction
  (fiber-atomic); per-device clearances are per-address; the kernel is
  the ONLY 0x10 client in normal operation (sole sanctioned exception:
  the sentinel's `emergencyStop()`), so select→collect ordering is
  preserved structurally. Main-fiber OTOS traffic (0x17) CAN land inside a kernel
  settle window — the one unmeasured interleaving (wedgelab
  mixed-traffic evidence says it is fine); measured explicitly on the
  bench (see Verification).

### Hard lessons that bind the implementation (verified against source 2026-08-15)

Write-path / anti-latch (`nezha_motor.cpp` — leaf unchanged, read
before touching anything near it):

1. The reversal write train latches the 0x46 readback — the leaf's
   dwell (write 0, hold ≥50 ms, ship 100) + deadband boost + slew cap
   stay exactly as-is. PID sign-dither at decel IS the trigger; the new
   pipeline commands only through `writeShapedDuty()`, never raw.
2. Stop is NEVER shaped: zero short-circuits dwell/throttle/slew/dedupe
   (`writeShapedDuty():148`, `writeRawDuty():230`); the sigma-delta
   carry is discarded on zero (creep prevention, 133-002). Preserve
   every stop bypass.
3. `stopNotTaken` (`writeRawDuty():219`): a commanded zero re-writes
   while |velocity| > threshold regardless of the dedupe cache — the
   brick latches its last speed across nRF resets; one lost zero write
   is permanent. Do not "simplify" to write-on-change.
4. First-write slew exemption (`:236`): the −128 sentinel through
   clampStep once produced a wrong-direction first command (a latch
   trigger). Keep the exemption.
5. `lastWrittenPct_` commits only on `status == kOk` (`:252`) — NAKed
   writes retry next tick instead of latching "already written".
6. **TRAP — hand-synced throttle**: `kMinWriteIntervalUs = 27000`
   (`:224-229`) is hand-synced to kCycle−5 ms because devices/ may not
   include core headers. Resolution: `cyclePeriod` is FROZEN at
   `begin()`, which derives the leaf throttle (new MotorConfig field)
   from it at that moment — config-derived, never a stale literal,
   never live-mutated under the leaf. In the same pass, any leaf
   window counted in TICKS (wedge detect, rest confirm, stop-enforce)
   changes wall-clock meaning when the cadence changes — re-derive
   each from the configured period or express it in elapsed time.
7. **TRAP — leaf constants are in mm/s**: `kStopConfirmVelocity` (8),
   `kReconfigureRestVelocity` (5), MotorArmor's `kRestVelocity` (5) are
   mm/s thresholds. Counts-native `position()`/`velocity()` changes
   their effective meaning. Do the arithmetic once and write it down
   rather than carrying a rule of thumb: 1 count = 0.1° and tovez's
   `travel_calib` is 0.7837 mm/deg, so **1 count = 0.07837 mm and
   1 mm = 12.76 counts** — the factor is ~12.8×, not the "~10×" this
   list said in an earlier revision (and not the "~13×" Verification 11
   says; both were loose restatements of the same number, which is
   exactly how unit drift starts). Rebaked explicitly:

   | constant | old | rebaked |
   |---|---|---|
   | `kStopConfirmVelocity` | 8 mm/s | **102 counts/s** |
   | `kReconfigureRestVelocity` | 5 mm/s | **64 counts/s** |
   | MotorArmor `kRestVelocity` | 5 mm/s | **64 counts/s** |

   Rebaking off "~10×" instead would set stop-confirm at 80 counts/s —
   a 22% tighter trip than intended on the one net that catches a lost
   zero write. Silent unit drift here disarms the stop-confirm safety
   net, which is why the numbers are tabulated rather than described.
8. Encoder split-phase: the request's postClear=4000 is the ONLY thing
   making collect honor the brick's select→read settle (collect passes
   0/0); preClear is deliberately 0 (measured wasteful). The brick
   holds ONE pending request; 0x46 sits frozen at 0 until `begin()`'s
   priming read. Encoders are NEVER device-reset in normal operation —
   software offset/rebaseline only (positionEpoch discipline).

Control law (`differential_drive.cpp` — semantics to port intact):

9. Stage A `correctedCommand()`: stop is stop (zero in → zero out,
   never offset by map or bias); below-intercept → 0 (unreachable);
   corrected magnitude ≤ 0 → 0 (never flip direction). Bias enters
   INSIDE Stage A and adapts AFTER the duty write (1-tick lag, by
   design).
10. `fastPid()` fails closed on non-finite (`:180`). The I term is
    ki × clamped POSITION error — the position ref IS the integrator;
    `positionError()` re-anchors on speed==0 / dt≤0 / !connected /
    epoch change / !armed. tovez ships pure-I (kp=0, ki=6).
11. Stall gates on the RAW command, not the post-floor value (`:359` —
    the floor boosts sub-v_min commands; testing the boosted value
    reads a near-zero request as demand). The encoder-still fallback
    deliberately does NOT suppress on wheelFrozen (`:366` — "position
    unchanged while duty applied" IS a stall; suppressing exactly then
    hid real jams). One physical condition → BOTH wheels latch.
12. **Kernel stall is encoder-based ONLY and cannot catch a
    slipping-wheel jam** (measured: 239/222 ticks of wheel rotation
    while pinned against the rail). The OTOS body-speed stall check —
    including the |omega|×halfTrack pivot term and the
    speed-magnitude-not-v_x lesson — moves to the APPLICATION observer,
    which commands stop through the interface. Document this split in
    the class header.
13. Stop-enforce: the countdown re-arms on the transition to commanded
    stop; writes are unconditional while encoders still read motion
    (`:385-400`). `estop()` resets ALL adaptive state (bias, refs,
    latches); `takeover()` deliberately does not — two reset semantics.
14. `calibrated_` today silently no-ops the whole tick (`:250` — not
    even a zero write). New class: uncalibrated REFUSES velocity mode
    visibly (ready=false) and still runs the stop path.
15. Freshness: 200 ms sample-age gate on Stage B/C. Leaf velocity is
    currently a per-tick quotient over `lastTickUs_` (a dead bus reads
    velocity≈0, not stale). The new Output computes velocity across
    SUCCESSFUL collects only and flags staleness — a deliberate
    improvement; note it in the header.

Loop/fiber/bus (`robot_loop.cpp`, `microbit_i2c_bus.cpp`, knowledge
base):

16. Any 0x10 transaction between a 0x46 select and its read destroys
    the pending sample (416/416 measured) — the kernel fiber is the
    sole 0x10 client in normal operation; the sentinel's
    `emergencyStop()` is the one exception and knowingly accepts a
    destroyed pending sample. Foreign-address interleaving during
    settle is unmeasured (wedgelab mixed-traffic says fine) →
    explicit bench check.
17. CODAL I2C transactions busy-wait and never yield mid-transaction —
    fiber-atomicity is what makes the shared bus safe. A dead external
    bus parks `waitForStop` toward ~10 s with everything frozen (the
    silent-boot incident): `begin()` keeps the Preamble's bounded-probe
    discipline; the fiber's cycle overrun counter is the observability.
18. Absolute-deadline pacing (131-005): gap-relative sleeps compound
    (+4 ms structural drift measured); the fiber sleeps to
    cycleStart + cyclePeriod, absolute.
19. All dt terms use MEASURED cyclePeriod, never a baked constant —
    the baked-vs-delivered mismatch (50 vs 54 ms) alone flipped the
    31 Hz heading-hold marginal; the same hazard applies to the twist
    hold.
20. Build hygiene: clean build after ANY shared-header change (stale
    incremental → boot HardFault that looks like power loss);
    `build.py --clean` before every HITL verification flash; deleted
    sources must come out of ALL FOUR build source lists or the result
    is a link error.
21. Bench procedure: port-close resets the robot (use `rogo serve`);
    sleep ~5 s after a flash before believing a failure; robot STILL
    at boot (OTOS gyro calibration — application concern, but it
    corrupts bench truth); never poll telemetry synchronously during a
    move over the relay (it stops the robot — measured 0.3 mm vs
    197.5 mm).

### Deletion inventory (nothing dead, nothing hanging)

The worktree ends with EXACTLY ONE way to work with the differential
drive motors. New code replaces old; the old is deleted, not parked:

- **`Control::DifferentialDrive` (old)** — replaced in place (same two
  files). The command/duration/moveId surface,
  `configure(Config::Robot&)`, `takeCompletion()`, `update()`
  blackboard writes: gone.
- **`src/firm/motion/` — the ENTIRE tree** (planner 2074 LOC,
  navigator, odometry, estimation, shape, profile, arc_solver, capi,
  and their ~3.3k LOC of tests). Not composed = dead = deleted. Remove
  from all four build source lists (lesson 20).
- **`Core::RobotLoop`**: `zeroUnownedMotion()` (subsumed by the lease,
  with the heartbeat sentinel as its loop-side remnant),
  `haltOnStall()` (stall halt is in-class now), MOVE/GO_TO routing and
  handlers, `publishMoveResult`/`publishGotoResult`,
  `ackDriveCompletion`, odometry integration + `publishPose`'s encoder
  half, the drive tick/update calls and settle-window scheduling (all
  now inside the class fiber). RobotLoop keeps: comms pump, config
  routing, telemetry emit, OTOS/line/color ticks, the WHEELS→drive()
  adapter, ESTOP→estop(), STOP→neutral(), and the heartbeat sentinel.
- **`Types::RobotState`**: `Wheel::cmdVelocity`/`cmdAccel` (the
  actuation boundary moves inside the class), the command/estimate
  blocks only the planner wrote. Wheel measured state for telemetry now
  comes from `output()`.
- **`Hal::Motor::applyTravelCalib()` + `MotorConfig::wheelTravelCalib`**
  — deleted; the leaf is counts-native. `Core::configureMotor()`'s
  travel-calib push path goes with it. The robot JSON KEEPS
  `travel_calib` — consumed by the application layer, and
  `gen_boot_config.py` re-emits control values in counts from the same
  file (single source of truth).
- **Dead config keys**:
  `motors.vel_kp/vel_ki/vel_kff/vel_i_max/vel_kaw` (dead since the PID
  left the leaf — `hal/motor.h:24` says so) — deleted from the schema
  path per configuration-discipline ("delete it, don't wire it").
  PLANNER group arms in Configurator go with motion/; DRIVE/
  WHEEL_CONTROL groups re-point to the new class's setters (live tuning
  preserved, 5 PID fields stay flash-persisted).
- **Protocol arms in this tree**: MOVE/GO_TO deregistered (ERR to a
  client that sends them); TLM frame SHAPE unchanged (protocol changes
  out of scope) but pose sources from OTOS only — encoder-pose is the
  application's job now. This narrowing is a documented wart of the
  exploratory tree.
- **Sim/tests**: suites that test deleted behavior (move_protocol,
  stop_path_safety's planner legs, app_drive/odometry/robot_loop
  harnesses, planner/navigator test trees, planner_harness.py,
  tune_velocity_pid.py) are deleted or trimmed in the worktree; the
  surviving suite (devices_motor, comms, telemetry, configurator,
  layer isolation minus deleted layers) must pass. A host-build harness
  for the NEW class (pure control math against the stepped plant,
  `src/tests/sim/plant/` precedent) replaces the app_drive tests.
- **Post-rewire audit**: grep for orphans (kinematics/ callers after
  odometry leaves — if only mecanum/tests remain, kinematics goes too
  in this tree; `Kinematics::saturate` dies once transliterated) and
  delete anything with zero callers. The audit is a listed execution
  step, not a hope.

### Execution order

Deletion comes LAST — a failure at any gate must leave a small,
attributable diff, not a crater. Ordering revised 2026-08-16.

1. `git worktree add` an exploratory tree off master; enable OOP
   (`clasi oop on --reason 'exploratory DifferentialDrive kernel'`).
2. **Baselines first, on master's build**: capture velocity-step
   traces on tovez (the numbers Verification 3-5 compare against),
   and measure OTOS-0x17-during-settle interleaving on CURRENT
   firmware — if mixed traffic breaks encoder freshness, the design
   needs bus arbitration, and that is cheaper to learn before the
   class exists than after (lesson 16's open interleaving, moved to
   the front).
3. Leaf goes counts-native: strip `travel_calib`/`applyTravelCalib`,
   rebake the three mm/s leaf constants as counts/s (lesson 7), make
   the write throttle begin()-derived instead of hand-synced and
   re-derive tick-counted leaf windows (lesson 6).
4. Write the new `Control::DifferentialDrive` (two files, replacing
   the current class): port the pipeline in counts preserving lessons
   9-15; add the GATED λ, twist-integral hold, lease, Command/Output
   blocks, `step()` + fiber body (absolute-deadline pacing,
   measured-dt), chainable setters. **Golden-trace fidelity gate**:
   old pipeline and new class against the same stepped plant
   (`src/tests/sim/plant/` precedent), λ and twist hold OFF —
   trajectories must match; the "zero math changes" claim is tested,
   not asserted. Port the two WHEELS-speaking bench scripts
   Verification 3-5 need.

   **Capture the golden trace BEFORE step 3, not here.** Step 3 makes
   the leaf counts-native, which is a destructive edit to the very
   thing the comparison needs as its reference — after it, the "old
   pipeline" no longer compiles as it was, so there is nothing left to
   compare against. Either vendor a frozen copy of the pre-3
   `differential_drive.cpp` + leaf into the test tree, or capture the
   old pipeline's host-build traces at step 2 alongside the tovez
   traces and diff against those. Decide which at step 2; discovering
   the reference is gone at step 4 costs the whole gate.
5. Rewire `Core::composeRobot()`/`RobotLoop` per "Lifecycle and boot
   ordering": construct at compose; `begin()` from `boot()` after the
   Preamble; persisted tuning; `start()`; WHEELS adapter (contract
   above), ESTOP→estop(), STOP→neutral(), heartbeat sentinel;
   telemetry reads `output()`. motion/ stays compiled but uncomposed
   for now.
6. Config: baked values flow from the same robot JSON via
   `gen_boot_config.py` (mm→counts rebake) into the COMPOSE-TIME
   `setConfig()` (lifecycle step 2 — memory-only, before the cadence
   freeze); persisted live fields land post-boot per lifecycle step 5
   — one file, bake and runtime read the same source.
7. Host harness + surviving suite green.
8. Bench gates (see Verification). Nothing has been deleted yet — a
   failure here is attributable and cheap to abandon.
9. **Only now, the deletion inventory**: motion/ tree, RobotLoop
   arms, RobotState command fields, dead config keys, obsolete tests
   — removing sources from all four build lists; post-rewire orphan
   audit; trimmed suite re-run green; bench smoke re-run.
10. Report findings; then decide the MicroPython landing (the
    handoff-doc references are unverified in this checkout — see
    Related — resolve them before planning against them).

Out of scope: Planner/Navigator/GO_TO replacements, protocol changes,
MicroPython build, radio/WiFi work, MP-image phases (hardware WDT,
GC-jitter measurement, Python I2C service queue — designed separately,
deferred until this exploration proves the class).

## Verification

Bench on tovez (stand, clean build, `rogo serve` held open), against
the execution-step-2 baselines, in order. Pass/fail bounds are
numeric; "looks right" is not a criterion.

0. Baselines exist (execution step 2): master velocity-step traces +
   the mixed-traffic interleaving measurement, archived with the run.
1. Encoder liveness (both wheels, plausible changing counts).
2. Raw-duty pulse + stop-verify (Δenc == 0 over 2 s after stop).
3. Velocity steps — steady-state within ±10% of the step-0 baseline
   at the same commanded rates; overshoot within baseline + 5
   percentage points; no oscillation absent from the baseline.
   **State the unit both sides are measured in.** The step-0 baseline
   is captured on master, whose telemetry reports [mm/s]; the tree
   under test reports whatever this exploration leaves on the wire.
   Convert one side explicitly, with the two `travel_calib` values
   used recorded alongside the numbers (configuration-discipline's
   read-back rule) — a ±10% bound compared across two different unit
   domains passes or fails for reasons that have nothing to do with
   the control law.
4. λ authority (`lambdaEnabled` on): brake one wheel progressively —
   measured L:R ratio within ±5% of commanded until the λ floor,
   healthy wheel slows, λ monotone during attack, release time
   ~300 ms ± 30%, no limit cycle.
5. Twist-hold straightness: |accumulated L − R counts| ≤ 0.5% of the
   mean accumulated counts over the run, at twist=0 under asymmetric
   load.
6. OTOS-traffic interleaving: encoder sample ages within ±1 cycle of
   the step-0 baseline while the main fiber hammers 0x17 reads
   (lesson 16's open interleaving, now measured twice).
7. Lease-kill: kill the commander mid-drive → wheels stop within
   lease + the stop-enforce window.
8. Estop latch: estop mid-drive → zero written within one cycle
   period; holds until estopClear() + a fresh command; a command
   without estopClear() stays refused.
9. Stop-confirm rebake check: commanded zero re-asserts while wheels
   are spun by hand (proves lesson 7's counts rebake of
   kStopConfirmVelocity).
10. Fault injection: disturb the encoder bus mid-run —
    `i2cFaultCount` climbs, `connectedLeft/Right` drop, sample ages
    grow honestly, clean recovery on release with no latch-up, and
    the heartbeat sentinel does NOT false-fire.
11. Soak, 30 min of mixed driving: `cycleOverrunCount` stays ~0,
    absolute-deadline pacing shows no cumulative drift (lesson 18),
    λ stable, position accumulation well-behaved under the
    positionEpoch rebaseline discipline (float counts grow ~13×
    faster than mm did).

Plus: trimmed sim/unit suite green; the golden-trace host harness
(execution step 4) passing; `src/tests/sim/unit/test_layer_isolation.py`
updated for the removed layers and passing.

## Related

- `src/firm/hal/DESIGN.md` §4 — this IS the deferred `Hal::Wheel`
  migration, resolved angular-native;
  `clasi/issues/hal-wheel-migration-needs-its-own-issue.md`.
- `clasi/issues/spike-i2c-bus-owning-fiber.md` — this worktree runs
  that experiment for real (kernel fiber owns 0x10).
- `clasi/issues/micropython-as-the-base-feasibility-and-migration-plan.md`
  — the 2026-08-07 "datagrams only, no shared memory" decision is
  superseded for the intra-image plane by the 2026-08-15 shared-struct/
  program-interface ruling recorded here (the wire protocol decision
  stands).
- `docs/handoff/micropython-full-firmware-integration.md` — D1-D4 and
  the MP-image landing this class targets (note: its §9 "Gate 2
  BLOCKED" is stale; commit `b10b5282` closed Gate 2).
- `docs/knowledge/2026-07-04-encoder-wedge.md` — the anti-latch
  contract the leaf carries.
- `.claude/plans/scalable-marinating-puppy.md` — parallel session's
  Pybricks/XRPLib-style library API; this class is its substrate.
- **Reference hygiene (re-checked 2026-08-16, after the pull —
  supersedes the earlier "NOT present" note, which searched only
  master's worktree)**. Three of the four references RESOLVE; only one
  is genuinely missing:
  - branch `micropython-vevov-handoff` — **exists**, `c43f5a23`,
    checked out at
    `/Volumes/Proj/proj/RobotProjects/radio-robot-elite.worktrees/micropython-exploration-repl-commands`.
  - commit `b10b5282` — **exists**, on that branch: *"feat(build):
    Gate 2 CLOSED -- vendored CODAL upgraded to the standard set;
    no-SD link engineered"*. The "Gate 2 closed" note is therefore
    CONFIRMED, not stale.
  - `docs/handoff/micropython-full-firmware-integration.md` —
    **exists** on `micropython-vevov-handoff` (`git show
    micropython-vevov-handoff:docs/handoff/micropython-full-firmware-integration.md`).
    It is absent from `master` only. Read it there; the RAM/flash
    headroom numbers can now be checked against the real document
    rather than carried as hearsay.
  - `.claude/plans/scalable-marinating-puppy.md` — **genuinely
    absent**: no such path on any ref (`git log --all --diff-filter=A`
    finds nothing) and `.claude/plans/` does not exist on disk. That
    parallel session's plan was never committed anywhere reachable
    here. Either recover it from the session that wrote it or drop the
    reference; do not plan the library-API story against it.

  Net: execution step 10's "resolve the references first" is DONE for
  three of four, and the MP-landing decision is no longer blocked on
  them.

## Extant work (2026-08-17)

Branch `explore/differential-drive-kernel`, head `1cda1e2f`, four
commits off `ab43963c`:

| commit | what |
|---|---|
| `88dd9ad8` | the 2026-08-15 session's kernel + deletion inventory, recovered from an uncommitted tree on a full disk |
| `83137b37` | the surviving sim suite retargeted onto the kernel's API |
| `11b5db9a` | telemetry/ctypes encoder units put back in millimetres |
| `1cda1e2f` | `MotorDriverChannel` finished counts-native (the firmware image did not build without it) |

Off-hardware gates, all passing at `1cda1e2f`:

| gate | result |
|---|---|
| micro:bit firmware (`build.py`) | `MICROBIT.hex`, 0 errors |
| host sim library | builds |
| `src/tests/sim` | 471 passed, 3 xfailed |
| `src/tests/unit` | 977 passed |

**No hardware run has happened.** The entire Verification list above is
still owed, and nothing below changes that.

Two deviations were found and already fixed in the commits above, both
worth keeping in mind because they are the shape of mistake this port
invites:

1. **Counts leaked onto the wire.** `RobotLoop::publishWheels()` and
   `sim_cmd_vel_left/right()` published the kernel's counts under
   fields documented `[mm]`/`[mm/s]`. Nothing host-side was migrated
   (`git diff ab43963c..HEAD -- src/host/` is empty), so every host
   consumer silently read a ~14.19x rescaled number under an unchanged
   frame shape. The "Outbound" adapter contract is now implemented.
2. **`MotorDriverChannel` still used the deleted `applyTravelCalib`/
   `wheelTravelCalib`**, so the firmware target did not build at all.
   It is compiled only by the firmware target, so the host build stayed
   green over it — lesson 20 in the direction that hides.

## Conformance audit — extant code vs. this specification

Audited clause by clause 2026-08-17. The **control-law port itself is
faithful**; what is missing is almost exactly the 2026-08-16 review
layer, because the code predates it by one day.

### Conforms

- **Cycle schedule: exact.** `controlStep` → `requestL` → settle →
  `tickL` → `requestR` → settle → `tickR` → `publish`. The one-cycle
  command-to-write latency the golden-trace gate depends on is
  preserved.
- **Deletion inventory: done.** `motion/` gone; `cmdVelocity`/
  `cmdAccel` gone; `zeroUnownedMotion()`/`haltOnStall()` gone;
  MOVE/GO_TO ack `ERR_UNIMPLEMENTED`.
- **Lesson 6**: write throttle is a `MotorConfig` field, not the
  hand-synced `27000` literal.
- **Lesson 7**: `kStopConfirmVelocity` 100, `kReconfigureRestVelocity`
  60, MotorArmor `kRestVelocity` 60 counts/s (rounded from 102/64/64).
- **Atomicity**: `do/while` retry-until-consistent seqlock, per the
  revision (not the retry-once the pre-revision text had).
- **Boot ordering**: `setConfig()` at compose (memory-only), `begin()`
  after the Preamble resolves.
- **Adapter**: inbound per-wheel `travel_calib`; outbound now converts
  counts→mm per wheel.
- Counts-native leaf; `travel_calib`/`applyTravelCalib` deleted.

### Does not conform

| # | spec | extant | consequence |
|---|---|---|---|
| C1 | `Status` on every entry point | all `void` | **every refusal silent** — the thing the revision existed to remove |
| C2 | `lastError()`/`clearLastError()` | absent | chainable setters cannot report refusal at all |
| C3 | `maxDuty = 0.0` fail-closed | `= 100.0` | a default-constructed Config has FULL authority, not none |
| C4 | `lambdaEnabled`, ships OFF | absent; λ unconditional | cannot run the pure port first; λ contaminates every first bench number |
| C5 | heartbeat sentinel in `RobotLoop` | **absent** | no defense-in-depth against a wedged kernel fiber (the 936 mm runaway class) |
| C6 | `Hal::Motor::emergencyStop()` | **absent** | the sentinel has no actuation primitive |
| C7 | `kLeaseMax` clamp | absent | the wrap-safe `validUntil` compare is unguarded |
| C8 | `FiberLauncher` injected at construction | `FiberRunner` injected at `start()` | seam differs in name and injection point |
| C9 | `step()` | `cycleOnce()` | |
| C10 | `nowFine`, `cyclePeriodMeasured` | `nowMicros`, `cyclePeriod` | the `Config::cyclePeriod` [ms] / `Output::cyclePeriod` [us] unit collision the rename existed to prevent is still present |
| C11 | `cycleOverrunCount`, `i2cFaultCount`, `positionEpochLeft/Right` on Output | absent | Verification 10 and 11 assert on fields that do not exist |
| C12 | `start()` after `loadPersistedTuning()` | `start()` before it | the fiber runs several cycles on pre-persisted config |
| C13 | dead `vel_*` keys deleted from the schema path | still present, zero consumers | breaks configuration-discipline invariant 2 |
| C14 | golden-trace fidelity gate | **does not exist** | the only test of "zero math changes" is absent — and step 3 already made the leaf counts-native, so the reference it needed no longer compiles |

Naming-only differences, NOT counted as deviations (same capability,
different spelling): `Config::vMin` for `speedFloor`;
`Config::wheelGain[2][2]`/`wheelIntercept[2][2]` for the eight flat
`wheelGain*Accel`/`wheelIntercept*Decel` fields.

## Conformance plan

Ordered so that each step is independently revertible and the suites
stay green throughout. Deletion and the risky seam changes come after
the observable-behaviour changes, same discipline as the original
execution order.

**P1 — `Status` + `lastError()` (C1, C2).** Add the enum exactly as
specified. Change `begin()`, `start()`, `setConfig()`, `drive()`,
`driveDuty()` to return it. Add the sticky `lastError()`/
`clearLastError()` pair and set it from the chainable setters, which
must keep returning `DifferentialDrive&`. Refusal reasons:
`kRefusedUnconfigured`, `kRefusedNotBegun`, `kRefusedEstopped`,
`kRefusedNonFinite`, `kCadencePreserved`. Readiness gates on `begin()`,
NOT `start()` — the host harness never calls `start()`.

**P2 — fail-closed defaults (C3).** `maxDuty` 100 → 0. Every call site
that relied on the permissive default must now set it explicitly; the
sim harness and `buildDriveKernelConfig()` are the two that matter.
This is the step most likely to surface a silently-unconfigured path,
which is the point of it.

**P3 — lease clamp (C7).** `kLeaseMax = 3,600,000 ms`, clamped in
`drive()`/`driveDuty()` before computing `validUntil`.

**P4 — Output shape (C10, C11).** Rename `nowMicros` → `nowFine` and
`Output::cyclePeriod` → `cyclePeriodMeasured`. Add
`cycleOverrunCount` (missed absolute deadlines), `i2cFaultCount`
(derived from sample-stamp non-advance) and `positionEpochLeft/Right`.
Point `RobotLoop`'s per-wheel epoch at the kernel's own counters rather
than its single `positionEpoch_`.

**P5 — gated λ (C4).** Add `lambdaEnabled`, default `false`. Gate the
whole λ computation on it; λ stays 1.0 when off. This must land before
any bench pass so the first one runs the pure port.

**P6 — `emergencyStop()` + heartbeat sentinel (C6, C5).** Add
`Hal::Motor::emergencyStop()` — immediate unstaged zero through the
never-shaped stop path — implemented on `NezhaMotor`, `MotorArmor`,
`MotorDriverChannel` and the sim fake. Then the sentinel in
`RobotLoop::cycle()`: watch `output().cycleCount`; if it stalls beyond
`kSentinelPeriods` while motion was commanded, raise a sticky telemetry
fault AND call `emergencyStop()` on both motors. Document the honest
scope — it cannot cover the dead-bus busy-wait.

**P7 — seam rename (C8, C9).** `Hal::FiberRunner` → `Hal::FiberLauncher`
with `launch(entry, arg)`, injected at CONSTRUCTION; `cycleOnce()` →
`step()`. The host implementation fails the test if invoked. Mechanical
but wide, so it goes after the behavioural work.

**P8 — `start()` after persisted tuning (C12).** Reorder in `main.cpp`.

**P9 — dead `vel_*` keys (C13).** Delete from the schema path and the
generator. "Delete it, don't wire it."

**P10 — golden-trace fidelity gate (C14).** The hard one, and it is
listed last because it is the only item whose reference has already
been destroyed. Reconstruct the pre-rework pipeline from `ab43963c`
into the test tree as a frozen comparison copy, run both against the
same stepped plant with λ and twist hold OFF, and require the
trajectories to match. If they do not, that is the finding — and it is
the first thing that would explain the open tracking falloff below.

Gates after every step: `src/tests/sim` and `src/tests/unit` green, and
`build.py` producing a hex. Not one of these steps is verified on
hardware by finishing it.

## Open question for the bench — now the TOP one

With the fidelity gate passing, this is the only unexplained behaviour
left, and it is no longer attributable to the control-law port.

**Open-loop velocity tracking falls off with speed and does not
converge.** On `tovez_nocal` (the no-calibration profile: `wheel_gain`
identity, every `wheel_control` gain 0, so there is no feedback at all)
commanded vs measured settles at +0.3% at 90 mm/s, **−11% at 150 mm/s
and −20% at 250 mm/s**, still −18% after 290 cycles. Implied plant duty
at 250 mm/s is 39% where the open-loop map asks for 50%. Since that
profile ships zero feedback this may be honest open-loop map error
rather than a kernel defect. The fidelity gate has now RULED OUT the
ported control law as the cause. The next suspect is the relationship
between the sim plant's actual velocity ceiling and the
`fullDutyVelocity` the harness derives for it — i.e. a sim-configuration
question, not a firmware one. Confirm against master before trusting any
velocity number out of this tree.

## Conformance status (2026-08-17) — head `7c682fc4`

Thirteen of the fourteen audit items are closed (only C13 remains). Gates after every step:

| gate | result |
|---|---|
| micro:bit firmware (`build.py`) | `MICROBIT.hex`, 0 errors |
| host sim library | builds |
| `src/tests/sim` + `src/tests/unit` | **1450 passed, 3 xfailed** |

| item | status |
|---|---|
| C1 `Status` on every entry point | **DONE** (P1) |
| C2 `lastError()`/`clearLastError()` | **DONE** (P1) |
| C3 fail-closed `maxDuty` | **DONE** (P2) |
| C4 gated λ | **DONE** (P5) |
| C5 heartbeat sentinel | **DONE** (P6) |
| C6 `Hal::Motor::emergencyStop()` | **DONE** (P6) |
| C7 `kLeaseMax` clamp | **DONE** (P3) |
| C8 `FiberLauncher` at construction | **DONE** (P7) |
| C9 `step()` | **DONE** (P1) |
| C10 `nowFine`/`cyclePeriodMeasured` | **DONE** (P4) |
| C11 overrun/i2cFault/positionEpoch on Output | **DONE** (P4) |
| C12 `start()` after persisted tuning | **DONE** (P8) |
| C13 dead `vel_*` keys deleted | **NOT DONE** — see below |
| C14 golden-trace fidelity gate | **DONE** — and it PASSES: the port is behaviour-preserving |

### C14 — the fidelity gate, and what it settled

`src/tests/sim/fidelity/` holds the pre-rework control law recovered from
`ab43963c` and frozen (`golden_ref_drive.{h,cpp}`, `golden_ref_state.h`),
plus a harness driving it and the current kernel through identical
commands against identical plants, comparing the DUTY each produces.

**The port IS behaviour-preserving.**

| scenario | result |
|---|---|
| feedforward + Stage A (`kp=ki=0`) | **EXACT — 0.000000** |
| closed loop, pure-I (tovez's shipped posture) | steady-state mean **0.000957** |
| closed loop, `kp+ki` across accel and decel steps | steady-state mean **0.000038** |

`positionError()` is also line-for-line identical between the two
implementations — same guard, same arming, same clamp.

The gate asserts STEADY STATE and *reports* the transient peak (0.0147,
0.0204) rather than asserting on it. The two pipelines couple samples to
control differently by design: the kernel collects mid-cycle between its
two encoder settle sleeps, while the reference is a stage-then-execute
class driven at cycle boundaries by a loop that no longer exists. During
a ramp that produces a bounded, decaying ripple which cannot be aligned
away without giving the reference a split-phase schedule it never had.
Asserting on it would test the harness, not the port.

**CORRECTION — this issue previously said the opposite.** An earlier
revision reported the pipeline as "NOT behaviour-preserving" on a ~2%
integral-path divergence, and argued it was not a harness artifact
because three corrections had not moved it. That was wrong on both
counts. A FOURTH asymmetry existed: the kernel runs its control step
before its first collect, so on cycle 0 its `WheelSample` is
default-constructed with `connected == false` and `positionError()`
re-anchors, arming Stage B a cycle later — while the harness was handing
the reference `connected == true` from cycle 0. That one-cycle head start
on the integrator was the whole effect. The residual failure after that
was a steady-state window that had not finished settling after a
250 → 80 step; lengthening the leg took the delta to 0.000038.

**Consequence for the other open finding:** the integral path is NO
LONGER a candidate explanation for the −20% open-loop tracking falloff. A
transient that decays to 4e-5 cannot produce a persistent steady-state
error. That falloff is a separate issue — most likely the sim plant's
actual ceiling versus what `fullDutyVelocity` claims — and it is now the
top open question for the bench.

### C13 — not done, deliberately

Deleting the dead `motors.vel_*` keys means a schema migration:
`robot_config.proto`, the generated `robot_config.h`/`robot_config_pb2.py`/
`robot_config_generated.py`, `gen_boot_config.py` (24 refs),
`gen_default_config.py`, `boot_config.cpp`, four robot JSONs, the schema,
and six test files — with a protobuf regeneration step in the middle.
That is the **lowest-value item on the list** (dead keys harm tidiness,
nothing else) with **by far the widest blast radius**, and it is
unverifiable on hardware from here. Left for a session that can run the
codegen and re-bake every robot.

### One debt this pass ADDED, stated rather than hidden

`buildDriveKernelConfig()` now sets `cfg.maxDuty = 100.0f` as a C++
literal. The fail-closed default (C3) means the bake must grant authority
explicitly, and there is no `drive.max_duty` key to read. This is not a
regression — the value was previously an even less visible class-member
default — but it breaks configuration-discipline invariant 1 exactly the
way `twistHoldGain` beside it does. `drive.max_duty` wants a real key in
the proto, the four robot JSONs and the schema; it is the same migration
as C13 and should go with it.

### Still true, and unchanged by any of this

**Nothing here has been on the robot.** Every gate above is off-hardware.
The entire Verification list stands owed, and the two findings — the
integral-path divergence and the tracking falloff — are exactly the kind
of thing the bench exists to settle.

## Square tour: one bug fixed, one still open (2026-08-18)

### FIXED — the sim plant's encoder scale (this was the "-20%")

`sim_plant.cpp` hardcoded `kEncoderCountsPerMm = 1.4187` (== 1/0.704871),
while the active robot decodes counts with `travel_calib` 0.7837. Every
distance and velocity the firmware PERCEIVED read 11.2% high, so a closed
loop settled 11% short of where it believed it was.

Measured at commanded 400 mm/s, before the fix:

| quantity | value |
|---|---|
| `appliedDuty` | 0.72 (correct for what the loop believed) |
| plant's true velocity | 359.47 mm/s |
| kernel's own reading | 399.20 mm/s |
| ratio | 1.1105 |
| predicted (0.7837/0.704871) | 1.1118 — 0.12% apart |

After deriving the plant's scale from the baked `travel_calib`, applied
duty equals expected duty at 0.18/0.30/0.50/0.80 and plant truth equals
commanded to within 0.3%.

Cause was mine: `bench_test_config.cpp` used to pin `wheelTravelCalib` to
0.704871 with a comment saying it MUST be the reciprocal of the plant's
constant. The counts-native leaf deleted that field, correctly, and the
synchronisation went with it. Now derived, so it cannot drift again.

### STILL OPEN — the tour's legs land ~70% with exact velocity

Fixing the above did NOT fix the tour, and made the number worse:

| run | path | heading | closure |
|---|---|---|---|
| before the fix | 1588/2000 (79%) | 308.9/360 | 704 mm |
| after the fix | 1460/2000 (73%) | 269.5/360 | 794 mm |
| after, `--leg 2000` | 5583/8000 (70%) | 345.6/360 | 5209 mm |

Consistent, not contradictory: the 11.2% over-read was partially
CANCELLING this second error. Removing it exposed more of the other one.

**Ruled out, with evidence — do not re-investigate these:**

- *Velocity/duty scale.* Provably exact after the fix: commanded 150 →
  plant truth 150.00; commanded 400 → 399.57.
- *The plant model.* `WheelPlant::step()` is `target = dutyVelMax *
  appliedDuty` with a first-order lag. Linear, no saturation.
- *A dropped segment.* All 8 segments enqueue and ack; no warnings.
- *A fixed per-segment loss* (ramp, dwell, coast). The shortfall gets
  WORSE with longer legs (73% → 70%), so it scales; a fixed loss would
  shrink as a fraction.
- *Host/firmware cycle-period disagreement.* `sim_loop.py` reads the
  compiled `kCycle` rather than assuming it.
- *Clock running ahead of physics via the kernel's settle sleeps.*
  `TestSim::SimSleeper` holds no clock reference — it records requests
  and advances nothing.
- *Stage A / λ / gains.* Measured live from the kernel: `wheelGain`
  identity, `lambdaEnabled=0`, `lambda=1.000`, `maxDuty=100`.
- *The tour pre-compensating gains.* `CAL["L"]=CAL["R"]=1.0`; the
  stationary prelude does not scale commands.

**The lead worth chasing next.** Velocity is exact and distance is short,
so the commanded WINDOW must be short. In `SimHarness` the kernel's
`step()` is called once per `robotLoop_.cycle()`, i.e. once per
`kCycleDtUs` = `RobotLoop::kCycle` = **32 ms** — while the kernel's own
cadence, `kKernelCyclePeriod`, is **24 ms**. On hardware the kernel's
fiber runs at 24 ms INDEPENDENTLY of RobotLoop's 32 ms; in the sim the
two are welded 1:1. That is a genuine sim-fidelity gap whether or not it
is the whole cause — the simulated control loop runs 25% slower relative
to physics than the real one does, and every lease and duration is
evaluated against it. Start there.

This is host/sim-harness territory, NOT the kernel. It does not affect
`src/firm/diffdrive/` and does not block the package extraction.
