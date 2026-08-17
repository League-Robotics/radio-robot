---
status: pending
---

# DifferentialDrive: one class, one fiber — exploratory worktree

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
  struct Config {                    // value type -- fetch/replace by copy
    float maxDuty = 100.0f;          // [%] authority rail
    float fullDutyVelocity = 0.0f;   // [counts/s] wheel rate at 100% duty;
                                     //   0 = uncalibrated, VELOCITY refused
    float kp = 0.0f;                 // [1]
    float ki = 0.0f;                 // [1/s] on clamped position error
    float iMax = 0.0f;               // [counts/s]
    float kaff = 0.0f;               // [s] accel feedforward
    float pidMax = 0.0f;             // [counts/s]
    float twistHoldGain = 0.0f;      // [1/s] twist-integral ratio hold; 0 = off
    float stallSpeed = 0.0f;         // [counts/s]
    float stallDemand = 0.0f;        // [counts/s] 0 = detector off
    float stallWindow = 0.0f;        // [ms]
    uint32_t cyclePeriod = 24;       // [ms] fiber cadence
  };

  struct Output {                    // value type -- output() returns a copy
    uint32_t now;                    // [ms] kernel clock at publish
    uint32_t nowMicros;              // [us] same instant -- age-math base
    uint32_t cycleCount;             // heartbeat
    uint32_t cyclePeriod;            // [us] measured (feeds all dt terms)
    uint32_t cycleBusy;              // [us]
    uint32_t sampleTimeLeft;         // [us] last SUCCESSFUL L collect
    uint32_t sampleTimeRight;        // [us] ~4-5 ms younger than L
    float positionLeft;              // [counts] accumulated, never device-reset
    float positionRight;             // [counts]
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
    uint32_t i2cFaultCount;
  };

  DifferentialDrive(Hal::Motor& left, Hal::Motor& right,
                    const Hal::Clock& clock);

  // ---- config surface 1: chainable single-field setters ----
  DifferentialDrive& setMaxDuty(float maxDuty);            // [%]
  DifferentialDrive& setFullDutyVelocity(float velocity);  // [counts/s]
  DifferentialDrive& setKp(float kp);
  DifferentialDrive& setKi(float ki);                      // [1/s]
  DifferentialDrive& setIMax(float iMax);                  // [counts/s]
  DifferentialDrive& setKaff(float kaff);                  // [s]
  DifferentialDrive& setPidMax(float pidMax);              // [counts/s]
  DifferentialDrive& setTwistHoldGain(float gain);         // [1/s]
  DifferentialDrive& setStall(float speed, float demand,
                              float window);   // [counts/s] [counts/s] [ms]
  DifferentialDrive& setCyclePeriod(uint32_t period);      // [ms]
  // ---- config surfaces 2 + 3: whole-block replace, fetch (copy) ----
  void setConfig(const Config& config);
  Config config() const;

  // ---- lifecycle: start the object, then start the fiber ----
  void begin();          // hardware init: prime encoders, boot zero-write
  void start();          // launch the kernel fiber (idempotent)
  bool running() const;

  // ---- commands; lease is a DURATION from now, expiry stops ----
  void drive(float velocity, float twist,
             uint32_t lease);         // [counts/s] [counts/s] [ms]
  void driveDuty(float dutyLeft, float dutyRight,
                 uint32_t lease);     // [%] [%] [ms]
  void neutral();        // commanded stop through the full stop path
  void estop();          // latch: zero NOW; holds until estopClear()
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
  void run();            // the kernel fiber body
  // + config snapshot (seq'd), internal Output instance (seq'd),
  //   estop latch (single u32, outside the seq handshake),
  //   ported pipeline state (position refs, biases, latch timers)
};

}  // namespace Control
```

### Interface semantics

- **Lease is relative**: `drive(v, t, 300)` = valid 300 ms from now.
  The class computes absolute expiry against its own clock internally —
  no clock-domain coupling in the API. Expiry zeroes through the full
  stop path (stop-enforce countdown, stopNotTaken), consulting nobody;
  it only ever moves motion toward zero (the `zeroUnownedMotion`
  monotone contract restated as a timestamp). Refreshing the command IS
  feeding the watchdog.
- **Estop is not a mode**: a kernel-side latch set outside the seq
  handshake (single atomic u32); cleared only by `estopClear()` + a
  fresh command.
- **Config is live**: the fiber snapshots the config block (under seq)
  at each cycle start, so chained setters work mid-run — preserves the
  bench live-tuning workflow. Write shaping (reversal dwell, deadband,
  slew) stays motor-leaf config.
- **Uncalibrated is visible**: `fullDutyVelocity == 0` refuses VELOCITY
  mode with `ready == false` — not the old silent no-write return.
- **Atomicity**: cooperative fibers on one core; aligned 32-bit stores
  are atomic; each block is single-writer, seq-committed; readers copy
  under a seq check (retry once). Methods are called from the main
  fiber; the kernel fiber is the only writer of Output.

### Measurement age semantics

- `sampleTimeLeft/Right` are stamped at collect SUCCESS only (the
  131-002 rule from `NezhaMotor::lastFreshUs_`): failed collects HOLD
  the stamp so age grows honestly; `connectedLeft/Right` complement it.
- Age = `(int32)(nowMicros − sampleTime)` — wrap-safe signed difference.
- L and R differ by construction: sequential split-phase collects L,
  then R one settle window later — R deterministically ~4-5 ms younger;
  an observer can model the fixed skew. Fault-driven divergence (ages
  separating by whole cycles) is the per-wheel staleness signal.
- Per-wheel velocity is computed over the GENUINE inter-sample
  interval, not the nominal cycle period.

### Control design (2-DOF: mean + differential)

- Per-wheel targets: `left = velocity − twist`, `right = velocity +
  twist` — the commanded RATIO is the (v, twist) pair.
- **Authority headroom (λ)**: from last cycle's pre-clamp duty demands,
  `λ = min(1, maxDuty/|dutyDemandLeft|, maxDuty/|dutyDemandRight|)`,
  asymmetric-filtered (fast attack, ~300 ms release); both targets
  scale by λ — curvature preserved, the healthy wheel slows when the
  other saturates. (Min-lambda shape from `Kinematics::Model::saturate`,
  `src/firm/kinematics/kinematics.h:76` — tested, zero callers.)
- **Twist-integral hold** (encoder-only ratio maintenance — NOT a
  heading feature): integral of commanded twist vs measured
  differential position, trim clamped to duty headroom. twist=0 →
  equal accumulated counts (straight as wheels can know); twist≠0 →
  the commanded arc. `twistHoldGain` 0 disables.
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
  moves up to the application.

### The fiber and the main loop

The kernel fiber free-runs `run()`: snapshot command+config → estop/
lease check → requestL → settle (fiber_sleep) → collectL → requestR →
settle → collectR → control step (λ, twist hold, PID, shaped writes; no
yields) → publish Output → sleep to the ABSOLUTE next-cycle deadline.

`Core::RobotLoop` no longer calls into the drive at all. The main fiber
keeps comms, telemetry, and sensors (OTOS et al.), interacting only via
class methods; a thin adapter maps WHEELS-style v5 commands to
`drive()` calls so rogo/bench tooling keeps working.

- The kernel's settle windows fiber_sleep — yielding to the main fiber,
  which is where comms pumping already happens; the current design's
  "spend the settle usefully" property is preserved by construction.
- Bus safety: CODAL I2C transactions never yield mid-transaction
  (fiber-atomic); per-device clearances are per-address; the kernel is
  the ONLY 0x10 client, so select→collect ordering is preserved
  structurally. Main-fiber OTOS traffic (0x17) CAN land inside a kernel
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
   include core headers. The new class has a CONFIGURABLE cyclePeriod →
   this must become config-derived (new MotorConfig field, set at
   compose from `Config::cyclePeriod`), not a stale literal.
7. **TRAP — leaf constants are in mm/s**: `kStopConfirmVelocity` (8),
   `kReconfigureRestVelocity` (5), MotorArmor's `kRestVelocity` (5) are
   mm/s thresholds. Counts-native `position()`/`velocity()` changes
   their effective meaning by ~10× (1 count = 0.1°; tovez travel_calib
   0.7837 mm/deg). Rebake all three as counts/s constants explicitly —
   silent unit drift here disarms the stop-confirm safety net.
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
    sole 0x10 client, ever. Foreign-address interleaving during settle
    is unmeasured (wedgelab mixed-traffic says fine) → explicit bench
    check.
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
- **`Core::RobotLoop`**: `zeroUnownedMotion()` (subsumed by the lease),
  `haltOnStall()` (stall halt is in-class now), MOVE/GO_TO routing and
  handlers, `publishMoveResult`/`publishGotoResult`,
  `ackDriveCompletion`, odometry integration + `publishPose`'s encoder
  half, the drive tick/update calls and settle-window scheduling (all
  now inside the class fiber). RobotLoop keeps: comms pump, config
  routing, telemetry emit, OTOS/line/color ticks, the WHEELS→drive()
  adapter, ESTOP→estop(), STOP→neutral().
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

1. `git worktree add` an exploratory tree off master; enable OOP
   (`clasi oop on --reason 'exploratory DifferentialDrive kernel'`).
2. Leaf goes counts-native: strip `travel_calib`/`applyTravelCalib`,
   rebake the three mm/s leaf constants as counts/s (lesson 7), make
   the write throttle config-derived instead of hand-synced (lesson 6).
3. Write the new `Control::DifferentialDrive` (two files, replacing the
   current class): port the pipeline in counts preserving lessons 9-15;
   add λ, twist-integral hold, lease, Command/Output blocks, fiber body
   (absolute-deadline pacing, measured-dt), chainable setters.
4. Execute the deletion inventory: motion/ tree, RobotLoop arms,
   RobotState command fields, dead config keys, obsolete tests —
   removing sources from all four build lists.
5. Rewire `Core::composeRobot()`/`RobotLoop`: construct + `begin()` +
   `start()`; WHEELS→drive() adapter, ESTOP→estop(), STOP→neutral();
   telemetry reads `output()`.
6. Config: baked values flow from the same robot JSON via
   `gen_boot_config.py` (mm→counts rebake) into a `setConfig()` call at
   compose time — one file, bake and runtime read the same source.
7. Post-rewire orphan audit; trimmed test suite green; host-build
   harness for the new class passing.
8. Bench (see Verification).
9. Report findings; then decide the MicroPython landing (this class +
   a modrobot binding IS the kernel-fiber plan in
   `docs/handoff/micropython-full-firmware-integration.md` D1-D4).

Out of scope: Planner/Navigator/GO_TO replacements, protocol changes,
MicroPython build, radio/WiFi work, MP-image phases (hardware WDT,
GC-jitter measurement, Python I2C service queue — designed separately,
deferred until this exploration proves the class).

## Verification

Bench on tovez (stand, clean build, `rogo serve` held open), in order:

1. Encoder liveness (both wheels, plausible changing counts).
2. Raw-duty pulse + stop-verify (Δenc == 0 over 2 s after stop).
3. Velocity steps — measured counts/s tracks commanded within the
   standard firmware's tuning envelope.
4. λ authority: brake one wheel progressively — measured ratio holds,
   healthy wheel slows, λ trace smooth, clean release on the slow
   filter.
5. Twist-hold straightness: equal accumulated counts at twist=0 under
   asymmetric load.
6. OTOS-traffic interleaving: encoder freshness unchanged while the
   main fiber hammers 0x17 reads (lesson 16's open interleaving).
7. Lease-kill: kill the commander mid-drive → wheels stop within
   lease + stop-enforce.
8. Estop latch: estop mid-drive, verify hold until estopClear() + new
   command.
9. Stop-confirm rebake check: commanded zero re-asserts while wheels
   are spun by hand (proves lesson 7's counts rebake of
   kStopConfirmVelocity).

Plus: trimmed sim/unit suite green; host-build harness for the new
class passing; `src/tests/sim/unit/test_layer_isolation.py` updated for
the removed layers and passing.

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
