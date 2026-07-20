---
id: '001'
title: 'Config-completeness gate: fail-closed motion refusal + delete SimHarness''s
  hardcoded planner/motor defaults'
status: open
use-cases:
- SUC-001
- SUC-004
depends-on: []
github-issue: ''
issue: config-as-truth-completion-no-defaults-fail-closed-version-erase.md
completes_issue: true
exception:
  thrown_by: programmer
  thrown_at: '2026-07-20T21:41:19.256474+00:00'
  attempted: 'Implemented steps 1-3 of the ticket''s Approach fully and correctly
    (verified by inspection, no issues): added ErrCode::ERR_NOT_CONFIGURED=8 to envelope.proto
    and regenerated envelope.h via gen_messages.py; added the configured_/markConfigured()/isConfigured()
    gate to App::RobotLoop with the refusal branch as the first statement in handleTwist()/handleMove()
    (handleStop()/handleConfig() left unconditional, confirmed unaffected); added
    robotLoop.markConfigured() to main.cpp right after RobotLoop construction. Then
    implemented step 4 literally: deleted SimHarness::makeMotorConfig()/makeExecutorConfig()
    in full, changed the ctor to construct motorL_/motorR_ with a default-constructed
    Devices::MotorConfig{}, removed the ctor-body armor.configure()/executor-config
    block, added hasConfiguredMotorL_/hasConfiguredMotorR_ tracking plus a maybeMarkConfigured()
    helper called from configurePlanner()/configureMotor() (the AND of all three flags
    calls robotLoop_.markConfigured()), and added the isConfigured() passthrough.
    Implemented step 5 (new src/tests/sim/support/bench_test_config.h/.cpp, TestSupport::benchTestPlannerConfig()/benchTestMotorConfig()/configureSimForBenchTest(),
    values copied byte-for-byte including every comment). To verify step 6 before
    batch-migrating all ~9 actual SimHarness-constructing harnesses (grep found 9
    files under src/tests/sim/**, not ~40 -- see note below), I migrated ONE pilot
    file (straight_twist_harness.cpp: added the #include plus TestSupport::configureSimForBenchTest(sim);
    right after construction, and added bench_test_config.cpp to test_straight_twist.py''s
    compiled-sources list) and ran it. Result: FAILS -- both wheels report velL=0.00/velR=0.00
    (frozen) for the ENTIRE run, even after configureSimForBenchTest() completes both
    configurePlanner() and both configureMotor() calls and isConfigured()==true. Root
    cause traced by full grep of src/firm/devices/nezha_motor.h/.cpp and motor_armor.h:
    Devices::MotorArmor::configure() (called by SimHarness::configureMotor()) reads
    ONLY config.outputDeadband (caches it into its own motionThreshold_) -- confirmed
    by reading motor_armor.h''s configure() body directly. Devices::NezhaMotor''s
    own config_ (port, fwdSign, velGains, velFiltAlpha, slewRate) is assigned exactly
    ONCE, in the constructor (`config_ = config;`, nezha_motor.cpp line 84) -- grepped
    every `config_\.` and `config_\s*=` usage in both files; the ONLY runtime mutator
    is applyGains() (velGains/wheelTravelCalib only, via the Motor interface). fwdSign
    has no runtime setter anywhere. nezha_motor.cpp:519 computes `effective = fwdSign
    * written` for every duty write -- with fwdSign=0 (Devices::MotorConfig{}''s zero
    default), every write is unconditionally zero, forever, regardless of what configureMotor()
    is later called with, because there is no path to change config_.fwdSign after
    construction.'
  conflict: 'Ticket 001''s Approach step 4 ("construct motorL_/motorR_ with a default-constructed
    Devices::MotorConfig{}") is structurally incompatible with two things this same
    ticket (and sprint) commits to: (a) ticket 001''s own acceptance criterion "After
    configurePlanner() + both configureMotor() calls, isConfigured() == true and a
    subsequent TWIST/MOVE is accepted normally," read together with sprint.md''s SUC-001
    postcondition "No motion occurred before step 6" (implying motion DOES occur once
    configured) -- empirically, motion never occurs, ever, post-configuration; and
    (b) sprint.md''s own Architecture "Boundary list," which explicitly classifies
    `fwd_sign` as "Behavioral, must come from data/robots/*.json, no code fallback"
    (grouped with travel_calib/gains) -- yet no mechanism reachable from ticket 001''s
    stated Files-to-Touch (src/sim/sim_harness.h; src/tests/sim/support/bench_test_config.*)
    can ever deliver a config-sourced fwd_sign to the real Devices::NezhaMotor once
    it exists, because Devices::MotorArmor::configure()/Devices::NezhaMotor (src/firm/devices/motor_armor.h,
    nezha_motor.h/.cpp -- NOT in ticket 001''s Files-to-Touch, and designed by 113-002
    with no runtime reconfiguration surface beyond applyGains()) have no port/fwdSign/velFiltAlpha/slewRate
    setter at all. Making SimHarness::configureMotor() "load-bearing for the first
    time" (the ticket''s own phrase) for producing a WORKING motor requires an unscoped
    architecture decision: either extend Devices::NezhaMotor''s public interface with
    new runtime setters for these fields, or make NezhaMotor/MotorArmor reconstructible
    in place -- both belong to devices/, arguably adjacent to ticket 003 (which already
    plans to touch nezha_motor.h for the neighboring reversalDwell/outputDeadband
    fields), not ticket 001. Separately, but compounding the same root cause: constructing
    BOTH motorL_ and motorR_ with the identical zero-valued Devices::MotorConfig{}
    (port=0 for both, since port is likewise construction-only) means TestSim::SimPlant::wheelPlant()''s
    `(port==2) ? right_ : left_` routing would alias both motors onto the SAME simulated
    wheel even if fwdSign were fixed -- a second, independent correctness gap from
    the same "port/fwdSign are construction-time-only" fact, since port is structurally
    exempt per the boundary list but is ALSO never propagated by configureMotor().
    Note on ticket scale: sprint.md/ticket text estimate "~40" pre-existing SimHarness-constructing
    harnesses; an exhaustive grep of src/tests/sim/** (excluding sim_harness_configure_harness.cpp
    per the ticket''s own exclusion, and excluding files that only mention "SimHarness"
    in a comment) found exactly 9 files that actually construct a bare SimHarness
    across 26 total construction sites, plus one Python pytest file (src/tests/sim/test_motor_primitive.py)
    that reaches an unconfigured SimHarness via the ctypes SimLoop path and would
    need the same treatment -- this discrepancy is reported for the resolving agent/team-lead''s
    awareness, not itself the blocker.'
  surface: internal
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Config-completeness gate: fail-closed motion refusal + delete SimHarness's hardcoded planner/motor defaults

## Description

Build the configuration-completeness gate that makes "unconfigured" a real,
refusable state, and delete the hardcoded planner/motor defaults from
`TestSim::SimHarness` that currently make it impossible to ever observe that
state. These two are one unit of work: building the gate without removing the
always-configured default would leave it structurally unreachable, and
removing the default without the gate would leave motion undefined when
nothing has been configured.

## Context

See sprint 114's `sprint.md` Architecture section (Design Rationale Decisions
1-3) for the full reasoning. Summary: `App::RobotLoop` gets one boolean, set
exactly once by whichever atomic boot path configured the whole graph
(`main.cpp`'s existing `Config::default*()` sequence, or `SimHarness`'s
`configurePlanner()`+`configureMotor()` pair). Real firmware is unaffected in
practice — its boot bake is already complete before `RobotLoop::run()`
starts, so `markConfigured()` is an unconditional, always-immediate call. The
sim is where this gate has teeth: today `SimHarness`'s constructor calls
`makeExecutorConfig()`/`makeMotorConfig()` (both private static methods in
`src/sim/sim_harness.h`) unconditionally, so it is *always* pre-configured
with hardcoded stand-in values — this is exactly the divergence that let
sprint 113's own `vel_kp` bug (0.003 hardcoded vs 0.002 configured) exist.
Deleting those two methods with nothing else would silently zero out the 9
existing `src/tests/sim/**` C++ harnesses (26 construction sites,
exhaustively grepped — not the "~40" this ticket originally estimated; see
below) that construct a bare `SimHarness` and immediately drive it, plus one
Python file reached via the ctypes `SimLoop` path. This ticket fixes that by
moving the SAME values (byte-for-byte) into a new, explicitly test-only
header, and adding one line/call to every affected file.

### Exception resolution (sprint-planner, 2026-07-20)

This ticket originally threw an internal/structural exception: step 4 below
("construct `motorL_`/`motorR_` with a default-constructed
`Devices::MotorConfig{}`, configure later") turned out to be structurally
impossible — `MotorArmor::configure()` (the only thing `configureMotor()`
called) forwarded just one field (`outputDeadband`, into its own cached
`motionThreshold_`); the wrapped `NezhaMotor`'s own `config_` was assigned
exactly once, in its constructor, with no runtime setter for
port/fwdSign/velFiltAlpha/slewRate/wheelTravelCalib. A motor built from
`MotorConfig{}` could therefore never become real: `fwdSign=0` zeroed every
duty write, `wheelTravelCalib=0` zeroed `position()` regardless of encoder
ticks, and both ports (both `port=0`) aliased onto the same simulated wheel.
Empirically confirmed: `straight_twist_harness.cpp`, migrated as this
ticket's own pilot file, reported `velL=velR=0.00` for its entire run even
after `configureSimForBenchTest()` completed and `isConfigured()==true`.

Resolved by adding `Devices::Motor::reconfigure()` — a new, guarded,
post-construction whole-config-replacement virtual, implemented by
`NezhaMotor` and forwarded by `MotorArmor` — so `configureMotor()` reaches a
genuinely working motor. Full rationale: sprint.md's Architecture Revision 1
/ Decision 6. This ticket's Approach (new step 5 below), Files to Touch, and
Acceptance Criteria are revised accordingly; everything already landed on
the branch for steps 1-3 and step 4's own two bullets (the `ErrCode`, the
`RobotLoop` gate, `main.cpp`'s `markConfigured()` call, `SimHarness`'s
deleted defaults + `hasConfiguredMotorL_`/`hasConfiguredMotorR_` tracking,
and the new `bench_test_config.h`/`.cpp`) was verified correct by inspection
and is preserved, not redone.

## Approach

1. **New `ErrCode`**: add `ERR_NOT_CONFIGURED = 8;` to the `ErrCode` enum in
   `src/protos/envelope.proto` (after the existing `ERR_OVERSIZE = 7`).
   Regenerate with `python3 src/scripts/gen_messages.py` (or however
   `build.py`'s codegen step invokes it) — this regenerates
   `src/firm/messages/envelope.h`. Never hand-edit the generated header.

2. **`App::RobotLoop` gate** (`src/firm/app/robot_loop.h`/`.cpp`):
   - Add `private: bool configured_ = false;` and two public methods:
     `void markConfigured();` and `bool isConfigured() const;`.
   - In `handleTwist()` and `handleMove()`, as the very first statement: if
     `!configured_`, ack `msg::AckStatus::ACK_STATUS_ERR` with
     `static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED)` and `return`
     immediately — do not touch `drive_`, `pilot_`, or `deadman_`.
   - Do **not** gate `handleStop()` or `handleConfig()` — both must remain
     unconditional (already always-live; this is existing, correct behavior,
     do not change it).
   - Verify (do not need to change) that PING/ECHO/ID/HELLO/DEVICE-banner
     handling lives entirely in `App::Comms`, not in `RobotLoop`'s
     `processMessage()` switch (confirmed during sprint planning via
     `grep -rln "PING\b" src/firm`) — these are unaffected by this gate by
     construction.

3. **`main.cpp`**: immediately after the existing
   `pilot.configureHeading(plannerConfig);` call (the last of the
   boot-configure sequence) and before `robotLoop.run();`, add
   `robotLoop.markConfigured();`. This is the only change to `main.cpp`.

4. **Delete `src/sim/sim_harness.h`'s hardcoded defaults**:
   - Delete the private static methods `makeMotorConfig(uint32_t port)` and
     `makeExecutorConfig()` in full.
   - In the constructor: construct `motorL_`/`motorR_` with a
     default-constructed `Devices::MotorConfig{}` instead of
     `makeMotorConfig(1)`/`makeMotorConfig(2)`. Remove the constructor body's
     `armorL_.configure(makeMotorConfig(1));` /
     `armorR_.configure(makeMotorConfig(2));` and the
     `msg::PlannerConfig cfg = makeExecutorConfig(); executor_.configure(cfg); ...`
     block entirely — `SimHarness` now leaves `executor_`/`headingSource_`/
     `drive_`/`pilot_` at their own default-constructed state after
     construction.
   - `SimHarness` must report `isConfigured() == false` (via
     `robotLoop_.isConfigured()` — add a thin passthrough accessor if
     `robotLoop_` isn't already reachable) immediately after construction,
     and `== true` only after both `configurePlanner()` and both
     `configureMotor(1, ...)`/`configureMotor(2, ...)` calls have landed.
     `configurePlanner()`/`configureMotor()` (113-002) already exist as the
     additive config-load surface — this ticket makes them load-bearing for
     the first time. Track completion with `hasConfiguredPlanner_` (already
     exists) plus new `hasConfiguredMotorL_`/`hasConfiguredMotorR_`; call
     `robotLoop_.markConfigured()` from whichever of the three calls
     completes the set. `markConfigured()` itself is idempotent (a plain
     `configured_ = true;`), so double-calling is harmless.

5. **NEW (Revision 1 — resolves this ticket's own thrown exception):
   `Devices::Motor::reconfigure()`, the runtime config-replacement surface
   step 4's `configureMotor()` needs to actually work.** Without this,
   `configureMotor()` only ever reached `MotorArmor`'s own cached
   `motionThreshold_` — the wrapped `NezhaMotor`'s `config_` stayed at
   whatever the constructor received, which per step 4 above is now
   `MotorConfig{}`'s all-zero default forever. See sprint.md's Architecture
   Revision 1 / Decision 6 for the full rationale (including why a
   deferred-construction redesign was considered and rejected); do not
   re-litigate that alternative here.

   - **`src/firm/devices/motor.h`**: add a new pure virtual to `Motor`,
     near `applyGains()`:
     `[[nodiscard]] virtual bool reconfigure(const MotorConfig& config) = 0;`
     Doc comment must distinguish it from `applyGains()` — this is NOT the
     live wire `CFG`-patch surface (`RobotLoop::handleConfig()` keeps using
     `applyGains()` only, unchanged) — and must state the at-rest
     precondition below.

   - **`src/firm/devices/nezha_motor.h`/`.cpp`**: implement
     `bool NezhaMotor::reconfigure(const MotorConfig& config)`:
     - Guard: refuse (return `false`, leave `config_` untouched) unless
       `mode_ == Mode::None` (never yet commanded) or the motor is
       independently at rest (`fabsf(filteredVelocity_) <
       kReconfigureRestVelocity && appliedDuty() == 0.0f`). Add
       `static constexpr float kReconfigureRestVelocity = 5.0f;  // [mm/s]
       mirrors MotorArmor's own kRestVelocity at-rest threshold` as a new
       private constant — do not try to share `MotorArmor::kRestVelocity`
       across the class boundary, it belongs to a different class.
     - On success: `config_ = config;` then re-derive `slewRate`
       (`if (config_.slewRate <= 0.0f) config_.slewRate =
       kDefaultSlewRate;`) and the write-shaping cache fields exactly as
       the constructor does today (`reversalDwell_ = config.reversalDwell.has
       ? config.reversalDwell.val : kDefaultReversalDwell;` / the
       `outputDeadband_` equivalent — ticket 003 later simplifies these two
       lines to plain field reads in THIS method, once `Opt<float>`
       collapses to `float`); return `true`.
     - **Change the constructor to delegate to this method**:
       `NezhaMotor::NezhaMotor(I2CBus& bus, const MotorConfig& config) :
       bus_(bus) { reconfigure(config); }` — `mode_`'s own member
       initializer (`Mode::None`) applies before the constructor body runs,
       so this always succeeds at construction time. Delete the
       now-duplicate substitution logic from the constructor body (it moved
       into `reconfigure()`) — do not keep two copies of it.

   - **`src/firm/devices/motor_armor.h`**: rename `MotorArmor::configure()`
     to `MotorArmor::reconfigure()`, mark it `override`, and extend it to
     forward to `inner_` before deriving its own cache:
     ```cpp
     bool reconfigure(const MotorConfig& config) override {
       bool applied = inner_.reconfigure(config);
       if (applied) {
         motionThreshold_ = config.outputDeadband.has
                                 ? config.outputDeadband.val
                                 : kDefaultMotionThreshold;
       }
       return applied;
     }
     ```
     Only update `motionThreshold_` when `applied` is true — an armor whose
     inner motor refused the new config must not silently drift its own
     wedge-detection threshold away from what the motor actually uses.

   - **`src/firm/main.cpp`**: rename the two existing
     `motorL.configure(motorCfgL);` / `motorR.configure(motorCfgR);` calls
     to `reconfigure()`, discarding the now-`[[nodiscard]]` return value
     explicitly: `(void)motorL.reconfigure(motorCfgL);` /
     `(void)motorR.reconfigure(motorCfgR);`. Always succeeds here (freshly
     constructed, `mode_ == Mode::None`) — pure rename, no real-hardware
     behavior change (Decision 2's "always-immediate" precedent).

   - **`src/sim/sim_harness.h`**: in `configureMotor(uint32_t port, const
     Devices::MotorConfig& cfg)`, change `armorR_.configure(cfg);` /
     `armorL_.configure(cfg);` to `armorR_.reconfigure(cfg);` /
     `armorL_.reconfigure(cfg);`. Keep `configureMotor()`'s own signature
     `void` — do not thread the bool through the ctypes/Python boundary,
     out of scope here — but do not silently drop a `false`: assert (or
     `std::fprintf(stderr, ...)` in a host/test build) if `reconfigure()`
     returns `false`, since for this ticket's own gate scenario (a freshly
     constructed, never-yet-commanded `SimHarness`) it must always be
     `true` — a `false` here is a real bug, not the expected
     operator-driven refusal that only happens via the independent
     mid-session `sim_configure_motor()`/TestGUI robot-select path (which
     calls this same method after the sim may already be driving).

   - **`src/tests/sim/unit/devices_motor_harness.cpp`**: add a
     `reconfigure()` override to this file's `MockMotor` test double (it
     `: public Devices::Motor`, so the new pure virtual must be implemented
     or the whole file fails to compile): `bool reconfigure(const
     Devices::MotorConfig& config) override { ++reconfigureCalls; return
     true; }` plus `int reconfigureCalls = 0;` alongside the existing
     `resetPositionCalls`/`rebaselineCalls`. Also rename this file's two
     existing `armored.configure(cfg);` call sites to
     `armored.reconfigure(cfg);` — both already pass the SAME `cfg` the
     wrapped motor was constructed with, so this is a
     byte-for-byte-behavior-preserving rename, not a new assertion.

6. **New test-support bench-config header**:
   `src/tests/sim/support/bench_test_config.h` + `.cpp` (new files),
   namespace `TestSupport`:
   - `msg::PlannerConfig benchTestPlannerConfig();` — byte-for-byte the
     deleted `SimHarness::makeExecutorConfig()` body (copy every field and
     every explanatory comment verbatim; only the enclosing
     function/namespace changes).
   - `Devices::MotorConfig benchTestMotorConfig(uint32_t port);` —
     byte-for-byte the deleted `SimHarness::makeMotorConfig(uint32_t port)`
     body.
   - `void configureSimForBenchTest(TestSim::SimHarness& sim);` —
     convenience wrapper calling `sim.configurePlanner(benchTestPlannerConfig())`
     then `sim.configureMotor(1, benchTestMotorConfig(1))` then
     `sim.configureMotor(2, benchTestMotorConfig(2))`.

7. **Batch-migrate every existing caller — 9 files, 26 construction sites**
   (exhaustively grepped; not the "~40" this ticket and sprint.md originally
   estimated — see sprint.md's Architecture Revision 1). The 9 files:
   `src/tests/sim/system/behavior_lock_harness.cpp`,
   `src/tests/sim/system/faults/fault_knobs_harness.cpp`,
   `src/tests/sim/system/heading_source_harness.cpp`,
   `src/tests/sim/system/move_queue_harness.cpp`,
   `src/tests/sim/system/pilot_distance_trim_harness.cpp`,
   `src/tests/sim/system/profiled_motion_harness.cpp`,
   `src/tests/sim/system/scripted_twist_demo_harness.cpp`,
   `src/tests/sim/system/sim_api_harness.cpp`, and
   `src/tests/sim/system/straight_twist_harness.cpp` (already migrated as
   this ticket's own pilot file — verify it still has
   `TestSupport::configureSimForBenchTest(sim);` right after construction;
   do not redo it). For each of the remaining 8, add
   `TestSupport::configureSimForBenchTest(sim);` (plus the `#include
   "bench_test_config.h"`) immediately after construction and before the
   first `injectTwist()`/`injectMove()`/`step()`/`boot()` call. Do **not**
   touch `src/tests/sim/unit/sim_harness_configure_harness.cpp` (113-002's
   own test) or any harness that already calls `configurePlanner()`/
   `configureMotor()` explicitly with its own values — those intentionally
   test the unconfigured-then-configured transition or a specific override.
   This convention (call `configureSimForBenchTest()` right after
   constructing a bare `SimHarness`) is now the required pattern for any
   *new* sim harness written by tickets 002-006 too.

   **Plus one Python file**: `src/tests/sim/test_motor_primitive.py`
   reaches an unconfigured harness via the ctypes `SimLoop` path — its
   `ideal_loop()` helper calls `loop.connect(start_tick_thread=False)` and
   then drives `loop.twist()` directly, with no config push at all. Fix: in
   `ideal_loop()`, immediately after `loop.connect(...)` and before the
   fault-knob-zeroing calls, add a `configure_from_robot()` push:
   ```python
   from robot_radio.config.robot_config import load_robot_config
   config = load_robot_config(_ROBOTS_DIR / "tovez_nocal.json")
   loop.configure_from_robot(config)
   ```
   mirroring `test_sim_configure_from_robot.py`'s own established pattern
   (add matching `_REPO_ROOT`/`_ROBOTS_DIR` module-scope path constants if
   this file doesn't already have them). Use the REAL `tovez_nocal.json`
   config here, not the C++ bench-config's own stand-in values — deliberate:
   this file's own `TRACK_WIDTH`/`TICKS_PER_MM` module constants already
   assume real `tovez_nocal.json` geometry, and the test is a from-scratch,
   zero-simulated-error accuracy check with generous tolerances (2mm / 1deg
   over a 2s run), not a shape/oscillation check — the `vel_kp` difference
   from the old bench stand-in (0.003) to the real configured value (0.002)
   is not expected to threaten those tolerances, but run it and confirm; if
   it needs a tolerance adjustment, that is diagnostic signal consistent
   with this sprint's own re-validation theme (SUC-006), not a regression
   to chase down elsewhere.

## Files to Touch

- `src/protos/envelope.proto` (new `ErrCode` value)
- `src/firm/messages/envelope.h` (regenerated, not hand-edited)
- `src/firm/app/robot_loop.h`, `src/firm/app/robot_loop.cpp` (gate)
- `src/firm/main.cpp` (`markConfigured()` call, plus Revision 1's
  `configure()` → `reconfigure()` rename on the two existing motor calls)
- `src/firm/devices/motor.h` (Revision 1: new `reconfigure()` pure virtual)
- `src/firm/devices/nezha_motor.h`, `.cpp` (Revision 1: implement
  `reconfigure()`, guard, constructor delegates to it)
- `src/firm/devices/motor_armor.h` (Revision 1: `configure()` →
  `reconfigure()`, now forwards to `inner_`)
- `src/sim/sim_harness.h` (delete two methods, change construction, add
  configured-tracking; Revision 1: `configureMotor()` calls
  `armorX_.reconfigure()`)
- `src/tests/sim/support/bench_test_config.h`, `.cpp` (new)
- `src/tests/sim/unit/devices_motor_harness.cpp` (Revision 1: `MockMotor`
  gains a `reconfigure()` override; rename its two `armored.configure(cfg)`
  call sites)
- The 9 `src/tests/sim/**/*.cpp` files (26 construction sites) that
  construct a bare `SimHarness` without configuring it — enumerated in
  Approach step 7 above (exhaustively grepped, corrected from an earlier
  "~40" estimate — see sprint.md's Architecture Revision 1)
- `src/tests/sim/test_motor_primitive.py` (Revision 1: `ideal_loop()` gains
  a `configure_from_robot()` push — the one Python file reached via the
  ctypes `SimLoop` path, not a C++ harness)

## Acceptance Criteria

- [ ] `ErrCode::ERR_NOT_CONFIGURED` exists in the regenerated `envelope.h`.
- [ ] A freshly-constructed `SimHarness`, before any `configurePlanner()`/
      `configureMotor()` call, has `isConfigured() == false`.
- [ ] Injecting a TWIST or MOVE against an unconfigured `SimHarness` yields
      `ACK_STATUS_ERR` / `ERR_NOT_CONFIGURED`; `drive_`/`pilot_`/`deadman_`
      state is unchanged (zero motor writes — assert via the plant's own
      write-count/duty-history hook).
- [ ] Injecting STOP or a CONFIG patch against an unconfigured `SimHarness`
      still acks `ACK_STATUS_OK`.
- [ ] After `configurePlanner()` + both `configureMotor()` calls,
      `isConfigured() == true` and a subsequent TWIST/MOVE is accepted
      normally **and produces real, nonzero measured wheel motion** — not
      merely an `ACK_STATUS_OK` (this is the acceptance criterion the
      original exception found unsatisfiable; it must now actually hold):
      `motorLeft().velocity()`/`motorRight().velocity()` become nonzero
      within a few ticks of an injected TWIST, and each port drives its own
      distinct simulated wheel (no port aliasing).
- [ ] `grep -n "makeExecutorConfig\|makeMotorConfig" src/sim/sim_harness.h`
      finds nothing.
- [ ] Every one of the 9 pre-existing `src/tests/sim/**` harnesses (26
      construction sites) compiles and passes unchanged (behaviorally)
      after adding its one `configureSimForBenchTest()` line, and
      `test_motor_primitive.py` passes after its `configure_from_robot()`
      addition — verified by a full targeted re-run, not a sample.
- [ ] `main.cpp`'s new `markConfigured()` call does not change any observable
      real-firmware behavior (it was always immediately true in practice).
- [ ] `NezhaMotor::reconfigure()` returns `true` and applies the new config
      when called on a never-yet-commanded (`mode_ == Mode::None`) or
      genuinely at-rest motor; returns `false` and leaves `config_`
      unchanged when called on a motor that is actively commanded and not
      at rest (unit-testable directly: `setDuty()`/`setVelocity()` then
      `tick()` to leave `Mode::None`, then attempt a `reconfigure()` with a
      differing `fwdSign` and assert it did NOT take effect).
- [ ] `MotorArmor::reconfigure()` only updates its own `motionThreshold_`
      when the inner motor's `reconfigure()` returned `true`.
- [ ] `devices_motor_harness.cpp` and every other file constructing a
      `Devices::Motor`-implementing type still compiles (the new pure
      virtual is implemented everywhere `: public Devices::Motor` appears —
      `NezhaMotor`, `MotorArmor`, `MockMotor`; confirmed exhaustively via
      `grep -rn "public Devices::Motor\|public Motor\b" src`).

## Testing

- **Existing tests to run**: the full `src/tests/sim` suite (unit + system) —
  this ticket's own migration touches most of it.
  `uv run python -m pytest src/tests/sim -v` as a first pass, then the full
  suite.
- **New tests to write**: a targeted `App::RobotLoop`/`SimHarness` gate test
  (new file, e.g. `src/tests/sim/unit/config_gate_harness.cpp` + its
  `test_config_gate.py` pytest wrapper, mirroring
  `sim_harness_configure_harness.cpp`'s existing shape) covering:
  unconfigured refusal (TWIST/MOVE → `ERR_NOT_CONFIGURED`, zero writes),
  unconfigured pass-through (STOP/CONFIG → OK), and the
  configured-then-accepted transition — **including that the accepted
  transition produces real measured wheel motion**, not just an OK ack (the
  gap the original exception found). Also add targeted `NezhaMotor::
  reconfigure()` unit cases to `devices_motor_harness.cpp`: succeeds and
  fully replaces `config_` (including `fwdSign`/`port`/`wheelTravelCalib`)
  when the motor has never been commanded; fails and leaves `config_`
  unchanged when the motor is actively driving and not at rest; succeeds
  again once the motor returns to rest.
- **Verification command**: `uv run python -m pytest src/tests/sim -v`
  (targeted), then full suite `uv run python -m pytest` before marking done.
  Also run `uv run python -m pytest src/tests/sim/test_motor_primitive.py -v`
  specifically, since Revision 1 changes what config it configures against.
