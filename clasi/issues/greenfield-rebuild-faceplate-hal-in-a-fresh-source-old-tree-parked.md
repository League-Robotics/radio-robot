---
status: pending
---

# Greenfield Rebuild: Faceplate HAL in a Fresh source/, Old Tree Parked

## Context

The 056–061 refactor landed the message model only at the subsystem tier; the
hardware layer never got it (`messages/motor.h`/`gripper.h`/`ports.h` have zero
includers, `IVelocityMotor` exposes ~10 encoder-plumbing virtuals, `Motor` exposes
six raw Nezha register verbs). Rather than incrementally refactoring the live tree
(slow, endless shimming), the stakeholder chose a **greenfield rebuild**: rename
`source/` → `source_old/` (parked, still buildable by flipping codal.json), create a
new `source/`, copy in only the infrastructure with **no dependency on
state/subsystems/robot/hal**, and build the new world from the protos up. Old code
is reference material to port guts from, not a patient to operate on.

**This ticket's goal:** a working debug system — DEV commands over the standard
protocol that drive motors individually and together through a Drivetrain, on a
minimal dev loop. Nothing else.

## Terminology & style (stakeholder-set)

- **Faceplate** — the four-interface contract every hardware device and subsystem
  presents (not "blocks"). Each interface is a **channel**: command-in, command-out,
  **observation** (the component's State, read from above), config.
- Verbs: `configure(Config)` · `apply(Command)` · `tick(now)` executes & emits
  (command-out is **returned**, never pushed) · `state()` observation ·
  `capabilities()`.
- **Setters/getters are the primitive operations; the message verbs are built on
  them, not the other way around.** `apply(Command)` validates against
  `capabilities()`, unpacks the oneof, and calls the matching setter;
  `state()` assembles the observation message from the getters. Messages are the
  wire/log form of the same operations.
- **Edge (command-out) types are named by their endpoints**:
  `<Producer>To<Consumer>Command`, e.g. `DrivetrainToMotorCommand`. Never by
  mechanism or moment (`…Tick`, `…Output`, `…Batch`) — long is fine, ambiguous
  is not.
- **Google C++ Style** for all new code — vendored (with inline PROJECT OVERRIDE
  banners) at `docs/reference/google-cppguide.html`; deviations recorded in
  `.claude/rules/naming-and-style.md` and `.claude/rules/coding-standards.md`:
  `.cpp` extension (CODAL build requirement), `#pragma once` if already
  conventional, generated `source/messages/*` exempt, verbatim-copied infra
  keeps its old filenames/style until touched. New files: snake_case filenames; **CamelCase naming, overriding
  Google's case rules** — capitalize the first letter (including all letters of
  an acronym) in a class/struct/protocol/namespace name (`Motor`, `HTTPServer`,
  `namespace Hal`); lower-case the first letter (including all letters of a
  leading acronym) in a variable or function name (`tick()`, `setVelocity()`,
  `leftObs`) — **functions are never PascalCase**; trailing-underscore members,
  `kConstant` constants. The vendored guide at
  `docs/reference/google-cppguide.html` carries inline override banners.
  **No units in ANY identifier** — method, property, or parameter
  (`speed`, never `speed_mms`; `setVelocity(float velocity)`, never
  `(float mm_per_s)`); units go in the `// [unit]` leading-tag trailing comment
  per `.claude/rules/coding-standards.md`. Name the quantity: `speed` for a directionless
  scalar, `velocity` for directed; positions are `x`/`y`; frame/axis subscripts
  are semantic and fine (`x_b` = x in body frame).

## Control-architecture decisions (locked)

1. **Velocity PID lives inside the Motor class** — embedded directly in the
   implementation (no separate controller object). Drive tier does **no PID**; it
   sends velocity/position commands and the device respects them.
2. **Ratio-keeping (sync) governor lives in Drivetrain**: if one motor bogs down,
   reduce the shared speed ceiling so the wheels hold their commanded ratio. This
   concept exists today as the sync coupling in
   `source_old/control/VelocityController.*` / `MotorController` (`syncGain`) —
   port the idea, operating on velocity **targets**, not duties.
3. Voltage mode stays in motor.proto, capability-gated; Nezha rejects it.
4. DEV commands are the only command family beyond bare liveness (PING/VER/HELP,
   which come free with the copied command infrastructure and which host tooling
   needs for boot detection).
5. Differential (Tovez) first. Mecanum, sim leaves, planner, telemetry, production
   motion commands: later tickets.

---

## Step 0 — Style guide + scaffold

1. DONE 2026-07-04: Google C++ Style Guide vendored at
   `docs/reference/google-cppguide.html` with inline PROJECT OVERRIDE banners
   (Naming, Function Names, Variable Names, Namespace Names); deviations
   recorded in `.claude/rules/naming-and-style.md` and
   `.claude/rules/coding-standards.md`.
2. `git mv source source_old` (one commit, pure rename — history follows).
   Rollback at any time = set codal.json `"application": "source_old"`.
3. Create new `source/` with `main.cpp` stub; codal.json keeps
   `"application": "source"`.
4. Build tooling: `build.py` — `gen_messages.py` already outputs to
   `source/messages/` (correct for the new tree); **skip/condition
   `gen_default_config.py` and `check_config_sync.py`** (no `source/robot/` yet).
   Host-side sim/test builds reference old paths — expected broken; do not chase
   them this ticket. Regenerate compile_commands + clangd restart after the move
   (known squiggles gotcha).

## Step 1 — Copy dependency-clean infrastructure into new source/

Copy verbatim (then verify each compiles with no includes reaching
state/subsystems/robot/hal; trim anything that does):

- `com/` — SerialPort, Radio, RadioChannel, I2CBus (Communicator only if
  dep-clean; else write a ~50-line serial+radio poll loop in main).
- `commands/` — CommandProcessor.{h,cpp}, ArgParse.{h,cpp} (the table-driven
  longest-prefix dispatcher + OK/ERR reply builders). NOT SystemCommands /
  ConfigCommands / DebugCommands / MotionCommand(s) / SimCommands (robot-coupled).
  CommandQueue only if dep-clean and needed; DEV dispatches immediately.
- `types/` — CommandTypes.h, ArgSchema.h, Protocol.h, ValueSet.h. NOT Config.h
  (RobotConfig — legacy blob; new world configures via msg:: types only).
- `kinematics/` — IKinematics/BodyKinematics (twist ↔ wheel speeds) if dep-clean.
- `messages/` — regenerated from protos (Step 2), not copied.

Liveness commands (PING/VER/HELP/ECHO/ID) re-registered in a small
`system_commands.cpp` (new file, Google style — handlers are a few lines each;
port bodies from `source_old/commands/SystemCommands.cpp`).

## Step 2 — Protos first (accuracy pass + regen)

`protos/motor.proto` gets the real fixes before anything is generated from it:

```proto
message MotorCommand {
  oneof control {
    float   duty_cycle = 1;   // [-1, 1]
    float   voltage    = 2;   // V (capability-gated; Nezha rejects)
    float   velocity   = 3;   // mm/s — closed by the motor's internal PID
    float   position   = 4;   // deg via onboard 0x5D (capability-gated)
    Neutral neutral    = 5;
  }
  optional float feedforward    = 6;  // added to PID output (duty)
  optional bool  reset_position = 7;  // zero encoder this tick; rides beside any arm
}

message MotorConfig {
  float travel_calib   = 1;  // mm/deg
  int32 fwd_sign       = 2;
  Gains vel_gains      = 3;  // per-motor loop (kp, ki, kff, i_max, kaw)
  float vel_filt_alpha = 4;
  float min_duty       = 5;  // stiction floor / integrator-freeze threshold
  float slew_rate      = 6;  // duty slew limit (MotorSlew semantics)
}

message MotorCapabilities {
  bool duty_cycle  = 1;
  bool voltage     = 2;  // false on Nezha
  bool velocity    = 3;  // true on Nezha (software PID in the leaf)
  bool position    = 4;  // true on Nezha (onboard 0x5D)
  bool has_encoder = 5;
}
```

- `drivetrain.proto`: verify `DrivetrainCommand/State` fit the minimal Drivetrain
  (twist + wheels + neutral arms are right); add a comment deprecating
  `vel_gains`/`min_wheel` in `DrivetrainConfig` (loop moved into MotorConfig); keep
  `sync_gain` (now the ratio governor's knob). Field-check `gripper.proto`/
  `ports.proto`/`sensors.proto` against source_old reality while we're in there —
  cheap now, they generate the future capability headers.
- Regenerate `source/messages/*.h`; `--emit-inventory` refresh.

## Step 3 — Capability faceplates (all protos), Motor implemented

Write `source/hal/capability/*.h` — one faceplate interface per proto component
(motor, gripper, line_sensor, color_sensor, ports, odometer). **Only Motor gets an
implementation this ticket**; the rest are headers that later tickets implement.

```cpp
// source/hal/capability/motor.h
namespace Hal {

class Motor {
 public:
  virtual ~Motor() = default;
  virtual void begin() {}

  // Primitive setters — the real implementations, one per command mode.
  // setVelocity() sets the target the embedded PID chases in tick();
  // setDutyCycle() stages the duty the slew limiter walks toward; etc.
  // Wheel motors have one degree of freedom, so a directionless magnitude is a
  // speed; velocity here is the signed scalar along that axis.
  virtual void setDutyCycle(float dutyCycle) = 0;         // [-1, 1]
  virtual void setVoltage(float voltage) = 0;             // [V] Nezha: unsupported (capability)
  virtual void setVelocity(float velocity) = 0;           // [mm/s] signed
  virtual void setPosition(float position) = 0;           // [deg]
  virtual void setNeutral(msg::Neutral mode) = 0;
  virtual void setFeedforward(float feedforward) = 0;     // [V]
  virtual void resetPosition() = 0;                       // zero encoder

  // Primitive getters — the real reads, served from what tick() last sampled.
  virtual float position() const = 0;                     // [mm]
  virtual float velocity() const = 0;                     // [mm/s] signed
  virtual float appliedDuty() const = 0;                  // [-1, 1]
  virtual bool connected() const = 0;
  virtual bool wedged() const = 0;

  // Faceplate verbs.
  virtual void configure(const msg::MotorConfig& config) = 0;
  virtual void tick(uint32_t now) = 0;                    // [ms] sample encoder; run the active mode
  virtual msg::MotorCapabilities capabilities() const = 0;

  // Message plane — implemented ONCE in this base class on top of the primitives:
  // apply() validates vs capabilities(), unpacks the oneof, calls the setter;
  // state() assembles a MotorState from the getters.
  void apply(const msg::MotorCommand& command);
  msg::MotorState state() const;
};

}  // namespace Hal
```

`source/hal/nezha/nezha_motor.{h,cpp}` — the concrete leaf (vendor named at the
seam). Port the guts from `source_old/hal/real/Motor.cpp`: register map, split-phase
0x46 encoder request/collect (**sequencing preserved exactly — wedge-latch
history**), slew limiting, wedge detection (→ `MotorState.wedged`). New here:

- `tick()` executes the staged command: DUTY → slew → registers; VELOCITY → embedded
  PID (encoder-derived filtered velocity vs target; gains/anti-windup/min-duty from
  `MotorConfig`) → duty; POSITION → onboard 0x5D; NEUTRAL; `reset_position` → zero.
- `apply()` rejects modes not in `capabilities()` (voltage on Nezha).
- Encoder plumbing and raw register verbs are **private**. The public surface is
  the faceplate above — nothing else.

`source/hal/nezha/nezha_hal.{h,cpp}` — minimal factory/owner: I2CBus + left/right
NezhaMotor + `tick(now)` orchestrating the split-phase bus schedule.

## Step 4 — Drivetrain subsystem (drive motors together)

`source/subsystems/drivetrain.{h,cpp}` — same faceplate one level up; command-out
returned, routed by the loop:

```cpp
namespace Subsystems {

// Command-out edge type, named by its endpoints: what the Drivetrain sends to
// its two wheel Motors. Edge types are always <Producer>To<Consumer>Command —
// long, but never confused with anything.
struct DrivetrainToMotorCommand {
  msg::MotorCommand left;
  msg::MotorCommand right;
};

class Drivetrain {
 public:
  // Primitive setters — the real implementations of each command arm.
  // A twist is a directed body-frame velocity: v_x, v_y, omega (matches
  // msg::BodyTwist3; math subscripts keep their underscore). v_y is honored
  // only on holonomic drivetrains; a directionless magnitude would be a
  // speed, and twist is never that.
  void setTwist(float v_x, float v_y, float omega);    // [mm/s] [mm/s] [rad/s]
  void setWheelTargets(float left, float right);       // [mm/s] signed wheel velocities
  void setNeutral(msg::Neutral mode);

  // Faceplate verbs.
  void configure(const msg::DrivetrainConfig& config);
  void apply(const msg::DrivetrainCommand& command);   // unpacks oneof → setters above
  DrivetrainToMotorCommand tick(uint32_t now,          // [ms]
                                const msg::MotorState& leftObs,
                                const msg::MotorState& rightObs);
  msg::DrivetrainState state() const;                  // assembled from getters
  msg::DrivetrainCapabilities capabilities() const;

 private:
  // Ratio governor: if a wheel underachieves its target (bogged down), lower the
  // shared speed ceiling so left/right hold their commanded ratio (curvature),
  // instead of letting the healthy wheel run away. Port of the syncGain concept
  // from source_old/control/VelocityController.* — applied to velocity targets.
  void governRatio(float* targetLeft, float* targetRight,   // [mm/s]
                   const msg::MotorState& leftObs,
                   const msg::MotorState& rightObs);
  ...
};

}  // namespace Subsystems
```

`tick()` = kinematics (twist → wheel targets, from copied BodyKinematics) → ratio
governor → return two `MotorCommand{velocity}` (or duty/neutral pass-throughs for
WHEELS/NEUTRAL arms). No PID here, ever. Observations arrive as arguments
(children's state + tick time) — no clock reads, no motor handles inside.

## Step 5 — DEV commands + dev loop

`source/commands/dev_commands.{h,cpp}` — a `Commandable` registered in the (new,
small) command table; standard OK/ERR taxonomy so `NezhaProtocol.send()` and the
relay path work unchanged:

```text
DEV M <n> DUTY <duty>          → motor.apply(duty_cycle)  OK DEV M 1 applied=0.30
DEV M <n> VEL <velocity>       → [mm/s] leaf PID closes the loop
DEV M <n> POS <position>       → [deg] onboard position move
DEV M <n> VOLT <voltage>       → [V] ERR unsupported (proves capability gating)
DEV M <n> NEUTRAL <B|C>
DEV M <n> RESET                → encoder zero via reset_position
DEV M <n> STATE                → OK DEV M 1 pos=123.4 vel=80.1 applied=0.28 wedged=0 conn=1
DEV M <n> CAPS                 → OK DEV M 1 duty=1 volt=0 vel=1 pos=1 enc=1
DEV M <n> CFG k=v ...          → motor.configure delta (kp=0.8 slew=400 ...)
DEV DT VW <vx> <vy> <omega>    → [mm/s mm/s rad/s] body twist (ratio-governed;
                                 vy honored only when holonomic — 0 on Tovez)
DEV DT WHEELS <left> <right>   → [mm/s] per-wheel velocity targets
DEV DT NEUTRAL <B|C> | DEV DT STATE | DEV DT STOP
DEV STATE                 → everything, one line per component
DEV STOP                  → all motors neutral, drivetrain idle
```

DEV handlers build a `msg::MotorCommand`/`msg::DrivetrainCommand` and go through
`apply()` — exercising the full message plane (capability validation included; the
`VOLT` rejection above comes from `apply`), which in turn exercises the setters.

Authority is trivial by construction: this firmware runs **only** the dev loop —
there is no planner/drive to fight. `DEV M …` motion deactivates drivetrain mode;
`DEV DT …` reactivates it. Serial-silence watchdog (default ~1 s, settable)
auto-neutrals everything — the runaway history keeps this in even on a bench build.

`source/main.cpp` — the whole loop (the wiring layer):

```cpp
while (true) {
  pollComms();                                   // dispatch DEV/PING via CommandProcessor
  hal.tick(now);                                 // split-phase encoder schedule
  if (drivetrainActive) {
    auto out = drivetrain.tick(now, left.state(), right.state());
    left.apply(out.left);
    right.apply(out.right);
  }
  left.tick(now);                                // staged commands execute (PID runs here)
  right.tick(now);
  watchdog.check(now);                           // silence → all neutral
}
```

Document the family in `docs/protocol-v2.md` §"Development commands".

## Verification

- Build: `python build.py --clean` produces the new hex; `source_old` untouched
  (rollback = codal.json application flip).
- Bench (Tovez on the stand, `mbdeploy deploy robot --hex …` with ROLE check):
  - `DEV M 1 DUTY 30` → wheel spins, `applied=0.30`, `DEV M 1 STATE` position climbs.
  - `DEV M 1 VEL 120` → converges; log applied duty vs measured velocity (step
    response sanity for the embedded PID).
  - `DEV M 1 VOLT 3` → `ERR unsupported`; `DEV M 1 RESET` → position rezeroes.
  - `DEV DT VW 150 0 0` → both wheels ~equal; hand-drag one wheel → **both** slow,
    ratio held (governor observable in `DEV DT STATE`).
  - Watchdog: stop sending → motors neutral within the window.
- Host: `tests/bench/dev_exercise.py` (new) scripting the above over
  `NezhaProtocol.send()` through serial and relay (`!GO` data plane).

## Later tickets (not this one)

Sensors/gripper/ports leaf implementations, sim leaves + host test harness for the
new tree, subsystem/planner tiers, production motion commands, telemetry, mecanum,
retiring `source_old/`.

## Process

One CLASI sprint; tickets ≈ (1) style guide + scaffold + infra move, (2) protos +
regen, (3) capability headers + NezhaMotor + HAL, (4) Drivetrain + governor,
(5) DEV commands + loop + protocol doc, (6) bench validation script + HITL run.

## Session notes (for memory after approval)

Faceplate/channel terminology; Google C++ style adopted with CamelCase override
(docs/reference/google-cppguide.html + .claude/rules/coding-standards.md);
Eric prefers greenfield parallel rebuild over incremental refactor — parks old tree,
builds fresh; motor PID in leaf, ratio governor in drivetrain.
