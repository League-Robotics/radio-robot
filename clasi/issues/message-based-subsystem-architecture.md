---
status: pending
---

# Message-Based Subsystem Architecture (Drive + Sensors + Planner)

## Summary

Restructure the firmware so the robot is a small network of **subsystems driven
exclusively through typed messages**. Each subsystem is the single authority over
a coherent slice of state (per
[docs/design/subsystem-and-drivetrain-modeling.md](docs/design/subsystem-and-drivetrain-modeling.md)),
and is driven the same way a motor is in
`radio-robot-c/frc-code-scout/knowledge/specs/portable-motor-interface.md`: you
**apply** a command, you **read** a state, and you **configure** it.

Two real subsystems: **Drive** (motors + encoders + OTOS + fusion) and **Sensors**
(line + color). Above them, a **Planner** (`MotionController`) that closes
position/heading goals by emitting twist commands down to Drive. Radio/serial are
**auxiliary transport**, not subsystems.

This issue replaces three earlier "design-only" plan issues (subsystem interface,
protobuf codegen, system integration) with **one concrete architecture plus three
sprintable implementation phases**. The architecture below is the spec — the
phases build it. Where the old issues left forks open, this issue resolves them
(see [Resolved decisions](#resolved-decisions)).

**Prerequisite:** the module reorganization in
[reorganize-source-io-into-hal-comms-robot-configs.md](reorganize-source-io-into-hal-comms-robot-configs.md)
(`source/io/` → `source/hal/` + `source/com/` + `source/robot/`) is **Phase 0** —
it must land first, because the target layout below assumes it.

---

## Target architecture (the concrete picture)

### Module layout (post-reorg)

```
source/com/          transports: Radio, SerialPort, I2CBus, Communicator   — AUXILIARY
source/hal/          Hardware base, capability/ ifaces, real/ + sim/ drivers
source/robot/        NezhaHAL / MecanumHAL (code-level robot assembly) + Robot + loop
source/messages/     GENERATED POD messages (Phase 1)
source/subsystems/   Drive, Sensors
source/superstructure/  MotionController (the Planner)
```

### The subsystem contract — 3 messages + 4 verbs

Every subsystem `S` defines **three POD message types** and exposes a **port of
four verbs**. The contract is a **documented structural convention** (optionally a
C++20 `concept`) — **NOT** a virtual base class, honoring `Drive`'s existing "no
virtual dispatch, no SubsystemBase" constraint and keeping the hot loop
devirtualized.

```
S::Command   // intent   (u) — what to do
S::State     // estimate (x) — what is happening (read-only snapshot)
S::Config    // params   (θ) — how to behave

CommandBatch periodic(uint32_t now);   // LIFECYCLE ENGINE — does the per-tick work; returns outbound commands
CommandBatch apply(const Command&);    // stage intent; returns outbound ROUTABLE commands (not S::Command)
const State& state() const;            // const-ref to the subsystem's internal State; getters on it
void         configure(const Config&); // delta-apply params; live-safe; read by the next periodic()
Capabilities capabilities() const;     // declared truth: which Command modes accepted, which State fields populated
```

- **`periodic(now)` is the engine.** It does the subsystem's per-tick work: drives
  hardware I/O, runs the internal control step, updates the owned `State` slice and
  actuator outputs. The other verbs are deliberately cheap around it.
- **`apply()` stages intent** (writes the desired/output slice). It does **not**
  touch hardware; the next `periodic()` acts on it.
- **`state()` returns a const reference** to the subsystem's internal `State`,
  which `periodic()` keeps fresh. No device I/O, no copy — callers hold the ref and
  call getters (the existing cached-accessor idiom, e.g. `IVelocityMotor::positionMm()`).
- **`configure()` delta-applies params**, read live by the next `periodic()`.

**Two-phase Drive, single-phase Sensors (resolved — not "optional").** Because I2C
is split-phase and the loop must order *all sensing before all actuation*, **Drive**
splits `periodic()` into `updateInputs(now)` (sense: request/collect encoders + OTOS,
run fusion, refresh State) and `update(now)` (act: kinematics → wheel PID → motor
outputs) — the AdvantageKit `updateInputs`/`periodic` seam. **Sensors** stays
single-phase `periodic(now)`. The scheduler calls Drive's `updateInputs()` early and
its `update()` late (see [run loop](#the-run-loop-explicit-ordered-tick)).

**Naming.** Devices use `tick(now_ms)` (`IVelocityMotor::tick`, `hal.tick`);
subsystems use `periodic(now)`. `subsystem.periodic()` drives the nested
`device.tick()` calls — same idea at two scales, not a conflict.

**Fluent ergonomics (the call-site form).** Message types are plain objects with
getters and (for `Command`/`Config`) chainable setters; the subsystem owns one
reused internal instance of each. The primary call-site form is a fluent builder
terminated by the verb, over the data-plane `apply(const Command&)` beneath it:

```cpp
drive.newCommand().setTwist(vx, vy, omega).apply();   // vy rejected unless capabilities().holonomic
sensors.line().newConfig().setLagMs(20).configure();

drive.updateInputs(now); drive.update(now);
float vx = drive.state().fused().twist().vx();        // getters on the const State ref, no copy
```

`newCommand()` returns a reference to the subsystem's reset internal `Command`;
each `setX()` mutates and returns `*this`; the terminal `apply()` commits it. One
reused instance, no allocation — embedded-friendly. The underlying
`apply(const Command&)` data-plane verb still exists for replayed/logged/wire-sourced
commands.

**Capability axes:** *command* (which `Command` modes are accepted) and
*observation* (which `State` fields are populated). A pure-sensor subsystem has an
**empty command axis** — `State` + `Config`, no meaningful `Command` — showing the
contract degrade gracefully.

**Conventions (adopted from the motor spec):** optional payloads, non-null
discriminants; null = "unknown" (State) / "don't override" (Config); name the
measurand not the unit; `max_`/`min_` bound prefixes. **Units deviation noted:**
this repo uses mm / mm·s⁻¹ / rad, not REP-103 SI metres — kept for embedded
continuity, documented as a deviation.

### The command-queue bus — inter-subsystem emission (RETURN model)

Both `apply()` and `periodic()` may produce **outbound commands addressed to other
subsystems**, not just mutate their own slice. This makes the robot a
message-passing network over the shared `CommandQueue`.

- **Consume typed, produce routable.** A verb takes its own typed `S::Command`, but
  emits commands addressed to *other* subsystems — heterogeneous — so the outbound
  type is a **compact routable `OutCommand`** (verb-id + args), batched as
  `CommandBatch { OutCommand cmds[K]; uint8 count; }`. The framework drains the
  returned batch onto the queue and routes it through the **existing**
  `CommandProcessor` verb router. No new addressing layer.
- **Return model (RESOLVED — see [Resolved decisions](#resolved-decisions)).** Verbs
  **return** a fixed-capacity `CommandBatch`; the scheduler drains it. Chosen over
  the sink model because it is testable by inspecting the return (no mock sink),
  self-documenting, and the **compact `OutCommand`** (Phase 1) keeps the by-value
  copy cheap — sidestepping the 424 B `ParsedCommand` cost on the depth-4 queue.
- **`periodic()` NEVER returns state.** `state()` is the only way to read state
  (const-ref to the single source of truth). What `periodic()` *produces* is
  outbound commands — same as `apply()`. Both verbs are command-producers; state is
  always a getter.
- **The bus dissolves the planner/drive layering.** The Planner is a unit whose
  `periodic()` reads `drive.state()` (pose) and **emits a `DrivetrainCommand{twist}`**
  into its returned batch → routed to `drive.apply()`. "Above/below" becomes message
  flow, not a call hierarchy.
- **Cascade policy (mandatory, actor-model tax).** Depth-4 queue ⇒ run-to-completion
  drain with a **max-iterations-per-tick guard** and a **defined drain order**.
  Safety (STOP/ESTOP) `OutCommand`s carry `priority=true` → routed to the **front**
  of the queue, never the back. All inter-subsystem traffic is on the bus ⇒
  uniformly loggable / replayable.

### Drive subsystem

**Membership:** `motorL/R` (+ encoders), OTOS (`IOdometer`), `MotorController`,
`BodyVelocityController`, `Odometry`/`PhysicalStateEstimate`, `Kinematics`. Today's
scattered control members **fold into Drive**. Pure velocity-in / pose-out; closed-loop
goals live above in the Planner. Only `BodyTwist3` and `Pose2D` cross the boundary;
`wheels[]` stays internal.

**`DrivetrainCommand`** — twist + mode only (+ a SetPose re-anchor, resolved below):
```
oneof control {
  BodyTwist3   twist;     // {vx,vy,omega} mm/s,mm/s,rad/s — body velocity (primary)
  WheelTargets wheels;    // per-wheel speed AND/OR position — direct wheel-level command
  Neutral      neutral;   // BRAKE | COAST
  SetPose      pose;      // {x,y,h} imperative re-anchor of the fused estimate (the old SI verb)
}
optional bool seed;       // immediate-seed (S-command semantics)

struct WheelTarget  { float? speed_mmps; float? position_mm; };  // null = "don't command this axis"
struct WheelTargets { WheelTarget w[kWheelCount]; };
//   speed-only    → velocity drive
//   position-only → distance/position drive — needs onboard-position capability, else rejected
//   both          → swerve-style (drive speed + steer angle) — future swerve extension
```
- **Boundary rule:** body-level command = velocity (twist); wheel-level = speed or
  position. Body POSE *goals* (GOTO/TURN) are closed-loop and live in the Planner.
- **`vy`** is honored only when `capabilities().holonomic`. On the compile-time
  differential build it is **unrepresentable / rejected**, not silently dropped —
  the worked example of "make the impossible command un-expressible."
- **SetPose** is a Drive *command* (not config): Drive owns the fused estimator, and
  this is a per-event imperative re-anchor of live state (the old `SI` verb / `handleSI`
  → `estimate.resetPose`).
- Stop conditions and goals are explicitly OUT (Planner owns them).

**`DrivetrainState`** — the belief + diagnostics:
```
PoseEstimate fused;          // authoritative pose+twist+stamp — THE boundary-crossing belief
PoseEstimate encoder;        // diagnostic (dead-reckon)
PoseEstimate optical;        // diagnostic (raw OTOS)
float  encMm [kWheelCount];  // diagnostic
float  velMms[kWheelCount];  // diagnostic
ValueSet enc; ValueSet otos; // freshness
bool   wheelWedged[kWheelCount];
bool   connected;            // discriminant
```
**Rule:** consumers above the boundary depend only on `fused`; per-wheel fields are
read-only diagnostics (telemetry/replay) and must never become an upward command
currency.

**`DrivetrainConfig`** — the `RobotConfig` slice to turn twist→wheels and integrate
odometry: geometry (`trackwidthMm`, mecanum half-track/wheelbase, `fwdSign*`), wheel
calibration (`mmPerDegL/R`), velocity PID (`velKp/Ki/Kff/IMax/Kaw`, `velFiltAlpha`,
`syncGain`, `minWheelMms`), saturation (`vWheelMax`, `steerHeadroom`), kinematics
selector, OTOS fusion (`alphaPos/Yaw`, `otosGate`, scales, rotation gains/offsets,
`odomOff*`, `rotationalSlip`), EKF noise (`ekfQ*`, `ekfR*`), `lagOtosMs`. **Motion
limits (`aMax`, `vBodyMax`, `yawRateMax`) are NOT here — they belong to PlannerConfig.**

### Sensors subsystem — line + color

Pure-observation: **`State` + `Config`, no `Command`** (empty command axis). Line and
color don't coordinate, so they are two independent read-only units behind a common
`ISensor`-shaped contract, aggregated under one `Sensors` facade for scheduling.

```
LineSensorState  { uint16 raw[4]; uint16 normalized[4]; ValueSet stamp; bool connected; }
LineSensorConfig { uint32 lagLineMs; threshold; normalization; channel map; }
ColorSensorState { uint16 r,g,b,c; ValueSet stamp; bool connected; }
ColorSensorConfig{ uint32 lagColorMs; integration; gain; calibration; }
```

### Planner (MotionController)

Not a hardware subsystem, but uses the same message shape. Receives user goals from
comms (`PlannerCommand`), owns the closed-loop logic (trapezoid accel/decel, heading,
stop conditions), reads `drive.state()` for pose/twist, and **emits**
`DrivetrainCommand{twist}` each tick onto the bus → `drive.apply()`.

- `PlannerCommand` ≡ today's `GoalRequest` union (velocity / goto / turn / rotation /
  distance / timed / stream / stop) + `StopCondition stops[4]` + style/origin.
- `PlannerState` ≡ today's `DesiredState` (mode, targets, body twist, deadline, active).
- `PlannerConfig` = the motion-only `RobotConfig` subset (`aMax`, `vBodyMax`,
  `yawRateMax`, jerk/decel, `arriveTol`, turn-in-place gate, etc.).

### Configuration flow (hybrid — push slices + keep live registry)

```
robot_config.schema.json  (SSOT; NEW per-field `subsystem:` annotation)
   │  gen_default_config.py
   ▼
DefaultConfig.cpp → RobotConfig (full POD, unchanged)
   │  projection fns: toDriveConfig() / toSensorsConfig() / toPlannerConfig()
   ▼
each subsystem OWNS its typed Config slice  ← configure(slice) AFTER construction
   ▲
   │  live SET: kRegistry[] kept; on atomic commit, the schema `subsystem:` annotation
   │  routes the changed field to its owner and calls owner.configure(delta).
   │  MotorController::updateVelGains generalizes to drive.configure().
```
- **Push** typed owned slices for structure; **keep** the reflection registry for
  live tuning. Both coexist (the stakeholder's "hybrid" choice). The schema gains a
  `subsystem:` key per firmware field so the projection and SET-routing table are
  **generated, not hand-maintained**.
- Assembly (how the robot is *wired*) is **code** (`NezhaHAL`/`MecanumHAL` in
  `source/robot/`), not config. Config carries tuning/positions/enables only.

### The run loop (explicit ordered tick)

Evolve `loopTickOnce` (not a generic iterator). Ordering keeps all sensing before
all actuation (split-phase I2C, M1-before-M2) and applies the bus cascade guard:

```
1. COMMS DRAIN   (source/com): serial/radio → parse → enqueue PlannerCommands. Bind reply channel.
2. DRIVE.updateInputs(now): SENSE — split-phase encoder request/collect (M1,M2), OTOS when due,
                  run fusion (predict/correct), refresh Drive State.
3. BUS DRAIN+ROUTE (bounded): user motion verbs → planner.apply(); emitted DrivetrainCommands →
                  drive.apply(). priority=true → push_front.
4. PLANNER.periodic(now): read drive.state(), advance trapezoid/heading/stop logic, RETURN a
                  CommandBatch with a DrivetrainCommand{twist}.
5. BUS DRAIN (bounded): route the planner's batch → drive.apply().
6. DRIVE.update(now): ACT — Kinematics::inverse + saturate → per-wheel PID → motor.setSpeed.
7. SENSORS.periodic(now): timed line/color reads → Sensors State.
8. TELEMETRY: emit from subsystem state() snapshots.
9. SLEEP to control deadline.
```

---

## Resolved decisions

These settle forks/conflicts left open across the three original issues:

1. **Inter-subsystem emission = RETURN model**, not sink. Verbs return a
   `CommandBatch` of compact `OutCommand`s; the scheduler drains/routes it. (Old
   issue A leaned return; old issue C assumed a `CommandSink`. Resolved to return:
   testable without mocks, and the planner-isolation test inspects the returned batch
   directly — *simpler* than a capturing sink. The sink model is rejected for hidden
   coupling + mock requirement.)
2. **Two-phase Drive / single-phase Sensors** is fixed, not "optional." Drive exposes
   `updateInputs`/`update`; Sensors exposes `periodic`. The scheduler ordering above
   depends on this.
3. **No virtual base class.** The contract is structural (documented convention,
   optional C++20 `concept`), per `Drive.h`'s constraint.
4. **SetPose is a Drive command** (the old `SI`), not config.
5. **Design docs are not the deliverable.** The architecture lives in *this issue*;
   the phases deliver **code**. The only doc artifact retained is the generated
   `docs/design/message-inventory.md` traceability table (Phase 1). The prose specs
   the old issues proposed (`subsystem-message-interface.md`,
   `system-integration-and-loop.md`) are folded into the sections above.

---

## Phase 1 — Proto message definitions + C++ codegen

**Goal:** define every subsystem message in proto3 as the SSOT, and generate C++11
POD structs the firmware uses. **Types only — no serialize/deserialize; the ASCII
wire is unchanged.**

**Why proto-as-schema, not full protobuf:** firmware is CODAL/CMake C++11,
`-fno-rtti -fno-exceptions`, no-heap/STL, 128 KB SRAM — `libprotobuf`/nanopb are
infeasible. But `scripts/gen_default_config.py` is the exact precedent (schema → C++
POD before every build). We extend that pattern.

**Deliverables:**
- `protos/{common,motor,drivetrain,sensors,gripper,ports,planner}.proto` — the
  message inventory below. Custom options `(units)` (metadata) and `(max_count)=N`
  on every `repeated`.
- `scripts/gen_messages.py` — extends the `gen_default_config.py` pattern: parse
  `.proto` on the **host** via `protoc`/`grpcio-tools` → `FileDescriptorSet` (the
  *device* never sees protobuf); emit C++11 PODs to `source/messages/*.h`:
  - fields → plain members; `oneof` → `Kind` enum tag + union-ish struct (no RTTI);
    nullable → generated `template<class T> struct Opt { bool has=false; T val{}; };`
    (**not** `std::optional` — unavailable in C++11); `repeated` → `T field[N]; uint8 count;`.
  - **getters** for every field; **chainable setters** (`setX(...) -> Msg&`) for
    `Command`/`Config` messages → enables the fluent `newCommand().setX().apply()` form.
  - **NO** serialization, heap, exceptions, or STL containers.
  - **Reuse** identical existing types (`Pose2D`, `BodyTwist3`, `RobotGeometry`) via a
    `using`/`static_assert` layout-compat bridge so generated and hand structs stay
    interchangeable during migration.
- `docs/design/message-inventory.md` — generated traceability table: every message
  field → its existing `ActualState`/`DesiredState`/`RobotConfig` member or verb arg.
- EDIT `build.py` (codegen hook beside `gen_default_config.py`) and `CMakeLists.txt`
  (add `source/messages/`).

**Message inventory** (`?`=nullable→`Opt<T>`; `[K]`=fixed-capacity repeated):

- `common.proto`: `Pose2D`, `BodyTwist`, `BodyTwist3`, `BodyAccel`, `ValueSet`,
  `PoseEstimate`, `WheelTarget{float? speed_mmps; float? position_mm}`, `Gains`,
  `enum Neutral{BRAKE;COAST}`,
  `OutCommand{uint32 verb_id; float args[4]; uint8 argc; bool priority}`,
  `CommandBatch{OutCommand cmds[K]; uint8 count}`,
  `Capabilities{...command_modes; ...state_fields; bool holonomic; bool onboard_position; uint8 wheel_count}`.
- `motor.proto`: `MotorCommand{oneof control{duty_cycle|voltage|velocity_mmps|position_mm|Neutral} float? feedforward}`,
  `MotorState{bool connected; float? position_mm; float? velocity_mmps; float? applied_pct; bool? wedged}`,
  `MotorConfig{float mm_per_deg; int8 fwd_sign}`,
  `MotorCapabilities{bool onboard_position; bool has_encoder}`.
- `drivetrain.proto`: `DrivetrainCommand{oneof control{BodyTwist3 twist|WheelTargets wheels|Neutral neutral|SetPose pose} bool? seed}`,
  `WheelTargets{WheelTarget w[kWheelCount]}`, `SetPose{float x_mm,y_mm,h_rad}`,
  `DrivetrainState{...≡ActualState slices, see above}`,
  `DrivetrainConfig{...≡RobotConfig drive slice, see above; motion limits EXCLUDED}`,
  `DrivetrainCapabilities{bool holonomic; bool onboard_position; uint8 wheel_count}`.
- `sensors.proto`: `LineSensorState`, `LineSensorConfig`, `ColorSensorState`, `ColorSensorConfig` (above).
- `gripper.proto`: `GripperCommand{float? angle_deg}`, `GripperState{float? angle_deg; bool connected}`,
  `GripperConfig{bool has_gripper; float gripper_offset_mm,min_deg,max_deg}`.
- `ports.proto`: `PortCommand{oneof{DigitalOut|AnalogOut}}`,
  `PortState{bool digital_in[4]; int16 analog_in[4]; ValueSet stamp}`,
  `PortConfig{uint32 lag_ports_ms; per-port direction}`.
- `planner.proto`: `PlannerCommand` (≡`GoalRequest` union + `StopCondition stops[4]` +
  style/origin), `StopCondition{Kind kind; float a,b,ax,ay; uint8 sensor; Cmp cmp}`
  with `enum Kind{NONE,TIME,DISTANCE,HEADING,POSITION,SENSOR,COLOR,LINE_ANY,ROTATION}`,
  `PlannerState{...≡DesiredState}`, `PlannerConfig{...motion-only RobotConfig subset}`,
  `enum DriveMode{IDLE=0;STREAMING=1;DISTANCE=3;GO_TO=4;VELOCITY=5}`, `StopStyle`, `Origin`.

**Scope:** all *physical* subsystems (Motor, Drivetrain, Sensor, Gripper, Ports) +
Planner + shared types. **Defer** System (HELLO/PING/STREAM/SAFE/ZERO/SI-as-system…),
Config-registry (SET/GET), and Debug command families. (SI re-anchor is modeled as
the Drive `SetPose` command, in scope.)

**Acceptance:**
- `protoc` parses all 7 `.proto` (CI lint).
- `gen_messages.py` runs in `build.py`; emits `source/messages/*.h`.
- Generated headers **compile under the real firmware flags** (`-std=c++11 -fno-rtti
  -fno-exceptions`) in both the host-sim and `build.py --clean` device builds.
- A host unit test instantiates representative messages and exercises fluent builders
  + getters (`DrivetrainCommand().setTwist(...).twist()` round-trips; `Opt<T>`
  present/absent); confirms no heap/RTTI.
- `static_assert` layout-compat bridges for reused types (`Pose2D`, `BodyTwist3`) compile.
- Traceability table generated; spot-check `DrivetrainState`↔`ActualState`,
  `PlannerCommand`↔`GoalRequest`, `MotorCommand`↔portable-motor-interface, `*Config`↔`RobotConfig`.

---

## Phase 2 — Subsystem contract: Drive + Sensors

**Goal:** realize the contract in code on top of the Phase 1 messages: build `Drive`
and `Sensors` as standalone message units. (Planner + full integration is Phase 3.)

**Deliverables:**
- The structural contract: documented convention + optional C++20 `concept`; the
  reused-internal-instance fluent builders (`newCommand()`/`newConfig()`).
- **Drive** — fold today's `MotorController` + `BodyVelocityController` +
  `Odometry`/`PhysicalStateEstimate` + OTOS into one subsystem with
  `updateInputs(now)` / `update(now)` / `apply(DrivetrainCommand)` /
  `state() -> const DrivetrainState&` / `configure(DrivetrainConfig)` / `capabilities()`.
  Handles `twist`/`wheels`/`neutral`/`SetPose`; rejects `vy` unless holonomic.
- **Sensors** — line + color as two read-only units behind a `Sensors` facade with
  `periodic(now)` / `state()` / `configure()`. Empty command axis.
- Projection functions `toDriveConfig()` / `toSensorsConfig()` from `RobotConfig`.
- Each subsystem keeps its `State` byte-traceable to today's `ActualState` slices so
  the golden-TLM oracle still passes.

**Acceptance:**
- **Subsystem-isolation tests** (new, on the existing `SimHardware`/`libfirmware_host`
  seam): construct ONE subsystem on sim devices; feed a list of `Command`s via
  `apply()` / fluent builder; call `periodic()` (or `updateInputs`+`update`) N times;
  read `state()`; assert. No full robot, no comms.
- Walk one concrete open-loop command (`VW`) end-to-end through Drive and confirm
  byte-plausible parity with today's control-collect + drive-advance + odometry path.
- No virtual dispatch in the control path (matches `Drive.h`).
- `uv run pytest` green; existing full-robot sim tests unaffected.

---

## Phase 3 — Integration: loop, bus, planner, config, testing

**Goal:** wire Drive + Sensors + Planner into the running robot over the
command-queue bus, with bottom-up config and the ordered tick.

**Deliverables:**
- **Boot/init sequence** (evolve `main.cpp`): load `RobotConfig` → assemble robot in
  code (`NezhaHAL`/`MecanumHAL`, `source/robot/`) → construct comms (`source/com/`) →
  construct Drive + Sensors from device refs → construct Planner (`MotionController`)
  with a ref to read `drive.state()` → **split config via projections and
  `configure()` each subsystem after construction** → construct scheduler with the
  subsystem set + comms + `CommandQueue` → enter loop.
- **Planner** (`MotionController`): `apply(PlannerCommand)` / `periodic(now)` returning
  a `CommandBatch{DrivetrainCommand{twist}}` / `state()` / `configure(PlannerConfig)`.
- **The ordered tick** (rewire `loopTickOnce` to the [run loop](#the-run-loop-explicit-ordered-tick)
  above) + the **bus drain/route** with the bounded-cascade guard and `push_front`
  safety priority.
- **Live SET routing**: schema `subsystem:` annotation → route changed field to its
  owning `subsystem.configure(delta)` (generalize `updateVelGains`).
- `Robot` struct thinned to own the subsystem set + the bus.

**Acceptance:**
- Walk `VW` and a `TURN` end-to-end through the new tick; confirm sense-before-actuate,
  split-phase encoder order, bounded per-tick work, and safety priority are preserved
  — byte-plausible parity with today's `loopTickOnce`.
- **Planner-isolation test** (new): construct `MotionController`; feed user goals
  (`timed`, `turn`, `distance`) via `apply()`; tick `periodic()`; **assert the returned
  `CommandBatch` of `DrivetrainCommand`s** (twist + yaw-rate sequence) matches the
  expected trapezoid/heading profile. (Return model ⇒ inspect the batch, no sink mock.)
- Live `SET vel.kP` routes to `drive.configure()`; init `configure()` sets OTOS offset
  / sensor mounting / enables.
- Full sim suite green; **device firmware builds clean** (`python build.py --clean`);
  bench smoke on tovez (the differential dev bot) confirms motion + telemetry parity.

---

## Critical files (reference — trace against, mostly do not modify)

`source/state/{ActualState,DesiredState,OutputState,PoseEstimate}.h`,
`source/types/Config.h`, `data/robots/robot_config.schema.json`,
`scripts/gen_default_config.py`, `source/robot/{ConfigRegistry,DefaultConfig}.cpp`,
`source/subsystems/drive/Drive.h`, `source/kinematics/IKinematics.h`,
`source/robot/LoopTickOnce.cpp`, `source/commands/CommandQueue.h`,
`source/control/{MotionEventSink,StopCondition,MotionControllerBegin}.*`,
`source/superstructure/Superstructure.h`, `source/io/capability/*` (→ `hal/capability/*`
after Phase 0), `radio-robot-c/frc-code-scout/knowledge/specs/portable-motor-interface.md`.

## Cross-references

- [docs/design/subsystem-and-drivetrain-modeling.md](docs/design/subsystem-and-drivetrain-modeling.md)
  — the first-principles modeling note this architecture realizes.
- `portable-motor-interface.md`, `portable-swerve-interface.md` (frc-code-scout) — the
  message-contract precedent.
- [reorganize-source-io-into-hal-comms-robot-configs.md](reorganize-source-io-into-hal-comms-robot-configs.md)
  — Phase 0 prerequisite (module reorg).

## Out of scope (whole program)

- Binary wire / serialize-deserialize — the ASCII wire stays.
- System / Config-registry / Debug command message families (deferred to a later phase).
- Folding goal logic into Drive — goals stay in the Planner.
- Swerve — the `WheelTarget` steer axis is reserved but no swerve image is built.
</content>
</invoke>
