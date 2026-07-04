# The Minimal Block Example — one interface per layer

*Status: design reference, 2026-07-04.*

This note is the smallest complete picture of the architecture: one capability
interface at the **hardware abstraction layer**, two subsystems at the
**subsystem layer** (a drive with two wheeled motors, a color-sensor
subsystem), and a **superstructure** that acts as a planner. Every layer
demonstrates the **four-part block structure**, and every name is the
*post-rename* form — no unit suffixes baked into identifiers (`velocity()`,
not `velocityMmps()`; units live in the doc comment).

It is illustrative, not a compilable library: production headers carry more
fields, split-phase I2C, sim hooks, and lifecycle detail. The shape is what
matters here.

---

## 0. The four-part block structure

Every active component is described by **four serializable message types plus
one engine verb**:

| Part | Symbol | Meaning | Verb that carries it |
|---|---|---|---|
| `Config` | θ | how to behave — tuning, calibration, identity | `configure(Config)` — delta-apply, live-safe |
| `Command` (in) | u | intent for *this* block, this tick | `apply(Command)` — **stage only**, never execute |
| `State` | x | what is happening — estimate + status | `state()` — const-ref snapshot, no I/O |
| `Command` (out) | — | intents for blocks *below* | the return value of `tick(now)` — a `CommandBatch` |

The one non-negotiable implementation rule: **emission is a return value,
never a side effect.** `tick()` is the only verb that produces outbound
commands, and it *returns* them; a block never holds a reference to the block
below it and pushes. The outer wiring loop routes each block's `CommandBatch`
to the next block's `apply()`. A block's Command-out is literally its child's
Command-in — commands are the edges of the tree.

Which of the four channels a component fills is what *kind* of component it
is. There is no type hierarchy of sensors vs. actuators vs. controllers; the
fill-pattern is the taxonomy:

| Component | Config | Cmd in | State | Cmd out | in one line |
|---|:--:|:--:|:--:|:--:|---|
| **Sensors** (color) | ✓ | – | ✓ | – | a pure source of observations |
| **Drive** (motors + wheels) | ✓ | ✓ setpoint | ✓ | – (leaf) | a controller over motors |
| **Planner** (superstructure) | ✓ | ✓ goal | ✓ | ✓ drive setpoints | a controller over subsystems |

Commands flow down (driver → planner → drive → motor), state flows up. The
planner and the drive are *the same kind of thing* — both fill all four
channels — differing only in whether their children are subsystems or motors.

---

## 1. The hardware abstraction layer

The HAL sits **below** the block tree. A device interface is the downward
edge of a leaf subsystem: it exposes cheap cached-accessor state refreshed by
`tick()`, and takes direct actuation calls from the one subsystem that owns
it. Vendor and bus code (Nezha registers, I2C) lives only in the concrete
implementations (`Motor`, `SimMotor`, `MockMotor`).

Naming rule demonstrated here: **capability in the type name, physics in the
method name, units in the comment.** The rename this note bakes in:

| Old (unit-suffixed) | New |
|---|---|
| `velocityMmps()` | `velocity()` — doc comment says `[mm/s]` |
| `readEncoder(...)` | `readEncoder(...)` — `[mm]`, float |
| `v_mmps`, `omega_rads` fields | `v`, `omega` — units on the struct doc |

```cpp
// IVelocityMotor — a motor you command to a wheel velocity.
// Named for the capability the caller depends on, not the hardware brand.
class IVelocityMotor {
public:
    virtual ~IVelocityMotor() = default;

    // Prime hardware at boot (encoder readback, etc.). Default no-op.
    virtual void begin() {}

    // Per-loop I/O phase: perform the encoder read and cache the results.
    // Called once per loop, before any control code runs. After tick(),
    // position()/velocity() are free — no further bus traffic.
    virtual void tick(uint32_t now) { (void)now; }

    // Command: signed output demand, -100..100 [% duty]. Positive = forward.
    virtual void setOutput(int8_t pct) = 0;

    // Cached state — refreshed by the most recent tick(), no I/O.
    virtual float position() const = 0;   // cumulative wheel travel [mm]
    virtual float velocity() const = 0;   // differentiated wheel speed [mm/s]

    // Zero the position accumulator (software offset — no bus traffic).
    virtual void rebaseline() = 0;
};
```

```cpp
// IColorSensor — an RGBC color source. Same cached-accessor idiom: tick()
// owns the bus transaction, color() is free.
class IColorSensor {
public:
    virtual ~IColorSensor() = default;

    // Raw 16-bit channel counts, valid as of the last completed read.
    struct Reading { uint16_t r, g, b, c; uint32_t stamp; bool fresh; };

    virtual void begin() = 0;
    virtual void tick(uint32_t now) = 0;          // non-blocking poll/advance
    virtual const Reading& color() const = 0;     // cached, no I/O
};
```

One registry hands the subsystems their devices. Concrete HALs (`NezhaHAL`
for firmware, `SimHardware` for the host plant, `MockHAL` for tests) own the
device objects; nothing above this line ever names a concrete device type.

```cpp
// Hardware — abstract device registry. The run-mode switch: REAL / SIM /
// REPLAY each resolve every interface here, in one place.
class Hardware {
public:
    virtual ~Hardware() = default;

    virtual IVelocityMotor& motorL()      = 0;
    virtual IVelocityMotor& motorR()      = 0;
    virtual IColorSensor&   colorSensor() = 0;

    virtual void begin() = 0;               // initialize all owned devices
    virtual void tick(uint32_t now) = 0;    // per-loop device I/O phase
};
```

---

## 2. The configuration object

Config is **parameters, not wiring**: identity and calibration that does not
change within a control session. Assembly — which devices belong to which
subsystem — is code, not config. The boundary test: *if it changes every
loop it's a Command; if it identifies or calibrates the block across a
session it's Config.*

One aggregate nests a slice per block; each slice flows down exactly once at
boot via `configure()`, and any slice may be re-applied live (a `SET` at
runtime becomes a fresh `configure()` call — the next `tick()` reads it).

```cpp
// DriveConfig — geometry + wheel-loop tuning.
struct DriveConfig {
    float trackwidth    = 120.0f;   // wheel separation [mm]
    float wheelDiameter = 48.0f;    // [mm]
    float maxWheel      = 400.0f;   // wheel speed limit [mm/s]
    float kP = 0.8f, kI = 0.2f, kFF = 0.25f;   // wheel velocity PID

    DriveConfig& setTrackwidth(float v) { trackwidth = v; return *this; }
    DriveConfig& setMaxWheel(float v)   { maxWheel   = v; return *this; }
    // ... chainable setters for every field (generated) ...
};

// SensorsConfig — read cadence + classification calibration.
struct SensorsConfig {
    uint32_t lag       = 50;      // min interval between reads [ms]
    uint16_t whiteC    = 9000;    // clear-channel calibration point
    uint16_t blackC    = 300;

    SensorsConfig& setLag(uint32_t v) { lag = v; return *this; }
    // ...
};

// PlannerConfig — motion limits for goal closure.
struct PlannerConfig {
    float vMax    = 300.0f;   // body speed limit [mm/s]
    float aMax    = 600.0f;   // accel limit [mm/s^2]
    float goalEps = 5.0f;     // position tolerance [mm]

    PlannerConfig& setVMax(float v) { vMax = v; return *this; }
    // ...
};

// RobotConfig — the one aggregate the robot owns. Slices flow down at boot;
// runtime SETs mutate this copy and re-configure() the affected block.
struct RobotConfig {
    DriveConfig   drive;
    SensorsConfig sensors;
    PlannerConfig planner;
};
```

---

## 3. The subsystem layer

### 3a. Drive — motors and wheels (fills all four channels)

Three POD messages plus the verbs. The Command is a tagged union — one
control kind per tick; staging a new command replaces the old one.

```cpp
// -- the four parts -----------------------------------------------------

// Command (in): what to do this tick. Staged by apply(), executed by tick.
struct DriveCommand {
    enum class Kind : uint8_t { NONE, TWIST, WHEELS, NEUTRAL };
    Kind kind = Kind::NONE;
    union {
        struct { float vx, vy, omega; } twist;   // body frame [mm/s, mm/s, rad/s]
        struct { float left, right;   } wheels;  // wheel frame [mm/s]
    } u = {};

    DriveCommand& setTwist(float vx, float vy, float om) {
        kind = Kind::TWIST; u.twist = {vx, vy, om}; return *this;
    }
    DriveCommand& setWheels(float l, float r) {
        kind = Kind::WHEELS; u.wheels = {l, r}; return *this;
    }
    DriveCommand& setNeutral() { kind = Kind::NEUTRAL; return *this; }
};

// State: estimate (pose, twist, wheels) AND status (connected, atTarget).
struct DriveState {
    struct { float x, y, h; }        pose;    // fused pose [mm, mm, rad]
    struct { float vx, vy, omega; }  twist;   // measured body twist
    float    wheel[2];                        // measured wheel speeds [mm/s]
    bool     connected = false;               // both motors responding
    bool     atTarget  = false;               // wheel loops within tolerance
    uint32_t stamp     = 0;                   // when this snapshot was taken [ms]
};

// Config: DriveConfig, §2.   Command (out): none — Drive is a leaf; its
// tick returns an EMPTY CommandBatch. (The slot exists so every block has
// the same signature; a swerve Drive commanding steer modules would use it.)

// -- the block ----------------------------------------------------------

class Drive {
public:
    Drive(IVelocityMotor& left, IVelocityMotor& right, const RobotConfig& cfg);

    // STAGE only — writes the pending-command slot. No I/O, no emission.
    void apply(const DriveCommand& cmd);

    // SENSE phase: read encoders (they were tick()ed by the HAL already),
    // run odometry/fusion, refresh _state. Called before ANY actuation.
    void tickUpdate(uint32_t now);

    // ACT phase: staged command → kinematics → wheel PID → motor outputs.
    // The ONLY emitter — and for this leaf, the batch is empty.
    CommandBatch tickAction(uint32_t now);

    // Const-ref snapshot, no I/O, no copy. tickUpdate() keeps it fresh.
    const DriveState& state() const { return _state; }

    // Delta-apply params; the next tick reads them. Live-safe.
    void configure(const DriveConfig& cfg);

    // Declared truth: accepted command kinds, populated state fields.
    Capabilities capabilities() const;

private:
    IVelocityMotor& _left;      // HAL edge — the only place device
    IVelocityMotor& _right;     // interfaces appear above the HAL
    DriveCommand    _pending;   // staged intent (internal memory, not State)
    DriveState      _state;
    DriveConfig     _cfg;
};
```

Drive is **two-phase** because the loop must order all sensing before all
actuation: `tickUpdate()` senses, `tickAction()` acts and emits. Sensors
below is single-phase — one `tick()` — because it never actuates.

### 3b. Sensors — the color sensor (pure observation: no Command)

A pure-observation subsystem has an **empty command axis**. That is not a
degenerate case to apologize for — it *is* the sensor fill-pattern from the
§0 table, expressed by simply not having `apply()`.

```cpp
// State: estimate (channel counts + classification) and status (connected).
struct SensorsState {
    uint16_t r, g, b, c;                       // raw counts, last read
    enum class Color : uint8_t { UNKNOWN, BLACK, WHITE, RED, GREEN, BLUE };
    Color    classified = Color::UNKNOWN;      // calibrated classification
    bool     connected  = false;
    uint32_t stamp      = 0;                   // time of last fresh read [ms]
};

class Sensors {
public:
    Sensors(IColorSensor& color, const RobotConfig& cfg);

    // Single-phase: when the lag gate fires, take the cached HAL reading,
    // classify it against calibration, refresh _state. Returns nothing —
    // a sensor commands nobody.
    void tick(uint32_t now);

    const SensorsState& state() const { return _state; }

    void configure(const SensorsConfig& cfg);

    Capabilities capabilities() const;   // command_modes empty; state fields listed

private:
    IColorSensor& _color;
    SensorsState  _state;
    SensorsConfig _cfg;
};
```

---

## 4. The superstructure — a planner

The planner is **goal closure**: it turns a goal plus the observed state into
a time-varying drive setpoint, and decides when the goal is reached. Velocity
loops live in Drive; the planner never touches a motor, a device, or a vendor
header (the vendor-confinement grep gate must see zero hits here).

Structurally it is *the same four-part block as Drive* — the executive is not
a special case. Its Command-in is a goal; its Command-out is Drive's
Command-in. That is the tree.

```cpp
// -- the four parts -----------------------------------------------------

// Command (in): a goal, not a setpoint. One goal active at a time;
// applying a new goal preempts the old one.
struct PlannerCommand {
    enum class Goal : uint8_t { NONE, GOTO, TURN, VELOCITY, STOP };
    Goal goal = Goal::NONE;
    union {
        struct { float x, y, speed; } gotoGoal;   // field target [mm, mm/s]
        struct { float heading;      } turn;      // absolute [rad]
        struct { float vx, omega;    } velocity;  // open-loop twist
    } u = {};

    PlannerCommand& setGoTo(float x, float y, float speed) {
        goal = Goal::GOTO; u.gotoGoal = {x, y, speed}; return *this;
    }
    PlannerCommand& setStop() { goal = Goal::STOP; return *this; }
    // ...
};

// State: status-dominant — for an executive, WHAT IT IS DOING is the primary
// output; the estimate is secondary (it lives in DriveState anyway).
struct PlannerState {
    PlannerCommand::Goal active = PlannerCommand::Goal::NONE;
    float    remaining = 0.0f;    // distance/angle to goal [mm or rad]
    bool     done      = true;    // goal reached (or none active)
    uint32_t stamp     = 0;
};

// Config: PlannerConfig, §2.
// Command (out): DriveCommand — carried in the returned CommandBatch.

// -- the block ----------------------------------------------------------

class Planner {
public:
    explicit Planner(const RobotConfig& cfg);

    // STAGE the goal. now baselines the motion profile (time is an input,
    // never read from a clock inside the block).
    void apply(const PlannerCommand& cmd, uint32_t now);

    // Advance goal closure one tick:
    //   1. read observations: drive.state() (fused pose/twist),
    //      sensors.state() (e.g. stop-on-red) — children's State IS the
    //      planner's observation set; it holds no device references.
    //   2. advance the profile / steering law; update _state (done? remaining?)
    //   3. RETURN a batch containing one DriveCommand{TWIST} setpoint.
    // The planner does not call drive.apply() itself — the wiring loop
    // routes the batch. Emission is the return value.
    CommandBatch tick(uint32_t now,
                      const DriveState& drive,
                      const SensorsState& sensors);

    const PlannerState& state() const { return _state; }

    void configure(const PlannerConfig& cfg);

private:
    PlannerCommand _goal;      // staged intent
    PlannerState   _state;
    PlannerConfig  _cfg;
    // internal memory (profile progress, last timestamp) — deterministic,
    // never exposed in State
};
```

---

## 5. The wiring loop — where the tree becomes a program

The outer loop is the only impure choreography in the system. It owns the
ordering rule — **all sensing before all actuation** — and it is the router
that turns one block's Command-out into another's Command-in:

```cpp
void Robot::loopOnce(uint32_t now) {
    // -- SENSE: devices first, then the blocks that read them ------------
    hal.tick(now);                    // HAL I/O phase: encoders, color read
    sensors.tick(now);                // color classification → SensorsState
    drive.tickUpdate(now);            // odometry/fusion → DriveState

    // -- PLAN: goals become setpoints (pure; emission = return value) ----
    CommandBatch fromPlanner =
        planner.tick(now, drive.state(), sensors.state());

    // -- ROUTE: planner's Command-out is Drive's Command-in --------------
    route(fromPlanner);               // → drive.apply(DriveCommand{TWIST})

    // -- ACT: staged commands reach hardware ------------------------------
    CommandBatch fromDrive = drive.tickAction(now);   // empty — Drive is a leaf
    route(fromDrive);

    // -- LOG: every channel is a POD; log cmd-in, state, cmd-out ---------
}
```

A driver or wire command enters the same way — the command processor builds a
`PlannerCommand` (or, for manual control, a raw `DriveCommand`) and stages it
via `apply()`:

```cpp
// "go to (500, 200) at 250 mm/s"
planner.apply(PlannerCommand{}.setGoTo(500.0f, 200.0f, 250.0f), now);

// manual teleop twist, bypassing the planner:
drive.apply(DriveCommand{}.setTwist(vx, 0.0f, omega));

// live retune, mid-session:
cfg.planner.setVMax(200.0f);
planner.configure(cfg.planner);
```

---

## 6. Why this shape is worth the discipline

Every property the architecture promises is visible in this one example:

- **Testability without hardware.** `Planner::tick` takes `DriveState` and
  `SensorsState` by value/ref and returns a batch — a test constructs the
  observations by hand and asserts on the return. No scheduler, no HAL, no
  clock (time arrives as `now`).
- **Sim / replay for free.** Swap the `Hardware` registry (`SimHardware`,
  `ReplayHAL`) and nothing above the HAL line changes. Replaying logged
  commands + observations through the pure ticks reproduces the run.
- **Uniform logging.** All four channels of every block are flat PODs — the
  same serializer logs a motor command and a planner goal.
- **The fill-pattern is the documentation.** `Sensors` has no `apply()`;
  Drive's batch is empty; the planner's batch carries `DriveCommand`s. What
  each block *is* can be read off its signature.

Reference implementations of the full-detail versions: `SubsystemContract.h`
(the contract), `subsystems/drive/Drive.h`, `subsystems/sensors/Sensors.h`,
`superstructure/Planner.h`, `hal/Hardware.h`, `hal/capability/*.h`. The
Block model this instantiates is chapter 25 of the elite-architecture book
(frc-code-scout, *The Portable Component Model — the Block*).
