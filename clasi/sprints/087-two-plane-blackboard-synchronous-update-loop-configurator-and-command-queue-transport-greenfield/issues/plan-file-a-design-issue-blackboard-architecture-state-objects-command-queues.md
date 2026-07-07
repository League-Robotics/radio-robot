---
status: in-progress
sprint: 087
tickets:
- 087-001
- 087-002
- 087-003
- 087-004
- 087-005
- 087-006
- 087-008
- 087-009
- 087-007
---

# Two-plane blackboard: subsystems own state, transport is a system property

**A design issue, to be executed and shipped.** This document is where the design
is worked out; once it's settled it feeds CLASI sprint planning
(architecture-update → sequenced tickets → implementation → merge) and ships as
real code. No code yet — the design is being finalized here. Migration is decided:
**greenfield by deletion** — delete the whole `main` loop and `dev_loop.*`, then
rebuild the subsystems and the loop from scratch, adding the new files (blackboard,
queue primitives, Configurator, router) as needed. The 50 Hz cadence is a
nice-to-have, not a gate: synchronous-update correctness is rate-independent. It is
"mostly about the blackboard and the queues," plus the execution model (synchronous
update), a dedicated Configurator, and a cyclic-executive loop that fell out of the
discussion.

## Context / motivation

Today every command family is wired through a `*State` struct that holds **raw
pointers to the subsystems** and either pokes them (actuators) or reads them
(observers): `ConfigCommandState`, `PoseCommandState`, `DevLoopState`,
`TelemetryState`, `MotionLoopState`, `OtosCommandState`, all assigned in `main.cpp`
and mirrored in the sim entry (grounded inventory at the end of this doc).

That inverts the dependency the wrong way and produces concrete smells:

- **A thing named `state` holds a pointer to a subsystem, and drives it.** The arrow
  points `commands → subsystems`; the command layer even `#include`s the whole
  subsystem layer. `ConfigCommandState` calls `drivetrain->configure()` synchronously
  from inside a handler.
- **The name lies.** `DevLoopState` is a grab-bag of (a) subsystem pointers, (b) an
  *outbox* staging a motor command, (c) config *shadows* — three unrelated jobs. A
  staged `CommandProcessorToHardwareCommand`, whose own type name says it is *mail
  between two parties*, should not live in a struct called `…State`.
- **Shadows are a workaround for missing readable state.** `motorConfigShadow[]` /
  `drivetrainConfigShadow` exist *only* because `Hal::Motor` / `Subsystems::Drivetrain`
  expose `configure()` (full replace) but **no getter for the current config**.
- **Cross-family reach-through.** `ConfigCommandState::sTimeoutWatchdog` points
  straight into another family's guts (`MotionLoopState::sTimeout`).
- **Duplication.** `hardware = &hardware` (and the other handles) are copied across
  **seven** holders — the six command states *plus* `DevLoop` — and re-wired
  identically again in the sim entry.
- **The faceplate is not even uniform.** `PoseEstimator` has no `apply()`/`state()`;
  `Hardware`'s config/state live per-port on `Hal::Motor`; `Communicator` has no
  `apply()`.

**Goal.** Subsystems should be constructible in isolation, own their state and config,
and expose an **enumerable, minimal per-tick dependency set** so they can be
unit-tested without a network of wiring or mocks — and the whole system should be
**deterministic and order-independent**. Nothing named `state` should hold a pointer
to a subsystem. The direction must invert: **subsystems read and write state; state
references nobody.**

The distinction that makes it possible: **a reference to a state object is not a
pointer to a subsystem.** A handler may hold a data-plane handle (a state cell, a
command queue); only a subsystem holds behavioral handles, and it points only at
state — the allowed direction.

## Decisions vs. open questions (after this session)

**Decided:** two-plane blackboard (state objects + command queues); **synchronous
update** with a double-buffer; a dedicated **Configurator** subsystem for all config
application; a **cyclic-executive** loop (best-effort control cadence + slack drain);
the **config-replace vs. state-reset** split (SI/ZERO are target-drained, not
Configurator-applied); and **greenfield-by-deletion migration** (delete `main`/
`dev_loop`, rebuild subsystems and loop). Open questions are collected at the end.

## Execution model: synchronous update (the core semantic)

Everybody reads the blackboard's **frozen snapshot `x[k]`**. Every subsystem computes
its next state from that same snapshot and writes into **its own cell**. After all
control ticks, the loop **bulk-copies** every subsystem's cell into the blackboard,
producing `x[k+1]`. Next pass, everybody reads `x[k+1]`.

That is `x[k+1] = f(x[k], inputs[k])` — a discrete-time state-space update, the same
discipline as a clocked digital circuit or a fixed-step solver with a unit-delay on
every wire. The blackboard is the register bank; the bulk copy is the clock edge.

**Why (this is the whole point): determinism and order-independence.** If a subsystem
read a *freshly-ticked* peer's output this pass, tick order would leak into behavior —
ticking pose before drivetrain would differ from after. Reading only the frozen
snapshot makes tick order **irrelevant to results**: same snapshot + same inputs →
same next snapshot, regardless of scheduling.

**The double-buffer is the mechanism.** Two state cells per subsystem:

- **Read buffer** = the blackboard `x[k]`, frozen for the whole pass.
- **Write buffer** = each subsystem's own cell, invisible to peers this pass.
- **Commit** = the bulk copy, flipping `x[k] → x[k+1]` atomically.

If a subsystem wrote straight into the blackboard mid-pass, a later-ticked peer would
see the new value and order would matter again — hence separate read and write cells,
committed all at once. Subsystems touch neither each other nor the blackboard: the
loop passes each its inputs as explicit args sourced from `x[k]`; each writes only its
own cell; the loop's copy step is the only reader of subsystem cells.

**Cost, stated plainly.** Every cross-subsystem signal carries exactly **one tick**
(~20 ms) of latency — uniform, known, *not* order-dependent. Sensor→pose is one tick;
sensor→pose→planner is two. Today's `dev_loop.cpp` deliberately does the opposite
(reads freshest post-slice-2 motor state, feeds the planner this-pass fused pose), so
this adds a clean `z⁻¹` per hop that the dt-based integration tolerates but the control
may want minor re-tuning — a phase/latency shift, not an instability. The one
sanctioned exception stays what it is today: an emergency stop (watchdog neutral) acts
immediately, not on the next edge.

The payoff is the testability goal: a subsystem test is purely functional — build it,
hand it a snapshot of plain values, tick, assert on its output cell. No ordering, no
wiring, no mocks.

## The blackboard: two planes

| Plane | Vehicle | Read semantics | Writers | Carries |
|---|---|---|---|---|
| **State** | state object (current-value cell) | non-destructive, always-current | one writer | observations (motor state), fused pose, current config, subsystem state |
| **Command** | queue (`WorkQueue` / `Mailbox`) | destructive `pop` (or non-destructive `iterate`) | many producers → one (mostly) consumer | inbound statements, drive commands, config deltas, state-reset commands |

**Observations flow through state objects, never through queues.** An observation is a
*current value*, not a message; a queue would force a state concept into a command
mechanism (and invent a "who clears the queue" problem a latched cell doesn't have).
Motor observations are read by drivetrain, pose, *and* planner; fused pose by planner
*and* telemetry — one writer, many readers: exactly a state object.

The blackboard owns both planes. State objects are double-buffered (above). Command
queues are single-consumer point-to-point, so they need no double-buffer — there is no
cross-subsystem read-ordering to protect. Everything a subsystem needs is passed into
`tick()` as **explicit typed parameters** — never a generic blackboard handle it
name-looks-up topics from, which would re-hide the dependencies we are making
enumerable. Generic queue *mechanism*, explicit *wiring*.

## The Faceplate contract

Each subsystem:

- **Constructs with no peers.** Owns its config and its internal (integrator) state.
- `configure(Config)` — full replace, **called externally by the Configurator**
  (see below), not from inside `tick()`. `config() -> Config` exposes current config
  as a readable state object (this getter is what kills the shadows).
- `apply(Command)` — post an inbound command onto an input queue (or the loop routes
  into it).
- `tick(now, <state-object args from x[k]>, <input queues>)` — read the passed
  snapshot values, consume input queues, advance internal state, **write its own state
  cell**. Its **entire dependency set is enumerable from the signature**.
- `state() -> State`, `capabilities() -> Capabilities`.

Note what `tick()` does *not* take: no output-state parameters (the subsystem updates
its own cell, exposed via `state()`; the loop reads it for both wiring and the commit)
and no config queue (config is the Configurator's job). So the control subsystems'
`tick` signatures collapse back to nearly what they are today — the new machinery is
in the loop, the blackboard, and the Configurator, not in the control subsystems.

## Queue taxonomy (command plane)

Two orthogonal axes collapse to a small set of concrete types:

| Type | Capacity | Read | Consumers | For |
|---|---|---|---|---|
| `Mailbox<T>` | 1, latest-wins (overwrite) | `pop` | one | absolute setpoints (twist, wheel/motor targets) |
| `WorkQueue<T>` | N, FIFO | `pop` (or `iterate`) | one (or many) | statements, config deltas, state-reset commands |

**Rule: coalesce absolute setpoints; queue increments.** A twist is absolute — only
the newest matters → `Mailbox`. A config write is a *delta* — `tw=128` then `rotSlip=0`
must **both** apply; deltas do not commute with overwrite → `WorkQueue`. It's a
property you read off the payload, not the subsystem. `iterate` (non-destructive,
multi-reader) exists on the command plane only, for the rare *command* several
subsystems must see; the common multi-reader case (observations) is state objects, so
most command queues are plain `pop`.

## Loop scheduling: a cyclic executive

The loop runs a **fixed-rate control task first** (the mandatory major cycle), then
drains commands/config in **best-effort slack** until the next control deadline. This
protects the control cadence: config can never push the control tick late, and a
config pileup never delays a motion command's execution.

The concrete loop is in **Reference code** below. Its properties:

- **Self-balancing.** Idle → cheap control tick → most of the 20 ms is slack → config
  drains fast. Driving hard → costlier control → less slack → config drains slower,
  which is exactly when config changes are rarest.
- **Dynamic slack.** Measure the deadline against the wall clock *after* the mandatory
  portion — do not budget a fixed control time. This absorbs the variable cost of the
  mandatory portion, which is **dominated by the I2C encoder reads** (`hardware.tick`'s
  0x46 request/collect), the one genuinely variable, occasionally-slow thing.
- **Routing beats application.** Routing a statement is parse + enqueue (µs); config
  *application* (`configure()`, incl. the EKF re-init) is the expensive part. Letting
  routing win means a 25-`SET`-then-motion burst lands all 25 in the config queue and
  the motion command in `driveIn` almost instantly (executed next mandatory pass, ≤ 20
  ms), while config application drains in the remaining slack, spanning passes if
  needed. A motion command never waits behind config *application*.
- **Graceful degradation.** If the mandatory portion overruns the period, there is no
  slack that pass — control keeps running (momentarily slower), config waits. Priority
  is preserved by construction.
- **"Busy wait" is productive** — hard-polling the communicator, ingesting bytes the
  instant they arrive; lowest command latency, not a dead spin.

The 50 Hz cadence is a **target, not a requirement.** It is I2C-bound (the
encoder-read tail), and if the mandatory portion can't fit the period we simply run
slower — synchronous-update correctness is rate-independent; only control *tuning*
cares about the actual rate. So the period is best-effort pacing, not a hard
deadline, and `kMargin`/deadline handling degrades to "just proceed" when a pass
overruns. The full loop is in **Reference code** below.

## The Configurator (single config authority)

A dedicated **Configurator subsystem** receives config messages from the command
processor and is the one place that applies them, in the slack.

- The router sends every config write — `SET`, `DEV M CFG`, `DEV DT CFG`, OTOS config —
  onto **one** `WorkQueue<ConfigDelta>`, each delta tagged with its target. This
  subsumes the three scattered shadow sets in `DevLoopState`, `ConfigCommandState`, and
  `OtosCommandState`.
- The Configurator is the **single source of truth for desired config**. It folds
  deltas FIFO into its own per-target copy and, in the slack, calls plain `configure()`
  on each *changed* target. Subsystems keep `configure()` exactly as today — no config
  queue, no desired/current bookkeeping in their ticks.
- It **publishes current config to state objects** for `GET`/telemetry.
- **Validation stays synchronous at the `SET` handler**: it reads the published
  current-config state object, folds the candidate, and replies `ERR` immediately on an
  invariant failure — nothing enqueued. The Configurator is pure apply, so the
  synchronous-`ERR` wire contract survives even though application is deferred.

Why this is not the smell we're removing: the objection was "a thing named *state*
holds a subsystem pointer." A **Configurator** whose literal job is configuration
legitimately reaches the things it configures — it is a purpose-built actor at a clean
phase boundary, replacing coupling smeared across six mislabeled "state" structs with
**one honestly-named component**, while command handlers hold **zero** subsystem
pointers. (If even that asymmetry is unwanted, the variant is: the Configurator
computes pending configs and the *loop* applies them, keeping all subsystem refs in
the loop alone.)

Two concrete wins beyond "no pointers": subsystems keep their existing `configure()`
untouched; and reconfigure lands at a **quiescent boundary** rather than mid-tick —
which matters here because `PoseEstimator::configure()` re-inits the EKF and re-zeroes
the fused pose/covariance, something you do not want firing in the middle of a pass's
control math.

## One-shot ops: config replace vs. state reset

SI/ZERO forces a distinction the design must record. There are **two flavors** of
one-shot / config-plane operation, split by whether the effect is entangled with the
target's per-tick math:

- **External config replace → Configurator-applied.** `configure()` is a clean full
  replace; nothing about it is entangled with the subsystem's integration. Applied at
  the boundary by the Configurator.
- **Internal state reset → target-drained.** A pose/baseline/encoder reset is
  entangled with the estimator's integration (the phantom-jump coherence lives *inside*
  the estimator). The **target subsystem** drains its own reset queue in its `tick()`;
  routing it through the Configurator would leak estimator internals into it.

Both are still "command lands in a queue, consumed at the clock edge" — they differ
only in *who* consumes.

## Worked example: config (Configurator path)

- `SET tw=128 rotSlip=0` → the `SET` handler reads the current `DrivetrainConfig` state
  object, validates the merged candidate (synchronous `ERR` on failure), and posts a
  `DrivetrainConfigDelta` (tagged `drivetrain`) onto the Configurator's queue. Replies
  `OK`. Holds **no** `Drivetrain*`.
- In the slack, the Configurator folds the delta into its desired `DrivetrainConfig`
  and calls `drivetrain.configure(desired)`; it publishes the new current config to the
  state object.
- `GET`/telemetry read the current-config state object. `SET sTimeout` posts a
  `PlannerConfigDelta` — the cross-family `sTimeoutWatchdog` reach-through disappears.

```cpp
// Control subsystem: config is external; tick stays lean.
void Drivetrain::tick(uint32_t now,
    const msg::MotorState& leftObs, const msg::MotorState& rightObs,  // from snapshot x[k]
    Mailbox<msg::DrivetrainCommand>& driveIn);                        // pop, latest-wins
// writes its own state cell; exposes msg::DrivetrainState state() / msg::DrivetrainConfig config();
// configure(const msg::DrivetrainConfig&) is called by the Configurator, not here.
```

## Worked example: SI/ZERO (state-reset path)

Grounded in today's handlers: `SI <x> <y> <h>` fans out to **two** targets —
`poseEstimator->setPose()` **and** `odometer->apply(SET_POSE)` (re-anchor estimator and
OTOS together, so the next fusion pass reads an odometer sample already agreeing with
the anchor). `ZERO enc` fans out to **three** — `motor(left/right).resetPosition()`
**and** `poseEstimator->resetEncoderBaseline()` (zero the encoders and resync the delta
baseline, or you integrate a delta across the discontinuity: a phantom jump).

Under this architecture:

- **Atomicity comes from the clock edge.** The router fans one wire verb out into
  typed reset commands on each target's reset queue; all are consumed on the *same*
  next edge and commit to `x[k+1]` together. No partial-reanchor window — the property
  the current inline pointer-pokes strain to guarantee falls out of simultaneous
  commit.
- **Router fan-out, no pointers.** The handler becomes a router step; it reads the port
  binding from the **blackboard snapshot** (not a `Drivetrain*`) to address the
  encoder-reset.
- **Target-drained** (per the split above): `PoseEstimator` drains its reset queue in
  its own tick, because the reset is entangled with its integration.
- **The phantom-jump coherence uses the existing mechanism.** Under synchronous update
  you must **not** set the baseline to the current snapshot reading (the encoder becomes
  0 on the same edge → a `−E` phantom jump next edge). `PoseEstimator` already sets a
  **pending flag** on `resetEncoderBaseline()` and snaps the baseline to the zeroed
  reading only when it propagates into the snapshot — the queued command just triggers
  that flag. No new complexity; the entanglement stays inside the estimator, which is
  why it's target-drained.
- **Fusion suppression:** when the estimator consumes a `setPose` on an edge, it
  overrides pose and suppresses that edge's odometer fusion (the old-frame reading), so
  the just-set anchor isn't immediately corrupted; next edge the odometer reads the
  re-anchored value and fusion resumes with zero residual.
- **Queues:** pose-reset = a small `WorkQueue<PoseResetCommand>` (`SI` and `ZERO` are
  distinct, ordered, both must apply); motor encoder-reset = an idempotent flag (reuse
  the existing `resetPosition()` staging — reset-twice = reset-once).
- **Wire reply:** synchronous `OK` (accepted + routed), effect lands next edge —
  identical to the `SET` pattern; nothing to validate, so `OK` is unconditional.

## Reference code (illustrative — for review)

Real type names; exact queue capacities, output-command edges, and the odometer
path are deferred to the architecture-update. The `Rt::` namespace is a placeholder.
This is the code we want to agree on before sprint planning.

### The two command-plane primitives — `queue.h`

```cpp
namespace Rt {

// Mailbox<T> — capacity 1, latest-wins. For ABSOLUTE setpoints: an unread older
// value is pure staleness, so post() overwrites.
template <typename T>
class Mailbox {
 public:
  void post(const T& v) { value_ = v; full_ = true; }   // overwrite
  bool empty() const    { return !full_; }
  T    take()           { full_ = false; return value_; }   // pop (destructive)
 private:
  T    value_ = {};
  bool full_  = false;
};

// WorkQueue<T, N> — FIFO, capacity N. For DELTAS/commands that must all apply, in
// order. post() returns false when full (caller decides drop vs. ERR).
template <typename T, uint32_t N>
class WorkQueue {
 public:
  bool     post(const T& v);            // append; false if full
  bool     empty() const;
  T        take();                      // pop front (destructive)
  const T* peek(uint32_t i) const;      // iterate (non-destructive), i in [0, size())
  uint32_t size() const;
 private:
  T        buf_[N];
  uint32_t head_ = 0, tail_ = 0, count_ = 0;
};

}  // namespace Rt
```

### The Faceplate — a shape, not one vtable

Every subsystem provides these, with its own message types. `tick()`'s argument
list is **deliberately per-subsystem** — it names exactly this subsystem's
dependencies (the enumerable-dependency / testability property). It reads only the
passed-in snapshot values (`x[k]`) and its input queues; it writes only its own
state cell.

```cpp
// The contract (pseudo-signature; each subsystem specializes the types + tick args):
//   Subsystem();                                     // constructs with NO peers
//   void         configure(const Config&);           // full replace; called by the Configurator
//   Config       config() const;                     // current config (readable) — kills the shadows
//   void         tick(now, <snapshot reads…>, <input queues…>);   // reads x[k], writes own cell
//   State        state() const;                      // current state (own write cell)
//   Capabilities capabilities() const;               // optional
//
// Concrete example — Drivetrain (config is external via the Configurator, so tick
// carries no config queue; it collapses to nearly today's signature):

namespace Subsystems {

class Drivetrain {
 public:
  Drivetrain();

  void configure(const msg::DrivetrainConfig& config);   // called by the Configurator, not tick()
  msg::DrivetrainConfig config() const;

  void tick(uint32_t now,
            const msg::MotorState& leftObs,             // from bb.motor[…], i.e. x[k]
            const msg::MotorState& rightObs,
            Rt::Mailbox<msg::DrivetrainCommand>& driveIn);   // pop, latest-wins

  msg::DrivetrainState        state() const;            // own cell → copied into the blackboard at commit
  msg::DrivetrainCapabilities capabilities() const;

  // Output edge (drained by the loop into Hardware's input): a subsystem that
  // EMITS a command exposes it the way today's code does.
  bool                        hasCommand() const;
  Hal::DrivetrainToHardwareCommand takeCommand();

  DrivetrainPorts ports() const;                        // bound pair, from current config

 private:
  // internal (integrator) state, authority, and configCurrent_ — all private.
};

}  // namespace Subsystems
```

Variations (already true today, kept): `PoseEstimator` publishes
`encoderPose()`/`fusedPose()` and drains a reset queue; `Hardware`'s config/state are
per-port on `Hal::Motor`; `Communicator` is a statement *source*
(`hasStatement()`/`takeStatement()`), no `apply()`.

### The blackboard — `blackboard.h`

```cpp
namespace Rt {

constexpr uint32_t kPortCount = Subsystems::Hardware::kPortCount;   // 4

// SI/ZERO fan-out consumed by PoseEstimator in its own tick (target-drained reset).
struct PoseResetCommand {
  enum Kind { kSetPose, kResetBaseline } kind;
  msg::SetPose pose;                        // valid when kind == kSetPose
};

// A target-tagged config delta headed for the Configurator's single queue.
struct ConfigDelta {
  enum Target { kDrivetrain, kMotor, kPlanner, kOdometer } target;
  uint32_t port;                            // motor index when target == kMotor
  // …the changed fields + a field mask (illustrative)…
};

// Owned by the loop. Holds NO subsystem pointers — only the committed snapshot
// x[k] (state plane) and the command queues (command plane).
struct Blackboard {
  // === State plane: the committed snapshot x[k]. Written ONLY by the loop's commit
  //     step (from each subsystem.state()); read-only to everyone during a pass. ===
  msg::MotorState       motor[kPortCount];        // from Hardware
  msg::DrivetrainState  drivetrain;               // from Drivetrain
  msg::PoseEstimate     encoderPose;              // from PoseEstimator
  msg::PoseEstimate     fusedPose;                // from PoseEstimator
  msg::PlannerState     planner;                  // from Planner
  bool                  otosValid = false;        // odometer sample present this snapshot?
  msg::PoseEstimate     otos;                     // from Hardware::odometer(), when valid

  // Current config — published by the Configurator on apply; read by GET/telemetry.
  msg::DrivetrainConfig drivetrainConfig;
  msg::MotorConfig      motorConfig[kPortCount];
  msg::PlannerConfig    plannerConfig;
  msg::OdometerConfig   odometerConfig;

  // === Command plane: queues. Each drained by exactly ONE consumer. ===
  WorkQueue<Subsystems::CommunicatorToCommandProcessorStatement, 16> statementsIn; // → router
  Mailbox<msg::DrivetrainCommand>    driveIn;              // router / Planner-out → Drivetrain
  Mailbox<msg::MotorCommand>         motorIn[kPortCount];  // → Hardware, per motor
  WorkQueue<ConfigDelta, 16>         configIn;             // router → Configurator
  WorkQueue<PoseResetCommand, 4>     poseResetIn;          // router → PoseEstimator
  bool                               motorResetIn[kPortCount] = {};  // ZERO enc → Hardware
  Mailbox<msg::SetPose>              otosSetPoseIn;        // SI re-anchor → odometer
};

}  // namespace Rt
```

### The main loop — replaces `main()`'s `while` + all of `dev_loop.*`

```cpp
int main() {
  // --- Construction: each subsystem built independently; owns its own state. ---
  MicroBit uBit;  uBit.init();
  I2CBus i2c(uBit);
  Subsystems::Communicator  comm;
  Subsystems::NezhaHardware  hardware(i2c, Config::defaultMotorConfigs());
  Subsystems::Drivetrain     drivetrain;
  Subsystems::PoseEstimator  poseEstimator;
  Subsystems::Planner        planner;
  Subsystems::Telemetry      telemetry;

  // The one component that legitimately holds subsystem refs: the config authority.
  Configurator configurator(drivetrain, poseEstimator, planner, hardware,
                            Config::defaultDrivetrainConfig(), defaultPlannerConfig());
  CommandRouter router;                 // wire statement → typed commands onto bb queues
  Rt::Blackboard bb;

  comm.begin();  hardware.begin();
  configurator.publish(bb);             // seed bb's current-config cells from boot config

  constexpr uint32_t kPeriod = 20;      // [ms] target cadence — best-effort, NOT a hard deadline

  for (;;) {
    uint32_t now = uBit.systemTime();
    Subsystems::DrivetrainPorts p = drivetrain.ports();     // bound pair, from config

    // === MANDATORY: control. Reads the committed snapshot bb (x[k]); consumes
    //     commands routed during the previous slack; each writes its OWN cell. ===
    hardware.tick(now, bb.motorIn, bb.motorResetIn);        // apply staged cmds + ZERO; sample encoders
    drivetrain.tick(now, bb.motor[p.left], bb.motor[p.right], bb.driveIn);
    poseEstimator.tick(now, bb.motor[p.left], bb.motor[p.right],
                       bb.otos, bb.otosValid, bb.poseResetIn);
    planner.tick(now, bb.motor[p.left], bb.motor[p.right], bb.fusedPose, bb.driveIn);

    // === COMMIT (clock edge): copy each subsystem cell into bb → x[k+1]. ===
    for (uint32_t i = 0; i < Rt::kPortCount; ++i) bb.motor[i] = hardware.motor(i).state();
    bb.drivetrain  = drivetrain.state();
    bb.encoderPose = poseEstimator.encoderPose();
    bb.fusedPose   = poseEstimator.fusedPose();
    bb.planner     = planner.state();
    hardware.sampleOdometer(&bb.otos, &bb.otosValid);       // false/no-op on NezhaHardware
    routeOutputs(bb, drivetrain, planner);                  // emitters' output cmds → next input queue
    telemetry.tick(now, bb);                                // reads x[k+1]; emits if due

    // === SLACK: yield, then ingest → route → apply config, until the next period.
    //     The uBit.sleep() is REQUIRED, not pacing garnish: the radio's RX is
    //     delivered by a CODAL MessageBus event listener (Radio::onData) that only
    //     runs when the main loop yields a fiber slice. Busy-waiting here starves
    //     radio while serial (IRQ ring buffer) keeps working — a silent, radio-only
    //     failure. Routing still wins (Decision 8). ===
    uint32_t deadline = now + kPeriod;
    do {
      uBit.sleep(1);                       // YIELD to the scheduler: radio delivery + other fibers run
      comm.tick(uBit.systemTime());
      if (comm.hasStatement())            router.route(comm.takeStatement(), bb);  // → bb queues
      else if (configurator.pending(bb))  configurator.applyOne(bb);   // fold a delta, configure(), publish
    } while (uBit.systemTime() < deadline);   // sleep(1) also guarantees ≥1 yield per pass under load
  }
}
```

Notes on the loop:
- **Nobody reads a subsystem directly.** Control ticks read `bb` (the frozen `x[k]`);
  the commit block is the only place `subsystem.state()` is read, and it writes into
  `bb`. Readers (telemetry, and the slack's router/GET handlers) read `bb`.
- **One-tick everywhere, deterministic.** A command routed in this pass's slack is
  consumed by next pass's mandatory ticks; a sensor sampled this pass is read by
  control next pass. Tick order within the mandatory block does not affect results.
- **`routeOutputs`** drains each emitter's output edge (`hasCommand()`/`takeCommand()`)
  into the next consumer's input queue — e.g. Drivetrain's addressed HAL command into
  `bb.motorIn[p.left/right]`.
- **The Configurator holds the subsystem refs** (constructed above) — the single,
  deliberate exception, because configuring is its job. The loop-applies variant would
  move those calls up here instead.
- **The `uBit.sleep(1)` is load-bearing, not pacing.** CODAL is a cooperative fiber
  scheduler; the radio delivers received packets through a `MessageBus` event listener
  (`Radio::onData`, `source/com/radio.cpp`) that only runs when the loop yields a fiber
  slice. A busy-wait starves radio — and *only* radio: serial RX is IRQ→ring-buffer
  (`serial_port.h` never sleeps), so it keeps working and hides the bug. So the hardware
  bench gate must verify **radio** comms specifically, not just serial, or a regression
  here ships silently.

## Target topology (the new state)

**The blackboard** — one struct, owned by the loop, transport only (no behavior, no
subsystem pointers):

*State objects* (double-buffered; committed at the clock edge):
- `msg::MotorState motor[kPortCount]` — written by Hardware; read by drivetrain, pose,
  planner, telemetry.
- `msg::DrivetrainState drivetrain` — written by Drivetrain; read by telemetry.
- `msg::PoseEstimate fusedPose` (+ `encoderPose`) — written by PoseEstimator; read by
  planner, telemetry.
- `msg::PlannerState planner` — written by Planner; read by telemetry.
- current configs `msg::DrivetrainConfig`, `msg::MotorConfig[kPortCount]`,
  `msg::PlannerConfig`, `msg::OdometerConfig` — written by the Configurator on apply;
  read by `GET`/telemetry. **These replace every shadow.**

*Queues* (command plane):
- `WorkQueue<Statement> statementsIn` — written by Communicator; popped by the router
  (replaces the single-latch seam).
- `Mailbox<msg::DrivetrainCommand> driveIn`, per-port `Mailbox<Hal::…HardwareCommand>`
  — absolute setpoints, latest-wins.
- `WorkQueue<ConfigDelta> configIn` — one queue to the Configurator, target-tagged.
- `WorkQueue<PoseResetCommand> poseResetIn`, per-motor encoder-reset flags,
  odometer-reset — SI/ZERO fan-out.

**What each command family becomes:**

| Family | today | becomes |
|---|---|---|
| Dev | 2 ptrs + 2 shadows + 2 outboxes | reads state objects for STATE/CAPS; posts to `driveIn` + `configIn`; **no ptrs, no shadows** |
| Telemetry | 4 ptrs | reads state objects only; **no ptrs** |
| Motion | poseEstimator ptr + outbox + owns `sTimeout` | posts to `driveIn`; reads `fusedPose`; `sTimeout` becomes Planner config |
| Config | 4 ptrs + cross-family `sTimeout` + 3 shadows | reads current-config state objects; posts to `configIn`; **no ptrs, no shadows, no reach-through** |
| Pose | 3 ptrs | fans out reset commands to `poseResetIn` + encoder-reset; reads binding from snapshot |
| Otos | 1 ptr + shadow | posts odometer-config deltas to `configIn`; reads otos state object |

New components: the **Blackboard**, the **Configurator**, the queue types
(`Mailbox`/`WorkQueue`), and the cyclic-executive loop. The seventh holder, `DevLoop`,
collapses into the loop body.

## What this deletes

- Subsystem pointers in all `*State` structs (handlers hold data handles, not
  subsystems).
- `motorConfigShadow[]` / `drivetrainConfigShadow` / `plannerShadow` / `configShadow`
  — `current` is now a readable state object owned by the Configurator.
- The cross-family `sTimeoutWatchdog` pointer.
- The command families' `#include`s of the subsystem layer (they depend on `msg::*` and
  the blackboard/queue types only).
- The seven-way duplication of the subsystem handles (subsystems live in one place: the
  loop).

Command handlers become pure translators: **read the statement queue → publish typed
commands onto the right queues → done.**

## Current-state inventory (grounded against `source/`, 2026-07-06)

**Six** command-family `*State` structs, each handed to a command-table factory (the
factory stores `&state` as every descriptor's `handlerCtx`; handlers recover it by
`static_cast`). No others exist — a tree-wide `struct *State` grep returns only these
six plus POD `msg::*State` snapshot types:

| Struct | file:line | (a) subsystem ptrs | (b) cross-family | (c) shadows | (d) outbox |
|---|---|---|---|---|---|
| `DevLoopState` | dev_commands.h:198 | hardware, drivetrain, watchdog | — | `motorConfigShadow[]`, `drivetrainConfigShadow` | `hasHardwareCommand`+cmd, `hasDrivetrainCommand`+cmd |
| `TelemetryState` | telemetry_commands.h:77 | hardware, drivetrain, poseEstimator, planner | — | — | — |
| `MotionLoopState` | motion_commands.h:95 | poseEstimator | — (owns `sTimeout` value) | — | `hasCommand`+command |
| `ConfigCommandState` | config_commands.h:99 | hardware, drivetrain, poseEstimator, planner | **`sTimeoutWatchdog` → `motionState.sTimeout`** | `drivetrainShadow`, `motorShadow[]`, `plannerShadow` | — |
| `PoseCommandState` | pose_commands.h:67 | hardware, drivetrain, poseEstimator | — | — | — |
| `OtosCommandState` | otos_commands.h:45 | hardware | — | `configShadow` (OdometerConfig) | — |

Plus a **seventh** pointer-holder: `DevLoop` (dev_loop.h:63), the per-pass loop-wiring
struct handed to `devLoopTick()` — it re-duplicates
hardware/drivetrain/poseEstimator/planner/watchdog *and* points back into three command
states (telemetry, devState, motionState) + the `CommandProcessor`.

**Two wiring sites, changing identically.** `source/main.cpp` wires inline as
function-statics (structs :180–260; `.sTimeoutWatchdog = &motionState.sTimeout` at :236;
`DevLoop` at :285–296). `tests/_infra/sim/sim_api.cpp` mirrors the pointer topology
**1:1** via `buildAndWireCommandTable()` (same structs, same pointers, same
`sTimeoutWatchdog → motionState.sTimeout` at :206, same shadow-seeding, same `DevLoop`).
It differs only in boot-config source, the Hardware leaf (`NezhaHardware` vs
`SimHardware`), packaging, and reply routing.

**The subsystem tier already conforms on the input axis.** Every subsystem holds **no**
cross-subsystem pointer; all cross-subsystem data flows in as explicit `tick()`
arguments — `Drivetrain::tick(now, leftObs, rightObs)` (drivetrain.h:118),
`PoseEstimator::tick(now, leftObs, rightObs, otosObs*)` (pose_estimator.h:82),
`Planner::tick(now, leftObs, rightObs, fusedPose)` (planner.h:136). `dev_loop.cpp` is
the single wiring layer that samples one subsystem's output and feeds the next — the
pattern the command tier failed to follow. (Note: today it does this *same-pass*, not
via a committed snapshot; synchronous update changes that.)

**The faceplate is duck-typed, not uniform.** `Drivetrain` has
apply/state/capabilities/hasCommand/takeCommand; `PoseEstimator` has none of
apply/state/capabilities/hasCommand (pure observer, outputs via
`encoderPose()`/`fusedPose()`); `Planner` adds hasEvent/takeEvent, no capabilities;
`Hardware` exposes no config/state at its own faceplate (per-port on `Hal::Motor`);
`Communicator` has no `apply()` (a statement source, output via
`hasStatement()/takeStatement()`).

**No blackboard / topic / pub-sub / mailbox / registry exists** anywhere in `source/`
(tree-wide grep: only descriptive comments and an unrelated I2C-test FIFO). The only
message passing is the concrete point-to-point held/taken edge pairs
(`hasCommand/takeCommand`, `hasEvent/takeEvent`, `hasStatement/takeStatement`) and the
`bool has* + msg::*` outbox fields — every one hand-wired. Greenfield for the
abstraction.

## Open questions

- **Control period (nice-to-have, not a gate).** 50 Hz / 20 ms is a target; if the
  I2C-bound mandatory portion can't hit it, run slower — the design doesn't depend on
  the rate. Worth measuring for tuning, but it does not block execution.
- **Re-tuning for the added latency.** Synchronous update injects a uniform `z⁻¹` per
  hop vs. today's same-pass control path; verify/adjust control tuning.
- **Config-current publish timing.** Bulk-committed next pass (clean one-tick GET) vs.
  the Configurator publishing immediately on apply (same-slack GET). Pick one.
- **Cross-`SET` validation base.** Validate a delta against `current` snapshot vs.
  `current + pending deltas` (a jointly-invalid pair of individually-valid `SET`s).
- **State-object home.** Blackboard-owned vs. subsystem-member write cell (functionally
  a wash; the blackboard at least holds/commits them).
- **Threading of ingestion.** A producer thread pushing statements is possible on the
  nRF52833/CODAL but needs an SPSC-safe queue and fights the current "comms stays
  in-loop" decision (081 Decision 3). The in-loop `WorkQueue<Statement>` already gets
  the backpressure without concurrency; treat a thread as a separate, later step.
- **Migration — decided: greenfield by deletion.** Delete the whole `main` loop and
  `dev_loop.*`, then rebuild the subsystems and the loop from scratch, adding new files
  (blackboard, queue primitives, Configurator, router). The subsystem tier already
  conforms on the input axis; the work is the command tier, the blackboard, the
  Configurator, the loop, and regularizing the output faceplate.
