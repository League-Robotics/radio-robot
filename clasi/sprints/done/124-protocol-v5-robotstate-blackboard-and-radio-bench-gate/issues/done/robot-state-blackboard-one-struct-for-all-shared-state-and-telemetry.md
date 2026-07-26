---
status: done
sprint: '124'
tickets:
- 124-007
- 124-009
---

# RobotState blackboard — one struct for all shared state, telemetry as its projection

## The idea (stakeholder discussion, 2026-07-25)

Introduce a single `RobotState` struct — mostly data, few methods — that is
the ONE sanctioned way subsystems share data with each other and the source
from which the telemetry frame is built. Subsystems keep private state for
anything they don't have to share; the moment something must cross a
subsystem boundary or reach the host, it goes through `RobotState`.

Three roles, one object:

1. **The blackboard** — every value one subsystem needs from another lives
   here. Kills the huge many-parameter method signatures: tick/update
   functions take `RobotState&` instead.
2. **The telemetry source** — `Telemetry::emit(state, now)` selects the
   host-relevant subset of the state and serializes it. `Telemetry::Frame`,
   `setFrame()` staging, and `Motion::StateEstimator::Input` (and cycle()'s
   field-by-field copy-out into it) all disappear.
3. **The test fixture** — a subsystem test is: construct a `RobotState`,
   fill the fields the module reads, call `tick(state, now)` /
   `update(state)`, assert on the fields it writes. Trivially copyable
   (plain fields, no pointers, no heap), so tests copy it for golden
   comparisons. No fakes for other devices unless testing a leaf itself.

## Settled design decisions

### Placement: `src/firm/types/`, plain header, no `msg::` types

`RobotState` is NOT a motion type — the robot must function without the
motion library, and the two must stay independent. It lives in
`src/firm/types/` as a dependency-free header (cstdint-level includes
only, no `msg::`/`messages/` types). When the motion library needs it,
motion includes that directory too. This amends the 122-002 boundary rule:
`firm/types` becomes the shared floor both trees may stand on (alongside
`Motion::WheelSink`, which remains the actuation interface; `RobotState`
is the data interface). The single place `msg::Telemetry` is touched is
the serialize path in `app/telemetry`.

### Subsystems take RobotState; devices do not

Leaf drivers (`Devices::Motor`, `Devices::Otos`, line/color) keep their
own generic APIs and stay ignorant of the state schema. Subsystems own
devices:

- **Drive** takes ownership of both motors again (today it owns nothing).
  The vendor bus choreography (request L / settle / collect L / clear /
  request R / settle / collect R) stays visible in the loop but moves
  behind NAMED phase methods — `requestLeft()`, `collectLeft(nowUs)`,
  `requestRight()`, `collectRight(nowUs)` — named for what they do, so
  ordering is self-evident (not tick1/tick2/tick3).
- **Sensors** (new subsystem) owns the OTOS + line + color leaves and runs
  the line/color alternation cursor internally.
- **RobotLoop owns no devices at all** — only subsystems and the schedule.

### Publish rule: as soon as coherent, exactly once per cycle

Not "all updates at the end," and not ad-hoc scattering either. The
invariant: each subsystem publishes its state section at the earliest
point in the cycle where that section is complete and internally
coherent, exactly once per cycle. Three properties make it deliberate:

1. one writer per section,
2. one publish point per section per cycle,
3. the publish points are readable straight off the cycle() body — the
   loop IS the publish schedule.

Applied:

- **Wheels** publish immediately after BOTH collects (`drive_.update()`
  right after `collectRight()`) — never after just L. Coherence for the
  wheel section means same-generation L/R samples (the 119-005
  straight-leg-crab pairing-skew lesson).
- **Command dispatch** (`processMessage`) writes `state.command` mid-cycle
  from the R-settle window, as today — a command is genuinely new external
  information; delaying it buys nothing. Drive already ticked, so
  actuation picks it up next cycle (one-cycle floor, unchanged), but
  MoveQueue's decision at the END of THIS cycle already sees it.
- **Sensors, odometry, estimator** publish in the pace block in dependency
  order: sensors → odom (consumes this-cycle wheels) → estimator
  (consumes both).
- **Telemetry** is constructed from the state ONCE, immediately after
  those updates — one frame, one assembly point (preserves 123-007's
  intent; the disease it cured was two parallel objects with
  field-by-field re-copies, which no longer exist).
- **MoveQueue** updates after emit, reading THIS cycle's estimate (the
  118-002 stop-decision requirement — data-to-decision is one cycle,
  never two), and its completion ack still rides the NEXT frame
  (protocol-v4 §7.2).

Target cycle body (sparse — this is the whole thing):

```cpp
void cycle() {
  state_.time = stamp();
  drive_.tick(state_);                              // command → wheel targets (pure)
  drive_.requestLeft();
  runAndWait(kSettle, [&]{ comms_.pump(cmd, ...); });
  drive_.collectLeft(nowUs);
  runAndWait(kClear,  [&]{});
  drive_.requestRight();
  runAndWait(kSettle, [&]{ processMessage(cmd); }); // → state_.command (deliberate mid-cycle write)
  drive_.collectRight(nowUs);
  drive_.update(state_);                            // both wheels, same generation
  runAndWait(kPace, [&]{
    sensors_.update(state_);                        // otos + line/color alternation
    odom_.update(state_);                           // wheels → pose
    estimator_.update(state_);                      // → estimate
    tlm_.emit(state_, now);                         // ONE frame, ONE assembly point
  });
  moveQueue_.update(state_, now);                   // this-cycle estimate → command
}
```

Outside their publish point, subsystem ticks read a stable state — no
other mid-cycle mutation without a stated reason at the call site.

### Struct sketch (sections grouped by writer)

```cpp
struct RobotState {
  struct Time  { uint32_t cycleStart; uint32_t cycleBusy, cyclePeriod; } time;  // writer: loop
  struct Wheel {                                     // sensed: Drive::update; cmdVelocity: Drive::tick
    float position, velocity; uint32_t sampleTime; bool connected;
    float cmdVelocity;
  } wheelLeft, wheelRight;
  struct Otos  {                                     // writer: Sensors::update
    bool present, connected;
    float x, y, heading, vx, vy, omega; uint32_t time;
  } otos;
  struct Perception { uint32_t line, color; bool lineFresh, colorFresh; } perception;  // Sensors::update
  struct Pose  { float x, y, heading, vx, vy, omega; } pose;      // writer: Odometry::update
  struct Estimate {                                  // writer: StateEstimator::update — ZOH BASES
    /* body + per-wheel bases (value, velocity, basisTime, valid) + innovations */
  } estimate;
  struct Command { /* mode, targetVx, targetOmega, moveActive */ } command;  // dispatch + MoveQueue
  struct Health  { /* i2cSafetyNetCount, commsMalformedCount, wedgeLatch,
                      deadmanExpired, shapingDisabled */ } health;  // owning modules
};
```

Field lists are indicative, to be finalized during design. Two notes:

- **Estimate carries ZOH bases, not just snapshots** — value + velocity +
  basisTime, so "predict to time t" becomes a pure free function over the
  state. A consumer holding a COPIED `RobotState` gets extrapolation for
  free; the estimator's query interface dissolves into data.
- **The telemetry flags word is computed at encode time** in telemetry.cpp
  from state fields (`otos.present` → bit 0, `health.wedgeLatch` → bit 7,
  …). The `setFlag()` choreography sprinkled through the loop goes away.
  The only telemetry-internal bit left is ack_fresh; acks stay OUT of
  `RobotState` entirely (protocol bookkeeping, not robot state).

### Telemetry is a lean projection — and TelemetrySecondary dies

`state ⊇ wire`: there is NO wire-visibility invariant. Telemetry exists
for the host; a state field the host doesn't need is simply not sent.
Intent: ONE lean primary frame running as fast as possible, so:

- **Delete `TelemetrySecondary` outright** — frame type, wire schema arm,
  and telemetry.cpp's tie-break/alternation machinery. Survivors the host
  actually needs (cmd velocities, probably; glitch counts, maybe) fold
  into the primary during the field pruning; the rest stops being sent.
- Prune the primary frame's field list to what the host actually uses.
- This is a wire-schema change host tools will feel — coordinate with the
  pending protocol-v5 issue
  ([protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md](protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md))
  so the schema cleanup rides together.

### What stays OUT of RobotState

Config, PID gains, calibration, fusion weights, persisted tuning, acks —
anything slow-changing. `RobotState` is per-cycle dynamics only. Config
keeps its own patch/persistence path.

## Why this matters

- Kills the dual `frame_` / `StateEstimator::Input` parallel-struct copy
  path and its whole class of drift/staleness bugs — one object, one
  source of truth for "what the robot currently is."
- Method signatures collapse to `f(RobotState&, now)`.
- Testing a subsystem needs only the struct — no mocking every other
  device; copy the object for golden comparisons.
- Recorded telemetry becomes a stream of (partial) `RobotState`s the bench
  can replay through the estimator or future motion planner offline — a
  direct enabler for
  [motion-library-development-kickoff-parallel-effort.md](motion-library-development-kickoff-parallel-effort.md).
- The cycle body becomes sparse and self-documenting: the loop is the
  publish schedule.

## Supersedes / relates

- **Supersedes**
  [telemetry-frame-is-the-robot-state.md](telemetry-frame-is-the-robot-state.md) —
  this issue resolves that issue's open design tension (the 123-007
  "single assembly block" guidance): progressive fill of the STATE is
  fine under the publish-when-coherent rule; the TELEMETRY is still
  assembled once, in one place, right before emit. That was the point.
- Relates to sprint 123 ticket 007 (single-point frame assembly) — this
  builds directly on it.
- Relates to
  [protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md](protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md)
  (wire-schema changes ride together) and
  [motion-library-development-kickoff-parallel-effort.md](motion-library-development-kickoff-parallel-effort.md)
  (planner/PIDs consume `RobotState`; motion includes `src/firm/types/`).

## Scope / touch points

- `src/firm/types/` — new `robot_state.h` (dependency-free).
- `src/firm/app/robot_loop.{h,cpp}` — cycle() restructure to the sketch
  above; RobotLoop drops all device references.
- `src/firm/app/drive.{h,cpp}` — owns both motors; named bus-phase
  methods; tick/update split.
- New Sensors subsystem — owns otos/line/color; alternation cursor moves in.
- `src/firm/app/telemetry.{h,cpp}` — `Frame`/`SecondaryFrame`/`setFrame`/
  `setFlag` removed; `emit(const RobotState&, now)`; flags derived at
  encode; secondary path deleted.
- `src/motion/state_estimator.{h,cpp}` — `Input` replaced by `RobotState`
  (motion now includes `src/firm/types/`); query methods become free
  functions over the state's ZOH bases.
- `src/protos/telemetry.proto` — secondary removed, primary pruned
  (with protocol-v5).
- Host tools / sim harness — follow the wire and construction changes.
