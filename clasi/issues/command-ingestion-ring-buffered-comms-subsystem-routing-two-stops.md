---
status: pending
---

# Command ingestion: ring-buffered Comms, subsystem routing, two stops

## Description

The firmware ingests **at most one command per control cycle** (~21/s at the
measured 47 ms loop). `App::Comms::pump()` reads one line — serial first,
radio only if serial had nothing — into a cycle-local `Cmd`, and
`RobotLoop::processMessage()` dispatches that one. There is no queue anywhere
in `Comms`.

Three problems follow:

1. **Commands are silently lost under burst.** The radio's completed-message
   handoff is a single slot (`src/firm/com/radio.h:104-106`): a message that
   completes while the previous one is unread is discarded
   (`radio.cpp:67`). The serial side is a 256-byte linear accumulator that
   silently truncates an overlong line (`src/firm/com/serial_port.cpp:38-39`)
   — the truncated remainder is delivered as if whole and fails CRC
   downstream. Separately, ~20% of inbound command packets are dropped by the
   DAPLink USB→UART bridge before they ever reach the nRF (measured 13/60 and
   12/60 in controlled probes, with a still-zero malformed count) — see
   the related command-loss hunt.
2. **Wheel commands have no queue and last writer wins.** Each `move_wheels`
   overwrites targets, deadline and move-id; the superseded command never
   completes and never acks, so a host chaining on completion acks hangs.
3. **There is only one kind of stop.** `STOP` means "halt everything now".
   There is no way to say "come to a stop when you reach this point in the
   queued sequence".

Intended architecture (stakeholder-specified, 2026-07-27):

```
serial / radio  ->  Comms ring buffer (pumped often, several times per cycle)
                        |
                   processMessages()  -- once per cycle, drains the ring
                        |
      +-----------------+---------------------+------------------+
      |                 |                     |                  |
   WHEELS            MOVE / STOP            ESTOP             CONFIG
      |                 |                     |                  |
   App::Drive      Motion::Planner      Drive + Planner    App::Configurator
  (dumb, timed)     (motion queue)     (zero + clear)      (patch appliers)
```

## Cause

The one-command-per-cycle shape is a consequence of `pump()` being written to
produce a single `Cmd` for a single dispatch site, with no buffer between
ingestion and handling. Ingestion is therefore rate-limited by the control
loop, and both transports' own buffers (a single slot on radio, a linear
accumulator on serial) are the only backpressure — both of which drop or
corrupt rather than queue.

Wheel commands additionally have no lifecycle owner: `RobotLoop` holds the
targets, deadline and move-id as loop members and overwrites them in place on
each arriving command.

## Proposed fix

Work lands in the `pid-removal` branch (worktree
`/Volumes/Proj/proj/RobotProjects/radio-robot-elite-pidfree`). Start from the
`wheel-layer-v1` tag — the files modified after it are a half-migrated
intermediate that builds but leaves the planner→wheels hand-off broken.

### 1. Ring buffer in `App::Comms`

`src/firm/app/comms.{h,cpp}`, mirroring the in-tree ring idiom at
`src/firm/app/telemetry.h:362-364` (`ring_[]` + `head_` + `count_`,
oldest-evicted-first):

- `Cmd cmdRing_[kCmdRingDepth]`, depth 12. `sizeof(Cmd)` is dominated by
  `msg::CommandEnvelope`; confirm the RAM cost and reduce the depth if it
  bites.
- `void pump(uint32_t now)` — drains **both** transports, looping until no
  transport yields a line or the ring is full; no longer takes a `Cmd&`.
  Malformed lines still bump `malformedCount_`; a push onto a full ring
  increments a new `commandsDroppedCount_`, published in health telemetry
  beside `commsMalformedCount`.
- `bool takeCommand(Cmd& out)` — pops the oldest.

`RobotLoop::cycle()` calls `pump()` inside **each existing `runAndWait`
body** (L-settle, kClear, R-settle, pace) so the transports are freed
several times per cycle. Do **not** add new `runAndWait` blocks —
`src/tests/sim/system/sim_api_harness.cpp:363` asserts exactly four
`sleepMillis()` calls per cycle. Once per cycle, where dispatch lives today:
`while (comms_.takeCommand(cmd)) routeCommand(cmd);`

Draining N commands in one cycle pushes N acks into the depth-4 ack ring in a
single frame, so a 5-command burst would lose one
(`src/host/robot_radio/robot/protocol.py:1489-1493` documents the margin).
Raise `kAckRingDepth` to match `kCmdRingDepth`.

### 2. Two new wire verbs

Follow `src/scripts/gen_messages.py`'s registry flow: add to
`src/protos/commands.proto` with `[(binary) = true]`, add a matching oneof
arm in `src/protos/envelope.proto` at a fresh, never-reserved field number.
The arm name **must** be the lowercase verb — `serial_conn.py:310` derives
the wire prefix, and therefore the CRC scope, from
`WhichOneof("cmd").upper()`. Regeneration is automatic in `build.py`. Then
add the `CmdKind` case, update `_EXPECTED_BINARY` in
`src/tests/unit/test_command_registry.py:54`, and update the verb table in
`docs/protocol-v5.md`. No `comms.{h,cpp}` edits are needed —
`maxVerbNameLength()` recomputes the prefix budget at compile time.

- **`WHEELS { v_left, v_right, duration }`** → `App::Drive`: the dumb teleop
  primitive, always time-bounded so a dead host cannot mean a runaway.
- **`ESTOP {}`** → immediate stop in both subsystems.

`STOP` keeps its verb and **changes meaning to a planned stop** entering the
planner's queue. Callers meaning "halt now" move to `ESTOP` (bench scripts,
the sim's `injectStop`).

### 3. Routing (`RobotLoop::routeCommand`)

| Command | Destination |
|---|---|
| `WHEELS` | `drive_.command(vL, vR, duration, id, now)` — supersedes the planner |
| `MOVE` (twist or wheels velocity, any stop kind) | `planner_.move(...)` |
| `STOP` | `planner_.plannedStop(id)` — queued, executes in sequence |
| `ESTOP` | `drive_.estop()` **and** `planner_.estop()` |
| `CONFIG` | `configurator_.apply(...)` |

Exactly one subsystem owns motion at a time, enforced at routing: a `WHEELS`
command clears the planner; a `MOVE` clears Drive's armed command. Routing
all `MOVE`s to the planner also fixes a live gap — wheels moves carrying a
DISTANCE stop (what the TestGUI's `D <l> <r> <mm>` path sends) currently run
to the timeout backstop instead of stopping on odometry.

### 4. `App::Drive` owns the wheel-command lifecycle

`src/firm/app/drive.{h,cpp}`, adopting the planner's two-method contract
(`tick` does work, `update` saves to the blackboard):

- `command(vLeft, vRight, durationMs, moveId, now)` — arm a bounded command.
- `estop()` — zero targets, disarm, no completion ack.
- `update(Types::RobotState& state, uint32_t now)` — expire at the deadline,
  latching a completion event, and publish targets into
  `state.wheelLeft/Right.cmdVelocity` while Drive owns motion.
- `tick(float speedLeft, float speedRight)` — speed→duty via the per-wheel
  calibration (L 1/560, R 1/510 from the measured plant), crawl shaping,
  leaf writes; keeps quiet-at-zero, `setCrawlPulse`, `setDutyPerSpeed`.
- `bool takeCompletion(uint32_t* moveId)` — one-shot event the loop acks.

Loop call sites are single lines:
`drive_.tick(state_.wheelLeft.cmdVelocity, state_.wheelRight.cmdVelocity);`
and `drive_.update(state_, state_.time.cycleStart);` in the pace block.

### 5. `Motion::Planner`

- `plannedStop(uint32_t moveId)` — enqueue a stop as a queue entry via an
  explicit `Move::Kind::Stop` (the existing kinds are stop *conditions*, so a
  bool would be muddier). On activation it drains to zero and completes at
  rest, acking `moveId`.
- `estop()` — clear active and pending immediately, command zero, no
  completion acks for discarded entries. Replaces the existing `stop()`.

### 6. Move the wheel-layer constants into robot config

Two per-robot measured properties are currently hard-coded rather than
configured, and must move into the robot JSON (`data/robots/*.json`,
`control.*` block) alongside `output_deadband` / `reversal_dwell_ms`, flowing
through `src/scripts/gen_boot_config.py` -> boot config -> the config path:

- **`drive.setCrawlPulse()`** — a bare call in `src/firm/main.cpp`.
- **Per-wheel `dutyPerSpeed`** (L 1/560, R 1/510) — hard-coded as member
  initializers in `drive.h`. This is one robot's gearboxes on one battery,
  measured on one evening, compiled into the class definition: every other
  robot silently inherits it, and changing it means editing C++ and
  reflashing. The `kff` wire key is not an escape hatch — it sets both wheels
  to one value, so a single config push flattens the ~10% L/R asymmetry with
  no way to restore it short of a rebuild.

  Requirement: `App::Drive` carries **no calibration defaults at all**. The
  values come from the robot JSON via boot config and are handed in at
  construction; an unconfigured Drive refuses to drive rather than quietly
  using another robot's numbers (the same fail-closed posture
  `RobotLoop`'s `configured_` gate already takes for motion commands).

**The committed crawl value is also wrong.** `0.20` was sized against the
duty sweep's 0.10-0.19 "dead zone", which the standalone `duty_min` prober
(`src/tests/firmware/duty_min/RESULTS.md`) later showed to be an artifact of
that sweep's criterion (all three cold-start 500 ms repeats must move). True
breakaway is **1-6% duty, state-dependent** — consistent with a vendor-blocks
rig driving the same motors at 1%. At 0.20 the crawl/continuous boundary sits
at ~107 mm/s commanded, so every speed below that runs pulsed, adding ripple
across a wide band for a stiction problem that largely does not exist there.

Action: default crawl **off**, re-measure the low end of
`src/tests/bench/speed_sweep.py` with it disabled, and enable it only if slow
speeds genuinely stall — sized from the prober data (~0.05), set through
config.

### 7. `App::Configurator`

New `src/firm/app/configurator.{h,cpp}` — named to avoid colliding with the
existing `Config::` namespace (`src/firm/config/persisted_tuning.h`). Owns
what `RobotLoop::handleConfig` does today: present-field patch merges, the
persisted-tuning snapshot and its change-detection write policy, and pushing
values into the owning subsystems (motor/calibration → Drive, estimator and
shaper → Planner, OTOS → the sensor leaf). `reapplyPersistedTuning()` moves
with it; `RobotLoop` keeps only `configurator_.apply(env)` plus the ack.

### 8. Host and bench

`src/host/robot_radio/robot/protocol.py` gains `wheels(v_left, v_right,
duration)` and `estop()`; `move_wheels()` stays (now a planned wheels move
through the planner). Bench `proto.stop()` calls meaning "halt now" become
`estop()`.

**TestGUI is explicitly out of scope**: leave `src/host/robot_radio/testgui/`
and `src/tests/testgui/` untouched.

## Verification

The square tour is the **only** gate on this change (stakeholder direction).
One script, two backends — `NezhaProtocol` already drives `SimLoop` through
`SimConfigConn` (`src/host/robot_radio/io/sim_config.py`) exactly as it
drives a `SerialConnection`, so the tour logic is written once:

`src/tests/bench/square_tour.py`, evolved from the tagged, working
`wheels_square_tour.py`:

- `--sim` → `SimLoop` + `SimConfigConn`, ground truth from
  `SimLoop.get_true_pose()`.
- `--port <dev>` → `SerialConnection`, ground truth from encoder odometry.
- Same 8 segments driven by the new `WHEELS` verb, same per-run
  self-calibration prelude, same two-panel chart and printed
  path/heading/closure summary. Exits nonzero if the tour does not complete
  or closure exceeds a stated bound.

Pass criteria: sim closes the square; hardware reproduces the `wheel-layer-v1`
result (359.9°/360, ~19 mm closure). The motion path itself is unchanged by
this work, so a regression there means the new command path broke something.

Also expected green: `cmake --build src/sim/build` and `just build` clean;
`src/tests/unit/test_command_registry.py` (add the two verbs); the planner's
own `ctest` suite (not collected by pytest — `pyproject.toml:171`).

### Deferred: the rest of the test suite

The one-command-per-cycle contract is baked into many harnesses. Per
stakeholder direction these are **not** to be re-derived as part of this
change. Any test that breaks and is not the square tour gets quarantined —
`@pytest.mark.skip` on the pytest wrapper carrying the literal marker
`DEPRECATED-COMMAND-INGEST` and a one-line note; for a C++ harness scenario
whose premise is gone, `#if 0` with the same marker. That marker is the grep
handle for a later big-bang test pass, which is explicitly deferred to its
own piece of work.

Known candidates: `move_protocol_harness.cpp` (A/B equality at :898-939 bakes
in "CONFIG lands one cycle after MOVE"), `app_comms_harness.cpp:391`
(`scenarioPumpBoundedToOneTransportPerCall`, the canonical statement of the
replaced contract), `app_move_queue_harness.cpp` and
`motion_move_queue_chained_harness.cpp` (legacy `Motion::MoveQueue`),
`app_robot_loop_harness.cpp` (already stale and `xfail`ed at
`test_app_robot_loop.py:121`), and anything under `src/tests/testgui/`.

The contract text itself is stated in three places that need updating when
the ring lands: `src/firm/app/comms.h:220-221`,
`src/tests/sim/support/fake_transport.h:33-34`, `src/sim/sim_harness.h:186-188`.

## Related

- `wheel-layer-v1` tag (branch `pid-removal`): the known-green, hardware-
  verified starting point — bare wheel layer, measured plant constants,
  square tour at 359.9°/360 with 19 mm closure.
- The DAPLink inbound command-loss hunt (~20% of command packets dropped
  whole, zero malformed count) — independent of this change but compounding
  the same symptom; bench scripts currently retry until acked.
- Occasional wild telemetry velocity sample (±27,000 mm/s single readings)
  seen twice during characterization sweeps.
