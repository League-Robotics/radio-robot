---
status: pending
---

# Tick Model, Command Flow, and the Command Board — design sketch

## Context

Stakeholder design discussion (2026-07-04, two sessions), building on pending issues [i2c-bus-lazy-clearance-timers](i2c-bus-lazy-clearance-timers.md) and [armor-motor-write-path-against-reversal-latch](armor-motor-write-path-against-reversal-latch.md), and on the now-**implemented** [turn-the-communicator-into-a-faceplate-subsystem](done/turn-the-communicator-into-a-faceplate-subsystem.md) (commit 2599df3 — this design revises its command-out shape; see "Deltas against the current tree").

The trigger: `NezhaMotor::tick()` blocks 4 ms per motor in `readEncoderSettle()` waiting out the 0x46 settle window (~8 ms of every tick spun with two motors). The encoder read is two halves — request, wait 4 ms, collect — a **flip-flop**: on any given tick we do one or the other, never both. That generalized into a whole-system model.

## The model

**The three-beat template: feed it, tick it, ask it.** Every entry in the main loop is some subset of:

1. **feed** — hand the thing its inputs: `apply(command)` (staged, latest-wins for setpoints).
2. **tick** — give it its slice of time: `tick(now)` returns **void** everywhere; tick means "make all your decisions." Each thing decides internally whether it is actually its time; if not, it passes. No central scheduler, no common interface — the list in main IS the schedule, and a thing may appear twice (the HAL does).
3. **ask** — collect its output explicitly: `hasX()` / `takeX()`. Producers **hold** their output; `take` clears, `has` doesn't. An untaken output stays put — command-out is *held after tick and explicitly taken, never pushed* (contract wording change from "returned from tick").

Supporting principles:

- **Statements vs commands**: a wire line is a **statement** (verb, args, kv pairs, correlation id). Parsing a statement yields a **command** (`msg::*Command`, `<Producer>To<Consumer>Command`). Only the internal control messages are called commands.
- **Messages above, methods below**: messages/commands are the currency *between* subsystems — every inter-subsystem edge is a plain, loggable, injectable struct crossing a faceplate. *Within* a subsystem, plain method calls: the HAL consumes a HAL command at its faceplate and talks to its motors and bus in direct calls (`motor.setVelocity(…)`, `bus.write(…)`) with no internal message ceremony. This is the vertical extension of the existing contract rule "setters/getters are the primitives; message verbs are built on them."
- **The chalkboard dissolved into the pattern**: no CommandBoard struct. Each producer's held output IS its outbox slot; main is the visible mover of every command. Backpressure lives in the producer: a Communicator with an untaken statement declines to poll its transports.
- **The subsystem is the unit of test**: the lowest level worth testing with energy. Testing a subsystem may fake the hardware below it — an accepted, confined exception to the no-mocks preference. The natural seam already exists: `I2CBus`'s `HOST_BUILD` path, so HAL tests run the REAL NezhaMotor register/flip-flop/throttle/dwell logic against a scripted brick. Drivetrain and processor need no fakes at all (structs in/out); the Communicator's hardware coupling is covered by the bench gate.

## Part 1 — the brick flip-flop (I2C spacing)

**Why the flip-flop can't be purely per-motor**: all four ports share device 0x10 and a single readback register ([nezha_motor.h:104](../../source/hal/nezha/nezha_motor.h)) — two outstanding 0x46 requests would clobber each other. So:

- **Per-motor** (NezhaMotor): sample math, staged command (`mode_`/targets — already there), write-path not-ready states (40 ms throttle, future 100 ms reversal dwell). A motor knows *how* to do each half, never *when*.
- **Per-brick** (the HAL): `activePort_` + `phase_` (REQUEST_DUE / COLLECT_DUE) + the schedule timers. This finally makes nezha_hal.h's "orchestrates the split-phase bus schedule" comment true. The ported-but-unwired `requestEncoder()`/`collectEncoder()` ([nezha_motor.cpp:508-530](../../source/hal/nezha/nezha_motor.cpp)) get wired at last.

```cpp
// The HAL — the brick sequencer. One bus action (or a pass) per slice.
void NezhaHal::tick(uint32_t now) {   // [ms]
  switch (phase_) {
    case Phase::REQUEST_DUE:
      motorAt(activePort_).requestSample();   // 0x46 write, postClear 4000 // [us]
      phase_ = Phase::COLLECT_DUE;
      break;
    case Phase::COLLECT_DUE:
      if (!bus_.clear(kNezhaAddr)) break;     // settle window still open -- pass
      motorAt(activePort_).tick(now);         // collect + EMA + PID + duty write
      activePort_ = nextPortInUse(activePort_);
      phase_ = Phase::REQUEST_DUE;
      break;
  }
}
```

**Only ports in use are cycled** (decision 1). Activity rule, kept dumb: a port becomes in-use on the first command the HAL distributes to it, and stays in-use — no auto-deactivation. Idle ports get no encoder sampling (accepted cost: no wedge/connectivity monitoring on ports nobody commanded). With no ports in use the schedule idles.

**The HAL is ticked twice per pass — sanctioned, not a hack** (decision 6). Slice 1 at the top of the loop: due collects land, so observations are fresh before anyone computes. Slice 2 after the producers: requests/writes go out, so a target staged this pass actuates this pass. The HAL doesn't know it's called twice; it just gets two slices and its flip-flop does at most one bus action per slice. This replaces the old bound-pair double-tick hack ([main.cpp:231-235](../../source/main.cpp)) with an explicit, visible feature of the list.

Duty writes happen **at collect time only** — the lazy-timer issue's constraint 4 (postClear on 0x10 holds off *any* 0x10 transaction) makes that the only legal slot anyway. Builds on the lazy-clearance issue plus **one API addition**: a non-spinning peek `bool I2CBus::clear(uint16_t addr) const` so the scheduler can pass instead of spin; spin-the-remainder stays as the safety net.

**Cadence** (100 kHz bus; loop iterating ~0.2–1 ms; per-port slot ≈ 5.3–6.5 ms, only ~1.3–2.2 ms bus-blocked):

| Config | Per-motor sample period | CPU free | Today (blocking) |
|---|---|---|---|
| 2 ports in use | ~11–13 ms (~80–90 Hz) | ~75–80% | ~10 ms, ~8 ms spun |
| 4 ports in use | ~21–26 ms (~40–47 Hz) | ~75–80% | ~32 ms loop, ~100% blocked; bound pair sampled unevenly |

Underrated win: **comms get polled every ~1 ms instead of every ~32 ms** — statement latency and watchdog granularity improve ~30×.

PID `dt` is already measured — cadence change absorbed. But `vel_filt_alpha` was bench-tuned at the old cadence — **retune on the stand** (the alpha=0 episode shows this class fails silently).

## Part 2 — the example main loop

```cpp
while (true) {
    uint32_t now = uBit.systemTime();   // [ms]

    hal.tick(now);                                  // slice 1: due collects land

    // Communicator: tick it, then ask it. Untaken statement => it declines to poll.
    comm.tick(now);
    if (comm.hasStatement()) {
        processor.apply(comm.takeStatement());      // feed (copies line + returnPath)
        watchdog.feed(now);
    }

    // Processor: pure transformer. Parse happens in its tick; replies go out
    // the statement's return path; commands land in per-consumer outboxes.
    processor.tick(now);
    if (processor.hasHalCommand())        hal.apply(processor.takeHalCommand());
    if (processor.hasDrivetrainCommand()) drivetrain.apply(processor.takeDrivetrainCommand());

    // Drivetrain: fed above; tick it, then ask it. Binding queried, not duplicated.
    if (drivetrain.active()) {
        Subsystems::DrivetrainPorts p = drivetrain.ports();
        drivetrain.tick(now, hal.motor(p.left).state(), hal.motor(p.right).state());
        if (drivetrain.hasCommand())      hal.apply(drivetrain.takeCommand());
    }

    hal.tick(now);                                  // slice 2: requests/writes go out

    if (watchdog.check(now)) { neutralizeAll(...); /* EVT dev_watchdog */ }
}
```

Same-pass latency: a statement is fed, parsed, routed, the drivetrain ticks, and slice 2 actuates — all in one pass, each arrow its own visible line.

## Part 3 — the processor is a pure transformer

The processor's identity: **statements in, commands out** (plus replies). It holds no device write access — feeding devices from inside handlers hides the command handoff exactly the way tick-return-values did.

- **Per-consumer outboxes** (decision 8): two consumers, two named edges — `CommandProcessorToHalCommand` (addressed motor traffic: DEV M, and broadcast e.g. DEV STOP) and `CommandProcessorToDrivetrainCommand`. The `<Producer>To<Consumer>Command` rule survives with fixed endpoints; no routing switch in main.
- **Queries produce replies, not commands — principled asymmetry.** PING / DEV M n STATE / VER transform into replies; the processor keeps *read* access to the observation and capabilities channels (`hal.motor(n).state()`) and answers at parse time. Commands mutate → they cross the visible board; observations are pull-only reads → direct.
- **`OK` means parsed, validated, delivered-for-staging** (same meaning it has today — apply was always staging, never execution). The processor pre-validates against `capabilities()` so delivery cannot fail.
- **Broadcast**: DEV STOP parses to one ALL-addressed neutral; expanding it across ports is the HAL distributor's job. The processor never loops over ports.
- **Wiring-level statements** (watchdog window; see Part 5 for PORTS) configure the loop state at parse time — they are not device commands.
- The statement feed **copies** the line (the Communicator's edge pointer aliases its internal buffer).
- The vestigial **`CommandQueue`** (source/commands/command_queue.h, 1.7 KB BSS, wired to nothing) is **deleted** (decision 5).

| Edge | Kind | Semantics | Where the slot is |
|---|---|---|---|
| Communicator → processor (statement) | event — dropping one loses a command | producer-blocks (untaken statement pauses transport polling; backlog holds in the drivers) | held in the Communicator |
| Processor → HAL / → Drivetrain | setpoint | latest-wins | processor outboxes (has/take) |
| Drivetrain → HAL | setpoint, regenerated every tick | latest-wins by construction | drivetrain outbox (has/take) |
| Write layer → brick (throttle, dwell) | time-gated | consume at the right time | motor-internal; upstream never knows |

## Part 4 — the HAL as the distribution subsystem (decision 3)

Motor ownership stays in the HAL — a Drivetrain-owned pair is ruled out by the brick (the 0x10 schedule spans all four ports). The HAL's three roles:

1. **Access** — reach the devices it owns: `hal.motor(port).state()`, `capabilities()`.
2. **Distribution** — consume addressed commands at its faceplate (`apply(CommandProcessorToHalCommand)`, `apply(DrivetrainToHalCommand)`; shared inner addressed-wheel struct), stage into the target motors via direct method calls (messages above, methods below), mark ports in-use, expand broadcasts.
3. **Timers** — home of the schedule timing: the brick flip-flop, the settle windows, driving the bus's per-device clearance table. Future I2C devices (OTOS 0x17, line 0x1A, color) join under the HAL tier so its scheduler fills the settle windows with their reads (Case 5).

**Motors are the HAL's internal business** — nothing outside the HAL touches a Motor's command plane.

## Part 5 — the Drivetrain produces commands for the HAL (decision 7)

Rigorously, the consumer is whoever the command is handed to — the HAL. `DrivetrainToMotorCommand` named a consumer the drivetrain never meets. Revised edge:

```cpp
struct DrivetrainToHalCommand {
  struct Wheel { uint32_t port; msg::MotorCommand command; };
  Wheel wheel[2];       // differential today; mecanum's 4 fits the same shape
};
```

Which requires the drivetrain to know its ports: **the port binding moves into DrivetrainConfig** — `DEV DT PORTS` becomes an ordinary drivetrain-addressed config statement. Which wheels are mine is the same kind of fact as my trackwidth. Purity survives: observations in as tick arguments, commands out as a held edge; a port number in config is data, not a handle; zero-mock testing unchanged. Main shuttles observations by querying `drivetrain.ports()` instead of holding its own copy (`devState.leftPort/rightPort` retire).

## Worked cases

**Case 1 — `DEV DT VW 100 0 0` arrives on serial** (ports 1/2 in use, at rest, loop ~1 ms):
- t=0: slice 1; statement taken from comm → fed to processor, watchdog fed; processor.tick parses → `OK` out serial, drivetrain outbox... → `drivetrain.apply(twist)` staged; drivetrain ticks on fresh states → holds addressed wheel targets → taken, fed to HAL; slice 2: port 2 mid-settle → pass.
- t≈2: port 2 collect → PID sees new target → slew-limited **25%** duty written. Wheel starts.
- t≈8: port 1 collect → **25%**. Both wheels turning ~8 ms after the statement (vs up to ~32 ms + a loop today).
- t≈48…: collects step 25→50→75→100 under the 40 ms throttle while the drivetrain re-governs every pass (latest-wins).

**Case 2 — flip-flop timeline, 2 ports in use** (request 0.85 ms, settle 4 ms, collect 0.45 ms + PID):

| t [ms] | hal slices do | loop otherwise |
|---|---|---|
| 0 | **reqP1** (readyAt = 4.85) | comms/processor/drivetrain between slices |
| 1–4 | pass (settling) | free — comms polled every pass |
| 5 | **collectP1** → EMA → PID → write | |
| 6 | **reqP2** | free |
| 7–10 | pass | free — an OTOS read fits here (case 5) |
| 11 | **collectP2** → PID → write | |
| 12 | **reqP1** — per-motor period ≈ 12 ms (~83 Hz), ~9.5 of every 12 ms free | |

**Case 3 — reversal, the dwell as the deeper flip-flop.** Cruising at VEL +120 (~+60% duty); `DEV M 1 VEL -120` arrives. Processor emits an addressed HAL command; the staged target is **overwritten** (latest-wins). Next collect: PID goes negative → write path sees the sign change → writes 0 immediately, opens the 100 ms dwell. For ~8 collects the PID keeps computing while `writeDuty` suppresses non-zero writes — **neither the processor, the outboxes, nor the Drivetrain knows**. First collect past the dwell: slew from 0 → −25%. "Consume at the right time" is entirely the write layer's internal state.

**Case 4 — watchdog fires.** Check runs every pass; longest block is one bus transaction (~0.9 ms), so it fires at t≈1000+1 ms (today: up to ~32 ms late behind a blocked sweep). Neutral staged everywhere via the one audited path; each motor executes at its next collect — worst case ~26 ms with four ports in use. **Accepted (decision 2)**: ~1 cm of motion; no escape hatch.

**Case 5 — OTOS fills a settle window.** During P2's settle, `otos.tick(now)` finds its own device slot (0x17) clear — timers are per-device — and runs a 1–2 ms pose read inside the window at zero motor-cadence cost. If it overruns, the next collect starts late; nothing corrupts. Gate: the lazy-timer issue's stand A/B.

## Decisions (stakeholder, 2026-07-04)

1. **Cycle only the ports in use** — no four-port sweep; in-use on first distributed command, sticky.
2. **~26 ms staged-neutral watchdog latency accepted** (~1 cm of motion); no stop escape hatch.
3. **The HAL is the distribution subsystem**: access + addressed-message distribution + home of the schedule timers. Motors not moved into the Drivetrain.
4. **"Statements" confirmed** — dedicated rename issue filed: [rename-wire-lines-to-statements](rename-wire-lines-to-statements.md).
5. **CommandQueue deleted** — vestigial.
6. **tick returns void; has/take collects** — command-out is held and explicitly taken; the HAL is ticked twice per pass (slice 1 collects, slice 2 requests/writes); the CommandBoard object dissolves into producer outboxes.
7. **The processor is a pure transformer** — statements in, commands out + replies; per-consumer outboxes; read-only observation access for queries; no device write access.
8. **The Drivetrain produces for the HAL** — `DrivetrainToHalCommand` with addressed wheel array; port binding moves into DrivetrainConfig (`DEV DT PORTS` becomes drivetrain config).
9. **Messages above, methods below** — messages between subsystems only; direct method calls within one.
10. **Subsystem is the unit of test** — hardware fakes below a subsystem accepted (confined to the `I2CBus` HOST_BUILD seam for the HAL).

## Deltas against the current tree

- `Subsystems::Communicator` (implemented 2026-07-04, 2599df3): `tick(now)` currently returns the edge — becomes void + `hasStatement()`/`takeStatement()`; untaken statement pauses transport polling; edge type renamed `CommunicatorToCommandProcessorStatement` (rename issue).
- `Subsystems::Drivetrain`: `DrivetrainToMotorCommand tick(now, leftObs, rightObs)` → `void tick(...)` + `hasCommand()`/`takeCommand()` yielding `DrivetrainToHalCommand`; ports into `DrivetrainConfig`; `setWheelTargets` naming per config binding.
- `CommandProcessor`: handlers stop calling devices; emit into per-consumer outboxes; keep read access for queries; DevLoopState sheds leftPort/rightPort and the direct motor/drivetrain pointers it no longer needs.
- `Hal::NezhaHal`: brick flip-flop + in-use tracking + `apply(...)` overloads + the two-slice contract; `NezhaMotor`: split-phase wired, spins removed (lazy-timer issue), tick becomes the collect half.
- `main.cpp`: the Part 2 loop.

## Risks

1. **Shared-0x10 clobber**: writes inside a settle window may corrupt the pending readback — postClear-on-request + write-at-collect-only must be structural. Verify on the stand that an abandoned collect's readback is cleanly overwritten by the next request.
2. **The armor issue is a co-requisite**: faster PID cadence + the writeDuty reversal exemption = *more frequent* latch-trigger reversal trains until deadband + dwell land. Sequence armor with or before the flip-flop wiring.
3. **`vel_filt_alpha` retune** at the new cadence (bench pass; silent-failure class).
4. **Settle-window traffic untested** — keep the lazy-timer issue's stand A/B gate.
5. **Statement feed must copy** — the Communicator edge pointer aliases its internal buffer.
6. **No sampling on idle ports** (decision 1's cost): a wedged/disconnected motor on a never-commanded port goes unnoticed until first use.

## Relationship to other issues

- **i2c-bus-lazy-clearance-timers** — the substrate; unchanged except the non-spinning `I2CBus::clear(addr)` peek.
- **armor-motor-write-path** — co-requisite (risk 2); its dwell is Case 3's deeper flip-flop.
- **turn-the-communicator-into-a-faceplate-subsystem** — DONE (2599df3); this design's deltas revise its command-out shape.
- **[rename-wire-lines-to-statements](rename-wire-lines-to-statements.md)** (decision 4, filed) — rule-4 payload amendment (`<Producer>To<Consumer><Payload>`, payload ∈ {Command, Statement}), `CommunicatorToCommandProcessorStatement`, protocol-doc sweep, CommandProcessor naming call (initial rec: keep the class name).
- **CommandQueue deletion** (decision 5) — fold into the processor-outbox work.

## Verification sketch

When implemented: build both `ROBOT_DEV_BUILD` forks; host tests at the subsystem level (drivetrain + processor with plain structs; HAL against a scripted `HOST_BUILD` I2CBus fake — flip-flop sequencing, throttle, dwell, in-use tracking); stand pass per hardware-bench-testing — encoder cadence + evenness via `DEV M n STATE` polling at speed, in-use-port cycling (idle ports generate zero bus traffic), statement round-trips over serial and radio, watchdog fire latency, the lazy-timer A/B (latch rate with settle-window traffic vs without); `vel_filt_alpha` retune with step responses matching pid_hold_speed tolerances.
