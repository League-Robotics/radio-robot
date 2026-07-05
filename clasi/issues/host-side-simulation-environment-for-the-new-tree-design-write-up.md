---
status: pending
---

# Host-Side Simulation Environment for the New Tree — Design Write-Up

**Status: reviewed design, pending sprint planning. Stakeholder review round complete
(2026-07-04) — the three open questions are resolved (see "Resolved decisions" below)
and folded into the design above. Not yet broken into a sprint / tickets.**

## Context

Sprint 077's greenfield rebuild parked the simulation environment with `source_old/`
and `tests_old/`. Today `tests/sim/` holds one placeholder test, the new `source/`
tree has no simulator, and the old 147-file sim suite can't run. This design brings
the sim over: **two builds** (one CODAL compile for the ARM firmware, one host-side
compile producing a shared **library**), the library **loaded from Python via ctypes**,
and the **main loop replicated so Python can drive it** tick by tick. The sim mostly
drives the real firmware C++; only two hardware devices are simulated — the **motors**
and the **OTOS**. The model carries over from the old tree:

```
                 ┌────────────────────────────────────────────┐
  wheel commands │  PhysicsWorld — errorless plant            │
  ──────────────►│  integrates TRUE pose + TRUE wheel travel  │──► what the camera
                 │  (the simulated reality, no error)         │    would report
                 └───────┬────────────────────────┬───────────┘
                         │ reads truth            │ reads truth
                 ┌───────▼──────────┐    ┌────────▼──────────┐
                 │ SimMotor/encoder │    │ SimOdometer (OTOS)│
                 │ own accumulator, │    │ own accumulator,  │
                 │ WITH error       │    │ WITH error        │
                 │ (scale/slip/     │    │ (noise/scale/     │
                 │  noise/stiction/ │    │  drift)           │
                 │  lag)            │    │                   │
                 └──────────────────┘    └───────────────────┘
```

One errorless integration is ground truth; the two sensor models integrate separately
with error applied. When all error knobs are zero, all three agree bit-for-bit.

## What we had (verified in the parked tree)

The old system is exactly this architecture, so this is a port-and-adapt, not an
invention. Inventory of what carries over:

- **`source_old/hal/sim/PhysicsWorld.{h,cpp}`** — the single ground-truth plant. One
  `update(dt)` midpoint-arc integrator owning true pose, true wheel travel/velocity,
  **plus** a separate *reported* encoder accumulator carrying the error model:
  per-wheel scale error / slip / Gaussian noise (sprint 058), stiction gate +
  first-order motor lag (072), body scrub + rotational slip on the pose path (069/073).
  Seeded RNG streams → deterministic runs.
- **`SimMotor`** — reads the plant's *reported* (errored) encoder. **`SimOdometer`** —
  samples the plant's *true* pose each tick, differences it, applies OTOS
  noise/scale/drift into its **own** accumulator (OTOS error independent of encoder
  error).
- **`SimCommands.cpp`** — `SIMSET`/`SIMGET` over a `kSimRegistry[]`, backed by shared
  `simsetters::` functions also used by the ctypes ABI (one canonical setter per knob).
- **Build** — `tests_old/_infra/sim/CMakeLists.txt` compiled the host-clean firmware
  sources + `hal/sim/` + a C-ABI shim (`sim_api.cpp`) into
  `libfirmware_host.{dylib,so}` with `-DHOST_BUILD=1`.
- **Main loop** — extracted into a shared `loopTickOnce(now, …)` called by both the
  ARM scheduler and the sim's `sim_tick(handle, now)`. Time fully injected and
  Python-stepped (~24 ms increments); real-time pacing was an optional Python overlay.
- **Python** — `firmware.py`'s `Sim` class (ctypes + pytest fixtures), `SimConnection`
  (drop-in serial-connection replacement), TestGUI's `SimTransport`.

**Hooks already live in the new tree** and self-heal the moment `tests/_infra/sim/`
exists: `build.py build_host_sim()`, `just build-sim`, and
`host/robot_radio/io/sim_conn.py`'s ctypes contract (needs only minor fix-up).

## Design for the new tree

### Components

New directory `source/hal/sim/`, a sibling leaf family to `hal/nezha/` — the
`hal/capability/` faceplates were explicitly designed as this swap point (motor.h's
own header comment names a future SimMotor):

| File | Base | Role |
|---|---|---|
| `physics_world.{h,cpp}` | — | **Ported from the old plant, trimmed to motor/pose/encoder only.** Keep the drivetrain math bit-identical (golden determinism) and the internal [-100,100] actuator scale so stiction-knob semantics and old test expectations port unchanged. **Drop the aux line/color/port truth channels** (`_lineRaw`/`_colorRGBC`/`_port` + setters/getters) — those sensors are not simulated; they add back only if a sensor sim is ever wanted (decision 2). |
| `sim_motor.{h,cpp}` | `Hal::Motor` | Observation model per port; holds `PhysicsWorld&` + a plant-side binding (LEFT/RIGHT/UNBOUND). Contract mapping below. |
| `sim_odometer.{h,cpp}` | `Hal::Odometer` | The OTOS model, ported from old `SimOdometer` — **the first concrete `Hal::Odometer` leaf in the tree**. `pose()` returns `msg::PoseEstimate` (pose + twist + freshness stamp = now). |
| `sim_hal.{h,cpp}` | `Hal::MotorHal` (new seam, below) | Mirrors `NezhaHal`'s surface (`motor(port)`, `tick(now)`, `begin()`, 4 ports). Owns the ONE plant + 4 SimMotors + the SimOdometer; its `tick(now)` advances the plant exactly once per new timestamp. |
| `sim_setters.h` | — | Header-only `simsetters::` free functions over `SimHal&` — the single-source-of-truth-per-knob pattern (069-005 lesson): one canonical call site per error knob, so the ctypes ABI can never drift internally. (Not a precursor to a wire registry — see below.) |

**One small firmware seam — `source/hal/capability/motor_hal.h`**: a thin abstract
owner interface (`virtual Hal::Motor& motor(port)`, `virtual void tick(now)`,
`kPortCount`), named `Hal::MotorHal` to match the existing `NezhaHal` naming — both
`NezhaHal` and the new `SimHal` become concrete leaves of this one abstract base.
**Why this is needed at all**: today `DevLoopState::hal` is typed to the concrete
`Hal::NezhaHal*`, and `dev_commands.cpp`'s DEV command handlers call through it
directly — so `SimHal` cannot stand in for it without either (a) both leaves sharing
one small interface, or (b) forking `dev_commands.cpp` into a sim-only copy of the same
DEV table. (b) is worse: it duplicates the exact command-routing code the sim exists to
exercise. So `DevLoopState::hal` retypes from `Hal::NezhaHal*` to `Hal::MotorHal*` — a
one-line change, no behavior change on the real robot.

**Also new, and shared with real hardware — `source/hal/velocity_pid.{h,cpp}`**:
`NezhaMotor::runVelocityPid()` is extracted into a pure, host-clean `MotorVelocityPid`
class (gains, feedforward, anti-windup — whatever the current embedded logic does),
taking `now` and measured/target velocity as parameters, no I2C/CODAL dependency.
`NezhaMotor` is refactored to call it — a behavior-preserving refactor, verified by
bench step-response comparison before/after (an explicit acceptance criterion, not
just a host-side test). `SimMotor` calls the identical class. **This is the key
correction from the stakeholder review: the sim must run the real PID, not a
re-derived approximation** — velocity-loop response (rise time, overshoot, settling)
is one of the things this harness exists to test, not something to model around.

**Compiled out, not stubbed**: `com/*`, `subsystems/communicator.*`, `hal/nezha/*.cpp`,
`main.cpp`. The sim bypasses the Communicator exactly as before — Python feeds
statements straight to the CommandProcessor.

### SimMotor: honoring the full `Hal::Motor` contract

`apply()`/`state()` come free (implemented once, inline in `capability/motor.h`).
The primitives:

- **`setDutyCycle` / `setNeutral`** — stage mode + target; consumed by `SimHal::tick`.
  **DUTY mode** stages the duty directly, fed straight to the plant, which applies the
  stiction gate + first-order lag (the 072 knobs, ported intact) before integrating.
- **`setVelocity`** — stages a target velocity for **VELOCITY mode**, which runs the
  **same shared `MotorVelocityPid`** NezhaMotor runs (see above). Each tick:
  `MotorVelocityPid::compute(target, measuredVelocity, now)` → a duty command → fed to
  the plant, same stiction/lag path as DUTY mode. The loop closes exactly as on real
  hardware: the PID sees next tick's *plant-lagged* velocity, not its own commanded
  value, so overshoot/settle behavior is genuinely exercised, not assumed away.
- **`position()`** — the plant's **reported (errored)** encoder: this is where
  "simulated motors integrate with error" lands. **`velocity()`** — post-stiction/lag
  wheel velocity. **`appliedDuty()`** — the duty actually fed to the plant (so
  `DEV M n STATE applied=` stays meaningful).
- **`configure()`** — stored; `fwd_sign` honored; PID gains / `travel_calib` /
  `vel_filt_alpha` inert (documented loudly + a test asserting inertness on purpose).
- **`capabilities()`** — `duty_cycle=true, voltage=false, velocity=true,
  position=false, has_encoder=true`. **No POSITION mode, by decision**: `DEV M n POS`
  gets `ERR unsupported` in sim, matching how a Nezha lacking the capability would
  answer — not modeled. `connected()=true`, `wedged()=false` in v1 — failure injection
  (dropout, stuck wedge) is a real v2 want (filed as a `later/` issue, decision 3) but
  not v1 scope.
- **Port→plant coupling** (4 anonymous ports vs an L/R differential plant): SimHal
  binds port 1→plant LEFT, port 2→plant RIGHT by default; ports 3/4 unbound (trivial
  standalone integrator so `DEV M 3 …` still observes plausible motion without touching
  world truth). Rebindable via the ABI to mirror `DEV DT PORTS`; post-079 this pairs
  with the binding moving into `DrivetrainConfig`.

### Main-loop replication: one shared body, two callers

**Recommendation: extract a shared tick body (the old LoopTickOnce pattern reborn)
and land the sim after sprint 079**, because 079 rewrites every signature the loop
touches (void ticks + has/take held outputs, `DrivetrainToHalCommand`, CommandQueue
deletion, port binding into config). The old tree already learned that a hand-mirrored
loop in `sim_api.cpp` drifts; with 079's "main is the visible mover of every command,"
a drifted sim copy would silently reorder command movement. One body, two callers.

Shared body `source/dev_loop.{h,cpp}` — beside main.cpp, host-clean, `now` always a
parameter (**not** under `source/robot/`: creating that directory reactivates
`gen_default_config.py` in build.py, which would emit an old-tree `DefaultConfig.cpp`
— verified in build.py:85-90). Post-079 shape:

```cpp
void devLoopTick(DevLoop& loop, uint32_t now) {   // [ms]
  loop.hal.tick(now);                             // slice 1 (sim: plant advances, dt-guarded)
  if (loop.hasStatement()) {                      // fed by the CALLER (fw: Communicator; sim: Python)
    loop.watchdog.feed(now);
    loop.processor.apply(loop.takeStatement());
  }
  loop.processor.tick(now);                       // parse → replies out; commands → outboxes
  if (loop.processor.hasHalCommand())        loop.hal.apply(loop.processor.takeHalCommand());
  if (loop.processor.hasDrivetrainCommand()) loop.drivetrain.apply(loop.processor.takeDrivetrainCommand());
  if (loop.drivetrain.active()) {
    Subsystems::DrivetrainPorts p = loop.drivetrain.ports();
    loop.drivetrain.tick(now, loop.hal.motor(p.left).state(), loop.hal.motor(p.right).state());
    if (loop.drivetrain.hasCommand())        loop.hal.apply(loop.drivetrain.takeCommand());
  }
  loop.hal.tick(now);                             // slice 2 (sim: same now → dt=0 pass-through)
  if (loop.watchdog.check(now)) { neutralizeAll(...); loop.emitEvt("dev_watchdog"); }
}
```

- **Firmware caller** (`main.cpp`): read the CODAL clock, tick the Communicator, feed
  any statement, call `devLoopTick`. The Communicator stays outside the shared body,
  in the one CODAL file.
- **Sim caller** (`sim_api.cpp`): `sim_tick(h, now)` = `devLoopTick(h->loop, now)` —
  time is stepped and Python-driven; nothing advances otherwise. `sim_command(h,
  line, …)` copies the line into the statement inbox and runs `devLoopTick` at the
  **same** `now` — SimHal's dt-guard makes the plant advance zero, but parse, replies,
  and command staging happen in exact firmware order with zero time consumed. This
  keeps synchronous send-and-reply semantics with no special dispatch path.
  (Requires every `tick(now)` to be dt=0-safe — an explicit acceptance criterion.)
- The two-slice HAL tick and today's bound-pair double-tick are both non-issues: the
  plant advances only when `now` increases.

### Clock

Verified: in the new tree, `now` flows as a parameter everywhere in the host-clean set
— the old `thread_local` sim-clock machinery is **not needed**. One exception:
`system_commands.cpp` reads `system_timer_current_time()` in PING (and CODAL identity
strings in ID). Fix with a tiny seam: `source/types/clock.h` declaring
`uint32_t systemClockNow(); // [ms]`, implemented on-target as the vendor call and
host-side as a global that `sim_tick`/`sim_command` update; fixed host identity
strings for ID. That makes PING/VER/HELP/ECHO/ID all round-trip in sim.

### Build

`tests/_infra/sim/CMakeLists.txt` → `add_library(firmware_host SHARED …)`:

- Defines `-DHOST_BUILD=1 -DROBOT_DEV_BUILD=1`; C++ standard matched to the firmware.
- **Explicit source list, not glob-then-filter** (old CMake's exclusion-regex pile was
  a maintenance smell): `kinematics/*.cpp`, `subsystems/drivetrain.cpp`,
  `commands/{arg_parse,command_processor,dev_commands,system_commands}.cpp`,
  `dev_loop.cpp`, `hal/sim/*.cpp`, `sim_api.cpp`. Absent: `com/*`, `communicator.*`,
  `hal/nezha/*.cpp`, `main.cpp`.
- Activation is free: `build.py build_host_sim()`, `just build-sim` (already runs
  codegen first), and `sim_conn.py`'s default lib path all already point at
  `tests/_infra/sim/`.
- ARM build untouched. Two compiles total, as required.

### C ABI + Python surface

Target `sim_conn.py`'s existing 28-symbol contract so `SimConnection` works nearly
as-is. v1 ABI ≈ 40 functions (vs the old ~120), over an opaque `SimHandle` owning
SimHal + Drivetrain + CommandProcessor + DevLoop + a reply store:

- **Lifecycle/loop**: `sim_create/destroy`, `sim_tick(h, now)`,
  `sim_command(h, line, reply, size)` (synchronous replies), `sim_get_async_evts`.
- **Ground truth**: `sim_get_true_pose_x/y/h` (+ legacy `exact_pose` aliases),
  `sim_set_true_pose`, `sim_get_true_enc_l/r`, `sim_get_true_vel_l/r`,
  `sim_set_true_wheel_travel`.
- **Errored observations**: `sim_get_enc_l/r` (reported encoder), `sim_get_vel_l/r`,
  `sim_get_pwm_l/r`, `sim_get_otos_x/y/h`, setters for injection.
- **Error knobs** (all forwarding through `simsetters::`): motor offset/slip, encoder
  noise/scale/slip, stiction/lag, trackwidth, body scrub, OTOS noise/scale/drift,
  plant port binding.

Python placement keeps the old split: `tests/_infra/sim/firmware.py` hosts the `Sim`
class (ctypes, context manager, `tick_for(total, step=24)`); fixtures land in
`tests/sim/conftest.py` (session-scoped `build_lib` running cmake, function-scoped
`sim`) — the shape its placeholder docstring already specifies; `pyproject.toml`
already collects `tests/sim`. **Watchdog policy**: the `sim` fixture issues
`DEV WD 3600000` after create (the 1 s `SerialSilenceWatchdog` would otherwise
neutralize motors during any long `tick_for`), with one dedicated test that lowers it
to verify the watchdog + `EVT dev_watchdog` path.

### Error-knob and sim-telemetry surface: ctypes backdoor only — no wire statements

**Decision, not a phased plan: sim parameters and sim-only telemetry never go on the
wire.** Everything test-only — error knobs (slip, noise, scale, stiction, lag, body
scrub, trackwidth), ground-truth reads (`sim_get_true_pose_*`, `sim_get_true_enc_*`),
and per-model sensor telemetry (reported encoder, OTOS accumulator) — is accessed
exclusively through direct ctypes calls into the library, exactly as the old system's
`sim_*` C-ABI side channel worked. There is **no `SIMSET`/`SIMGET` wire command
family** in this design and **no sim-specific `TLM` fields**: the wire protocol carries
only the real DEV/protocol-v2 surface a physical robot also understands. `sim_setters.h`
(`simsetters::` free functions over `SimHal&`) is still written as the one canonical
call site per knob — not to back a future wire registry, but so the ctypes ABI itself
has a single, ungameable source of truth per parameter. This keeps the wire protocol
honest (a test harness cannot accidentally teach the real robot to answer `SIMSET`)
and keeps sim-only concerns out of the message schema entirely.

### The OTOS gap: the sim leads the firmware

No firmware consumer for OTOS exists yet (new Drivetrain has no odometry/EKF;
DrivetrainState pose fields are zero-defaults). In v1 the SimOdometer feeds **only the
C ABI** for test assertions — "OTOS accumulator tracks plant truth within noise bounds
while the encoder path diverges under slip." That's a feature: the day fusion lands,
its acceptance tests (truth vs encoder-only vs OTOS-corrected) are already runnable.
Per odometer.h's own gap note, **no `protos/odometer.proto` in this work** — sim error
knobs are test infrastructure, not device Config; the real OTOS leaf triggers the
proto. **No sim-specific lever-arm modeling, by decision**: a lever-arm offset is a
robot configuration concern (mounting offset), the same on a real OTOS leaf as on a
simulated one — orthogonal to the sim's physics/error model. `SimOdometer` reports the
plant's sampled pose directly; if/when a real OTOS leaf's config carries a lever-arm
offset, that same config path applies to the sim leaf too, with no separate mechanism.

## What the sim validates — and deliberately doesn't

Validates: everything **above and including the velocity-PID control loop** —
protocol parsing, command routing, the drivetrain governor, main-loop ordering, the
watchdog, world-outcome geometry/estimation behavior under injectable error, **and**
velocity-PID response (rise time, overshoot, settling) — because `SimMotor` runs the
identical `MotorVelocityPid` class the real `NezhaMotor` runs, not an approximation.

Does **not** validate: the I2C register write-path itself — throttle timing, the 0x46
settle/read sequence, slew clamping against a real write-on-change brick, wedge
detection from actual bus behavior. Those stay hardware-specific, covered by sprint
078's `tests/unit` C++ fixture harness, 079's subsystem tests against the scripted
HOST_BUILD `I2CBus` fake, and the bench gate. Sim-green means the control loop and
everything above it is correct; it does not mean the I2C write path is bench-safe —
that distinction must stay documented wherever the suite is described.

## Sequencing and phasing

Recommended slot: **after 079** (which rewrites every edge the sim loop crosses),
before or alongside 080. Ticket sketch:

| # | Ticket | Contents | Depends on |
|---|---|---|---|
| 1 | Extract shared velocity PID | Pull `NezhaMotor::runVelocityPid()` into pure, host-clean `source/hal/velocity_pid.{h,cpp}` (`MotorVelocityPid`); refactor `NezhaMotor` to call it — behavior-preserving, verified by bench step-response comparison | none — **can start immediately** |
| 2 | Plant + sim devices | Port PhysicsWorld; SimMotor (VELOCITY mode calls `MotorVelocityPid`) + SimOdometer + SimHal + sim_setters.h under `source/hal/sim/` | 1 |
| 3 | Host-clean seams | `Hal::MotorHal` abstract owner + `DevLoopState` retype; clock seam + system_commands HOST_BUILD guards; extract `devLoopTick` into `source/dev_loop.{h,cpp}`; ARM build behavior-identical (bench smoke) | 079 merged — or **folded into 079's main-loop ticket** (recommended) |
| 4 | Build + C ABI | CMakeLists, sim_api.cpp (SimHandle, reply store, dt=0 command trick, ctypes-only knob/telemetry surface); `just build-sim` green | 2, 3 |
| 5 | Python wrapper + fixtures + first tests | firmware.py `Sim`; conftest fixtures; sim_conn.py fix-up; protocol/plant/PID/determinism tests | 4 |
| 6 | Port high-value old tests | Encoder-error, OTOS-error, stiction/lag suites from tests_old/simulation/ | 5 |

First tests (v1 — EKF/fusion tests excluded until firmware fusion exists): plant
correctness (drive/turn geometry vs truth), errored-observation split (knobs make
reported diverge from true by the configured amount; zero knobs restore bit-agreement),
**velocity-PID response** (step command → rise time/overshoot/settle within the same
envelope the bench uses, now that sim and hardware share the PID class), stiction/lag
response envelopes, protocol round-trips (PING, DEV M/DT family, ERR unsupported,
watchdog EVT), determinism (identical scripts → bit-identical logs).

## Risks

1. **Sequencing vs 079** — the loop-extraction ticket (3) done early gets written
   twice. Mitigation: tickets 1-2 are 079-independent and can start now; ticket 3 rides
   079 (or folds into its main-loop ticket).
2. **PID-extraction regression** — pulling `runVelocityPid()` out of `NezhaMotor` must
   be strictly behavior-preserving (same gains, same anti-windup, same output), or the
   real robot's control loop changes as a side effect of a testing project. Mitigation:
   bench step-response comparison before/after as an explicit ticket-1 acceptance
   criterion, not just a host-side test.
3. **dt=0 safety** — the synchronous `sim_command` trick requires every `tick(now)` to
   be safe at a repeated timestamp — now doubly true with the PID in the loop, since a
   repeated-`now` call must not double-integrate the PID's anti-windup state; explicit
   acceptance criterion.
4. **Config inertness** — any config field (e.g. `travel_calib`) that doesn't map onto
   the shared PID's own parameters is a silent-divergence class (cf. the old
   `vel_filt_alpha=0` episode). Since the PID itself is now shared, this narrows to
   whatever NezhaMotor-only fields (if any) remain outside the extracted class — audit
   during ticket 1.
5. **Capability divergence** — `position=false` in sim vs true on Nezha means the same
   wire command answers differently per target; document in the protocol doc's sim
   notes.
6. **SimTransport/TestGUI revival** is a separate later effort — the transport-level
   `SimConnection` works, but the old protocol verbs above it (T/D/VW) don't exist in
   the new firmware.

## Resolved decisions (stakeholder review, 2026-07-04)

1. **Sequencing** — **decided: fold ticket 3 (the loop-extraction seam) into sprint
   079's own main-loop rewrite.** 079 already rewrites this loop; it writes it directly
   into the shared `dev_loop.{h,cpp}` so it is authored once, structured for both the
   firmware and sim callers. The remaining sim work (build + C ABI + Python wrapper +
   tests, tickets 4-6) then follows as its own body of work on top of that seam.
   Tickets 1-2 (PID extraction, plant + sim devices) are 079-independent and can start
   immediately.
2. **Aux truth channels** — **decided: trim them out.** The line/color/port truth
   fields are dropped from the ported `PhysicsWorld` — we are not simulating those
   sensors, we are not using them in sim, they work fine on real hardware, and they are
   cheap to verify on the bench, so they likely never need simulating. Port only the
   motor / pose / encoder plant. If a sensor sim is ever wanted, the fields come back
   then. (Supersedes the "keep dormant" note in the components table.)
3. **v2 failure injection** — **decided: file a CLASI `later/` issue now.** Fault
   injection (`connected=false`, `wedged=true`, encoder dropout / stuck value) is out of
   v1 scope but captured as a backlog issue so it is not lost — the encoder-wedge
   history makes a deterministic in-sim wedge repro genuinely valuable later. See
   `clasi/issues/later/sim-hardware-fault-injection.md`.

## Verification (when implemented)

- PID-extraction gate: bench step-response (command a velocity step, capture rise
  time/overshoot/settle) matches pre-extraction `NezhaMotor` behavior — ticket 1 is
  not done until this is confirmed on real hardware.
- `just build-sim` produces `libfirmware_host.dylib`; `uv run python -m pytest`
  collects and passes `tests/sim/` (real tests replacing the placeholder).
- Determinism gate: same command script twice → bit-identical state logs.
- Zero-error gate: all knobs zero → true enc == reported enc == OTOS accumulator
  (bit-for-bit), matching old golden-TLM discipline.
- ARM build byte-behavior unchanged: flash + bench smoke per hardware-bench-testing
  gate (the loop extraction and DevLoopState retype must not change firmware behavior).
