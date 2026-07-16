---
status: in-progress
sprint: '108'
tickets:
- 108-001
- 108-002
- 108-003
- 108-004
- 108-005
- 108-006
- 108-007
- 108-009
- 108-010
---

# Plan: Pure I2CBus/Clock interfaces + a real SimPlant simulator

## Context

The host firmware simulator was built on a **scripted-FIFO I2C fake**
(`i2c_bus_host.cpp`'s `scriptWrite`/`scriptRead`, living *inside*
`i2c_bus.h` behind `#ifdef HOST_BUILD`) plus a per-cycle write-count
**predictor** (sprint 105's `TestSim::SimApi` + `DutyPredictor`). Driven by a
stream of arbitrary profiled twists (a tour) that predictor desyncs the shared
FIFO and the wheels diverge (left encoder freezes, right runs away). A
"live responder" bolt-on I added made it two-faced — it answered reads from a
plant but ACK'd duty writes and read `motor.appliedDuty()` out a side channel;
the write side of the register protocol was theater.

The real problems: (1) `i2c_bus.h` is a concrete class with two `#ifdef`
forks and test-only scripting glued into the production header; (2) the
"simulator" needs to *predict* firmware I2C behavior instead of *responding*
to it; (3) three parallel sims (SimApi, my LiveSim, the dead host Python
stack) each re-construct the subsystem graph and re-hardcode fixtures.

**Goal (stakeholder-directed):**
- `source/devices/i2c_bus.h` → a **pure abstract interface** (the I2C contract
  only), mirroring the existing `App::Transport` interface pattern
  (`source/app/comms.h:34`). No implementation, no `#ifdef HOST_BUILD`, no
  `scriptWrite`/`scriptRead`, no clock/sim methods.
- `Devices::MicroBitI2CBus : I2CBus` — the real CODAL implementation (today's
  `i2c_bus.cpp` machinery).
- `SimPlant : I2CBus` — a real state machine: parses the Nezha `0x60` duty
  writes and `0x46` encoder-selects and OTOS registers on `write()`,
  integrates per-wheel physics + OTOS pose when `tick()`ed, and returns live
  encoder/pose bytes on `read()`. **No script, no predictor, no
  `appliedDuty()` back-channel** — duty comes off the wire.
- The concrete bus is chosen by what gets injected into the `I2CBus&` slot
  (`main.cpp:99` for the robot; the ctypes harness for the laptop) — not by
  compile-time `#ifdef`.
- `Clock`/`Sleeper` (`source/devices/clock.h`) get the **same** pure-interface
  treatment (real `MicroBitClock`/`MicroBitSleeper` vs `SimClock`/
  `SimSleeper`), eliminating the last `#ifdef HOST_BUILD` device fork.
- Fault-injection / device-condition knobs (disconnect a wheel, encoder
  wedge/dropout/noise, OTOS drift) live on `SimPlant`, reachable through the
  ctypes ABI — **never** in `i2c_bus.h`.
- **No scripted I2C bus at all.** `SimPlant` carries an optional per-direction
  **read/write hook** (middleware): when a hook is registered, `read()`/
  `write()` invoke it and hand it the *default* behavior as an invocable. The
  hook decides — call the default (pass-through: a read returns the plant's
  real bytes, a write applies normally) or don't (fully override with its own
  bytes / swallow the write). The hook registration is exposed through the
  ctypes ABI so **all scripting is done in Python** — the 13 register-level
  unit tests become Python tests that register hooks; no C++ scripted bus,
  no FIFO.
- Every superseded sim (SimApi/DutyPredictor, my LiveSim, the dead host Python
  ABI) is deleted.

**Outcome:** pressing **Tour 1** in the TestGUI (Sim) drives the real compiled
firmware against `SimPlant` and draws the trace — over an architecture with a
clean interface, one honest simulator, and no `#ifdef` soup.

## Target architecture

```
         I2CBus (pure interface)              Clock / Sleeper (pure interfaces)
        /            \                          /            \
MicroBitI2CBus      SimPlant                MicroBit*     Sim*   (host)
   (ARM,             (host sim, tests/ — the ONE bus for
    source/)          the laptop; owns wheel+OTOS physics,
                      tick(), fault knobs, and a read/write
                      HOOK for Python-side scripting)
```

### The hook (middleware, on SimPlant — not on I2CBus)

`SimPlant::read()`/`write()` structure:
```
int SimPlant::read(addr, data, len)  { return readHook_  ? readHook_(addr, data, len)  : defaultRead(addr, data, len); }
int SimPlant::write(addr, data, len) { return writeHook_ ? writeHook_(addr, data, len) : defaultWrite(addr, data, len); }
```
`defaultRead`/`defaultWrite` are SimPlant's own state machine (parse duty,
return live encoder/pose). A registered hook is called *instead*, and may call
the default back for pass-through — `defaultRead`/`defaultWrite` never re-enter
the hook, so no recursion.

ctypes surface (Python does all scripting):
```
sim_set_read_hook(h, cb, ctx)   / sim_set_write_hook(h, cb, ctx)   // register / clear (cb=NULL)
sim_default_read(h, addr, buf, len) / sim_default_write(h, addr, buf, len)  // the pass-through the Python cb calls
```
A Python read hook fills/inspects the buffer and, for pass-through, calls
`sim_default_read(...)` (which runs the plant's real response); a write hook
calls `sim_default_write(...)` to apply, or returns without it to swallow.

Interface membership (what firmware actually calls — Explore-confirmed):
`I2CBus` = `write`, `read`, `clearanceSafetyNetCount()` (robot_loop.cpp:124),
plus any diagnostic (`dumpRecent`/`setLogging`) reached by a live command
handler — grep at implementation time; add only those. Everything else
(txnCount/errCount/clear/irqGuard/reentry/log) is test-only or real-bus
internal and lives on the concrete classes, not the interface.

Source placement rule (kills the CMake `FILTER EXCLUDE` hack): `source/`
contains **only** interfaces + ARM implementations; **all** host/sim/test
implementations live under `tests/`. The ARM CMake glob of `source/**` then
never sees a host `.cpp`, so no `#ifdef` and no exclude filter are needed.

## Work — staged

Note on CI during the transition: the 13 register-scripting sim-unit tests are
the ones being *replaced*. They go red the moment the scripted fake is removed
(Stage 1) and come back green as Python hook tests once SimPlant + the ctypes
hook ABI exist (Stage 4). The ARM firmware build and every non-scripting test
stay green throughout. No temporary scripted-bus scaffold is built.

### Stage 1 — Split I2CBus (firmware, behavior-preserving)
- `source/devices/i2c_bus.h`: reduce to the pure abstract `I2CBus`
  (virtual dtor + `write`/`read`/`clearanceSafetyNetCount` = 0, + any
  command-reached diagnostic). Delete the `#ifdef` forks, the scripted
  surface, the `Responder` seam I added, and all private members.
- New `source/devices/microbit_i2c_bus.{h,cpp}`: `MicroBitI2CBus : I2CBus`
  holding the current real machinery verbatim (MicroBitI2C member, clearance
  timers, reentry/IRQ guard, slot table, stats, ring log). Move
  `i2c_bus.cpp`'s body here; delete `i2c_bus.cpp` and `i2c_bus_host.cpp`.
- `source/main.cpp:99`: `Devices::I2CBus bus(uBit.i2c)` → `MicroBitI2CBus`.
- `CMakeLists.txt:300`: drop the `i2c_bus_host.cpp` FILTER-EXCLUDE line.
- **Verify:** ARM firmware builds (`python build.py --fw-only`); no `#ifdef
  HOST_BUILD` in i2c files. (13 sim-unit tests now red — expected, Stage 4.)

### Stage 2 — SimPlant: the one honest simulator bus
- New `tests/_infra/sim/sim_plant.{h,cpp}`: `SimPlant : Devices::I2CBus`.
  - Owns two `TestSim::WheelPlant` + one `TestSim::OtosPlant` (reuse the
    physics — good models; SimPlant owns the *protocol*).
  - `defaultWrite()`: dispatch by address. Motor `0x10`: parse the 8-byte
    Nezha frame — `0x60` → set that port's duty (dir+speed → [-1,1]) **off the
    wire** (no `appliedDuty()` back-channel); `0x46` → remember selected port.
    OTOS `0x17`: track register pointer + swallow init writes. Color/line: NAK.
  - `defaultRead()`: motor → selected port's accumulated encoder (4-byte LE
    tenths-of-mm); OTOS → product-ID `0x5F` or 12-byte pose burst per register
    pointer.
  - `read()`/`write()`: the hook wrappers (see architecture) around
    `defaultRead`/`defaultWrite`.
  - `tick(dt)`: step both wheel plants from their parsed duty, step OTOS,
    accumulate encoder counts. Called once per cycle by the harness.
  - Fault knobs as plain methods (reuse WheelPlant setDisconnected/
    freezePosition/setDropoutRate; add OTOS noise/drift) + `setReadHook`/
    `setWriteHook`. The ctypes "device conditions"/scripting surface — NOT on
    I2CBus.
- New `tests/_infra/sim/sim_harness.h` (replaces `live_sim.h`): constructs the
  real `App::RobotLoop` graph with a `SimPlant` in the `I2CBus&` slot;
  `boot()`; `step(n)` = `plant.tick(dt)` + `robotLoop.cycle()`; inject via
  `serialLink` + `armor*Command`; drain telemetry; expose true pose.
- **Delete:** `tests/sim/support/sim_api.{h,cpp}` (+ DutyPredictor),
  `tests/_infra/sim/live_sim.h`, the `Responder` seam.
- Migrate the 4 whole-robot scenario tests
  (`tests/sim/system/{sim_api,profiled_motion,scripted_twist_demo,
  faults/fault_knobs}_harness.cpp`) onto `SimPlant`/`sim_harness`.
- **Verify:** migrated system tests pass; a standalone C++ driver shows a
  straight twist stays straight (divergence bug gone).

### Stage 3 — ctypes ABI + host Python + GUI tour
- `tests/_infra/sim/sim_ctypes.cpp`: rewrite over `sim_harness`/`SimPlant`
  — create/destroy/step/inject_twist/inject_stop/drain_tlm/true-pose, the
  fault-condition setters, AND the hook surface (`sim_set_read_hook`/
  `sim_set_write_hook` + `sim_default_read`/`sim_default_write`). `build.py`:
  compile `sim_plant.cpp` + reuse the real `source/` graph.
- Delete dead `host/robot_radio/io/sim_conn.py`. New
  `host/robot_radio/io/sim_loop.py`: a `TwistTransport`-shaped object
  (`twist`/`stop`/`read_pending_binary_tlm_frames`) over the new ABI, with a
  wall-clock tick thread stepping the sim and delivering telemetry+truth; and
  a Python `set_read_hook`/`set_write_hook` wrapper (ctypes `CFUNCTYPE`) that
  hands the callback the addr+buffer and a `pass_through()` helper calling
  `sim_default_read/write`.
- Rewire `SimTransport` (`host/robot_radio/testgui/transport.py`) onto
  `sim_loop`: add `.protocol`, `suspend_telemetry_reader`/
  `resume_telemetry_reader`; repoint `sim_prefs.py`'s knobs at SimPlant's
  fault setters. Un-gate the Tour buttons for Sim (`__main__.py:730-757` +
  the `_TOUR_SIM_TOOLTIP` gating in `_on_connect`).
- **Verify:** headless — run Tour 1 through `sim_loop`, assert every leg runs
  and closure is finite/small; GUI — `just testgui`, Connect (Sim), press
  **Tour 1**, watch the trace draw.

### Stage 4 — Migrate the 13 register-level unit tests to Python hooks
- Rewrite each former `tests/sim/unit/*_harness.cpp` (+ `test_*.py`) and
  `tests/sim/plant/*` as a **pure Python** test: boot the ctypes sim, register
  a read/write hook that injects the specific register scenario (wrong OTOS
  product ID, motor NAK, sensor-probe absence, boot-detection sequence), step,
  and assert on resulting telemetry / device state. Delete the C++ harnesses.
- **Verify:** `uv run python -m pytest tests/sim` fully green again.

### Stage 5 — Clock/Sleeper purification (same pattern)
- `source/devices/clock.h` → pure `Clock`/`Sleeper` interfaces.
- `source/devices/microbit_clock.{h,cpp}`: real (`system_timer_current_time_us`
  / `fiber_sleep` / `schedule`). Delete `clock_real.cpp`/`clock_host.cpp`,
  drop the CMake exclude line.
- `tests/_infra/sim/sim_clock.{h,cpp}` (or reuse in sim_harness):
  `SimClock`/`SimSleeper` (steppable counter + sleep/yield counters).
- `main.cpp` + the sim harness updated to the new impls.
- **Verify:** ARM builds; full pytest gate green; `grep -rn HOST_BUILD source/`
  returns nothing in `devices/`.

## Notes / risks
- Making `write`/`read` virtual adds one vtable indirection per I2C
  transaction on the ARM hot path — negligible vs the microsecond transaction
  itself; the codebase already tolerates virtual dispatch (`App::Transport`,
  `MotorArmor`).
- `comms.h`'s `#ifndef HOST_BUILD` around `SerialTransport`/`RadioTransport`
  is a smaller, separate residue (adapters over the already-pure
  `App::Transport`); optional follow-up — extract them to an ARM-only `.cpp`
  to drop that guard too. Not required for the tour.
- This is a large, multi-file refactor of core firmware + the whole sim test
  suite. Stages 1–2 are behavior-preserving and keep every gate green before
  any simulator rework begins.

## Verification (end to end)
1. `python build.py --fw-only` — ARM firmware still builds (Stages 1 & 5).
2. `uv run python -m pytest tests/sim` — full gate green after each stage.
3. `grep -rn "HOST_BUILD" source/` — returns nothing in devices/ (goal).
4. Standalone: straight twist → heading stays ~0 (divergence fixed).
5. Headless Tour 1 via `sim_loop` → all legs run, closure finite/small.
6. `just testgui` → Connect (Sim) → **Tour 1** → trace draws on the canvas.
