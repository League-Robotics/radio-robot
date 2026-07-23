# Sim (`src/sim`) — Host-Build Firmware Simulator

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-21 · **Status:** in-flux

---

## 1. Purpose

`src/sim/` compiles the real `src/firm/` control loop into a host shared
library (`build/libfirmware_host.dylib`) and drives it from Python over a
small `extern "C"` ABI. It exists so that "does the firmware behave
correctly" can be answered on a developer laptop, with no hardware and no
serial/radio link — by running the identical `App::RobotLoop` code that
ships to the robot against a simulated I2C bus instead of a real one.
There is **one** sim object shared by both `src/tests/sim/`'s pytest
suite and the TestGUI's Sim transport: the exact same command-in/
telemetry-out path a serial or radio robot presents, so a test drives
precisely what a developer watching the GUI drives — never two divergent
sim implementations that could silently diverge from each other.

## 2. Orientation

| File | Role |
|---|---|
| `sim_plant.h` / `sim_plant.cpp` | `SimPlant` — the one honest simulated I2C bus. Owns the wire *protocol* (Nezha `0x60` duty write / `0x46` encoder select+read, OTOS register map); physics is delegated to `WheelPlant`×2 + `OtosPlant` (`src/tests/sim/plant/`). |
| `sim_harness.h` | `SimHarness` — composition root: wires the **real** `App::RobotLoop` firmware graph (the same modules `src/firm/main.cpp` constructs — two `Devices::NezhaMotor`/`MotorArmor` pairs, `Devices::Otos`/`ColorSensorLeaf`/`LineSensorLeaf`, `App::Comms`/`Telemetry`/`Deadman`/`Drive`/`Odometry`/`Preamble`) against `SimPlant`, a fake clock, and two `TestSupport::FakeTransport` links (serial + radio). |
| `sim_ctypes.cpp` | `extern "C"` ABI over `SimHarness`/`SimPlant` — every export is a thin call-through so Python's `ctypes` can drive the sim with no binding generator. |
| `sim_clock.h` / `sim_clock.cpp` | `SimClock`/`SimSleeper` — steppable virtual time (`Devices::Clock`/`Sleeper` implementations); nothing advances the clock except an explicit `advanceMicros()` call from `SimHarness::step()`. |
| `CMakeLists.txt` | Builds `firmware_host` from these files + `src/firm/` (HOST_BUILD) + `src/tests/sim/{plant,support}`. |

The Python side lives in `src/host/robot_radio/`:

- `io/sim_loop.py` — `SimLoop`: loads the dylib, owns the tick thread,
  implements `planner/executor.py`'s `TwistTransport` protocol
  (`twist()`/`stop()`/`read_pending_binary_tlm_frames()`) directly
  against the current minimal firmware's command surface.
- `testgui/transport.py` — `SimTransport`: the TestGUI transport backend
  that wraps a `SimLoop`.

**115-002/115-003/115-006 (gut-to-minimal-firmware S1 motion-stack
excision) rewired the composition root, not just the walkthrough.**
`SimHarness` no longer constructs or references `App::Pilot`/
`Motion::Executor`/`App::HeadingSource` — every accessor that only
existed to reach into them (`configurePlanner()`/`plannerConfig()`,
`pilotQueueDepth()`/`pilotActiveId()`/`pilotState()`,
`headingSourceIsOtos()`, `debugHeadingLead()`,
`setLeadCompensation()`/`setYawRateMax()`/`setDistanceKp()`,
`plannedRefLeft()`/`plannedRefRight()`) is gone, and the matching
`sim_ctypes.cpp` exports (`sim_configure_planner`,
`sim_read_planner_config`, `sim_set_lead_compensation`,
`sim_set_yaw_rate_max`, `sim_debug_heading_lead`) were deleted with them
— their old symbol names survive only as tombstone comments in
`sim_ctypes.cpp`, not as callable exports. `SimHarness::injectMove()` and
a dedicated `sim_inject_move` ctypes export do not exist either: sprint
109's `Move` message was deleted (`envelope.proto`'s `CommandEnvelope`
field 20 is `reserved`, not reused — sprint 116's planned MOVE protocol
reintroduces a `Move`-shaped arm at a fresh number, never 20). The
current command surface this simulator drives is **TWIST + STOP +
CONFIG{motor,otos} + deadman only** — see
[`../firm/DESIGN.md`](../firm/DESIGN.md) and
[`../firm/app/DESIGN.md`](../firm/app/DESIGN.md) for the firmware side of
the same statement.

Everything below traces one concrete TWIST round trip, with file/line
references current as of 2026-07-21 (function names are the durable
anchor if lines drift):

> **The TestGUI's "Test S — drive 700mm (unmanaged)" button sends a
> TWIST command. The robot drives forward in the sim. The encoder
> velocities appear back in the TestGUI's telemetry panel.**

There are, as of this review, **two visually similar "Test S" paths in
the TestGUI with opposite functional status** — see §6. This walkthrough
follows the one that actually works against the current firmware.

---

## 0. The big picture

```
 TestGUI (Qt main thread)                 Python                              C++ (one dylib)
┌──────────────────────────┐  ┌────────────────────────────────┐  ┌─────────────────────────────────────┐
│ "Test S (unmanaged)"      │  │ SimTransport.run_unmanaged()    │  │ sim_inject_twist(v_x, omega,         │
│  button                   │─▶│  └─ SimLoop.twist(v_x, omega,   │─▶│                  duration, corr_id)  │
│                           │  │      duration_ms)                │  │  └─ SimHarness::injectTwist()       │
│                           │  │                                  │  │      └─ FakeTransport inbound FIFO  │
│                           │  │                                  │  │                                     │
│                           │  │                                  │  │ SimLoop tick thread: sim_step(n)    │
│                           │  │                                  │  │  └─ SimHarness::step():             │
│                           │  │                                  │  │      SimPlant::tick(dt)   (physics) │
│                           │  │                                  │  │      SimClock advance               │
│                           │  │                                  │  │      App::RobotLoop::cycle():       │
│                           │  │                                  │  │        comms_.pump → handleTwist    │
│                           │  │                                  │  │        Drive::setTwist + Deadman    │
│                           │  │                                  │  │        drive_.tick → NezhaMotor PID │
│                           │  │                                  │  │        → 0x60 write ────────────────┼─▶ SimPlant duty
│                           │  │                                  │  │        0x46 read ◀───────────────────┼── WheelPlant pos
│                           │  │                                  │  │        updateTlm → Telemetry::emit  │
│ telemetry panel           │  │ SimLoop._drain_tlm_into_queue    │  │        → Comms::sendReply           │
│  vel: L +150 R +150 mm/s  │◀─│  └─ TLMFrame.from_pb2 (vel=)     │◀─│        → FakeTransport sent_ FIFO   │
│  (Qt bridge, main thread) │  │      on_telemetry callback       │  │ sim_drain_tlm() drains sent_        │
└──────────────────────────┘  └────────────────────────────────┘  └─────────────────────────────────────┘
```

Two FIFOs inside `TestSupport::FakeTransport`
(`src/tests/sim/support/fake_transport.h:42-76`) are the *entire*
host↔firmware boundary — armored `*B<base64>` text lines in both
directions, byte-identical to what a serial port would carry.

---

## 1. Button click → transport call

**File: `src/host/robot_radio/testgui/transport.py`**

`SimTransport.run_unmanaged(distance_mm)` (transport.py, class
`SimTransport`) goes "straight through `twist` → `Drive::setTwist` — NO
Motion::Executor/Ruckig" (its own docstring) — this is the deliberately
minimal path that matches what the current firmware actually understands:

```python
duration_ms = distance_mm / _UNMANAGED_SPEED * 1000   # 700 / 150 * 1000 ≈ 4667 ms
self._loop.twist(v_x, omega, duration_ms)
```

Contrast this with `_dispatch()`'s `_MOTION_VERBS = ("D", "RT")` branch
(the "managed" Test S/T path), which calls `_run_motion_async()` →
`planner.tour.parse_tour()`/`run_tour()` → `SimLoop.move()` — see §6,
this path is currently non-functional against the present wire schema
and must not be used as a reference for how a TWIST-based test should
work.

---

## 2. SimLoop injects the twist

**File: `src/host/robot_radio/io/sim_loop.py`**

```python
def twist(self, v_x: float, omega: float, duration: float) -> int:
    corr_id = self._next_corr_id()
    self._run_or_enqueue(lambda: self._lib.sim_inject_twist(
        self._handle, ctypes.c_float(v_x), ctypes.c_float(omega),
        ctypes.c_float(duration), ctypes.c_uint32(corr_id)))
    return corr_id
```

`_run_or_enqueue()` marshals the ctypes call onto the sim tick thread —
the only thread that ever touches the ctypes handle.

**File: `src/sim/sim_ctypes.cpp` → `src/sim/sim_harness.h`**

`sim_inject_twist()` → `SimHarness::injectTwist()` → a convenience
wrapper over `TestSupport::armorTwistCommand()` that armors a
`msg::CommandEnvelope{cmd_kind: TWIST}` and calls the same
`injectCommand()` → `serialLink_.enqueueInbound(armoredLine)` path as
every other inbound command — the inbound FIFO of the serial-side
`FakeTransport`.

The command is now sitting "on the wire", exactly as if a serial port had
received the line.

---

## 3. The tick thread steps the sim; the firmware loop consumes the command

**File: `src/host/robot_radio/io/sim_loop.py`**

Every tick-thread iteration calls `self._lib.sim_step(self._handle,
cycles)`, paced to one 40 ms sim cycle (`kCycleDtUs = 40000`) per
wall-clock interval at 1× speed factor.

> Note (118 ticket 003, resolved — see §8's own former Open Question):
> `SimHarness::kCycleDtUs` is **derived from firmware's own
> `App::RobotLoop::kCycle`** (`kCycleDtUs = App::RobotLoop::kCycle *
> 1000`, `src/sim/sim_harness.h`, with a `static_assert` pinning the two
> together so they cannot drift apart independently again), not an
> independently-hardcoded literal. Both are 40 ms/~25 Hz. `sim_step(1)`
> now corresponds to EXACTLY one 40 ms firmware cycle — "N `sim_step()`
> calls == N firmware cycles" is literally true, not approximately true
> modulo a translation factor. Before this ticket, `kCycleDtUs` sat at a
> hand-picked 50 ms (2.5× the firmware's then-regressed `kCycle=20ms`),
> chosen only to dodge `Devices::NezhaMotor`'s write-rate throttle at
> that shorter cycle — never a deliberate simulation-fidelity choice; see
> `clasi/issues/sim-cycle-must-match-firmware-period.md` for the full
> history. The throttle itself now carries its own jitter margin
> (`nezha_motor.cpp`'s `kMinWriteIntervalUs`) so an on-schedule write at
> exactly the (now-equal) cycle/throttle period does not need a coarser
> sim step to avoid it.

**File: `src/sim/sim_harness.h`**

`sim_step()` → `SimHarness::step()`, whose ordering is the harness's one
invariant — plant physics **before** the loop reads it:

```cpp
void step(int cycles = 1) {
  for (int i = 0; i < cycles; ++i) {
    plant_.tick(static_cast<float>(kCycleDtUs) / 1e6f);  // physics first
    clock_.advanceMicros(kCycleDtUs);                     // then virtual time
    robotLoop_.cycle();                                   // then the real firmware
    ++cycleCount_;
  }
}
```

**File: `src/firm/app/robot_loop.cpp` — one `cycle()`**

The cycle is the same schedule described in
[`../firm/app/DESIGN.md`](../firm/app/DESIGN.md) §2/§4 — left-motor
request/settle/collect/PID, comms pump + command dispatch, telemetry
emit, right-motor request/settle/collect/PID, deadman check, trailing
perception+odometry+pace block. The command-relevant slice:

1. A `runAndWait` settle block calls `comms_.pump(cmd)` — decodes at
   most one frame off the wire into the cycle-local `cmd`.
2. A second `runAndWait` settle block calls `processMessage(cmd)`, which
   switches on `cmd.env.cmd_kind`:
   ```cpp
   case msg::CommandEnvelope::CmdKind::TWIST:  handleTwist(cmd.env);  break;
   case msg::CommandEnvelope::CmdKind::CONFIG: handleConfig(cmd.env); break;
   case msg::CommandEnvelope::CmdKind::STOP:   handleStop(cmd.env);   break;
   ```
3. `handleTwist()`:
   ```cpp
   void RobotLoop::handleTwist(const msg::CommandEnvelope& env) {
     if (!configured_) { tlm_.ack(env.corr_id, ERR_NOT_CONFIGURED); return; }
     drive_.setTwist(env.cmd.twist.v_x, 0.0f, env.cmd.twist.omega);
     deadman_.arm(env.cmd.twist.duration);
     driving_ = true;
     tlm_.ack(env.corr_id, 0);
   }
   ```
   The immediate `ack(env.corr_id, 0)` rides the next telemetry frame's
   single ack slot (`flags` bit 5, `ack_corr`/`ack_err`) — see
   [`../firm/messages/DESIGN.md`](../firm/messages/DESIGN.md) and
   [`../firm/app/DESIGN.md`](../firm/app/DESIGN.md) §4.
4. The deadman check runs in the same settle block right after
   `processMessage()`: `deadman_.expired()` forces `drive_.stop()` and
   `driving_ = false` if the armed duration has elapsed with no
   refreshing command.

---

## 4. Twist → per-wheel velocity targets → duty write

- `Drive::setTwist(v_x, 0, omega)` stores the staged target;
  `Drive::tick()` (called every cycle, from the top of `cycle()`) splits
  it into wheel velocities via `BodyKinematics::inverse()` and calls
  `left_.setVelocity(vL)`/`right_.setVelocity(vR)` on the two
  `MotorArmor`-wrapped `NezhaMotor` leaves. For a straight 700 mm leg at
  150 mm/s, both targets ramp to ±150 mm/s.
- Each `NezhaMotor::tick()`'s velocity PID computes a duty from target vs.
  measured velocity, and `armoredWrite()` (the `MotorArmor` policy gate)
  issues the vendor duty frame:
  ```cpp
  uint8_t buf[8] = { 0xFF, 0xF9, port, direction, 0x60, speed, 0xF5, 0x00 };
  bus_.write(0x10 << 1, buf, 8, ...);
  ```

**The sim boundary:** `bus_` here **is** the `SimPlant` (constructor
injection into `Devices::NezhaMotor`, mirroring the real ARM build's
`Devices::MicroBitI2CBus`). `SimPlant::write()` dispatches to
`handleMotorWrite()`, which parses the same 8-byte frame the real Nezha
brick would: `cmd == 0x60` → `duty = ±speed/100` stored into
`leftDuty_`/`rightDuty_` by `port`. **No prediction, no back-channel** —
the plant reacts only to bytes actually written.

On the next `SimHarness::step()`, `SimPlant::tick(dt)` integrates the
physics — delegated to `src/tests/sim/plant/`'s `WheelPlant`/`OtosPlant`,
never reimplemented here:

```cpp
void SimPlant::tick(float dt) {
  left_.step(leftDuty_, dt);
  right_.step(rightDuty_, dt);
  otos_.step(fwdSignL * left_.position(), fwdSignR * right_.position(), dt);
}
```

`WheelPlant`/`OtosPlant` are a first-order duty→velocity→position model
with fault-injection knobs (motor disconnect, encoder wedge/dropout,
encoder scale/slip/tick-quantization error, OTOS drift/raw-scale error)
that `SimPlant`'s own `set*` methods (and the matching
`sim_set_wheel_*`/`sim_set_enc_*`/`sim_set_otos_*` ctypes exports)
expose to Python test code — see
[`../tests/DESIGN.md`](../tests/DESIGN.md) §2.

---

## 5. Encoder read-back

Split-phase, exactly as on hardware (see
[`../firm/devices/DESIGN.md`](../firm/devices/DESIGN.md) §4):

1. **Request** — `NezhaMotor::requestSample()` writes the encoder-select
   frame `{0xFF,0xF9,port,0x00,0x46,0x00,0xF5,0x00}`. `SimPlant`'s
   `handleMotorWrite()` records `selectedPort_ = port` for the same
   address, since the encoder-select opcode (`0x46`) and the duty-write
   opcode (`0x60`) share the motor's wire address.
2. **Collect** — `NezhaMotor::collectEncoder()` issues a 4-byte read.
   `SimPlant::handleMotorRead()` answers with the selected wheel's
   position, converted mm → raw counts (`kEncoderCountsPerMm = 1.4187`)
   and packed little-endian in tenths — the same wire format the real
   brick uses.
3. **Convert + estimate velocity** — `NezhaMotor::tick()` converts the
   raw count to mm and updates `filteredVelocity_` via the freshness-gated
   EMA/line-fit estimator (see
   [`../firm/devices/DESIGN.md`](../firm/devices/DESIGN.md) §4).
   `velocity()` is the "encoder velocity" the GUI displays.

---

## 6. Telemetry out and back into Python

`RobotLoop`'s `updateTlm()`/`Telemetry::emit()` stage and send the
primary frame exactly as described in
[`../firm/app/DESIGN.md`](../firm/app/DESIGN.md) §4 — `EncoderReading`
per wheel (position + velocity + sample time), `OtosReading` (position +
heading + v_x/v_y/omega + burst time), the single `flags` word, the
single ack slot, packed line/color words — every 40 ms firmware cycle,
which (118 ticket 003) is now also exactly every `sim_step()` call — see
§3's note on how `kCycleDtUs` is derived from firmware's own `kCycle` so
the two are no longer two different step sizes. `Comms::sendReply()`
armors and broadcasts on both
`FakeTransport`s; `FakeTransport::send()` appends to its `sent_` capture.

**File: `src/sim/sim_harness.h` / `src/sim/sim_ctypes.cpp`**

`SimHarness::drainRawTelemetry()` returns every not-yet-drained armored
line from `serialLink_.sent()` (its own drain index, separate from the
higher-level `drainTelemetry()`'s index). `sim_drain_tlm()` newline-joins
them for the caller's buffer.

**File: `src/host/robot_radio/io/sim_loop.py`**

Every tick-thread iteration, right after `sim_step()`,
`_drain_tlm_into_queue()`: calls `sim_drain_tlm()`, dearmors + decodes
each line with the same `pb2` codec a real robot's replies go through,
builds a `TLMFrame` via `TLMFrame.from_pb2()`
(`src/host/robot_radio/robot/protocol.py`), pushes it onto the bounded
`_tlm_queue` (what `read_pending_binary_tlm_frames()` polls) and delivers
it to `self.on_telemetry(frame)` — the push path the TestGUI's telemetry
panel renders from (thread-hopped onto the Qt main thread via a queued
signal).

`sim_cmd_vel_left()`/`sim_cmd_vel_right()` (reading
`NezhaMotor::velocityTarget()` directly, not via the wire) are a
sim-only diagnostic extra ("Path B") — the *commanded* per-wheel
velocity is not on the primary wire frame (envelope budget), so it is
stamped onto the Python-side frame separately, feeding the
commanded-vs-actual wheel-speed graph. It is read-only diagnostic
telemetry, never plant input (see the "No back-channel" invariant
below).

---

## Threads and ownership (who runs what)

| Thread | Created by | Runs |
|---|---|---|
| Qt main thread | TestGUI `__main__.py` | Button handlers, `_dispatch()`, all widget updates |
| `sim-direct-motion` | `SimTransport._run_motion_async` | The MANAGED path's `parse_tour`/`run_tour` (currently broken — §6) |
| sim tick thread | `SimLoop.connect()` | **Sole owner of the ctypes handle**: `sim_step`, command-queue drain, `sim_drain_tlm`, `on_telemetry`/`on_truth` callbacks |

The ctypes handle (`SimHarness*`) is not thread-safe; every call is
either executed by the tick thread (fire-and-forget via
`_run_or_enqueue`) or synchronously round-tripped onto it.

## Invariants worth keeping

1. **Plant ticks before the loop reads it** — `SimHarness::step()`'s
   fixed order; reversing it makes every sensor read one cycle stale.
2. **No back-channel** — `SimPlant` learns duty only from parsed `0x60`
   frames, never from `NezhaMotor::appliedDuty()` directly. The one
   sanctioned exception is the read-only `cmd_vel` stamp (§6's "Path B"),
   which is diagnostic telemetry, not plant input.
3. **One decode path** — sim telemetry is dearmored/parsed by the same
   `pb2` codec and `TLMFrame.from_pb2()` a real robot's replies use;
   tests and the TestGUI therefore exercise the identical wire format.
4. **One firmware** — `App::RobotLoop` and everything below it compiles
   unmodified into the dylib; the sim substitutes only the `I2CBus`
   (`SimPlant`), `Clock`/`Sleeper` (`SimClock`), and `Transport`
   (`FakeTransport`) seams.

## 7. Interfaces

### Exposes

- **`extern "C"` ABI (`sim_ctypes.cpp`):** lifecycle
  (`sim_create`/`sim_destroy`/`sim_booted`/`sim_cycle_count`/
  `sim_firmware_version`), stepping (`sim_step`), command injection
  (`sim_inject_twist`/`sim_inject_stop`/`sim_inject_command`), telemetry
  (`sim_drain_tlm`), true-pose readback (`sim_true_x/y/h`,
  `sim_set_true_pose`), fault-injection knobs
  (`sim_set_wheel_disconnected/freeze/dropout_rate`,
  `sim_set_otos_drift`, `sim_set_enc_scale_err/tick_quant/slip`,
  `sim_set_otos_raw_scale_err`), config load
  (`sim_configure_motor`/`sim_read_motor_config`), read/write hooks
  (`sim_set_read_hook`/`sim_set_write_hook`/`sim_default_read`/
  `sim_default_write`), and diagnostic commanded-velocity readback
  (`sim_cmd_vel_left/right`, `sim_set_pid_enabled`). `sim_inject_move`
  and every planner/pilot/heading-source accessor are gone (see §2).
- **`libfirmware_host.dylib`** itself — the build artifact `SimLoop`
  loads; see [`../tests/DESIGN.md`](../tests/DESIGN.md) for the CMake
  target that produces it.

### Consumes

- **`src/firm/` (compiled `-DHOST_BUILD`)** — the real firmware graph;
  see [`../firm/DESIGN.md`](../firm/DESIGN.md) §4 "Two build targets, one
  tree."
- **`src/tests/sim/plant/`'s `WheelPlant`/`OtosPlant`** and
  **`src/tests/sim/support/`'s `FakeTransport`/`wire_test_codec.h`** —
  physics and transport doubles this directory's own code never
  reimplements; see [`../tests/DESIGN.md`](../tests/DESIGN.md) §2.

## 8. Open Questions / Known Limitations

- **Two "Test S"-shaped TestGUI paths currently have opposite functional
  status**, and this is easy to trip over: `SimTransport.run_unmanaged()`
  (straight `twist()`, no planner) works against the current firmware;
  the `_MOTION_VERBS`/`_run_motion_async()` → `planner.tour.run_tour()`
  → `SimLoop.move()` path is code-reachable from the GUI but throws at
  runtime, because it builds an `envelope_pb2.Move` message that no
  longer exists in the current `protos/envelope.proto` schema (deleted
  115-003; sprint 116 reintroduces a differently-shaped `Move` arm at a
  fresh field number). `SimLoop.move()` itself is not deleted — it is
  dead-but-reachable code, left in place because the host-side
  planner/tour machinery it serves is DORMANT-by-stakeholder-decision
  (see [`../host/robot_radio/DESIGN.md`](../host/robot_radio/DESIGN.md)),
  not because it still works. Do not use the managed Test S/T path as a
  reference for "how TWIST works" — use `run_unmanaged()`.
- **`src/tests/sim/plant/wheel_plant.h`'s own header comment describes a
  stale "leaf-getter-driven" design** (reading
  `Devices::NezhaMotor::appliedDuty()` directly) that predates and no
  longer matches how `SimPlant` actually drives it today (explicit
  wire-parsed duty passed into `step()`) — a doc-only staleness item in
  that file, not fixed as part of this review (out of this doc's own
  directory scope).
- **`sim_ctypes.cpp`'s own top-of-file comment undercounts its current
  export list** (says "19 exports"; the actual list is longer — includes
  `sim_cmd_vel_left/right`, `sim_set_pid_enabled`,
  `sim_firmware_version`) — a doc-only staleness item in that file, not
  fixed as part of this review.
