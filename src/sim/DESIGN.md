# src/sim — Simulator Design & End-to-End Data Flow

This directory holds the **host-build firmware simulator**: the real
`src/firm/` control loop compiled into a shared library
(`build/libfirmware_host.dylib`) and driven from Python over a small
`extern "C"` ABI. There is **one** sim object shared by tests and the
TestGUI — the exact same command-in / telemetry-out path a serial or radio
robot presents, so a test drives precisely what the GUI drives.

> **STALE (115-002/115-003/115-006, gut-to-minimal-firmware S1
> motion-stack excision), not yet rewritten.** The worked example below
> ("Test S — drive 700mm") and §§4/5/8 walk through the pre-gut MOVE
> command path — `handleMove()`, `Motion::Cmd`/`Motion::fromMove()`,
> `Pilot::enqueue()`/`Pilot::tick()`, `Motion::Executor` — all DELETED
> wholesale this sprint, along with `App::Pilot`/`App::HeadingSource`/
> `vendor/ruckig`. S1's minimal firmware has no MOVE command at all: the
> command surface is TWIST+STOP+CONFIG{motor,otos}+deadman only (see
> `src/firm/app/DESIGN.md`'s own 115-005 deletion note for the current,
> accurate picture). `sim_harness.h`'s own composition root was updated for
> this (ticket 115-006, "sim lockstep"); this walkthrough doc was not — the
> STRUCTURAL flow it describes (TestGUI → SimTransport → SimLoop →
> `sim_inject_command()` → `SimHarness::step()` → `App::RobotLoop::cycle()`
> → `Comms::sendReply()` → `sim_drain_tlm()` → TestGUI) is still
> essentially correct for a TWIST; only the MOVE-specific middle (dispatch
> case, `Pilot`/`Motion::Executor` staging, completion-event draining) is
> gone. Full rewrite tracked as a follow-up, not done as part of 115-009
> (which named `src/firm/*/DESIGN.md` specifically, not this file).

| File | Role |
|---|---|
| `sim_plant.h` / `sim_plant.cpp` | `TestSim::SimPlant` — the one honest simulated I2C bus. Owns the wire *protocol* (Nezha `0x60`/`0x46` frames, OTOS register map); physics is delegated to `WheelPlant`×2 + `OtosPlant` (`src/tests/sim/plant/`). |
| `sim_harness.h` | `TestSim::SimHarness` — composition root: wires the **real** `App::RobotLoop` firmware graph against `SimPlant`, a fake clock, and two `TestSupport::FakeTransport` links. |
| `sim_ctypes.cpp` | `extern "C"` ABI over `SimHarness`/`SimPlant` — every export is a thin call-through so `ctypes` can drive the sim without a binding generator. |
| `sim_clock.h` / `sim_clock.cpp` | `TestSim::SimClock`/`SimSleeper` — steppable virtual time (`Devices::Clock`/`Sleeper` impls). |
| `CMakeLists.txt` | Builds `firmware_host` from these files + `src/firm/` + `src/tests/sim/{plant,support}`. |

The Python side lives in `src/host/robot_radio/`:

- `io/sim_loop.py` — `SimLoop`: loads the dylib, owns the tick thread,
  implements the `TwistTransport`/`MoveTransport` protocol
  (`twist()`/`stop()`/`move()`/`read_pending_binary_tlm_frames()`).
- `testgui/transport.py` — `SimTransport`: the TestGUI transport backend
  that wraps a `SimLoop`.

Everything below traces one concrete round trip, with file/line
references (line numbers as of 2026-07-18; function names are the durable
anchor if lines drift):

> **You click "Test S — drive 700mm" in the TestGUI. The robot drives
> forward 700 mm in the sim. The encoder velocities appear back in the
> TestGUI's telemetry panel.**

---

## 0. The big picture

```
 TestGUI (Qt main thread)                         Python                          C++ (one dylib)
┌───────────────────────────┐   ┌────────────────────────────────┐   ┌─────────────────────────────────────┐
│ Test S button             │   │ SimTransport._dispatch("D…")   │   │ sim_inject_command()                │
│  └─ command("D 150 150    │──▶│  └─ _run_motion_async (thread) │──▶│  └─ SimHarness::injectCommand()     │
│      700")                │   │      └─ parse_tour / run_tour  │   │      └─ FakeTransport inbound FIFO  │
│                           │   │          └─ SimLoop.move()     │   │                                     │
│                           │   │              *B<base64 Move>   │   │ SimLoop tick thread: sim_step(n)    │
│                           │   │                                │   │  └─ SimHarness::step():             │
│                           │   │                                │   │      SimPlant::tick(dt)   (physics) │
│                           │   │                                │   │      SimClock advance               │
│                           │   │                                │   │      App::RobotLoop::cycle():       │
│                           │   │                                │   │        Comms::pump → handleMove     │
│                           │   │                                │   │        Pilot/Executor → Drive       │
│                           │   │                                │   │        NezhaMotor PID → 0x60 write ─┼─▶ SimPlant duty
│                           │   │                                │   │        0x46 read ◀──────────────────┼── WheelPlant pos
│                           │   │                                │   │        updateTlm → Telemetry::emit  │
│ telemetry panel           │   │ SimLoop._drain_tlm_into_queue  │   │        → Comms::sendReply           │
│  vel: L +150 R +150 mm/s  │◀──│  └─ TLMFrame.from_pb2 (vel=)   │◀──│        → FakeTransport sent_ FIFO   │
│  (Qt bridge, main thread) │   │      on_telemetry callback     │   │ sim_drain_tlm() drains sent_        │
└───────────────────────────┘   └────────────────────────────────┘   └─────────────────────────────────────┘
```

Two FIFOs inside `TestSupport::FakeTransport`
(`src/tests/sim/support/fake_transport.h:42-76`) are the *entire*
host↔firmware boundary — armored `*B<base64>` text lines in both
directions, byte-identical to what a serial port would carry.

---

## 1. Button click → wire command string

**File: `src/host/robot_radio/testgui/__main__.py`**

1. The button is built at line **903** (`test_s_btn = QPushButton("Test S —
   drive 700mm")`) and wired at line **2928**
   (`test_s_btn.clicked.connect(lambda: _run_sim_test("S"))`).
2. `_run_sim_test("S")` (**2886-2926**) runs a worker thread that
   rebuilds the sim dylib (`gen_version.py`, `gen_messages.py`,
   `cmake --build src/sim/build --target firmware_host`, **2896-2900**),
   copies it to a unique temp path for hot-reload (**2914-2919**, dlopen
   caches by path), then emits `_test_bridge.rebuilt`.
3. `_finish_test` (**2854-2882**, queued back onto the Qt main thread at
   **2884**) disconnects any old transport, selects the Sim transport,
   points the next connect at the fresh dylib
   (`_set_sim_lib_override(fresh)`, **2868**; defined in
   `transport.py:843-847`), reconnects (`_on_connect()`, **2870**),
   resets pose/avatar (`_set_origin()`, **2876**), and finally sends the
   command:

   ```python
   wire = "D 150 150 700" if kind == "S" else "RT 36000"   # __main__.py:2877
   _state["transport"].command(wire, read_timeout=500)      # __main__.py:2879
   ```

   `"D 150 150 700"` = drive, left wheel 150 mm/s, right wheel 150 mm/s,
   distance 700 mm.

---

## 2. SimTransport routes the verb into a planner run

**File: `src/host/robot_radio/testgui/transport.py`**

1. `SimTransport.command()` (**1219-1230**) logs the line and calls
   `_dispatch(line)`.
2. `_dispatch()` (**1232-1272**) tokenizes; verb `"D"` is in
   `_MOTION_VERBS = ("D", "RT")` (**1203**), so it calls
   `_run_motion_async("D 150 150 700")` (**1252-1253**).
3. `_run_motion_async()` (**1464-1511**) spawns the `sim-direct-motion`
   thread. Its `_worker` (**1482-1507**):
   - `legs = parse_tour(["D 150 150 700"])` (**1489**)
   - `run_tour(loop, params, heading, legs, should_stop=…)` (**1498-1501**)
     where `loop` is the connected `SimLoop` (`self._loop`).

**File: `src/host/robot_radio/planner/tour.py`**

4. `parse_tour()` (**247-284**) turns the string into a typed leg
   (**269-274**):

   ```python
   TourLeg(kind="distance", value=700.0, speed=150.0)   # (|150|+|150|)/2
   ```

5. `run_tour()` (**486-641**) is the shared per-leg loop (same code a
   hardware tour uses). `_move_kwargs_for_leg()` (**383-395**) converts
   the leg into `Move` kwargs:

   ```python
   dict(distance=700.0, delta_heading=0.0, v_max=150.0)
   ```

   `send_leg(0)` (**553-554, 568**) calls `transport.move(**kwargs)` —
   which is `SimLoop.move()` because `SimLoop` structurally satisfies
   `MoveTransport` (**102-118**). The tour then polls
   `_wait_for_move_terminal()` (**424-463**) → `_drain_and_poll()`
   (**398-421**) on `transport.read_pending_binary_tlm_frames()` until an
   ack with `corr_id == move_id` and `status != ACK_STATUS_OK` appears —
   the leg's own completion event (see §8).

---

## 3. SimLoop builds the binary envelope and injects it

**File: `src/host/robot_radio/io/sim_loop.py`**

1. `SimLoop.move()` (**454-490**) builds the protobuf envelope with the
   same `pb2` codec real hardware uses (**482-488**):

   ```python
   envelope = pb2.CommandEnvelope(
       corr_id=move_id,
       move=pb2.Move(distance=700.0, delta_heading=0.0,
                     v_max=150.0, omega=0.0, time=0.0,
                     replace=False, id=move_id))
   armored = base64.b64encode(envelope.SerializeToString())
   self.inject_command(f"*B{armored}")                     # sim_loop.py:489
   ```

   `move_id` doubles as the enqueue ack's `corr_id` **and** `Move.id`,
   the key of the later completion event.
2. `inject_command()` (**524-531**) enqueues
   `lambda: self._lib.sim_inject_command(self._handle, encoded)` onto
   `self._cmd_queue` via `_run_or_enqueue()` (**834-840**) — the tick
   thread is the only thread that ever touches the ctypes handle.
3. The tick thread `_tick_loop()` (**872-974**) executes it on its next
   iteration via `_drain_cmd_queue()` (**918**, body **976-993**).

**File: `src/sim/sim_ctypes.cpp` → `src/sim/sim_harness.h`**

4. `sim_inject_command()` (`sim_ctypes.cpp:231-233`) →
   `SimHarness::injectCommand()` (`sim_harness.h:175`) →
   `serialLink_.enqueueInbound(armoredLine)` — the inbound FIFO of the
   serial-side `FakeTransport` (`fake_transport.h:46`).

The command is now sitting "on the wire", exactly as if a serial port had
received the line.

---

## 4. The tick thread steps the sim; the firmware loop consumes the command

**File: `src/host/robot_radio/io/sim_loop.py`**

Every tick-thread iteration calls
`self._lib.sim_step(self._handle, cycles)` (**936**), paced to one 50 ms
sim cycle per 50 ms of wall clock at 1× (`_CYCLE_DURATION_S`, **140**;
speed factor scales `cycles`, **933**; an idle plant drops to a 2 s
heartbeat, **922-931**).

**File: `src/sim/sim_harness.h`**

`sim_step()` (`sim_ctypes.cpp:221`) → `SimHarness::step()` (**162-169**),
whose ordering is the harness's one invariant — plant physics **before**
the loop reads it:

```cpp
plant_.tick(kCycleDtUs / 1e6f);   // physics first          sim_harness.h:164
clock_.advanceMicros(kCycleDtUs); // then virtual time (50ms) :165
robotLoop_.cycle();               // then the real firmware   :166
```

**File: `src/firm/app/robot_loop.cpp` — one `cycle()` (412-495)**

The cycle is four `runAndWait` pacing blocks (~40 ms budget, `kCycle`,
**25**):

1. `motorL_.requestSample()` (**417**) — 0x46 encoder-select write for
   the left motor (see §6).
2. Settle block (**419-421**): `comms_.pump(cmd)` — **this is where our
   command comes off the wire**. `Comms::pump()`
   (`src/firm/app/comms.cpp:49-53`) → `pumpTransport()` (**55-76**) reads
   one line from the `FakeTransport`, sees `*B`, and
   `decodeArmoredLine()` (**78-115**) base64-decodes and
   `msg::wire::decode()`s it into a `msg::CommandEnvelope`.
3. `motorL_.tick(...)` (**423**) — left collect + PID + duty write (§5/§6).
4. Clear block (**425-433**): `updateTlm(); tlm_.emit(cycleStart)` —
   telemetry out (§7).
5. `motorR_.requestSample()` (**435**).
6. Settle block (**437-470**): `processMessage(cmd)` (**443**) dispatches
   by `cmd_kind` (**351-372**); `MOVE` → `handleMove()` (**319-337**):

   ```cpp
   Motion::Cmd cmd = Motion::fromMove(env.cmd.move);   // robot_loop.cpp:320
   Motion::EnqueueOutcome outcome = pilot_.enqueue(cmd);          // :321
   ...
   tlm_.ack(env.corr_id, msg::AckStatus::ACK_STATUS_OK, 0);       // :327
   ```

   `Motion::fromMove()` is a field-for-field copy
   (`src/firm/motion/cmd.h:50`); `Pilot::enqueue()` forwards to
   `Motion::Executor::enqueue()` (`src/firm/app/pilot.h:136` →
   `src/firm/motion/executor.cpp:466`). The immediate `ACK_STATUS_OK`
   rides the next telemetry frame's ack ring — that is `run_tour()`'s
   enqueue acknowledgment.

   Then `pilot_.tick(cycleStart, nowUs)` (**459**) and `drive_.tick()`
   (**469**) stage this cycle's motion (§5), and `drainPilotEvents()`
   (**467**, body **339-344**) forwards any completion events to the ack
   ring (§8).
7. `motorR_.tick(...)` (**472**).
8. Pace block (**484-494**): OTOS read, `odom_.integrate()`,
   `frame_.pose = {...}` (**487**), and `pilot_.plan()` (**493**) — at
   most one jerk-limited trajectory solve per cycle
   (`Motion::Executor::plan()`, `executor.cpp:538`).

---

## 5. Move → twist → per-wheel velocity targets

Each cycle while the executor is running:

- `Pilot::tick()` (`src/firm/app/pilot.cpp:7-44`) samples the heading
  source, gets this cycle's setpoint from
  `Motion::Executor::tick(dt, odom_.lastDistance(), heading, headingLead)`
  (**20-21**; `executor.cpp:669`), adds the heading-PD correction to
  `omega` (**33-36**), and stages it:

  ```cpp
  drive_.setTwist(twist.v, omega);        // pilot.cpp:42
  ```

- `Drive::setTwist()` stores `v_x_`/`omega_`
  (`src/firm/app/drive.cpp:10-13`); `Drive::tick()` (**20-26**) splits
  the twist into wheel velocities via `BodyKinematics::inverse()` and
  calls `left_.setVelocity(vL); right_.setVelocity(vR)` — for a straight
  700 mm leg both targets ramp to ±150 mm/s.

- `NezhaMotor::setVelocity()` stores `velocityTarget_`
  (`src/firm/devices/nezha_motor.cpp:103-107`). In
  `NezhaMotor::tick()` step 4 (**380-391**) the velocity PID computes a
  duty from target vs. measured:

  ```cpp
  float duty = pid_.compute(velocityTarget_, filteredVelocity_, dt,
                            config_.velGains, config_.velDeadband);  // nezha_motor.cpp:384
  armoredWrite(duty, nowMs);                                        // :387
  ```

  (`VelocityPid::compute` lives in `src/firm/devices/velocity_pid.cpp`;
  `armoredWrite` is the `MotorArmor` policy gate,
  `src/firm/devices/motor_armor.h`.)

- `writeRawDuty()` (**423-489**) clamps/slews and issues the vendor duty
  frame; `writeMotorRun()` (**491-508**) puts the actual 8 bytes on the
  bus:

  ```cpp
  uint8_t buf[8] = { 0xFF, 0xF9, port, direction, 0x60, speed, 0xF5, 0x00 };
  bus_.write(0x10 << 1, buf, 8, ...);                    // nezha_motor.cpp:493-507
  ```

**The sim boundary:** `bus_` here **is** the `SimPlant` (constructor
injection, `sim_harness.h:109-110`). `SimPlant::write()`
(`src/sim/sim_plant.cpp:83-86`) dispatches to `handleMotorWrite()`
(**113-146**), which parses the same frame the real brick would:
`cmd==0x60` → `duty = ±speed/100` stored into `leftDuty_`/`rightDuty_` by
`port` (**121-139**). No prediction, no back-channel — the plant reacts
only to bytes actually written.

On the next `SimHarness::step()`, `SimPlant::tick(dt)`
(`sim_plant.cpp:238-242`) integrates the physics:

```cpp
left_.step(leftDuty_, dt);      // duty -> velocity -> position (WheelPlant)
right_.step(rightDuty_, dt);
otos_.step(left_.position(), right_.position(), dt);
```

(`WheelPlant`/`OtosPlant` are in `src/tests/sim/plant/` — a first-order
duty→velocity model with fault knobs and, on this ctypes path, rest-encoder
jitter enabled at `sim_ctypes.cpp:195`.)

---

## 6. Encoder read-back: where "encoder velocity" is measured

The firmware reads encoders split-phase, exactly as on hardware:

1. **Request** — `NezhaMotor::requestSample()`
   (`nezha_motor.cpp:543-549`) → `requestEncoder()` (**551-564**) writes
   the encoder-select frame `{0xFF,0xF9,port,0x00,0x46,0x00,0xF5,0x00}`.
   In the sim, `SimPlant::handleMotorWrite()` records
   `selectedPort_ = port` (`sim_plant.cpp:141-144`).
2. **Collect** — `NezhaMotor::collectEncoder()` (**566-590**) issues a
   4-byte read. `SimPlant::handleMotorRead()` (`sim_plant.cpp:158-164`)
   answers with the selected wheel's position, converted mm → raw counts
   (`kEncoderCountsPerMm = 1.4187`, **156**) and packed little-endian in
   tenths — the same wire format the real brick uses.
3. **Convert** — back in `NezhaMotor::tick()` step 2
   (`nezha_motor.cpp:248-250`):

   ```cpp
   int32_t raw = collectEncoder();
   float pos = (raw / 10.0f) * config_.wheelTravelCalib * fwdSign;  // counts -> mm
   ```

4. **Velocity estimate** — the freshness gate (**286**) only computes on
   a genuinely new raw count; then either the EMA path (**335-338**,
   `rawVel = (pos - lastPosition_) / freshElapsed` smoothed by
   `velFiltAlpha`) or the line-fit estimator (`lineFitVelocity()`,
   **199-218**) updates `filteredVelocity_`.
5. **Getters** — `position()` / `velocity()` (**224-225**) expose
   `lastPosition_` / `filteredVelocity_`. **`velocity()` is the "encoder
   velocity" the GUI will display.**

---

## 7. Telemetry out: velocities onto the wire

**File: `src/firm/app/robot_loop.cpp`**

`updateTlm()` (**137-184**) stages the per-cycle frame — the two lines
that matter for this trace:

```cpp
frame_.velLeft  = motorL_.velocity();    // robot_loop.cpp:144
frame_.velRight = motorR_.velocity();    // robot_loop.cpp:145
```

plus `encLeft/encRight` (**141-142**), the fused body twist (**157-159**),
`pose` (staged in the pace block, **487**), `active = driving_` (**162**),
and executor visibility (**166-168**). `tlm_.setFrame(frame_)` (**183**),
then `tlm_.emit(cycleStart)` (**432**).

**File: `src/firm/app/telemetry.cpp`**

`Telemetry::emit()` (**62-103**) paces primary (~25 Hz, every sim cycle
since `kCycleDtUs`=50 ms ≥ `kPrimaryPeriod`=40 ms) vs. secondary frames.
`emitPrimary()` (**105-154**) copies the staged frame into the protobuf:

```cpp
tlm.vel_left  = frame_.velLeft;     // telemetry.cpp:119
tlm.vel_right = frame_.velRight;    // telemetry.cpp:120
```

with the ack ring riding the same frame (**107-108**), wraps it in
`ReplyEnvelope{body: TLM}` (**144-147**), and calls
`comms_.sendReply(env)` (**149**).

**File: `src/firm/app/comms.cpp`**

`sendReply()` (**117-142**) encodes and armors — `'*' 'B' + base64` —
and broadcasts on both transports (**140-141**). In the sim those are
the two `FakeTransport`s; `FakeTransport::send()` appends the armored
line to its `sent_` capture (`fake_transport.h:60`).

---

## 8. Completion event (how the tour learns the 700 mm is done)

When `Motion::Executor` finishes the DISTANCE profile it emits a
`CompletionEvent{id, kDone}` (ring at `executor.cpp:149`, popped at
**163**). Each cycle `RobotLoop::drainPilotEvents()`
(`robot_loop.cpp:339-344`) converts it:

```cpp
tlm_.ack(event.id, toWireAckStatus(event.status), 0);   // ACK_STATUS_DONE
```

`Telemetry::ack()` (`telemetry.cpp:34-50`) pushes it into the 3-deep ack
ring, so it rides the next primary frame. Host-side,
`run_tour()`'s `_drain_and_poll()` (`tour.py:413-421`) matches
`ack.corr_id == move_id && status != ACK_STATUS_OK` → terminal →
`RunOutcome.COMPLETED` (`_outcome_for_status`, **466-476**), and
`SimTransport._run_motion_async`'s worker logs
`"'D 150 150 700' -> completed"` (`transport.py:1506-1507`).

---

## 9. Telemetry back into Python

**File: `src/sim/sim_harness.h` / `src/sim/sim_ctypes.cpp`**

- `SimHarness::drainRawTelemetry()` (`sim_harness.h:289-296`) returns
  every not-yet-drained line from `serialLink_.sent()` (its own drain
  index, **496**).
- `sim_drain_tlm()` (`sim_ctypes.cpp:237-248`) newline-joins them into
  the caller's buffer (snprintf-style return so Python can detect
  truncation).

**File: `src/host/robot_radio/io/sim_loop.py`**

Every tick-thread iteration, right after `sim_step()`,
`_drain_tlm_into_queue()` (**940**, body **995-1063**):

1. `self._lib.sim_drain_tlm(self._handle, buf, 16384)` (**1011**, retry
   sized-exactly on truncation **1012-1019**).
2. Per line: `_dearmor_reply()` (**206-218**) strips `*B`, base64-decodes,
   parses `pb2.ReplyEnvelope`; non-TLM bodies are skipped (**1030**).
3. `frame = TLMFrame.from_pb2(reply.tlm)` (**1032**) — the **same**
   decoder a real robot's replies go through
   (`src/host/robot_radio/robot/protocol.py:216-341`); the velocities
   land at:

   ```python
   if telemetry.has_vel:
       frame.vel = (int(telemetry.vel_left), int(telemetry.vel_right))  # protocol.py:314-315
   ```

   (`TLMFrame.vel` declared at `protocol.py:196` — differential
   `(vL, vR)` in mm/s.)
4. Sim-only extra ("Path B"): the **commanded** per-wheel velocity is not
   on the primary wire frame (186-byte envelope budget) — it is read
   straight off the live firmware objects and stamped onto the frame
   (**1039-1043**) via `sim_cmd_vel_left/right()`
   (`sim_ctypes.cpp:216-217` → `NezhaMotor::velocityTarget()`), feeding
   the commanded-vs-actual wheel-speed graph.
5. The frame goes onto the bounded `_tlm_queue` (**1048-1056**, feeding
   `read_pending_binary_tlm_frames()` — what `run_tour()` polls) **and**
   is delivered to `self.on_telemetry(frame)` (**1058-1062**) — the push
   path to the GUI.

---

## 10. GUI display: the encoder velocities you see

**Wiring (at connect time):**

- `SimTransport.connect()` sets `loop.on_telemetry = self._deliver_tlm`
  (`transport.py:1120`); `Transport._deliver_tlm` (**466-472**) forwards
  to `self.on_telemetry`.
- The GUI set that to `_on_telemetry_thread_v2`
  (`__main__.py:2645`).

**Thread hop (tick thread → Qt main thread):**

- `_on_telemetry_thread_v2()` (`__main__.py:1952-1965`) — runs on the
  SimLoop tick thread — caches `_state["last_tlm"]`, puts the frame on
  `_pending_frames` (declared **1485**), and emits
  `_bridge.frame_ready` (`_TelemetryBridge`, **1529-1535**), connected
  with `Qt.QueuedConnection` to a bound-method slot (**1642**) so the
  slot runs on the Qt main thread.

**Main-thread render:**

- `on_frame_ready()` (**1592-1624**) drains `_pending_frames`, feeds each
  frame to the trace model and graph panel
  (`graph_panel.add_tlm(...)`, **1603** —
  `TurnTraceRecorder.add_tlm` records `vel_l`/`vel_r`/`cmd_l`/`cmd_r`,
  `turn_graphs.py:141`; the "Wheel speed — commanded vs actual" graph is
  `turn_graphs.py:366`), refreshes the canvas avatar once per burst, and
  updates the breakout panel with the freshest frame:
  `telemetry_ctrl.update_frame(last_frame)` (**1624**).

**File: `src/host/robot_radio/testgui/telemetry_panel.py`**

- The `vel` row is declared at **359**
  (`("vel", "tlm_val_vel", True, "tlm_arrow_vel")`).
- `update_frame()` (**431-460**) renders it:

  ```python
  self._values["tlm_val_vel"].setText(fmt_vel(getattr(frame, "vel", None)))  # :436
  wheel = wheel_velocity(getattr(frame, "vel", None))                        # :456
  self._arrows["tlm_arrow_vel"].set_vector(*(wheel or (0.0, 0.0)))           # :457
  ```

- `fmt_vel()` (**162-168**) produces the text you read:

  ```
  vel   L +150   R +150   mm/s
  ```

- The rolling 10-second "Wheel speed" strip chart next to the readouts is
  `StripChartCanvas("Wheel speed", "mm/s", WHEEL_SPEED)`
  (**393**, series schema `turn_graphs.py:34`), redrawn on a 200 ms timer
  (**410-412**) from the shared `TurnTraceRecorder`.

That closes the loop: button → wire text → planner leg → binary `Move`
envelope → FakeTransport → real firmware loop → PID → 0x60 duty writes →
WheelPlant physics → 0x46 encoder reads → `filteredVelocity_` →
`Telemetry` frame → armored reply → drain → `TLMFrame.vel` → Qt bridge →
`vel` row + wheel-speed chart.

---

## Threads and ownership (who runs what)

| Thread | Created by | Runs |
|---|---|---|
| Qt main thread | `__main__.py` | Button handlers, `_dispatch()`, all widget updates (`on_frame_ready`, `update_frame`) |
| `sim-test-S` worker | `_run_sim_test` (`__main__.py:2926`) | dylib rebuild/copy only |
| `sim-direct-motion` | `SimTransport._run_motion_async` (`transport.py:1509`) | `parse_tour`/`run_tour` — sends `move()`, polls acks |
| `sim-loop-tick-thread` | `SimLoop.connect()` (`sim_loop.py:399`) | **Sole owner of the ctypes handle**: `sim_step`, command-queue drain, `sim_drain_tlm`, `on_telemetry`/`on_truth` callbacks |

The ctypes handle (`TestSim::SimHarness*`) is not thread-safe; every
call is either executed by the tick thread (fire-and-forget via
`_run_or_enqueue`, `sim_loop.py:834`) or synchronously round-tripped onto
it (`_call_on_tick_thread`, **842-866**).

## Invariants worth keeping

1. **Plant ticks before the loop reads it** — `SimHarness::step()` order
   (`sim_harness.h:162-169`); reversing it makes every sensor read one
   cycle stale.
2. **No back-channel** — `SimPlant` learns duty only from parsed 0x60
   frames (`sim_plant.h:188-191`), never from
   `NezhaMotor::appliedDuty()`. The one sanctioned exception is the
   read-only `cmd_vel` stamp (Path B, §9.4), which is diagnostic
   telemetry, not plant input.
3. **One decode path** — sim telemetry is dearmored/parsed by the same
   `pb2` codec and `TLMFrame.from_pb2()` a real robot's replies use;
   tests and the TestGUI therefore exercise the identical wire format.
4. **One firmware** — `App::RobotLoop` and everything below it compiles
   unmodified into the dylib; the sim substitutes only the `I2CBus`
   (`SimPlant`), `Clock`/`Sleeper` (`SimClock`), and `Transport`
   (`FakeTransport`) seams.
