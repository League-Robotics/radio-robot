---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 103 Use Cases

Parent context: this sprint executes phases P3 (the loop) + P4 (the wire
protocol) + a minimal P5 slice of
`clasi/issues/single-loop-firmware-p3-p7-continuation.md`, the successor to
the archived plan in
`clasi/sprints/done/102-single-loop-firmware-spikes-archive-and-delete-to-stub-p0-p2/issues/single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md`.
Each use case below maps 1:1 to one sprint ticket, in dependency order.

## SUC-001: Prune the wire protocol to twist/config/stop with an ack-ring return path

Parent: continuation issue, P4 core schema; refines spike-003's scratch draft
(`scratch/102-003-frame-budget`, not merged).

- **Actor**: Firmware engineer; `scripts/gen_messages.py`/`gen_pb2.py`;
  `wire.h` static_asserts.
- **Preconditions**: Current `protos/envelope.proto`/`telemetry.proto` still
  carry the full pre-102 arm set (drive/segment/replace/pose_fix/otos/get/
  stream/plan_dump); spike-003's numbers (ring depth 3 = 179B vs 186B
  ceiling, 7B margin) are the design target, re-derived here for real, not
  inherited silently (spike-003's own hand-off note).
- **Main Flow**:
  1. Prune `CommandEnvelope.cmd` to `corr_id` + `oneof { Twist twist;
     ConfigDelta config; Stop stop; }`; every other arm deleted, not
     commented out. `ReplyEnvelope.body` narrowed to `ok`/`err`/`tlm`.
  2. Extend `Telemetry` with a `repeated AckEntry acks` ring (depth 3:
     `{corr_id, status, err_code}`) and `fault_bits`/`event_bits`; confirm
     the primary-frame field trim spike-003 measured (or a sprint-103-owned
     equivalent) keeps the worst case under 186B with real margin.
  3. Split `acc_*`/`glitch_*`/`ts_*`/`cmd_vel_*` into a new
     `TelemetrySecondary` message; decide (and document in
     architecture-update.md) how it rides the wire — a second `*B`-armored
     line or a `ReplyEnvelope` oneof arm — this is an open P4 decision
     spike-003 explicitly left unresolved.
  4. Run `scripts/gen_messages.py`/`gen_pb2.py` for real (not a scratch
     dry-run); regenerate `source/messages/*` and the host `envelope_pb2`/
     `telemetry_pb2`.
  5. Rewrite `wire_codec_harness.cpp`, `test_wire_codec.py`,
     `test_wire_differential.py`, `test_wire_fuzz.py` against the pruned
     schema (spike-003's flagged known gap) — keep the protobuf
     differential oracle.
- **Postconditions**: `protos/` and every generated artifact reflect the
  pruned P4 schema, merged (not scratch-only); `wire.h`'s static_asserts
  pass for real; the wire test suite is green against the new schema.
- **Acceptance Criteria**:
  - [ ] `CommandEnvelope`/`ReplyEnvelope` pruned exactly as above; every
        removed arm's field number is `reserved`, not reused.
  - [ ] Ack ring depth and primary-frame field set chosen and justified
        against the 186B ceiling with a real (non-scratch) `gen_messages.py`
        run; margin recorded in architecture-update.md.
  - [ ] `TelemetrySecondary`'s wire framing is a stated, justified decision,
        not left implicit.
  - [ ] `source/messages/*` and host `*_pb2` regenerated and compiling.
  - [ ] `wire_codec_harness.cpp` + the three `test_wire_*.py` files pass
        against the pruned schema; the protobuf differential oracle still
        runs.
  - [ ] No hardware needed for this ticket (schema/codegen/host-compile
        only, matching spike-003's own scope).

## SUC-002: Harden the NezhaMotor/I2CBus write path (C1, M1)

Parent: continuation issue, "review fixes C1 and M1 folded into the port";
`clasi/issues/nezha-motor-write-path-hardening.md`.

- **Actor**: Firmware engineer.
- **Preconditions**: `nezha_motor.cpp:325-372`'s `writeRawDuty()` commits
  `lastWrittenPct_`/`lastWriteTimeUs_` unconditionally regardless of
  `bus_.write()` status; `i2c_bus.cpp:67-68,112-113`'s clearance waits are
  hard busy-spins with no yield.
- **Main Flow**:
  1. C1: change `writeRawDuty()` to commit `lastWrittenPct_`/
     `lastWriteTimeUs_` ONLY on `status == kOk`; a failed `pct==0` (stop)
     write is treated as must-retry-next-tick, not silently latched as
     written.
  2. M1: replace the hard busy-spin clearance wait with the
     already-designed posture for the single-loop rebuild — the required
     gaps become the new loop's explicit `runAndWait`/`sleepUntil` calls
     (ticket 008), and `I2CBus`'s own per-device `readyAt` stamps remain
     ONLY as a sleep-not-spin safety net that raises a telemetry fault bit
     (SUC-001's `fault_bits`) if it ever fires.
  3. Confirm existing `devices_*` unit tests (`devices_motor_harness.cpp`
     and siblings) stay green through both changes.
- **Postconditions**: A transient NAK on a stop write no longer permanently
  defeats the watchdog's neutral re-assertion; no clearance wait blocks the
  scheduler for more than the safety-net threshold, and if it ever does,
  telemetry reports it via a fault bit rather than failing silently.
- **Acceptance Criteria**:
  - [ ] `writeRawDuty()` commits write-path state only on `kOk`; a failed
        stop write retries next tick (stop stays throttle-exempt).
  - [ ] No remaining busy-spin (`while(clockUs()<deadline){}` with no
        yield) in the write/clearance path; the safety net raises a fault
        bit (wired against SUC-001's `fault_bits` field) if it fires.
  - [ ] `devices_*` unit tests green.
  - [ ] `clasi/issues/nezha-motor-write-path-hardening.md` is referenced
        (`issue:` back-ref) by this ticket, for completion tracking.

## SUC-003: Retire DeviceBus/handles.h — leaves become the loop's direct dependency

Parent: continuation issue, "devices leaves direct (no fiber)"; resolves the
drift between the archived plan's bare-leaf main-loop sketch and
`Devices::DeviceBus`'s already-narrowed-but-still-handle-based public
surface (architecture-update.md Decision 1).

- **Actor**: Firmware engineer.
- **Preconditions**: `Devices::DeviceBus` (device_bus.{h,cpp}) and
  `handles.h` currently expose a `runPreamble()`/`runCycleOnce()`/
  `neutralizeAllMotors()` + `Motor`/`ColorSensor`/`LineSensor`/`Odometer`
  handle-based surface, narrowed but not deleted by sprint 102. Nothing in
  `source/app/` exists yet to replace it.
- **Main Flow**:
  1. Confirm (grep) that nothing outside `device_bus.{h,cpp}`/`handles.h`
     and their own tests references `DeviceBus`/the handle types.
  2. Delete `source/devices/device_bus.{h,cpp}` and `source/devices/
     handles.h`.
  3. Delete their now-orphaned C++ test harnesses
     (`tests/sim/unit/device_bus_cycle_harness.cpp`,
     `tests/sim/unit/test_device_bus_cycle.py`) — no port, no successor
     (DeviceBus itself is retired, not replaced 1:1; its job is absorbed by
     new, flatter `source/app/` modules in later tickets of this sprint).
  4. Leave `tests/bench/{rig_dev,rig_soak,device_bus_bringup}.py` and
     `tests/unit/test_device_bus_bringup_bench.py` untouched this ticket —
     their rewrite onto the new binary plane is explicitly sprint 104
     scope (continuation issue P6); they will not run against this
     sprint's firmware until then, which is accepted (Out of Scope, sprint.md).
- **Postconditions**: `source/devices/` contains only the leaves
  (`nezha_motor`, `otos`, `color_sensor`, `line_sensor`, `motor_armor`,
  `velocity_pid`, `i2c_bus`, `clock`, `interpolation`, `measurement_ring`,
  `device_config`, `device_types`) — no aggregate/orchestration class.
  `source/app/` (later tickets) becomes the ONLY caller of these leaves.
- **Acceptance Criteria**:
  - [ ] `device_bus.{h,cpp}` and `handles.h` deleted.
  - [ ] `device_bus_cycle_harness.cpp`/`test_device_bus_cycle.py` deleted.
  - [ ] `grep -rn "DeviceBus\|handles\.h"` under `source/` returns nothing.
  - [ ] `devices_*` leaf unit tests (unaffected — they test the leaves
        directly, not through DeviceBus) stay green.
  - [ ] `CMakeLists.txt` updated if it names either deleted file.

## SUC-004: Comms (armor codec) and Deadman (the one staleness rule)

Parent: continuation issue P3/P4; transcription source:
`clasi/sprints/done/102-.../notes/armor-wire-codec-transcription.md`.

- **Actor**: Firmware engineer.
- **Preconditions**: SUC-001's pruned `msg::CommandEnvelope`/
  `msg::ReplyEnvelope` types and `msg::wire::encode()`/`decode()`/
  `WireRuntime::base64Encode()`/`base64Decode()` exist and compile
  (`source/messages/wire.{h,cpp}`, `wire_runtime.{h,cpp}` — unchanged,
  KEPT primitives). `binary_channel.cpp`'s own orchestration is gone
  (deleted sprint 102); only its framing SHAPE survives in the
  transcription note.
- **Main Flow**:
  1. Build `source/app/Comms`: reproduces the armor/dearmor sequence from
     the transcription note (`"*B"` prefix, base64 encode/decode, buffer
     sizing from `kCommandEnvelopeMaxEncodedSize`/`kReplyEnvelopeMaxEncodedSize`)
     — NOT the old per-oneof dispatch switch (that is the new loop's own
     job, ticket 008). `Comms::pump(Cmd& out)` drains RX, decodes at most
     one frame into a `Cmd` per call, non-blocking, no sleep.
  2. Build `source/app/Deadman`: ONE staleness rule for every actuation
     source — `arm(duration)` on a twist command, `disarm()` on stop,
     `expired()` checked once per cycle. No second timeout path anywhere
     else in the new loop (continuation issue: "ONE staleness rule for
     every actuation source").
  3. Base64 alphabet pinned to standard RFC 4648 (`+/`), matching the host's
     `base64.b64encode`/`b64decode` defaults — transcription note's explicit
     warning: no negotiation, no version byte, must match exactly.
- **Postconditions**: A `*B<base64>` line on the wire decodes into exactly
  one `Cmd` per call; a twist/stop command arms/disarms the one deadman
  timer that gates all actuation.
- **Acceptance Criteria**:
  - [ ] `Comms::pump()` never sleeps, never blocks; decodes at most one
        frame per call.
  - [ ] Malformed armor (`line[1] != 'B'`) and malformed base64/protobuf
        are rejected cleanly (no crash, no partial state).
  - [ ] `Deadman` is the ONLY staleness/timeout mechanism gating actuation
        in the new loop — grep confirms no second ad hoc watchdog timer
        exists in `source/app/`.
  - [ ] Host-buildable unit coverage for `Comms`'s encode/decode round-trip
        using the `HOST_BUILD` seam already in `wire_runtime`.

## SUC-005: Telemetry — always-on frame builder with the ack ring and fault bits

Parent: continuation issue P4; consumes SUC-001's schema and SUC-004's
`Comms`-decoded `corr_id`s / `Deadman`'s trip state.

- **Actor**: Firmware engineer.
- **Preconditions**: SUC-001's `Telemetry`/`TelemetrySecondary`/`AckEntry`
  types exist; the recommended common cadence from spike-001 is 25 Hz
  (40 ms period) on BOTH transports, no baud change.
- **Main Flow**:
  1. Build `source/app/Telemetry`: `emit()` builds and sends one primary
     frame (`now`/`mode`/`seq`/`enc`/`vel`/`pose`/`otos`+`otos_connected`/
     `twist`/`active`/`conn_left`/`conn_right`/`fault_bits`/`event_bits`
     per spike-003's depth-3-fitting field set, or this ticket's own
     re-derived equivalent) carrying the ack ring (last 3 `{corr_id,
     status, err_code}`, repeated every frame so a dropped frame cannot
     lose an ack).
  2. `ack(corrId, status, errCode)` pushes one entry into the ring (called
     by the new loop's command-dispatch switch, ticket 008 — this ticket
     builds the ring mechanism and the call surface, not the switch
     itself).
  3. Wire fault bits: at minimum, the `I2CBus` `readyAt` safety-net trip
     (SUC-002) and a deadman trip (SUC-004) each set a distinct
     `fault_bits`/`event_bits` flag — bit layout is this ticket's decision
     to make and document (spike-003 left it undefined).
  4. Emit `TelemetrySecondary` on the framing decided in SUC-001, paced so
     the combined primary+secondary cadence stays at ~25 Hz per frame type
     (not competing for the same slot every cycle) — re-measure this
     sprint's own emission pacing against spike-001's pre-rewrite baseline
     (spike-001's own flagged gap: ~30.3 Hz armed vs. ~26.8 Hz actual was
     not root-caused; this ticket's new loop should not silently repeat it
     unmeasured).
- **Postconditions**: Every cycle from boot onward emits a telemetry frame;
  a client polling the ack ring across a small window of frames reliably
  observes every ack even if one frame is dropped.
- **Acceptance Criteria**:
  - [ ] Primary frame worst-case encoded size measured (real
        `gen_messages.py` run) and recorded with its margin against 186B.
  - [ ] Ack ring depth 3, each entry `{corr_id, status, err_code}`,
        survives a single dropped frame without losing an ack (unit-level
        proof: encode 4 sequential acks across a ring depth of 3, confirm
        the newest 3 are always present in the next frame).
  - [ ] `fault_bits`/`event_bits` layout documented; at least the I2CBus
        safety-net and deadman-trip bits are wired.
  - [ ] Measured emission cadence recorded (both frame types) against the
        25 Hz/40 ms target.

## SUC-006: Drive, Odometry, and OTOS perception — twist to wheels and back

Parent: continuation issue P3; uses the KEPT, unchanged
`kinematics/body_kinematics.*`.

- **Actor**: Firmware engineer.
- **Preconditions**: `BodyKinematics::inverse()`/`forward()` are unchanged,
  proven primitives (no code change needed — this ticket is integration,
  not kinematics work). SUC-003's bare leaves (`NezhaMotor` x2, `Otos`) are
  the actuation/perception targets; SUC-001's `msg::Twist{v_x, omega,
  duration}` is the command shape. The pruned `Telemetry` primary frame
  (SUC-001/SUC-005) carries `otos`/`otos_connected` but no `line`/`color`
  fields — those were never part of the STREAM/SNAP-derived schema this
  sprint prunes from, not something this sprint removes.
- **Main Flow**:
  1. Build `source/app/Drive`: `setTwist(v_x, omega)` stores the target;
     `stop()` zeroes it; `tick()` calls `BodyKinematics::inverse(v_x, omega,
     trackWidth, vL, vR)` and stages `vL`/`vR` onto the two `NezhaMotor`
     leaves via their existing `setVelocity()` primitive setter (PID stays
     enabled — `pidEnabled_` default `true`, unchanged leaf behavior).
  2. Build `source/app/Odometry`: `integrate()` reads both motors'
     `position()` (or per-cycle delta), calls `BodyKinematics::forward()`
     to get body twist, and accumulates world `x`/`y`/`theta` — the encoder
     odometry the continuation issue's P3 bullet names explicitly ("encoder
     odometry via BodyKinematics"), independent of OTOS/camera fusion
     (host fuses those; this is the on-robot dead-reckoning leg only).
  3. A minimal OTOS perception step (owned by this ticket, not a separate
     module): one `Otos` sample per cycle (or per the archived plan's
     round-robin slot, this ticket's call), feeding `Telemetry`'s
     `otos`/`otos_connected` fields (SUC-005's fields, this ticket's data
     source). The archived plan's full 3-way `Perception` round-robin
     (otos|line|color) is intentionally NOT built this sprint —
     `line`/`color` have no telemetry field to feed yet (Step 7 open
     question in architecture-update.md); building a round-robin scheduler
     for two devices with no wire consumer is deferred, not silently
     dropped.
- **Postconditions**: A twist command staged via `Drive::setTwist()`
  results in both motors' velocity-PID targets being set correctly-signed
  and correctly-scaled next cycle; `Odometry` produces a monotonically
  updating world pose estimate purely from encoder deltas; `Telemetry`'s
  `otos`/`otos_connected` fields reflect a live-sampled OTOS reading.
- **Acceptance Criteria**:
  - [ ] `Drive::tick()`'s wheel targets match `BodyKinematics::inverse()`'s
        output exactly (no additional scaling/sign logic duplicated in
        `Drive`).
  - [ ] `Drive::stop()` results in both wheel targets going to 0 within one
        cycle.
  - [ ] `Odometry::integrate()` uses `BodyKinematics::forward()` (not a
        hand-rolled equivalent) and accumulates world pose correctly for a
        straight-line and a pure-rotation host-buildable test case.
  - [ ] No new state duplicated between `Drive`/`Odometry` and the
        `NezhaMotor` leaves' own cached position/velocity — `Odometry`
        reads the leaves, it does not maintain a shadow copy.
  - [ ] `Otos` is sampled at least once per cycle (or per the documented
        slot schedule) and its result reaches `Telemetry` before that
        cycle's frame is built.
  - [ ] `line`/`color` steady-state sampling explicitly deferred (documented,
        not silently dropped) — Preamble (SUC-007) still detects their
        presence at boot.

## SUC-007: Preamble — the boot-time device-detection driver

Parent: continuation issue P3; replaces `DeviceBus::runPreamble()`
(retired, SUC-003) with an app-level driver over the same bare leaves.

- **Actor**: Firmware engineer.
- **Preconditions**: Each leaf (`NezhaMotor::begin()`, `Otos::begin()`,
  `ColorSensorLeaf::beginStep(nowUs)`, `LineSensorLeaf::beginStep(nowUs)`)
  already owns its own per-device detection/retry state machine
  (unchanged, KEPT — continuation issue's keep list: "preamble state
  machines"). What's missing is the app-level driver that calls them in a
  boot loop and knows when detection is DONE.
- **Main Flow**:
  1. Build `source/app/Preamble`: `step()` advances each leaf's own
     `begin()`/`beginStep(nowUs)` at most once per call (one bounded probe
     action per pass, matching the archived plan's boot-loop comment); no
     leaf's own retry loop is reimplemented here, only sequenced.
  2. `done()` returns true once every leaf has reached a terminal state
     (present-and-ready, or absent-after-exhausting-retries — an absent
     sensor must not hang boot forever, mirroring `DeviceBus::runPreamble()`'s
     retired `kMaxPreambleTicks` defensive bound).
  3. The boot loop (ticket 008) calls `preamble.step()` + `tlm.emit()` +
     a pacing sleep each pass until `done()` — commands are not consumed
     during boot (host waits for the ready signal in the frame, per the
     archived plan).
- **Postconditions**: Every device is resolved (present+ready, or
  confirmed absent) before the main cycle begins; boot never blocks
  indefinitely on one unresponsive sensor.
- **Acceptance Criteria**:
  - [ ] `Preamble::step()` calls each leaf's own detection entry point at
        most once per invocation — no busy-loop-within-a-loop.
  - [ ] `Preamble::done()` is reachable even with one or more sensors
        absent (bounded retries, not infinite).
  - [ ] No I2C traffic from any leaf before `Preamble` has begun probing it
        (mirrors the retired `DeviceBus::kPowerSettleMs` boot power-settle
        behavior — this ticket's decision whether to keep an explicit
        settle wait or rely on each leaf's own retry pacing, documented
        either way).

## SUC-008: The real main.cpp — boot loop and runAndWait cycle

Parent: continuation issue P3; the archived plan's one-page main loop,
implemented verbatim in shape (stakeholder-mandated pattern:
`runAndWait(gap, body) == markTime; body(); sleepUntil(mark, gap)`).

- **Actor**: Firmware engineer.
- **Preconditions**: SUC-002 (hardened leaves), SUC-004 (Comms/Deadman),
  SUC-005 (Telemetry), SUC-006 (Drive/Odometry), SUC-007 (Preamble) all
  exist and compile as standalone modules.
- **Main Flow**:
  1. Replace `source/main.cpp` (the sprint-102 banner-only stub) with the
     real single loop: construct `I2CBus`, the two `NezhaMotor`s, `Otos`,
     `ColorSensorLeaf`, `LineSensorLeaf` directly (no `DeviceBus`, per
     SUC-003); construct `Comms`, `Telemetry`, `Drive`, `Deadman`,
     `Preamble`, `Odometry`.
  2. Boot loop: `while (!preamble.done()) { preamble.step(); tlm.emit();
     uBit.sleep(kPreamblePace); }`.
  3. Main loop, per cycle: `motorL.requestEncoder()`; `runAndWait(kSettle,
     [&]{ comms.pump(cmd); })`; `motorL.collectAndControl()`;
     `runAndWait(kClear, [&]{ tlm.emit(); })`; `motorR.requestEncoder()`;
     `runAndWait(kSettle, [&]{ switch (cmd.take()) { ... } if
     (deadman.expired()) drive.stop(); drive.tick(); })`;
     `motorR.collectAndControl()`; `perception.step()`;
     `odom.integrate()`; `sleepUntil(cycleStart, kCycle)`.
  4. `markTime()`/`sleepUntil()`/`runAndWait()` are the three primitives
     the whole schedule is built from — implement them once, use them
     everywhere a gap is borrowed (no ad hoc sleep calls elsewhere in the
     cycle body).
- **Postconditions**: The firmware image is the single loop described in
  the continuation issue: telemetry from power-on, ~16ms cycle, one
  command decoded-and-applied per cycle, deadman-gated actuation, no
  fiber, no cross-thread state anywhere.
- **Acceptance Criteria**:
  - [ ] `grep 'runAndWait\|sleepUntil' source/main.cpp` (or
        `source/app/*` if the primitives live there) shows the complete
        timing schedule: three `runAndWait` blocks + one `sleepUntil` pace
        call, matching the archived plan's schedule one-for-one.
  - [ ] No device call sleeps or blocks on its own — every required gap is
        a `runAndWait`/`sleepUntil` call, matching M1's fix (SUC-002).
  - [ ] `just build` produces a hex; flashes; boots; identifies.
  - [ ] Boot loop does not consume commands before `preamble.done()`.
  - [ ] One decoded-but-unapplied command per cycle, applied via the
        `switch (cmd.take())` dispatch shown above; every path acks via
        `tlm.ack(cmd.corrId)`.

## SUC-009: Minimal host slice — twist/stop + ack-ring matcher

Parent: continuation issue P5 (minimal slice only — full P5 is sprint 104).

- **Actor**: Host engineer; `NezhaProtocol`
  (`host/robot_radio/robot/protocol.py`).
- **Preconditions**: SUC-001's pruned `envelope_pb2`/`telemetry_pb2` are
  regenerated and importable host-side. `NezhaProtocol` already owns
  `_send_envelope()`/reply-queue infrastructure built for the OLD
  synchronous-reply design (one `OK`/`ERR` per command) — the NEW design is
  telemetry-only return path (no per-command synchronous reply), so the
  ack must be read out of subsequent `Telemetry.acks` frames instead.
- **Main Flow**:
  1. Add `NezhaProtocol.twist(v_x, omega, duration)`: builds a
     `CommandEnvelope{corr_id, twist:{v_x, omega, duration}}`, sends it
     (fire, no wait for a synchronous reply — there isn't one anymore),
     returns the `corr_id` used.
  2. Add `NezhaProtocol.stop()`: same shape, `CommandEnvelope{corr_id,
     stop:{}}`.
  3. Add an ack-ring matcher: given a `corr_id` and a timeout, poll
     incoming `Telemetry` frames' `acks` list (via the existing
     `read_pending_binary_tlm_frames()`/binary telemetry delivery path)
     until a matching entry appears or the timeout elapses; tolerate a
     re-delivered ring entry (the same `corr_id` appearing in more than one
     frame is not an error).
  4. Write one `tests/bench/` script that: connects, arms telemetry,
     sends a `twist`, confirms the ack via the matcher, confirms encoders
     move, sends `stop`, confirms the ack.
- **Postconditions**: A host script can drive the rig's wheels and confirm
  command receipt without any synchronous per-command reply — purely via
  the telemetry ack ring.
- **Acceptance Criteria**:
  - [ ] `NezhaProtocol.twist()`/`stop()` build and send the pruned envelope
        shape; no dependency on the retired synchronous OK/ERR reply path
        for these two commands.
  - [ ] Ack-ring matcher correctly finds a `corr_id` across multiple
        frames and tolerates re-delivery; has a bounded timeout (no
        infinite wait).
  - [ ] New `tests/bench/` drive script runs end-to-end against the new
        firmware (requires ticket 008's firmware to be flashed — this
        ticket may be authored and unit-tested against schema/mocks before
        that, but its bench script is exercised for real in ticket 010).

## SUC-010: Bench gate — the robot drives, on the new firmware

Parent: continuation issue P3/P4/P5 combined gate; the hard scoping rule's
"every sprint ends bench-runnable" requirement.

- **Actor**: Firmware + host engineer; the bench rig
  (`.claude/rules/hardware-bench-testing.md`, wheels off the ground).
- **Preconditions**: SUC-008 (firmware) and SUC-009 (host slice) both
  complete; `mbdeploy` available; rig connected both by direct USB and
  through the radio relay.
- **Main Flow**:
  1. `mbdeploy deploy --build`; confirm boot banner + telemetry from
     power-on (before any command is sent).
  2. Send a `twist` via SUC-009's script; confirm both wheels spin under
     velocity PID; confirm encoders increment in the commanded direction,
     roughly proportional to the commanded speed, in BOTH directions
     (forward/backward and left/right turn).
  3. Confirm the twist's `corr_id` appears in the telemetry ack ring, over
     BOTH direct USB serial AND the radio relay's `!GO` data plane
     (reconnect and repeat the send/observe step on the second transport).
  4. Deadman kill-test: arm a twist, then stop sending (kill the host
     script/sender); confirm the wheels stop within one stale window
     (`Deadman`'s configured timeout) with no further host input.
  5. `grep 'runAndWait\|sleepUntil' source/main.cpp` (or wherever the
     primitives/cycle body live) and confirm the printed schedule matches
     the archived plan's cycle one-for-one (three settle/clearance windows
     + one pace sleep).
- **Postconditions**: The robot is verified, on real hardware, on the
  stand, driving under the new firmware via the new wire protocol and the
  new minimal host slice. This IS the sprint's Definition of Done — no
  sprint in this arc closes on tests alone (project standing rule,
  `.claude/rules/hardware-bench-testing.md`).
- **Acceptance Criteria**:
  - [ ] Telemetry observed from power-on, before any command sent.
  - [ ] Twist drives both wheels, both directions, encoders tracking.
  - [ ] Ack observed in the TLM ring over BOTH USB and relay.
  - [ ] Deadman kill-test passes (wheels stop with no further input).
  - [ ] `runAndWait`/`sleepUntil` grep matches the archived schedule.
  - [ ] No motor left energized at the end of the verification session.
