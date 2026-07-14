---
status: pending
---

# Single-Loop Firmware: de-fiber, delete the Elite plumbing, telemetry-only return path

## Context

The 2026-07-13/14 code review (docs/code_review/2026-07-13-devices-drive-review.md) found
that nearly every major DeviceBus bug lives at the fiber boundary (staging, stale gates,
ring stamps, adapter seam), that the Elite architecture's plumbing (blackboard, router,
Hal seam, Configurator) is where behavior hides (no-op ticks, blind wedge flags, silent
segment drops), and ~7,100 lines of dead code. The stakeholder is removing on-robot
trajectory planning: **host plans, robot follows.** The robot becomes a velocity/yaw
follower with continuous, honest telemetry — a firmware you can read top to bottom.

Decisions (stakeholder, 2026-07-14): host-side planning; **delete the old stack up
front** (fallback = git tag + archived hex, not parked code); keep `*B` base64-armored
line framing and **raise the serial baud**; **relay stream spike first**.

## The main loop (the whole program, one page)

```cpp
int main() {
    uBit.init();
    serial.begin();                       // DEVICE: banner + text HELLO stay (host connect
    radio.begin();                        // classifier); everything else is binary

    Devices::I2CBus bus(uBit.i2c);        // leaves own their hardware; no fiber, no handles,
    Devices::NezhaMotor motorL(bus), motorR(bus);          // no staging layer
    Devices::Otos otos(bus);  Devices::ColorSensorLeaf color(bus);  Devices::LineSensorLeaf line(bus);
    Perception perception(otos, line, color);   // round-robin slot scheduler (one device/cycle)
    Preamble preamble(motorL, motorR, otos, color, line);  // begin/probe state machine, non-blocking
    Odometry odom;                        // encoder x,y,theta integration (host fuses OTOS/camera)
    Comms comms(serial, radio);           // RX pump + *B armor codec (transcribed from binary_channel)
    Telemetry tlm(serial, radio);         // always-on frame builder; ack ring rides in every frame
    Drive drive(motorL, motorR);          // twist -> wheel targets via BodyKinematics::inverse
    Deadman deadman;                      // ONE staleness rule for every actuation source
    Command cmd;                          // at most one decoded-but-unapplied command per cycle

    // Boot: resolve every device before entering the control loop. Telemetry
    // flows from power-on (frames report "booting" + per-device status), so the
    // host can tell booting from dead; commands are NOT consumed until the main
    // loop starts (host waits for the ready bit in the frame).
    while (!preamble.done()) {
        preamble.step();                    // one bounded probe action per pass
        tlm.emit();                         // boot frames: device detection status, faults
        uBit.sleep(kPreamblePace);          // paces probes AND yields (radio RX)
    }

    // Main loop: devices resolved, no readiness checks below this line.
    // TIMING: device calls are pure bus transactions and NEVER sleep. Every
    // required gap is a runAndWait block: it marks time on entry (immediately
    // after the bus event that starts the clock), runs its body, then sleeps
    // until at least the gap has elapsed since the mark. The block visibly
    // scopes exactly the work that borrows the dead time; the body never
    // touches the bus and never sleeps. runAndWait(gap, body) == markTime;
    // body(); sleepUntil(mark, gap). `grep 'runAndWait\|sleepUntil'` = the
    // firmware's complete timing schedule. I2CBus keeps per-device readyAt
    // stamps as a sleep-not-spin safety net (+ telemetry fault bit), so a
    // mis-ordered loop degrades loudly, never silently.
    for (;;) {
        uint32_t cycleStart = markTime();          // [ms] pace anchor

        motorL.requestEncoder();                   // 0x46 write (brick holds ONE pending read)
        runAndWait(kSettle, [&] {                  // >=4ms: L encoder settling, meanwhile --
            comms.pump(cmd);                       //   drain RX, decode <=1 frame into cmd
        });
        motorL.collectAndControl();                // encoder read -> velocity PID -> armored duty write

        runAndWait(kClear, [&] {                   // >=4ms: brick clears L's duty write, meanwhile --
            tlm.emit();                            //   main frame ~30Hz, slow frame on other cycles
        });
        motorR.requestEncoder();
        runAndWait(kSettle, [&] {                  // >=4ms: R encoder settling, meanwhile --
            switch (cmd.take()) {                  //   apply <=1 command; every path acks via tlm
                case Cmd::Twist:  drive.setTwist(cmd.v_x, cmd.omega); deadman.arm(cmd.duration);
                                  tlm.ack(cmd.corrId);  break;
                case Cmd::Config: config.apply(cmd.delta);  tlm.ack(cmd.corrId);  break;
                case Cmd::Stop:   drive.stop();  deadman.disarm();  tlm.ack(cmd.corrId);  break;
                case Cmd::None:   break;
            }
            if (deadman.expired()) drive.stop();   //   host silent -> wheels stop. No exceptions.
            drive.tick();                          //   twist -> wheel targets (R consumes them below)
        });
        motorR.collectAndControl();

        perception.step();                         // ONE of otos|line|color -- different bus address
        odom.integrate();                          // odometry from both fresh wheel samples

        sleepUntil(cycleStart, kCycle);            // pace to ~16ms; covers post-R-write clearance;
    }                                              //   always sleeps >=1ms (radio yield)
}
```

Properties to reason from: one sequential program, ~16 ms cycle (~60 Hz motor servo,
~30 Hz telemetry, perception ~20 Hz/device); both settle waits (`kSettle` = 4 ms, from
`device_bus.h:301`) do useful work and yield via `uBit.sleep`; a new twist takes effect
next cycle (~16 ms, small next to the ~130 ms actuation lag); no cross-thread state
anywhere. The brick's four timing constraints (two encoder settles, two post-duty-write
clearances) map one-to-one onto the three `runAndWait` blocks plus the pace `sleepUntil`
— together they ARE the firmware's timing schedule. `runAndWait(gap, body)` (stakeholder
decision 2026-07-14) marks time at entry — immediately after the bus event that starts
the clock — runs the body, and sleeps until the gap has elapsed; the block visibly
scopes exactly the work that borrows each wait, and no device or work function ever
sleeps on its own.
Boot is its own telemetry-emitting loop (stakeholder decision 2026-07-14):
the main loop carries no readiness conditionals — everything below the boot loop runs
with devices resolved — and the cost is only that commands sent during the ~2-5 s boot
are not consumed, which is free since the motors aren't ready to obey anyway.
Carried over intact from the reviewed DeviceBus: 0x46 alternation, MotorArmor,
perception slot schedule, preamble probes. Review fixes folded in by construction:
unified deadman (M2/M3), no fiber handoff (M6, m3), always-on frames kill silent
failures (drive #7); fix the NAK'd-stop-write latch (C1) and clearance busy-spins (M1)
in the port.

## Wire protocol

- **Commands (binary `*B` lines):** `twist{v_x, omega, duration}`, `config{delta}`,
  `stop{}`. Every command carries `corr_id`. Segment/mover/plan_dump/STREAM/GET/EVT
  arms are pruned from `protos/envelope.proto`. Text plane keeps exactly `HELLO` +
  the `DEVICE:` boot banner (host connect classifier, serial_conn.py).
- **Telemetry = the only return path.** Extend `protos/telemetry.proto`: ack ring
  (last 4–8 `{corr_id, status, err_code}` — repeated every frame so dropped frames
  can't lose an ack) + fault/event bits (bus errors, wedge flags, deadman trips).
  Budget: worst-case TLM is 165 B against a 186 B envelope ceiling (wire.h:53-59,
  static-asserted at build) — move `acc_*`, `glitch_*`, `ts_*`, `cmd_vel_*` to a
  slower secondary frame. All edits in `protos/` + generators, never generated files.
- **Baud:** `SerialPort::setBaud()` already exists (serial_port.cpp:70-82; 230400/921600/1M).
  Boot at 115200, host commands the switch on an open port (reopen pulses DTR = reset).
  30 Hz × ~252 B armored ≈ 66% of 115200 — the raise is load-bearing.

## Delete / keep (full inventory: Plan-agent report, merged below)

Already deleted on this branch (in progress): motion/, nezha_hardware, hal/nezha, hal/otos.

**Delete (~15,900 lines):** `source/main.cpp` (replaced), `runtime/`, `subsystems/`,
`commands/` (transcribe the `*B` armor + `msg::wire` encode/decode out of
`binary_channel.cpp` first — it's the only working framing implementation), `drive/`,
`telemetry/`, `hal/` (all: capability/, sim/, velocity_pid), `com/i2c_bus*`,
`estimation/` (EkfTiny — host fuses), `types/{arg_schema,command_types,clock*,value_set}`,
`kinematics/i_kinematics.h`, `devices/{bringup_main.cpp,fiber_runner.h}` + fiber/staging
machinery in `device_bus.{h,cpp}` + `handles.h`, `codal.devicebus.json`. Orphaned vendored
libs: `libraries/{ruckig,tinyekf,cmon-pid}` + their CMake lines (:224-232, :377).
Dead CMake filters/flags (BENCH_OTOS_ENABLED, PRODUCTION_BUILD, USE_ORDERED_TICK,
stale exclusion regexes, `application_entry` block :408-414).

**Keep:** `devices/` core (leaves, armor, PID, rings, I2CBus, preamble state machines),
`messages/` (regenerated from pruned protos), `com/{serial_port,radio,radio_channel}`,
`config/boot_config` (slimmed; gen_boot_config drops planner emission),
`kinematics/body_kinematics.*` (`inverse()` = twist→wheels, `forward()` = odometry),
`types/{protocol.h,version_generated.h}`, and the two load-bearing CMake exclusions
`devices/{i2c_bus_host,clock_host}.cpp` (duplicate-symbol guards).

**New code:** `source/app/` (loop, Comms, Telemetry+ack ring, Deadman, Drive, Odometry,
Preamble driver) + new `source/main.cpp`. NOT `source/robot/` — build.py:85-90 traps
that name (dead generator trigger).

**Test/host fallout:** tests/_infra/sim + tests/_infra/drive deleted (no sim build in
phase 1 — sim is rebuilt around the new steppable loop as its own later phase);
devices_* + wire_* unit tests survive (wire tests regenerate; keep the protobuf
differential oracle); ~35 old-stack pytest files + harnesses die; testgui parked;
pyproject `testpaths` pruned; justfile `build-sim`/`build-drive` recipes removed.
Host: `protocol.py` gains `twist/stop` builders and loses segment/stream builders;
`serial_conn.py`'s corr_id router becomes an ack-ring matcher (tolerate re-delivered
ring entries); legacy translators deleted; `nav/`/`path/`/`controllers/` survive as
the future host planner. Bench: `rig_dev.py` family rewritten to the binary plane
(phase 6); segment-based scripts (`turn_sweep.py`, teleop micro-segments, ruckig
verify) die; already-stale DEV-era scripts deleted.

## Phases (flashable image at every boundary)

- **P0 spikes:** (a) relay sustained-push: stream binary TLM @30 Hz through the relay
  with CURRENT firmware, measure delivery (standing note says async STREAM drops;
  ack-ring design tolerates loss, but if the relay hard-blocks pushes, fix relay fw or
  fall back to host-paced polling of the same frame). (b) baud ceiling on DAPLink —
  robot USB and relay dongle both. (c) frame budget: draft protos, run gen_messages —
  the wire.h static_asserts are the pass/fail, no hardware needed.
- **P1 tag + archive:** annotated tag `pre-single-loop`; archive known-good default hex
  AND a devicebus-bringup hex (rig keeps working through P3–P5); reflash once to prove
  the artifacts.
- **P2 delete + stub (ONE commit):** all deletions + build/test/host pruning + a ~50-line
  stub main (banner only, motors never energized) so the no-firmware window is one
  commit. Gate: builds, flashes, banners; surviving pytest green; grep for every
  deleted header returns nothing.
- **P3 the loop:** inline the cycle per the sketch; fix review C1 (commit
  lastWrittenPct_ only on kOk) and M1 (no busy-spin >1 ms — sleep-based clearance) in
  the port. Gate on the rig: motors servo under PID both directions, encoders ~60 Hz,
  OTOS/line/color detected; boot loop emits telemetry from power-on; I2CBus readyAt
  safety net never fires over a soak (proves the loop schedule covers all clearances).
- **P4 wire protocol:** pruned protos + ack ring + secondary frame; unified deadman;
  always-on 30 Hz emission. Gate: wire tests green; scope confirms cadence; budget
  asserts pass.
- **P5 host:** twist/config/stop builders, ack-ring matcher, baud raise both ends.
  Gate: live round-trip — twist drives wheels, ack observed in TLM, kill host
  mid-stream → wheels stop on deadman.
- **P6 bench gate:** rewritten rig_dev/rig_soak binary-plane soak — zero I2C errors,
  zero wedge latch, TLM drop rate measured over USB and relay, deadman kill-test.
- **P7 (later, own sprint):** sim rebuild — thin sim_api around the steppable loop
  (devices layer is already host-buildable via HOST_BUILD fakes); then testgui revival.

## Gotchas (full list in Plan-agent report; the ones that bite)

serial TX buffer is uint8_t (255 max — never two frames in one window); radio RX is
single-slot (host command spacing > cycle period; ack ring makes drops detectable);
keep `MICROBIT_RADIO_MAX_PACKET_SIZE=250` in codal.json; keep newline-terminated
armored lines ≤ ~250 B (relay line-splitting + 256 B RX buffers); settle windows must
`uBit.sleep`, never spin, never `sendReliable()` (5 ms wait > 4 ms window); newlib-nano
has no %f; DTR pulse on port-reopen resets the robot (baud switch on open port);
CODAL RAM near-full is normal.

## Verification

P3/P5/P6 gates above are the verification: real hardware on the rig/stand per
.claude/rules/hardware-bench-testing.md (sensors alive, wheels drive both directions,
round-trip over the real link), plus the deadman kill-test and relay/USB telemetry
drop-rate measurement. Surviving pytest subset stays green from P2 onward.

## Process

This is a new architecture baseline: after approval it goes through CLASI — dispatch
sprint-planner for an architecture update + sprint breakdown (P0–P1 spike/prep sprint,
P2–P4 firmware sprint(s), P5–P6 host+bench sprint); do not start implementing directly
from this plan. Update `.clasi/knowledge/` relay note with the P0 spike result either way.
