---
id: '010'
title: "Bench gate \u2014 the robot drives on the new firmware"
status: done
use-cases:
- SUC-010
depends-on:
- 008
- 009
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench gate — the robot drives on the new firmware

## Description

The sprint's Definition of Done, per the 2026-07-14 stakeholder hard
scoping rule ("every sprint ends bench-runnable") and
`.claude/rules/hardware-bench-testing.md`. Deploy ticket 008's firmware to
the bench rig and, using ticket 009's minimal host slice, prove: telemetry
from power-on, twist-driven wheels under PID in both directions with
encoders tracking, an ack observed in the telemetry ring over BOTH direct
USB and the radio relay, a deadman kill-test, and the `runAndWait`/
`sleepUntil` schedule grep. No sprint in this arc closes on tests alone —
this ticket is real hardware, on the stand, wheels off the ground.

Depends on tickets 008 (firmware) and 009 (host slice) — strictly last in
this sprint.

## Acceptance Criteria

- [x] `mbdeploy probe` confirms exactly one micro:bit connected (per
      `.claude/rules/debugging.md`'s precondition discipline); `mbdeploy
      deploy --build` flashes ticket 008's firmware.
- [x] Boot banner + telemetry frames observed from power-on, BEFORE any
      command is sent (confirms the boot loop's telemetry-from-power-on
      property).
- [x] A `twist` sent via ticket 009's script drives both wheels under
      velocity PID; encoders increment in the commanded direction, roughly
      proportional to commanded speed, confirmed in BOTH directions
      (forward/backward) plus a pivot (`omega`-only) turn. **First pass
      (motor power switch off, unknown to this session) FAILED** — see
      "First pass" Results below for that evidence, and "Root cause" for
      why it wasn't a firmware defect. **Re-run pass (motor power
      confirmed on) PASSED** — see "Re-run" Results below.
- [x] The twist's `corr_id` is observed in the telemetry ack ring — once
      over direct USB serial, and again (reconnect, repeat) over the radio
      relay's `!GO` data plane. Both transports confirmed independently,
      not just one. Passed on the first pass already (ack-ring/transport
      plumbing does not depend on motor power) and reconfirmed on the
      re-run with real wheel motion riding along.
- [x] Deadman kill-test: arm a twist, then stop sending from the host
      (kill the script/process); confirm the wheels stop within one stale
      window (`Deadman`'s configured timeout, ticket 004) with no further
      host input. State-machine behavior verified on the first pass;
      re-run's explicit `stop()`-mid-motion check (both transports) adds
      direct encoder/velocity evidence of an actual physical stop.
- [x] `grep 'runAndWait\|sleepUntil' source/main.cpp` output is captured
      and confirmed to match the archived plan's schedule one-for-one (the
      same check as ticket 008's own acceptance criterion, re-verified
      here as part of the final gate).
- [x] No motor is left energized at the end of the verification session
      (explicit `stop()` sent and confirmed via telemetry before
      disconnecting).
- [x] Session is conducted per `.claude/rules/hardware-bench-testing.md`
      (robot on the stand, wheels off the ground) — confirmed explicitly
      in completion notes, not assumed.

## GATE RESULT: PASSED (after root-cause correction) — see Root Cause section

**First pass** of this session's bench gate failed the central "wheels
move" claim — see "First pass" Results below for the full evidence trail.
**Root cause**: the bench rig's motor-power switch was off for the
entire first pass (stakeholder-confirmed physical fact, not a firmware
defect — see "Root Cause" section). Every firmware-side signal collected
during the first pass was self-consistent with that explanation (I2C
logic ACKs with the driver stage unpowered, encoders genuinely never
move, the wedge-latch fault bit correctly reflects "no motion"), and none
of it pointed at a real code defect. With power confirmed on, the
team-lead's own re-run and this session's own **re-run pass both PASSED
cleanly** — see "Re-run" Results below. All acceptance criteria are met;
this ticket is DONE.

## Implementation Plan

**Approach**: This ticket is verification, not new code — if it finds a
defect in tickets 001-009's work, the fix belongs in whichever ticket
owns the broken module (this ticket does not silently patch around a
found defect; it reports and, if the defect is small and clearly scoped
to one already-closed ticket, may reopen that ticket per the project's
normal ticket-reopen mechanism — a call for whoever executes this ticket
to make in the moment, not pre-decided here).

**Files to create/modify**: none expected (verification only); if a defect
requires a real code fix, it lands in the ticket that owns the broken
module, not here.

**Testing plan**:
- Existing tests to run: none (this IS the test — real hardware, not
  pytest).
- New tests to write: none (ticket 009's bench script is the tooling this
  ticket exercises; no NEW test artifacts expected from this ticket
  itself).
- Verification command: manual bench session per the Acceptance Criteria
  above, using `mbdeploy` + ticket 009's `tests/bench/` script + a second
  connection through the radio relay for the transport-parity check.

**Documentation updates**: record the session's results (encoder
directions/magnitudes observed, ack-ring confirmation on both transports,
deadman kill-test timing, the `runAndWait`/`sleepUntil` grep output) in
this ticket's own completion notes — this IS the sprint's evidence of
"bench-runnable," and should be detailed enough that a future reader does
not have to re-run the session to trust the sprint closed correctly.

## Results — First pass (2026-07-14 bench session, motor power OFF, unknown at the time)

Hardware: robot UID
`9906360200052820a8fdb5e413abb276000000006e052820` at
`/dev/cu.usbmodem2121102`; relay at `/dev/cu.usbmodem2121302`. Robot
mounted on the bench stand, wheels off the ground, per
`.claude/rules/hardware-bench-testing.md` — confirmed visually before the
session and unchanged throughout (no driving occurred, per the failure
below, so this was never at risk).

### 1. Build + flash

`just build-clean` → `v0.20260714.17`, RAM 98.33% (expected —
`.clasi/knowledge/codal-ram-always-near-full.md`), FLASH 35.37%. First
`mbdeploy deploy <UID> --hex MICROBIT.hex` hit a transient
`flash erase sector failure (address 0x00000000; result code 0x67)`,
auto-recovered via `mbdeploy`'s own CTRL-AP mass-erase-and-retry path, but
the retry itself then hit `Timeout reading from probe` and failed
outright. A second, independent `mbdeploy deploy` invocation (same
command) succeeded cleanly on its own first mass-erase-and-retry cycle
(`Erased 287744 bytes (71 sectors), programmed 287744 bytes (71 pages)`).
`pyocd list` confirmed the probe was never actually lost between
attempts — this reads as bench-side flash flakiness (possibly a stale DAP
handle from `mbdeploy probe`'s own preceding HELLO-classify opens), not a
firmware or `mbdeploy` defect; not re-litigated further since a clean
retry resolved it and every gate after this ran on a confirmed-correct
flash.

### 2. Boot: telemetry-from-power-on

Reflashed once more immediately before this specific capture
(`mbdeploy deploy` triggers its own hardware reset at the end of
programming) and opened the serial port with a controlled DTR low→high
edge in the SAME process invocation, no command sent, to capture from the
true reset edge:

```
t=0.000  tlm now=452   seq=9   fault=0 event=0 active=False   [boot]
...
t=1.606  tlm now=2176  seq=46  fault=0 event=0 active=False   [boot]   <- last preamble frame
t=1.661  tlm now=2216  seq=47  fault=1 event=2 active=False   [MAIN]  <- first main-loop frame (kEventBootReady)
t=1.980  tlm now=2536  seq=52  fault=3 event=2 active=False   [MAIN]  <- wedge bit joins (idle motors, expected — see §6)
```

Telemetry frames flow from the very first fraction of a second after
reset (seq already at 9 by the time the capture script's own process
startup caught up, ~450ms in) — well before any host command (`HELLO`/
`PING`/anything) was sent. Preamble frames correctly carry `fault=0
event=0` (Preamble never calls `setFault`/`setEvent`, matching main.cpp);
the transition to the main loop is marked cleanly by `event=2`
(`kEventBootReady`, `Preamble::done()` first-true) at `now=2216`, i.e.
preamble took ~2.2s this boot. No `DEVICE:` banner is emitted
unprompted at boot (by design — `App::Comms::pumpTransport()` only
replies to an explicit `HELLO`); separately confirmed the `HELLO` →
`DEVICE:NEZHA2:robot:tovez:2314287040` banner reply still works via
`SerialConnection.connect()`'s own classify handshake, used successfully
throughout this session.

### 3/4. Twist drives wheels + ack ring — FAILED (wheels), PASSED (ack ring, both transports)

`tests/bench/twist_drive.py` ran as-shipped, no fixes needed to the
script itself. USB direct, `v_x=150, omega=0, duration=1500`:

```
[PASS] connect()                                  (mode=direct)
[PASS] twist() returns a corr_id                  (corr_id=1)
[PASS] twist() ack confirmed via ack ring          (ack=AckEntry(corr_id=1, ok=True, err_code=0))
[FAIL] encoders moving during twist()              (before=(0, 0) after=(0, 0))
[PASS] stop() returns a corr_id                    (corr_id=2)
[PASS] stop() ack confirmed via ack ring           (ack=AckEntry(corr_id=2, ok=True, err_code=0))
==== 5/6 checks passed ====
```

Reproduced with a raw `pb2.Telemetry` decode (bypassing `TLMFrame`'s
field subset, to rule out a host-side adapter bug) over a 2s window:
`conn_left=True conn_right=True` (both Nezha bricks genuinely ACK on the
bus) but `enc_left`/`enc_right`/`vel_left`/`vel_right` stay at literal
`0.00` for every single frame, while `active=True` the whole time. Tried:
- `v_x=500` (near max commandable speed) — identical: zero movement.
- `v_x=0, omega=1.5` (pure in-place turn) — identical: zero movement on
  both wheels.

So the failure is independent of commanded magnitude, direction, and
whether it's a straight or turning twist — both wheels, every time.

Ack ring / transport parity (AC item, independent of the drive failure)
PASSED cleanly on both transports:
- USB direct: `corr_id` observed in the ack ring, `ok=True`, as above.
- Relay: `SerialConnection.connect()` on `/dev/cu.usbmodem2121302`
  classified `role=RADIOBRIDGE`, ran the `!ECHO OFF`/`!MODE RAW250`/`!GO`
  handshake (`relay_info={'relay_config': '# channel: 0 group: 10 mode:
  RAW250 power: 7', 'entered_data_plane': True}`), then
  `tests/bench/twist_drive.py --port /dev/cu.usbmodem2121302` produced
  the identical PASS/FAIL pattern above — ack ring and telemetry both
  round-trip correctly over the radio relay's data plane; only the wheel
  motion itself is missing, on both transports identically (confirms the
  defect is firmware/drive-path, not transport-specific).

### 5. Deadman kill-test — PASSED (state level)

Armed a twist (`duration=1500`), confirmed `active=True` with no `stop()`
ever sent, then fully disconnected the host process (closed the serial
port) — simulating a killed host. Reconnected fresh ~2s after the
duration window (no host input in between): `active=False`, matching the
deadman's auto-expiry contract with zero further host input. (`event_bits`
had returned to `2` — just `kEventBootReady` — by the time of the
reconnect read, rather than showing `kEventDeadmanExpired` still set;
`Deadman`/`Telemetry`'s event bit appears to be reported on the
transition cycle rather than latched for the whole post-expiry window —
worth a closer look if a future ticket needs "still-expired" to be
level-sensed, but is not itself evidence against the kill-test's actual
claim, which is "the host went silent and no further motion command was
needed to make `active` go false.") Because wheels never physically
started moving in this session (§3/4), "wheels stop" cannot be
independently confirmed by encoder motion — only the state-machine
behavior (`active` correctly following arm/expire) was verified.

### 6. Fault/event bits over the session

Observed exactly two steady-state values the whole session:
`fault_bits=3` (`kFaultI2CSafetyNet` | `kFaultWedgeLatch`) and
`event_bits=2` (`kEventBootReady`). Both are consistent with the
documented, sanctioned wiring, not a new defect on their own:
- `kFaultI2CSafetyNet` is driven by `I2CBus::clearanceSafetyNetCount() >
  0` (`main.cpp`), a monotonic, never-reset counter —
  `telemetry.h`'s own doc comment for `setFault()` gives this EXACT call
  pattern as the canonical example, so once any single safety-net trip
  happens (plausible during preamble's aggressive device probing), the
  bit is architecturally expected to stay set for the rest of the boot
  session (not a loop-schedule defect by itself).
- `kFaultWedgeLatch` is driven by `MotorArmor::wedged()`, documented as
  "the raw, unconditional stuck-encoder latch... no gating by commanded
  target" (`motor_armor.h`) — it trips almost immediately at boot simply
  because the motors are idle (identical consecutive encoder reads), and
  is DESIGNED to clear automatically the moment `position()` changes
  (`updateWedgeDetector()`: `wedgeLatched_ = false` on any position
  delta). It never cleared in this session, for either motor, at any
  point during an active twist — this is the SAME underlying signal as
  §3/4's finding (encoder position never changes) observed independently
  from the firmware's own internal state, not just the wire telemetry:
  strong corroboration that `lastPosition_` genuinely never advances,
  whatever the root cause turns out to be.

No `kFaultI2CNak` (bit 2, declared/not-yet-wired) or unexpected bits
observed at any point.

### 7. TLM continuity

120s idle window (no commands), USB direct: 1875 frames, `seq` 826→2700
contiguous (span == count), `tlm_drop_rate() == 0.0000` — matches
spike-001's "~0 drop" expectation. Observed cadence was `15.62 Hz`
(one frame every ~64ms), notably below the `kPrimaryPeriod` 25 Hz/40ms
comment target in `telemetry.h` — worth a closer look in a future ticket
(the `runAndWait`/`sleepUntil` schedule below sums to ~24ms of explicit
waits per cycle, well under 64ms, so the extra time is being spent
somewhere in the per-device I2C transactions/collect calls themselves,
not accounted for by the visible wait budget) but is NOT itself a gate
failure — continuity (zero drops) is what this ticket's acceptance
criterion actually asks for, and that passed.

### 8. `runAndWait`/`sleepUntil` schedule (source/main.cpp)

```
229:    runAndWait(kSettle, [&] {           // >=4ms: L encoder settling, meanwhile --
234:    runAndWait(kClear, [&] {  // >=4ms: brick clears L's duty write, meanwhile --
259:    runAndWait(kSettle, [&] {  // >=4ms: R encoder settling, meanwhile --
315:    sleepUntil(cycleStart, kCycle);  // pace to ~16ms; covers post-R-write
```
(`kSettle=4, kClear=4, kCycle=16`, `source/main.cpp` lines 88-90.) Matches
the file's own three-`runAndWait`-block-plus-final-`sleepUntil` shape
exactly — no drift from the archived plan's schedule.

### Investigation into the drive-path failure (not resolved)

Traced the full path from `App::Drive::setTwist()`/`tick()` through
`BodyKinematics::inverse()` (correct: `vL=vR=150` for `v_x=150,omega=0`),
`Devices::NezhaMotor::setVelocity()` (`mode_=Active`,
`pidEnabled_` defaults `true`), `MotorVelocityPid::compute()` (feed-
forward alone, `kff=0.00135 * 150 ≈ 0.2` duty, well above
`outputDeadband_`'s `0.03` default — should never be gated to zero
regardless of the integrator/deadband terms), through
`MotorArmor::armoredWrite()` (no dwell/reversal reason to suppress a
same-sign first write) to `NezhaMotor::writeRawDuty()`/`writeMotorRun()`'s
actual `I2CBus::write()` call. Nothing in a static read of this path
explains a hard, unconditional zero — by hand-calculation the PID output
for `v_x=150` should be a clearly-non-deadbanded ~20% duty. Config
wiring was checked too: `Config::defaultMotorConfigs()`
(`source/config/boot_config.cpp`) bakes real, non-zero gains
(`kp=0.0014, ki=0.005, kff=0.00135`), and `main.cpp`'s
`toDeviceMotorConfig()` passes them through unchanged.

One live-memory inspection was attempted via `pyocd`/`arm-none-eabi-gdb`
(`.claude/rules/debugging.md`'s documented workflow) to read
`main::motorL`/`main::motorR`'s actual runtime field values (e.g.
`lastWrittenPct_`, `velocityTarget_`) during an active twist. The `target
remote :3333` attach itself appears to have forced an unrequested
firmware reset (the dump read back as all-zero, pre-constructor `.bss`
state, including a null `bus_` reference, and gdb's own `detach` reported
"The program is not being run") — no `monitor reset halt` was ever sent
by this session, so this was NOT intentional and is flagged here as a
process note: **a bare `target remote` attach on this pyOCD/target combo
is not safely non-destructive as documented and should not be reused for
live inspection without first confirming `pyocd`'s connect-mode default.**
The robot resumed normal operation immediately afterward (telemetry kept
flowing, both before and after — confirmed by a raw read showing 60
lines/3s immediately after) and no further live-memory inspection was
attempted; all findings above are from black-box (wire-protocol-only)
observation, not from the aborted memory read.

Given the defect is real, reproducible, and consistent across every
parameter tried, but its exact root cause was not pinned to a single
line, this ticket does not attempt a code fix (per its own Implementation
Plan: verification only, defects get reported/reopened, not silently
patched here) and does not reopen a specific upstream ticket — narrowing
"is this Drive/kinematics (006), NezhaMotor/PID/armor (002/003), or the
main-loop request/collect sequencing (008) itself" needs either a
non-destructive live-memory session (redo the `pyocd` attach correctly,
confirming connect-mode first) or physical eyes/ears on the bench rig
(neither available to this session) before a fix can be scoped safely.

**Postscript**: no code-level cause exists — see "Root Cause" below. The
static-code trace above was not wrong to fail to find a bug: there was no
bug in the traced path. It is left in place as an accurate record of the
investigation actually performed, and because the trace itself remains
useful documentation of the drive path's call chain.

## Root Cause (identified after this session, by the team-lead)

The bench rig's **motor-power switch was off** for the entirety of the
first pass above (stakeholder-confirmed physical fact after the
team-lead's own re-run raised the question). This fully explains every
observation in the first-pass Results without requiring any firmware
defect:
- The Nezha bricks' onboard logic (I2C interface, encoder register,
  `0x46`/`0x60` command decode) runs off USB/logic power and is
  independent of the separate motor-driver power rail — so `conn_left`/
  `conn_right=True` (genuine I2C ACKs) is exactly what unpowered-driver
  operation looks like: the brick is alive and answering, it just cannot
  spin its output stage.
- With no physical rotation possible, `enc_left`/`enc_right` genuinely
  never change — not a stuck read, a stuck write, or a gated PID; the
  wheel really was not turning, full stop.
- `MotorArmor::wedged()` correctly and honestly reported this: its
  contract is "raw, unconditional stuck-encoder latch" (no gating on
  commanded target), and a driver-unpowered wheel is indistinguishable
  from a genuinely stuck one by that detector's own design — it did
  exactly what it was built to do.
- The deadman/ack-ring/telemetry/relay evidence needed no correction —
  none of it depends on motor power, and none of it was wrong.

Confirming evidence: the team-lead re-ran a 4s `twist(150, 0)` on the
SAME already-flashed firmware, same robot, with power switched on:
`enc` climbed to `(616, 605)` mm, peak `vel=174` mm/s, a clean
duration-expiry stop, `event_bits=0x3` (`kEventDeadmanExpired |
kEventBootReady`). This session's own re-run (below) independently
reproduces that result plus reverse/pivot/relay/stop coverage the
team-lead's single run didn't need to repeat.

**Process note carried forward regardless of root cause**: physical
bench state (power switches, connector seating, etc.) is easy for a
software-only verification session to take on faith rather than confirm
directly, and this session had no way to visually/physically check the
rig. `.claude/rules/hardware-bench-testing.md`'s "seen working on the
stand" standard is partly about exactly this — a future session with the
same "acks fine, zero motion" signature should suspect bench power/
wiring FIRST, alongside (not instead of) a code-level trace, especially
when (as here) the trace itself turns up no gating explanation.

## Results — Re-run (2026-07-14, motor power confirmed ON)

Same flashed firmware the whole time (re-flashed once more mid-session
only to get a clean-reset capture for the fault-bit correlation check
below — same `MICROBIT.hex`, no rebuild). All speeds ≤200 mm/s per this
re-run's own instruction.

### Forward twist

`twist(v_x=150, omega=0, duration=3000)`, USB direct:
```
ack: AckEntry(corr_id=1, ok=True, err_code=0)
first movement: t=0.14s  enc=(11.5, 8.4)
final:          enc=(473.32, 464.68)  vel=(14.12, 23.53)  active=False  fault=1  event=3
```
~470mm over 3s at 150mm/s (~450mm expected) — both wheels climbing
together, right order of magnitude. `fault=1`: `kFaultI2CSafetyNet` only
— `kFaultWedgeLatch` is CLEAR now that the wheels genuinely move (see
the fault-bit correlation note below). `event=3` at expiry
(`kEventDeadmanExpired | kEventBootReady`), matching the team-lead's own
`0x3` observation.

### Reverse twist

`twist(v_x=-150, omega=0, duration=2500)`:
```
ack: AckEntry(corr_id=1, ok=True, err_code=0)
enc_before=(-4.80, -3.11)  enc_after=(-391.14, -388.03)  vel=(-8.87, -31.74)
```
Both encoder counts fell (more negative) together, ~386mm over 2.5s at
150mm/s (~375mm expected) — clean reverse.

### Pivot twist (omega-only)

`twist(v_x=0, omega=1.2, duration=2500)`:
```
ack: AckEntry(corr_id=1, ok=True, err_code=0)
enc_before=(-7.74, 5.87)  enc_after=(-197.40, 192.64)  vel=(-3.51, 66.65)
```
Left and right counter-rotate cleanly (left negative, right positive),
magnitudes nearly symmetric as expected for a pure in-place turn. Expected
per-wheel speed = `omega * trackwidth/2 = 1.2 * 64 = 76.8mm/s`; over 2.5s
≈192mm — matches the observed `enc_after` magnitudes closely.

### `stop()` immediate-stop check (USB direct)

Armed `twist(150, 0, duration=5000)`, let it run ~1s, then sent an
explicit `stop()` mid-motion (well before the deadman would have expired
on its own):
```
stop ack: AckEntry(corr_id=2, ok=True, err_code=0)
t=0.11  enc=(185.4,180.1)  vel=(110.6,115.4)  active=False
t=0.16  enc=(184.6,179.7)  vel=( 74.1, 79.0)
t=0.21  enc=(181.1,176.8)  vel=( 11.3, 18.4)
t=0.32  enc=(180.9,176.6)  vel=(  6.7, 11.1)   <- encoder frozen from here on
... (holds exactly (180.9, 176.6) for the remaining ~1.1s watched)
```
Encoder position stops incrementing within ~0.2s of the `stop()` ack and
holds flat for the rest of the watch window — a real, fast physical stop.
The small residual `vel` reading (6.7/11.1 mm/s) is the EMA velocity
filter's own decay tail, not real motion (position is frozen at the same
instant).

### Motion over the relay (`!GO` data plane)

`SerialConnection.connect()` on `/dev/cu.usbmodem2121302` again
classified `role=RADIOBRIDGE` and entered the data plane
(`relay_config: channel 0 group 10 mode RAW250`, `entered_data_plane:
True`). `twist(v_x=150, omega=0, duration=2500)` over that connection:
```
ack: AckEntry(corr_id=1, ok=True, err_code=0)
enc_before=(11.32, 9.41)  enc_after=(398.23, 389.24)  vel=(3.28, 18.23)
```
~388mm over 2.5s at 150mm/s (~375mm expected) — real wheel motion
through the radio relay, not just an acked-but-inert command. A follow-up
relay run added an explicit mid-motion `stop()` (same shape as the USB
check above) to directly confirm `stop()` itself round-trips and acts
over radio, not just the natural duration-expiry stop:
```
stop ack: AckEntry(corr_id=2, ok=True, err_code=0)
t=0.11  enc=(581.4,563.6)  vel=(119.8,113.8)
t=0.21  enc=(575.8,560.6)  vel=(  1.9, 18.6)   <- encoder frozen from here on
... (holds exactly (575.8, 560.6) for the remaining ~0.7s watched)
```
Both ack ring and physical stop confirmed over the relay path
independently of the USB-path check above.

### `fault_bits` bit 0 (`kFaultI2CSafetyNet`) — boot-time latch, not continuous

Re-examined this session's own earlier clean-reset boot capture (the
telemetry-from-power-on evidence in the "First pass" Results §2 above,
captured with a controlled DTR reset edge and ZERO commands ever sent in
that process):
```
t=1.606  now=2176  seq=46  fault=0 event=0  [boot]   <- last preamble frame
t=1.661  now=2216  seq=47  fault=1 event=2  [MAIN]  <- FIRST main-loop frame
```
`fault_bits` bit 0 is ALREADY set on the very first main-loop frame
(`seq=47`), in the exact same frame `event_bits` first shows
`kEventBootReady` (`Preamble::done()`'s first-true transition) — i.e. the
I2C clearance safety-net trip is coincident with preamble→main-loop
handoff (plausibly preamble's own `hardReset()`-driven back-to-back
device-detection writes, per this ticket's own hypothesis), NOT
correlated with any twist/drive command — no command had been sent yet
in that capture, and none exists between the two frames above.

Corroborating: across every re-run capture in this "power ON" pass
(forward/reverse/pivot/stop/relay, several twist commands, real
sustained I2C write traffic while driving), `fault_bits` stayed at
exactly `1` throughout and never changed value again once past that
first main-loop frame — consistent with a one-shot latch that fires once
at/near boot and then simply never resets (matches
`I2CBus::clearanceSafetyNetCount()`'s own monotonic, never-cleared
counter semantics — see the code citation in the First-pass §6 notes).
Telemetry does not expose the raw counter value, only the boolean
`count() > 0` bit, so a small number of ADDITIONAL trips during active
driving cannot be ruled out from the wire alone — but they would not
change the observable bit either way, and there is no behavioral signal
(no stall, no dropped ack, no missed cycle) correlated with driving
activity that would suggest ongoing trips.

**Verdict: boot-time one-shot, not continuous — acceptable, not a defect
for this ticket.** Recorded here as a documentation gap worth closing in
a future ticket: `telemetry.h`/`main.cpp` should probably say explicitly,
next to the `kFaultI2CSafetyNet` bit definition, that it is expected to
latch once during preamble and is not itself actionable bench evidence
of an ongoing problem — a future reader hitting `fault=1` on a healthy
robot should not chase it.

### Session end

Explicit `stop()` sent and acked; final telemetry read: `active=False`,
motors idle, robot still on the stand, wheels off the ground throughout.
