# Hardware Bench Testing

## STOP — there is more than one robot on the USB hub

**Our robot is `tovez`. Address it by UID, never by port number.**

```
tovez  9906360200052820a8fdb5e413abb276000000006e052820   <- OURS
vizev  99063602000528205560754f2f401c2f000000006e052820   <- NOT OURS. Never touch.
```

If you are doing mainline development, you are developing on `tovez`. However, you may be told that you can also deploy to other machines, but do not do this without specific instructions. 

**Every default path currently leads to the WRONG robot**, verified 2026-08-01:

- `mbdeploy deploy` with no target auto-picks "the unique non-relay device" —
  which is `vizev` whenever `tovez` is unplugged.
- `pyocd` with no `-u` picks the only probe it can see — `vizev`.
- A remembered port number is worse than useless: **port numbers move on every
  re-enumeration.** `/dev/cu.usbmodem2121102` was `tovez` in the morning and
  `togov` (a third robot) by afternoon; `vizev` now holds `2121402`, which the
  `zavaz` relay held that same morning.

Use `mbdeploy` freely to find out what is attached -- discovery is fine and
expected:

```bash
uv run mbdeploy probe         # refresh the registry
uv run mbdeploy list          # UID -> port -> name, live
```

Identify BOTH boards, then target `tovez` and leave `vizev` alone. The rule is
about which device you act on, not about looking.

### Radio channel 3 is ours (2026-08-05)

`tovez` and the `getez` relay both run on **channel 3**, group 10. This is a
private channel, and it is not a preference — it is how the bench stays
usable.

On channel 0 (the old default) several boards answer at once: every `ID`
came back two or three times, `POSE`/`STATUS` returned another robot's
numbers as readily as tovez's, and one board flooded the channel with `DBG:`
output. That flood is self-sustaining, because protocol v5 ends a verb at
the first colon: a robot's own `DBG:` OUTPUT is a syntactically valid `DBG`
COMMAND to every robot that hears it, so robots answer each other forever
and the DBG action ring saturates. Commands sent to a shared channel also
reach every robot on it.

- firmware: `radiochan::kDefault` (`src/firm/com/radio_channel.h`), applied
  at `main.cpp`'s `radio.begin(radiochan::kDefault)`.
- relay: `!C 3` on its control plane, persisted in its flash (`?` reports
  `channel: 3 group: 10`). A relay that has been `!DEFAULTS`-ed drops back
  to 0 and must be set again.

Both ends must match or the robot is simply unreachable. If a session sees
duplicate replies to one command, check the channel before anything else.

### `mbdeploy probe` writes ports into a TRACKED file

`config/devices.json` is the registry, it is checked in, and `probe` rewrites
its `port` fields with whatever the ports happen to be right now. Since port
numbers move on every re-enumeration (above), those values are worthless to
anyone else and actively misleading. **Never commit them** —
`git checkout -- config/devices.json` before staging anything after a bench
session.

### Per-robot fleet facts (measured 2026-08-09)

- **`gopiv` has NO OTOS, NO line sensor, and NO colour sensor fitted.** Its
  telemetry `flags` word reads 216 (`0b11011000`) — motors connected, bits 0
  (otos), 13 (line) and 14 (colour) all clear — and `otos`/`line`/`color`
  never appear on a frame. This is the hardware population, not a firmware
  fault: confirmed by flashing pristine `master` and the branch under test to
  the same robot and getting the identical flags word. Encoders, pose, and
  the whole MOVE protocol work normally. Do not spend a session re-deriving
  this, and do not gate a `gopiv` run on OTOS.
- **`gopiv.json` and `togov.json` were missing
  `wheel_control.stall_{speed,demand,window}`**, which `gen_boot_config.py`
  requires with no source-side default — so `build.py` could not produce an
  image for either robot AT ALL. `gopiv.json` was filled in 2026-08-09 (held
  at 0, the documented inert state). **`togov.json` still has the gap and is
  still unbuildable.**

### Reaching a robot that hangs off `gauti` (no local USB)

`gauti` (the companion Pi) holds tovez's micro:bit on `/dev/ttyACM0`, and
tovez does not appear in `mbdeploy list` on the Mac at all. There is **no web
service on gauti** — only SSH on :22. Two things you need, both worked out
2026-08-09:

**Flash via the DAPLink mass-storage drive.** gauti has no pyocd, but the
MSD is there. The block device node MOVES (`sda` → `sdb` …) every time the
board resets, so find it by LABEL, and always confirm the Unique ID before
copying — it is the only check that you are flashing the robot you think:

```bash
scp MICROBIT.hex ros@gauti:/tmp/
ssh ros@gauti 'sudo umount /mnt/microbit 2>/dev/null
  D=$(lsblk -no NAME,LABEL | awk "/MICROBIT/{print \$1}")
  sudo mount /dev/$D /mnt/microbit
  grep "Unique ID" /mnt/microbit/DETAILS.TXT     # MUST match the target robot
  sudo cp /tmp/MICROBIT.hex /mnt/microbit/ && sync
  sleep 14; cat /mnt/microbit/FAIL.TXT 2>/dev/null'   # silence == flashed
```

Writing to a stale mount fails with **"No space left on device"** — that is
the wrong-device-node symptom, not a full disk.

**Bridge the serial port to a local PTY** so the host tooling works
unmodified. `SerialConnection` does `serial.Serial()` + `.port =` + DTR and
HUPCL fiddling, so it needs a REAL tty — a `socket://` URL will not do:

```bash
# one command: ssh -f backgrounds locally, socat stays alive as the remote
# command. Do NOT try setsid/nohup -- it does not survive the session.
ssh -f -L 7777:127.0.0.1:7777 ros@gauti \
  'exec socat TCP-LISTEN:7777,reuseaddr,bind=127.0.0.1,fork \
        FILE:/dev/ttyACM0,b115200,raw,echo=0'
```

then a local PTY pump (there is no socat on the Mac — a ~40-line
`os.openpty()` + socket select loop does it) symlinked to e.g.
`/tmp/tovez-tty`. Tear BOTH ends down before re-flashing; a second socat
fails with "Address already in use" and the stale one keeps the port.

**Do not read sub-100 ms timings through this bridge.** Every byte crosses
WiFi twice. Measured 2026-08-09: `move_protocol_bench.py` scores 48-49/57
over the bridge versus 57/57 on direct USB, and the failures MOVE between
runs — they cluster on "completion ack observed" and on travel/rotation
magnitudes, both of which are telemetry-window-bound. Verified as transport,
not firmware, by running the identical bench over the identical bridge on
pre-reorganization firmware: same 49/57, same eight checks, travel 69.0 mm
vs 61.2 mm. Protocol correctness and encoder direction survive the bridge
fine; timing numbers do not.

### The first command after a flash races the boot

`mbdeploy deploy` resets the board, and boot is not instant: the LED boot
identity, device detection with paced retries, and `RealOtos::init()`'s
~612 ms IMU calibration all run first. A bench script started immediately
after flashing can connect, send, and see no telemetry at all —
`twist_drive.py` scored 3/6 that way and 6/6 on a retry seconds later, with
nothing changed. **Sleep ~5 s after a flash**, and re-run once before
believing a failure.

### Then, before any hardware command

Confirm the row says `tovez`, take the PORT from that row for this session only,
and pass the **UID** to anything that accepts a target:

```bash
uv run mbdeploy deploy --hex ./MICROBIT.hex 9906360200052820a8fdb5e413abb276000000006e052820
pyocd gdbserver -t nrf52833 -u 9906360200052820a8fdb5e413abb276000000006e052820
```

If `mbdeploy list` does not show `tovez`, it is **unplugged** — stop and say so.
Do not fall back to "the only device present"; that is how you flash someone
else's robot. (`mbdeploy probe` prints the stale registry too, including devices
that are not connected and several rows sharing one port — `list` is the live
view, `probe` is not.)

---

The robot is **connected and mounted on a stand**. Its wheels are off the ground,
so it **cannot drive away** — it is safe to power the motors and spin the wheels
freely during verification. Use the real hardware to confirm changes, not just
unit tests.

## Bench-room lights (turn them on yourself)

The bench-room lights are on a network relay at `192.168.1.122`. **They turn off
when the stakeholder leaves the room**, which blinds the overhead playfield
camera — dark frames, or AprilTag detections dropping to zero, usually means the
lights are off, not that the camera or the tags are broken.

Turn them back on and keep working. Do not stop to ask; this is
pre-authorized (stakeholder directive 2026-07-29).

```bash
curl -s "http://192.168.1.122/rpc/switch.set?id=0&on=true"    # lights ON
curl -s "http://192.168.1.122/rpc/switch.set?id=0&on=false"   # lights OFF
```

Both return `{"was_on":<bool>}` — the state *before* the call, so
`{"was_on":false}` from the ON call means you just turned them on. Verified
working 2026-07-29.

Notes:
- `/rpc/switch.set` is the Shelly Gen2+ RPC shape; `id=0` is the single relay
  channel. `curl -s "http://192.168.1.122/rpc/switch.getStatus?id=0"` reads
  current state without changing it — the `output` field is the lights
  (`"output":true` = on).
- Prefer leaving the lights **on**. Only turn them off when explicitly asked.
- If the host does not answer, the relay is unreachable (network or power) —
  report that plainly rather than re-attributing dark camera frames to the
  camera, the tags, or the playfield calibration.

## Standing verification gate

Every firmware sprint that touches the HAL, motor control, sensing, or the
command protocol must, as part of its acceptance, **deploy to the robot and
exercise it on the stand**:

1. **Sensors are alive.** Talk to every sensor and confirm it responds with
   plausible, changing values: encoders (motor controller), OTOS
   (position/velocity), line sensor (4 channels), color sensor (RGBC), and the
   digital/analog ports.
2. **Wheels drive and encoders run.** Command the wheels (both directions) and
   confirm the **encoders increment** in the expected direction and roughly in
   proportion to commanded speed. Because the robot is on the stand, drive freely.
3. **Round-trip over the real link.** Confirm commands and replies work over the
   actual transport (serial at the bench; radio relay when testing the relay
   path).

A sprint is not "done" on tests alone — it must be seen working on the stand.

## How to deploy and drive

Build + flash with the project's deploy tool (the robot is a flashable probe):

```bash
mbdeploy probe          # discover/refresh the connected device registry
mbdeploy deploy --build # build firmware and flash the robot
```

Then open the serial port (or drive through the radio relay) and issue commands.
The current command surface is **protocol v5** — see
[docs/protocol-v5.md](../../docs/protocol-v5.md) (one uniform
`<COMMAND>[':' <data>]'\n'` grammar, both directions, text or binary,
generated from one command registry; four cleartext verbs — `HELLO`/
`PING`/`ID`/`VER` — plus a binary command plane with exactly three arms,
`move`/`config`/`stop`; and an always-on binary telemetry push) —
dispatched by
[src/firm/core/robot_loop.cpp](../../src/firm/core/robot_loop.cpp)'s
`processMessage()`. There is no bare-command REPL shape any more: every
motion is a bounded `Move` sent through `NezhaProtocol`
([src/host/robot_radio/robot/protocol.py](../../src/host/robot_radio/robot/protocol.py)),
not a hand-typed wire line — use `rogo repl` or one of the bench scripts
below rather than typing verbs directly at the serial port.

### `rogo serve` — hold the port open; share the robot between programs

**Closing the serial port RESETS the robot** (macOS HUPCL drops DTR on last
close — `clasi/issues/later/A-port-close-resets-the-robot-live-config-still-
wiped.md`), so every one-shot `rogo` invocation reboots the MCU and wipes
live-pushed config. For any session with more than one command, run the
daemon instead:

```bash
uv run rogo serve
```

It opens the serial connection ONCE, holds it for its whole lifetime, and
serves the repl grammar on TCP `127.0.0.1:7646` (`--listen HOST:PORT`, or
`$ROGO_ADDR`) to any number of concurrent local programs:

- interactively: `uv run rogo repl --connect`
- from Python: `robot_radio.io.client.RogoClient` —
  `RogoClient().cmd("drive 200")`, `.estop()`, `.subscribe_tlm()`
- from anything: newline-delimited plain text (or `{"id":..,"cmd":..}`
  JSON) in, JSON lines out — even `nc 127.0.0.1 7646`

`estop`/`halt` from ANY client jumps the daemon's command queue and aborts
an in-progress wait, and the daemon halts the robot (`halt_now`) before it
ever closes the port (Ctrl-C, SIGTERM, or a client's `shutdown` verb).
`sub tlm [N]` streams every Nth telemetry frame to that client; `status`
reports the held port, uptime, and client count.

**Full reference: `uv run rogo --agent`** — the agent manual (same
convention as `mbdeploy --agent`): complete verb grammar, the daemon's
socket protocol event shapes, `RogoClient` recipes, and the failure modes.

### Standing radio-relay bench gate (sprint 124's own acceptance gate)

For a one-shot, self-scoring run of the full protocol v5 acceptance
surface **over the radio relay** (banner-on-connect, `HELLO`/`PING`/`ID`/
`VER`, `move_wheels` start/stop with climbing encoders, enqueue+completion
acks via the ack ring, a `kFaultCommsMalformed`-stays-clear check, the
0x0A-embedding-move hardware repro, and a wire-quality measurement against
a stated loss budget), run:

```bash
uv run python src/tests/bench/radio_bench_gate.py --port /dev/cu.usbmodemRELAY123
```

`--port` is the RELAY's own serial port (the host's connection to the
radio-relay dongle, NOT the robot's direct USB port — confirm with
`mbdeploy list`'s ROLE column). Prints a PASS/FAIL line per check and
exits nonzero on any failure — see the script's own docstring for the
full flag set (a shorter `--wire-quality-duration` for iteration, an
optional `--usb-port` comparison leg).

### Quick smoke sequence (protocol v5 / MOVE-era)

Drive this either interactively via `rogo repl` (the `rogo` console script,
`src/host/robot_radio/io/cli.py`) or by running
[src/tests/bench/twist_drive.py](../../src/tests/bench/twist_drive.py) for
steps 1-4 in one shot:

```bash
uv run python src/tests/bench/twist_drive.py --port /dev/cu.usbmodem2121102
```

| Step | Call (`NezhaProtocol`) | Expect |
|---|---|---|
| Identify | connect (sends `HELLO`) | `DEVICE:NEZHA2:<name>:microbit:<serial>` banner |
| Liveness | `PING` | `PONG:t=<ms>` |
| Identity | `ID` / `VER` | `ID:<drivetrain>:<profile>:<version>` / `VER:<version>` |
| Config push | `config(**{"pid.kp": ...})` / `otos_config(...)` | ack observed in the next `Telemetry` frame's bounded `acks` ring (`corr_id` == the enqueue `corr_id`, `err == 0`) — the ring is the ONLY ack path (the older single scalar ack slot is deleted) |
| Drive (on stand) | `move_twist(v_x=150, stop_time=..., timeout=...)` | enqueue ack, then telemetry frames with `enc_left`/`enc_right`/`pose` climbing while the `Move` runs |
| Completion | *(no separate call — the same `Move` ends on its own)* | a later frame's `acks` ring carries an entry with `corr_id == Move.id` (the completion ack, `err` always 0 — timeout vs. stop-condition is `flags` bit 15, not `err`) |
| Stop (planned) | `stop()` | enqueue ack; queues BEHIND whatever `Move` is already active and only takes effect once it's that stop's turn — does **not** interrupt the in-flight `Move`. Measured 2026-07-29: sent 0.5s into a 400mm leg, the robot travelled the ENTIRE leg (39.8cm) and `flags` bit 2 (`kFlagActive`) stayed set for 5.9s. |
| Halt now | `estop()` | enqueue ack; zeroes wheel targets AND clears the planner queue in the SAME cycle — `flags` bit 2 (`kFlagActive`) drops within one cycle, encoders hold. Measured 2026-07-29 on the same repro: 2.9cm of travel, 0.10s to clear. This is the verb every "stop the robot now" call site (geofence, Ctrl-C, panic) must use — see `.claude/rules/playfield-testing.md`'s "Halting" section. |
| Odometry / OTOS | read `Telemetry.pose` / `Telemetry.otos` off any frame | `pose` always present; `otos` valid when `flags` bit 0 is set |
| Line / color | read `Telemetry.line` / `Telemetry.color` off any frame | valid when `flags` bits 13/14 are set — 4 plausible channel values each |

For the fuller MOVE-protocol surface (distance/angle stop conditions, the
`wheels` velocity variant, chaining, `replace=True` preemption, the 5-deep
`ERR_FULL` queue limit, the no-deadman empty-queue drain, the `timeout`
safety backstop, `STOP` mid-motion, and a `CONFIG` patch arriving mid-`Move`),
run the full bench-gate script:

```bash
uv run python src/tests/bench/move_protocol_bench.py --port /dev/cu.usbmodem2121102
```

which prints a PASS/FAIL line per scenario. See
[src/tests/bench/](../../src/tests/bench/) for the rest of the bench-script
catalog (`tlm_log.py` for a flat CSV telemetry capture,
`move_accuracy_bench.py`/`turn_prediction_capture.py` for accuracy
characterization, `velocity_step_response.py` for sensor/PID
characterization).

## The robot must be STILL when it boots (OTOS gyro calibration)

`RealOtos::init()` runs a one-shot ~612ms IMU bias calibration at every
boot, and nothing waits for it or checks stillness — whatever the gyro
feels in that window becomes "zero rotation" for the whole session. A robot
that boots while being handled (battery swap, being carried, set down)
drives with a poisoned heading until its next boot.

Measured on `tovez` 2026-08-08: booted mid-battery-swap → **+1.44 deg/s**
standstill heading drift (camera-confirmed motionless) plus ~3cm/30s
position creep (the phantom rotation sweeping the 47.8mm sensor lever arm).
One still reboot → **−0.006 deg/s**, after which it held a 90cm centre-line
pass to ±1.6cm on odometry alone. Symptoms: heading rotates while parked;
pivots complete by odometry while the camera sees a fraction of the turn;
GO_TO curves off-line. Do not blame wheel scrub or the Navigator, and do
not steer from the camera to compensate — reboot instead.

**Rescue without flashing** — `gauti` (the companion Pi on the robot)
holds the micro:bit's direct USB port; closing it drops DTR and resets the
nRF. With the robot PARKED and untouched:

```bash
ssh ros@gauti 'ls -l /dev/serial/by-id/'   # confirm tovez UID 990636...b276...
ssh ros@gauti 'timeout 1 cat /dev/ttyACM0 >/dev/null'   # DTR drop = reset
```

Wait ~5s (boot + preamble + calibration), then VERIFY: standstill heading
drift over 20–30s must be ≈0 before driving. A reboot zeroes the pose and
wipes live-pushed config — re-seed and re-push after.

## Safety notes

- On the stand the wheels spin free; still avoid loose clothing/fingers near the
  drivetrain and the gripper.
- A locked/protected nRF recovers via `mbdeploy`'s automatic mass-erase on a
  failed flash (no manual step needed).
