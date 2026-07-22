# Sprint 115 Bench Checklist — Gut S1: Motion-Stack Excision + Minimal Per-Cycle Telemetry

> # STAKEHOLDER-RUN — NOT AGENT-EXECUTED
>
> No agent working this sprint had hardware access — at ticket 010's execution
> time (2026-07-21) there was no `/dev/cu.usbmodem*` port and `pyocd list`
> reported "No available debug probes are connected" (re-verified
> immediately before writing this document). Every check below must be run
> by a human at the bench, on the real robot mounted on its stand (wheels
> clear of the surface — see
> [`.claude/rules/hardware-bench-testing.md`](../../.claude/rules/hardware-bench-testing.md)).
> This file is sprint 115 ticket 010's fallback deliverable, modeled on
> sprint 114's own precedent
> ([`sprint-114-config-and-deadband.md`](sprint-114-config-and-deadband.md)).
> Everything in this document was verified by exercising the equivalent
> path in the **sim** (`SimLoop`/`libfirmware_host`, no ARM hardware) —
> see ticket 010's completion notes for that sim dry-run's results — but
> **no claim below has been confirmed on real hardware**. Treat every `[ ]`
> as genuinely open.

## What sprint 115 changed, and why this checklist exists

Sprint 115 deleted the executor/pilot/Ruckig motion stack wholesale
(`src/firm/motion/`, `App::Pilot`, `App::HeadingSource`, `vendor/ruckig/` —
~164 KiB flash, confirmed by this ticket's own `python build.py` run: FLASH
now 136388 B / 372736 B = 36.59% used) down to a minimal
command-controlled-speed base (velocity-PID motor control, dead-reckoned
odometry, OTOS + rate-limited line/color sensing), and rewrote the
`Telemetry` wire frame field-for-field (timestamped `EncoderReading`/
`OtosReading` objects, one `flags` bit-string, a single ack slot, packed
`line`/`color` words) emitted **every loop iteration** (primary period now
equals the cycle period, 20 ms — was 40 ms). It also bumped the
persisted-tuning schema version 1→2 (blob 110→85 bytes) because the blob
layout dropped its planner-config slot. None of this is testable by an
agent:

- The robot's real motor/encoder/OTOS/line/color hardware, and the real
  velocity-PID's response to it, only exist on the bench.
- The persisted-tuning store is backed by `MicroBitStorage` (real on-chip
  flash) — there is no host-build stand-in.
- The one-time schema-version wipe + radio-channel re-pick is a real,
  physical first-boot side effect on a device that previously ran
  older-schema firmware.

**Confirmed working (agent-side, this ticket, 2026-07-21):**
`python build.py` builds both the firmware hex AND the host sim lib clean
(`v0.20260720.3`); `uv run python -m pytest` is green — **1183 passed, 13
skipped, 10 xfailed, 1 xpassed** — including one expected
`PytestUnhandledThreadExceptionWarning` from `test_set_origin.py`
(`robot_radio.planner.tour` failing to import `telemetry_pb2.
ACK_STATUS_DONE`, since that enum was deleted with the executor) — this is
**exactly** sprint.md's own predicted, accepted "dormant host
planner/tour code" breakage (Decision 6), not a regression; and a full sim
dry-run (`SimLoop` against the real `src/firm/` tree) of everything sim
CAN prove passed 15/15 checks — see ticket 010's completion notes for the
full transcript. **None of that substitutes for the hardware run below.**

## A note on wire commands used below

This firmware's wire is binary-only, **exactly three** `CommandEnvelope`
oneof arms: `twist` / `config` / `stop` (see
`src/host/robot_radio/robot/protocol.py`'s own module docstring — the
authoritative reference, more current than either `docs/protocol-v2.md` or
`docs/protocol-v3.md`, both of which are explicitly-marked-stale snapshots
predating this rebuild by several sprints; **do not** send `docs/
protocol-v2.md`'s or `docs/protocol-v3.md`'s ASCII/other-era command
grammar directly at the robot's serial/radio port). There is no `pose`,
`otos`, `get`, `stream`, `move`, `drive`, `segment`, or per-command
synchronous reply arm — a command's outcome rides the single ack slot
inside the next `Telemetry` push (`NezhaProtocol.wait_for_ack()`), and
telemetry itself is **always on** (no arming step) at ~50 Hz (every 20 ms
cycle). Every command below uses `src/tests/bench/`'s own scripts
(`twist_drive.py`, `rig_soak.py`, `rig_dev.py`, `tlm_log.py`) or a short
inline `NezhaProtocol` snippet — exact, copy-pasteable commands, not a
re-derivation of the wire format.

**A naming note on `rig_soak.py`/`rig_dev.py`**: the "Rig" name is
historical — these scripts were originally written for the separate
stationary bench test rig (`.clasi/knowledge/bench-test-rig-layout.md`),
not the robot chassis. Mechanically, though, they drive whatever two
motors are wired to ports 1/2 as **the firmware's own left/right
drivetrain** via the identical `twist(v_x, omega, duration)` call the
robot's own drive wheels use — there is no per-port addressing left on this
wire at all (`rig_dev.py`'s own module docstring). Point `--port` at the
**robot's** serial port for every command below; the scripts work
identically against the robot.

## IMPORTANT — order matters: capture the S0 baseline BEFORE flashing S1

The tag `pre-gut-motion-stack` (verified present, `10c99d95`) is the
pre-deletion reference. As of this ticket, `git log
pre-gut-motion-stack..HEAD -- src/` shows **9 commits of real source
drift** (tickets 001–009's actual deletion/rewrite work) — the tag is a
genuine pre-S1 snapshot now, not a no-op alias. **A baseline captured on
the post-gut S1 image is not a baseline** — do Section 1 first, in a
separate git worktree so your sprint-branch working tree is undisturbed,
*then* Section 2 (deploy S1) onward.

If you have independent knowledge that the robot currently sitting on the
bench is **already** flashed with a pre-115 image (i.e., nobody has
`mbdeploy deploy`'d anything from this sprint branch yet), you may treat
*that* currently-flashed image as the S0 baseline directly — skip
re-flashing the tag in Section 1, just run Section 1's `tlm_log.py`
capture step against the robot as it currently sits, **then** proceed to
Section 2. Do not guess; if you aren't sure what's currently flashed,
flash the tag explicitly (Section 1's default path).

---

## 0. Setup

```bash
mbdeploy probe              # discover/refresh the connected device registry --
                             # PROBE is live truth; `mbdeploy list`'s ROLE column
                             # is cached (see .clasi/knowledge/verify-microbit-before-flashing.md)
```

Confirm the probed device is the **robot**, not a relay dongle — both can
share `/Volumes/MICROBIT` on macOS; verify by full UID, not just presence.

Find your serial port (`mbdeploy probe` prints it, or check
`/dev/cu.usbmodem*`). Set it once for this session:

```bash
export ROBOT_PORT=/dev/cu.usbmodem2121102   # replace with your actual port
```

Robot is **on the stand, wheels clear of the surface** — safe to drive
freely per
[`hardware-bench-testing.md`](../../.claude/rules/hardware-bench-testing.md).

---

## 1. S0 baseline: pre-S1 telemetry capture (BEFORE flashing S1 — see warning above)

Build and flash the **pre-gut** image from a separate worktree, so your
sprint-branch checkout is untouched:

```bash
git worktree add /tmp/pre-gut-baseline pre-gut-motion-stack
cd /tmp/pre-gut-baseline
python build.py --clean
mbdeploy deploy --build     # flash the pre-S1 image; confirm it's the robot's UID
cd -                        # back to your sprint-branch checkout
```

Capture a ~2-minute baseline telemetry log against that pre-S1 image:

```bash
uv run python src/tests/bench/tlm_log.py --port "$ROBOT_PORT" --duration 120 \
    --csv /tmp/s0_baseline_tlm.csv
```

| Step | Action | Expect | Done |
|---|---|---|---|
| 1a | Ran the capture above | `wrote <N> rows` printed, N > 0 | [ ] |
| 1b | Compute the drop rate (below) | recorded as the S0 baseline number the 10-min soak (Section 7) must meet or beat | [ ] |

Compute the drop rate from the captured CSV (reuses the SAME
`tlm_drop_rate()` the codebase itself uses, not a re-derivation):

```bash
uv run python -c "
import csv
from robot_radio.robot.protocol import TLMFrame, tlm_drop_rate
with open('/tmp/s0_baseline_tlm.csv') as f:
    rows = list(csv.DictReader(f))
frames = [TLMFrame(seq=int(r['seq'])) for r in rows if r['seq']]
print(f'S0 baseline: {len(frames)} frames, drop_rate={tlm_drop_rate(frames):.4%}')
"
```

**Record the printed `drop_rate` here for later comparison in Section 7:**
S0 baseline drop rate = ______

Clean up the temporary worktree once done (optional, before or after
finishing the rest of this checklist):

```bash
git worktree remove /tmp/pre-gut-baseline
```

---

## 2. Deploy the S1 build

Back in your normal sprint-branch checkout:

```bash
python build.py --clean
mbdeploy deploy --build     # flash the S1 build; verify it's the robot's UID, not a relay dongle
```

| Step | Action | Expect | Done |
|---|---|---|---|
| 2a | `python build.py --clean` | Builds firmware hex AND host sim lib clean; note the printed FLASH usage (agent's own run measured 136388 B / 372736 B = 36.59%, ~164 KiB freed vs. pre-gut — confirm yours is in the same ballpark, not close to 100%) | [ ] |
| 2b | `mbdeploy deploy --build` | Flashes successfully; confirms robot UID | [ ] |
| 2c | Open a serial terminal (`screen "$ROBOT_PORT" 115200` or similar) briefly | Boot banner observed (`DEVICE:NEZHA2:robot:<name>:<serial>`) | [ ] |

---

## 3. Standing sensors-alive + drive gate: twist forward/reverse/pivot

```bash
uv run python src/tests/bench/twist_drive.py --port "$ROBOT_PORT" --v-x 150 --omega 0 --duration 800
uv run python src/tests/bench/twist_drive.py --port "$ROBOT_PORT" --v-x -150 --omega 0 --duration 800
uv run python src/tests/bench/twist_drive.py --port "$ROBOT_PORT" --v-x 0 --omega 0.8 --duration 800
```

Each invocation prints a `[PASS]`/`[FAIL]` line per check (`connect()`,
`twist() returns a corr_id`, `twist() ack confirmed via ack ring`,
`encoders moving during twist()`, `stop() ...`).

| Step | Command | Expect | Done |
|---|---|---|---|
| 3a | forward (`--v-x 150 --omega 0`) | `4/4 checks passed`; encoders increase | [ ] |
| 3b | reverse (`--v-x -150 --omega 0`) | `4/4 checks passed`; encoders decrease from wherever 3a left them | [ ] |
| 3c | pivot (`--v-x 0 --omega 0.8`) | `4/4 checks passed`; **the two wheels move in opposite directions** (mirror-wheel shape — one net-positive, one net-negative over the window) | [ ] |

**A known, verified host-side decode gap — do not be alarmed by it**: the
`mode` field firmware now sets while driving via TWIST is
`msg::DriveMode::VELOCITY` (`src/firm/app/robot_loop.cpp:145`,
`driving_ ? VELOCITY : IDLE`) — a value `telemetry.proto` added this
sprint (`DriveMode.VELOCITY = 5`). `protocol.py`'s own `_DRIVE_MODE_CHAR`
lookup table, however, was never given a `VELOCITY` entry, so it falls
back to `"I"` (the same character `IDLE` produces) — **confirmed on the
sim dry-run**: `tlm_log.py`'s `mode` CSV column reads `"I"` throughout an
active drive, never distinguishing "driving" from "idle" by that column
alone. This is a real, pre-existing host decode gap (not something you
did wrong) — **use the `flag_active` column (or `TLMFrame.active`,
`flags` bit 2) to confirm "motion in progress", not `mode`,** until a
future ticket adds the missing dict entry. Worth flagging to the
team-lead as a small follow-up if not already tracked.

Also confirm, while driving above (e.g. via a `rogo`-style quick read or
by eyeballing a `tlm_log.py` capture — Section 6):

| Step | Check | Expect | Done |
|---|---|---|---|
| 3d | `flag_conn_left`/`flag_conn_right` (flags bits 3/4) | both `True` while driving (motor bus connectivity) | [ ] |
| 3e | Encoder reading sample times (`enc_left_time`/`enc_right_time`) | roughly the cycle period (~20 ms) apart between consecutive frames, monotonic non-decreasing | [ ] |
| 3f | OTOS (`otos_x`/`otos_y`/`otos_heading` or the `flag_otos_present` column) | plausible, changing pose while driving (if OTOS is connected on this robot) | [ ] |
| 3g | Line sensor (4 channels) | plausible values, not all-zero/all-saturated (Section 6 shows how to capture) | [ ] |
| 3h | Color sensor (RGBC) | plausible values (Section 6) | [ ] |

**Fail (file an issue):** any check above fails, encoders don't move in the
expected direction/proportion, or a sensor reads all-zero/all-saturated/
erroring.

---

## 4. Bounded-motion safety: deadman neutralizes on silence

S1 keeps the pre-existing unified Deadman (unchanged this sprint — see
sprint.md's SUC-046, "regression-only, not new behavior"): one bounded
command, then silence, must neutralize the motors within the command's own
`duration` lease.

Save this as `/tmp/deadman_check.py`:

```python
#!/usr/bin/env python3
import sys, time
from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem2121102"
conn = SerialConnection(port=port)
conn.connect()
time.sleep(2.0)
proto = NezhaProtocol(conn)
proto.read_pending_binary_tlm_frames()  # drop stale frames

corr = proto.twist(v_x=150.0, omega=0.0, duration=300.0)  # 300ms lease, no reissue
ack = proto.wait_for_ack(corr)
print(f"twist ack: {ack}")

saw_expired = False
t0 = time.monotonic()
while time.monotonic() - t0 < 2.0:  # watch well past the 300ms lease
    for f in proto.read_pending_binary_tlm_frames():
        if f.event_deadman_expired:
            saw_expired = True
        if f.enc_left is not None:
            print(f"  t={f.t} vel=({f.enc_left.velocity:.1f},{f.enc_right.velocity:.1f}) active={f.active}")
    time.sleep(0.05)

print(f"deadman_expired event observed: {saw_expired}")
proto.stop()
conn.disconnect()
```

```bash
uv run python /tmp/deadman_check.py "$ROBOT_PORT"
```

| Step | Check | Expect | Done |
|---|---|---|---|
| 4a | `twist ack:` line | `AckEntry(..., ok=True, ...)` | [ ] |
| 4b | Printed velocity trace | wheels ramp toward the commanded 150 mm/s, THEN fall back toward 0 without any further command sent | [ ] |
| 4c | `deadman_expired event observed:` | `True` | [ ] |

**Fail (file an issue):** motors keep running past the 300 ms lease with no
further command, or `deadman_expired event observed` is `False`.

---

## 5. STOP: immediate neutral while streaming twists at ~10 Hz

Save this as `/tmp/stop_check.py`:

```python
#!/usr/bin/env python3
import sys, time
from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem2121102"
conn = SerialConnection(port=port)
conn.connect()
time.sleep(2.0)
proto = NezhaProtocol(conn)
proto.read_pending_binary_tlm_frames()

t0 = time.monotonic()
while time.monotonic() - t0 < 2.0:      # stream twists at ~10 Hz for 2s
    proto.twist(v_x=150.0, omega=0.0, duration=300.0)
    proto.read_pending_binary_tlm_frames()
    time.sleep(0.1)

stop_sent_at = time.monotonic()
proto.stop()
print("stop() sent")

while time.monotonic() - stop_sent_at < 0.5:
    for f in proto.read_pending_binary_tlm_frames():
        if f.enc_left is not None:
            print(f"  t={f.t} vel=({f.enc_left.velocity:.1f},{f.enc_right.velocity:.1f})")
    time.sleep(0.02)

proto.stop()
conn.disconnect()
```

```bash
uv run python /tmp/stop_check.py "$ROBOT_PORT"
```

| Step | Check | Expect | Done |
|---|---|---|---|
| 5a | Velocity trace during the 2s streaming window | wheels tracking ~150 mm/s | [ ] |
| 5b | Velocity trace in the 0.5s AFTER `stop() sent` | drops toward 0 within one or two frames (~20-40 ms), not a slow coast | [ ] |

**Fail (file an issue):** motors keep running noticeably after `stop()`.

---

## 6. Telemetry-as-dataset: `tlm_log.py` capture

```bash
uv run python src/tests/bench/tlm_log.py --port "$ROBOT_PORT" --duration 60 \
    --csv /tmp/s1_drive_session.csv
```

While it's capturing (60s window), drive the robot around a bit with
`twist_drive.py` invocations in another terminal (forward/reverse/pivot, a
few times) so the CSV actually captures motion, not just idle.

| Step | Check | Expect | Done |
|---|---|---|---|
| 6a | `wrote <N> rows` | N > 0 | [ ] |

Check frame rate (target ≈ 50 Hz, every 20 ms cycle):

```bash
uv run python -c "
import csv
with open('/tmp/s1_drive_session.csv') as f:
    rows = list(csv.DictReader(f))
times = [int(r['now']) for r in rows if r['now']]
deltas = [b - a for a, b in zip(times, times[1:]) if b > a]
avg = sum(deltas) / len(deltas)
print(f'{len(rows)} rows, avg inter-frame delta {avg:.1f} ms -> {1000.0/avg:.1f} Hz')
"
```

| Step | Check | Expect | Done |
|---|---|---|---|
| 6b | Printed rate | ≈ 50 Hz (every cycle) — a much higher rate than the pre-115 40ms/25Hz primary period | [ ] |
| 6c | `enc_left_time`/`enc_right_time`/`otos_time` columns | populated, monotonic, roughly consistent with each row's own `now` | [ ] |
| 6d | `line_ch1..4` columns (during a window with `flag_line_present=True`) | 4 plausible, CHANGING values, not constant/all-zero/all-saturated | [ ] |
| 6e | `color_r/g/b/c` columns (during a window with `flag_color_present=True`) | plausible RGBC values | [ ] |
| 6f | `otos_v_x`/`otos_v_y`/`otos_omega` columns (when `flag_otos_present=True`) | nonzero while driving (these are NEW this sprint — previously OTOS velocities were silently dropped) | [ ] |

**Fail (file an issue):** frame rate well below ~50 Hz, `line`/`color`
never present or never change, or OTOS velocities stay flat zero while
genuinely driving with OTOS connected.

---

## 7. Soak: ≥10 minutes at 5-10 Hz alternating commands

`rig_soak.py` is the purpose-built tool for this — it already interleaves
sine-varying `twist()` reissues (~6.7 Hz) with periodic `stop()` segments
(every 6s), tracks TLM drop rate, fault-bit regressions, and
commanded-vs-responsive encoder deltas.

```bash
uv run python src/tests/bench/rig_soak.py --port "$ROBOT_PORT" --duration 600 \
    --json-out /tmp/s1_soak_result.json
```

This takes ~10 minutes and prints progress every 10s, then a `=== RESULT
===` block.

| Step | Check | Expect | Done |
|---|---|---|---|
| 7a | No reboot during the run | No boot banner re-appears on a concurrent serial monitor (if you're also watching one); `rig_soak.py` itself doesn't reconnect mid-run | [ ] |
| 7b | `TLM drop rate` (printed) | ≤ Section 1's recorded S0 baseline drop rate (and, per `rig_soak.py`'s own threshold, ≤ 2%) | [ ] |
| 7c | `new fault bits` | `none` (a bit that was ALREADY set on the very first frame, e.g. the boot-time `kFlagFaultI2CSafetyNet` one-shot, does not count — only a bit that turns on DURING the run) | [ ] |
| 7d | `responsive intervals` | ≥ 80% (`rig_soak.py`'s own `MIN_RESPONSIVE_RATE`) | [ ] |
| 7e | `secondary TLM samples` | > 0 (the ~5 Hz `TelemetrySecondary` diagnostic frame is still flowing — untouched by this sprint) | [ ] |
| 7f | Final `PASS: True` line | printed | [ ] |
| 7g | Responsive at end | run one more `twist_drive.py` invocation right after the soak finishes — still `4/4 checks passed` | [ ] |

**Note**: `rig_soak.py`'s own `seq`-gap accounting IS the drop-rate check
(7b) — a monotonic `seq` at the doubled (20ms-cycle) rate is what "drop
rate ≤ threshold" already certifies; there's no separate seq-monotonic
check to run by hand.

**Fail (file an issue with the `--json-out` file attached):** any of the
above fails, especially `new fault bits` (regression) or `responsive
intervals` well under 80%.

---

## 8. Persisted-tuning: one-time schema wipe, then a config patch survives a power cycle

The 85-byte, schema-v2 blob layout (85 bytes, 3 chunks, `kConfigSchemaVersion
== 2`) is new this sprint. If this specific device was previously running
**any** older (pre-schema-v2) firmware that had persisted a tuning blob,
the very first S1 boot (Section 2's flash) should have wiped the ENTIRE
`KeyValueStorage` — including the co-located radio-channel key, producing
a one-time radio-channel re-pick. **This is expected, not a regression**
(sprint.md Design Rationale, Decision 3). If this device has never run any
tuning-store-writing firmware before (e.g. a fresh/never-before-flashed
board), there is nothing to wipe and you won't observe anything here —
that's also fine, just note it.

| Step | Check | Expect | Done |
|---|---|---|---|
| 8a | If driving over the radio relay: did you have to re-pick/re-pair the radio channel after Section 2's flash? | Either "yes, once" (expected) or "n/a — direct USB only, not observable on this transport, or nothing to wipe" | [ ] |

There is **no live config read-back** on this wire (the `get` arm is
reserved/pruned) — verify the persisted patch **behaviorally**, the same
way sprint 114's checklist did: push an aggressively different `pid.kp`
and watch the wheel-speed response visibly change, then confirm it
survives a power cycle. `tovez_nocal.json` boot-bakes `vel_kp = 0.002`
(the currently-active robot profile, `data/robots/active_robot.json` ->
`tovez_nocal.json`).

### 8.1 Baseline response (default `pid.kp=0.002`)

```bash
uv run python src/tests/bench/tlm_log.py --port "$ROBOT_PORT" --duration 5 --csv /tmp/pid_baseline.csv &
sleep 0.5
uv run python src/tests/bench/twist_drive.py --port "$ROBOT_PORT" --v-x 150 --omega 0 --duration 2000 --watch 1.5
wait
```
Eyeball `/tmp/pid_baseline.csv`'s `enc_left_velocity` ramp-up shape/speed to
the 150 mm/s target — this is your "before" reference.

### 8.2 Push a weaker `pid.kp`, confirm it changes behavior

```bash
uv run python -c "
import time
from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol
conn = SerialConnection(port='$ROBOT_PORT'); conn.connect(); time.sleep(2.0)
proto = NezhaProtocol(conn)
corr = proto.config(**{'pid.kp': 0.0005, 'pid.ki': 0.0, 'pid.kff': 0.002})
print(proto.wait_for_ack(corr))
conn.disconnect()
"
```

| Step | Command | Expect | Done |
|---|---|---|---|
| 8.2a | Run the snippet above | `AckEntry(..., ok=True, ...)` (not `ERR_UNIMPLEMENTED` — a live-config-apply regression if seen) | [ ] |
| 8.2b | Repeat 8.1's capture+drive | Visibly SLOWER/weaker velocity ramp toward 150 mm/s than the 8.1 baseline (a 4x weaker `kp`) | [ ] |

### 8.3 Power-cycle: confirm the tune survives

| Step | Action | Expect | Done |
|---|---|---|---|
| 8.3a | Power-cycle the robot (unplug USB / toggle its power switch), wait 5s, reconnect | Robot re-boots | [ ] |
| 8.3b | Repeat 8.1's capture+drive | STILL the weakened `pid.kp=0.0005` response from 8.2b, NOT the 8.1 baseline — `Config::PersistedTuning` reapplying the stored patch at boot, at the new 85-byte layout | [ ] |

### 8.4 Restore the default before continuing to use the robot

```bash
uv run python -c "
import time
from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol
conn = SerialConnection(port='$ROBOT_PORT'); conn.connect(); time.sleep(2.0)
proto = NezhaProtocol(conn)
corr = proto.config(**{'pid.kp': 0.002, 'pid.ki': 0.0, 'pid.kff': 0.002})
print(proto.wait_for_ack(corr))
conn.disconnect()
"
```

| Step | Action | Expect | Done |
|---|---|---|---|
| 8.4a | Run the restore snippet | `AckEntry(..., ok=True, ...)` | [ ] |

**Pass:** 8.2b and 8.3b show the SAME weakened behavior; 8.2a/8.3-implied
apply are OK-acked, not `ERR_UNIMPLEMENTED`.
**Fail (file an issue):** `config()` acks `ERR_UNIMPLEMENTED` (live apply
regression), or 8.3b reverts to the 8.1 baseline (persistence broken at
the new layout).

---

## Reporting

- All checks pass: tell the team-lead sprint 115 is bench-verified — record
  the S0/soak drop-rate numbers, the measured flash size, and the frame
  rate observed in the completion notes so they can be compared at the
  next sprint's own baseline.
- Any failure: file a GitHub issue (or a `clasi/issues/*.md` file) with the
  exact step, the observed vs. expected behavior, and any captured
  `.csv`/`.json` artifact.
- The `mode` column always reading `"I"` while driving (Section 3's
  callout) is a KNOWN gap, not something to file blind — mention it to the
  team-lead if it isn't already tracked as a follow-up.
- Two existing issues were provisionally described as "moot after the
  excision" by the gut issue: `clasi/issues/
  bench-turns-spin-forever-non-termination.md` and `clasi/issues/
  nocal-straight-terminal-wedge-needs-velocity-integrator.md`. Once this
  checklist confirms the deleted completion machinery they indicted is
  actually gone AND the robot drives cleanly without it (Sections 3-5
  above), close or re-scope both — do not close them before running this
  checklist for real.
