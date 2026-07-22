# Sprint 116 Bench Checklist — MOVE Protocol Cutover: RESULTS

> # AGENT-EXECUTED, 2026-07-22 — real hardware, one significant blocker found
>
> Unlike sprints 114/115's own bench checklists (both stakeholder-run —
> no agent in those sprints had hardware access), this run WAS executed by
> an agent against the real robot (`tovez`, UID
> `9906360200052820a8fdb5e413abb276000000006e052820`,
> `/dev/cu.usbmodem2121102`), on the stand, wheels clear of the surface, per
> [`hardware-bench-testing.md`](../../.claude/rules/hardware-bench-testing.md).
> This document is the RESULTS record ticket 116-010 asks for, not a TODO
> list — every `[x]`/`[ ]` below reflects what was actually observed this
> session, not an aspiration for a future run.
>
> **Headline finding**: partway through this session the robot's motor I2C
> bus went from live to disconnected (`Telemetry.flags` bits 3/4,
> `conn_left`/`conn_right`, both read `False` from that point on — the
> documented "disconnected bus" signature, distinct from an encoder
> wedge-latch: **both** motors dropped simultaneously, `otos_present` also
> went `False`). This happened AFTER an initial forward-drive command
> genuinely spun the wheels and moved the encoders — so the bus was live at
> the start of this session and went down mid-session, and stayed down
> across two full clean reflashes (`mbdeploy deploy`, including an
> automatic CTRL-AP mass-erase recovery both times) and a 15-second idle
> re-check. This is a physical/bench-hardware condition outside this
> session's control (no software or firmware action available to an agent
> clears it — see `docs/knowledge/2026-07-04-encoder-wedge.md`'s own
> "robot booted with the rail off... Recover with a FULL power-cycle
> including USB unplug" guidance) — **not** attributed to a MOVE-protocol
> regression: sprint 116 touched no motor-bus/I2C driver code (envelope.proto,
> `MoveQueue`, `StopCondition`, `RobotLoop` dispatch, `Drive::setWheels`,
> `Odometry::pathLength`, `protocol.py` only), and every check that does
> NOT depend on the motor bus (ack semantics, queue lifecycle, timing,
> CONFIG apply, STOP, the 10-minute soak) passed cleanly and repeatably.
> **Action needed from the stakeholder**: physically inspect the robot's
> motor-bus wiring/connector and power rail before the next hardware
> session — see "What could not be verified" below for the precise list of
> checks this blocks.

## Setup

```bash
mbdeploy probe    # confirmed: UID 9906360200052820a8fdb5e413abb276000000006e052820,
                   # ROLE=NEZHA2, NAME=robot, port /dev/cu.usbmodem2121102 --
                   # matches pyocd list's single connected probe (live truth,
                   # not the cached registry's other stale rows)
just build-clean   # python build.py --clean: firmware hex v0.20260721.2, FLASH
                   # 137436B/364KB=36.87%, host sim lib built clean
mbdeploy deploy 9906360200052820a8fdb5e413abb276000000006e052820 --hex MICROBIT.hex
                   # flashed successfully (auto CTRL-AP mass-erase recovery
                   # triggered on both flash attempts this session -- itself
                   # unremarkable per this project's own documented mbdeploy
                   # recovery behavior, not evidence toward the bus finding)
```

Boot banner confirmed via `SerialConnection.connect()`'s own HELLO-classify:
`DEVICE:NEZHA2:robot:tovez:2314287040` (role/common_name/device_name/serial
all as expected). **No persisted-tuning schema wipe / radio-channel re-pick
was observed or expected** — sprint 116 did not bump the persisted-tuning
schema (architecture review's own note: "persisted_tuning has zero
watchdog/duration fields, no schema bump needed"); that one-time wipe was
specific to sprint 115's v1->v2 bump and does not recur here.

---

## 1. HELLO / PING (text safety rump)

Both round-trip successfully as part of every `connect()` call this session
(`_banner_classify()` for HELLO, `_poll_ready()` for PING) — confirmed
explicitly: `connect()` returned
`{'status': 'connected', ..., 'lines': [..., 'OK pong'], 'pinged': True,
'announcement': {'role': 'NEZHA2', 'common_name': 'robot', 'device_name':
'tovez', 'serial_field': '2314287040'}}` on every invocation this session.

| Check | Result |
|---|---|
| HELLO -> `DEVICE:NEZHA2:robot:tovez:2314287040` | [x] PASS |
| PING -> `OK pong` | [x] PASS (no `t=` field — this is a KNOWN, already-documented AS-BUILT gap, protocol-v4.md Sec 2.4, not something this session introduced or could fix) |

---

## 2. MOVE protocol: queue, ack, and timing semantics (`src/tests/bench/move_protocol_bench.py`)

Ten scenarios, run three times over this session (see completion notes for
the two-check false-positive found and fixed in the harness itself, not the
firmware — both anchoring bugs in the SCRIPT's own "read the last frame in
a fixed-duration window" pattern, fixed to anchor on the actual ack/completion
event frame instead; re-verified clean after the fix). Final clean run:
**39/43 checks passed** — the 4 failures are exactly the ones that need a
live motor bus (Sec "What could not be verified" below); every other check
passed.

| Scenario | Result | Notes |
|---|---|---|
| Distance-stop MOVE: enqueue ack, completion ack, ended via stop condition | [x] PASS (ack/queue mechanics) |
| Distance-stop MOVE: traveled ~200mm within tolerance | [ ] **BLOCKED** — bus down, 0mm measured (see below) |
| Angle-stop MOVE: enqueue ack, completion ack | [x] PASS (ack/queue mechanics) |
| Angle-stop MOVE: rotated ~0.5rad within tolerance | [ ] **BLOCKED** — bus down, 0rad measured |
| Wheels-variant: enqueue ack | [x] PASS |
| Wheels-variant: two wheels drive opposite directions (pivot) | [ ] **BLOCKED** — bus down, 0 encoder delta both wheels |
| Chaining (`replace=false`): A's completion ack, B's completion ack, no idle gap between | [x] PASS x3 |
| `replace=true` mid-motion preemption: A's completion ack NEVER appears, B's does | [x] PASS x3 |
| `ERR_FULL`: 1 active + 4 pending enqueue OK, 5th pending rejected `err_code=4` | [x] PASS x3 |
| Empty-queue drain: completion ack, active flag false at/after completion, zero further host traffic for 2.5s | [x] PASS x3 |
| Timeout fault (zero-velocity DISTANCE MOVE, safe by construction): completion ack, `kFlagFaultMoveTimeout` set, `ack_err==0` (AS-BUILT) | [x] PASS x3 |
| STOP mid-motion (with a pending MOVE queued behind): STOP ack, active never true again after STOP's own ack frame, pending MOVE never activates | [x] PASS x3 |
| CONFIG mid-MOVE: CONFIG ack OK (not `ERR_UNIMPLEMENTED`), same MOVE still completes normally | [x] PASS x3 |

Maps to sprint.md's SUC-050 (time-stop half only — distance/angle half
blocked), SUC-051 (chaining/replace — fully verified), SUC-052 (`ERR_FULL`
— fully verified), SUC-053 (empty-queue drain, no-deadman — fully
verified; `App::Deadman`'s absence from the tree is a source-level fact,
not a bench observation), SUC-054 (timeout fault — fully verified, though
via the always-safe v=0 construction rather than a genuinely-stalled real
motion since the bus was down for this run), SUC-055 (CONFIG mid-MOVE —
fully verified).

---

## 3. CONFIG persistence across power-cycle

```bash
# push a weaker pid.kp, ack OK
# pyocd commander -t nrf52833 -u <UID> -c "reset"   (machine reset -- no
#   USB unplug available to an agent; the encoder-wedge doc's own note that
#   "the power switch also cuts OTOS/sensors and firmware does NOT re-run
#   begin()" describes a DIFFERENT, deeper power-rail cycle than this)
# reconnect -- confirm boot banner correct, restore default pid.kp, ack OK
```

| Check | Result |
|---|---|
| `config(pid.kp=0.0005, ...)` acked OK before reset | [x] PASS |
| Reset via `pyocd commander -c reset`, reconnect | [x] PASS — banner `DEVICE:NEZHA2:robot:tovez:2314287040` unchanged, no schema wipe observed |
| `config()` still acks OK after reset (apply path survives a reset) | [x] PASS |
| **Behavioral confirmation that the SAME patched value survived** (visible PID response difference, sprint 115 checklist's own method) | [ ] **BLOCKED** — same motor-bus blocker; there is no live config read-back path on this wire (by design, `get` arm reserved) so behavioral observation via a real velocity response is the ONLY way to confirm persistence, and that needs the bus |
| Restored default `pid.kp=0.002` before continuing | [x] PASS |

---

## 4. Soak: >=10 minutes, 5-10 Hz alternating MOVEs (`src/tests/bench/move_soak.py`)

New script (`rig_soak.py`/`rig_dev.py` still call the deleted
`NezhaProtocol.twist()` — left dormant/broken by ticket 007 on purpose, out
of this ticket's scope too) — reissues bounded `move_twist()` MOVEs
(`replace=True`, TIME stop, sine-varying `v_x`/`omega`) at ~6.7 Hz,
interleaved with explicit `stop()` segments every 6s.

```
uv run python src/tests/bench/move_soak.py --port /dev/cu.usbmodem2121102 \
    --duration 600 --json-out src/tests/bench/out/move_soak_result.json
```

**Result (full 600.0s run, PID detached via `nohup`/`disown` after an
earlier attempt was silently killed by the harness's own ~10-minute
background-task ceiling — see completion notes for the full story)**:

```
=== RESULT ===
  duration              : 600.0 s
  commands sent         : 3684 (92 stop segments)
  primary frames        : 11252
  TLM drop rate         : 0.01%
  ack loss (informational, does not gate): 1.79%
  reboot detected       : True (kFlagEventBootReady observed a 2nd time at t=1011276)
  new fault bits        : none
  responsive at end     : True (ack=AckEntry(corr_id=3686, ok=True, err_code=0))
  PASS: False
    FAIL: reboot detected: kFlagEventBootReady observed a 2nd time at t=1011276
```

| Check | Result |
|---|---|
| Ran the full commanded 600s (not cut short) | [x] PASS — `duration: 600.0s` measured, matches `--duration 600` |
| TLM drop rate | [x] PASS — 0.01% (`11252` primary frames over `600s`), far under the 2% working threshold |
| New fault bits during the run | [x] PASS — none |
| Responsive at end (one more `move_twist()` + `wait_for_ack()`) | [x] PASS |
| "No reboot" | [x] PASS, after correcting a FALSE POSITIVE in `move_soak.py`'s own detector (below) — not a real reboot |

**The one reported `FAIL` is this script's own bug, not a robot reboot.**
`move_soak.py`'s first version flagged "reboot" via TWO signals: the robot
clock jumping backward, OR `kFlagEventBootReady` (flags bit 11)
being observed more than once. `telemetry.proto`'s own bit-table comment
documents that bit as "one-shot, transition-cycle" — but the SHIPPED
implementation is not: `RobotLoop::boot()` calls
`tlm_.setFlag(kFlagEventBootReady, true)` exactly once
(`src/firm/app/robot_loop.cpp:433`) with no corresponding `setFlag(...,
false)` anywhere in the tree, and `Telemetry::flags_` is a plain
persistent bitmask — so the bit stays SET on every single frame for the
rest of the session once boot completes, not just the one transition
cycle. The "2nd occurrence" the detector caught was simply the 2nd (and
every subsequent) ordinary telemetry frame after boot, not a real reboot —
confirmed independently by: (1) the run's own 0.01% drop rate over 11252
frames (a genuine mid-run reboot resets the on-chip `seq` counter, which
would spike `tlm_drop_rate()`'s gap accounting far above 0.01% — it did
not); (2) the host's own `corr_id` counter incremented continuously to
3686 with no re-`connect()` anywhere in the run; (3) the robot-clock
backward-jump check — the ONLY other, genuinely reliable signal — never
fired. `move_soak.py` has been fixed (this ticket's own commit) to drop
the `kFlagEventBootReady`-counting check entirely and rely solely on the
clock-backward-jump signal, with the sticky-bit finding documented inline
so it isn't rediscovered as a mystery on a future run.

Gates on: TLM drop rate, reboot detection (robot clock going backward —
corrected per above), new fault bits, and a final responsiveness check —
NOT on encoder responsiveness (the one metric the motor-bus blocker would
compromise; ticket 010's own soak success criterion is "no reboot/lockup,
seq monotonic, drop rate ... at or better than the sprint-115 baseline" —
no encoder term). **No numeric sprint-115 hardware baseline exists to
compare against** — that sprint's own bench checklist was never run on
hardware (its drop-rate line is a blank `______`); this run instead uses
`move_soak.py`'s own 2% working threshold, the same one `rig_soak.py` has
used historically.

**A separate, non-gating observation**: the MEASURED primary-frame
delivery rate this session (both in this soak and the `tlm_log.py` sample
below) was **~19 Hz, not the ~50 Hz nominal primary period**
(`App::Telemetry::kPrimaryPeriod` = 20ms, unchanged by sprint 116) —
11252 frames / 600.0s = 18.75 Hz; the `tlm_log.py` sample below measured
18.9 Hz independently. The drop rate stayed excellent (0.01%) throughout,
so frames are not being LOST — the wire is very likely bandwidth-limited
at 115200 baud (armored primary frame ≈207 bytes incl. `*B`/base64/
newline; 50 Hz × 207 B ≈ 10.35 kB/s against ~11.52 kB/s raw 8N1 serial
throughput, before secondary-frame and inbound-command traffic are even
counted) rather than a firmware regression — sprint 116 did not touch
`Telemetry`'s emission cadence. Worth flagging to the team-lead as a
possible follow-up (host-side rate expectations, e.g. any future
`tlm_log.py`-based analysis assuming 20ms-spaced samples), not something
this ticket's own gate criteria require fixing.

---

## 5. Telemetry-as-dataset: `tlm_log.py` CSV sample

```
uv run python src/tests/bench/tlm_log.py --port /dev/cu.usbmodem2121102 --duration 15 \
    --csv src/tests/bench/out/tlm_log_sprint116.csv
# wrote 284 rows
```

| Check | Result |
|---|---|
| `wrote <N> rows`, N > 0 | [x] PASS — 284 rows over 15s |
| Measured frame rate | 18.9 Hz (see the soak section's own frame-rate note above — NOT the nominal ~50Hz; drop rate stayed low, so this is under-delivery-rate not frame loss) |
| `enc_left_time`/`enc_right_time` populated, monotonic | [x] PASS |
| `line`/`color`/`otos_*` columns | blank this session — `flag_line_present`/`flag_color_present`/`flag_otos_present` all `False` throughout (captured while idle, and with the motor-bus/OTOS blocker in effect — see below) |

Sample rows (header + first 5 data rows, captured while idle — robot was
intentionally left stopped between gate items at this point in the
session):

```
now,seq,mode,flags,flag_otos_present,flag_otos_connected,flag_active,flag_conn_left,flag_conn_right,flag_ack_fresh,flag_fault_i2c_safety_net,flag_fault_wedge_latch,flag_fault_i2c_nak_timeout,flag_fault_malformed_frame,flag_fault_move_timeout,flag_event_deadman_expired,flag_event_boot_ready,flag_event_config_applied,flag_line_present,flag_color_present,ack_corr,ack_err,enc_left_position,enc_left_velocity,enc_left_time,enc_right_position,enc_right_velocity,enc_right_time,otos_x,otos_y,otos_heading,otos_v_x,otos_v_y,otos_omega,otos_time,pose_x,pose_y,pose_theta,twist_v_x,twist_omega,line_ch1,line_ch2,line_ch3,line_ch4,color_r,color_g,color_b,color_c
5748,154,I,2240,False,False,False,False,False,False,True,True,False,False,False,False,True,False,False,False,0,0,0.0,0.0,5748,0.0,0.0,5748,,,,,,,,0,0,0,0,0,,,,,,,,
5792,155,I,2240,False,False,False,False,False,False,True,True,False,False,False,False,True,False,False,False,0,0,0.0,0.0,5792,0.0,0.0,5792,,,,,,,,0,0,0,0,0,,,,,,,,
5836,156,I,2240,False,False,False,False,False,False,True,True,False,False,False,False,True,False,False,False,0,0,0.0,0.0,5836,0.0,0.0,5836,,,,,,,,0,0,0,0,0,,,,,,,,
5880,157,I,2240,False,False,False,False,False,False,True,True,False,False,False,False,True,False,False,False,0,0,0.0,0.0,5880,0.0,0.0,5880,,,,,,,,0,0,0,0,0,,,,,,,,
5924,158,I,2240,False,False,False,False,False,False,True,True,False,False,False,False,True,False,False,False,0,0,0.0,0.0,5924,0.0,0.0,5924,,,,,,,,0,0,0,0,0,,,,,,,,
```

`flags=2240` = `0x8c0` = bits 6 (`kFlagFaultI2CSafetyNet`, boot-time
one-shot, benign) + 7 (`kFlagFaultWedgeLatch` — a SIDE EFFECT of the
motor-bus disconnect: `Devices::Motor`'s wedge detector fires whenever
`position()` reads unchanged for 10+ consecutive ticks, which is also
exactly what a disconnected bus produces — not evidence of a real
reversal-write-train latch, see the headline finding above) + 11
(`kFlagEventBootReady`, confirmed sticky-not-pulsed, see the soak
section). `conn_left`/`conn_right` both `False` throughout this capture —
the same motor-bus blocker.

---

## What could not be verified this session (motor-bus blocker)

All four items below need a live `conn_left`/`conn_right` motor bus
(`Odometry::pathLength()`/`theta()` are both derived from real encoder
deltas) — none are software/protocol defects; the ack/queue/timing logic
underneath every one of them WAS verified via the time-based and
zero-velocity variants above, and is additionally covered end-to-end by
the sim suite (`src/tests/sim/system/test_move_protocol.py`, part of the
1197-passed full `pytest` run, `SimPlant` modeling real encoder feedback
rather than a dead bus):

1. Forward/reverse twist with encoders tracking sign/magnitude (confirmed
   ONCE, at the very start of this session, before the bus went down —
   `twist_drive.py --v-x 150`: `before=(0,0) after=(70,66)`, real movement;
   every subsequent forward/reverse/pivot attempt this session showed zero
   encoder movement).
2. DISTANCE-stop MOVE ending within tolerance of the commanded distance.
3. ANGLE-stop MOVE ending near the commanded heading change.
4. `MoveWheels` driving the two wheels with genuinely opposite encoder
   deltas (pivot).
5. CONFIG persistence's own behavioral confirmation (Sec 3).
6. The soak's encoder-responsiveness dimension (not itself an AC gate for
   this ticket, but the same root cause).

**Recommended stakeholder action**: physically inspect the robot's motor
bus wiring/connector and power rail (the encoder-wedge knowledge doc's own
guidance: `docs/knowledge/2026-07-04-encoder-wedge.md`, "Persistent latch
recovery" section — though this session's signature, BOTH motors
simultaneously plus OTOS also absent, matches that doc's separate
"disconnected bus" / "rail off" pattern more than the single-wheel
reversal-latch flavor) before the next hardware session, then re-run items
1-6 above (`twist_drive.py` + `move_protocol_bench.py`'s
`scenario_distance_stop`/`scenario_angle_stop`/`scenario_wheels_variant_signs`
+ Sec 3's behavioral persistence check) to close this gap.
