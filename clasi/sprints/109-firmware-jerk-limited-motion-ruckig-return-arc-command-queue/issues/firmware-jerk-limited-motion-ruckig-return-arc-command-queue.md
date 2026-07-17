---
status: in-progress
sprint: '109'
tickets:
- 109-001
- 109-003
- 109-005
- 109-006
- 109-009
---

# Firmware Jerk-Limited Motion: Ruckig Return + Arc-Command Queue

## Context

Before the single-loop rebuild (sprints 102–107), the firmware planned jerk-limited
trajectories on-target with vendored Ruckig, and sprint 098's firmware heading PD
(`heading_kp=6`, servoed tolerance+dwell completion) landed 100% of turns within ±1°.
The rebuild deleted all of it; planning moved to the host (`src/host/robot_radio/planner/`),
which streams plain trapezoid twists at ~6.7 Hz with `heading_kp=0.4` and plan-exhaustion
completion. Result: turns regressed to ~±4° with a +15° outlier still open. The host loop
is too slow to close; planning and loop closure return to the firmware.

Everything needed is recoverable: `git show c63ec6c:` has `libraries/ruckig/` (proven
on-target on the nRF52833), `source/motion/jerk_trajectory.{h,cpp}` (clean CODAL-free
`Ruckig<1>` wrapper with a hard-won seeding contract), and `segment_executor.{h,cpp}`
(heading PD, dwell completion, divergence replan, dead-time compensation — reference
material; its 3-phase model is replaced, see below).

**Stakeholder decisions (2026-07-16):**
- Every motion command is an **arc**: linear distance + angular distance, coupled — a
  curve of constant radius (∞ when Δheading=0; pure pivot when distance=0). No separate
  TURN kind, no pivot-translate-pivot phase machine.
- TIMED command `time` = **total duration from activation**, ramps included; ramps down
  to finish at the deadline.
- Heading source: **OTOS virtually always**; automatic fallback to encoders, and *which
  source is active must be visible* (telemetry + TestGUI).
- Hard acceptance: **tours close in sim** — TestGUI → Sim → click a tour → completes,
  closes the loop, turns within **1%** — against a sim OTOS **with drift** and encoders
  **with realistic error**. Keep working until achieved or proven impossible.

## Command model (wire)

New lean `Move` message + `CmdKind::MOVE` (the dead `PlannerCommand`/`MotionSegment`
protos stay dead — wrong-shaped and would blow the 186-byte envelope static_assert):

```proto
message Move {
  float distance = 1;   // [mm] signed arc length along the path; 0 + delta_heading!=0 = pivot
  float delta_heading = 2; // [rad] signed heading change over the arc; 0 = straight
  float v_max = 3;      // [mm/s] linear ceiling (DISTANCE) / signed target (TIMED)
  float omega = 4;      // [rad/s] signed target yaw rate (TIMED only)
  float time = 5;       // [ms] 0 = distance-bounded; >0 = TIMED (total duration)
  bool  replace = 6;    // replace last queued (or the active cmd if queue empty)
  uint32 id = 7;        // host correlation id, echoed in ack + completion event
}
```

- **DISTANCE mode** (`time==0`): travel `distance` while heading changes by
  `delta_heading`, scaled together — heading reference `theta(s) = delta_heading * s/|distance|`.
  Pure pivot = `distance==0`: the rotational channel is planned instead.
- **TIMED mode** (`time>0`): ramp up to `v_max`/`omega`, hold, ramp down to finish at
  rest at the deadline — unless a successor exists (carry) or a replace lands (replan).
  This is the teleop primitive; the ramp-down tail is the deadman decay.
- TWIST/STOP/CONFIG stay wire-compatible. TWIST preempts the queue (clears it).

## Execution semantics (firmware)

**Dominant-channel planning.** Ruckig plans ONE channel per command — linear for arcs
(`|distance|>0`), rotational for pivots. The other channel is slaved by the arc ratio:
`omega_ff(t) = (delta_heading/distance) * v(t)`. The heading PD (below) corrects around
the continuous reference `theta(s)`. Curvature steps at command boundaries are absorbed
by the PD + wheel-level slew in v1 (noted as a known simplification).

**Queue**: fixed ring of 8 normalized `Motion::Cmd` entries (validated, defaults folded).
Overflow → `ERR_FULL` ack, plan untouched. Degenerate commands (zero distance+heading,
time≤0) → acked `TRIVIAL`, never queued.

**Boundary velocity (the "no decel between same-vmax commands" requirement).** Plan only
the active command, with one-command lookahead handed to Ruckig's position interface as a
nonzero `target_velocity` (verified supported: `input_parameter.hpp` c63ec6c):

```
exitSpeed(active, next):
  none, sign reversal, or pivot on either side  -> 0
  else ve = min(vmaxEff(active), vmaxEff(next))
  if next is DISTANCE: ve = min(ve, reachableEntrySpeed(|next.distance|))
  reachableEntrySpeed(d) = -k + sqrt(k² + 2·aDecel·d), k = aDecel²/(2·jerk)  // jerk==0: sqrt(2·aDecel·d)
```
Pivot→pivot chains carry rotational velocity the same way (same code path, rotational
domain); only rest-terminated pivots get the servoed dwell landing (you can't dwell at a
heading you're sweeping through — chained intermediates are encoder/OTOS-accurate,
final is servo-accurate).

**Replan triggers** (each: re-solve affected channel, seeded from the channel's OWN last
sample — never measured sensors; the 087-009 limit-cycle contract is inviolable):
(a) enqueue adjacent to active and exit speed changes >1 mm/s → `retarget(remaining)` with
new target velocity; (b) replace — tail: as (a); active: in-place `solveToVelocity` (TIMED)
or full re-activate from moving state; (c) divergence — old thresholds verbatim (5 mm
retarget / 40 mm reanchor linear, 0.3 rad reanchor rotational, 60 ms min interval;
reanchor is the one sanctioned measured-state seed, accel forced 0); (d) handoff —
activate next, velocity-continuous by construction; (e) STOP — flush queue (`FLUSHED`
events) + `solveToVelocity(0)` both channels.

**Heading loop (the turn-accuracy fix).** Sprint 098's cascade, restored in-firmware at
the 40 ms cycle: `omega_cmd = omega_ff + heading_kp*(theta_des − theta_meas) +
heading_kd*(omega_des − omega_meas)`, gains from `PlannerConfig` (bench-proven kp=6.0 in
`data/robots/tovez.json`), active whenever `delta_heading≠0` or pivoting, gated off during
terminal decel. Completion for rest-terminated commands with heading content: |err|<0.5°
AND rate<1°/s held 150 ms, STOP_TIME backstop. Distance completion: encoder-relative
travel ≥ |distance| with signed overshoot carry into same-sign successors.

**Deadman**: stays the single staleness gate. Executor re-arms it every non-IDLE cycle
with a ~300 ms lease; expiry (executor wedged or idle-host TWIST path) → flush + immediate
stop + event bit. TIMED's own deadline is the teleop decay bound.

**State machine**: IDLE → RUNNING → (handoff self-loop) → RAMP_TO_REST (empty queue at
speed; accepts mid-decel enqueue with moving-state replan) → IDLE; STOP → STOPPING
(enqueues accepted, activate at rest). Per-command events `{id, status}`:
`DONE/TRIVIAL/SUPERSEDED/FLUSHED/TIMEOUT/SOLVE_FAIL`, plus polled TLM fields
(`queueDepth`, `activeId`, `state`, `headingSource`).

## Heading source seam

`App::HeadingSource` (`src/firm/app/heading_source.{h,cpp}`): passive reader, no bus
traffic (consumes what the loop already samples — OTOS has a clean 20 ms slot in kPace;
the 098-era "OTOS tick wrecks the bus" failure mode is structurally gone).
- Policy: **OTOS whenever `present() && connected() && poseFresh()`**; automatic fallback
  to encoder-differential heading `(encR−encL)/trackwidth` after N stale cycles; re-promote
  when OTOS recovers.
- **Visibility is a requirement**: active source in every primary TLM frame + an event on
  fallback transition; TestGUI surfaces it (indicator when NOT on gyro).
- Per-robot override in robot JSON (`control.heading_source`) via `gen_boot_config.py`
  → `PlannerConfig.heading_source` (new field).

## Architecture (modules, cycle placement)

| Piece | Location | Role |
|---|---|---|
| Vendored Ruckig | `src/vendor/ruckig/` (restore via `git archive c63ec6c libraries/ruckig`) | solver; ARM glob + sim explicit list |
| `Motion::JerkTrajectory` | `src/firm/motion/jerk_trajectory.{h,cpp}` (port from c63ec6c) | per-channel wrapper; ADD `solveToState(pos, vel, vmax)` + remembered target velocity; keep seeding contract, jerk==0→trapezoid sentinel, retarget/reanchor |
| `Motion::Cmd` + `Motion::Executor` | `src/firm/motion/` (new, replaces old SegmentExecutor) | queue ring + state machine + boundary velocity + heading PD + completion |
| `App::Pilot` | `src/firm/app/pilot.{h,cpp}` | glue: enqueue/stop/telemetry accessors; `tick()` (sample→`drive_.setTwist`) and `plan()` (solve execution) |
| `App::HeadingSource` | `src/firm/app/heading_source.{h,cpp}` | as above |

Cycle placement (respecting the single-loop invariants):
- `pilot_.tick()` — motorR settle block, after `processMessage()`/deadman, before
  `drive_.tick()` (robot_loop.cpp:263-273). Sample-only (`Trajectory::at_time` ~100 µs/ch),
  builds Measured{dist, heading, headingRate, v} from Odometry + HeadingSource, applies
  dead-time lead (kDeadTime **re-derived at the 40 ms cycle** — old 120 ms value assumed a
  20 ms tick; bench-tune).
- `pilot_.plan()` — kPace block (28 ms budget), after `odom_.integrate()`. **All Ruckig
  solves here, ≤1 per cycle** (soft-double on the M4F — low-ms each; a fresh command is
  ready ~2 cycles/80 ms after enqueue, acceptable). Solve requests are event-driven
  (triggers a–e), execution is paced.
- Dispatch: `handleMove()` in `processMessage()` (acks echo `Ack.q`=depth, `Ack.rem`);
  dispatch-before-tick ordering makes replace-at-handoff deterministic.
- Wiring: `main.cpp` + mirrored in `src/sim/sim_harness.h` (sim runs the real Pilot;
  `sim_inject_command()` already injects arbitrary armored lines).

Build: root `CMakeLists.txt` re-adds ruckig include + source glob (~lines 220/270; gnu++20
already forced); `src/sim/CMakeLists.txt` adds explicit motion+ruckig source lists. New
`src/firm/motion/DESIGN.md` + root DESIGN.md map/diagram row. Protos regen via
`scripts/gen_messages.py`; host pb2 regen in the same change; host `Move` encoder next to
the existing twist armoring; `wire_test_codec.cpp` gains `armorMoveCommand()`.

## Sim fidelity (hard acceptance gate)

The sim runs the real RobotLoop against `SimPlant` (real `I2CBus` impl), so the planner is
sim-testable by construction. Required plant upgrades:
- **OTOS model with drift**: heading random-walk + rate noise + configurable bias
  [deg/min], linear scale error; configurable via SIMSET keys (≤8 kv per message — known
  truncation gotcha).
- **Encoder error model**: per-wheel tick quantization + scale mismatch + slip events
  (extend the existing rest-jitter hook in SimPlant).
- **Acceptance**: TestGUI → Sim → run TOUR_1/TOUR_2 → completes, visibly tour-shaped,
  loop closes, every turn within **1%** of commanded angle, with drift/noise enabled.
  Iterate until achieved or a written impossibility argument exists.

## Verification

- **Sim tests** (`src/tests/sim/system/`): single arc S-curve trace (jerk bound asserted);
  two same-vmax DISTANCE with no inter-command decel (velocity never dips below vmax·(1−ε));
  teleop replace stream then silence → jerk ramp to zero; pivot accuracy vs sim-OTOS drift
  + fallback-to-encoder transition (and TLM source visibility); TWIST/STOP preemption;
  queue overflow ERR_FULL; unit tests for the ring, boundary-velocity table, and the
  JerkTrajectory seeding-contract regression.
- **Bench (every sprint ends on the stand)**: sprint-1 `solve_time_characterize.py`
  (p99 solve + sample times) + `arm-none-eabi-size` flash check; teleop via gamepad
  (MOVR-stream); `bench_ruckig_motion_verify.py` + `turn_sweep.py` (±1° gate);
  `velocity_step_response.py` (no ~140 mm/s resonance resurrection under S-curve);
  tour closure notebook on hardware.

## Sprint sequencing (each ends bench-runnable; roadmap all up front per project rule)

1. **Ruckig back in the image** — vendor restore, `motion/jerk_trajectory` port +
   `solveToState`, CMake (ARM+sim), unit tests, on-target solve-time + flash gates.
   Robot behavior unchanged.
2. **Wire + queue + TIMED (teleop path)** — Move proto/regen, `Motion::Cmd` ring,
   Executor velocity mode + replace + ramp-to-rest, Pilot wiring (loop/main/sim), deadman
   lease, TLM fields. Bench: jerk-limited gamepad teleop; TWIST/STOP regression.
3. **DISTANCE arcs, at-rest chaining** — dominant-channel arc planning, heading reference
   + heading PD + dwell completion, HeadingSource seam with OTOS-first policy + visibility,
   dead-time re-derivation. Every command ends at rest. Bench: arc/pivot accuracy sweep.
4. **Cross-boundary carry + sim fidelity gate** — boundary velocities, overshoot carry,
   divergence replan; OTOS-drift + encoder-error plant models; **the sim tour closure
   demo (1% turns) is this sprint's acceptance**. Bench: two-command no-decel run,
   hardware tour.
5. **Host adoption + cleanup** — tours/TestGUI send MOVE queues instead of streamed
   twists (host planner demoted to teleop input shaping), live PLANNER config patches
   un-stubbed for gain tuning, retire dead host streaming path for tours.

## Key risks

- **Soft-double solve time** on M4F — gated sprint 1 by re-measurement; mitigation is the
  jerk==0 trapezoid sentinel per-robot, not code surgery.
- **Flash growth** (Ruckig position solvers are large; old image fit) — sprint-1 size check.
- **kDeadTime** re-derivation at 40 ms cycle — bench-tune before judging accuracy.
- **1%-in-sim gate** depends on plant fidelity as much as control — budget iteration time;
  the impossibility escape hatch is explicit.
- **Envelope size** — Move ≈ 35 bytes worst case vs 186 static_assert; compiler-enforced.
