---
status: pending
sprint: '116'
---

# Gut to minimal firmware: tagged baseline, motion-stack excision, MOVE protocol, minimal per-cycle telemetry

## Description

Tag the current tree for recovery, then ruthlessly delete the firmware down to the minimum that supports one use case — **command controlled speed** — as a solid base for rebuilding odometry and motion planning later. The minimum surface:

- Motion: the bounded **MOVE** command — twist or wheel-speed velocity variant + stop condition (time | distance | angle) + timeout backstop + replace flag against a 1-active + 4-pending queue — plus STOP as the immediate halt. Full contract: `protocol-set-point-the-minimal-firmware-s-complete-command-surface.md`.
- Configuration parameters (boot + live patches).
- **Telemetry every loop iteration** — the tightened, timestamped frame (see the telemetry amendment issue) carrying the newest reading from every source. The host logs the stream; **that log is the dataset** for all analysis and future odometry work.
- Velocity-PID motor control.
- Per-cycle encoder + OTOS (+ rate-limited line/color) reads stored in the central latest-value structure (`RobotLoop::frame_`).

Delete first, rebuild simplest, and **verify on the hardware at every stage** (robot on stand) that the loop runs without lockups.

Stakeholder decisions (Eric, 2026-07-21), binding:

- **No measurement rings, no ring-dump command.** With a timestamped frame emitted every cycle, on-chip history buffers are redundant — the host reconstructs any window from the logged stream. Only the last reading is kept on-chip (`frame_`). The unused ring scaffolding (`measurement_ring.h`, `interpolation.h`, their test harnesses) is deleted with everything else; the tag preserves it.
- **Firmware + sim only this pass.** Host motion/tour code (planner/, path/, nav/, TestGUI tour/turn modules, ~30–40 files) stays in place, dormant/broken; its deletion is a separate follow-up. Only bench-toolchain-forced host edits land here (~10 lines in `protocol.py` + one sim-config file) plus a small TLM-to-CSV logging script.
- **`v_y` rides in `MoveTwist`**, accepted-and-ignored by differential kinematics (wire-forward for a future holonomic base). The legacy `Twist` arm stays untouched through S1 (keeps the robot drivable at the S1 gate) and is deleted in S2's protocol cutover.
- **Keep** the line/color sensor drivers and the boot preamble device-probe. Per the telemetry amendment, line/color gain a live consumer: packed `line`/`color` words in the frame, read at a rate-limited, bus-safe cadence.

## Cause

Weeks of motion-control work on the executor/pilot/Ruckig stack never produced a completing tour (turn non-termination, terminal wedge), and the planned predict-to-now arc was pre-empted before execution. The motion machinery — `src/firm/motion/` (executor 914+710 lines, jerk_trajectory, cmd.h), `app/pilot.*`, `app/heading_source.*`, `vendor/ruckig/` (~5,900 LOC, ~164 KiB flash ≈ 27% of the nRF52833's 512 KiB) — is the bulk of the firmware's complexity and none of it serves the minimal use case. The simplest controlled-speed base is easier to reach by deletion than by repair; the tag preserves everything for later recovery.

Key verified facts:

- No per-wheel command exists (envelope oneof = config/stop/twist/move) — the Wheels arm is net-new.
- `RobotLoop::frame_` (a `Telemetry::Frame`) already **is** the central latest-value structure — it needs the amendment's reshape, not a replacement.
- The deadman (`app/deadman.*` + the expiry branch in `RobotLoop::cycle()` :607–621) is the **only** neutralize-on-silence path and must survive **S1**. S2's MOVE protocol makes every motion self-bounding (stop condition or timeout), which structurally supersedes it — the deadman is deleted in S2, not S1.
- `kCycle` is 20 ms but `Telemetry::kPrimaryPeriod` is 40 ms with a doc comment claiming they match (`kcycle-kprimaryperiod-mismatch.md`) — "telemetry every loop iteration" resolves this by setting the primary period equal to the cycle.

## Proposed fix

Two stages after the tag; each ends flashable, sim-suite green for what remains, and hardware-verified.

### S0 — Tag and baseline

- `git tag pre-gut-motion-stack` on master; keep a copy of the flashed `MICROBIT.hex` outside the tree.
- Baseline bench reference: deploy current hex, capture a ~2-min telemetry log (seq continuity / drop rate) for later soak comparison.

### S1 — Excise the motion stack + minimal per-cycle telemetry (one coherent unit; no intermediate state compiles)

- **Delete**: `src/firm/motion/`, `src/firm/app/pilot.{h,cpp}`, `src/firm/app/heading_source.{h,cpp}`, `vendor/ruckig/`, and the unused ring scaffolding `src/firm/devices/measurement_ring.h` + `interpolation.h` (+ their unit-test harnesses).
- **Build edits**: root `CMakeLists.txt` — remove the ruckig `include_directories` (:245) and `RUCKIG_SOURCES` GLOB/append (:303–304). `src/sim/CMakeLists.txt` — remove `RUCKIG_DIR` (:26), pilot/heading_source from `APP_SOURCES` (:90,:92), `MOTION_SOURCES` (:124–127), `RUCKIG_SOURCES` (:131–141). Firmware CMake globs `src/firm` recursively, so directory deletion is otherwise self-cleaning.
- **Protos** (`build.py` reruns `gen_messages.py`/`gen_pb2.py`/`gen_boot_config.py` — prune protos and generators, never generated files):
  - `envelope.proto`: delete `Move` + arm 20, extend reserved to include 20; delete `PlannerConfigPatch planner = 3` from `ConfigDelta`, reserve 3. (`Twist` stays as-is through S1; it dies in S2's protocol cutover.)
  - `telemetry.proto`: **full frame rewrite per `telemetry-frame-tightening-amendment-to-gut-s1.md`** — timestamped `EncoderReading`/`OtosReading` objects, one flags bit-string absorbing all bools + fault/event bits, single ack slot, packed `line`/`color` words, clean 1-byte-tag renumber, `AckEntry`/`AckStatus`/`ExecutorState`/`HeadingSourceStatus` deleted, `DriveMode` moved here from planner.proto.
  - Delete `planner.proto` and `motion.proto`; delete `PlannerConfigPatch` from `config.proto`.
  - `src/scripts/gen_boot_config.py`: remove `defaultPlannerConfig()` emission + planner helpers.
- **Firmware**: `main.cpp` — drop pilot/heading/executor includes + construction (:147–161) + ctor args. `robot_loop.{h,cpp}` — remove `Pilot&`, `handleMove`, `drainPilotEvents`, all `pilot_.*` calls, the MOVE dispatch case; reshape `updateTlm()` per the amendment (readings with sample times, flags assembly, line/color staging); keep the timing schedule, the motor request/tick interleave (:579–582, load-bearing for the 0x46 encoder-select latch), comms pump, the deadman expiry → `drive_.stop()`, and the kPace `applyOtosSample` + `odom_.integrate`. **Telemetry cadence: primary period = cycle period (20 ms — emit every loop iteration)**, closing `kcycle-kprimaryperiod-mismatch.md` (fix both constants' rate labels while there). `drive.{h,cpp}` — delete `configure()`/`actuationLag_`/accel members → pure twist→`BodyKinematics::inverse`→`setVelocity` follower; `setTwist` gains the ignored `v_y` param. `odometry.{h,cpp}` — drop `lastDistance`/`lastHeadingDelta` (executor-only accessors). `telemetry.{h,cpp}` — Frame reshape per the amendment. Line/color reads: rate-limited in-driver, kPace block, at most one of the two per cycle — never naive per-pass bus reads (the 098-004 regression is the standing precedent).
- **Persisted tuning — CRITICAL**: `persisted_tuning.{h,cpp}` drops the planner slot (blob 110→85 bytes, chunks 4→3) and **`kConfigSchemaVersion` must bump 1→2 in the same commit** — without the bump, `load()` deserializes the old blob's planner floats as the OTOS calibration section (silent corruption); with it, version mismatch → clean wipe. Expected first-boot side effect: the wipe erases the whole KeyValueStorage including the radio-channel key → one-time radio channel re-pick (goes in the gate checklist).
- **Sim lockstep**: `src/sim/sim_harness.h` — strip executor/pilot/heading includes, members, `configurePlanner()`, accessors; `src/tests/sim/support/wire_test_codec.*` — strip MOVE helpers. Optional: extend `OtosPlant` to emit modeled v_x/v_y (currently hard-zeroed, `sim_plant.cpp:221-226`) so the `OtosReading` velocities are assertable in sim.
- **Host (bench-toolchain-forced minimum)**: `robot/protocol.py` — decode the new frame (nested readings, flags-derived properties, single ack), repoint DriveMode from planner_pb2 → telemetry_pb2; `calibration/sim_boot_config.py` — drop the `planner_pb2.HeadingSourceMode` use; `robot/nezha_state.py`/`robot_state.py` — adapter mapping readings + flags onto existing attribute names. **New `src/tests/bench/tlm_log.py`**: stream frames → CSV (one row per frame, all reading fields + times + flags) — the dataset-construction tool the minimal base is designed around.
- **Tests**: delete the ~40 executor/pilot/tour/ruckig harnesses + pytests + bench scripts (move_queue, profiled_motion, heading_source, pilot_distance_trim, deadband_terminal_correction, behavior_lock, boundary_velocity, jerk_trajectory, motion_executor, tour/segment bench scripts, parked-094) plus the measurement_ring/interpolation harnesses. Edit survivors (app_robot_loop, app_drive, app_telemetry, config_gate, persisted_tuning → version 2 / 85-byte blob, sim_harness_configure, wire codec suite). Post-gut green bar: app_comms/deadman/drive/odometry/preamble/robot_loop/telemetry, config_gate, devices_*, persisted_tuning, wire codec/differential/fuzz, sim system straight_twist / scripted_twist_demo / sim_api / sim_boot_config_parity / sim_configure_from_robot.

### S2 — MOVE protocol cutover (full contract: `protocol-set-point-the-minimal-firmware-s-complete-command-surface.md`)

- `envelope.proto`: new `Move` arm **21** (`MoveTwist{v_x, v_y, omega} | MoveWheels{v_left, v_right}` velocity oneof + `time|distance|angle` stop oneof + required `timeout` + `replace` + `id`); **delete the `Twist` arm → reserve 19**; delete the ConfigDelta `watchdog` arm → reserve 4.
- New `Motion::StopCondition` (tiny: kind + threshold + activation baselines from clock/`App::Odometry`; `tick()` → stop) and `App::MoveQueue` (1 active + 4 pending; replace flushes + preempts; ERR_FULL on overflow; completion ack against `Move.id`; timeout → stop + flags bit 15 fault). `App::Odometry` gains a `pathLength()` accessor.
- **Delete `app/deadman.*`** — every motion is self-bounding; host silence ends with the queue drained and motors stopped. STOP remains the immediate halt.
- **Validated trap (still applies)**: `drive_.tick()` restages targets every cycle — the active MOVE's velocity stages **through `Drive`** (`setTwist`/`setWheels`, last-wins), never writes motors directly; `Drive::stop()` zeroes both.
- Host: `NezhaProtocol.move_twist(...)` / `move_wheels(...)` / `stop()`; `wait_for_ack` unchanged.
- Tests: stop-condition units (time/distance/angle + timeout), queue semantics (chain/replace/overflow/drain), robot_loop dispatch, sim system scenarios for seamless chaining and empty-queue stop.

### Traps recap (all validated against source)

- Tuning-blob schema version bump 1→2 is mandatory with the layout change.
- The active MOVE's velocity stages through `Drive`, never writes motors directly.
- Line/color reads are rate-limited and scheduled — never per-pass bus reads.
- Reserve (don't renumber) removed proto fields in envelope.proto/config.proto (envelope 20, config 3) — MOVE-era firmware shipped on real hardware. (telemetry.proto is a clean co-deployed rewrite per the amendment.)
- Host pb2 regenerates from the protos on every build → the `protocol.py` rework is forced; everything else host-side stays dormant.
- Every-cycle telemetry doubles the frame rate (25 → 50 Hz) at ~137 B/frame (~9 KB/s armored — fine for serial CDC); the soak's drop-rate check validates it, and the radio-relay path gets its own rate check before any radio bench work relies on it.
- The dirty generated files from the prior session (`boot_config.cpp`, `envelope_pb2.py`) are superseded by regeneration from committed protos/config — don't deliberately carry their diffs.

### Process

One sprint, tickets grouped by stage (S1 excision+telemetry / S2 wheels), each stage's final ticket being its hardware gate — the sprint is bench-testable at every stage boundary. Sprint number 115 is clean (DB orphans removed, verified max = 114). Naming rules apply to all new code: lowerCamelCase functions, no units in identifiers (units in `// [unit]` tags).

## Verification

Per stage: `uv run python -m pytest` green on the surviving suite; `python build.py` builds firmware + host sim lib clean; then the hardware gate (robot on stand, wheels free, per `.claude/rules/hardware-bench-testing.md`):

1. Deploy via `just build` + `mbdeploy deploy` (hex by full UID — verify it's the robot, not the relay dongle); boot banner on serial.
2. Drive: `NezhaProtocol` over serial — twist forward/reverse/pivot with encoder readings tracking sign and magnitude (times ~cycle-period apart), mode VELOCITY, conn flags set; **S2 runs the protocol gate from the set-point issue** (MOVE × both variants × all three stop conditions, chaining, replace, ERR_FULL, stalled-timeout fault); every command acked OK via the single slot.
3. Bounded-motion safety: S1 — one bounded command then silence → deadman neutralizes within the lease; S2 — empty-queue expiry stops motors with zero host traffic (the no-deadman contract).
4. STOP: while streaming twists ~10 Hz → immediate neutral.
5. Telemetry-as-dataset: `tlm_log.py` captures a drive session → CSV with per-reading times; frame rate ≈ 50 Hz (every cycle); `line`/`color` words plausible and changing; OTOS reading carries velocities when present.
6. Soak: **≥10 min per stage** streaming alternating commands at 5–10 Hz (adapt surviving `src/tests/bench/` scripts — `twist_drive.py`, `rig_soak.py`, `pid_hold_speed.py` survive). Pass: no reboot (no banner re-emission), seq monotonic at the doubled rate, drop rate ≈ S0 baseline, no motion-timing regression from the added sensor reads, responsive at end.
7. S1 only: observe the one-time tuning-store wipe + radio re-pick; then a config patch → power-cycle → patch reapplies at the new layout.

End state: firmware = MOVE/config/stop in, per-cycle timestamped telemetry out, PID wheel-speed control, encoder+OTOS+line/color → `frame_`; ~164 KiB flash freed; the tag `pre-gut-motion-stack` preserves everything deleted (motion stack, ring scaffolding, deadman, legacy Twist).

## Related

- `protocol-set-point-the-minimal-firmware-s-complete-command-surface.md` — the S2 contract: the complete command surface (MOVE/STOP/CONFIG + text plane) and response semantics this gut converges to.
- `telemetry-frame-tightening-amendment-to-gut-s1.md` — the frame spec this issue's S1 implements (timestamped per-source reading objects, one flags bit-string, single ack, packed line/color, ~179 B → ~137 B). Plan as one sprint with this.
- `kcycle-kprimaryperiod-mismatch.md` — **resolved by this issue** (primary period = cycle period; both rate labels fixed).
- `predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md` — the abandoned arc; its estimator/fusion/controller content becomes future build-out on this minimal base, with the host-logged telemetry stream replacing its ring-capture/dump mechanism as the dataset source.
- `bench-turns-spin-forever-non-termination.md`, `nocal-straight-terminal-wedge-needs-velocity-integrator.md` — blockers that indicted the deleted completion machinery; moot after the excision (close or re-scope when this lands).
- `turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md` — the module it tracks is deleted wholesale in S1's test sweep.
- `on-chip-fake-otos-test-device.md` — future OTOS-present bench regime; unaffected (note: its Context's "OTOS on a servo port" premise is wrong — the OTOS is on the robot's I2C bus).
- `cycle-order-ab-verdict-e7fb9be2-is-worst-recommend-b.md`, `cycle-order-reorder-experiment-ab-before-hardware.md` — cycle-order decision; the pilot-ordering half dies with the pilot, the drive-vs-motor-tick placement question survives.
- `sim-loop-hook-registration-race-with-tick-thread.md` — small co-located fix that could ride the same sprint.
