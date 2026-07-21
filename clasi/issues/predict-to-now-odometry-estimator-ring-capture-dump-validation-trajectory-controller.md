---
status: pending
sprint: '117'
---

# Predict-to-now odometry: estimator + ring capture/dump + validation + trajectory controller

> **Re-scope note (2026-07-21, post-gut planning):** this issue's MECHANISM sections are partially superseded by the minimal-firmware gut. The on-chip measurement rings, ring-dump commands, capture builds, and clock-sync-stamped ring analysis are replaced by the simpler dataset path: the tightened telemetry frame (timestamped per-source readings) emitted every loop iteration, logged host-side (`telemetry-frame-tightening-amendment-to-gut-s1.md`). What STANDS from this issue: the predict-to-now estimator core (`whereAmI()` / `stateAt(t)`, wheel + body peer estimates, ZOH v1 then fit-based), the leave-one-out one-step-ahead RMS validation methodology (now run over the host TLM log instead of dumped rings), the fake OTOS, the external/camera pose source + time sync (clock_sync revival still applies — `PING t=` lands in the gut), and the remaining-distance trajectory controller as the end goal. Detail planning re-derives from the post-gut minimal base.

## Description

Build an odometry subsystem that accurately predicts where the robot is **right now**, then rebuild motion termination on top of it. Timestamped measurements (encoder, OTOS, external/camera) publish into on-chip rings; a `whereAmI()` query extrapolates from the newest measurements to the current instant (v1: last velocity × age). The subsystem is validated standalone — capture long rings while running varied motor patterns, stop, dump the rings over debug commands, and analyze prediction-vs-actual residuals host-side — before a new controller consumes it: each tick, remaining distance to the segment goal is computed from the estimate (at a higher rate than measurements arrive) and velocity is shaped to arrive with the commanded terminal velocity.

Stakeholder decisions (Eric, 2026-07-21), binding on planning:

- **Full arc planned up front, estimator first**; the controller sprint is detailed only after the estimator is bench-proven.
- **Wheel states and body state are peer first-class estimates**, each with its own residual stream (each wheel's traveled distance + velocity; body pose + twist).
- **Fake OTOS is in scope as a given**: build-selectable test device synthesizing an OTOS pose from encoder kinematics, giving the stand an OTOS-present regime (wheels spin free on the stand, so the real OTOS sees zero translation).
- **Concrete rings, one ring per source, in a single measurements container**: a frequent source must never push an infrequent source's records out (the external fix is the most accurate and the most infrequent), and reconstruction reads the newest item of each ring directly — no searching. Records carry no source field (the ring is the source). Per-ring sizes are compile-time constants; same total memory as one shared ring. Debug commands dump each ring.
- **External pose source + time sync**: host-timestamped external poses (camera) enter over the wire stamped in the robot clock domain; revive the ping-pong clock sync.
- **Ground truth (corrected 3×):** the robot's OTOS is rigidly mounted on the robot's I2C bus (0x17). Never write "OTOS on a servo/servo port." `on-chip-fake-otos-test-device.md` carries that wrong premise in its Context — correct its text when that issue is picked up (the want itself stands).

## Cause

Weeks of motion-control tuning have not produced a completing tour. The two standing blockers — turn non-termination and the terminal straight-leg wedge — are both terminal-behavior failures of `Motion::Executor`'s profile-elapsed/dwell completion machinery (`src/firm/motion/executor.cpp:885-908`). Root gaps in the current architecture:

- The firmware deliberately does not fuse: encoder odometry and raw OTOS pose ride to the host separately (`src/firm/app/odometry.h`). No queryable "state at time t" exists; control consumes per-cycle deltas.
- `measurement_ring.h` exists but **nothing publishes into it** (verified: zero call sites) — measurement history is discarded each cycle.
- `Devices::Otos::tick()` burst-reads position **and** velocity and stamps `lastReadUs_` (`otos.cpp:328-348`), but `applyOtosSample()` (`odometry.cpp:51-59`) drops the velocity.
- The only predict-to-now in the tree is heading-only: `HeadingSource::headingLead()` (`heading_source.cpp:88-94`) = `heading + omega × age`. It should be generalized, not duplicated.

## Proposed fix

### Measurement records + the measurements container (`src/firm/devices/`)

Records carry no source field — **the ring a record lives in is its source**:

```cpp
struct PoseRecord {
  uint64_t stamp;     // [us] robot clock (external ring: host-computed robot-domain stamp)
  float v_x, v_y;     // [mm/s] robot-relative twist; v_x forward — on tovez v_y ≡ 0
  float omega;        // [rad/s]
  float x, y;         // [mm]
  float heading;      // [rad]
};                    // stamp + 6 data fields

struct EncoderRecord {
  uint64_t stamp;     // [us]
  float velocity;     // [mm/s]
  float position;     // [mm]
};
```

One container object owns **one ring per source** (this is the object the estimator and the dump commands read):

```cpp
class Measurements {   // src/firm/devices/measurements.h — name negotiable
  // one fixed-capacity ring per source; slot counts are compile-time constants
  external;      // PoseRecord — camera/PoseFix; most accurate, most infrequent
  otos;          // PoseRecord — real or fake OTOS bursts
  encoderPose;   // PoseRecord — per-cycle encoder-odometry pose + twist
  encoderLeft;   // EncoderRecord
  encoderRight;  // EncoderRecord
};
```

- Each ring: plain fixed array + head index, gap-write publish discipline borrowed from `measurement_ring.h` (single-writer fiber, immutable published slots, by-value reads). Whether the ring is a tiny capacity-parameterized helper or two concrete variants is implementer's choice — the container's five named members are the contract.
- **Why per-source rings**: a frequent source never pushes out an infrequent one, and "reconstruct current state" = take the newest record of each ring directly — no searching, no source filtering. This is what enables the fusion recompute below (anchor on the last external, apply everything newer on top).
- Normal slot counts: small per ring (~8; constants). **Capture build** (`ROBOT_RING_CAPTURE` compile option): the high-rate rings (encoderLeft/Right, encoderPose, otos) grow as long as memory allows (linker map shows ~100 KB heap slack; statics shrink it 1:1; leave a runtime-verified safety margin) — thousands of records, tens of seconds; external stays modest. Capture rings are dump-after-stop only, never streamed live.

Publishers (pure memory writes — zero added bus traffic): `NezhaMotor::tick()` publishes an `EncoderRecord` into its wheel's ring per accepted (non-glitch) sample, collect-time stamp; encoder odometry publishes into `encoderPose` per cycle (pose from the integrator, twist via `BodyKinematics::forward`); `Otos::tick()` publishes into `otos` per successful burst (pose + velocity — stop dropping it); the external-pose handler publishes into `external` with the message's own stamp.

### Debug ring-dump commands

New `CommandEnvelope` arm (revive a reserved slot per `envelope.proto:166`): `RingDump{ring}`, ring ∈ {external, otos, encoderPose, encoderLeft, encoderRight} — one selector per `Measurements` ring. Reply is a burst of frames, one record per packet, through the existing host frame-drain path (`protocol.py:1102-1119`), terminated by a count/done frame. Host tool reconstructs each ring → CSV. Bench dumps ride serial (the radio relay drops async frames; relay retrieval would need ack-paced chunking later if ever needed).

### Time sync + external pose injection

- Append `t=<robot ms>` to the pong reply (`comms.cpp:65`) — this activates the existing, complete, unit-tested host estimator `src/host/robot_radio/robot/clock_sync.py` (min-RTT ping-pong offset + skew, with `to_robot_time()` built for stamping camera observations). Host syncs at session start and periodically for skew.
- Revive `PoseFix` (`drivetrain.proto:33-40`, currently pruned off the envelope, reserved field 7) as an envelope arm, **extended with velocity** (`v_x, v_y, omega`). The host stamps `t` via `to_robot_time()` at the observation instant, not arrival — radio delay is exactly why the stamp rides in the message. Firmware handler publishes `PoseRecord{kExternal}`; keep `reset`/`zero_encoders` semantics for hard re-anchor. The firmware consumer must be built new (099-008 never landed).
- Estimator fusion weight for the external source starts at 0 (plumbing + ring first; camera fusion is floor work).

### `App::StateEstimator` (`src/firm/app/state_estimator.{h,cpp}`)

Pure computation over published ring records — never touches the I2C bus. Greenfield alongside the legacy path: `App::Odometry`/`HeadingSource` keep feeding Pilot/TLM unchanged until the controller sprint switches consumers.

- **State**: per-wheel `WheelEstimate{distance, velocity, basisStamp, valid}` (`// [mm] [mm/s] [us]`) and `BodyEstimate{x, y, heading, v_x, v_y, omega, basisStamp, valid}` as peers.
- **API**: `wheelAt(wheel, t)`, `bodyAt(t)`, `whereAmI()` (= `bodyAt(now)`), `wheelNow(wheel)`, `reset(x, y, heading)`, `innovations()`. v1 = ZOH extrapolation: `distance = basis.position + basis.velocity × age`; body heading = fused heading + fused omega × age (the `headingLead()` equation promoted to full state). Age math: one uint64 subtract cast to float seconds; no 64-bit divides per query.
- **Fusion v1**: complementary blend per channel across the source rings (`w_h`, `w_omega` for OTOS; `w_xy` and `w_ext` exist but default 0), staleness threshold collapses a source's weight to 0. All weights are fail-closed boot keys (`data/robots/*.json` → `gen_boot_config.py`) + live-tunable via `handleConfig()` patch. Not an EKF.
- **Fusion recompute pattern the per-source rings enable (the intended end state)**: anchor on the newest `external` record (most accurate, most infrequent), then apply every newer encoder/OTOS update on top of it — integrate forward from the anchor. When a new external record arrives, use the current velocity estimate to reconcile the higher-rate sources against it (e.g. re-baseline the OTOS/encoder pose). v1 ships the simple blend; the anchor-and-replay recompute is the follow-on the container design is shaped for.
- **Cycle placement**: kPace block, after `applyOtosSample()`/`odom_.integrate()`, before `pilot_.plan()`.
- Hazard: OTOS VELOCITY_XL decode reuses position LSB constants (documented wrong scale, `otos.h:280-286`) — linear-velocity fusion weight stays 0 until a characterization ticket lands.

### Fake OTOS (`src/firm/devices/fake_otos.{h,cpp}`)

Extract `Devices::PoseSensor` interface (the union of what the app graph calls on `Otos`); `Otos` implements it; consumers retype to `PoseSensor&`. `FakeOtos` holds `Motor&` L/R + trackWidth, integrates diff-drive forward kinematics over encoder deltas at the real chip's ~20 ms cadence, publishes `PoseRecord{kOtos}`; `present()` always true; zero bus traffic. Build seam: `ROBOT_FAKE_OTOS` CMake option at the `main.cpp` composition root (`main.cpp:91, 122-128`); production hex byte-identical with it off.

### Trajectory controller (final sprint — sketch, detailed only after the estimator gate)

Replaces the Executor completion machinery. Per wheel per segment: `remaining = goal − wheelNow(wheel).distance`; `v_cmd = sign(remaining) · sqrt(v_terminal² + 2·a_dec·|remaining|)` capped at v_max — arrives *with* the commanded terminal velocity, which addresses both the terminal-droop wedge and the never-ramps-down pivot. Completion decided from estimates at control rate; stale basis (`valid`/`basisStamp`) fails safe to the timeout backstop.

### Sprint arc (next number 115; every sprint ends deployed and seen working on the stand)

| Sprint | Content | Gate |
|---|---|---|
| **115** | `Measurements` container (five per-source rings) + publish wiring (motor L/R, encoder-odom, OTOS incl. velocity); ring-dump command arm + host reconstruction → CSV; pong `t=`; capture-build size option. Surface the cycle-order-B decision (see Related) to the stakeholder. | Sim first: dump rings from a sim run → CSV; then stand: spin wheels, dump, plausible timestamped records; clock sync converges over serial |
| **116** | `App::StateEstimator` v1 (encoder-only ZOH) + fail-closed config keys + capture script + leave-one-out RMS notebook (+ `libfirmware_host` cross-check). Rebaseline-discontinuity absorption. | Sim first, then stand: capture → dump → one-step-ahead walk: RMS ≈ measurement noise at constant velocity; ZOH lag on ramps matches theory; position-error integration yields a leg-level projection |
| **117** | `PoseSensor` extraction + `FakeOtos` + `ROBOT_FAKE_OTOS` seam. First action: fix the servo-premise text in `on-chip-fake-otos-test-device.md`. | Fake-OTOS hex on stand: otos-present, pose ring carries `kOtos` records, estimator fuses heading/omega |
| **118** | `PoseFix` revival + velocity extension + firmware consumer (`kExternal` publish); full loop-de-loop suite + notebook; OTOS VELOCITY_XL scale characterization; sim `OtosPlant` v_x/v_y (currently hard-zeroed, `sim_plant.cpp:221-226`). | Host-injected external poses land in the ring with correct robot-domain stamps; stakeholder ratifies residual thresholds from real bench CSVs — precondition for 119 |
| **119** | Trajectory controller (remaining-distance velocity shaping from estimates); targets closing both blocker issues; `Odometry`/`HeadingSource` consolidation decision. | Tour completes on the bench; no timeout-fault turns, no terminal wedge |

### Risks / open items

- **Fake-OTOS circularity**: derives from the same encoders — validates plumbing/latency/fusion math, not accuracy. Independence arrives on the floor: real OTOS translation + camera (external source) ground truth.
- **Capture-build RAM**: fits the linker map, but runtime heap high-water is unverified — first capture-build boot needs a heap check; fallback is a shorter window or int16-packed capture records.
- **Encoder stamp bias** (~4 ms latch-to-collect): accepted in v1; a `stampBias` knob is reserved if residuals show a constant offset.
- **v_x/v_y convention**: project convention is v_x forward (so tovez's zero component is v_y). Flip only on stakeholder say-so.
- **Flash budget** is the real constraint (RAM ~98% is by design — never flag it): measure the map at each sprint gate.

## Verification

Per sprint: `uv run python -m pytest` + sim suite; `just build-clean`; `mbdeploy deploy` (hex by full UID); hardware bench gate per `.claude/rules/hardware-bench-testing.md`.

The arc's core proof is the stakeholder-specified methodology, run **in sim first, then on the bench** over the same command path:

1. Build with capture-length rings (as long as memory allows).
2. `src/tests/bench/estimator_bench_run.py` sends motion commands at different speeds for different times — both directions, turns, straights — filling the rings, then stops and dumps both rings via the debug commands.
3. **Leave-one-out, one-step-ahead walk** (`src/tests/notebooks/estimator_validation.ipynb`, plain Python over the dumped rings): for every measurement k, exclude it, take measurement k−1 as basis, extrapolate to k's timestamp, compare against the actual measurement k; walk the entire ring (2→1, 3→2, …), per stream.
4. **RMS analysis** of the one-step-ahead errors per stream, broken out by pattern phase (steady, ramp, reversal, pivot), then propagate the per-step error through position integration to project accumulated position/heading error over a leg.

Accept thresholds are ratified by the stakeholder from the real RMS tables, not pre-committed. The ZOH lag signature (`a·k` velocity, `½a·k²` distance during ramps) is the first thing the notebook confirms — it decides whether a fit-based predictor is warranted. Secondary cross-check: replay the same rings through the firmware estimator compiled into `libfirmware_host.dylib` and confirm it matches the notebook to float noise.

End state: the tour completes on the bench with estimate-driven termination.

## Related

- `bench-turns-spin-forever-non-termination.md` — blocker; closed by sprint 119.
- `nocal-straight-terminal-wedge-needs-velocity-integrator.md` — blocker; closed by sprint 119.
- `on-chip-fake-otos-test-device.md` — absorbed into sprint 117; its Context's "OTOS on a servo port" premise is wrong and must be corrected on pickup.
- `cycle-order-ab-verdict-e7fb9be2-is-worst-recommend-b.md` — decision surfaced at sprint 115 planning; stakeholder call, not silently adopted.
