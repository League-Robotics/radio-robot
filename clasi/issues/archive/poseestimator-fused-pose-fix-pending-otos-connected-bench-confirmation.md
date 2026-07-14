---
status: obsolete
---

> **OBSOLETE (2026-07-14 stakeholder triage).** Superseded by the single-loop
> firmware rebuild (`clasi/issues/single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md`;
> review: `docs/code_review/2026-07-13-devices-drive-review.md`). Subsystems::PoseEstimator and EkfTiny are deleted; the robot stops fusing (host fuses OTOS/camera against encoder odometry).

# PoseEstimator frozen fused-pose: root-cause hypothesis formed, fix pending one bench confirmation

> **Parked to `later/` 2026-07-09 (stakeholder triage).** Sprint 093 unticked
> `Subsystems::PoseEstimator` (only Hardware + Drivetrain tick in the gutted
> `Rt::MainLoop`), and the fused `pose=` / `otosconn=` TLM fields are off the
> wire, so the confirming bench test below cannot currently run. The
> hypothesis and next-step procedure remain the valuable part — re-run them
> as part of restoring the pose stack
> ([[restore-goto-pursuit-with-pose-estimator]]), before any EKF gating fix
> is designed.

## Status: needs a bench session with a free serial port before the EKF-side fix is designed

Sprint 092 ticket 002 investigated `clasi/issues/poseestimator-fused-pose-
frozen-on-hardware.md` (`PoseEstimator`'s fused pose, `TLM pose=`, never
accumulating on real hardware — 089-007's bench evidence). The ticket's
own PRIMARY diagnostic (does `encpose=` freeze too, or only `pose=`?) is
answered from the EXISTING 089-007 raw trace: `encpose=` (encoder-only
dead reckoning) does **not** freeze — it swings through the full ±180°
wrap range consistent with genuine, fast, continuous rotation; only
`pose=` (the EKF-fused reading) stays pinned within ~1° / a few mm of the
origin the entire session (both in the isolated TURN diagnostic and in
G's 1.3+ m run). This resolves architecture-update.md (092) Decision 5's
diagnostic in favor of **Candidate B** (an EKF/OTOS-specific mechanism),
ruling out Candidate A (a shared step-1/encoder-accumulator bookkeeping
bug, which would freeze both readings identically).

## What ticket 092-002 additionally found (code-level, confirmed by direct read)

1. **Decision 5's own literal "degenerate `dt`" sub-hypothesis is
   refuted.** `EkfTiny::predict()` (`source/estimation/ekf_tiny.cpp`)
   computes the state MEAN update (`fx[0..2]`) entirely from `dCenter`/
   `dTheta`/the EKF's own previous state — `dt` only scales the
   covariance process-noise term (`Q`). `ekf_predict()` (vendored
   TinyEKF, `libraries/tinyekf/tinyekf.h`) unconditionally does
   `memcpy(ekf->x, fx, ...)`. A zero/degenerate `dt` cannot, by itself,
   freeze the mean state — contrary to Grounding's assumption that it
   "changes nothing."
2. **No numerically-stuck-covariance path was available either**: Tovez's
   firmware has no `OI`/`updatePosition()`/`updateHeading()` traffic
   unless `otosFusableThisPass()` is true and `otosObs->stamp.valid`, so
   a "bad correction poisoned the state" story requires a correction to
   have happened at all.
3. **`Subsystems::NezhaHardware` unconditionally constructs a REAL
   `Hal::OtosOdometer`** (`source/main.cpp:126-130`, `nezha_hardware.cpp`),
   regardless of whether a physical OTOS chip is actually present/
   functioning — this is a different fact than 089-007's own belief
   ("Tovez has no real OTOS chip either way"), which was inferred from
   `DBG OTOS` replying `ERR unknown`. That inference does not hold: `DBG
   OTOS`/`DBG OTOS BENCH` are `source_old/` (pre-rewrite) debug verbs,
   never ported into `source/commands/otos_commands.cpp` (which only
   registers `OI/OZ/OR/OP/OV/OL/OA`) — `ERR unknown` reflects an
   unported debug command, NOT an absent `Hal::OtosOdometer`. Similarly,
   `ID`'s `caps=` field does not exist at all in this tree's
   `formatDeviceAnnouncement()` (`system_commands.cpp:137-139` explicitly
   dropped it) — the 089-007 bench log's "caps= was empty" observation
   cannot have come from the CURRENT firmware's actual `ID` reply either.
   Both planks of "no OTOS chip" were drawn from stale/legacy-protocol
   assumptions, not verified against this tree's actual behavior.
4. **`EkfTiny` has no rejection/gating mechanism** — 082's own
   architecture doc (Decision 2) deliberately dropped "Mahalanobis
   chi-squared gating on any channel" when trimming the 5-state parent
   class down to this 3-state filter. `updatePosition()`/`updateHeading()`
   unconditionally trust whatever `otosObs` they are given.
5. **The old tree had a purpose-built escape hatch for exactly this
   testing scenario** — `DBG OTOS BENCH` (`docs/protocol-v2.md` §14,
   `BenchOtosSensor`, sprint 031/034) existed specifically because "the
   robot is on a stand and the floor sensor sees no motion." This was
   never re-ported into `Subsystems::PoseEstimator`/`Hal::OtosOdometer`'s
   ordered-tick rewrite (`grep -rl BenchOtosSensor` outside docs/history
   finds nothing under `source/`).

## Leading hypothesis (NOT bench-confirmed)

If Tovez's `OtosOdometer` genuinely detects a chip at boot (`begin()`'s
`PRODUCT_ID` read succeeds) — plausible, since nothing in 089-007's own
evidence actually rules this out (see point 3 above) — then, mounted on a
stand with wheels off the ground, the chip's own optical tracking has no
real surface to see and would report a near-static reading with
`stamp.valid = true` roughly every `kReadPeriod` (20 ms). With no
rejection/gating (point 4), `PoseEstimator::tick()`'s step 4
(`updatePosition()`/`updateHeading()`) would unconditionally fuse this
static-but-"valid" reading every cycle, continuously dragging the EKF
belief back toward near-(0,0,0) — fighting against, and roughly
cancelling, the encoder-informed `predict()` step. This explains every
observed symptom: `fusedPose` pinned near the origin the entire session
(both TURN's diagnostic and G's 1.3 m run), `encoderPose` unaffected
(never touches OTOS), `RT` unaffected (uses the raw encoder-arc stop
condition, not `fusedPose`), and sim not reproducing it (`Hal::
SimOdometer` reports a pose consistent with the SIMULATED ground-truth
motion — fusing an accurate reading is correct, not a bug — and sim has
no notion of "robot lifted off the ground with a still-responding OTOS
chip" at all).

## Why this ticket did not ship a fix

This hypothesis is well-reasoned but genuinely unconfirmed: I could not
reach the bench this session to check it. `mbdeploy deploy` succeeded
(fresh build flashed), but every subsequent attempt to open
`/dev/cu.usbmodem2121102` (raw `pyserial`, `robot_radio.io.serial_conn`,
with and without sandbox restrictions) failed with `Resource busy`.
`lsof /dev/tty.usbmodem2121102` shows the paired tty device held open by
a local VS Code extension-host process (`Code Helper (Plugin)`, PID
61702 at investigation time) — almost certainly a stale serial-monitor
panel from an earlier session, not something this ticket's programmer
should kill unilaterally (a concurrent session could plausibly be using
it — see `.claude/` project memory on parallel-session hazards). Per
ticket 092-002's own instructions, an EKF-gating change is exactly the
kind of speculative fix that should not ship without confirmation: it is
a real design decision (what threshold, applied to which channel(s),
whether/how it interacts with `Hal::Odometer::fusableThisPass()`) that
deserves a confirmed root cause, not a guess.

## What ticket 092-002 landed instead (diagnostic only)

A new TLM wire field, `otosconn=<0|1>` (`source/telemetry/tlm_frame.{h,cpp}`,
`source/runtime/blackboard.h`, `source/runtime/main_loop.cpp`), surfacing
`Hal::Odometer::connected()` live every pass — the ONE fact 089-007
couldn't check because no existing wire verb exposed it. Sim regression
coverage added (`tests/sim/unit/tlm_frame_harness.cpp`); full
`uv run python -m pytest tests/sim` stays green (311 passed, 2 xfailed,
unchanged).

## Concrete next step (bench, once a session has the serial port free)

1. Close whatever local process/panel holds `/dev/tty.usbmodem2121102`
   (check `lsof /dev/tty.usbmodem*` first).
2. Flash the current tree (`just build-clean` + `mbdeploy deploy <UID>
   --hex MICROBIT.hex`).
3. `STREAM 100` (or repeated `SNAP`), spin the wheels (`TURN 9000` or a
   manual `DEV DT VW`), and read `otosconn=`:
   - **`otosconn=1` throughout** → the hypothesis above is confirmed.
     The fix is then a genuine EKF/PoseEstimator design decision: either
     a bounded innovation-consistency gate inside `EkfTiny::
     updatePosition()`/`updateHeading()` (re-introducing a narrow form of
     082 Decision 2's dropped gating, now with a fresh, evidenced
     reason), or a re-ported stand/bench-mode toggle analogous to the
     old tree's `DBG OTOS BENCH` (redirect fusion off entirely on
     command). Needs its own ticket/architecture pass — do not
     freehand a threshold without characterizing it in a sim test first
     (mirrors this sprint's own D/T seed-correction discipline).
   - **`otosconn=0` throughout** → the hypothesis is refuted; `otos=`'s
     own frozen-at-boot-default values were never actually fused (this
     rules out the OTOS-fusion story entirely), and the frozen-`pose=`
     defect needs a fresh investigation pass focused elsewhere (re-open
     `poseestimator-fused-pose-frozen-on-hardware.md`'s own candidate
     list with this new negative result folded in).

## References

- `clasi/sprints/092-motion-otos-hardware-fixes-bounded-stop-decel-seed-correction-hardware-pose-fusion-investigation-and-otos-sparkfun-library-port/tickets/002-poseestimator-hardware-fused-pose-investigation-and-fix.md`
  (completion notes have the full investigation trace)
- `clasi/sprints/092-.../architecture-update.md` Grounding + Decision 5
- `clasi/issues/poseestimator-fused-pose-frozen-on-hardware.md` (original
  finding)
- `docs/protocol-v2.md` §8 (new `otosconn=` field), §14 (the retired
  `DBG OTOS`/`DBG OTOS BENCH` verbs, for historical/porting reference)
