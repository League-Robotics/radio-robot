# Code Review: DeviceBus, I2C Loop, Drive Drivetrain + Dead-Code Audit

**Date:** 2026-07-13
**Branch:** sprint/101-bench-test-rig-debugging-restore-heading-feedback-otos-pose-and-turn-control
**Scope (stakeholder-requested):**
1. Dead code in `source/motion/` and `source/subsystems/`
2. Dead code in `source/devices/` (new) and `source/hal/`
3. Design review: `source/devices/` + the I2C loop
4. Design review: the drivetrain in `source/drive/`

**Method:** four parallel review agents, each primed with the build reality
(default image = `source/main.cpp` via codal.json; bring-up image =
`source/devices/bringup_main.cpp` via codal.devicebus.json; CMake exclusion
list; host sim/test builds) and the project's hard-won failure-mode knowledge
(wedge/latch reversal trains, single-pending-0x46, per-pass I2C ticks wrecking
motion timing, the fabsf backward-completion bug, sprint-098 heading loop,
~120–140 ms actuation latency).

---

# Part 0 — Overnight bench corroboration (2026-07-14 rig report, commit 346c7296)

Stakeholder ran production firmware (v0.20260713.1) on the test rig overnight:
sim suite green (697 passed), bench turns land on target with encoder heading
(90° → 85.31°, 180° → 179.86°, 360° → 357.96°), after fixing two blockers.
Each observed issue maps onto this review as follows:

**B1 — "Segments admitted/ACKed but never executed (motors idle,
active=False)" when the decoupled rig OTOS poisoned the fused pose.**
This is the observable signature of **Part 2 findings #7 and #9**: `admit()`
ACKs synchronously against `bb.chainTail`, then `startNextPlan()` solves from
the *measured* (here garbage) pose; a late `plan()` failure silently
`continue`s with only `lateSolveFailures_++` — no EventNotify, no wire error.
The stakeholder had to bisect by hand (direct MOVER command, then EKF
distrust) to find what a single wire-visible "late solve failed" event would
have said immediately. Bench cost of the silent-drop path is now demonstrated;
raise the priority of #7 (and #9 — garbage pose fields should be caught at
the boundary, or at minimum surfaced when the solve rejects them). Verify by
checking whether `lateSolveFailures_` incremented during the failure window.

**B2 — OTOS distrust (`SET ekfROtosTheta=1e9 ekfROtosXy=1e9`) required every
session on this rig.** The rig's OTOS is servo-mounted and mechanically
decoupled from the wheels, so its pose is *structurally* invalid there — this
is a per-robot property, not a runtime tuning decision. Recommendation (new,
adjacent to Part 1 m1's absence-handling): give the rig profile a persistent
"OTOS untrusted / encoder-only pose" switch (robot JSON / boot config) so the
fused-pose poisoning cannot recur by forgetting a SET after reboot. Note the
adjacent hazard from **Part 1 M6**: even on robots where OTOS *is* trusted,
the setPose re-anchor window feeds stale-frame poses to the EKF.

**B3 — Sim 180° and 360° pivots both land at ~272–273° while the bench lands
on target.** Two different targets converging on the *same* absolute heading
(272.x° vs 273.x°) does not smell like tuning — it smells like an angle-wrap
attractor somewhere in the sim-side pivot path (eTheta normalization /
projection in `arc_math`/`motion_plan`, or the sim plant's heading
convention), possibly interacting with **Part 2 #2** (replan clock
rebaseline corrupting the rate limit → sustained saturated tracking) and
**#11** (pivot-mode inference from instantaneous ref.v). Also check what
status the sim runs reported: if they reported DONE_STOP despite ~90° of
terminal error, that is **#10** (unbounded overshoot completes as success)
confirmed in sim. Investigation queued (stakeholder priority 2).

**B4 — 90° lands 4.7° short with 12 velocity humps (ringing); v2 limits are
untuned starting values.** The known ~140 mm/s velocity-loop resonance plus
untuned v2 rotational limits is the plausible mechanism, and **Part 2 #16**
(governRatio as a second wheel authority downstream of the tracker) is a
candidate contributor to both the ringing and the short landing. Critically,
**Part 2 #3/#4 block the planned fix**: the gains that need bench-tuning
(`track_k_theta`, `track_k_s`, `trim_*`, `v_wheel_max`, `wheel_step_max`, and
every policy constant) are boot-config or `constexpr` only — each tuning
iteration is a reflash. Fixing #3 (extend `PlannerConfigPatch` to the fields
v2 actually consumes) should come BEFORE the bench-tuning campaign, or that
campaign will be done at reflash cadence.

**B5 — `segment_for_move` (retired non-primitive) still emitted by host
tooling; firmware correctly rejects with ERR_UNIMPLEMENTED.** The notebook is
fixed (346c7296), `turn_sweep.py` is known-stale (stakeholder priority 1).
Action for the dead-code audits: grep the host tree for remaining
`segment_for_move` callers and other retired wire shapes so the cutover
completes host-side too.

**Positive corroboration:** bench turns landing within −0.14°…−4.69° on
encoder heading confirm the sprint-098 heading approach survives in the v2
pivot path on real hardware (Part 2 design summary), and production build +
binary telemetry + sim suite are healthy on the cutover firmware.

---

# Part 1 — Design Review: `source/devices/` + I2C loop

## Design as built (mental-model check)

- One CODAL fiber (spawned by `DeviceBus::start()` via `CodalFiberRunner::run`
  → `create_fiber`, device_bus.cpp:384–395) owns the I2C bus and all device
  leaves. Body: `runPreamble()` once, then `while(!stopRequested_) runCycleOnce()`.
- **Preamble:** 50 ms power settle → `motor1_.begin()`/`motor2_.begin()`
  (hardReset, median-of-3 atomic reads) → OTOS product-ID probe retried
  20×100 ms (~2 s) → color+line non-blocking `beginStep()` state machines paced
  50 ms, capped 64 ticks. Absent devices latch `present()==false` and are
  skipped forever.
- **Cycle** (device_bus.cpp:67–79): `drainStagedInputs()` (velocity-stale
  watchdog, no bus) → `serviceMotor(motor1)` = 0x46 request → `fiber_sleep(4)`
  → collect + PID + armored 0x60 write → `serviceMotor(motor2)` same →
  `perceptionSlotStep()` round-robins ONE of line|color|OTOS (each leaf
  internally rate-limited: line 50 ms, color 100 ms, OTOS 20 ms) →
  `publishSamples()` → `fiber_sleep(12)` pace. Nominal ~20 ms; realistically
  ~25–30 ms with bus time + clearance waits.
- Alternating per-motor request→settle→collect honors the brick's
  single-pending-0x46 constraint; nothing touches the bus inside a motor's
  request→collect window (the 093 hazard is structurally absent). OTOS is
  folded into the slot schedule, not per-pass — conforms to the flip-flop rule.
- **Data out:** single-writer `MeasurementRing<T>` (6 slots, 5 published,
  gap-write, head advance = single uint8 store) read by-value through handle
  classes that never touch the bus. **Data in:** handle setters are plain
  stores onto leaf staged fields (depth-1, last-write-wins); `Odometer::setPose`
  stages into Otos's `posePending_`, drained at the OTOS slot.
- `I2CBus` is a diagnostic wrapper: per-device txn/err counters, re-entrancy
  detector, txn ring log, per-device lazy preClear/postClear clearance timers
  (busy-spin at entry), full-transaction IRQ masking default-ON (TWIM errata
  armor — preserved per project rule).
- Armor policy (reversal dwell, output deadband, standstill-guarded reset,
  wedge latch) centralized in `MotorArmor` base; PID and write shaping (slew,
  write-on-change, 40 ms write throttle) in the `NezhaMotor` leaf.
- **Legacy seam:** `DeviceBusHardware` owns one DeviceBus; `DeviceBusMotor` /
  `DeviceBusOdometer` are passthrough Hal leaves with no-op `tick()` (except
  `processResetIfPending`), avoiding the double-PID/double-armor trap.
  `main.cpp` builds `DeviceBusHardware` (cutover live).

## Findings

### CRITICAL

- **C1 — failed duty write latched as written; write-on-change suppresses all
  retries.** nezha_motor.cpp:325–372 (`writeRawDuty`) + :374–391
  (`writeMotorRun`) + motor_armor.h:162–168. `bus_.write()` status is ignored;
  `lastWrittenPct_`/`lastWriteTimeUs_` update unconditionally, and
  write-on-change (nezha_motor.cpp:333) then suppresses every retry of the same
  value. Failure scenario: a transient NAK on a **stop** write (pct==0) is
  permanently lost — the watchdog gate's documented "re-asserts Neutral every
  cycle" robustness (device_bus.cpp:100–108) is defeated because
  `armoredWrite(0)` → `pct == lastWrittenPct_ == 0` → early return, no bus
  write ever again. `appliedDuty()` reads 0.0 while the wheel physically
  drives, which also blinds `wedgeSuspect()`. Wheels-keep-spinning with no
  signal and no recovery until a nonzero duty is commanded.
  **Fix direction:** commit `lastWrittenPct_`/`lastWriteTimeUs_` only on
  `status == kOk`; treat a failed pct==0 write as must-retry-next-tick.

### MAJOR

- **M1 — clearance busy-spins block the CODAL scheduler ~4 ms+ per cycle.**
  i2c_bus.cpp:67–68, 112–113; hot sites nezha_motor.cpp:434–447 (preClear
  4000 µs), :390 (postClear 4000 µs), otos.cpp:337–340 (write-then-read pair =
  back-to-back 4 ms spins), nezha_motor.cpp:397–424 (hardReset ≈ 8 txns ≈
  ~32 ms spin, ×2 motors at preamble). The clearance wait is a hard
  `while(clockUs()<deadline){}` with no yield, executed in the device fiber —
  motor1's duty write stamps readyAt=+4 ms and motor2's request follows
  immediately, so ~4 ms of scheduler-blocking spin nearly every cycle. On the
  cooperative scheduler this stalls all other fibers (serial pump, radio in
  the legacy image) — against the "loop must yield every pass" rule.
  **Fix direction:** waits >~1 ms in fiber context go through
  `Sleeper`/`fiber_sleep`, or reorder the cycle so the pace sleep sits between
  motor1's write and motor2's request.

- **M2 — velocity-stale watchdog fights live DUTY commands.** handles.h:132–147
  + device_bus.cpp:109–114 (`applyStaleGate`). `Motor::setDuty()` neither
  cancels nor re-arms `velocityStaged_`, so once any past `setVelocity()` is
  >300 ms old, the stale gate force-stages `Neutral::Coast` every cycle while
  the consumer's `setDuty()` stages `Mode::Active` — last store wins →
  intermittent coast/drive flapping; a one-shot DUTY command is overridden
  ~permanently. Repro: `M 1 VEL 100`, wait 1 s, `M 1 DUTY 0.3`.
  **Fix:** `setDuty()` (and arguably `setPidEnabled(false)`) must cancel or
  re-stamp the staleness bookkeeping.

- **M3 — DUTY mode has no deadman at all.** handles.h:138. Only velocity
  targets are watchdog-guarded; a host that crashes after `setDuty(0.5)`
  leaves the wheel driven forever. The invariant "the main loop can crash and
  the wheels still stop" holds only for velocity mode.
  **Fix:** timestamp every actuation-staging setter and stale-gate
  `Mode::Active` generally.

- **M4 — APDS color-sensor probe is success-on-failure.** color_sensor.cpp:57–70
  (`beginStep` ApdsProbe) + :195–199 (`readReg8` ignores status). A NAK'd
  readback leaves `out=0`, and `en == 0x00` is exactly the "detected"
  condition — a robot with NO color sensor latches `present()==true`, runs
  `initApds()` against nothing, and issues failing transactions at every due
  perception slot forever. **Fix:** probe via `readReg8Status()` and require
  transaction OK.

- **M5 — motor port map aliases indices 2/3 onto motor 1.** device_bus.cpp:50–53
  (`DeviceBus::motor`) + device_bus_hardware.cpp:293–294/306–307 +
  hal_command.h:45. Any port ≠2 resolves to channel 1, so a command addressed
  to motor index 2 or 3 physically actuates motor 1, and `motorState(2/3)`
  reports motor 1's data under ports-3/4 labels (legacy NezhaHardware had real
  inert leaves). **Fix:** inert null handle for 2/3, or make
  `DeviceBus::motor()` fallible/strict.

- **M6 — setPose re-anchor window feeds stale-frame OTOS poses to the EKF.**
  device_bus_hardware.cpp:272–282 (`fusableThisPass` override) vs
  hal/capability/odometer.h:98–118. The override discards the base's
  `resetAppliedThisPass_` suppression while the fiber applies `setPose`
  asynchronously (drained at the OTOS slot ≤~60–90 ms later; confirming read
  one slot later still). Between an SI/setPose re-anchor and the post-anchor
  read, the ring keeps publishing old-frame poses with advancing stamps →
  main_loop.cpp:130–141 feeds 1–2 stale-frame observations into PoseEstimator
  against the freshly re-anchored state — transient EKF corruption on every
  re-anchor. **Fix:** suppress fusable until a ring publish that postdates the
  fiber's drain (require two stamp advances, or surface an anchor-applied flag
  from `Otos`).

- **M7 — wedge telemetry is structurally blind in the cutover image.**
  device_bus_hardware.h:45–62 (documented limitation #1).
  `msg::MotorState.wedged/wedge_suspect` are always false (non-virtual
  Hal::Motor accessors read base armor state that never updates), even though
  the Devices leaf computes correct wedge signals. Given the wedge/latch bench
  history, the cutover image is blind on its most-litigated failure family.
  **Fix:** virtualize the accessors — schedule it, don't leave it a comment.

### MINOR

- **m1** — No post-preamble re-detection and no bus-wedge recovery: a device
  absent at boot is absent until reboot (device_bus.cpp:276–319,
  otos.cpp:30–47). Suggest a slow background re-probe slot.
- **m2** — `start()` not idempotent (device_bus.cpp:218–226): a second call
  spawns a second fiber = two bus masters. Cheap guard.
- **m3** — Motor ring samples stamped with one cycle-shared `nowUs` taken after
  motor2's settle (device_bus.cpp:74,187–193): motor1's reading is ~8–12 ms
  older than its stamp. The leaf already holds the true collect instant —
  stamp each motor with it.
- **m4** — measurement_ring.h:31–40 immutability doc overclaims: the oldest
  published slot becomes the write gap after ONE more publish, not five —
  false for age>0 reads under a preemptive writer; comment invites unsafe
  reuse (e.g. ISR readers).
- **m5** — otos.cpp:94–101: setPose drain applies before clearing
  `posePending_`; a request arriving mid-apply is swallowed. Clear-then-apply
  is the robust idiom.
- **m6** — device_bus.h:29–35,301–308: "~16 ms cycle / encoders ~60 Hz /
  perception ~20 Hz" arithmetic is optimistic — real cycle ≈ 25–30 ms →
  motors ~35–40 Hz, perception ~10–13 Hz each; OTOS `kReadPeriod=20000` never
  binds. Restate derived claims against measured cycle time.
- **m7** — Preamble worst case ≈5.3 s with no motor service and no watchdog;
  commands staged during that window execute abruptly at loop start (stale
  gate covers velocity but not duty, per M3).
- **m8** — Motor rings republish every cycle under fresh stamps even when the
  brick's ~80 ms register hasn't refreshed — the "fabricated reading" pattern
  publishSamples' own comment forbids for perception. Add a freshness flag if
  fusion ever weights encoder samples by stamp.

### STYLE (naming rules — units in identifiers; systematic)

- clock.h:43 `nowMicros()`, :47–48 `setMicros()/advanceMicros()`, :67
  `sleepMillis()`; i2c_bus.h:281 `clockUs()`; i2c_bus_host.cpp:42
  `g_fakeClockUs`.
- Unit-suffixed constants: device_bus.h `kPowerSettleMs`,
  `kPreambleRetryPacingMs`, `kOtosBeginRetryPacingMs`, `kEncoderSettleMs`,
  `kCyclePaceMs`, `kVelocityStaleUs`; nezha_motor.cpp `kMinWriteIntervalUs`,
  `kDelayUs`; otos.h:305–306 `kPosMmPerLsb`/`kHdgRadPerLsb`.
- Pervasive `nowUs` parameters (nezha_motor.h:132, otos.h:164/189,
  color_sensor.h, line_sensor.h, device_bus.h:323–326); members
  `lastTickUs_/lastFreshUs_/lastWriteTimeUs_/velocityStagedUs_/lastReadUs_/
  lastAttemptUs_`; otos.h:380 `writePoseMm()`.
- Everything else conforms: case rules, trailing underscores, snake_case
  filenames, `// [unit]` tags used extensively and correctly.

### Conformance positives (hard-won-knowledge checklist)

- Single-pending-0x46 respected (alternating request→settle→collect;
  pipelined form removed after DB-009 HITL).
- OTOS never free-runs per pass — folded into the 3-way round-robin slot.
- 093 REQUEST→COLLECT hazard structurally absent.
- Wedge/reversal armor ported intact and centralized in `MotorArmor`;
  gap-then-write preserved; no target-gating/arming-grace reintroduced.
- IRQ guard default-ON preserved (i2c_bus.cpp:28); fiber yields via
  `fiber_sleep` at settle and pace points (except the M1 spins).
- Double-PID/double-armor trap at the adapter seam correctly avoided.
- newlib-nano %f/%ll gaps correctly worked around in bringup_main.cpp.

---

# Part 2 — Design Review: `source/drive/` drivetrain

## Design as built (mental-model check)

Command path: binary `segment` (arcLength/deltaHeading/exitSpeed) →
`binary_channel.cpp::handleSegment` runs a synchronous throwaway
`Drive::Drivetrain::admit()` against `bb.chainTail` (advancing it on OK) →
Goal posted to `bb.segmentIn` → `Subsystems::Drivetrain::tick()` drains into an
8-slot ring. When idle, `startNextPlan()` pops a Goal and runs the exact Ruckig
solve (`Drive::Drivetrain::plan()`): ONE jerk-limited 1-DoF master profile
(path length for arcs, heading for pivots; omega = kappa·v derived), goal pose
composed and frozen at plan time. `Drive::MotionPlan` is immutable; all mutable
state is a 5-scalar caller-owned `StepState` plus the adapter's `planStart_`
anchor. Each tick the adapter builds `StepInput` (t = now−planStart_, measured
pose+twist from `bb.bodyState`) and calls `plan_.step()`: closed-form reference
sample → exact SE(2) arc projection (`arc_math`) → P-only Kanayama trims
(clamped in arc mode; pivot mode = sprint-098 unclamped heading P, kTheta 6.0)
→ differential IK → curvature-preserving saturation → one-sided forward-arc
wheel clamp (`tracker.cpp`) → policy (`policy.cpp`): replan envelopes
(saturated AND out-of-envelope, 200 ms sustain, 300 ms rate limit, N-max 3),
terminal settle machine (banded one-sided walk-in 50–100 mm/s, 15 mm/15 mm/s
gate, 150 ms dwell, T+1.5 s timeout), flying-handoff envelope, pose-fix
absorb/bypass. Status reactions live in the adapter (REPLAN_DUE → `replan()`;
DONE_* → pop next ring entry seeded from reference exitSpeed; ABORT_* → flush +
re-anchor chainTail + EventNotify). MOVER teleop = `replace` arm →
`planVelocity()` (two velocity-mode profiles, deadman = duration,
latest-wins). Timing: purely elapsed-time reference sampling with hold-at-end;
NO dead-time/actuation-lag compensation by explicit design (envelopes sized as
lag allowance); wheel setpoints go to unchanged Level-2 motor PIDs.

## Findings

### CRITICAL

1. **Velocity-mode (MOVER) plans are fought by their own pose tracker.**
   motion_plan.cpp:105–121 + :54–73 + policy.cpp:105–113. `referenceAt()`
   holds x/y/theta at the anchor for the whole plan; `step()` calls `track()`
   unconditionally. Pure-spin MOVER (v=0 → pivotMode) reaches equilibrium at
   eTheta = ref.omega/trackKTheta ≈ 1.22/6 ≈ 0.2 rad — the robot turns ~12°
   and stalls despite a continuous omega command; forward teleop reaches
   vCmd=0 at eAlong = v/trackKS (50 mm at 100 mm/s, kS=2) if requests aren't
   refreshed, and carries a systematic −kS·v·Δt_refresh drag when they are.
   **Fix direction:** in velocity mode bypass the tracker (feed ref.v/ref.omega
   straight to IK+saturate), or integrate the commanded twist into the
   reference pose.

### MAJOR

2. **Replan clock rebaseline corrupts the rate limit.** policy.cpp:75 +
   subsystems/drivetrain.cpp:349–361. `attemptReplan` stores `lastReplan = t`
   (old plan clock); the adapter resets `planStart_` on swap but preserves
   `state_`, so on the new plan `(t − lastReplan) < 0.3` stays true for
   ~the old plan's entire elapsed time (replan at t=3 s blocks the next for
   ~3.3 s) while the robot tracks with saturated trims.
   **Fix:** rebase `lastReplan` when the plan clock rebases, or use a
   segment-global clock.

3. **Sprint-098 live tunability is broken both ways.**
   binary_channel.cpp:528–554 + drive_bridge.h:65–79. `PlannerConfigPatch`
   only carries `min_speed`/`heading_kp`/`heading_kd`, but v2 consumes NEITHER
   heading key (the live heading gain is `track_k_theta`, boot-config only —
   boot_config.cpp:129–136 — along with `track_k_s`, `track_k_cross`,
   `trim_v_max`, `trim_omega_max`, `v_wheel_max`, `wheel_step_max`).
   Wire-patching heading_kp is a silent no-op; retuning the actual loop
   requires a reflash. **Fix:** extend the patch to fields 15–31; wire
   heading_kp→trackKTheta or remove the dangling keys.

4. **Every policy constant is hardcoded while `msg::PlannerConfig` already
   carries matching, never-read fields.** policy.cpp:19–56 vs
   boot_config.cpp:137–145 (replan envelopes, sustain, rate limit, N-max,
   walk-in band, arrive tol, dwell, handoff budget). Two sources of truth that
   will silently drift; same tunability cost as #3.
   **Fix:** thread through `Drive::Limits`.

5. **Stale `bb.chainTail` after preemption.** subsystems/drivetrain.cpp:114–135,
   258–280. `dispatchEscapeHatch()` and the MOVER `replaceIn` path flush the
   ring without re-anchoring `chainTail` (only `abortAndFlush()` does). The
   next segment is admitted against a phantom tail — wrong accept/reject, and
   admission geometry disagrees with `startNextPlan()`'s actual start.
   **Fix:** re-anchor chainTail on every ring flush / segment-mode exit.

6. **`nextEntrySpeed_` never cleared on preemption/flush/idle.**
   subsystems/drivetrain.cpp:369–378, 137–176. A chain ending in a flying
   segment leaves `nextEntrySpeed_ = exitSpeed`; the NEXT chain minutes later
   is solved with that stale entry speed from a stationary plant — reference
   starts e.g. 200 mm/s ahead, instant trim saturation and replan churn.
   **Fix:** zero it in `dispatchEscapeHatch()`, `abortAndFlush()`, the MOVER
   path, and on segment-mode idle-out.

7. **Silent mid-chain segment drop on late plan failure.**
   subsystems/drivetrain.cpp:150–161. A late `plan()` failure just `continue`s
   to the next ring entry — the remaining chain executes geometrically SHIFTED
   while `bb.chainTail` still predicts the unshifted tail; no EventNotify, no
   wire error (only `lateSolveFailures_++`).
   **Fix:** treat late solve failure as abort-and-flush of the whole chain.

8. **`admit()`'s exit-reachability check is direction-blind.**
   drive/drivetrain.cpp:86–95. `deltaVSq = exit² − entry²` with `fabsf`
   discards both signs — a reversing joint (entry +150 → exit −150 gives
   deltaVSq=0) or an exitSpeed opposing arcLength passes admission, is ACKed,
   then deterministically fails `solveToExit()`'s no-reversal band at pop time
   → finding 7's silent drop. This is the surviving fabsf-style
   direction-blind check in the stack.
   **Fix:** sign-aware reachability in the segment's travel direction.

9. **No NaN/Inf validation anywhere on the wire path.**
   binary_channel.cpp:89–100 + drive/drivetrain.cpp (zero
   `isfinite`/`isnan` in drive/, adapter, bridge). All admit() guards are
   `x > limit`, false on NaN; NaN kappa survives the ceiling fold via
   `std::min` NaN semantics; the only backstop is Ruckig's own `validate()` —
   an undocumented dependency for wire-input sanitation.
   **Fix:** reject non-finite Goal fields at `driveGoal()`/`admitSegment()`
   with a typed ERR.

10. **Unbounded overshoot completes as success.** policy.cpp:236–275. The
    `overshotBand` (issue-relative eAlong < −15 mm) has no outer bound on the
    dwell path — a 100 mm coast past goal reports clean DONE_STOP, while the
    parallel timeout path DOES discriminate at 2× tolerance.
    **Fix:** bound overshot completion at the same 2× tolerance; beyond it,
    ABORT_TIMEOUT at grace expiry.

11. **Pivot mode inferred from instantaneous `|ref.v| < minSpeed` instead of
    the plan's own `isPivot_`.** tracker.cpp:97 + motion_plan.cpp:119. The
    decel tail of every translating stop segment flips the tracker into pivot
    mode — forward command forced to 0 with reference distance remaining,
    plus an UNCLAMPED heading trim; velocity-mode teleop below minSpeed is
    dead. Sub-mm at the 10 mm/s default, but a bench retune to 30–50 mm/s
    makes the land-short window quadratically worse — the historical
    stall→lunge terminal signature.
    **Fix:** pass `isPivot_`/`isVelocityMode_` into `track()`; reserve the
    speed threshold for trim scheduling only.

### MINOR

12. policy.cpp:71–88,96,132,150–154,184 — ABORT_REPLAN_LIMIT can return with a
    live (possibly saturated) `result.command`, violating PolicyResult's
    documented "every ABORT_* forces {0,0}". Masked by the adapter today.
13. tracker.cpp:137–147 — the structural no-reversal wheel clamp exists only
    for forward arcs; backward arcs (ref.v < 0) can flip one wheel's sign
    under trim near saturation — the per-wheel-reversal wedge hazard,
    unguarded in the mirror direction. Mirror the clamp.
14. subsystems/drivetrain.cpp:369–395 — trailing flying segment (nonzero
    exitSpeed, nothing queued) resolves DONE_HANDOFF → instant neutral BRAKE
    from speed, no decel plan, no error/event.
15. types.h:150–158 — heading loop survives as P-only (kTheta 6.0) but
    `heading_kd` remains boot-configured and wire-patchable as a dead key
    (see #3) — misleading tuning surface.
16. subsystems/drivetrain.cpp:409 — `governRatio()` still scales
    Drive-produced wheel setpoints: a second, independent wheel authority
    downstream of the tracker's saturation cascade; slows plant response →
    grows eAlong → interacts with replan envelopes. Documented interim (M13).
17. master_profile.h:74–81 + policy.cpp:19–23 — the ~120–140 ms actuation lag
    is deliberately unmodeled; it lives implicitly in envelope allowances
    (kAlongEnvelopeRate 0.25 s·|v| ≈ 2× lag) — coherent, but those constants
    are the same hardcoded ones from #4: if the lag changes (bus schedule
    changes have done this before), retuning requires a reflash.
18. subsystems/drivetrain.cpp:296–297 — `assert()` is the only guard against a
    1-based→0-based port underflow; no-op under NDEBUG → `motorState(2³²−1)`
    UB.
19. pose_estimator.h:213–224 + main_loop.cpp:75–81 — completion gates consume
    twist derived from wheel-encoder velocity with `has=false → 0.0f`: on a
    dead/stale encoder bus, twist reads 0, the velocity gate is trivially
    satisfied, and a coasting robot can dwell to DONE_STOP. WheelState
    validity flags exist in StepInput but nothing reads them.

### STYLE

20. drive/drivetrain.cpp:40 `kRadiusFloorMm` — unit in identifier.
21. subsystems/drivetrain.h:344 `lastVelSampleMs_` — unit in identifier.
22. subsystems/drivetrain.cpp — 4-space indentation vs the 2-space convention
    used by `source/drive/`; the two halves of the same seam disagree.
23. policy.cpp:235 `issueEAlong` — named after the planning document rather
    than the quantity (goal-relative along error); e.g. `eAlongToGoal`.

Otherwise clean: naming/case/member-underscore conventions across
`source/drive/`, consistent unit tags, no integral/derivative accumulators in
the outer loops (no windup surface), walk-in law structurally non-negative,
terminal commands snap to literal zero — the wedge/reversal lessons are well
encoded for forward stop segments (#13 is the one gap).

---

# Part 3 — Dead code: `source/motion/` + `source/subsystems/`

**Totals:** ~3,621 lines certain firmware-dead in these two dirs (motion/
3,257 + nezha_hardware 364) — deletion already pre-authorized as ticket
100-013 / main.cpp's "later cleanup ticket", gated on bench/field sign-off —
plus ~60 lines of smaller certain finds, plus an orphaned hal/ cluster
(~1,100+ lines) that Part 4 covers.

## A. `source/motion/` — entire tree firmware-dead, parked (3,257 lines)

All of namespace `Motion` (segment_executor.{h,cpp} 403+1018,
jerk_trajectory.{h,cpp} 238+229, stop_condition.{h,cpp} 97+169, segment.h 62,
motion_baseline.h 41). Category 1, confidence **certain**:

- Zero `#include "motion/` anywhere in source/ — only the three ACTIVE unit
  harnesses (`tests/sim/unit/{stop_condition,segment_executor,jerk_trajectory}_harness.cpp`).
  All other mentions in source/ are retirement doc comments.
- subsystems/drivetrain.h:9–14 says it verbatim: parked, not deleted; ticket
  100-013 deletes it after bench/field sign-off. Superseded by `source/drive/`
  (master_profile.h is a hand-port of jerk_trajectory.h).
- Still COMPILED into the default ARM image (not on the CMake exclusion list;
  relies on `--gc-sections`) and into the host sim library, whose own
  CMakeLists comment calls it "dead weight, parked".
- **Test-only alive:** 3 harness .cpp + 3 pytest files in the active unit dir
  still exercise it. Deleting motion/ = delete those 6 files + the
  `MOTION_SOURCES` glob (tests/_infra/sim/CMakeLists.txt:153,175).
- Stale comment cross-refs to clean at deletion: main.cpp:33,116;
  hal/otos/otos_odometer.h:197; drive/types.h:138; devices/interpolation.h:66.
- motion_baseline.h:2–14 references `Subsystems::Planner` /
  subsystems/planner.h — relocated to source_parked/094/ two sprints ago.

## B. `source/subsystems/` — live tree, with one parked class and scattered dead members

- **B1 — `Subsystems::NezhaHardware` (nezha_hardware.{h,cpp}, 364 lines):
  firmware-dead, parked. Certain.** Only includers are unit harnesses
  (nezha_flipflop, drivetrain, hardware_seam + parked-093). main.cpp:96–107
  says it's parked pending "a later cleanup ticket". Still compiled into the
  ARM image. Deleting it orphans the parked Hal cluster only it reaches:
  hal/nezha/nezha_motor.{h,cpp}, hal/otos/otos_odometer.{h,cpp}, and possibly
  com/i2c_bus.{h,cpp} (reachability of `I2CBus` from elsewhere unverified —
  needs-human-check; Part 4 covers hal/).
- **B2 — Communicator: `state()` + `capabilities()` (communicator.h:86–87,
  .cpp:86–99) dead, certain (~18 lines)** — zero callers; the
  `msg::CommunicatorState/Capabilities` types are never published or emitted.
  `serialLines_`/`radioLines_` counters (h:110–111) are written but their only
  reader is the dead `state()` (~5 lines, falls out with it).
- **B3 — `Hardware::apply(CommandProcessorToHardwareCommand&)` /
  `apply(DrivetrainToHardwareCommand&)` seam (hardware.h:111–115):
  production-dead, test-only alive.** Zero production constructors of either
  edge type (093/094 teardown removed the producers; Drivetrain stages via
  `motor(i).apply(msg::MotorCommand)` instead). Removing the seam cascades to
  the three overrides (nezha_hardware — dying anyway; device_bus_hardware.h:335–336
  /.cpp:331–349; sim_hardware) and orphans the two edge structs in
  hal/capability/hal_command.h (~40 lines). Certain no production caller;
  **needs-human-check** whether the seam is deliberately retained for a
  planned DEV-command revival. ~105 lines total.
- **B4 — Subsystems::Drivetrain (live adapter) dead members, all certain
  (~37 lines):**
  - `v_y_` (h:325) — written, never read; `commandedWheelTargets()` reads only
    `v_x_`/`omega_`. Keep the `setTwist(v_x, v_y, omega)` signature, drop the
    member.
  - `lateSolveFailures_` (h:306, ++ at .cpp:160) — no getter, no reader.
    **Cross-ref Part 0 B1 / Part 2 #7: don't just delete this — it is the
    only trace of the silent segment-drop path; wire it to telemetry instead.**
  - `capabilities()` (h:198, .cpp:508–514) — zero callers anywhere.
  - `setMotorCapabilities()` + `leftMotorCaps_`/`rightMotorCaps_`
    (h:200–201,349–350, .cpp:516–520) — called from main.cpp:125 and
    sim_api.cpp:356, but the members' only reader is the dead
    `capabilities()`: a write-only chain end to end.
  - `active()` (h:243, .cpp:526) — only parked-093/094 harnesses (excluded
    from pytest) call it; `active_` itself stays (read by `state()`).
- **B5 — PoseEstimator: essentially fully live.** Only `config()`/`config_`
  (h:75,424) and `fixDropped()` (h:324) are test-only (harness-asserted;
  cheap to keep). Doc comments reference a nonexistent
  `commands/pose_commands.cpp` (h:228–229,251–252) — the real route is
  binary_channel `handlePose()` → `bb.poseResetIn` → tick drain.
- **B6/B7/B8/B9 — fully alive:** device_bus_hardware (production hardware
  layer), sim_hardware (host-build infra by design), drive_bridge.h (every
  function has verified callers), wire_command.h.

## Dead config key (needs-human-check — wire-protocol change)

- **`steer_headroom`** (PlannerConfig field 16): live-settable via the wire
  patch path (configurator.cpp:42 + wire.cpp codec row) but has ZERO readers —
  drive_bridge.h:60–64 documents "deliberately not read… Drive::Limits never
  grew a field for it". A SET round-trips into a cache nobody reads. Removal
  is a protocol change (wire keys excluded from rename convention).

## Verified ALIVE (things that might have looked dead)

1. subsystems/drivetrain.{h,cpp} — fully live wafer adapter fronting
   source/drive/ (governRatio, accel EMA, escape hatch, MOVER, ring, plan
   dump all have callers).
2. Nothing in source/ includes motion/ anymore — the earlier premise that
   subsystems/drivetrain.h still used it is out of date since the 100-007
   cutover (comment mentions only).
3. pose_estimator including the 099-008 pose-fix ring machinery — fully
   exercised from binary_channel and main_loop.
4. communicator (tick/hasCommand/takeCommand/send*/begin/configure) — live.
5. `Hardware::setMotorPolled/motorConfig/motorState/odometer/begin/motor` —
   production callers confirmed. Note: `setMotorPolled` on the live
   DeviceBusHardware hits the base no-op (only NezhaHardware overrode it) —
   a **behavioral gap**, not dead code.
6. The motion/ + nezha_hardware test harnesses are ACTIVE pytest — parked
   source deletion must take the tests and the sim CMake glob with it.

No never-enabled `#ifdef` branches and no commented-out code blocks in either
directory (only HOST_BUILD forks, every branch enabled by some build).

---

# Part 4 — Dead code: `source/devices/` + `source/hal/`

**Key verdict:** the cutover is complete — both firmware images run entirely on
`source/devices/`; only `hal/capability/{motor,odometer,null_odometer,hal_command}.h`
are reachable from any firmware entry point (via the Hardware seam). Everything
else under `source/hal/` is either firmware-compiled-but-unreachable (parked
legacy cluster) or host-sim-only. The bring-up image reaches ZERO hal/ code.

## Which duplicate is live in which build (the key question)

| Pair | Default image | Bring-up image | Host sim/tests |
|---|---|---|---|
| `hal/nezha/nezha_motor` vs `devices/nezha_motor` | devices (hal compiled, unreachable) | devices only | both harness-tested |
| `hal/velocity_pid` vs `devices/velocity_pid` | devices | devices only | **hal alive** (SimMotor embeds it + sim lib compiles it) |
| `hal/otos/otos_odometer` vs `devices/otos` | devices (hal unreachable) | devices only | both harness-tested |
| `hal/capability/motor.h` armor vs `devices/motor_armor.h` | devices armor RUNS; hal armor inert (only `processResetIfPending`) | devices only | both tested |
| `com/i2c_bus` vs `devices/i2c_bus` | devices (com unreachable) | devices only | both harness-tested |

## A. The deletable legacy cluster (~3,440 lines, certain)

All compiled into the shipping default image today (flash cost only what
`--gc-sections` misses); keystone is `subsystems/nezha_hardware` (Part 3 B1) —
deleting it orphans the rest:

- `subsystems/nezha_hardware.{h,cpp}` — 364 lines (Part 3 B1).
- `hal/nezha/nezha_motor.{h,cpp}` + `motor_slew.h` — 1,016 lines. Sole include
  path is nezha_hardware.h:78. Within it, vendor wrappers
  `timedMove()/resetHome()/setGlobalSpeed()/readVersion()`
  (nezha_motor.h:172–175, .cpp:707–783, ~85 lines) are dead even by the file's
  own admission — called by nothing including tests.
- `hal/otos/otos_odometer.{h,cpp}` — 940 lines. Only nezha_hardware.h:79 +
  test harnesses; live path is `DeviceBusOdometer` → `Devices::Otos`.
- `com/i2c_bus.{h,cpp}` + `com/i2c_bus_host.cpp` — 932 lines. Only consumers
  are the parked cluster (host fake is test-alive). Confidence: likely
  (clock.h include chain not exhaustively traced).
- `hal/capability/{gripper,ports,color_sensor,line_sensor}.h` — 191 lines,
  **never included by anything anywhere** (header-only faceplates, zero uses
  repo-wide). Certain; no CMake change needed.

**Prerequisite:** retire or re-point 5 live pytest modules
(`test_nezha_flipflop`, `test_hardware_seam`, `test_drivetrain` scenario 4,
`test_otos_odometer`, `test_motor_policy` — the last also uses hal/sim +
hal/velocity_pid so needs surgery, not deletion) and delete the parked-093
dir. No firmware CMake edits (glob-swept). Heavy comment rot referencing
NezhaHardware remains in main.cpp, hardware.h, runtime/*, scripts/gen_*.py.

**Keep:** `hal/velocity_pid.{h,cpp}` (host-sim alive — SimMotor embeds it),
`hal/sim/*` (host-only by design), `hal/capability/motor.h/odometer.h/
null_odometer.h/hal_command.h` (the live seam).

## B. `hal/capability/motor.h` — live file, firmware-inert armor half

On the default image the only Hal::Motor leaf is `DeviceBusMotor`, whose
`tick()` calls only `processResetIfPending()` — so `armoredWrite()`,
`updateRestTracking()`, `updateWedgeDetector()`, `trackAcceleration()` and all
their state fields (motor.h:140–210) never run on-target;
`wedged()/wedgeSuspect()/acceleration()` return constants into
`msg::MotorState`. This is the documented cutover limitation and the same gap
as **Part 1 M7** — the fix is the "virtualize/forward to handle" follow-up,
not deletion (hal/sim leaves still use the base armor).

Also: `Hal::Odometer::state()` (odometer.h:108) — declared, never defined,
never called. ~6 lines, certain.

## C. `source/devices/` member-level dead code (new code)

- **handles.h:** `ColorSensor::sampleAt()`, `LineSensor::sampleAt()`,
  `Odometer::sampleAt()`, `lerpUint()` — zero callers anywhere (~55 lines,
  certain). Only `Motor::sampleAt()` is called, and only by a test harness.
- **interpolation.h:** `lerpAngle()`/`wrapAngle()` only served the dead
  `Odometer::sampleAt()`; the file is test-only alive as a unit (92 lines).
  Needs-human-check — it's the documented DB-002 deliverable for a host-side
  time-alignment consumer that hasn't landed.
- **i2c_bus.{h,cpp} + i2c_bus_host.cpp:** the transaction-log ring
  (`setLogging()`/`dumpRecent()`/`logTxn()`/`TxnLog` + fields) is
  write-guarded dead — `setLogging()` has zero callers so `logOn_` is
  permanently false (~110 lines across the three files, certain; the twin API
  in com/i2c_bus.h dies with the cluster). Re-entrancy accessor quartet
  (`resetStats()`/`reentryViolations()`/`reentryInFlightAddr()`/`reentryNewAddr()`,
  ~25 lines) — capture fields written, never read. `clear()` — firmware-unused
  (fiber sleeps fixed settles instead), test-only.
- **otos.{h,cpp}:** `resetTracking()`, `setOffset()`, `getOffset()`,
  `signalProcessConfig()`, `imuCalibrationSamplesRemaining()` (~55 lines) —
  unreachable from both images (the `Odometer` handle exposes only
  `setPose()`; DeviceBusOdometer's OI/OR/OL/OA overrides are documented
  no-ops). Needs-human-check: these are the natural landing pads for
  restoring OI/OR/OL/OA through the handle (cutover limitation #4).
- **nezha_motor.h:** `sampleTime()` (:133) — zero callers, certain.
  `pidEnabled()` — test-only.
- **motor_armor.h:** `hardResetCount()`/`softResetCount()` — no firmware
  reader (test-only, ~6 lines).
- **line_sensor:** `captureCalibMin()/captureCalibMax()/setSmoothingAlpha()`
  (~40 lines) — no callers anywhere live; ported API awaiting a calibration
  verb that doesn't exist. Likely.
- **Deliberate, not dead (flagged for the record):** `DeviceBus::stop()`/
  `neutralizeAllMotors()`/FiberRunner join machinery (DB-008 lifecycle/safety
  contract, test-exercised, no firmware caller yet); `setIrqGuard()` (bench
  A/B, non-negotiable default-ON policy); `color()`/`line()` handles in the
  default image — the fiber samples and publishes those rings but NOTHING
  bridges them to any wire command (cutover limitation #5): running-but-
  unconsumed output, a completeness gap rather than dead code.

## D. Dead build flags (CMakeLists.txt) — all certain unless noted

- **`BENCH_OTOS_ENABLED` + the entire `PRODUCTION_BUILD` option block**
  (CMakeLists.txt:360–378): the macro is referenced by ZERO files —
  BenchOtosSensor and the DBG OTOS commands were deleted. The
  PRODUCTION_BUILD toggle currently toggles nothing. (Note: the overnight
  rig report described the build as "production firmware …
  BENCH_OTOS_ENABLED" — the flag is inert either way.)
- **`USE_ORDERED_TICK`** (CMakeLists.txt:380–382): referenced nowhere;
  LoopTickOnce.cpp no longer exists.
- **`hal/ReplayHAL.cpp` exclusion regex**: the file doesn't exist.
- **`ROBOT_DEV_BUILD`** (codal.json config + sim CMakeLists:21): no `#ifdef`
  remains (prose comment only) — likely dead; check codal.*.json variants
  before removing.

---

# Combined dead-code totals

| Cluster | Lines | Confidence |
|---|---|---|
| `source/motion/` whole tree (ticket 100-013) | 3,257 | certain |
| Legacy device cluster (nezha_hardware + hal/nezha + hal/otos + com/i2c_bus + 4 capability faceplates) | ~3,440 | certain (com/i2c_bus: likely) |
| Member-level dead in live subsystems/ files (Part 3 B2–B4) | ~60 | certain |
| Member-level dead in devices/ (Part 4 C) | ~290 | certain/likely mix |
| Dead build flags + stale CMake regexes | ~25 | certain |
| **Total** | **~7,100 lines** | |

Plus needs-human-check items: Hardware::apply seam pair (~105 lines),
`steer_headroom` wire key (protocol change), interpolation.h (DB-002
deliverable), Otos config setters (OI/OR/OL/OA landing pads),
PoseEstimator::config()/fixDropped().
