# Post-sprint-130 review: the wheels-solid base, the planner, and the road to a characterized sim

**Date:** 2026-08-02 · **Tree:** `master` @ `a5f7b06b` (v0.20260802.2, sprint 130 merged and closed)
**Requested by:** stakeholder, 2026-08-02 — remove cruft and layered hacks, reassess rotted
structures, absorb the sprint-130 residuals, and assess how much the hardware can actually
be asked to do. Framed against two goals: **(G1)** the base firmware rock-solid on its own,
with PID-managed wheel speed as its core contract, testable and observable; **(G2)** the
hardware characterized well enough to drive an error-model simulation the motion planner
can be developed against.
**Method:** six parallel read-only review agents (firmware base, wheel-controller path,
motion planner, host tree, sim stack, sprint-130 residuals synthesis), findings
cross-checked against each other, the sprint-130 post-mortem
(`docs/post-mortem/2026-08-02-sprint-130-wheels-solid.md`), and spot-verified against the
tree by the team-lead (every CRITICAL below was re-read at the cited lines). Format per
`docs/code_review/GUIDELINES.md`. **Findings are discussion items, not work orders.**

---

## Verdict

Sprint 130 genuinely improved the tree — one controller, one composition root, one state
machine, honest measurements over claimed ones — and the review found the *skeletons*
sound: RobotState's ownership discipline held, the Drive staging order is right, the
planner is two includes away from full separability, and the sim/systest rails are better
than they looked from the issue list. But the tree is **not yet the thing G1 and G2 ask
for**, for three structural reasons. First, a small set of real correctness defects sit
directly on the core contract (a pose-destroying rebaseline, a takeover path that wipes
the controller's learned state on every MOVE, a stop that Stage B can leak through, a
host stack whose recommended tools raise AttributeError or silently no-op). Second, the
observability the contract depends on is largely wired to nothing — the deficit flag
cannot fire as shipped, three fault bits are dead, one is saturated, the setpoint isn't
on the wire, and there is still no config read-back. Third, the controller corrects the
quantity the plant keeps stable (intercept) while hard-coding the quantity that
measurably drifts ±25% between sessions (gain) — so the accuracy story is bounded by a
design choice, not by tuning. None of these is hard to fix; all of them are the kind of
accretion this review exists to catch. The sim, meanwhile, has excellent bones and an
idealized heart: every error knob biases what the robot *believes*, none biases what its
body *does*, which is exactly backwards for G2 — and the fix path is short because the
seams already exist.

---

## Part 0 — The twelve findings that matter most (cross-domain, ranked)

### F1. CRITICAL / correctness — Position rebaseline destroys the pose; `positionEpoch` has no consumer

**Evidence (team-lead verified):** `src/firm/app/robot_loop.cpp:369-374` rebaselines a
wheel at ±30,000&nbsp;mm and bumps `positionEpoch`; the same cycle's
`robot_loop.cpp:541` feeds raw `motor.position()` into
`Motion::Odometry::integrate()` (`src/motion/odometry.cpp:17-21`), which computes a
bare delta with no epoch check — a ~−30,000&nbsp;mm step: heading jumps ~234&nbsp;rad,
x/y by ~15&nbsp;m. The planner's `PoseTracker`/`WheelChannel`
(`src/motion/planner/estimation.cpp:8-48`) have the identical raw-delta shape, so an
in-flight Move at the boundary is corrupted too. `positionEpoch` — invented to signal
exactly this — is consumed *only* by the telemetry wire field; no motion-side code
reads it. **Trigger:** ~30&nbsp;m net travel on a wheel ≈ 75&nbsp;s of 400&nbsp;mm/s on
the stand — a plain soak run. Secondary: `rebaseline()` zeroes `velocity_`, so the
wheel reports v=0 for 1–2 cycles mid-motion. The only test of the policy checks the
wire epoch — and lives in a harness that doesn't compile (F8).
**Design answer:** the epoch must have consumers — `Odometry` and the planner's
channels re-anchor on epoch change (both already have the re-anchor primitive), or
`publishWheel()` hands motion the delta rather than the absolute position.

### F2. CRITICAL / correctness (design-vs-plant) — The controller adapts the intercept; the plant drifts in slope

**Evidence:** `Drive::kDutyPerSpeed = 0.001182f` ("MEASURED, NOT CONFIGURED",
`src/firm/app/drive.h:138`) is baked from the 07-31 fit (853.6/837.8&nbsp;mm/s-per-duty)
that the 08-01 population sweep note in `data/robots/tovez.json` explicitly
*supersedes* (left refit 1101.3 fwd / 1023.1 rev — the old fit blended saturation into
the slope). Across three sessions the measured plant gain and ceiling swung ~25%
(saturation 760–795 → 584 → 571.7&nbsp;mm/s L), and the 130-001 parallel-lines verdict
came back **slope-dominated (fanned)** — the opposite of the intercept-only hypothesis
Stage C is built on (n=3, LOW CONFIDENCE, but it is the only population data that
exists). The one online corrector is an additive ±23.8&nbsp;mm/s bias with a 30&nbsp;s
time constant; a 20–25% gain error is 30&nbsp;mm/s at cmd 150 and 80–120&nbsp;mm/s at
cmd 400 — beyond the bias clamp at any speed above ~120&nbsp;mm/s, and unreachable
inside a 4&nbsp;s move regardless. Measured consequence: fresh-start delivery
**70–85% of commanded** (130-006), silent (F5).
**Design answer:** adapt or re-anchor the *slope* — cheapest first: a boot-time
constant-free saturation probe scaling `dutyPerSpeed` per session (the six-second
measurement the post-mortem already canonized); or a bounded slow multiplicative gain
trim with Stage C's exact clamp/steady/freshness discipline; or pack-voltage telemetry
so the drift can finally be attributed. Until one lands, the honest contract is the
one in Part 6.

### F3. CRITICAL / correctness (for G2) — The sim's plant is an idealized toy and its error knobs bias belief, not motion

**Evidence:** `src/tests/sim/plant/wheel_plant.h:44-45` — one shared compile-time gain
(`kDutyVelMax=500`), linear from duty 0, symmetric, no breakaway, no saturation knee,
no two-wheel coupling; `src/sim/sim_harness.h:130` then installs the plant's *exact
inverse* as the controller's feedforward, so the sim controller lives in a
zero-map-error world by construction. Every runtime error knob
(`sim_plant.h:117-157`: encoder freeze/jitter/scale, OTOS drift) applies in
`reported*()` — the *belief* side; true motion is untouched by design. The measured
plant realities (±25% session gain, breakaway 0.10–0.16 duty, saturation ~570,
54&nbsp;ms delivered period, load coupling) are all unrepresentable without
recompiling.
**Design answer:** Part 5 — the seams exist; parameterize the wheel plant from
measured data, add the true-motion error half, decouple delivered from believed
period. Note the planner-tier `NoisyPlant`
(`src/motion/planner/tests/test_support.h:95-165`) already has the right vocabulary —
per-wheel gain, lag, quantization, staleness — it just never reached the full-firmware
sim.

### F4. CRITICAL / correctness — The recommended host tools are dead or lying: `rogo repl` motion verbs raise, its `stop` is the planned stop, and the MCP calibration push silently no-ops

**Evidence:** `.claude/rules/hardware-bench-testing.md` directs users to `rogo repl`,
but `repl.py:199,225,246` call `proto.twist(...)` — deleted at the 116-001 MOVE
cutover → AttributeError on the first motion command; `repl.py:339` builds an
`envelope_pb2.Twist` that does not exist (field 19 reserved). Worse, the repl help
table says `stop — panic-stop the drivetrain` (`repl.py:363`) while `stop()` is the
PLANNED stop (measured: full 39.8&nbsp;cm leg driven first), and **the repl has no
estop verb at all** — a user typing `stop` during a runaway gets the queued stop, the
exact defect class `.claude/rules/playfield-testing.md` exists to prevent (Ctrl-C
cleanup does go through `halt_now`; only the interactive verb surface is wrong).
Meanwhile the MCP connect-time calibration push (`robot_mcp.py:97` →
`calibration/push.py:386-395`) writes v2-era text `SET k=v` lines that v5 firmware
has no plane for — nothing is applied — and returns `{"status": "ok"}`: every MCP
session runs on build-time calibration while claiming otherwise. This is the same
defect `rogo sync cal` was retired for (124-014), alive one file over.
**Design answer:** repl motion verbs move to `move_twist`/`move_wheels`; `stop`
relabeled honestly and an `estop` verb added; the MCP push routed through the same
`binary_bridge.translate_command()` path the TestGUI already uses (which is live), or
deleted until it can be.

### F5. MAJOR / accretion+correctness — Ownership handover is implemented as `estop()`: every accepted MOVE wipes the controller's learned state

**Evidence (team-lead verified; found independently by two reviewers):**
`src/firm/app/robot_loop.cpp:227` calls `drive_.estop()` on *every* non-duplicate
accepted MOVE; since 130-004, `src/firm/app/drive.cpp:43-50` also zeroes both PID
integrators, both biases, and the deficit latches. A chained tour that tops up the
queue mid-leg resets the 30-second-timescale bias *mid-flight* (a non-bumpless ~1–2%
duty step — biasR had converged to +14&nbsp;mm/s); between legs, every Move cold-starts
adaptation, so Stage C can never converge on the planner path — the navigation surface
it exists for. The teleop path (`handleWheels`, line 275) keeps Drive's bias — the
asymmetry is undocumented and backwards. Provenance: the takeover `estop()`
(bd7f75b8, 07-27) predates the controller state added to `estop()` (130-004); the
interaction was never examined.
**Design answer:** split `Drive::takeover()` (zero targets, disarm, keep learned
state) from `Drive::estop()` (full safety reset, reserved for the ESTOP verb). Pair
with sign-aware bias application (Part 2, C3) before enabling it across reversals.

### F6. MAJOR / explicitness — The "loud, never silent" observability contract is inert as shipped

**Evidence:** the deficit latch requires `pidMax > 0 && |pid| >= pidMax`
(`src/firm/app/drive.cpp:338-350`) and every robot ships `wheel_pid_max=0` — so the
single loud observable for F2's silent slow-running *cannot* fire (130-006 ran four
sweeps at 70–80% delivery; the flag stayed clear — correct per code, useless per
contract). Around it: `kFlagFaultI2CNak` (bit 8) has no data path —
`MicroBitI2CBus`'s `errCount()/reentryViolations()` and its transaction ring have
**zero callers** (`microbit_i2c_bus.h:88-145`); bit 6 saturates within ~1&nbsp;s of
boot because every OTOS read trips the clearance safety net (`otos.cpp:333-353`);
bits 10/12 (deadman, config-applied) are wired to nothing; `msg::Telemetry` carries
**no commanded wheel velocity and no applied duty**, and no effective gains; and
there is no firmware→host config read-back at all, while flash-persisted `pid.*`
silently overrides the JSON at boot (`configurator.cpp:89-103`,
`boot_wiring.cpp:108-118`) — three layers of tuning truth, none observable. The
post-mortem independently ranked config read-back the #1 missing capability.
**Design answer:** a `ConfigSnapshot` arm in `config.proto`; per-wheel
`cmd_velocity` (+ applied duty) in the frame; decouple the deficit flag from
`pidMax` (bias-pinned OR sustained-error should speak); widen `Devices::I2CBus` with
the error aggregate; count only *unexpected* clearance trips; delete or wire bits
10/12.

### F7. MAJOR / accretion — One physical quantity, many constants: the period (50 vs 54), the floor (90 vs 99.7), the map (three duty_per_speeds)

**Evidence:** the loop delivers a rock-stable **54.000&nbsp;ms** against `kCycle = 50`
(130-011; same +4 at kCycle=40 → 44 — structural: the four `runAndWait` gaps sum to
kCycle but real work runs *between* the marks, `robot_loop.cpp:512-534`, plus
per-block whole-ms `fiber_sleep` round-up, while `robot_loop.h:49-57` asserts
"exactly 50&nbsp;ms"). Downstream: Drive integrates with measured dt
(`drive.cpp:298`) but the planner's profile math and `cmdAccel` use the baked 50
(`planner.cpp:1347`) — a systematic 8% disagreement *inside one control path*; the
sim can only ever step at exactly 50 (F3); and 130-011 showed 8% is enough to flip a
marginal loop (heading-hold) unstable. Same pattern at the floor: firmware
`wheel_v_min=99.7` (`drive.cpp:150-156`) vs the host taper's `_UNMANAGED_FLOOR =
90.0` (`testgui/transport.py:200-201`); and at the map: baked 0.001182
(`drive.h:138`) vs the JSON's dead 0.00187325 (still generated, deliberately ignored,
`boot_calibration.cpp:83-93`) vs `duty_sweep.py`'s hand-mirrored
`KNOWN_DUTY_PER_SPEED` vs the sim path pushing the JSON value
(`sim_loop.py:715-719`) — a TestGUI sim session runs a *third* feedforward. This is
the exact duty_per_speed/wheel_gain disease sprint 130 was chartered to kill, alive
at three other layers.
**Design answer:** per layer, pick the one owner. Period: make the final block an
absolute end-of-cycle deadline (`sleepUntil(cycleStartMark, kCycle)`) so delivered ==
nominal, *or* rename the constant to mean the delivered value — then feed planner,
Drive, and sim from that one number. Floor and map: single source, everything else
reads it (config read-back, F6, is what makes the host side honest).

### F8. MAJOR / correctness (latent) — Stop-is-stop has a Stage-B-sized hole, and Stage B has no freshness gate

**Evidence:** the zero-command guard lives only in `correctedCommand()`
(`drive.cpp:114`); `tick()` adds the PID term unconditionally
(`drive.cpp:299-313`). With gains on: a retained integrator after normal WHEELS
expiry keeps duty ≠ 0 at rest, `commandedStop`/`alreadyQuiet` require duty
`== 0.0f` exactly, and `writeShapedDuty()` *boosts* any nonzero sub-deadband duty to
the 3% floor (`nezha_motor.cpp:274-280`) — a parked robot can creep or buzz on
encoder-quantization noise. Separately, Stage B consumes `state.wheelLeft.velocity`
with none of Stage C's fresh/connected/frozen gates (`drive.cpp:299-306` vs
`:323-328`), and a failed encoder collect manufactures velocity=0
(`nezha_motor.cpp:468-487`) — so Stage B will wind against a wheel it cannot see, and
any tuning sweep run on a glitching bus is contaminated. Latent today *only* because
gains ship 0 — i.e. exactly until the first thing the next bench session does.
**Design answer:** commanded-zero gets the same explicit treatment the intercept got
(skip PID, or actively decay the integrator, at command==0); Stage B adopts Stage C's
measurement gates.

### F9. MAJOR / correctness (meta) — The base's own integration coverage is a corpse, and the suite's green overstates it

**Evidence:** `app_robot_loop_harness.cpp` includes `motion/move_queue.h` and
`motion/state_estimator.h` — deleted two sprints ago; `test_app_robot_loop.py:110`
xfails it as "does not COMPILE at all" (~28 errors), non-strict, silently green since
07-26. Its unrelocated coverage — boot-NACK, routing arbitration, **the rebaseline
policy (F1)**, most flag derivations — exists nowhere else. `motion_tests` runs zero
tests by design (`src/motion/CMakeLists.txt:83-98`), so `body_kinematics`/`odometry`
(the very code F1 lives in) have no standalone coverage either. Two other stale
xfails now xpass. Meanwhile four sprint-130 tickets measured "no regressions" against
a baseline of "2 known failures" that turned out to be tests that had *never
compiled* (fixed in 87cbdb2d — the post-mortem's proxy-checking finding, in the test
tier itself).
**Design answer:** delete the corpse harness and rebuild the missing scenarios on
`TestSim::SimHarness` (where the TLM half already moved); give F1 a pose-sanity test
across the rebaseline boundary; make surviving xfails strict.

### F10. MAJOR / correctness — `decelLatched` is a one-way trap: one pessimistic tick can silently truncate a Move (top TOUR_2 hypothesis)

**Evidence:** once any tick's `profileStep()` returns Decel/Closing,
`active_.decelLatched` clamps every later λ to min (never rise) for the Move's
remainder (`planner.cpp:1232-1240`) — but the closing step triggers off
`plannedRemaining`, a *prediction* over sample-age + actuationDelay
(`planner.cpp:756-824`). One transient under-estimate drives the command to ~0, the
latch forbids recovery (directly contradicting `profile.cpp:102-104`'s "let
re-measurement recover"), and the 0.5&nbsp;s stall backstop completes the Move
wherever it parked, `settled=false`. That is an **angle-independent, additive**
short-completion mechanism — matching the TOUR_2 residual's signature (146° turn
−10.12°, residual does NOT scale with angle). Fingerprint to check on the single-step
harness: a stall-event completion with no timeout on that turn. No test exercises a
transient misprediction against the latch; no Angle scenario above 90° exists in
`planner_tests`.
**Design answer:** let the latch release when measured remaining *rises* materially
above the closing envelope (re-measurement recovers, as the comment promises), and
add the missing scenarios (large angles, tour-shaped chains under `NoisyPlant` lag).

### F11. MAJOR / interface-bleed — The `Nezha` facade is a protocol-v2 corpse with three pose owners; the CLI's goto family points at the dead stack

**Evidence:** ~29 of ~36 public methods on `robot/nezha.py` call deleted
`NezhaProtocol` methods — `rogo drive/turn/go/grip/enc --zero/ez/sync pose`, the MCP
drive/grip tools, and the whole `nav/navigator.py` raise AttributeError on first
hardware contact. Eleven `_proto._conn` private reach-arounds persist (plus
`pathplan/planner.py:564`'s documented one). Three independent pose owners coexist
with *disagreeing unit conversions* (`nezha.py:600-646` mm/centideg;
`nezha_state.py:124-165` converts to cm; `nezha_kinematic.py` a third WPILib
estimator) — the latter two have zero live consumers. The live, well-built goto
(`pathplan/planner.py` — verified estop-in-finally, geofence-inside-the-loop) has
**no rogo command**, while `rogo goto/turnto` tombstone onto the dead stack.
**Design answer:** the filed facade-rebuild issue is accurate — rebuild on the v5
binary surface with one pose owner; delete `nezha_state.py`/`nezha_kinematic.py`/
`kinematics/differential_drive.py`; point the CLI at `pathplan`.

### F12. MAJOR / process — The only sprint-130-introduced regression pair is tracked nowhere

**Evidence:** the post-mortem (`docs/post-mortem/...md:254`) and the knowledge doc
both cite `sprint-130-regressions-speed-floor-snaps-differential-and-shaper-defaults.md`
— `applySpeedFloor()` snapping a ~3&nbsp;mm/s planner differential correction up to
~99.7&nbsp;mm/s (a lurch), with **four testgui tour tests FAULTing on it** — and
`playfield-actuation-floor-...md:53` links to it. **The file was never created**
(repo- and history-wide search). A second referenced issue
(`make-irq-guard-off-permanent-and-reconcile-the-docs.md`) is also missing. The
mechanism itself is real: the floor runs on the *summed* per-wheel command, so
planner trim differentials are quantized to ±vMin (interacts with F7's floor split
and Part 2 C1's vMin-blind planner).
**Design answer:** create the issue first (the highest-value single file in this
review), then decide the floor's semantics: floor the *common-mode* speed, pass the
differential through.

---

## Part 1 — Firmware base (`src/firm`), remaining findings

Ranked; F1/F5/F6/F7/F8/F9 above originate here and are not repeated.

- **MAJOR / correctness — persisted-tuning keys repurposed without a schema bump.**
  130-005 repointed `pid.kff → kaff` [s] and `pid.kaw → pidMax` [mm/s]
  (`configurator.cpp:141-147`) but `Config::kConfigSchemaVersion` is still 2
  (`persisted_tuning.h:63`), violating its own documented bump rule. A robot carrying
  pre-130 duty-domain gains in flash will boot them into mm/s-domain fields —
  silently arming Stage B with wrong-domain numbers. *Design answer:* bump the schema
  (the mechanism exists precisely for this); ideally also NACK old-shape snapshots
  loudly.
- **MAJOR / placement — line sensor 100% dead; perception pacing has no owner**
  (team-lead verified: `cycleCount_` declared `robot_loop.h:201`, tested
  `robot_loop.cpp:544`, incremented nowhere — the header comment at `:195`
  *documents* the defect instead of fixing it, since 8a691651, 07-26). Preamble
  still spends boot I2C probing a sensor that will never be read. The flip-flop,
  `packLine()/packColor()` and publish all live inline in RobotLoop. *Design
  answer:* the chartered Sensors subsystem owns the cursor and pacing budget;
  increment-or-delete decides whether the line sensor exists at all (it costs an I2C
  transaction per 2 cycles when live — measure per the existing issue).
- **MINOR / correctness — freshness dishonesty on a dead bus:** `NezhaMotor::tick()`
  stamps `lastTickUs_` even when the collect failed, so `EncoderReading.age` reads
  fresh on a disconnected bus; `motor.h:103-109` promises a `lastFreshUs_` that does
  not exist. Stage C survives via the separate `connected` conjunct only.
- **MINOR / correctness — ESTOP latency floored at one cycle by design:** drained in
  block 3 (~12–16&nbsp;ms in), first duty write next cycle's `drive_.tick()` — worst
  case ~54&nbsp;ms command-to-zero + stop-reissue + spin-down ≈ the measured
  ~0.19&nbsp;s. A staged flag serviced at the first legal bus slot of the *same*
  cycle saves up to a full cycle. (The pending issue's arithmetic still says
  40&nbsp;ms — stale in the wrong direction.)
- **MINOR / correctness — OTOS velocity registers decoded with position LSB
  constants** (`otos.cpp:137-143`) — known and carried, but it matters now that
  estimator-v2 OTOS fusion is queued: the wire's `otos.v_*` are systematically
  mis-scaled.
- **MINOR / correctness — MOVE-timeout is one flag frame on a 5–11%-loss link**
  (completion ack retries carry `err=0` always; timeout is flags-bit-15 only) — ~1
  in 10–20 timeouts invisible to the host.
- **MINOR / explicitness — `command.mode` granularity is dead** (only
  `Velocity`/`Idle` ever written; `Streaming/Timed/Distance/GoTo` enum values dead)
  and `robot_state.h:230-242` describes the write order backwards.
- **MINOR / explicitness — cleartext replies can spin up to 5&nbsp;ms inside a
  4&nbsp;ms settle window** (`serial_port.cpp:76-90` bounded TX spin under
  `comms_.pump()` in all four blocks) — host-chattiness-dependent jitter the pacer
  can't reclaim; compounds F7.
- **NOTE — `Preamble::probeSlot` has no timeout** (post-mortem rec #3): the most
  common bench condition (motor power off) produces total silence and needed a
  debugger four times in two days. Bound it and report it — this is also the
  observability face of the wedge issue.

**Cruft ledger (firm)** — verified zero-consumer/stale, all cheap deletes or one-line
doc fixes: `Config::EstimatorBootConfig`/`ShaperBootConfig` + defaults (zero
consumers); generated `msg::MotorConfig` vel-gains + `setVelFiltAlpha` baked every
build, read by nothing, comments citing command surfaces dead since 102–107; JSON
keys `duty_per_speed_*` (generated, deliberately ignored — the cleanup lost its
tracking issue when 04-continuous-... closed), `v_wheel_max=620` (two-generations-
stale plant claim), `vel_*`, `steer_headroom`, `wheel_step_max`, `track_k_*`,
`trim_v_max/omega_max`, `replan_*`, `handoff_*`, `arrive_*` (no generator consumer);
`EstimatorConfigPatch.weight_*/staleness_ms` accepted on the wire, applied nowhere,
acked 0 (silent-off — NACK `ERR_UNIMPLEMENTED` or track in estimator-v2);
`Comms::setStatus()` dead API; stale doc/comment set: `src/firm/DESIGN.md` (pre-122
shape; three different kCycle values across DESIGN docs vs actual 50),
`app/DESIGN.md` (deleted classes as live; defends the pacing design F7 indicts),
`types/DESIGN.md`, `design.md` §2 rows listing deleted classes, `telemetry.cpp:19`
ack-ring size comment, `serial_port.h` framing comments, `line_sensor.h` phantom
methods, `boot_config.h` phantom static_asserts, `fake_otos.cpp`/`radio.h` dead
references. `src/firm/motion/` and `src/firm/kinematics/` are code-empty
validator-placater directories — fold their DESIGN.md content and delete. (Genuinely
good hygiene note: zero TODO/HACK/FIXME markers anywhere in `src/firm`.)

**Structural assessment (firm):** RobotState blackboard **sound**; devices tree
**sound** (NezhaMotor write shaping is careful, measured engineering); config
codegen **sound, carrying dead weight**; comms/telemetry **sound** except the flag
word; `App::Drive` **needs targeted rework** (F5/F6/F8); RobotLoop schedule **needs
rework** (F1 consumer, F7 anchor, perception owner); test infrastructure **needs
rework** (F9).

---

## Part 2 — Wheel-speed controller path, remaining findings

F2/F5/F6/F7/F8 originate here or in Part 0. Remaining, ranked:

- **C1. MAJOR / correctness — the vMin floor silently rewrites the planner's
  terminal taper.** Every nonzero command below 99.7 is boosted
  (`drive.cpp:150-156`); the planner's jerk-limited taper sweeps 100→0 continuously
  and has **no concept of vMin** (`PlannerLimits` carries none) — delivered landing
  is "hold 99.7, step to 0". With `crawl_pulse=0` shipped there is *nothing* below
  breakaway: minimum sustained speed ~100&nbsp;mm/s ≈ 5.4&nbsp;mm of travel per
  54&nbsp;ms tick, while `settle_epsilon_linear=4.0&nbsp;mm` still promises a
  4&nbsp;mm arrival tolerance. This is also F12's regression mechanism against
  differential corrections. *Design answer:* the planner must know the floor (a
  `PlannerLimits` field fed from the same single source), the floor must exempt the
  differential, and the 4&nbsp;mm epsilon waits on the real measured actuation floor
  (deferred 130-012 issue).
- **C2. MAJOR / correctness — on the teleop path, `cmdAccel` is the planner's
  zero:** the sole writer is `planner.cpp:1341-1349`; Drive never writes it. A
  teleop step 0→150 looks "steady" from tick one — Stage B winds through the
  transient, `kaff` feedforward is permanently zero on teleop, and the 90&nbsp;s
  Stage-C proof run ran entirely in this mis-gated regime. Also divides by nominal
  50 while Drive integrates measured 54 (F7). *Design answer:* Drive derives its own
  commanded-accel from its own target sequence; the field carries one owner per mode
  like `cmdVelocity` does.
- **C3. MINOR / correctness — signed bias mis-signs across direction reversal**
  (`drive.cpp:122`: `copysign(magnitude, desired) + bias`): a forward-learned +14
  bias *reduces* reverse magnitude and perturbs pivots until re-learned at
  tau=30&nbsp;s. Latent today only because F5 resets bias constantly; becomes real
  the moment F5 is fixed — fix them together (bias per signed-direction, or decay on
  sign flip).
- **C4. MINOR / correctness — `correctedCommand()` returns hard 0 below the
  intercept** (`drive.cpp:121`) — latent while correction is identity, but a future
  refit with intercept&nbsp;>&nbsp;0 silently zeroes small commands *after* the
  floor already ran. The same silent-stall class the floor was built to kill.
- **C5. NOTE — deadband config vs measured breakaway:** `output_deadband=0.03` vs
  measured breakaway 0.10 fwd / 0.164 right-reverse — the boost compensates to a
  boundary 3–5× below the real one. Re-measure and re-bake alongside the map.

**Verified clean:** ticket 007's duty-stage deletion left no half-deleted remnants —
no `WheelPid`/`stageDuty`/`applyVelGains`/`dutyFloor` code anywhere; removal
comments only. The staging order (map+bias → PID → duty; bias computed post-duty,
rate-limited, bumpless), anti-windup shape, NaN fail-closed, measured-dt
integration, and the fail-closed uncalibrated refusal are all correct and
well-tested.

---

## Part 3 — Motion planner (`src/motion`), remaining findings

F10 originates here. The standalone suite builds and passes 8/8 in ~2&nbsp;s with no
Python and no firmware — the separability story is fundamentally sound.

**Separability scorecard (exhaustive, grep-verified):** exactly **three** includes
cross into `src/firm`: `planner.h:35` + `tests/test_support.h:13` →
`types/robot_state.h` (the sanctioned boundary, itself `<cstdint>`-clean), and
`body_kinematics.{h,cpp}` → `messages/common.h` for the `msg::BodyTwist3` array
overloads only. No `Devices::*`, `App::*`, `Config::*`, clock, bus, or telemetry
type anywhere. **Path to zero:** (1) relocate `robot_state.h` to a neutral shared
root so a future `subtree split` of `src/firm` doesn't take the planner's one
include with it — a `mv` plus include edits; (2) give `BodyKinematics` a
motion-owned twist triple (one header, ~4 call sites). The real coupling is the
**build**: the planner source list is duplicated in ~14 places — its own CMake,
`src/sim/CMakeLists.txt:135-143`, the root glob's exclusion regexes, and **eleven**
pytest harnesses hardcoding the four .cpp paths. A missing entry is a link error
(memory confirms this bites). *Design answer:* one shared source-list constant/glob
for the pytest tier; the planner dir globbed like the ARM build.

- **P1. MAJOR (part of F9) — `app_robot_loop_harness.cpp` deletion** — see F9.
- **P2. MINOR / correctness — the 130-010 Angle completion gate is a no-op for
  Wheels-kind Moves:** the gate tests `profileVelocity_ ≤ settleRestOmega`
  (`planner.cpp:552-558`) but the Wheels branch never writes `profileVelocity_` and
  `activateNext()` zeroes it — `|0| ≤ floor` is trivially true; the comment sells a
  guarantee the code doesn't provide. Tours use Twist, so TOUR_2 is unaffected.
- **P3. MINOR / correctness — `estop()` doesn't invalidate the ledger carry:** a
  completes-into-pending-Stop window (`planner.cpp:705-714`) lets a post-estop Move
  adopt pre-estop baselines. One line (`carryValid_ = false`) closes it.
- **P4. MINOR / explicitness — `move(replace=true)` leaves `lifecycle_` stale until
  next tick** while `estop()` got the immediate-consistency treatment — the two
  preemption paths disagree on observability.
- **P5. MINOR — Wheels Moves ramp under linear ceilings only** (never
  `alphaMax/alphaDecel`) — consistent with "direct wheel commands" but undocumented
  at the limits level.
- **P6. MINOR — ctypes mirror drift:** `planner_harness.py` still declares
  `MovePhase.SETTLE = 4` (deleted in 130-008) and `plannerTrim()` keeps its stale
  ABI name; the `PlannerLimits` mirror itself is verified coherent (18 fields,
  offsets checked). `RobotState`'s mirror has no per-field offset guard — a
  same-size mid-struct swap corrupts silently; add the offset table it already has
  for limits.

**State machine verdict (ticket 008):** genuine state machine, not renamed booleans —
stored lifecycle, one written-down transition table, explicit event priority, every
transition behaviorally pinned. Caveats: parts are re-derived per tick from
observables (the documented stale-`Breakaway` label survives, defended only by every
consumer re-testing a boolean — the exact style the rewrite retired), and
`MovePhase`+`decelLatched` (F10) is a parallel latch living *outside* the machine —
the one wedge state the lifecycle can't see or escape.

**Test quality:** real behavioral tests (physical assertions, injected disturbances,
sign-asserted monotonicity), and `NoisyPlant` already gives the scripted-fake-plant
capability G2 asks for at the planner tier. Missing: any Angle case above 90°,
chained leg→large-turn→leg under lag, transient-misprediction-vs-latch (all F10),
and a characterized parameter set matching the measured plant instead of round
numbers.

**Cruft (motion):** `WheelChannel::positionAt()` (zero callers),
`ActiveMove::closingIssued` (written, never read), `PoseTracker::blendHeading()`
(production-dead, deliberately kept for estimator-v2 — documented, fine),
`profile.h:51` stale WheelTrim comment, root CMake still naming deleted
`wheel_pid`, DESIGN.md counts off by one. Host `planner/profile.py` dormancy
**confirmed** (only its own test + a re-export) — the filed delete-or-keep decision
stands ready. PlannerLimits 34→18 is coherent; no orphan config keys
(`plant_gain`/`plant_tau` are documented recorded-but-unread measurements); `pid.*`
wire keys verified repointed.

---

## Part 4 — Host tree and bench tooling

F4 and F11 are the headliners. Remaining, ranked:

- **H1. MAJOR / correctness — `rogo enc/opos/line/color` read passively on a
  firmware whose idle default is silent** (`cli.py:1060-1074` never sends
  `TLM:NOW`; firmware `TlmMode` default is `kAuto` — silent when parked). On an
  idle robot `rogo enc` prints cached `ENC 0 0` as fact. `protocol.py:28-31` still
  asserts "telemetry is always-on (no arming step)" — contradicted by the
  `tlmOn/tlmOff/tlmNow` section in the same file.
- **H2. MAJOR / correctness — `VisionConfig.robot_tag_id` still fails open to the
  field-origin tag** (`robot_config.py:77` — `robot_tag_id: int = 1`; three
  hardcoded `100` fallbacks at `cli.py:166,645,747`). A config that omits the field
  confidently tracks the stationary origin tag as the robot. The filed issue is
  accurate and unfixed.
- **H3. MAJOR / placement — TestGUI monolith:** `_build_main_window()` is ~2,935
  lines (`__main__.py:308-3243`) with the Test S/T edit→cmake→dylib-hot-swap build
  pipeline still embedded in the GUI (`:3115-3243`). The chartered controller
  extractions are not started. (The STOP path itself is now correct and
  acceptance-tested — credit where due.)
- **H4. MAJOR / accretion — the calibration stack is dead twice over:**
  `io/calibrate.py` calls six deleted protocol methods (AttributeError on trial 1);
  `calibration/linear.py`/`angular.py` write raw v2 text that fails *silently* at
  the wire — they "complete" without moving anything. Two parallel interactive
  stacks is how this rotted unnoticed; the stakeholder fold-or-delete decision is
  pending.
- **H5. MAJOR / correctness (G2 blocker) — `fit_sim_error_model.py` is unrunnable**
  (targets deleted SIMSET registry, nonexistent `src/sim/firmware.py`, frozen text
  parser). Everything downstream of "fit an error model" must be rebuilt — see Part
  5 item 4.
- **H6. MINOR:** sim geometry override half-purged (Sim Errors panel still has a
  live `trackwidth` spin, `__main__.py:1087-1088,1159`); TraceModel
  `_feed_otos/_feed_fused` baselines only refresh while appending
  (`traces.py:418-425` — the one live item from the misc-changes issue);
  `robot_config.py` has no `extra="forbid"` anywhere — the silently-dropped-key
  class remains open; `rogo binary stop` blocks on a reply v5 never sends;
  `make_robot()` fails *open* to a legacy `Cutebot` driver on missing config
  (`connection.py:314-319` — violates the fail-closed posture); `cmd_pose`
  hardcodes a 101.0&nbsp;cm field width against the real 134.3; a bare
  `except Exception: pass` sits in `nezha_state.py:256-259`'s drive loop (dead
  code, but the exact swallow pattern the halt rules ban).
- **CRUFT:** `nezha_state.py` (261 ln), `nezha_kinematic.py` (337 ln),
  `kinematics/differential_drive.py` — zero live consumers; `testkit/` empty husk;
  `_legacy_tlm_text.py` (all four consumers are themselves dead stacks);
  `cutebot.py` + v1/v2 lazy exports; `testgui/commands.py` (empty schema + dead
  builder); the TestGUI's internal command representation is still the retired text
  grammar re-translated per transport (a standing shim where typed calls would do);
  **166 CSV/PNG artifacts committed inside `src/tests/bench/`**;
  `square_tour_velocity.py` is a retirement stub that ImportErrors before printing
  its own notice; ~350 lines of unreachable v2 code below `rogo goto/turnto`'s
  NotImplementedError raises; `rogo`'s argparse still says "QBot Pro".

**Bench catalog:** 45 scripts — **31 CURRENT / 12 STALE / 2 retired stubs** (full
table in the host reviewer's report, preserved in the review discussion notes). The
STALE set is two whole families: the DEV-text-plane scripts
(`dev_exercise`, `pid_hold_speed`, `ratio_governor_curve`, `friction_rig_soak`,
`velocity_chart`, `comms_plane_verify`) and the rig family
(`rig_dev/drive/soak/stress`, `otos_drift`) — both wholesale deletable (the rig is
also quarantined by standing stakeholder rule), plus `velocity_step_response.py`
(one dead `proto.twist()` call from being CURRENT — worth reviving: it is the step-
response characterization tool G2 wants). Duplicated plumbing across the CURRENT
set: 8 copies of a PASS/FAIL recorder, ~20 of connect boilerplate, ≥13 hand-rolled
telemetry pumps, the `tlmOn`+boot-drain preamble copy-pasted with identical
comments — the new `src/tests/system/` recorder/runner is the right skeleton for
the one shared capture lib.

---

## Part 5 — Simulation stack and the path to a characterization-driven sim (G2)

F3 is the headline. Remaining findings:

- **S1. MAJOR — the measured-error→sim-config loop is dead code aimed at a deleted
  sim** (= H5): `fit_sim_error_model.py` (069) targets the retired SIMSET registry
  and a `src/sim/firmware.py` that no longer exists; zero live callers. Its
  `DEFAULT_CANDIDATE_KEYS` are a useful fossil — the old PhysicsWorld had
  *true-motion* knobs (`bodyRotScrub`, `motorOffsetL/R`) the current plants lack.
- **S2. MAJOR — the sim-vs-hardware validation loop is built but has never seen
  hardware:** `src/tests/system/` works end-to-end (tour files, JSONL recorder,
  signals, golden-compare with real callers, 47 datasets) — all sim-tier; no bench
  backend wired, no `goldens/` blessed. The three system-test issues charter
  exactly this; the rails are ready.
- **S3. MAJOR — Python config-path drift** (the exact class 130-002 killed in
  C++): `SimLoop.configure_from_robot()` pushes the JSON's stale `duty_per_speed`
  (`sim_loop.py:715-719`) while hardware runs the baked constant — folded into F7.
- **S4. MINOR:** no transport-loss knob in `FakeTransport` (defensible; one-class
  seam if wanted); OTOS never goes invalid in sim; DBG fault plane is
  sim-always/hardware-`--debug`-only (standing asymmetry to remember); determinism
  is excellent but seedless — dispersion studies (which the fit doctrine needs for
  noise sigmas) are impossible.
- **S5. CRUFT:** TestGUI Sim-Errors panel knobs with no setter (warn-and-skip);
  `src/sim/DESIGN.md` body is 118-era (40&nbsp;ms, retired TWIST surface) under a
  fresh 130-002 §2; three overlapping legacy square-tour drivers awaiting the
  chartered retirement.

**Composition root (130-002) — genuinely good:** one graph, both roots through
`App::composeRobot()`, four documented seams, field-by-field parity test, no
`#ifdef SIM` in `src/firm`. Its one flaw is that the period unification
*over*-unifies (F7): believed and delivered period are the same constexpr, which is
how the sim became unable to represent the real 54.

**Plant fidelity table (real behavior → sim):**

| Measured plant behavior | Modeled? | Where / gap |
|---|---|---|
| Actuation lag ~120–140&nbsp;ms | partial | first-order τ=0.13 hardcoded (`wheel_plant.h:44`); not per-wheel, not configurable |
| Deadband + copysign boost | **no (plant)** | compensation runs against a plant without the defect it corrects |
| Per-wheel gain asymmetry, breakaway, saturation knee | **no** (full sim); yes (planner-tier `NoisyPlant`) | the 130 controller's raison d'être is unrepresentable |
| Two-wheel load coupling / derating | **no** | wheels step independently (`sim_plant.cpp:249-250`); real data still unmeasured (wedge) |
| Delivered 54 vs believed 50&nbsp;ms | **no** | one constexpr (F7); seam exists only at motion_tests tier |
| Encoder 0.1° quantization, never-reset, software offset | yes | adequate |
| I2C wedge / disconnect / dropout | yes | best-covered area (two tiers, scriptable) |
| Radio 5–11% frame corruption | no | defensible omission |
| OTOS drift/scale | yes | reporting-side only |
| Session-to-session gain drift ±25% | **no** | no mechanism; needs config-driven plant (below) |

**The minimal path (work items, with existing seams):**
1. Parameterize `WheelPlant` (per-wheel gain/τ, breakaway band, saturation knee)
   from a pushed config; one new `sim_configure_wheel_plant()` ctypes export + a
   `sim_plant` section in the robot JSON via the existing
   `configure_from_robot()`. Numbers already exist in `tovez.json`'s 130-001 note
   (finish the right wheel first). Drop (or make opt-in) the exact-inverse
   feedforward.
2. Decouple delivered from believed period: settable step dt (default `kCycle`) +
   optional deterministic jitter sequence; `PlannerLimits.controlPeriod` stays the
   JSON value. Feed it the measured 54.0.
3. Add the true-motion error half: per-wheel achieved-gain error, load-coupled
   derating f(|dutyL|+|dutyR|), turn-scrub — the vocabulary `NoisyPlant` and the
   dead 069 PhysicsWorld both already established.
4. Retire `fit_sim_error_model.py`; re-implement "fit" as a `systest.py` subcommand
   over a hardware JSONL vs the same tour's sim JSONL (`signals.py` is already
   transport-symmetric), emitting a versioned error-model JSON the sim loads.
5. Wire the bench tier and bless one hardware square-tour dataset — simultaneously
   the first fit input and the first cross-tier golden.
6. Close S3 and fold all error-model application into `configure_from_robot()`
   (one injection surface for pytest, systest, TestGUI — the standing
   one-Sim-object rule), pruning the dead Sim-Errors knobs in the same pass.

**Characterization inventory (what the host can already measure vs what's missing):**
already measurable with CURRENT tooling — duty→speed population map incl. breakaway
and the both-wheels grid (`duty_sweep.py`), commanded→actual maps with hysteresis
(`speed_map/speed_sweep`), sub-breakaway crawl regime (`crawl_sweep`), first-order
gain/τ/dead-time fit (`plant_id`), encoder refresh cadence, move-level accuracy,
comms loss by bucket on both transports, whole-run datasets in three formats.
Missing for a complete error model: **the fit step itself** (item 4); actuation
latency as a *fitted, per-robot* parameter (known ~120–140&nbsp;ms, emitted
nowhere); loop-period jitter analysis (data exists in every capture, analysis
absent); encoder quantization as a propagated model parameter; a measured
battery-state protocol (the fresh/lowbatt CSVs were a one-off hand experiment — and
the no-unmeasured-power-narratives rule makes a protocol the only acceptable path);
the two-wheel derating grid (blocked on hardware); comms loss as a sim knob
(defensible to skip).

---

## Part 6 — What the hardware can actually do (the expectations decision)

Numbers are repo-measured (sources: 129-006/130-001/130-006/130-011 records,
committed CSVs, residual issues). This is the section that feeds the "dial back
expectations" conversation.

| Quantity | Measured reality |
|---|---|
| Plant gain | session-variable: 853.6/837.8 (07-31) → 1101.3/1023.1 fwd/rev L (08-01) → delivery 70–85% of commanded (08-02). **~25% swing in 2 days, unattributed** (no pack-voltage telemetry) |
| Saturation ceiling | 760–795/696 → 584 → **571.7/532.5&nbsp;mm/s** over the same window |
| Breakaway | duty 0.09–0.10 fwd; **0.164 right-reverse**; configured deadband 0.03 is 3–5× low |
| Minimum sustained speed | ~100&nbsp;mm/s (`vMin` 99.7, n=3 LOW CONFIDENCE); nothing below breakaway with `crawl_pulse=0` |
| Actuator resolution | int8 duty percent → **1 LSB ≈ 8–11&nbsp;mm/s**; write-rate limit 45&nbsp;ms; slew 25&nbsp;pct/write; latency ~120–140&nbsp;ms |
| Converged speed hold | mean within ~2&nbsp;mm/s of setpoint after 15–20&nbsp;s; **p2p ripple ~23&nbsp;mm/s per wheel** (open-loop map+bias) |
| Distance over a move | encoders good: 510/502&nbsp;mm on a 500&nbsp;mm command, 8&nbsp;mm split |
| Turns | ≤90°: ~2.7° worst in sim post-130-010; the 146° residual −10.1° (F10 suspect) |
| Control period | 54.000&nbsp;ms ± 0.006 idle / ±0.38 loaded (against nominal 50) |
| ESTOP settle | ~0.19&nbsp;s floor (one cycle + reissue + spin-down) |
| Two-wheel simultaneous derating | **NO DATA** — the grid was wiped out by the wedge; prior single point suggests real derating (104/68 vs 129 solo) |
| Bench availability | 4 hard I2C wedges in 2 days, each needing a physical power cycle; probeSlot silent |

**Specs the plant (as currently controlled) does not meet, with the reason:**

| Spec | Bound | Measured | Assessment |
|---|---|---|---|
| +500 rise to cruise | ≤0.3&nbsp;s | 0.59&nbsp;s | marginal *by design*: 120–140&nbsp;ms latency + 45&nbsp;ms write cadence + slew limit consume most of the budget; Stage B kp=0.3 hit 0.18&nbsp;s but worsened ripple — a real trade, not a tuning miss |
| Ripple through cruise | ≤±10&nbsp;mm/s | ±22–24&nbsp;mm/s | **below the actuator floor**: 1 duty LSB ≈ 8–11&nbsp;mm/s; the one Stage B trial made it worse (33.7), consistent with a quantizer limit cycle. This bound needs a decision (filtered/windowed criterion, or accept ~±25) |
| L/R split through cruise | ≤10&nbsp;mm/s | ~21&nbsp;mm/s | same floor argument per wheel; converged split is much better — this is a *transient* spec conflicting with cold-start bias (F5 makes every planner leg a cold start) |
| Settle epsilon | 4&nbsp;mm | ~5.4&nbsp;mm/tick at vMin | unreachable below ~100&nbsp;mm/s without a crawl mechanism; awaits the real actuation-floor measurement |
| Heading hold | stable | 1&nbsp;Hz limit cycle at gain 2.0 / 54&nbsp;ms | disabled (tovez only — togov/nocal still carry 2.0). Input problem, not gain: closes on encoder-only heading, a delayed function of the differential it commands |

**Proposed honest contract (discussion input, not a decision):** short open-loop
moves ±25% in speed until slope adaptation lands (F2); converged holds ±5&nbsp;mm/s
mean / ±25&nbsp;mm/s instantaneous; distance-stop moves ±2% + 10&nbsp;mm; turns
≤90° to ~3°, larger pending F10; nothing commanded below ~100&nbsp;mm/s sustained;
positioning resolution ~1&nbsp;cm until the actuation floor is measured; heading
hold off until re-founded on a fused heading. What makes these *acceptable* rather
than defeatist is the pairing: every ceiling above is either recoverable in
software (F2 slope adaptation, F10 latch release, crawl pulses) or attributable
with one new measurement (pack voltage, the simultaneous grid, the playfield
floor) — and G2's sim only needs the *characterized* numbers, not better ones.

---

## Part 7 — What's good

Worth saying plainly, because most of it is new since 128–130: the RobotState
ownership discipline held under three sprints of churn. The v5 wire (grammar
registry, CRC-then-COBS, bounded ack ring) is clean and well-tested. NezhaMotor's
write shaping and stop-reissue are careful, measured engineering. The Drive staging
order, anti-windup shape, NaN fail-closed, and uncalibrated-refusal are right. The
planner passes 8/8 standalone in ~2&nbsp;s with two includes to its name; the
lifecycle rewrite is a real state machine with behaviorally-pinned transitions;
`NoisyPlant` is the seed of exactly the G2 harness we need. The composition-root
unification is the cleanest ticket of the sprint and the parity test makes the
drift class it killed hard to reintroduce. The systest rails (tours/JSONL/goldens)
are production-quality and merely awaiting a hardware tier. The host halt story is
genuinely solid now — one shared `halt_now()` (bounded retry, loud, re-raising),
used by cli/repl/calibrate/geofence/MCP, acceptance-tested. 31 of 45 bench scripts
are current and several (duty_sweep's constant-free saturation anchor, wire_truth)
are exemplary measurement design. Sprint 130's own artifacts — honest
deferred-not-faked acceptance notes, the post-mortem's proxy-vs-observable
analysis — are the standard the review guidelines describe. And `src/firm` contains
zero TODO/HACK/FIXME markers: what debt exists is documented debt.

## Part 8 — Suggested discussion order (not a work order)

1. **The five-minute files:** create the missing F12 regression issue (and the
   missing IRQ-guard issue); fix the knowledge-doc falsehoods (rotation constants
   still present in `tovez_nocal.json`; heading-hold gain still 2.0 in
   togov/nocal); the bookkeeping items from the residuals synthesis (misfiled
   resolved issue, ticket-007 checkboxes, four broken issue cross-links).
2. **G1 firmware-solid sprint candidates:** F1 (epoch consumers) + F5
   (takeover/estop split, with C3) + F8 (stop hole + Stage B gates) + F7 period
   reconciliation + F6 observability (ConfigSnapshot, setpoint/duty on wire,
   deficit decoupling, I2C error surfacing, probeSlot timeout) + the schema bump +
   F9 test resurrection + the line-sensor decision. Everything here is
   bench-verifiable on the stand.
3. **G2 characterization sprint candidates:** Part 5's six work items + the
   deferred hardware measurements when the bench allows (right wheel, simultaneous
   grid, pack-voltage telemetry decision, playfield actuation floor) + reviving
   `velocity_step_response.py` on the v5 surface.
4. **Planner:** F10 (latch release + large-angle scenarios + TOUR_2 fingerprint
   run), P2–P6, separability items 1–2, the source-list collapse.
5. **Host:** F4 (repl + MCP push) and F11 (facade rebuild, one pose owner) first —
   they block every bench session; then H1–H4, the calibrate decision, tag-id
   fail-closed, testgui decomposition — sequenced against how much bench time each
   unlocks.
