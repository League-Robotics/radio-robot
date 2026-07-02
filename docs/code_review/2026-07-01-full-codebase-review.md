# Full-Codebase Correctness Review — 2026-07-01

**Author:** Claude (team-lead), at stakeholder request
**Scope:** Firmware motion/odometry/EKF pipeline, firmware command layer, simulation HAL + sim C-ABI + host sim bindings, host transport (serial/relay/sim), host odometry and TestGUI traces, TestGUI drive/STOP paths. Working-tree state of branch `sprint/063-…` (uncommitted changes included).
**Goals stated by stakeholder:** find serious issues that will cause problems; immediate interest is a reliably-running simulation; long-term goal is solid robot software. Known symptom history: encoder odometry "often wrong / goes in the wrong direction" (believed fixed), occasional "freakouts" where the robot heads off in a weird direction, and the simulation currently crashing.

**Method note:** subagent dispatch was unavailable for most of this session (harness classifier outage), so this is a single-reviewer deep pass over the highest-risk subsystems rather than a parallel fan-out. Files read in full are listed in §6; areas NOT deeply reviewed are listed there too and are candidates for a follow-up pass.

---

## 1. Executive summary

The core estimation firmware (Odometry, EKFTiny, MotionCommand/StopCondition, Planner) is in **good shape**: every motion phase now carries a TIME safety net (the wild-spin class is structurally closed), pose resets rebaseline the encoder snapshot correctly, angle math is wrap-safe throughout, and the corr-id reply demux on the host kills the stale-reply class of bugs. The sim transport threading (single tick-thread owning the Sim) is correct, and the ctypes ABI wrapper is disciplined.

The serious problems found cluster in four places:

1. **A confirmed process-abort in the stop-condition plumbing** (CR-01) — `D`/`T` commands with ≥2 `stop=`/`sensor=` clauses overflow `kMaxStopConds` and hit `assert(false)`, which **aborts the entire host process** in the sim build. This is the strongest candidate for "the simulation is crashing."
2. **Encoder-integrity holes** (CR-02, CR-03): the per-tick encoder read ignores I2C failures (a failed read manufactures a huge phantom jump), and the outlier filter that is supposed to absorb such jumps **lost its recovery path** in the sprint-060 cutover — one large baseline divergence (e.g. the robot repositioned by hand while idle) freezes encoder updates permanently. A frozen encoder means frozen heading, which is precisely the precondition for every historical "freakout."
3. **A safety-architecture regression** (CR-04, CR-05): the host now streams `+` keepalives unconditionally, and the firmware treats *any* inbound line as a watchdog feed — so the watchdog only protects against host-process death, not host-logic failure. Combined with the TestGUI sending STOP exactly once, fire-and-forget, over a link that drops 15–50 % of lines, a dropped STOP leaves the robot driving indefinitely.
4. **Sim-fidelity inversions** (CR-07, CR-08): the simulated OTOS re-integrates the *commanded wheel velocities* (the same signal the encoders see) instead of sampling plant ground truth, and models no lever arm — so the sim structurally cannot catch the two most important OTOS bug classes (encoder-vs-OTOS disagreement, lever-arm regressions like `db11b7c`).

Also notable: the 2026-06-17 change that fuses OTOS despite WARNING status bits re-opens the "spin on placement" failure mode that the D9 gate was built to prevent (CR-06).

---

## 2. Findings — ordered by severity

Finding IDs (CR-nn) are stable; use them when cutting issues.

### CR-01 · CRITICAL · `addStop` overflow aborts the process — D/T with extra stop clauses

[MotionCommand.cpp:53-61](../../source/commands/MotionCommand.cpp#L53-L61), [MotionCommands.cpp:759-781](../../source/commands/MotionCommands.cpp#L759-L781), [Superstructure.cpp:51-61](../../source/superstructure/Superstructure.cpp#L51-L61), [PlannerBegin.cpp:292-295](../../source/control/PlannerBegin.cpp#L292-L295)

`kMaxStopConds` is 4, and `MotionCommand::addStop` handles overflow with `assert(false && "addStop overflow")`. The D command path double-books stops: `beginDistance()` installs DISTANCE + TIME internally (2), then `Superstructure::requestGoal(DISTANCE)` **re-adds** `gr.stops[]`, which starts with a *duplicate* DISTANCE stop ([MotionCommands.cpp:759](../../source/commands/MotionCommands.cpp#L759)) plus any `stop=`/`sensor=` clauses. Totals: plain `D` = 3 (incl. one duplicate — benign but wrong); `D … stop=… sensor=…` (2 clauses) = 5 → assert fires. The T path is one clause closer to the edge (TIME internal + TIME duplicate + clauses).

The sim CMake build sets no `CMAKE_BUILD_TYPE`/`NDEBUG`, so `assert` is live: the abort kills the **whole Python process** (pytest run or TestGUI — this looks exactly like "the simulation crashed"). On real firmware the CODAL assert path panics the micro:bit mid-drive.

**Fix direction:** stop double-adding (either `requestGoal` should not re-add the goal's own primary stop, or `begin*` should not install stops that the caller will supply); make `addStop` overflow return `ERR` to the host instead of asserting; add a regression test sending `D 150 150 300 stop=time:9000 sensor=line0>500`.

### CR-02 · HIGH · Encoder outlier filter has no recovery path — one big jump freezes odometry forever

[Drive.cpp:394-444](../../source/subsystems/drive/Drive.cpp#L394-L444), [Drive.h:148](../../source/subsystems/drive/Drive.h#L148)

`_runOutlierFilter` rejects any per-tick encoder delta > `max(40 mm, 0.2·target)` and holds the previous value; the retry re-reads are accepted **only if they land within the threshold of the same stale baseline**. `_filterRejectStreakL/R` are incremented, and `kFilterRejectStreakThreshold = 3` is still declared in the header — but **nothing consumes the streak**: the legacy streak-based rebaseline was lost in the 060 ordered-tick cutover. Additionally the filter body runs only `if (driving)`, so `_hw.encMm[]` is never updated while idle.

**Failure scenario (freakout-shaped):** operator lifts/rolls the robot to a new spot while idle (wheels rotate > 40 mm) → next `VW`/`TURN`/`G` starts, every fresh read is > 40 mm from the stale baseline → rejected forever → `encMm` frozen → `Odometry::predict` sees zero deltas → heading/pose frozen → TURN/G spin at commanded ω until their TIME net expires (for TURN that is 2×nominal + 2 s of full-rate spinning; the wedge detector may also fire and *hold* heading, which does not help). Only a `D` command escapes, because `distanceDrive` calls `resetEncoders()`. This mechanism plausibly explains both residual "encoders wrong/ignore turns" reports and on-placement freakouts.

**Fix direction:** restore the streak recovery (after N consecutive rejections, rebaseline `_hw.encMm` to the fresh reading); run a baseline refresh (without integration) when idle or on the idle→driving transition; add a sim test that jumps the plant encoder while idle and asserts odometry recovers.

### CR-03 · HIGH · Encoder I2C reads ignore failure — a failed read manufactures a phantom jump

[Motor.cpp:305-325](../../source/hal/real/Motor.cpp#L305-L325) (`collectEncoder`), [Motor.cpp:336-354](../../source/hal/real/Motor.cpp#L336-L354) (`readEncoderMmFSettle`, the per-tick path), [Motor.cpp:215-263](../../source/hal/real/Motor.cpp#L215-L263) (`readEncoderAtomic`)

All three encoder read paths ignore the return codes of `_i2c.read()`/`write()` — on failure the response buffer stays `{0,0,0,0}`, so the "position" becomes `0 − _encOffset`, i.e. a jump to a large arbitrary value for that tick. Contrast `readSpeedRaw` ([Motor.cpp:462-478](../../source/hal/real/Motor.cpp#L462-L478)) which checks both and returns a sentinel. Downstream consequences: (a) while driving, the outlier filter absorbs a *single* bad tick — but see CR-02 for what happens on sustained failure; (b) `resetEncoder()`'s median-of-3 uses `readEncoderAtomic` — **three failing reads produce a confidently-wrong offset** (median of garbage); the readback check helps only if reads start succeeding again; (c) the `EVT ROTSTOP` diagnostic in [MotionCommand.cpp:163-179](../../source/commands/MotionCommand.cpp#L163-L179) exists precisely because garbage reads have been observed corrupting turn baselines — this is the untreated source.

**Fix direction:** check return codes; on failure return a "no reading" status and have callers hold the last value (and count failures for telemetry/wedge logic) rather than fabricating `−offset`.

### CR-04 · HIGH · Dropped STOP + unconditional keepalive = unbounded manual-drive runaway

[drive.py:279-288](../../host/robot_radio/testgui/drive.py#L279-L288), [serial_conn.py:682-711](../../host/robot_radio/io/serial_conn.py#L682-L711), [PlannerBegin.cpp:167-178](../../source/control/PlannerBegin.cpp#L167-L178)

The TestGUI KeyboardDriver drives with `VW` (open-ended; **no TIME stop** by design — keepalive-bound) and on key release sends `STOP` **once, fire-and-forget** (`Transport.send` → `send_fast`, no ack check, no retry). Direct USB drops 15–50 % of lines intermittently. If the STOP line is dropped: the VW-resend timer stops, but the `SerialConnection` keepalive daemon keeps streaming `+` every ~150 ms for as long as the port is open — and the firmware watchdog resets on *any* inbound line. Result: the robot continues at the last commanded velocity **indefinitely** (also reachable via window focus loss, which suppresses the keyRelease event entirely). This is a high-probability, recurring "freakout": ~1 in 2–7 key releases on a bad USB day.

**Fix direction (pick at least one, ideally two layers):**
- GUI: send STOP via the acked `command()` path and re-send until an `OK` arrives (STOP is idempotent).
- GUI: treat the resend timer as a deadman — after key release, send STOP on the next N timer ticks instead of stopping the timer immediately.
- Architecture (see CR-05): stop letting the connection-level `+` feed the motion watchdog.

### CR-05 · HIGH (architecture) · The watchdog can no longer catch anything but host death

[serial_conn.py:86-87,662-711](../../host/robot_radio/io/serial_conn.py#L86-L87), [Superstructure.cpp:122-158](../../source/superstructure/Superstructure.cpp#L122-L158)

The June wild-spin postmortem's killer combination was "runaway motion × watchdog silenced by keepalives." The TIME nets fixed the *bounded* verbs, but open-ended motion (`S`/`VW`/`R`) is still guarded **only** by the keepalive watchdog — and the host now feeds that watchdog automatically from a connection-level daemon that runs whenever the port is open, regardless of whether any host logic is attending to the robot. Every host program using `SerialConnection` (CLI, TestGUI, bench scripts) therefore silently disables the last safety layer for open-ended motion: a hung host script with an open port = a robot that never stops.

**Fix direction:** make motion-keepalive intentional rather than ambient. Options: firmware distinguishes watchdog-feeding lines (only `+` or motion commands reset the *motion* watchdog, and the host sends `+` only while a motion source is actively driving); or the host keepalive daemon is armed/disarmed by the layer that owns motion (KeyboardDriver, tour worker, bench script) instead of by `connect()`. Also consider a firmware-side max-duration cap on `VW` (e.g. auto soft-stop after N seconds without a *fresh* VW, independent of `+`).

### CR-06 · HIGH · OTOS fused despite WARNING bits — "spin on placement" regression re-opened

[Robot.cpp:200-267](../../source/robot/Robot.cpp#L200-L267), [EKFTiny.cpp:217-250](../../source/state/EKFTiny.cpp#L217-L250), [EKFTiny.cpp:420-437](../../source/state/EKFTiny.cpp#L420-L437)

The two-tier gate comment (READABLE vs HEALTHY, D9/027-005) says fusion requires `otosStatus == 0`; the code below it now sets `healthy = poseOk` (change dated 2026-06-17), so a robot with `warnOpticalTracking` persistently set (lifted, on the stand, or freshly placed) has its **frozen** OTOS pose and near-zero velocity fused into the EKF. The Mahalanobis gates reject the frozen pose only temporarily: the gate-recovery paths (`kRebaselineP`, `kRebaselinePTheta`) force-snap position and heading to the OTOS observation after 10 consecutive rejections. Net effect: hold the robot in the air while the wheels spin, or carry it to a new spot, and within ~10 OTOS samples the fused pose/heading snaps to stale garbage — the exact precondition D9 was written to prevent ("passing zero velocity … drags fused velocity to zero and fights the controller — root cause of the spin-on-placement symptom"). The 06-17 rationale (transient warn bits shouldn't drop fusion) is legitimate; the implementation just lost the distinction between *transient* and *persistent*.

**Fix direction:** gate fusion on warn-bit persistence — e.g. fuse through ≤ K consecutive warn samples, block after; or block fusion whenever `warnOpticalTracking` has been continuously set for > ~250 ms. Keep telemetry visibility as-is.

### CR-07 · MEDIUM · Sim OTOS integrates commanded wheel speeds, not plant truth

[SimHardware.cpp:63-65](../../source/hal/sim/SimHardware.cpp#L63-L65), [SimOdometer.cpp:87-127](../../source/hal/sim/SimOdometer.cpp#L87-L127), [PhysicsWorld.cpp:84-101](../../source/hal/sim/PhysicsWorld.cpp#L84-L101)

`SimOdometer::tick(velL, velR, tw, dt)` re-runs the same differential-kinematics integration the encoders/odometry use, fed by the same wheel velocities. The real OTOS is an *independent ground-truth-tracking* sensor. Two consequences: (a) the sim OTOS can never disagree with the encoders except by injected noise, so EKF-fusion tests validate a regime that doesn't exist on hardware (where OTOS≈truth ≠ encoders under scrub); (b) with slip configured, the plant heading applies `effectiveSlip` ([PhysicsWorld.cpp:95-96](../../source/hal/sim/PhysicsWorld.cpp#L95-L96)) but the sim-OTOS heading integration applies none — the sim OTOS diverges from sim ground truth in exactly the way the real OTOS does *not*. The right model is one line: sample `plant.truePose*()` (+ noise/drift/quantization).

### CR-08 · MEDIUM · Sim OTOS models no lever arm — the host-side compensation path is never exercised

[SimOdometer.cpp:16-32](../../source/hal/sim/SimOdometer.cpp#L16-L32) vs [OtosSensor.cpp:122-148](../../source/hal/real/OtosSensor.cpp#L122-L148)

The real driver must subtract `R(hF)·odomOff` because the chip's offset register is unwritable (a past regression here, `db11b7c`, produced 433 mm of phantom translation on a pure spin). The sim odometer reports the robot centre directly, so that entire code path — the one with the worst historical failure — has zero sim coverage. Model the sensor at `odomOffX/Y` in the sim and run the same `readTransformed` compensation, then a "pure spin → no translation" sim test becomes possible (and should be added).

### CR-09 · MEDIUM · TestGUI encoder-trace reset heuristic misses resets on slow TLM links

[traces.py:88-95](../../host/robot_radio/testgui/traces.py#L88-L95), [traces.py:334-350](../../host/robot_radio/testgui/traces.py#L334-L350)

The (new, uncommitted) reset detector recognizes a firmware encoder zeroing only when the incoming frame reads **within 20 mm of zero on both wheels**. Over the relay, TLM arrives at ~1–2 Hz while the robot moves 100–200 mm between frames — the first post-reset frame will often already exceed 20 mm, so the reset is missed and integrated as spurious reverse motion whose `(dR−dL)` cancels the just-turned heading: the *original* "ignores turns / drifts into a corner" bug survives on exactly the transport (relay/playfield mode) where it matters. A genuine return-through-zero can also false-positive. [tests/testgui/test_traces.py:176-231](../../tests/testgui/test_traces.py#L176-L231) covers only the prompt-reset case.

**Fix direction:** stop inferring resets from data. The GUI knows when it (or the tour) issues `D`/`ZERO enc` — rebaseline on the command boundary; or better, have firmware include a reset counter/epoch in the TLM `enc=` field so any consumer can rebaseline exactly.

### CR-10 · MEDIUM · otos/fused traces rotate world-frame deltas as if firmware frame were zeroed at anchor

[traces.py:371-403](../../host/robot_radio/testgui/traces.py#L371-L403)

`_feed_otos`/`_feed_fused` subtract a baseline (translation handled) and then rotate the *firmware-world-frame* delta by the **anchor yaw** via `_tw()`. That is correct only when the firmware pose frame was freshly re-referenced (heading 0) at anchor time ("Set Robot @ 0,0"). Anchoring mid-session with a non-zero firmware heading leaves those two traces rotated by the firmware heading at baseline relative to the camera trace. Rotate by `(anchor_yaw − firmware_heading_at_baseline)` instead (the baseline tuples already carry `hdg_cdeg`, unused).

### CR-11 · MEDIUM · `Planner::apply()` passes `now = 0` into `begin*()` — a landmine for the message-architecture path

[Planner.cpp:368-380](../../source/superstructure/Planner.cpp#L368-L380), [MotionCommand.cpp:98-116](../../source/commands/MotionCommand.cpp#L98-L116), [StopCondition.cpp:124-130](../../source/control/StopCondition.cpp#L124-L130)

`apply()` hard-codes `now = 0`, which becomes `MotionBaseline.t0Ms = 0`; every TIME stop then computes `elapsed = now_ms − 0` = full uptime, so any timed motion started through this path terminates instantly (reason=time) once uptime exceeds the timeout. Currently unreachable — BusDrain's PLANNER verb is an explicit no-op placeholder ([BusDrain.cpp:73-83](../../source/robot/BusDrain.cpp#L73-L83)) and live traffic flows through `requestGoal` with real timestamps — but whoever finishes the PlannerCommand encoding will trip this immediately and mysteriously. Fix now (thread a real timestamp or defer baseline capture to first `tick()`), or at minimum leave a loud comment + a failing-by-construction test.

### CR-12 · MEDIUM · `OdomTracker` world transform is an untested convention stack

[odom_tracker.py:277-310](../../host/robot_radio/sensors/odom_tracker.py#L277-L310)

`_to_world_mm` treats TLM pose x/y as "x=right, y=forward" and composes an axis swap with a **CW-positive** world-yaw convention; firmware pose is world-frame with heading 0 = +X, CCW-positive. The composition is a proper rotation (no mirroring), but whether the resulting angle offset matches the camera world frame is exactly the class of "guessed geometry" that produced past incidents (`camera_yaw + 90`), and there is no test anchoring it to the aprilcam convention (A1-centred, +x east, +y north). If any navigation consumer picks this up (it is exported from `robot_radio.sensors`), a convention error sends the robot off in a "weird direction." Add a convention test: anchor at a known camera pose, feed a synthetic straight-ahead TLM track, assert the world track matches the camera's expectation.

### CR-13 · MEDIUM · Sim C-ABI global clock and non-thread-safety

[sim_api.cpp:39-44](../../tests/_infra/sim/sim_api.cpp#L39-L44), [sim_api.cpp:187-195](../../tests/_infra/sim/sim_api.cpp#L187-L195)

`g_sim_now_ms` is a process-global: `sim_create()` resets it to 0, so creating a second SimHandle (GUI reconnect racing a slow-exiting tick thread — `disconnect()` gives up joining after 3 s and `connect()` will happily start a new Sim) yanks the clock backwards for the still-live instance, corrupting watchdog/TIME-stop deltas. The shared `replyStore` is also unsynchronized; today only the SimTransport tick-thread discipline protects it, and nothing enforces that for other embedders (`sim_conn.py` is documented single-thread but unchecked). Low-cost hardening: move the clock into SimHandle; document/assert single-thread usage.

### CR-14 · MEDIUM · `SimConnection._raw_command` truncates replies at 512 bytes

[sim_conn.py:333-337](../../host/robot_radio/io/sim_conn.py#L333-L337)

The reply store is 2048 bytes and `firmware.py` uses 2048 ("so multi-line replies, e.g. chunked GET CFG, are not truncated") but `SimConnection` passes a 512-byte buffer — long replies are silently cut. Harmless for OK/ERR, wrong for `GET CFG`-style output.

### CR-15 · LOW · Assorted smaller items

- **`PhysicsWorld` true heading is never wrapped** — `_truePoseH += dTh` unbounded ([PhysicsWorld.cpp:100](../../source/hal/sim/PhysicsWorld.cpp#L100)); consumers comparing headings must wrap; SimOdometer wraps its own copy, creating an avoidable representation mismatch.
- **`probe_devices()` still speaks the retired `>PING` relay-prefix protocol** ([serial_conn.py:918-946](../../host/robot_radio/io/serial_conn.py#L918-L946)) — cannot reach a robot through current relay firmware; misleading if anything still calls it.
- **`relay_info` collected but dropped** in `connect()` ([serial_conn.py:334,352](../../host/robot_radio/io/serial_conn.py#L334)) — the operator-visible channel/group mismatch logging the docstring promises never reaches the result dict.
- **`SimTransport.connect()` sets `_connected = True` before the tick thread has successfully created the Sim** ([transport.py:644-652](../../host/robot_radio/testgui/transport.py#L644-L652)) — early `command()` calls block their full timeout and return "" if lib load fails.
- **traces encoder integration uses post-increment heading** (not midpoint) ([traces.py:361-363](../../host/robot_radio/testgui/traces.py#L361-L363)) — small systematic display drift on turns; fine for a display, worth a comment.
- **Duplicate DISTANCE stop** on every queued `D` (see CR-01 mechanics) — benign today, but it wastes one of only four stop slots.
- **`rgbToHSV` lives in StopCondition.cpp** (existing `FIXME` at [StopCondition.cpp:27](../../source/control/StopCondition.cpp#L27)) — misplaced; move to a color utility.
- **KeyboardDriver multi-key release** — releasing one arrow while another is held sends STOP and drops the held command ([drive.py:263-288](../../host/robot_radio/testgui/drive.py#L263-L288)); minor UX, but confusing on the playfield.

---

## 3. What is in good shape (verified, not assumed)

- **Runaway protection on bounded verbs:** TURN, RT, D, G-PRE_ROTATE, and G-PURSUE all carry TIME nets sized 2×nominal + margin ([PlannerBegin.cpp:466-476](../../source/control/PlannerBegin.cpp#L466-L476), [537-542](../../source/control/PlannerBegin.cpp#L537-L542), [Planner.cpp:256-283](../../source/superstructure/Planner.cpp#L256-L283)); PURSUE has a backtrack re-gate and a curvature clamp; RT stops on encoder arc so it works even with a broken heading.
- **Pose-reset hygiene:** `setPose` rebaselines `_prevEncL/R` to current encoders and resets the encoder-only accumulator ([Odometry.cpp:182-216](../../source/control/Odometry.cpp#L182-L216)); `resetEncoders()` is atomic across hardware, MC baselines, outlier baseline, odometry snapshot, and Drive's private copy ([Robot.cpp:326-345](../../source/robot/Robot.cpp#L326-L345)); the `beginDistance` baseline race (033-004) is fixed at the source.
- **Angle math:** wrap-safe innovation/blending everywhere I looked (`wrapPi` via `atan2f(sin,cos)`, wrap-safe HEADING stop, wrap-safe EKF heading update).
- **Watchdog arithmetic:** the uint32-underflow class is systematically handled with signed casts (Odometry dt, TIME stop, Superstructure watchdog).
- **Host reply correlation:** per-command corr-id queues mean a late/duplicate/stale reply cannot ack a different command ([serial_conn.py:585-599, 745-797](../../host/robot_radio/io/serial_conn.py#L585-L599)); retry fires only on `ERR unknown` (proof of non-execution), never on a dropped OK.
- **Wire-format consistency:** TLM `enc=` is L,R end-to-end (emitter [RobotTelemetry.cpp:82](../../source/robot/RobotTelemetry.cpp#L82) sends `ds.enc()[1]` = L first; parser stores `(left, right)`; traces treat `enc[0]` as L). VW omega is mrad/s on the wire, range-checked ±3142, converted once.
- **SimTransport threading:** command queue + single tick-thread exclusively owning the Sim object is the right pattern; plant teleports stop motion state correctly via queued actions.
- **`firmware.py` ctypes wrapper:** every function used has explicit `argtypes`/`restype` (no pointer-truncation crash lurking there).
- **Encoder-wedge defenses:** write-on-change + 40 ms write throttling + stop/reversal exemptions in Motor.cpp match the documented root cause; the wedge detector gates phantom omega out of the EKF.

---

## 4. Test-coverage gaps (against the bug classes that matter here)

| Bug class | Covered? | Notes |
|---|---|---|
| Runaway/unbounded motion | Largely yes | `test_goto_bounds.py`, `test_incident_scenarios.py`, TIME-net tests. Keep. |
| Heading reset (SI without OZ) | Yes (new) | `test_sim_otos_heading_reset.py` (063-006). |
| Outlier-filter recovery after idle-time encoder jump | **No** | CR-02. No test moves the plant encoders while idle and asserts recovery. |
| I2C encoder read failure | **No** | CR-03. Sim has `sim_set_otos_read_failure` for OTOS but no encoder-read-failure injection; SimMotor `setFrozen` freezes (wedge case), which is a different failure. |
| Stop-clause overflow (CR-01) | **No** | One-line test would have caught the abort. |
| Encoder-reset rebaseline with delayed TLM | **No** | CR-09. `test_traces.py` covers only the prompt-reset frame. |
| Per-wheel `fwdSign` regression | **Structurally untestable in sim** | SimMotor/PhysicsWorld don't model `fwdSign` at all — real-Motor sign config has no sim coverage by construction. Needs a bench check or a HAL-level sign contract test. |
| OTOS lever-arm compensation (pure spin → no translation) | **Structurally untestable in sim** | CR-08. Becomes testable once the sim models the sensor offset. |
| Warn-bit fusion gating | **No** | CR-06. Sim OTOS has `setLift` (returns read-failure) but no "warn-bit-set-but-readable" state, so the regressed two-tier gate is invisible to tests. |

Infrastructure: `tests/conftest.py` does run `cmake --build` per session (good), but it inherits the known silent-staleness hazard of incremental builds on `/Volumes` — consider a content-hash check or `--clean` option for CI parity.

---

## 5. Recommended issue breakdown (priority order)

1. **CR-01** — stop-clause overflow abort (sim crash candidate #1; also firmware panic risk). Small, well-bounded fix + regression test.
2. **CR-02 + CR-03 together** — encoder integrity: check I2C return codes, restore streak-based filter recovery, idle rebaseline; add the two missing sim tests. This is the best candidate for the residual "encoders wrong / robot freaks out" reports.
3. **CR-04 + CR-05** — STOP delivery reliability + keepalive/watchdog architecture. Decide the ownership model for motion keepalives; make the GUI STOP acked/repeated.
4. **CR-06** — warn-bit fusion persistence gate (spin-on-placement regression).
5. **CR-07 + CR-08** — sim OTOS fidelity (sample plant truth; model lever arm) + the pure-spin sim test. Directly serves the "accurate simulation" goal.
6. **CR-09 + CR-10** — TestGUI trace correctness on slow links / mid-session anchors (relay/playfield usability).
7. **CR-11, CR-12, CR-13, CR-14** — landmine cleanups before the message-architecture work resumes.
8. **CR-15** — batch of small cleanups; fold into a maintenance ticket.

## 6. Review coverage

**Read in full / near-full:** `source/control/Odometry.{h,cpp}`, `EKFTiny.cpp`, `StopCondition.cpp`, `PlannerBegin.cpp`, `superstructure/Planner.cpp`, `Superstructure.cpp`, `commands/MotionCommand.cpp`, `MotionCommands.cpp` (D/T/VW paths), `robot/Robot.cpp` (otosCorrect/resetEncoders/distanceDrive), `robot/BusDrain.cpp` (drain switch), `hal/real/Motor.cpp`, `hal/real/OtosSensor.cpp` (transform paths), `subsystems/drive/Drive.{h,cpp}`, `hal/sim/*` (PhysicsWorld, SimOdometer, SimHardware), `tests/_infra/sim/sim_api.cpp`, `tests/_infra/sim/firmware.py`, `host/robot_radio/io/serial_conn.py`, `io/sim_conn.py`, `sensors/odom_tracker.py`, `testgui/transport.py`, `testgui/traces.py`, `testgui/drive.py`, `testgui/operations.py` (top), `robot/RobotTelemetry.cpp` (TLM emit).

**Not deeply reviewed (follow-up candidates):** `MotorController.cpp` PID internals (wedge/windup structure spot-checked only), `BodyVelocityController`, `CommandProcessor` tokenization, `LoopTickOnce/LoopScheduler` ordering, `io/cli.py` (1783 lines), `nav/navigator.py`, `nav/camera_goto.py`, `kinematics/`, `field/playfield.py`, `testgui/__main__.py` beyond the STOP/tour wiring (four open issues already cover its worst threading defects), `media/`, `calibration/`. The four filed testgui issues (`clasi/issues/testgui-*.md`) were reviewed for plausibility and look sound; they are not re-reported here.
