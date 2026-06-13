# Bench 032 findings — root-cause diagnosis (code analysis)

Analysis of the three bench-032 findings, traced through the firmware source.
Companion to `.clasi/issues/fr-bench-dbg-otos-no-reply.md`,
`fr-bench-twist-fusedv-zero.md`, `fr-bench-right-encoder-wedge.md`.

---

## STAKEHOLDER DIRECTIVE — bench testing uses the SERIAL PORT, not the radio

**Do not run bench tests over the radio relay. The robot is on the stand,
physically next to you, with a USB cable available — the whole point of the
bench is that you are hooked up to the serial port. Use the serial port.**

This is not a style preference; it is the root of most of this session's
confusion:

- The DBG subsystem deliberately replies on serial (`ForceReply::SERIAL`).
  Run the bench over USB serial and every DBG command replies exactly as
  designed — finding 2's "no reply" symptom does not exist on the correct
  transport.
- The radio relay adds an unreliable hop (the data plane drops async output,
  and the `DBG OTOS BENCH 1` enable provably never took effect on the robot)
  on top of a transport the DBG commands were never meant to answer on.
- Driving the bench through the relay means you cannot distinguish "firmware
  bug" from "relay ate it." Over USB serial that ambiguity disappears.

Rewrite the bench harness (`tests/bench/bench_validation_032.py`) to open the
robot's USB serial port directly. Reserve relay/radio testing for what it
actually validates: the radio link itself.

---

## Finding 2 — `DBG OTOS BENCH` / `DBG OTOS` silent on hardware

**Root cause (high confidence): `ForceReply::SERIAL` routes every matched DBG
reply to the robot's local USB serial; the bench harness listens over the radio
relay and can never see them. The reply path is not "broken" — it is pointed at
a port nobody is reading.**

Chain:

1. Every `DebugCommandable` descriptor is registered with `ForceReply::SERIAL`
   (`DebugCommandable.cpp:694-704`; stated as a design rule in
   `DebugCommandable.h:23-24` — "debug output always goes to the serial port
   regardless of which channel the command arrived on").
2. On hardware, `main.cpp:207/216` wires `cmd.setSerialReply(serialReply,
   &comm.serial())`. In `CommandProcessor::dispatchTable`
   (`CommandProcessor.cpp:107-110`), any matched descriptor with
   `ForceReply::SERIAL` swaps the reply fn to that serial sink — for the OK/ERR
   reply *and* for parse-error (`badarg`) and queue-full (`full`) replies.
3. The 032 harness (`tests/bench/bench_validation_032.py`) talks through the
   relay's radio data plane. Replies sent to the robot's own USB serial never
   reach it.
4. The one reply that *was* observed — `DBG` alone → `ERR unknown` — is emitted
   at `CommandProcessor.cpp:96-100`, *before* the ForceReply override, on the
   originating (radio) channel. That asymmetry exactly reproduces the observed
   signature: unmatched lines reply, matched DBG lines are silent.
5. Sim is green because the host harness never calls `setSerialReply`;
   `_serialFn == nullptr` skips the override (`CommandProcessor.cpp:107`) and
   replies return on the test channel. Textbook sim-green/hardware-dark.

**Corollary:** if the line reaches the robot, the handler *executes* — only the
reply is invisible. So `DBG OTOS BENCH 1` may silently toggle bench mode. (But
see the cross-check below: the 032 telemetry indicates bench mode was NOT
active during the drives, so the enable line most likely never reached the
robot's dispatcher — relay-side drop is the remaining suspect. The serial probe
below discriminates.)

**Verification (do first):** connect USB serial directly to the robot, keep the
relay link open, send `DBG OTOS BENCH 1` over radio. Expect `OK dbg otos
bench=1` on USB and silence on radio. That confirms both the routing diagnosis
and whether the command arrives at all.

**Fix:** per the stakeholder directive above, the correct fix is to run the
bench harness on the robot's USB serial port, where these replies already go.
The firmware is behaving as designed; the harness used the wrong transport.
Do NOT change `ForceReply` to chase radio replies for bench work. (If remote
radio access to these commands ever becomes a real requirement, that's a
separate decision — and note `handleDbgOtos` emits a ~150-char pose line
(`pose_buf[200]`, `DebugCommandable.cpp:520-527`) that may exceed radio
payload limits.)

---

## Finding 3 — `twist=` reads 0,0 while driving

**Root cause (high confidence): the EKF velocity states are only ever written
by `updateVelocity()`, and `updateVelocity()` is only reachable through
`Robot::otosCorrect()` — downstream of the OTOS validity gates. On the bench
stand the real OTOS is lifted/invalid, `otosCorrect()` early-returns every
tick, and the encoder-derived velocity is never fused. v/omega stay at their
init value of 0 forever.**

Chain:

1. `twist=` emits `state.inputs.fusedV/fusedOmega` (`Robot.cpp:509-516`),
   written from `_ekf.v()/_ekf.omega()` in `Odometry::predict()`
   (`Odometry.cpp:73-74`).
2. The EKF predict step is a random walk for the velocity block — it inflates
   covariance but never moves the mean (`EKF.h:13-14`, `EKF.cpp:204-208`). The
   only writer of `_x[3]/_x[4]` is `updateVelocity()`.
3. `updateVelocity()` is called solely from `Odometry::correctEKF()`
   (`Odometry.cpp:203-206`), called solely from `Robot::otosCorrect()`
   (`Robot.cpp:285-288`) — *after* the gates: `is_initialized` (197), STATUS
   byte / `lastReadOk` (218, the D9 lifted-robot gate), same-tick
   `readTransformed` failure (255).
4. On the stand the lifted OTOS reports tracking-invalid status (that is the
   D9 gate's design case) — `otosCorrect` returns at line 236 on every 100 ms
   tick. No update is ever *attempted*, which also explains `ekf_rej=0`:
   rejections are counted inside the update functions (`EKF.cpp:426,463`), and
   none ran.
5. Sim is green because the sim/bench OTOS is always valid, so the velocity
   updates always run.

**The architectural bug:** encoder-velocity fusion (`enc_v`/`enc_omega`,
computed in `predict()` independent of OTOS) is needlessly nested inside the
OTOS-gated path. Encoder velocity is available every tick regardless of OTOS
health.

**Fix:** fuse encoder velocity unconditionally (e.g. its own
`_ekf.updateVelocity(enc_v, enc_omega, _rEncV, _rEncV)` call in the predict
phase or an ungated step in the OTOS block), keeping OTOS pose/heading/velocity
fusion behind the validity gates. Caveat from Finding 1: gate the *enc omega*
observation on both encoders being healthy, or a wedged wheel injects phantom
omega into the fused state (see below).

**Cross-check that ties findings 2+3 together:** if `DBG OTOS BENCH 1` had
actually executed during the run, the bench sensor passes every gate
(`BenchOtosSensor` always-valid by design), fusion would have run every 100 ms,
and twist/ekf_rej could not both have stayed 0 while pose ran to a 131°
phantom heading. So the run provably executed with bench mode OFF — i.e. the
enable command did not take effect, not merely its reply lost. After fixing
the reply routing, re-verify the enable actually lands (relay drop is the
open suspect).

---

## Finding 1 REVISITED — the wedge EVT cannot be trusted as evidence of an encoder fault

Update after the equal-wheel balance retest (encR/encL = 0.87–1.00 across 12
drives, "R under-counts" flags traced to a harness guard bug): the encoders
are healthy. So why did `EVT enc_wedged wheel=R enc=0 n=10` fire, and why did
TLM show `enc=22,0` and `enc=736,57`?

### What the detector actually is

`MotorController::controlTick` (`MotorController.cpp:246-343`): per wheel,
every control tick (~24 ms, both encoders read every tick), if the commanded
target is non-zero and the encoder mm value is float-identical to the previous
tick's, increment a stuck counter; at 10 consecutive identical reads (~240 ms)
emit `EVT enc_wedged` once (latched); re-arm on any change. Resolution is
0.1° of wheel (≈0.06 mm), so "identical for 240 ms" really does mean "measured
position not advancing." The logic is internally sound.

### The problem: it watches the FILTERED value, so it conflates three causes

The value it compares, `state.inputs.encLMm/RMm`, is written by the
speed-scaled outlier filter in `Robot::controlCollectSplitPhase`
(`Robot.cpp:114-163`): any read whose delta from the *stored* value exceeds
`max(40 mm, tgt×0.2)` is rejected (2 silent retries) and the old value is
**held**. Therefore `EVT enc_wedged` fires for any of:

1. **True chip/I2C readback wedge** — the fault it was built for (015-003).
2. **Wheel physically not turning** — stall, battery droop, slow spin-up. The
   detector arms the moment `tgt != 0`; a drained motor that takes >240 ms to
   move 0.06 mm trips it. (The retest's own observation — "barely moved at
   speed 200 after hours of driving" — puts the bench run squarely in this
   regime.)
3. **Outlier-filter hold** — if the stored value and the chip's count diverge
   by more than the gate, *every* subsequent read is rejected and the stored
   value freezes. The filter has no telemetry; a filter-freeze is
   indistinguishable from a wedge in both the EVT and the `enc=` TLM field.

### Cause 3 has a specific, likely trigger: ZERO enc offset corruption

`Motor::resetEncoder()` (`Motor.cpp:131-141`) is a *software* reset:
`_encOffset += readEncoderAtomic()`. If that one atomic read returns garbage —
a documented chip behavior (`Robot.cpp:123-125`: "the chip still occasionally
returns ~149 mm garbage reads") — the offset is silently corrupted by the
garbage amount X. `Robot::resetEncoders()` then sets the filter baseline to 0
with no read-back verification. Every subsequent read returns
`raw − corrupted offset` ≈ −X away from the baseline → outlier gate rejects →
`encRMm` freezes at 0 → wedge EVT fires with `enc=0`, and the frozen value
only unfreezes once the wheel has physically traveled ~X mm back inside the
gate — after which the wheel *tracks correctly but lags X behind*.

That reproduces the entire bench-032 signature — `enc=22,0` (R stuck at 0
while L counts), `EVT enc_wedged wheel=R enc=0 n=10`, and `enc=736,57`
(R apparently under-counting by a ~constant offset) — with zero hardware
fault. It is also sim-invisible (the mock chip never returns garbage), i.e.
exactly the class the bench exists to catch, just one layer down from where
the issue filed it.

### Verdict and fixes

Trust `EVT enc_wedged` as "measured count not advancing while commanded."
Do NOT trust it as "encoder hardware fault" — it cannot make that distinction
as built. To make it diagnostic:

- **Verify ZERO enc took:** after `resetEncoder()`, read back and require
  |value| ≈ 0; retry the snapshot on failure. Cheap and kills the offset-
  corruption path. Consider median-of-3 for the offset snapshot read.
- **Instrument the filter hold path:** count consecutive rejected reads per
  wheel and emit an EVT (or include the streak in TLM) when it exceeds a few
  ticks. A silent permanent hold is the worst failure mode in this chain.
- **Include the raw read in the wedge EVT** alongside the filtered value:
  raw frozen too → real wedge/stall; raw moving while filtered frozen →
  filter hold. One field makes the EVT self-disambiguating.
- **Arming grace at drive start:** require the wheel to have moved once since
  command start (or scale the threshold with commanded speed) so spin-up lag
  on a drained battery doesn't fire it.
- Battery voltage in TLM would settle the stall-vs-wedge question directly.

The original `fr-bench-right-encoder-wedge.md` issue should be re-pointed at
this chain: the hardware encoder is exonerated by the balance retest; the
remaining work is the reset verification + filter instrumentation above, then
a re-run on a fresh battery.

### Post-refutation addendum (after fr-bench-right-encoder-wedge was refuted)

The issue's correction (D syntax is `D <left> <right> <distance>`, so the
"equal-wheel" drives were actually unequal) is confirmed against
`MotionCommandHandlers.cpp` and the 032 log. Two refinements from the log
(`docs/bench-validation-032/tlm_log.txt`):

- The measured L/R ratios *exceed* the commanded ratios (D_slow: commanded
  250/150 = 1.67×, measured 217/81 = 2.7×; D_med: commanded 400/300 = 1.33×,
  measured 421/177 = 2.4×). The slower-commanded right wheel under-performs
  disproportionately — visible in the ramp (`vel=78,19` … `234,120`). On a
  weak battery the lower-commanded wheel sits near the min-PWM/stiction floor
  and lags badly at drive start. That start-up lag (R crawling 3→7→9 mm while
  L runs 26→38→45) is precisely the regime that trips the 240 ms wedge
  detector — the likely true source of `EVT enc_wedged wheel=R enc=0 n=10`.
  Not a readback wedge; a slow-starting wheel.
- Re-run the ratio check on a fresh battery before drawing calibration
  conclusions from any unequal-speed drive.

---

## NEW FINDING (from the 032 log) — D distance-stop fires instantly when prior encoder average ≈ target: baseline snapshot races the encoder reset

Reading the TLM log exposed an unreported anomaly: **sqD2 and sqD4 never moved
at all** — `enc=0,0`, `mode=I`, pose frozen for their entire 3 s windows —
while sqD1/sqD3 and all seq-3 drives ran normally. Nobody flagged it
(`analyze()` only checks jumps/residuals, and the verdict says "clean
starts/stops").

**Root cause (confirmed by ordering in source):**
`MotionController::beginDistance` (`MotionController.cpp:340-389`) resets the
*hardware* accumulators, then calls `_activeCmd.start(inputs, now_ms)` which
snapshots `base.enc0Mm = (encLMm + encRMm)/2` from `state.inputs` — but
`state.inputs.encLMm/R` are zeroed only *after* `beginDistance` returns, in
`Robot::distanceDrive` (`Robot.cpp:432-441`). The comment at
`MotionController.cpp:382-386` claims "the baseline enc0 captured by
MotionCommand::start() will be 0" — wrong: at snapshot time the inputs still
hold the previous command's values. Next tick, the collect reads the
freshly-zeroed hardware (≈0), and the DISTANCE stop
(`StopCondition.cpp:131-139`) computes `traveled = |0 − enc0|` = the stale
average. If the previous command left avg-encoder ≥ targetMm, the stop fires
on the first evaluate and the drive completes instantly with zero motion.

**Log verification (exact numbers):**

- sqD2 (`D 300 250 250`, target 250): prior enc from sqT1 = `90,410` → avg
  **250.0** ≥ 250 → instant stop. Observed: zero motion. ✓
- sqD4: prior enc from sqT3 = `183,319` → avg **251** ≥ 250 → instant stop.
  Observed: zero motion. ✓
- sqD3: prior enc from sqT2 = `67,−66` → avg **0.5** → ran normally. ✓
- All seq-3 drives follow `ZERO enc` → enc0 = 0 → ran normally. ✓

The alternating success/failure of the square legs is fully explained: each
D+TURN pair leaves avg ≈ 250 mm on the accumulators (TURN does not reset
encoders; its symmetric ±211 mm rides on top of the D leg's 301,199).

Corollary even when it doesn't instant-fire: any leftover average shortens
(or, if negative, lengthens) the next D by that amount — a silent distance
error on every D not preceded by `ZERO enc`.

**Fix:** make the baseline snapshot see zeroed inputs — e.g. zero
`state.inputs.encLMm/R` (the full `resetEncoders()` sequence) *before*
`_activeCmd.start()`, or explicitly set `enc0Mm = 0` for D-origin commands.
Add a sim test: D → TURN → D with no ZERO between; second D must travel the
full distance.

**Also unexplained (separate, smaller):** `T_timed_1500` (`T 1500 300 300`)
produced zero motion for its whole window (enc=0,0, pose 0,0,0, mode=I from
t=0.41). Check the T argument order — if T is `<left> <right> <ms>`, 1500 is
an out-of-range speed and the command may have been rejected (reply invisible
over the relay, same as every other reply problem in this run). Worth a
serial-port re-test before filing.

---

## Finding 1 (original) — right encoder wedge corrupting odometry (firmware-hardening angle)

The hardware fault itself is confirmed and out of scope here, but the firmware
currently has detection without defense:

- Detection exists: `MotorController` wedge detector (015-003) —
  `_stuckCountL/R`, `kWedgeThreshold=10`, `EVT enc_wedged` emission
  (`MotorController.h:183-204`).
- Nothing consumes it: `Odometry::predict()` integrates `dL/dR`
  unconditionally (`Odometry.cpp:40-55`). A wedged right wheel turns the
  missing counts into `dTheta = (dR-dL)/track` — the phantom heading swing —
  and EKF predict propagates it into fused pose with no opposing observation
  (OTOS was gated out, Finding 3).

**Hardening options for the odometry side:**
- Expose per-wheel wedge state from `MotorController` (e.g.
  `bool wheelWedged(L/R)` from stuck counters/latches).
- While a wheel is flagged wedged: stop integrating the differential into
  `dTheta` (hold heading; optionally estimate `dCenter` from the healthy wheel
  alone), and mark pose degraded so the host knows the estimate is coasting.
- When encoder-velocity fusion is un-gated (Finding 3 fix), suppress the
  `enc_omega` observation while wedged for the same reason.

Acceptance for the firmware part stays as the issue states: no phantom heading
swing on a single wedged encoder, characterized with `enc_selftest`
before/after.

---

## Suggested fix order

1. **Switch the bench harness to the robot's USB serial port** (see the
   stakeholder directive at the top). This makes the DBG replies visible as
   designed and removes the relay as a confound. Finding 2 then reduces to
   verifying the commands behave on serial; no `ForceReply` change is needed
   for bench work. (A `ForceReply` change is only worth considering if remote
   radio access to these commands is ever a real requirement — do not do it
   as part of this fix.)
2. Re-run bench 032 over USB serial with `DBG OTOS BENCH 1` confirmed via its
   `OK dbg otos bench=1` reply — this alone should make `twist=` nonzero
   (bench passes the gates), separating the transport problem from the fusion
   gating bug.
3. Finding 3 un-gating of encoder-velocity fusion — correct for real-floor
   operation whenever OTOS drops out; sim test: OTOS invalid + wheels moving →
   twist tracks encoder velocity.
4. Finding 1 odometry wedge defense, after the hardware investigation decides
   whether the wedge is fixable at the bus/electrical level.
