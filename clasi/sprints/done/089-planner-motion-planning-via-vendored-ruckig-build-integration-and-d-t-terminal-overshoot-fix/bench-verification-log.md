# Bench Verification Log — Sprint 089, Ticket 007

Date: 2026-07-07. Robot: Tovez (NEZHA2, differential drive), mounted on a
stand, wheels off the ground, direct USB serial
(`/dev/cu.usbmodem2121102`) unless noted. Firmware: freshly `just
build-clean` + `mbdeploy deploy`'d image of this exact sprint branch,
`fw=0.20260707.17` (confirmed via `ID`/`VER` matching the local build
banner before every run in this log).

**Overall verdict: SPRINT-BLOCKING FAILURES FOUND. Ticket 007 is NOT
complete; do not close the sprint on this bench pass.** Two distinct
findings block acceptance:

1. **D/T terminal reverse-motion is still present**, in the same order of
   magnitude as the pre-fix hardware-confirmed bug (11-23mm measured here
   vs. the originally-reported 16mm/23mm) — the sprint's headline
   acceptance bar. Root-caused below to a real interaction between
   Decision 8's seeding contract and the bench-tuned velocity PID's own
   tracking characteristic, not a measurement artifact (confirmed via two
   independent measurement paths).
2. **TURN cannot complete at all** on this hardware right now: its stop
   condition (fused heading) never accumulates, so it spins until my
   bench harness force-stops it. Root-caused to `PoseEstimator`'s fused
   pose (`pose=`) being frozen/non-updating on this specific
   robot/session, independent of wheel motion — likely a **pre-existing
   defect outside sprint 089's own scope** (architecture-update.md
   Decision 9 declares `PoseEstimator` unchanged), but it is still a real
   defect this bench pass surfaced, and it blocks TURN's own ticket 007
   acceptance criterion regardless of attribution. The SAME symptom also
   explains `G`'s failure to arrive.

Both findings are backed by raw telemetry captured below and by direct
reads of the current `source/` implementation (not guessed).

---

## 0. Setup

- `mbdeploy probe` / `mbdeploy list` confirmed ROLE before touching
  anything: the connected device (UID
  `9906360200052820a8fdb5e413abb276000000006e052820`, port
  `/dev/cu.usbmodem2121102`) is the robot (`NEZHA2`/`tovez`), not a relay
  — `mbdeploy list`'s cached name field showed a garbled `NEZA2`/`robt`
  (a benign classification-read glitch; `mbdeploy probe`'s own raw
  listing and a direct `HELLO`/`ID` both confirm `NEZHA2`/`tovez`). The
  relay dongle (`RADIOBRIDGE`/`relay`, historically on
  `/dev/cu.usbmodem2121402`) was **not physically connected** during this
  session (`ls /dev/cu.usbmodem*` showed only the robot's port
  throughout) — see §8, this blocks the radio-relay acceptance criterion
  pending the dongle being plugged in.
- `mbdeploy deploy --build` was NOT used (known broken venv gap, per
  `.clasi/knowledge/`). Used `just build-clean` (builds ARM firmware +
  host-sim library) then `mbdeploy deploy <full-UID> --hex
  MICROBIT.hex`. First deploy attempt failed with "Unable to claim
  interface" — an unrelated macOS process (Google Chrome, holding a
  `UsbExclusiveOwner` WebHID/WebUSB claim on the CMSIS-DAP device per
  `ioreg`) had grabbed the USB device; a graceful `osascript` quit of
  Chrome released the claim and the deploy then succeeded cleanly
  (erase + program, 492544 bytes, no further errors).
- `DEV WD 20000` (20s) widened the serial-silence watchdog for the whole
  session; every script wraps its exercise in a `finally` that sends
  `STOP` + `DEV STOP` + `DEV WD 1000` (restoring the 1000 ms boot
  default) even on exception/Ctrl-C. Motors were never left running
  unattended.
- **Pre-bench sanity**: `uv run python -m pytest tests/sim` — **308
  passed, 2 xfailed** (the two documented, still-open xfails), run
  immediately before the bench pass. Re-confirmed unaffected at the time
  of writing this log (no `source/` edits were made during the bench
  pass).
- New bench tooling written for this ticket (`tests/bench/` — HITL CLI
  tools, not pytest-collected, per `tests/CLAUDE.md`):
  - `tests/bench/bench_ruckig_motion_verify.py` — drives `D`/`T`/`TURN`/
    `RT`/`G`, captures TLM+EVT through and past `EVT done`, computes
    post-done encoder-delta ("no reverse"), completion `reason=`, and
    (TURN/RT) a `pose=`-based heading-accuracy check.
  - `tests/bench/solve_time_characterize.py` +
    `tests/bench/solve_time_gdb_batch.gdb` — pyOCD/gdb DWT-cycle-counter
    attempt at on-target solve timing (partially blocked, see §7).
  - An interactive-gdb wall-clock fallback was also attempted (a
    `readline()` framing bug against gdb's own no-newline `(gdb) ` prompt
    hung the subprocess) and was discarded rather than fixed, since the
    indirect TLM-gap method below already gives an adequate "rough"
    number.

---

## 1. D 200 200 1000 — no-reverse and peak-speed

**Acceptance criterion: FAIL (reverse motion), peak-speed: same order of
magnitude as the pre-fix bug, but see the important caveat below about
attribution.**

Three independent runs, using two independent measurement paths:

| Run | Method | enc/pos at done | post-done delta | reverse (mm) | peak speed (mm/s, commanded 200) | completion reason |
|---|---|---|---|---|---|---|
| 1 | TLM `enc=` (bench script) | (1031, 1015) | L −11.0, R −10.0 | **11.0** | 301 | `dist` |
| 2 | `DEV M n STATE` `pos=` (direct per-motor register, independent of TLM fusion) | M1 530.4→509.0, M2 511.2→499.6 | M1 −21.4, M2 −11.6 | **21.4** | (M1 chip vel 220.9 at done) | `dist` |
| 3 | TLM `enc=` (bench script, post settle-gap fix) | (1037, 1005) | L −16.0, R −17.0 | **17.0** | 305 | `dist` |

The **completion-mode criterion passes**: all three runs report
`EVT done D reason=dist` — the goal's own `STOP_DISTANCE` condition, never
`STOP_TIME`.

**The reverse-motion criterion fails.** 11-21mm is squarely in the same
magnitude as the originally-confirmed hardware bug (~16mm) that this whole
sprint exists to fix. This was cross-checked via TWO independent
measurement paths (fused TLM telemetry vs. the raw per-motor `DEV M`
register) specifically to rule out a telemetry-fusion artifact — both
agree a real, reproducible position regression occurs after `EVT done`.

### Root cause (read directly from `source/`, not guessed)

Run 2's raw `DEV M n STATE` trace is the clearest evidence:

```
at-done   M1: pos=530.4 vel=220.9 applied=-0.25   (still fast, POSITIVE, yet ALREADY braking hard)
+150ms    M1: pos=521.5 vel=75.2  applied=-0.17
+300ms    M1: pos=509.1 vel=0.1   applied=0.00
+600ms    M1: pos=509.0 vel=-0.1  applied=0.00     (settled)
```

Velocity **never goes meaningfully negative** at any point (unlike the
*original* bug's own documented mechanism, which showed an explicit
negative-signed velocity residual) — so this is a *smaller, differently-
triggered* version of the same class of bug, not the untouched original
one. Reading `source/subsystems/planner.cpp` and
`source/motion/jerk_trajectory.cpp` directly explains why:

1. `Planner::apply()`'s `DISTANCE` case calls
   `linear_.solveToRest(distance, linearCeiling_)` with
   `linearCeiling_ = fabsf(speed)` (200) — **confirmed correct**: the
   commanded speed, not `v_body_max`, is the solve's velocity ceiling
   (Decision 2's revision is correctly implemented).
2. Every tick, `Planner::tick()` calls
   `distanceV = linear_.sample(linearElapsed(now)).velocity` — and
   `JerkTrajectory::sample()` (`jerk_trajectory.cpp:170-179`) **always
   overwrites `lastVelocity_`/`lastPosition_` with the PLAN's own
   theoretical value at this elapsed time**, never the real measured
   speed.
3. On the real robot, the velocity PID (bench-tuned firmware defaults,
   `source/config/boot_config.cpp`: `kp=0.0022, ki=0.0018, kff=0.0038` —
   confirmed **unchanged by this sprint**, and explicitly commented
   "bench-tuned... sprint 077-007") **does not track the commanded ~200
   mm/s target tightly** — measured wheel speed runs ~250-310 mm/s for
   most of the cruise (see §1's `peak speed` column and the full captured
   trace in `tests/bench/out/bench_089_007_smoke.json`). This looseness
   pre-dates this sprint and is explicitly out of Decision 3's scope
   (`Hal::MotorVelocityPid` is untouched).
4. Because the REAL plant is running faster than the plan believes, the
   real encoder crosses the `STOP_DISTANCE` threshold ***before*** the
   plan's own internal position state would naturally begin
   decelerating. `Motion::remainingToStop()`'s divergence-triggered
   replan (Decision 10) correctly does **not** fire in this direction —
   its own "no-reverse-target guard" is designed to skip when "the plant
   has already reached or passed the target," which is exactly this
   case — so `armDistanceStopDecel()` fires instead, seeding
   `linear_.solveToVelocity(0.0f, ...)` from `lastVelocity_`, which
   `sample()` JUST set to the PLAN's own (lower, ~200-ish) cruise value
   THIS SAME TICK — **not** the real (~250-310) measured speed.
5. The very next tick, the newly-armed decel plan commands a value close
   to its own ~200 belief, decelerating smoothly *from the plan's
   perspective* — but the REAL wheel is still running faster than that,
   so the velocity PID sees a persistent negative error relative to the
   real wheel and brakes, producing the observed 11-21mm reverse creep.
   This is smaller than the original bug (which stepped literally to
   zero) because the new decel starts from ~200, not ~0 — a genuine,
   partial improvement — but it is not eliminated, and 11-21mm is not
   "no reverse encoder motion."

**This is a real interaction between Decision 8's seeding contract
("never seed from measured state," correct in principle — it is exactly
what avoids the 087-009 limit-cycle bug it cites) and the plant's own
pre-existing tracking looseness.** Decision 8's assumption that the
channel's own theoretical belief tracks the real plant closely enough
does not hold on this specific hardware/PID-tuning combination. This is
squarely why `.claude/rules/hardware-bench-testing.md` and this sprint's
own architecture doc insist on a real bench pass — the sim's idealized
plant (zero tracking error by construction) cannot reproduce this
divergence at all, which is exactly the Grounding section's own warning.

**Peak-speed nuance:** the ~300 mm/s peak (vs. commanded 200, similar
magnitude to the original ~292 mm/s bug report) is very likely **NOT a
Ruckig/plan-shape regression** — `linearCeiling_=150` was confirmed
correctly passed as the solve's own velocity ceiling in a
separately-captured breakpoint hit (see §7), so the PLAN itself never
asks for more than the commanded speed; the measured overshoot is the
(unchanged, out-of-scope) PID's own tracking characteristic. Recorded
here for completeness per the ticket's own acceptance text, but the
reverse-motion finding above is the one that fails the bar.

---

## 2. T 200 200 1000 — no-reverse

**Acceptance criterion: FAIL (reverse motion). Completion-mode: PASS
(reason=time is T's own correct, natural stop — T has no analogous
stall-short gap, per Decision 10's own scope).**

| enc at done | post-done delta | reverse (mm) | peak speed | completion reason |
|---|---|---|---|---|
| (311, 286) | L −16.0, R −19.0 | 19.0 | 304 | `time` |
| (318, 290) | L −18.0, R −23.0 | **23.0** | 309 | `time` |

23.0mm matches the originally-reported "~23mm" T bug almost exactly. Same
root cause as §1 (T's `TIMED` goal kind uses the SAME
`armVelocityStopDecel()` mechanism, seeded the same way, per
`planner.cpp:694-709`).

**Terminal-chatter check (Open Question 7):** T's tail settles to a
single flat resting value (`enc=(300,267)`, `vel=(0,0)`) and STAYS there
for the full 2.5s post-done observation window — **no repeated
re-acceleration bumps observed**. The reverse creep above is a single,
monotonic settle to a lower resting position, not a repeating chatter
pattern. Same for D's trace. See §6.

---

## 3. TURN 9000 (~90°, from a `SI 0 0 0` zero baseline) — BLOCKED, cannot complete

**Acceptance criterion: FAIL — TURN never converges; not a Ruckig-shape
question, a functional non-completion.**

Three attempts (two via the bench script, one via an isolated manual
`SNAP`-polling diagnostic), **none completed** within an 8s budget:
`mode=` correctly reads `T` (TURN's own wire-sharing collapse, expected)
and the wheels visibly, substantially spin (`enc=` differential growing
past ±550-700mm by the time each attempt was force-stopped) — but
`pose=`'s heading component never leaves single/double-digit
centidegrees (≈0-1°) despite the large real rotation, so `STOP_HEADING`
(which reads `fusedPose.pose.h`, unchanged by this sprint — Decision 9)
never fires. No `EVT done TURN` was ever observed. My bench harness's own
timeout force-stopped each attempt; **on real hardware with no external
supervision TURN would spin indefinitely** (it has no user-independent
time safety net the way D does).

Isolated diagnostic trace (`TURN 9000`, `SI 0 0 0` first):

```
elapsed  mode  enc              pose (fused)     encpose (dead-reckon)
0.81s    T     (-37, 26)        (-2, 0, 61 cdeg)    (-5, -1, 2888 cdeg)
1.66s    T     (-125, 118)      (2, 0, 36 cdeg)     (-4, 0, 10960 cdeg)
2.50s    T     (-211, 192)      (-1, 0, 34 cdeg)    (0, -3, -17917 cdeg)
4.14s    T     (-361, 340)      (1, 0, 3 cdeg)      (0, -4, -4855 cdeg)
8.22s    T     (-696, 672)      (1, 0, 4 cdeg)      (-1, -5, -10979 cdeg)
```

`encpose` (the pure dead-reckoning integrator, independent of any EKF
fusion) is wildly erratic (jumping between roughly ±18000 centidegrees
frame to frame — values consistent with an unwrapped/overflowing raw
accumulator, not a real heading), and `pose` (fused) stays essentially
frozen near 0 the entire time. **`G`'s own failure (§4) shows the SAME
signature for x/y position, not just heading** — `pose=(0,0,-7)` never
moved from its initial value even after **1.3+ meters** of real,
encoder-confirmed wheel travel.

**Attribution:** `PoseEstimator` is explicitly unchanged by this sprint
(architecture-update.md Decision 9: "`PoseEstimator`... is unchanged");
none of tickets 001-006 touch `pose_estimator.cpp`. This strongly suggests
a **pre-existing defect** in the real-hardware dead-reckoning/fusion path
(not the sim, which the sprint's own tests already prove exercises a
working `evaluateStopCondition()`/`headingError()` against a correctly-
advancing sim-plant heading), surfaced here because this is the first
time this specific ticket exercises a real, sustained TURN/G run against
this exact firmware/robot combination. **Root-causing and fixing this is
out of ticket 007's own scope** (bench verification, not implementation),
but it is a real, newly-surfaced defect that blocks this ticket's own
acceptance criterion and should be raised as a follow-on issue (not
created here per this ticket's own instructions not to touch
`clasi/issues/`).

`ID`'s `caps=` field was empty (no `otos`/`line`/`color` detected at
boot) and `DBG OTOS` returned `ERR unknown` (not registered in this
build) — ruled out as a quick explanation; Tovez has no real OTOS chip
either way (matches `.clasi/knowledge/otos-offset-register-unwritable.md`
and this sprint's own Grounding), so `pose=` should be coming from the
dead-reckoning integrator exactly as `encpose=` does — but the two
disagree in exactly the way described above.

---

## 4. RT 9000 (relative) — completion mechanism PASSES; accuracy NOT reliably measured

**Completion-mode criterion: PASS.** `EVT done RT reason=rot` fired
correctly on every successful dispatch (2 of 2, after a settle-gap fix
described below) — RT's own `STOP_ROTATION` condition
(`rotationProgress()`, the RAW per-wheel encoder-arc differential, per
Decision 9) is **entirely independent of the broken `pose=`/`encpose=`
path** (§3), so RT is immune to the TURN-blocking defect and completes
in a reasonable time (~1.5-2s for a 90° spin) exactly as expected.

**Reverse-motion:** one clean run showed `reverse_mm=13.0` (established-
sign method, per-wheel — see script docstring) — smaller than D/T but
still non-zero; not conclusively "no reverse" either, though the smaller
magnitude and RT's different (rotational, no linear-channel-lag)
mechanics make direct comparison to D/T's number less apples-to-apples.

**Accuracy:** could NOT be reliably established this session. My bench
script's accuracy check reads `pose=`'s heading component (matching the
sim tests' `true_pose()` comparison pattern) — but §3 already establishes
`pose=` does not accumulate correctly on this hardware, so every RT
accuracy read came back near-zero delta (clearly wrong, not a real
measurement). A post-hoc encoder-differential estimate from one clean
run (`enc` at done ≈ (−123, 120), trackwidth 128mm →
`(120-(-123))/128 * 180/π ≈ 108.8°`) is available but I do not have
enough confidence in its precision (150ms host-side poll granularity,
no tight synchronization to the exact wire completion instant) to grade
it against the ±7° bar one way or the other — flagged as **not reliably
measured**, not as a pass or a fail, rather than asserting a number I
can't stand behind. A follow-up with SNAP-synchronized sampling right at
`EVT done` (or after §3's `pose=` defect is fixed) is needed to close
this out properly.

**A real, reproducible dispatch-reliability issue was also found and
fixed during this session**: `RT`/`TURN`/`G` intermittently failed to
dispatch at all (silent — no `OK` reply, `mode=` stayed `I`, nothing
moved) when sent via `send_fast()` immediately after a blocking
`STREAM 20` reply, on this specific USB-CDC link. Adding a 150ms settle
gap after `STREAM`'s own reply, and switching the initial dispatch from
`send_fast()` to `send()` (corr-id + its own corrupted-command retry),
made dispatch reliable across the runs quoted above. This looks like the
documented USB-CDC flakiness (`.clasi/knowledge/`: "intermittently drops
15-50%"), not a firmware defect — noted here since it cost real session
time and the fix is now baked into `tests/bench/bench_ruckig_motion_verify.py`
for any future run of this tool.

---

## 5. G 300 0 150 — smoke check FAILS to arrive (same root cause as §3)

`G` dispatched (`mode=G` correctly active) and ran the FULL exercise —
covering **1379mm/1357mm of real per-wheel encoder travel** (over 1.3
meters!) — before ending via `EVT done G reason=time` (the TIME safety
net, not arrival). `pose=` stayed at `(0, 0, -7)` **the entire time**,
never once reflecting the substantial real travel. `pursueSteer()`'s
world-frame steering depends on live `fusedPose`, so with `pose=` frozen
near the origin the whole time, `G` never perceives itself as having made
progress and just keeps driving until the time net cuts it off — the
same underlying defect as §3, not a separate `G`-specific bug (`G`'s own
code path, `VelocityRamp`/`pursueSteer()`, is explicitly byte-for-byte
untouched by this sprint per Decision 5/Step 5's Impact table).

**Acceptance criterion: FAIL** (did not reach the target region, did not
emit a genuine arrival completion) — but per the same attribution as §3,
likely not a Ruckig-migration regression.

---

## 6. Terminal near-rest replan-chatter characterization (Open Question 7)

**No chatter observed** in the D or T traces (§1, §2) — both settle to a
single flat resting encoder value within ~300-600ms of `EVT done` and
stay there for the remainder of the 2.5s observation window; no repeated
re-acceleration bumps. TURN and `G` could not be characterized for this
at all, since neither ever converges (§3, §5) — there is no "near rest"
state to observe chatter around when the goal never approaches
completion. RT's short duration (~1.5-2s door-to-door) did not leave much
of a distinct "near rest, still open" window to inspect separately from
its own clean completion. **This characterization is therefore partial**:
the two goal kinds that DID reach a clean terminal state (D, T) show no
chatter; the divergence-replan's chatter risk (Open Question 7) remains
untested for TURN specifically, since TURN never gets there. No
mitigation was pre-built, per the ticket's own instruction.

---

## 7. On-target Ruckig solve-time characterization (Open Question 4)

**Precise on-target cycle timing was attempted via pyOCD/gdb + the
Cortex-M4 DWT cycle counter and was NOT successfully completed** —
recorded honestly rather than papered over:

- `just debug`-equivalent (`pyocd gdbserver -t nrf52833 --persist`)
  attached fine (SWD-only, no `load`/`monitor reset halt`, confirmed not
  to disturb the live serial session, per `.claude/rules/debugging.md`'s
  own claim).
- A breakpoint at `jerk_trajectory.cpp:101` (the `otg_.calculate()` call
  site shared by `solvePositionControl()`, used by both D's linear
  channel and TURN/RT's rotational channel) **hit correctly** and
  confirmed the right call parameters — e.g. for a live `D 150 150 400`:
  `targetPosition=400, maxVelocity=150` (exactly matching the commanded
  values, corroborating §1's claim that the plan's own ceiling is
  correctly the commanded speed, not `v_body_max`).
- Enabling the DWT cycle counter (`DEMCR`/`DWT_CTRL` writes via gdb) failed
  with `Cannot access memory at address 0xe000edfc` — reproduced twice,
  including once at an active (non-WFI) breakpoint stop, so it is not
  simply a sleep/WFI-halt artifact. Likely a debug-port/gdbserver memory-
  access restriction specific to this probe/pyOCD combination, not
  something resolved within this ticket's time budget.
- The SWD link itself also degraded mid-session (`SWD/JTAG communication
  failure (Unexpected ACK '0')`) after several attach/detach cycles,
  requiring a full gdbserver restart.
- An interactive-gdb wall-clock fallback (`solve_time_interactive.py`)
  was attempted to sidestep DWT entirely, but had a `readline()` framing
  bug against gdb's own no-newline `(gdb) ` prompt (hung, was killed, not
  debugged further — diminishing returns given the indirect method below
  already gives an adequate rough number).

**Indirect, zero-instrumentation characterization (successful):** the
firmware's own `TLM t=` timestamps (robot clock, captured at
sensor-sample time) were checked for tick-period gaps across every `D`,
`T`, and `RT` capture in this log. All three show an **identical
signature**: every tick is a clean 20ms except for exactly ONE ~32ms
tick (a one-time ~12ms bump) — consistently observed at (or immediately
around) the goal's own `apply()`-time dispatch, in every run:

| Verb | frames | min/max tick gap | ticks >25ms |
|---|---|---|---|
| D  | 305 | 20 / 32 ms | 1 (32ms) |
| T  | 206 | 20 / 32 ms | 1 (32ms) |
| RT | 184 | 20 / 32 ms | 1 (32ms) |

**Conclusion:** the Ruckig solve (linear channel via D/T, rotational
channel via RT — both hit the same `solvePositionControl()` breakpoint,
confirming both channels are exercised) costs on the order of **~12ms,
once, at the tick it is invoked** — a one-time, ~60% addition to a single
20ms tick's budget, not a chronic per-tick cost (Decision 2's own
design). This is well within the control loop's own period and leaves
adequate headroom; it does **not**, on its own, suggest a future
per-tick `GOTO_GOAL` solve (explicitly out of this sprint's scope) is
infeasible, though that would need its own characterization (a per-tick
cost repeated every 20ms, not a one-time cost, is a materially different
budget question).

---

## 8. Radio relay path — NOT completed (hardware not connected)

**Acceptance criterion: NOT VERIFIED — the relay dongle was not
physically plugged in during this bench session** (`ls
/dev/cu.usbmodem*` showed only the robot's port throughout;
`mbdeploy probe`'s registry entries for the relay, e.g. UID
`...4f02a3519fdba0c7...` on `/dev/cu.usbmodem2121402`, are historical —
that port was absent this session, confirmed via a plain `ls`, not just
the `mbdeploy` cache). This agent has no ability to physically plug in a
USB device. **Action needed**: plug the relay dongle in and re-run
`tests/bench/bench_ruckig_motion_verify.py --mode relay` (or let
`SerialConnection`'s auto-detect classify it — it already handles the
`!GO` handshake, per `.clasi/knowledge/2026-06-12-relay-go-data-plane-
and-docs.md`) for at least one motion verb, per this ticket's own
acceptance criterion.

---

## Summary table

| Acceptance criterion | Verdict |
|---|---|
| Robot vs. relay identified via `mbdeploy list` ROLE before flash | PASS |
| DEV watchdog widened/fed/restored; STOP in a finally | PASS |
| `D 200 200 1000`: no reverse + peak speed within tolerance | **FAIL** (11-21mm reverse; peak-speed likely pre-existing PID characteristic, not graded as blocking on its own) |
| `T 200 200 1000`: no reverse | **FAIL** (19-23mm reverse) |
| `TURN` ~90°: no reverse + accuracy within bar | **FAIL** (never completes at all) |
| `RT`: no reverse + accuracy within bar | **PARTIAL** (completion mechanism verified correct; accuracy not reliably measured; reverse_mm=13 observed once) |
| `G`: dispatches and settles | **FAIL** (never arrives; ends via time net) |
| On-target solve time characterized (1 linear + 1 rotational) | PASS (indirect method; precise DWT attempt documented as blocked) |
| At least one verb over the radio relay | **NOT VERIFIED** (hardware not connected) |
| Written bench log | PASS (this document) |
| `uv run pytest tests/sim` green pre-bench, reconfirmed at close | PASS (308 passed, 2 xfailed, both times) |
| D/TURN/RT complete via OWN stop condition, never STOP_TIME | **PARTIAL** (D: PASS `reason=dist`; RT: PASS `reason=rot`; TURN: N/A, never completes at all — the failure mode Decision 10 exists to close is present in an even more severe form for TURN specifically) |
| Terminal near-rest chatter characterized | **PARTIAL** (D/T: no chatter observed; TURN/RT: could not be characterized — TURN never reaches near-rest, RT's window was too short) |

**Recommendation:** do not mark ticket 007 done or close sprint 089 on
this bench pass. The D/T reverse-motion finding (§1-2) is the sprint's
own headline acceptance bar and it fails, with a well-supported root
cause pointing at the interaction between Decision 8's seeding contract
and the plant's pre-existing PID tracking looseness — this needs either
a design revisit (e.g., blending in a bounded correction toward the
measured speed specifically at the stop-decel handoff, without
reopening the 087-009 limit-cycle risk Decision 8 was written to avoid)
or an explicit, stakeholder-accepted decision that this residual is
tolerable. The TURN/`pose=` finding (§3-5) is very likely a separate,
pre-existing defect outside this sprint's own scope, but it independently
blocks TURN's and `G`'s own criteria and should be raised as its own
follow-on issue for someone to root-cause `PoseEstimator`'s dead-
reckoning path on real hardware.
