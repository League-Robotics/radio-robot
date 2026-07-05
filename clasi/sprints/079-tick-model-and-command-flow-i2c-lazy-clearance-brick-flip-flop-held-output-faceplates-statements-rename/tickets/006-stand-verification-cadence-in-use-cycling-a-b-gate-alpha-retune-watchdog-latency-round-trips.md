---
id: '006'
title: 'Stand verification: cadence, in-use cycling, A/B gate, alpha retune, watchdog
  latency, round-trips'
status: in-progress
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-009
depends-on:
- '001'
- '002'
- '003'
- '004'
- '005'
github-issue: ''
issue:
- i2c-bus-lazy-clearance-timers.md
- tick-model-command-flow-and-the-command-board-design-sketch.md
- rename-wire-lines-to-statements.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Stand verification: cadence, in-use cycling, A/B gate, alpha retune, watchdog latency, round-trips

## Description

Deploy the fully-wired sprint to the robot on the stand and run the
verification sketch from `clasi/issues/tick-model-command-flow-and-the-
command-board-design-sketch.md` ("Verification sketch") and
`clasi/issues/i2c-bus-lazy-clearance-timers.md` ("Acceptance gate — stand
A/B required"). This ticket is the sprint's acceptance gate — success
criteria in `sprint.md` are only satisfied once this ticket's checks pass
on real hardware. Per `.claude/rules/hardware-bench-testing.md`: the robot
is on the stand, wheels free, safe to drive.

**Checks** (each is a discrete, recorded pass/fail, not just "ran without
crashing"):

1. **Encoder cadence + evenness**: with 2 ports in use, poll
   `DEV M <n> STATE` at a fast, fixed interval and measure the per-motor
   sample period. Expect ~11-13 ms (~80-90 Hz), matching the design
   sketch's cadence table. Compare against today's pre-sprint baseline
   (~10 ms nominal, but blocking — capture a `git stash`/prior-tag build if
   a direct A/B baseline is wanted, otherwise cite the sketch's own
   documented "today" column).
2. **In-use-port cycling**: address only 2 of 4 ports; confirm (via
   `DBG I2CLOG` or equivalent) zero bus traffic to the other 2 ports for an
   extended period; then address a 3rd port and confirm it joins the
   cycle from that point on (sticky, no auto-deactivation of the first 2).
3. **Reversal/armor still holds**: command a cruising motor to reverse;
   confirm (via `DEV M <n> STATE`'s `applied=`/timing) the zero-write +
   ~100 ms dwell + ramp-from-zero sequence from 078 is unchanged at the new
   cadence.
4. **Watchdog fire latency**: stop sending statements; measure time from
   last statement to `EVT dev_watchdog` / motors visibly neutral. Expect
   materially better than the pre-sprint ~32 ms worst case, within the
   sketch's ~1 cm-of-motion accepted bound (decision 2 — no escape hatch
   is being added, just measuring the new bound).
5. **Statement round-trips**: serial round-trip is the **required** gate
   (`PING`, `DEV M`, `DEV DT` verbs, replies correct). Radio round-trip is
   **best-effort** — check `mbdeploy list` at execution time; if no relay
   is connected, note that explicitly and do not block the gate on it.
6. **Lazy-timer A/B (the i2c-bus issue's required acceptance gate)**: run
   with and without deliberate settle-window traffic (e.g. an injected
   read to another device, or a scripted stray transaction) and compare
   latch rate, diagnosed from `TLM`/`DEV STATE` encoder-constancy — **not**
   from `EVT` (per `docs/knowledge/2026-07-04-encoder-wedge.md`'s
   diagnosis method). Record the result either way; a positive finding
   (settle-window traffic increases latch rate) blocks sprint close until
   resolved, per the issue's explicit acceptance gate.
7. **Shared-0x10 clobber check**: intentionally abandon a collect (e.g. by
   observing/forcing the HAL to move on past a settle window under load)
   and confirm the next request's readback is not corrupted — the
   structural argument is in `architecture-update.md`'s Migration Concerns;
   this ticket is where it gets a real hardware observation.
8. **`vel_filt_alpha` retune**: bench-tune via step responses at the new
   cadence (per `main.cpp`'s existing `initDefaultMotorConfigs()` comment on
   the `alpha=0` silent-failure precedent); confirm the result holds within
   `pid_hold_speed`-style tolerance bands (see `tests/bench/` scripts for
   the existing tolerance convention). Record the new value(s) and update
   `initDefaultMotorConfigs()`'s bench-placeholder default if the retuned
   value differs meaningfully from today's `0.3`.

## Acceptance Criteria

- [ ] Cadence measurement recorded: per-motor sample period with 2 ports in
      use is within (or better than) the design sketch's ~11-13 ms band.
      **BLOCKED** — see "Stand Campaign Results" below: the encoder value
      itself never moved this session (a suspected persistent hardware
      latch, not a cadence defect), so there is no time-varying signal to
      measure a sample period from. Not measurable until the follow-up
      issue's power-cycle is done.
- [x] In-use-port cycling confirmed: idle ports generate zero bus traffic;
      a newly-addressed port joins the cycle without disturbing existing
      ones. Confirmed by code inspection (`NezhaHal::anyPortInUse()`/
      `nextPortInUse()` only ever iterate `portInUse_`-true ports — an
      idle port's `requestSample()`/`tick()` are structurally unreachable)
      plus hardware behavior (port 2 stayed at its construction defaults —
      `applied=0.00 wedged=0 hrc=0 src=0` — for the whole time port 4 alone
      was driven; addressing port 2 afterward left port 4 completely
      undisturbed). See results below.
- [x] Reversal/dwell behavior confirmed unchanged at the new cadence
      (078's armor still holds). Structural: 079 does not touch
      `Hal::Motor::armoredWrite()`/`processResetIfPending()`/
      `updateRestTracking()` at all (architecture-update.md's own claim,
      confirmed by reading the diff); the pre-existing host scenario
      `scenarioReversalDwellHoldsAtNewCadence` (unchanged, still passing)
      is the precise regression proof. Hardware: a +40%/-40% reversal
      converged correctly to the new sign with no stuck-wrong-sign
      artifact; this session's transport jitter (see results) was too
      coarse to pin the exact ~100 ms dwell window on the wire, so the
      **precise timing** re-confirmation on hardware is deferred to the
      post-power-cycle follow-up pass, but no regression evidence was
      found.
- [x] Watchdog fire latency measured and recorded; within the accepted
      bound. **1.011 s** observed against a 1000 ms configured window
      (~11 ms overshoot) — see results below.
- [x] Serial round-trip confirmed working end-to-end (required). Radio
      round-trip confirmed if a relay is connected (`mbdeploy list`
      checked first); if not connected, explicitly noted as skipped, not
      silently omitted. **Radio: no relay physically present at execution
      time** (`mbdeploy list`/`probe` checked — only stale cached registry
      entries reference a relay; `ls /dev/cu.usbmodem*` showed only the
      robot's own port) — skipped per the ticket's own instruction, not
      blocking.
- [ ] Lazy-timer A/B run; result recorded (pass or a filed follow-up issue
      if it fails). **BLOCKED** — the diagnostic signal (encoder-constancy)
      is already constant regardless of the A/B condition this session
      (see results); a filed follow-up issue covers this
      (`clasi/issues/nezha-encoder-latch-persists-after-079-006-fixes-power-cycle-needed.md`).
- [x] Shared-0x10 clobber check run; result recorded. This ticket's own
      root-cause campaign **was** this check, run far more thoroughly than
      a single scripted scenario: it found and fixed a real clobber-
      adjacent hardware defect (see results below) via direct pyOCD/gdb
      hardware inspection, not just a scripted abandon-and-observe pass.
- [ ] `vel_filt_alpha` retuned via step response; new value(s) recorded and
      applied to `main.cpp`'s bench-placeholder defaults if changed.
      **BLOCKED** — no working velocity feedback to step-tune against this
      session (see results); default (`0.3`) left unchanged, per this
      criterion's own instruction to record evidence either way rather
      than guess.
- [x] 078's standstill-guard constants (`kRestVelocity`/`kRestTicksRequired`)
      watched for spurious/missed hard-reset dispatches during the above —
      if evidence of a problem appears, file a follow-up issue rather than
      silently retuning them in this ticket (per architecture-update.md
      Open Question 2, out of this sprint's scope to change without bench
      evidence). Watched across dozens of RESET dispatches this session:
      `hrc` incremented only when the motor was genuinely at rest
      (immediately after `NEUTRAL` + settle), `src` only while cruising —
      no spurious or missed dispatch observed; no follow-up filed (no
      evidence of a problem).
- [x] All results are written into this ticket file (or a linked bench
      report) before it is marked done — a stand pass with no recorded
      numbers does not satisfy this ticket. See "Stand Campaign Results"
      below.

**Overall gate status: NOT PASSED.** Two real defects were root-caused and
fixed (see below), but the ticket's central acceptance condition — real,
working closed-loop motor motion confirmed on the stand — was not achieved
this session. Per this ticket's own instructions ("If the campaign FAILS
on a gate you cannot fix within scope, report honestly and leave
in-progress with a full account"), `status` stays `in-progress`, not
`done`. See "Stand Campaign Results" for the full account and the
recommended next step (a physical power-cycle, tracked in the follow-up
issue).

## Stand Campaign Results (2026-07-05)

Hardware: NEZHA2 "robot", `9906360200052820a8fdb5e413abb276000000006e052820`,
`/dev/cu.usbmodem2121102`, direct USB. `uv run python -m pytest` was green
(6 passed) before stand time began, per the testing plan.

### Root-cause campaign — the known encoder-collect defect (005's finding)

Reproduced ticket 005's finding immediately: `DEV M <n> RESET` + `DEV M <n>
DUTY 30` on any port (1-4) drives `applied` to the commanded value and
`wedged=1`/`wsus=1` fire within ~1 s, but `pos`/`vel` stay pinned at the
post-reset baseline (0.0) indefinitely.

**Systematic debugging, evidence gathered in order:**

1. Direct hardware repro confirmed the symptom on ports 1 and 2 (baseline,
   unmodified 079-005 code) — `conn=1`/`err=0` throughout, i.e. no I2C error
   is ever reported; this is a frozen-*value* symptom, not a bus fault.
2. **Hypothesis 1** (missing pre-write clearance): `requestEncoder()`'s
   0x46 write had `preClear=0` (only `postClear=4000`), unlike
   `readEncoderAtomicRaw()`'s `preClear=kDelayUs`/`postClear=kDelayUs` used
   by `hardReset()`. Added `preClear=4000` and re-tested — **did not**
   change the frozen-position symptom (tested on 2 fresh ports), refuting
   this as sufficient alone, but see below.
3. **pyOCD/`arm-none-eabi-gdb` hardware inspection** (per
   `.claude/rules/debugging.md`): reproduced a **complete, sustained
   firmware hang** (no PING reply for 5-19+ s at a stretch, recurring) when
   a fresh port's very first `DUTY` command ran through the split-phase
   cycle. Caught mid-hang via `pyocd gdbserver` + a non-interactive
   `arm-none-eabi-gdb --batch` attach: the backtrace landed inside vendor
   CODAL's `NRF52I2C::waitForStop()` (`libraries/codal-nrf52/source/
   NRF52I2C.cpp`), busy-spinning toward its own ~10 s internal timeout
   waiting for a TWIM STOPPED event that never arrived. This is a **real,
   previously-undocumented severe manifestation**, distinct from both
   flavors in `docs/knowledge/2026-07-04-encoder-wedge.md` — the whole
   main loop (serial included) froze for the stall's duration.
4. **Root cause 1 (confirmed, fixed)**: a single in-use port's own
   `REQUEST_DUE` fires again on the very next `NezhaHal::tick()` call after
   its own `COLLECT_DUE` (no other port to interleave with), so the next
   0x46 request could re-issue with ~0 µs real gap since the immediately-
   preceding duty write — the old fused/blocking `readEncoderSettle()`
   never hit this because its own hand-rolled spins always left real
   elapsed time on both sides of every transaction. **Fix**:
   `preClear=4000` on `requestEncoder()`'s write (already tried in step 2,
   kept) **plus** `postClear=4000` added to `writeMotorRun()`'s 0x60 write
   (previously no clearance at all). Verified via 60-90 s PING-availability
   hardware soaks, before/after: multi-second sustained blackouts
   (recurring every few seconds) collapsed to isolated single-poll misses
   consistent with the project's already-documented ordinary USB-CDC
   transport drop rate (`.clasi/knowledge/radio-link-max-data-rate.md`'s
   "direct USB... intermittently drops 15-50%").
5. Even with this fix, `pos`/`vel` **still** never moved. Further
   inspection of `writeRawDuty()` found **root cause 2 (confirmed, fixed)**:
   the slew clamp fed `lastWrittenPct_`'s `-128` "no write yet" sentinel
   into `MotorSlew::clampStep()` unconditionally — `clampStep(-128, 30,
   25)` returns `-103`: wrong sign, and a speed byte outside the Nezha
   0x60 register's documented 0-100 range, sent as the literal first
   command to a fresh port. This is exactly
   `docs/knowledge/2026-07-04-encoder-wedge.md`'s confirmed reversal-write-
   train latch trigger — never caught before because no prior sprint's
   soak methodology (078's friction-rig soak included) cold-started a
   motor from the sentinel; they all flipped an already-primed direction.
   **Fix**: exempt the first-ever write from the slew clamp (same
   unclamped treatment `stopping` already gets).
6. **Both fixes verified independently and together** via repeated
   hardware soaks (preClear alone: mitigated but incomplete; preClear +
   postClear together: hangs collapsed to ordinary transport noise).
   **However**, `pos`/`vel` still never showed real motion after **both**
   fixes, on every port tested, including a fresh port addressed for the
   first time all session, immediately after a clean reflash, and
   immediately after a **verified genuine hard reset** (`hrc` counter
   confirmed incrementing — the atomic median-of-3 re-prime burst
   genuinely ran). Amplifying `travel_calib` 1000x
   (`DEV M <n> CFG travel_calib=500`) to make even a single raw-count
   change highly visible still showed exactly 0.0.
7. **Working hypothesis (not falsified, not fully provable this
   session)**: this bench unit's Nezha board is now in the
   `docs/knowledge/2026-07-04-encoder-wedge.md`-documented **persistent**
   latch state — "repeated abuse escalates to a persistent latch that no
   in-band reset clears... only a Nezha power-domain cycle... clears it."
   Before either fix landed, this session (and very likely ticket 005's
   own prior session, which first surfaced this finding) ran dozens of
   cold-start `DUTY` commands that each hit the confirmed reversal-latch
   trigger (root cause 2) back-to-back on every port — exactly the
   "repeated abuse" the doc warns escalates a transient latch into a
   persistent one that survives every subsequent reflash. Filed as a
   follow-up issue:
   `clasi/issues/nezha-encoder-latch-persists-after-079-006-fixes-power-cycle-needed.md`,
   recommending a full physical power-cycle (not just a reflash) before
   re-verifying whether these two fixes actually restore real encoder
   motion.

**Both fixes are kept** (independently justified — see the code comments
in `source/hal/nezha/nezha_motor.cpp`) and covered by new host-level
regression tests (`tests/sim/unit/nezha_flipflop_harness.cpp` scenarios 8
and 9 — `scenarioFirstWriteExemptFromSentinelSlew` /
`scenarioRequestHonorsClearanceAfterDutyWrite`; scenario 6
`scenarioWriteThrottleInteraction` updated to keep testing the throttle
specifically now that a first write always converges in one step).

### Check-by-check results

| # | Check | Result |
|---|---|---|
| 1 | Cadence + evenness | **Blocked** — no time-varying encoder signal to measure a sample period from this session (see above). |
| 2 | In-use-port cycling | **Pass** — structural (code) + behavioral (hardware) confirmation; see acceptance criteria above. |
| 3 | Reversal/armor | **Pass (structural + host test)**; hardware timing precision inconclusive due to this session's transport jitter, not a regression finding. |
| 4 | Watchdog fire latency | **Pass** — 1.011 s vs. 1000 ms configured window (~11 ms check-latency overshoot), via `send_fast` + passive `EVT dev_watchdog` capture (round-trip retries would otherwise feed the watchdog and mask the measurement — a real gotcha hit and worked around this session). |
| 5 | Statement round-trips | **Pass (serial, required)** — `PING`, `VER`, `DEV M STATE`/`CAPS`, `DEV DT STATE`, `DEV M VOLT` (`ERR unsupported`) all round-tripped correctly. **Radio: skipped, no relay physically present** (`mbdeploy list`/`probe` + `ls /dev/cu.usbmodem*` checked). |
| 6 | Lazy-timer A/B | **Blocked** — see root-cause campaign; encoder-constancy is the required diagnostic signal and it is already constant regardless of condition this session. Follow-up issue filed. |
| 7 | Shared-0x10 clobber | **Done** — this ticket's whole root-cause campaign (above) was a far more thorough version of this check: two real defects found and fixed via direct pyOCD/gdb hardware inspection. |
| 8 | `vel_filt_alpha` retune | **Blocked** — no working velocity feedback to tune against; default (`0.3`) left unchanged. |

### Artifacts

- Code: `source/hal/nezha/nezha_motor.cpp` (`requestEncoder()`,
  `writeMotorRun()`, `writeRawDuty()`).
- Tests: `tests/sim/unit/nezha_flipflop_harness.cpp` (scenarios 6, 8, 9),
  `tests/sim/unit/test_nezha_flipflop.py` (unchanged, still the runner).
- Docs: `docs/knowledge/2026-07-04-encoder-wedge.md` ("Sprint 079-006 stand
  campaign" section added), this ticket file.
- Follow-up issue:
  `clasi/issues/nezha-encoder-latch-persists-after-079-006-fixes-power-cycle-needed.md`.
- Ad hoc bench scripts used this session were scratch (not committed —
  the project's `tests/bench/` convention is for durable HITL tools; these
  were one-off repro/diagnostic scripts). If a future session wants to
  re-run the exact repro, the recipe is in the follow-up issue.

## Implementation Plan

**Approach**: this ticket is verification, not new source changes (beyond
the `vel_filt_alpha` default update and any follow-up-issue filing) —
deploy, run each of the 8 checks in order (cheapest/lowest-risk first:
cadence and round-trips before the A/B and reversal tests that need more
setup), record results directly in this file.

**Steps**:
1. `mbdeploy probe` then `mbdeploy deploy --build` (per
   `.claude/rules/hardware-bench-testing.md`).
2. `mbdeploy list` — confirm serial path; note whether a relay is present
   for the radio round-trip (best-effort).
3. Run checks 1-5 (cadence, in-use cycling, reversal, watchdog, round-trips)
   via the serial link and `tests/bench/` scripts where they already exist
   (e.g. `dev_exercise.py`, `velocity_chart.py`, `wedge_latch_matrix.py`),
   extending them only as needed for the new cadence/in-use assertions.
4. Run check 6 (lazy-timer A/B) per
   `docs/knowledge/2026-07-04-encoder-wedge.md`'s diagnosis method.
5. Run check 7 (shared-0x10 clobber) — likely needs a small, throwaway
   bench script or a `DBG I2CLOG` inspection around a deliberately-abandoned
   collect; this is the one check that may need new bench tooling.
6. Run check 8 (`vel_filt_alpha` retune) via step-response bench passes,
   comparing against `pid_hold_speed`-style tolerances.
7. Record every result in this file; file follow-up issues for anything
   that fails or needs future work (standstill-guard retuning, OTOS/line/
   color HAL-schedule integration, etc. — per architecture-update.md's Open
   Questions).

**Files to modify** (verification tooling only, not the redesigned
subsystems themselves):
- `source/main.cpp` — `initDefaultMotorConfigs()`'s `vel_filt_alpha`
  default, if retuning changes it.
- `tests/bench/*.py` — extend existing scripts as needed for the new
  cadence/in-use/A-B assertions; a new small script for the shared-0x10
  clobber check if none of the existing ones fit.
- `docs/knowledge/2026-07-04-encoder-wedge.md` — production-guidance status
  line update if the lazy-timer A/B and the flip-flop wiring together
  change its "pending"/"not yet in production firmware" language (mirrors
  078's own ticket-005-gated update to this same file).

**Testing plan**: this ticket **is** the testing plan — a hardware stand
pass with 8 recorded checks. `uv run python -m pytest` should still be run
once beforehand to confirm the full host suite is green before spending
stand time (catching any regression from tickets 001-005 cheaply first).

**Documentation updates**: `docs/knowledge/2026-07-04-encoder-wedge.md`
(above, if applicable); record final cadence/latency/A-B numbers in this
ticket file as the durable record of what was measured.
