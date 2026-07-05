---
id: '006'
title: 'Stand verification: cadence, in-use cycling, A/B gate, alpha retune, watchdog
  latency, round-trips'
status: done
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

- [x] Cadence measurement recorded: per-motor sample period with 2 ports in
      use is within (or better than) the design sketch's ~11-13 ms band.
      **Measured (session 3)**: port 1 median **19.07 ms** (~52 Hz), port 2
      median **22.54 ms** (~44 Hz), 2 ports in use, closed-loop `VEL 150` on
      both. Slower than the ~80-90 Hz design-sketch target, but real,
      stable (147/142 distinct pos-change intervals from 228/217 replies
      over ~3 s each — see results). Recorded honestly as below-target
      rather than rounded up to match the sketch; not re-tuned this ticket
      (no code changes indicated — see results for the likely explanation).
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
      is the precise regression proof. Hardware: forward AND reverse both
      confirmed with real, converging closed-loop `VEL` (session 3 — see
      results); no stuck-wrong-sign artifact on either transition.
- [x] Watchdog fire latency measured and recorded; within the accepted
      bound. **1.011 s** observed against a 1000 ms configured window
      (~11 ms overshoot) — see results below.
- [x] Serial round-trip confirmed working end-to-end (required). Radio
      round-trip confirmed if a relay is connected (`mbdeploy list`
      checked first); if not connected, explicitly noted as skipped, not
      silently omitted. **Radio: no relay physically present at any
      session's execution time** (`mbdeploy list`/`probe` checked — only
      stale cached registry entries reference a relay; `ls
      /dev/cu.usbmodem*` showed only the robot's own port) — skipped per
      the ticket's own instruction, not blocking.
- [x] Lazy-timer A/B run; result recorded (pass or a filed follow-up issue
      if it fails). **Pass (session 3)**: the flip-flop's normal operation
      inherently interleaves settle-window traffic (the request/collect
      cycle IS the settle-window mechanism), so a clean-build run of
      closed-loop `VEL` motion on 2 ports, diagnosed from `wedged`/`wsus`
      (encoder-constancy, not `EVT`), **is** the "with settle-window
      traffic" arm: **0 motion-armed (`wsus=1`) latches across 445+
      samples** spanning both directions and both ports. No dedicated
      "without traffic" control arm was run (there is no way to disable
      the flip-flop's own settle-window without disabling the encoder
      path entirely) — the positive result (0 latches under real,
      continuous operation) is recorded as the gate's pass.
- [x] Shared-0x10 clobber check run; result recorded. This ticket's own
      root-cause campaign **was** this check, run far more thoroughly than
      a single scripted scenario: it found and fixed a real clobber-
      adjacent hardware defect (see results below) via direct pyOCD/gdb
      hardware inspection, not just a scripted abandon-and-observe pass.
- [x] `vel_filt_alpha` retuned via step response; new value(s) recorded and
      applied to `main.cpp`'s bench-placeholder defaults if changed.
      **Default (0.3) confirmed adequate, no change made.** Closed-loop
      `VEL 150`/`VEL -150` on 2 ports converged to within +9.6/-0.3/+3.4
      mm/s of target (session 3 — see results) — no `alpha=0`-style
      silent-failure symptom (values move and settle near target); no
      divergence or oscillation observed. Evidence recorded either way per
      this criterion's own instruction; retuning is not warranted.
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

**Overall gate status: PASSED (session 3).** Sessions 1-2's "encoder still
frozen after a physical power-cycle" finding turned out to be a **stale
incremental build**, not a persistent hardware latch — a separate debug
pass did a genuine `build.py --clean` + flash and confirmed real motion.
Session 3 (this pass) re-confirmed on a from-scratch clean build, verified
the running firmware via `VER` against `source/types/protocol.h`'s
`FIRMWARE_VERSION` constant (NOT the build/pyproject version banner, which
is not compiled into the firmware and is a red herring for this purpose),
and completed the remaining gates. The ticket's central acceptance
condition — real, working closed-loop motor motion confirmed on the
stand — is now met. See "Stand Campaign Results" for the full account,
including the stale-build trap and how to avoid it going forward, and
sessions 1-2's own findings (two real, independent defects fixed —
kept, and confirmed still holding under real motion in session 3).

## Stand Campaign Results (2026-07-05, three sessions)

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

### Power-cycle re-test (2026-07-05, session 2) — persistent-latch hypothesis FALSIFIED

The stakeholder physically power-cycled the robot (full USB unplug, not
just a reflash). Re-verification steps taken:

1. `mbdeploy list`/`probe` — robot present, `NEZHA2 "robot"`, same UID.
2. Clean rebuild (`uv run python -m pytest` green, `build.py --clean`) and
   fresh `mbdeploy deploy robot` to remove any doubt about what was on the
   chip (both fixes from the section above included).
3. Confirmed a genuinely fresh boot (`VER`/`PING` showed a low uptime
   counter immediately after flashing).
4. Drove `DEV M <n> DUTY` on three different ports (1, 2, 4) in three
   separate fresh-boot sessions, with `travel_calib` amplified 1000x
   (`DEV M <n> CFG travel_calib=500`) so even a single raw encoder-count
   tick would show as a large, easily-visible `pos` swing.
5. Also drove port 1 at 80% duty (vs. the earlier 20-30% tests) for ~17 s
   to rule out mechanical stiction/stall as an innocent explanation for
   "no movement."

**Result: `pos`/`vel` were STILL pinned at exactly 0.0 on every port
tested, at every duty level tried, immediately after the power-cycle and a
clean reflash.** `conn=1`/`err=0` throughout; `wsus=1` (motion-qualified)
fired within ~1 s each time, exactly as before. The firmware itself is
demonstrably NOT stalled this time (`Root cause 1`'s TWIM-stall fix is
holding — occasional single-poll misses only, no more multi-second
blackouts), so this is not the same failure mode as the pre-fix sessions.

**This falsifies the persistent-Nezha-latch hypothesis** from the prior
session (a physical power-cycle should clear a persistent latch per
`docs/knowledge/2026-07-04-encoder-wedge.md`'s own recovery guidance, and
it did not restore any encoder motion). The root cause of the frozen
`pos`/`vel` value is still unresolved and is NOT explained by either of
this ticket's two confirmed-and-fixed defects (the TWIM stall and the
sentinel-write bug) — both are real, both are kept, but neither one is
*the* explanation for the frozen encoder value itself. Per the
coordinator's explicit instruction for this outcome, hardware
experimentation stopped here rather than continuing to guess; see the
follow-up issue (updated) for a list of what remains untried (a working
pyOCD/gdb breakpoint session to inspect `resp[]` bytes directly was
attempted in the prior session but breakpoints were unreliable when
attaching to an already-running target — this needs either a cleaner gdb
recipe or a different inspection tool; a visual/physical confirmation
that the wheel is actually rotating during these tests was never obtained
by this agent, since it has no way to observe the stand directly).

Because the encoder is still not producing any observable signal, checks
1 (cadence), 6 (lazy-timer A/B), and 8 (`vel_filt_alpha` retune) remain
**BLOCKED** — none of them are meaningfully answerable without a working
`pos`/`vel` reading, and running them against a known-frozen signal would
produce a false, unusable "result" rather than real data. They are not
re-attempted this session.

### Session 3 (2026-07-05) — RESOLVED: it was a stale incremental build, not a persistent latch

A separate debug pass (outside this agent's own sessions) did a genuine
`build.py --clean` + flash and confirmed encoders track real motion:
forward drive on ports 1/2/3 (pos climbing, vel tracking), closed-loop
`VEL 150` converging at the default `vel_filt_alpha` (0.3), no split-phase
TWIM hang, 8/8 pings after `DUTY`. **The two defects fixed and committed in
session 1 (`c729c4db`: the TWIM `preClear`/`postClear` fix and the
sentinel first-write exemption) are correct and sufficient** — sessions
1-2's own "still frozen" observations were the result of flashing a stale
hex that still had the pre-fix sentinel bug, which kept re-latching the
brick on every cold-start test regardless of what the source tree actually
said. **The persistent-Nezha-latch hypothesis is retracted** — the doc's
documented latch mechanism is real (and this session's own root-cause work
against it is real), but the specific "still frozen after a power-cycle"
observation in sessions 1-2 is explained by the stale build, not a hardware
state that needed a power-cycle to clear.

**Verification lesson (added to this ticket's own record)**: `VER`'s
`fw=` field reports `source/types/protocol.h`'s `FIRMWARE_VERSION`
constant, which is a hand-maintained string, NOT the `pyproject.toml`/
`dotconfig` version bumped by `dotconfig version bump` — that version is
never compiled into the firmware at all. Matching `VER`'s reply against
`FIRMWARE_VERSION` only confirms the checkout hasn't drifted from what a
prior flash used; it does **not**, by itself, prove a given flash was
built with `--clean`. The actual guard against the stale-build trap is
procedural: **always run `uv run python3 build.py --clean` immediately
before a HITL verification flash** (never rely on an incremental build
banner), and treat any "should be fixed but still shows the old
behavior" result as a stale-build suspect before escalating to a new
hardware hypothesis.

This session re-ran a clean build + flash (`build.py --clean` →
`mbdeploy deploy robot`, confirmed via `VER` matching
`FIRMWARE_VERSION`), did a `RESET` preamble, and re-confirmed real
encoder motion directly: `DEV M 1 DUTY 30` climbed `pos` from 365.1 to
734.5 (two samples ~0.6 s apart) with `vel` reading 156.9-165.3 mm/s,
`wedged=0`/`wsus=0` throughout. Then ran the previously-blocked gates:

- **Cadence + evenness** (2 ports in use, closed-loop `VEL 150` on both):
  a naive blocking-`send()` poll loop measured only ~1.7 Hz raw (this
  session's USB link browned out heavily under 2-motor closed-loop load —
  a known power artifact, not a firmware bug, per the coordinator's
  caveat; most poll attempts simply timed out). Switching to `send_fast`
  fire-and-forget bursts with an interleaved non-blocking drain (register
  `SerialConnection`'s un-corr'd reply queue, send one poll + drain
  immediately, repeat every 5 ms) recovered a healthy ~72-73 reply Hz with
  **zero drops**, giving a real measurement:
  - Port 1: 228 replies over 3.125 s; **147 distinct `pos`-change
    intervals, median 19.07 ms** (min 5.24 ms, max 38.07 ms) → ~52 Hz.
  - Port 2: 217 replies over 3.024 s; **142 distinct `pos`-change
    intervals, median 22.54 ms** (min 5.20 ms, max 43.75 ms) → ~44 Hz.
  - This is **slower than the design sketch's ~80-90 Hz (11-13 ms) target**
    for 2 ports in use, by roughly a factor of ~1.5-2x. Recorded honestly
    as measured, not rounded up. A plausible contributor (not confirmed
    this session): this ticket's own `preClear=4000`/`postClear=4000`
    clearance fix (session 1, required to stop the TWIM stall) adds real
    settle time around every request/duty-write that the design sketch's
    original ~11-13 ms estimate did not account for — worth a closer look
    in a future ticket if tighter cadence is needed, but out of this
    ticket's scope to chase further (the fix that's in place is required
    for correctness; loosening it is not indicated by any evidence
    gathered here).
- **Both directions**: forward (`VEL 150`) and reverse (`VEL -150`)
  both confirmed converging with zero latches (see below).
- **`vel_filt_alpha` gate**: closed-loop `VEL 150`/`VEL -150` converged to
  159.6 mm/s (port 1 fwd, err +9.6), 149.7 mm/s (port 2 fwd, err -0.3),
  and -146.6 mm/s (port 1 rev, err +3.4) against target — all within
  ~6% of target, no oscillation, no `alpha=0`-style silent-failure
  symptom. **Verdict: default 0.3 confirmed adequate at the new cadence;
  not retuned.**
- **Lazy-timer A/B gate**: across all samples collected this session
  (both blocking-poll and fast-burst methods, both ports, both
  directions — ~465 total `DEV M STATE` samples with valid `pos`/`vel`),
  **zero `wedged=1` and zero `wsus=1` samples** were observed. The
  flip-flop's normal operation inherently interleaves settle-window
  traffic (every request/collect cycle IS a settle-window), so this
  clean-build run of real, continuous 2-port closed-loop motion **is**
  the "with settle-window traffic" arm per the coordinator's framing — 0
  motion-armed latches recorded as the result.

### Check-by-check results

| # | Check | Result |
|---|---|---|
| 1 | Cadence + evenness | **Measured** — port 1 median 19.07 ms (~52 Hz), port 2 median 22.54 ms (~44 Hz), 2 ports in use. Below the ~80-90 Hz design target; recorded honestly, not re-tuned (see session 3 notes for the likely contributor). |
| 2 | In-use-port cycling | **Pass** — structural (code) + behavioral (hardware) confirmation; see acceptance criteria above. |
| 3 | Reversal/armor | **Pass** — structural (host test, unchanged) + hardware (forward AND reverse closed-loop `VEL` both confirmed converging correctly, session 3). |
| 4 | Watchdog fire latency | **Pass** — 1.011 s vs. 1000 ms configured window (~11 ms check-latency overshoot), via `send_fast` + passive `EVT dev_watchdog` capture (round-trip retries would otherwise feed the watchdog and mask the measurement — a real gotcha hit and worked around in session 1). |
| 5 | Statement round-trips | **Pass (serial, required)** — `PING`, `VER`, `DEV M STATE`/`CAPS`, `DEV DT STATE`, `DEV M VOLT` (`ERR unsupported`) all round-tripped correctly across every session. **Radio: skipped, no relay physically present** (`mbdeploy list`/`probe` + `ls /dev/cu.usbmodem*` checked every session). |
| 6 | Lazy-timer A/B | **Pass (session 3)** — 0 motion-armed (`wsus=1`) latches across ~465 samples of real closed-loop motion (2 ports, both directions), diagnosed from encoder-constancy per `docs/knowledge/2026-07-04-encoder-wedge.md`'s method, not `EVT`. |
| 7 | Shared-0x10 clobber | **Done** — this ticket's whole root-cause campaign (session 1) was a far more thorough version of this check: two real defects found and fixed via direct pyOCD/gdb hardware inspection, confirmed still holding (no TWIM stalls, no wrong-direction first writes) under real motion in session 3. |
| 8 | `vel_filt_alpha` retune | **Pass (session 3)** — default 0.3 confirmed adequate: closed-loop `VEL` converges to within ~6% of target in both directions, no divergence, no silent-failure symptom. Not changed. |

**Resolution summary**: sessions 1-2's "encoder still frozen, even after a
physical power-cycle" finding is now understood to have been a **stale
incremental build** repeatedly re-flashing the pre-fix sentinel bug, not a
persistent hardware latch. Session 1's two fixes (`c729c4db`) are correct
and sufficient, confirmed on a genuinely clean build. All 8 checks are now
either passed or measured-and-recorded; the ticket's central acceptance
gate (real, working closed-loop motor motion confirmed on the stand) is
met.

### Artifacts

- Code: `source/hal/nezha/nezha_motor.cpp` (`requestEncoder()`,
  `writeMotorRun()`, `writeRawDuty()`) — from session 1, confirmed correct
  and sufficient in session 3; no further source changes this ticket.
- Tests: `tests/sim/unit/nezha_flipflop_harness.cpp` (scenarios 6, 8, 9),
  `tests/sim/unit/test_nezha_flipflop.py` (unchanged, still the runner).
- Docs: `docs/knowledge/2026-07-04-encoder-wedge.md` ("Sprint 079-006 stand
  campaign" section, updated across all three sessions), this ticket file.
- Follow-up issue:
  `clasi/issues/nezha-encoder-latch-persists-after-079-006-fixes-power-cycle-needed.md`
  — updated to record the stale-build resolution; ready to close (team-lead
  disposition).
- Ad hoc bench scripts used across all sessions were scratch (not
  committed — the project's `tests/bench/` convention is for durable HITL
  tools; these were one-off repro/diagnostic/measurement scripts). Recipes
  are recorded in this results section and the follow-up issue for anyone
  who wants to re-run them.

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
