---
status: in-progress
sprint: '118'
tickets:
- 118-001
---

# Restore the interleaved request→settle→tick loop schedule

## Description

`RobotLoop::cycle()` in `src/firm/app/robot_loop.cpp` no longer matches the
run-and-wait design. The intended per-cycle schedule for each motor is:

```
motorX_.requestSample()          // 0x46 select write, kicks the async encoder read
runAndWait(kSettle, { ...work })  // >=4ms encoder settle, dead bus time BORROWED for
                                  //   non-bus work (comms pump / command dispatch)
motorX_.tick(now)                 // collect the sample -> velocity PID -> duty write
```

`kSettle` is specifically the **encoder-settle window between a motor's own
request and its collect** — not a comms-pump gap. Today it is misapplied: the
two `runAndWait(kSettle, …)` blocks wrap `comms_.pump` / `processMessage` while
`requestSample()` and `tick()` sit adjacent above them, so no borrowed-work
window separates a request from its collect.

Target = the last-correct skeleton (commit `39c084c1`, 2026-07-18), carrying
today's richer block bodies (StateEstimator, MoveQueue, `updateLineColor`,
frame-v2 TLM) unchanged:

```
cycleStart = markTime(); Cmd cmd

motorL_.requestSample()                                   // 0x46 select port 1
runAndWait(kSettle, { comms_.pump(cmd, cycleStart) })     // >=4ms L settle (borrowed)
motorL_.tick(now)                                         // collect L -> PID -> duty

runAndWait(kClear, { updateTlm(cycleStart); tlm_.emit(cycleStart) })   // >=4ms duty clear

motorR_.requestSample()                                   // 0x46 select port 2
runAndWait(kSettle, {                                     // >=4ms R settle (borrowed)
    processMessage(cmd)
    moveResult = moveQueue_.tick(now, odom_)              // + fault flag + completion ack
    drive_.tick()                                         // twist -> wheel targets
})
motorR_.tick(now)                                         // collect R -> PID -> duty

runAndWait(kPace, {                                       // perception + odometry + pace
    applyOtosSample(...); odom_.integrate(); frame_.pose = {...}
    stateEstimator_.update(...); updateLineColor(now)
})
```

Per-port interleave is preserved (select L → collect L → select R → collect R),
so the `0x46` single-latched-select invariant still holds. `drive_.tick()` is
pure computation (no bus), so it is legal borrowed work inside the R-settle
block. The resulting timing asymmetry is the deliberate `39c084c1` baseline:
`motorL_.tick()` writes the target staged by *last* cycle's `drive_.tick()`
(L: −1 cycle); `motorR_.tick()` writes this cycle's.

## Cause

Introduced by commit `5f5a2ba7` ("Refactor motor interface…", 2026-07-18): it
collapsed `requestSample()`/`tick()` adjacent per motor, pushed `comms_.pump`
into a `runAndWait(kSettle)` block placed *after* both collects, and zeroed the
schedule (`kSettle 4→0`, `kClear 4→0`, `kCycle 40→20`). Commit `c75f528e`
(2026-07-20) then hoisted `drive_.tick()` above both motor ticks (the "112-005"
cycle-order experiment).

The impact is not cosmetic: with request/collect adjacent and `kSettle=0`, the
vendor 4ms settle is still enforced — but as a *blocking*
`MicroBitI2CBus::waitForClearance()` sleep *inside* `motorL_.tick()`/
`motorR_.tick()`. That (a) trips the I2C clearance safety-net fault bit
(telemetry fault bit 0) every cycle, and (b) leaks the settle time *outside* the
pace schedule, so the advertised cadence is fiction.

The `runAndWait` design was introduced 2026-07-14 (sprint 103-008, commit
`8da0cb62`) and was correct through `39c084c1`.

## Proposed fix

Stakeholder-confirmed decisions: restore `kCycle=40` (25Hz) together with
`kSettle=4`/`kClear=4` (full original 106-001 budget; `kPace = kCycle − kWindows
= 28ms`), and fully restore the `39c084c1` order — including moving
`drive_.tick()` back inside the R-settle block, which **retires the 112-005
hoist experiment.**

**1. `src/firm/app/robot_loop.cpp` (core change)**
- Constants (lines ~25–27): `kSettle 0→4`, `kClear 0→4`, `kCycle 20→40`.
  `kWindows`/`kPace`/`static_assert` are derived — no edit needed (12 ≤ 40).
- Rewrite the constants doc comment (lines ~13–24): it currently cites 115-005 /
  50Hz / `kPrimaryPeriod=20`; restore 106-001-era wording (~25Hz / ~40ms).
- Restructure `cycle()` (lines ~498–581): delete the hoisted `drive_.tick()` and
  the collapsed adjacent `requestSample`/`tick` block; relocate the four
  `requestSample`/`tick` calls to bracket the existing `runAndWait` blocks per
  the schedule above; add `drive_.tick()` to the end of the R-settle block body.
  Keep the existing block bodies and their detailed comments verbatim — only the
  call placement moves. Update the "Request/collect MUST interleave" comment to
  describe the restored interleaved schedule.
- The stale `// >=4ms:` call-site comments become accurate again (kSettle=4).

**2. `src/firm/app/telemetry.h` (coupled, recommended)**
- `kPrimaryPeriod 20→40` (line ~121) so it again equals `kCycle` — the original
  106-001 coupling. Primary still emits every cycle, now at 25Hz. If left at 20,
  primary still emits every cycle at 25Hz, but the "matches kCycle" comment goes
  stale and the telemetry harness needs no clock-advance change.

**3. Design docs (cadence numbers only)**
- `src/firm/app/DESIGN.md` §4 (lines ~238–244): `20ms / ~50Hz` → `40ms / ~25Hz`.
- `docs/design/design.md` (line ~320): `kCycle = 20 ms (~50 Hz)` → `40 ms
  (~25 Hz)`.

**4. Test / harness impact**
- `src/tests/sim/unit/app_robot_loop_harness.cpp`: its comments already describe
  the interleaved order with `drive_.tick()` *between* the motor ticks — the
  restore should make code and harness agree again. Run it; fix expected-value
  drift.
- `src/tests/sim/unit/app_telemetry_harness.cpp`: advances the fake clock
  ~20ms/cycle to cross `kPrimaryPeriod`. If `kPrimaryPeriod→40`, bump that
  advance to ~40ms/cycle or primary-emission assertions break.
- Run and fix if cadence-sensitive: `straight_twist_harness.cpp`,
  `state_estimator_tracking_harness.cpp`, `devices_motor_harness.cpp`,
  `plant_harness.cpp`.

## Verification

**Sim first:** build firmware and run the affected harnesses (project firmware
build/test path). All must pass, including the updated `app_robot_loop_harness`
and `app_telemetry_harness`.

**Bench gate (required — touches HAL/motor/loop timing per
`.claude/rules/hardware-bench-testing.md`; robot on the stand, wheels free):**
1. Deploy to the robot (`mbdeploy deploy`; note the `--build` caveat — prefer
   `just build` then deploy the hex).
2. Drive both directions; confirm encoders increment in the right direction and
   roughly proportional to command.
3. Confirm TLM velocity reads are plausible and non-zero while driving.
4. Before/after signal: confirm the I2C clearance safety-net fault bit
   (telemetry fault bit 0) is now clear while driving — it was tripped every
   cycle by the blocking in-`tick()` settle.
5. Measure the real cycle period from TLM timestamps (~40ms / ~25Hz expected)
   and confirm comms/radio stay responsive (comms pump now runs in the borrowed
   settle window).

## Related

- Regression chain: `8da0cb62` (introduced, correct) → `39c084c1` (last correct)
  → `5f5a2ba7` (regressed order + zeroed constants) → `c75f528e` (drive_.tick
  hoist).
- `clasi/issues/later/kcycle-kprimaryperiod-mismatch.md` tracked the
  `kSettle=kClear=0 / kCycle=20` drift — this change resolves it; candidate to
  move to done.
- Project memory `robot-loop-reorder-is-live-experiment` says keep the hoist;
  this decision retires it — update/delete that memory when implemented.
