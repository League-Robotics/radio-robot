---
id: '007'
title: 'HITL validation: bench scripts and stand/coupled-rig verification'
status: in-progress
use-cases:
- SUC-008
depends-on:
- '006'
github-issue: ''
issue: greenfield-rebuild-faceplate-hal-in-a-fresh-source-old-tree-parked.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# HITL validation: bench scripts and stand/coupled-rig verification

## Description

Run the sprint's exit gate for real: build, flash, and exercise the new
dev-loop firmware on the stand (robot `tovez`, four motors wired) and on the
coupled two-motor bench rig (ports 3 and 4, mechanically linked). This
ticket is the sprint's acceptance gate — per
`.claude/rules/hardware-bench-testing.md`, "a sprint is not 'done' on tests
alone — it must be seen working on the stand." No new source code is
expected from this ticket; if the bench pass surfaces a real bug in tickets
1-6's work, fix it in this ticket (and note which prior ticket's acceptance
criteria the fix corresponds to) rather than deferring — this is the last
ticket in the sprint.

## Acceptance Criteria

All bullets below mirror the linked issue's "Verification" section
verbatim; each must be observed and recorded (log output, TLM excerpt, or
screen capture as appropriate), not just asserted.

- [x] **Build**: `python build.py --clean` produces the new hex;
      `source_old` is untouched (confirm via `git status`); rollback
      (`codal.json` `application: source_old` + rebuild) still works.
- [x] **Flash**: `mbdeploy deploy robot --hex …` with the ROLE check passing
      (never a blind `cp` to `/MICROBIT` — see
      `.clasi/knowledge/verify-microbit-before-flashing.md`).
- [x] **Bench, single motor** (robot on the stand, wheels free):
  - `DEV M 1 DUTY 30` → wheel spins; `DEV M 1 STATE` reports `applied=0.30`
    and `position` climbing.
  - `DEV M 1 VEL 120` → converges; capture applied-duty-vs-measured-velocity
    over the step response (sanity check for the embedded PID — no
    formal tolerance required, just a plausible converging step response).
  - `DEV M 1 VOLT 3` → `ERR unsupported`.
  - `DEV M 1 RESET` → position rezeroes.
- [ ] **Bench, drivetrain**:
  - `DEV DT VW 150 0 0` → both wheels approximately equal. **Verified**
    (see results below).
  - Hand-drag one wheel → **both** wheels slow, ratio held (observable via
    `DEV DT STATE`) — the governor is doing something, not coasting.
    **NOT performed** — requires a human physically loading a wheel while
    the script polls; no human was in the loop during this agent-run
    session. Needs a follow-up HITL pass with an operator present.
- [x] **Watchdog**: stop sending commands mid-motion → motors reach neutral
      within the configured window (default ~1 s).
- [ ] **Host script — `dev_exercise.py`**: scripts the above sequence over
      `NezhaProtocol.send()`, run once over direct serial and once over the
      relay's `!GO` data plane; both pass. **Direct serial: PASS (19/19,
      see results)**. **Relay path: NOT run** — no radio relay dongle was
      part of this session's bench setup (direct-USB only, per the
      dispatching prompt); needs a follow-up run through the relay.
- [ ] **Interactive — `velocity_chart.py`**: run live while hand-loading
      wheels; visually confirm the wheel-velocity/applied-duty panels track
      real behavior and the vR-vs-vL phase plot shows the ratio governor's
      diagonal. Used at this step to tune the in-motor PID gains if the step
      response from the single-motor bullet above looked implausible —
      record any gain changes made and where (`MotorConfig.vel_gains`
      defaults) they should be persisted for future bench sessions.
      **NOT performed** — this is an interactive matplotlib GUI tool needing
      a human at the keyboard/hand-loading wheels; PID gains WERE tuned and
      persisted (see results) via direct scripted bench trials instead of
      this tool, satisfying the gain-tuning intent without the visual tool.
- [ ] **Coupled-rig acceptance** (ports 3 and 4, mechanically linked — running
      one loads the other):
  - `pid_hold_speed.py` PASS: motor-3 measured velocity stays inside a
    tolerance band and recovers within a bounded settle time after each load
    step (assist → freewheel → drag → reverse on motor 4), with applied
    duty visibly rising as load increases.
  - `ratio_governor_curve.py` PASS: with `DEV DT PORTS 3 4` and an unequal
    wheel-target curve, the governor lowers BOTH targets so the measured
    wheel-speed ratio holds the commanded ratio within tolerance; re-run
    with the governor off (`sync_gain=0`) and confirm the ratio visibly
    drifts (the required negative control).
  - **BLOCKED — physical coupling is absent on this rig.** Per the
    dispatching prompt's explicit instruction ("verify this empirically
    early... report if the linkage is absent... do not fake the rig
    tests"), this was checked first: `DEV M 4 DUTY 40` (and repeated at
    ±20%/±30%/±40%) while polling `DEV M 3 STATE` showed motor 3's position
    completely static (`pos=-0.1`/`0.0` unchanged, `wedged=1` throughout)
    while motor 4 span freely. Motor 3 DOES move correctly under its own
    direct `DEV M 3 DUTY 30` command, so its motor/encoder are healthy in
    isolation — there is simply no mechanical link transmitting load between
    ports 3 and 4 right now. Both scripts were still run for the record (see
    results) but their outcomes are **not evidence of the governor/PID
    working under real load** — only that each wheel independently tracks
    its own target when unloaded. **This bullet cannot be checked off
    without either a hardware fix (verify/restore the physical coupling) or
    a stakeholder decision to accept this gap.**
- [x] Any defect found in tickets 1-6's work during this bench pass is fixed
      in this ticket, with a note identifying which prior ticket's
      acceptance criterion was not actually met and why the bench pass
      caught it where the build-only gate did not. **Two defects found and
      fixed — see results.**

## Results (2026-07-04 HITL session)

Robot: `robot` (NEZHA2) on `/dev/cu.usbmodem2121102`, four motors attached,
on the stand. Direct USB only — no relay dongle in this session's setup.

### Prerequisite fix — `DEV DT CFG`

Gap from ticket 006: `DrivetrainConfig.sync_gain` booted at `0` (governor
OFF) with no live setter. Added `DEV DT CFG k=v ...` (`sync_gain`,
`trackwidth`) to `source/commands/dev_commands.{h,cpp}` (`DtMode::CFG`,
`applyDrivetrainCfgKey`, `handleDevDtCfg`), a `drivetrainConfigShadow` field
on `DevLoopState`, seeded from `main.cpp`'s boot `dtConfig`. Boot default
for `sync_gain` intentionally left at `0` per the dispatching prompt ("keep
boot=0; scripts set it explicitly"). Documented in `docs/protocol-v2.md`
§16. Verified live:

```
DEV DT CFG sync_gain=0.5
OK DEV DT sync_gain=0.500
DEV DT CFG trackwidth=200
OK DEV DT trackwidth=200.0
DEV DT CFG badkey=1
ERR badkey badkey
DEV DT CFG sync_gain=0.8 trackwidth=128
OK DEV DT sync_gain=0.800 trackwidth=128.0
```

`tests/bench/ratio_governor_curve.py`'s `--sync-gain` now actually sends
this command (was label-only) and echoes the firmware-confirmed applied
value.

### Build / flash / rollback

`python build.py --clean` → `MICROBIT.hex` (v0.20260704.11 label;
`FIRMWARE_VERSION` constant in `source/types/protocol.h` independently
reports `0.20260704.6` — a separate, not-kept-in-lockstep version string;
not a staleness bug, confirmed by directly exercising the new `DEV DT CFG`
verb on the just-flashed image). `git status` confirms `source_old/`
untouched throughout. Rollback verified: flipped `codal.json`
`"application"` to `"source_old"`, `build.py --clean` succeeded (202 KB
flash image, the full legacy firmware), flipped back to `"source"` and
rebuilt/reflashed the dev tree — `git diff codal.json` is clean against
HEAD. `mbdeploy list` confirmed ROLE=NEZHA2 before every flash (never a
blind copy). `mbdeploy deploy robot --hex MICROBIT.hex` needed its
automatic CTRL-AP mass-erase recovery on 3 of 4 flashes in this session
(pre-existing, documented `mbdeploy` behavior, not a new issue).

### `dev_exercise.py` — direct serial: PASS (19/19)

First pass (before defect fixes below) was 10/19 and 18/19 across two
runs, entirely from transport noise (see Defect 2) plus the expected
PID-gain gap (Defect 1) — not real firmware failures. After both fixes,
reflashed, reran cleanly:

```
19/19 checks passed
```

Key lines: `DEV M 1 DUTY 30` → `applied=0.30`, position `pos0=0.0
pos1=237.0 delta=237.0` (climbed). `DEV M 1 VEL 120` → converged to
`vel=117.1`/`vel=117.2` (tolerance 25). `DEV M 1 VOLT 3` →
`ERR unsupported volt`. `RESET` → `pos_after_reset=0.0`. Watchdog: `EVT
dev_watchdog` observed, `applied_after=0.0`.

### Coupled-linkage verification (ports 3/4) — LINKAGE ABSENT

Per the dispatching prompt's explicit early-check instruction. `DEV M 4
DUTY 40` (and repeated at ±20%) while polling `DEV M 3 STATE`:

```
baseline port3: OK DEV M 3 pos=0.0  ...
driving port 4 at DUTY 40 ...
  t=0.2s  port3=OK DEV M 3 pos=-0.1 ...   port4=OK DEV M 4 pos=1126.2 ...
  t=2.8s  port3=OK DEV M 3 pos=-0.1 ...   port4=OK DEV M 4 pos=3794.7 ...
```

Motor 3's position never moved (`-0.1`, `wedged=1` throughout) across
±20/±30/±40% duty on motor 4 in both directions, while motor 4 spun freely.
Confirmed motor 3's own motor+encoder are healthy: `DEV M 3 DUTY 30` driven
directly climbed `pos` from `0.0` to `1438.5` over 1.4 s. **Conclusion: no
mechanical coupling is currently transmitting load between ports 3 and 4 on
this rig.** `pid_hold_speed.py`/`ratio_governor_curve.py` were still run
(below) for the record, but per the "do not fake the rig tests" instruction
their results are reported as informational, not as satisfying the
coupled-rig acceptance bullet.

### Bench, drivetrain (ports 1/2 — the real drive pair)

`DEV DT VW 150 0 0`: both wheels converged near the 150 mm/s target and
close to each other (M1 ≈142-157, M2 ≈135-155.7 mm/s) — the automatable
half of this bullet passes. Hand-drag (needs a human) not performed.

### `pid_hold_speed.py` / `ratio_governor_curve.py` — informational only (rig not coupled)

`pid_hold_speed.py --pid-port 3 --load-port 4`: FAIL overall (`velocity
held`: FAIL, `applied duty rose`: FAIL) — expected given no real load ever
reaches port 3; the "load" steps on port 4 don't perturb port 3 at all.
CSV: `tests/bench/out/pid_hold_speed.csv`.

`ratio_governor_curve.py --sync-gain 0` (negative control, `DEV DT PORTS 3
4`, curve 200/80 mm/s): measured ratio ≈2.5-2.6 vs commanded 2.5 — holds
fine even with the governor off, because there is no real disturbance to
drift under in the first place. This is consistent with (not contradicting)
the absent-coupling finding: an unloaded independent wheel tracks its own
target regardless of `sync_gain`. CSV:
`tests/bench/out/ratio_governor_curve.csv`.

### PID gain tuning (Defect 1) and persistence

**Defect 1 (found this bench pass): `MotorConfig.vel_filt_alpha` booted at
`0`.** This is the EMA coefficient in `NezhaMotor::tick()`'s
`filteredVelocity_ = a * rawVel + (1 - a) * filteredVelocity_` — `a = 0`
means `filteredVelocity_` **never incorporates a new sample**, so `DEV M
<n> STATE`'s `vel=` field reports exactly `0.0` forever regardless of real
motion. Confirmed live: under `DEV M 1 VEL 120`, `pos` climbed from `0.4`
to `1209.3` (clearly spinning) while `vel=0.0` the entire time; setting
`vel_filt_alpha=0.3` via `DEV M 1 CFG` on the SAME motor immediately
produced real, converging `vel=` readings. Ticket 3/4's build-only gate
could not have caught this (no bench pass ever read a live velocity value)
— exactly why this HITL pass exists. Not just kp/ki/kff at `0` (expected,
tunable) — the missing filter coefficient made closed-loop VEL control
silently unobservable/unusable on ANY port regardless of gains.

Tuned on the stand (ports 1 and 3, targets 120/150/-100 mm/s), final gains
persisted in `source/main.cpp`'s `initDefaultMotorConfigs()` (all 4 ports):

```
kp=0.0022  ki=0.0018  kff=0.0038  i_max=0.3  vel_filt_alpha=0.3
```

Step response (port 3, target 150): `t=1.48s vel=165.8`, `t=1.68s
vel=164.0`, `t=1.85s vel=166.4` — converges within ~1.5 s, ~10% overshoot,
settles near target. Port 1, target 120: `t=1.48s vel=119.6`, `t=2.01s
vel=116.4` — similarly fast/clean. Rebuilt + reflashed + reran
`dev_exercise.py` to confirm (19/19, above) — the persisted defaults now
converge correctly out of the box, not just under a live `DEV M CFG`
override.

### Defect 2 (found this bench pass): bench-script `dev_send()` had no retry against known transport noise

This bench's direct-USB CDC link measured occasionally, burstily dropping
replies outright — a bare `ECHO` loop saw miss streaks up to 3 in a row; a
`DEV M 1 STATE` loop with a 3.5 s per-call budget still saw one full drop
in 20 calls. This matches the already-documented
`.clasi/knowledge/radio-link-max-data-rate.md` finding ("direct USB...
intermittently drops 15-50% (NOT firmware)") — not a new environmental
issue, but ticket 6's `dev_exercise.py`/`pid_hold_speed.py`/
`ratio_governor_curve.py` all sent every command through a single-attempt
`dev_send()` with no retry, so a dropped reply on any one-shot check
(DUTY/VEL/RESET/VER/PING) produced a false FAIL indistinguishable from a
real firmware defect — ticket 6's acceptance ("the scripts function
correctly as a bench acceptance gate") wasn't actually met against real,
noisy hardware, only against the ideal case its self-tests exercise. Fixed
by adding a bounded retry (6 attempts, 100 ms spacing) to `dev_send()` in
all three PASS/FAIL-gated scripts — safe because every command they send
is either a pure query or an idempotent absolute-value write.
`velocity_chart.py`'s `dev_send()` was deliberately left as-is (it polls in
a tight, high-rate loop for live plotting; a dropped frame there is
cosmetic, and adding retry delay there would degrade its responsiveness for
no acceptance-gate benefit). A residual, longer (multi-second) stall
pattern remains occasionally observable beyond what 6 retries fully
absorbs (see `pid_hold_speed.py`/`ratio_governor_curve.py` CSVs above,
which still show some gaps) — this is the same pre-existing, documented
transport characteristic, not chased further here since the coupled-rig
tests are blocked by the absent physical linkage regardless of transport
reliability.

### Full regression

`uv run python -m pytest` → 1 passed (placeholder; the new tree's sim
harness doesn't exist yet, per `tests/CLAUDE.md` — expected, unchanged by
this ticket).

### Outstanding for ticket closure (needs a human / hardware decision)

1. **Physical coupling on ports 3/4 is absent** — verify/restore the
   mechanical link (belt/gear/shaft), or explicitly accept this gap, then
   re-run `pid_hold_speed.py` and `ratio_governor_curve.py --sync-gain
   0`/`--sync-gain 0.5-1.0` for the real coupled-rig acceptance evidence.
2. Hand-drag observation (drivetrain bullet + `velocity_chart.py`) needs an
   operator physically present — not performed by this agent session.
3. `dev_exercise.py` relay-path run (`!GO` data plane) not performed — no
   relay dongle in this session's bench setup.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (regression check —
  nothing in tickets 1-6 should have broken the new tree's pytest
  collection); all bench scripts from ticket 6 (`dev_exercise.py`,
  `pid_hold_speed.py`, `ratio_governor_curve.py`, `velocity_chart.py`).
- **New tests to write**: None expected — this ticket exercises what ticket
  6 built. If a bench-pass fix requires new coverage (e.g., a regression the
  bench pass caught that a unit test could have caught earlier), add it to
  `tests/unit/` and note why it wasn't in ticket 3/4's original scope.
- **Verification command**: `python build.py --clean`, `mbdeploy deploy
  robot --hex build/...`, then the manual/scripted bench sequence above. No
  single command captures the full gate — this ticket's acceptance is the
  full checklist, observed on real hardware.
