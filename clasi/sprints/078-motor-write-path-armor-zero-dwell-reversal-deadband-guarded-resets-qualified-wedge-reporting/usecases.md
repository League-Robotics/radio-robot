---
sprint: "078"
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 078 Use Cases

These are bench-operator and host-tooling use cases against the `ROBOT_DEV_BUILD`-only
`DEV` command family (`docs/protocol-v2.md` §16). No existing top-level use case in
`docs/usecases.md` covers dev-bench motor-safety tooling directly (that catalog
predates the new tree's `DEV` protocol and is itself stale in places — see
`architecture-update.md`'s Migration Concerns). Each use case below is parented to
the closest existing top-level use case by subject matter; SUC-005 has no
reasonable parent (it is developer-facing test infrastructure, not robot
behavior) and is left unparented.

## SUC-001: Hot-flip reversal soak completes with zero motion-armed latches
Parent: UC-004 (Stop Robot Immediately — closest existing UC for motor
write-path safety guarantees)

- **Actor**: Bench operator (human, HITL) running `tests/bench/`'s friction-rig
  soak script against a robot on the stand, rig ports 3/4 under mechanical
  friction load.
- **Preconditions**: Firmware built with the sprint's armor in place
  (`reversal_dwell` default 100 ms, `output_deadband` default 0.03); robot
  connected over USB or radio relay; `DEV WD` widened for the session;
  motors mechanically coupled per the rig's port-3/4 wiring.
- **Main Flow**:
  1. Operator starts the soak script, which commands ≥100 hot sign flips at
     ±30–50% duty on the loaded motor via `DEV M <n> DUTY`.
  2. Script polls `DEV M <n> STATE` after each flip, reading `pos=`,
     `applied=`, and the motion-qualified `wsus=` field (never the raw
     `wedged=`).
  3. Script logs each poll to a CSV and a transcript.
  4. On completion (or Ctrl-C, or an exception), the script sends `DEV STOP`
     and restores the boot-default watchdog window.
- **Postconditions**: Zero motion-armed latches (`wsus=1` observed at any
  point during the soak) over the full run; CSV + transcript saved;
  motors are neutral.
- **Acceptance Criteria**:
  - [ ] Soak runs ≥100 hot flips without a single `wsus=1` observation
        (`wedged=1` at rest between flips is expected/benign and does not
        fail the run).
  - [ ] CSV and transcript are written and retained.
  - [ ] Session ends with `DEV STOP` regardless of pass/fail/exception.

## SUC-002: Mid-motion RESET takes the soft path, never bursts the bus while moving
Parent: UC-005 (Query Encoder Positions — closest existing UC for encoder
reset/zero semantics)

- **Actor**: Bench operator / host tooling issuing `DEV M <n> RESET` while a
  motor is actively spinning.
- **Preconditions**: Motor commanded to a nonzero duty/velocity and observed
  moving (`vel=` nonzero on `STATE`).
- **Main Flow**:
  1. Operator/script reads `DEV M <n> STATE`, records `hrc=`/`src=` (hard/soft
     reset counts).
  2. Operator/script sends `DEV M <n> RESET` while the motor is still moving.
  3. Firmware accepts immediately (`OK DEV M <n> reset=1`) — the hard-vs-soft
     decision is made at the top of the next `tick()`, not synchronously with
     the reply.
  4. Operator/script polls `DEV M <n> STATE` again and compares `hrc=`/`src=`
     against the values recorded in step 1.
- **Postconditions**: `src=` incremented by exactly 1; `hrc=` unchanged;
  `pos=` reads ~0 on the next poll; no atomic 0x46 read-burst appears on the
  bus while the motor was moving (verifiable via `DBG I2CLOG` if needed).
- **Acceptance Criteria**:
  - [ ] `RESET` issued while `|vel|` is above the rest threshold always takes
        the soft path (`src=` increments, `hrc=` does not).
  - [ ] `pos=` reads within tolerance of 0 on the very next `STATE` poll,
        regardless of which path fired.
  - [ ] `RESET` issued while genuinely at rest (`vel=` ~0, `applied=` 0 for
        several consecutive polls) takes the hard path (`hrc=` increments).

## SUC-003: An idle, at-rest motor is never reported wedge-suspect
Parent: UC-005 (Query Encoder Positions — closest existing UC for `STATE`
polling semantics)

- **Actor**: Bench operator reading `DEV M <n> STATE` / `DEV STATE` on a
  motor that has been sitting idle (commanded and applied duty both 0) for
  longer than the raw stuck-encoder threshold.
- **Preconditions**: Motor idle (no `DUTY`/`VEL` command active, or duty
  commanded to and settled at 0) for ≥10 ticks (the raw detector's existing
  unconditional threshold).
- **Main Flow**:
  1. Operator polls `DEV M <n> STATE` repeatedly while the motor sits idle.
  2. Firmware's raw stuck-encoder counter latches (`wedged=1`) — expected,
     unconditional, unchanged from sprint 077.
  3. The motion-qualified counter does **not** latch, because
     `|appliedDuty()|` never exceeded `output_deadband` during the window.
- **Postconditions**: `STATE` reports `wedged=1` (benign, diagnostic) and
  `wsus=0` (not suspect) simultaneously for the same idle motor.
- **Acceptance Criteria**:
  - [ ] An idle motor eventually reports `wedged=1` (raw counter, unchanged
        semantics) but never `wsus=1` (motion-qualified) while it stays idle.
  - [ ] `docs/protocol-v2.md` documents this distinction explicitly so an
        operator reading a bench log does not mistake `wedged=1` on an idle
        motor for a real fault.

## SUC-004: Legacy A/B comparison via the `dwell` config knob
Parent: UC-014 (Tune Calibration Parameters at Runtime — closest existing UC
for runtime `CFG`-style tuning)

- **Actor**: Bench operator running a controlled A/B comparison to prove the
  armor is doing something on a given set of motors (per the knowledge doc's
  "always bracket with controls" discipline).
- **Preconditions**: A motor known or suspected to be latch-susceptible
  (e.g. the wedgelab's historical M1/M2 pair); rig set up per SUC-001.
- **Main Flow**:
  1. Operator sends `DEV M <n> CFG dwell=0` (explicit legacy: dwell
     disabled) and runs the hot-flip soak (SUC-001's script, control arm).
  2. Operator records latch occurrences (expects to reproduce the historical
     trigger on susceptible hardware — a clean control-arm run on immune
     motors is *not* evidence the armor works, per the knowledge doc's own
     caveat).
  3. Operator sends `DEV M <n> CFG dwell=100` (or omits — 100 ms is the ship
     default) and re-runs the identical soak (treatment arm).
  4. Operator compares latch counts between arms and records which motors
     were used.
- **Postconditions**: Both arms' CSVs/transcripts retained, motor identity
  recorded, `dwell=0` config restored to a safe value (never left at 0) at
  session end.
- **Acceptance Criteria**:
  - [ ] `DEV M <n> CFG dwell=0` is accepted and literally disables the
        dwell (verified via the off-hardware policy test, SUC-005, and
        observable on the bus as an immediate reversal write).
  - [ ] The soak script supports running both arms and records which was
        run and on which physical motors.
  - [ ] Documentation calls out that `dwell=0` is for A/B bench comparison
        only and must never ship as a default.

## SUC-005: Off-hardware verification of the armor's write/reset/wedge decisions
Parent: none (developer-facing test infrastructure, not robot behavior)

- **Actor**: Developer / CI-less regression check, running
  `uv run python -m pytest` on a laptop with no robot attached.
- **Preconditions**: `Hal::Motor`'s armor policy is implemented; a
  dependency-free `MockMotor` test leaf exists under `tests/sim/unit/`.
- **Main Flow**:
  1. Developer runs `uv run python -m pytest`.
  2. A pytest test compiles a small standalone C++ harness (including only
     `capability/motor.h` and `messages/*.h` — no CODAL, no I2C) with the
     system C++ compiler and runs it.
  3. The harness drives a `MockMotor` through scripted command sequences and
     asserts: a sign change triggers a zero-write-then-hold; the hold lasts
     at least `reversal_dwell`; a sub-deadband duty writes 0; a commanded
     stop is immediate even mid-dwell; a reset requested while "moving"
     dispatches the soft path and while "at rest" dispatches the hard path;
     a stuck-but-idle motor reports `wedged` but not `wedgeSuspect`, while a
     stuck motor with applied duty above the deadband reports both.
- **Postconditions**: Test passes/fails deterministically with no hardware
  attached, in well under a second.
- **Acceptance Criteria**:
  - [ ] All six scripted-sequence assertions in the Main Flow pass.
  - [ ] The test is collected by `uv run python -m pytest` (added to
        `tests/sim/unit/` or wired into `pyproject.toml`'s `testpaths` if
        placed elsewhere).
  - [ ] The test requires no connected hardware and no CODAL/ARM toolchain.
