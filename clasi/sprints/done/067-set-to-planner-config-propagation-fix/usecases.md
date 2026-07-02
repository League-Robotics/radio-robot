---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 067 Use Cases

Parent: `docs/usecases.md` UC-014 "Tune Calibration Parameters at Runtime" —
that use case's Main Flow already promises "the updated parameter takes
effect immediately on the next relevant computation." These SUCs narrow that
promise to the specific keys this sprint proves (or, for dead keys,
documents as never having worked) and adds the regression guard the promise
was silently violating.

## SUC-001: SET of a motion-critical geometry/calibration key takes effect on the next motion command
Parent: UC-014

- **Actor**: Python host (any caller: TestGUI, bench script, `rogo`, a
  sim-to-hardware fitting workflow).
- **Preconditions**: Robot firmware (or sim) is running; no motion command is
  currently blocking config changes (SET is always accepted regardless of
  drive state).
- **Main Flow**:
  1. Host sends `SET <key>=<value>` for one of: `rotSlip`, `tw`, `vWheelMax`,
     `rotGainPos`, `rotGainNeg`, `turnGate`, `ctrlPeriod` (the plain,
     unannotated keys this sprint's audit found the Planner's private
     `RobotConfig` copy could never see).
  2. Firmware replies `OK set <key>=<value>`; `GET <key>` reflects the new
     value (this already worked before this sprint).
  3. Host issues a motion command whose behavior depends on that key (e.g.
     `RT <cdeg>` for `rotSlip`/`tw`/`rotGainPos`/`rotGainNeg`/`turnGate`).
  4. The command's actual trajectory/target reflects the newly SET value —
     not the value compiled into `DefaultConfig.cpp` or loaded from the
     per-robot JSON at boot.
- **Postconditions**: The Planner's goal-closure math and Drive's EKF-predict
  step both operate on the live `RobotConfig`, with no private snapshot to
  go stale.
- **Acceptance Criteria**:
  - [ ] `SET rotSlip=<x>` measurably changes the encoder-arc target `RT
        9000` drives on the *next* invocation, isolated from any prior RT
        call (fresh `Sim()` or a properly-accepted `ZERO enc` between
        measurements — see SUC-003 for why this isolation matters).
  - [ ] `SET tw=<x>` alone (not bundled with a `vel.*`/other `"drive"`-tagged
        key in the same `SET` line) changes the trackwidth Drive's
        EKF-predict step uses on the very next tick.
  - [ ] `SET vWheelMax=<x>`, `SET rotGainPos=<x>`, `SET rotGainNeg=<x>`,
        `SET turnGate=<x>` each change Planner's use of that value without
        requiring any other key to be SET in the same command.
  - [ ] `SET ctrlPeriod=<x>` changes Planner's own tick-throttle cadence
        (today it only changes `LoopScheduler`'s cadence, which already read
        the live value).

## SUC-002: SET of an EKF sensor-fusion noise parameter takes effect without resetting the live pose estimate
Parent: UC-014

- **Actor**: Python host tuning EKF fusion confidence at runtime (part of the
  sim-to-hardware fitting workflow this sprint unblocks for sprint 069).
- **Preconditions**: Robot has been driving and has a non-trivial fused pose
  (not at the origin); `SET ekfRHead=<x>` is issued mid-session.
- **Main Flow**:
  1. Host sends `SET ekfRHead=<x>` (OTOS heading measurement noise).
  2. Firmware pushes the new noise value into the EKF's live noise state.
  3. The very next OTOS heading fusion uses the new noise weighting.
  4. The robot's current fused pose/velocity estimate is **unchanged** by
     the SET itself — only the *weighting* of future corrections changes.
- **Postconditions**: EKF noise tuning is live-SETtable the same way every
  other calibration key is, with no destructive side effect on in-flight
  state estimation.
- **Acceptance Criteria**:
  - [ ] `SET ekfRHead=<x>` changes how strongly a subsequent OTOS heading
        disagreement is corrected (observable via a deliberately-injected
        heading disagreement in sim).
  - [ ] Immediately after `SET ekfRHead=<x>`, the fused pose/velocity read
        back identically to their pre-SET values (no reset-to-origin
        regression from reusing the EKF's state-resetting `init()` path).

## SUC-003: A SET-to-consumer propagation regression suite catches staleness before it ships
Parent: UC-014

- **Actor**: CI / any engineer running the default pytest suite.
- **Preconditions**: None — runs as part of `uv run python -m pytest`.
- **Main Flow**:
  1. A table-driven test SETs each motion-critical key identified by this
     sprint's audit and exercises the one sim-observable behavior that
     depends on it.
  2. Each assertion is driven from an isolated measurement — a fresh `Sim()`
     instance or a *successful* (reply-checked) `ZERO enc`, not a bare
     `ZERO` — so a passing assertion cannot be a cross-call accumulation
     artifact.
  3. `tests/simulation/unit/test_rt_slip.py`'s existing `_arc_after_rt()`
     helper (which currently sends a bare `ZERO` — rejected with `ERR
     badarg` by `parseZero()`, silently ignored, letting encoder readings
     accumulate across sequential `RT` calls within one test — confirmed by
     direct instrumentation during this planning pass to mask the exact bug
     this sprint fixes) is corrected to use `ZERO enc` and check the reply.
- **Postconditions**: A future regression that reintroduces a stale config
  copy anywhere in this key set fails a specific, named test — not a test
  that happens to pass regardless of the underlying behavior.
- **Acceptance Criteria**:
  - [ ] `test_rt_slip.py`'s three existing tests pass for the right reason
        (verified: with `_cfg` live, `SET rotSlip=0` vs `SET rotSlip=0.74`
        genuinely produce different RT arcs; before this sprint they passed
        by coincidence — see the sprint's `architecture-update.md` "Why").
  - [ ] A new sweep test covers `rotSlip`, `tw`, `vWheelMax`, `rotGainPos`,
        `rotGainNeg`, `turnGate`, `ctrlPeriod`, and `ekfRHead`.
  - [ ] Full default pytest suite stays green (baseline 2506 passed, 0
        failed, plus this sprint's new tests).
