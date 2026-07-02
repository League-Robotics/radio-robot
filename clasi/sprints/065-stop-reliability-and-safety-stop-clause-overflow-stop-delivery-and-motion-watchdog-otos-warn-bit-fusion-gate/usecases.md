---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 065 Use Cases

Parents reference `docs/usecases.md` (UC-003 Drive Robot a Specific Distance,
UC-004 Stop Robot Immediately, UC-012 Initialize and Read OTOS Sensor). That
document predates the v2 wire protocol (`stop=`/`sensor=` clauses, EKF
fusion, the system watchdog) — the parent links are for traceability of
intent (drive-with-stop-conditions, stop-the-robot, OTOS-derived pose), not
literal command-syntax equivalence.

## SUC-001: Distance/timed drive with multiple stop clauses does not crash
Parent: UC-003

- **Actor**: Python host (any caller: TestGUI, bench script, `rogo`)
- **Preconditions**: Robot firmware (or sim) is running and connected.
- **Main Flow**:
  1. Host sends `D <l> <r> <mm> stop=<clause> sensor=<clause>` (or `T ... stop=... sensor=...`)
     with any legal combination of clauses up to the protocol's advertised limit.
  2. Firmware installs the DISTANCE/TIME primary stop plus every supplied
     clause, without exceeding `MotionCommand::kMaxStopConds` through
     redundant internal double-booking.
  3. The drive runs until the earliest-firing condition among all installed
     stops.
  4. Firmware emits the completion EVT with `reason=<token>` for whichever
     condition fired.
- **Postconditions**: The command completed (or was safely rejected) without
  aborting the hosting process (sim) or panicking the firmware (hardware).
- **Acceptance Criteria**:
  - [ ] `D 150 150 300 stop=time:9000 sensor=line0>500` runs to completion in
        sim without process abort and honors whichever clause fires first.
  - [ ] A plain `D`/`T` (no extra clauses) installs exactly its intended
        primary stop(s) — no wasted duplicate DISTANCE/TIME slot.
  - [ ] If the clause count still cannot fit (e.g. 3+ wire clauses on a `D`
        that already carries 2 internal stops), the command is cancelled and
        the host receives a wire-visible `ERR`, never a crash.

## SUC-002: A single dropped or unacknowledged STOP does not cause a runaway
Parent: UC-004

- **Actor**: TestGUI operator driving with the keyboard (`KeyboardDriver`)
  over a lossy link (direct USB or radio relay).
- **Preconditions**: An open-ended `VW` session is active (arrow key held);
  the link intermittently drops 15-50% of lines.
- **Main Flow**:
  1. Operator releases the arrow key (or the window loses focus while a key
     is logically held).
  2. `KeyboardDriver` stops resending the driving `VW` and instead resends
     `STOP` for a bounded number of ticks (a deadman sequence), rather than
     sending `STOP` once and going silent.
  3. Even if some `STOP` datagrams are dropped, at least one reaches the
     firmware within the deadman window.
- **Postconditions**: The robot comes to rest within a bounded time after
  key release, regardless of which individual `STOP` transmissions were
  dropped.
- **Acceptance Criteria**:
  - [ ] Sim/bench test: suppress one simulated `STOP` send — the robot still
        stops within the deadman window.
  - [ ] Window-focus loss while a key is held is treated as a release (STOP
        deadman sequence fires) rather than leaving the robot driving with no
        further host input.

## SUC-003: An idle or hung host process cannot mask the motion watchdog
Parent: UC-004

- **Actor**: Any connected host process (TestGUI, bench script, a hung or
  crashed script that still holds the serial port open).
- **Preconditions**: A motion command is active on the firmware.
- **Main Flow**:
  1. Firmware's motion watchdog is reset only by an explicit `+` keepalive or
     by a motion-verb command (`S`, `T`, `D`, `G`, `R`, `TURN`, `RT`, `VW`,
     `_VW`, `X`, `STOP`) — never by an unrelated query (`GET`, `SNAP`, ...).
  2. The host's `SerialConnection` keepalive daemon is armed only while a
     motion source (e.g. `KeyboardDriver`) is actively driving, not for the
     lifetime of the connection.
  3. For open-ended velocity commands (`VW`/`S`/`R`, which carry no `TIME`
     stop), the firmware additionally requires a genuinely fresh
     velocity-target refresh within `sTimeoutMs` — an ambient `+` alone,
     with no fresh `VW`-class command behind it, is not sufficient to keep
     the command alive indefinitely.
- **Postconditions**: A host that stops issuing real commands/keepalives (by
  crashing, hanging, or simply going idle) no longer keeps an open-ended
  motion command alive past `sTimeoutMs`.
- **Acceptance Criteria**:
  - [ ] Sim test: an active `VW`/open-ended command with only `GET`/`SNAP`
        traffic (no `+`, no fresh `VW`) safety-stops at `sTimeoutMs`.
  - [ ] Sim test: an active `VW` kept alive by `+` only (no fresh `VW`
        resend) still safety-stops at `sTimeoutMs` — closes the "keepalive
        thread outlives a frozen VW-issuing layer" gap.
  - [ ] A hung host process holding the port open no longer keeps an
        open-ended motion command alive past the watchdog window.
  - [ ] Self-terminating commands (`T`/`D`/`G`/`TURN`/`RT`, which already
        carry a `TIME` stop) are unaffected — still watchdog-exempt.

## SUC-004: OTOS fusion is gated on warning-bit persistence, not just readability
Parent: UC-012

- **Actor**: Firmware EKF fusion pipeline (`Robot::otosCorrect`).
- **Preconditions**: The OTOS chip reports a successful I2C read (readable)
  but a WARNING status bit (e.g. `warnOpticalTracking`) is set — a lifted
  robot, a robot on the stand, or one freshly placed on the playfield.
- **Main Flow**:
  1. Each tick, firmware tracks how many consecutive ticks have reported a
     WARNING bit.
  2. While the streak is short (≤ K ticks), the OTOS observation is still
     fused into the EKF — a brief warn blip does not interrupt fusion.
  3. Once the streak exceeds K, fusion is blocked: the OTOS pose is still
     shown in telemetry (`otos=`), but `addOtosObservation` is not called —
     fused pose/heading tracks encoder-derived odometry instead.
  4. Fusion is re-admitted only after N consecutive clean (`otosStatus==0`)
     ticks.
- **Postconditions**: A robot with a persistently degraded OTOS reading never
  has its frozen pose/near-zero velocity fused into the EKF, so the EKF's
  own gate-recovery force-snap path is never invoked against garbage data.
- **Acceptance Criteria**:
  - [ ] Sim gains a "warn-bit-set-but-readable" OTOS state (parallel to the
        existing lift/read-failure states) so this gate is testable.
  - [ ] Test: with the warn bit persistently set and wheels spinning, fused
        pose follows encoder odometry — no snap to the frozen OTOS pose.
  - [ ] Test: a 1-2 sample warn blip does not interrupt fusion.
  - [ ] Raw OTOS telemetry (`otos=`) is unaffected — this gate only changes
        what gets fused, not what is reported.
