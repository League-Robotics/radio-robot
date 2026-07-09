---
status: done
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 093 Use Cases

This sprint does not add end-user capability — it removes capability
(Planner-mediated multi-verb motion, telemetry, config, watchdogs) in
exchange for a minimal, legible bare wheel-driving executive. The use cases
below describe the surface that MUST still work after the gut, and the one
operational concern (bench-only safety) that governs whether the removal is
acceptable at all.

## SUC-001: Operator drives both wheels directly by commanded velocity
Parent: UC (bench operation / hardware verification)

- **Actor**: Bench operator (human, or a test harness standing in for one)
  issuing wire commands over serial or the radio relay.
- **Preconditions**: Robot booted, on the stand, `DEVICE:` banner sent.
- **Main Flow**:
  1. Operator sends `S <left> <right>` with signed wheel velocities (`// [mm/s]`).
  2. `MainLoop` posts a `msg::DrivetrainCommand{WHEELS}` directly to
     `bb.driveIn` — no kinematics, no ramp.
  3. `Drivetrain::apply()` sets wheel targets and activates authority.
  4. Each `NezhaMotor`'s own PID tracks the new target starting the next tick.
- **Postconditions**: Both wheels spin at (approximately) the commanded
  velocity and direction; encoders increment accordingly.
- **Acceptance Criteria**:
  - [ ] `S 200 200` drives both wheels forward at comparable magnitude.
  - [ ] `S 200 -200` spins the wheels in opposite directions.
  - [ ] No acceleration ramp is applied — this is an accepted behavior change
        from the pre-sprint Planner-mediated `S`.

## SUC-002: Operator stops the robot immediately
Parent: UC (bench operation / hardware verification)

- **Actor**: Bench operator.
- **Preconditions**: Robot driving under an active `S`, or idle.
- **Main Flow**:
  1. Operator sends `STOP`.
  2. `MainLoop` posts the canonical `buildDrivetrainStop(msg::Neutral::BRAKE)`
     command to `bb.driveIn`.
  3. `Drivetrain`/`Hardware` neutralize both wheels on the next tick.
- **Postconditions**: Wheels come to rest; encoders hold.
- **Acceptance Criteria**:
  - [ ] `STOP` neutralizes both wheels regardless of prior `S` state.
  - [ ] Reply is `OK stop`.

## SUC-003: Host/relay identifies and pings the robot
Parent: UC (host auto-detection)

- **Actor**: `robot_radio` / TestGUI host software, or a human at a terminal.
- **Preconditions**: Robot booted (banner already sent) or already running.
- **Main Flow**:
  1. Host sends `PING`; robot replies `OK`.
  2. Host sends `HELLO`; robot replies with the same `DEVICE:...` banner
     text emitted at boot (`formatDeviceAnnouncement()`).
- **Postconditions**: Host confirms liveness and identity without needing
  any of the removed command families.
- **Acceptance Criteria**:
  - [ ] `PING` → `OK` over serial and over the radio relay.
  - [ ] `HELLO` → `DEVICE:...` banner, byte-identical in shape to the boot
        announcement.

## SUC-004: Stakeholder accepts bench-only safety posture
Parent: UC (operational safety)

- **Actor**: Stakeholder (owns the decision), bench operator (lives with the
  consequence).
- **Preconditions**: Robot is mounted on a stand, wheels off the ground
  (`.claude/rules/hardware-bench-testing.md`) for the lifetime of this
  sprint's command surface.
- **Main Flow**:
  1. The serial-silence safety watchdog, its `estop()`, and the streaming-
     drive watchdog are removed from the tick — a command silence or a wire
     stall no longer neutralizes the motors automatically.
  2. The stakeholder has explicitly accepted this trade for a minimal loop,
     conditioned on the robot never running on the stand with wheels able to
     drive off a surface.
- **Postconditions**: No software safety net exists once this sprint ships;
  operational discipline (stand-only, hand near power) is the only
  remaining safeguard.
- **Acceptance Criteria**:
  - [ ] The sprint's architecture and sprint docs both call this out
        explicitly as a deliberate, stakeholder-owned removal, not an
        oversight.
  - [ ] No ticket re-introduces a watchdog/estop as a "safety improvement"
        without a new, explicit stakeholder decision.
