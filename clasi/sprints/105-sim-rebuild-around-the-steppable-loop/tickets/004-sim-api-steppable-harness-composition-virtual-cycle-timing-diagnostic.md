---
id: "004"
title: "sim_api: steppable harness composition + virtual-cycle timing diagnostic"
status: open
use-cases: [SUC-021]
depends-on: ["001", "002", "003"]
github-issue: ""
issue: ""
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# sim_api: steppable harness composition + virtual-cycle timing diagnostic

## Description

Tickets 001 (`App::RobotLoop`), 002 (`FakeTransport`), and 003 (the
motor+OTOS plant) each exist independently. This ticket composes them into
one reusable C++ harness — `sim_api` — that other test binaries (this
sprint's own ticket 006 scenarios, and sprint 106's future profile-
validation work) link against instead of each re-deriving the composition.

`sim_api` constructs the full stack: `Devices::I2CBus`, two
`Devices::NezhaMotor`, `Devices::Otos`, color/line leaves (scripted
present()==false — telemetry carries no `line=`/`color=` fields yet per
sprint 103's Open Question 1, so there is nothing for a plant response to
feed), a fake `Devices::Clock`/`Devices::Sleeper`, two `FakeTransport`
instances (serial + radio stand-ins), `App::Comms`, `App::Telemetry`,
`App::Deadman`, `App::Drive`, `App::Odometry`, `App::Preamble` — wired to
`App::RobotLoop` (ticket 001) and stepped by the plant (ticket 003) each
cycle. It exposes a minimal, stable stepping surface: construct, step(N
cycles), inject an armored command line, drain decoded telemetry frames.

It also reports its own virtual cycle timing, using `Devices::Sleeper`'s
existing `sleepCount()`/`lastSleepMillis()`/`yieldCount()` instrumentation
— a diagnostic for sprint 106 to compare against the real loop's measured
~36ms/cycle (104's own finding vs. the `kCycle=16ms` target), not a
pass/fail assertion in this sprint.

## Acceptance Criteria

- [ ] `sim_api` exposes at minimum: a constructor (boots the sim to a
      known pre-`Preamble` state), `step(int cycles)` (advances the boot
      loop and/or main cycle the given number of times, driving the plant
      each cycle), `injectCommand(const char* armoredLine)` (pushes onto
      the inbound `FakeTransport`), and `drainTelemetry()` (returns decoded
      `msg::ReplyEnvelope`/`Telemetry`/`TelemetrySecondary` frames captured
      since the last drain).
- [ ] A harness-level test boots the sim (steps until `Preamble::done()`
      is observably true via the decoded boot telemetry's `kEventBootReady`
      bit), arms a twist via `injectCommand()`, steps N cycles, and
      observes `encLeft`/`encRight`/`velLeft`/`velRight` in the decoded
      telemetry moving in the commanded direction, ramping per ticket 003's
      own plant time-constant.
- [ ] The harness reports its own virtual cycle timing: for a representative
      run, it logs (to stdout or a returned struct) which `runAndWait`
      block (encoder settle / duty clearance / cycle pace) consumed the
      most virtual sleep time per cycle, using `Sleeper::lastSleepMillis()`/
      `sleepCount()`/`yieldCount()` — no pass/fail assertion required this
      sprint, this is explicitly a diagnostic for 106.
- [ ] `sim_api`'s own public surface has no dependency on `MicroBit.h` or
      any ARM-only header — confirmed by the same "no MicroBit.h in the
      compiled translation units" check ticket 001 established.
- [ ] File placement decision recorded (architecture-update.md Step 7 Open
      Question 1): document in the code/comments whether `sim_api` lives
      at `tests/sim/system/sim_api.{h,cpp}` or a new `tests/sim/support/`
      directory, and why.

## Testing

- **Existing tests to run**: everything from tickets 001-003's own test
  suites (`test_app_robot_loop.py`, `test_fake_transport.py`,
  `tests/sim/plant/` tests) must stay green — this ticket composes, it
  does not modify, any of them.
- **New tests to write**: `tests/sim/system/sim_api_harness.cpp` (or
  `tests/sim/support/`, per the file-placement decision) + a pytest
  wrapper proving the boot→twist→observe-motion flow and the timing
  diagnostic's presence.
- **Verification command**: `uv run python -m pytest tests/sim/system/ -v`.

## Implementation Plan

**Approach**: `sim_api` is purely compositional — it owns instances of
every class from tickets 001-003 (plus the `App::` modules 103/104 already
built, unchanged) and wires them together exactly as `main.cpp`/
`RobotLoop` do on ARM, substituting the fake `Clock`/`Sleeper`/
`FakeTransport`/plant-scripted `I2CBus` for their real counterparts. Each
`step(N)` call: (1) if still booting, advances `Preamble`/the boot loop N
times (or until done, per the API's own chosen semantics — ticket-time
decision); (2) once booted, advances the main cycle N times, and between
each cycle, calls into the ticket-003 plant to read `appliedDuty()` from
the just-completed cycle and pre-script the NEXT cycle's `I2CBus`/OTOS
responses. Telemetry capture: after each `RobotLoop` cycle, drain
`FakeTransport`'s outbound queues, dearmor + `msg::wire::decode()` each
line, and append to an internal decoded-frame buffer `drainTelemetry()`
returns.

**Files to create**:
- `tests/sim/system/sim_api.h` / `.cpp` (or `tests/sim/support/`, per the
  file-placement decision) — the composed harness class.
- `tests/sim/system/sim_api_harness.cpp` + pytest wrapper — the
  acceptance-criteria proof.

**Files to modify**: none (pure composition of tickets 001-003's existing
outputs; no production code touched).

**Testing plan**: the harness-level test IS the testing plan for this
ticket — boot, twist, observe motion, confirm the timing diagnostic
produces output. No bench gate needed (no ARM/production code changes).

**Documentation updates**: `sim_api.h`'s own file-header comment documents
its public surface and the file-placement rationale (Open Question 1);
no external doc changes needed yet (ticket 006 updates `tests/CLAUDE.md`
once the full sim tier, including this harness, is in its final shape).
