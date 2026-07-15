---
id: '004'
title: 'sim_api: steppable harness composition + virtual-cycle timing diagnostic'
status: done
use-cases:
- SUC-021
depends-on:
- '001'
- '002'
- '003'
github-issue: ''
issue: ''
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

- [x] `sim_api` exposes at minimum: a constructor (boots the sim to a
      known pre-`Preamble` state), `step(int cycles)` (advances the boot
      loop and/or main cycle the given number of times, driving the plant
      each cycle), `injectCommand(const char* armoredLine)` (pushes onto
      the inbound `FakeTransport`), and `drainTelemetry()` (returns decoded
      `msg::ReplyEnvelope`/`Telemetry`/`TelemetrySecondary` frames captured
      since the last drain).
- [x] A harness-level test boots the sim (steps until `Preamble::done()`
      is observably true via the decoded boot telemetry's `kEventBootReady`
      bit), arms a twist via `injectCommand()`, steps N cycles, and
      observes `encLeft`/`encRight`/`velLeft`/`velRight` in the decoded
      telemetry moving in the commanded direction, ramping per ticket 003's
      own plant time-constant.
- [x] The harness reports its own virtual cycle timing: for a representative
      run, it logs (to stdout or a returned struct) which `runAndWait`
      block (encoder settle / duty clearance / cycle pace) consumed the
      most virtual sleep time per cycle, using `Sleeper::lastSleepMillis()`/
      `sleepCount()`/`yieldCount()` — no pass/fail assertion required this
      sprint, this is explicitly a diagnostic for 106.
- [x] `sim_api`'s own public surface has no dependency on `MicroBit.h` or
      any ARM-only header — confirmed by the same "no MicroBit.h in the
      compiled translation units" check ticket 001 established.
- [x] File placement decision recorded (architecture-update.md Step 7 Open
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

## Completion Notes

**File placement**: `TestSim::SimApi` lives at `tests/sim/support/sim_api.{h,cpp}`
(NOT `tests/sim/system/`) — its primary consumers (ticket 006's own
`tests/sim/system/` scenario files, and sprint 106's future profile-validation
work) are not colocated with it, exactly the role `tests/sim/support/
fake_transport.h` already established for itself. `tests/sim/system/`'s own
README scopes that directory to whole-robot *scenario* files (this ticket's
own `sim_api_harness.cpp` lives there, correctly, as a consumer of the
library). Full rationale in `sim_api.h`'s own file header.

**A second support file was needed and not anticipated by the plan**:
`tests/sim/support/wire_test_codec.{h,cpp}`. The generated `msg::wire` codec
(`source/messages/wire.{h,cpp}`) is deliberately asymmetric — firmware only
ever *decodes* `CommandEnvelope` (host→firmware) and *encodes*
`ReplyEnvelope`/`TelemetrySecondary` (firmware→host); there is no
`decode(ReplyEnvelope)`/`decode(TelemetrySecondary)` and no
`encode(CommandEnvelope)` anywhere in the generated code (confirmed against
`app_telemetry_harness.cpp`'s own identical finding, predating this ticket).
`drainTelemetry()` needs to *read* arbitrary decoded values out of a live
telemetry stream (not just compare against a pre-built expectation, the
existing harnesses' own workaround), and `injectTwist()`/`injectStop()` need
to *build* a `CommandEnvelope` — neither is possible with the generated codec
alone. `wire_test_codec.{h,cpp}` is a minimal, hand-written, flat-tag-dispatch
codec scoped to exactly the message shapes `SimApi` actually exchanges (not a
reimplementation of `wire.cpp`'s generic FieldDesc/MessageTable engine, which
is generated and internal-linkage anyway). No production file was touched.

**Plant/PID tuning decision**: unlike every prior sim harness in this sprint
(001-003), which kept `velGains` at zero so duty stays deterministically 0
forever, this ticket's own twist-ramp scenario needs *real* duty movement
through the actual `App::Drive`→PID path. `MotorConfig.kp = 0.01` (pure
proportional, `ki=kff=iMax=kaw=0`) plus `slewRate = 100` (wide enough to reach
saturation in one write) were chosen so that every twist this harness ever
injects (`|v_x|` always well above the plant's own achievable ceiling,
`TestSim::kDefaultDutyVelMax`) saturates the PID output to ±1.0 *immediately*
and *stays* saturated for the life of the scenario — this keeps the shared-
I2CBus exact-write-count scripting (105-003's own "CRITICAL prior finding")
tractable by hand: every actuation change (initial mode-activation, a fresh
twist, an explicit stop, or a deadman expiry) is a single, immediately-
saturated write, never a multi-write slew ramp. Full derivation in
`sim_api.h`'s "Plant/PID tuning" section.

**A real bug found and fixed during implementation**: the per-cycle bus-
script write-count helper (`scriptCycleBusResponses()`) used a `pendingEventCycle_
== -1` sentinel for "no command pending"; `pendingEventCycle_ + 1` then
evaluated to `0` and spuriously matched `cycleCount_ == 0`, handing the left
motor a phantom second write on cycle 0 of any run that hadn't yet injected a
command — desyncing the shared write/read FIFOs (105-003's own documented
hazard) from that point on and leaving `velLeft` pinned at 0 for the rest of
the run. Found via the four-phase debugging protocol (bus `errCount()`
instrumentation showed a steady +1/cycle leak starting at cycle 1, present
only in the scenario that ran several settle cycles before its first command
injection). Fixed by explicitly guarding both the R and L conditions on
`pendingEventCycle_ >= 0`.

**Virtual-cycle-timing diagnostic — the numbers for sprint 106**:

| | value |
|---|---|
| `Sleeper::sleepCount()` delta per `cycle()` | 4 (3 `runAndWait` blocks + final `sleepUntil`) |
| `Sleeper::lastSleepMillis()` (final/pace block) | 16ms |
| `Sleeper::yieldCount()` delta | 0 (`cycle()` never calls `Sleeper::yield()` directly) |
| **Derived virtual total per cycle** | **28ms** (3×4ms settle/clear/settle + 16ms pace) |
| `kCycle` design target (robot_loop.cpp) | 16ms |
| Sprint 104's measured real hardware cycle | ~36ms |

This is fully deterministic and provable, not merely observed: the HOST_BUILD
fake `Devices::Clock` never advances during a single `cycle()` call (only
`SimApi::step()`, between calls, ever touches it — `Devices::Sleeper::
sleepMillis()` itself never moves the paired Clock, per `clock_host.cpp`'s own
file header), so every `runAndWait`/`sleepUntil`'s `elapsed-since-mark` is
provably `0`, meaning each of the four sleeps requests **exactly** its own
gap parameter — no work, no I/O, and no real hardware latency ever shortens
it. That makes the sim number a *lower bound on the schedule itself*, with
zero real-world overhead folded in.

Reading the three numbers together: the **28ms virtual/scheduled** figure
already exceeds the **16ms `kCycle` design target** by 12ms — entirely
explained by the three 4ms settle/clearance windows (`kSettle`×2 + `kClear`)
that `sleepUntil(cycleStart, kCycle)`'s own naive 16ms target does not
subtract off (on real hardware, elapsed time already includes those three
sleeps by the time the final `sleepUntil` runs, so the *real* total is not
simply capped at 16ms either — it is at least `kSettle+kClear+kSettle` plus
whatever the final block's own remaining slice is). The remaining gap between
the 28ms scheduled figure and the **~36ms measured hardware figure** (104) —
about 8ms — is real I2C bus/hardware latency the sim does not (and structurally
cannot) model, since HOST_BUILD bus transactions are instant, zero-time
function calls. **Conclusion for 106**: of the ~20ms gap between the 16ms
design target and the ~36ms measured reality, roughly 12ms (60%) is the
*schedule itself* (the three settle/clearance windows, an artifact of
`sleepUntil`'s own naive target not accounting for time already spent) and is
tunable by revisiting `kSettle`/`kClear`/`kCycle`'s own values or the
`sleepUntil` accounting; the remaining ~8ms (40%) is real-world I2C overhead
no schedule change can remove.

**Test totals**: 2 new pytest tests (`test_sim_api_harness_compiles_and_passes`,
`test_sim_api_no_microbit_dependency`) in `tests/sim/system/test_sim_api.py`,
compiling and running a 5-scenario hand-rolled C++ harness
(`sim_api_harness.cpp`: boot, twist-ramp, stop, deadman-expiry, timing
diagnostic). Full `tests/sim/` suite: 345 passed. Full project suite
(`uv run python -m pytest`): 567 passed, 0 failed.
