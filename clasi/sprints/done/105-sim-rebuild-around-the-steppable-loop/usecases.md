---
sprint: '105'
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Use Cases — Sprint 105: Sim rebuild around the steppable loop

Continues SUC numbering from sprint 104 (SUC-001..SUC-017). This sprint is
P7 of `clasi/issues/single-loop-firmware-p3-p7-continuation.md` — the final
phase of the single-loop firmware arc.

## SUC-018: The main-loop cycle body is host-buildable and steppable

- **Actor**: Firmware/test engineer building a sim harness.
- **Preconditions**: `source/main.cpp`'s `int main()` unconditionally
  constructs `static MicroBit uBit;` and `#include "MicroBit.h"`, and its
  own timing primitives (`markTime()`/`sleepUntil()`/`runAndWait()`) call
  `system_timer_current_time()` and `uBit.sleep()` directly — vendor/ARM-only
  calls with no `#ifndef HOST_BUILD` guard anywhere in the file. This means
  `main.cpp` cannot be compiled under `-DHOST_BUILD` at all today, even
  though every module it composes (`App::Comms`, `App::Telemetry`,
  `App::Drive`, `App::Odometry`, `App::Deadman`, `App::Preamble`, and every
  `Devices::` leaf) already is host-buildable. Separately,
  `source/devices/clock.h`'s own file header states the intended design
  explicitly: "The cycle body is parameterized on a sleeper/clock
  interface: fiber_sleep + system_timer on hardware; the steppable fake
  clock in host tests" — `App::Deadman` and `App::Preamble` already take a
  `const Devices::Clock&` for exactly this reason, but `main.cpp`'s own
  outer-loop pacing does not use `Devices::Clock`/`Devices::Sleeper` at all.
- **Main Flow**: The boot loop and main cycle body currently inline in
  `source/main.cpp`'s `int main()` are extracted into a new, host-buildable
  module that takes `Devices::Clock&`/`Devices::Sleeper&` (not raw vendor
  timer calls) for every time read and every sleep/yield, and references to
  already-constructed leaves/`app/` modules (not a `MicroBit&`). `main.cpp`
  itself becomes a thin ARM-only wrapper: construct real hardware
  (`MicroBit`, `SerialPort`, `Radio`, `I2CBus`, the leaves, the real
  `Devices::Clock`/`Devices::Sleeper`), then call into the extracted
  function/class.
- **Postconditions**: The exact same cycle logic runs on ARM (unchanged
  behavior, confirmed by the bench gate) and compiles and runs under
  `HOST_BUILD` with an injected fake `Clock`/`Sleeper` and scripted
  `I2CBus`/`FakeTransport`, with no `MicroBit.h` dependency.
- **Acceptance Criteria**:
  - [ ] `source/main.cpp`'s boot loop + main `for(;;)` cycle body (the
        `runAndWait`/`markTime`/`sleepUntil` schedule and the command-dispatch
        switch) are extracted into a new `source/app/` module compiled
        without `MicroBit.h` under `-DHOST_BUILD`.
  - [ ] Every time read/sleep/yield in the extracted module goes through
        `Devices::Clock`/`Devices::Sleeper` — no `system_timer_current_time()`
        or `uBit.sleep()` call survives inside it.
  - [ ] `main.cpp` itself shrinks to hardware construction + one call into
        the extracted module; a diff against the pre-105 tree shows zero
        change to cycle ordering, timing constants (`kSettle`/`kClear`/
        `kCycle`/`kPreamblePace`), or dispatch semantics.
  - [ ] Bench-verified per `.claude/rules/hardware-bench-testing.md`: the
        real robot on the stand behaves identically post-refactor (wheels
        drive, encoders climb, telemetry emits) — a regression check, not a
        new capability.

## SUC-019: A wire-level fake transport drives Comms/Telemetry off-hardware

- **Actor**: Sim harness / test author.
- **Preconditions**: `App::Transport` (`source/app/comms.h`) is already an
  abstract, `HOST_BUILD`-safe interface (`readLine()`/`send()`/
  `sendReliable()`) with two ARM-only concrete adapters
  (`SerialTransport`/`RadioTransport`); no `HOST_BUILD` implementation
  exists yet.
- **Main Flow**: A new `Transport` implementation (an in-memory, FIFO-based
  fake) is built: a test pushes armored `"*B..."` command lines into it
  (`Comms::pump()` reads them as if from a real serial/radio line), and
  captures every armored line the loop sends out (`Comms::sendReply()`,
  `Telemetry`'s primary/secondary emit paths) for the test to dearmor and
  decode.
- **Postconditions**: A sim harness can drive `App::Comms`/`App::Telemetry`
  end-to-end (armored bytes in, armored bytes out) with no real serial port
  or radio, reusing the exact same encode/armor/decode path production
  firmware uses.
- **Acceptance Criteria**:
  - [ ] A `HOST_BUILD`-only `Transport` implementation exists with a
        test-driving surface to enqueue inbound lines and inspect outbound
        lines (both `send()`/`sendReliable()` sinks).
  - [ ] A unit test round-trips a real `twist`/`stop` `CommandEnvelope`
        through it into `Comms::pump()`, and decodes a real
        `ReplyEnvelope`/`Telemetry` frame captured from `Telemetry::emit()`.

## SUC-020: A deterministic plant model stands in for the physical drivetrain

- **Actor**: Sim harness / test author.
- **Preconditions**: No plant model exists post-102 (the old
  `SimMotor`/`PhysicsWorld` stack was deleted, not migrated — sprint 102's
  own architecture-update.md). `Devices::I2CBus`'s `HOST_BUILD` fork
  (`i2c_bus_host.cpp`) offers only a static, pre-scripted FIFO
  (`scriptWrite()`/`scriptRead()`), not a live responder. `Devices::
  NezhaMotor::appliedDuty()` is a public getter already used by an existing
  test (`devices_motor_harness.cpp` scenario 6, "PID-on chases a velocity
  target") to drive a one-line first-order-lag plant stand-in for a single
  motor in isolation — this use case generalizes that proven pattern across
  the WHOLE extracted loop (both motors + OTOS), not just one leaf's own
  `tick()` in isolation.
- **Main Flow**: A new, small, seeded, deterministic plant class reads each
  motor's `appliedDuty()` after a cycle, integrates a first-order
  duty→velocity response (time constant matching the ~120-140ms actuation
  lag documented from bench characterization) and a velocity→position
  integration, and — before the NEXT cycle's `requestSample()`/`tick()`
  pair — schedules the resulting encoder reading onto the shared
  `Devices::I2CBus` via `scriptWrite()`/`scriptRead()` (the same two-write
  one-read pairing `scriptEncoderRequestCollect()` already establishes).
  A parallel, much simpler OTOS plant answers `Devices::Otos`'s burst-read
  registers with a pose derived from the same two wheel positions (through
  the SAME `BodyKinematics::forward()` the firmware's own `Odometry` already
  uses — not a second, independently-derived heading formula). The plant
  contains no heading-wrap or angle-projection logic of its own; all heading
  state lives in `Odometry`'s (unchanged, already-proven) midpoint-arc
  integration, keeping the sprint's carried B3 caution's precondition (reused
  heading-wrap math) out of the new code entirely — see architecture-update.md
  Decision 3 for a scenario that re-verifies this directly.
- **Postconditions**: A twist command applied through the extracted loop
  produces physically plausible, ramping (not instantaneous) encoder motion
  and OTOS pose over many simulated cycles, with no real hardware.
- **Acceptance Criteria**:
  - [ ] Plant lives in `tests/sim/plant/` (test-only, not `source/` — it is
        never linked into the ARM firmware image).
  - [ ] Deterministic and seeded: two runs with the same command script and
        the same seed produce bit-identical (or tolerance-identical, if any
        float noise model is added) trajectories.
  - [ ] A velocity step (constant commanded twist) shows a visible ramp
        (not a step) in simulated velocity, with a time constant in the
        120-140ms range.
  - [ ] The plant computes NO heading of its own; a code-level check (review
        or grep) confirms no atan2/heading-wrap logic exists in the plant —
        heading only ever comes from `Odometry`'s own integration reading
        the plant's two independent wheel positions.

## SUC-021: `sim_api` composes the extracted loop, the plant, and the fake transport into one steppable harness

- **Actor**: Sim harness author; sprint 106 (profile validation).
- **Preconditions**: SUC-018 (steppable loop), SUC-019 (fake transport), and
  SUC-020 (plant) each exist independently.
- **Main Flow**: A reusable C++ harness class/library (`sim_api`) constructs
  the full composition — `Devices::I2CBus`, two `Devices::NezhaMotor`,
  `Devices::Otos`, color/line leaves (present-at-boot=false for this
  sprint — telemetry carries no `line=`/`color=` fields yet per 103's Open
  Question 1, so there is nothing for a plant response to feed), a fake
  `Devices::Clock`/`Devices::Sleeper`, the two `App::Transport` fakes
  (SUC-019), `App::Comms`, `App::Telemetry`, `App::Deadman`, `App::Drive`,
  `App::Odometry`, `App::Preamble` — wired to the extracted loop (SUC-018)
  and stepped by the plant (SUC-020) each cycle. Exposes a minimal stepping
  surface: advance N cycles, inject an armored command line, and read back
  captured telemetry frames.
- **Postconditions**: One reusable harness other C++ test binaries (and
  sprint 106's own future profile-validation tests) link against instead of
  each re-deriving the composition.
- **Acceptance Criteria**:
  - [ ] `sim_api` exposes: construct, step(N cycles), injectCommand(armored
        line), and drain captured telemetry frames (decoded, not raw bytes).
  - [ ] A harness-level test boots the sim (runs the boot loop to
        `Preamble::done()`), arms a twist, steps N cycles, and observes
        `encLeft`/`encRight`/`velLeft`/`velRight` in the decoded telemetry
        moving in the commanded direction.
  - [ ] The harness reports its own virtual cycle timing: using
        `Devices::Sleeper`'s existing `sleepCount()`/`lastSleepMillis()`/
        `yieldCount()` instrumentation, it logs which `runAndWait` block
        (encoder settle / duty clearance / cycle pace) consumes the most
        virtual time per cycle — a diagnostic for sprint 106 to compare
        against the real loop's measured ~36ms/cycle (104's own finding vs.
        the `kCycle=16ms` target), not a pass/fail assertion in this sprint.

## SUC-022: Fault injection — motor disconnect, encoder wedge, encoder dropout

- **Actor**: Sim harness / test author verifying firmware fault-reaction
  paths off-hardware.
- **Preconditions**: `clasi/issues/later/sim-hardware-fault-injection.md`
  (retargeted 2026-07-14: "a thin steppable-loop sim over the devices
  layer's HOST_BUILD fakes, whose scripted I2CBus can natively fake NAKs,
  stale reads, and wedge latch-ups"). The wedge detector
  (`Devices::MotorArmor::wedged()`/`wedgeSuspect()`) already exists and is
  already unit-tested in isolation (`devices_motor_harness.cpp` scenario 4);
  this use case exercises it through the FULL loop via the plant, not in
  isolation.
- **Main Flow**: The plant (SUC-020) gains three fault knobs: (a) motor
  disconnect — script a NAK/error status for a named port's transactions,
  verifying `NezhaMotor::connected()` and the loop's `frame.connLeft`/
  `connRight` telemetry fields go false; (b) encoder wedge — freeze a
  motor's reported position at its current value while the plant's
  internal velocity state keeps advancing, verifying `wedged()`/
  `wedgeSuspect()` latch and the loop's `kFaultWedgeLatch` telemetry bit
  sets; (c) encoder dropout — hold a fraction of cycles' scripted reads at
  the LAST value (a stale-not-fresh read, using the freshness-gate pattern
  `devices_motor_harness.cpp` scenario 8 already proves) to exercise the
  fresh-sample gate's outlier/glitch handling under sustained partial data
  loss. OTOS staleness injection (the parked issue's fourth sketch item) is
  explicitly OUT of scope this sprint — the firmware does not fuse OTOS at
  all yet (`Odometry`'s own file header: "no pose fusion happens here...
  the robot does not fuse"), so there is no firmware reaction to verify
  against; revisit when host-side fusion (106+) exists.
- **Postconditions**: The three fault knobs are regression-testable,
  deterministic, off-hardware substitutes for provoking each failure mode
  on the bench.
- **Acceptance Criteria**:
  - [ ] Motor disconnect: a pytest scenario shows `connLeft`/`connRight`
        (or the equivalent port) flip false in decoded telemetry while the
        knob is active, and recover when cleared.
  - [ ] Encoder wedge: a pytest scenario freezes one motor's position while
        driving and shows `kFaultWedgeLatch` set in decoded telemetry
        within the wedge detector's own documented threshold.
  - [ ] Encoder dropout: a pytest scenario drops a configurable fraction of
        fresh samples and shows `encGlitchCount()`/telemetry stay sane (no
        false wedge, no velocity starvation) per the freshness-gate
        contract already proven in isolation.
  - [ ] `clasi/issues/later/sim-hardware-fault-injection.md` is updated to
        reflect this sprint's actual delivered scope (which of its sketch
        items shipped, which — OTOS staleness — remain deferred and why).

## SUC-023: A green, runnable pytest sim tier with a headless scripted-twist demo

- **Actor**: Any developer or CI run; a stakeholder verifying the sprint's
  bench-runnable-equivalent deliverable.
- **Preconditions**: `tests/sim/conftest.py` currently references deleted
  infrastructure (`tests/_infra/sim`, `firmware.py`'s `Sim` class, a
  `just build-sim` recipe that no longer exists in the `justfile`) — any
  test using its `sim`/`build_lib` fixtures fails immediately. `tests/sim/
  system/` is a skeleton-only placeholder (077-006, never populated).
  `pyproject.toml`'s `testpaths = ["tests/sim", "tests/unit"]` already
  collects this domain.
- **Main Flow**: `tests/sim/conftest.py`'s stale fixtures are removed (no
  surviving caller) or replaced with fixtures appropriate to this sprint's
  actual `sim_api` (SUC-021) — a ticket-time call on whether a shared
  fixture is worth adding given the established ad hoc per-file compile
  convention (`test_app_drive.py`'s own pattern: each pytest file compiles
  its own harness + sources via `subprocess`, no shared build step). A new
  scenario test under `tests/sim/system/` boots the sim, connects (via the
  fake transport), arms a scripted twist, steps enough cycles to observe
  simulated encoder motion and telemetry, sends stop, and confirms the
  wheels return to zero velocity — end-to-end, headless, no hardware. This
  is runnable directly (not only via pytest) as this sprint's own
  stakeholder-visible "run one command and see the sim loop move" proof.
  `tests/CLAUDE.md`'s `sim/` domain description (which currently states "a
  fresh simulator harness for the new `source/` tree does not exist yet")
  is updated to describe what actually exists after this sprint.
- **Postconditions**: `uv run python -m pytest` collects and passes the sim
  tier (unit-level module harnesses from 103/104 plus this sprint's new
  system-level scenario and fault-injection tests) with no hardware
  attached; a single documented command runs the scripted-twist demo
  standalone.
- **Acceptance Criteria**:
  - [ ] `tests/sim/conftest.py` contains no reference to deleted
        infrastructure (`tests/_infra/sim`, `firmware.py`, `build-sim`).
  - [ ] A new `tests/sim/system/` scenario test: boot → twist → observe
        ramping encoder/telemetry motion → stop → observe convergence to
        zero — green under `uv run python -m pytest`.
  - [ ] The scripted-twist demo is runnable standalone (a documented
        command, e.g. `uv run python tests/sim/system/<script>.py` or
        direct invocation of the compiled harness binary) and prints a
        human-readable trace of commanded vs. observed motion.
  - [ ] `uv run python -m pytest` is fully green (SUC-018 through SUC-022's
        tests included) — the sprint's own Definition of Done.
  - [ ] `tests/CLAUDE.md`'s `sim/` section is updated to match reality.
