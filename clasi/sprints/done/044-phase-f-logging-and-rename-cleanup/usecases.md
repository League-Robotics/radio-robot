---
sprint: '044'
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Use Cases — Sprint 044: Phase F — Logging and rename/cleanup

## SUC-001: TLM readers use the estimate seam for pose and velocity

**Actor:** Host operator / CI golden-TLM canary.

**Preconditions:** `PhysicalStateEstimate` exists (`source/state/`) with `getPose`
and `getVelocity` static forwarders. `HardwareState.poseX/Y/poseHrad/fusedV/fusedOmega`
are still the primary store (written by `Odometry`). A Phase C back-compat comment
noted that existing readers read directly from `HardwareState` fields and would be
repointed in Phase F.

**Main Flow:**
1. `buildTlmFrame` calls `PhysicalStateEstimate::getPose(s, x, y, h)` instead of
   `Odometry::getPose(s, x, y, h)`.
2. `buildTlmFrame` calls `PhysicalStateEstimate::getVelocity(s, v, omega)` instead
   of reading `s.fusedV` / `s.fusedOmega` directly.
3. `MotionController::getPoseFloat` calls `PhysicalStateEstimate::getPose(s, ...)`.
4. The same float values are produced because `PhysicalStateEstimate::getPose` and
   `getVelocity` read the same `HardwareState` primary fields.

**Postconditions:** All TLM pose/velocity reads go through the seam object. The
golden-TLM canary is byte-exact. No behavioral change to TLM content.

**Acceptance Criteria:**
- [ ] `buildTlmFrame` uses `estimate.getPose` and `estimate.getVelocity`.
- [ ] `MotionController::getPoseFloat` uses `PhysicalStateEstimate::getPose`.
- [ ] Golden-TLM canary passes byte-exact.
- [ ] `Odometry::getPose` is no longer called from `RobotTelemetry.cpp` or
      `MotionController.cpp`.

---

## SUC-002: Inputs struct lives at source/types/Inputs.h; RobotState name retired

**Actor:** Developer reading or compiling the source tree.

**Preconditions:** `source/control/RobotState.h` defines `HardwareState`,
`MotorCommands`, `TargetState`, `RobotStateContainer`, `defaultInputs`. At least
twelve files include it directly. The "RobotState" blob name is transitional.

**Main Flow:**
1. `source/types/Inputs.h` is created with the same content as `RobotState.h`.
2. All `#include "RobotState.h"` and `#include "control/RobotState.h"` references
   are rewritten to `#include "types/Inputs.h"` (or the relative equivalent).
3. `source/control/RobotState.h` is deleted.
4. The name "RobotState" is removed from maintained source; a grep confirms zero
   non-comment occurrences in `source/`.

**Postconditions:** `source/types/` is complete: `Config.h`, `Protocol.h`,
`CommandTypes.h`, `Inputs.h`. Struct field layout is unchanged. All builds green.

**Acceptance Criteria:**
- [ ] `source/types/Inputs.h` exists with full content of former `RobotState.h`.
- [ ] `source/control/RobotState.h` does not exist (deleted, not shimmed).
- [ ] Zero `#include "RobotState.h"` or `#include "control/RobotState.h"` in
      maintained source.
- [ ] Host build and ARM firmware build green.
- [ ] Golden-TLM canary passes byte-exact.

---

## SUC-003: Alias shim headers deleted; callers use canonical include paths

**Actor:** Developer browsing `source/io/` root and `source/control/` shims.

**Preconditions:** Six alias shims exist at `source/io/` root (`IMotor.h`,
`IServo.h`, `IOtosSensor.h`, `IColorSensor.h`, `ILineSensor.h`, `IPortIO.h`).
Two control-layer shims exist (`source/control/EKF.h`, `source/control/MotionController.h`).
All callers still compile via these shims.

**Main Flow:**
1. Each file that includes a shim is updated to include the canonical path directly.
2. The eight shim files are deleted.
3. Build verifies no dangling includes.

**Postconditions:** `source/io/` root contains only `Hardware.h`, `ReplayHAL.h`,
and subdirectories. No alias shims remain. All includes point to canonical paths.

**Acceptance Criteria:**
- [ ] `source/io/IMotor.h`, `IServo.h`, `IOtosSensor.h`, `IColorSensor.h`,
      `ILineSensor.h`, `IPortIO.h` do not exist.
- [ ] `source/control/EKF.h` alias shim does not exist.
- [ ] `source/control/MotionController.h` alias shim does not exist.
- [ ] Host build and ARM firmware build green.

---

## SUC-004: DebugCommandable I2CBus leak resolved; vendor-confinement grep returns zero

**Actor:** CI vendor-confinement gate / developer auditing vendor leaks.

**Preconditions:** `source/app/DebugCommandable.h` has `I2CBus* bus` in `DbgCtx`
and a forward declaration `class I2CBus`. `DebugCommandable.cpp` `#include "I2CBus.h"`
inside `#ifndef HOST_BUILD`. The `vendor_baseline.txt` lists exactly these four
occurrences. These are the sole remaining hits above `source/io/`.

**Main Flow:**
1. `DbgCtx` replaces `I2CBus* bus` with `IBusDiagnostics* busDiag` (the interface
   already exists from Phase A).
2. Handler functions that call `ctx.bus->txnCount(addr)`, `errCount(addr)`,
   `lastErr(addr)`, `reentryViolations()`, `resetStats()`, `setLogging()`,
   `dumpRecent()`, `setIrqGuard()`, `irqGuard()`, and `write()`/`read()` are
   updated. Raw write/read (I2CW, I2CR) need `I2CBus` directly; those handlers
   are either moved to `source/io/real/` as a debug device, or a richer interface
   covers them (see architecture section for chosen approach).
3. `main.cpp` passes `&hardware.busDiagnostics()` (already exposed by `NezhaHAL`)
   instead of `&hardware.bus()`.
4. `vendor_baseline.txt` is cleared.
5. `test_vendor_confinement.py` asserts zero violations.

**Postconditions:** Zero vendor/CODAL/I2CBus references in any file above
`source/io/` (excluding `main.cpp` which is intentionally firmware-only and exempt).
`DBG I2C`, `DBG I2CLOG`, `DBG IRQGUARD`, `I2CW`, `I2CR` handlers work correctly.

**Acceptance Criteria:**
- [ ] `DebugCommandable.h` has no `I2CBus` forward declaration or member.
- [ ] `DebugCommandable.cpp` does not `#include "I2CBus.h"`.
- [ ] `test_vendor_confinement.py` reports zero violations.
- [ ] `vendor_baseline.txt` is empty.
- [ ] DBG I2C commands still work on firmware (ARM build green; behavioral
      preservation verified by code inspection since bench tier is opt-in).
- [ ] Host build green.

---

## SUC-005: REPLAY mode stub compiled and exercised by a test

**Actor:** Developer / CI simulation tier.

**Preconditions:** `ReplayHAL.h` and `ReplayHAL.cpp` exist as no-op stubs.
`RobotMode::REPLAY` enum value is defined. No test yet exercises the stub.

**Main Flow:**
1. A new test in `tests/simulation/unit/` or `tests/simulation/system/` instantiates
   `ReplayHAL`, calls `begin()` and `tick(0)`, and verifies it runs without error.
2. The test optionally exercises a single no-op `loopTickOnce` pass with the
   `ReplayHAL` wired in.

**Postconditions:** The REPLAY stub is confirmed compilable and runnable. The final
verification criterion "REPLAY stub exercised" is satisfied.

**Acceptance Criteria:**
- [ ] A test exists under `tests/simulation/unit/` that instantiates `ReplayHAL`
      and calls `begin()` + `tick(0)` without error.
- [ ] Simulation tier green (test passes).

---

## SUC-006: Seam-presence test confirms all three architectural seams exist

**Actor:** CI simulation tier / developer auditing architecture.

**Preconditions:** `source/io/capability/` directory, `source/state/PhysicalStateEstimate.h`,
and `source/superstructure/Superstructure.h` all exist following prior sprints.

**Main Flow:**
1. A test asserts each of the three seam paths exists on the filesystem relative
   to the repo root.
2. Test runs in the simulation tier (no hardware needed — filesystem check only).

**Postconditions:** Architecture's three seams are machine-verified as present.
Final verification criterion "three seams findable" is satisfied.

**Acceptance Criteria:**
- [ ] `source/io/capability/` directory is asserted to exist.
- [ ] `source/state/PhysicalStateEstimate.h` is asserted to exist.
- [ ] `source/superstructure/Superstructure.h` is asserted to exist.
- [ ] The four-file device quartet per capability is asserted (capability header +
      real impl + sim impl present in directory tree).
- [ ] Test passes in simulation tier.

---

## SUC-007: Logging contract confirmed — subsystems write inputs in updateInputs only

**Actor:** Developer maintaining the loop / CI lint check.

**Preconditions:** Phase E subsystems (`Drive`, `LineSensor`, `ColorSensor`, `Ports`,
`Gripper`) exist in `source/subsystems/`. They call `updateInputs()` inside
`periodic()`. Bodies were moved verbatim so no print statements were introduced.

**Main Flow:**
1. A grep-based assertion (pytest test or linting step) confirms no `printf`,
   `snprintf` with output routing, or `telemetryEmit` call appears inside any
   `source/subsystems/` source file.
2. The assertion runs as part of the simulation tier.

**Postconditions:** §6 logging contract is machine-verified. No subsystem prints
during the loop tick.

**Acceptance Criteria:**
- [ ] A test or lint step asserts zero `printf`/`telemetryEmit` calls in
      `source/subsystems/`.
- [ ] Test passes in simulation tier.
