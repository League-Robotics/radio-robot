---
id: '002'
title: 'Host-clean loop seams: Subsystems::Hardware, dev_loop, clock'
status: done
use-cases:
- SUC-002
depends-on: []
github-issue: ''
issue: host-side-simulation-environment-for-the-new-tree-design-write-up.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host-clean loop seams: Subsystems::Hardware, dev_loop, clock

## Description

`source/main.cpp` inlines the whole dev-loop body (two-slice hardware tick,
statement dispatch, outbox drain, `Subsystems::Drivetrain` governance,
watchdog check) directly in `main()` — sprint 079 rewrote this loop but did
not extract it into a shared, host-clean function, and no abstract seam
exists yet for a simulated hardware owner to stand in for
`Subsystems::NezhaHardware`. This ticket introduces the three seams the sim
needs to plug in behind, **without changing ARM firmware behavior at all**:

1. `Subsystems::Hardware` — an abstract owner base
   (`source/subsystems/hardware.h`) that `Subsystems::NezhaHardware` is
   retrofitted to implement, and that `Subsystems::SimHardware` (ticket 003)
   will implement too. See `architecture-update.md` Decision 1 for why this
   is named `Subsystems::Hardware`, not `Hal::MotorHal` (the design
   write-up's name for this seam refers to symbols — `Hal::NezhaHal`,
   `DrivetrainToHalCommand` — that were renamed the same day the design was
   reviewed; do not reintroduce any pre-rename name while implementing this
   ticket).
2. `source/dev_loop.{h,cpp}` — `devLoopTick(DevLoop&, uint32_t now, const DevLoopStatement*)`,
   the shared loop body, matching `main.cpp`'s actual current loop exactly
   (not the design write-up's schematic pseudocode). See
   `architecture-update.md` Decision 3 for the `DevLoopStatement` shape and
   the loop-originated default reply sink `devLoopTick` needs for the
   watchdog-fire `EVT dev_watchdog` emission.
3. `source/types/clock.h` — `systemClockNow()`, replacing
   `system_commands.cpp`'s direct `system_timer_current_time()` call, the
   one remaining CODAL vendor-clock call in the host-clean command set.

This ticket has no dependency on ticket 001 (either order is buildable) and
does not require `Subsystems::SimHardware` to exist — its acceptance is
proven entirely against the one existing concrete owner,
`Subsystems::NezhaHardware`, via an ARM build + bench smoke pass.

## Acceptance Criteria

- [x] `Subsystems::Hardware` (`source/subsystems/hardware.h`) declares
      `virtual Hal::Motor& motor(uint32_t port) = 0;`,
      `virtual void tick(uint32_t now) = 0;`,
      `virtual void apply(const Hal::CommandProcessorToHardwareCommand&) = 0;`,
      `virtual void apply(const Hal::DrivetrainToHardwareCommand&) = 0;`,
      a virtual no-op `begin()`, and `static constexpr uint32_t kPortCount = 4;`
      — no more, no less (no speculative extra methods).
- [x] `Subsystems::NezhaHardware` becomes `: public Subsystems::Hardware`;
      `override` added to all four methods; its own `kPortCount`
      redeclaration is removed (inherited from the base); no method body
      changes. `main.cpp`'s existing
      `static_assert(Config::kMotorConfigCount == Subsystems::NezhaHardware::kPortCount, ...)`
      still compiles unchanged (inherited static member lookup).
- [x] `source/dev_loop.{h,cpp}` compiles with no `#include "MicroBit.h"` and
      no `Subsystems::Communicator` dependency. `devLoopTick`'s body
      reproduces `main.cpp`'s current loop **exactly**: `hardware.tick(now)`
      (slice 1), statement-triggered `watchdog->feed(now)` +
      `processor->process(...)` only when `statement != nullptr`, outbox
      drain (`hasHardwareCommand`/`hasDrivetrainCommand`), Drivetrain
      governance when `drivetrain->active()`, `hardware.tick(now)` (slice
      2), then the watchdog check that applies
      `buildBroadcastNeutral()`/`buildDrivetrainStop()` and emits
      `EVT dev_watchdog` via `DevLoop`'s `defaultReply`/`defaultReplyCtx`.
- [x] `commands/dev_commands.h`'s `DevLoopState::hardware` retypes from
      `Subsystems::NezhaHardware*` to `Subsystems::Hardware*`; its
      `#include "subsystems/nezha_hardware.h"` is replaced with
      `#include "subsystems/hardware.h"`. `dev_commands.cpp`'s
      `handleDevState`'s port-count loop bound reads
      `Subsystems::Hardware::kPortCount` (no file in `commands/` names the
      concrete `NezhaHardware` type any more).
- [x] `source/types/clock.h` declares `uint32_t systemClockNow(); // [ms]`;
      `source/types/clock.cpp` (on-target) implements it as
      `return system_timer_current_time();`; `source/types/clock_host.cpp`
      (new, host-only — not yet linked into any build until ticket 004's
      CMake source list includes it, mirroring 079's
      `i2c_bus_host.cpp`/`i2c_bus.cpp` split) implements it as a settable
      global plus `setHostClockNow(uint32_t now)`.
      `commands/system_commands.cpp`'s `handlePing` calls
      `Types::systemClockNow()` instead of `system_timer_current_time()`
      directly; `handleId`'s `microbit_friendly_name()`/
      `microbit_serial_number()` calls sit behind an
      `#ifdef HOST_BUILD` branch returning fixed host identity strings
      (e.g. `"HOST-SIM"` / a fixed serial number), with the on-target
      branch unchanged.
- [x] `source/main.cpp`'s loop body collapses to: read the clock, tick the
      Communicator, build a `DevLoopStatement` from any taken statement
      (`nullptr` if none), call `devLoopTick(loop, now, stmt)`. The
      Communicator stays the only CODAL-touching piece in the loop, per the
      design write-up's own constraint.
- [x] **ARM build + bench smoke (hardware-bench-testing gate, required):**
      deploy (`mbdeploy deploy --build`) and confirm PING, the DEV M/DT
      family, and the watchdog `EVT dev_watchdog` path all round-trip
      byte-identically to pre-ticket behavior.
      **DONE (team-lead, 2026-07-05, on the stand, Tovez, v0.20260705.8).**
      `just build-clean` (correct uv env) + `mbdeploy deploy robot --hex …`,
      then `tests/bench/dev_exercise.py --skip-dt` ×3. All substantive
      round-trips PASS every run: PING, VER, DEV M 1–4 STATE+CAPS, DUTY
      position-climb, VEL-120 convergence (~128 mm/s, matching 001's
      envelope), VOLT capability-reject (`ERR unsupported`), and the watchdog
      `EVT dev_watchdog` fire (proves the loop-originated
      `defaultReply`/`defaultReplyCtx` sink emits correctly). RESET flaked to
      `None` once (intermittent DEV serial reply-drop, identical to what the
      byte-for-byte 001 build did — a transport artifact, not a loop
      regression; PASS on both re-runs, 18/18). Confirms the `devLoopTick`
      extraction + `Subsystems::Hardware` seam + the `comm.tick`/`hardware.tick`
      slice-1 reorder are behavior-neutral on real hardware.
      NOTE (separate follow-up, not a 002 defect): the documented
      `mbdeploy deploy --build` path is broken in this env — `build.py` runs
      `gen_messages.py` via `sys.executable`, which under mbdeploy is its pipx
      venv (no `grpcio-tools`/`google.protobuf`); `just build*` uses the uv env
      (protobuf 6.33.6) and works. Worked around by building with `just` and
      flashing with `--hex`.
- [x] Existing `tests/sim/unit/*` harnesses still pass with no regression.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/sim tests/unit`;
  every existing `tests/sim/unit/*_harness.cpp` (these harnesses touch
  `Hal::Motor`/`Subsystems::Drivetrain`/`CommandProcessor`, none of which
  change signature in this ticket, only `DevLoopState::hardware`'s type and
  `NezhaHardware`'s inheritance — confirm none of the harnesses name
  `Subsystems::NezhaHardware` directly in a way that would break).
- **New tests to write**: a small standalone harness (ad hoc compile,
  matching the existing convention) instantiating
  `Subsystems::NezhaHardware` behind a `Subsystems::Hardware*` and calling
  `motor()`/`tick()`/both `apply()` overloads through the base pointer, to
  prove the abstract seam is real (not just declared) before
  `Subsystems::SimHardware` exists to provide a second implementation.
- **Verification command**: `uv run python -m pytest tests/sim -q`, plus the
  ARM build + bench smoke pass described above.

## Implementation Plan

**Approach:**

1. Write `source/subsystems/hardware.h` (the abstract base, no `.cpp` —
   pure interface, matching `hal/capability/*.h`'s headers-only
   convention).
2. Retrofit `source/subsystems/nezha_hardware.h`/`.cpp`: add
   `#include "subsystems/hardware.h"`, change the class declaration to
   `class NezhaHardware : public Hardware`, add `override` to
   `motor()`/`tick()`/both `apply()` overloads, remove the redundant
   `kPortCount` redeclaration.
3. Write `source/dev_loop.h` declaring `DevLoopStatement`, `DevLoop`
   (holding `Subsystems::Hardware*`, `Subsystems::Drivetrain*`,
   `CommandProcessor*`, `SerialSilenceWatchdog*`, `DevLoopState*`, and the
   `defaultReply`/`defaultReplyCtx` pair), and
   `void devLoopTick(DevLoop&, uint32_t now, const DevLoopStatement*);`.
   Write `source/dev_loop.cpp` implementing it — copy `main.cpp`'s current
   loop body verbatim, adapting only the statement-feed step (read from the
   `DevLoopStatement*` parameter instead of directly from `comm`) and the
   watchdog-fire EVT emission (via `defaultReply`/`defaultReplyCtx` instead
   of the hardcoded `serialReply`/`&comm`).
4. Write `source/types/clock.h`; `source/types/clock.cpp` (on-target);
   `source/types/clock_host.cpp` (host-only, unlinked until ticket 004).
5. Edit `source/commands/dev_commands.h`/`.cpp`: retype
   `DevLoopState::hardware`, swap the `#include`, update the port-count
   loop bound in `handleDevState`.
6. Edit `source/commands/system_commands.cpp`: replace the direct
   `system_timer_current_time()` call with `Types::systemClockNow()`; gate
   the identity calls in `handleId` under `#ifdef HOST_BUILD`.
7. Rewrite `source/main.cpp`'s loop body to call `devLoopTick()`, keeping
   the Communicator tick/statement-take logic in `main.cpp` itself (per
   architecture-update.md's constraint that Communicator never enters the
   shared body).
8. Deploy and run the bench smoke sequence per
   `.claude/rules/hardware-bench-testing.md`.

**Files to create:**
- `source/subsystems/hardware.h`
- `source/dev_loop.h`
- `source/dev_loop.cpp`
- `source/types/clock.h`
- `source/types/clock.cpp`
- `source/types/clock_host.cpp`

**Files to modify:**
- `source/subsystems/nezha_hardware.h`
- `source/subsystems/nezha_hardware.cpp`
- `source/commands/dev_commands.h`
- `source/commands/dev_commands.cpp`
- `source/commands/system_commands.cpp`
- `source/main.cpp`

**Testing plan:** see "Testing" section above.

**Documentation updates:** none required to `docs/protocol-v2.md` (no wire
change). If `devLoopTick`'s extraction changes anything about how a future
maintainer should read `main.cpp`, a short comment update in `main.cpp`
itself (matching its existing extensive header-comment style) is
appropriate — no separate doc file needed.
