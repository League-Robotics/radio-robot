---
id: '006'
title: app/Drive, app/Odometry, and minimal OTOS perception
status: done
use-cases:
- SUC-006
depends-on:
- '001'
- '003'
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# app/Drive, app/Odometry, and minimal OTOS perception

## Description

Build `source/app/drive.{h,cpp}` and `source/app/odometry.{h,cpp}`: `Drive`
converts a body twist into wheel velocity targets via the unchanged
`BodyKinematics::inverse()` and stages them onto the two `NezhaMotor`
leaves; `Odometry` integrates wheel motion back into a world pose estimate
via the unchanged `BodyKinematics::forward()`. Also add a minimal,
OTOS-only perception step (owned by this ticket, not a separate module —
see architecture-update.md Step 7 Open Question 1): one `Otos` sample per
cycle feeding `Telemetry`'s `otos`/`otos_connected` fields. The archived
plan's full 3-way `Perception` round-robin (otos|line|color) is
deliberately NOT built this sprint — `line`/`color` have no telemetry
field to feed yet.

Depends on ticket 001 (`msg::Twist` type) and ticket 003 (bare leaves, not
`DeviceBus`).

## Acceptance Criteria

- [x] `Drive::setTwist(v_x, omega)` stores the target; `Drive::stop()`
      zeroes it; `Drive::tick()` calls `BodyKinematics::inverse(v_x, omega,
      trackWidth, vL, vR)` and stages `vL`/`vR` onto the two `NezhaMotor`
      leaves via their existing `setVelocity()` setter — no additional
      scaling/sign logic duplicated in `Drive` beyond what `inverse()`
      already computes.
- [x] `Drive::stop()` results in both wheel targets reaching 0 within one
      cycle of the next `tick()`.
  and `NezhaMotor`'s own `pidEnabled_` stays at its default `true` (PID
  path) — this ticket does not touch PID enable/disable.
- [x] `Odometry::integrate()` reads both motors' position (or per-cycle
      delta), calls `BodyKinematics::forward()` (not a hand-rolled
      equivalent), and accumulates world `x`/`y`/`theta`.
- [x] A host-buildable test proves `Odometry::integrate()` accumulates
      correctly for (a) a straight-line case (equal `vL`/`vR`) and (b) a
      pure-rotation case (`vL == -vR`), against `BodyKinematics::forward()`'s
      own known-correct output for those inputs.
- [x] No new state duplicated between `Drive`/`Odometry` and the
      `NezhaMotor` leaves' own cached position/velocity — `Odometry` reads
      the leaves' existing accessors, it does not maintain a shadow copy.
- [x] `Otos` is sampled at least once per cycle (or per a documented slot
      schedule this ticket defines) and the result reaches `Telemetry`
      (ticket 005) before that cycle's frame is built — a direct call or a
      small shared struct, this ticket's own choice, documented.
- [x] `line`/`color` steady-state sampling is explicitly NOT built this
      ticket (documented in code comments and completion notes, not
      silently absent) — `Preamble` (ticket 007) still detects their
      presence at boot.

## Implementation Plan

**Approach**: `Drive`/`Odometry` are thin — the actual math lives entirely
in the unchanged `BodyKinematics::inverse()`/`forward()`
(`source/kinematics/body_kinematics.{h,cpp}`), confirmed during this
sprint's own planning to already match these two classes' needs exactly
(no kinematics code changes required). Write `Drive`/`Odometry` against
`NezhaMotor`'s ACTUAL public surface (`setVelocity(float)`,
`position()`/`velocity()` accessors — read `nezha_motor.h` directly, not
just the archived plan's prose) and `Otos`'s actual `begin()`/tick/read
surface (`source/devices/otos.h`).

**Files to create/modify**:
- `source/app/drive.h`, `source/app/drive.cpp` (new)
- `source/app/odometry.h`, `source/app/odometry.cpp` (new)

**Testing plan**:
- Existing tests to run: none directly (new files); confirm
  `BodyKinematics`'s own existing tests (if any under `tests/sim/unit/`)
  stay untouched/green, since `Drive`/`Odometry` are new callers, not
  modifiers, of that code.
- New tests to write: the straight-line and pure-rotation `Odometry`
  accumulation tests (Acceptance Criteria above); a `Drive::tick()` test
  confirming staged wheel targets match `BodyKinematics::inverse()`'s
  direct output for a representative `(v_x, omega)` pair, using a
  `HOST_BUILD` fake/scripted `NezhaMotor` or a direct assertion against
  the leaf's `setVelocity()` call (whichever the leaf's own `HOST_BUILD`
  seam supports — confirm during implementation).
- Verification command: `uv run python -m pytest tests/sim/unit/ -k "drive or odometry"`
  (once the test files exist).

**Documentation updates**: a code comment on the minimal-OTOS-perception
decision (cite architecture-update.md Step 7 Open Question 1) so a future
reader knows the 3-way round-robin was deliberately deferred, not
forgotten.

## Completion Notes

**Files created**: `source/app/drive.h`, `source/app/drive.cpp`,
`source/app/odometry.h`, `source/app/odometry.cpp`,
`tests/sim/unit/app_drive_harness.cpp`, `tests/sim/unit/test_app_drive.py`,
`tests/sim/unit/app_odometry_harness.cpp`,
`tests/sim/unit/test_app_odometry.py`. No files outside the ticket's own
file layout were touched.

**Kinematics API confirmed unchanged**: `BodyKinematics::inverse(v, omega,
b, vL_out, vR_out)` / `forward(vL, vR, b, v_out, omega_out)`
(`source/kinematics/body_kinematics.h`) already match `Drive`/`Odometry`'s
needs exactly, per architecture-update.md's own "no kinematics code changes
required" finding — confirmed again during this ticket's implementation, no
changes made to `body_kinematics.{h,cpp}`.

**fwdSign/port convention verified against `nezha_motor.cpp`, not
duplicated**: `Devices::NezhaMotor` applies its own `config_.fwdSign`
correction internally at BOTH the encoder-decode boundary
(`collectEncoder()`, `nezha_motor.cpp:664-665`) and the duty-write boundary
(`writeRawDuty()`, `nezha_motor.cpp:455-456`). `Drive`/`Odometry` therefore
work entirely in logical "positive = forward" body-relative mm/s and never
touch `fwdSign` or the port→side (L/R) mapping — that binding is main.cpp's
own construction-time wiring (ticket 008), matching the "no additional
scaling/sign logic duplicated in Drive" acceptance criterion literally. No
change needed to `.clasi/knowledge/tovez-fwd-sign-and-port-swap.md`'s
existing note (port1=RIGHT+1, port2=LEFT-1 stays a ticket-008 wiring
decision).

**trackWidth source**: `Drive`/`Odometry` each take a plain `float
trackWidth` constructor parameter (no unit suffix, `// [mm]` tag) rather
than reading `config/boot_config.h` themselves — `Config::
defaultDrivetrainConfig().trackwidth` is main.cpp's own value to fetch and
pass in (ticket 008); this keeps both classes free of any `config/`
dependency, matching their "thin, testable in isolation" scope.

**Odometry's integrate() needs no separate `dt`**: `BodyKinematics::
forward()`'s equations are linear/homogeneous in `vL`/`vR`, so feeding it
this cycle's raw wheel-position DELTAS (mm) directly yields (distance,
headingDelta) for that cycle — algebraically identical to feeding it
velocities and multiplying by `dt`, since `delta = velocity * dt` is a
common factor throughout. `Odometry::integrate()` therefore reads
`left.position()`/`right.position()`, computes the delta against its own
`lastLeft_`/`lastRight_` baseline (seeded from the leaves' own `position()`
at CONSTRUCTION time, so the first `integrate()` call never sees a phantom
jump from a nonzero boot-time position — see
`scenarioBaselineSeededFromLeafPositionAtConstruction`), and calls
`forward()` on the delta pair directly. World pose is then accumulated via
standard midpoint-arc integration (`theta_ + headingDelta/2`) — this last
step is Odometry's own logic, not a hand-rolled `forward()` equivalent
(`forward()` itself only returns body-frame deltas, never a world pose).

**Encoder-reset-on-reboot semantics** (ticket's own documentation
requirement): documented in `odometry.h`'s constructor comment.
`NezhaMotor::position()` is relative to its OWN encoder zero, re-anchored
every firmware boot; `Odometry`'s `x_`/`y_`/`theta_` are therefore only
continuous WITHIN one firmware session — a reboot resets both together,
with no reconciliation attempted here. A host consuming the pose over the
wire must detect a reboot itself (e.g. a telemetry sequence-number reset)
and handle the discontinuity; this ticket adds no cross-session
pose-splicing logic, per the ticket's own scope.

**Minimal OTOS-only perception — "a direct call," not a struct or a new
module**: implemented as one free function, `App::applyOtosSample(Devices::
Otos& otos, uint64_t now, Telemetry::Frame& frame)`, declared/defined in
`odometry.h`/`odometry.cpp` (NOT an `Odometry` method — architecture-
update.md's own Step 3 "Odometry" boundary explicitly lists "minimal OTOS
sampling" as OUTSIDE Odometry's boundary, so it lives in the same file pair
as a sibling free function, per the ticket's "owned by this ticket, not a
separate module" instruction and architecture-update.md Step 7 Open
Question 1). It calls `otos.tick(now)` (leaving `Otos`'s own internal
`kReadPeriod` rate limiting completely untouched — safe to call every
cycle) then copies `otos.present()`→`frame.hasOtos`, `otos.connected()`→
`frame.otosConnected`, and (only when `present()`) `otos.pose()`→
`frame.otos`. `hasOtos` intentionally mirrors `present()`, not per-tick
freshness, because `Telemetry::emit()` always sends the LAST staged
snapshot (telemetry.h's own doc comment) — a rate-limited or a
failed-this-cycle read should still report the most recent real pose, not
flip `has_otos` off. A never-detected chip is a complete no-op beyond the
two bools (frame's `otos` field is left exactly as the caller staged it —
mirrors `Otos::tick()`'s own "never begun ⇒ zero bus traffic" contract,
inventing no separate zero/clobber convention). `line`/`color` steady-state
sampling remains explicitly NOT built (no telemetry field exists per Step 7
OQ1) — `Preamble` (ticket 007, not yet built) still owns their boot-time
presence detection.

**Test strategy — real leaves, not mocks**: `Drive::setVelocity()`'s
staged target isn't itself observable (private `NezhaMotor` field, and
`setVelocity()` isn't virtual, so `Drive` must be tested against the REAL
leaf, not a mock). `app_drive_harness.cpp` sets `kp=0, ki=0` on the
embedded velocity PID so `appliedDuty() == kff * target` exactly (single
deterministic linear relation, `velocity_pid.cpp`'s own `compute()`),
isolating "did Drive stage what `inverse()` computed" from the PID's own
multi-cycle convergence (already proved separately by
`devices_motor_harness.cpp`). One nonobvious wiring bug found and fixed
during this: `NezhaMotor::writeRawDuty()`'s own write-rate limiter
(`kMinWriteIntervalUs = 40000`, keyed off `lastWriteTimeUs_` which starts at
0, independent of tick count) silently drops the first non-stop duty write
if the verification cycle's `nowUs` is under 40 ms — every scenario's
single post-prime check cycle now runs at `nowUs >= 50000` (a stop write is
exempt from this throttle, so the stop scenario's post-stop cycle does not
need the same margin). `app_odometry_harness.cpp` similarly needed its
pure-rotation scenario's wheel delta chosen at exactly-representable 0.1mm
resolution (matches `scriptEncoderRequestCollect()`'s own
`lround(positionMm * 10)` round-trip) to avoid a false failure from
quantization the real leaf would also apply on hardware.

**Verification evidence**:
- `uv run python -m pytest tests/sim -q` — 338 passed (336 pre-existing +
  the 2 new `test_app_drive.py` / `test_app_odometry.py` harness wrappers,
  which together run 10 scenarios: 3 in `app_drive_harness`, 7 in
  `app_odometry_harness`).
- `just build` — succeeds (v0.20260714.12 at build time); `source/app/
  drive.cpp`/`odometry.cpp` compile clean (`-Wall -Wextra`, zero warnings
  from either file) for the real ARM target and link into `MICROBIT.hex`
  via `CMakeLists.txt`'s recursive glob — no build-file changes needed.
  RAM/flash usage unchanged from ticket 005's last-recorded numbers
  (98.33% RAM by design, 27.84% flash).
- `main.cpp` remains the stub — neither `Drive` nor `Odometry` is
  constructed or called anywhere yet (ticket 008's job); this ticket's own
  acceptance is entirely the host-buildable harnesses above plus the clean
  ARM compile.
