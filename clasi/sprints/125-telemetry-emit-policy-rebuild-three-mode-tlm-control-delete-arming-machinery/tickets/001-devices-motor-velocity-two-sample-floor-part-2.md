---
id: '001'
title: Devices::Motor velocity() two-sample floor (Part 2)
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: telemetry-emit-policy-rebuild-spec.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Devices::Motor velocity() two-sample floor (Part 2)

## Description

Fix the source-level defect that motivated telemetry's now-deleted arm-2
gate: the first encoder read after power-on can report a bogus nonzero
velocity on a robot that has never moved, because a velocity is a
difference quotient and there is no second sample yet to difference
against. Per the issue (Part 2), `velocity()` on `Devices::Motor`'s
concrete leaf(s) must return `0.0f` until the device has collected at
least two valid samples; the first sample establishes a baseline only,
it does not yield a velocity.

This is a prerequisite for ticket 002, which deletes the
`changeReportingArmed_` gate on telemetry's coast-down arm (Part 1 item
7) — that gate exists ONLY to suppress this bogus reading, and Part 2 is
explicit that it must be fixed here, not with a telemetry-side guard.
Land this ticket first so 002 can delete the gate with nothing left
relying on it.

Scope note: `Devices::NezhaMotor` (`src/firm/devices/nezha_motor.{h,cpp}`)
is the primary target — its `tick()` doc comment already documents the
naive per-tick difference-quotient `velocity()`/`position()` behavior
(see the file's own header comment, "the freshness gate ... DELETED
OUTRIGHT"). Check whether any other concrete `Devices::Motor` (sim/fake
leaf used by `src/sim`/`src/tests`) has an independent `velocity()`
implementation that needs the same fix — if it shares NezhaMotor's
sampling code path, no separate change is needed there; state which is
true in the ticket's completion notes.

## Acceptance Criteria

- [x] `Devices::Motor::velocity()` (and every concrete leaf) returns
      exactly `0.0f` until two valid samples have been collected since
      construction/`begin()`.
- [x] After a second valid sample arrives, `velocity()` returns the real
      difference-quotient value (unchanged behavior from today, just no
      longer computed from a single, meaningless sample).
- [x] New unit test (per the issue, Part 2 exactly): construct, connect,
      collect one sample → `velocity() == 0`; second sample → real,
      nonzero-capable value.
- [x] No telemetry-side guard is added anywhere for this condition — grep
      `src/firm/app/telemetry.*` confirms no new sample-count/first-read
      special case was introduced there.
- [x] `position()` is unaffected — this ticket touches `velocity()` only;
      position on a single sample is still a real, valid position.

## Completion notes

**Finding: the source-level fix was already in place.** `Devices::
NezhaMotor::tick()` (`src/firm/devices/nezha_motor.cpp`) already carries a
`hasLastTick_`-gated "boot anchor" — the first-ever `tick()` call after
construction/`begin()` seeds `lastPosition_`/`lastTickUs_` but explicitly
does NOT compute `velocity_` (see the `else` branch's own "Boot anchor"
comment); `velocity_` only updates on the SECOND and later calls, as a real
difference quotient between two genuinely collected samples. This landed in
commit `29ec28c2` ("pid-removal: reapply 125-003 NezhaMotor shrink + fix its
disclosed defect"), predating this ticket's authorship. `begin()`
(`hardReset()`) also resets `hasLastTick_ = false` and `velocity_ = 0.0f`,
so the two-sample floor re-arms after every hard reset, not just at
construction.

No production code change was required. What this ticket added: a unit
test making the two-sample-floor guarantee explicit and literal per the
issue's own Testing section — `scenarioVelocityReadsZeroUntilTwoValidSamplesCollected()`
in `src/tests/sim/unit/devices_motor_harness.cpp`. It deliberately scripts
a NONZERO position (3.0mm) at a NONZERO `nowUs` (20ms) for the first
sample — a naive per-tick diff with no anchor concept would compute
`(3.0 - 0.0) / 0.020 == 150 mm/s` against the phantom zero-initialized
pre-boot baseline, exactly the bogus-nonzero-velocity defect Part 2
describes — and asserts `velocity() == 0.0f` there, then asserts the real
`250 mm/s` difference quotient on the second sample.

**Other concrete `Devices::Motor` leaves:** grepped the whole repo for
`: public Devices::Motor` / `: public Motor`. Only two real classes exist:
`Devices::NezhaMotor` (the leaf fixed above) and `Devices::MotorArmor`
(`motor_armor.h`), which is a pure decorator — its `velocity()` override is
a straight `return inner_.velocity();` forward with no sampling logic of
its own (`motor_armor.h` line 90). `src/sim` composes the bare
`NezhaMotor` directly (no separate sim/fake leaf — confirmed against
`motor.h`'s own header, "The sim composes the bare NezhaMotor directly").
The only other `Devices::Motor` subclass in the tree is `MockMotor`, a
test-only double in `devices_motor_harness.cpp`/`app_fake_otos_harness.cpp`
that returns a test-set mock velocity with no sampling logic — not a
production leaf, out of scope. No second fix site exists.

Verification run: `uv run python -m pytest src/tests/sim/unit/test_devices_motor.py`
passes (new scenario green, `bus.errCount() == 0`). `uv run python3
build.py` (`just build` equivalent) built both the ARM firmware
(`MICROBIT.hex`, FLASH 41.58% used) and the HOST_BUILD sim lib
(`libfirmware_host`) clean — no source outside the test harness was
touched, so this was a no-regression check, not a fix-verification build.
Full `uv run python -m pytest src/tests/sim/ -q`: 415 passed, 1 skipped, 1
xfailed, 3 failed. The 3 failures (`test_fault_knobs.py`,
`test_scripted_twist_demo.py`, `test_sim_api.py`, all failing on
`kFlagEventBootReady`/"telemetry frame after boot") are PRE-EXISTING on
this branch, unrelated to this ticket — confirmed by stashing this
ticket's only change (`devices_motor_harness.cpp`) and re-running:
identical 3 failures with the exact same assertions. They stem from this
branch's already-uncommitted, in-progress `telemetry.{h,cpp}` edits (Part
1/Part 3 boot-flag work, ticket 125-002's territory), which were dirty in
the working tree before this ticket started and were left untouched per
this ticket's scope discipline.

## Testing

- **Existing tests to run**: the full `src/tests/sim` unit suite
  (`uv run python -m pytest src/tests/sim` or the project's HOST_BUILD
  sim-test target — confirm the exact invocation from an existing CI/just
  recipe) to confirm no existing test depended on the old first-sample
  bogus-velocity behavior.
- **New tests to write**: a `Devices::Motor`/`NezhaMotor` unit test
  (construct → `begin()` → one `tick()` → assert `velocity() == 0`;
  second `tick()` with a distinct encoder reading → assert `velocity()`
  reflects the real difference quotient). Place alongside existing
  motor/device unit tests (see `src/tests/sim/unit/` for the project's
  existing device-test layout and naming convention).
- **Verification command**: `uv run pytest` (host-side) plus whatever
  `just`/CMake target builds and runs the HOST_BUILD sim unit tests for
  `src/firm/devices/` — confirm the exact target name from the existing
  build config before running.
