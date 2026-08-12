---
id: '006'
title: Control law out of core/ -- new control/ layer
status: open
use-cases: ["SUC-002"]
depends-on: ["005"]
github-issue: ''
issue: firmware-layering-cleanup-interfaces-to-hal-impls-to-platform-microbit.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Control law out of core/ -- new control/ layer

## Description

Phase 4 of the layering-cleanup issue. `core/differential_drive.{h,cpp}`
is the wheel-speed **control law** — `fastPid()` plus the Stage A/B/C
correction, bias adaptation, and stall/deficit-latch machinery — not
orchestration and not kinematics. Give it a new, single-purpose home:
`src/firm/control/`, holding `Control::DifferentialDrive` (renamed from
`Core::DifferentialDrive`).

**This is a pure relocation.** The control law itself — every PID gain,
every stage, every stall-detection threshold — is untouched. Only the
namespace (`Core::` → `Control::`) and file location change.

See sprint.md's Design Rationale for why `control/` is a new top-level
layer rather than folded into `kinematics/` (stateless geometry, would
break its cohesion) or left in `core/` (orchestration, not control) or
folded into `hal/` (interfaces-only, no stateful logic anywhere else in
it). Layer position: `hal/` → `control/` → `core/` — `control/` may reach
down into `hal/` and `firm/types/`, but not into `core/` or `motion/`. No
`test_layer_isolation.py` entry is needed (matching `core/`,
`kinematics/`, `motion/`'s existing treatment — none of the three are in
that test's three-layer table).

The `Hal::Wheel` migration described in `hal/DESIGN.md` §4 (moving this
control law onto a closed-loop actuator abstraction) stays deferred — that
is a behavioral change to the code that drives the robot and belongs in
its own future sprint, not this one.

## Acceptance Criteria

- [ ] `src/firm/control/differential_drive.{h,cpp}` exists, class renamed
      `Control::DifferentialDrive` (from `Core::DifferentialDrive`), logic
      byte-for-byte unchanged — no PID gain, stage, or stall threshold
      edited.
- [ ] `core/differential_drive.{h,cpp}` no longer exists.
- [ ] Every call site updated: `core/boot_wiring.*`'s composition,
      `core/robot_loop.*`'s tick call, any test harness that names the
      class or its old path.
- [ ] `src/firm/control/DESIGN.md` created: purpose (own the closed-loop
      wheel-speed control law), boundary (`hal/` and `firm/types/` only,
      downward), and a note that this is where the deferred `Hal::Wheel`
      migration will eventually land — not part of this ticket.
- [ ] No `test_layer_isolation.py` entry added for `control/`.
- [ ] Every explicit (non-glob) source list naming
      `core/differential_drive.cpp` by path updated to
      `control/differential_drive.cpp` — `src/firm/platform/host/
      CMakeLists.txt`'s literal reference (confirmed during planning)
      re-verified and updated, plus a fresh sweep of the full ~49-file
      explicit-list set for any other reference, excluding
      `.claude/worktrees/rogo-revival/`.
- [ ] `just build-sim` and ARM build both clean.
- [ ] Any unit test targeting the control law directly (wherever it's
      compiled today) updated for the new namespace/path, with its own
      assertions unchanged in content.

## Testing

- **Existing tests to run**: `just build-sim`, `uv run python -m pytest
  src/tests/sim/unit/test_layer_isolation.py`, ARM build.
- **New tests to write**: none required — pure relocation; existing
  control-law coverage must pass unmodified in content under its new
  name/path.
- **Verification command**: `just build-sim && uv run python -m pytest
  src/tests/sim/unit/test_layer_isolation.py`
