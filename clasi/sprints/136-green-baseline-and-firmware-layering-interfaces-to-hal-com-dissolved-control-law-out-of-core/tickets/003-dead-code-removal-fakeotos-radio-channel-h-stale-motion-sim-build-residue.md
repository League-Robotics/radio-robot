---
id: '003'
title: Dead code removal -- FakeOtos, radio_channel.h, stale motion/sim build residue
status: open
use-cases: ["SUC-002"]
depends-on: ["002"]
github-issue: ''
issue: firmware-layering-cleanup-interfaces-to-hal-impls-to-platform-microbit.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Dead code removal -- FakeOtos, radio_channel.h, stale motion/sim build residue

## Description

**Half B starts here — gated on ticket 002's baseline being done.** This is
Phase 1 of `firmware-layering-cleanup-interfaces-to-hal-impls-to-platform-
microbit.md`: remove confirmed dead code before anything else moves, so
later phases aren't touching (or accidentally preserving) code with zero
live consumers.

Three deletions, each with zero-consumer confirmation already done during
sprint planning (re-verify before deleting, don't just trust this ticket's
inherited claim):

1. **`Core::FakeOtos`** (`core/fake_otos.{h,cpp}`) — an ARM-only bench
   fallback (`-DFAKE_OTOS`, default `OFF`) for a real robot built without a
   physical OTOS chip. Confirmed during planning: `--fake-otos` defaults
   `False` in `build.py`, `option(FAKE_OTOS ... OFF)` in the root
   `CMakeLists.txt`, and no robot JSON, CI script, or `justfile` recipe
   ever sets it. Delete the class, every `#ifdef FAKE_OTOS` site in
   `core/boot_wiring.{h,cpp}` (keep the `Hal::Otos&` seam itself — only the
   `FakeOtos` branch goes; `otos_` binds `realOtos_` unconditionally after
   this), the CMake option, the `build.py` flag, the test harness, and
   every comment reference.
2. **`com/radio_channel.h`** — zero includers repo-wide; no
   `radio_channel.cpp` exists. The radio channel has come from
   `Config::kRadioChannel` (baked from the robot JSON's
   `connection.radio_channel`) since that cutover.
3. **`src/motion`/`src/sim` build residue** — gitignored, zero tracked
   files (`git ls-files` returns nothing for either), whose CMake targets
   (`motion_stop_condition_tests`, `motion_velocity_shaper_tests`,
   `motion_move_queue_chained_tests`) build files sprint 128 ticket 014
   already deleted as dead code.

**Trap**: `.claude/worktrees/rogo-revival/` is a second checkout holding a
full pre-reorganization copy (`src/firm/app/`, `src/motion/`,
`src/firm/devices/`). Every repo-wide grep confirming a "zero consumers"
claim in this ticket must exclude it — it will otherwise report a false
positive.

## Acceptance Criteria

- [ ] `src/firm/core/fake_otos.{h,cpp}` deleted.
- [ ] Every `#ifdef FAKE_OTOS` member/ctor-init block in
      `core/boot_wiring.{h,cpp}` removed; `otos_` binds `realOtos_`
      unconditionally; the `Hal::Otos&` seam itself is **kept**.
- [ ] `option(FAKE_OTOS ...)` and its `-DFAKE_OTOS=ON/OFF` wiring removed
      from the root `CMakeLists.txt`.
- [ ] `--fake-otos` removed from `build.py` (the parser option,
      `print_build_summary`'s `fake_otos` param, the cmake-arg
      construction).
- [ ] `src/tests/sim/unit/app_fake_otos_harness.cpp` and
      `src/tests/sim/unit/test_app_fake_otos.py` deleted.
- [ ] Every comment reference to `FAKE_OTOS`/`FakeOtos` in
      `hardware/DESIGN.md`, `core/DESIGN.md`, `core/debug.h`,
      `src/protos/commands.proto` removed or updated to state it's gone
      (verify each file's current exact reference before editing — don't
      assume the issue's inherited file list is still accurate post-
      reorg).
- [ ] `com/radio_channel.h` deleted — zero-includers reconfirmed by a
      fresh repo-wide grep (excluding `.claude/worktrees/rogo-revival/`)
      immediately before deletion, cited in Completion Notes.
- [ ] `rm -rf src/motion src/sim` — `git ls-files src/motion src/sim`
      confirmed empty immediately before deletion (proves this is not a
      tracked-file deletion), cited in Completion Notes.
- [ ] Any CMakeLists.txt entries referencing `motion_stop_condition_tests`,
      `motion_velocity_shaper_tests`, `motion_move_queue_chained_tests`
      removed.
- [ ] `just build-sim` clean; `uv run python -m pytest
      src/tests/sim/unit/test_layer_isolation.py` passes.
- [ ] Full ARM build (`uv run python3 build.py`) succeeds against the
      default (`FAKE_OTOS=OFF`, `ROBOT_DEBUG=OFF`) flag state.

## Testing

- **Existing tests to run**: `just build-sim`, `uv run python -m pytest
  src/tests/sim/unit/test_layer_isolation.py`, `uv run python3 build.py`
  (ARM).
- **New tests to write**: none — deletion only.
- **Verification command**: `just build-sim && uv run python -m pytest
  src/tests/sim/unit/test_layer_isolation.py`
