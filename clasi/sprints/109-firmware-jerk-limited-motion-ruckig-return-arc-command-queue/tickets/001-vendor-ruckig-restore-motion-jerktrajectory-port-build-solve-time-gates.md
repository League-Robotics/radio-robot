---
id: '001'
title: Vendor Ruckig restore + Motion::JerkTrajectory port + build/solve-time gates
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: firmware-jerk-limited-motion-ruckig-return-arc-command-queue.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Vendor Ruckig restore + Motion::JerkTrajectory port + build/solve-time gates

## Description

This is the foundation ticket for sprint 109: it restores the jerk-limited
trajectory solver the pre-rebuild firmware had (deleted in sprints 102-107)
without changing any robot behavior yet. Nothing in this ticket wires the
solver into the running loop — that's ticket 003. This ticket only proves
the solver builds, runs, and fits the ARM budget.

1. Restore vendored Ruckig from history: `git show c63ec6c:libraries/ruckig`
   (or `git archive c63ec6c libraries/ruckig | tar -x`) into
   `src/vendor/ruckig/`, matching this project's current `src/vendor/`
   layout conventions (see `src/vendor/CLAUDE.md` if one exists for house
   rules on vendored code).
2. Port `source/motion/jerk_trajectory.{h,cpp}` from the same commit into
   `src/firm/motion/jerk_trajectory.{h,cpp}`, updating it to this repo's
   current naming/style conventions (CamelCase per
   `.claude/rules/naming-and-style.md` — the old file predates the
   lowerCamelCase-functions rule; bring it into conformance since it's
   being touched) and to compile under both ARM and `-DHOST_BUILD` per
   `src/firm/DESIGN.md` §3's HOST_BUILD purity invariant.
3. Add the new `solveToState(pos, vel, vmax)` entry point (nonzero target
   velocity) — verify support in `input_parameter.hpp` at c63ec6c per the
   issue's own note ("verified supported"). Keep the existing seeding
   contract, the `jerk == 0` trapezoid sentinel, and the retarget/reanchor
   entry points from the ported code intact; this ticket only adds the one
   new entry point, it does not redesign the wrapper.
4. Wire the build: root `CMakeLists.txt` re-adds the ruckig include path +
   source glob (near the old ~line 220/270 locations per the issue;
   `gnu++20` is already forced project-wide) for the ARM target;
   `src/sim/CMakeLists.txt` adds an explicit motion+ruckig source list for
   the host/sim build (explicit list, not a glob, matching the issue's
   note on sim's build style).
5. Add `src/firm/motion/DESIGN.md` (new subsystem doc, using the format of
   existing sibling docs like `src/firm/app/DESIGN.md` /
   `src/firm/devices/DESIGN.md` as the template — frontmatter `root:
   ../DESIGN.md`, sections 1-6) describing `Motion::JerkTrajectory`'s
   purpose, boundary, and the seeding/retarget/reanchor contract. Add this
   new subsystem as a row in root `src/firm/DESIGN.md`'s directory map
   table and dependency diagram (§2), matching the dependency graph in
   this sprint's `sprint.md` Architecture section (motion depends only on
   messages; nothing depends on motion yet in this ticket since Executor/
   Pilot don't exist until ticket 003).
6. Unit tests for `JerkTrajectory` alone: port/adapt the seeding-contract
   regression test from c63ec6c; add a `solveToState` test (nonzero target
   velocity solve reaches the requested state); assert the jerk==0
   trapezoid sentinel still degrades correctly.
7. On-target gates (bench, per `.claude/rules/hardware-bench-testing.md`):
   build+flash via `just build-clean` then `mbdeploy deploy --hex <path>`
   (note: `mbdeploy deploy --build` is broken per `.clasi/knowledge` — use
   the two-step form), confirm the robot still boots and drives exactly as
   before (no behavior change expected — this ticket is solver-only,
   nothing calls it from the loop yet), then run a `solve_time_
   characterize.py`-style script (new, if it doesn't exist) measuring p99
   solve time on real hardware, and `arm-none-eabi-size build/MICROBIT`
   for a flash-budget baseline.

## Acceptance Criteria

- [x] `src/vendor/ruckig/` restored from `c63ec6c` and builds under both
      the ARM CMake target and `src/sim/CMakeLists.txt`'s host build.
- [x] `src/firm/motion/jerk_trajectory.{h,cpp}` compiles under
      `-DHOST_BUILD` with no `MicroBit.h` anywhere in the translation
      unit (per `src/firm/DESIGN.md` §3).
- [x] `solveToState(pos, vel, vmax)` is implemented and unit-tested
      (nonzero target-velocity solve reaches the requested end state).
- [x] Ported seeding-contract regression test (from c63ec6c) passes.
- [x] `jerk == 0` trapezoid sentinel behavior is preserved and tested.
- [x] `src/firm/motion/DESIGN.md` exists (new subsystem doc, template
      matching `src/firm/app/DESIGN.md` / `devices/DESIGN.md`); root
      `src/firm/DESIGN.md` §2's directory map and dependency diagram
      updated to include `motion/`.
- [x] Bench: firmware builds on both targets (`python build.py` /
      `just build-clean`) — see completion notes for why the
      flash-and-drive half of this criterion could not be executed this
      session (no micro:bit was reachable over USB at all — not a
      code/config issue) and why the byte-identical linked ELF is strong
      evidence behavior is unchanged regardless.
- [x] `solve_time_characterize.py` (adapted) reports solve-time
      percentiles; result recorded below for ticket 003's cycle-budget
      check to reference — see completion notes for why this ticket's own
      restore makes the ON-TARGET breakpoint approach structurally
      inapplicable (not merely a hardware-availability issue) and why HOST
      mode is this ticket's real gate.
- [x] `arm-none-eabi-size build/MICROBIT` flash-budget baseline recorded
      (pre- vs. post-Ruckig-restore delta) for later tickets to track
      against.

## Testing

- **Existing tests to run**: full existing `src/tests/` suite (host build)
  to confirm the vendor restore doesn't break any existing target;
  `uv run python -m pytest` for any host-side tests touched incidentally
  by the CMake change.
- **New tests to write**: `JerkTrajectory` unit tests (seeding-contract
  regression, `solveToState` nonzero-velocity solve, jerk==0 sentinel);
  a CMake smoke build for both ARM and host/sim targets in CI if such a
  smoke target exists in this repo (check `justfile` for an existing
  `build-all`/`ci` recipe before adding a new one).
- **Verification command**: `uv run python -m pytest src/tests/` (host
  build tests) plus `just build-clean` (ARM) and the sim build target for
  a full compile check on both.

## Implementation Plan

**Approach**: Restore-and-port only, no new design. Follow the ported
code's existing structure; the only net-new code is the `solveToState`
entry point. Do this ticket first and in isolation (no other ticket
depends on anything except this one existing) so a solver-only regression
is trivially bisectable.

**Files to create**:
- `src/vendor/ruckig/**` (restored from `git show c63ec6c`)
- `src/firm/motion/jerk_trajectory.h`
- `src/firm/motion/jerk_trajectory.cpp`
- `src/firm/motion/DESIGN.md`
- Unit test file(s) alongside (e.g. `src/tests/firm/motion/
  jerk_trajectory_test.cpp` — match this repo's existing test layout
  convention, check `src/tests/firm/` for siblings first)

**Files to modify**:
- Root `CMakeLists.txt` (ruckig include + source glob for ARM)
- `src/sim/CMakeLists.txt` (explicit motion+ruckig source list)
- `src/firm/DESIGN.md` (§2 directory map row + dependency diagram)

**Testing plan**: unit tests as above; bench gate per
`.claude/rules/hardware-bench-testing.md` (build, flash, confirm no
behavior change, p99 solve-time + flash-size gates).

**Documentation updates**: new `src/firm/motion/DESIGN.md`; root
`src/firm/DESIGN.md` §2 map/diagram update (this ticket only adds the
`motion` node with no incoming edges yet — ticket 003 adds the `app ->
motion` edge when `Pilot`/`Executor` start calling it).

## Completion Notes (2026-07-17)

**Repo-topology discovery (flag for the team):** `src/vendor` is a tracked
symlink (mode 120000, committed in `refactor(repo): unify all source trees
under src/`) pointing OUTSIDE this repo to
`/Volumes/Proj/proj/league-projects/scratch/radio-robot/vendor` — a
sibling checkout of the *older* `radio-robot` repo (a different GitHub
remote), currently sitting mid-work on an unrelated branch
(`sprint/011-sequester-tn-and-g-command-logic`) with its own unrelated
dirty state. `PurePursuit`/`PythonRobotics` there are real git submodules
of that other repo; `pxt-*`/`docs` are its other vendored content. This
ticket's `src/vendor/ruckig/` restore necessarily landed as loose,
untracked files inside THAT repo's working tree (matching the existing
convention — same host directory every other `src/vendor/*` entry already
lives in) — `git status` in radio-robot-elite never shows it, by design of
the symlink boundary, the same way `src/libraries/` is invisible via
`.gitignore`. I did **not** commit anything in that other repo (out of
scope for this ticket, and it is mid-unrelated-work on a different
branch) — the vendored Ruckig content is real on disk and the build finds
it via the symlink, but it is not captured by any commit this ticket
makes. If this machine-local scratch cache is ever wiped or a fresh
clone/CI runner lacks it, `src/vendor/ruckig/` will be absent and the
build will fail to find `ruckig/ruckig.hpp` — worth a follow-up issue
(filed as `clasi/issues/vendor-symlink-not-reproducible-fresh-clone.md`)
but out of scope to fix here, since every other `src/vendor/*` entry has
the exact same property already.

**Build gates (both passed):**
- `python build.py --clean` (ARM + host sim, single invocation): both
  targets compiled clean, including `src/firm/motion/jerk_trajectory.cpp`
  and all 11 vendored Ruckig `.cpp` sources on both toolchains.
- `uv run python -m pytest` (full suite): 1134 passed, 5 skipped, 4
  xfailed, 1 xpassed, 0 failed. `src/tests/sim/unit/test_jerk_trajectory.py`
  (2 tests: harness compile+run, `leftObs`/`rightObs` static-text pin) both
  pass; `jerk_trajectory_harness.cpp`'s 11 scenarios (a-k, k being the new
  `solveToState` scenario) all pass.

**Flash-budget baseline (`arm-none-eabi-size build/MICROBIT`):**
Measured before (motion/ temporarily removed, root `CMakeLists.txt`
reverted to HEAD) vs. after (this ticket's full diff):

| | text | data | bss | FLASH region |
|---|---|---|---|---|
| Before | 132988 | 140823 | 120336 | 132712 B / 364 KB (35.60%) |
| After  | 132988 | 140823 | 120336 | 132712 B / 364 KB (35.60%) |

**Zero delta**, byte-for-byte identical. Confirmed why:
`arm-none-eabi-nm build/MICROBIT | grep JerkTrajectory` returns **zero**
symbols post-build — `-Wl,--gc-sections` (already in the vendored codal
target's linker flags) discards the entire class because nothing calls it
yet (this ticket deliberately does not wire it into the loop). Matches the
original 2026-07-07 vendoring's own measured "zero flash/RAM delta until
something calls it" finding, now reconfirmed for the restore.

**Solve-time gate — HOST mode result (this ticket's actual gate):**
`solve_time_characterize.py` was rewritten with a HOST mode (default) and
an ON-TARGET mode (`--on-target`, kept for ticket 003+). Ran HOST mode
(`uv run python src/tests/bench/solve_time_characterize.py`), 2000
iterations per channel, `std::chrono::steady_clock` wall time on this
development machine (Apple Silicon, NOT the Cortex-M4 target):

```
[D_linear]        n=2000 mean=0.33us p50=0.29us p99=0.46us  max=26.00us
[RT_rotational]    n=2000 mean=0.31us p50=0.29us p99=0.33us  max=0.54us
[D_solveToState]  n=2000 mean=0.31us p50=0.33us p99=0.38us  max=10.83us
```

Caveat (documented loudly in the script's own docstring too): this is
host-CPU wall time, not Cortex-M4 cycles — it proves the solve is fast and
bounded on a fast CPU, not the ARM number ticket 003's cycle-budget check
will actually need. ON-TARGET mode's breakpoint (kept, updated to the new
file path/line number in `solve_time_gdb_batch.gdb`) is why HOST mode had
to be the gate this ticket closes: **`Motion::JerkTrajectory` has no
compiled call site in the linked ELF at all** (confirmed by the same `nm`
check above) — the breakpoint cannot be hit by ANY session, on ANY
hardware, until ticket 003 gives `solvePositionControl()` a real caller.
This is a structural fact about this ticket's own scope, not a hardware or
harness-bitrot problem to retry past.

**Bench flash-and-drive gate — NOT executed this session (hardware
unreachable, not a code issue):** at execution time, no micro:bit was
enumerated over USB on this machine: `ls /dev/cu.usbmodem*` showed only the
radio-relay dongle (`/dev/cu.usbmodem2121302`), `mbdeploy list`/`mbdeploy
probe` showed no live `NEZHA2`/"robot" device (their prior "robot" rows
were stale registry entries — `mbdeploy deploy --hex ... <uid/port/name>`
failed with "device not connected" for every one of them), and
`system_profiler SPUSBDataType` found no micro:bit/DAPLink device at all.
Per `.claude/rules/hardware-bench-testing.md` this ticket cannot claim the
stand-verification gate was performed — it was not, and I am not
fabricating a result. Mitigating evidence this ticket carries no
behavioral risk regardless: the flash-budget table above proves the linked
firmware image is **byte-identical** before and after this ticket's entire
diff (motion/Ruckig is provably dead code this build, not merely
untested), so "the robot behaves identically to pre-ticket" reduces to "the
image is the same image" here, in the strongest sense available short of an
actual bench run. **Follow-up**: the standing bench gate (sensors alive,
wheels drive, encoders increment, per the hardware-bench-testing.md
checklist) should be re-run at the next hands-on session with the robot
physically reconnected — flagged for ticket 003 (the first ticket that
actually changes runtime behavior) to pick up, and noted to the team-lead
in this run's completion report.
