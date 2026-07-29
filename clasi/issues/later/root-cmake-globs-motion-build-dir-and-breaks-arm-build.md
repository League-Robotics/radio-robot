---
status: pending
filed: 2026-07-26
filed_by: team-lead (surfaced during sprint 125 ticket 002)
tickets: []
---

# Root CMakeLists globs `src/motion/**/*.cpp` with no `build/` exclusion, breaking `just build`

## The trap

The root `CMakeLists.txt` globs `src/motion/**/*.cpp` **recursively, with no
exclusion for a nested `build/` directory**.

`src/motion` has its own standalone, Python-free `motion_tests` build. The
natural way to run it — and the way sprint 125 ticket 002's own Testing
section instructs — is:

```
cmake -S src/motion -B src/motion/build
cmake --build src/motion/build --target motion_tests
```

That leaves CMake's own generated compiler-probe source
(`CompilerIdCXX/CMakeCXXCompilerId.cpp`) sitting under `src/motion/build/`,
*inside the globbed tree*. The next `just build` sweeps it into the firmware
target and fails with:

```
AvailabilityMacros.h: No such file or directory
```

The failure is confusing because it names a macOS system header while
building ARM firmware, and it appears in a build that touched no source the
developer edited. Nothing in the error points at the stray probe file.

## Workaround used

Move or delete `src/motion/build` (it is gitignored and disposable) before
running `just build`.

## Fix options

- Exclude `*/build/*` from the glob in the root `CMakeLists.txt` (narrowest).
- Better: stop globbing recursively and list sources explicitly, or glob only
  the directories that hold real sources. A recursive glob over a tree that is
  *also* a valid CMake source root is inherently fragile.
- Alternatively, standardize the `motion_tests` build directory somewhere
  outside the source tree entirely (e.g. `build/motion-tests/`) and update the
  ticket/DESIGN guidance that currently tells people to build in-tree.

## Why it matters

This is a booby trap for anyone following the documented `motion_tests`
workflow: do the documented thing, and the *next* unrelated firmware build
breaks with an error that gives no hint of the cause. Sprint 125 hit it on
ticket 002 and will hit it again on every later ticket that touches
`src/motion` (004, 005, 006, 011 at minimum) unless it is fixed or the
guidance changes.

Related: this compounds the repo's existing stale-incremental-build behavior
on `/Volumes` (see project memory `stale-incremental-build-on-volumes`), where
builds silently skip recompiles when mtimes move backwards — two independent
ways for a build to lie about what it just compiled.
