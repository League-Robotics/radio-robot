---
id: '006'
title: Port high-value legacy error-injection test suites
status: open
use-cases: [SUC-006]
depends-on: ['005']
github-issue: ''
issue: host-side-simulation-environment-for-the-new-tree-design-write-up.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Port high-value legacy error-injection test suites

## Description

`tests_old/simulation/` holds pre-rebuild encoder-error, OTOS-error, and
stiction/lag suites that the old system's error models (sprints 058, 069,
072, 073 in the pre-greenfield tree) already proved out — this is the
harness's clearest test-value delivery, and the encoder-wedge history makes
deterministic, off-hardware error-model regression coverage genuinely
valuable (not merely nice-to-have). This ticket ports the highest-value
subset onto the new tree's `Sim`/`sim_conn` API (ticket 005).

Depends on ticket 005 (the Python wrapper, fixtures, and first tests must
exist to port additional tests against).

## Acceptance Criteria

- [ ] Encoder-error suite ported (per-wheel scale error, slip, Gaussian
      noise knobs — confirm reported encoder diverges from true encoder by
      the configured amount, and that zeroing the knobs restores agreement).
- [ ] OTOS-error suite ported (noise/scale/drift knobs — confirm the
      `SimOdometer` accumulator diverges from true pose independently of
      the encoder error model, per `Hal::PhysicsWorld`'s two-independent-
      accumulators design).
- [ ] Stiction/lag suite ported (stiction gate + first-order motor lag
      response envelopes from ticket 003's `PhysicsWorld`).
- [ ] Every ported test is adapted to the new `Sim`/`sim_conn` API and the
      new tree's naming — **no ported test references a pre-rename
      `Hal::NezhaHal`/`...ToHalCommand`/`DevLoopState::hal` name, and no
      ported test reintroduces a unit-suffixed identifier**
      (`.claude/rules/naming-and-style.md`); grep the ported files for both
      before considering the ticket done.
- [ ] Ported tests are placed under `tests/sim/unit/` or `tests/sim/system/`
      per `tests/CLAUDE.md`'s domain split (whole-robot scenario assertions
      go under `system/`; narrower per-model assertions under `unit/`).
- [ ] EKF/fusion-dependent tests from the legacy suite are **explicitly
      excluded**, with a comment stating why (no firmware consumer of OTOS
      exists yet in the new tree — `architecture-update.md`'s "OTOS gap"
      note) — not silently skipped and not mis-asserted against a fusion
      path that doesn't exist.
- [ ] `uv run python -m pytest tests/sim` remains green with the ported
      suites included.

## Testing

- **Existing tests to run**: full `uv run python -m pytest` — the ported
  suites must not regress anything from tickets 001-005.
- **New tests to write**: the ported encoder-error, OTOS-error, and
  stiction/lag suites themselves (this ticket's entire content).
- **Verification command**: `uv run python -m pytest tests/sim -q`.

## Implementation Plan

**Approach:**

1. Read `tests_old/simulation/`'s encoder-error, OTOS-error, and
   stiction/lag test files in full; identify which specific assertions are
   "high-value" (test a real, previously-hard-won error-model behavior) vs.
   which were incidental to the old harness's own now-obsolete API shape.
2. Port each selected test's assertions onto ticket 005's `Sim` wrapper and
   ticket 003's error-knob setters, translating any old symbol name
   (`Hal::NezhaHal`-era or otherwise pre-rename) to its current equivalent
   — never propagate a stale name into the new tree (per this sprint's own
   reconciliation discipline, `architecture-update.md`'s "Reconciliation"
   section).
3. Explicitly mark and skip (with a comment, not silently) any legacy
   assertion that depends on EKF/fusion — no firmware consumer of OTOS
   exists yet.
4. Place ported files per `tests/CLAUDE.md`'s `unit/`/`system/` split.
5. Run the full suite; confirm determinism (re-running the ported suite
   twice produces identical results, consistent with ticket 003/005's
   determinism gate).

**Files to create:**
- New test files under `tests/sim/unit/` and/or `tests/sim/system/`
  (encoder-error, OTOS-error, stiction/lag — exact filenames chosen by the
  implementer to match the existing naming convention in those
  directories).

**Files to modify:** none expected outside `tests/sim/` itself.

**Testing plan:** see "Testing" section above.

**Documentation updates:** none required — this ticket is pure test content
with no architectural or wire-visible surface. If porting surfaces a
genuine error-model gap (a legacy assertion that cannot be reproduced
faithfully against the new `PhysicsWorld`), flag it as a new `clasi/issues/`
entry rather than silently weakening the assertion.
