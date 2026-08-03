---
id: '008'
title: applyGroup() + per-target re-appliability table + boot-only ERR_NOT_LIVE rejection
status: open
use-cases:
- SUC-003
depends-on:
- '007'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# applyGroup() + per-target re-appliability table + boot-only ERR_NOT_LIVE rejection

## Description

Add `Configurator::applyGroup(ConfigTarget, const uint8_t* wire, size_t
len)` — decodes straight into `config_` (no patch, no presence flags, no
merge) using the group's generated wire codec (ticket 002's C++ target
IS the wire codec type — no adapter needed, since `wire.cpp`'s
`decodeInto` already writes through an arbitrary `base + offset`). Add a
per-`ConfigTarget` re-appliability table declaring which groups are
live-configurable (`DRIVE`, `WHEEL_CONTROL`, `MOTORS` (travel_calib only,
guarded), `OTOS`, `ESTIMATOR` — per the issue's own 8-setter audit) vs.
boot-only (`GEOMETRY`, `PLANNER` — `trackWidth`/`vMax`/`omegaMax`/
`controlPeriod`/`actuationDelay`/`landing.*`/`headingHoldGain` have no
post-construction setter per the issue's ~14-value audit). A push to a
boot-only target returns `ERR_NOT_LIVE`, not `OK`.

This ticket wires the dispatch mechanism and the table; it does not need
every `install(target)` branch to be fully correct yet for OTOS/ESTIMATOR
(tickets 009/010 do that) — this ticket's own scope is: decode correctly,
reject boot-only correctly, and call some `install(target)` path for the
live-configurable targets.

## Acceptance Criteria

- [ ] `Configurator::applyGroup(ConfigTarget, const uint8_t*, size_t)`
      exists, decodes into `config_` using the generated group codec, and
      validates bounds inline (reusing the existing `wire.cpp`
      decode/validate machinery — no new hand-written parsing).
- [ ] A per-`ConfigTarget` re-appliability table exists (a static array
      or switch) declaring at minimum: `DRIVE`/`WHEEL_CONTROL`/`MOTORS`/
      `OTOS`/`ESTIMATOR` = live; `GEOMETRY`/`PLANNER` = boot-only.
- [ ] A push to `GEOMETRY` or `PLANNER` returns `ERR_NOT_LIVE`, verified
      by a new test.
- [ ] A push to a live target decodes into `config_` and calls
      `install(target)`.
- [ ] Compiles under `HOST_BUILD`.

## Testing

- **Existing tests to run**: unaffected — this is new dispatch machinery;
  the old `apply(CommandEnvelope)` patch-based method still exists until
  ticket 013.
- **New tests to write**: one test per target confirming live vs.
  boot-only classification; one test confirming a boot-only push returns
  `ERR_NOT_LIVE` without mutating `config_`.
- **Verification command**: `uv run python -m pytest <configurator unit
  test path> -q`.

## Implementation Plan

**Approach**: Add the table and the new method to `Configurator`, reusing
`wire.cpp`'s existing `decodeInto()`-shaped decode path by calling it
directly against `&config_.<group>` as the base pointer.

**Files to modify**: `src/firm/app/configurator.{h,cpp}`.

**Testing plan**: as above.

**Documentation updates**: `configurator.h`'s doc comment gets a table
(or a pointer to one) listing each `ConfigTarget`'s re-appliability and
why.
