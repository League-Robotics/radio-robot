---
id: '006'
title: Migrate MotionCommands.cpp to ArgSchema
status: done
use-cases:
- SUC-002
- SUC-004
depends-on:
- '003'
github-issue: ''
issue: plan-declarative-argument-schema-layer-for-source-commands.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migrate MotionCommands.cpp to ArgSchema

## Description

Migrate `source/commands/MotionCommands.cpp` to use the ArgSchema framework.
This is the most complex migration ticket: motion commands have range checks with
specific detail strings, `packKv` for sensor=, and complex VW/TURN/R parsers that
must be handled carefully.

**Migration map for MotionCommands:**

| Command | Old form | New form |
|---------|----------|----------|
| STOP | parseNoArgs (no-arg stub) | `parseFn = nullptr` |
| S | parseS (2 INT, range [-1000,1000]) | `makeSchemaCmd` with schema `{ndefs=2, minTokens=2, ranged=true, lo=-1000, hi=1000, name="l"/"r"}` |
| T | parseT (3 INT + packSensorArg) | `makeSchemaCmd` + `packKv="sensor"`; ranges: l/r [-1000,1000], ms [1,30000] |
| D | parseD (3 INT + packSensorArg) | `makeSchemaCmd` + `packKv="sensor"`; ranges: l/r [-1000,1000], mm [1,10000] |
| G | parseG (3 INT with ranges) | `makeSchemaCmd`; ranges: x/y [-10000,10000], speed [1,1000] |
| R | parseR (2 INT with ranges) | `makeSchemaCmd`; speed [-1000,1000], radius [-10000,10000] |
| RT | parseRT (1 INT with range) | `makeSchemaCmd`; rel_cdeg [-180000,180000], name="deg" |
| X | parseX (variadic: optional "soft" token) | `makeSchemaCmd` with `variadic=true` |
| TURN | parseTURN (1 INT + optional eps= KV + packSensorArg) | keep custom parseFn; rewrite with `argInt`, `kvInt`, `kvFind` |
| VW, _VW | parseVW/parse_VW (2 INT with ranges) | keep custom parseFn (multi-arity); rewrite with `argInt` |

**CRITICAL behaviour-preservation notes:**

- S/T/D/G/R/RT range detail strings MUST match exactly:
  - S: `"l"`, `"r"` (ERR range l, ERR range r)
  - T: `"l"`, `"r"`, `"ms"` (ERR range ms for [1,30000])
  - D: `"l"`, `"r"`, `"mm"` (ERR range mm for [1,10000])
  - G: `"x"`, `"y"`, `"speed"` (ERR range speed for [1,1000])
  - R: `"speed"`, `"radius"`
  - RT: `"deg"` (ERR range deg for [-180000,180000])
- T, D, TURN `packKv="sensor"`: `parseSchema` appends `sensor=` value as the last
  STR arg. The handler validates the sensor token before replying OK (N16 fix). This
  validation must remain in the handler — it is not part of `parseSchema`.
- `setIntArg` replaced by `argInt` throughout (including in handler helper functions
  `pushVW`, `packKVArg`).
- `packSensorArg`, `vwScanKV`, `vwHasKey` locals in MotionCommands.cpp replaced by
  `kvFind`, `kvInt`, `kvHas` from ArgParse.h.
- `parseNoArgs` (shared between STOP and formerly X) deleted; STOP uses `nullptr`.
- `parseX` becomes variadic (tokens-as-STR); handleX already checks `args.args[0].sval`.

## Acceptance Criteria

- [x] `parseS`, `parseD`, `parseG`, `parseR`, `parseRT` deleted; replaced by static
  `ArgSchema` structs with correct `ndefs`, `minTokens`, `ranged=true`, `lo`, `hi`,
  and `name` matching the existing ERR detail strings.
- [x] `parseT` deleted; replaced by schema with `packKv="sensor"`.
- [x] `parseD` deleted; replaced by schema with `packKv="sensor"`.
- [x] `parseX` (variadic) deleted; replaced by schema with `variadic=true`.
- [x] `parseNoArgs` deleted; STOP registered with `parseFn=nullptr`.
- [x] `parseTURN` retained; body rewritten with `argInt`, `kvFind`, and inline
  `kvFind`+`argStr` for sensor= (replacing `packSensorArg`).
- [x] `parseVW`, `parse_VW` retained; `setIntArg` calls replaced by `argInt`.
- [x] `setIntArg` local helper deleted; all call sites use `argInt` from ArgParse.h.
- [x] `packSensorArg` local helper deleted; `kvFind` / `argStr` used instead.
- [x] `vwScanKV` and `vwHasKey` deleted; replaced by `argsScanKV` / `argsHasKey`
  static helpers (same logic, renamed to avoid confusion with KVPair-based kvInt/kvHas).
- [x] All range ERR detail strings byte-identical: `S 99999` -> `ERR range l`;
  `T 0 0 99999` -> `ERR range ms`; `D 0 0 0` -> `ERR range mm`; etc.
- [x] `T` and `D` sensor forwarding still works: `T 500 500 1000 sensor=line0:ge:500`
  -> `OK drive l=500 r=500 ms=1000` with sensor validated.
- [x] `uv run --with pytest python -m pytest tests/simulation -q` — no new failures.
- [x] Primary oracle: `tests/simulation/system/test_stop_condition_coverage.py`.

## Implementation Plan

### Approach

1. Add `#include "ArgParse.h"` to MotionCommands.cpp.
2. Write static `ArgSchema` structs for S, T, D, G, R, RT, X.
3. Delete parseS/T/D/G/R/RT/X/parseNoArgs.
4. Rewrite parseTURN body using `argInt`, `kvInt`, `kvFind`.
5. Rewrite parseVW, parse_VW using `argInt`.
6. Delete `setIntArg`, `packSensorArg`, `vwScanKV`, `vwHasKey` locals.
7. Update `getMotionCommands()` registrations.
8. Update `vwDesc` initialisation (uses `parseVW` which is still present).

### Files to Modify

- `source/commands/MotionCommands.cpp` — remove parse functions, add schemas,
  update helpers, update registrations.

### Testing Plan

Run: `uv run --with pytest python -m pytest tests/simulation -q`

Primary oracle: `tests/simulation/system/test_stop_condition_coverage.py` (T/D/sensor
coverage) and sim unit tests for S, G, R, RT, TURN.

Spot checks:
- `S 500 500` -> `OK drive l=500 r=500`
- `S 99999` -> `ERR range l`
- `T 300 300 1000` -> `OK drive l=300 r=300 ms=1000`
- `T 300 300 1000 sensor=line0:ge:500` -> `OK drive ...` (sensor validated)
- `D 500 500 1000` -> `OK drive l=500 r=500 mm=1000`
- `RT 9000` -> `OK rt rot=9000`
- `X` -> `OK x`; `X soft` -> `OK x`

### Documentation Updates

Update comment block in MotionCommands.cpp to note the ArgSchema migration and
that `setIntArg`/`packSensorArg`/`vwScanKV`/`vwHasKey` have been removed.
