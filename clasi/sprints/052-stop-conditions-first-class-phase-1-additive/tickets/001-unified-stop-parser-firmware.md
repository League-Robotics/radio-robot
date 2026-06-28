---
id: '001'
title: Unified stop= parser (firmware)
status: open
use-cases:
- SUC-001
depends-on: []
issue: stop-conditions-as-a-first-class-system-primitive.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Unified stop= parser (firmware)

## Description

Add a unified `stop=<kind>:<args>` token parser to
`source/commands/MotionCommands.cpp` and wire it into the handlers for VW, S,
T, D, R, and TURN. This makes all 7 StopCondition kinds accessible from any
open-loop motion command on the wire, not just `sensor=` on T/D/TURN.

The `sensor=` form is preserved as a back-compat alias. No existing routing,
Goal dispatch, or EVT output changes in this ticket.

## Implementation Plan

### Step 1: Add mc_parseStopToken

Add a static helper below `mc_parseSensorToken` in `MotionCommands.cpp`:

```c
static bool mc_parseStopToken(const char* value, MotionCommand& mc)
```

`value` is the string after `stop=` (e.g. `d:300`, `line:ge:512`,
`sensor:line0:ge:512`, `color:120:0.5:0.4:0.1`, `heading:4500:300`, `rot:250`).

Dispatch on the prefix before the first `:`:
- `t:<ms>` → `makeTimeStop((float)ms)` → `mc.addStop(...)`
- `d:<mm>` → `makeDistanceStop((float)mm)` → `mc.addStop(...)`
- `line:<ge|le>:<thr>` → `makeLineAnyStop(thr, cmp)` → `mc.addStop(...)`
- `sensor:<ch>:<ge|le>:<thr>` → reuse channel-lookup table from
  `mc_parseSensorToken` → `makeSensorStop` → `mc.addStop(...)`
- `color:<h>:<s>:<v>:<dist>` → `makeColorStop(h, s, v, dist)` → `mc.addStop(...)`
- `heading:<cdeg>:<eps_cdeg>` → convert cdeg→rad (divide by 100.0 * 180/π) →
  `makeHeadingStop(rad, eps_rad)` → `mc.addStop(...)`
- `rot:<arc_mm>` → `makeRotationStop((float)arc_mm)` → `mc.addStop(...)`

Extend the channel map to include `analogIn0`-`analogIn3` (indices 8-11) to
match the full SENSOR kind support in `StopCondition`.

### Step 2: Add mc_applyStopClauses

```c
static void mc_applyStopClauses(const char* const* tokens, int ntokens,
                                MotionCommand& mc)
```

Iterates `tokens[0..ntokens-1]` and for each token whose prefix is `stop=` or
`sensor=` (back-compat), calls `mc_parseStopToken(token + prefix_len, mc)`.
Returns after processing all tokens up to kMaxStopConds.

### Step 3: Extend parseVW to forward stop= tokens

`parseVW` currently validates only `v` and `omega` (positions 0 and 1).
Extend it to copy remaining tokens (positions 2..N) as STR args in
`args.args[2..]` (using `argStr`), up to MAX_ARGS. The handler then calls
`mc_applyStopClauses` over those STR args.

### Step 4: Wire into handleVW

After each `requestGoal` call in `handleVW` (T, D, R, TURN, and open-ended
VW branches), call `mc_applyStopClauses` over `args.args[2..]`. The
`mc_applyStopClauses` call replaces the existing per-branch `sensor=`
forwarding loops (those loops look for `sensor=` in args and call
`mc_parseSensorToken` — `mc_applyStopClauses` subsumes this behavior since it
handles both `sensor=` and `stop=`).

### Step 5: Wire into converter handlers (handleS, handleR)

`handleS` and `handleR` push a VW ParsedCommand onto the queue. After their
`requestGoal` (or `beginStream`/`beginArc`) call, pack any `stop=` tokens from
the original args into the VW ParsedCommand's STR args (using `packKVArg`
style, at indices beyond the verb-specific slots). `handleVW` will then process
them via `mc_applyStopClauses`.

For `handleT` and `handleD`: these already pack `sensor=` as args[3] via
`tSchema.packKv="sensor"`. Add a second pass that also packs `stop=` tokens.

### Step 6: TURN stop= support

The TURN branch in `handleVW` already calls `addStop` via the existing
`sensor=` loop. Replace that loop with a call to `mc_applyStopClauses` which
handles both forms.

## Files to Create or Modify

- `source/commands/MotionCommands.cpp` — all changes are in this file.
  Lines to study: 54-105 (`mc_parseSensorToken`), 688-715 (`parseVW`),
  718-984 (`handleVW` and its branches), 237-420 (handleS, handleT, handleD),
  575-635 (parseTURN / TURN branch of handleVW).

## Acceptance Criteria

- [ ] `VW 200 0 stop=d:300` accepted; active MotionCommand has 1 DISTANCE stop condition.
- [ ] `VW 200 0 stop=t:1000` accepted; active MotionCommand has 1 TIME stop condition.
- [ ] `VW 200 0 stop=line:ge:512` accepted; active MotionCommand has 1 LINE_ANY stop condition.
- [ ] `VW 200 0 stop=sensor:line0:ge:512` accepted; active MotionCommand has 1 SENSOR stop condition.
- [ ] `VW 200 0 stop=color:120:0.5:0.4:0.1` accepted; active MotionCommand has 1 COLOR stop condition.
- [ ] `VW 200 0 stop=heading:4500:300` accepted; active MotionCommand has 1 HEADING stop condition.
- [ ] `VW 200 0 stop=rot:250` accepted; active MotionCommand has 1 ROTATION stop condition.
- [ ] `VW 200 0 stop=d:300 stop=t:5000` accepted; 2 stop conditions added (OR-combined).
- [ ] `T 200 200 1000 stop=sensor:line0:ge:512` accepted; 2 stops (time + sensor).
- [ ] `D 200 200 300 stop=t:5000` accepted; 2 stops (distance + time).
- [ ] `S 200 200 stop=line:ge:512` accepted; 1 stop added.
- [ ] `R 200 500 stop=d:300` accepted; 1 stop added.
- [ ] `TURN 9000 stop=t:5000` accepted; 1 time stop added.
- [ ] `T 200 200 1000 sensor=line0:ge:512` still accepted and works (back-compat).
- [ ] Firmware clean build: `python build.py --clean` exits 0.
- [ ] Sim tests: `uv run --with pytest python -m pytest tests/simulation -q` — no new failures.

## Testing

**Verification command**: `uv run --with pytest python -m pytest tests/simulation -q`

**Pre-existing baseline**: 2 failures. No new failures acceptable.

**New tests to write** — add to `tests/simulation/unit/test_stop_condition_coverage.py`
or a new file `tests/simulation/unit/test_052_stop_parser.py`:

- Parametrize over all 7 stop kinds on VW: send via sim, confirm the command
  terminates when the condition is synthetically met (use the sim's ability to
  inject sensor values), or inspect via `isOpenEnded()` behavior.
- Stacking: `VW 200 0 stop=d:300 stop=t:5000` → 2 conditions (command not
  open-ended, has 2 stops).
- Back-compat: `T 200 200 1000 sensor=line0:ge:512` accepted without error (existing
  behavior unchanged).
- Each converter: S, T, D, R, TURN each correctly forward the stop= clause.

**Firmware build**: `python build.py --clean` (exit 0 required — ARM-only errors
are not caught by the Python sim).
