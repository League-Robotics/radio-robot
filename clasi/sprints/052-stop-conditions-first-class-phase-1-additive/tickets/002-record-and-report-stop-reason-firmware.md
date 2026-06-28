---
id: '002'
title: Record and report stop reason (firmware)
status: open
use-cases:
- SUC-002
depends-on:
- 052-001
issue: stop-conditions-as-a-first-class-system-primitive.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Record and report stop reason (firmware)

## Description

When `MotionCommand::tick()` fires a stop condition, record which
`StopCondition::Kind` fired (and the channel index for SENSOR). Extend
`MotionCommand::emitEvt` to append a `reason=<token>` trailing token to the
emitted EVT string. Add `reason=watchdog` to the safety-stop EVT emit in
`Superstructure::evaluateSafety`.

This makes completion reason visible to the host. It is strictly additive:
the `reason=` token appears after any existing `#id` token; all existing
prefix-match logic continues to work.

## Implementation Plan

### Step 1: Add fired-condition fields to MotionCommand

In `source/commands/MotionCommand.h` (private section):

```cpp
StopCondition::Kind _firedKind    = StopCondition::Kind::NONE;
uint8_t             _firedChannel = 0;  // valid only when _firedKind == SENSOR
```

Reset both to NONE / 0 in `MotionCommand::configure()`.

### Step 2: Record the fired condition in tick()

In `MotionCommand::tick()` (MotionCommand.cpp, in the stop-condition
evaluation loop at line ~159):

When `_stops[i].evaluate(...)` returns true, before setting `stopped = true`:
```cpp
_firedKind    = _stops[i].kind;
_firedChannel = _stops[i].sensor;
```

### Step 3: Add reason token mapping

Add a static private helper (or a free function in MotionCommand.cpp) that
maps `StopCondition::Kind` + channel to a reason token string:

```
NONE     → "" (no reason appended)
TIME     → "time"
DISTANCE → "dist"
ROTATION → "rot"
HEADING  → "heading"
POSITION → "pos"
LINE_ANY → "line"
COLOR    → "color"
SENSOR   → channel name (line0, line1, line2, line3, colorR, colorG, colorB, colorC,
                          analogIn0, analogIn1, analogIn2, analogIn3)
```

The channel-name lookup for SENSOR uses the same index-to-name table already
present in `mc_parseSensorToken`'s `chMap`. Extract this table to a shared
location (e.g. a static array in `MotionCommand.cpp` or a small inline helper)
so both `mc_parseSensorToken` and the reason-mapper share it.

### Step 4: Extend emitEvt

`MotionCommand::emitEvt` (MotionCommand.cpp, line ~246):

1. Grow the `msg` buffer from 48 to 80 bytes.
2. After building `<base> [#id]`, if `_firedKind != Kind::NONE`, append
   ` reason=<token>` using `snprintf` into the remainder of the buffer.

Example resulting strings:
- `EVT done T #12 reason=time`
- `EVT done VW reason=dist`
- `EVT done T reason=line0` (when SENSOR kind, channel=line0)
- `EVT done D reason=line` (when LINE_ANY)

### Step 5: Add reason=watchdog to Superstructure::evaluateSafety

In `source/superstructure/Superstructure.cpp`, at the `CommandProcessor::replyEvt`
call for `"safety_stop"` (line ~153):

Change:
```cpp
CommandProcessor::replyEvt(wdBuf, sizeof(wdBuf),
                           "safety_stop", "",
                           ts.activeFn, ts.activeCtx);
```
To:
```cpp
CommandProcessor::replyEvt(wdBuf, sizeof(wdBuf),
                           "safety_stop", "reason=watchdog",
                           ts.activeFn, ts.activeCtx);
```

Verify the `replyEvt` signature accepts a body string and appends it. If the
body is appended as a plain string, `EVT safety_stop reason=watchdog` is
correct. If `replyEvt` formats differently, adapt accordingly.

### Step 6: Update simulation MotionCommand model

The Python-side simulation in `tests/_infra/sim/` has a Python
`MotionCommand` model (in `firmware.py` or the ctypes binding). If the
simulation builds the firmware as a shared library and the test exercises the
C++ code directly, no Python model update is needed. If there is a Python mock
`MotionCommand` in unit tests that generates EVT strings, update those mocks
to append `reason=` as well.

Check: `tests/simulation/unit/test_motion_command.py` line 172-195 shows a
Python `MotionCommand` mock that builds EVT strings. Update the `emitEvt`
equivalent in that mock to append `reason=<token>` when a stop fires.

### Step 7: Audit and update EVT string assertions

Search all simulation test files for string equality assertions on EVT lines.
Update any that assert an exact EVT string without `reason=` to either:
- Add the expected `reason=<token>` suffix, or
- Change to prefix-based assertion (`assert evt.startswith("EVT done T")`).

Files likely to need updates (from inspection):
- `tests/simulation/unit/test_motion_command.py` (lines 384, 436, 453, 472, 806, 818, 832, 946)
- `tests/simulation/unit/test_turn_command.py`
- `tests/simulation/unit/test_vw_command.py`
- `tests/simulation/unit/test_nezha_drive.py`
- Scan all simulation test files: `grep -rn "EVT done\|EVT cancelled" tests/simulation/`

## Files to Create or Modify

- `source/commands/MotionCommand.h` — add `_firedKind`, `_firedChannel` fields.
- `source/commands/MotionCommand.cpp` — reset fields in `configure()`; set in
  `tick()` when stop fires; extend `emitEvt` buffer and append `reason=`.
- `source/superstructure/Superstructure.cpp` — add `reason=watchdog` body to
  `replyEvt` call.
- `tests/simulation/unit/test_motion_command.py` — update expected EVT strings
  and mock emitEvt if applicable.
- Other simulation tests as found by the EVT string audit.

## Acceptance Criteria

- [ ] `T 200 200 1000` (no explicit stop) emits `EVT done T reason=time`.
- [ ] `D 200 200 300` emits `EVT done D reason=dist`.
- [ ] A ROTATION stop fires with `reason=rot`.
- [ ] A HEADING stop fires with `reason=heading`.
- [ ] A POSITION stop fires with `reason=pos`.
- [ ] A LINE_ANY stop fires with `reason=line`.
- [ ] A COLOR stop fires with `reason=color`.
- [ ] A SENSOR stop for channel line0 fires with `reason=line0`.
- [ ] Watchdog fire emits `EVT safety_stop reason=watchdog` (not just `EVT safety_stop`).
- [ ] `EVT done T #12 reason=time` — corr_id and reason both present in correct order.
- [ ] No existing test breaks due to the additive nature (prefix matches still valid).
- [ ] Sim tests pass: `uv run --with pytest python -m pytest tests/simulation -q` — no new failures beyond the 2 pre-existing.
- [ ] Firmware clean build: `python build.py --clean` exits 0.

## Testing

**Verification command**: `uv run --with pytest python -m pytest tests/simulation -q`

**Pre-existing baseline**: 2 failures. No new failures acceptable.

**New tests to write** — add to `tests/simulation/unit/test_motion_command.py`
or `test_052_stop_reason.py`:

- For each stop kind: create a MotionCommand with the given stop, tick until
  it fires, assert the emitted EVT string ends with `reason=<expected_token>`.
- Test corr_id + reason ordering: `EVT done T #abc reason=time` is the exact format.
- Test NONE kind (open-ended command cancelled externally): reason= absent from
  `EVT cancelled` (reason is not appended to cancel events).
- Watchdog test: use `test_incident_scenarios.py` or a new test that lets the
  watchdog fire and asserts `EVT safety_stop reason=watchdog`.

**Firmware build**: `python build.py --clean` (exit 0 required).
