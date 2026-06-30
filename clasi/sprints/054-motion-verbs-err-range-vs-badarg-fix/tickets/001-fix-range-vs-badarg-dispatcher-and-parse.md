---
id: 054-001
title: Fix range-vs-badarg in dispatcher and parse functions
status: done
sprint: '054'
use-cases:
- SUC-001
depends-on: []
issue: motion-verbs-err-badarg-instead-of-range.md
---

# 054-001: Fix range-vs-badarg in dispatcher and parse functions

## Description

Out-of-range arguments to motion verbs S, T, D, and R currently reply
`ERR badarg <field>` instead of the correct `ERR range <field>`. The
regression was introduced in sprint 051 when these verbs were migrated
to custom `parseFn` registrations. The dispatcher ignores `res.err.code`
and always formats errors from `desc.errFmt` ("badarg").

This ticket fixes the two affected code sites:

1. `CommandProcessor.cpp` — `dispatchTable` `parseFn` branch: honor
   `result.err.code` when non-null, falling back to `desc.errFmt`.
2. `MotionCommands.cpp` — `parseS`, `parseT`, `parseD`, `parseR`: set
   `res.err.code = "range"` on every ranged-value failure return. Arg-count
   failures continue to leave `res.err.code = nullptr`.

## Acceptance Criteria

- [x] `S 99999 0` → `ERR range l`
- [x] `S 0 99999` → `ERR range r`
- [x] `T 0 0 0` → `ERR range ms`
- [x] `D 0 0 0` → `ERR range mm`
- [x] `R 99999 0` → `ERR range speed`
- [x] `S` (no args) → `ERR badarg` (arg-count path unaffected)
- [x] `T 0 0` (two args, not three) → `ERR badarg`
- [x] `uv run pytest` passes

## Implementation Plan

### Approach

Minimal surgical fix. No new types or API surface. Reference implementation
is `parseTURN` (lines 924-948 in `MotionCommands.cpp`) and `parseVW`
(lines 1082-1107), which already use `res.err.code = "range"` / `"badarg"`.

### Files to Modify

**`source/commands/CommandProcessor.cpp`** — `dispatchTable()`, `parseFn` branch
(lines ~139-149):

Change:
```cpp
const char* detail = (result.err.detail != nullptr) ? result.err.detail : nullptr;
const char* code   = (desc.errFmt != nullptr) ? desc.errFmt : "badarg";
```
To:
```cpp
const char* detail = (result.err.detail != nullptr) ? result.err.detail : nullptr;
const char* code   = (result.err.code  != nullptr) ? result.err.code
                   : (desc.errFmt      != nullptr) ? desc.errFmt
                   : "badarg";
```

**`source/commands/MotionCommands.cpp`** — four parse functions:

In `parseS` (lines ~421-425), change both range-failure returns from:
```cpp
res.ok = false; res.err.code = nullptr; res.err.detail = "l"; return res;
```
To:
```cpp
res.ok = false; res.err.code = "range"; res.err.detail = "l"; return res;
```
Apply the same pattern to the `"r"` failure in `parseS`; the `"l"`, `"r"`,
`"ms"` failures in `parseT`; the `"l"`, `"r"`, `"mm"` failures in `parseD`;
and the `"speed"`, `"radius"` failures in `parseR`.

Arg-count failures (e.g. `ntokens < 2`) continue to use `err.code = nullptr`
so they fall through to `desc.errFmt = "badarg"`.

### New Tests

Add a live-sim test class `TestMotionVerbRangeErrors` in
`tests/simulation/unit/test_protocol_v2.py`:

```python
class TestMotionVerbRangeErrors:
    """Verify S/T/D/R emit ERR range (not ERR badarg) for out-of-range args."""

    def test_s_range_l(self, sim): ...      # sim.send("S 99999 0") == "ERR range l"
    def test_s_range_r(self, sim): ...      # sim.send("S 0 99999") == "ERR range r"
    def test_s_badarg_no_args(self, sim): ... # sim.send("S") == "ERR badarg"
    def test_t_range_ms(self, sim): ...     # sim.send("T 0 0 0") == "ERR range ms"
    def test_t_badarg_too_few(self, sim): ... # sim.send("T 0 0") == "ERR badarg"
    def test_d_range_mm(self, sim): ...     # sim.send("D 0 0 0") == "ERR range mm"
    def test_r_range_speed(self, sim): ...  # sim.send("R 99999 0") == "ERR range speed"
```

Each test uses the `sim` fixture (from `conftest.py`) which builds the
firmware sim and provides `sim.send(cmd) -> str`.

### Testing Plan

Run `uv run pytest tests/simulation/unit/test_protocol_v2.py -q` to
verify new tests pass. Run full `uv run pytest` for CI green.

### Documentation Updates

None required. This is a bug fix restoring documented behavior.
