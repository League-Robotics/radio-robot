---
id: '001'
title: Fix ArgSchema query-mutates-state bug (DBG IRQGUARD, RF, OL, OA)
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: dbg-irqguard-query-disables-guard.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix ArgSchema query-mutates-state bug (DBG IRQGUARD, RF, OL, OA)

## Description

`parseSchema()`'s positional path (`source/commands/ArgParse.cpp:60-96`)
always fills every declared `ArgDef` slot, using `atoi(nullptr)==0` /
`atof(nullptr)==0.0f` / `""` when a token is omitted. `res.args.count` is
therefore always `== ndefs`, regardless of how many tokens were actually
supplied. Any handler that checks `args.count >= 1` to decide "was an
optional argument given" is checking a value that can never be less than
`ndefs` — the check is always true.

An audit of every `ArgSchema` instance in the codebase for the exact shape
that exposes this defect (`ndefs >= 1 && minTokens < ndefs && !variadic` —
at least one *optional* positional argument) found exactly four matches:

| Command | File | Bug on a bare query |
|---|---|---|
| `DBG IRQGUARD` | `DebugCommands.cpp` | Silently disables the IRQ guard (filed issue). |
| `RF` | `SystemCommands.cpp` | The existing `args.count < 1` "Query" branch is dead code; falls through to `ch=0`, which is in-range (`radiochan::kMin==0`) — **silently retunes the radio to channel 0 and persists it to flash**, breaking the link. |
| `OL` | `OtosCommands.cpp` | Silently zeros the OTOS linear calibration scalar. |
| `OA` | `OtosCommands.cpp` | Silently zeros the OTOS angular calibration scalar. |

Every other `ArgSchema` in the codebase has `minTokens == ndefs`
(all-or-nothing: a missing token fails the `minTokens` guard outright) or is
variadic (`args.count` already reflects the real token count there) — this
ticket's fix is fully scoped by the table above; do not go looking for a
fifth handler unless a NEW schema of this shape is added later.

## Acceptance Criteria

- [x] `ArgList` (`source/types/CommandTypes.h`) gains a new `int
      suppliedCount;` field. `ArgList` remains a plain aggregate (no default
      member initializer) so `ParseResult`'s C++11 unrestricted-union
      trivial-constructibility invariant (see the existing header comment)
      is preserved.
- [x] `parseSchema()` (`source/commands/ArgParse.cpp`) sets `suppliedCount`:
      `min(ntokens, schema.ndefs)` on the positional path; `count` (already
      correct) on the variadic and no-arg paths.
- [x] Every hand-rolled `ParseFn` (`parseDbgWedge`, `parseDbgOtosBench`,
      `parseI2cw`, `parseI2cr` in `DebugCommands.cpp`) sets
      `res.args.suppliedCount = res.args.count;` so the field is never left
      uninitialized on any code path.
- [x] `handleDbgIrqguard` (`DebugCommands.cpp`): guard changes from
      `args.count >= 1` to `args.suppliedCount >= 1`.
- [x] `handleRf` (`SystemCommands.cpp`): guard changes from `args.count < 1`
      to `args.suppliedCount < 1` (this makes the existing "Query." branch
      reachable for the first time).
- [x] `handleOL`, `handleOA` (`OtosCommands.cpp`): guard changes from
      `args.count >= 1` to `args.suppliedCount >= 1`.
- [x] No other line in any of the four handlers changes.
- [x] `uv run --with pytest python -m pytest -q` is green (2 known-baseline
      failures allowed, no new failures). (Observed 0 known-baseline
      failures in this run: 2435 passed, 0 failed.)

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_argparse.py` (must
  stay green — verifies `parseSchema()`'s existing behavior is unchanged),
  full default suite.
- **New tests to write**:
  - In `test_argparse.py`: for a schema shaped like `dbgIrqguardSchema`
    (`ndefs=1, minTokens=0, ranged=false`), assert `suppliedCount == 0` when
    called with zero tokens and `suppliedCount == 1` when called with one
    token — including the case where the supplied token's value happens to
    equal the default (`atoi` of an explicit `"0"` token vs. an omitted
    token must both parse to `ival==0` but differ in `suppliedCount`). This
    is the direct regression test for the root cause; extend
    `sim_parse_schema` (`tests/_infra/sim/sim_api.cpp`) with an
    `out_supplied_count` output parameter to expose it.
  - A host-reachable regression test for `OL`/`OA`: since
    `OtosCommands.cpp`'s handlers are NOT `#ifndef HOST_BUILD`-guarded (only
    the `NezhaHAL` bench-noise branch is), issue `OL 5` then bare `OL`
    through `sim_command()` and assert the reply's `scalar=` value is still
    `5`, not `0`. Same pattern for `OA`. Place in
    `tests/simulation/unit/test_dbg_otos_commands.py` or a new
    `test_otos_calibration_commands.py`.
  - `DBG IRQGUARD` and `RF` handler bodies are `#ifndef HOST_BUILD`-guarded
    (no I2C bus / no radio in sim) — their end-to-end behavior cannot be
    exercised through `sim_command()`; the `parseSchema`-level test above is
    the verification for these two, plus a code-review pass confirming the
    one-line guard change is applied identically to all four handlers.
- **Verification command**: `uv run --with pytest python -m pytest -q`
