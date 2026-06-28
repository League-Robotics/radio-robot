---
status: ready
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 051 Use Cases

## SUC-001: Declare a no-arg command without a parse function

- **Actor**: Firmware developer
- **Preconditions**: A command handler needs no arguments (e.g. OI, OZ, PING, HELLO, SNAP).
- **Main Flow**:
  1. Developer registers the command with `parseFn = nullptr` in `getCommands()` /
     `buildCommandTable`.
  2. `CommandProcessor::dispatchTable` detects `parseFn == nullptr` and `schema == nullptr`
     and passes an empty `ArgList` to the handler.
  3. Handler executes with zero arguments.
- **Postconditions**: No parse function exists; no boilerplate stub needed.
- **Acceptance Criteria**:
  - [ ] All no-arg stubs (OI, OZ, OR, OP, HELLO, PING, ID, VER, HELP, SNAP, GET VEL, +)
        are replaced with `nullptr` parseFn entries.
  - [ ] Dispatch continues to pass an empty ArgList when `parseFn == nullptr && schema == nullptr`.
  - [ ] Sim suite passes with no new failures.

---

## SUC-002: Declare a positional-argument command via ArgSchema

- **Actor**: Firmware developer
- **Preconditions**: A command takes a fixed set of positional INT/FLOAT/STR arguments,
  optionally with range checks and an optional trailing `sensor=` KV arg.
- **Main Flow**:
  1. Developer writes a static `ArgSchema` with the argument definitions
     (kind, ranged flag, lo, hi, minTokens, packKv).
  2. Developer registers the command with `makeSchemaCmd` instead of `makeCmd`.
  3. `CommandProcessor::dispatchTable` calls `parseSchema` to validate and populate
     the `ArgList` using the schema.
  4. On validation failure, `dispatchTable` emits `ERR <errFmt> <detail>` with the
     same code and detail string the old parse function produced.
  5. Handler receives the populated `ArgList` and executes.
- **Postconditions**: No per-command parse function; schema struct provides full
  declarative specification.
- **Acceptance Criteria**:
  - [ ] Commands S, T, D, G, R, TURN, RT, OV, OL, OA, RF, SI are converted to `ArgSchema`.
  - [ ] `parseSchema` produces byte-identical `ArgList` contents versus the deleted stubs.
  - [ ] Range error replies (`ERR range l`, `ERR range ms`, etc.) are byte-identical.
  - [ ] `OV`/`SI` use `ranged=false`; their silent int16 truncation is preserved.

---

## SUC-003: Declare a variadic (tokens-as-STR) command via ArgSchema

- **Actor**: Firmware developer
- **Preconditions**: A command accepts all tokens as raw STR copies (e.g. ECHO, GET,
  STREAM, SAFE, ZERO, HALT, DBG LOOP).
- **Main Flow**:
  1. Developer writes a static `ArgSchema` with `variadic = true`.
  2. `parseSchema` copies all tokens into `ArgList.args[i].sval`, applying MAX_ARGS cap
     and initialising `ival=0`, `fval=0.0f` for each entry.
  3. Handler operates on the STR args as before.
- **Postconditions**: Identical `ArgList` produced; no duplicated copy-loop code.
- **Acceptance Criteria**:
  - [ ] Variadic copy behaviour matches old parsers byte-for-byte (MAX_ARGS cap,
        `ival=0/fval=0` init, bounded sval copy to 31 chars + NUL).
  - [ ] Commands converted: ECHO, GET, STREAM, SAFE, ZERO, X, DBG LOOP.

---

## SUC-004: Reuse KV and arg helpers in custom parse functions

- **Actor**: Firmware developer
- **Preconditions**: A command is too complex for a pure schema declaration (e.g. VW,
  HALT, SET, I2CW/I2CR, DBG OTOS BENCH). It retains a custom `parseFn`.
- **Main Flow**:
  1. Developer rewrites the custom parser's body using `argStr(...)`, `kvFind(...)`,
     `kvInt(...)`, `kvFloat(...)`, `kvHas(...)` from `ArgParse.h`.
  2. The handler may also use `argStr` and `kv*` instead of inline loops.
  3. Behaviour (wire replies, error codes) is unchanged.
- **Postconditions**: Duplicated hand-rolled copy loops and KV scanners are eliminated
  even for commands that cannot use `ArgSchema`.
- **Acceptance Criteria**:
  - [ ] `argStr` is the one true bounded sval copy (replaces `setIntArg` and every
        inline copy loop).
  - [ ] `kvFind / kvInt / kvFloat / kvHas` replace `vwScanKV`, `vwHasKey`, and the
        inline kv loops in DebugCommands.
  - [ ] All custom parsers that previously had duplicated copy loops use helpers.
  - [ ] Sim suite passes with no new failures.

---

## SUC-005: Validate command behaviour byte-identically after migration

- **Actor**: QA / CI system
- **Preconditions**: All five command files have been migrated; firmware builds cleanly.
- **Main Flow**:
  1. Run `uv run --with pytest python -m pytest tests/simulation -q`.
  2. Protocol-string suites assert exact OK/ERR reply strings for representative
     verbs: S, T, D, G, R, TURN, OV, GET, HALT, ECHO, SAFE, and their error paths.
  3. Spot-check: `S 500 500` -> `OK drive l=500 r=500`; `S 99999` -> `ERR range l`;
     `OV 1 2 3` -> `OK setpos x=1 y=2 h=3`; `OV 1` -> `ERR badarg`.
- **Postconditions**: Zero new test failures beyond the 2 known pre-existing baselines.
- **Acceptance Criteria**:
  - [ ] No new sim test failures introduced by the migration.
  - [ ] Binary size does not grow (helpers are inline; net should shrink).
  - [ ] `python build.py --clean` succeeds under `-fno-exceptions -fno-rtti`.
