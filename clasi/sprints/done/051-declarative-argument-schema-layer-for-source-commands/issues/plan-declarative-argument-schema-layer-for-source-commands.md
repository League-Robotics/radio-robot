---
status: in-progress
sprint: '051'
tickets:
- 051-001
- 051-002
- 051-003
- 051-004
- 051-005
- 051-006
- 051-007
- 051-008
- 051-009
---

# Plan: Declarative argument-schema layer for `source/commands/`

## Context

`source/commands/` contains ~48 hand-written `parseXxx` functions across five files
(`MotionCommands.cpp`, `SystemCommands.cpp`, `DebugCommands.cpp`, `ConfigCommands.cpp`,
`OtosCommands.cpp`). They re-implement the same four shapes by hand, command after command:

1. **No-arg stubs** (~12): `ParseResult r; r.ok=true; r.args.count=0; return r;` —
   e.g. `parseOI/OZ/OR/OP`, `parseHello/Ping/Id/Ver/Help/Snap/GetVel/Keepalive`.
   These are pure boilerplate: the dispatcher already treats `parseFn == nullptr` as
   "empty args" ([CommandProcessor.cpp:119](source/commands/CommandProcessor.cpp#L119)).
2. **Tokens-as-STR copies** (~8): a bounded char-by-char copy loop into `sval[32]`,
   duplicated verbatim in `parseGet`, `parseEcho`, `parseSafe`, `parseStream`,
   `parseZero`, `parseHalt`, `parseDbgLoop`, `parseX`.
3. **Int-with-range parsers** (~9): `atoi` each token, then per-field
   `if (v<lo||v>hi) return range("name")` — `parseS/T/D/G/R/RT/TURN/OV/SI`.
4. **KV scans** (3+ near-identical): `vwScanKV`, `vwHasKey`, `packSensorArg`,
   plus inline `for (i…) if (strcmp(kvs[i].key,…))` loops in `DebugCommands`.

The framework itself is sound (table-driven dispatch, `Commandable::getCommands()`,
`CommandDescriptor`, longest-prefix match). The duplication is entirely in the per-command
*parse* layer, with a smaller amount in the *handler* layer (repeated `nodev` guards and
`snprintf(body…) + replyOK` boilerplate). Local one-off helpers already exist
(`setIntArg`, `packSensorArg`, `vwScanKV` in `MotionCommands.cpp`) — proof the pattern
wants factoring; it just hasn't been done framework-wide.

**Goal:** replace the bespoke parse functions with a small **declarative argument-schema**
that most commands *declare* instead of *implement*, keeping a clean escape hatch for the
genuinely complex commands. Behaviour (wire replies, error codes, value handling) must stay
**byte-identical** — the python sim suites assert exact reply strings.

**Constraints (hard):** C++11, `-fno-exceptions -fno-rtti`, no heap, fixed buffers
(`Argument::sval[32]`, `MAX_ARGS=10`). Everything below is inline/stack-based.

## Design

### 1. Declarative `ArgSchema` (the library)

New types in **`source/types/ArgSchema.h`** (kept in `types/` so `CommandDescriptor` can
reference them without `commands/` depending the wrong way):

```cpp
enum class ArgKind : uint8_t { INT, FLOAT, STR };

struct ArgDef {
    const char* name;       // detail string for "range"/"badarg" errors
    ArgKind     kind;       // INT / FLOAT / STR
    bool        ranged;     // INT only: apply [lo,hi] check (opt-in — see note)
    int32_t     lo, hi;
};

struct ArgSchema {
    const ArgDef* defs;     // positional arg definitions
    int           ndefs;
    int           minTokens;     // <minTokens -> ERR badarg (detail = nullptr)
    bool          variadic;      // true => copy ALL tokens as STR (GET/ECHO/SAFE/…)
    const char*   packKv;        // optional: append this kv value as a trailing STR arg
                                 // (absorbs packSensorArg for T/D/TURN; nullptr = none)
};
```

A **single** generic parser implements all three common shapes:

```cpp
// source/commands/ArgParse.{h,cpp}
ParseResult parseSchema(const char* const* tokens, int ntokens,
                        const KVPair* kvs, int nkv, const ArgSchema& s);
```

- `variadic` → the tokens-as-STR path (shape 2).
- otherwise → `ndefs` positional args by `kind`, with opt-in range check (shapes 1 & 3;
  `ndefs==0` reproduces the no-arg case but those just use `nullptr`).
- `packKv` → after positional args, find `kvs[k].key == packKv` and append its value as a
  trailing STR arg (reproduces `packSensorArg` exactly).

### 2. Framework wiring (minimal, additive)

In **`source/types/CommandTypes.h`**:
- Add one field `const ArgSchema* schema;` to `CommandDescriptor` (default `nullptr`).
- Add `makeSchemaCmd(prefix, &schema, handler, ctx, errFmt, forceReply, flags)` alongside
  the existing `makeCmd` (which stays for the escape-hatch / custom-parseFn commands).

In **`source/commands/CommandProcessor.cpp`** `dispatchTable()`, the existing
`if (desc.parseFn != nullptr)` block ([CommandProcessor.cpp:119](source/commands/CommandProcessor.cpp#L119))
gains a prior branch:
```
if (desc.schema)        args = parseSchema(argTokens, argNtok, kvs, nkv, *desc.schema)…
else if (desc.parseFn)  …existing path…   // escape hatch unchanged
```
No change to `ParseFn`, `ArgList`, `ParsedCommand`, or the queue path — lowest regression
surface. Custom parsers keep working untouched.

### 3. Inline helpers (escape-hatch commands + handlers)

Also in `ArgParse.h`, small `inline` helpers that the remaining custom parsers and the
handlers reuse (these replace the 6+ hand-rolled copy loops and the local KV scanners):
- `argInt/argFloat/argStr(Argument&, …)` — `argStr` is the one true bounded `sval[32]` copy
  (replaces `setIntArg` and every inline copy loop).
- `kvFind / kvInt(def) / kvFloat(def) / kvHas(key)` — fold `vwScanKV`, `vwHasKey`,
  `packSensorArg`'s scan, and the `DebugCommands` inline kv loops into one set.

### 4. Light handler-side cleanup

- **OTOS `nodev` guard:** the identical "not initialized → ERR nodev" guard repeats 6× in
  [OtosCommands.cpp](source/commands/OtosCommands.cpp) (OI/OZ/OR/OV/OL/OA). Factor to one
  inline `bool otosReady(void* handlerCtx, const char* verb, char* rbuf, …)` that emits the
  `nodev` reply and returns false. Each handler becomes `if (!otosReady(...)) return;`.
- **Reply boilerplate:** add variadic `CommandProcessor::replyOKf(verb, corrId, fn, ctx,
  fmt, …)` / `replyErrf(...)` next to the existing `replyOK/replyErr`. This collapses the
  ubiquitous `char body[N]; snprintf(body,…); replyOK(…, body, …)` triple into one call,
  removing dozens of local `body[]` buffers. Apply opportunistically, not exhaustively.

### Migration map (what becomes a declaration vs. stays custom)

| Command(s) | New form |
|---|---|
| OI OZ OR OP, HELLO PING ID VER HELP SNAP GET-VEL `+` | `parseFn = nullptr` (delete stub) |
| GET ECHO SAFE STREAM ZERO DBG-LOOP X | `variadic` schema |
| S R RT OV SI OL OA RF | positional `ArgSchema` |
| T D TURN | positional `ArgSchema` + `packKv="sensor"` |
| VW/_VW (multi-arity), HALT (sub-verbs), SET (kv→"k=v"), I2CW I2CR, DBG OTOSBENCH/I2C/IRQGUARD/WEDGE | keep custom `parseFn`, but rewrite bodies with `argStr`/`kv*` helpers |

### Behaviour-preservation notes (critical — tests assert exact strings)

- **Opt-in ranges only.** `OV`/`SI` currently do `(int16_t)atoi` with *no* range check
  (silent truncation). Their schema sets `ranged=false`; the handler keeps the int16 cast.
  Do **not** add range validation where none existed, or `ERR range` replaces today's
  truncation and breaks byte-compatibility. `S/T/D/G/R` keep their exact existing
  `[lo,hi]` + `detail` strings (`"l"`, `"r"`, `"ms"`, `"mm"`, …).
- `minTokens` reproduces the existing `if (ntokens<N) badarg(nullptr)` guards exactly.
- `variadic` reproduces the `MAX_ARGS` cap and `ival=0/fval=0` init the copy loops do today.
- `packKv` must match `packSensorArg` byte-for-byte (key `"sensor"`, trailing STR position).

## Files to modify / add

- **New:** [source/types/ArgSchema.h](source/types/ArgSchema.h) — `ArgKind/ArgDef/ArgSchema`.
- **New:** [source/commands/ArgParse.h](source/commands/ArgParse.h) +
  [source/commands/ArgParse.cpp](source/commands/ArgParse.cpp) — `parseSchema`, `argInt/Float/Str`, `kv*`.
- [source/types/CommandTypes.h](source/types/CommandTypes.h) — `schema` field + `makeSchemaCmd`.
- [source/commands/CommandProcessor.h](source/commands/CommandProcessor.h) /
  [.cpp](source/commands/CommandProcessor.cpp) — dispatch branch + `replyOKf/replyErrf`.
- [source/commands/OtosCommands.cpp](source/commands/OtosCommands.cpp),
  [SystemCommands.cpp](source/commands/SystemCommands.cpp),
  [MotionCommands.cpp](source/commands/MotionCommands.cpp),
  [ConfigCommands.cpp](source/commands/ConfigCommands.cpp),
  [DebugCommands.cpp](source/commands/DebugCommands.cpp) — migrate per the table above;
  update each `getCommands()` / `buildCommandTable` registration to `makeSchemaCmd`/`nullptr`.
- Add the schema/parse unit coverage to the sim build
  ([tests/_infra/sim/CMakeLists.txt](tests/_infra/sim/CMakeLists.txt)) if a new TU needs it.

## Process

CLASI is active (no `.clasi/oop`) and this touches the protocol-critical parse layer broadly,
so it routes as a **CLASI sprint**. After you approve this plan I will, as team-lead, drive:
sprint-planner → architecture update (lock the `ArgSchema` shape + migration order) →
sequenced tickets (framework types/dispatch first, then one ticket per command file) →
programmer execution → pre/post-close review. The migration is ordered so the framework
lands first and each command file is converted independently behind it.

## Verification (end-to-end, behaviour must be byte-identical)

1. **Clean firmware build** (incremental builds go stale on this repo — see memory):
   `python build.py --clean` and confirm it compiles under `-fno-exceptions -fno-rtti` and
   the binary size doesn't grow (helpers are inline; net code should shrink).
2. **Sim build + unit/system tests** — the python suites drive real wire commands and assert
   exact `OK/ERR` reply strings, so they are the regression oracle:
   - `tests/simulation/unit/test_system_commands_coverage.py`
   - `tests/simulation/system/test_stop_condition_coverage.py` (HALT/T/D/sensor=)
   - `tests/simulation/system/test_ekf_odometry_commands_coverage.py` (OTOS verbs)
   - full sim unit suite via `tests/_infra/coverage.sh`.
3. **Spot-check parity** on a few representative verbs (`S 500 500`, `T 300 300 1000 sensor=line0:ge:500`,
   `OV 1 2 3`, `GET trackwidthMm`, `HALT TIME 1000 SOFT`) before/after to confirm identical
   replies, including error paths (`S 99999` → `ERR range l`, `OV 1` → `ERR badarg`).
