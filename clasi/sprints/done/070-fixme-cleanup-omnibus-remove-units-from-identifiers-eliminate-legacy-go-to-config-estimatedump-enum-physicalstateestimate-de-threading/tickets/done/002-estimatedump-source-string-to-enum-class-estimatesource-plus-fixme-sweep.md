---
id: '002'
title: EstimateDump.source string to enum class EstimateSource, plus FIXME sweep
status: done
use-cases:
- SUC-002
- SUC-003
depends-on:
- '001'
github-issue: ''
issue: fixme-cleanup-legacy-config-and-estimatedump-enum.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# EstimateDump.source string to enum class EstimateSource, plus FIXME sweep

## Description

`EstimateDump.h:23`'s "FIXME should be an enum" marks `EstimateDump::source`
(a raw `const char*`) for conversion to a compile-time-checked
`enum class EstimateSource { Encoder, Optical, Fused }`. `DebugCommands.cpp
::handleDbgEst` is confirmed (by grep) to be the **only** consumer that ever
turns the tag into wire text, so the "single to-string mapping at the emit
point" requirement is structurally trivial: the mapping lives in
`EstimateDump.h` but is only ever called from the one emit site. `DBG EST`
wire output must stay byte-identical.

This ticket also folds in the sprint's FIXME sweep: this sprint's own
clean-grep pass found two markers neither of the two sprint issues tracks
(`ArgSchema.h`, `OutputState.h`), plus two historical/already-resolved
references (`StopCondition.cpp:20`, `ColorUtil.cpp:4`) that still contain the
literal string `FIXME`. Resolving all four here is what makes the sprint's
target state achievable: `grep -ri FIXME source/` clean except the 8
units-suffix markers deferred to sprint 071.

Depends on ticket 001: the sweep's final clean-grep check needs ticket 001's
`Config.h` FIXME already removed to be meaningful. See
`architecture-update.md` Step 5 ("EstimateDump enum + FIXME sweep"), Decision
2 (enum design), Decision 5 (`ArgKind`/`ArgType` resolution); `usecases.md`
SUC-002, SUC-003.

## Acceptance Criteria

- [x] `source/state/EstimateDump.h:23` — `const char* source;` replaced with
      `EstimateSource source;` where `enum class EstimateSource : uint8_t {
      Encoder, Optical, Fused };`. FIXME comment removed.
- [x] `source/state/EstimateDump.h` — new `inline const char*
      toString(EstimateSource)` using a `switch` (not a lookup array — a
      `switch` gives a `-Wswitch` warning if a fourth `EstimateSource` value
      is ever added without updating `toString()`), mapping
      `Encoder→"enc"`, `Optical→"otos"`, `Fused→"fuse"`.
- [x] `EstimateDump.h`'s `fill()` lambda and `dumpEstimates()`'s three call
      sites pass `EstimateSource::Encoder`/`Optical`/`Fused` instead of the
      string literals `"enc"`/`"otos"`/`"fuse"`.
- [x] `source/commands/DebugCommands.cpp::handleDbgEst` — the `snprintf`
      call's `d.source` argument becomes `toString(d.source)`.
- [x] `grep -rn "EstimateDump" source/` confirms no other file constructs an
      `EstimateDump` with a raw string literal for `source`.
- [x] `DBG EST` reply text is byte-identical before/after (verified by
      existing `DBG EST` test coverage, or new coverage if none exists).
- [x] `source/types/ArgSchema.h` — the FIXME on `ArgKind` replaced with a
      resolved comment explaining why `ArgKind` (schema layer) and
      `CommandTypes::ArgType` (runtime tagged-union layer) are intentionally
      kept separate rather than merged (Decision 5). No code change.
- [x] `source/state/OutputState.h` — the FIXME on `digitalDirty`/
      `analogDirty` replaced with a comment stating the fields are currently
      dead (confirmed by grep: no producer or consumer anywhere in
      `source/`) — same disposition as sprint 067 Decision 5 ("document dead
      things, don't fix them"). No code change.
- [x] `source/control/StopCondition.cpp:20`, `source/control/ColorUtil.cpp:4`
      — reworded so neither contains the literal string `FIXME` (both
      already describe an already-resolved historical issue). No code
      change.
- [x] `grep -ri FIXME source/` output is exactly the 8 units-suffix markers
      cross-referenced in `remove-units-from-identifier-names.md` — zero
      other markers remain.
- [x] Full test suite green; TLM output byte-identical (this ticket touches
      no TLM field or format, only the in-memory tag type).

## Testing

- **Existing tests to run**: any existing `DBG EST` coverage; full default
  suite (`uv run python -m pytest`).
- **New tests to write**: if no existing test asserts `DBG EST`'s exact reply
  text, add one that issues `DBG EST` and checks the three `EST enc/otos/fuse
  ...` lines are byte-identical to the pre-refactor format.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Convert `EstimateDump::source` to a `enum class EstimateSource
: uint8_t` and add exactly one free-function `toString()` next to the enum
declaration, called only from `handleDbgEst`'s `snprintf`. This is a narrow,
single-consumer change confirmed safe by grep before starting. Fold in the
FIXME sweep as four small, independent comment-only edits (two previously-
untracked markers resolved with documentation, two historical mentions
reworded) so the sprint's target grep state is achievable.

**Files to modify**:
- `source/state/EstimateDump.h` — enum, `toString()`, `fill()`/
  `dumpEstimates()` call sites.
- `source/commands/DebugCommands.cpp` — `handleDbgEst`'s `snprintf` argument.
- `source/types/ArgSchema.h` — comment-only.
- `source/state/OutputState.h` — comment-only.
- `source/control/StopCondition.cpp:20` — comment-only.
- `source/control/ColorUtil.cpp:4` — comment-only.

**Testing plan**: run any existing `DBG EST` test in isolation to confirm
byte-identical output, then the full suite. `grep -ri FIXME source/` as a
final manual check that only the 8 units-suffix markers remain.

**Documentation updates**: none beyond the FIXME comments themselves (already
covered in Acceptance Criteria); no `docs/` file references `EstimateDump`,
`ArgKind`, or `digitalDirty` by name.
