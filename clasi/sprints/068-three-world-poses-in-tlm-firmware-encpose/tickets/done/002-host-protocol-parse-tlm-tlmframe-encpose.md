---
id: '002'
title: 'Host protocol: parse_tlm() + TLMFrame.encpose'
status: done
use-cases:
- SUC-001
depends-on:
- '001'
github-issue: ''
issue: tlm-three-world-poses-encoder-only-pose.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host protocol: parse_tlm() + TLMFrame.encpose

## Description

Ticket 001 puts `encpose=<x>,<y>,<h>` on the wire. This ticket teaches the
host-side protocol layer to parse it into a structured, typed frame,
mirroring the existing `otos=` field exactly — `encpose=` is, like `otos=`
and `pose=`, an absolute wire pose with identical shape (mm, mm,
centidegrees), so the parse logic is a straight copy of the `otos` block,
not a new pattern.

Find the host protocol module (per `architecture-update.md`, this is
`host/robot_radio/robot/protocol.py`; confirm exact path/module name in the
codebase before editing — the sprint architecture doc's location may drift
slightly from a fresh `grep` for `TLMFrame`/`parse_tlm`).

## Acceptance Criteria

- [x] `TLMFrame` (dataclass or equivalent) gains an `encpose: tuple[int,
      int, int] | None = None` field, matching the type/shape of the
      existing `otos` field.
- [x] `parse_tlm()` gains a parse block for the `"encpose"` key, identical
      in shape to the existing `"otos"` parse block (splits the
      comma-separated `x,y,h` value, converts to the appropriate numeric
      types, assigns into `frame.encpose`).
- [x] A TLM line with `encpose=` present parses correctly into
      `frame.encpose`.
- [x] A TLM line with `encpose=` absent (e.g., a pre-068 firmware, or a
      `STREAM fields=...` subscription that excludes it) leaves
      `frame.encpose` as `None` — no exception, no crash. This is the
      version-skew case `architecture-update.md`'s Migration Concerns
      section calls out explicitly.
- [x] A malformed `encpose=` token (wrong arity, non-numeric component) is
      handled the same way the existing `otos=`/`pose=` parse blocks
      handle malformed input (whatever that established behavior is —
      match it, don't invent a new error-handling policy for this one
      field).
- [x] New/extended protocol unit tests cover all three cases above:
      `encpose` present, `encpose` absent, `encpose` malformed.
- [x] No other `TLMFrame` field or `parse_tlm()` parse block is changed.
- [x] Full default pytest suite green (`uv run python -m pytest`).

## Testing

- **Existing tests to run**: the host protocol unit test module covering
  `parse_tlm`/`TLMFrame` (locate via `grep -r "parse_tlm" tests/`); full
  default suite via `uv run python -m pytest`.
- **New tests to write**: three cases — `encpose` present (assert parsed
  tuple matches expected x/y/h), `encpose` absent (assert `frame.encpose is
  None`), `encpose` malformed (assert the same handling as an existing
  malformed-field test for `otos`/`pose`, e.g. skip/ignore/raise per
  existing convention).
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Locate `TLMFrame`/`parse_tlm` (expected at
`host/robot_radio/robot/protocol.py` per the architecture doc; verify with
`grep -rn "class TLMFrame\|def parse_tlm" host/`). Copy the `otos` field
declaration and its parse block verbatim, renaming to `encpose`. This
ticket depends on Ticket 001 because the golden-TLM fixture and any
live-firmware-format test fixtures used to validate parsing require
`encpose=` to actually be on the wire.

**Files to modify**:
- `host/robot_radio/robot/protocol.py` (or wherever `TLMFrame`/`parse_tlm`
  actually live — confirm exact path first) — add `encpose` field and
  parse block.
- The corresponding protocol unit test file — add present/absent/malformed
  coverage for `encpose`.

**Testing plan**:
- Add the three new/extended unit test cases described above.
- Run the full default suite (`uv run python -m pytest`) and confirm no
  regressions to existing `otos=`/`pose=` parsing.
- Confirm the golden-TLM-derived fixtures (from Ticket 001's regenerated
  capture) parse cleanly with `frame.encpose` populated.

**Documentation updates**: none beyond what Ticket 001 already covers in
`docs/protocol-v2.md` — this ticket is host-side parsing only, no
wire-format change.
