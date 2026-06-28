---
id: '004'
title: 'Documentation: protocol-v2.md and COMMANDS.md'
status: open
use-cases:
- SUC-004
depends-on:
- 052-002
issue: stop-conditions-as-a-first-class-system-primitive.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Documentation: protocol-v2.md and COMMANDS.md

## Description

Update `docs/protocol-v2.md` §10 and `source/COMMANDS.md` to document the new
`stop=` grammar and `reason=` field introduced in sprint 052. This ticket is
documentation-only; no source code changes.

## Implementation Plan

### docs/protocol-v2.md §10 changes

The relevant section is §10 (Motion Commands), lines ~589-813.

**1. EVT Completion Events subsection (lines ~599-632)**

Add `reason=` to the EVT format description. Current text:

> EVT done T/D/G and EVT safety_stop carry a trailing '#<id>'...

Add after the existing paragraph:

> Starting with sprint 052, every `EVT done …` and `EVT safety_stop` line
> also carries a trailing `reason=<token>` field indicating why the motion ended:
>
> | Reason token | Fired by                                  |
> |---|---|
> | `time`      | Time stop (`stop=t:` or T/D built-in time stop) |
> | `dist`      | Distance stop (`stop=d:` or D built-in distance stop) |
> | `rot`       | Rotation stop (`stop=rot:`)              |
> | `heading`   | Heading stop (`stop=heading:`)           |
> | `pos`       | Position stop (G/GOTO arrival)           |
> | `line`      | Line-any stop (`stop=line:`)             |
> | `color`     | Color-match stop (`stop=color:`)         |
> | `<channel>` | Sensor stop (`stop=sensor:<ch>:`) — token is the channel name, e.g. `line0` |
> | `watchdog`  | Safety watchdog expired (`EVT safety_stop reason=watchdog`) |
>
> The `reason=` token is additive: existing hosts that match on the verb
> (`EVT done T`) continue to work. `reason=` follows any `#<id>` token:
>
> ```
> EVT done T #12 reason=time
> EVT done D reason=dist
> EVT safety_stop reason=watchdog
> ```

**2. stop= Grammar subsection (new, after EVT Completion Events)**

Add a new `### stop= Clauses` subsection:

> Any open-loop motion command (VW, S, R, T, D, TURN) may carry one or more
> `stop=<kind>:<args>` clauses. Each clause adds a stop condition that fires
> when the condition is satisfied; conditions are OR-combined.
> Up to 4 `stop=` clauses per command (kMaxStopConds = 4).
>
> | Clause | Fires when |
> |---|---|
> | `stop=t:<ms>` | Duration ≥ ms milliseconds |
> | `stop=d:<mm>` | Average encoder travel ≥ mm millimetres |
> | `stop=line:<ge\|le>:<thr>` | Any of line[0..3] satisfies the threshold |
> | `stop=sensor:<ch>:<ge\|le>:<thr>` | Named channel satisfies the threshold (ch: line0..line3, colorR..colorC, analogIn0..analogIn3) |
> | `stop=color:<h>:<s>:<v>:<dist>` | HSV colour distance from target ≤ dist |
> | `stop=heading:<cdeg>:<eps_cdeg>` | Heading within eps of target (centi-degrees) |
> | `stop=rot:<arc_mm>` | Per-wheel encoder arc ≥ arc_mm |
>
> `sensor=<ch>:<op>:<thr>` is accepted as a back-compat alias for
> `stop=sensor:<ch>:<op>:<thr>`.
>
> T and D retain their positional time/distance args AND may have additional
> `stop=` clauses (OR-combined with the built-in stop):
> ```
> T 200 200 1000 stop=sensor:line0:ge:512
> D 200 200 300 stop=t:5000
> VW 200 0 stop=d:300 stop=t:5000
> S 200 200 stop=line:ge:512
> ```

**3. Update per-verb examples**

In the S, T, D, VW subsections (lines ~625-770), add `stop=` to the syntax
lines and at least one example per verb.

### source/COMMANDS.md changes

The verb table (around line 55-60) currently has columns for verb, description,
etc. Add a `stop=` column or a note below the table:

> All open-loop motion verbs (VW, S, T, D, R, TURN) accept one or more
> `stop=<kind>:<args>` clauses. See `docs/protocol-v2.md` §10 for the full
> grammar. Up to 4 clauses per command; OR-combined.

## Files to Create or Modify

- `docs/protocol-v2.md` — §10 additions (EVT Completion Events, new stop=
  Clauses subsection, per-verb syntax updates).
- `source/COMMANDS.md` — note in verb table about stop= support.

## Acceptance Criteria

- [ ] `docs/protocol-v2.md` §10 has a `reason=` table listing all 9 reason tokens.
- [ ] `docs/protocol-v2.md` §10 has a `stop=` Clauses subsection with the 7-kind
  grammar table.
- [ ] `docs/protocol-v2.md` §10 shows `EVT done T #12 reason=time` as an example EVT line.
- [ ] `docs/protocol-v2.md` §10 shows `EVT safety_stop reason=watchdog` as an example.
- [ ] `source/COMMANDS.md` notes stop= acceptance on VW, S, T, D, R, TURN.
- [ ] Back-compat note: `sensor=` alias is documented.
- [ ] Sim tests pass: `uv run --with pytest python -m pytest tests/simulation -q` — no new failures (docs changes only; test_lint.py checks are expected to pass).

## Testing

**Verification command**: `uv run --with pytest python -m pytest tests/simulation -q`

**Pre-existing baseline**: 2 failures. No new failures acceptable.

No new tests needed. Verify `test_lint.py` still passes (it checks Python files,
not Markdown). The `test_imports_smoke.py` and `test_tools_smoke.py` tests verify
no import errors; they are unaffected by Markdown changes.
