---
id: "003"
title: "P0 spike: wire-frame budget dry run"
status: open
use-cases: ["SUC-003"]
depends-on: []
github-issue: ""
issue: single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# P0 spike: wire-frame budget dry run

## Description

Draft the pruned wire protocol on a **scratch branch only** and confirm it
fits the existing envelope size budget, with no hardware involved. The
current envelope is `kCommandEnvelopeMaxEncodedSize=168` against a 186 B
ceiling (`source/messages/wire.h:56,58`). This ticket prunes
`protos/envelope.proto` to the P4 command shape
(`twist{v_x, omega, duration}` / `config{delta}` / `stop{}` + `corr_id`,
dropping segment/replace/plan_dump/etc.) and extends
`protos/telemetry.proto` with an ack ring (4-8 entries of
`{corr_id, status, err_code}`) plus fault/event bits, moving
`acc_*`/`glitch_*`/`ts_*`/`cmd_vel_*` fields to a slower secondary frame —
then runs `scripts/gen_messages.py` and reads the `wire.h` static_assert
pass/fail as the verdict. **The draft protos do NOT merge into this
sprint's branch or into `protos/` on `master`** — this is a paper/codegen
exercise that de-risks the P4 design before P2 deletes the current
production protocol surface.

## Acceptance Criteria

- [ ] Work happens on a scratch branch (not this sprint's branch), branched
      from the current HEAD.
- [ ] `protos/envelope.proto` pruned per the issue's field list: `twist`,
      `config`, `stop` command arms + `corr_id`; segment/mover/plan_dump/
      STREAM/GET/EVT arms removed.
- [ ] `protos/telemetry.proto` extended with the ack ring (4-8 entries,
      `{corr_id, status, err_code}`) and fault/event bits; `acc_*`,
      `glitch_*`, `ts_*`, `cmd_vel_*` moved to a slower secondary frame
      definition.
- [ ] `scripts/gen_messages.py` runs clean against the draft protos and
      regenerates the message headers on the scratch branch.
- [ ] The build's `wire.h` static_asserts are read as the pass/fail
      verdict — no separate interpretation layer. If they fail, the
      specific overflowing field(s) and their byte cost are reported so P4
      (sprint 103/104) can resize before committing to the design.
- [ ] Scratch branch and its generated output are explicitly NOT merged
      into sprint 102's branch or into `master`'s `protos/` — verify no
      `protos/` or generated-message diff appears in this sprint's actual
      commits.
- [ ] No hardware used for this ticket.

## Implementation Plan

**Approach**: Create a scratch branch off current HEAD (outside this
sprint's branch), edit `protos/envelope.proto` and `protos/telemetry.proto`
per the field list above, run `scripts/gen_messages.py`, then build (or at
minimum compile the generated `wire.h` translation unit) to trigger the
static_asserts. Record the pass/fail result and byte-budget numbers.
Discard or archive the scratch branch — do not merge.

**Files to create/modify** (scratch branch only, never merged):
- `protos/envelope.proto`
- `protos/telemetry.proto`
- Regenerated `source/messages/*` (generated output, scratch branch only)

**Files to create/modify (this sprint's branch, the only persistent
artifacts)**:
- This ticket file, recording the verdict and byte-budget numbers.
- Optionally a short note under `.clasi/knowledge/` or referenced from
  `architecture-update.md`'s Step 7 item 3, if the result is non-obvious
  enough to be worth a standing note (author's judgment at execution time).

**Testing plan**: The `wire.h` static_asserts ARE the test — a build
failure means the frame doesn't fit; a clean build means it does. No new
pytest needed. If the static_assert fails, report the specific field(s)
over budget and by how much (not just "it failed").

**Documentation updates**: record the verdict and byte-budget numbers in
this ticket; no change to `protos/` or `source/messages/` survives on this
sprint's branch.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (surviving suite —
  unaffected, since nothing merges to this sprint's branch from the
  scratch work).
- **New tests to write**: none on this sprint's branch; the scratch
  branch's own build IS the test, and it is discarded/archived, not
  committed here.
- **Verification command**: `uv run pytest`
