---
id: '004'
title: Relocate narrative comments to DESIGN.md/git; refresh stale doc references
status: done
use-cases: []
depends-on:
- '001'
- '002'
- '003'
- '005'
github-issue: ''
issue: relocate-narrative-comments-and-refresh-stale-docs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Relocate narrative comments to DESIGN.md/git; refresh stale doc references

## Description

The contract belongs in the header; the history belongs in DESIGN.md and
git. Runs LAST so it trims only what survives tickets 1-3's own changes.

**Amended (2026-07-23, mid-execution): now also depends on ticket 005**
(`straight-leg-crab-118-001-actuation-and-telemetry-pairing-skew.md`,
filed from a concurrent stakeholder-directed session, fixing a
same-generation actuation/telemetry defect in `robot_loop.cpp`). Ticket
005 edits `src/firm/app/DESIGN.md` directly to describe the corrected
schedule — this ticket's own `move_queue.h`/`robot_loop.cpp`-adjacent
narrative work in that same file must build on ticket 005's landed
description, not the pre-005 one. If ticket 005 hasn't landed by the
time this ticket starts, wait for it rather than describing the
about-to-be-superseded schedule.

**Scope re-measured against the tree (2026-07-23, post-118) — this is
what SURVIVES 118, not the original issue's pre-118 list verbatim:**

1. **`move_queue.h` is STILL 73% comment (281/385 lines).** 118 ticket
   004 added substantial new derivation prose for the land-at-zero
   formula on top of the pre-118 narrative the original issue measured
   (71%) — shrink post-118, do not re-confirm the stale pre-118 number.
2. **`sim_harness.h` is STILL 63% comment (383/608 lines)**, unchanged by
   118 (118 only touched `kCycleDtUs` and one DESIGN-note there). Full
   scope stands as originally issued.
3. **`tour.py`'s 104-line module docstring + mostly-prose constants
   block** — unchanged, still needs the relocation treatment. Coordinate
   with ticket 003's kwarg deletion (already landed by the time this
   ticket runs) so the docstring doesn't describe deleted kwargs.
4. **`test_tour_closure_gate.py`'s xfail reasons already shrank as a side
   effect of 118 ticket 004 rewriting their content** —
   `_XFAIL_REASON_IDEAL` is now ~30 lines (was 116), `_XFAIL_REASON_REALISTIC`
   is now ~21 lines (was 65) — real work remains to reach the issue's own
   "one sentence + live issue link" target. **`_STOP_LEAD_MS`'s ~51-line
   block is CONFIRMED FULLY GONE** (118 deleted it with the field) —
   nothing to do there.
5. **The dangling xfail citations of the deleted
   `cycle-order-reorder-experiment-ab-before-hardware.md` are CONFIRMED
   ALREADY FIXED** — 118 ticket 001 re-pointed both (in
   `test_tour_closure_gate.py` and `src/tests/sim/unit/test_app_robot_loop.py`)
   as part of its own scope; `grep -rn "cycle-order-reorder-experiment"`
   returns zero hits in either file today. **This item is REMOVED from
   this ticket's scope** — it is done. (Ticket 002, separately, handles
   the boundary test's OWN different xfail — a different test, a
   different reason, citing the now-archived `restore-the-interleaved-...`
   issue and a stale reorder-experiment theory 118 also invalidated. Do
   not duplicate ticket 002's work here; if ticket 002 hasn't landed by
   the time this ticket runs — it should have, per depends-on — verify
   before assuming it's done.)
6. **`.claude/rules/hardware-bench-testing.md` STILL points at
   `docs/protocol-v2.md` as current** (2 live links, verified 2026-07-23)
   and links the dead `source/commands/CommandProcessor.cpp` path — full
   scope stands. Rewrite the deploy/verify flow against
   `docs/protocol-v4.md` + MOVE-era bench scripts.
7. **`.claude/rules/coding-standards.md`'s `SystemCommands.cpp`
   reference** (the `HALT POS` wire-string example) — unfixed, unchanged.
   Re-point to its live equivalent or mark as a historical example (this
   one string is deliberately excluded from the units-in-identifiers
   convention as a wire-format constant — do not confuse "re-point the
   dead source path" with "rename the wire string," which stays stable).
8. **TestGUI "Managed — Ruckig" label** (`testgui/__main__.py:925`, stale
   comments `:735,:758`) — unfixed, unchanged; rename to "Managed" or
   "Managed — MOVE". **Scope check**: a broader repo grep for "Ruckig"
   turns up many more hits, ALL of them legitimate — historical sprint
   records (`docs/architecture/architecture-update-*.md`), point-in-time
   code reviews (`docs/code_review/`), and
   `docs/design/simple-velocity-control-guide.md`/
   `wpilib-motion-stack-comparison.md`'s own why-we-didn't-use-it
   discussion. Only the GUI label and its two stale comments are in
   scope; do not touch the legitimate historical/comparison references.
9. **`docs/specification.md`/`docs/architecture.md`** — verified: BOTH
   documents describe the ENTIRE pre-077 `source/` tree (ASCII
   `CommandProcessor`, `DriveController`, `PathFollower`/`PurePursuit`/
   `Stanley` nav layer) wholesale, not a handful of stale lines — the
   documents' complete subject is the deleted architecture. Per the
   issue's own "refresh or mark superseded-by pointers to docs/design/"
   instruction: add a clear superseded-by banner at each document's top
   pointing to `docs/design/design.md`. A full line-by-line rewrite of
   two documents describing dead code is explicitly OUT of scope for
   this mechanical-relocation ticket.
10. **No rule/doc points at `docs/protocol-v4.md` as current** —
    confirmed; same root cause as item 6 above (fixing that pointer
    fixes this).
11. **`src/host/robot_radio/DESIGN.md`'s `planner/` row claims `from
    robot_radio.planner.tour import TOUR_1, TOUR_2` "raises
    `AttributeError` at import time"** — **empirically verified FALSE**
    (2026-07-23): `uv run python -c "from robot_radio.planner.tour import
    TOUR_1, TOUR_2; print(TOUR_1 is not None, TOUR_2 is not None)"`
    prints `True True` cleanly. The `planner/` package is live enough to
    power `test_tour_closure_gate.py` and the button-acceptance suite's
    managed-motion tests, which both import and call `run_tour()`
    directly. Correct the row to the actual mixed status (live enough to
    power the closure gate and managed-motion tests; say precisely which
    parts, if any, are genuinely still dormant).
12. **NEW finding (2026-07-23, not in the original issue — discovered
    verifying the tree per this sprint's own "verify, don't trust the
    roadmap blindly" instruction):** `docs/protocol-v4.md` §8's own
    header line ("... primary period == cycle period, ~50 Hz / 20 ms
    ... unchanged by sprint 116") is now WRONG — 118 changed
    `kCycle`/`kPrimaryPeriod` to 40ms/~25Hz and this specific line was
    never updated (118's own doc-update scope named
    `src/firm/app/DESIGN.md`/`src/sim/DESIGN.md`/`docs/design/design.md`,
    not this file — a gap, not a deliberate exclusion). Fold this fix
    into item 10 above (same document, same class of staleness).

## Scope guard

- Design-docs opt-in is enabled: `motion/DESIGN.md`'s own overlay (this
  sprint's slot, owned by ticket 002) must have already landed before
  this ticket touches anything that references it; `src/firm/app/DESIGN.md`
  (ticket 001) and `src/host/robot_radio/DESIGN.md` (ticket 001 partially,
  this ticket for the `planner/` row + `tour.py` docstring relocation)
  likewise. This ticket's own `DESIGN.md`-adjacent edits ride whatever
  overlay/direct-edit mechanism the owning subsystem uses; coordinate so
  this ticket's diff doesn't clobber tickets 1-3's already-landed edits
  to the SAME files (`src/host/robot_radio/DESIGN.md` is touched by both
  ticket 1 and this ticket — verify the file's state before editing, and
  add to it rather than reverting).

## Acceptance Criteria

- [x] `move_queue.h`/`sim_harness.h`/`tour.py` headers reduced to
      contract-only (target: comment ratio < 40% in the headers — both
      currently well above that; re-measure after edits); relocated
      history lands in the owning `DESIGN.md` (`src/firm/app/DESIGN.md`
      for `move_queue.h`, `src/sim/DESIGN.md` for `sim_harness.h`,
      `src/host/robot_radio/DESIGN.md` for `tour.py`); no sprint numbers
      outside DESIGN.md/git references in the headers themselves.
      **Measured: `move_queue.h` 73.4% (292/398) → 39.8% (68/171);
      `sim_harness.h` 63.0% (383/608) → 37.4% (132/353); `tour.py`
      772 → 673 lines. Code-line counts verified unchanged by diff
      (78 and 185 respectively) — no behavior touched.**
- [x] `test_tour_closure_gate.py`'s xfail reasons shrunk to one sentence
      + a live issue link (both `_XFAIL_REASON_IDEAL` and
      `_XFAIL_REASON_REALISTIC`, currently ~30 and ~21 lines
      respectively). **Shrunk to 6 and 5 lines; both link
      `clasi/issues/land-at-zero-at-orthogonal-chain-boundaries.md`
      (verified exists, live/pending, and matches the currently
      measured 0.2-2.2deg residual magnitude).**
- [x] `.claude/rules/hardware-bench-testing.md` rewritten against
      `docs/protocol-v4.md` + MOVE-era bench scripts; no
      `docs/protocol-v2.md`/`source/commands/` reference survives there.
- [x] `.claude/rules/coding-standards.md:170`'s `SystemCommands.cpp`
      reference re-pointed or marked historical.
- [x] TestGUI "Managed — Ruckig" label renamed; the two stale comments
      (`:735`, `:758`) corrected. **(actual current line numbers `:740`,
      `:763`, `:930` — content matched, line numbers had drifted since
      the issue was filed.)**
- [x] `docs/specification.md` and `docs/architecture.md` each carry a
      clear superseded-by banner pointing to `docs/design/design.md`.
      **`docs/overview.md` also banner'd — same pre-077-wholesale
      failure mode, found while verifying doc entry points for the
      protocol-v4 pointer criterion below.**
- [x] `docs/protocol-v4.md` established as the pointed-to protocol doc
      wherever `.claude/rules/` or doc entry points currently point at
      v2; §8's own stale cadence line corrected to 40ms/~25Hz.
      **§5.2's own matching stale `~50Hz/20ms` line (same class of
      staleness, found in the same document) fixed alongside it.**
- [x] `src/host/robot_radio/DESIGN.md`'s `planner/` row corrected to its
      actual live/mixed status, verified via the exact import command
      above (or an equivalent re-check at execution time). **Re-verified
      2026-07-23: import succeeds; `run_tour()` confirmed called
      directly by `test_tour_closure_gate.py` and
      `test_gui_button_acceptance.py`.**
- [x] Grep gate: no reference to `source/commands/`, `protocol-v2.md`
      (outside `docs/protocol-v2.md` itself and archives), `Ruckig`
      (outside historical DESIGN.md/architecture-update/code_review/
      design-guide notes), or the deleted
      `cycle-order-reorder-experiment-ab-before-hardware.md` filename.
      **Full accounting in the closing report below — one out-of-scope
      finding flagged (CMakeLists.txt's own stale Ruckig-restored
      comment), not fixed here per the issue's own "ALL legitimate"
      scope check; left for a follow-up.**
- [x] Full `uv run python -m pytest` suite green; no behavior diffs
      (comment/docs/label changes only, except the one GUI label
      string). **1387 passed, 2 skipped, 9 xfailed, 2 xpassed
      (non-strict, both pre-existing/unrelated to this ticket's files),
      exit 0. `python build.py` also green (ARM hex + HOST_BUILD sim
      dylib both compiled clean).**
- [x] Bench verification is DEFERRED to the phase-B bench session — not
      required to close this ticket.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite) — a
  behavior-diff check, since this ticket should produce none beyond the
  GUI label string.
- **New tests to write**: none expected — mechanical relocation/doc
  fixes with an existing regression net.
- **Verification command**: `uv run python -m pytest`, plus the grep
  gates listed in Acceptance Criteria (document exact commands used in
  this ticket's own closing notes for future reference).
