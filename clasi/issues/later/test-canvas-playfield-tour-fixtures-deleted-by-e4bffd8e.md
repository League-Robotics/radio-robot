---
status: pending
priority: medium
filed: 2026-08-11
filed_by: "programmer (sprint 136 ticket 002, full-suite baseline triage)"
tickets:
- 136-002
---

# `test_canvas.py`'s three fixture files were deleted by an unrelated commit (e4bffd8e) -- probably accidental, needs a stakeholder call

## Description

`src/tests/testgui/test_canvas.py`'s three tests
(`test_asset_path_constants_resolve_under_tests_old`,
`test_load_calibration_reads_real_json_dimensions`,
`test_build_playfield_calibration_reads_real_json_dimensions`) all fail
because `canvas.py`'s three asset-path constants point at files that no
longer exist anywhere in the repository:

```
src/archive/tests_old/old/playfield_tour/playfield.jpg
src/archive/tests_old/old/playfield_tour/playfield_calibration.json
```

`git ls-files | grep -i playfield_tour` returns **zero** matches at current
HEAD -- these files, and the entire `src/archive/` tree they lived under
(`tests_old/`, `source_old/`, `wedgelab/`, `hex/`, `host_scripts/`), are
untracked. They were committed as recently as `575ef391` ("unify all source
trees under src/") and confirmed present at that commit.

## Root cause -- traced to a specific commit, not assumed

```
git log --all --diff-filter=D --name-only --oneline | grep -i playfield_tour
```

names exactly one commit: **`e4bffd8e`**, `"feat: Introduce MicroPython as
the base for on-robot programmability and I2C bus-owning fiber spike"`,
dated 2026-08-07. Its full diffstat: **639 files changed, 953
insertions(+), 198420 deletions(-)**. The insertions are two planning
markdown docs (`kernel-packaging-host-sim-rigor-and-hardware-abstraction-
program-plan.md`, `micropython-as-the-base-feasibility-and-migration-
plan.md`) plus `spike-i2c-bus-owning-fiber.md` and a `.clasi/.clasi.db`
blob update -- **the commit message names none of the ~200K deleted lines**,
which span the ENTIRE `src/archive/` tree (not just `tests_old/`):
`wedgelab/` utilities (esptool.py, cmake toolchains, doc generators),
`source_old/` (the pre-077-rebuild C++ tree), two baked `.hex` images
(~48K lines), `host_scripts/` calibration scripts, and `tests_old/`
(including this test's fixtures).

This is confirmed reachable from the current tree
(`git merge-base --is-ancestor e4bffd8e HEAD` -> yes, on both `master` and
this sprint branch).

**This looks like an accidental bulk deletion bundled into an unrelated
feature commit** (e.g. a stray `git rm -r src/archive` or a broad
`git add -A` after local cleanup, committed together with the two actually-
intended doc additions) -- but this is not confirmed, and it is also
possible the deletion was intentional cruft removal that simply got a
misleading commit message. Nothing in `clasi/issues/later/C-cruft-ledger-
sweep-zero-consumer-code.md` (the tracked cruft-sweep issue) mentions
`src/archive/`, `wedgelab/`, `source_old/`, or `tests_old/` at all -- it is
scoped to `src/firm`, `src/motion`, `src/host` -- so it is NOT the source of
an intentional decision to remove this tree.

## Why this ticket does not just fix the test

Restoring ~200K lines from history, or deciding this deletion was correct
and rewriting `test_canvas.py`'s fixtures from scratch, are both real
decisions with consequences (repo size, whether `canvas.py`'s calibration-
loading fallback behavior is still meant to be exercised against a real
fixture at all) that are out of proportion for a baseline-triage ticket
and need a stakeholder call, not a triage judgment.

## What to do

1. **Stakeholder decision**: was `e4bffd8e`'s deletion of `src/archive/`
   intentional? If yes, `test_canvas.py`'s three tests should be deleted or
   rewritten against a small, purpose-built fixture instead of the archived
   one. If no, restore at minimum
   `src/archive/tests_old/old/playfield_tour/{playfield.jpg,
   playfield_calibration.json}` from `git show 575ef391^:...` (the last
   commit before the deletion that has them) -- `git checkout
   575ef391^ -- src/archive/tests_old/old/playfield_tour/` recovers them
   directly if the tree is wanted back.
2. Either way, split `e4bffd8e`'s two legitimate doc additions from the
   archive deletion in the historical record is not necessary -- just fix
   forward per (1).

## Verification

- `test_canvas.py`'s three tests pass, either against a restored fixture or
  a deliberately rewritten one.
- The stakeholder has confirmed whether `src/archive/`'s removal was
  intended, so it isn't silently re-litigated by a future cruft sweep.

## Related

- `clasi/issues/later/C-cruft-ledger-sweep-zero-consumer-code.md` -- the
  actual tracked cruft-removal scope; does not cover this tree.
- Commit `e4bffd8e` -- the deletion.
- Commit `575ef391` -- last commit confirmed to carry the files intact.
