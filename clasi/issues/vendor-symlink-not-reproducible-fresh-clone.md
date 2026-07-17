---
title: src/vendor is a symlink into an unrelated sibling repo's working tree — not reproducible on a fresh clone/CI
status: open
filed-by: programmer (sprint 109 ticket 001)
filed-date: 2026-07-17
updated: 2026-07-17 — ruckig relocated out of src/vendor, resolved for
  ruckig specifically; the shared reference pool's remaining contents are
  the still-open question.
---

# `src/vendor` is a symlink into an unrelated sibling repo's working tree

## Update 2026-07-17 — ruckig resolved, moved to repo-root `vendor/`

The concrete trigger for filing this issue (sprint 109 ticket 001's Ruckig
restore landing as uncommitted files inside the sibling repo's working
tree) is **resolved**: Ruckig now lives at this repo's own root,
`vendor/ruckig/` (a REAL, git-tracked directory, sibling to `src/`, NOT
under the `src/vendor` symlink) — a compiled-in firmware dependency has to
be reproducible from radio-robot-elite's own git history alone, which
`src/vendor/*` structurally cannot provide (see below). Every reference
(`CMakeLists.txt`, `src/sim/CMakeLists.txt`, `src/firm/motion/DESIGN.md`,
`src/firm/DESIGN.md`, `vendor/ruckig/README.vendored.md`) was updated to
the new path; `git ls-files vendor/ruckig` confirms the files are tracked
in this repo; the stray untracked copy that had landed in the sibling
repo's working tree was removed, restoring that repo to exactly the state
it was found in (its own pre-existing unrelated dirty state, untouched).

**What remains open** (the rest of this issue, below, is otherwise
unchanged): the `src/vendor` symlink itself was deliberately left as-is —
it still serves the OTHER, reference-only material (`PurePursuit`,
`PythonRobotics`, `pxt-Cutebot-Pro`, `pxt-nezha2`, `pxt-planetx`, `docs`),
none of which anything compiles against. Whether THAT shared pool should
also move in-repo (or be documented/bootstrapped more robustly) is still
a stakeholder decision — ruckig was the one entry with a compiled-in
build dependency forcing an immediate fix; the rest can wait for a
deliberate decision rather than an emergency one.

## What was found (original filing)

`src/vendor` (radio-robot-elite) is a tracked symlink (git mode `120000`,
committed in `refactor(repo): unify all source trees under src/`) pointing
to `/Volumes/Proj/proj/league-projects/scratch/radio-robot/vendor` — the
working tree of a **different, older sibling project** (`radio-robot`,
remote `https://github.com/ericbusboom/radio-robot.git`), currently on an
unrelated branch (`sprint/011-sequester-tn-and-g-command-logic`) with its
own independent dirty state.

Every existing entry under `src/vendor/` in radio-robot-elite
(`PurePursuit`, `PythonRobotics`, `pxt-Cutebot-Pro`, `pxt-nezha2`,
`pxt-planetx`, `docs`) physically lives inside that other repo's checkout.
`PurePursuit`/`PythonRobotics` are real git submodules of the OTHER repo;
they are not tracked by radio-robot-elite's own git history at all.

Sprint 109 ticket 001 restored `src/vendor/ruckig/` (vendored Ruckig,
`git show c63ec6c:libraries/ruckig`) into this same shared location,
matching the existing convention — but this means:

- `git status`/`git log` in radio-robot-elite never shows any change under
  `src/vendor/*` — by design of the symlink boundary (the same way
  `src/libraries/` is invisible via `.gitignore`, but for a structurally
  different reason: it's not ignored, it's simply outside this repo).
- A fresh clone of radio-robot-elite, or a CI runner, will have a **dangling
  symlink** at `src/vendor` unless it ALSO has that exact sibling checkout
  at that exact absolute path (`/Volumes/Proj/proj/league-projects/scratch/
  radio-robot/vendor`) — the build will fail to find
  `ruckig/ruckig.hpp`/`PurePursuit`/etc. with no obvious error pointing at
  the real cause.
- Any developer restoring/adding a new `src/vendor/*` entry (as this
  ticket did for `ruckig/`) is committing content into an **unrelated
  project's git history's working tree**, on whatever branch that project
  happens to be on at the time — a latent risk of cross-project
  contamination if that content is ever accidentally `git add`ed over
  there.

## Why this wasn't fixed in ticket 109-001

Out of scope for a foundation/restore ticket — this is a pre-existing
repo-wide structural property (every `src/vendor/*` entry already has it),
not something introduced by the Ruckig restore. Ticket 109-001 followed the
existing convention rather than relitigating it.

## Suggested follow-up (stakeholder decision needed)

- Decide whether `src/vendor/` should become a REAL, in-repo, tracked
  directory (submodules or vendored-in-place, as `109-001`'s
  `README.vendored.md` describes) instead of a symlink to a sibling
  project's working tree, OR
- If the shared-cache symlink is intentional (e.g. to avoid duplicating
  large vendored trees across sibling `radio-robot`/`radio-robot-elite`
  checkouts on the same machine), document that convention explicitly
  (e.g. in `src/vendor/CLAUDE.md`, which currently only has generic CLASI
  boilerplate) and add a fresh-clone bootstrap step (a `just setup`-style
  recipe) that fails loudly with a clear message if the symlink target is
  missing, rather than a confusing "header not found" compile error.
