---
id: '114'
title: 'Repo hygiene and reproducibility: devices naming sweep, stale-comment audit,
  vendor symlink'
status: roadmap
branch: sprint/114-repo-hygiene-and-reproducibility-devices-naming-sweep-stale-comment-audit-vendor-symlink
worktree: false
use-cases: []
issues:
- devices-naming-sweep-units-in-identifiers.md
- audit-stale-comments-repo-wide.md
- vendor-symlink-not-reproducible-fresh-clone.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 114: Repo hygiene and reproducibility: devices naming sweep, stale-comment audit, vendor symlink

## Goals

Lowest-urgency cleanup of the four roadmap sprints planned this round.

(a) Mechanical units-in-identifiers sweep of `source/devices/` (methods,
constants, members, parameters currently carrying `Us`/`Ms`/`Mm`/etc.
suffixes) per `.claude/rules/coding-standards.md` — name the quantity,
move the unit into a `// [unit]` trailing tag. Batch-dispatch style: bulk
regex, per-ticket commits. Sequenced after the single-loop rebuild's P2/P3
(already landed), so survivors are stable and this is ready now.

(b) Repo-wide audit of source comments (live trees `src/`, `host/`, `tests/`
only — parked trees `source_old/`, `tests_old/` out of scope) to validate,
fix, or delete stale design/architecture claims, cross-references, and
behavioral claims — e.g. `src/sim/plant/wheel_plant.h`'s stale "LEAF-GETTER-
DRIVEN, not bus-byte-driven" claim, superseded by sprint 108. Do not weaken
true, load-bearing comments — the goal is accuracy, not reduction.

(c) Resolve the `src/vendor` symlink reproducibility question. Ruckig
itself is already relocated to repo-root `vendor/ruckig/` and resolved; the
remaining shared reference pool (`PurePursuit`, `PythonRobotics`,
`pxt-Cutebot-Pro`, `pxt-nezha2`, `pxt-planetx`, `docs` — currently a symlink
into an unrelated sibling repo's working tree) needs a stakeholder decision:
either move it in-repo (tracked/vendored), or document the shared-cache
convention explicitly and add a fail-loud fresh-clone bootstrap step.

## Scope

### In Scope

- `source/devices/` units-in-identifiers rename sweep (methods, constants,
  members, parameters).
- Stale-comment audit and correction across `src/`, `host/`, `tests/`.
- `src/vendor` symlink reproducibility decision and follow-through
  (in-repo migration, or documented convention + fresh-clone bootstrap
  step).

### Out of Scope

- Motion accuracy / wedge-latch — sprint 111.
- Comms/device robustness — sprint 112.
- Host P4 mid-layer rewrite — sprint 113.
- Any `source_old/`/`tests_old/` parked-tree content (comment audit
  explicitly excludes these; naming sweep is scoped to `source/devices/`
  only).

## Acceptance Sketch (at-a-glance)

- No remaining unit-suffixed identifier in `source/devices/`; every renamed
  quantity carries a correct `// [unit]` trailing tag; existing tests still
  pass (mechanical rename, no behavior change).
- Comment audit's known starting instance (`wheel_plant.h`) and any other
  comments citing deleted/superseded sprint-105-era mechanisms are fixed or
  removed; no new false claims introduced.
- `src/vendor`'s shared reference pool is either a real in-repo tracked
  directory, or has a documented convention plus a bootstrap step that fails
  loudly (not a confusing header-not-found compile error) when the symlink
  target is missing.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [ ] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
