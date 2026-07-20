---
status: pending
---

# BUG (upstream CLASI tooling): `create_sprint` reuses sprint IDs that still exist in the state DB, silently corrupting phase/gate tracking

**Component:** CLASI itself (MCP server / state DB), *not* robot-project code.
CLASI version observed: `0.20260718.1` (`stale: false`).
Filed here because the CLASI source is an installed pipx package
(`~/.local/pipx/venvs/clasi/.../clasi/`) and is not reachable from this repo.

**Severity:** high — it silently writes gate results and phase transitions onto
the *wrong sprint's* record, and the documented process rules leave no legal
way to repair it.

## Summary

`create_sprint` allocates the next sprint ID from the **filesystem** (highest
`clasi/sprints/**/NNN-*` directory) without cross-checking the **state DB**
(`.clasi/.clasi.db`, table `sprints`). When a planned sprint's directory is
deleted but its DB row is not, the next `create_sprint` re-uses that ID. The
result is one ID with two different identities: the filesystem says one sprint,
the DB says another. Tools then disagree depending on which they read, and
writes land on the wrong record.

## Observed state (2026-07-20, this repo)

DB rows: **114**. Filesystem sprint dirs: **113**.

| id | DB `slug` | filesystem dir | status |
|---|---|---|---|
| 111 | `motion-accuracy-reliability-exact-turns-and-wedge-latch-flicker` | `111-motion-control-terminal-blips-close-the-loop-and-delete-the-compensator-stack` | **slug mismatch** |
| 112 | `firmware-comms-and-device-robustness-ack-ring-delivery-relay-handshake-fault-absent-device-re-probe` | `112-motion-control-terminal-blips-close-the-loop-feedback-feedforward-delete-the-compensator-stack` | **slug mismatch** |
| 113 | `host-p4-rewire-nezha-facade-nav-and-calibrate-on-the-binary-twist-plane` *(before repair)* | `113-config-as-truth-sim-configures-on-open-from-the-robot-config-file` | **collided; hand-repaired** |
| 114 | `repo-hygiene-and-reproducibility-devices-naming-sweep-stale-comment-audit-vendor-symlink` (`phase: roadmap`) | *(none)* | **orphan row** |

All four rows were created in one batch at `2026-07-17T21:34`, i.e. a
roadmap-planning session that scoped sprints 111–114. Those sprints were later
pre-empted and their directories deleted; **the DB rows were never removed.**
IDs 111–113 were subsequently re-used by new sprints on disk.

## How it broke a real sprint

Opening sprint 113 (config-as-truth):

1. `create_sprint` saw filesystem max = 112, allocated **113**, and created
   `clasi/sprints/113-config-as-truth-.../` — colliding with the stale row 113
   (`host-p4-...`, created three days earlier, no directory/branch/worktree
   anywhere).
2. `get_sprint_phase("113")` returned the **orphan's** identity — its slug,
   branch, and `created_at` — not the sprint that actually exists on disk.
3. `record_gate_result(113, "architecture_review", "passed")` and
   `advance_sprint_phase(113)` were then applied to that stale row. The
   architecture-review gate and a phase transition were recorded against a
   sprint that does not exist.

Nothing warned. The planning agent only caught it because the returned slug
looked unfamiliar.

## Why it is silent: readers disagree

- Read the **filesystem** (correct): `list_sprints`, `get_sprint_status`.
- Read the **DB** (stale): `get_sprint_phase`, `advance_sprint_phase`,
  `create_ticket`'s phase check.

So `get_sprint_status("113")` and `get_sprint_phase("113")` described two
different sprints simultaneously, with no error from either.

## No supported recovery path

There is no MCP tool to delete, re-anchor, or reconcile a sprint row.
`clear_sprint_recovery` targets the recovery singleton; `insert_sprint` does not
repair. The only way forward was editing `.clasi/.clasi.db` by hand — which
`.claude/rules/mcp-required.md` explicitly forbids ("Do NOT improvise
workarounds. All SE process operations require the MCP server"). The rule and
the tooling are in direct conflict the moment this bug fires.

## Secondary defect: `state_drift` is 100% noise

The status hook reports `state_drift` for **every** correctly-closed sprint
(109–113): *"declares status='closed' but is_close_report_present,
is_branch_merged, is_review_satisfied are False"*, and separately
*"'planning-docs' is not a recognised sprint machine state. Known states:
['open','planned','pre-flight','ticketed','executing','review','closed']"*.

The legacy `phase` vocabulary written by `create_sprint`/`detail_sprint`/
`advance_sprint_phase` (`roadmap`, `planning-docs`, `ticketing`, `executing`,
`done`) is not the state machine's vocabulary. Because it fires on healthy
sprints, the drift signal is untrustworthy — which is exactly why the genuine
ID-collision drift above blended into the background.

## Suggested fixes

1. **Allocate IDs from `max(filesystem, state DB) + 1`** — or better, have
   `create_sprint` refuse an ID that already exists in `sprints` and report the
   conflict instead of silently colliding.
2. **Make sprint teardown a supported operation.** "Delete the sprint dirs" is a
   normal thing to be told to do when sprints are pre-empted; it must have an
   MCP counterpart (`abandon_sprint`/`delete_sprint`) that removes or archives
   the DB row atomically with the directory.
3. **Add a reconcile/repair tool** for sprint rows, in the same spirit as
   `reconcile_worktrees`: detect (a) DB rows with no filesystem directory and
   (b) directories whose DB slug disagrees, then clean or re-anchor them. Until
   this exists, the "no improvised workarounds" rule has no legal escape hatch.
4. **Fail loud on DB↔FS disagreement.** `get_sprint_phase` and `create_ticket`'s
   phase check should compare the DB slug against the on-disk sprint directory
   for that ID and raise, rather than silently serving another sprint's record.
5. **Reconcile the two state vocabularies** (or map legacy phases onto the
   machine's states) so `state_drift` only fires on real drift.

## Workaround applied

Row 113 was hand-repaired (slug/branch/phase re-anchored to the real sprint;
the architecture-review gate was valid for it and kept) after backing up
`.clasi/.clasi.db`. Rows 111, 112, and the 114 orphan are being cleaned up
out-of-process separately; see the cleanup note appended when that is done.
