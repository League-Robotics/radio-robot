---
date: 2026-07-02
sprint: 068
category: emergent-gap
---

# Uncommitted ticket-to-done moves were silently half-reverted by `git reset --hard`

## What Happened

At the end of sprint 068, tickets 001 and 002 appeared in **both** the main
`tickets/` directory and `tickets/done/`, while ticket 003 was cleanly present
only in `tickets/done/`. The stakeholder noticed the inconsistency and asked why
001 and 002 "did not get moved."

The framing turned out to be inverted: **all three were moved correctly.** 001
and 002 were then *resurrected* in the main directory. Reconstruction from the
filesystem and git reflog:

| Time (2026-07-02) | Event |
|---|---|
| 16:01:32 | commit `f24d92c` — 068-001 done (ticket updated in-place, main dir) |
| 16:05:51 | commit `0852bd1` — 068-002 done (ticket updated in-place, main dir) |
| ~16:06–16:20 | `move_ticket_to_done` run for 001 and 002 → filesystem `rename()` into untracked `tickets/done/` |
| **16:20:16** | **`git reset` (reflog: "reset: moving to HEAD")** — restored the tracked-but-deleted `main/001` and `main/002` from HEAD |
| 16:21:41 | ticket 003 finalized |
| 16:26:43 | commit `88d7c4f` — 068-003 done |
| after 16:26:43 | `move_ticket_to_done` run for 003 → `rename()` into `tickets/done/` (survives; no later reset) |

The decisive evidence: `main/001` and `main/002` both carry mtime **16:20:16**,
matching the reset reflog entry to the second. `main/003` has no such mtime — it
stays deleted (` D` in `git status`) because its move happened *after* the reset.

The root mechanism, confirmed in the CLASI source
(`clasi/ticket.py::Ticket.move_to_done` → `self.path.rename(new_path)`):

- `move_to_done()` is a **pure filesystem move with no git operations** — no
  `git mv`, no `git add`, no commit.
- To git, the move is two half-changes: a working-tree **deletion** of a tracked
  file (`tickets/001.md`) and an **untracked** new file (`tickets/done/001.md`).
- These moves were **never committed** for sprint 068 (unlike sprint 067, which
  has an explicit `chore(067): move completed tickets ... to done` commit).
- `git reset --hard HEAD` reverts tracked working-tree changes (restoring the
  deleted `main/001`, `main/002`) but **leaves untracked files alone**
  (`done/001`, `done/002` survive). The result is a duplicate, not a clean state.

Ticket 003 escaped only by timing luck — its move landed after the last reset.

## What Should Have Happened

Each ticket-to-done move should have been **committed to git promptly** after
closing the ticket (as sprint 067 did), so the move is durable and atomic in
git's eyes. Had the 001/002 moves been committed before 16:20, the
`git reset --hard` would have been a no-op against them and no duplicates would
exist.

Equally, the `git reset --hard` at 16:20:16 should not have been run while CLASI
had uncommitted working-tree moves. A narrower tool (`git stash`, or
`git checkout -- <specific path>`) would have discarded only the intended change
instead of resurrecting archived tickets.

## Root Cause

**Emergent gap.** The happy path works: move a ticket to `done/`, then a later
`git add -A && git commit` captures it. The process has no rule that the move
must be committed *before* the working tree is next reset, and the
`move_to_done` tool leaves git state that a `git reset --hard` silently reverts
by *half* (deletion undone, untracked addition kept). Nothing guards the window
between an uncommitted CLASI move and its commit, and nothing warns that a hard
reset in that window produces duplicates rather than a clean rollback.

Contributing factors:
- The CLASI `move_to_done()` uses `Path.rename()`, not `git mv`, so the move is
  never staged — it is invisible to git until a manual commit.
- Ticket closure and "commit the move" are decoupled steps; the second is easy
  to defer or skip.

## Proposed Fix

**1. Immediate remediation (sprint 068 working tree).** The `main/001` and
`main/002` copies are byte-identical to their `done/` counterparts. Delete the
two duplicates from `tickets/`, then commit all three moves so `done/` is
tracked:
```
git rm clasi/sprints/068-.../tickets/001-*.md clasi/sprints/068-.../tickets/002-*.md
git add clasi/sprints/068-.../tickets/done/
git commit -m "chore(068): move completed tickets 001-003 to done"
```

**2. Process rule (team-lead workflow).** Commit the ticket move as part of
closing each ticket — never leave `move_ticket_to_done` results uncommitted
across other git operations. Treat "ticket done" as: update status → move to
done → **commit the move** in the same step.

**3. Guard against destructive resets.** Before running `git reset --hard`,
check `git status` for untracked `tickets/done/` entries or ` D` deletions of
ticket files; if present, commit or stash them first. Prefer `git stash` /
targeted `git checkout -- <path>` over a blanket hard reset when CLASI artifacts
are in flight.

**4. Upstream TODO (CLASI tool).** Consider having `move_to_done()` use `git mv`
when the artifact lives in a git repo, so the move is staged atomically and a
subsequent `git reset --hard` reverts it *cleanly* (both halves) rather than
leaving a duplicate. File this against the CLASI repo
(`/Volumes/Proj/proj/ai-projects/clasi`).
