---
status: pending
priority: high
---

# CLASI's role-guard hook sees every subagent as team-lead, so no programmer can write its own ticket body

## Description

Found while executing sprint 136 ticket 001 (2026-08-12). A dispatched
`programmer` agent finished its implementation cleanly but **could not check off
its own acceptance criteria or write Completion Notes**. Three attempts, same
rejection each time:

```
[clasi hook role-guard]: CLASI ROLE VIOLATION: team-lead (tier 0)
attempted direct file write
```

The programmer was not team-lead. The hook only thinks it was.

## Cause

`clasi/hook_handlers.py` resolves the acting agent purely from an environment
variable, defaulting to team-lead when it is unset:

```python
# hook_handlers.py:651
agent_name = os.environ.get("CLASI_AGENT_NAME", "team-lead")
# also :302, :980 -- same pattern
```

Claude Code hooks run in the **main session's process environment**. The Agent
tool does not propagate a per-subagent `CLASI_AGENT_NAME`, and a shell-level
`export` inside a subagent's Bash call cannot reach the hook process either (the
hook is spawned by the harness, not by that shell). `CLASI_AGENT_NAME` is unset
in this project -- it appears nowhere in `.claude/` or `.mcp.json`.

So **every** dispatched subagent is evaluated as tier-0 team-lead, regardless of
what it actually is.

Confirmed by direct test, both directions:

- As real team-lead, editing a ticket with no ticket in-progress ->
  `"sprint 136 execution lock is held but no ticket is in-progress"`.
- As real team-lead, editing a ticket *with* one in-progress ->
  `"team-lead cannot directly edit sprint artifacts"`.

Both are the guard working correctly for team-lead. The programmer got the
*same* team-lead treatment, which is the defect.

## Impact

Narrower than it first appears, but real:

- **Source files are unaffected.** The guard only gates *sprint artifacts*, so
  ticket 001's actual work (two robot JSONs, six test files, a doc) landed
  normally and is committed at `fc2869a2`.
- **Ticket bodies degrade.** No programmer can tick its acceptance criteria or
  write Completion Notes. Frontmatter still works, because
  `update_ticket_status` goes through MCP and does not hit the pre-write hook --
  so a ticket ends up `status: done` with unchecked boxes and a template body.
- **The audit trail moves to commit messages.** That is where the real record of
  ticket 001 now lives, along with the team-lead's own verification.
- **There is no MCP escape hatch.** `write_artifact_frontmatter` writes
  frontmatter only; no tool writes ticket body content.

This contradicts the team-lead role contract, which requires acceptance criteria
to be checked off (`- [x]`) before a ticket is done, while also forbidding the
team-lead from writing ticket content -- with the programmer, the only agent
permitted to do it, structurally unable to.

**A second, wider symptom of the same design:** while a sprint holds the
execution lock and no ticket is `in-progress`, the team-lead cannot write
`clasi/issues/` files either -- even though the role contract names issues and
reflections as the team-lead's own direct-write scope. Writing this very issue
required first moving a ticket to `in-progress`. That gate is keyed on lock
state rather than on the target path, so it catches files it was not aimed at.

## Proposed fix

Options, roughly in order of preference:

1. **Have the dispatch layer set `CLASI_AGENT_NAME` per subagent.** The correct
   fix, and it makes the whole tier system work as designed rather than
   accidentally collapsing to team-lead.
2. **Resolve the agent from something the harness actually propagates** rather
   than an env var -- e.g. an explicit argument, or a session/agent id the hook
   can map. Removes the silent-default failure mode entirely.
3. **Fail closed instead of open.** With `CLASI_AGENT_NAME` unset the hook
   *assumes* team-lead. An explicit "unknown agent" state that says so would
   have made this a five-second diagnosis instead of a mid-sprint investigation.
4. **Add an MCP tool for ticket completion** (check criteria / append Completion
   Notes), so the recording path never depends on file-write permissions at all.
   Useful regardless of 1-3.
5. **Scope the lock-held gate by path**, so `clasi/issues/` stays writable by
   team-lead during execution (see the second symptom above).

**Do not** "fix" this by setting `CLASI_AGENT_NAME=programmer` globally in
`.claude/settings.json` -- that would make the main interactive session a
programmer and disable the team-lead guards that are working correctly.

**Do not** reach for `clasi oop on` as the routine workaround. It disables every
CLASI gate for the session, which is far broader than this defect warrants.

## Verification

- Dispatch a programmer agent against a ticket in `in-progress` and confirm it
  can check a criterion box and append Completion Notes.
- Confirm the team-lead session *still* cannot edit sprint artifacts directly
  (the guard must keep working for the role it was written for).
- Confirm a subagent with no CLASI role still cannot write sprint artifacts.
- Confirm team-lead can write `clasi/issues/` while a sprint holds the lock with
  no ticket in-progress.

## Related

- `clasi/hook_handlers.py` lines 302, 651, 980 -- the three
  `os.environ.get("CLASI_AGENT_NAME", ...)` sites.
- Sprint 136 ticket 001 -- first observed occurrence; its Completion Notes are
  missing for this reason and its evidence lives in commit `fc2869a2` and in the
  team-lead's independent verification (all nine acceptance criteria confirmed
  by direct test/inspection before the ticket was moved to done).
- The project memory note "bogus ROLE VIOLATION = stale tier rows" describes a
  *different* failure with the same error text; this one is an unset env var, not
  stale database rows. Neither is fixed by touching `.clasi/.clasi.db`.
