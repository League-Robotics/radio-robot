---
id: '002'
title: 'nav/ goto stack: loud front-door gate (Step 1 only)'
status: in-progress
use-cases:
- SUC-007
depends-on: []
github-issue: ''
issue: nav-goto-stack-is-dead-gate-it-loudly-then-rebuild-or-delete.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# nav/ goto stack: loud front-door gate (Step 1 only)

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

**Scope note**: this ticket is Step 1 of the source issue's two-step
plan only — the loud gate. Step 2 (delete `nav/camera_goto.py`/
`navigator.py` outright and repoint `rogo goto`/`turnto` and the MCP
tools at `pathplan`) is explicitly deferred to a future sprint, which is
why `completes_issue` is `false`: this ticket does not close the issue,
it only lands the P0 risk-reduction half of it.

## Description

`nav/camera_goto.py:149,295` calls `proto.drive(...)`;
`nav/navigator.py:211` calls `self._robot.go_to(...)` — both deleted from
the wire surface. `rogo goto`/`rogo turnto` and the MCP tools
`navigate`/`follow_path`/`visit_tags` (`io/robot_mcp.py:229,271`) crash
with an `AttributeError` three call-frames deep, mid-loop. This ticket
makes the dead surface fail at the front door instead.

## Acceptance Criteria

- [x] `go_to_world_camera`, `Navigator.navigate`, `Navigator.follow_path`,
      and `visit_tags` each raise a `NotImplementedError` at the top of
      the function, naming the replacement
      (`pathplan.gotoWorld` / `pathplan.followPath`, sprint 127) and this
      tracking issue filename, e.g.:
      ```python
      raise NotImplementedError(
          "dead since the v5 wire cutover: NezhaProtocol.drive()/go_to() "
          "were deleted (104-002). The replacement is pathplan.gotoWorld/"
          "followPath (sprint 127). Tracked: clasi/issues/"
          "nav-goto-stack-is-dead-gate-it-loudly-then-rebuild-or-delete.md")
      ```
- [x] The MCP tools (`navigate`, `follow_path`, `visit_tags` in
      `io/robot_mcp.py`) catch the `NotImplementedError` and return an
      honest "not available" result rather than propagating a stack
      trace to the LLM operator.
- [x] `rogo goto 30 10` prints the message immediately with no traceback
      mid-drive.
- [x] A unit test asserts both the CLI-path and the MCP-path behavior.
- [x] Nothing about the rebuild-or-delete decision for `nav/` is resolved
      by this ticket — `nav/pose.py` (live, imported by several genuinely
      live modules) is untouched; `nav/camera_goto.py`/`navigator.py`
      themselves are NOT deleted this ticket, only gated.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite —
  confirm nothing currently depends on `camera_goto`/`navigator` actually
  executing their old bodies).
- **New tests to write**: a unit test invoking `go_to_world_camera`/
  `Navigator.navigate`/`follow_path`/`visit_tags` directly, asserting each
  raises `NotImplementedError` with the expected message; an MCP-layer
  test asserting `navigate`/`follow_path`/`visit_tags` return an honest
  error result (not an unhandled exception) when called.
- **Verification command**: `uv run python -m pytest src/tests/unit -k nav -q`,
  then `rogo goto 30 10` against a connected (or disconnected) robot,
  confirming the `NotImplementedError` message prints immediately.

## Implementation Notes

- **Approach**: this is a small, standalone, low-risk commit — add the
  guard clauses and the MCP catch, nothing else. Do not attempt the
  rebuild or deletion half; if you find yourself editing more than the
  guard clauses and the MCP catch/report path, stop — that's Step 2's
  scope, not this ticket's.
- **Files to modify**: `src/host/robot_radio/nav/camera_goto.py`,
  `src/host/robot_radio/nav/navigator.py`, `src/host/robot_radio/io/robot_mcp.py`.
- **Documentation updates**: none — `robot_radio/DESIGN.md` already
  correctly marks `nav/`'s goto stack as dormant; no row changes needed
  for a gate that doesn't change dormancy status.
