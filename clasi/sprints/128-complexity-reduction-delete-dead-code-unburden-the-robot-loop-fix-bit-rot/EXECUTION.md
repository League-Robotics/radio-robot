# Sprint 128 — Execution Handoff

Operational directions for whoever executes this sprint. Written by the
team-lead that planned it (2026-07-31), after setting up the execution
environment and measuring the baseline. **Read this before dispatching any
ticket.** Everything here was measured or verified, not assumed.

`sprint.md` is the plan (goals, architecture, use cases). This file is the
*how and where* of running it.

---

## 1. Where the work happens — a separate clone, not a worktree

```
/Volumes/Proj/proj/RobotProjects/radio-robot-elite-sprint128
```

on branch
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.

This is a **full independent clone** (stakeholder directive 2026-07-31),
created with `git clone` from the main checkout. `origin` points back at
`/Volumes/Proj/proj/RobotProjects/radio-robot-elite`, so merging later is a
normal fetch/merge. It began as a git worktree; the stakeholder deleted that
and asked for a separate repo instead.

**Do not write to `/Volumes/Proj/proj/RobotProjects/radio-robot-elite`.**
Another session is working there out-of-process with uncommitted changes
across five source files. Writing to it will corrupt their work. Every
programmer dispatch must be told the clone path explicitly and must verify
`git rev-parse --abbrev-ref HEAD` before editing anything — a subagent's
default project root is the *main* checkout, so this is easy to get wrong.

The stakeholder's words: "It's not a work tree. It's a whole separate repo.
[...] We'll worry about merging on the other side."

## 2. The CLASI database has forked — do not move tickets via MCP

`.clasi/.clasi.db` is a **tracked file**, so the clone carries its own copy.
That fork is what bought the isolation described in §3, but it has a
consequence:

> An MCP call made from a session rooted in the main checkout updates the
> **main checkout's** database, not the clone's. Ticket state would move in
> one repo while the code lands in the other.

**Therefore, for this sprint only:** programmers update ticket `status:`
frontmatter **as files in the clone**, committed alongside their code. Do not
call `update_ticket_status` / `move_ticket_to_done` from a session rooted in
the main checkout.

This is a deliberate, stakeholder-sanctioned deviation from the normal rule
that only an MCP call moves a ticket (`.claude/rules/source-code.md` §5). It
is recorded here so it is not mistaken for process sloppiness. **CLASI state
must be reconciled at merge time** — that is a real closing task, not
optional; `reconcile_worktrees` and a pass over every ticket's frontmatter
are the tools.

If instead you run the sprint from a session rooted *in the clone*, the MCP
server resolves to the clone's own database and normal MCP ticket handling
works. That is the cleaner option if available — prefer it.

## 3. Why the isolation matters: OOP is active in the main repo

At 2026-07-31 ~16:12 another session set a repo-global CLASI OOP bypass:

> "stakeholder: out of process — one-line firmware fix so the TestGUI sim
> tour runs (pid.kff clobbering App::Drive duty-per-speed)" — expires ~7h.

Per `.claude/rules/mcp-required.md`, an active OOP bypass tells agents to
**skip all CLASI process gates**. Dispatching sixteen tickets into that would
have silently bypassed the entire process. The clone's database snapshot
predates that toggle by ~4 minutes, so in the clone `clasi oop status`
reports **not active** and the sprint is properly gated.

**Verify this before starting** (`clasi oop status` from the clone). If OOP
ever shows active in the clone, stop and ask — something has leaked.

### Consequence for ticket 015

That out-of-process fix touches `pid.kff` clobbering `App::Drive`'s
duty-per-speed — **the same duty-stage code ticket 015 parks**. Before
dispatching 015, re-read its premise against whatever that fix actually
landed. Do not let the two silently disagree and discover it at merge.

## 4. The test baseline is RED — six known failures

Measured in the clone at `c7a955c2`, before any sprint changes:

```
6 failed, 1436 passed, 138 skipped, 1 xfailed in 213.32s (0:03:33)
```

```
FAILED src/tests/testgui/test_error_divergence.py::test_enc_scale_err_separates_encoder_trace_from_camera_truth
FAILED src/tests/testgui/test_otos_calibration_convergence.py::test_otos_calibration_push_converges_pose_via_the_real_config_path
FAILED src/tests/testgui/test_sim_errors_from_calibration.py::test_calibrated_robot_yields_inverse_calibration
FAILED src/tests/testgui/test_sim_errors_panel.py::TestSimErrorsPanelExistence::test_spin_boxes_populated_from_defaults
FAILED src/tests/testgui/test_sim_errors_panel.py::TestSimErrorsApplyButton::test_apply_saves_field_values
FAILED src/tests/testgui/test_sim_errors_panel.py::TestSimErrorsApplyButton::test_apply_calls_live_apply_on_connected_sim_transport
```

**All six share one root cause, and it is NOT a product regression.** The
tests build a `FakeConnectedSimTransport` and rename it to `SimTransport`
(`src/tests/testgui/test_sim_errors_panel.py:436-475`). The fake never grew a
`set_speed_factor`, but `testgui/__main__.py:2214` (`_apply_sim_speed`, called
on connect) now calls one. The real `SimTransport`
(`testgui/transport.py:2094`) **does** have the method — verified by import.
This is stale-test-double drift, pre-existing on committed master.

**Every ticket's bar is therefore: exactly those six failures and no others.**
A seventh failure belongs to the ticket that introduced it. Tell every
programmer the exact baseline, or they will waste a cycle debugging inherited
red.

**Open decision the stakeholder has not yet ruled on:** whether to repair the
six stale fakes as a small baseline-fix commit before starting. It is the same
class of bit rot this sprint exists to kill, and it sits squarely in the area
tickets 003/004/005/010/011 touch — which makes "tests pass" unusable as their
acceptance bar until it is fixed. Recommended: fix it first. Not done, because
it is scope the stakeholder did not ask for.

## 5. Environment — already prepared in the clone

- `uv sync --group gui` — **done**. The `gui` group is required; several
  tickets touch testgui and `test_gui_button_acceptance.py`.
- `just build` — **verified passing**, exit 0, producing `MICROBIT.hex` and
  `src/sim/build/libfirmware_host.dylib` at v0.20260731.3.
- `src/vendor` is a tracked symlink to an absolute path
  (`/Volumes/Proj/proj/league-projects/scratch/radio-robot/vendor`), so it
  resolves from the clone. There is no `libraries/` directory and none is
  needed — CODAL comes through that symlink.
- `active_agents` was cleared in the clone's database. A stale row (sprint
  127's planner, pointing at the *other* repo's log path) rode along in the
  clone. CLASI resolves agent tier with `SELECT tier FROM active_agents
  LIMIT 1` — no `WHERE agent_id` — so a ghost row applies its tier to every
  caller and throws bogus `ROLE VIOLATION` errors. If you hit an inexplicable
  role violation, inspect that table first.
- Expect a harmless `VIRTUAL_ENV ... does not match the project environment
  path` warning if a shell inherited the main checkout's venv var. `uv run`
  ignores it and uses the clone's `.venv`.

`just build` auto-bumps the version via a `version_bump` hook on **every**
build — that is the project's own behavior, not a manual bump, and does not
violate the once-per-sprint cadence rule in `.claude/rules/git-commits.md`.
It leaves `pyproject.toml` / `config/dotconfig.yaml` dirty; let it ride along
with the next commit.

## 6. Ticket ordering and the constraints that matter

Dependency-free, can start immediately: **001, 002, 003, 006, 007, 008, 012,
013**. Then 004 (needs 003), 005 (needs 004), 009 (needs 006/007/008), 010
(needs 003/004), 011 (needs 004), 014 (needs 012), 015 and 016 (need 014).

Run tickets **serially**. This is one repo, not per-ticket worktrees
(`worktree: false` in the sprint frontmatter is intentional — the stakeholder
asked for one sprint repo, not parallel per-ticket ones). Concurrent
programmers would race on commits.

Constraints that will bite if forgotten:

- **003 must land before 004.** `Transport.halt()` has to exist before
  `binary_bridge.py`'s dead half is deleted, or there is a window with no
  halt path at all. This is the safety-critical edge in the whole sprint.
- **Four build source lists.** Any ticket deleting a `.cpp` from `src/motion`
  (014, 015, 016) must update CMake ×2 **plus** the pytest `_APP_SOURCES`
  lists. Missing one is a **link** error, not a compile error — it surfaces
  late and looks unrelated.
- **Clean build after shared-header changes.** Tickets 012, 014, 016 touch
  `robot_state.h` / `telemetry.h`. Use `just build-clean`, never incremental.
  A stale incremental build welds mismatched objects and produces a boot
  HardFault that looks exactly like power loss.
- **014 and 016 both edit `robot_state.h` writer comments.** 016 must check
  what 014 did rather than clobber it.
- **No per-ticket hardware runs.** Bench verification is a **sprint-level**
  acceptance gate, run once after 012-016 land, per
  `.claude/rules/hardware-bench-testing.md`. Use
  `src/tests/bench/radio_bench_gate.py` and/or `move_protocol_bench.py`.

## 7. Settled decisions — do not reopen

Both were flagged at planning and approved by the stakeholder 2026-07-31.
Tickets 014 and 015 are written as plan-of-record, not as open options:

1. **firm↔motion boundary — option (a).** Promote
   `Types::RobotState::Wheel::cmdVelocity` to be the documented actuation
   boundary (it already is the real one) and delete the
   `WheelSink`/`MoveQueue`/`StopCondition`/`VelocityShaper` generation
   (~1,500 lines), rewriting the three design docs.
2. **Duty stage — PARK.** Stop calling `stageDuty()` from the live tick; keep
   the class and its ctest tiers as the validated basis for a future
   duty-sink cutover, with a `src/motion/DESIGN.md` note naming intent and
   owner. `WheelVelocityPid` is deleted either way (zero call sites).

Planner-level decisions made while ticketing, also settled: ticket 016 deletes
`StateEstimator` rather than parking it; ticket 002 does only the loud-gate
half of the `nav/` issue and leaves that issue open for a later
rebuild-or-delete sprint (`completes_issue: false`).

## 8. Scope is closed

Exactly the **18 issues** linked in `sprint.md`, all from `clasi/issues/`.
The stakeholder was explicit: **nothing from `clasi/issues/later/`**, and the
excluded set (the Nezha facade rebuild, the `io/calibrate` rewrite, the
testgui `__main__` decomposition, `main.cpp` constants, the square-tour system
test, and five others) stays excluded. This sprint adds **no new capability** —
every ticket deletes code, relocates responsibility to its rightful owner, or
fixes a defect the 2026-07-30 review found. If a ticket seems to want new
capability, that is a signal to stop and re-read, not to build it.
