---
id: 009
title: Final regression gate -- full pytest, gopiv bench verification, tovez restore
status: done
use-cases:
- SUC-005
depends-on:
- 008
github-issue: ''
issue:
- firmware-layering-cleanup-interfaces-to-hal-impls-to-platform-microbit.md
- main-cpp-holds-code-that-does-not-belong-in-main.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Final regression gate -- full pytest, gopiv bench verification, tovez restore

## Description

The sprint's closing ticket — the one deliberately-last "breaking step"
(sprint-end-must-be-testable: mid-sprint may break, the sprint ENDS
runnable). Every prior Half B ticket used a fast smoke test only
(`just build-sim` + `test_layer_isolation.py`); this ticket runs the one
full regression pass the Test Strategy reserves for sprint end, plus the
standing hardware bench gate this sprint's own scope requires (it touches
the HAL and the command transport — `.claude/rules/hardware-bench-
testing.md`'s "Standing verification gate" applies).

**`gopiv` is the only robot on this session's hub** — `tovez` is not
locally connected. Address `gopiv` by UID, never by port number (ports
move on every re-enumeration): `9906360200052820049d38a46da36a83000000
006e052820`. `gopiv` has **no OTOS, no line sensor, no colour sensor**
(telemetry `flags` word 216) — do not gate on those three; encoders,
pose, and the MOVE protocol work normally on it.

**Building for `gopiv` rewrites tracked files.** `build.py` regenerates
`src/firm/config/boot_config.cpp` from whatever
`data/robots/active_robot.json` points at, and that pointer file is
itself tracked. This ticket MUST restore both to `tovez` before the
sprint can close — an un-restored pointer silently ships the next
session a tree that boots `tovez` hardware from `gopiv`'s baked config.

## Acceptance Criteria

- [ ] `uv run python3 build.py --clean` succeeds against whichever robot
      is active before this ticket touches anything (the tree's own
      resting state, confirmed clean first).
- [ ] Full `uv run python -m pytest` across `src/tests/{unit,sim,testgui}`
      run fresh; the failure set matches ticket 002's recorded baseline
      exactly (same strict-xfail entries, zero new failures). Any
      deviation is investigated and either fixed (if attributable to this
      sprint's own work) or escalated (if it looks like a planning-time
      miss) — never silently accepted.
- [ ] `data/robots/active_robot.json` pointed at `gopiv.json`; firmware
      built and flashed by UID `9906360200052820049d38a46da36a83000000
      006e052820` (never by port number — confirm the row in `mbdeploy
      list` says `gopiv` before flashing).
- [ ] ~5s sleep after flashing before the first command (boot races the
      first command otherwise).
- [ ] `src/tests/bench/twist_drive.py` run against `gopiv`: banner on
      connect, `HELLO`/`PING`/`ID`/`VER`, wheel drive both directions with
      climbing encoders. OTOS/line/colour checks skipped — `gopiv` has
      none.
- [ ] `src/tests/bench/move_protocol_bench.py` run against `gopiv`:
      enqueue+completion acks via the ack ring, round-trip over direct
      USB.
- [ ] Boot-display sequence bench-confirmed (heart → digits → heart-
      stays-lit → dark at loop start) — this is ticket 005's `main.cpp`
      relocation getting its actual hardware confirmation, deferred here
      per that ticket's own note.
- [ ] `DEVICE:NEZHA2:...` banner and `ID:...` line confirmed byte-
      identical to pre-sprint behavior, over the real link.
- [ ] `data/robots/active_robot.json` restored to `tovez.json`;
      `src/firm/config/boot_config.cpp` rebuilt from it
      (`uv run python3 build.py` against `tovez`) — its own explicit step,
      not left dangling after the `gopiv` bench session.
- [ ] `git diff`/`git status` confirms no stray `gopiv`-pointed
      `active_robot.json` or `boot_config.cpp` remains staged or
      committed.
- [ ] Every checkbox in `sprint.md`'s Success Criteria is satisfied; this
      ticket's Completion Notes cross-reference each one with its
      evidence (command run, output observed).

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/unit
  src/tests/sim src/tests/testgui` (full suite), `uv run python3 build.py
  --clean` (ARM + host), `src/tests/bench/twist_drive.py`,
  `src/tests/bench/move_protocol_bench.py` (both against `gopiv` by UID).
- **New tests to write**: none — this ticket verifies, it does not add
  coverage.
- **Verification command**: `uv run python -m pytest src/tests/unit
  src/tests/sim src/tests/testgui` plus the two bench scripts named above.
