---
id: '007'
title: Host protocol.py MOVE cutover
status: done
use-cases:
- SUC-050
- SUC-051
depends-on:
- '001'
github-issue: ''
issue:
- protocol-set-point-the-minimal-firmware-s-complete-command-surface.md
- gut-to-minimal-firmware-motion-stack-excision-move-protocol-minimal-telemetry.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host protocol.py MOVE cutover

## Description

Only needs ticket 001's regenerated `envelope_pb2` — independent of the
firmware-side tickets 002-006, so it can run any time after 001. Adds
`NezhaProtocol.move_twist(v_x, v_y, omega, stop=..., timeout=...,
replace=..., id=...)` / `move_wheels(v_left, v_right, stop=..., timeout=...,
replace=..., id=...)` builders per the new arm-21 `Move` shape; deletes
`NezhaProtocol.twist()` (its wire arm, `Twist`, is gone). Also folds in
the 115 handoff item: `_DRIVE_MODE_CHAR` is missing `DriveMode.VELOCITY`
(decodes as `"I"` while driving — noted in
`docs/bench-checklists/sprint-115-gut-s1.md`). And removes `sTimeout`/
`watchdog` from `config()`/`set_config()`'s curated key vocabulary
(`_ALL_SET_KEYS`) — `ConfigDelta.watchdog` is deleted by ticket 001, so
both builders' existing `envelope_pb2.ConfigDelta(watchdog=...)` call
sites would otherwise break at the protobuf level the moment `envelope_pb2`
regenerates.

Confirmed by grep against the live (non-`archive/`) tree: `.twist(` is
called from `protocol.py` itself, `test_twist_stop_ack_matcher.py`,
`test_rig_dev.py`, and bench scripts (`twist_drive.py`, `rig_dev.py`,
`rig_soak.py`, `velocity_step_response.py`); `sTimeout`/`watchdog` is
tested directly in `test_protocol_config.py` and
`test_protocol_binary_client.py`. Per the parent gut issue's own scope
note ("host motion/tour code stays in place, dormant/broken; only
bench-toolchain-forced edits land here"), this ticket fixes `protocol.py`
itself and its direct unit tests, plus whichever bench script ticket 010's
own gate actually invokes — everything else stays dormant/broken and is
explicitly listed as out-of-scope in this ticket's own completion notes,
not silently left broken without a record.

## Acceptance Criteria

- [x] `NezhaProtocol.move_twist(...)` and `move_wheels(...)` added,
      building `CommandEnvelope{move: Move{...}}` per the arm-21 shape;
      both fire-and-poll (matching `twist()`/`stop()`'s existing
      "telemetry-only return path" shape), returning the corr_id for
      `wait_for_ack()`.
- [x] `NezhaProtocol.twist()` deleted.
- [x] `_DRIVE_MODE_CHAR` gains `telemetry_pb2.VELOCITY: "V"` (or
      whichever character the implementer confirms doesn't collide with
      the existing I/S/T/D/G set).
- [x] `sTimeout`/`watchdog` removed from `_ALL_SET_KEYS`; `config()`/
      `set_config()` no longer build `ConfigDelta(watchdog=...)` — a
      `sTimeout=` kwarg now raises the same "unknown key" `ValueError`
      any other bogus key does.
- [x] `test_twist_stop_ack_matcher.py` rewritten (or split/renamed) to
      cover `move_twist()`/`move_wheels()` instead of `twist()`.
- [x] Watchdog-specific test cases deleted/updated:
      `test_config_watchdog_key_builds_correct_envelope`,
      `test_config_spanning_drivetrain_and_watchdog_raises_value_error`
      (`test_protocol_config.py`); `test_set_config_binary_watchdog_arm`,
      `test_set_config_watchdog_key_sends_binary_and_returns_applied`
      (`test_protocol_binary_client.py`).
- [x] Whichever bench script(s) ticket 010's own gate depends on for
      driving MOVEs (e.g. `twist_drive.py`) are ported to
      `move_twist`/`move_wheels`.
- [x] Every other live `.twist(`/`sTimeout` caller not covered above
      (`rig_dev.py`, `test_rig_dev.py`, `rig_soak.py`,
      `velocity_step_response.py`, and any TestGUI/host motion-code
      caller) is explicitly listed in this ticket's completion notes as
      left dormant/broken, per the parent gut issue's scope — not
      silently discovered later.
- [x] `src/host/robot_radio/DESIGN.md` updated in place: §2's `robot/`
      orientation-table row and §5's `Exposes` bullet for
      `robot.protocol.NezhaProtocol` reflect `move_twist()`/`move_wheels()`
      replacing `twist()` as the live wire-adapter surface. This doc does
      not ride the sprint's design overlay — same co-located-`DESIGN.md`
      collision noted on ticket 001's `messages/DESIGN.md` bullet — so it
      is edited directly on the canonical doc here; see sprint.md's Design
      overlay note.

## Completion Notes

**Python API shape decided (parent issue left `stop=...` as an ellipsis
for the implementer to pin):** `move_twist(v_x, v_y, omega, *, stop_time=
None, stop_distance=None, stop_angle=None, timeout, replace=True,
move_id=0)` / `move_wheels(v_left, v_right, *, stop_time=None,
stop_distance=None, stop_angle=None, timeout, replace=True, move_id=0)`.
`Move.stop` is itself a oneof (time/distance/angle, each a DIFFERENT
unit — ms/mm/rad), so it is exposed as three separate, mutually-exclusive
keyword-only args rather than one generic `stop=` value — this keeps each
arg's own `# [unit]` tag on its own parameter per
`.claude/rules/coding-standards.md`, instead of one parameter whose unit
depends on a second value. Exactly one of the three is required
(`ValueError`, no wire traffic, otherwise) — validated by a shared
`_build_move_stop_kwargs()` helper both builders call. `timeout` is
keyword-only with NO default (Python's own missing-arg error enforces
"required"); a non-positive value raises `ValueError` host-side before
any wire traffic, mirroring envelope.proto's own "`<=0` -> `ERR_BADARG`"
contract. `replace` defaults `True` (preempt-and-start-now) — matches
every existing single-shot caller's own pre-Move "just drive this" usage.
The wire field `Move.id` is exposed as the Python kwarg `move_id` (not
bare `id`) to avoid shadowing the `id()` builtin — the wire name itself is
unaffected (wire field names are excluded from the naming convention).
`_DRIVE_MODE_CHAR` gains `telemetry_pb2.VELOCITY: "V"` — unused by every
other entry (I/S/T/D/G), no collision.

**`_ALL_SET_KEYS`**: `sTimeout` is deleted (not renamed) — no live target
remains for a per-command deadman window since `App::Deadman` is gone
sprint-wide.

**Files touched:**
- `src/host/robot_radio/robot/protocol.py` — `move_twist()`/`move_wheels()`
  added (+ shared `_build_move_stop_kwargs()` helper); `twist()` deleted;
  `_DRIVE_MODE_CHAR` gains `VELOCITY: "V"`; `sTimeout`/`watchdog` removed
  from `_ALL_SET_KEYS` and both `set_config()`/`config()` bodies; every
  docstring mentioning `twist()`/the `twist`/`config`/`stop` oneof updated
  to `move_twist()`/`move_wheels()`/`move`/`config`/`stop`; the stale
  "Move -- DELETED" comment block (pre-this-ticket placeholder) removed
  now that Move has a live implementation.
- `src/tests/unit/test_twist_stop_ack_matcher.py` — section 1 rewritten:
  `test_move_twist_builds_correct_envelope_and_returns_corr_id`,
  `test_move_wheels_builds_correct_envelope_and_returns_corr_id`,
  `test_move_twist_stop_angle_variant_builds_correct_envelope`,
  `test_move_twist_requires_exactly_one_stop_condition`,
  `test_move_twist_requires_positive_timeout`,
  `test_move_and_stop_each_get_a_fresh_corr_id` added/renamed;
  `test_stop_builds_correct_envelope_and_returns_corr_id` kept unchanged.
  Sections 2/3 (TLMFrame flags-derived fields, `wait_for_ack()` adapter)
  untouched — not Move-related. Filename kept as-is (this ticket's own
  acceptance criteria list it by name); a docstring note explains the
  content no longer tests a `twist()` method.
- `src/tests/unit/test_protocol_binary_client.py` —
  `test_from_pb2_mode_mapping_matches_modechar`'s `VELOCITY` case updated
  `"I"` -> `"V"`; `test_set_config_binary_watchdog_arm` and
  `test_set_config_watchdog_key_sends_binary_and_returns_applied` deleted;
  `test_set_config_binary_not_connected_returns_none` and
  `test_set_config_spans_multiple_targets_sends_one_envelope_per_target`
  rewritten onto a non-watchdog `ConfigDelta`/kwarg pair (the latter now
  demonstrates drivetrain+motor fan-out instead of drivetrain+watchdog).
- `src/tests/unit/test_protocol_config.py` —
  `test_config_watchdog_key_builds_correct_envelope` and
  `test_config_spanning_drivetrain_and_watchdog_raises_value_error`
  deleted; `test_config_each_call_gets_a_fresh_corr_id` and
  `test_config_invalid_call_sends_nothing` rewritten off `sTimeout=` onto
  still-valid keys.
- `src/tests/testgui/test_binary_bridge.py` —
  `test_command_oneof_no_longer_has_drive_segment_replace`'s assertion set
  updated `{config, stop, twist}` -> `{config, stop, move}`;
  `test_outbound_command_renders_readable_text_not_raw_armor` rebuilt from
  `cmd.move.twist.*` instead of the now-nonexistent `cmd.twist.*`; header
  docstring and one inline comment updated to match.
- `src/tests/bench/twist_drive.py` — ported from `proto.twist(v_x, omega,
  duration)` to `proto.move_twist(v_x, v_y=0.0, omega, stop_time=
  args.duration, timeout=<duration+500ms margin, new --timeout flag>,
  replace=True)`; docstring/CLI help/result labels updated to match. This
  is the bench script ticket 010's own gate depends on for driving MOVEs.
- `src/host/robot_radio/DESIGN.md` — §2's shared "Live" legend (directly
  above the table, defining the term the `robot/` row's own status relies
  on) updated `twist/stop/config` -> `move/config/stop`; §2's `robot/` row
  gains a clause noting `move_twist()`/`move_wheels()` replace `twist()`;
  §5's `Exposes` bullet for `robot.protocol.NezhaProtocol` updated the
  same way. No other section touched (out of this ticket's pinned scope —
  §4/§6's broader "Sprint 116 not yet executed" framing is stale but
  belongs to a future consolidation pass, not this ticket).

**Left dormant/broken (confirmed unexercised by any test — full suite
stayed green with zero changes to these files):**
- `src/tests/bench/rig_dev.py` (`Rig.twist()`/`Rig.config(sTimeout=...)`),
  `src/tests/unit/test_rig_dev.py` (tests `Rig.twist()` against a fake
  `proto`, never the real `NezhaProtocol` — stays green either way, but
  exercises now-dormant production code),
  `src/tests/bench/rig_soak.py`, `src/tests/bench/velocity_step_response.py`
  — all call `proto.twist(...)`/`rig.twist(...)` directly; explicitly named
  by this ticket's own scope note as left for a future ticket.
- `src/host/robot_radio/planner/executor.py`'s `StreamingExecutor.tick()`
  (`self._transport.twist(v_x, omega, duration)`) — `planner/` stays
  dormant by the sprint-115 stakeholder decision this sprint's own
  architecture-update.md reaffirms ("116's host-side scope was limited to
  `protocol.py`'s low-level `move_twist()`/`move_wheels()` builders;
  reviving the higher-level tour/nav machinery... is explicit future
  work, not part of 116").
- `src/host/robot_radio/io/repl.py` — `verb_twist`/`verb_drive`/`verb_turn`
  (`session.proto.twist(...)`) and `verb_raw`'s `raw twist ...` builder
  (`envelope_pb2.CommandEnvelope(twist=envelope_pb2.Twist(...))`, which
  now fails at the pb2 level the moment that verb is invoked) — a "host
  motion-code caller," per this ticket's own scope note; no test imports
  `io.repl`.
- `src/host/robot_radio/nav/camera_goto.py` — `proto.set_config(sTimeout=
  ...)` (two call sites) — `nav/` is already dormant (calls `Robot.go_to()`,
  itself dead); `sTimeout=` now silently no-ops via `set_config()`'s
  existing "unknown key -> None, no wire traffic" contract rather than
  configuring a watchdog, consistent with the rest of this dormant
  surface's existing failure mode.
- `src/tests/playfield/pose_fix_convergence.py`,
  `src/tests/playfield/world_goto_chart.py` — playfield HITL scripts using
  `sTimeout`/`SET sTimeout=...`; not pytest-collected, out of scope.
- `src/host/robot_radio/testgui/transport.py:1481`
  (`self._loop.twist(...)`) is NOT a `NezhaProtocol.twist()` call — `self.
  _loop` is `io.sim_loop.SimLoop`, an unrelated class with its own,
  unaffected `twist()` method (confirmed: the whole `testgui`/`sim` test
  suite, which exercises `SimLoop.twist()` extensively, stayed green with
  zero changes).

**Verification:** `uv run python -m pytest` (full suite, `testpaths =
src/tests/sim, src/tests/unit, src/tests/testgui`): 1192 passed, 13
skipped, 10 xfailed, 1 xpassed, 4 failed — the 4 failures are exactly
ticket 008's sim-system residuals (`test_fault_knobs`,
`test_scripted_twist_demo`, `test_sim_api`, `test_straight_twist`, all
failing on a missing `src/firm/app/deadman.cpp` — unrelated to this
ticket, owned by 008). Before this ticket: 1182 passed / 14 failed (the
same 4 plus the 10 this ticket fixes). `python build.py` builds firmware
+ host sim lib clean (no pb2 diff — 001 already regenerated it).

## Testing

- **Existing tests to run**: `src/tests/unit/test_protocol_config.py`,
  `test_protocol_binary_client.py`, `test_twist_stop_ack_matcher.py`
  (post-rewrite).
- **New tests to write**: `move_twist()`/`move_wheels()` envelope-building
  tests mirroring the deleted `twist()` tests' shape (correct fields,
  fresh corr_id per call); a `_DRIVE_MODE_CHAR` regression test asserting
  `VELOCITY` no longer falls back to `"I"`.
- **Verification command**: `uv run python -m pytest src/tests/unit/`
