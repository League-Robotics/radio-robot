---
id: '004'
title: OTOS health on the wire
status: open
use-cases:
- SUC-004
- SUC-005
depends-on:
- '003'
github-issue: ''
issue: otos-not-used-frozen-pose-ekf-rejects-everything.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# OTOS health on the wire

## Description

The issue's acceptance sketch asks that "a persistent OTOS read failure or
fusion block is surfaced on the wire ... not silent" and that TLM `otos=`'s
actual semantics ("raw read vs last-accepted") be determined. Today the only
wire-visible symptom of a fusion block is an indirectly-inferred one: a
climbing `ekf_rej=` counter combined with a suspiciously static `otos=`
value — exactly the pattern that made this issue hard to diagnose in the
first place. Tickets 001-003 make the fusion gate itself correct (bench
parity, live indirection, stuck-value detection); this ticket makes its
STATE observable per-frame, independent of `ekf_rej`.

Add one new, additive, unconditional TLM field: `otos_health=<status>,
<blocked>` — the raw OTOS STATUS byte and the current
`Drive::_otosFusionBlocked` state. Follows the EXACT precedent
`wedge=<L>,<R>` already established (`RobotTelemetry.cpp:81-92`,
064-004): unconditional on freshness (no N8 gate), because the health field
must stay visible precisely when `otos=` itself is going stale — gating it
the same way `otos=` is gated would hide the signal exactly when it matters
most. `TLM_FIELD_ALL` widens by one bit (`0x1FF` → `0x3FF`) since the field
is on by default like every other TLM field added to this project.

This also closes the issue's fourth investigation pointer: `otos=` itself is
documented as the raw, last-successfully-read pose, independent of fusion
gate state (it does not go stale or change meaning when
`_otosFusionBlocked` is true) — and a regression test proves the read-
failure path actually clears its freshness envelope rather than repeating a
stale value forever.

See `architecture-update.md` Step 3 "Module: OTOS health telemetry", Step
4b (TLM frame schema diagram), Step 5 item 5, Design Rationale Decision 4
(why one combined unconditional clause, not an EVT or a freshness-gated
field); `usecases.md` SUC-004, SUC-005.

## Acceptance Criteria

- [ ] `protos/drivetrain.proto`'s `DrivetrainState` message
      (`protos/drivetrain.proto:44-51`) gains two new fields at the next
      free field numbers (10, 11): `uint32 otos_status = 10;` (raw STATUS
      byte) and `bool otos_fusion_blocked = 11;`. `python3
      scripts/gen_messages.py` is re-run to regenerate
      `source/messages/drivetrain.h`, adding `uint32_t otos_status = 0;` and
      `bool otos_fusion_blocked = false;` members (plus their `get_*()`
      getters, matching this file's existing generated pattern).
- [ ] `Drive` (`Drive.h`, near `_otosFusionBlocked`) gains `uint8_t
      _lastOtosStatus = 0;`, updated in STEP 5's `poseOk` branch right after
      `_hal.otos().readStatus(otosStatus)` succeeds (`Drive.cpp:159-160`,
      after ticket 003's changes land there) — `_lastOtosStatus =
      otosStatus;`. On a read failure, `_lastOtosStatus` is left UNCHANGED
      (same "preserve last-known-good" convention as `_hw.otos.valid` and
      ticket 003's `_prevOtosValid`).
- [ ] `Drive::tickUpdate()`'s STEP 6 (`Drive.cpp:186-233`, the `_hw` → `_state`
      copy) gains: `_state.otos_status = _lastOtosStatus;` and
      `_state.otos_fusion_blocked = _otosFusionBlocked;`, alongside the
      existing `_state.otos.lag/last_upd/valid` assignments.
- [ ] `Config.h` gains `constexpr uint16_t TLM_FIELD_OTOS_HEALTH = (1u <<
      9);` immediately after `TLM_FIELD_ENCPOSE` (bit 8); `TLM_FIELD_ALL`
      widens from `0x1FFu` to `0x3FFu`.
- [ ] `RobotTelemetry.cpp`'s `buildTlmFrame` emits `otos_health=<status>,
      <blocked>` UNCONDITIONALLY once `config.tlmFields & TLM_FIELD_OTOS_HEALTH`
      is set (no freshness/staleness gate, matching `wedge=`'s precedent at
      lines 81-92) — placed immediately after the existing `ekf_rej=` clause
      (the current last field in the emission order), preserving the
      append-only field-ordering convention this file already documents.
      `<status>` is `ds.otos_status` (integer); `<blocked>` is
      `ds.otos_fusion_blocked` (0 or 1).
- [ ] A code comment is added at the existing `otos=` emission site
      (`RobotTelemetry.cpp:134-148`) AND at `Drive::tickUpdate()` STEP 5
      (`Drive.cpp`, near the `poseOk`/`else` branches) stating plainly: `otos=`
      reflects the most recent RAW, successfully-read pose from whichever
      odometer is active, independent of whether that reading was admitted
      into EKF fusion — it does NOT go stale or change meaning when
      `_otosFusionBlocked` is true; `otos_health=` is what tells a host
      fusion is blocked.
- [ ] `host/robot_radio/robot/protocol.py`'s `TLMFrame` dataclass gains
      `otos_health: tuple[int, bool] | None = None` (with a docstring entry
      matching the style of the existing `wedge`/`encpose` entries), and
      `parse_tlm()` gains a clause parsing `otos_health=<status>,<blocked>`
      into that tuple (mirroring the existing `wedge`/`ekf_rej` parse
      clauses' try/except-ValueError shape).
- [ ] `tests/_infra/golden_tlm_capture.json` is regenerated using the EXACT
      documented recipe in `tests/simulation/unit/test_golden_tlm.py`'s
      module docstring (`s = Sim(); ... json.dumps(frames, indent=2)`),
      done in the SAME commit as the firmware and host-parser changes (the
      sprint's hard contract: golden-TLM additive + lock-step). Every one of
      the 15 captured frames gains an `otos_health=0,0` clause (the fixed
      command sequence never triggers a fusion block or a nonzero STATUS
      byte, and the field is unconditional/on-by-default).
      `test_golden_tlm.py` passes unmodified against the regenerated
      capture.
- [ ] New sim test: drive the fusion gate into the blocked state (reusing
      ticket 003's stuck-value injection OR the existing STATUS-bit
      `sim.set_otos_warn(True)` injection — either is a valid trigger for
      this field), enable `STREAM`, and assert a captured TLM frame's
      `otos_health=` reflects `blocked=1` while blocked and `blocked=0`
      once re-admitted, using `host/robot_radio/robot/protocol.py`'s
      `parse_tlm()` to decode the frame (not string-matching).
- [ ] New regression test for SUC-005: `sim.set_otos_read_failure(True)` (or
      the sim's existing equivalent hook), tick past `2 * lagMs` (the
      existing N8 freshness window `otos=` already uses,
      `RobotTelemetry.cpp:140-144`), and assert `otos=` is ABSENT from the
      next TLM frame (not present with a stale value) — confirming no
      stale-cache-masks-a-read-failure defect exists in the raw path,
      independent of `otos_health=`'s own (unconditional) presence in the
      same frame.
- [ ] No existing `TLM_FIELD_*` bit, wire key, or field ordering changes
      meaning or position — purely additive at the end of the sequence.
- [ ] Full suite (`uv run python -m pytest`) passes at the running baseline
      (2672 + tickets 001-003's net additions) + this ticket's net new test
      count, zero unexplained failures. The `data/robots` drift noted in
      the sprint's hard contract is environmental — do not chase or touch
      it.

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_golden_tlm.py`
  (against the regenerated capture), `test_otos_warn_persistence.py` and
  ticket 003's new stuck-value test (confirm the new field's `blocked`
  value tracks `_otosFusionBlocked` correctly in both trigger scenarios),
  full suite.
- **New tests to write**: the blocked-state wire-visibility test and the
  read-failure/freshness regression test described above, likely both in a
  new file (e.g. `tests/simulation/unit/test_otos_health_tlm.py`) or
  appended to an existing OTOS-telemetry-focused test file if one already
  groups `otos=`-related TLM assertions.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Regenerate the proto-derived header first (`gen_messages.py`)
so the two new `msg::DrivetrainState` fields exist before wiring anything to
them. Wire `Drive`'s STEP 5/STEP 6 plumbing (`_lastOtosStatus` capture +
`_state` copy). Add the `Config.h` bit and `RobotTelemetry.cpp`'s emission
clause. Update the host parser. Write the two new sim tests. Regenerate
`golden_tlm_capture.json` LAST, in the same commit, only once the firmware
and host sides both compile and the new field's shape is final — regenerating
early risks locking in a wire format that still needs adjustment.

**Files to create/modify**:
- `protos/drivetrain.proto` — two new `DrivetrainState` fields.
- `source/messages/drivetrain.h` — regenerated via `scripts/gen_messages.py`
  (do not hand-edit; verify the diff is exactly the two new fields + getters).
- `source/subsystems/drive/Drive.h` — `_lastOtosStatus` member.
- `source/subsystems/drive/Drive.cpp` — STEP 5 capture, STEP 6 copy.
- `source/types/Config.h` — `TLM_FIELD_OTOS_HEALTH`, widened `TLM_FIELD_ALL`.
- `source/robot/RobotTelemetry.cpp` — new emission clause + `otos=` comment.
- `host/robot_radio/robot/protocol.py` — `TLMFrame.otos_health` +
  `parse_tlm()` clause.
- `tests/_infra/golden_tlm_capture.json` — regenerated (documented recipe
  in `test_golden_tlm.py`'s module docstring).
- New test file for the two new sim tests.

**Testing plan**: run `test_golden_tlm.py` immediately after regenerating
the capture (fast feedback that the regeneration matches what the current
firmware+host actually produce); run the two new tests; run
`test_otos_warn_persistence.py` and ticket 003's test by name to confirm
the new field's value is consistent with each gate-trigger path; then the
full suite.

**Documentation updates**: `RobotTelemetry.cpp`'s `otos=` comment and
`Drive.cpp`'s STEP 5 comment (SUC-005's documentation requirement, see
Acceptance Criteria); `protos/drivetrain.proto`'s `DrivetrainState` message
comment if the codegen convention expects one (check sibling field
comments, e.g. `ValueSet otos = 7; // OTOS freshness`, and match the style
for the two new fields).
