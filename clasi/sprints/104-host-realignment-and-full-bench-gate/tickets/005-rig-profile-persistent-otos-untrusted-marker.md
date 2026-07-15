---
id: '005'
title: "Rig profile — persistent OTOS-untrusted marker"
status: open
use-cases:
- SUC-015
depends-on: []
github-issue: ''
issue: rig-persistent-otos-distrust.md
completes_issue:
  rig-persistent-otos-distrust.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Rig profile — persistent OTOS-untrusted marker

## Description

`clasi/issues/rig-persistent-otos-distrust.md`: the bench rig's OTOS is
servo-mounted and mechanically decoupled from the wheels. Under the
pre-single-loop architecture this required a per-session manual `SET
ekfROtosTheta=1e9 ekfROtosXy=1e9` ritual to stop a poisoned fused pose
from blocking motion (forgetting it silently reproduced "segments
admitted/ACKed but never executed"). Under the single-loop architecture
the robot no longer fuses pose on-robot at all — 103 ticket 010's own
bench session drove the rig cleanly with NO manual SET, first-hand
evidence the failure mode's root cause (an on-robot EKF gating on garbage
pose) is structurally gone for this firmware.

What remains, per the issue's own text ("the per-robot 'this OTOS does
not track the wheels' fact still belongs in the robot profile either
way — host-side fusion must know to ignore it on the rig too") and
architecture-update.md Decision 3, is future-proofing: persist the fact
now, inert, so sprint 106's host-side fusion has an authoritative source
from day one — without building fusion logic that doesn't exist yet
(explicitly rejected as speculative generality).

No dependency on other tickets — this is a standalone data/config change
plus a rig re-verification.

## Acceptance Criteria

- [ ] A persistent field exists in the rig's robot profile (`tovez_nocal.
      json` — the profile 103-010's session actually used — or a
      dedicated rig profile if ticket-time investigation finds that
      cleaner; a naming/location decision, not pre-decided in the
      architecture doc) marking OTOS as mechanically decoupled/untrusted,
      with a doc comment explaining why (servo-mounted, decoupled from the
      wheels — not a runtime tuning value).
  - [ ] Field schema location resolved: `CalibrationConfig`, a new
        `PeripheralsConfig` sub-field, or a standalone top-level key in
        `host/robot_radio/config/robot_config.py`'s `RobotConfig`
        pydantic model — pick one and document why in this ticket's
        completion notes (architecture-update.md Step 7 Open Question 1).
- [ ] Re-verified on the actual rig: reboot, drive a `twist` with NO
      manual `SET`, motion executes and reported (encoder) pose tracks —
      confirms 103-010's finding holds on this sprint's tree too, per
      `.claude/rules/hardware-bench-testing.md` (robot on the stand,
      wheels off the ground).
- [ ] `clasi/issues/rig-persistent-otos-distrust.md` is updated to reflect
      the single-loop architecture's actual resolution of the root failure
      mode (on-robot fusion no longer exists, so the original "segments
      never execute" failure mode is structurally gone), with the
      persisted flag noted as the remaining forward-looking piece for
      sprint 106. Do not close the issue outright — it stays open,
      `completes_issue: false` on this ticket, because full resolution
      (a host-side consumer that reads and honors the flag) is sprint
      106's scope, not this ticket's.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/unit -k
  robot_config` (confirm the pydantic model change doesn't break existing
  config-loading tests).
- **New tests to write**: a unit test confirming the new field
  round-trips through `RobotConfig` (load/dump) for a profile that sets
  it and one that omits it (default behavior).
- **Verification command**: `uv run python -m pytest
  tests/unit/test_robot_config.py -v` (or wherever `RobotConfig` tests
  live currently).

## Implementation Plan

**Approach**: Read `host/robot_radio/config/robot_config.py`'s existing
`CalibrationConfig`/`PeripheralsConfig` structure first (already has
`otos_angular_scale`/`otos_linear_scale` precedent in
`CalibrationConfig`) to decide the least-surprising home for the new
field, favoring co-location with the existing OTOS-related fields unless
a clear reason argues otherwise. Then the rig re-verification is a real
bench session, not a code change — schedule it after the profile field
lands.

**Files to create/modify**:
- `host/robot_radio/config/robot_config.py` — new optional field.
- `data/robots/tovez_nocal.json` (or the chosen rig profile) — set the
  field.
- `data/robots/robot_config.schema.json` — regenerate/update if this
  project auto-generates the JSON schema from the pydantic model (check
  how existing fields keep the two in sync).
- `clasi/issues/rig-persistent-otos-distrust.md` — update per Acceptance
  Criteria.

**Testing plan**: covered above; the bench re-verification is manual, per
`.claude/rules/hardware-bench-testing.md`.

**Documentation updates**: this ticket's own completion notes should
record the bench re-verification's evidence (encoder motion observed,
no manual SET issued) with the same level of detail 103-010 used, since
this IS this ticket's evidence the issue's root cause is resolved.

## SUC-015: Rig profile — persistent OTOS-untrusted marker

Parent: `rig-persistent-otos-distrust.md`.

- **Actor**: Bench operator; future host-fusion code (sprint 106+).
- **Preconditions**: On-robot fusion removed by 102/103; 103-010 drove
  the rig with no manual SET.
- **Main Flow**: Persist the marker; re-verify on the rig.
- **Postconditions**: The "this OTOS does not track the wheels" fact is
  version-controlled, not tribal knowledge.
- **Acceptance Criteria**: see above.
