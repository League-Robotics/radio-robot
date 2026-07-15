---
id: '005'
title: "Rig profile \u2014 persistent OTOS-untrusted marker"
status: done
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

- [x] A persistent field exists in the rig's robot profile (`tovez_nocal.
      json` — the profile 103-010's session actually used — or a
      dedicated rig profile if ticket-time investigation finds that
      cleaner; a naming/location decision, not pre-decided in the
      architecture doc) marking OTOS as mechanically decoupled/untrusted,
      with a doc comment explaining why (servo-mounted, decoupled from the
      wheels — not a runtime tuning value).
  - [x] Field schema location resolved: `CalibrationConfig`, a new
        `PeripheralsConfig` sub-field, or a standalone top-level key in
        `host/robot_radio/config/robot_config.py`'s `RobotConfig`
        pydantic model — pick one and document why in this ticket's
        completion notes (architecture-update.md Step 7 Open Question 1).
- [x] Re-verified on the actual rig: reboot, drive a `twist` with NO
      manual `SET`, motion executes and reported (encoder) pose tracks —
      confirms 103-010's finding holds on this sprint's tree too, per
      `.claude/rules/hardware-bench-testing.md` (robot on the stand,
      wheels off the ground).
- [x] `clasi/issues/rig-persistent-otos-distrust.md` is updated to reflect
      the single-loop architecture's actual resolution of the root failure
      mode (on-robot fusion no longer exists, so the original "segments
      never execute" failure mode is structurally gone), with the
      persisted flag noted as the remaining forward-looking piece for
      sprint 106. Do not close the issue outright — it stays open,
      `completes_issue: false` on this ticket, because full resolution
      (a host-side consumer that reads and honors the flag) is sprint
      106's scope, not this ticket's.

## Completion Notes (2026-07-15)

**Field name/shape**: `geometry.otos_untrusted: bool = False` — added to
`GeometryConfig` in `host/robot_radio/config/robot_config.py`, right next to
the existing `odometry_chip_upside_down` field. **Schema location decision**:
chose `GeometryConfig`, not `CalibrationConfig`/`PeripheralsConfig`/a
standalone top-level key. `odometry_chip_upside_down` is the closest existing
precedent — both are boolean facts about the OTOS chip's *physical mounting*
(construction-time HAL wiring truths), not tuning numbers. `CalibrationConfig`
holds measured scale/gain/offset values (`otos_angular_scale`, `rotation_gain`,
...) — a boolean "is this even trustworthy" fact is categorically different
from a calibration coefficient, and the ticket's own field doc comment says
explicitly "not a runtime tuning value," which argues against co-locating with
tuning fields just because they happen to also mention OTOS.
`config_sync_allowlist.json`'s existing `geometry.odometry_chip_upside_down`
entry ("construction-time HAL wiring flag") independently confirms this is the
established category for this kind of fact. Added a matching
`geometry.otos_untrusted` allowlist entry (host-side-only, no wire config verb
— `check_config_sync.py` passes clean) and a JSON-schema property in
`data/robots/robot_config.schema.json`.

**Profile-location correction from the ticket text**: the ticket suggested
`tovez_nocal.json` as "the profile 103-010's session actually used." Checked
this at ticket time and found it's not quite right: `git log -- data/robots/
active_robot.json` shows commit `dfb3e3bb` (sprint 093) switched the active
pointer from `tovez_nocal.json` to `tovez.json` for good ("was the
tovez_nocal.json 0.487 placeholder that corrupted every distance/velocity
reading") and it has stayed on `tovez.json` ever since. Separately,
`match_robot_by_id()`'s own documented exact-robot-name-match preference means
a real robot announcing `name=tovez` resolves to `tovez.json`
(`identity.robot_name == "tovez"`, exact match) over `tovez_nocal.json`
(`identity.robot_name == "tovez nocal"`, not exact) even without the pointer.
`tests/bench/twist_drive.py` (used for the re-verification below) doesn't load
either config at all — it's transport-only. So the profile actually in force
on the connected rig today is `tovez.json`, not `tovez_nocal.json`. Set
`otos_untrusted: true` in **both** `tovez.json` (the active one) and
`tovez_nocal.json` (same physical hardware — `connection.serial_last_6`
`f137c0` in both — so the mounting fact survives if the pointer is ever
switched back).

**Bench re-verification** (2026-07-15, robot UID
`9906360200052820a8fdb5e413abb276000000006e052820` at
`/dev/cu.usbmodem2121102`, on the stand, wheels off the ground per
`.claude/rules/hardware-bench-testing.md`; confirmed via `mbdeploy list`
immediately before the session, role `NEZHA2`/`robot`, same UID as 103-010's
session — same already-flashed single-loop firmware, no rebuild/reflash
needed since this ticket is host-config-only):

1. Rebooted via a non-destructive hardware reset (no reflash):
   `pyocd commander -t nrf52833 -u 9906360200052820a8fdb5e413abb276000000006e052820 -c "reset"`
   → `Resetting target`.
2. `uv run python tests/bench/twist_drive.py --port /dev/cu.usbmodem2121102
   --v-x 150 --omega 0 --duration 1200 --watch 0.9` — **no `SET` command
   issued anywhere in this session, manually or by the script** (confirmed by
   reading the script: it only calls `connect()`/`twist()`/`stop()`/telemetry
   reads):

   ```
   [PASS] connect()                                  (mode=direct)
   [PASS] twist() returns a corr_id                  (corr_id=1)
   [PASS] twist() ack confirmed via ack ring          (ack=AckEntry(corr_id=1, ok=True, err_code=0))
   [PASS] encoders moving during twist()              (before=(0, 0) after=(137, 132))
   [PASS] stop() returns a corr_id                    (corr_id=2)
   [PASS] stop() ack confirmed via ack ring           (ack=AckEntry(corr_id=2, ok=True, err_code=0))
   ==== 6/6 checks passed ====
   ```

   Encoders climbed from `(0, 0)` to `(137, 132)` together, in the commanded
   forward direction, roughly proportional over the ~1.2s window at 150mm/s —
   matches 103-010's own re-run shape (both wheels climbing together, right
   order of magnitude) and directly confirms the acceptance criterion:
   motion executes and reported (encoder) pose tracks with zero manual
   configuration. `twist_drive.py`'s own `finally` block re-issued `stop()`
   and disconnected cleanly; no motor left energized.

**Tests**: `uv run python -m pytest tests/unit -k robot_config` (6 new tests
in `tests/unit/test_robot_config.py` — default-false, explicit-true/false
round-trip, both rig profiles carry the flag, `togov.json` unaffected) and
the full suite both pass: 574 passed (568 baseline + 6 new), zero failures,
zero regressions. `scripts/check_config_sync.py` also re-run clean
(`OK — no drift detected`) confirming the new field's allowlist entry is
correctly wired.

**Surprises**: none in the code; the one correction was the ticket-text's
profile-location suggestion (`tovez_nocal.json`) not matching which profile
is actually live on the rig today (`tovez.json`, since sprint 093) — resolved
by setting the flag in both files rather than picking one, since it's the
same physical hardware either way.

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
