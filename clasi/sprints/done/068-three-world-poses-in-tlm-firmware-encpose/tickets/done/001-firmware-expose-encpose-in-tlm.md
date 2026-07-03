---
id: '001'
title: 'Firmware: expose encpose= in TLM'
status: done
use-cases:
- SUC-001
- SUC-004
depends-on: []
github-issue: ''
issue: tlm-three-world-poses-encoder-only-pose.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: expose encpose= in TLM

## Description

The encoder-only dead-reckoned pose **already exists** in firmware and has
since sprint 047 — this ticket is a wire-exposure change, not new
integration math. `Odometry::predict()` (`source/control/Odometry.cpp`)
maintains a private `_encPoseX/Y/H` accumulator, arc-integrated from wheel
deltas only, written every tick into `ActualState.encoder` — a
`PoseEstimate` field that sits alongside `.optical` (raw OTOS) and `.fused`
(EKF) in the same struct. `subsystems::Drive::state()` already copies it
through into `msg::DrivetrainState.encoder` (`Drive.cpp:206`:
`copyPE(_hw.encoder, _state.encoder);`) — identically to how `.fused` and
`.optical` are copied. The only thing missing is one `snprintf` call in
`Robot::buildTlmFrame()` (`source/robot/RobotTelemetry.cpp`) to put
`ds.encoder.pose.{x,y,h}` on the wire.

Two non-trivial blockers surfaced during planning that this ticket must
also resolve (see `architecture-update.md` Decisions 1 and 2 for full
rationale):

1. **`RobotConfig::tlmFields` is a `uint8_t` with all 8 bits already
   assigned** (`source/types/Config.h:7-15`; confirmed by the sprint-064
   comment at `RobotTelemetry.cpp:82`). A 9th field needs a wider bitmask.
   Chosen fix: widen to `uint16_t` and reuse the existing `STREAM fields=`
   opt-in/opt-out mechanism (not an unconditional field like `wedge=` was —
   `encpose=` is 24-29 bytes, large enough that a bandwidth-constrained
   consumer should be able to exclude it; see SUC-004).
2. **`buildTlmFrame()`'s stack buffer (`char tlmBuf[160]`,
   `RobotTelemetry.cpp:208`) is already near/over its worst-case limit**
   with the *existing* field set alone. Worst-case byte accounting
   (coordinates ±10,000mm, heading ±18,000 cdeg, velocities ±1,000mm/s,
   `t=` at `uint32_t` max):

   ```
   "TLM t=4294967295 mode=V seq=65535"   34
   " wedge=1,1"                          10
   " enc=-10000,-10000"                  18
   " pose=-10000,-10000,-18000"          26
   " vel=-1000,-1000"                    16
   " twist=-1000,-3142"                  18
   " otos=-10000,-10000,-18000"          26
   " ekf_rej=999999"                     15
                                        -----
                                         163  <- already exceeds 160
   ```

   Adding `encpose=` (another ~26-29 bytes) pushes the realistic worst case
   to ~192 bytes without line/color, or ~243 bytes with a line+color-
   equipped robot streaming everything. Because the emit loop's overflow
   behavior is silent-drop (a field whose `snprintf` return would exceed
   remaining space is simply skipped — safe, but invisible), this would
   manifest as an intermittent, magnitude-dependent field disappearance
   with no error signal. Chosen fix: grow the buffer to 256 bytes (clean
   round number, comfortably covers the full worst case including
   line+color, headroom for one more field-sized addition).

## Acceptance Criteria

- [x] `source/types/Config.h`: `RobotConfig::tlmFields` widens `uint8_t` →
      `uint16_t`; new `constexpr uint16_t TLM_FIELD_ENCPOSE = (1u << 8);`;
      `TLM_FIELD_ALL` widens from `0xFFu` to `0x1FFu` (9 bits).
- [x] `source/robot/DefaultConfig.cpp:119`: `p.tlmFields = 0xFF;` →
      `p.tlmFields = TLM_FIELD_ALL;` so `encpose=` is on by default (every
      existing `STREAM <ms>` caller in `host/`, `tests/`, and TestGUI uses
      no `fields=` clause and relies on the default subscription).
- [x] `source/commands/SystemCommands.cpp::handleStream`: local `mask`
      widens `uint8_t` → `uint16_t`; add
      `if (strcmp(fbuf, "encpose") == 0) mask |= TLM_FIELD_ENCPOSE;`;
      `kFieldNames[]` table (and its `bit` field type) gains
      `{TLM_FIELD_ENCPOSE, "encpose"}`; the loop bound `fi < 8` → `fi < 9`.
- [x] `source/robot/RobotTelemetry.cpp::buildTlmFrame`: new block, gated on
      `config.tlmFields & TLM_FIELD_ENCPOSE`, inserted immediately after
      the existing `pose=` block (before `vel=`), emitting
      `encpose=<x>,<y>,<h>` (mm, mm, centidegrees) from
      `ds.encoder.pose.{x,y,h}`. No freshness/staleness gate — the encoder
      pose updates unconditionally every control tick.
- [x] `source/robot/RobotTelemetry.cpp::telemetryEmit`: `char tlmBuf[160]`
      → `char tlmBuf[256]`.
- [x] No change to `Odometry::predict()`, `ActualState`,
      `msg::DrivetrainState`, or `Drive::state()`'s `copyPE` call — verify
      by inspection that these are untouched (they already produce/copy
      the data correctly).
- [x] `docs/protocol-v2.md` §8 (TLM Frame Format): `encpose=` added to the
      field table and example. Do NOT attempt a full backfill of the
      section's pre-existing drift (missing `wedge=`/`twist=`/`otos=`/
      `ekf_rej=` from earlier sprints) — out of scope, flagged as Open
      Question 1 in `architecture-update.md`.
- [x] `tests/_infra/golden_tlm_capture.json` regenerated (requires a
      `--clean` sim build first — stale incremental builds on `/Volumes`
      build silently, per project knowledge) so `test_golden_tlm_unchanged`
      passes with `encpose=` present in every frame of the fixed sequence.
      This regeneration MUST land in this ticket, not a later one — the
      golden-capture test does an exact string match and fails the instant
      `encpose=` appears anywhere.
- [x] `STREAM fields=...` with `encpose` included/excluded round-trips
      correctly through `handleStream`'s parse-and-echo path (`OK stream
      fields=...` reflects the actual subscription). **Note**: this exposed
      a pre-existing, unrelated bug — `CommandProcessor::parseKV()` rewrites
      any `key=value` token in place before dispatch (truncating it at the
      `=`), so the old plain-variadic `streamSchema` never actually saw a
      reconstructed `fields=<csv>` token through the real command pipeline
      (only direct-ArgList unit tests, which never exercised the real
      tokenizer, exercised this path). Fixed by giving `STREAM` a custom
      `parseFn` (`parseStream`) that reconstructs `fields=<value>` from
      `kvs[]`, mirroring the existing `parseSet` idiom in
      `ConfigCommands.cpp`. This was a load-bearing fix, not scope creep:
      without it, this criterion is unsatisfiable for any field name,
      encpose included. See `source/commands/SystemCommands.cpp`.
- [x] Full default pytest suite green (`uv run python -m pytest`) after
      this ticket lands. Result: 2523 passed (baseline 2520 + 3 new
      `STREAM fields=` round-trip tests), 0 failed.

## Testing

- **Existing tests to run**: `tests/_infra/` golden-TLM test(s); any
  `tests/simulation/unit/test_tlm_stream.py` field-name coverage; full
  default suite via `uv run python -m pytest`.
- **New tests to write**: a `STREAM fields=` round-trip test exercising
  `encpose` inclusion/exclusion (may extend an existing
  `handleStream`-focused test file rather than adding a new one — check
  `tests/simulation/unit/test_tlm_stream.py` first).
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: This is a small, well-scoped diff across the TLM
field-registry and serializer files — no new computation, no changes to
the odometry/estimation pipeline. Follow the existing `otos=`/`pose=`
pattern exactly for the new `snprintf` block. Widen the bitmask storage
type and buffer size as the two structural prerequisites, per
`architecture-update.md` Decisions 1 and 2.

**Files to modify**:
- `source/types/Config.h` — `tlmFields` type, `TLM_FIELD_ENCPOSE` constant,
  `TLM_FIELD_ALL` widening.
- `source/robot/DefaultConfig.cpp` — default `tlmFields` literal.
- `source/commands/SystemCommands.cpp` — `handleStream`'s mask type, name
  table, loop bound.
- `source/robot/RobotTelemetry.cpp` — new `encpose=` emission block in
  `buildTlmFrame`; `tlmBuf` size in `telemetryEmit`.
- `docs/protocol-v2.md` — §8 TLM Frame Format, `encpose=` only.
- `tests/_infra/golden_tlm_capture.json` — regenerated fixture.

**Testing plan**:
- Build the sim with `--clean` (required — incremental builds on `/Volumes`
  go stale silently; verify the build banner is trustworthy per project
  knowledge) before regenerating the golden capture.
- Run the golden-TLM test and confirm it passes with `encpose=` present.
- Manually issue `STREAM fields=pose,otos` (omitting `encpose`) and confirm
  the echoed subscription and subsequent frames omit `encpose=`; issue bare
  `STREAM <ms>` and confirm `encpose=` is present by default.
- Run the full default suite (`uv run python -m pytest`) and confirm green
  before handing off to Ticket 002 (which depends on this field existing
  on the wire).

**Documentation updates**: `docs/protocol-v2.md` §8 — add `encpose=` to
the TLM field table and example line. Do not otherwise rewrite the
section.
