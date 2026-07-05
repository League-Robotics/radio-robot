---
id: '003'
title: DEV protocol wiring + protocol-v2.md documentation
status: open
use-cases:
- SUC-002
- SUC-003
- SUC-004
depends-on:
- '002'
github-issue: ''
issue: armor-motor-write-path-against-reversal-latch.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# DEV protocol wiring + protocol-v2.md documentation

## Description

Wire ticket 002's new `Hal::Motor` capabilities to the `DEV` bench-command
surface and document them in `docs/protocol-v2.md` §16. Depends on ticket
002 (the getters/CFG-relevant fields must exist and behave correctly
before they're exposed on the wire). No firmware *policy* logic changes
here — this ticket only reads/writes the already-implemented armor state
through `dev_commands.cpp`.

**In `source/commands/dev_commands.cpp`**:
- `applyMotorCfgKey()` (the `DEV M <n> CFG k=v ...` key table) gains two
  rows:
  - `dwell` → `cfg.reversal_dwell` (`Opt<float>`): parse the value, set
    `.has = true, .val = atof(value)`, format the applied-echo as `%.1f`.
  - `deadband` → `cfg.output_deadband` (`Opt<float>`): same shape, `%.3f`.
- The shared per-motor `STATE` line formatter (the function building
  `pos=.. vel=.. applied=.. wedged=.. conn=..`, used by both
  `DEV M <n> STATE` and the aggregate `DEV STATE`) grows three tokens,
  appended after `wedged=` and before `conn=`: `wsus=%d` (from
  `s.wedge_suspect`), `hrc=%u` (from `s.hard_reset_count`), `src=%u` (from
  `s.soft_reset_count`) — final line shape: `pos=.. vel=.. applied=..
  wedged=.. wsus=.. hrc=.. src=.. conn=..`.
- No change to `DEV M <n> RESET`'s handler itself (it already just calls
  `apply()` with `reset_position=true`); only its *documented* semantics
  change (below).
- Verify `snprintf` buffer sizes accommodate the three new tokens (check
  the existing buffer size constants near the STATE formatter — grow if
  needed).

**In `docs/protocol-v2.md` §16**:
- `DEV M <n> CFG`'s Named Key Table gains two rows (`dwell`, `deadband`)
  matching the format used for `slew`/`min_duty`. The `deadband` row's
  description **must explicitly state it is not the same knob as
  `min_duty`** (`min_duty` gates the PID's integrator-freeze threshold on
  `|target|` in mm/s, despite its name; `deadband` gates the write-path's
  output duty fraction) — this is a carried-forward item from the
  architecture self-review (a real operator-confusion risk flagged
  because the codebase now has two differently-scoped "deadband"-shaped
  knobs).
- `DEV M <n> STATE` / `DEV STATE`'s example lines and the "always all five
  fields" language are updated to "always all eight fields," showing
  `wsus=`/`hrc=`/`src=` in their documented position.
- A new paragraph under `RESET` documents: the `OK` reply reports
  acceptance, not completion-kind — the hard-vs-soft decision is made at
  the top of the next `tick()`, observable via `hrc=`/`src=` deltas on a
  subsequent `STATE` poll, never via the `RESET` reply itself.
- A short note documents `wedged=` (raw, unconditional, unchanged
  semantics — includes idle motors) vs. `wsus=` (motion-qualified,
  suspect only while genuinely commanded to move) so a bench operator
  reading a log does not conflate the two (ties to SUC-003).

## Acceptance Criteria

- [ ] `DEV M <n> CFG dwell=<value>` and `DEV M <n> CFG deadband=<value>`
      are accepted, apply to the named `MotorConfig` field with `.has =
      true`, and echo the applied value in the `OK` reply, matching the
      existing `CFG` key-table row format (unrecognized keys still emit
      `ERR badkey <key>` without blocking other valid keys in the same
      command — unchanged existing behavior, verify it still holds).
- [ ] `DEV M <n> STATE` and `DEV STATE` both emit `wsus=`, `hrc=`, `src=`
      in the documented position and format.
- [ ] `docs/protocol-v2.md`'s CFG key table, STATE examples, "always all
      N fields" language, and `RESET` semantics paragraph are all updated
      to match the shipped wire format exactly (no drift between doc and
      code in the same ticket).
- [ ] The `deadband`-vs-`min_duty` distinction is documented explicitly in
      the CFG key table (not just in this ticket file) — this is the
      carried-forward architecture-review item; do not close this ticket
      without it.
- [ ] `host/robot_radio/robot/protocol.py`'s `parse_response()` verified
      (by a quick manual round-trip or existing test) to read the new
      tokens correctly with no code change (it is a generic key=value
      splitter).
- [ ] `just build` succeeds; a manual bench smoke check (or ticket 005's
      later soak) confirms `DEV M 1 CFG dwell=0` round-trips and
      `DEV M 1 STATE` shows all eight fields.

## Testing

- **Existing tests to run**: `uv run python -m pytest`; `just build`.
- **New tests to write**: none required at the Python/host level (the
  `DEV` family has no existing pytest coverage — `tests/bench/` scripts
  are HITL CLI tools, not pytest-collected, per `tests/CLAUDE.md`). If
  time permits, extend `tests/bench/dev_exercise.py`'s liveness/STATE
  checks to assert the new fields are present (optional, not required for
  this ticket's acceptance).
- **Verification command**: `just build`; manual smoke via
  `uv run python tests/bench/dev_exercise.py --port <port>` to confirm
  `DEV M <n> STATE` still round-trips cleanly with the longer line.
