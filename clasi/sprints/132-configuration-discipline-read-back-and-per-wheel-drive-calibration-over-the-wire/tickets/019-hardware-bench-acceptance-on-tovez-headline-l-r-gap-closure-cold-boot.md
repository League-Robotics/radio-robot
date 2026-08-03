---
id: '019'
title: "Hardware bench acceptance on tovez — headline L/R gap closure, cold boot"
status: open
use-cases:
- SUC-006
depends-on:
- '018'
github-issue: ''
issue: the-configuration-object.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Hardware bench acceptance on tovez — headline L/R gap closure, cold boot

## Description

The sprint's headline deliverable and second acceptance-concentrated
closing ticket. `tovez` is confirmed alive and healthy as of a recent
live check (PONG, telemetry flowing, wheels driving, encoders climbing,
both motors connected, on the stand) — address it by UID
`9906360200052820a8fdb5e413abb276000000006e052820`, never by port
(`.claude/rules/hardware-bench-testing.md`). A stale `mbdeploy list` row
with no ROLE/DEVICE NAME is a failed-probe artifact, not a dead-robot
signal — re-probe (`mbdeploy probe` then `mbdeploy list`) and trust the
live check, not a remembered row.

Push per-wheel Stage-A drive correction live over the new wire (ticket
009's capability) and re-run `src/tests/bench/velocity_profile_gate.py`
before/after, demonstrating the measured 11.1-point L/R plateau-tracking
gap (L 96.0% / R 84.9%) closing — including from a COLD BOOT
(power-cycle or reflash between the correction being applied/persisted
and the verification run — not just a warm, already-tuned session), per
`.claude/rules/hardware-bench-testing.md`'s standing verification gate.

## Acceptance Criteria

- [ ] `mbdeploy list` confirms `tovez` present by UID before any hardware
      action (re-probe if the row looks stale).
- [ ] Baseline measurement: `velocity_profile_gate.py` run on `tovez`
      (by UID) BEFORE any correction is pushed, confirming (or
      re-measuring) the L/R plateau split.
- [ ] Per-wheel Stage-A correction is pushed live over the wire (ticket
      009's `DRIVE` group push), and/or baked via the reshaped
      `tovez.json` (ticket 017) — both paths are legitimate; the ticket
      records which was used.
- [ ] The correction survives a COLD BOOT — the robot is power-cycled or
      reflashed between the correction landing and the final
      measurement, not just measured warm in the same session.
- [ ] `velocity_profile_gate.py` run AFTER, from cold boot, shows the L/R
      gap closed relative to baseline — the specific before/after
      numbers are recorded in this ticket's completion notes, not just a
      pass/fail.
- [ ] The standing bench gate's other checks (sensors alive, wheels
      drive and encoders increment in the expected direction, round-trip
      over the real link) are confirmed as part of this session, per
      `.claude/rules/hardware-bench-testing.md`.
- [ ] Bench-room lights confirmed on (Shelly relay per the rule file)
      before any camera/visual portion of the check that needs it — not
      applicable if this ticket's checks are stand-only with no camera
      dependency; confirm and note either way.

## Testing

- **Existing tests to run**: n/a — this ticket IS a hardware test.
- **New tests to write**: none — `velocity_profile_gate.py` already
  exists.
- **Verification command**: `uv run python
  src/tests/bench/velocity_profile_gate.py --port <tovez's live port,
  taken fresh from mbdeploy list, never a remembered port number>`
  (before and after).

## Implementation Plan

**Approach**: Probe for `tovez` first, confirm UID match. Run baseline
`velocity_profile_gate.py`. Push or bake the correction (whichever
ticket 018's findings suggest is more representative of the sprint's
actual end state — likely the baked/reshaped path, since that's what a
real deployment would use, with the live wire push as the demonstration
of the wire capability itself). Cold-boot the robot. Re-run
`velocity_profile_gate.py`. Record before/after numbers.

**Files to modify**: none expected (this is a verification run, not code
changes) — unless the bench run surfaces a genuine bug, in which case
it's reported and the fix belongs in the owning earlier ticket (reopened),
not patched here.

**Testing plan**: as above.

**Documentation updates**: this ticket's own completion notes ARE the
documentation — before/after numbers, port used, UID confirmation,
cold-boot method (power-cycle vs. reflash).
