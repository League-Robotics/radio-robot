---
id: '010'
title: Hardware bench gate (S1 verification)
status: open
use-cases: [SUC-045, SUC-046, SUC-047, SUC-048, SUC-049]
depends-on: ["009"]
github-issue: ''
issue: telemetry-frame-tightening-amendment-to-gut-s1.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Hardware bench gate (S1 verification)

## Description

Final ticket of the sprint: the hardware bench gate, per
`.claude/rules/hardware-bench-testing.md` and the gut issue's own
Verification list (items 1-5, 7 — item 6's "S2 runs the protocol gate
from the set-point issue" and item 3's empty-queue-expiry half are
sprint 116/S2 scope, not this ticket's). As of sprint-planning time
(2026-07-21) the robot is **not connected** (no CMSIS-DAP probe, no
usbmodem port) — this ticket's acceptance criteria branch on hardware
availability at execution time, per the team-lead's own explicit
structuring instruction. Do not let hardware absence block sprint
closure: if absent, this ticket's job is to produce a complete,
accurate checklist document for the stakeholder to run later, not to
silently skip or fake a pass.

**Order matters for the S0 baseline** (sprint.md Open Questions #1): the
`pre-gut-motion-stack` tag is verified identical to pre-S1 HEAD (zero
source drift) — meaning the CURRENT flashed image on the robot, if it
predates this sprint's flash, already IS the S0 baseline reference. If
no baseline capture exists yet, it must be captured **before** flashing
the S1 build produced by tickets 002-009 — a baseline captured on the
post-gut image is not a baseline.

## Acceptance Criteria

**If hardware is available at execution time:**
- [ ] S0 baseline: if not already captured, deploy the pre-S1 image (or
      confirm the currently-flashed image predates this sprint) and
      capture a ~2-minute telemetry log (seq continuity / drop rate)
      BEFORE flashing the S1 build.
- [ ] Deploy the S1 build via `just build` + `mbdeploy deploy` (hex by
      full UID — verify it's the robot, not the relay dongle); boot
      banner observed on serial.
- [ ] Drive: `NezhaProtocol` over serial — twist forward/reverse/pivot
      with encoder readings tracking sign and magnitude, sample times
      ~cycle-period apart; mode VELOCITY; `conn_left`/`conn_right` set.
- [ ] Bounded-motion safety (S1 form): one bounded command then silence
      → deadman neutralizes within its lease.
- [ ] STOP: while streaming twists at ~10 Hz → immediate neutral.
- [ ] Telemetry-as-dataset: `tlm_log.py` (ticket 008) captures a drive
      session → CSV with per-reading times; frame rate ≈ 50 Hz (every
      cycle); `line`/`color` words plausible and changing; OTOS reading
      carries velocities when present.
- [ ] Soak: ≥10 minutes streaming alternating commands at 5-10 Hz
      (adapt `twist_drive.py`/`rig_soak.py`/`pid_hold_speed.py` — the
      gut issue's own confirmed survivors). Pass: no reboot (no banner
      re-emission), seq monotonic at the doubled rate, drop rate at or
      better than the S0 baseline, no motion-timing regression from the
      added line/color sensor reads, responsive at end.
- [ ] Persisted-tuning: the one-time tuning-store wipe + radio-channel
      re-pick observed once (expected side effect of the schema bump,
      ticket 004 — not a regression); a subsequent CONFIG patch then
      survives a power cycle at the new 85-byte layout.
- [ ] Every sensor confirmed alive per the standing bench gate: encoders
      (both wheels), OTOS (position/velocity), line sensor (4 channels),
      color sensor (RGBC).
- [ ] Completion notes record the actual hardware session (what was
      observed, any deviation from expected, the actual measured flash
      size / frame size / drop rate numbers).

**If hardware is absent at execution time:**
- [ ] Produce `docs/bench-checklists/sprint-115-gut-s1.md` (the
      sprint-114 precedent for this exact situation) containing the
      full checklist above, in explicit order (S0 baseline capture
      BEFORE flashing S1 — called out prominently, not buried), with
      enough context (exact commands, expected values, what a pass vs.
      fail looks like for each step) that the stakeholder can run it
      unassisted.
- [ ] Completion notes say so **honestly** — hardware was not available,
      the gate was not run, this ticket produced a checklist document
      instead of a pass. Do not mark this ticket's SUC acceptance
      criteria as satisfied when they were not actually exercised on
      real hardware.

## Implementation Plan

**Approach**: Check hardware availability first (`pyocd list` / `mbdeploy
probe`, per `.claude/rules/debugging.md`'s own preconditions section)
before deciding which branch of this ticket applies. If available,
follow `.claude/rules/hardware-bench-testing.md`'s deploy-and-drive
recipe in the order given above (baseline before flash). If absent,
write the checklist doc modeled directly on
`docs/bench-checklists/sprint-114-config-and-deadband.md`'s own
structure (read it first for the established format before writing a
new one from scratch).

**Files to create (hardware-absent branch only)**:
`docs/bench-checklists/sprint-115-gut-s1.md`.

**Files to modify**: none (this ticket doesn't touch source — it is
purely a verification/documentation step).

**Testing plan**: this ticket IS the testing plan — see Acceptance
Criteria.

**Documentation updates**: `clasi/issues/bench-turns-spin-forever-non-termination.md`
and `clasi/issues/nocal-straight-terminal-wedge-needs-velocity-integrator.md`
(both cited by the gut issue as "moot after the excision") are candidates
for closing once this ticket confirms the deleted completion machinery
they indicted is actually gone and the robot drives cleanly without it —
close or re-scope them if the hardware session (or the checklist's own
future run) confirms this; do not close them speculatively if hardware
was absent this pass.
