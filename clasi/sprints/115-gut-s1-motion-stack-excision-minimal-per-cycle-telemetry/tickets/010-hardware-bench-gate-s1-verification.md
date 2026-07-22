---
id: '010'
title: Hardware bench gate (S1 verification)
status: done
use-cases:
- SUC-045
- SUC-046
- SUC-047
- SUC-048
- SUC-049
depends-on:
- 009
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
- [x] Produce `docs/bench-checklists/sprint-115-gut-s1.md` (the
      sprint-114 precedent for this exact situation) containing the
      full checklist above, in explicit order (S0 baseline capture
      BEFORE flashing S1 — called out prominently, not buried), with
      enough context (exact commands, expected values, what a pass vs.
      fail looks like for each step) that the stakeholder can run it
      unassisted.
- [x] Completion notes say so **honestly** — hardware was not available,
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

## Completion Notes (2026-07-21)

**Hardware: absent.** Re-verified myself at execution time, independent of
the team-lead's own prior check: `ls /dev/cu.usbmodem*` → "no matches
found"; `pyocd list` → "No available debug probes are connected". The
hardware-available branch of this ticket's acceptance criteria was **not
exercised** — none of those checkboxes are checked, and none of this
sprint's SUC acceptance criteria (SUC-045 through SUC-049) should be read
as hardware-confirmed by this ticket. The fallback branch was executed
instead, honestly, per this ticket's own instructions.

**Deliverable**: `docs/bench-checklists/sprint-115-gut-s1.md` — a
copy-pasteable, self-contained checklist (setup/UID-verification, S0
baseline capture via a separate `git worktree` on the `pre-gut-motion-stack`
tag BEFORE flashing S1, S1 deploy, standing sensors-alive + twist
forward/reverse/pivot drive gate, deadman, STOP-while-streaming,
`tlm_log.py` capture + frame-rate check, a 10-minute `rig_soak.py` soak,
and the persisted-tuning schema-wipe + power-cycle-survival check) modeled
on the sprint-114 precedent (`docs/bench-checklists/
sprint-114-config-and-deadband.md`).

**Everything non-hardware, run for real this pass:**
- `python build.py` (no `--clean`, incremental) builds BOTH the firmware
  hex and the host sim lib successfully: `v0.20260720.3`.
  Measured flash usage: **136388 B / 372736 B = 36.59%** (FLASH region) —
  a real, build-measured confirmation of the sprint's ~164 KiB-freed
  claim (Success Criteria / Migration Concerns). Measured worst-case wire
  sizes from the same build: `CommandEnvelope` total 50 B (config=44B
  worst arm), `ReplyEnvelope` total 153 B (`tlm`=147B worst arm) — this is
  the real number for sprint.md's Open Question #2 (~137 B estimate was
  for the `Telemetry` payload alone; 147B/153B here are the
  `tlm`-arm/whole-`ReplyEnvelope` figures from this build), both well
  under the 186 B cap.
- `uv run python -m pytest` (full suite): **1183 passed, 13 skipped, 10
  xfailed, 1 xpassed**, one `PytestUnhandledThreadExceptionWarning` from
  `test_set_origin.py` (`robot_radio.planner.tour` failing to import the
  deleted `telemetry_pb2.ACK_STATUS_DONE`) — this is EXACTLY sprint.md's
  own predicted, accepted "dormant host planner/tour code" breakage
  (Design Rationale Decision 6), not a new regression.
- A bounded, foreground sim dry-run (`SimLoop`/`libfirmware_host` against
  the real `src/firm/` tree, `data/robots/tovez_nocal.json` via
  `configure_from_robot()`, no ARM hardware) covering everything sim CAN
  prove: **15/15 checks passed**.
  1. Twist forward (`v_x=150`): both encoders advanced ~74mm over 13
     frames, `enc_left.time` monotonic non-decreasing (`[2300, 2400, 2450,
     ..., 3050]`, ~50-100ms apart at the sim's own real-time-detached
     manual-step granularity — NOT a stand-in for hardware's real 20ms
     cycle rate, see caveat below).
  2. Twist reverse (`v_x=-150`): both encoders decreased ~88mm.
  3. Twist pivot (`omega=0.8`): wheels moved in OPPOSITE directions
     (left −33.06mm, right +30.38mm) — the mirror-wheel shape the
     hardware checklist's Section 3c also asks for.
  4. `stop()`: velocity converged from ~180mm/s to within ±3mm/s inside
     the watched window.
  5. Deadman: one bounded twist (300ms lease), then silence — the
     `event_deadman_expired` flag was observed, and velocity was neutral
     (±1.5mm/s) by the end of the window. SUC-046 (deadman, "regression-
     only, not new behavior") holds in sim; still unconfirmed on real
     motors/encoders.
  6. `tlm_log.py`'s own production `stream_to_csv()` function (not a
     re-implementation) run directly against a live, background-tick-
     thread `SimLoop`: 38 rows captured over a 2.5s window, header
     matched `CSV_FIELDNAMES` exactly, `seq` gaps were all `1` (no drops
     in the sim capture), `enc_left_velocity` showed plausible nonzero
     values while driving.
  - **Caveat**: the sim capture's own real-time frame cadence (~62ms avg,
    ~16 Hz) is a `SimLoop` tick-thread/Python-timing artifact, NOT a
    stand-in for the real firmware's 50 Hz (20ms cycle) emission rate —
    the checklist's Section 6 frame-rate check is a hardware-only
    verification; the sim number above should not be quoted as if it
    were that number.

**A genuine, verified finding (not from the hardware — from reading
`src/firm/app/robot_loop.cpp:145` and `src/host/robot_radio/robot/
protocol.py`'s `_DRIVE_MODE_CHAR` table directly, then confirmed
empirically by the sim dry-run above)**: firmware now sets
`Telemetry.mode = msg::DriveMode::VELOCITY` while driving via TWIST
(`telemetry.proto`'s new `VELOCITY = 5` enumerator, added this sprint).
`protocol.py`'s host-side `_DRIVE_MODE_CHAR` lookup table was never given
a `VELOCITY` entry, so `TLMFrame.mode`/`tlm_log.py`'s `mode` CSV column
fall back to `"I"` — the SAME character `IDLE` produces. Confirmed on the
sim dry-run: `tlm_log.py`'s `mode` column read `["I"]` throughout an
active drive. This means the hardware checklist's own "mode VELOCITY"
acceptance line (as literally worded in this ticket's "if hardware is
available" branch and the gut issue's Verification item 2) **cannot
currently be observed via the `mode` column** — the checklist (Section 3)
tells the stakeholder to use `flag_active`/`TLMFrame.active` (`flags` bit
2) instead, and flags this as a known host-side decode gap worth a
follow-up ticket, not a stakeholder-visible failure. This ticket's own
scope is documentation-only (no source files touched), so the gap is
reported here rather than fixed.

**Two candidate issues for closing** (`bench-turns-spin-forever-non-
termination.md`, `nocal-straight-terminal-wedge-needs-velocity-
integrator.md`) were **NOT closed or re-scoped** this pass, per this
ticket's own instruction — hardware was absent, so "the robot drives
cleanly without [the deleted completion machinery]" was not actually
confirmed. Left for the checklist's own future stakeholder run.

**Net**: this ticket is `done` via the documented fallback path (the
sprint-114 precedent), not via a hardware pass. The hardware bench gate
itself — and this sprint's SUC-045 through SUC-049 hardware acceptance
criteria — remain **PENDING STAKEHOLDER EXECUTION** of
`docs/bench-checklists/sprint-115-gut-s1.md`.
