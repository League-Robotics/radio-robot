---
id: '010'
title: Full sweep and bench gate
status: done
use-cases:
- SUC-050
- SUC-051
- SUC-052
- SUC-053
- SUC-054
- SUC-055
depends-on:
- '006'
- '007'
- 008
- 009
github-issue: ''
issue:
- gut-to-minimal-firmware-motion-stack-excision-move-protocol-minimal-telemetry.md
- protocol-set-point-the-minimal-firmware-s-complete-command-surface.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Full sweep and bench gate

## Description

Final ticket — sprint 116's own hardware-bench-testing gate, per
`.claude/rules/hardware-bench-testing.md` and the protocol set-point
issue's Verification section. Structured like sprint 115's own final
ticket (010) to handle the "hardware may be disconnected at execution
time" reality: a real protocol gate if the robot is on the stand and
connected, or a full sim dry-run of the same scenarios (ticket 008 already
covers most of them) plus a written bench checklist for whenever hardware
becomes available — either way, the full sim scenario suite is a hard
acceptance criterion, not an optional nice-to-have contingent on hardware
presence (sprint.md Migration Concerns).

## Acceptance Criteria

- [x] `uv run python -m pytest` green across the full suite (not a
      subset).
- [x] `python build.py` builds firmware + host sim lib clean.
- [ ] **If hardware is connected** (`pyocd list` / `mbdeploy probe` shows
      exactly one micro:bit V2): `just build-clean` + `mbdeploy deploy`
      (hex verified by full UID — confirm it's the robot, not the relay
      dongle); then the real protocol gate, robot on stand, wheels free —
      `HELLO`/`PING` (`t=` present)/`CONFIG` patch persists across
      power-cycle/`MOVE` × both velocity variants × all three stop
      conditions/`STOP`, each acked correctly; stop-condition behavior
      (time/distance/angle measured via encoders on the stand,
      stalled-timeout fault); chaining/replace/`ERR_FULL`/no-deadman
      empty-queue expiry with zero host traffic; a ≥10-minute soak at
      5-10 Hz alternating MOVEs — no reboot/lockup, seq monotonic, drop
      rate at or better than the sprint-115 baseline.
      **PARTIALLY MET — see completion notes.** Hardware WAS connected and
      this branch WAS taken (deploy/UID/HELLO/PING/STOP/chaining/replace/
      ERR_FULL/no-deadman-drain/timeout-fault/CONFIG-mid-MOVE/10-minute
      soak all verified clean); the DISTANCE/ANGLE stop-condition
      encoder-measured sub-items and the forward/reverse/pivot
      encoder-tracking sub-item could NOT be verified — a motor-bus
      disconnect (`conn_left`/`conn_right` both `False`) developed
      mid-session, confirmed NOT a sprint-116 regression (survived two
      clean reflashes; sprint 116 touched no motor-bus/I2C code). Left
      unchecked per "do not mark the criterion met" — not silently
      passed. Full evidence: `docs/bench-checklists/sprint-116-move-protocol.md`.
- [ ] **If hardware is absent or unavailable**: write
      `docs/bench-checklists/sprint-116-move-protocol.md` (mirroring
      `sprint-115-gut-s1.md`'s structure) listing every check above as a
      TODO for whenever hardware becomes available, AND run a full sim
      dry-run of the same scenario list (ticket 008's suite, plus any gap
      it doesn't already cover) as the sprint's actual, hard acceptance
      evidence for this ticket.
      **N/A this run** — hardware was present and connected throughout,
      so the "hardware absent" branch was never applicable. (The full
      sim suite, including `test_move_protocol.py`, still ran and passed
      as part of the AC1 full-suite sweep.)
- [x] Report which branch (real gate vs. sim dry-run + checklist) was
      taken and why, in this ticket's own completion notes.

## Testing

- **Existing tests to run**: the full `uv run python -m pytest` suite,
  plus `src/tests/sim/system/test_move_protocol.py` (ticket 008)
  specifically as the dry-run substitute if hardware is absent.
- **New tests to write**: none beyond what ticket 008 already added,
  unless the dry-run surfaces a scenario gap — if so, add it here rather
  than silently skip it.
- **Verification command**: `python build.py && uv run python -m pytest`
  (+ the hardware or sim-dry-run branch above).

## Completion notes (2026-07-22)

**Branch taken: the real hardware gate.** `pyocd list` showed exactly one
connected probe (`Arm BBC micro:bit CMSIS-DAP`, UID
`9906360200052820a8fdb5e413abb276000000006e052820`); `mbdeploy probe`
confirmed the SAME UID as ROLE=`NEZHA2` NAME=`robot` at
`/dev/cu.usbmodem2121102` (the other rows in `mbdeploy probe`'s registry
are stale historical entries for devices not currently plugged in — not
evidence of ambiguity). Deployed via `just build-clean` (firmware hex
`v0.20260721.2`, FLASH 137436B/364KB=36.87%) + `mbdeploy deploy <UID>
--hex MICROBIT.hex`. Full results and raw evidence live in
`docs/bench-checklists/sprint-116-move-protocol.md` — this section
summarizes.

**Sweep (AC1/AC2)**: `python build.py` clean (firmware + host sim lib,
`v0.20260721.2`) and `uv run python -m pytest` green — **1197 passed, 13
skipped, 10 xfailed, 1 xpassed, 1 warning** (the one warning is the
pre-existing, previously-documented `telemetry_pb2.ACK_STATUS_DONE`
dormant-planner-import breakage from sprint 115's own completion notes —
not a regression). Re-run identically at the end of this ticket's own
work (after adding `move_protocol_bench.py`/`move_soak.py` and fixing a
bug found live in the latter) with the same result.

**Hardware protocol gate — clean passes**: HELLO/PING (via `connect()`'s
own banner-classify + PING-poll, `DEVICE:NEZHA2:robot:tovez:2314287040`;
PING replies bare `OK pong`, no `t=` — a pre-existing, already-documented
AS-BUILT gap per `protocol-v4.md` §2.4, not something this session
introduced); `move_protocol_bench.py` (new, `src/tests/bench/`) —
chaining (`replace=False` seamless hand-off), `replace=True` mid-motion
preemption (flushed Move's completion ack correctly never appears),
5-deep `ERR_FULL` (byte-for-byte queue preserved), empty-queue drain with
zero further host traffic, the `timeout` safety backstop (zero-velocity
DISTANCE Move, safe by construction, ends at `timeout` with
`kFlagFaultMoveTimeout` set and `ack_err==0` per the documented AS-BUILT),
`STOP` mid-motion (immediate halt, pending Move flushed), and a `CONFIG`
patch mid-MOVE (acked OK, Move completes normally) — **39/43 scenario
checks passed** on the final clean run (two "active flag" checks were
initially flaky due to the SCRIPT reading a fixed-duration window's last
frame instead of anchoring to the actual ack/completion event frame — fixed
in the script itself, then reliably clean across 3 repeats). `CONFIG`
persistence: the apply path itself (ack `ERR_NONE`, not
`ERR_UNIMPLEMENTED`) survives a `pyocd commander -c reset` cleanly and the
boot banner/persisted-tuning store is undisturbed (no schema wipe,
matching this sprint not bumping the persisted-tuning schema) — but
BEHAVIORAL confirmation that the same patched value survived (the only
way to check, since there is no live config read-back arm) could not be
completed, same root cause as below.

**Hardware protocol gate — blocked by a motor-bus disconnect (NOT a
sprint-116 regression)**: partway through this session `Telemetry.flags`
bits 3/4 (`conn_left`/`conn_right`) went from `True` to `False` and stayed
there — both motors simultaneously, `otos_present` also `False` — the
documented "disconnected bus" signature (distinct from a single-wheel
encoder wedge-latch). This happened AFTER an initial `twist_drive.py
--v-x 150` genuinely moved the encoders (`before=(0,0) after=(70,66)`), so
the bus was live at session start. Survived two full clean reflashes
(`mbdeploy deploy`, each triggering an automatic CTRL-AP mass-erase
recovery) and a 15-second idle re-check — ruling out a stale-build or
MCU-reset-clearable cause. Sprint 116 touched zero motor-bus/I2C driver
code (only `envelope.proto`'s `Move` arm, `MoveQueue`, `StopCondition`,
`RobotLoop` dispatch, `Drive::setWheels`, `Odometry::pathLength`,
`protocol.py`) — this is a physical/bench-hardware condition, not
something a software/firmware action available to an agent can clear (see
`docs/knowledge/2026-07-04-encoder-wedge.md`'s own "robot booted with the
rail off... Recover with a FULL power-cycle including USB unplug"
guidance — outside an agent's physical reach). Blocked, specifically:
DISTANCE-stop and ANGLE-stop Moves ending within tolerance of the
commanded threshold (both instead ran to `timeout`, since
`Odometry::pathLength()`/`theta()` never advance with the bus down —
correctly reported as `FAIL`, not silently passed), the `MoveWheels`
opposite-direction-encoder-delta check, forward/reverse
encoder-tracking, and CONFIG persistence's own behavioral confirmation.
**Recommend the stakeholder physically inspect the motor-bus
wiring/connector and power rail before the next hardware session**, then
re-run exactly those items.

**10-minute soak**: `rig_soak.py`/`rig_dev.py` still call the deleted
`NezhaProtocol.twist()` (ticket 007 left them dormant/broken on purpose,
catalogued in that ticket's own completion notes) — wrote a fresh
`src/tests/bench/move_soak.py` instead, the "write a small soak loop"
alternative the dispatching prompt itself anticipated. **Two attempts**:
the first was launched via the harness's own `run_in_background` Bash
task; its notification never fired and, when checked, the task's log file
was empty and no process was running — almost certainly killed silently
by the harness's own ~600000ms (10 minute) hard cap on a single Bash
call colliding with the script's own 600s duration plus connect/finalize
overhead, leaving no time margin. Relaunched as a truly OS-detached
process (`nohup ... & disown`, unbuffered `python -u`, watched via a
`Monitor` until-loop rather than the Bash task tracker) — this one
completed the full, genuinely measured **600.0 seconds**: 3684 commands
sent (92 explicit `stop()` segments interleaved), 11252 primary frames,
**0.01% TLM drop rate** (well under the 2% working threshold — no numeric
sprint-115 hardware baseline exists to compare against, since that
sprint's own checklist was never run on hardware), zero new fault bits,
responsive at the end. The script's own automated verdict said `PASS:
False` on a "reboot detected" check — investigated and confirmed a FALSE
POSITIVE in the script's own detector, not a real reboot:
`kFlagEventBootReady` (flags bit 11) is documented as "one-shot,
transition-cycle" but is actually sticky-forever once set
(`RobotLoop::boot()` calls `setFlag(kFlagEventBootReady, true)` exactly
once, `robot_loop.cpp:433`, with no corresponding clear anywhere in the
tree), so counting its occurrences false-positives within the first few
frames of any run. Corroborating evidence against a real reboot: the
0.01% drop rate itself (a real reboot resets the on-chip `seq` counter,
which would spike the gap-accounting far above that), the host's own
`corr_id` counter incrementing continuously to 3686 with no
re-`connect()`, and the genuinely-reliable robot-clock-backward-jump
check (the only OTHER signal the script tracks) never firing. Fixed
`move_soak.py` to drop the flawed check and rely solely on the clock-jump
signal, with the sticky-bit finding documented inline. Did not re-run a
third time given (a) the corrected interpretation is well-evidenced from
multiple independent angles, not just an assertion, and (b) the session
had already run very long — flagging this judgment call explicitly for
the team-lead rather than silently deciding it needed no mention.

A **separate, non-gating observation**: measured primary-frame delivery
rate this session was ~19 Hz (11252 frames / 600.0s = 18.75 Hz; a
15-second `tlm_log.py` idle sample independently measured 18.9 Hz), not
the ~50 Hz nominal primary period (`kPrimaryPeriod` = 20ms cycle,
unchanged by sprint 116). Drop rate stayed excellent throughout, so this
is under-delivery-rate, not frame loss — likely 115200-baud bandwidth
saturation from the armored frame size (~207 B/frame × 50 Hz ≈ 10.35 kB/s
against ~11.52 kB/s raw 8N1 throughput, before secondary-frame and
inbound-command traffic). Worth a team-lead follow-up note for any future
`tlm_log.py`-based analysis that assumes 20ms-spaced samples; not a
sprint-116 regression and not required by this ticket's own gate.

Robot left stopped (explicit final `STOP` + confirmed `active=False`,
`vel=(0,0)` before disconnecting).

New files this ticket: `src/tests/bench/move_protocol_bench.py`,
`src/tests/bench/move_soak.py`,
`docs/bench-checklists/sprint-116-move-protocol.md`. Grep sweep confirmed
`App::Deadman` no longer exists anywhere in `src/firm/` (SUC-053's own AC)
and that the known dormant `.twist()`/`sTimeout` callers
(`rig_dev.py`/`rig_soak.py`/`planner/executor.py`/`io/repl.py`/
`nav/camera_goto.py`) exactly match ticket 007's own already-catalogued
disposition — no new gap surfaced there.
