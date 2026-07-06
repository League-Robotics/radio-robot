---
id: '009'
title: Sim and hardware bench verification
status: open
use-cases: [SUC-001, SUC-002, SUC-003, SUC-004, SUC-005, SUC-006, SUC-007, SUC-008]
depends-on: ['005', '008']
github-issue: ''
issue:
- firmware-closed-loop-motion-verbs.md
- firmware-config-and-pose-set-surface.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim and hardware bench verification

## Description

The sprint's acceptance backstop, per `.claude/rules/hardware-bench-
testing.md`: "a sprint is not 'done' on tests alone — it must be seen
working on the stand." This ticket has two required, distinct halves —
**both must pass; neither substitutes for the other**:

1. **Sim geometry/wire verification** against `libfirmware_host` (the
   host-side ctypes simulator, sprint 081), covering every verb and
   config surface tickets 001-008 landed.
2. **Real-hardware bench gate**, robot mounted on the stand per
   `.claude/rules/hardware-bench-testing.md` (wheels off the ground, safe
   to drive freely), exercising the closed-loop motion and config/
   pose-set surface on the physical robot.

This ticket completes **both** `firmware-closed-loop-motion-verbs.md` and
`firmware-config-and-pose-set-surface.md` — it is the last ticket
referencing either issue, and its acceptance criteria are drawn directly
from both issues' own "Acceptance (sketch)" sections.

**The OTOS-gap caveat must be recorded explicitly, not silently.** No
real-hardware OTOS driver exists this program (`clasi/issues/
nezha-hardware-otos-driver-for-new-source-tree.md`, deferred) — this is
the identical caveat sprint 082 already carried forward honestly. The
bench report for this ticket must say so in as many words: OTOS verbs are
sim-only-verifiable; on the stand they are expected and required to
return `ERR nodev`, not to be skipped or silently unverified.

## Acceptance Criteria

### Sim geometry / wire verification (against `libfirmware_host`)

- [ ] `D 200 200 500` moves true pose ~500 mm; `EVT done D reason=dist`.
- [ ] `RT 9000` rotates ~90° (within plant tolerance); `EVT done RT`.
- [ ] `T`/`S`/`R`/`TURN`/`G`/`STOP` each verified per tickets 002-004's
      own acceptance criteria (this ticket is a consolidated re-run
      across the full verb set, not a re-derivation of new tolerances).
- [ ] `stop=` clauses (`{t, d, heading, pos, rot}`) honored, OR-combined
      with each verb's built-in stop; `sensor`/`color`/`line` clauses
      confirmed to reject with `ERR badarg`, not silently ignored or
      crash.
- [ ] `mode=` returns to `I` at completion of every verb family, polled
      via `SNAP` with no `EVT` listener (confirms `mode=I` is sufficient
      for tour-completion detection independent of `EVT` delivery, per
      SUC-004 and the issue's own tour-runner motivation).
- [ ] `SET tw=...` then `GET` round-trips and visibly changes drivetrain/
      turn geometry; every dropped key (Decision 2's table) returns
      `ERR badkey`.
- [ ] `SI x y h` teleports the fused pose (confirmed via `SNAP`); `ZERO
      enc` rezeroes `enc=`/`encpose=` with no phantom-jump discontinuity.
- [ ] All seven OTOS verbs (`OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/`OA`) ack
      against the sim.
- [ ] Full existing test suite (`tests/sim/`, `tests/bench/`,
      `tests/playfield/`, `tests/unit/`) stays green — no regression in
      anything sprints 077-083 already shipped.

### Hardware bench gate (on the stand, per `.claude/rules/
    hardware-bench-testing.md`)

- [ ] Firmware deployed via `mbdeploy deploy --build`.
- [ ] Sensors alive: encoders (motor controller) respond with plausible,
      changing values while driving.
- [ ] Closed-loop drive and turn verbs (`D`/`T`/`R`/`TURN`/`RT`/`G`/`S`)
      commanded on the stand; wheels drive in both directions; encoders
      increment proportionally to commanded speed and in the expected
      direction.
- [ ] `STOP` halts immediately on the stand.
- [ ] `SET`/`GET` take visible effect on the physical robot (e.g. a
      `tw`/`ml`/`mr` change visibly alters turn/arc behavior on the
      stand).
- [ ] `SI`/`ZERO enc` take visible effect on the physical robot's
      reported pose/encoders.
- [ ] All seven OTOS verbs return `ERR nodev` on the physical robot (no
      real OTOS driver this program) — verified explicitly, not assumed;
      **no crash**.
- [ ] Round-trip command/reply confirmed over the real serial link (the
      required gate per `.claude/rules/hardware-bench-testing.md`); radio
      is best-effort, checked via `mbdeploy list` at execution time, not
      assumed present.
- [ ] Bench report explicitly states the OTOS-gap caveat (no real driver
      this program; `ERR nodev` is the PASSING result for those seven
      verbs on hardware, not a partial failure).

## Implementation Plan

**Approach:** No new production `source/` files — this ticket is test
code plus the bench session and its written report. Sim tests are
authored/extended incrementally as tickets 001-008 land (each ticket's
own "Testing plan" already specifies its own sim tests); this ticket adds
the **cross-cutting** consolidated pass (full verb sequences, `stop=`
combinations spanning multiple verbs, `mode=` polling across a whole
session) that no single earlier ticket owns end-to-end, plus the bench
session itself.

**Files to create:**
- `tests/sim/test_motion_verbs_full_sequence.py` (or equivalent —
  consolidated cross-verb geometry + `stop=` + `mode=` pass)
- `tests/sim/test_config_pose_set_otos_surface.py` (or equivalent —
  consolidated `SET`/`GET`/`SI`/`ZERO`/OTOS pass)
- A bench-session report (markdown, checked in under this ticket or the
  sprint directory) recording the hardware gate's results, explicitly
  including the OTOS-gap caveat.

**Files to modify:** None expected in `source/` (verification-only
ticket); may touch `docs/protocol-v2.md` for any last consolidation-pass
discrepancy found between tickets 002-008's individual doc edits (should
be none, but this ticket is where such a gap would surface).

**Testing plan:**
- Full `uv run pytest` sim suite (all tiers: `sim/`, `bench/`,
  `playfield/`, `unit/`).
- Hardware bench session per `.claude/rules/hardware-bench-testing.md`'s
  quick-smoke-sequence spirit, adapted to this sprint's actual verb set
  (the doc's own table is flagged stale/pre-protocol-v2; this ticket's
  bench report is itself a contribution toward refreshing it, though a
  full rewrite of that table is not required by this ticket).

**Documentation updates:** The bench-session report (new). Any
last-mile `docs/protocol-v2.md` corrections found during the
consolidated pass, called out explicitly rather than silently folded in.
