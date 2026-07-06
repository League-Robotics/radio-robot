---
id: 009
title: Sim and hardware bench verification
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
- SUC-008
depends-on:
- '005'
- 008
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

- [x] `D 200 200 500` moves true pose ~500 mm; `EVT done D reason=dist`.
- [x] `RT 9000` rotates ~90° (within plant tolerance); `EVT done RT`.
- [x] `T`/`S`/`R`/`TURN`/`G`/`STOP` each verified per tickets 002-004's
      own acceptance criteria (this ticket is a consolidated re-run
      across the full verb set, not a re-derivation of new tolerances).
- [x] `stop=` clauses (`{t, d, heading, pos, rot}`) honored, OR-combined
      with each verb's built-in stop; `sensor`/`color`/`line` clauses
      confirmed to reject with `ERR badarg`, not silently ignored or
      crash.
- [x] `mode=` returns to `I` at completion of every verb family, polled
      via `SNAP` with no `EVT` listener (confirms `mode=I` is sufficient
      for tour-completion detection independent of `EVT` delivery, per
      SUC-004 and the issue's own tour-runner motivation).
- [x] `SET tw=...` then `GET` round-trips and visibly changes drivetrain/
      turn geometry; every dropped key (Decision 2's table) returns
      `ERR badkey`.
- [x] `SI x y h` teleports the fused pose (confirmed via `SNAP`); `ZERO
      enc` rezeroes `enc=`/`encpose=` with no phantom-jump discontinuity.
- [x] All seven OTOS verbs (`OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/`OA`) ack
      against the sim.
- [x] Full existing test suite (`tests/sim/`, `tests/bench/`,
      `tests/playfield/`, `tests/unit/`) stays green — no regression in
      anything sprints 077-083 already shipped.

### Hardware bench gate (on the stand, per `.claude/rules/
    hardware-bench-testing.md`)

- [x] Firmware deployed via `mbdeploy deploy --build`.
- [x] Sensors alive: encoders (motor controller) respond with plausible,
      changing values while driving.
- [x] Closed-loop drive and turn verbs (`D`/`T`/`R`/`TURN`/`RT`/`G`/`S`)
      commanded on the stand; wheels drive in both directions; encoders
      increment proportionally to commanded speed and in the expected
      direction.
- [x] `STOP` halts immediately on the stand.
- [x] `SET`/`GET` take visible effect on the physical robot (e.g. a
      `tw`/`ml`/`mr` change visibly alters turn/arc behavior on the
      stand).
- [x] `SI`/`ZERO enc` take visible effect on the physical robot's
      reported pose/encoders.
- [x] All seven OTOS verbs return `ERR nodev` on the physical robot (no
      real OTOS driver this program) — verified explicitly, not assumed;
      **no crash**.
- [x] Round-trip command/reply confirmed over the real serial link (the
      required gate per `.claude/rules/hardware-bench-testing.md`); radio
      is best-effort, checked via `mbdeploy list` at execution time, not
      assumed present.
- [x] Bench report explicitly states the OTOS-gap caveat (no real driver
      this program; `ERR nodev` is the PASSING result for those seven
      verbs on hardware, not a partial failure).

## Verification Results (2026-07-06)

### Sim verification
Consolidated cross-verb tests added (`tests/sim/unit/test_motion_verbs_full_sequence.py`,
`test_config_pose_set_otos_surface.py`): full D/T/R/TURN/RT/G/S/STOP sequences,
`stop=` clauses OR-combined, `sensor/color/line`→`ERR badarg`, `mode=` polled to `I`
at every verb's completion with NO EVT listener, `SET tw`→`GET`+visible geometry
change, full 17-key dropped-key `ERR badkey` table, `SI`/`ZERO enc`, all 7 OTOS
verbs ack. **`uv run pytest tests/sim` → 246 passed; full suite → 383 passed**, no
regressions. Doc consolidation fix applied to `docs/protocol-v2.md` (`SI` odometer
re-anchor; `ZERO pose`→`ERR badarg` note).

### Hardware bench gate (robot on stand, wheels free)
Clean ARM build `v0.20260706.5` (`just build-clean`, since `mbdeploy deploy --build`
is broken — its venv lacks protobuf) flashed by UID via `mbdeploy deploy <uid> --hex
MICROBIT.hex`. Transcript (`scratchpad/bench_084.py` + supplementary T/R/G drive):

```
VER fw=0.20260706.5 proto=2
D 200 200 500     : enc 0,0 -> 522,515   EVT done D reason=dist
RT 9000 / RT -9000: turn-in-place (opposite wheel signs)  EVT done RT reason=rot
TURN 0            : EVT done TURN reason=heading
T 200 200 1000    : enc +264,+261   EVT done T reason=time
R 200 500 stop=t: : arc (asymmetric wheels)  EVT done R reason=time
G 300 0 200       : enc +272,+288   EVT done G reason=pos
S +200 / S -200   : enc +391,+383 fwd / -506,-496 rev
STOP              : OK stop (robot halts)
SET tw=100 -> RT  : visibly different rotation vs nominal; SET badkey -> ERR badkey
SI 1000 500 900 + ZERO enc: settled SNAP pose=1000,500,900 encpose=1000,500,900
OTOS OI/OZ/OR/OP/OV/OL/OA : ERR nodev <verb> (no crash)
```

**All hardware-gate criteria pass**: encoders alive and changing; every closed-loop
verb family drives both directions with the correct `EVT done ... reason=` token;
`STOP` halts; `SET`/`GET`/`SI`/`ZERO enc` take visible effect; round-trip confirmed
over the real USB-serial link.

**OTOS-gap caveat (stated explicitly, per the ticket):** no real-hardware
`Hal::Odometer` leaf exists in `Subsystems::NezhaHardware` this program (deferred:
`clasi/issues/nezha-hardware-otos-driver-for-new-source-tree.md`). `ERR nodev` is the
PASSING result for the seven OTOS verbs on hardware, not a partial failure; OTOS
behavior is sim-verified only.

**Known fidelity limitations (functional, imprecise — refinement deferred):** the
closed-loop verbs show a terminal settle-back / turn under-rotation of a few
mm/degrees from the sprint-081 velocity-PID zero-crossing dwell + reset-guard armor
during ramp-through-zero (no decel/coast anticipation ported this sprint — Open
Question 1). Verbs complete and emit their events at the target; terminal precision
is the deferred refinement. Filed as a follow-up issue at sprint close.

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
