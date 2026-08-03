---
status: done
---

# Land the systest firmware changes: DBG inbound arm, wedge injection, sim cleartext plane

## Description

UPDATE (2026-08-01, stakeholder-directed): master had not moved since the
branch point, so the whole feature INCLUDING the firmware changes was
merged to master conflict-free (fast-forward to `d08264dc`). This issue
is now the REVIEW record for those firmware changes — verify the landed
code against the rationale below (it merged without a second reviewer) —
plus the follow-ups in the landing notes.

The system-test feature was built on branch `systest-feature` and
verified on the sim tier before merging. The firmware half is the
`d08264dc` commit (213 insertions, 7 files).

Verified working end to end on sim before handoff:

- `fault_wedge.tour` 13/13: inbound `DBG:wedge left 1500` raises telemetry
  flag bit 7 within one cycle, `DBG:clear` clears it, and the dataset
  shows the wedge window exactly bracketed by the `mark inject` /
  `mark recovery` echo records (seq 93 < 95..132 < 149 < 168).
- `DBG:ping` → `DBG:pong`; `SEND STATUS` → the full STATUS reply lands as
  a queryable dataset record and `EXPECT '.payload.ready==1'` passes.
- The square tour runs 20/21 steps; its one failure is REAL (see
  Verification note below).

## Cause

The system test needs three capabilities firmware did not have:

1. **Inbound DBG commands** (fault injection + in-band dataset markers).
   The DBG verb existed robot→host only; `dispatchCleartext()` routed an
   inbound DBG to the malformed count.
2. **A way to induce a wedge.** Firmware could only detect faults, never
   cause them; the fault tour needs the latch raised on demand, on sim and
   hardware identically (the sim runs real firmware, so a firmware-level
   override serves both tiers).
3. **The whole cleartext plane in sim datasets.** `sim_drain_debug()`
   filtered `drainReliable()` down to `DBG:` lines, silently discarding
   READY/STATUS/PONG/DEVICE — so a sim dataset could never carry the
   records a hardware dataset does, and `SEND STATUS`+`EXPECT` could not
   work.

## Proposed fix (= the patch, file by file)

- `src/firm/app/comms.h` — `DbgActionKind`/`DbgAction` + a **4-deep FIFO
  ring** (`pushDbgAction`/`takeDbgAction`). Deliberately NOT TlmAction's
  collapse-to-last: a tour legitimately sends `DBG:mark X` then
  `DBG:wedge ...` in one cycle, and the collapse silently lost the mark
  (caught by the dataset on the first fault-tour run).
- `src/firm/app/comms.cpp` — `classifyDbgArg()` (grammar: `mark <text>` /
  `ping` / `wedge left|right|both [ms]` / `clear`) + a `Verb::DBG`
  interception in `dispatchLine()` mirroring the TLM pattern, guarded by
  `#ifdef ROBOT_DEBUG` — a shipped image compiles the whole surface out
  and inbound DBG stays a malformed count there. The tokenizer treats ALL
  whitespace as separators (a trailing `0x0A` reached the parser via the
  sim inject path and made `clear` parse as `clear\n` — also caught by
  the dataset).
- `src/firm/app/robot_loop.{h,cpp}` — `applyDbgAction(now)` drained at
  the top of the cycle beside the TLM action: `mark` echoes verbatim
  through `App::debugf()` (markers land in the robot's own output stream,
  correctly ordered against telemetry), `ping` → `pong`, `wedge` forces
  the armor latch with a duration deadline (`UINT32_MAX` = latched until
  `clear`), expiry checked each cycle so a timed wedge clears even with
  no further DBG traffic.
- `src/firm/devices/motor.h` — `virtual void setForcedWedge(bool) {}`
  (default no-op).
- `src/firm/devices/motor_armor.h` — the override: `forcedWedge_` OR-ed
  into `wedged()`, so every consumer (health publication, flag bit 7,
  Drive's wedge handling) sees an induced wedge exactly as a real one;
  the real detector's own latch state is untouched.
- `src/sim/sim_ctypes.cpp` — `sim_drain_debug()` returns the WHOLE
  reliable cleartext plane; the Python side
  (`SimLoop._drain_debug_into_queue`, already committed on the branch)
  routes DBG to the debug queue and everything else to the new
  `on_cleartext` observer, so `drain_debug_lines()`'s DBG-only contract
  is preserved for existing callers.

Landing notes for the reviewer:

- The interception mirrors TLM rather than widening
  `dispatchCleartext()` to carry a data pointer. Widening is the better
  long-term shape (and would absorb the TLM special case too) — a
  follow-up, not a blocker.
- `HELP` walks the verb table, so DBG appears in HELP output on debug
  builds — intended.
- Known sim-inject quirk (host-side workaround already committed): the
  sim inject path treats the injected buffer as one line, so the host
  must NOT append `0x0A` — with a terminator, a bare verb like `STATUS`
  fails the registry lookup entirely. Worth a look at whether
  `SimHarness::injectCommand`'s framing should strip a trailing
  delimiter for hardware parity.

## Verification

From the worktree (or after applying the patch anywhere):

```bash
cmake -S src/sim -B src/sim/build && cmake --build src/sim/build -j8
uv run python src/tests/system/systest.py run --tier sim src/tests/system/tours/fault_wedge.tour
```

Expect 13/13 including both EXPECT lines (flag bit 7 rises after
`DBG wedge left 1500`, absent after `DBG clear`). Then:

```bash
uv run python src/tests/system/systest.py run --tier sim src/tests/system/tours/square.tour
```

Expect 20/21 with ONLY the final CAMFIX failing (~361 mm closure error):
that failure is genuine — the sim's turns undershoot ~10° each (80.0°,
159.5°, 240.9°, 321.9° at the four boundaries vs 90/180/270/360,
measured from the run dataset's per-boundary marks), corroborating
[`sim-tour-turn-shaping-undershoots-90-degree-turns.md`](sim-tour-turn-shaping-undershoots-90-degree-turns.md).
Do NOT tune the tour to make it pass; fix the undershoot.

Golden machinery (already demonstrated; goldens deliberately left
uncommitted — nothing self-blesses): `bless` a square dataset, `compare`
a no-change repeat (10/10 green, deviations ≈ 0 in the deterministic
sim), `compare` a +30 mm/s perturbed run (5/10 red, with exactly the
commanded/measured/heading signals failing and the path-shape signals
green).

## Related

- [`system-test-minimal-system-test-one-program-tour-files-one-jsonl-dataset-dbg-fault-injection.md`](system-test-minimal-system-test-one-program-tour-files-one-jsonl-dataset-dbg-fault-injection.md)
  — the charter this implements. Divergence from its plan, decided during
  implementation: the runner is a purpose-built lean executor (ack-ring
  completion + one-leg lookahead, ~same policy as `run_tour()`) rather
  than a `MoveStep` generalization of `planner/tour.py` — `run_tour()` is
  entangled with `PlannerParams`/`HeadingCorrector`, and a tour file must
  stay a pure protocol script with no host-side heading trim.
- [`sim-tour-turn-shaping-undershoots-90-degree-turns.md`](sim-tour-turn-shaping-undershoots-90-degree-turns.md)
  — the bug the square tour now catches mechanically, with per-boundary
  heading evidence in every dataset.
- Sprint 129's DBG channel (129-003) is the outbound half this builds on.
