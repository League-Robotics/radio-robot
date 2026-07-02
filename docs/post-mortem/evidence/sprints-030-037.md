# Evidence extract: Sprints 030–037 (external review + bench reckoning)

(Compiled by a reader agent. Quotes verbatim from artifacts.)

## 030 — Fable round-2 correctness fixes N1–N16 (10 tickets)
External review (`docs/code_review/2026-06-12-Fable-correctness-review/findings.md`) found what the per-ticket sim-verified process structurally missed — cross-cutting invariant violations:
- **N2 (most damning)**: "sim wires+tests the queue path; firmware runs the immediate path… After the first safety stop/halt the firmware permanently switches to queued dispatch." Entire sim regression suite validated a code path the hardware never ran.
- N1: "every `D` teleports the fused pose backward by the prior segment's length."
- N3: "`SET tlmPeriod=100` with no prior STREAM → null fn-pointer call → HardFault"; header comment "describes a guard nothing implements."
- N4/N5: 4 of 7 begin*() entry points skipped cancel-if-active; "P1.1's own verify scenario... fails here" — an earlier sprint's acceptance scenario never actually passed.
- Silent classes: queue overflow after OK (N7), sticky validity bits (N8), `SET aDecel=-100` → NaN (N6), dead RatioPidController "constructed, SET-tunable, never run".
- Verification remained sim-only; 033 later found 030's N1 fix **incomplete** ("the snapshot still precedes the input zeroing").

## 031 — Bench OTOS debug sensor
Planted three bugs/debts that consumed 032–034: (1) `DBG OTOS BENCH` enable shipped broken (union-aliasing clobber — parser wrote `.ival` then zeroed `.fval` on the same union); (2) `DBG OTOS` used `%f` which prints NOTHING on newlib-nano, invisible in host sim ("host sim's libc has full %f"); (3) "chose the fastest path to a working feature: Robot::benchOtosTick downcasts hal to NezhaHAL*" — violating HAL-agnosticism, reworked in 034. Hardware bench execution explicitly out of scope, "deferred to post-sprint team-lead validation."

## 032 — Comprehensive bench validation
- Premise concedes the gap: "Sprint 030 and 031 delivered significant firmware changes... None of these were hardware-validated after the merge."
- Run confused by FIVE compounding bugs: wrong transport, bench-enable never engaging, `twist=` permanently zero (since 023!), D-after-TURN instant-complete, ambiguous wedge detector.
- The validation harness itself "parses TLM integers with wrong unit assumptions, producing absurd million-degree heading values and meaningless assertions."
- ALL-CAPS escalation: "STAKEHOLDER DIRECTIVE — bench testing uses the SERIAL PORT, not the radio... it is the root of most of this session's confusion" (`docs/code_review/bench-032-diagnosis.md`).
- False hardware accusation refuted: "I named the encoder before verifying with an equal-wheel test" (resolution: refuted).

## 033 — Bench-found firmware fixes
- Fixes 031 (union bug — bench mode never worked since it shipped), completes 030's N1, fixes 023 (`twist=` zero on hardware since EKF velocity fusion landed; "including any real-world OTOS dropout").
- Recorded dead-end spiral: "an earlier attempt reached for objdump and a 'nRF52 pointer-comparison' theory — both dead ends. The bug was found by reading the parser and adding one en=%d probe."
- Post-fix hardware validation succeeded (8/8) but immediately opened NEW findings (F1 float-printf) feeding 034. Fix-validate-find-new-bugs became the loop.

## 034 — Push actuator state through Hardware::tick (rework of 031)
- Issue headed "Stakeholder design direction (Eric, 2026-06-12)" — the human personally specified the target architecture.
- "The F1 integer-format fix is NOT verifiable by the host sim (host libc has full %f)" — on-hardware verification by stakeholder required.

## 035 — Pose authority consolidation (A1)
- All three issues: "Provenance: sprint 029 (navigation-ownership) ticket 002, closed unimplemented" — an earlier sprint's tickets closed without being done, resurrected here.
- "Firmware fixes from sprints 024–027... have no effect when an agent uses the host-side navigator or CLI inline controller" — months of firmware fixes bypassed by parallel host code.
- Deleted: pure_pursuit.py (216 lines), stanley.py (198), ltv.py (293); navigator.py 1349→~400 lines.
- Hard gates: "Do not begin until the stakeholder-approved design doc explicitly authorises deletion of the specific files."

## 036 — Stateful Robot object + Playfield
- Bench validation found **basic host-library functions had never worked against real hardware**: `get_id()` always returned None (reader dropped the ID reply); `refresh()` returned None (snap waited on a corr-id the reply never carries). "Both bugs exist on master." Tests mocked at layers that encoded the same wrong assumptions.
- T007 self-correction: "Corrected root-cause analysis (supersedes earlier ticket draft): dtr=False does NOT suppress/mute the relay" — first AI diagnosis wrong; knowledge note "contains a now-disproven claim."

## 037 — Consolidate tests into one tree
- "Three separate test roots force constant guessing"; circular-mean helper "duplicated in playfield_tour_camera.py, playfield_random_tour.py, and tests/playfield_tour/tour_goto.py."
- Five tour-script variants retired at once, incl. one sprint 036 had just rewritten.
- Only sprint of the eight with all Definition-of-Ready boxes checked.

## Batch synthesis (verbatim from reader)
This era is the project's reckoning with the sim-to-hardware gap; its shape is a correction loop rather than a line. Roughly half the era's engineering is rework of the era's or earlier eras' own output. Seeds visible at close: hardware verification chronically deferred to "post-sprint team-lead" checkboxes left unchecked (031, 033, 036, 037); Definition-of-Ready stakeholder-approval gates skipped in five of eight sprints; a knowledge base already carrying one "now-disproven claim"; a validation strategy that still trusts a sim whose libc, transport, and dispatch path have each been proven to diverge from the device.
