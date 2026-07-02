# Evidence extract: Sprints 020–029 (sim-first build → field crisis → recovery roadmap)

(Compiled by a reader agent. Quotes verbatim from artifacts.)

## 020 — HAL Abstraction, Motion Overhaul, Command Queue (12 tickets; largest of batch)
- Motion system built in 017–019 had already forked: "two parallel code paths to the motor (legacy bypass vs BVC), duplicated keepalive watchdogs."
- **Planted two seeds of the 026 crisis**: (1) 020-011 put S/T/D/G/R/TURN→VW converters inside MotionController — the layering inversion later named A2, root cause of the double-OK defect; (2) 020-003/004 built `sim_api.cpp` **without wiring the CommandQueue** built in 020-010 — the sim/hardware dispatch split later called "the single largest reason 'it works in sim and fails on the field'". Queue and sim wrapper built in one sprint, never connected.

## 021 — Mock noise model + figure-eight demo
- Noise model got the **turn-slip sign wrong** (discovered sprint 024) and shipped **off by default** — 027: "'Passes in sim' has meant almost nothing."

## 022/023 — EKF pose + velocity fusion
- Heading fusion deferred TWICE ("OTOS heading is not fused... out of scope"); 024's D1 traces "gets turned around and drives into the boards" directly to this gap.
- Mahalanobis gate landed with no recovery path ("confidently wrong, forever" divergence trap D3).
- 023 test strategy: "Primarily offline… On-robot bench verification via rogo is optional and deferred" — three consecutive estimator sprints with essentially no hardware validation.
- 023 fixed 022-era bugs: setPose() spurious-jump; "the `OV` command comment says 'report velocity' but it actually calls `setPositionRaw()`".

## 024 — Field-safety P0 fixes (pure defect sprint, D1–D5,D7 from external review)
- Sources: `docs/code_review/2026-06-11-sim2real-architecture-review.md` and `2026-06-11-wild-spin-and-cursing-forensics.md`. Field failure: "robot goes wild and spins until I power it off".
- D2: "`rotationalSlip` (default 0.74)… is referenced in **zero firmware logic**" — calibrated, stored, registered, dead.
- D4: operator workarounds (SAFE off; a daemon streaming `+` every 150ms) "demoted the watchdog from motion-supervisor to dead-process detector".
- **Sprint failed its own field test the day it shipped**: "implemented and all 1434 host/dev tests pass, but the on-field behavior is not fixed" — same-day run "produced a full-speed spin that ended with the robot jammed into the boards. Same class of failure the sprint set out to fix" (recorded in 027's issue).
- `sTimeout=60000` overrides scattered in test fixtures were masking watchdog behavior — test scaffolding hiding the defect it should have caught.

## 025 — Trustworthy host I/O
- Headline bug corrupted ALL prior observability: "`SerialConnection.send()` calls `reset_input_buffer()` before every write, discarding in-flight TLM frames, EVT done lines, and safety_stop events… the primary cause of 'the stream keeps dying'."
- "Fixing observability before changing behavior avoids chasing ghosts in a broken stream."
- 025–029 planned together as one recovery roadmap with "Why First / Second / Third / Fourth / Last" sections.

## 026 — One dispatch path ("highest-risk sprint in the roadmap")
- "`host_tests/sim_api.cpp` never wires a CommandQueue… One OK reply. D6/D11 simply don't exist [in sim]."
- "`sim_api.cpp` **hand-mirrors** the LoopScheduler loop with a 'MUST mirror LoopScheduler.cpp exactly' comment — a divergence generator by construction."
- Verdict on the 020–023 test strategy: "**Every sim test validates a system that does not exist on hardware**."
- Issue names stakeholder frustration verbatim: "the direct cause of the repeated 'go run the actual simulator on our code' frustration."
- MotionController.cpp 1953 → ~900 lines.

## 027 — Behavioral fixes on the single path
- D6: the documented API itself taught the destructive pattern — host docstrings *recommended* the keepalive that stomps active commands; "firmware emits `EVT done TURN` as if it succeeded."
- Field-profile issue: "the sim validates a friendlier system than reality: OTOS/EKF fusion off by default, MockMotor slip = 0… 'Passes in sim' has meant almost nothing — and is the source of the recurring 'you didn't actually test it on our code' frustration."
- Operator quoted: "your program is supposed to detect problems… run it for a little bit and stop… forces the operator to lunge for the power switch."
- 027-006 finally root-caused the field-024 SNAP anomaly as benign tick-ordering after it consumed diagnostic effort across three sprints.

## 028 — Calibration and host consolidation
- "Calibration logic exists in four places… calibration outputs were calibrated, stored, and registered — but read by nothing in firmware." A7: "already cost real field-debugging sessions."
- a6 names an AI-specific failure mode: "the MCP path the main agent uses and the CLI path a human uses are not the same code… CLI/MCP drift directly causes 'works for the human, fails for the agent' reports." (left in-progress into 029)
- D10: telemetry "fights its consumers" — silent-idle by design, radio commands steal the stream, no seq numbers.

## 029 — Navigation ownership
- "The same closed-loop 'drive to a world point' capability exists in three stacks with no shared code, parameters, or pose state" + "three pose estimators with no defined authority… Every navigation bug must be hunted in three stacks."
- Host stacks existed because the firmware path was untrusted: "Until sprints 025–027 prove the firmware G path trustworthy on the field, consolidating onto it risks consolidating onto a broken target."
- Human wired in as hard gate: "If no agreement is reached, the sprint is blocked — do not proceed with implementation under ambiguity."
- Later history: a1 consolidation needed sprint 035 to fully land (029 tickets closed unimplemented).

## Batch synthesis (verbatim from reader)
Sprints 020–023 were a rapid, sim-first capability build executed almost entirely offline, planting three fatal seeds: queue and sim never connected; noise model off-by-default with wrong slip sign; heading fusion deferred twice plus a gate with no recovery path. The bill came due on 2026-06-11 ("wild spin and cursing"); 024–029 is the repayment, planned as one roadmap. Sprint 024 fixed all six confirmed root causes, passed 1,434 tests, and STILL spun full-speed into the boards the same day — forcing the lesson that observability (025), path unification (026), and test-profile realism (027) had to precede behavioral trust. Duplication was consistently a symptom of distrust: host navigators grew because firmware wasn't believed; keepalive daemons and sTimeout fixtures grew because the watchdog killed legitimate motion — each workaround then masked the defect it routed around. rotationalSlip resurfaces in project memory 2026-07-02 ("SET rotSlip silently no-ops for turns") — the config-consumption defect class outlived the a8 lint built to kill it.
