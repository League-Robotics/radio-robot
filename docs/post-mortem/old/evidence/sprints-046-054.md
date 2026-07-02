# Evidence extract: Sprints 046–054 (consolidation-after-construction at extreme velocity)

(Compiled by a reader agent. Quotes verbatim from artifacts.)

## 046 — Mecanum drivetrain (8 tickets)
- Planned on the premise "A second robot is now on the bench" — but both HITL tickets sit in tickets/done/ with **every hardware acceptance checkbox unchecked**; commit 71868df: "HITL mecanum calibration deferred to follow-up issue… (blocked: no mecanum robot on the bench)".
- The `#ifdef ROBOT_DRIVETRAIN_MECANUM` design "metastasized to **81+ sites across ~15 files**… The abstraction it was meant to localize (IKinematics.h, sprint 046) leaked."
- A full omnidirectional drivetrain shipped "done" **without a single motor ever spinning on real hardware**; the integration was deleted 5 days later in 048 — and mecanum was later rebuilt (togov is a live mecanum robot today): a complete build→delete→rebuild cycle.

## 047 — Robust robot state object
- Positive collaboration example: issue "intended for your review before any code is written," five design questions individually resolved ("Resolved decisions (stakeholder, 2026-06-27)").
- Motivation: the team didn't trust their own fusion — "The pre-fusion dead-reckoned pose is discarded"; "poseX is actually the EKF output, not raw dead-reckoning."

## 048 — (dir name is legacy) Eliminate ROBOT_DRIVETRAIN_MECANUM entirely
- A sprint that was itself re-scoped: original plan ("kinematics namespace alias") SUPERSEDED because it was "cleanup of the design introduced in sprint 046" that still didn't meet its goal.
- Stakeholder correction recorded: "The stakeholder wants the macro gone **completely**… 1. **Supersede sprint 048** — don't do the partial refactor then immediately redo it. 2. **Compile differential-only now.**" A fully ticketed sprint abandoned before execution.
- Even the deletion needed a redo: commit ef6b1fd "strip residual control-layer mecanum sites **missed in first pass**".

## 049 — Consolidate PID onto cmon-pid
- `RatioPidController` (sprint 004) tabled as "**Dead** — update() never called" — silently orphaned by architecture churn; 049 just buried it.
- Tooling landmines embedded in sprint.md: "Do NOT use bare `uv run pytest` — falsely reports mass failures."
- First appearance of the normalized broken-window baseline: "Known pre-existing baseline: exactly 2 failures" — waiver copy-pasted into every sprint through 053.

## 050 — Replace EKF with TinyEKF (parity-gated)
- Fact-check found TinyEKF "provides **only** the bare predict/update linear algebra… **Our EKF's hard-won robustness — χ² gating per channel, D3 gate-recovery, wedge-aware omega suppression… is exactly what TinyEKF lacks.**" The bespoke code survived because it encoded field experience no library had; only matrix arithmetic was swapped.
- Model for safe replacement: keep-old-run-new-at-parity-then-delete — the opposite of the 046 pattern.

## 051 — Declarative ArgSchema layer (9 tickets)
- **Key process failure of the batch**: validation ticket 009 checked `[x]` the spot-check "`S 99999` → `ERR range l`" — yet sprint 054's bench run on real firmware got "`ERR badarg l`". Root cause per 054: "the simulation tests… **used static string literals, not live firmware calls**, so the regression passed CI." A green validation checklist disproven by hardware five days later.
- 053's validation ticket warns: "Check for any ARM-specific compile errors not caught by the host sim build (**this has bitten the project before — sprint 051**)."
- (Also introduced, found in 064: query-mutates-state on DBG IRQGUARD and `RF` silently retuning the radio to channel 0.)

## 052/053 — Stop conditions Phase 1/2
- Debt being redone is named: "leftover scaffolding from incremental 'behavior-preservation' seams (sprints 026/042). It is exactly the mirroring we want gone."
- 053 deliberately rebaselined the golden-TLM canary ("must be reviewed — not blindly accepted") — the primary behavior-preservation instrument invalidated by design during the refactor.
- Pace: 051+052+053 (20 tickets) all executed ~13:00–16:31 on 2026-06-28; 052 in ~30 minutes; DoR "Stakeholder has approved" unchecked in closed artifacts.

## 054 — ERR range vs badarg fix
- "Found during **post-roadmap bench validation of sprints 048–053**" — five refactor sprints ran on sim-only gates; first bench contact immediately found a wire regression.

## Batch synthesis (verbatim from reader)
Of 9 sprints (51 tickets), only 046 and 052 add capability; the rest refactor, replace, or repair. The two defining failures are both validation gaps, not coding gaps. The middle sprints show the process at its best (047's reviewed design; 050's parity gate). But the June-28 blitz outran the test oracle: the era's motto of "byte-identical behavior" was enforced by oracles that 053 had to deliberately re-baseline and 054 had to substantially rewrite — refactoring safety was only as good as sim fidelity, and hardware kept issuing corrections the moment it was consulted.
