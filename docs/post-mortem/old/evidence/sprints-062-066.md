# Evidence extract: Sprints 062–066 (TestGUI era + regression archaeology)

(Compiled by a reader agent. Quotes verbatim from artifacts.)

## 062 — TestGUI (PySide6) + baseline fixes (10 tickets)
- The two "baseline fix" tickets cleared CI failures **tolerated as known noise for ~8 sprints (054→062)**.
- Stakeholder gate on golden refresh: "do NOT rubber-stamp the snapshot."
- The GUI's drive design ("repeatedly send VW ±v 0 on a ~100 ms QTimer (doubles as keepalive)... On release → send STOP") is exactly the pattern sprint 065 later classifies as safety defects CR-04/05 — new capability shipped with latent safety debt.

## 063 — Mode-driven TestGUI (11 tickets; planned 3, grew to 11 mid-sprint)
- "Ship, operate live, file issues, extend sprint" loop: tickets 004–006 from new stakeholder requests; 007–011 from live bugs.
- **Knowledge base ignored → same bug re-solved**: ticket 002's relay probe "re-introduced the passive-banner assumption" already refuted in `.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md`; ticket 010 redid it with HELLO-classify "which is what SerialConnection already does."
- **Threading bug pattern hit twice**: "a QueuedConnection to a non-QObject callable is delivered on the worker thread (the same behavior that caused the tour/GOTO segfault)."
- Fix unmasked half-finished design: repairing frame delivery exposed the avatar TLM-vs-camera fight ("jumps all over the place").
- **Out-of-process work introduced defects the sprint absorbed**: "Introduced with the Tour feature (out-of-process work on 2026-07-01)."
- 062's `_set_origin` button was "display-only" — looked functional, sent no wire command.

## 064 — Encoder pipeline hardening (6 tickets)
- **The long wedge arc (015→033→051→060→064)**: 015 IRQ guard; 033 wedge detector + EKF gating; 051 ArgSchema migration silently broke the guard's query ("a bare DBG IRQGUARD query silently disables the guard"); 060 cutover silently dropped the outlier filter's recovery; 07-02 stand session found **two new triggers the guard never covered** and "**EVT enc_wedged fired for NONE of ~18 episodes**" — ten sprints of defenses, 0% detector recall on real episodes.
- The tooling sabotaged its own experiment: "the harness preflight queried the guard and thereby disabled it" — contaminating the stand-repro baselines.
- Audit found a worse sibling: a bare `RF` "silently retunes the radio to channel 0 and persists it to flash, breaking the link."
- Audit also found: "every D command currently fires the full hardware burst twice" — flagged as Open Question, not silently fixed.
- Human ran a controlled 5-arm stress matrix on the stand; sprint plan cites arm numbers per fix. DoR: "auto-approve session."

## 065 — Stop reliability and safety (ACTIVE; 001–005 done, 006 open)
Three defects, three provenances:
- **CR-01 (new-architecture integration defect)**: stop-clause double-booking between Planner::beginDistance and Superstructure::requestGoal → assert(false) "aborts the whole Python process hosting the sim."
- **CR-04/05 (long-standing, amplified)**: "the same 'watchdog silenced by keepalives' mechanism from the June wild-spin postmortem, now structural"; exposure amplified by 062's KeyboardDriver over a link that "drops 15–50% of lines."
- **CR-06 (regression of an older fix)**: "A 2026-06-17 change set healthy = poseOk… reopening the exact 'spin on placement' failure the original D9 gate (027-005) existed to prevent… the implementation lost the transient-vs-persistent distinction."

## 066 — Sim fidelity (PLANNED, roadmap only)
- "the sim OTOS can never disagree with the encoders except via injected noise (so EKF fusion is validated in a regime that doesn't exist on hardware)"; past regression db11b7c (433mm phantom translation on a pure spin) had "zero sim coverage" — success criterion: "a db11b7c-style regression now fails in sim."
- "Existing tests that relied on OTOS==encoders may need updating — that agreement was the bug."
- Encoder-track bug on its **third iteration**: "the original 'encoder track ignores turns' bug survives on exactly the transport (relay/playfield mode) where it matters."

## Synthesis (verbatim from reader)
This era is the bill coming due after the architecture program (055–061), paid down via a new feedback instrument (the TestGUI) plus two deliberate audit events. The moment a human operated the GUI live, defects poured out (063 grew 3→11 tickets). Rework here is overwhelmingly **regression archaeology with explicit provenance** (lost in 060 cutover; arrived with 051 ArgSchema; reopens D9/027-005; re-introduced an assumption already refuted in knowledge). The era's distinctive process moves: a full-codebase review generating a numbered CR-01..15 backlog scoping sprints 065–066 wholesale; issues written with confirmed file:line mechanisms before ticketing; auto-approve sessions replacing per-sprint sign-off while HITL validation is consistently deferred to the human; and a recurring failure shape — fixes validated in sim or on fast links that fail on hardware or the 1–2 Hz relay. The AI's leverage is exhaustive audit; the human's leverage is live operation, bench experiments, and SSOT/safety adjudication.
