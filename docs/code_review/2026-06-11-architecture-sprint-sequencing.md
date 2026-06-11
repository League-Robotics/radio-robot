# Sprint Sequencing for the Architecture Issues (A1–A8)

**Date:** 2026-06-11
**Audience:** team-lead agent / sprint planner.
**Inputs:** issues `a1`–`a8` in `.clasi/issues/` (a4 was absorbed into the existing
`sim-runs-real-dispatch-path`), plus the pending D-issues from the sim2real review.
**Assumption:** the P0 fixes (heading fusion, watchdog, PRE_ROTATE supervision,
rotationalSlip) are implemented; remaining D-issues are d06, d08, d09, d10, d11,
d11a, d12 and the harness/process issues. If that's wrong, P0 items still come first.

The ordering principle: **make evidence trustworthy before changing behavior, make
the dispatch path single before fixing dispatch bugs, make structure right before
consolidating features onto it.**

---

## Sprint 1 — Trustworthy host I/O (small, all host-side)

**Goal:** every subsequent test and field report can be believed.

1. `d11a` — single reader thread, stop clearing the input buffer.
2. `a5-serial-transport-encapsulation` — immediately after d11a, same sprint;
   without it d11a's guarantees can be bypassed by any `_ser` reach.
3. `a8-config-registry-sync-lint` — independent half-day filler; lands the lint
   that A7 and the calibration work depend on.

**Exit criteria:** stream survives idle→drive→idle + command bursts with zero lost
EVT/TLM; `_ser` unreachable outside `io/serial_conn.py` (CI-guarded); config lint in
CI and green.

**Why first:** cheap, no firmware risk, and every later sprint's verification runs
through this plumbing.

## Sprint 2 — One dispatch path (firmware refactor)

**Goal:** sim and hardware run the same code; one reply per command — by
construction, not by patch.

1. `sim-runs-real-dispatch-path` (P1.3, including the `tickOnce()` extraction).
2. `a2-protocol-out-of-control-layer` — the structural fix.
3. `d11-single-ok-per-command` — falls out of A2; keep its test as the acceptance
   gate rather than implementing its `quiet=true` patch separately.
4. `a3-split-motioncontroller-and-robot` — not a separate task: write A3's file-size
   and separation targets into A2's review criteria.

**Exit criteria:** no `CommandProcessor.h`/`CommandQueue.h` includes in `control/`;
no hand-mirrored loop ("MUST mirror" lint); D11 test passes **in sim**; full
hardware smoke ritual passes after flash.

**Why second:** this is the biggest-risk sprint; Sprint 1 gives it a trustworthy
test harness, and everything in Sprint 3 needs the single path to be testable.

## Sprint 3 — Behavioral fixes on the single path

**Goal:** the remaining field-behavior defects, now reproducible in sim.

1. `d06` keepalive-must-not-mutate (its sim test requires the queue-wired path).
2. `d08` pursuit-law hardening.
3. `d09` OTOS validity gating.
4. `field-profile-test-harness-and-ci` if not already landed; the four incident
   scenarios become named regression tests here.

**Exit criteria:** the §4 scenario tests from the sim2real review pass in the field
profile; hardware smoke ritual green.

**Why here and not earlier:** sequencing these before Sprint 2 means writing tests
against a dispatch path that's about to be deleted.

## Sprint 4 — Calibration and host consolidation

**Goal:** one calibration pipeline whose outputs all land somewhere.

1. `a7-consolidate-calibration` (lint from Sprint 1 already enforcing
   every-key-is-read).
2. `a6-extract-library-logic-from-cli` — the calibration-push and `_make_robot`/
   port-resolution pieces; full A6 completion can trail into Sprint 5.
3. `d10` trustworthy telemetry stream (remaining firmware-side items: seq numbers,
   idle rate, channel binding) — fits here; host side was Sprint 1.

**Exit criteria:** one calibration package; CLI and MCP front-ends call the same
library functions for everything they share; telemetry drop rate measurable.

## Sprint 5 — Navigation ownership (A1)

**Goal:** one go-to-point implementation per regime, one pose authority.

1. **Decision first** — a short design doc (stakeholder sign-off) assigning
   ownership: suggested split is firmware owns short-horizon motion + pose fusion,
   host owns route planning + camera corrections as pose resets. The sprint planner
   should treat the decision as a deliverable, not a given.
2. Fold `cmd_goto`'s inline controller into `nav/` (this item may be pulled into
   any earlier sprint — it's independent).
3. Delete/demote redundant controllers and pose trackers per the decision; document
   pose authority in `docs/architecture.md`.

**Exit criteria:** exactly one implementation per regime; no control loops in
cli.py; written pose-authority statement matches the code.

**Why last:** consolidating navigation onto the firmware G path only makes sense
after Sprints 2–3 have proven that path on the field. Doing A1 earlier consolidates
onto an unproven target.

---

## Dependency summary

```
S1: d11a → a5        a8 (independent)
S2: P1.3 + a2 (+a3 as criteria) → d11
S3: d06, d08, d09    (require S2's single path for sim repro)
S4: a7 (requires a8), a6 (partial), d10-firmware
S5: a1 decision → a1 consolidation (requires S2–S3 field-proven)
Anytime: cmd_goto → nav/ fold-in; d12 hygiene items as fillers.
```

Rough sizing: S1 small (1 session), S2 large (2–3 sessions, highest risk), S3
medium (2), S4 medium (1–2), S5 medium plus a decision gate.
