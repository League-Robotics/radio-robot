# Field Log

Hardware smoke-ritual results logged here after each sprint acceptance run.
One entry per run, with date, git SHA, step results, and anomaly notes.

---

## Sprint 026 — one-dispatch-path

**Date:** 2026-06-11
**Git SHA:** deb3c29156c7139a3dd9d0fd35544d4d5fd07254
**Ritual script:** `tests/bench/smoke_ritual.py`
**Flash command:** `mbdeploy deploy robot --clean`

| Step | Name               | Result                                | Notes |
|------|--------------------|---------------------------------------|-------|
| 1    | Safety check       | PENDING — stakeholder field test      |       |
| 2    | TURN ×4 closure    | PENDING — stakeholder field test      |       |
| 3    | G square           | PENDING — stakeholder field test      |       |
| 4    | No double-OK       | PENDING — stakeholder field test      |       |
| 5    | Stream aliveness   | PENDING — stakeholder field test      |       |

**Overall:** PENDING — reserved for stakeholder field test.

To run the ritual:

```
mbdeploy deploy robot --clean   # flash first
uv run python tests/bench/smoke_ritual.py --port /dev/cu.usbmodem<N>
```

Ticket 026-004 will be updated to PASS once all five steps are confirmed on
the real robot. See ticket for acceptance criteria.

## 2026-06-20T01:07Z  sha=ba14700  overall=FAIL
- Check 1 (Safety check): PASS
- Check 2 (RT x4 closure): FAIL
- Check 3 (G square (200mm)): FAIL
- Check 4 (Lift test (EVT otos lost)): SKIP
- Check 5 (TLM drop-rate): FAIL

## 2026-06-20T04:04Z  sha=874d678  overall=PASS
- Check 1 (Safety check): PASS
- Check 2 (RT x4 closure): PASS
- Check 3 (G square (200mm)): SKIP
- Check 4 (Lift test (EVT otos lost)): SKIP
- Check 5 (TLM drop-rate): SKIP

## 2026-06-20T04:05Z  sha=874d678  overall=FAIL
- Check 1 (Safety check): SKIP
- Check 2 (RT x4 closure): SKIP
- Check 3 (G square (200mm)): FAIL
- Check 4 (Lift test (EVT otos lost)): SKIP
- Check 5 (TLM drop-rate): SKIP

## 2026-06-27T05:13Z  sha=60baf5e  overall=FAIL
- Check 1 (Safety check): PASS
- Check 2 (RT x4 closure): SKIP
- Check 3 (G square (200mm)): SKIP
- Check 4 (Lift test (EVT otos lost)): SKIP
- Check 5 (TLM drop-rate): FAIL

## 2026-06-27T05:43Z  sha=60baf5e  overall=FAIL
- Check 1 (Safety check): PASS
- Check 2 (RT x4 closure): PASS
- Check 3 (G square (200mm)): FAIL
- Check 4 (Lift test (EVT otos lost)): SKIP
- Check 5 (TLM drop-rate): SKIP
