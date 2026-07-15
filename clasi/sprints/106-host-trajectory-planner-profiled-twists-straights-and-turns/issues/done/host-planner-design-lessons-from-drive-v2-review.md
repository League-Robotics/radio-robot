---
status: done
sprint: '106'
tickets:
- 106-001
- 106-002
- 106-003
- 106-004
- 106-005
- 106-006
---

# Host planner: design requirements carried from the drive/ v2 review

The single-loop rebuild deletes `source/drive/` and moves trajectory planning to the
host (`host/robot_radio/` — nav/, path/, controllers/ survive as its seed). The
2026-07-13 code review of drive/ v2 (docs/code_review/2026-07-13-devices-drive-review.md,
Part 2) found eleven critical/major defects. Most die with the deletion — but they are
the failure modes of *this problem domain*, not of that code, and the host planner will
face every one of them again. This issue is the checklist so they are designed out, not
re-invented.

## Requirements for the host planner (each traces to a Part 2 finding)

1. **No direction-blind completion or admission checks** (findings #8, #11; also the
   historical DISTANCE-fabsf bug): every reachability/completion predicate must be
   sign-aware in the segment's travel direction. Grep-level rule: no `fabsf` on a
   signed velocity/displacement in a predicate.
2. **No silent drops** (#7, bench-confirmed as the 2026-07-14 "ACKed but never
   executed" evening): any plan/solve failure must produce a visible, host-logged error
   tied to the command that caused it. The robot's ack ring + fault bits make the
   firmware side loud; the host planner must be equally loud internally.
3. **Clock discipline across replans** (#2): any rate-limit/timeout state must be
   rebased when a plan clock rebases — or keep one segment-global clock. A replan must
   never inherit a stale timestamp that blocks the next replan.
4. **Preemption invalidates chain state** (#5, #6): flushing a queue must re-anchor the
   admission tail and clear carried entry speeds. Test: queue a chain, preempt it,
   queue another — second chain must plan from measured state.
5. **Validate wire inputs** (#9): reject non-finite/absurd values at the boundary;
   never rely on a downstream solver's validate() as the only guard.
6. **Bound overshoot in completion** (#10): "arrived" must have an outer bound in both
   directions; a large overshoot is a failure, not a success.
7. **Terminal-phase care** (#11, #13 + wedge history): no zero-dwell wheel reversals at
   segment boundaries in either travel direction; terminal mode selection from plan
   intent, not instantaneous reference speed.
8. **Latency is a first-class parameter** (#17): the ~120-140 ms actuation lag must be
   an explicit, configurable model input, not constants smeared into envelope margins.
9. **Everything tunable live** (#3, #4, the 098 lesson): every gain/threshold the
   planner uses must be adjustable without redeploying anything — this is the entire
   reason planning moved host-side.
10. **Heading-loop lesson stands** (098 + Part 2 #15): turn accuracy came from an outer
    heading feedback loop, not geometry/coast tweaks. The host planner streams twists;
    heading correction closes against streamed pose/heading telemetry — verify the
    achievable correction bandwidth over the telemetry link before committing to a
    control split (this is partly what the P0 relay/rate spikes inform).

## Also carried

The sim 180°/360° pivot runs both landing at ~272-273° (Part 0 B3) was never root-caused
— if the host planner reuses any of the old sim plant/tracker math, re-check for the
angle-wrap attractor before trusting sim results.
