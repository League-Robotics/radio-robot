---
id: '113'
title: 'Host P4 rewire: Nezha facade, nav, and calibrate on the binary twist plane'
status: roadmap
branch: sprint/113-host-p4-rewire-nezha-facade-nav-and-calibrate-on-the-binary-twist-plane
worktree: false
use-cases: []
issues:
- nezha-facade-and-midlayer-dead-verb-residue.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 113: Host P4 rewire: Nezha facade, nav, and calibrate on the binary twist plane

## Goals

Restore the host mid-layer that is currently non-functional against P4
firmware — `host/robot_radio/robot/nezha.py` and everything downstream
(`nezha_state.py`, `nezha_kinematic.py`, `nav/`, `io/calibrate.py`,
`io/robot_mcp.py`'s `grip()` path, `testgui/binary_bridge.py`,
`testkit/safety.py`) still call now-deleted `NezhaProtocol` verb methods
(`ping`/`get_id`/`drive`/`distance`/`go_to`/`turn`/`zero_encoders`/etc.)
that have no P4 wire replacement.

1. Decide the P4-era liveness/identity story (e.g. "liveness = telemetry
   arriving at all" — the ack-ring characterization from sprint 112 is
   expected to feed this decision) and rewire `Nezha.connect()` on it —
   `ping()`/`get_id()` are both retired with no wire replacement.
2. Redesign `Nezha`'s motion surface around `twist()`/telemetry only — no
   blocking T/D/G/TURN/RT primitives exist on the wire; host-side
   trajectory generation (streaming `twist()` calls) is the only path,
   matching the single-loop firmware's "host computes the trajectory"
   design.
3. Decide per-module fate for `nav/` (`camera_goto.py`, `navigator.py`) and
   `io/calibrate.py`: rebuild atop the new motion surface, or retire if the
   capability's value doesn't justify the redesign cost — a stakeholder
   call, not a default.
4. Fold `testgui/binary_bridge.py`/`testkit/safety.py` into whatever the
   testgui revival needs — already known-broken the same way.

**Sequencing note**: detail-plan and execute this sprint AFTER sprint 112,
because 112's ack-ring characterization ("liveness = telemetry arriving")
directly feeds the `connect()` redesign in item 1 above.

## Scope

### In Scope

- P4-era liveness/identity decision and `Nezha.connect()` rewire.
- `Nezha` motion-surface redesign around `twist()`/telemetry.
- Stakeholder decision + follow-through for `nav/` and `io/calibrate.py`
  (rebuild or retire).
- `testgui/binary_bridge.py` / `testkit/safety.py` alignment with whatever
  the new `Nezha` surface looks like.

### Out of Scope

- Firmware-side motion accuracy / wedge-latch work — sprint 111.
- Firmware-side comms/device robustness — sprint 112 (consumed as an input
  here, not re-done).
- Repo hygiene / naming sweep / comment audit / vendor symlink — sprint 114.

## Note for Detail Planning

**LARGE sprint** — will likely split into two sprints at detail-planning
time: (a) liveness + motion-surface redesign (items 1-2, foundational), and
(b) nav + calibrate rebuild-or-retire (items 3-4, depends on (a)'s new
surface existing). The sprint-planner detailing this should assess splitting
explicitly rather than forcing everything into one set of tickets.

## Acceptance Sketch (at-a-glance)

- `Nezha.connect()` succeeds against a live P4-firmware robot using the
  decided liveness/identity mechanism — no calls to retired
  `NezhaProtocol` verb methods anywhere in `nezha.py`/`nezha_state.py`/
  `nezha_kinematic.py`.
- `Nezha`'s motion surface is fully `twist()`/telemetry-based; no
  references to `drive()`/`distance()`/`go_to()`/`turn()`/etc. remain.
- `nav/` and `io/calibrate.py` are either rebuilt and passing a smoke test
  against P4 firmware, or formally retired (removed or flagged) per the
  stakeholder decision — not left silently broken.
- Repo-wide grep for the retired `NezhaProtocol` verb-method names (per
  the 104-002 acceptance criterion this issue traces to) is clean across
  `host/`.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [ ] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
