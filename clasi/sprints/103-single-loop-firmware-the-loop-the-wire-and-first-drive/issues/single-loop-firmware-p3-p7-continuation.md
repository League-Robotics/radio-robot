---
status: in-progress
sprint: '103'
tickets:
- 103-001
- 103-003
- 103-004
- 103-005
- 103-006
- 103-007
- 103-008
- 103-009
- 103-010
- 103-002
---

# Single-loop firmware, phases P3–P7 (continuation of the archived 102 plan)

Sprint 102 delivered P0–P2 of the single-loop rebuild (spike verdicts, the
`pre-single-loop` tag + proven archive hexes, and the −42,987-line deletion to
a banner-only stub main). The governing plan — target main loop
(`runAndWait`/`markTime`/`sleepUntil`), wire protocol (twist/config/stop,
always-on telemetry with ack ring), delete/keep inventory, phase gates — is
archived with that sprint:
`clasi/sprints/done/102-single-loop-firmware-spikes-archive-and-delete-to-stub-p0-p2/issues/single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md`
(entombed at close with `status: in-progress`; THIS issue carries the
remaining work).

## Spike results that bind the remaining phases (from sprint 102)

- **Relay push validated** (spike-001): USB 0.0000% / relay 0.031% drop over
  240 s at ~30 Hz armed; "relay drops async STREAM" RETRACTED in the
  knowledge note. Push-based telemetry-only return path is GO; recommended
  common cadence ~25 Hz, both transports, serial stays 115200 (radio sets
  the budget — stakeholder decision, no baud change).
- **Frame budget** (spike-003): pruned CommandEnvelope worst-case 115 B;
  telemetry + ack ring fits at **depth 3** (179 B vs 186 ceiling); draft
  protos parked on `scratch/102-003-frame-budget` (10985ec1).
- **Wire codec transcription** for the new Comms:
  `clasi/sprints/done/102-.../notes/armor-wire-codec-transcription.md`.

## Remaining phases

- **P3** — the single loop itself (`source/app/` + real `main.cpp`): boot
  loop (telemetry from power-on), `runAndWait(gap, body)` cycle per the
  archived plan's one-page main loop; devices leaves direct (no fiber);
  review fixes C1 (commit lastWrittenPct_ only on kOk) and M1 (sleep-based
  clearance safety net) folded into the port; encoder odometry via
  BodyKinematics.
- **P4** — wire protocol: pruned protos land (twist/config/stop + ack ring
  depth 3 + fault bits + slow secondary frame), unified deadman, always-on
  ~25 Hz emission.
- **P5** — host realignment: twist/config/stop builders, ack-ring matcher in
  serial_conn, legacy translator removal.
- **P6** — bench gate: binary-plane soak on rig + robot, deadman kill-test,
  TLM drop-rate over USB and relay.
- **P7** — sim rebuild around the steppable loop (own sprint; see also
  clasi/issues/later/sim-hardware-fault-injection.md retarget).

## Process constraints (stakeholder, 2026-07-14 — memory: sprint-end-must-be-testable)

1. **Every sprint in this arc must END bench-runnable.** The sprint that
   builds the loop must include enough wire protocol AND a minimal host
   twist-sender to drive the rig and verify on the stand in the SAME sprint
   — never end at a non-runnable waypoint again.
2. Mid-sprint tickets may freely break the build on the branch — no stub
   ceremony.
3. **Plan the whole arc up front**: detail the next sprint AND create
   roadmap-stage entries for every successor sprint through P7, so the
   full sequence is visible before execution starts.

Related: [[nezha-motor-write-path-hardening]] (folded into P3),
[[host-planner-design-lessons-from-drive-v2-review]] (post-arc),
[[rig-persistent-otos-distrust]], [[absent-device-reprobe-after-boot]],
[[devices-naming-sweep-units-in-identifiers]] (sequenced after P3).
