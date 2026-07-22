---
source_file: design.md
source_hash: fac05884db1d8bf1fc9b5dd7662340d247b7956c757852427dd88f3fc75d4345
---
# Diff: design.md

Comparison of the sprint overlay copy of `design.md` against its pristine (seed-commit) canonical version.

```diff
--- design.md (pristine)
+++ design.md (current)
@@ -13,28 +13,36 @@
 
 This project builds and drives a small differential-drive robot (a
 PlanetX Nezha V2 chassis on a BBC micro:bit V2 / nRF52833) over a
-host/robot split: a minimal C++ firmware that follows velocity twists
-and streams telemetry, and a Python host package that talks to it over
-USB serial or a radio relay, with a parallel host-build simulator for
-development without hardware. The current architecture (post sprint 115,
-firmware v0.20260721.1 on `master`) is deliberately minimal — the
-firmware speaks exactly three inbound commands (**TWIST / CONFIG /
-STOP**) plus a deadman safety timer, and emits one telemetry frame
-(**frame v2**: per-wheel `EncoderReading`/`OtosReading` with their own
-sample times, a single `flags` bit-string, a single ack slot, packed
-line/color words) every 20 ms cycle. There is **no** motion-planning
-stack on the firmware side today — no move-command queue, no
-jerk-limited trajectory solver, no heading-source policy — sprint 115
-("gut-to-minimal-firmware S1") deleted all of it. Sprint 116 (a bounded
-MOVE protocol) is planned but **not yet executed**; this document
-describes what runs today, not what is planned.
+host/robot split: a minimal C++ firmware that follows bounded MOVE
+commands and streams telemetry, and a Python host package that talks to
+it over USB serial or a radio relay, with a parallel host-build
+simulator for development without hardware. The current architecture
+(post sprint 116, "MOVE protocol cutover", on `master`) is deliberately
+minimal — the firmware speaks exactly three inbound commands (**MOVE /
+CONFIG / STOP**): `Move` carries its own velocity (twist or wheels
+variant), a stop condition (time/distance/angle), and a required
+`timeout` backstop, queued 1-active + 4-pending — and emits one
+telemetry frame (**frame v2**: per-wheel `EncoderReading`/`OtosReading`
+with their own sample times, a single `flags` bit-string, a single ack
+slot, packed line/color words) every 20 ms cycle. There is still **no**
+jerk-limited trajectory solver and no heading-source policy on the
+firmware side — sprint 115 ("gut-to-minimal-firmware S1") deleted the
+old motion stack, and sprint 116 ("MOVE protocol cutover", S2) replaced
+the interim TWIST+deadman surface with the bounded, queued `Move`
+command rather than reviving it. Every motion is now structurally
+self-bounding (its own stop condition or timeout), which supersedes the
+deadman it replaces — there is no `App::Deadman` anywhere in this tree.
 
 The host side (`src/host/robot_radio/`) still carries the code that was
 built against the pre-115 motion stack — tour/path/navigation planning —
 but by deliberate stakeholder decision (sprint 115's Design Rationale,
-Decision 6) that code was left in the tree rather than deleted, expected
-to go dormant/broken until sprint 116 gives it a new wire surface to
-target. See [`src/host/robot_radio/DESIGN.md`](../../src/host/robot_radio/DESIGN.md)
+Decision 6) that code was left in the tree rather than deleted. Sprint
+116 gave the host a new low-level wire surface to target
+(`NezhaProtocol.move_twist()`/`move_wheels()`) but deliberately did NOT
+revive the higher-level tour/path/navigation machinery — `planner/`,
+`path/`, `nav/`, and the TestGUI tour/turn modules stay dormant, by the
+same stakeholder decision, until a separate future sprint takes that on.
+See [`src/host/robot_radio/DESIGN.md`](../../src/host/robot_radio/DESIGN.md)
 for exactly which parts are live today and which are dormant.
 
 ## 2. Subsystem Map
@@ -49,12 +57,13 @@
 
 | Subsystem | Role |
 |---|---|
-| [`app/`](../../src/firm/app/DESIGN.md) | The single cooperatively-timed control loop (`App::RobotLoop`) and its passive modules: Comms, Telemetry, Drive, Odometry, Deadman, Preamble. |
+| [`app/`](../../src/firm/app/DESIGN.md) | The single cooperatively-timed control loop (`App::RobotLoop`) and its passive modules: Comms, Telemetry, Drive, Odometry, MoveQueue, Preamble. |
 | [`com/`](../../src/firm/com/DESIGN.md) | ARM-only raw transports: USB CDC serial, the micro:bit radio, persisted radio-channel storage. |
 | [`config/`](../../src/firm/config/DESIGN.md) | Generated boot configuration — per-robot calibration baked at build time from `data/robots/active_robot.json`. |
 | [`devices/`](../../src/firm/devices/DESIGN.md) | I2C-attached device leaves (Nezha motors, OTOS, color/line sensors), the shared `MotorArmor` policy, the velocity PID, and the pure `I2CBus`/`Clock`/`Sleeper` hardware seams. |
 | [`kinematics/`](../../src/firm/kinematics/DESIGN.md) | Stateless differential-drive math: inverse/forward twist↔wheel maps, curvature-preserving saturation. |
 | [`messages/`](../../src/firm/messages/DESIGN.md) | The wire schema: generated message structs, the generated envelope codec, the hand-written byte-level wire runtime. |
+| [`motion/`](../../src/firm/motion/DESIGN.md) | Pure, bounded-motion stop/timeout comparison logic (`Motion::StopCondition`) — no owned state beyond what's passed into `tick()`, no dependency on `MoveQueue`/`Drive`/wire types. A fresh, tiny directory (116) — not a revival of the larger `motion/` tree sprint 115 deleted. |
 | [`types/`](../../src/firm/types/DESIGN.md) | Vestigial protocol-v2 text-tag constants and the firmware-version generation seam (mostly dead code — see its own §6). |
 
 (`src/firm/README-DESIGN.md` is a one-paragraph pointer back to this
@@ -212,10 +221,13 @@
 closes per-wheel velocity loops, integrates odometry, and exchanges
 binary-armored protobuf-style messages with a host over USB serial and
 the micro:bit radio. It is the "plant" end of the host/robot split: the
-host plans motion (currently just profiled twists — see
+host plans motion (currently just profiled twists/wheel-velocity
+MOVEs — see
 [`src/host/robot_radio/DESIGN.md`](../../src/host/robot_radio/DESIGN.md));
-the firmware follows twist commands, enforces a deadman, and streams
-telemetry. Everything under this directory compiles into one image
+the firmware follows bounded MOVE commands — each self-bounding via a
+stop condition and a required timeout, queued 1-active + 4-pending —
+and streams telemetry; there is no deadman. Everything under this
+directory compiles into one image
 (`main.cpp` is the ARM entry point); the same modules minus the ARM
 adapters also compile under `-DHOST_BUILD` for host-side tests and
 simulation (`src/sim/`).
@@ -229,23 +241,43 @@
 motion-stack excision):** `Motion::Executor`/`Motion::JerkTrajectory`/
 `vendor/ruckig`, `App::Pilot`, and `App::HeadingSource` are DELETED
 wholesale — the `motion/` directory (and `motion/DESIGN.md`) no longer
-exist. There is no MOVE command, no arc/segment queue, and no
-heading-source policy in S1's minimal firmware; the robot is a pure
+exist. There is no arc/segment queue and no heading-source policy in
+S1's minimal firmware; the robot was, at that point, a pure
 TWIST-follower plus a deadman. `msg::PlannerConfig` and
 `PlannerConfigPatch` are gone with them (`planner.proto` deleted). This
 is tagged `pre-gut-motion-stack` for full recoverability — the tag and
 sprint 115's own `architecture-update.md` are where to read about the
 pre-gut architecture, not this doc.
 
+**116 (MOVE protocol cutover, S2) — landed.** The TWIST+deadman surface
+above is superseded, not extended: `Twist` (arm 19) and
+`ConfigDelta.watchdog` (field 4) are `reserved`, not reused; `App::
+Deadman` is deleted (`app/deadman.{h,cpp}`, both test harnesses). A new
+`Move` arm (21) carries its own velocity (twist or wheels variant), a
+stop condition (time/distance/angle), and a required `timeout`,
+dispatched through a new `App::MoveQueue` (1 active + 4 pending) that
+drives one `Motion::StopCondition` per active `Move`. `motion/` is
+recreated as a fresh, tiny directory containing only
+`Motion::StopCondition` — pure stop/timeout comparison logic, unrelated
+to and much smaller than the deleted `Motion::Executor`/
+`Motion::JerkTrajectory` tree above. See
+[`src/firm/app/DESIGN.md`](../../src/firm/app/DESIGN.md) and
+[`src/firm/motion/DESIGN.md`](../../src/firm/motion/DESIGN.md) for the
+full detail.
+
 Flow of one cycle, at orientation altitude:
 
 1. **Comms in** — `App::Comms` polls the two transports (serial, radio)
    for one armored `*B` line, dearmors and decodes it into a
    `msg::CommandEnvelope`.
-2. **Dispatch** — the loop's own switch acts on the command: a Twist
-   stages a target on `App::Drive` and arms `App::Deadman`;
-   config/queries reply via the primary telemetry frame's single ack
-   slot (`ack_corr`/`ack_err`, valid iff `flags` bit 5 — see
+2. **Dispatch** — the loop's own switch acts on the command: a Move
+   enqueues onto `App::MoveQueue` (1 active + 4 pending; `replace=true`
+   flushes pending and preempts the active `Move`, `replace=false`
+   enqueues or acks `ERR_FULL` past 4 pending), which stages the active
+   motion's velocity onto `App::Drive` and drives its own
+   `Motion::StopCondition`; a Stop flushes the queue and halts `Drive`
+   immediately; config/queries reply via the primary telemetry frame's
+   single ack slot (`ack_corr`/`ack_err`, valid iff `flags` bit 5 — see
    [`src/firm/app/DESIGN.md`](../../src/firm/app/DESIGN.md) §2).
 3. **Motor service** — the loop runs each `Devices::NezhaMotor`'s
    split-phase encoder request → settle → collect → PID → duty-write
@@ -311,10 +343,13 @@
   tokens, reply tag strings, and the `DEVICE:NEZHA2:...` banner format
   are frozen protocol surface, excluded from the naming-convention
   rename sweep — see §3.
-- **Deadman is the only staleness gate:** one `App::Deadman`, armed by
-  every actuation command, checked by the loop, expiry →
-  `Drive::stop()`. No second ad hoc watchdog timer belongs anywhere in
-  the firmware.
+- **No deadman — every `Move` is structurally self-bounding:**
+  `App::MoveQueue::tick()` runs unconditionally every cycle and drains
+  to `Drive::stop()` once the active `Move`'s stop condition or
+  `timeout` fires and nothing is pending — an emergent property of every
+  queued command carrying its own bound, not a second, independently-
+  timed staleness timer. `App::Deadman` does not exist in this tree. No
+  ad hoc watchdog belongs anywhere in the firmware.
 - **`newlib-nano` has no `%f`:** `printf`-family float formatting emits
   nothing on ARM (works fine in host builds). Floats cross the wire as
   scaled integers or via the binary codec.
@@ -326,7 +361,7 @@
 **Wire boundary.** Armored binary command/reply protocol: `*B<base64>`
 lines over USB serial (115200 CDC) and the micro:bit radio (group 10,
 channel 0–35 persisted in flash). Payloads are `msg::CommandEnvelope` in
-(`twist`/`config`/`stop` oneof), `msg::ReplyEnvelope` (`ok`/`err`/`tlm`
+(`move`/`config`/`stop` oneof), `msg::ReplyEnvelope` (`ok`/`err`/`tlm`
 oneof) out, plus an independently-armored `msg::TelemetrySecondary`
 frame. Schema source of truth: `src/protos/*.proto`. Boot banner:
 `DEVICE:NEZHA2:robot:<name>:<serial>` — byte-frozen. See
@@ -342,17 +377,22 @@
 §2's `updateLineColor()`); `src/firm/messages/event.h` remains orphaned
 dead code (see that doc's own §6); `src/firm/types/` remains a
 vestigial grab-bag (see that doc's own §6); sprint 116's MOVE protocol
-is the next planned change to this tree and has not landed as of this
-review.
+has landed — `Twist` (arm 19) and `ConfigDelta.watchdog` (field 4) are
+`reserved`, not reused; `App::Deadman` is deleted; see
+[`src/firm/motion/DESIGN.md`](../../src/firm/motion/DESIGN.md) for the
+new `Motion::StopCondition` module.
 
 ## 6. Open Questions / Known Limitations (system-level)
 
-- **Sprint 116 (the bounded MOVE protocol) is planned but not executed.**
-  Several dormant host-side modules (`src/host/robot_radio/planner/`,
-  `path/`, `nav/`, the TestGUI tour/turn modules) and one firmware
-  telemetry bit (`kFlagFaultMoveTimeout`) are pre-declared against that
-  future work — do not treat their presence as evidence the MOVE
-  protocol already exists.
+- **Sprint 116 (the bounded MOVE protocol) has landed.**
+  `kFlagFaultMoveTimeout` (bit 15) is now wired firmware-side (set on
+  the cycle an active `Move` ends via `timeout` rather than its stop
+  condition). Host-side `src/host/robot_radio/planner/`, `path/`,
+  `nav/`, and the TestGUI tour/turn modules remain dormant — 116's
+  host-side scope was limited to `protocol.py`'s low-level
+  `move_twist()`/`move_wheels()` builders; reviving the higher-level
+  tour/nav machinery onto the new wire surface is explicit future work,
+  not part of 116.
 - **The design-doc-set's mechanical validator cannot express "this
   child is out of scope because it symlinks outside the repository."**
   `src/vendor` remains permanently undocumented for that reason (§4).
```
