# Evidence extract: Sprints 001–009

(Compiled by a reader agent from sprint.md, issues/, tickets/, architecture-update.md in each sprint directory. Quotes verbatim from artifacts.)

## Sprint 001 — HAL Layer and Project Skeleton
- **Goal**: Stand up the C++ firmware from a placeholder `main.cpp`: eight HAL drivers, type headers, Robot skeleton, boot announcement, `HELLO` response. A port of an existing TypeScript firmware ("plan-c-port-of-radio-robot-firmware" issue).
- **Type**: NEW-CAPABILITY
- **Ticket count**: 6. No issues/ dir.
- **Rework evidence**: None inside the sprint — but nearly every structure it created was later redone: `NezhaV2` (renamed/split in 008), `GripperServo` (renamed in 008), `Announcer` (deleted in 009), Robot owning `MicroBit` and the hidden `run()` loop (reversed in 007). Sprint 001's own architecture note — "Static instances in Robot... controls initialization sequence" — is precisely what 007's issue calls "wrong-placed."
- **Human-AI friction**: Ticket 006 is a whole ticket dedicated to "run python build.py and fix all compile errors," pre-listing expected CODAL API mismatches — the plan anticipated the AI writing code against imagined APIs and budgeted a fix-it pass.
- **Notable**: "Hardware-in-the-loop only — the CODAL framework does not support unit testing off-device" — no automated test safety net for any firmware sprint in this era.

## Sprint 002 — Control Layer and Core Motion Commands
- **Goal**: Make the robot driveable: MotorController (simple PI+FF), Odometry, CommandProcessor with drive-mode state machine, S-watchdog, motion/calibration K commands, wire-compatible with legacy Python host.
- **Type**: NEW-CAPABILITY
- **Rework evidence**: Planned-in-advance rework: "simple PI + feed-forward — ratio PID comes in sprint 5... Sprint 5 replaces the body only" — a deliberate throwaway control loop. The wire protocol it painstakingly matched ("must match TypeScript exactly") was hard-deleted in sprint 009. The K-command family (13 setters) replaced by `SET`/`GET` in 009. The CommandProcessor built here became 007's "God component."
- **Human-AI friction**: `architecture-update.md` is an **unfilled template** — sprint closed with its required architecture artifact blank. Process gate not enforced.

## Sprint 003 — Full Sensor and OTOS Command Set
- **Goal**: Feature parity with the TypeScript firmware for all 30+ commands.
- **Type**: NEW-CAPABILITY
- **Rework evidence**: Knowingly shipped a command destined for deletion; the gripper `G` was removed one sprint later, leaving the gripper **uncontrollable** until 009 restored it as `GRIP`.
- **Human-AI friction**: `architecture-update.md` again an unfilled template.
- **Notable**: All handlers piled into CommandProcessor ("No new files or classes in this sprint") — feeding the god-component problem 007 had to unwind.

## Sprint 004 — Ratio PID Motor Control and G Go-To Command
- **Goal**: Replace the sprint-002 PI+FF tick with a cumulative-distance ratio PID (ported from confirmed-working TypeScript), plus a two-phase `G` go-to command.
- **Type**: REWORK-OR-REFACTOR (planned) + NEW-CAPABILITY
- **Rework evidence**: Premise is redoing sprint 002. Also destroys sprint 003 functionality: "this is a deliberate scope trade-off — the gripper G command is sacrificed for the go-to G command" — un-sacrificed in 009.
- **Human-AI friction**: **Hardware acceptance criteria left unchecked in a done ticket** (ticket 004: build/deploy `[x]` but nine of eleven physical tests `[ ]`, status `done`). Issue spec's "What Not To Do" section reads as guardrails against anticipated AI implementation mistakes.
- **Notable**: Sprint-numbering churn: 004's own architecture notes refer to its content as "sprint 5" — sprints 004/005 were swapped from the original plan.

## Sprint 005 — Navigation Layer
- **Goal**: On-device navigation — PoseProvider/PathFollower interfaces, PurePursuit and Stanley, a `NAV` waypoint command. "a key design goal of the C++ rewrite: offload path following to the firmware".
- **Type**: NEW-CAPABILITY — **never executed**
- **Ticket count**: **0**. Tickets table empty; frontmatter reads `status: open` though the directory sits in `done/`. Only sprint.md exists.
- **Rework evidence**: Abandonment. Sprint 006: "Sprint 005 (Navigation Layer) — left untouched". Capability delivered the *opposite* way: sprint 009 copied the host's `robot_radio` package (incl. `nav/`, `kinematics/`) — navigation stayed host-side.
- **Human-AI friction**: A fully-written 143-line sprint plan silently shelved and archived as-if-done. No document records the decision. Major architectural pivot ("offload path following to firmware" reversed) with no written rationale.

## Sprint 006 — mbdeploy Package
- **Goal**: Consolidate three loose deploy scripts + ad-hoc device registry into a pipx-installable `mbdeploy` package with relay-flash protection.
- **Type**: PROCESS-OR-TOOLING (+ rework of sprint-001-era scripts)
- **Human-AI friction**: The relay-flash hazard ("a real hazard") implies near-miss incidents. Decisions section headed "Decisions (from the user):" — heavy stakeholder steering.
- **Notable**: First sprint with real unit tests and use-cases wired in; process compliance improved markedly here.

## Sprint 007 — Firmware Architecture Foundation
- **Goal**: Restructure the firmware built in 001–004: MicroBit out of Robot, visible main loop, DriveController extracted, unified RobotConfig, thin CommandProcessor, fix async-reply channel bug.
- **Type**: REWORK-OR-REFACTOR + BUG-FIX-RECOVERY ("keystone sprint")
- **Rework evidence**: Redoes sprints 001–004 structure. Issue catalog: "**`MicroBit` is wrong-placed.**... **`CommandProcessor` does far too much.**... **Config is duplicated and can diverge.**... **The main loop is hidden**... its `tick()` replies are hardwired to **serial** even when the command arrived over **radio**". Architecture-update names the CommandProcessor a "**God component**".
- **Human-AI friction**: Issue opens "the stakeholder wants it restructured" and records "**Stakeholder decisions (locked)**" — human explicitly overrode the structure previous sprints produced. Verification-deferral again: "Bench gate" box unchecked in a done ticket. Issue frontmatter still `status: in-progress` — bookkeeping drift.
- **Notable**: The async-reply bug shipped in 002 lived through five sprints. Decisions made here were themselves later reversed: Robot-facade model undone by 016; CommandProcessor refactored again in 019.

## Sprint 008 — Motor/HAL Layer: Vendor Coverage, Chip Velocity, Cleanup
- **Type**: REWORK-OR-REFACTOR + NEW-CAPABILITY
- **Rework evidence**: Corrects sprint-001 HAL design: "'Nezha' is the whole controller board, not a motor"; hardcoded signs should be per-motor config; "GripperServo → Servo... generic hobby-servo driver, not gripper-specific". The 0x47 velocity register "was only ever a `return 0` stub in the old TypeScript" — the port faithfully carried over a hole.
- **Human-AI friction**: FIXME markers scattered then stripped into a tracked issue. Distrust of hardware behavior explicit: "**Do not assume the laps→mm scale**... pin it empirically".
- **Notable**: Vendored the advisory `pxt-nezha2` driver in-repo so audits stop re-deriving the I2C protocol — early knowledge-capture pattern. Encoder/I2C read path touched here resurfaces as sprint 015.

## Sprint 009 — Protocol v2 and Host Controller Migration
- **Goal**: Hard-break rewrite of the entire wire protocol + migration of the Python host controller into the repo.
- **Type**: REWORK-OR-REFACTOR + NEW-CAPABILITY — largest sprint of the era
- **Rework evidence**: Discards the protocol 002–004 built to exactly match TypeScript: "We are taking a **hard break** — no backward compatibility... There is a lot of flux right now, so a clean break is cheaper than dual-parsing". Sprint 001's `Announcer` deleted. `GRIP` restores gripper control sacrificed in 004. All 24 K-commands (13 added five sprints earlier) replaced.
- **Human-AI friction**: "**Decisions locked with the stakeholder**" format recurs — suggesting prior sessions relitigated settled choices. Bench-deferral in a done ticket recurs. Cross-plan inconsistency the AI had produced flagged in the issue.
- **Notable**: Pivots the project's center of gravity to the host (`host/robot_radio/` arrives here). The copied-verbatim `nav/`, `kinematics/`, `controllers/` packages and the rewritten CommandProcessor get reworked repeatedly later (013, 017–020).

## Batch synthesis
Fast greenfield port (001–004) followed immediately by systematic un-doing of that port's structural choices (006–009), with sprint 005 abandoned in between. Two planned-rework moves were healthy; the larger pattern is unplanned: 007's issue is an itemized stakeholder correction of AI-built structure; 008 renames/re-splits the sprint-001 HAL; 009 hard-deletes the wire protocol 002–004 had matched "exactly". Process signals: architecture updates left blank in done sprints; sprint 005 archived open; recurring pattern of hardware/bench acceptance criteria left `[ ]` inside tickets marked `done` (004-004, 007-004, 009-008). Seeds of later rework: 007's Robot-facade (reversed in 016), CommandProcessor (redone 019–020), fiber main loop (redone 014), encoder I2C path (015, 064), wholesale-copied host nav/kinematics (rewritten 013, 017–018).
