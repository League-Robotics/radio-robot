---
status: pending
---

# Subsystem Design Documents — first light pass

## Context

The project has no consultable current-state design: `docs/architecture.md` is a stale monolith and `docs/architecture/` is a base doc plus 74 per-sprint patch files — nothing describes any subsystem as it exists today. Eric is moving to spec-driven development: one living design document per subsystem (subsystem = source directory), plus a top-level system doc, all in one place so they can be read together. Sprints will later produce per-subsystem *design update* files that programmers fold back into the base docs — but **that process wiring is explicitly out of scope for this pass**. We write the documents, test the workflow manually, and CLASI instructions come later.

**Hard non-goals (stakeholder-mandated):**
- NO changes to the CLASI MCP server, `.clasi/`, `clasi/`, `.claude/agents/`, `.claude/skills/`, `.agents/skills/`, CLAUDE.md, or any rules/instructions.
- NO code moves or refactors. Docs describe the tree **as it is**; known-bad boundaries (host import cycles, `io/` grab-bag, sim straddling `tests/`) are recorded as flagged issues inside the docs, not fixed.
- `.clasi/oop` is present — no CLASI process ceremony for this work.

## Decisions already made with Eric

- **Scope:** all three trees — firmware `source/`, host `host/robot_radio/`, and the simulator (documented as one logical subsystem even though it spans `tests/sim/`, `tests/_infra/sim/`, and `host/robot_radio/io/sim_loop.py`).
- **Location:** `docs/design/`. Existing four essays there stay untouched (they remain free-standing reference notes).
- **Naming:** slugified source path, `/` → `-`: `source-devices.md`, `host-robot_radio-testgui.md`. The simulator has no single directory; it gets `sim.md` (the doc itself notes the missing first-class home).
- **Depth:** light but complete — every subsystem gets all sections, 1–3 pages each; interfaces and invariants, not exhaustive behavior narratives.

## Document template (every subsystem doc)

Frontmatter: `path:` (the source directory), `status: current`, `updated:` (date + sprint if known).

1. **Introduction** — what this subsystem is, what it does, why it exists (the "why" matters: e.g. why `devices/` has its isolation invariant).
2. **Overview** — what it connects to (dependencies in/out, verified from the include/import analysis), its operations, how it participates in the system. Basic systems engineering: inputs, outputs, collaborators.
3. **Requirements** — mandatory requirements that must be consulted and maintained when changing this subsystem (e.g. devices' no-messages/no-config isolation invariant; comms framing rules; the newlib-nano no-printf-float constraint; deadman timing; wire-key stability rules).
4. **Design description, piece by piece** — walk the contents. When a piece is itself a subsystem (subdirectory), give an overview *without inner detail* and reference its own design file. Files/classes get interface-first treatment: public surface, then how it works, with `file:line`-style references. Known defects/misfits are flagged in a short **Known issues** subsection (e.g. `applyOtosSample()` parked in odometry; testgui's Qt-free library modules; robot↔io cycle).

## Files to create (24 total, all under `docs/design/`)

**Top level (2):**
- `README.md` — the convention: doc structure, naming rule, and the intended lifecycle (sprint planning writes `design-update-<slug>.md` files in the sprint dir; to understand a subsystem you read base doc + pending updates; the programmer updates the base doc when finishing and the updates archive with the sprint). Descriptive documentation of the workflow we're testing — NOT wired into any skill/agent.
- `system.md` — whole-system design: the three systems (firmware, host, simulator) + test domains (`tests/sim|bench|playfield|testgui|unit`), how they connect (wire protocol, C ABI, transports), and a per-subsystem index with one-paragraph overviews linking to each subsystem doc (per template rule 4, applied at system level).

**Firmware (7):** `source-app.md`, `source-com.md`, `source-config.md`, `source-devices.md`, `source-kinematics.md`, `source-messages.md`, `source-types.md`.

**Host (14):** `host-robot_radio-calibration.md`, `-config.md`, `-controllers.md`, `-field.md`, `-io.md`, `-kinematics.md`, `-media.md`, `-nav.md`, `-path.md`, `-planner.md`, `-robot.md`, `-sensors.md`, `-testgui.md`, `-testkit.md`. (Tiny leaves like `controllers`, `media` will be well under a page — that's fine.)

**Simulator (1):** `sim.md` — the full logical subsystem: physics plants (`tests/sim/plant/`), composition + C ABI (`tests/_infra/sim/`), Python drivers (`io/sim_loop.py`, testgui `SimTransport`), and the `libfirmware_host` build; flags the tests/-placement inversion as a known issue.

## Source material (already gathered — reuse, don't re-derive)

Three completed cohesion analyses from this session cover: firmware dependency graph + per-file line counts; host import-coupling matrix + cycles + per-package misfit lists; sim boundary map. Their findings feed the Overview/Known-issues sections directly. Doc authors must still read the actual code they document (headers/docstrings, public surfaces) — the analyses give structure, not content.

## Execution

1. Write `docs/design/README.md` (convention) and the template — team-lead writes these directly, since they encode the stakeholder's process design.
2. Fan out doc-writing subagents in parallel batches, each given: the template, the naming/coding conventions (identifier rules apply to any code snippets), the relevant analysis excerpt, and its list of subsystem docs to write. Suggested batching: source tree (2 agents), host tree (3–4 agents grouped by related packages, e.g. motion cluster nav/path/planner/controllers/kinematics to one agent so the overlap story is told consistently), sim (1 agent).
3. Team-lead writes `system.md` last, after subsystem docs exist, so its index and one-paragraph overviews match them.
4. Consistency pass: uniform headings, working relative links, no propagated naming violations in snippets.

## Verification

- Every directory under `source/` and `host/robot_radio/` (excluding `__pycache__`, `pb2/`) has exactly one doc; `ls` cross-check against the file list above.
- All relative links in `docs/design/*.md` resolve (quick script: extract `](...)` targets, test existence).
- Spot-check 3 docs against code: interface listings match the actual public surfaces (e.g. `source/app/robot_loop.h`, `host/robot_radio/planner/executor.py`).
- Frontmatter parses and headings follow the template in every file.
- `git status` shows only new files under `docs/design/` — nothing else touched.
