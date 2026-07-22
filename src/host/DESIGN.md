# src/host — Host-Side Python (root overview)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-22 · **Status:** in-flux

---

## 1. Purpose

`src/host` is the host half of the host/robot split — everything that
talks to the robot (or a simulated one) from a laptop. It is the
**planner/operator** end; the firmware ([`src/firm`](../firm/DESIGN.md))
is the plant that follows bounded MOVE commands and streams telemetry.
The host owns transports, the wire-protocol adapter, per-robot config
loading, calibration, sensor decoding, the `rogo` CLI, an MCP server,
and the PySide6 TestGUI.

Structurally this root is a single importable Python package
([`robot_radio/`](robot_radio/DESIGN.md)) — it is the only one-level-down
subsystem here, so this root doc is a thin orientation layer and the
per-file detail lives in that subsystem's own doc.

`src/host` is one of the two declared design-doc-set source roots
(`.clasi/config.yaml`'s `sources:` — the other is `src/firm`).
System-wide context spanning both roots — the project overview, global
naming/style conventions, and why the roots are exactly these two — lives
one level up in [`docs/design/design.md`](../../docs/design/design.md);
this doc is the host-tree map only.

## 2. Subsystem Map

| Subsystem | Role |
|---|---|
| [`robot_radio/`](robot_radio/DESIGN.md) | The importable host package: transports, the `NezhaProtocol` wire adapter, per-robot config loading, calibration, sensor decoding, the `rogo` CLI, the `robot_mcp` MCP server, and the TestGUI. **Mixed live/dormant** — see §3 and the subsystem doc's own file-by-file split. |

The Python package root is `src/host/robot_radio/` (the `pyproject.toml`
that installs it lives at the repo root); `import robot_radio` resolves
here.

## 3. Two Eras in One Tree — the One Fact to Know First

**This root currently holds two eras of code in one package, and that is
the single fact every reader needs before touching anything here.**
Sprint 115 gutted the firmware's motion stack; sprint 116 gave the host a
new low-level wire surface (`NezhaProtocol.move_twist()`/`move_wheels()`)
but — by deliberate, recorded stakeholder decision (sprint 115's Design
Rationale, Decision 6) — did **not** delete the host-side code built
against the pre-115 stack, nor revive it. The result:

- **Live:** the narrow, close-to-the-wire surface — `robot/protocol.py`
  (`NezhaProtocol`), `io/serial_conn.py`, `io/repl.py`, `io/sim_loop.py`,
  `config/robot_config.py`, and the `stop`/`move`/`config` command path.
- **Dormant, by decision:** `planner/`, `path/`, `nav/`, and the TestGUI
  tour/turn modules — built around the deleted `Move`/tour abstraction,
  broken against today's firmware, parked (not deleted) pending a future
  sprint that revives them onto the MOVE surface.
- **Mixed, file-by-file:** several nominally-live directories (`robot/`,
  `io/`, `sensors/`, `calibration/`, `testgui/`) contain both current
  and dead code. **A directory being "live" does not mean every function
  in it is callable** — check the subsystem doc's per-file notes before
  adding a caller, not just which directory a file lives in.

Do **not** "clean up" by deleting dormant code without a separate,
explicit decision — it is parked, not gone. Full detail (which function
in which file is which) is in
[`robot_radio/DESIGN.md`](robot_radio/DESIGN.md) §2–§3.

## 4. Conventions Every Host Subsystem Doc May Assume

- **Naming & units (Python).** The project's identifier rules apply here
  too: name the *quantity*, never the unit; units go in a leading
  bracketed comment tag — Python uses `# [ms]`, `# [mm/s]`. Full
  convention in [`docs/design/design.md`](../../docs/design/design.md) §3
  and `.claude/rules/coding-standards.md`.
- **Wire keys are protocol, not identifiers.** JSON config keys in
  `data/robots/*.json` are mirrored 1:1 by
  `config/robot_config.py`'s pydantic field names; renaming one is a
  data-format change, out of scope for the identifier-naming rules.
- **Generated protobuf bindings are never hand-edited.**
  `robot_radio/robot/pb2/*_pb2.py` are compiled from `src/protos/*.proto`
  by `src/scripts/gen_pb2.py`; a hand edit is destroyed on the next
  build. Fix the `.proto` or the generator.
- **Config is fail-closed truth from `data/robots/*.json`.** Per sprint
  114, an unconfigured device or a JSON missing a required calibration
  key fails loudly rather than substituting a silent bench default.
- **Firmware is binary-only; text goes through the translator shim.** Any
  code still thinking in text verbs (`SET`/`OI`/`TN…`) must route through
  `testgui/binary_bridge.py` or call `NezhaProtocol`'s binary methods
  directly — never emit a bare text line and expect the firmware to
  answer it (the firmware's 6-verb text rump aside). This is the host end
  of [`src/firm/DESIGN.md`](../firm/DESIGN.md) §4's wire-boundary
  contract.

## 5. Relationship to the Rest of the Repo

- **Consumes the wire schema** via `robot/pb2/` — the compiled Python
  bindings for `src/firm/messages/`; source of truth
  [`src/protos`](../protos/DESIGN.md).
- **Consumes the simulator** — `io/sim_loop.py` loads
  [`src/sim`](../sim/DESIGN.md)'s dylib (the real firmware compiled
  `-DHOST_BUILD`) and drives it over the same `CommandEnvelope` surface
  as a real robot: one command-in Sim, not a second code path.
- **Consumes per-robot calibration** — `data/robots/*.json` via
  `config/robot_config.py`, the same JSON the firmware bakes at build
  time.
- **Consumes AprilCam** — `field/`, `media/`, and camera-dependent
  `testkit/`/`testgui/` modules use `aprilcam.client.control.DaemonControl`;
  no firmware wire dependency.
- **Is exercised by** [`src/tests`](../tests/DESIGN.md)'s `unit/` and
  `testgui/` categories, which import this package directly.

## 6. Open Questions / Known Limitations

- **Sprint 116's MOVE protocol is the expected path back to life for most
  of `planner/`/`path/`/`nav/`** — but the higher-level revival has not
  been executed. Until it lands, treat every dormant entry as broken
  today, not "probably fine."
- **The live/dormant split is not clean at the file level.** The sharpest
  traps — `robot/nezha.py`, `sensors/otos.py`, `calibration/push.py`,
  `io/robot_mcp.py`'s `connect` — each sit in a nominally-live directory
  but their most obviously named entry point calls a firmware verb that
  no longer exists. See [`robot_radio/DESIGN.md`](robot_radio/DESIGN.md)
  §2–§3 and §6.
- **Whether to delete vs. rewrite the dead halves** (of `nezha.py`,
  `sensors/otos.py`, `calibration/push.py`'s text route) is an open call,
  deferred until sprint 116's higher-level revival clarifies what the
  next wire surface needs.
