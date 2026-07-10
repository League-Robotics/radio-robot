---
status: pending
---

# Realign host tooling (TestGUI / robot_radio) + testgui tests to the current wire surface

> Refreshed 2026-07-09 (stakeholder triage). Originally filed against sprint
> 093's four-verb surface; the surface has since grown back under sprint 094
> and the teleop work — the fallout and the ask are updated below. The
> filename keeps its original (now historical) "four-verb" wording so
> existing references from archived sprint docs stay valid.

## Context

Sprint 093 gutted the firmware wire surface
(`simplify-the-main-loop-strip-it-to-bare-wheel-driving.md`, archived at
`clasi/sprints/done/093-simplify-the-main-loop-bare-wheel-driving-executive/issues/done/`);
sprint 094 + the teleop OOP work rebuilt it around the
segment-executing Drivetrain. The **current live surface** (see
`buildTable()` in `source/runtime/command_router.cpp` — only the `system`
and `motion` families are wired) is:

- **System:** `PING` `VER` `HELP` `ECHO` `ID` `HELLO`
- **Motion:** `S` `STOP` `D` `T` `RT` (re-parsed into `Motion::Segment`s),
  `MOVE`, `MOVER` (REPLACE/deadman teleop segment), `TLM` (one-shot pull),
  `QLEN`

Still **unregistered** (files intact on disk, families un-wired):
`SET`/`GET`, `STREAM`/`SNAP`, all `DEV *`, the OTOS verbs
(`OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/`OA`), and `SI`/`ZERO`. `R` (arc), absolute
`TURN`, and `G` (GOTO) are off pending the pose-stack restoration
([`restore-goto-pursuit-with-pose-estimator.md`](restore-goto-pursuit-with-pose-estimator.md)).

Sprint 093 deliberately did not touch host-side code, and that fallout
stands: TestGUI / `robot_radio` still drive removed verbs — the
calibration-`SET` push on connect, `GET`, `STREAM`/`SNAP` telemetry polling,
GOTO, tours, OTOS — and `tests/testgui/` dropped from 364/364 green
(pre-093) to 16 failures across 7 files
(`test_calibration_push_on_connect.py`, `test_error_divergence.py`,
`test_goto.py`, `test_set_origin.py`, `test_sim_errors_panel.py`,
`test_tour1_geometry.py`, `test_traces.py`, `test_transport.py`).
`tests/sim` and `tests/unit` are green; the sim close-gate is honest.

## Update 2026-07-10 (sprint 097 discovery — this issue now OWNS the
binary-plane migration of every consumer named below)

Sprint 097 ("Protocol v3 Sprint 3: host completion and text retirement")
set out to delete the text motion/config/telemetry command families now
that their binary replacements are proven (095/096). Ticket 097-003's own
implementation, plus a team-lead follow-up grep, found that essentially
every one of those text families still has at least one LIVE, production
consumer that reaches the wire directly, bypassing `NezhaProtocol`
entirely, and has never migrated to the binary plane. Per the protocol-v3
issue's own rule ("a text family is deleted only after its binary
replacement is bench-proven AND its consumers migrated"), 097 deferred
essentially all firmware text deletion — see
`clasi/sprints/097-protocol-v3-sprint-3-host-completion-and-text-retirement/
architecture-update-r1.md` Decision 8 for the full finding. **This issue
now explicitly owns migrating the following to the binary plane**, which
is the precondition a future sprint needs before it can actually delete
the text families protocol-v3 built binary replacements for:

- **TestGUI's manual command panel** (`host/robot_radio/testgui/
  commands.py`'s `COMMANDS` table + `build_wire_string()`, wired into
  `testgui/__main__.py`) — sends raw text `S`/`T`/`D`/`R`/`TURN`/`RT`/`G`
  via `transport.command()` for any connected transport, bypassing
  `NezhaProtocol` entirely. Also sends a hardcoded `"STREAM 50"` on every
  connect.
- **`host/robot_radio/io/robot_mcp.py`**: `push_calibration(_robot._proto,
  _config)` — `NezhaProtocol` has no `push_calibration` method, so
  `calibration/push.py`'s documented fallback always sends raw text `SET`.
  Give `NezhaProtocol` a binary `push_calibration` method (building
  `pb2.ConfigDelta`s, per 096-007's `set_config_binary()`) so this falls
  through to binary instead.
- **`host/robot_radio/io/cli.py`**: `cmd_turn`'s default (non-
  `--open-loop`) path sends raw text `RT` via `proto.send()` directly, not
  through 097-002's Legacy Verb Translator — wire it through the
  translator (or a dedicated binary `rotate()` method) instead.
  `_push_calibration()` (`rogo sync-cal`) sends raw text `SET`/`OI`/`OL`/
  `OA` — migrate the `SET` portion alongside `robot_mcp.py`'s.
- **`host/robot_radio/calibration/push.py`**, **`host/
  calibrate_verify.py`**: raw text `SET`/`GET` — migrate onto
  `NezhaProtocol.set_config()`/`.get_config()` (binary since 097-002).
- **`host/robot_radio/calibration/linear.py`/`angular.py`**: talk over
  `calibration/_conn_helpers.py`'s `RelaySerial`/`DirectSerial` (raw
  pyserial, chosen deliberately for relay-handshake/DTR timing control
  `SerialConnection` doesn't expose) — send raw text `D`/`T`/`STREAM`/
  `SNAP`. These need EITHER a binary-capable variant of
  `RelaySerial`/`DirectSerial`, or a rework onto `SerialConnection` itself
  if its timing-control gap can be closed. `sim.command()`/
  `SimConnection.send()` already forward `*B<base64>` lines to the SAME
  dispatcher `SerialConnection` does (per `tests/sim/unit/
  test_binary_channel.py`'s own precedent) — worth checking whether the
  relay path has an equivalent transparent pass-through before assuming a
  new transport layer is needed.
- **`host/robot_radio/testgui/transport.py`'s `SimTransport`**: uses
  `robot_radio.io.sim_conn.SimConnection` (ctypes ABI) — same open
  question as above re: binary pass-through.
- **`tests/bench/gamepad_teleop.py`**: raw text `MOVER` — migrate onto the
  binary `replace` arm (`rogo binary replace`'s own pattern).
- **`tests/bench/dtr_drive_demo.py`/`random_segment_demo.py`**: raw text
  `MOVE` — migrate onto the binary `segment` arm.

**Open schema question this issue should also resolve**:
`calibration/fit_sim_error_model.py`'s `_residual_vector()` structurally
depends on `TLMFrame.encpose`, which `telemetry.proto`'s `Telemetry`
message never carries (096-001 Decision 6 trimmed it for the 186-byte
envelope budget) — no transport fix restores it. Decide with the
stakeholder whether `encpose` should be restored to `telemetry.proto`
(budget permitting) now that a real, non-cosmetic consumer is known to
need it live (previously only `testgui/traces.py`'s "encoder" trace used
it, a bounded cosmetic loss already accepted).

A frozen bridge module, `host/robot_radio/robot/_legacy_tlm_text.py`
(`parse_historical_tlm_line()`), currently keeps the four unmigrated
telemetry consumers above working against text `STREAM`/`SNAP` — it is
NOT a general-purpose replacement for the deleted `parse_tlm()` and should
be retired once this issue's migration lands.

**Full firmware text retirement of the motion/config/telemetry families**
(deleting `S`/`D`/`T`/`RT`/`MOVE`/`MOVER`/`ECHO`/`VER`,
`config_commands.{h,cpp}`, and text `STREAM`/`SNAP` +
`Telemetry::buildTlmFrame()`) is **gated on this issue's migration landing
first** — it is a follow-up sprint's job after this one closes, not
something to attempt piecemeal here.

## Scope (to be decided in planning)

- **TestGUI / robot_radio**: target the surface above — stop sending
  removed verbs on connect (calibration-`SET` push, GOTO, tours,
  `STREAM`/`SNAP` polling), or gate them on a capability/verb-probe so they
  degrade gracefully. Adopt the new verbs where they replace old ones:
  `MOVE`/`MOVER` for motion, pull-based `TLM` for telemetry (the teleop
  tool already speaks `MOVER` — reuse its pattern).
- **`SET`/`GET` decision**: the config family is unwired, so nothing
  host-side can push calibration. Decide with the stakeholder whether the
  config family comes back on the wire (e.g. for `jmax`/`yawjmax`, the
  jerk knobs sprint 094's drivetrain issue wanted live) or the host stops
  assuming it — this issue should not unilaterally re-wire firmware
  families.
- **tests/testgui/**: park the removed-surface failures (a `parked-NNN/`
  leaf + `norecursedirs`, as sprint 093 did for `tests/sim`) or update them
  to the new surface — whichever matches the host realignment above.
- **Gate hygiene**: `pyproject.toml` `testpaths` still includes
  `tests/testgui`, but `tests/CLAUDE.md` claims the collected gate is
  `tests/sim` only — reconcile the stale doc with the actual gate,
  alongside the testgui triage.

## Not blocking

The sim close-gate (`tests/sim` + `tests/unit`) is green and the bench gate
is firmware-only; this host realignment is deliberately deferred. Best done
once the wire surface stabilizes after sprint 094 closes.
