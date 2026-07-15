---
status: pending
---

# `testgui/binary_bridge.py`'s R/TURN/G translation targets envelope arms that no longer exist on the wire

## Update (107-003): the breakage was worse than "wrong verbs" — a total import failure

Ticket 003's own bench-verification step (attempting to actually launch
`just testgui` / `python -m robot_radio.testgui` for the first time since
this issue was filed) discovered that this module's problem was not
limited to `R`/`TURN`/`G` constructing a dead `segment`/`replace` envelope
arm at RUNTIME, as originally diagnosed below. It could not even be
IMPORTED:

```
File ".../testgui/transport.py", line 132, in <module>
    from robot_radio.testgui import binary_bridge
File ".../testgui/binary_bridge.py", line 53, in <module>
    from robot_radio.robot import legacy_render as render
ImportError: cannot import name 'legacy_render' from 'robot_radio.robot'
```

Root cause: commit `129cbcb3` (`feat(104-002): delete retired
legacy-translator/rogo-proxy modules`, landed ~6 hours before sprint 107
was created) deleted `robot/legacy_render.py` AND `robot/legacy_verbs.py`
**wholesale**, with no replacement, but never updated `binary_bridge.py`
(which imports both at module level and depends on ~800 lines of their
surface throughout: tokenizing, one-arm verb dispatch, and every reply
rendering path). Because `testgui/transport.py` imports `binary_bridge`
unconditionally at ITS OWN module level (for `_HardwareTransport`'s
send/recv log-line rendering), this didn't just break the verbs this
issue originally diagnosed — it prevented `transport.py`, and therefore
the entire TestGUI (`__main__.py` imports `transport.py`), from importing
**at all**. Every command row, not just `D`/`RT`/`R`/`TURN`/`G`, and the
tour buttons themselves (even after 107-003's own rewrite onto the
twist-based `planner.tour.run_tour()` surface, which no longer calls
`binary_bridge` in the tour path at all) were blocked from ever running,
because the GUI process itself could not start.

**107-003's disposition**: per team-lead scope ruling, a MINIMAL
launch-unblock was applied directly in `binary_bridge.py` (not a rewrite,
and this issue's own scope below is UNCHANGED) — both imports are now
guarded (`try`/`except ImportError`), and every code path that needs
`legacy_render`/`legacy_verbs` degrades to an explicit, user-visible `ERR`
reply (`translate_command()`) or falls back to `google.protobuf.
text_format` rendering (`render_log_line()`) instead of crashing at import
time. This makes `transport.py`/the TestGUI importable and launchable
again, but `S`/`T`/`SET`/`GET`/`STREAM`/`SNAP` and every other verb this
module used to translate are now UNIFORMLY non-functional through
`binary_bridge.py` (they all return the same explicit
`_LEGACY_UNAVAILABLE_REPLY`, pointing back at this issue) — a WIDER
regression than this issue originally scoped (which believed `S`/`T`/
one-arm binary verbs still worked; see "Recommended direction" step 1
below, now stale on that specific claim). The tour path itself is
unaffected (107-003 rerouted it around `binary_bridge.py` entirely).
Fixing this module's translation/rendering for real — the scope described
in the rest of this issue — remains open and unattempted.

## Problem

`host/robot_radio/testgui/binary_bridge.py`'s `translate_command()`
translates several text-plane verbs (`D`/`RT`/`R`/`TURN`/`G`) into
`segment`/`replace` envelopes via `legacy_verbs.BINARY_DISPATCH` /
`legacy_translate.segment_for_arc()`/`segment_for_turn()`/
`segment_for_goto_relative()` — per that module's own docstring (dated to
sprint 097): "R/TURN/G (097, this ticket): UN-GATED — each now translates
to an open-loop `segment`/`replace` envelope."

Direct investigation during sprint 107's own architecture pass (reading
`protos/envelope.proto` directly, not assumed from `binary_bridge.py`'s
own possibly-stale docstring) confirms the current `CommandEnvelope.cmd`
oneof carries **exactly three arms: `twist`/`config`/`stop`** — no
`segment`/`replace` arm exists at all. Those arms belonged to the pre-102
on-robot `Motion::SegmentExecutor`, deleted in the single-loop rebuild
(sprint 102/103). Every `D`/`RT`/`R`/`TURN`/`G` command the TestGUI's
manual command rows (or `_GotoRunner`, for `G`) send today is therefore
silently targeting dead wire vocabulary — the firmware presumably ERRs or
mishandles it (not independently re-verified by this issue; the point is
the HOST-side translation itself is provably wrong, independent of how the
firmware responds).

## Relationship to the existing `nezha-facade-and-midlayer-dead-verb-
residue.md` issue

That issue already names `testgui/binary_bridge.py` as part of a broader
"dead-verb residue" sweep, but at the coarser grain of "calls a retired
`NezhaProtocol` method." This issue is a SHARPER, independently-verified
finding from sprint 107's own reading: the specific failure mode is not
merely "calls something retired" — it is "constructs a wire envelope arm
(`segment`/`replace`) that has been REMOVED FROM THE PROTOBUF SCHEMA
ITSELF," a stronger and more precisely diagnosable defect. Filed as its
own issue (rather than folded silently into the existing one) so a future
sprint scoping this work has the precise, verified root cause in hand
rather than needing to re-derive it.

## Why this was not fixed in sprint 107

Sprint 107 fixed ONLY the tour path (`D`/`RT`, tour buttons specifically)
by routing tours around `binary_bridge.py` entirely — through the new
`planner/tour.py` + `StreamingExecutor` twist-based path instead of any
text-verb translation. `binary_bridge.py` itself was explicitly left
UNCHANGED (architecture-update.md's own "Impact on Existing Components":
"This sprint does not claim to fix `binary_bridge.py` generally — only the
tour path is rerouted around it"). The TestGUI's manual command rows
(`S`/`T`/`D`/`R`/`TURN`/`RT`/`G` — `commands.py`'s `COMMANDS` schema) and
`_GotoRunner` still send through the broken translation path and remain
non-functional for anything beyond `S`/`T` (which route through simpler,
still-valid one-arm binary verbs, not `segment`/`replace`) — fixing this
generally is a larger, separate scope than one sprint's tour-focused work
justified taking on.

## Recommended direction

A future sprint should:
1. Confirm which of `binary_bridge.py`'s translated verbs are ACTUALLY
   broken (this issue confirms `R`/`TURN`/`G` structurally are, via the
   deleted `segment`/`replace` arms; `D`/`RT` were already fixed for the
   TOUR path specifically by sprint 107, but the manual GUI command ROWS
   for `D`/`RT` still go through the old, broken `binary_bridge.py` path
   — only tours were rerouted, not the manual rows) vs. which still work
   (`S`/`T`/one-arm binary verbs via `BINARY_DISPATCH`'s non-`segment`
   builders).
2. Decide whether the manual `D`/`RT`/`R`/`TURN`/`G` GUI command rows and
   `_GotoRunner` should be rewired onto the `twist`-based planner surface
   (mirroring sprint 107's own tour-path fix, generalized to single-shot
   manual commands) or retired if the capability's value doesn't justify
   the redesign cost — a stakeholder call, not a default (mirroring
   `nezha-facade-and-midlayer-dead-verb-residue.md`'s own established
   "decide per-module fate, don't delete unilaterally" precedent).

## Evidence

- `protos/envelope.proto` — `CommandEnvelope.cmd` oneof: `config`/`stop`/
  `twist` only (grep-verified, sprint 107's own architecture pass).
- `host/robot_radio/testgui/binary_bridge.py`'s own module docstring
  (097-era) — documents the `segment`/`replace` translation directly.
- `clasi/issues/nezha-facade-and-midlayer-dead-verb-residue.md` — the
  broader, coarser-grained prior finding this issue sharpens.
- `clasi/sprints/107-testgui-revival-tours-execute-and-close/
  architecture-update.md` Step 1 finding 1, Step 7 Open Question 5.
