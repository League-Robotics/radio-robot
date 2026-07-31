---
id: '004'
title: Delete binary_bridge.py's dead half; direct-call the survivors
status: done
use-cases:
- SUC-005
depends-on:
- '003'
github-issue: ''
issue: delete-binary-bridge-dead-half-and-direct-call-the-survivors.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete binary_bridge.py's dead half; direct-call the survivors

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

**Depends on ticket 003**: `Transport.halt()` must already exist and be
wired into `on_stop()`/`_safe_stop()`/`_set_origin()` before this ticket
deletes `binary_bridge.py`'s dead dispatch — otherwise there would be a
window with the STOP path routing to nothing.

## Description

`binary_bridge.py:112-119` pins `_LEGACY_TRANSLATION_AVAILABLE = False`
permanently. Everything behind it — the main dispatch body (312-341),
`_handle_binary_verb()`, `_handle_set()`/`_handle_get()`, `_handle_stream()`,
`_handle_snap()`, and the `render.*` branches of `render_log_line()` — is
unreachable (~250 of 632 lines). `translate_command()` returns
`ERR unavailable ...` for `STOP`/`STREAM`/`SNAP`/`ZERO`/`OZ`/`SI` and the
whole `S`/`T`/`R`/`TURN`/`G` family without touching the protocol object.
`__main__.py:606-738` builds a COMMANDS panel row per `commands.COMMANDS`
entry, then hides the whole panel; five of its seven verbs fall into the
dead stub anyway. The `ERR` reply text and three code comments cite a
phantom `binary-bridge-segment-replace-arms-deleted.md`, which does not
exist under `clasi/issues/`.

## Acceptance Criteria

- [x] Delete the dead dispatch tail, the four unreachable `_handle_*`
      functions, `_LEGACY_TRANSLATION_AVAILABLE`, and the `render`
      fallbacks.
- [x] Replace verbs that still have a real meaning with the direct-call
      pattern `_handle_otos_patch`/`_handle_set_patch` already proved for
      `OI`/`OL`/`OA`/`SET` — one short function per verb calling a live
      `NezhaProtocol` method, no translation layer (e.g. `STREAM`→
      `tlmOn()`/`tlmOff()` per ticket 003's own note).
- [x] Verbs with no v5 equivalent (`SNAP`, `ZERO`, the `S`/`T`/`R`/`TURN`/`G`
      family) get no shim: delete the verb from the surface entirely.
- [x] Delete the dead COMMANDS rows (`S`/`T`/`R`/`TURN`/`G`), the hidden
      panel construction in `__main__.py:606-738`, and the matching
      `commands.COMMANDS` entries. Keep `D`/`RT` only if the
      Managed/Unmanaged preset panel does not already cover them (per
      the code's own comment, it does — confirm before deleting).
      Confirmed via the code's own comment (stakeholder 2026-07-17: "I
      don't need full parameters, I just need buttons") — deleted D/RT
      too; `COMMANDS` is now `[]`.
- [x] Reconcile the phantom reference: either file
      `binary-bridge-segment-replace-arms-deleted.md` with its real
      history, or repoint the comments at this issue.
- [x] `binary_bridge.py` is meaningfully smaller after this ticket — if
      it is still ~600 lines, the gutting was not decisive enough.
      632 → 335 lines (~47% smaller).
- [x] `grep -n "_LEGACY_TRANSLATION_AVAILABLE\|legacy_verbs\|legacy_render" src/host/robot_radio/testgui/`
      returns nothing.
- [x] No GUI code path can send a verb that yields `ERR unavailable
      legacy verb`; unsupported verbs are simply absent from the UI.
- [x] `python -c "from robot_radio.testgui import binary_bridge"` still
      works.
- [x] `robot_radio/DESIGN.md`'s `testgui/` row is updated to reflect
      `binary_bridge.py`'s new, much-smaller shape (part of ticket 009's
      doc-rot sweep may also touch this row — coordinate, don't
      duplicate the edit).

## Testing

- **Existing tests to run**: existing headless button-acceptance suite
  (`test_gui_button_acceptance.py`); full `uv run python -m pytest`.
- **New tests to write**: the STOP-path test from ticket 003 must still
  pass (verifying it doesn't regress once the bridge's dead half is
  gone); a test per surviving direct-call helper (e.g. `_handle_stream`)
  confirming it calls the live protocol method, not the deleted
  translator.
- **Verification command**: `uv run python -m pytest src/tests -k "gui_button or binary_bridge" -q`.

## Implementation Notes

- **Approach**: delete first, then re-add only the direct-call helpers
  for verbs that still have a live wire meaning. Do not preserve any
  code "just in case" — the issue's own acceptance bar is that what
  remains should read as "a small module of direct-call helpers."
- **Files to modify**: `src/host/robot_radio/testgui/binary_bridge.py`,
  `src/host/robot_radio/testgui/__main__.py`,
  `src/host/robot_radio/testgui/commands.py`,
  `src/host/robot_radio/DESIGN.md`.
- **Documentation updates**: `robot_radio/DESIGN.md`'s `testgui/` row
  (binary_bridge.py's status changes from "a translation shim converting
  legacy text verbs to binary calls" to a small direct-call helper
  module).
