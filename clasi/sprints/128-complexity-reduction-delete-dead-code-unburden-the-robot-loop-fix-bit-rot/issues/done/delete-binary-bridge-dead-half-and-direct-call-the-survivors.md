---
status: done
sprint: '128'
tickets:
- 128-004
---

# Delete binary_bridge.py's dead half; direct-call the surviving verbs; delete the dead COMMANDS rows

**Source:** code review 2026-07-30, `05-testgui-testkit.md` MAJOR §4, §5,
NOTE §12.
**Priority:** P1 — the single highest-leverage simplification in `testgui`:
~250 of 632 lines are unreachable by the module's own admission, and the
reachable part silently returns `ERR` for most verbs.
**Goal served:** every string verb the GUI sends today passes through a
translation layer whose main dispatch can never run
(`_LEGACY_TRANSLATION_AVAILABLE = False`, permanently). A reader tracing any
GUI command must rediscover this dead gate every time. Replacing
"string → translator → maybe wire" with "button → protocol method" makes each
command path one hop, auditable at a glance.

## What is wrong

- `binary_bridge.py:112-119` pins `_LEGACY_TRANSLATION_AVAILABLE = False`.
  Everything behind it — the main dispatch body (312-341),
  `_handle_binary_verb()`, `_handle_set()`/`_handle_get()`,
  `_handle_stream()`, `_handle_snap()`, and the `render.*` branches of
  `render_log_line()` — is unreachable. Confirmed live: `translate_command()`
  returns `ERR unavailable ...` for `STOP`/`STREAM`/`SNAP`/`ZERO`/`OZ`/`SI`
  and the whole `S`/`T`/`R`/`TURN`/`G` family without touching the protocol
  object.
- `__main__.py:606-738` builds a COMMANDS panel row per `commands.COMMANDS`
  entry, then hides the whole panel (line 738); five of its seven verbs fall
  into the dead stub anyway.
- The `ERR` reply text and three code comments cite
  `clasi/issues/binary-bridge-segment-replace-arms-deleted.md`, which does
  not exist anywhere under `clasi/issues/` — a phantom reference.

## What to do

1. **Delete** the dead dispatch tail, the four unreachable `_handle_*`
   functions, `_LEGACY_TRANSLATION_AVAILABLE`, and the `render` fallbacks.
2. **Replace** verbs that still have a real meaning with the direct-call
   pattern `_handle_otos_patch`/`_handle_set_patch` already proved for
   `OI`/`OL`/`OA`/`SET` — each one short function calling a live
   `NezhaProtocol` method, no translation layer:

```python
# The whole "bridge" a verb needs, when it needs one at all:
def _handle_stop(proto: NezhaProtocol) -> str:
    proto.estop()               # halt-now; see the Transport.halt() issue
    return "OK estop"

def _handle_stream(proto: NezhaProtocol, rate: int) -> str:
    proto.tlmOn() if rate > 0 else proto.tlmOff()
    return f"OK stream {'on' if rate > 0 else 'off'}"
```

   Verbs with no v5 equivalent (`SNAP`, `ZERO`, the `S`/`T`/`R`/`TURN`/`G`
   family) get no shim: delete the verb from the surface entirely rather
   than returning a polite `ERR` that callers ignore.
3. **Delete** the dead COMMANDS rows (`S`/`T`/`R`/`TURN`/`G`), the hidden
   panel construction in `__main__.py:606-738`, and the matching
   `commands.COMMANDS` entries. Keep `D`/`RT` only if the Managed/Unmanaged
   preset panel does not already cover them (per the code's own comment, it
   does).
4. **Reconcile the phantom reference:** either file
   `binary-bridge-segment-replace-arms-deleted.md` with its real history or
   repoint the comments at this issue.

What remains of `binary_bridge.py` afterwards should be a small module of
direct-call helpers — if it is still ~600 lines, the gutting was not decisive
enough.

## Acceptance

- `python -c "from robot_radio.testgui import binary_bridge"` still works;
  `grep -n "_LEGACY_TRANSLATION_AVAILABLE\|legacy_verbs\|legacy_render" src/host/robot_radio/testgui/` returns nothing.
- No GUI code path can send a verb that yields `ERR unavailable legacy verb`;
  unsupported verbs are simply absent from the UI.
- Existing headless button-acceptance suite passes; the STOP-path test from
  the `Transport.halt()` issue passes on the hardware-shaped mock.
