---
status: done
sprint: '128'
tickets:
- 128-001
---

# Every "halt now" call site must use estop(), and a failed halt must be loud

**Source:** code review 2026-07-30, `03-host-core.md` CRITICAL §4 and MINOR
(Cutebot), `04-host-planning.md` MAJOR §5, cross-cutting theme (b).
**Priority:** P0 — safety. This is the measured 39.8 cm-vs-2.9 cm difference,
and several of these sites also swallow the halt's own failure.
**Goal served:** one halt idiom, used everywhere, means a reader auditing any
"stop the robot" path checks one function instead of re-deriving the
stop-vs-estop distinction at every site. Divergent halt idioms are where our
worst bug hid ("a fence detected correctly and stopped nothing").

## What is wrong

`NezhaProtocol.stop()` is a PLANNED stop (queues behind the in-flight Move —
measured: full 400 mm leg driven, 5.9 s). `estop()` is the halt-now verb
(2.9 cm, 0.10 s). Six-plus live call sites use `stop()` where the context is
"halt now", several wrapped in `except Exception: pass`:

| Site | Context | Defect |
|---|---|---|
| `io/robot_mcp.py:560` | the MCP tool literally named `stop` | planned `stop()` |
| `io/cli.py:540,555,576` | `except KeyboardInterrupt` in `cmd_drive`/`cmd_drive_stream` | planned `stop()` |
| `io/cli.py:675-679,765-769` | `cmd_turnto`/`cmd_goto` `finally` | planned `stop()` AND swallowed |
| `io/repl.py:129-133` | `RogoSession.close()` after Ctrl-C | planned `stop()` AND swallowed |
| `io/calibrate.py:501-505,908-916` | cleanup `finally` (incl. Ctrl-C) | planned `stop()` AND swallowed |
| `robot/cutebot.py:71-78` | `speed()` `GeneratorExit` handler | swallowed send, then polls for a confirmation that can never arrive |
| `src/tests/CLAUDE.md` | doc guidance | tells future authors to use `stop()` in Ctrl-C handlers |

`nav/camera_goto.py`/`nav/navigator.py`'s halt branches have the same defect
but that code is dead — fix them inside
`nav-goto-stack-is-dead-gate-it-loudly-then-rebuild-or-delete.md`, not here.

## What to do

1. Add ONE shared helper, modeled directly on the proven template
   `field/geofence.py::Geofence._halt()` (retry three times, raise loudly),
   e.g. `src/host/robot_radio/robot/halt.py`:

```python
import time


def halt_now(proto, log=print) -> None:
    """Halt the robot NOW. estop(), never the planned stop() -- measured
    2026-07-29: stop() sent mid-leg rode out the entire 400mm leg (39.8cm,
    5.9s); estop() halted in 2.9cm/0.10s. Raises on total failure: a halt
    that silently failed is indistinguishable from one that worked.
    """
    err: Exception | None = None
    for _ in range(3):
        try:
            proto.estop()
            return
        except Exception as exc:
            err = exc
        time.sleep(0.05)
    log(f"ERROR: estop() failed 3x -- ROBOT MAY STILL BE MOVING: {err!r}")
    raise err
```

2. Convert every site in the table to call it. A cleanup path that must not
   raise (a `finally` during interpreter shutdown) may catch — but must log,
   never `pass`:

```python
# BEFORE (cli.py:675-679)
finally:
    try:
        proto.stop()
    except Exception:
        pass

# AFTER
finally:
    try:
        halt_now(proto)
    except Exception:
        # halt_now already logged ROBOT MAY STILL BE MOVING; the operator
        # has been told -- which is the entire point.
        pass
```

3. Rename or re-document the MCP `stop` tool so its wire behavior matches its
   name: the tool an LLM reaches for when it wants the robot stopped must be
   `estop()`. If a planned-stop tool is still wanted, expose it separately as
   `planned_stop` with a docstring stating it queues behind the active Move.

4. Fix `robot/cutebot.py::speed()`'s `GeneratorExit` handler with the same
   helper, and fix or delete `rotate()` (line 180 references an undefined
   name `robot` — guaranteed `NameError` on any call).

5. Update `src/tests/CLAUDE.md`'s stale guidance to name `estop()`/
   `halt_now()` for Ctrl-C/exception paths.

6. Have `field/geofence.py::_halt()` delegate to the shared helper (keeping
   its `GeofenceViolation` wrapper) so there is exactly one halt
   implementation to audit.

## Acceptance

- `grep -rn "proto\.stop()\|robot\.stop()\|_robot\.stop()" src/host/robot_radio/{io,robot}` finds
  only sites whose surrounding context genuinely means "planned, sequenced
  stop" — each carrying a comment saying so.
- No halt call anywhere in `src/host` is wrapped in a bare
  `except Exception: pass` without a log line.
- Bench check (robot on stand): Ctrl-C during `rogo drive` visibly halts the
  wheels within ~0.1 s (watch encoders in telemetry), not after the leg
  completes.
