---
status: pending
---

# repl.py confirm() reads the deleted `TLMFrame.ack` slot — scan the ack ring

**Source:** code review 2026-07-30, `03-host-core.md` CRITICAL §2.
**Priority:** P0 — every motion verb in `rogo repl` crashes the whole REPL
process today.
**Goal served:** this is the smallest possible fix that restores the one
interactive bench surface we use to *find* firmware bugs. It also removes a
snare: the crash happens three frames away from the command that caused it,
which is exactly the kind of failure this cleanup campaign exists to prevent.

## What is wrong

`io/repl.py:105-106` (`RogoSession.confirm`):

```python
if f.ack is not None and f.ack.corr_id == corr_id:
    return f.ack
```

`TLMFrame` has no `ack` field. It was deleted by 124-008 — the frame's own
docstring says so ("ring membership in `acks` below already means 'really
acked'"). The first telemetry frame pumped during any command-issuing verb
(`twist`, `stop`, `drive`, `turn`, `config`, `raw`) raises `AttributeError`,
which `dispatch()` does not catch, killing the REPL.

## What to do

Rewrite `confirm()` to scan the ack ring, keeping the pump-through-session
path (do **not** delegate to `NezhaProtocol.wait_for_ack()` — that pumps
frames itself and would bypass `session.recorder`, silently dropping frames
from recordings):

```python
def confirm(self, corr_id: int, timeout_ms: int = ACK_TIMEOUT):
    """Pump frames until one carries an ack for ``corr_id`` (or timeout).
    Returns the matching ``AckEntry`` or ``None``."""
    deadline = time.monotonic() + timeout_ms / 1000.0
    while time.monotonic() < deadline:
        for f in self.pump():
            # Ring membership == really acked (124-008); the scalar
            # ``TLMFrame.ack`` slot no longer exists.
            for ack in f.acks:
                if ack.corr_id == corr_id:
                    return ack
        time.sleep(0.005)
    return None
```

Also fix the stale sentence in `pump()`'s docstring (`repl.py:90-91`), which
still tells callers to "scan their single ack slot (``TLMFrame.ack``)".

`RogoSession.close()`'s swallowed `proto.stop()` is covered by the separate
halt-verb sweep issue
(`halt-now-call-sites-must-use-estop-and-never-swallow-failure.md`) — fix it
in the same touch if convenient.

## Acceptance

- `rogo repl` connected to the bench robot: `twist 150 0 0 800`, `stop`,
  `config pid.kp=...`, and `raw` each print a `corr_id=N OK` (or a real
  error string), and the REPL survives all of them.
- A unit test constructs a `TLMFrame` with a populated `acks` ring and
  asserts `confirm()` matches by `corr_id`; a frame with an empty ring times
  out to `None` instead of raising.
- `grep -n "\.ack\b" src/host/robot_radio/io/repl.py` returns nothing.
