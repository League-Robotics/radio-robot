---
id: '003'
title: Host-side stop= builders and reason= parsing
status: open
use-cases:
- SUC-003
depends-on:
- 052-002
issue: stop-conditions-as-a-first-class-system-primitive.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host-side stop= builders and reason= parsing

## Description

Add a `Stop` builder class to `host/robot_radio/robot/protocol.py` and extend
the six motion command methods to accept `stop=[...]`. Update `wait_for_evt_done`
to return `(outcome, reason)` tuple and audit all call sites.

The `reason=` token is already parsed into `ParsedResponse.kv` by the existing
key=value extraction loop in `parse_response` — no changes to `parse_response`
are needed.

## Implementation Plan

### Step 1: Add Stop builder class

Add to `host/robot_radio/robot/protocol.py` (near the top, after the existing
dataclasses):

```python
class Stop:
    """Builder for stop= clause tokens sent with motion commands.

    Each class method returns a formatted stop= string that can be passed
    in the stop=[...] list argument to motion command methods.

    Grammar matches the firmware mc_parseStopToken dispatch table:
      stop=t:<ms>
      stop=d:<mm>
      stop=line:<ge|le>:<thr>
      stop=sensor:<ch>:<ge|le>:<thr>
      stop=color:<h>:<s>:<v>:<dist>
      stop=heading:<cdeg>:<eps_cdeg>
      stop=rot:<arc_mm>
    """

    @classmethod
    def time(cls, ms: int) -> str:
        return f"stop=t:{ms}"

    @classmethod
    def dist(cls, mm: int) -> str:
        return f"stop=d:{mm}"

    @classmethod
    def line(cls, cmp: str, threshold: int) -> str:
        """cmp: 'ge' or 'le'"""
        return f"stop=line:{cmp}:{threshold}"

    @classmethod
    def sensor(cls, channel: str, cmp: str, threshold: int) -> str:
        """channel: 'line0'..'line3', 'colorR'..'colorC', 'analogIn0'..'analogIn3'"""
        return f"stop=sensor:{channel}:{cmp}:{threshold}"

    @classmethod
    def color(cls, h: float, s: float, v: float, dist: float) -> str:
        return f"stop=color:{h}:{s}:{v}:{dist}"

    @classmethod
    def heading(cls, cdeg: int, eps_cdeg: int) -> str:
        return f"stop=heading:{cdeg}:{eps_cdeg}"

    @classmethod
    def rot(cls, arc_mm: int) -> str:
        return f"stop=rot:{arc_mm}"
```

### Step 2: Extend motion command methods

Add `stop: list[str] | None = None` parameter to: `vw()`, `drive()`, `arc()`,
`timed()`, `distance()`, `turn()`.

In each method, append the stop tokens to the wire command string before the
trailing newline:

```python
if stop:
    cmd += " " + " ".join(stop)
```

Example for `vw()` (currently at line ~493):
```python
def vw(self, v_mms: int, omega_mrads: int,
       corr_id: str | None = None,
       stop: list[str] | None = None) -> None:
    ...
    cmd = f"VW {v_mms} {omega_mrads}"
    if stop:
        cmd += " " + " ".join(stop)
    if corr_id is not None:
        cmd += f" #{corr_id}"
    self._conn.send(cmd)
```

Verify order: `stop=` tokens must come before the `#id` correlation token,
as the firmware processes KV tokens from left to right and `#id` is a
special suffix, not a KV pair.

### Step 3: Update wait_for_evt_done return type

Change return type from `str` to `tuple[str, str | None]`:

```python
def wait_for_evt_done(self, verb: str, timeout_ms: int,
                      corr_id: str | None = None) -> tuple[str, str | None]:
    """Block until 'EVT done <verb>' or 'EVT safety_stop' arrives.

    Returns (outcome, reason) where:
      outcome: "done", "safety_stop", or "timeout"
      reason:  the reason= token from the EVT line, or None if absent
               (e.g. pre-052 firmware or EVT safety_stop without reason=watchdog).
    """
    deadline = time.time() + timeout_ms / 1000.0
    while time.time() < deadline:
        for raw_line in self._conn.read_lines(duration_ms=100):
            r = parse_response(raw_line)
            if r is None:
                continue
            if r.tag == "EVT":
                if corr_id is not None and r.corr_id is not None:
                    if r.corr_id != corr_id:
                        continue
                reason = r.kv.get("reason")  # None if absent
                if r.tokens and r.tokens[0] == "done":
                    if len(r.tokens) < 2 or r.tokens[1] == verb:
                        return "done", reason
                elif r.tokens and r.tokens[0] == "safety_stop":
                    return "safety_stop", reason
    return "timeout", None
```

Note: `r.kv` is already populated by `parse_response` for `key=value` tokens.
Since `reason=time` follows the existing KV extraction pattern, no changes to
`parse_response` are needed.

### Step 4: Audit and update all call sites of wait_for_evt_done

Run: `grep -rn "wait_for_evt_done" host/ tests/`

Update each call site to unpack the tuple:
```python
# Before:
result = proto.wait_for_evt_done("T", timeout_ms=5000)

# After (if caller uses result):
result, reason = proto.wait_for_evt_done("T", timeout_ms=5000)

# After (if caller ignores reason):
result, _ = proto.wait_for_evt_done("T", timeout_ms=5000)
```

Known locations to check:
- `host/robot_radio/robot/protocol.py` — `drive_until_sensor` and `stream_drive`
  do not call `wait_for_evt_done`, but verify.
- `tests/simulation/unit/test_nezha_drive.py`
- `tests/simulation/unit/test_protocol_v2.py`
- `tests/bench/` scripts (not collected by default sim suite, but still update).

### Step 5: Update docstring in protocol.py module header

The module docstring at line ~18 shows:
```
EVT  — async event:  "EVT done T", "EVT done T #12", "EVT safety_stop"
```
Update to mention the new `reason=` trailing token.

## Files to Create or Modify

- `host/robot_radio/robot/protocol.py` — `Stop` class, 6 method signatures,
  `wait_for_evt_done` return type and implementation.
- `tests/simulation/unit/test_protocol_v2.py` — new tests for Stop builder and
  `wait_for_evt_done` tuple return; update existing call sites.
- `tests/simulation/unit/test_nezha_drive.py` — update call sites if present.
- `tests/bench/` scripts — update call sites (not in default sim suite, but
  must be syntactically correct Python).

## Acceptance Criteria

- [ ] `Stop.time(1000)` returns `"stop=t:1000"`.
- [ ] `Stop.dist(300)` returns `"stop=d:300"`.
- [ ] `Stop.line("ge", 512)` returns `"stop=line:ge:512"`.
- [ ] `Stop.sensor("line0", "ge", 512)` returns `"stop=sensor:line0:ge:512"`.
- [ ] `Stop.color(120, 0.5, 0.4, 0.1)` returns `"stop=color:120:0.5:0.4:0.1"`.
- [ ] `Stop.heading(4500, 300)` returns `"stop=heading:4500:300"`.
- [ ] `Stop.rot(250)` returns `"stop=rot:250"`.
- [ ] `vw(200, 0, stop=[Stop.dist(300)])` sends `VW 200 0 stop=d:300\n` on the wire.
- [ ] `timed(200, 200, 1000, stop=[Stop.sensor("line0", "ge", 512)])` sends
  `T 200 200 1000 stop=sensor:line0:ge:512\n`.
- [ ] Multiple stop tokens: `vw(200, 0, stop=[Stop.dist(300), Stop.time(5000)])` sends
  `VW 200 0 stop=d:300 stop=t:5000\n`.
- [ ] `corr_id` still works with stop tokens: `vw(200, 0, corr_id="7", stop=[Stop.dist(300)])`
  sends `VW 200 0 stop=d:300 #7\n`.
- [ ] `wait_for_evt_done` returns `("done", "time")` when EVT line is `EVT done T reason=time`.
- [ ] `wait_for_evt_done` returns `("done", None)` when EVT line is `EVT done T` (no reason=).
- [ ] `wait_for_evt_done` returns `("safety_stop", "watchdog")` when EVT is
  `EVT safety_stop reason=watchdog`.
- [ ] `wait_for_evt_done` returns `("timeout", None)` on timeout.
- [ ] All existing call sites of `wait_for_evt_done` updated to unpack tuple.
- [ ] Sim tests pass: `uv run --with pytest python -m pytest tests/simulation -q` — no new failures.

## Testing

**Verification command**: `uv run --with pytest python -m pytest tests/simulation -q`

**Pre-existing baseline**: 2 failures. No new failures acceptable.

**New tests to write** — add to `tests/simulation/unit/test_protocol_v2.py`:

- Parametrize `Stop` builder methods: each returns the expected string.
- `vw()` with stop=[Stop.dist(300)] sends correct wire string (use the
  existing mock conn pattern in test_protocol_v2.py).
- `wait_for_evt_done` with reason= present in EVT: mock the connection to
  return `"EVT done T reason=time\n"`, assert return value is `("done", "time")`.
- `wait_for_evt_done` with no reason= in EVT: return value is `("done", None)`.
- `wait_for_evt_done` timeout: return value is `("timeout", None)`.
- Corr_id filtering still works with reason= present.
