---
id: 054-002
title: Harden sim tests to assert exact ERR range/badarg strings
status: done
sprint: '054'
use-cases:
- SUC-001
depends-on:
- 054-001
issue: motion-verbs-err-badarg-instead-of-range.md
---

# 054-002: Harden sim tests to assert exact ERR range/badarg strings

## Description

`tests/simulation/unit/test_motion_verbs_v2.py` currently contains range-error
test methods that assert against static string literals (e.g. `"ERR range l"`)
rather than live firmware responses via the `Sim` fixture. This means the tests
can pass even when the firmware produces `ERR badarg l` — they test the test
helper, not the firmware.

This ticket converts those static-string range tests to live `Sim` calls, and
tightens the assertions to the exact expected wire string. It depends on ticket
001 so the firmware already produces the correct response when these tests run.

## Acceptance Criteria

- [x] All range-error test methods in `test_motion_verbs_v2.py` that currently
      use static strings are converted to live `Sim` fixture calls.
- [x] Each converted test asserts the exact response string
      (e.g. `assert resp == "ERR range l"`).
- [x] Badarg (arg-count) test methods similarly converted to live calls.
- [x] No currently-passing test case is removed.
- [x] `uv run pytest` passes.

## Implementation Plan

### Approach

The `Sim` fixture is available in `test_protocol_v2.py` already. The
`test_motion_verbs_v2.py` file needs the same `build_lib` and `sim` fixtures
wired in. Check whether the file already requests `sim` in any test; if not,
add the fixture parameter to the affected test methods or convert the class
to use a `sim` fixture at class scope.

### Files to Modify

**`tests/simulation/unit/test_motion_verbs_v2.py`**

In `TestSCommand`:

- `test_s_range_l_too_high` — currently asserts against `"ERR range l"` (a
  static literal, not a firmware call). Convert to:
  ```python
  def test_s_range_l_too_high(self, sim) -> None:
      resp = sim.send("S 1001 0")
      assert resp == "ERR range l"
  ```
- `test_s_range_r_too_high` — similarly convert using `sim.send("S 0 1001")`.
- `test_s_badarg_no_args` — convert using `sim.send("S")`.

In `TestTCommand`:
- `test_t_range_ms_too_large` — convert using `sim.send("T 0 0 31000")`.
- `test_t_badarg_too_few` — convert using `sim.send("T 200 150")`.

In `TestDCommand`:
- `test_d_range_mm_zero` — convert using `sim.send("D 0 0 0")`.
- `test_d_badarg_too_few` — convert using `sim.send("D 200 200")`.

Note: `test_s_range_l_too_high` in the existing file uses `S 1001 0`
(exceeds max 1000); the issue acceptance criteria use `S 99999 0` — both
are out-of-range so either value is valid. Keep the existing value (`1001`)
to minimize churn; just convert to a live call.

If `TestSCommand` / `TestTCommand` / `TestDCommand` are plain classes (no
`sim` fixture today), add `sim` as a method parameter and ensure
`conftest.py`'s `sim` fixture is in scope (it is, as `test_motion_verbs_v2.py`
is in the same directory as the conftest that provides it).

### Testing Plan

Run `uv run pytest tests/simulation/unit/test_motion_verbs_v2.py -q` after
changes and confirm all tests pass. Run full `uv run pytest` for CI green.

### Documentation Updates

None required.
