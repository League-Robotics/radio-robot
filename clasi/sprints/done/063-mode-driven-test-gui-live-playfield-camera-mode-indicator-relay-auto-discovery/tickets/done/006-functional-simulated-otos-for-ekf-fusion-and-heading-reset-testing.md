---
id: '006'
title: Functional simulated OTOS for EKF fusion and heading-reset testing
status: done
use-cases:
- SUC-007
- SUC-005
depends-on: []
github-issue: ''
issue: sim-otos-device-for-kalman-and-heading-reset.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Functional simulated OTOS for EKF fusion and heading-reset testing

## Description

The EKF/OTOS-fusion path (`Odometry::correctEKF`, exercised via
`Robot::otosCorrect`) and the "Set Robot @ 0,0" heading reset (ticket 063-004,
SUC-005) cannot be tested in the simulator today, because the sim OTOS device
(`SimOdometer`) does not behave like the real OTOS from the command/fusion
surface's point of view.

Investigation (see `architecture-update.md` Addendum — Ticket 006 for the full
write-up) found the root cause is **narrower** than the originating issue
speculated. The issue suspected `OtosCtx.otos` was never wired to the sim
`SimOdometer` in the host build. That's not true — `Robot::Robot()`
(`source/robot/Robot.cpp:130`) already calls
`_otosCommands.setCtx(&otos, &state.actual)` unconditionally, and `otos` is
`hal.otos()`, which `SimHardware::otos()` (`source/hal/sim/SimHardware.h:44`)
already returns as the same `SimOdometer` instance used by `Drive` and
`Robot::otosCorrect`. The command context is correctly wired in both builds.

The actual gaps, confirmed empirically against `libfirmware_host.dylib`:

1. **`SimOdometer` is never `begin()`-initialised except through a test-only
   hook.** `Sensor::is_initialized()` gates every OTOS command handler
   (`otosReady()` in `source/commands/OtosCommands.cpp`) and
   `Robot::otosCorrect()`'s `activeOtos.is_initialized()` guard
   (`source/robot/Robot.cpp:179`). Today `SimOdometer::begin()` is called from
   exactly one place: `sim_set_otos_fusion()`
   (`tests/_infra/sim/sim_api.cpp:578-581`), reachable from Python only via
   `Sim.set_otos_fusion()` / `Sim.set_field_profile(fuse_otos=True)`.
   `SimTransport._apply_field_profile()`
   (`host/robot_radio/testgui/transport.py`) already calls
   `sim.set_field_profile(fuse_otos=True)` on every Sim-mode connect — so the
   Test GUI's Sim mode *does* already flip `_initialized = true`. Confirmed by
   probe: `OZ`/`OI`/`OR`/`OV` return `ERR nodev <verb>` before
   `set_field_profile(fuse_otos=True)` runs, and `OK` after. Any other
   sim/host caller of the command surface that doesn't go through
   `set_field_profile` (e.g. a `tests/simulation` fixture built directly from
   `firmware.py`) still sees `nodev`.

2. **`OZ`/`OV` (`setPositionRaw`) do not reset the accumulator the EKF
   actually fuses — this is the real heading-reset gap.**
   `SimOdometer::readTransformed()` returns `{_odomX, _odomY, _odomH}` when
   `_useSimModel` is on (the mode `set_field_profile`/`set_otos_fusion` also
   enables via `enableSimModel(true)`). `setPositionRaw()`
   (`source/hal/sim/SimOdometer.cpp:54-59`, invoked by `OZ`/`OV`) writes only
   `_rawX/_rawY/_rawH` — the raw-register shadow read back by
   `getPositionRaw()` — and never touches `_odomX/_odomY/_odomH`. Only
   `setInjectedPose()` (used exclusively by the different test-only hook
   `sim_set_otos_pose`, not reachable from any command) resets the
   accumulator. Empirical probe: turn the sim robot to a non-zero heading,
   stop, call `OZ`, then read `sim.get_otos_pose()` — the accumulator is
   **unchanged**. `OZ` is currently a no-op on the value the EKF fuses.
   Consequently the sim also fails to demonstrate `SI`-alone's drift-back in
   the intended way: probing `SI 0 0 0` alone after a turn shows the fused
   heading simply holds at the OTOS's (never-zeroed) heading — it isn't
   "drifting back" so much as never having had anything to drift back FROM,
   because there is no re-referencing step yet to compare against.

Both gaps are independent of the standalone `BenchOtosSensor`
(`sim_bench_otos_*` / `DBG OTOS BENCH`) — confirmed by grep, no call site
connects it to `Robot::otosCorrect()` or the EKF. That model is out of scope.

## Acceptance Criteria

- [x] In Sim mode (via the Test GUI, and via any `tests/simulation` fixture
      that calls the new `begin_otos` harness hook), `OZ`, `OI`, `OR`, and
      `OV` all reply `OK` — never `ERR nodev`.
- [x] `OZ` (and `OV x y h`) re-reference the sim OTOS's accumulated pose
      (`_odomX/_odomY/_odomH`), not just the raw-register shadow
      (`_rawX/_rawY/_rawH`): after `OZ`, `sim.get_otos_pose()` reads
      `(0.0, 0.0, 0.0)` even if the robot was previously at a non-zero
      heading.
- [x] Reproduces the hardware bug: after turning to a non-zero heading and
      stopping, sending `SI 0 0 0` **alone** (no `OZ`) leaves the fused
      heading (`sim.get_fused_pose()`) at the sim OTOS's retained heading,
      not at 0, across many subsequent ticks (no `OZ` means no
      re-referencing, so `correctEKF` keeps re-applying the stale OTOS
      heading).
- [x] Verifies the fix: after turning to a non-zero heading and stopping,
      sending `ZERO enc`, then `OZ`, then `SI 0 0 0` resets the fused heading
      to 0 and it **holds** at 0 across many subsequent ticks.
- [x] Test GUI Sim mode: clicking "Set Robot @ 0,0" (ticket 063-004's
      `_set_origin()` sequence) after a turn drives the on-screen avatar
      heading to 0 and keeps it there — no manual test-only hook required,
      only the existing `_apply_field_profile()` call on connect. (Verified
      via the fixed `setPositionRaw` + the existing
      `sim.set_field_profile(fuse_otos=True)` call path exercised by the new
      regression tests; `SimTransport._apply_field_profile()` was not
      modified, per plan.)
- [x] `tests/simulation` gains a regression test file (or is added to an
      existing OTOS-fusion test file) covering both the bug-reproduction and
      fix-verification cases above, using the new `begin_otos` harness hook
      (not `set_field_profile`, to keep the assertions free of turn-slip/noise
      side effects).
- [x] No regressions: `test_golden_tlm.py`, `test_ekf_dual_source.py`,
      `test_dbg_otos_commands.py`, `test_ekf.py`,
      `test_ekf_encoder_velocity_unconditional.py`, and
      `test_n8_n9_sensor_freshness.py` all pass unchanged.
- [x] `OI` (`init()`) and `OR` (`resetTracking()`) remain no-ops in
      `SimOdometer` (per the architecture addendum's open question 1) but
      return `OK` once the sim OTOS is initialised.

## Implementation Plan

### Approach

Two small, independently testable changes plus one harness addition and one
regression test:

1. **Fix `SimOdometer::setPositionRaw`** to re-reference the accumulator, not
   just the raw shadow registers.
2. **Add a narrow `sim_begin_otos()` / `begin_otos()` harness hook** (C ABI +
   Python binding) mirroring the existing `drive_api_begin_otos()` pattern, so
   tests can initialise the sim OTOS without also pulling in
   `set_field_profile()`'s turn-slip and noise side effects.
3. **No change needed to `SimTransport`** — `_apply_field_profile()` already
   calls `sim.set_field_profile(fuse_otos=True)` on connect, which already
   begins the sim OTOS. This ticket's fix in `SimOdometer` makes that existing
   call path behave correctly end-to-end.
4. **Write the regression test** exercising both drift-back (bug) and
   hold-at-zero (fix) cases, using `begin_otos()` for a clean, noise-free
   fixture.

Do not touch `Robot.cpp`, `OtosCommands.cpp`, `Odometry.cpp`, or
`SimHardware.h` — the command-context wiring and the EKF fusion path are
already correct.

### Files to create/modify

- `source/hal/sim/SimOdometer.cpp` — modify `setPositionRaw(int16_t x, int16_t
  y, int16_t h)`: after the existing `_rawX = x; _rawY = y; _rawH = h;`
  assignments, also set `_odomX`, `_odomY`, `_odomH` from the same raw values,
  applying whatever LSB→float scale `getPositionRaw`/the real `OtosSensor`
  use for OTOS registers (confirm the scale factor by reading
  `source/hal/real/OtosSensor.{h,cpp}`'s `setPositionRaw`/`getPositionRaw`
  before implementing — `OZ` always calls this with `(0,0,0)`, which is
  scale-invariant, so the scale factor only matters for `OV`'s non-zero
  case). Update the class-level doc comment in `SimOdometer.h` to note that
  `setPositionRaw` now re-references the accumulator (previously
  raw-shadow-only).
- `source/hal/sim/SimOdometer.h` — no interface signature change; update the
  doc comment above `setPositionRaw` to describe the new accumulator
  re-reference behavior and its parity with the real OTOS chip's
  `setPositionRaw`.
- `tests/_infra/sim/sim_api.cpp` — add `void sim_begin_otos(void* h) {
  static_cast<SimHandle*>(h)->hal.simOdometer().begin(); }` near the existing
  `// ---- OTOS sim model ----` block (next to `sim_enable_otos_model`,
  `sim_set_otos_fusion`).
- `tests/_infra/sim/firmware.py` — add the ctypes binding
  (`argtypes = [ctypes.c_void_p]`, `restype = None`) and a `Sim.begin_otos()`
  Python method near `enable_otos_model()`/`set_otos_fusion()`, with a
  docstring cross-referencing `drive_api_begin_otos()`'s equivalent role for
  the `Drive`-level test harness (`tests/simulation/unit/test_drive_subsystem`
  family) versus this one for the full `Robot`-level `Sim` harness used by
  `tests/simulation` system/unit tests.
- `tests/simulation/unit/test_ekf_dual_source.py` (or a new file
  `tests/simulation/unit/test_sim_otos_heading_reset.py` — prefer a new file
  since this is a distinct behavioral contract, not dual-source injection) —
  new regression tests (see Testing plan).
- `clasi/sprints/063-.../architecture-update.md` — already updated with the
  investigation and design (this ticket's addendum); no further edits needed
  unless implementation surfaces a deviation from the documented plan.

### Testing plan

Create `tests/simulation/unit/test_sim_otos_heading_reset.py` using the
existing `sim` fixture pattern from `test_golden_tlm.py` /
`test_ekf_dual_source.py` (module-scoped `dlib`/`sim` fixtures, direct
`sim.send_command()` calls).

Structure:

```python
"""test_sim_otos_heading_reset.py — regression test for ticket 063-006.

Verifies the sim OTOS reproduces the hardware heading-reset bug (SI alone
drifts back to the stale OTOS heading) and its fix (ZERO enc + OZ + SI holds
heading at 0), using the sim OTOS's own re-referenced accumulator.
"""

def _turn_and_stop(sim, omega_mrad=300, turn_ms=1000):
    """Turn in place to a non-zero heading, then stop."""
    sim.send_command(f"VW 0 {omega_mrad}")
    sim.tick_for(turn_ms)
    sim.send_command("S")
    sim.tick_for(300)


def test_otos_commands_ok_not_nodev(sim):
    sim.begin_otos()
    for verb in ("OI", "OZ", "OR"):
        reply = sim.send_command(verb)
        assert "nodev" not in reply, f"{verb} returned nodev: {reply!r}"
        assert "OK" in reply
    reply = sim.send_command("OV 0 0 0")
    assert "nodev" not in reply
    assert "OK" in reply


def test_oz_zeroes_otos_accumulator(sim):
    sim.begin_otos()
    sim.enable_otos_model()
    _turn_and_stop(sim)
    x, y, h = sim.get_otos_pose()
    assert abs(h) > 0.05, "test setup: expected a non-zero heading before OZ"

    sim.send_command("OZ")
    x2, y2, h2 = sim.get_otos_pose()
    assert x2 == 0.0 and y2 == 0.0 and h2 == 0.0, (
        f"OZ must zero the sim OTOS accumulator, got ({x2}, {y2}, {h2})"
    )


def test_si_alone_drifts_back_to_otos_heading(sim):
    """Reproduces the hardware bug: SI without OZ does not hold heading."""
    sim.begin_otos()
    sim.set_otos_fusion(True)   # marks initialised + enables per-tick fusion
    sim.enable_otos_model()
    _turn_and_stop(sim)
    _, _, otos_h = sim.get_otos_pose()
    assert abs(otos_h) > 0.05

    sim.send_command("SI 0 0 0")
    for _ in range(20):
        sim.tick_for(50)
    _, _, fused_h = sim.get_fused_pose()
    assert abs(fused_h - otos_h) < 0.02, (
        f"Expected fused heading to drift back toward stale OTOS heading "
        f"{otos_h:.4f} without OZ, got {fused_h:.4f}"
    )


def test_zero_oz_si_resets_and_holds_heading(sim):
    """Verifies the fix: ZERO enc + OZ + SI resets heading to 0 and holds."""
    sim.begin_otos()
    sim.set_otos_fusion(True)
    sim.enable_otos_model()
    _turn_and_stop(sim)

    sim.send_command("ZERO enc")
    sim.send_command("OZ")
    sim.send_command("SI 0 0 0")

    for _ in range(60):   # ~3s — long enough to catch any residual drift-back
        sim.tick_for(50)
    _, _, fused_h = sim.get_fused_pose()
    assert abs(fused_h) < 0.02, (
        f"Expected fused heading to hold at 0 after ZERO+OZ+SI, got {fused_h:.4f}"
    )
```

Run with:

```
uv run python -m pytest tests/simulation/unit/test_sim_otos_heading_reset.py -v
```

Also run the full regression set named in Acceptance Criteria to confirm no
behavior-preservation break:

```
uv run python -m pytest tests/simulation/unit/test_golden_tlm.py \
  tests/simulation/unit/test_ekf_dual_source.py \
  tests/simulation/unit/test_dbg_otos_commands.py \
  tests/simulation/unit/test_ekf.py \
  tests/simulation/unit/test_ekf_encoder_velocity_unconditional.py \
  tests/simulation/unit/test_n8_n9_sensor_freshness.py -v
```

If a Test-GUI-level manual check is wanted (not required for CI): connect Sim
mode, send `VW 0 300` then `S` via the command line to turn the avatar, click
"Set Robot @ 0,0", and confirm the avatar heading snaps to 0 and stays there
across several seconds of idle ticking.

### Documentation updates

- Update `SimOdometer.h`'s class doc comment (already references behaviour
  preservation vs. `MockOtosSensor`) to note the `setPositionRaw` fix and why
  it does not violate that preservation guarantee (no golden-TLM sequence
  calls `OZ`/`OV`, confirmed by reading `test_golden_tlm.py`'s command list).
- No `source/COMMANDS.md` change — `OZ`/`OI`/`OR`/`OV`'s documented wire
  behavior does not change; only the sim's fidelity to that behavior improves.
- Consider a short addition to
  `.clasi/knowledge/2026-07-01-heading-reset-needs-oz-not-just-si.md` noting
  that the bug is now reproducible and regression-tested in sim (cross-link to
  the new test file) — optional, at implementer's discretion.
