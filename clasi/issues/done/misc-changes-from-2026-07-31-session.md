---
status: done
priority: medium
---

# Remaining changes from the 2026-07-31 session (abandoned; captured here)

Smaller items from the same working session, recorded so nothing is lost when
the working tree is discarded. Each is independent.

## 1. Trace baselines only refreshed while appending

`TraceModel._feed_otos` / `_feed_fused` sat wholly inside `if append:`, so while
the robot was idle their baselines were never refreshed — while the encoder
trace's *was*. The pose kept moving under a stale baseline, and the next move
drew fused/OTOS from the OLD origin while the encoder trace continued correctly.
Reported as *"the fused trace is starting off at zero and the encoder trace is
starting off where I left off."*

Fix: give both an `append` parameter so baseline bookkeeping always runs and
only the polyline append is gated — matching `_feed_encpose`. **Whether a trace
is continuous across moves must not depend on which source it is.**

## 2. TestGUI Geometry & Actuation overrides removed

Stakeholder: *"Remove these from the TestGUI, and always use the robot's
configured values."* The Sim Errors panel had a `trackwidth` spin box feeding
`SimLoop(track_width=...)` from a prefs file, defaulting to the **raw** 128.0 mm
— so every Sim session ran ~10% different geometry from the robot it was
simulating (tovez: raw 128.0 vs effective 140.4), and the discrepancy was
editable in a dialog, which is how it survived unnoticed. Replaced by
`_effective_track_width(config)` reading the robot's own config. Touches
`sim_prefs.py`, `__main__.py`, `transport.py`.

## 3. `tovez_nocal.json` rotation constants were fitted to a Sim artifact

`rotation_gain` / `rotation_gain_neg` carried 1.006 / +12.1 deg — values fitted
against a **Sim artifact**, not the robot, injecting ~12.5 deg of
under-rotation into every turn on the `nocal` profile. Reset to identity. Any
future rotation calibration must be fitted against camera truth on the
playfield and say so in the file.

## 4. `wheel_gain_*` / `wheel_intercept_*` silently dropped by pydantic

Eight config keys were declared in the robot JSON but absent from
`ControlConfig`, so pydantic dropped them without complaint and they never
reached the robot. Fixed by declaring them and allowlisting in
`config_sync_allowlist.json`. **A config key that is silently ignored is worse
than one that errors** — consider a strict mode that rejects unknown keys, or a
round-trip assertion that every key in the JSON reaches the firmware.

## 5. `sim_configure_drive` binding

`SimHarness` never installed the drive calibration; `pid.kff` had been standing
in for it, so removing the `kff` misrouting dropped the Sim to 3 mm/s. Added a
ctypes binding so the Sim installs the same calibration the robot does. Touches
`sim_ctypes.cpp`, `sim_loop.py`, `src/sim/CMakeLists.txt`.

## 6. Incidental / not to be carried forward

`.clasi/.clasi.db`, `uv.lock`, `pyproject.toml`, `config/dotconfig.yaml`,
`build.py`, `data/testgui/camera_prefs.json` — environment and tooling churn,
not deliberate changes.

## 7. Test churn

~15 Sim test files were touched to follow the composition-root unification
(`boot_wiring` linkage, harness `_APP_SOURCES` lists). Per
[[planner-cpp-has-four-build-source-lists]] the source lists are duplicated in
CMake x2 plus 8 pytest harness files — a missing one is a **link** error, not a
compile error. Any re-land of [[unify-sim-and-robot-composition-roots]] inherits
this.
