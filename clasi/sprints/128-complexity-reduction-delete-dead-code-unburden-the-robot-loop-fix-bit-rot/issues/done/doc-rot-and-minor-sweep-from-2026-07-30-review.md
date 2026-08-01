---
status: done
sprint: '128'
tickets:
- 128-009
---

# Doc-rot and minor sweep from the 2026-07-30 craftsmanship review

**Source:** code review 2026-07-30 — the MINOR/NOTE findings not absorbed by
a larger issue. One mechanical-sweep ticket's worth of small, independent
fixes.
**Priority:** P2 — but do it as ONE batch pass (the project's established
mechanical-sprint pattern), because each item is exactly the kind of stale
signpost that misdirects a bug hunt.

## The list

**Stale docs certifying broken/changed things**
- [x] `src/host/robot_radio/DESIGN.md` `io/` row: says `repl`/`stop` are
  live with no caveat, and that `turnto`/`goto` route "through the dead half
  of nezha.py" (they route through `nav/camera_goto.py`, which has no row at
  all). Re-audit the row, add a `nav/` row. (03 MINOR) — DONE 128-009: `io/`
  row corrected (`cli.py`'s `turnto`/`goto` route through
  `nav/camera_goto.py`, not `nezha.py`); the `nav/` row (which already
  existed, added by an earlier ticket in this sprint) updated to record
  128-002's loud NotImplementedError gate on `camera_goto.py`/`navigator.py`.
- [x] `io/sim_loop.py:1073-1078`: `step()` docstring says "50ms sim-time
  each"; `_CYCLE_DURATION_S = 0.040`. Fix to 40 ms. (03 NOTE) — DONE 128-009.
- [x] `src/firm/app/robot_loop.cpp:502-503`: trailing whitespace. (01 NOTE)
  — DONE 128-009.

**Unit-suffixed identifiers that survived the rename sprints**
- [x] `robot.py:20,24`, `nezha.py:231,244`, `cutebot.py:106,111`: parameters
  named `ms`/`mm` → name the quantity, unit in `# [ms]` tag. (03 NOTE) —
  DONE 128-009: renamed to `duration`/`distance` with `# [ms]`/`# [mm]` tags
  in all three files (`Robot` ABC + both concrete implementations); all
  call sites were positional, no keyword-arg breakage.
- [x] `nezha_kinematic.py:66-126`: `_trackwidth_m`, `_encoder_offset_m`,
  `left_total_m`, `left_delta_m` etc. — strip per
  `.claude/rules/coding-standards.md` (skip if the file is deleted by the
  Nezha rebuild issue first). (03 NOTE) — DONE 128-009: file still exists
  (not deleted by any Nezha-facade rebuild this sprint); renamed
  `_trackwidth_m`→`_trackwidth`, `_encoder_offset_m`→`_encoder_offset`,
  `left_total_m`/`right_total_m`→`left_total`/`right_total`,
  `left_delta_m`/`right_delta_m`→`left_delta`/`right_delta`, all with
  `# [m]` tags. No test imports `NezhaKinematic`, so no test breakage.

**Fail-open / contract stragglers**
- [x] `config/robot_config.py:135`: `laser_port: Optional[int] = 4` — the
  one non-None default in the model; make it `None` (togov already opts out
  explicitly). (03 NOTE) — DONE 128-009: default changed to `None`; no test
  depended on the old default.
- [x] `field/geofence.py`: `checkPlayfieldLights()` and
  `_playfieldLightsOn()` duplicate the Shelly status fetch — share one
  helper. (03 MINOR) — DONE 128-009: both now call a shared
  `_fetchPlayfieldLightsStatus()` helper; original per-function warning
  messages preserved exactly (helper returns `(status, exc)` so each caller
  keeps its own wording).
- [x] `testgui/canvas.py:214` imports `media.movie._deskew_frame` across a
  package boundary — promote `_deskew_frame` to public. (03 NOTE) — DONE
  128-009: renamed to `deskew_frame` in both `media/movie.py` and its
  `canvas.py` import/call sites. No test referenced the private name.

**Landmine annotations (cheap comments that prevent expensive confusion)**
- [x] `kinematics/differential_drive.py`: CW-positive against the project's
  CCW convention — add a WARNING block at the top; delete the module instead
  if the Nezha rebuild removes its one caller. (04 MINOR §6) — DONE 128-009:
  its one caller (`nezha_kinematic.py`) was NOT removed by any Nezha-facade
  rebuild this sprint, so the module was kept and got the WARNING block
  (CW-vs-CCW convention) plus a note on its orphan status per DESIGN.md's
  `kinematics/` row.
- [x] `odometry.h`/`estimation.h`: one-line float32 unwrapped-heading
  precision bound note (at ~10,000 rad accumulated, epsilon ≈ a per-cycle
  increment). (02 NOTE §8) — DONE 128-009: both files now live under
  `src/motion/` (post-122 motion-library extraction) as `odometry.h`
  (`Motion::Odometry::theta_`) and `state_estimator.h`
  (`Motion::BodyPeer::heading`) respectively; precision-bound comment added
  to both fields.
- [x] `robot/clock_sync.py`: note that `record_ping()` never updates
  `_last_sync_s` and samples span reboots — zero live callers today, so a
  docstring warning suffices. (03 NOTE) — DONE 128-009.

**Small decisions to record (each one line of stakeholder input)**
- [x] `testgui/turn_control.py`'s TCP control socket has zero callers: add a
  small bench script that exercises it, or remove until one exists. (05 §9)
  — DECIDED 128-009 (programmer's call): KEPT, not removed and no dedicated
  bench script added. Reason recorded in the module's own docstring: this
  is a deliberately exposed external automation entry point (an
  agent/script driving the running GUI from outside the process), not
  orphaned application logic, and `test_gui_button_acceptance.py` already
  exercises the same underlying wire primitive (`SEG 0 <cdeg>`) this
  socket's `"turn"` command forwards, at the entry-point level — so the
  bit-rot risk a dedicated bench script would guard against is already
  covered.
- [x] `telemetry_panel.py`: add a "no frames in N s" staleness banner so a
  dead link doesn't render as fresh values. (05 NOTE §11) — DONE 128-009:
  a hidden-by-default red banner (objectName `telemetry_stale_banner`)
  shows "no frames in Ns" once `_STALE_AFTER_S` (3s) has elapsed since the
  last `update_frame()` call, polled every `_STALE_POLL_INTERVAL_MS`
  (500ms) by a QTimer; covered by a new test in
  `src/tests/testgui/test_telemetry_panel.py`.
- [x] Line sensor is 100% dead on current firmware (`cycleCount_` never
  incremented — self-documented in `robot_loop.h:171-181`). Decide: fix the
  parity tick, or record the deferral in an issue of its own so nothing
  downstream waits on `kFlagLinePresent`. (01 MINOR §5) — DEFERRED 128-009
  per the ticket's own explicit instruction (this is a firmware BEHAVIOR
  fix, not a doc-rot item): filed as
  `clasi/issues/line-sensor-dead-parity-tick-cycle-count-never-increments.md`,
  NOT fixed and NOT silently dropped.

## Acceptance

- Each checkbox either done or explicitly struck with a reason in this file.
  — All items done (see per-item notes above); none struck, everything in
  this list's own scope was completable within 128-009.
- `uv run python -m pytest` green; `grep` spot-checks per item. — Verified
  128-009 (targeted test runs; see the ticket's own final report for exact
  commands/counts — the full suite was not re-run wholesale per this
  sprint's stated testing policy of targeted runs over touched files).

## Additional sweep items (team-lead-accumulated from earlier tickets this sprint)

Not part of the original 2026-07-30 review checklist above, but folded into
128-009's scope by the team-lead as sprint-wide follow-on cleanup from
tickets 001/005/007. Recorded here for the same done-or-struck-with-reason
discipline.

- [x] Swallowed-halt sites outside ticket 001's own table:
  `calibration/angular.py:383`/`calibration/linear.py:276` (bare
  `ser.write_line("STOP")` in `except: pass`), `robot/nezha.py`'s `speed()`
  `GeneratorExit` handler (`send_fast("STOP")` in `except: pass`). DONE
  128-009: each now logs on failure (`print()` in the two calibration
  scripts, matching their existing print-based output convention; a new
  `logging.getLogger(__name__)` in `nezha.py`, matching `canvas.py`'s
  existing pattern) instead of silently swallowing. These are legacy
  protocol-v2 text-plane files, so a log line (not a reroute through
  `halt_now()`/`estop()`) is the accepted minimal fix per the team-lead's
  own instruction.
- [x] Keepalive machinery orphaned by ticket 005's `KeyboardDriver`
  deletion: `Transport.arm_keepalive()`/`disarm_keepalive()`
  (`testgui/transport.py`), `SerialConnection.start_keepalive()`/
  `stop_keepalive()` (`io/serial_conn.py`). VERIFIED zero live callers
  (grep across `src/host`/`src/tests`); NOT deleted — deletion turned out
  nontrivial: `_keepalive_loop()`'s idle-gate reads `_last_write_s`, which
  is written by ~6 other write call sites throughout `serial_conn.py`
  (`send()`, `send_fast()`, etc.); deleting the keepalive methods cleanly
  without leaving `_last_write_s` as new dead bookkeeping requires
  auditing/un-threading every one of those call sites — a materially
  bigger change than this mechanical-sweep ticket's scope. Filed as
  `clasi/issues/orphaned-serial-keepalive-machinery-after-keyboarddriver-deletion.md`.
- [x] `testgui/commands.py` after ticket 004: `COMMANDS` empty,
  `build_wire_string()` a tested pure function with zero live callers.
  ALREADY DECIDED (by 128-004 itself, per that file's own module
  docstring): KEEP, with the reason already recorded inline
  ("`build_wire_string()` is kept as a small, independently-tested pure
  function ... in case a future command row needs it again"). 128-009
  reviewed and confirms this decision stands; no further edit needed.
- [x] `planner/profile.py` after ticket 007: only consumers are its own
  unit test and `planner/__init__.py`'s re-export. NOT deleted (per
  explicit instruction) — filed as
  `clasi/issues/planner-profile-py-dormancy-candidate.md`, a dormancy
  candidate for a future sprint's explicit decision, matching the
  `planner/executor.py` precedent (128-007 deleted that only after an
  explicit call, not by default).
