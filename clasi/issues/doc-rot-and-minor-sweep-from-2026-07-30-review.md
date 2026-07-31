---
status: pending
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
- [ ] `src/host/robot_radio/DESIGN.md` `io/` row: says `repl`/`stop` are
  live with no caveat, and that `turnto`/`goto` route "through the dead half
  of nezha.py" (they route through `nav/camera_goto.py`, which has no row at
  all). Re-audit the row, add a `nav/` row. (03 MINOR)
- [ ] `io/sim_loop.py:1073-1078`: `step()` docstring says "50ms sim-time
  each"; `_CYCLE_DURATION_S = 0.040`. Fix to 40 ms. (03 NOTE)
- [ ] `src/firm/app/robot_loop.cpp:502-503`: trailing whitespace. (01 NOTE)

**Unit-suffixed identifiers that survived the rename sprints**
- [ ] `robot.py:20,24`, `nezha.py:231,244`, `cutebot.py:106,111`: parameters
  named `ms`/`mm` → name the quantity, unit in `# [ms]` tag. (03 NOTE)
- [ ] `nezha_kinematic.py:66-126`: `_trackwidth_m`, `_encoder_offset_m`,
  `left_total_m`, `left_delta_m` etc. — strip per
  `.claude/rules/coding-standards.md` (skip if the file is deleted by the
  Nezha rebuild issue first). (03 NOTE)

**Fail-open / contract stragglers**
- [ ] `config/robot_config.py:135`: `laser_port: Optional[int] = 4` — the
  one non-None default in the model; make it `None` (togov already opts out
  explicitly). (03 NOTE)
- [ ] `field/geofence.py`: `checkPlayfieldLights()` and
  `_playfieldLightsOn()` duplicate the Shelly status fetch — share one
  helper. (03 MINOR)
- [ ] `testgui/canvas.py:214` imports `media.movie._deskew_frame` across a
  package boundary — promote `_deskew_frame` to public. (03 NOTE)

**Landmine annotations (cheap comments that prevent expensive confusion)**
- [ ] `kinematics/differential_drive.py`: CW-positive against the project's
  CCW convention — add a WARNING block at the top; delete the module instead
  if the Nezha rebuild removes its one caller. (04 MINOR §6)
- [ ] `odometry.h`/`estimation.h`: one-line float32 unwrapped-heading
  precision bound note (at ~10,000 rad accumulated, epsilon ≈ a per-cycle
  increment). (02 NOTE §8)
- [ ] `robot/clock_sync.py`: note that `record_ping()` never updates
  `_last_sync_s` and samples span reboots — zero live callers today, so a
  docstring warning suffices. (03 NOTE)

**Small decisions to record (each one line of stakeholder input)**
- [ ] `testgui/turn_control.py`'s TCP control socket has zero callers: add a
  small bench script that exercises it, or remove until one exists. (05 §9)
- [ ] `telemetry_panel.py`: add a "no frames in N s" staleness banner so a
  dead link doesn't render as fresh values. (05 NOTE §11)
- [ ] Line sensor is 100% dead on current firmware (`cycleCount_` never
  incremented — self-documented in `robot_loop.h:171-181`). Decide: fix the
  parity tick, or record the deferral in an issue of its own so nothing
  downstream waits on `kFlagLinePresent`. (01 MINOR §5)

## Acceptance

- Each checkbox either done or explicitly struck with a reason in this file.
- `uv run python -m pytest` green; `grep` spot-checks per item.
