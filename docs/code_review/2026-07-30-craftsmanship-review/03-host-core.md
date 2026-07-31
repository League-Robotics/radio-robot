# Host Core Craftsmanship Review — `robot/`, `io/`, `config/`, `sensors/`, `field/`, `media/`

**Scope:** all `.py` under `src/host/robot_radio/{robot,io,config,sensors,field,media}`
(~17k lines), read in full. Reviewed against `docs/code_review/GUIDELINES.md`.
Repo state: `sprint/127-host-side-path-planner-goto-and-path-following`, WIP from
a parallel session, reviewed as-is.

## Verdict

**Rework, not merge.** The safety-critical halt path (`field/geofence.py`) is
genuinely excellent and should be the template for the rest of the tree — but
everything upstream of it that is supposed to *drive* the robot is in
significantly worse shape than the tree's own documentation admits. The
protocol v4→v5 rewrites (culminating in sprint 124's binary-only cutover) left
three separate, still-imported call surfaces — `robot/nezha.py` (the `Nezha`
driver facade), `io/calibrate.py`, and `nav/camera_goto.py` (reached from
`io/cli.py`'s `goto`/`turnto`) — calling `NezhaProtocol`/`SerialConnection`
methods that no longer exist. This is not dead code by any honest measure:
`data/robots/active_robot.json` points at `tovez.json`, whose
`"hardware_model": "DFRobot Nezha"` makes `robot/connection.py::make_robot()`
construct exactly this `Nezha` facade for the currently-configured robot, and
`io/cli.py`'s dispatch table (`goto`, `turnto`, `enc`, `grip`, …) and
`io/repl.py`'s command verbs reach it directly. A user running `rogo goto`,
`rogo turnto`, `rogo calibrate distance`, or almost any `rogo repl` motion verb
against the currently-configured robot hits an `AttributeError` on the first
call. `src/host/robot_radio/DESIGN.md` certifies several of these paths as
"live" and doesn't reflect this. Separately, the halt-verb doctrine this
project has bench-measured and written down repeatedly (`estop()`, never the
planned `stop()`, for any "halt now" event) is violated at several **live**
call sites in `cli.py`, `calibrate.py`, and `camera_goto.py`'s own
`go_to_world_camera` — even though its sibling function in the *same file*,
`spin_to_yaw_camera`, gets it right and even documents why. None of this is
subtle: it is a single grep away in every case below. The fix is not
line-patches at each call site — it's picking one current, correct calling
convention (`NezhaProtocol`'s binary `move_twist`/`move_wheels`/`stop`/`estop`/
`config`/`wait_for_ack` surface) and rebuilding `Nezha`, `calibrate.py`, and
`camera_goto.py`'s halt paths against it, the way `field/geofence.py` and
`sensors/odometry.py` already do it correctly.

---

## Findings

### CRITICAL — `Nezha` (the constructed, active robot driver) calls ~25 `NezhaProtocol` methods that do not exist

**Category:** correctness / accretion

**Evidence:** `robot/nezha.py` calls `self._proto.ping()` (163), `.get_id()`
(168, 645), `.grip()` (208), `.stream_drive()` (221, 577), `.timed()` (240),
`.wait_for_evt_done()` (241, 262, 408, 443), `.distance()` (260), `.stream()`
(327, 341, 344, 359, 370, 412, 446, 526, 556, 665), `.go_to()` (406, 413),
`.turn()` (441, 447), `.zero_encoders()` (456), `.drive()` (562), `.snap()`
(673, 691), `.zero_otos()` (702), `.otos_init()` (720),
`.otos_reset_tracking()` (724), `.otos_set_position()` (728),
`.set_internal_pose()` (763), `.otos_get_position()` (781),
`.otos_{set,get}_{linear,angular}_scalar()` (785–797), `.port_{read,write}*()`
(805–817), `.get_config()` (653).

`NezhaProtocol`'s actual method surface (`robot/protocol.py`, every `def`
enumerated) is: `is_open, mode, send, send_fast, read_lines,
read_pending_lines, parse_response, set_config, set_config_binary,
move_twist, move_wheels, move, wheels, estop, stop, config, otos_config,
estimator_config, wait_for_ack, read_binary_tlm_frames,
read_pending_binary_tlm_frames, tlmOn/Off/Now`. None of the names `Nezha`
calls exist there — they are protocol-v2/v3-era verb names
(`ping`/`go_to`/`turn`/`distance`/`stream`/`snap`/`otos_*`) that sprint
103/104/116/124's arm-prune deleted from `NezhaProtocol` but that `Nezha`
itself was never migrated off.

**Failing scenario:** `data/robots/active_robot.json` → `data/robots/tovez.json`
→ `"hardware_model": "DFRobot Nezha"`. `robot/connection.py:314-316`:
```python
hw_model = getattr(cfg, "hardware_model", "cutebot").lower() if cfg else "cutebot"
if "nezha" in hw_model:
    robot = Nezha(NezhaProtocol(conn))
```
`"nezha" in "dfrobot nezha".lower()` is `True` — this is exactly what gets
constructed for the currently-configured robot. `io/cli.py`'s dispatch table
routes `goto`/`enc`/`grip`/etc. straight to `robot.go_to()`/`robot.
zero_encoders()`/`robot.grip()`. Any of these calls raises `AttributeError`
immediately, on real hardware, against the active robot config. This is
tracked in `clasi/issues/later/nezha-facade-and-midlayer-dead-verb-residue.md`
as a known pending gap; this review confirms it is still true against the
current tree and traces it to the specific live CLI verbs that hit it.

**Design answer:** this is not a per-call patch — `Nezha` needs to be rebuilt
on `move_twist`/`move_wheels`/`wheels`/`stop`/`estop`/`wait_for_ack`, the way
`field/geofence.py` and `nav/camera_goto.py::spin_to_yaw_camera` already
correctly call the current surface. Until that lands, the class should fail
loudly (or the CLI verbs that reach it should be disabled) rather than
presenting as a working driver.

---

### CRITICAL — `io/repl.py`'s command-confirmation path reads a `TLMFrame` field deleted by sprint 124-008

**Category:** correctness

**Evidence:** `io/repl.py:105-106` (`RogoSession.confirm`):
```python
if f.ack is not None and f.ack.corr_id == corr_id:
    return f.ack
```
`TLMFrame` (`robot/protocol.py`, fields enumerated lines 412-437) has no `ack`
field. Its own docstring says so explicitly (protocol.py:371-373): "`ack_corr`/
`ack_err`/`ack` — DELETED (124-008, issue §B4): the single 'freshest ack' slot
they adapted is gone; ring membership in `acks` below already means 'really
acked.'" The only surviving field is `acks: list[AckEntry]`.

**Failing scenario:** `confirm()` is called from `_ack_str()` (repl.py:153),
which every command-issuing verb (`twist`, `stop`, `drive`, `turn`, `config`,
`raw`) uses to report its outcome. As soon as one telemetry frame is pumped
during any of those verbs, `f.ack` raises `AttributeError`. `dispatch()`
(repl.py:354-381) only catches `CommandError`/`ConnectionError`, so this
propagates out of the whole REPL loop and kills the process. Net effect: the
one entry point `io/cli.py`'s own docstring and `DESIGN.md` certify as live
(`repl`/`run`/`exec`) can read telemetry but cannot confirm a single command
it issues — only the read-only verbs (`enc`/`pose`/`otos`/`vel`/`line`/
`color`/`tlm`, which use dynamic `getattr` against `session.latest()`) still
work.

**Design answer:** `confirm()` must scan `f.acks` for a matching `corr_id`,
mirroring `NezhaProtocol.wait_for_ack()`'s ring-scan — not the deleted scalar
slot.

---

### CRITICAL — `nav/camera_goto.py` (the current `goto`/`turnto` implementation) calls `NezhaProtocol.drive()`, which does not exist

**Category:** correctness

**Evidence:** `nav/camera_goto.py:149` (`spin_to_yaw_camera`) and `:295`
(`go_to_world_camera`) both call `proto.drive(...)`. `NezhaProtocol` has no
`drive` method (full method list above). `io/cli.py:655-656` and `:741-742`
obtain `proto` via `getattr(robot, "_proto", None)` and pass it into these
functions — this is the current, reworked implementation behind
`cmd_turnto`/`cmd_goto` (ticket 035-001 moved these off the dead `Nezha`
facade specifically to fix a prior instance of this exact defect class, but
introduced a new instance of it against a different retired verb).

**Failing scenario:** `rogo turnto 90` reaches `spin_to_yaw_camera`'s control
loop and calls `proto.drive(-direction*speed, direction*speed)` at line 149 →
`AttributeError`. `rogo goto 30 10` fails identically inside
`go_to_world_camera`'s burst-drive loop (line 295) and via its own re-aim call
into `spin_to_yaw_camera`. No test under `src/tests` imports `camera_goto`
(confirmed empty grep) — this module appears never exercised since its
extraction.

**Design answer:** replace both `proto.drive(...)` calls with a bounded
`proto.move_wheels(v_left, v_right, stop_time=..., timeout=...)` (or
`wheels()` for the fixed-duration teleop shape) — the same fix direction as
the `Nezha` finding above, since this is the same retired-verb residue in a
different file.

---

### CRITICAL — no call site in `cli.py`/`repl.py`/`robot_mcp.py`/`calibrate.py` ever calls `estop()`; several halt-now paths use the planned `stop()` instead, sometimes with the failure swallowed

**Category:** explicitness (halt-path rule, §4) / correctness

**Evidence — confirmed live call sites, all using `stop()` where the context
is "halt now":**
- `io/robot_mcp.py:560` — the MCP tool literally named `stop` (registered
  `:200-202`, described "Stop motors") calls `_robot.stop()`.
- `io/cli.py:540/555/576` — every `except KeyboardInterrupt` handler in
  `cmd_drive`/`cmd_drive_stream` falls through to `robot.stop()`.
- `io/cli.py:675-679`, `:765-769` — `cmd_turnto`/`cmd_goto`'s `finally` blocks
  call `proto.stop()` wrapped in `try: ... except Exception: pass` — a halt
  call whose own failure is silently discarded.
- `io/repl.py:129-133` (`RogoSession.close()`, invoked from the top-level
  `finally` after `KeyboardInterrupt`) — the identical `try/except Exception:
  pass` around `self.proto.stop()`.
- `io/calibrate.py:501-505` and `:908-916` — both cleanup `finally` blocks
  (reached on normal completion, `q`, **and** `KeyboardInterrupt`) call
  `proto.stop()` inside `try: ... except Exception: pass`.
- `nav/camera_goto.py:242, 248, 254, 300` — `go_to_world_camera`'s own
  arrival/timeout/re-aim halts all call `proto.stop()`.

This project has bench-measured and repeatedly written down the concrete cost
of this exact substitution: `robot/nezha.py:194-204`'s own docstring and
`.claude/rules/playfield-testing.md` both cite the same measurement — a
`stop()` sent 0.5s into an in-flight 400mm leg rode out the **entire leg**
(39.8cm, 5.9s) before taking effect, versus `estop()`'s 2.9cm/0.10s on the
identical repro. None of the six call sites above use `estop()`. The one
correct usage in this whole area is `camera_goto.py:142,146`
(`spin_to_yaw_camera`), which even carries the "estop(), not stop()" reasoning
comment inline — its sibling function in the *same file*,
`go_to_world_camera`, does not apply that same reasoning to its own halts.

**Failing scenario:** an operator (or an LLM driving the MCP server) calls the
`stop` MCP tool expecting an immediate halt mid-motion and instead gets a
queued stop that runs out whatever move is already active — per this
project's own measurement, up to a full remaining leg and several seconds of
continued travel. A Ctrl-C during `rogo drive`/`rogo repl`/`rogo calibrate`
gets the same queued behavior instead of a halt, and if the queued `stop()`
call itself raises (degraded connection), the operator gets no signal at all
that the "stop" didn't happen — indistinguishable from a stop that worked,
which is the exact failure mode this project's own history names ("a fence
detected correctly and stopped nothing").

**Design answer:** every one of the six call sites above should call
`estop()`, not `stop()`, and the two `except Exception: pass` wrappers around
a halt call (`cli.py:675-679/765-769`, `calibrate.py:501-505/908-916`,
`repl.py:129-133`) should at minimum log the exception rather than discard
it — matching `field/geofence.py::_halt()`'s retry-then-raise pattern (see
"What's good," below), which is the correct template already present in this
same tree.

---

### CRITICAL — `io/calibrate.py`'s two live commands (`rogo calibrate distance` / `rogo calibrate turns`) call `NezhaProtocol` methods that do not exist

**Category:** correctness / accretion

**Evidence:** `io/calibrate.py:400-401` (`proto.zero_encoders()`,
`proto.zero_otos()`), `:406` (`proto.distance(...)`), `:413,419,851`
(`proto.read_encoders()`), `:420` (`proto.read_otos()`), `:748,829`
(`proto.set_stream_otos(...)`). `_make_proto_cfg()` (line 263) constructs a
real `NezhaProtocol(conn)`; none of these method names exist on it (same
method enumeration as the `Nezha` finding above — these are the identical
protocol-v2-era verb names). Both commands are wired live in
`io/cli.py:1619-1622`, and `rogo` is the installed CLI entry point
(`pyproject.toml`).

**Failing scenario:** `rogo calibrate distance` connects, prints its banner,
and on the first trial hits `proto.zero_encoders()` (line 400) →
`AttributeError`. `rogo calibrate turns` fails identically at line 744.
Neither calibration command can complete a single trial. Even the calls that
would not immediately `AttributeError` — the raw-text pushes
`conn.send(f"OL{ol_int8:+d}", ...)` (line 487), `conn.send(tn_cmd, ...)`
(line 753), `conn.send(f"OA{oa_int8:+d}", ...)` (line 866) — are dead on
arrival for a second, independent reason: `NezhaProtocol.send()`'s own
docstring (protocol.py:879-887) states "the P4 firmware has no text-plane
command parser... a text line sent through it reaches no live firmware
handler." A fix that only patches the `AttributeError`s still leaves every
`OL`/`OA`/`TN`/`K+TW`/`K+ML`/`K+MR` push reaching nothing.

**Design answer:** rewrite against the current binary surface —
`otos_config(linear_scale=...)`/`otos_config(angular_scale=...)` in place of
the `OL`/`OA` text pushes, a bounded `move_wheels(..., stop_distance=...)` in
place of the `D`-command drive, `config()` in place of `K+TW`/`K+ML`/`K+MR`.
Whether the `zero_encoders`/`zero_otos` calls should be restored at all is a
separate question worth checking against this project's own "encoders are
never reset — software offset" convention before assuming they belong back.

---

### MAJOR — interface bleed: `nezha.py`, `cli.py`, and `robot_mcp.py` reach around `NezhaProtocol`'s/`Robot`'s public surface into private fields, even where a public equivalent already exists

**Category:** interface-bleed

**Evidence:**
- `robot/nezha.py:326,365,369,529,537,547,551,555` —
  `self._proto._conn.send_fast(...)`; `robot/nezha.py:282,330,539` —
  `self._proto._conn.read_lines(...)`. `NezhaProtocol` already publicly
  exposes `send_fast()` (protocol.py:889) and `read_lines()`/
  `read_pending_lines()` (protocol.py:893,897) — this isn't excused by a
  missing public API, it's simply not used. These are exactly the
  abort/timeout/generator-close (halt-adjacent) paths — where going through
  the documented adapter matters most.
- `io/cli.py:655,741` (`cmd_turnto`/`cmd_goto`) — `proto = getattr(robot,
  "_proto", None)`; `io/robot_mcp.py:96` — `push_calibration(_robot._proto,
  _config)`. Three unrelated call sites across two files agree by convention
  to reach past `Robot`/`Nezha`'s public surface for the raw protocol object
  rather than `Robot` exposing a supported accessor.

**Design answer:** if `send_fast`/`read_lines` passthroughs, or a
camera-closed-loop-drive/calibration-push protocol handle, are legitimately
needed by callers, that is `Robot`/`Nezha`'s contract to own explicitly (a
named accessor or the operation itself) — not three independent calling
sites agreeing informally to poke `_proto`.

---

### MAJOR — three independent, unreconciled pose/state trackers inside `robot/` — the same historical anti-pattern the review guidelines name explicitly

**Category:** interface-bleed ("is there exactly one way in?") / placement

**Evidence:**
- `Nezha._apply_tlm` (nezha.py:587-633) builds `RobotState.pose` from
  `tlm.pose` via `math.radians(heading/100.0)`.
- `NezhaState._apply_tlm` (nezha_state.py:124-165) independently builds a
  **second** `RobotState` (`self.robot_state`) via
  `heading/18000.0*math.pi` — same result, separately re-derived — but only
  inside `if tlm.pose is not None:`, and it **omits** `encoders`/`twist`/
  `line`/`color`/`otos_pose` even though `NezhaState` is simultaneously
  tracking those exact values as sibling raw attributes in the same method. A
  consumer reading `state.robot_state.encoders` gets `None` forever while
  `state.encoders` has live data on the same object.
- `NezhaKinematic._update_odometry` (nezha_kinematic.py:106-140) layers a
  **third** estimator (a WPILib `DifferentialDriveOdometry`), fed from
  `NezhaState`'s raw encoder/heading attributes, independent of either
  `RobotState.pose` above it.

**Mechanism:** none of the three delegates to or is reconciled with the
others; each hand-rolls its own unit conversion and its own notion of
"current pose." This is the same shape the guidelines cite by name from the
2026-06-11 review ("three independent go-to-point stacks and three pose
estimators with no owner").

**Design answer:** pick one pose/state owner (`Nezha.robot.state`, per the
class's own "canonical" docstring claim, is the natural candidate); the other
two layers should read from it rather than reconstruct independently from raw
telemetry.

---

### MAJOR — `sensors/calibration.py` sends wire commands its own docstring says are retired; `sensors/cam_tracker.py`/`odom_tracker.py` are unreferenced duplicates of `sensors/odometry.py`

**Category:** accretion

**Evidence:**
- `sensors/calibration.py`'s own docstring states the `K`-command wire family
  it implements "has been retired" in favor of `SET`, yet `_MAPPING`,
  `apply()`, and `to_wire_values()` still build and send exactly those
  retired `K*` commands. Zero live callers anywhere in `src/host`/`src/tests`
  (only self-references and a re-export in `sensors/__init__.py`) — and a
  *separate*, actually-live `robot_radio.calibration` package already exists
  (referenced by `ControlConfig`'s own docstring) under a nearly identical
  name, sitting right next to this dead one.
- `sensors/cam_tracker.py` (`CamTracker`) and `sensors/odom_tracker.py`
  (`OdomTracker`, its non-deprecated part) have zero live callers repo-wide,
  yet remain exported public API in `sensors/__init__.py` (with a lazy-load
  slot reserved for `CamTracker`) alongside their functional replacement,
  `sensors/odometry.py::Odometry`. `CamTracker.update()` also independently
  reproduces the "frozen-anchor tracker" stale-pose hazard this project has
  hit before: it returns `True`/`False` but leaves `self.pos`/`self.yaw` at
  the last-good value on a miss, with no `is_valid`-style staleness flag the
  way `Odometry` correctly has.

**Design answer:** delete `sensors/calibration.py`'s `K`-command path (the
live `robot_radio.calibration` package is the one to keep) and either wire
`CamTracker`/`OdomTracker` to a real caller with the same staleness contract
`Odometry` already has, or remove them — half-retired code here is
indistinguishable, to a future reader, from a second supported pose source.

---

### MAJOR — `VisionConfig.robot_tag_id` defaults to the fixed field-origin tag, failing open instead of closed

**Category:** correctness (fail-closed model, §5)

**Evidence:** `config/robot_config.py:77`, `robot_tag_id: int = 1`. Tag 1 is
the fixed, always-visible field-origin AprilTag (`.claude/rules/
playfield-testing.md`: "Tag 1 is fixed... if tag 1 is missing, the problem is
the room, not the robot"). Every real robot config already overrides this —
`tovez.json`/`tovez_nocal.json`: 100, `togov.json`: 101 — and three
independent call sites (`io/cli.py:164,640,738`, `io/calibrate.py:314,648`)
already hardcode a fallback of `100`, never `1`, for the no-config case: the
rest of the codebase has already learned 1 is wrong; only the pydantic
default has not.

**Failing scenario:** a robot JSON that omits `robot_tag_id` (or a code path
that constructs `VisionConfig()` directly rather than through one of the
call sites above) silently tracks the stationary origin tag as "the robot."
Because tag 1 is always visible, nothing about this looks unconfigured:
`Odometry.is_valid` stays `True`, camera-vision code reports a confident,
static, wrong pose rather than failing loudly — the opposite of sprint 114's
established fail-closed model.

**Design answer:** make `robot_tag_id` a required field (or `None`-default
with an explicit "unconfigured" error at first use), matching the pattern
every sibling config field in this same file already follows.

---

### MINOR — `Cutebot` (the non-Nezha driver, also live via `connection.py:319`) has two independent halt/correctness bugs

**Category:** correctness

**Evidence:**
- `robot/cutebot.py:71-78` — `speed()`'s `GeneratorExit` handler:
  `try: self._conn.send_fast("X") except Exception: pass`, then polls up to
  0.5s for a stop confirmation that will never arrive if the send itself
  failed — the exact "halt that raises must not be swallowed" violation, on
  the Ctrl-C/generator-close path specifically.
- `robot/cutebot.py:180` (`rotate()`) — `left_enc, right_enc =
  robot.read_encoders()` references an undefined name `robot` (not `self`,
  not a parameter, not imported) — guaranteed `NameError` on any call. No
  call sites found repo-wide (`grep -rn "\.rotate("` empty), so currently
  latent, but a live landmine in a class that is constructed for any
  non-Nezha `hardware_model`.

**Design answer:** fold into the same halt-path rework as the CRITICAL
finding above (`estop()`, propagate failures); fix or delete `rotate()`.

---

### MINOR — orchestration ritual repeated three times in `cli.py::cmd_drive`/`cmd_drive_stream` instead of living as one method

**Category:** placement

**Evidence:** the "Ctrl-C → print stopping → `robot.stop()` → drain
`read_lines(duration=500)` and log" sequence appears at `cli.py:540-549`
(stream branch), `:555-565` (plain-loop branch), and again, missing the drain
step, in `cmd_drive_stream:576-579` — a three-way near-duplicate with its own
internal inconsistency. Per §2 this is a missing method on `Robot`
(`haltAndDrain()`-shaped), not three inline rituals; will resurface unchanged
whenever `cmd_drive` itself is repaired (it currently calls the same dead
`Nezha.speed()`/`stream_drive()` verbs as the CRITICAL findings above).

---

### MINOR — accretion in `field/geofence.py`: duplicated lights-status fetch

**Category:** accretion

**Evidence:** `checkPlayfieldLights()` (geofence.py:160-190) and
`_playfieldLightsOn()` (198-208) independently fetch and parse the same
Shelly relay status endpoint. Should share one helper — low-severity since
both are correct, just duplicated.

---

### MINOR — stale `DESIGN.md` certifies broken paths as live

**Category:** accretion (§3: "fixes that don't update the design docs")

**Evidence:** `src/host/robot_radio/DESIGN.md`'s `io/` row states `repl.py`'s
docstring is "explicitly post-gut-aware" and lists `repl`/`stop`/`binary
stop` as cli.py's live subset with no caveat — both false per the CRITICAL
findings above. The same document says `turnto`/`goto` route "through the
dead half of `nezha.py`" — no longer accurate; ticket 035-001 moved them onto
`nav/camera_goto.py` (which doesn't appear in DESIGN.md's table at all), and
they now fail for the unrelated `proto.drive()` reason documented above.

**Design answer:** re-audit the `io/` row and add a `nav/camera_goto.py` row
before the next reader trusts this document over the method bodies — the
exact trap the document itself warns about elsewhere.

---

### NOTE — miscellaneous style/staleness

- `robot.py:20,24` / `nezha.py:231,244` / `cutebot.py:106,111`: parameter
  names `ms`/`mm` are themselves bare units (`speed_for_time(self, left,
  right, ms: int)`), a residue the dedicated 076-003 units-rename sprint
  otherwise cleared from this same file family.
- `nezha_kinematic.py:66,73,121,122,125,126`: `_trackwidth_m`,
  `_encoder_offset_m`, `left_total_m`/`right_total_m`,
  `left_delta_m`/`right_delta_m` — unit-suffixed fields, one (`_trackwidth_m`)
  introduced by the very commit (076-003) meant to remove unit suffixes.
- `config/robot_config.py:135`, `PeripheralsConfig.laser_port: Optional[int]
  = 4` is the one field in the model with a real non-`None` default, breaking
  the "`None` = unconfigured" contract every sibling field follows.
  Currently inert (no consumer reads it), but `togov.json` already had to
  explicitly set it to `null` to opt out.
- `io/sim_loop.py:1073-1078`: `step()`'s docstring still says "50ms sim-time
  each"; the module's own adjacent comment and `_CYCLE_DURATION_S = 0.040`
  say the cycle period is 40ms (post-ticket-118). Doc-only, half-updated
  cutover.
- `testgui/canvas.py:214` imports the underscore-private
  `robot_radio.media.movie._deskew_frame` across a package boundary — a
  small, stable function worth promoting to public rather than reached
  around.
- `clock_sync.py`: `record_ping()` never updates `_last_sync_s` (only
  `ping_burst()` does), and samples accumulate unbounded across a robot
  reboot with no epoch-discontinuity detection — a real latent design gap,
  but `ClockSync` has zero live construction sites today, so this is a NOTE,
  not a finding against running code.
- `src/tests/CLAUDE.md` still instructs bench scripts to call the planned
  `stop()` in Ctrl-C/exception `finally` blocks — stale guidance that could
  lead a future author into exactly the wrong verb, even though the actual
  reference scripts (`radio_bench_gate.py`, `square_tour.py`) already do the
  right thing.

---

## What's good

- **`field/geofence.py::Geofence._halt()`** (lines 74-97) is the standout of
  this review and should be the template everywhere else in this tree: it
  retries `estop()` three times, and on total failure it **raises**
  `GeofenceViolation` with the underlying exception chained and an explicit
  `"ROBOT MAY STILL BE MOVING"` message — never swallowing a failed halt.
  `check()` runs at true 10 Hz from inside the drive loop
  (`src/tests/bench/square_tour.py:300-302`), not just between segments, and
  fails closed on tag loss rather than returning a stale "safe" verdict. This
  is a direct, correct fix for the exact historical bug class ("a fence
  detected correctly and stopped nothing") this project's own guidelines
  cite by name.
- **`sensors/odometry.py::Odometry`** has a correct, explicit staleness
  contract (`STALE_AGE`, `is_valid`, `source`) with OTOS fallback bounded by
  both re-alignment-on-fresh-fix and a max-age window — it never serves a
  stale value as fresh, the exact discipline `sensors/cam_tracker.py` (above)
  lacks.
- **`nav/camera_goto.py::spin_to_yaw_camera`**'s inline comment explaining
  exactly why `estop()` (not `stop()`) is required at arrival is a model of
  §4 explicitness — it states the physical reasoning, not just the call,
  which is precisely what makes its sibling function's omission (the CRITICAL
  halt-path finding above) easy to spot and cite.
- **`robot/clock_sync.py`** is a genuinely well-crafted, self-contained
  class independent of its current lack of a live caller — NTP-style
  min-RTT sample selection plus an OLS skew model, no external dependencies,
  an injectable clock function for testability.
- **`io/sim_loop.py`** is disciplined about its threading model: every
  timing constant carries a provenance comment tying it to a ticket and a
  physical reason, and its idle-heartbeat/full-rate state machine correctly
  reasons about a real deadlock trap before it could bite.
- **`nav/camera_goto.py`**'s extraction itself (ticket 035-001) is an
  on-the-nose instance of this project's own prescribed fix for a
  historical failure mode: the old ~165-line inline pure-pursuit loop that
  used to live inside `cmd_goto` is now a standalone, unit-testable module
  with an explicit "must NOT import from `robot_radio.io.cli`" constraint
  stated at the top to keep the dependency direction one-way — the shape is
  right even though (per the CRITICAL findings above) one call inside it
  targets a retired verb.
