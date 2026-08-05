---
status: pending
priority: low
---

# Cruft ledger: zero-consumer code across `src/firm`, `src/motion`, and `src/host`

## Description

The 2026-08-02 post-130 review produced three cruft ledgers, each item
grep-verified zero-consumer or stale. Collected here so the list is not lost;
this is a sweep, not a design change, and it is deliberately **C** — none of it
is hurting anything today, and a sweep is cheapest folded into whichever sprint
already touches the file.

**Rule for whoever runs this:** each deletion needs a one-line "walked the
callers, found none" note in the commit. Sprint 128 and 130 both found that a
"dead" thing had one live reader. Do not batch-delete on trust.

## `src/firm`

- `Config::EstimatorBootConfig` / `ShaperBootConfig` + defaults — zero consumers.
- Generated `msg::MotorConfig` vel-gains + `setVelFiltAlpha` — baked every build,
  read by nothing; comments cite command surfaces dead since 102–107.
- JSON keys with no generator consumer: `v_wheel_max=620` (a two-generations-stale
  plant claim), `vel_*`, `steer_headroom`, `wheel_step_max`, `track_k_*`,
  `trim_v_max`/`omega_max`, `replan_*`, `handoff_*`, `arrive_*`.
  (`duty_per_speed_*` is also dead but belongs to
  [[B-one-owner-per-constant-speed-floor-and-duty-per-speed]] — do not remove it
  here out of lockstep with the sim and the sweep tool.)
- `EstimatorConfigPatch.weight_*`/`staleness_ms` — accepted on the wire, applied
  nowhere, **acked 0**. That is silent-off, the pattern this project keeps
  getting burned by: either NACK `ERR_UNIMPLEMENTED` or track it under
  estimator-v2. Do not leave it acking success.
- `Comms::setStatus()` — dead API.
- `src/firm/motion/` and `src/firm/kinematics/` — code-empty
  validator-placater directories. Fold their `DESIGN.md` content out and delete.

Genuinely good hygiene note, worth preserving: **`src/firm` contains zero
TODO/HACK/FIXME markers.** What debt exists is documented debt. Keep it that way.

## `src/motion`

- `WheelChannel::positionAt()` — zero callers.
- `ActiveMove::closingIssued` — written, never read.
- `PoseTracker::blendHeading()` — production-dead but **deliberately kept** for
  estimator-v2 and documented as such. Leave it; it is the model for how to park
  something honestly.
- Root CMake still names the deleted `wheel_pid`; `DESIGN.md` counts off by one;
  `profile.h:51` has a stale WheelTrim comment.

## `src/host`

- `nezha_state.py` (261 lines), `nezha_kinematic.py` (337 lines),
  `kinematics/differential_drive.py` — zero live consumers. Their fate is
  decided by [[A-rebuild-nezha-facade-on-the-v5-binary-surface]] (one pose
  owner); do not delete ahead of it.
- `testkit/` — empty husk.
- `_legacy_tlm_text.py` — all four consumers are themselves dead stacks.
- `cutebot.py` + v1/v2 lazy exports; `testgui/commands.py` (empty schema + dead
  builder).
- `square_tour_velocity.py` — a retirement stub that ImportErrors *before*
  printing its own notice.
- ~350 lines of unreachable v2 code below `rogo goto`/`turnto`'s
  `NotImplementedError` raises.
- `rogo`'s argparse still says "QBot Pro".
- **166 CSV/PNG artifacts committed inside `src/tests/bench/`** — decide a policy
  (keep the ones a note cites, drop the rest, or move to an artifacts dir).

## Bench script catalog

45 scripts: **31 current / 12 stale / 2 retirement stubs**. Two whole stale
families are wholesale deletable:

- the DEV-text-plane family — `dev_exercise`, `pid_hold_speed`,
  `ratio_governor_curve`, `friction_rig_soak`, `velocity_chart`,
  `comms_plane_verify`;
- the rig family — `rig_dev/drive/soak/stress`, `otos_drift` (the rig is
  quarantined by standing stakeholder rule anyway).

**One exception worth reviving rather than deleting:**
`velocity_step_response.py` is one dead `proto.twist()` call away from working,
and it is the step-response characterization tool the sim-fidelity work wants
([[B-sim-plant-is-idealized-and-biases-belief-not-motion]]).

Also noted: the current set duplicates plumbing badly — 8 copies of a PASS/FAIL
recorder, ~20 of connect boilerplate, ≥13 hand-rolled telemetry pumps, and the
`tlmOn`+boot-drain preamble copy-pasted with identical comments.
`src/tests/system/`'s recorder/runner is the right skeleton for one shared
capture library.

## Verification

- Each removed item has a recorded caller-walk.
- Build and full suite green after each batch.
- `EstimatorConfigPatch` no longer acks success for work it does not do.

## Related

- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` — cruft ledgers
  in Parts 1, 3, 4.
