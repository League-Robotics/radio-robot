---
status: done
sprint: '132'
---

# Configuration discipline: one file authors every value, and everything baked is configurable

> **CLOSED, 2026-08-03 — promoted out of the issue queue, not completed.** A
> standing stakeholder rule is not an issue: issues get closed, rules persist.
> The rule now lives at **`.claude/rules/configuration-discipline.md`**, where
> it is auto-loaded into every session instead of waiting in a queue.
> The design work that implements it is [[the-configuration-object]], whose one
> schema enforces the rule structurally rather than by lint — which collapses
> this issue's 27-field work list into that design's step 1.

## Description

Stakeholder rule, 2026-08-03:

> We will bake, but we have to be able to configure everything we bake. If you
> set a configuration, there's one file for it, and that file is the one used
> for baking. To send configuration you edit that file, send that file, and the
> next rebuild bakes the same file.

Two invariants follow, and they run in opposite directions:

1. **Every value the robot uses comes from the file.** No value authored in a
   C++ constant, a host-side literal, or a call-site argument.
2. **Every value in the file reaches the robot, both ways.** Each field has a
   runtime wire arm *and* a bake path, and both read the same file — so a
   pushed config and a rebuilt image cannot disagree.

### The rule binds PRODUCTION BOOT, not development

Refined by the stakeholder the same day, and this is not a loophole — it is the
point:

> It doesn't mean you have to do it for development. You should be able to
> configure individual items without the file — we're going to do a sweep, so
> we should allow that. In general, if you're booting up the robot in
> production, then it better come from a file.

So:

| context | rule |
|---|---|
| production boot | every value comes from the file, no exceptions |
| development / bench tuning | ad-hoc single-value pushes are expected and allowed |

**Read-back is what makes the dev relaxation safe, and is therefore a
prerequisite, not a companion.** An ad-hoc push creates exactly the invisible
divergence-from-file this rule exists to prevent; it is acceptable only because
you can interrogate the robot and see it. Without read-back, permitting ad-hoc
pushes reintroduces the failure — a robot running values nobody can enumerate —
in the one place it is hardest to notice, mid-tuning. See
[[A-no-firmware-to-host-config-readback]].

Practical consequence: a gain sweep pushes literals directly and does **not**
need to round-trip a scratch JSON per trial. The discipline it owes is
different — read back what it pushed, and promote the winner into the file
before anything is baked or shipped.

The payoff is that "what is this robot running?" always has one answer: the
file in production, and read-back at the bench. Today it has neither — see Cause.

This supersedes the "MEASURED, NOT CONFIGURED" decision of 2026-07-31
(`boot_calibration.cpp:84-88`), which deliberately authored
`Drive::kDutyPerSpeed` in C++ and ignored the file's own
`duty_per_speed_left/right`. **That reversal is deliberate and stakeholder-made.**
The concern behind the old decision — circular calibration between
`duty_per_speed` and `wheel_gain` — is real and does not disappear; it is
addressed by *ownership* (one owner per physical quantity, per
[[B-one-owner-per-constant-speed-floor-and-duty-per-speed]]), not by hiding a
value in C++ where nothing can see or change it.

## Cause

**27 fields in the robot JSON have no runtime wire arm** — they are baked-only.
They are enumerated today in `src/scripts/config_sync_allowlist.json`'s
`pydantic-field-no-patch` category (58 entries, 27 of them boot-only), each with
a comment justifying itself as deliberate. Under the rule above that file stops
being an allowlist and becomes **the work list**:

```
calibration.fwd_sign_left/right
control.a_decel, a_max, alpha_decel, alpha_max, j_max, yaw_jerk_max, v_body_max
control.crawl_pulse
control.duty_per_speed_left/right
control.wheel_v_min, wheel_bias_max, wheel_tau_adapt, wheel_a_steady,
        wheel_deficit_threshold
control.wheel_gain_{left,right}_{accel,decel}      (4)
control.wheel_intercept_{left,right}_{accel,decel} (4)
planner.*
control.vel_kp        <- see below: this one must be DELETED, not wired
```

Three distinct violation classes, needing three different fixes:

- **Baked, not configurable** (the 27 above) — need wire arms.
- **Authored outside the file entirely** — no lint catches these today:
  `Drive::kDutyPerSpeed = 0.001182f` (`drive.h:200`) overrides the file's own
  `duty_per_speed_*` at `boot_calibration.cpp:88`; the host taper's
  `_UNMANAGED_FLOOR = 90.0` (`testgui/transport.py:200-201`) disagrees with
  firmware's `wheel_v_min = 99.7`; `duty_sweep.py`'s `KNOWN_DUTY_PER_SPEED` and
  `speed_sweep.py`'s `GAIN = 535.0` are hand-mirrored copies. Catalogued in
  [[B-one-owner-per-constant-speed-floor-and-duty-per-speed]].
- **In the file but dead** — `control.vel_kp/ki/kff/imax/kaw` fed the deleted
  `NezhaMotor MotorVelocityPid`; `toDeviceMotorConfig()`
  (`boot_calibration.h:43-47`) does not copy them and no firmware code reads
  them. A value in the file that nothing consumes violates invariant 2 as much
  as a missing one. **Delete, don't wire.**

Also violating: the entire DRIVETRAIN patch target (`tw`, `rotSlip`, `ekfQxy`,
`ekfQtheta`, `ekfROtosXy`, `ekfROtosTheta`) is authored in the file and
accepted by the host, but `Configurator::apply()` returns `ERR_UNIMPLEMENTED`
for it (`configurator.cpp:82-84`). And `EstimatorConfigPatch`'s
`weight_heading_otos`/`weight_omega_otos`/`staleness_ms` ack **OK** and land
nowhere (`configurator.cpp:58-60`) — worse than a rejection, because it reports
success.

## Proposed fix

### Mechanism: push the config structs, not 27 hand-written fields

Adding 27 individual patch fields is mechanical, large, and leaves the two paths
(push vs bake) as separate hand-maintained mappings that can drift again — which
is the failure this rule exists to prevent.

Better: the generator already produces exactly the structs the firmware boots
from — `Config::DriveBootConfig`, `WheelControllerBootConfig`, etc.
(`src/firm/config/boot_config.h`, emitted by `src/scripts/gen_boot_config.py`).
A wire message carrying those same shapes makes "send the file" one operation,
and makes push/bake provably identical because both derive from one generated
definition. Envelope headroom exists: `kCommandEnvelopeMaxEncodedSize = 55`
against a 240 B budget (`wire.h:69-73`); chunk across envelopes if a struct
exceeds it.

Open design point: whether this replaces the existing per-key `pid.*`/`ml`/`mr`
patch surface or coexists with it for interactive single-value tuning. A
reasonable split is bulk-struct push for "apply this file" and the existing
typed patches for "nudge one value" — provided the nudge path also writes back
to the file (see Workflow).

### Workflow

**Production / anything baked or shipped:**

1. Edit the robot JSON (the one file).
2. Push **from that file** — `src/host/robot_radio/calibration/push.py`'s
   `calibration_kwargs()` is the existing file→wire path to build on.
3. Rebuild bakes the same file.

**Development / bench tuning:** push individual values directly. No scratch-file
round trip required. What a sweep owes instead:

- read back what it pushed, so a silently-ignored patch is caught (the
  `EstimatorConfigPatch` failure mode: acks OK, lands nowhere);
- record the pushed values alongside its results, so a captured dataset is
  self-describing rather than depending on session memory;
- promote the winning values into the robot JSON before anything is baked,
  shipped, or used as a baseline for a later measurement.

### Enforcement: flip the lint

`src/scripts/check_config_sync.py` already compares JSON fields against patch
fields and force-fails on `unmapped-patch-field`
(`FORCED_FAIL_CATEGORIES`, `:249`). Extend it so the **other** direction —
a config field with no wire arm — is also a forced failure, and empty
`config_sync_allowlist.json`'s `pydantic-field-no-patch` category as fields are
wired. When the allowlist is empty the discipline is mechanically enforced
forever, and this issue cannot regress.

Add a check for the third class too (values authored outside the file): at
minimum, assert no firmware constant shadows a config key — `kDutyPerSpeed` is
the known instance.

### Sequencing

1. **Delete dead config** (`control.vel_*`) — smallest, unblocks nothing but
   removes noise from the work list.
2. **Resolve the outside-the-file authors** — `kDutyPerSpeed` into the file
   (this is the "MEASURED, NOT CONFIGURED" reversal), host `_UNMANAGED_FLOOR`
   reading firmware's `wheel_v_min`.
3. **Wire the Stage A eight** — the first real consumer, and the one with a
   waiting bench need; see
   [[per-wheel-drive-calibration-as-runtime-configuration]].
4. **Wire the rest** in whatever grouping the mechanism above makes natural.
5. **Flip the lint** and empty the allowlist.

## Verification

- `uv run python src/scripts/check_config_sync.py` — the primary gate. Success
  is an **empty** `pydantic-field-no-patch` allowlist with the check forced-failing.
- For each newly wired field: push a value, read it back or observe its effect,
  power-cycle, confirm the robot returns to the baked value. A field that acks
  OK and changes nothing (the `EstimatorConfigPatch` failure mode) must be
  caught here — an ack is not evidence.
- Bake/push parity: build an image from a file, push the same file to a robot
  running a *different* baked config, and confirm identical behaviour. This is
  the invariant the whole rule exists to guarantee, so it deserves a direct test
  rather than being inferred.
- Read-back closes the loop and is currently missing
  ([[A-no-firmware-to-host-config-readback]]) — without it, "confirm the robot
  is running the file" is unverifiable by construction. Strongly consider doing
  that issue first or alongside.
- **Build for ARM** before declaring done — several config paths have
  HOST_BUILD-invisible constraints (e.g. the persisted-tuning chunk budget).

## Related

- [[per-wheel-drive-calibration-as-runtime-configuration]] — the first consumer;
  its 8 Stage A fields are 8 of the 27.
- [[A-no-firmware-to-host-config-readback]] — the missing half of this
  discipline. Without read-back you can enforce that the file is the *author*
  but not verify the robot is *running* it.
- [[B-one-owner-per-constant-speed-floor-and-duty-per-speed]] — the catalogue of
  values authored outside the file; the "one owner per quantity" rule is what
  replaces "MEASURED, NOT CONFIGURED" as the guard against circular calibration.
- [[B-observability-contract-is-inert-as-shipped]] — config that acks OK and
  lands nowhere is the same disease.
- `boot_calibration.cpp:84-88` — the superseded 2026-07-31 decision.
