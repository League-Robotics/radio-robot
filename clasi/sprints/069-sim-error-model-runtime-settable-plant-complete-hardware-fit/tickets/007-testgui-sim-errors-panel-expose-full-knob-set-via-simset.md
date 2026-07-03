---
id: '007'
title: 'TestGUI Sim Errors panel: expose full knob set via SIMSET'
status: open
use-cases:
- SUC-007
depends-on:
- '004'
- '005'
github-issue: ''
issue:
- expose-sim-error-model-knobs-in-testgui.md
- sim-error-model-runtime-settable-hardware-fit.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI Sim Errors panel: expose full knob set via SIMSET

## Description

This ticket is issue `expose-sim-error-model-knobs-in-testgui.md`'s entire
ask, folded into the `SIMSET` wire surface (tickets 002-005) rather than
plumbed through the legacy per-field ctypes path — per the sprint's explicit
directive.

Today's panel (`host/robot_radio/testgui/__main__.py:655-729`) exposes
exactly 4 knobs (`encoder_noise_mm`, `slip_turn_extra`, `otos_linear_noise`,
`otos_yaw_noise`), backed by `sim_prefs.DEFAULT_PROFILE`
(`host/robot_radio/testgui/sim_prefs.py:48-53`) and applied via
`transport.py`'s `SimTransport._apply_profile_to_sim()`
(`transport.py:1063-1103`), which calls four separate Python methods
(`sim.set_field_profile(...)`, `sim.set_otos_linear_noise(...)`,
`sim.set_otos_yaw_noise(...)`, `sim.set_encoder_noise(...)`) on the
connection object — the legacy per-field ctypes path.

This ticket:
1. Extends `sim_prefs.DEFAULT_PROFILE` with every newly-surfaced knob from
   tickets 002-004: `enc_scale_err_l`/`enc_scale_err_r`,
   `otos_lin_scale_err`, `otos_ang_scale_err`, `otos_lin_drift_mms`,
   `otos_yaw_drift_degs`, and — per `architecture-update.md` Open Question 6
   — optionally `motor_offset_l`/`motor_offset_r`, `trackwidth_mm`,
   `body_rot_scrub`, `body_lin_scrub`.
2. Adds a module-level key-name map (profile key → `SIMSET` wire-key name)
   so `transport.py` can build one command from the whole profile dict
   without hardcoding the mapping inline.
3. Rewrites `_apply_profile_to_sim()` to send ONE `SIMSET k1=v1 k2=v2 …`
   command through `sim.command(...)` (the existing raw wire-command method,
   `transport.py:874`) instead of four separate per-field Python calls.
4. Extends the GUI panel with grouped spinbox rows for the new fields.

**CORRECTNESS-CRITICAL default-value note** (not spelled out explicitly in
`architecture-update.md`'s "defaults of 0.0" summary, which is only true for
the ADDITIVE error knobs): the two new scrub knobs
(`body_rot_scrub`/`body_lin_scrub`) and the motor-offset knobs
(`motor_offset_l`/`motor_offset_r`) are MULTIPLICATIVE and their no-op value
is `1.0`, NOT `0.0` — confirm against ticket 002's `PhysicsWorld` field
defaults (`_bodyRotationalScrub`/`_bodyLinearScrub` default `1.0f`;
`_offsetFactorL`/`_offsetFactorR` default `1.0f`). `trackwidth_mm` has NO
safe universal default at all — `PhysicsWorld`'s sub-step B divides by
`_trackwidthMm` (`PhysicsWorld.cpp:96`), so sending `SIMSET trackwidthMm=0`
(if `0.0` were naively used as "no-op") would be a plant-breaking
divide-by-zero. If `trackwidth_mm` is included in the panel (Open Question 6
leaves this to the implementer), it MUST default to a genuine no-op — either
omit the key from the built `SIMSET` string entirely when the field holds
its sentinel/unset value, or default the spinbox to
`PhysicsWorld::kDefaultTrackwidthMm` (150.0) and document that this
overrides the plant's trackwidth unconditionally once Apply is pressed
(no silent "0 means don't touch" special-casing inside `SimCommands`, which
would violate ticket 003's atomic apply-what-was-sent contract).

## Acceptance Criteria

- [ ] `sim_prefs.py`: `DEFAULT_PROFILE` extended with the new keys listed
      above, with correct no-op defaults (`0.0` for every additive error
      term; `1.0` for `body_rot_scrub`, `body_lin_scrub`,
      `motor_offset_l`, `motor_offset_r`; a documented, non-zero, genuinely
      neutral choice for `trackwidth_mm` if included — see note above).
- [ ] `sim_prefs.py`: new module-level `PROFILE_TO_SIMSET_KEY: dict[str, str]`
      (or equivalently named) mapping every profile key to its `SIMSET`
      wire-key name (e.g. `"enc_scale_err_l": "encScaleErrL"`).
      `load_sim_error_profile()`/`save_sim_error_profile()` extended to
      round-trip all new keys (mirroring the existing `for key in
      DEFAULT_PROFILE` loop shape at `sim_prefs.py:72,94`).
- [ ] `transport.py`: `_apply_profile_to_sim()` rewritten to build one
      `SIMSET k1=v1 k2=v2 …` string from the full profile (via the new
      key-name map) and send it with `sim.command(...)` — replacing the four
      separate `sim.set_field_profile/set_otos_linear_noise/
      set_otos_yaw_noise/set_encoder_noise` calls. The historical
      `slip_turn_extra` knob (which drives `sim_set_motor_slip` via
      `set_field_profile`, NOT a `SIMSET` key — it is the pre-existing,
      untouched `_rotationalSlip` test-infra channel per Design Rationale
      Decision 4) is retained as-is, applied SEPARATELY from the new
      `SIMSET` string (do not attempt to fold it into `SIMSET`; it has no
      `SIMSET` key).
- [ ] `turn_scrub_factor` (`transport.py:734-755`) is UNCHANGED — it reads
      `slip_turn_extra` from the same profile dict, unrelated to any new key.
- [ ] `__main__.py`: Sim Errors panel gains grouped spinbox rows (suggested
      grouping: "Encoder Report Error", "Body-Truth Scrub", "Geometry &
      Actuation", "OTOS Error"), reusing the existing `_make_sim_err_spin()`
      helper (`__main__.py:663-683`) for each new field, with ranges
      appropriate to each knob's semantics (e.g. scrub factors `[0.0, 2.0]`
      or narrower per Decision 2's clamp discussion, not `[0.0, 50.0]` like
      `encoder_noise_mm`).
- [ ] `_on_sim_errors_apply()` (`__main__.py:706-727`) extended to read all
      new spinbox values into the `profile` dict passed to
      `sim_prefs.save_sim_error_profile()`/`transport.apply_error_profile()`
      — same call shape, larger dict.
- [ ] Existing 4 knobs' behavior is EXACTLY preserved (byte-identical
      `SIMSET`/ctypes calls for those four, or an equivalent-effect `SIMSET`
      translation for the three that ARE `SIMSET` keys — `otos_linear_noise`
      → `otosLinNoise`, `otos_yaw_noise` → `otosYawNoise`,
      `encoder_noise_mm` → `encNoiseL`+`encNoiseR` both set to the same
      value, matching today's `sim.set_encoder_noise(0, ...)` +
      `sim.set_encoder_noise(1, ...)` pair).
  - [ ] Threading: `_apply_profile_to_sim()`'s new `SIMSET`-string body
      continues to run via the existing `_action`/`self._cmd_queue.put(...)`
      dispatch (`transport.py:1135-1138`), which already marshals the
      mutation onto the tick-thread that exclusively owns the `Sim` object —
      do not introduce a new Qt signal/slot path for this. If any NEW
      cross-thread signal connection is introduced elsewhere in this ticket
      (not expected, but verify), it MUST go through a main-thread `QObject`
      bridge with a kept reference, never a bare-function
      `QueuedConnection` (project knowledge: sprint 063/007-009's
      PySide QueuedConnection bare-function gotcha — a `QueuedConnection` to
      a bare function silently runs on the wrong thread or drops frames).
- [ ] Defaults reproduce today's no-op-until-opted-in behavior exactly: with
      every new field at its default, the sim's observable behavior after
      Apply is unchanged from before this ticket.
- [ ] The profile persists to `data/testgui/sim_error_profile.json` (existing
      mechanism, extended with the new keys) — verify by inspecting the
      written file after Apply.
- [ ] Full default suite green: `uv run python -m pytest`.

## Testing

- **Existing tests to run**: `tests/testgui/test_sim_prefs.py`,
  `tests/testgui/test_transport.py`; full default suite.
- **New tests to write**: extend both test files above for the new profile
  keys, the key-name map, and the `_apply_profile_to_sim()` →
  single-`SIMSET`-string rewrite (assert the exact `SIMSET` string sent for
  a given profile dict, and that `slip_turn_extra` is still applied via the
  separate legacy path).
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Extend the existing profile/persistence/panel pattern rather
than redesigning it — `sim_prefs.py`'s load/save shape, `__main__.py`'s
`_make_sim_err_spin()` helper, and `transport.py`'s `_cmd_queue`-dispatched
apply pattern are all already correct and thread-safe; this ticket adds
keys and rewires the wire-transport half of `_apply_profile_to_sim()` only.

**Files to modify**:
- `host/robot_radio/testgui/sim_prefs.py` — `DEFAULT_PROFILE` extension,
  new key-name map, load/save extension.
- `host/robot_radio/testgui/transport.py` — `_apply_profile_to_sim()`
  rewrite (single `SIMSET` string via `sim.command(...)`, `slip_turn_extra`
  kept on its separate legacy path).
- `host/robot_radio/testgui/__main__.py` — new grouped spinbox rows,
  `_on_sim_errors_apply()` extension.

**Testing plan**:
- Extend `tests/testgui/test_sim_prefs.py` for the new keys and map.
- Extend `tests/testgui/test_transport.py` to assert the exact `SIMSET`
  string built from a representative profile dict, and that
  `slip_turn_extra` still routes through `set_field_profile`.
- Manual/scripted check: Apply with all defaults produces no `SIMSET`
  divergence from a freshly-constructed sim's baseline telemetry.
- Full `uv run python -m pytest`.

**Documentation updates**: none beyond in-code docstrings — no wire-protocol
change (this ticket is a client of `SIMSET`, not a producer).
