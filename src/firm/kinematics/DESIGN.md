---
root: ../DESIGN.md
---

# Kinematics (`src/firm/kinematics`, namespace `BodyKinematics`)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-16 · **Status:** stable

---

## 1. Purpose

`BodyKinematics` is the differential-drive math: the (v, ω) ↔ (vL, vR)
twist/wheel-speed maps and curvature-preserving saturation. It is split out
of `devices/` and `app/` because it is pure math with no bus, no timing, and
no state — the one place in the firmware that can be trusted to compile,
link, and be exercised identically on ARM and in a host unit test with zero
scaffolding. Nothing else owns this math; `app/Drive` and `app/Odometry`
call into it rather than duplicating the equations.

## 2. Orientation

Three operations, each with a scalar form (`float` in/out params) and an
array/message form (`msg::BodyTwist3`, `wheels[2]`) used by the app layer:

- **`inverse`** — body twist → wheel speeds.
- **`forward`** — wheel speeds → body twist (used by `Odometry` to integrate
  encoder deltas back into a twist).
- **`saturate`** — clamp wheel speeds to a ceiling without breaking the
  commanded curvature.

The math itself — derivation, arc geometry, why saturation preserves ratio
rather than clamping each wheel independently — is **not** repeated here.
It lives in [docs/kinematics-model.md](../../../docs/kinematics-model.md)
§1.3 (inverse/forward) and §1.7 (saturation); the header/`.cpp` doc comments
carry the equations a caller needs at the call site. Read that document
first if you need to understand *why* the equations are what they are.

## 3. Constraints and Invariants

- **Stateless and pure — no I2C, no globals, no heap.** This is the whole
  reason the subsystem is split out: it must compile and behave identically
  under `HOST_BUILD` and on ARM with no fakes or seams. Adding any state,
  side effect, or bus access here defeats that and would force every caller
  (and every host test) to start injecting fakes for what should be a bare
  function call.
- **CCW-positive `omega`; signed `vL`/`vR`.** Yaw rate is positive
  counterclockwise (right-hand convention about +z, matching
  `docs/kinematics-model.md`'s body frame: +x forward, +y left). Wheel
  speeds are signed mm/s, not signed PWM or magnitude+direction — reversing
  a wheel is a negative speed, not a separate flag.
- **`saturate` preserves curvature, never clamps a wheel independently.**
  When the faster wheel would exceed `vWheelMax - steerHeadroom`, both wheel
  speeds scale by the same factor. Clamping only the offending wheel breaks
  the commanded wheel-speed ratio and sends the robot off its arc — see
  kinematics-model.md §1.7 for the failure mode this avoids.
- **`steerHeadroom` is a deliberate non-goal to remove.** It exists so a
  straight-line command at top speed still leaves the outer control loop
  some authority to steer; do not "simplify" saturation to use the raw
  `vWheelMax` as the ceiling.

## 4. Design

Each scalar function is a direct transcription of the corresponding equation
in kinematics-model.md — there is no additional structure to describe. The
array-form overloads (`inverse`/`forward`/`saturate` taking `msg::BodyTwist3`
and `wheels[2]`) exist as the API shape `app/Drive` and `app/Odometry`
actually call: they wrap the scalar forms, fixing `v_y` to 0 (a differential
drivetrain cannot strafe) rather than duplicating logic. There is no
independent array-form implementation to keep in sync with the scalar one.

## 5. Interfaces

### Exposes
- **`inverse(v, omega, b, vL_out, vR_out)`** / **`inverse(msg::BodyTwist3, b,
  wheels[2])`** — body twist to wheel speeds. Pure, no failure mode; `b` is
  the caller's calibrated track width (see root DESIGN.md /
  `config/DESIGN.md` for where that calibration comes from).
- **`forward(vL, vR, b, v_out, omega_out)`** / array form — wheel speeds to
  body twist. Used by `app/Odometry` each cycle to fold encoder deltas into
  the twist it integrates.
- **`saturate(vL, vR, vWheelMax, steerHeadroom, vL_out, vR_out)`** / array
  form — curvature-preserving ceiling. Pass-through when already under the
  effective ceiling.

### Consumes
- **`msg::BodyTwist3`** (from `messages/`) — the wire-plane twist type used
  by the array-form overloads; see [messages/DESIGN.md](../messages/DESIGN.md).
- **Calibrated track width `b` and saturation limits** (`vWheelMax`,
  `steerHeadroom`) are supplied by the caller (`app/Drive`), sourced from
  `config/` — see [config/DESIGN.md](../config/DESIGN.md).

## 6. Open Questions / Known Limitations

- The array-form overloads were originally added as a differential-drive
  adapter for a shared `IKinematics` contract shared with a mecanum
  implementation; that shared contract and its only consumer were deleted in
  the sprint 102 single-loop rebuild. The array-form API is kept because
  `app/` calls it, not because the shared-contract alias still exists — if a
  second drivetrain kind is ever added, revisit whether a shared interface
  is worth reintroducing.
