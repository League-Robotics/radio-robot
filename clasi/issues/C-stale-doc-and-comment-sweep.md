---
status: pending
priority: low
---

# Stale docs and comments: DESIGN.md rot, phantom methods, and the "47 ms" fossils

## Description

Enumerated by the 2026-08-02 post-130 review and the sprint-130 midpoint review
(finding #11). Low priority individually; collected because a reader who
believes any one of these loses more time than the fix costs.

## The specific offenders

**Timing fossils** — the previous generation of the same confusion that
[[A-nominal-50ms-vs-delivered-54ms]] is about. Verified present today:

- `src/firm/main.cpp:80` — "~47 ms budget"
- `src/firm/app/boot_wiring.h:36` — "why the ROBOT JSON bakes a measured 47ms"
- `src/firm/app/robot_loop.h:52` — a "47 ms" reference in explanatory context

Fix these **with** the period issue, not before it — they should end up stating
whatever that issue decides is true, once.

**DESIGN.md rot:**

- `src/firm/DESIGN.md` — pre-122 shape; **three different `kCycle` values across
  the DESIGN docs** against an actual 50.
- `src/firm/app/DESIGN.md` — lists deleted classes as live, and actively
  *defends* the pacing design that F7 indicts.
- `src/firm/types/DESIGN.md`, `docs/design/design.md` §2 — rows listing deleted
  classes.
- `src/sim/DESIGN.md` — body is 118-era (40 ms, retired TWIST surface) sitting
  under a fresh 130-002 §2.

**Comments that describe code that is not there:**

- `telemetry.h:126-128` — claims bit 8 has "no per-transaction NAK aggregate";
  the aggregate exists and is merely unreachable (see
  [[B-observability-contract-is-inert-as-shipped]]).
- `telemetry.cpp:19` — ack-ring size comment.
- `serial_port.h` — framing comments.
- `line_sensor.h` — phantom methods.
- `boot_config.h` — phantom `static_assert`s.
- `fake_otos.cpp`, `radio.h` — dead references.
- `robot_state.h:230-242` — describes the write order **backwards**.
- `motor.h:103-109` — promises a `lastFreshUs_` that does not exist (this one
  is load-bearing; it belongs to
  [[A-commanded-zero-leaks-through-stage-b]]).
- `sim_harness.h:117-121` — claims "same values as hardware", false (belongs to
  [[B-one-owner-per-constant-speed-floor-and-duty-per-speed]]).
- `ratio_lock_test.cpp:169` — cites a deleted test.
- `test_sim_wire_loopback.py:322` — a tolerance justified by the deleted trim.
- `protocol.py:28-31` — asserts "telemetry is always-on (no arming step)",
  contradicted by the `tlmOn`/`tlmOff`/`tlmNow` section in the same file.

**Robot JSON self-contradictions** (`data/robots/tovez.json`):

`_drive_calibration_note` says the derived figures are "report-only, **NOT
applied** to any config key" while `wheel_v_min: 99.7` and
`wheel_bias_max: 23.8` are set live at `:136-137`, and
`_wheel_controller_note` says Stage C "ships LIVE" with exactly those numbers.
One of the two notes is wrong. This is sprint-130 midpoint finding #2 (MAJOR
there, because the floor actively rounds sub-100 mm/s commands up — that half
is [[A-speed-floor-snaps-the-planner-differential]]); what remains here is the
provenance contradiction.

Also flagged by the review's Part 8 as a five-minute file: the knowledge doc
states rotation constants were "removed this sprint", but `tovez_nocal.json`
still carries 1.006 / +12.1 — see
[[B-rotation-calibration-vs-live-heading-hold-gain]].

## What to do

Sweep them. Where a comment documents a defect instead of fixing it (the
`cycleCount_` header comment is the canonical example — see
[[C-sensors-line-sensor-dead-and-perception-pacing-owner]]), the sweep's job is
to file or fix, not to reword.

## Verification

- No DESIGN.md names a deleted class or a wrong `kCycle`.
- The three "47 ms" sites say whatever the period issue settled.
- `tovez.json`'s two notes agree with each other and with the live keys.

## Related

- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` — stale-doc sets
  in Parts 1, 3, 4, 5.
- `docs/code_review/2026-08-02-sprint-130-midpoint.md` findings 2, 6, 11.
