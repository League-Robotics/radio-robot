"""test_069_knob_telemetry_sweep.py — ticket 069-006: comprehensive per-knob
telemetry sweep for the `SIMSET`/`SIMGET` registry (`kSimRegistry[]` in
`source/commands/SimCommands.cpp`).

Sprint success criteria (069/sprint.md) requires "a test [that] sweeps each
knob and observes the corresponding telemetry change." This is that test:
REGISTRY-DRIVEN (it enumerates the ACTUAL key set via a bare `SIMGET`, not a
hardcoded Python list), so a future ticket that appends a new
`kSimRegistry[]` row is caught here immediately -- see
``test_no_unmapped_simset_keys`` below, which FAILS LOUDLY the moment
``SIMGET`` returns a key absent from ``KNOB_MAP``. Once that failure points a
future author here, they add one entry to ``KNOB_MAP`` (the one thing that
cannot be auto-discovered: which TLM field a given knob's perturbation should
visibly move) and the parametrized sweep (``test_knob_moves_expected_signal``,
which iterates ``sorted(KNOB_MAP)``) picks the new key up automatically.

EKF ``SET``/``GET`` keys are OUT OF SCOPE for this file by design. The seven
EKF noise fields (ticket 069-001) are a fundamentally different mechanism --
fusion WEIGHTING, not plant/observation ERROR -- and live on the `SET`/`GET`
surface, not `SIMSET`/`SIMGET`, so a bare `SIMGET` never returns them; they
are already covered by ``tests/simulation/unit/test_069_001_ekf_noise_registry.py``
(round-trip + "does not reset fused pose" + "changes correction strength").
Re-deriving that fusion-weighting test shape here (see ticket 069-006's
Implementation Plan) would just duplicate it, not extend coverage.

Design: four knob GROUPS, each with a different observable signature and
therefore a different check function. All four groups compare a BASELINE run
(fresh ``Sim()``, knob left at its no-op default) against a PERTURBED run
(fresh ``Sim()``, ONE knob `SIMSET` to a distinctly non-default value) driving
the IDENTICAL maneuver -- "fresh Sim() per knob" (never a shared/reset sim)
satisfies acceptance criterion 5 ("resets the sim... before the next key")
trivially: there is no persistent sim for state to accumulate on.

  1. GROUND_TRUTH_SCRUB (bodyRotScrub, bodyLinScrub, trackwidthMm) --
     perturbs ``PhysicsWorld`` ground truth through a channel the encoder
     dead-reckoning path never reads (independent scrub multipliers /
     PhysicsWorld's own trackwidth cache, distinct from
     ``RobotConfig.trackwidthMm``, which Odometry always uses). Because OTOS
     (fusion enabled) samples ground truth directly, ``otos=``/``pose=``
     track whatever the (possibly-scrubbed) TRUE pose IS almost exactly --
     estimate-vs-truth WITHIN one run shows ~zero divergence by
     construction. The divergence that reveals the injected error is
     therefore baseline-SCENARIO-vs-perturbed-SCENARIO, not estimate-vs-
     truth. ``encpose=``/``enc=`` (pure encoder arc-integration) are
     provably untouched -- confirmed bit-exact-unchanged empirically.

  2. PHYSICAL_ASYMMETRY (motorOffsetL, motorOffsetR) -- a genuine per-wheel
     motor-strength difference. Designated signal: per-wheel ``enc=``
     divergence after an equal-command straight drive. Unlike the other
     three groups, this knob has NO isolable "stays unchanged" signal --
     a real motor asymmetry legitimately propagates into every downstream
     estimate (``enc=``, ``encpose=``, ``otos=``, ``pose=`` all move
     together, empirically confirmed) -- so, per the ticket's own "if a
     knob has no independently observable isolation signature, document
     that honestly" guidance, no false isolation claim is asserted here.

  3. ENCODER_REPORT_ERROR (encScaleErrL/R, encSlipL/R, encNoiseL/R) --
     reuses ``test_069_004_encoder_otos_knobs.py``'s exact isolation
     pattern: these bias the REPORTED per-wheel encoder channel (which also
     feeds the closed-loop velocity PID, so the TRUE path itself curves
     slightly -- comparing estimate-vs-baseline would conflate "the
     scenario changed" with "the estimator is wrong"). The correct
     within-run comparison is estimate-vs-TRUE (``sim.get_true_pose()``):
     ``otos=``/``pose=`` (OTOS is a disjoint input, fusion pulls fused pose
     back toward true) stay close to true; ``encpose=`` (encoder-only,
     never EKF-corrected) visibly diverges from true. ``enc=`` itself
     (the raw wire reading) is additionally checked baseline-vs-perturbed,
     since it is a direct reported value, not an estimate.

  4. OTOS_ERROR (otosLinScaleErr, otosAngScaleErr, otosLinNoise,
     otosYawNoise, otosLinDriftMmS, otosYawDriftDegS) -- SimOdometer's
     observation-error model is fully disjoint from PhysicsWorld's encoder
     channel (confirmed bit-exact-unchanged empirically), so this group
     reuses GROUND_TRUTH_SCRUB's exact check function: baseline-vs-
     perturbed on ``otos=``/``pose=`` (designated), bit-exact-unchanged on
     ``enc=``/``encpose=`` (isolated).

All non-default values and thresholds below were picked empirically (a
throwaway probe script, not checked in) against this sim build, with
generous margin below the smallest observed divergence and above the
largest observed "unchanged" jitter (which was consistently exactly 0)
-- the same "measure, don't guess" discipline
``test_069_004_encoder_otos_knobs.py``'s docstrings already document.
"""
from __future__ import annotations

import math

import pytest

from firmware import Sim
from robot_radio.robot.protocol import parse_tlm, TLMFrame


# ---------------------------------------------------------------------------
# Sim / TLM helpers
# ---------------------------------------------------------------------------

def _fresh_sim() -> Sim:
    """A brand-new Sim, watchdog extended (mirrors the ``sim`` fixture).

    Deliberately NOT the ``sim`` pytest fixture -- each check function needs
    TWO independent fresh sims (baseline + perturbed) per knob, sometimes
    more than once per test, so this is called directly rather than via
    fixture injection.
    """
    s = Sim()
    s.send_command("SET sTimeout=60000")
    return s


def _otos_sim() -> Sim:
    """A fresh Sim with the OTOS sim-model integrator enabled and EKF fusion
    turned on -- required for ``otos=``/``pose=`` to be meaningfully
    populated/corrected in TLM (see ``Drive::tickUpdate``'s
    ``otos.is_initialized()`` + freshness gate)."""
    s = _fresh_sim()
    s.enable_otos_model()
    s.set_otos_fusion(True)
    return s


def _simset_ok(sim: Sim, key: str, value: float) -> None:
    reply = sim.send_command(f"SIMSET {key}={value}")
    assert reply.upper().startswith("OK"), f"SIMSET {key}={value} -> {reply!r}"


def _snap(sim: Sim) -> TLMFrame:
    reply = sim.send_command("SNAP")
    frame = parse_tlm(reply)
    assert frame is not None, f"SNAP did not parse as a TLM frame: {reply!r}"
    return frame


def _drive_straight(sim: Sim, dur_ms: int = 2000) -> None:
    reply = sim.send_command(f"T 200 200 {dur_ms}")
    assert reply.upper().startswith("OK"), f"T 200 200 {dur_ms} -> {reply!r}"
    sim.tick_for(dur_ms + 500)


def _drive_rt(sim: Sim, cdeg: int = 9000) -> None:
    reply = sim.send_command(f"RT {cdeg}")
    assert reply.upper().startswith("OK"), f"RT {cdeg} -> {reply!r}"
    sim.tick_for(8000)


def _dist_xy(a: tuple, b: tuple) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _discover_simset_keys(sim: Sim) -> list[str]:
    """Parse every key name out of a bare ``SIMGET`` dump.

    Bare ``SIMGET`` may chunk across multiple ``SIMCFG`` lines once the
    registry grows past ``kSimCfgChunkMax`` (SimCommands.cpp) -- collect
    every line, not just the first.
    """
    reply = sim.send_command("SIMGET")
    keys: list[str] = []
    for line in reply.splitlines():
        if not line.startswith("SIMCFG"):
            continue
        for tok in line.split()[1:]:
            if "=" in tok and not tok.startswith("#"):
                keys.append(tok.split("=", 1)[0])
    assert keys, f"bare SIMGET returned no keys at all: {reply!r}"
    return keys


# ---------------------------------------------------------------------------
# Group 1 / Group 4 shared check: baseline-vs-perturbed on otos=/pose=,
# bit-exact-unchanged on enc=/encpose=.
# ---------------------------------------------------------------------------

def _check_scrub_or_otos_error(key: str, cfg: dict) -> None:
    value = cfg["value"]
    maneuver = cfg["maneuver"]
    axis = cfg["axis"]          # 0=x(mm), 1=y(mm), 2=heading(cdeg)
    min_delta = cfg["min_delta"]

    baseline = _otos_sim()
    maneuver(baseline)
    base = _snap(baseline)

    perturbed = _otos_sim()
    _simset_ok(perturbed, key, value)
    maneuver(perturbed)
    pert = _snap(perturbed)

    assert base.otos is not None and pert.otos is not None, (
        f"{key}: otos= missing from TLM (OTOS model+fusion enabled on both "
        f"runs) -- base={base} pert={pert}"
    )
    otos_delta = abs(pert.otos[axis] - base.otos[axis])
    assert otos_delta >= min_delta, (
        f"{key}={value} should move otos=[{axis}] by >= {min_delta} "
        f"relative to the default-knob baseline; base otos={base.otos} "
        f"pert otos={pert.otos} (delta={otos_delta})"
    )

    assert base.pose is not None and pert.pose is not None
    pose_delta = abs(pert.pose[axis] - base.pose[axis])
    assert pose_delta >= min_delta * 0.5, (
        f"{key}={value}: fused pose= (OTOS fusion enabled) should also "
        f"move -- otos= tracks the (scrubbed/errored) ground truth and "
        f"fusion pulls pose= toward it; base pose={base.pose} "
        f"pert pose={pert.pose}"
    )

    # Isolation: the encoder-only dead-reckoning channel is untouched --
    # empirically bit-exact-unchanged for every knob in these two groups.
    assert base.encpose is not None and pert.encpose is not None
    tols = (3, 3, 15)  # (x_mm, y_mm, h_cdeg)
    for i, tol in enumerate(tols):
        assert abs(base.encpose[i] - pert.encpose[i]) <= tol, (
            f"{key}={value} should NOT perturb encpose= (encoder dead "
            f"reckoning never reads plant-truth scrub / OTOS observation "
            f"error); base={base.encpose} pert={pert.encpose}"
        )
    assert base.enc is not None and pert.enc is not None
    for i in range(2):
        assert abs(base.enc[i] - pert.enc[i]) <= 3, (
            f"{key}={value} should NOT perturb enc=; base={base.enc} "
            f"pert={pert.enc}"
        )


# ---------------------------------------------------------------------------
# Group 2 check: per-wheel enc= divergence, no isolable "unchanged" signal.
# ---------------------------------------------------------------------------

def _check_physical_asymmetry(key: str, cfg: dict) -> None:
    value = cfg["value"]
    side = cfg["side"]  # 0=left, 1=right -- which enc=[side] should move

    baseline = _otos_sim()
    _drive_straight(baseline)
    base = _snap(baseline)

    perturbed = _otos_sim()
    _simset_ok(perturbed, key, value)
    _drive_straight(perturbed)
    pert = _snap(perturbed)

    assert base.enc is not None and pert.enc is not None
    delta = abs(pert.enc[side] - base.enc[side])
    assert delta >= 20, (
        f"{key}={value} should move enc=[{side}] by >= 20 mm relative to "
        f"the default-knob baseline after an equal-command straight "
        f"drive; base enc={base.enc} pert enc={pert.enc}"
    )
    # No isolation assertion: a genuine per-wheel motor-offset difference
    # is a REAL physical asymmetry (the same _offsetFactor feeds true
    # encoder accumulation, reported encoder accumulation, AND true-pose
    # integration -- source/hal/sim/PhysicsWorld.cpp's sub-steps A/A'/B all
    # read the same velL/velR). It legitimately propagates into every
    # downstream signal (enc=, encpose=, otos=, pose= all move together,
    # confirmed empirically) -- there is no "should stay unchanged" channel
    # to assert against, unlike the disjoint-model knobs in the other three
    # groups. Documented per the ticket's own guidance rather than asserting
    # a false isolation claim.


# ---------------------------------------------------------------------------
# Group 3 check: enc= baseline-vs-perturbed (designated) + estimate-vs-TRUE
# isolation (otos=/pose= close to true, encpose= diverges from true) --
# mirrors test_069_004_encoder_otos_knobs.py's pattern exactly.
# ---------------------------------------------------------------------------

def _check_encoder_report_error(key: str, cfg: dict) -> None:
    value = cfg["value"]
    side = cfg["side"]

    baseline = _otos_sim()
    _drive_straight(baseline)
    base = _snap(baseline)

    perturbed = _otos_sim()
    _simset_ok(perturbed, key, value)
    _drive_straight(perturbed)
    pert = _snap(perturbed)

    assert base.enc is not None and pert.enc is not None
    delta = abs(pert.enc[side] - base.enc[side])
    assert delta >= 3, (
        f"{key}={value} should move enc=[{side}] relative to the default-"
        f"knob baseline; base enc={base.enc} pert enc={pert.enc}"
    )

    assert pert.otos is not None and pert.pose is not None and pert.encpose is not None
    true_pose = perturbed.get_true_pose()
    otos_d = _dist_xy(pert.otos[:2], true_pose[:2])
    pose_d = _dist_xy(pert.pose[:2], true_pose[:2])
    encpose_d = _dist_xy(pert.encpose[:2], true_pose[:2])

    assert otos_d < 5.0, (
        f"{key}={value}: otos= should track the plant's TRUE pose closely "
        f"(the encoder-report-error channel is a disjoint input -- "
        f"SimOdometer samples ground truth directly); got {otos_d:.2f} mm "
        f"off true (otos={pert.otos} true={true_pose})"
    )
    assert pose_d < 5.0, (
        f"{key}={value}: EKF-fused pose= (OTOS fusion enabled) should be "
        f"corrected back toward true/otos every tick; got {pose_d:.2f} mm "
        f"off true (pose={pert.pose} true={true_pose})"
    )
    assert encpose_d > 8.0, (
        f"{key}={value}: encoder-only dead reckoning (encpose=, never "
        f"touched by the EKF) should visibly diverge from true when this "
        f"knob is injected; got only {encpose_d:.2f} mm off true "
        f"(encpose={pert.encpose} true={true_pose})"
    )


# ---------------------------------------------------------------------------
# EKF_KEYS_OUT_OF_SCOPE -- documentary only, NOT iterated by the sweep.
#
# The seven ticket-069-001 EKF noise fields are `SET`/`GET` (RobotConfig)
# keys, not `SIMSET`/`SIMGET` (kSimRegistry[]) keys -- a bare `SIMGET` never
# returns them, so `_discover_simset_keys`/`KNOB_MAP` correctly never sees
# them. They tune FUSION WEIGHTING (how strongly a disagreement between
# otos=/encpose= pulls the fused pose=), a fundamentally different mechanism
# from every other knob in this file (which inject plant/observation ERROR).
# The mapping this ticket's acceptance criteria ask for is: expected field =
# fused pose='s divergence from otos=/encpose= under a deliberate
# disagreement, exercised by `test_069_001_ekf_noise_registry.py::
# test_ekfrotosxy_changes_position_correction_strength` (varying ekfROtosXy
# changes how strongly an OTOS/encoder disagreement is corrected) -- reusing
# that existing, already-passing test rather than re-deriving a second
# "perturb the EKF weighting and observe pose=" harness here, per this
# ticket's own Implementation Plan.
# ---------------------------------------------------------------------------
EKF_KEYS_OUT_OF_SCOPE = {
    "ekfQxy":     "process noise, position -- see test_069_001_ekf_noise_registry.py",
    "ekfQtheta":  "process noise, heading -- see test_069_001_ekf_noise_registry.py",
    "ekfQv":      "process noise, linear velocity -- see test_069_001_ekf_noise_registry.py",
    "ekfQomega":  "process noise, angular velocity -- see test_069_001_ekf_noise_registry.py",
    "ekfROtosXy": "OTOS position measurement noise -- see test_069_001_ekf_noise_registry.py "
                  "(test_ekfrotosxy_changes_position_correction_strength)",
    "ekfROtosV":  "OTOS velocity measurement noise -- see test_069_001_ekf_noise_registry.py",
    "ekfREncV":   "encoder velocity measurement noise -- see test_069_001_ekf_noise_registry.py",
}


# ---------------------------------------------------------------------------
# KNOB_MAP -- key -> (group, check function, per-key config).
#
# Every value/axis/threshold below was picked empirically against this sim
# build (fresh Sim() baseline vs one perturbed knob, identical maneuver) --
# see the module docstring's "Design" section for the per-group rationale.
# ---------------------------------------------------------------------------

GROUND_TRUTH_SCRUB = {
    # RT 9000 true heading: baseline ~94 deg true (rotSlip=0.92 default
    # inflates the commanded arc -- see test_069_rt_90deg_body_scrub.py);
    # bodyRotScrub=0.5 roughly halves the ACTUAL body rotation for that same
    # commanded arc. Observed delta ~4700 centideg; threshold set to 1000.
    "bodyRotScrub": dict(value=0.5, maneuver=_drive_rt, axis=2, min_delta=1000.0),
    # T 200,200 straight: bodyLinScrub=0.5 halves true linear travel for the
    # same commanded wheel arc. Observed delta ~194 mm; threshold 100.
    "bodyLinScrub": dict(value=0.5, maneuver=_drive_straight, axis=0, min_delta=100.0),
    # RT 9000: trackwidthMm=200 (vs. the plant's boot-synced 128 mm default)
    # changes the true dTheta-per-differential-arc ratio while Odometry's
    # own kinematics keep using the fixed RobotConfig.trackwidthMm=128 --
    # a genuine heading-RATE discrepancy. Observed delta ~3385 centideg;
    # threshold 1000.
    "trackwidthMm": dict(value=200.0, maneuver=_drive_rt, axis=2, min_delta=1000.0),
}

PHYSICAL_ASYMMETRY = {
    "motorOffsetL": dict(value=0.8, side=0),
    "motorOffsetR": dict(value=0.8, side=1),
}

ENCODER_REPORT_ERROR = {
    "encScaleErrL": dict(value=0.05, side=0),
    "encScaleErrR": dict(value=-0.05, side=1),
    "encSlipL": dict(value=0.05, side=0),
    "encSlipR": dict(value=0.06, side=1),
    # Noise's parameter is added to a per-tick VELOCITY term then scaled by
    # dt_s (legacy MockMotor "bit-for-bit" behaviour, PhysicsWorld.cpp sub-
    # step A' comment) -- its effective per-tick position contribution is
    # much smaller than the nominal sigma value, so a realistic-looking
    # sigma (~1-3 mm) is invisible at integer-mm TLM precision over a 2 s
    # drive. 50.0 is chosen for a robust, clearly-observable signal, not as
    # a realistic hardware noise figure.
    "encNoiseL": dict(value=50.0, side=0),
    "encNoiseR": dict(value=50.0, side=1),
}

OTOS_ERROR = {
    "otosLinScaleErr": dict(value=0.10, maneuver=_drive_straight, axis=0, min_delta=20.0),
    "otosAngScaleErr": dict(value=0.10, maneuver=_drive_rt, axis=2, min_delta=500.0),
    "otosLinNoise": dict(value=0.5, maneuver=_drive_straight, axis=0, min_delta=8.0),
    "otosYawNoise": dict(value=0.15, maneuver=_drive_rt, axis=2, min_delta=50.0),
    "otosLinDriftMmS": dict(value=10.0, maneuver=_drive_straight, axis=0, min_delta=5.0),
    # Yaw drift while driving STRAIGHT (no commanded rotation) makes the
    # reported track curve -- the clearest observable axis is heading, not x.
    "otosYawDriftDegS": dict(value=5.0, maneuver=_drive_straight, axis=2, min_delta=200.0),
}

KNOB_MAP: dict[str, tuple] = {}
for _k, _cfg in GROUND_TRUTH_SCRUB.items():
    KNOB_MAP[_k] = (_check_scrub_or_otos_error, _cfg)
for _k, _cfg in PHYSICAL_ASYMMETRY.items():
    KNOB_MAP[_k] = (_check_physical_asymmetry, _cfg)
for _k, _cfg in ENCODER_REPORT_ERROR.items():
    KNOB_MAP[_k] = (_check_encoder_report_error, _cfg)
for _k, _cfg in OTOS_ERROR.items():
    KNOB_MAP[_k] = (_check_scrub_or_otos_error, _cfg)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_no_unmapped_simset_keys(sim) -> None:
    """Every key a bare ``SIMGET`` returns MUST have a ``KNOB_MAP`` entry.

    This is what makes the sweep's "automatic coverage of future additions"
    claim actually enforced: a future ticket that appends a new
    ``kSimRegistry[]`` row without updating ``KNOB_MAP`` fails HERE, loudly,
    naming the exact unmapped key -- rather than the sweep silently only
    covering the keys someone happened to remember to add.
    """
    discovered = set(_discover_simset_keys(sim))
    unmapped = discovered - set(KNOB_MAP)
    assert not unmapped, (
        f"SIMGET returned key(s) with no entry in this test file's "
        f"KNOB_MAP: {sorted(unmapped)!r} -- add a "
        f"(designated-TLM-field, maneuver, threshold) entry for each to "
        f"tests/simulation/system/test_069_knob_telemetry_sweep.py before "
        f"this test can pass."
    )


def test_no_stale_mapped_keys(sim) -> None:
    """Every ``KNOB_MAP`` entry must correspond to a REAL registered key.

    Guards the reverse direction: if a knob is ever removed from
    ``kSimRegistry[]``, this fails loudly instead of the sweep silently
    keeping (and passing) a dead entry.
    """
    discovered = set(_discover_simset_keys(sim))
    stale = set(KNOB_MAP) - discovered
    assert not stale, (
        f"KNOB_MAP has entry/entries for key(s) SIMGET no longer reports: "
        f"{sorted(stale)!r} -- these knobs appear to have been removed "
        f"from kSimRegistry[]; remove the corresponding KNOB_MAP entries."
    )


@pytest.mark.parametrize("key", sorted(KNOB_MAP))
def test_knob_moves_expected_signal(key: str) -> None:
    """For each mapped SIMSET key: drive the same maneuver on a fresh
    baseline sim (knob at default) and a fresh perturbed sim (knob set to a
    distinctly non-default value), and assert the knob's designated TLM
    signal diverges while any meaningfully-isolable unrelated signal does
    not. See the four ``_check_*`` functions above for the per-group
    assertion shape and the module docstring for why each group needs a
    different one.
    """
    check_fn, cfg = KNOB_MAP[key]
    check_fn(key, cfg)
