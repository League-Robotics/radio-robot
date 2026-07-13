"""tests/sim/unit/test_otos_fusion_live.py -- 099-007 (SUC-003): proves OTOS
fusion is genuinely LIVE in `Rt::MainLoop::tick()` -- the "one-token flip"
that stops passing a literal `nullptr` for `otosObs` into
`Subsystems::PoseEstimator::tick()` and instead assembles a real
observation gated on `Hal::Odometer::fusableThisPass()`
(`source/runtime/main_loop.cpp`).

Before this ticket, `bb.fusedPose` was numerically IDENTICAL to
`bb.encoderPose` on every tick -- `otosObs` was always `nullptr`
(`pose_estimator_harness.cpp`'s own scenario (a) proves `fusedPose() ==
encoderPose()` exactly whenever that holds: predict() still runs, but
`updatePosition()`/`updateHeading()` never fire with nothing to correct
against). Fusion existed in code, wired and unit-tested in isolation
(ticket 006's EKF gate harness, `test_ekf_tiny.py`), but never actually ran
end to end through the live main loop.

This test drives the sim robot with a deliberately INJECTED encoder error
(`Hal::PhysicsWorld`'s per-wheel encoder slip, `firmware.py`'s
`set_enc_slip()` -- affects ONLY the REPORTED encoder accumulator
`PoseEstimator`'s dead-reckoning reads, never the true chassis pose
`Hal::SimOdometer` samples via `PhysicsWorld::truePose*()` -- see
`physics_world.h`'s own file header: "PhysicsWorld tracks BOTH a true
(unslipped) encoder accumulator AND a reported ... accumulator") so
`encoderPose()` provably diverges from the true pose while the OTOS
reading stays accurate, then confirms `fusedPose()` is pulled MEASURABLY
toward the accurate (true/OTOS) reading rather than drifting along with
the biased encoder-only dead reckoning -- proof the correction step
(`EkfTiny::updatePosition()`/`updateHeading()`) is actually being reached
and having a real numerical effect on the live main loop, not just
wired-but-inert.

A zero-injected-error control run is deliberately NOT the primary proof
here: with a perfect (zero-noise, zero-scale-error) `SimOdometer` sampling
a perfect (zero-slip) `PhysicsWorld`, dead reckoning and the OTOS reading
already agree almost exactly, so a live EKF correction has near-zero
innovation to act on and the resulting fusedPose/encoderPose gap would be
too small (single-digit mm, observed ~0.04mm in exploratory testing) to
distinguish "fusion is live but has nothing to correct" from "fusion never
runs" -- exactly the "trivial/zero pose that makes fusion a no-op"
limitation this ticket's own instructions warn against silently
papering over. Injecting encoder slip manufactures a real, sustained
disagreement so the test is unambiguous either way.
"""
from __future__ import annotations

import pathlib
import sys

import pytest

# tests/sim/unit/test_otos_fusion_live.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_HOST_DIR = _REPO_ROOT / "host"
if str(_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_HOST_DIR))

# tests/sim/conftest.py's build_lib fixture already inserts this path; guard
# against a double-insert if this module is imported before that fixture runs
# (same guard _binary_envelope.py/test_binary_channel.py both use).
_SIM_INFRA_DIR = _REPO_ROOT / "tests" / "_infra" / "sim"
if str(_SIM_INFRA_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_INFRA_DIR))

from _binary_envelope import read_tlm_now, send_drive  # noqa: E402


def test_fused_pose_tracks_true_pose_better_than_encoder_only_under_slip(sim):
    """With 15% encoder slip injected on both wheels (a symmetric bias --
    straight driving stays straight, but reported travel undercounts true
    travel), `encoderPose()` (`sim.enc_pose()`) drifts substantially behind
    the true chassis pose (`sim.true_pose()`) while `SimOdometer` -- which
    samples `PhysicsWorld`'s TRUE pose directly, never the reported/slipped
    encoder accumulator -- stays accurate. `fusedPose()` (TLM `pose=`,
    `read_tlm_now(sim).pose`) must end up MEASURABLY closer to the true
    pose than the encoder-only estimate: proof `PoseEstimator::tick()` is
    now receiving a real, non-null `otosObs` and its EkfTiny correction step
    is having a real effect, not just wired-but-inert (as it was before this
    ticket, when `otosObs` was a literal `nullptr` and `fusedPose() ==
    encoderPose()` always -- `pose_estimator_harness.cpp` scenario (a))."""
    sim.set_enc_slip(0, 0.15)
    sim.set_enc_slip(1, 0.15)

    assert send_drive(sim, 150, 150).WhichOneof("body") == "ok"
    sim.tick_for(2000)   # drive 2s -- enough for the slip bias to accumulate

    true_x, _true_y, _true_h = sim.true_pose()
    enc_x, _enc_y, _enc_h = sim.enc_pose()
    fused = read_tlm_now(sim)

    enc_error = abs(enc_x - true_x)
    fused_error = abs(fused.pose.x - true_x)

    # Sanity check: the injected slip must have actually created a real,
    # substantial divergence for encoder-only dead reckoning -- otherwise
    # the comparison below would prove nothing (observed ~50mm at 2s/150mm/s
    # with 15% slip; 25mm leaves ample margin against build/float drift).
    assert enc_error > 25.0, (
        f"encoder-only dead reckoning only diverged {enc_error:.2f}mm from "
        "true pose -- the injected 15% slip should have produced a much "
        "larger, unambiguous divergence for this test to be meaningful"
    )

    # The actual proof: fusedPose() is pulled substantially closer to the
    # true (OTOS-informed) pose than the biased encoder-only estimate --
    # observed ~3.2mm fused error vs ~50.4mm encoder error (~15x tighter);
    # 0.5x leaves generous margin while still failing hard if fusion were
    # silently disabled again (in which case fused_error == enc_error
    # exactly, since fusedPose() would just equal encoderPose()).
    assert fused_error < enc_error * 0.5, (
        f"fusedPose().pose.x ({fused.pose.x:.2f}, error {fused_error:.2f}mm "
        f"from true {true_x:.2f}mm) is not measurably closer to the true "
        f"pose than encoder-only dead reckoning (error {enc_error:.2f}mm) -- "
        "OTOS fusion does not appear to be live"
    )

    # Direct confirmation that fusedPose() and encoderPose() have actually
    # DIVERGED from each other -- the pre-007 invariant (otosObs always
    # nullptr -> fusedPose() == encoderPose() exactly) no longer holds now
    # that a real observation is being fed in.
    assert fused.pose.x != pytest.approx(enc_x, abs=1.0), (
        "fusedPose().pose.x still equals encoderPose().x -- OTOS fusion "
        "does not appear to be reaching PoseEstimator::tick() at all"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
