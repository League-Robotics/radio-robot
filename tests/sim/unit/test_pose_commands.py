"""Off-hardware acceptance proof for ticket 084-007 (SUC-006): the pose-set
command surface -- `SI` (re-anchor the believed world pose, calling
``Subsystems::PoseEstimator::setPose()`` directly -- architecture-update.md
(084) Decision 1) and `ZERO enc` (rezero the bound pair's hardware encoders
AND ``PoseEstimator``'s own encoder-baseline accumulator in the same wire
dispatch, so the next tick's delta is computed against the freshly-zeroed
encoders with no phantom-jump discontinuity).

Drives ``libfirmware_host`` through the full wire dispatch (``Sim.command()``)
-- ``CommandProcessor`` -> ``source/commands/pose_commands.cpp`` ->
``Subsystems::PoseEstimator``/``Subsystems::Hardware`` -- mirroring this
directory's existing ``test_tlm_stream_snap.py``/``test_motion_commands_*``
pattern.

**`SI`'s `pose=` (fused) caveat -- recorded, not silently hidden.** `SI`
re-anchors `PoseEstimator::setPose()` alone (Decision 1's exact scope);
it does NOT re-anchor the active `Hal::Odometer`'s own world-frame reading
-- that capability (`Hal::Odometer::apply()`'s new `set_pose` arm, wired to
the `OV` wire verb) is added by ticket 084-008, sequenced directly after
this one (`008-otos-command-surface-*.md`'s `depends-on: ['007']`). Since
the sim always has a live odometer (`Subsystems::SimHardware::odometer()`
is never null), the very next `devLoopTick()` pass fuses a fresh,
un-reanchored OTOS reading into the EKF, pulling `pose=` partway back
toward the odometer's own (unrelated) frame -- measured (2026-07-06): gain
`P/(P+R) = 100/(100+50) ~= 0.667` (`EkfTiny::setPose()`'s `kPriorXY=100`
vs. `PoseEstimator::configure()`'s default `ekf_r_otos_xy` fallback of 50),
landing `pose=` roughly a third of the way from the pre-SI value to the new
anchor rather than exactly at it. This is the SAME "starts right, then
rotates away" hazard `source_old/commands/SystemCommands.cpp`'s own
`handleSI` comment names, which that implementation patched with a SECOND
call (`hal.otos().setWorldPose()`) this ticket's `Hal::Odometer` faceplate
does not yet expose -- a known, ticket-008-sequenced gap, not a defect in
this ticket's own `PoseEstimator::setPose()`/`SI` wiring. `encpose=` (pure
dead reckoning -- the EKF never writes there) is fully immune to this and
reads back EXACTLY.

**`ZERO enc`'s deferred-hardware-effect subtlety.** `Hal::Motor::
resetPosition()` is itself STAGED ("zero encoder (staged, not immediate)"
-- `hal/capability/motor.h`), so its actual hardware effect lands only on
the leaf's next GENUINELY time-advancing `tick()` -- not necessarily the
SAME command-dispatch pass (the sim harness's `sim_command()` dt=0 replay
trick -- `tests/_infra/sim/sim_api.cpp`'s own Decision 4 doc comment --
makes the `ZERO enc` command's OWN pass a guaranteed hardware-tick no-op).
`PoseEstimator::resetEncoderBaseline()` accounts for this: it only ARMS a
pending flag, applied by `PoseEstimator::tick()` on the first subsequent
call whose `dt` is genuinely `> 0` (see that method's own doc comment,
`pose_estimator.h`) -- this test exercises exactly that path via a real
`tick_for()` immediately after `ZERO enc`.
"""

from __future__ import annotations


def _parse_tlm(line: str) -> dict[str, str]:
    """Parse one "TLM t=... mode=... ..." wire line into a key->value dict.

    Local, small, deliberately duplicated per test file -- mirrors this
    directory's existing precedent (e.g. test_tlm_stream_snap.py's own
    _parse_tlm()).
    """
    parts = line.strip().split()
    assert parts[0] == "TLM", f"not a TLM line: {line!r}"
    return dict(p.split("=", 1) for p in parts[1:])


def _snap(sim) -> dict[str, str]:
    """Issue SNAP and parse its reply (see test_tlm_stream_snap.py's own
    _snap() for the "exactly one TLM line" precondition)."""
    reply = sim.command("SNAP").strip()
    lines = reply.splitlines()
    assert len(lines) == 1, f"expected exactly one TLM line from SNAP, got: {reply!r}"
    return _parse_tlm(lines[0])


# ---------------------------------------------------------------------------
# SI -- SI <x> <y> <h> (mm, mm, cdeg): re-anchors PoseEstimator::setPose()
# directly (Decision 1) -- never routes through Drivetrain::apply()'s POSE
# arm, which stays the documented no-op it is today.
# ---------------------------------------------------------------------------


def test_si_replies_ok_setpose_with_the_supplied_values(sim):
    reply = sim.command("SI 1000 500 900")
    assert reply.strip() == "OK setpose x=1000 y=500 h=900"


def test_si_encpose_reads_back_exactly_on_the_next_snap(sim):
    """encpose= (PoseEstimator::encoderPose(), the pure dead-reckoning
    accumulator the EKF never writes to) is fully deterministic and reads
    back EXACTLY what SI set -- see this file's module docstring for why
    pose= (fused) is checked separately, with a documented tolerance."""
    sim.command("SI 1000 500 900")
    tlm = _snap(sim)
    assert tlm["encpose"] == "1000,500,900"


def test_si_substantially_shifts_the_fused_pose_toward_the_new_anchor(sim):
    """pose= (PoseEstimator::fusedPose()) is ALSO re-anchored by setPose()
    -- confirmed here by a large, unambiguous shift away from its pre-SI
    value -- but is immediately partially pulled back toward the (un-
    reanchored) live sim odometer's own frame by the very next OTOS fusion
    (see this file's module docstring for the measured ~0.667 gain and the
    ticket-008 sequencing that closes this gap). This test records that
    substantial-but-partial shift rather than asserting an exact readback.
    """
    before = _snap(sim)
    assert before["pose"] == "0,0,0"

    sim.command("SI 1000 500 900")
    tlm = _snap(sim)
    x, y, h = (int(v) for v in tlm["pose"].split(","))
    # Measured (2026-07-06): lands near (333, 166, 510) -- a substantial,
    # unambiguous shift toward (1000, 500, 900), not a no-op and not an
    # exact match. Loose bounds below only prove "moved substantially
    # toward the anchor," not the exact converged value (which depends on
    # EKF noise constants this ticket does not own).
    assert x > 200, f"expected a substantial shift toward x=1000, got x={x}"
    assert y > 100, f"expected a substantial shift toward y=500, got y={y}"
    assert h > 200, f"expected a substantial shift toward h=900, got h={h}"


def test_si_too_few_args_rejected_with_badarg(sim):
    assert sim.command("SI 1000 500").strip() == "ERR badarg"
    assert sim.command("SI 1000").strip() == "ERR badarg"
    assert sim.command("SI").strip() == "ERR badarg"


def test_si_mid_g_does_not_cancel_the_active_goal(sim):
    """Open Question 4 (architecture-update.md (084)): SI does not itself
    cancel an in-flight Planner command -- source_old's own SI never did
    either, and this sprint preserves that. RECORDS the observed behavior
    (G keeps running against its already-resolved world-frame target and
    still completes) rather than asserting a specific "corrected"
    trajectory, which was never designed."""
    reply = sim.command("G 300 0 200")
    assert reply.strip() == "OK goto x=300 y=0 speed=200"

    # Let PURSUE engage; G's world-frame target anchor is resolved on the
    # FIRST tick() (Planner::captureBaseline()) against the near-(0,0,0)
    # starting pose and stays FIXED from then on -- SI re-anchoring the
    # BELIEVED pose later does not move this already-resolved target.
    sim.tick_for(200)
    assert sim.get_async_evts() == "", "should still be mid-flight, not yet arrived"
    assert _snap(sim)["mode"] == "G"
    x_before, _y_before, _h_before = sim.true_pose()
    assert x_before > 0.0, "expected forward progress before the SI correction"

    # A deliberate, large "camera says you're somewhere else" re-anchor,
    # far from where the controller currently believes it is.
    si_reply = sim.command("SI -200 300 0")
    assert si_reply.strip() == "OK setpose x=-200 y=300 h=0"

    # Observed: G keeps running (not cancelled) immediately after SI --
    # mode= stays 'G', the plant keeps responding to Planner's steering.
    assert _snap(sim)["mode"] == "G"

    # Recorded, not asserted-specific: the goal eventually resolves one way
    # or another (does not hang or silently vanish) despite the mid-flight
    # pose teleport -- Open Question 4 leaves the exact "corrected"
    # trajectory unspecified.
    sim.tick_for(6000)
    evts = sim.get_async_evts()
    assert "EVT done G" in evts, (
        f"expected G to eventually reach SOME completion despite the mid-flight "
        f"SI correction (Open Question 4's course-correction, not cancellation), "
        f"got evts={evts!r}"
    )


# ---------------------------------------------------------------------------
# ZERO enc -- rezeroes the bound pair's hardware encoders AND PoseEstimator's
# encoder-baseline accumulator in the same wire dispatch.
# ---------------------------------------------------------------------------


def test_zero_enc_replies_ok(sim):
    assert sim.command("ZERO enc").strip() == "OK zero enc"


def test_zero_without_enc_token_rejected_with_badarg(sim):
    assert sim.command("ZERO").strip() == "ERR badarg"
    assert sim.command("ZERO pose").strip() == "ERR badarg"


def test_zero_enc_rezeroes_reported_encoders(sim):
    sim.command("S 200 200")
    sim.tick_for(500)
    sim.command("STOP")
    sim.tick_for(200)
    assert sim.enc() != (0.0, 0.0), "expected non-zero accumulated travel before ZERO"

    sim.command("ZERO enc")
    # Hal::Motor::resetPosition() is staged, not immediate (hal/capability/
    # motor.h) -- one real tick is needed for the leaf's tick() to actually
    # apply it (see this file's module docstring).
    sim.tick_for(24)

    enc_l, enc_r = sim.enc()
    assert abs(enc_l) < 1.0 and abs(enc_r) < 1.0, (
        f"expected ZERO enc to rezero the reported encoders, got enc=({enc_l}, {enc_r})"
    )


def test_zero_enc_no_phantom_jump_on_the_following_tick(sim):
    """The core acceptance proof: PoseEstimator::resetEncoderBaseline()
    (armed by ZERO enc's handler, applied by tick() on the first
    subsequent dt > 0 call -- see this file's module docstring) must
    prevent the freshly-zeroed hardware reading from being diffed against
    the STALE pre-zero baseline, which would otherwise fabricate a large
    phantom jump in encpose= the instant the staged hardware reset lands.
    """
    sim.command("S 200 200")
    sim.tick_for(500)
    sim.command("STOP")
    sim.tick_for(200)

    before = _snap(sim)
    x_before, y_before, h_before = (int(v) for v in before["encpose"].split(","))
    assert x_before > 20, f"expected meaningful accumulated travel before ZERO, got x={x_before}"

    sim.command("ZERO enc")
    # The first post-ZERO real tick -- the one where the staged hardware
    # reset actually lands (see test_zero_enc_rezeroes_reported_encoders).
    sim.tick_for(24)

    after = _snap(sim)
    x_after, y_after, h_after = (int(v) for v in after["encpose"].split(","))

    # No phantom jump: encpose= must stay close to its pre-ZERO value (the
    # robot is stationary throughout -- ZERO enc does not itself move the
    # believed pose, only resyncs the encoder-delta baseline it is computed
    # from going forward). A bug here (the one-shot guard consumed too
    # early by a stale dt==0 replay pass -- see the module docstring)
    # previously fabricated a ~-95 mm jump on this exact assertion.
    assert abs(x_after - x_before) < 5, (
        f"phantom jump detected: encpose x moved from {x_before} to {x_after} "
        f"across ZERO enc's first following tick"
    )
    assert abs(y_after - y_before) < 5, (
        f"phantom jump detected: encpose y moved from {y_before} to {y_after} "
        f"across ZERO enc's first following tick"
    )
    assert abs(h_after - h_before) < 50, (
        f"phantom jump detected: encpose h moved from {h_before} to {h_after} "
        f"across ZERO enc's first following tick"
    )

    # And the hardware encoder itself really did rezero (this is not just
    # "ZERO enc was a no-op").
    enc_l, enc_r = sim.enc()
    assert abs(enc_l) < 1.0 and abs(enc_r) < 1.0

    # Future dead-reckoning resumes correctly from the fresh zero baseline
    # (not corrupted) -- driving again afterward accumulates a sane delta.
    sim.command("S 200 200")
    sim.tick_for(500)
    after_drive = _snap(sim)
    x_after_drive, _y, _h = (int(v) for v in after_drive["encpose"].split(","))
    assert x_after_drive > x_after + 20, (
        "expected encpose to keep advancing normally after ZERO enc's rebaseline"
    )
