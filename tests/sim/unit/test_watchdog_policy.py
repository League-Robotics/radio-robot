"""Watchdog policy test (sprint 081, ticket 005): lowering the
serial-silence watchdog window below the ``sim`` fixture's wide default and
then going quiet for longer than that window fires exactly one
``EVT dev_watchdog``, drained via ``Sim.get_async_evts()`` -- and
neutralizes the commanded motor.
"""

_NARROW_WINDOW = 100   # [ms] -- above dev_commands.cpp's kDevWdArgs floor (50)


def test_watchdog_fires_after_window_expires_and_neutralizes(sim):
    # Override the sim fixture's wide (60 s) default with a narrow window.
    reply = sim.command(f"DEV WD {_NARROW_WINDOW}")
    assert reply.strip() == f"OK DEV WD window={_NARROW_WINDOW}"

    sim.command("DEV M 1 VEL 50")

    # Still well inside the window -- no EVT yet.
    sim.tick_for(60)
    assert "dev_watchdog" not in sim.get_async_evts()

    # Push past the window with no further command line arriving.
    sim.tick_for(200)
    evts = sim.get_async_evts()
    assert "EVT dev_watchdog" in evts

    # The watchdog neutralized the motor (broadcast BRAKE neutral).
    state = sim.command("DEV M 1 STATE")
    assert "applied=0.00" in state


def test_watchdog_does_not_fire_while_commands_keep_arriving(sim):
    sim.command(f"DEV WD {_NARROW_WINDOW}")
    sim.command("DEV M 1 VEL 50")

    # Re-feed the watchdog every 60 ms (< the 100 ms window) via a liveness
    # PING -- any command line resets the timer, regardless of content.
    for _ in range(5):
        sim.tick_for(60)
        sim.command("PING")

    assert "dev_watchdog" not in sim.get_async_evts()


def test_watchdog_neutralizes_within_the_same_pass_it_fires_in(sim):
    """Ticket 087-007's own safety-critical acceptance: the emergency
    neutralize is a narrow bypass straight to ``Hardware::apply()``/
    ``Drivetrain::apply()`` (``Rt::MainLoop::emergencyNeutralize()``, called
    as the FIRST action of ``Rt::MainLoop::tick()`` -- see main_loop.h's file
    header) -- it never posts to ``bb.motorIn[]``/``bb.driveIn`` and so never
    waits an extra pass for those one-tick queues to be drained. Placing the
    watchdog check before this pass's own ``Hardware::tick()`` call means the
    SAME pass that detects the expired window already stages the motor's
    neutral mode before that pass's own ``SimMotor::tick()`` runs -- so the
    plant's raw commanded actuator value (``sim.pwm()``, ground truth,
    independent of any Blackboard cell) reads exactly 0 by the end of THAT
    SAME ``sim_tick()`` call, not a later one. A queue-routed neutralize
    would need a WHOLE EXTRA pass just to be drained before ever reaching
    this same staging point -- this test would catch that regression by
    seeing a still-nonzero pwm on the firing pass.
    """
    reply = sim.command(f"DEV WD {_NARROW_WINDOW}")
    assert reply.strip() == f"OK DEV WD window={_NARROW_WINDOW}"

    sim.command("DEV M 1 VEL 50")

    step = 20   # [ms] -- fine-grained single-tick stepping (not tick_for()'s
                # bunched multi-tick advance) so the exact firing pass is
                # observed in isolation, with no later tick masking a latent
                # extra-pass delay.

    # Let the PID actually spin the motor up before the window can matter.
    sim.tick_for(60, step=step)
    assert sim.pwm()[0] != 0.0, "expected the motor to be driving before the watchdog fires"
    assert "dev_watchdog" not in sim.get_async_evts()

    fired = False
    for _ in range(20):
        sim.tick_for(step, step=step)
        if "EVT dev_watchdog" in sim.get_async_evts():
            fired = True
            break

    assert fired, "expected the watchdog to fire within the loop"
    assert sim.pwm() == (0.0, 0.0), (
        "expected the SAME pass that fired the watchdog to already show the "
        "motor neutralized (ground-truth pwm=0) -- no additional pass of "
        "bb.motorIn[]/bb.driveIn queue latency"
    )
