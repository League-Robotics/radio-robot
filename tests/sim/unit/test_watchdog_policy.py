"""Watchdog policy test (sprint 081, ticket 005): lowering the
serial-silence watchdog window below the ``sim`` fixture's wide default and
then going quiet for longer than that window fires exactly one
``EVT dev_watchdog``, drained via ``Sim.get_async_evts()`` -- and
neutralizes the commanded motor.

091-003 (architecture-update.md Decision 3) adds the fire-GATE: comms
silence past the window only fires while motors are actually commanded to
run (``bb.drivetrain.active || any(bb.motors[i].active)``). The three
tests above (unmodified per this ticket's acceptance) already command a
motor before going silent, so ``bb.motors[0].active`` stays true
throughout and their outcome is unchanged. The two tests below cover the
NEW idle case: motors stopped/neutral + silence past the window must NOT
fire, whether never commanded at all, or driven and then explicitly
neutralized before going silent (proving the gate reads CURRENT state, not
"was ever commanded").
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
    ``Drivetrain::apply()`` (``Rt::MainLoop::estop()``, called
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


def test_watchdog_does_not_fire_when_idle(sim):
    """091-003 (architecture-update.md Decision 3): the robot is completely
    idle -- no ``DEV M``/``DEV DT`` motion verb has EVER been issued -- so
    ``bb.drivetrain.active`` and every ``bb.motors[i].active`` are false.
    Comms silence past the (narrowed) window must NOT fire: no neutralize,
    no ``EVT dev_watchdog``. This is the spurious-fire case the issue
    (watchdog-arm-only-while-motors-running.md) exists to fix.
    """
    reply = sim.command(f"DEV WD {_NARROW_WINDOW}")
    assert reply.strip() == f"OK DEV WD window={_NARROW_WINDOW}"

    # No motion verb ever issued. Push well past the window with no further
    # command arriving.
    sim.tick_for(200)

    evts = sim.get_async_evts()
    assert "dev_watchdog" not in evts

    # The (already-neutral) motor state is unaffected -- no spurious estop.
    assert sim.pwm() == (0.0, 0.0)


def test_watchdog_does_not_fire_after_explicit_neutralize(sim):
    """091-003: a port WAS driven, then explicitly neutralized
    (``DEV M <n> NEUTRAL B``) before comms go silent past the window --
    still no fire. Proves the fire-gate reads bb's CURRENT commanded state
    each pass, not a "was this port ever commanded" latch -- the exact
    one-way-latch anti-pattern Decision 3 rejects (architecture-update.md's
    Decision 3 alternative (c)).
    """
    sim.command(f"DEV WD {_NARROW_WINDOW}")
    sim.command("DEV M 1 VEL 50")

    sim.tick_for(60)
    assert sim.pwm()[0] != 0.0, "expected the motor to be driving before neutralizing"

    sim.command("DEV M 1 NEUTRAL B")

    # Silence past the window, with the port already explicitly neutral.
    sim.tick_for(200)

    evts = sim.get_async_evts()
    assert "dev_watchdog" not in evts
    assert sim.pwm() == (0.0, 0.0)
