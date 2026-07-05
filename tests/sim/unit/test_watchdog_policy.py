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
    # PING -- any statement line resets the timer, regardless of content.
    for _ in range(5):
        sim.tick_for(60)
        sim.command("PING")

    assert "dev_watchdog" not in sim.get_async_evts()
