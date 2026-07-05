"""Determinism test (sprint 081, ticket 005): running an IDENTICAL command
+ tick script on two independent, freshly-created Sim instances produces
bit-identical state logs. This is the general-purpose sibling of
test_errored_observation.py's zero-error gate -- it holds regardless of
which error knobs are configured (including nonzero ones), because
PhysicsWorld's and SimOdometer's std::mt19937 generators are freshly
constructed with a fixed seed (42u/43u) every ``sim_create()`` call, so two
fresh instances draw the identical noise sequence for the identical
sequence of ticks/commands.
"""
from firmware import Sim

_WATCHDOG_WIDE_WINDOW = 60000   # [ms] -- see tests/sim/conftest.py


def _run_script(s: Sim) -> list[tuple]:
    """A representative command + tick script: straight drive, a turn, a
    velocity step, and injected (nonzero) noise -- exercising the plant,
    the PID, and the stochastic error model all in one pass."""
    log: list[tuple] = []

    s.command(f"DEV WD {_WATCHDOG_WIDE_WINDOW}")
    s.set_enc_noise(2, 1.5)          # [mm] nonzero -- proves determinism
    s.set_otos_linear_noise(0.02)    # holds even with the RNG streams live
    s.set_otos_yaw_noise(0.01)

    s.command("DEV M 1 DUTY 70")
    s.command("DEV M 2 DUTY 70")
    for _ in range(10):
        s.tick_for(50)
        log.append((s.true_pose(), s.enc(), s.otos_pose(), s.vel(), s.pwm()))

    s.command("DEV M 1 DUTY 30")
    s.command("DEV M 2 DUTY -30")
    for _ in range(10):
        s.tick_for(50)
        log.append((s.true_pose(), s.enc(), s.otos_pose(), s.vel(), s.pwm()))

    s.command("DEV M 1 VEL 150")
    s.command("DEV M 2 VEL 150")
    for _ in range(10):
        s.tick_for(50)
        log.append((s.true_pose(), s.enc(), s.otos_pose(), s.vel(), s.pwm()))

    return log


def test_identical_script_produces_bit_identical_state_logs(build_lib):
    with Sim() as s1:
        log1 = _run_script(s1)
    with Sim() as s2:
        log2 = _run_script(s2)

    assert len(log1) == len(log2)
    assert log1 == log2
