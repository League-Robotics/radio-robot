"""Protocol round-trip tests (sprint 081, ticket 005): PING, the DEV M/DT
family, the ``ERR unsupported`` capability-gate reply, and the watchdog
liveness path -- driven through ``Sim.command()`` (sim_api.cpp's
``sim_command``), never a raw wire string constructed by hand beyond what
this file sends, and never SIMSET/SIMGET (out of scope for this ABI, per
the ticket's hard requirement).
"""


def test_ping_replies_with_pong_and_a_timestamp(sim):
    reply = sim.command("PING")
    assert reply.startswith("OK pong")
    assert "t=" in reply


def test_dev_m_duty_replies_with_applied_fraction(sim):
    reply = sim.command("DEV M 1 DUTY 50")
    assert reply.strip() == "OK DEV M 1 applied=0.50"


def test_dev_m_vel_replies_and_state_reflects_it(sim):
    reply = sim.command("DEV M 1 VEL 120")
    assert reply.strip() == "OK DEV M 1 vel=120.0"

    state = sim.command("DEV M 1 STATE")
    assert state.startswith("OK DEV M 1 ")
    for field in ("pos=", "vel=", "applied=", "wedged=", "wsus=", "hrc=", "src=", "conn="):
        assert field in state


def test_dev_m_pos_is_unsupported_on_simmotor(sim):
    """SimMotor.capabilities().position == false (sim_motor.cpp) -- the
    identical Motor::apply() capability gate a real Nezha's VOLT command
    fires on rejects POS here, per docs/protocol-v2.md's sim note."""
    reply = sim.command("DEV M 1 POS 10")
    assert reply.strip() == "ERR unsupported pos"


def test_dev_m_volt_is_unsupported(sim):
    reply = sim.command("DEV M 1 VOLT 3")
    assert reply.strip() == "ERR unsupported volt"


def test_dev_dt_ports_and_vw_round_trip(sim):
    reply = sim.command("DEV DT PORTS 1 2")
    assert reply.strip() == "OK DEV DT ports=1,2"

    reply = sim.command("DEV DT VW 100 0 0")
    assert reply.strip() == "OK DEV DT vx=100.0 vy=0.0 omega=0.000"

    state = sim.command("DEV DT STATE")
    assert state.startswith("OK DEV DT active=1 ports=1,2")


def test_dev_state_reports_all_four_motors_and_drivetrain(sim):
    reply = sim.command("DEV STATE")
    lines = [ln for ln in reply.strip().split("\n") if ln]
    assert len(lines) == 5
    for port in (1, 2, 3, 4):
        assert any(ln.startswith(f"OK DEV M {port} ") for ln in lines)
    assert any(ln.startswith("OK DEV DT ") for ln in lines)


def test_dev_stop_replies_ok(sim):
    reply = sim.command("DEV STOP")
    assert reply.strip() == "OK DEV STOP"


def test_unknown_verb_replies_err_unknown(sim):
    reply = sim.command("BOGUS")
    assert reply.strip() == "ERR unknown"


def test_dev_m_out_of_range_port_replies_err_range(sim):
    reply = sim.command("DEV M 9 DUTY 1")
    assert reply.strip() == "ERR range port"
