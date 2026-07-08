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


def test_hello_reemits_the_device_identity_banner(sim):
    """088-005: HELLO (re-added under v1->v2) re-emits the same
    `DEVICE:NEZHA2:robot:<name>:<serial>` banner main.cpp sends once at
    boot on both channels (formatDeviceAnnouncement(), system_commands.cpp)
    -- a bare reply, no OK/ERR wrapper, no #id echo. <name>/<serial> are the
    HOST_BUILD placeholder identity handleId's ID reply already uses.
    The boot-time both-channels announcement itself is not exercised here:
    the sim harness (sim_api.cpp) constructs CommandRouter/CommandProcessor
    directly, never through main(), so main.cpp's boot call is bench-verified
    instead (ticket 088-009)."""
    reply = sim.command("HELLO").strip()
    assert reply == "DEVICE:NEZHA2:robot:HOST-SIM:0"


def test_help_enumerates_the_live_registered_command_table(sim):
    """088-003: HELP used to reply with a hardcoded "PING VER HELP ECHO ID"
    regardless of what was actually registered. It must now enumerate the
    live table CommandRouter::buildTable() assembles -- every family, not
    just the five liveness verbs."""
    reply = sim.command("HELP").strip()
    assert reply.startswith("OK help ")
    body = reply[len("OK help "):]
    tokens = body.split()

    # Liveness family (system_commands.cpp) -- still registered, still
    # present. HELLO (088-005) joined this family.
    for verb in ("PING", "VER", "HELP", "ECHO", "ID", "HELLO"):
        assert verb in tokens

    # Motion family (motion_commands.cpp) -- this is the whole point of the
    # fix: these never appeared under the old hardcoded string.
    for verb in ("S", "T", "D", "R", "TURN", "RT", "G", "STOP"):
        assert verb in tokens

    # Config family (config_commands.cpp).
    assert "SET" in tokens
    assert "GET" in tokens

    # Telemetry family (telemetry_commands.cpp).
    assert "STREAM" in tokens
    assert "SNAP" in tokens

    # Pose family (pose_commands.cpp).
    assert "SI" in tokens
    assert "ZERO" in tokens

    # OTOS family (otos_commands.cpp).
    for verb in ("OI", "OZ", "OR", "OP", "OV", "OL", "OA"):
        assert verb in tokens

    # DEV family (dev_commands.cpp) -- multi-token prefixes ("DEV M", "DEV
    # DT", ...) are joined as their literal registered prefix string, so
    # check them as substrings rather than single tokens.
    for prefix in ("DEV M", "DEV DT", "DEV STATE", "DEV STOP", "DEV WD"):
        assert prefix in body


def test_help_reply_is_not_truncated(sim):
    """Regression check: HELP's reply buffers (system_commands.cpp) must be
    sized to hold the full live table without snprintf silently truncating
    the tail. otos_commands.cpp's OA is the last verb CommandRouter::
    buildTable() registers, so it is the canary for a truncated reply."""
    reply = sim.command("HELP").strip()
    assert reply.endswith("OA")


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


def test_dev_stop_neutralizes_a_driving_motor(sim):
    """088-008: the smoke test above only checks the ack text -- DEV STOP's
    actual job (dev_commands.cpp's handleDevStop: a broadcast neutral posted
    to bb.hardwareBroadcastIn, staged onto every motor's setter at THIS
    pass's own hardware_.apply() call) is never proven to actually reach the
    plant. Drives port 1 directly via DEV M VEL (bypassing Drivetrain
    entirely, mirroring test_velocity_pid_response.py), confirms it is
    genuinely spinning, then DEV STOP and confirms the plant's raw commanded
    actuator value (sim.pwm(), ground truth) returns to zero. Unlike the
    watchdog's own estop() bypass (test_watchdog_policy.py's
    own same-pass proof), DEV STOP's broadcast is only STAGED on the pass it
    is issued (apply(), not tick()) -- one further real tick is needed for
    the motor's own tick() to execute the now-NEUTRAL mode and actually
    write the zero duty (armoredWrite()'s own "stop is always immediate,
    unclamped" contract -- source/hal/capability/motor.h)."""
    sim.command("DEV M 1 VEL 150")
    sim.tick_for(600)
    assert sim.pwm()[0] != 0.0, "expected the motor to be driving before DEV STOP"

    reply = sim.command("DEV STOP")
    assert reply.strip() == "OK DEV STOP"

    sim.tick_for(24)   # one real tick -- lets the staged NEUTRAL mode execute
    assert sim.pwm() == (0.0, 0.0), (
        "expected DEV STOP to neutralize the plant's actuator output"
    )


def test_unknown_verb_replies_err_unknown(sim):
    reply = sim.command("BOGUS")
    assert reply.strip() == "ERR unknown"


def test_dev_m_out_of_range_port_replies_err_range(sim):
    reply = sim.command("DEV M 9 DUTY 1")
    assert reply.strip() == "ERR range port"
