"""Off-hardware acceptance proof for ticket 084-002 (SUC-001): registers
``S``/``T``/``D``/``STOP`` as top-level wire verbs staging a
``msg::PlannerCommand`` into ticket 001's ``Subsystems::Planner``, wired into
the shared dev-loop (``source/dev_loop.cpp``'s new per-pass step).

Drives ``libfirmware_host`` through the full wire dispatch (``Sim.command()``)
-- unlike ``test_planner.py``'s standalone harness (which tests
``Subsystems::Planner`` in isolation against hand-built fixtures, landing no
wire verb), this file is the first to exercise S/T/D/STOP as an end user
would: over the wire, through ``CommandProcessor``, ``source/commands/
motion_commands.cpp``, ``MotionLoopState``'s outbox, and ``dev_loop.cpp``'s
drain step, ending in the SAME simulated plant ``test_plant_correctness.py``
et al. already exercise.

``EVT done <verb> reason=<token>``/``EVT safety_stop reason=watchdog`` are
loop-originated (never triggered by the inbound command itself -- they may
fire many passes later) -- drained via ``Sim.get_async_evts()``, exactly like
``test_watchdog_policy.py``'s own ``EVT dev_watchdog`` checks.
"""


def test_d_moves_true_pose_and_emits_done_dist(sim):
    reply = sim.command("D 200 200 500")
    assert reply.strip() == "OK drive l=200 r=200 mm=500"

    sim.tick_for(3000)

    x, y, h = sim.true_pose()
    # SMOOTH stop style (Planner's default) ramps the last ~25 mm off under
    # a_decel after the 500 mm distance stop fires, so this lands a little
    # past 500 mm, not exactly at it -- "~500 mm" per the ticket's own
    # acceptance wording, not a bit-exact target.
    assert abs(x - 500.0) < 100.0, f"expected true x near 500 mm, got {x}"
    assert abs(y) < 5.0
    assert abs(h) < 0.05

    evts = sim.get_async_evts()
    assert "EVT done D reason=dist" in evts


def test_d_straight_drive_moves_both_wheels_the_same_sign(sim):
    """088-008: test_plant_correctness.py proves the same-sign/opposite-sign
    wheel relationship at the raw `DEV M DUTY` layer (straight vs in-place
    turn); this closes the same gap at the actual wire verb a bench
    operator/host issues -- D's equal l/r targets must drive BOTH wheels'
    true (unslipped, ground-truth) travel in the SAME direction, not just
    leave the FUSED pose (true_pose()) looking straight."""
    reply = sim.command("D 200 200 500")
    assert reply.strip() == "OK drive l=200 r=200 mm=500"

    sim.tick_for(1000)
    enc_l, enc_r = sim.true_wheel_travel()
    assert enc_l > 20.0, f"expected meaningful forward left-wheel travel, got {enc_l}"
    assert enc_r > 20.0, f"expected meaningful forward right-wheel travel, got {enc_r}"
    assert (enc_l > 0) == (enc_r > 0), (
        f"expected both wheels traveling the SAME sign for a straight D, got enc=({enc_l}, {enc_r})"
    )


def test_s_wheel_speeds_converge_to_and_hold_the_commanded_value(sim):
    """088-008: test_s_streams_open_ended_with_no_natural_completion below
    only proves S keeps the robot moving forward (x > 0) and never completes
    on its own -- it never checks that the actual per-wheel speed converges
    to and HOLDS the commanded l/r value, which is S's own defining behavior
    (docs/protocol-v2.md section 10: streamed, open-loop VELOCITY drive).
    Re-feeds S repeatedly (same re-feed pattern as that test, so the
    sTimeout watchdog never intervenes) and asserts sim.vel() (the two
    default plant-bound SimMotors' own filtered velocity -- port 1=L,
    port 2=R) settles into, and then still holds, test_velocity_pid_
    response.py's own already-measured VEL-120 bench-envelope bracket
    (90-170 mm/s) -- directly comparable here because S's straight body
    twist (v_x=120, omega=0) splits to equal l/r wheel targets driven by the
    SAME Hal::MotorVelocityPid law and default gains that test exercises
    directly via DEV M 1 VEL 120."""
    reply = sim.command("S 120 120")
    assert reply.strip() == "OK drive l=120 r=120"

    for _ in range(5):
        sim.tick_for(200)
        sim.command("S 120 120")   # re-feed -- keeps the sTimeout watchdog from firing

    vel_l, vel_r = sim.vel()
    assert 90.0 <= vel_l <= 170.0, f"expected left wheel settled near 120 mm/s, got {vel_l}"
    assert 90.0 <= vel_r <= 170.0, f"expected right wheel settled near 120 mm/s, got {vel_r}"

    # Holds across a further re-feed cycle -- not decaying or diverging.
    sim.tick_for(200)
    sim.command("S 120 120")
    vel_l2, vel_r2 = sim.vel()
    assert 90.0 <= vel_l2 <= 170.0, f"expected left wheel to still hold near 120 mm/s, got {vel_l2}"
    assert 90.0 <= vel_r2 <= 170.0, f"expected right wheel to still hold near 120 mm/s, got {vel_r2}"


def test_d_negative_wheels_drives_backward(sim):
    reply = sim.command("D -150 -150 300")
    assert reply.strip() == "OK drive l=-150 r=-150 mm=300"

    sim.tick_for(3000)

    x, _y, _h = sim.true_pose()
    assert x < -200.0, f"expected true x well negative (backward drive), got {x}"

    evts = sim.get_async_evts()
    assert "EVT done D reason=dist" in evts


def test_t_drives_for_duration_and_emits_done_time(sim):
    reply = sim.command("T 150 150 1000")
    assert reply.strip() == "OK drive l=150 r=150 ms=1000"

    # Well before the 1 s duration elapses: no completion yet.
    sim.tick_for(200)
    assert "EVT done T" not in sim.get_async_evts()

    sim.tick_for(2000)
    evts = sim.get_async_evts()
    assert "EVT done T reason=time" in evts


def test_s_streams_open_ended_with_no_natural_completion(sim):
    reply = sim.command("S 120 120")
    assert reply.strip() == "OK drive l=120 r=120"

    # S is open-ended: ticking well past what a bounded T/D would need
    # produces no "done" event on its own, as long as S keeps being refed.
    for _ in range(5):
        sim.tick_for(200)
        sim.command("S 120 120")

    evts = sim.get_async_evts()
    assert "EVT done" not in evts
    assert "safety_stop" not in evts

    x, _y, _h = sim.true_pose()
    assert x > 0.0, "S should have been driving the robot forward"


def test_stop_halts_immediately_with_no_evt(sim):
    sim.command("D 200 200 500")
    sim.tick_for(120)   # let the ramp get underway, well short of 500 mm

    reply = sim.command("STOP")
    assert reply.strip() == "OK stop"

    # No EVT at all -- STOP is a silent, immediate halt (docs/protocol-v2.md
    # section 10: "Stops motors immediately... No EVT is emitted").
    assert sim.get_async_evts() == ""

    x_after_stop, _y, _h = sim.true_pose()
    sim.tick_for(1000)
    x_later, _y2, _h2 = sim.true_pose()

    # The halted pose must not keep drifting once STOP has taken effect --
    # allow one pass of latency for the ramp to actually reach zero.
    assert abs(x_later - x_after_stop) < 5.0
    assert sim.get_async_evts() == ""


def test_stop_clause_t_fires_before_built_in_stop(sim):
    reply = sim.command("T 150 150 5000 stop=t:400")
    assert reply.strip() == "OK drive l=150 r=150 ms=5000"

    sim.tick_for(2000)
    evts = sim.get_async_evts()
    assert "EVT done T reason=time" in evts


def test_stop_clause_d_fires_before_built_in_stop(sim):
    reply = sim.command("T 150 150 5000 stop=d:100")
    assert reply.strip() == "OK drive l=150 r=150 ms=5000"

    sim.tick_for(2000)
    evts = sim.get_async_evts()
    assert "EVT done T reason=dist" in evts


def test_stop_clause_rot_fires_on_arced_turn(sim):
    reply = sim.command("T -80 80 5000 stop=rot:40")
    assert reply.strip() == "OK drive l=-80 r=80 ms=5000"

    sim.tick_for(3000)
    evts = sim.get_async_evts()
    assert "EVT done T reason=rot" in evts


def test_stop_clause_heading_fires_on_turn(sim):
    reply = sim.command("T -80 80 5000 stop=heading:9000:300")
    assert reply.strip() == "OK drive l=-80 r=80 ms=5000"

    sim.tick_for(4000)
    evts = sim.get_async_evts()
    assert "EVT done T reason=heading" in evts


def test_stop_clause_sensor_color_line_rejected_with_badarg(sim):
    for clause in (
        "stop=sensor:line0:ge:512",
        "stop=color:120:0.5:0.4:0.1",
        "stop=line:ge:512",
        # Back-compat bare "sensor=" alias (docs/protocol-v2.md section 10) --
        # still a SENSOR-kind clause, still unsupported.
        "sensor=line0:ge:512",
    ):
        reply = sim.command(f"D 100 100 200 {clause}")
        assert reply.strip().startswith("ERR badarg"), (
            f"expected ERR badarg for clause {clause!r}, got {reply!r}"
        )

    # None of the rejected commands should have staged a goal -- the robot
    # never moved.
    x, y, h = sim.true_pose()
    assert (x, y, h) == (0.0, 0.0, 0.0)


def test_stop_clause_malformed_rejected_with_badarg_not_silently_ignored(sim):
    reply = sim.command("D 100 100 200 stop=xyz:1")
    assert reply.strip().startswith("ERR badarg")

    x, y, h = sim.true_pose()
    assert (x, y, h) == (0.0, 0.0, 0.0)


def test_s_range_validation(sim):
    assert sim.command("S 1500 0").strip() == "ERR range l"
    assert sim.command("S 0 -1500").strip() == "ERR range r"


def test_t_range_validation(sim):
    assert sim.command("T 100 100 0").strip() == "ERR range ms"
    assert sim.command("T 100 100 40000").strip() == "ERR range ms"
    assert sim.command("T 1500 100 1000").strip() == "ERR range l"


def test_d_range_validation(sim):
    assert sim.command("D 100 100 0").strip() == "ERR range mm"
    assert sim.command("D 100 100 20000").strip() == "ERR range mm"
    assert sim.command("D 100 -1500 200").strip() == "ERR range r"


def test_s_timeout_watchdog_fires_independently_of_dev_wd(sim):
    """sTimeout (084-002, default 500 ms) must fire even though the `sim`
    fixture already widened DEV WD's SerialSilenceWatchdog to 60 s -- proof
    the two watchdogs are genuinely independent state, not the same timer
    under two names (architecture-update.md (084) Risk 2)."""
    reply = sim.command("S 100 100")
    assert reply.strip() == "OK drive l=100 r=100"

    # Still well inside the 500 ms window -- no fire yet.
    sim.tick_for(200)
    assert "safety_stop" not in sim.get_async_evts()

    # Push past the window with no further S arriving.
    sim.tick_for(500)
    evts = sim.get_async_evts()
    assert "EVT safety_stop reason=watchdog" in evts
    # DEV WD's own watchdog (60 s window) must not have also fired.
    assert "dev_watchdog" not in evts

    # The streaming drive was actually halted.
    x_at_fire, _y, _h = sim.true_pose()
    sim.tick_for(500)
    x_later, _y2, _h2 = sim.true_pose()
    assert abs(x_later - x_at_fire) < 5.0


def test_s_timeout_does_not_fire_while_s_keeps_arriving(sim):
    reply = sim.command("S 100 100")
    assert reply.strip() == "OK drive l=100 r=100"

    for _ in range(6):
        sim.tick_for(200)   # < the 500 ms sTimeout window
        sim.command("S 100 100")

    assert "safety_stop" not in sim.get_async_evts()


def test_set_stimeout_changes_the_streaming_drive_watchdog_firing_time(sim):
    """088-008: the two tests above prove sTimeout's BOOT DEFAULT (500 ms)
    behavior; test_config_registry.py's own parametrized round-trip proves
    `SET sTimeout=<n>` -> `GET sTimeout` echoes back the new value -- but
    nothing proves the SET actually RETUNES the window S's own streaming-
    drive watchdog fires against (source/runtime/main_loop.cpp posts
    bb.streamWatchdogWindowIn -> MainLoop::tick() drains it into
    streamWatchdog_.setWindow() every pass, unconditionally). Narrows it to
    150 ms, then confirms S's watchdog fires well before the 500 ms default
    would have allowed."""
    reply = sim.command("SET sTimeout=150")
    assert reply.strip() == "OK set sTimeout=150"

    reply = sim.command("S 100 100")
    assert reply.strip() == "OK drive l=100 r=100"

    # Comfortably inside the NEW 150 ms window -- no fire yet.
    sim.tick_for(96)
    assert "safety_stop" not in sim.get_async_evts()

    # Past the new (narrowed) 150 ms window, well short of the 500 ms boot
    # default the two tests above exercise -- must have already fired.
    sim.tick_for(120)
    evts = sim.get_async_evts()
    assert "EVT safety_stop reason=watchdog" in evts, (
        f"expected the SET-narrowed 150 ms sTimeout to have fired by ~216 ms, got evts={evts!r}"
    )
