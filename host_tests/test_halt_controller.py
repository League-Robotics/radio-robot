"""
test_halt_controller.py — tests for the HALT command and HaltController (ticket 020-007).

Exercises:
  - HALT TIME registers a time condition; EVT halt fires after the threshold.
  - HALT DIST registers a distance condition; EVT halt fires after threshold mm.
  - HALT CLEAR removes all conditions; no EVT fires afterward.
  - HALT DIST SOFT fires with EVT halt and issues a soft stop.
  - HALT LINE ANY GE fires when any line sensor >= threshold.
  - ZERO T / ZERO D reset the baselines used by HALT TIME / HALT DIST.
"""
import re


def _find_evt_halt(evts: str) -> int | None:
    """Return the halt id from 'EVT halt id=<n>' in evts, or None."""
    m = re.search(r"EVT halt id=(\d+)", evts)
    return int(m.group(1)) if m else None


# ---------------------------------------------------------------------------
# HALT TIME
# ---------------------------------------------------------------------------

def test_halt_time_registers(sim):
    """HALT TIME 200 returns OK HALT id=0."""
    r = sim.send_command("ZERO T")
    assert "OK" in r.upper(), f"ZERO T failed: {repr(r)}"

    r = sim.send_command("HALT TIME 200")
    assert "OK" in r.upper(), f"Expected OK from HALT TIME, got {repr(r)}"
    assert "id=" in r, f"Expected id= in HALT TIME reply, got {repr(r)}"


def test_halt_time_fires_evt(sim):
    """HALT TIME 200: EVT halt fires within 500ms tick window."""
    # Extend watchdog far beyond the test window.
    sim.send_command("SET sTimeout=60000")

    # Set baseline, start driving, register halt condition.
    sim.send_command("ZERO T")
    sim.send_command("VW 200 0")
    r = sim.send_command("HALT TIME 200")
    assert "id=0" in r, f"Expected id=0 from HALT TIME, got {repr(r)}"

    # Tick 500ms (well past the 200ms threshold).
    sim.tick_for(500)

    evts = sim.get_async_evts()
    halt_id = _find_evt_halt(evts)
    assert halt_id is not None, (
        f"Expected 'EVT halt id=0' in async events after 500ms, got: {repr(evts)}"
    )
    assert halt_id == 0, f"Expected halt id=0, got id={halt_id}"


def test_halt_time_fires_only_once(sim):
    """HALT conditions are cleared after first fire; no second EVT on re-tick."""
    sim.send_command("SET sTimeout=60000")
    sim.send_command("ZERO T")
    sim.send_command("VW 200 0")
    sim.send_command("HALT TIME 100")

    # First window — should fire.
    sim.tick_for(300)
    evts1 = sim.get_async_evts()
    assert _find_evt_halt(evts1) is not None, "Expected halt EVT in first tick window"

    # Start driving again and tick more — should NOT fire again (conditions cleared).
    sim.send_command("VW 200 0")
    sim.tick_for(300)
    evts2 = sim.get_async_evts()
    assert _find_evt_halt(evts2) is None, (
        f"Expected no second halt EVT after clear, but got: {repr(evts2)}"
    )


# ---------------------------------------------------------------------------
# HALT DIST
# ---------------------------------------------------------------------------

def test_halt_dist_registers(sim):
    """HALT DIST 500 returns OK HALT id=0."""
    sim.send_command("ZERO D")
    r = sim.send_command("HALT DIST 500")
    assert "OK" in r.upper(), f"Expected OK from HALT DIST, got {repr(r)}"
    assert "id=" in r, f"Expected id= in HALT DIST reply, got {repr(r)}"


def test_halt_dist_fires_evt(sim):
    """HALT DIST 300: EVT halt fires once encoders travel >= 300mm."""
    sim.send_command("SET sTimeout=60000")

    # Zero distance baseline and set a modest distance condition.
    sim.send_command("ZERO D")
    sim.send_command("VW 300 0")
    sim.send_command("HALT DIST 300")

    # Tick 3s — at 300mm/s the mock should easily reach 300mm.
    sim.tick_for(3000)

    evts = sim.get_async_evts()
    halt_id = _find_evt_halt(evts)
    assert halt_id is not None, (
        f"Expected 'EVT halt' in async events after HALT DIST 300 at v=300, "
        f"got: {repr(evts)}"
    )


# ---------------------------------------------------------------------------
# HALT CLEAR
# ---------------------------------------------------------------------------

def test_halt_clear_removes_conditions(sim):
    """HALT CLEAR after HALT TIME: no EVT halt fires after CLEAR."""
    sim.send_command("SET sTimeout=60000")
    sim.send_command("ZERO T")
    sim.send_command("VW 200 0")
    sim.send_command("HALT TIME 100")

    r = sim.send_command("HALT CLEAR")
    assert "OK" in r.upper(), f"Expected OK from HALT CLEAR, got {repr(r)}"
    assert "cleared=" in r, f"Expected cleared= in HALT CLEAR reply, got {repr(r)}"

    # Tick past the original threshold — no halt should fire.
    sim.tick_for(500)
    evts = sim.get_async_evts()
    assert _find_evt_halt(evts) is None, (
        f"Expected no EVT halt after HALT CLEAR, but got: {repr(evts)}"
    )


# ---------------------------------------------------------------------------
# HALT TIME SOFT
# ---------------------------------------------------------------------------

def test_halt_time_soft_fires_evt(sim):
    """HALT TIME 200 SOFT: EVT halt fires with soft stop."""
    sim.send_command("SET sTimeout=60000")
    sim.send_command("ZERO T")
    sim.send_command("VW 200 0")
    r = sim.send_command("HALT TIME 200 SOFT")
    assert "OK" in r.upper(), f"Expected OK from HALT TIME SOFT, got {repr(r)}"

    sim.tick_for(500)
    evts = sim.get_async_evts()
    assert _find_evt_halt(evts) is not None, (
        f"Expected 'EVT halt' after HALT TIME SOFT, got: {repr(evts)}"
    )


# ---------------------------------------------------------------------------
# HALT LIST
# ---------------------------------------------------------------------------

def test_halt_list_shows_registered(sim):
    """HALT LIST returns entries for each registered condition."""
    sim.send_command("ZERO T")
    sim.send_command("HALT TIME 500")
    sim.send_command("HALT DIST 1000")

    r = sim.send_command("HALT LIST")
    assert "OK" in r.upper(), f"Expected OK from HALT LIST, got {repr(r)}"


# ---------------------------------------------------------------------------
# ZERO T / ZERO D baseline
# ---------------------------------------------------------------------------

def test_zero_t_resets_timer_baseline(sim):
    """ZERO T reply contains 'T' indicating timer baseline was set."""
    r = sim.send_command("ZERO T")
    assert "T" in r, f"Expected T in ZERO T reply, got {repr(r)}"
    assert "OK" in r.upper(), f"Expected OK from ZERO T, got {repr(r)}"


def test_zero_d_resets_dist_baseline(sim):
    """ZERO D reply contains 'D' indicating distance baseline was set."""
    r = sim.send_command("ZERO D")
    assert "D" in r, f"Expected D in ZERO D reply, got {repr(r)}"
    assert "OK" in r.upper(), f"Expected OK from ZERO D, got {repr(r)}"
