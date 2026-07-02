"""
test_otos_calibration_commands.py — 064-001 regression tests for the
ArgSchema query-mutates-state bug in the OL/OA OTOS calibration commands.

Root cause (see clasi/issues/dbg-irqguard-query-disables-guard.md and
sprint 064 ticket 001): parseSchema()'s positional path always fills every
declared ArgDef slot even when the caller omits the token, so a handler
that tested `args.count >= 1` to decide "was the optional value supplied"
was always true — a bare query silently zeroed the calibration scalar
instead of just reporting it.

handleOL/handleOA (OtosCommands.cpp) are NOT #ifndef HOST_BUILD-guarded
(only the NezhaHAL bench-noise branch is), so this is a full host-reachable
regression test via sim_command()/send_command(), unlike DBG IRQGUARD and RF
whose handler bodies are guarded out of the sim build.
"""
import pytest

from firmware import Sim


@pytest.fixture
def sim_otos(sim):
    """A sim with the OTOS odometer model enabled (so OL/OA find a device).

    Without an enabled odometer, otosReady() rejects with 'ERR nodev'.
    """
    sim.send_command("SET sTimeout=60000")
    sim.enable_otos_model()
    sim.set_otos_fusion(True)
    return sim


class TestOLQuerySafe:
    def test_bare_query_after_set_does_not_zero_scalar(self, sim_otos):
        """OL 5 then bare OL: scalar must still read 5, not silently reset to 0."""
        set_reply = sim_otos.send_command("OL 5")
        assert "scalar=5" in set_reply, f"OL 5 did not set scalar: {set_reply!r}"

        query_reply = sim_otos.send_command("OL")
        assert "scalar=5" in query_reply, (
            f"Bare 'OL' must report the existing scalar (5), not mutate it: "
            f"{query_reply!r}"
        )

    def test_bare_query_repeated_is_idempotent(self, sim_otos):
        """Repeated bare queries must not progressively erode the scalar."""
        sim_otos.send_command("OL 7")
        for _ in range(3):
            reply = sim_otos.send_command("OL")
            assert "scalar=7" in reply, (
                f"Repeated bare 'OL' must keep reporting scalar=7: {reply!r}"
            )

    def test_explicit_zero_still_sets_zero(self, sim_otos):
        """An explicit 'OL 0' must still be able to set the scalar to 0
        (suppliedCount distinguishes this from an omitted token; the ranged
        set path itself is unchanged by this fix)."""
        sim_otos.send_command("OL 9")
        reply = sim_otos.send_command("OL 0")
        assert "scalar=0" in reply, f"Explicit 'OL 0' must set scalar to 0: {reply!r}"


class TestOAQuerySafe:
    def test_bare_query_after_set_does_not_zero_scalar(self, sim_otos):
        """OA 3 then bare OA: scalar must still read 3, not silently reset to 0."""
        set_reply = sim_otos.send_command("OA 3")
        assert "scalar=3" in set_reply, f"OA 3 did not set scalar: {set_reply!r}"

        query_reply = sim_otos.send_command("OA")
        assert "scalar=3" in query_reply, (
            f"Bare 'OA' must report the existing scalar (3), not mutate it: "
            f"{query_reply!r}"
        )

    def test_bare_query_repeated_is_idempotent(self, sim_otos):
        """Repeated bare queries must not progressively erode the scalar."""
        sim_otos.send_command("OA -4")
        for _ in range(3):
            reply = sim_otos.send_command("OA")
            assert "scalar=-4" in reply, (
                f"Repeated bare 'OA' must keep reporting scalar=-4: {reply!r}"
            )

    def test_explicit_zero_still_sets_zero(self, sim_otos):
        """An explicit 'OA 0' must still be able to set the scalar to 0."""
        sim_otos.send_command("OA 6")
        reply = sim_otos.send_command("OA 0")
        assert "scalar=0" in reply, f"Explicit 'OA 0' must set scalar to 0: {reply!r}"
