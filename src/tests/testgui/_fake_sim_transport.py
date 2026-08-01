"""src/tests/testgui/_fake_sim_transport.py -- shared ``FakeConnectedSimTransport``
test double (128-003 Part 1 baseline fix).

``test_sim_errors_panel.py`` and ``test_sim_errors_from_cal_button.py`` each
defined a BYTE-FOR-BYTE duplicate of a fake ``SimTransport`` stand-in (a fake
whose class name is literally ``"SimTransport"`` so ``operations.
is_sim_transport()``'s duck-type check -- ``type(t).__name__`` -- matches, and
that never touches real hardware or the ctypes sim). That duplication was
itself the drift mechanism: the real ``SimTransport`` grew a
``set_speed_factor()`` method (``transport.py``, called unconditionally by
``__main__.py``'s ``_apply_sim_speed()`` on every Sim connect -- see
``_on_connect()``'s own call site), and neither copy of this fake was updated
to match, so BOTH files' connect-flow tests failed with ``AttributeError:
'SimTransport' object has no attribute 'set_speed_factor'`` the moment
``_on_connect()`` reached that line -- a defect a single shared definition
would have caught (and now fixed) in one place instead of two.

Consolidated here as ONE definition, imported by both test files (and any
future one that needs a fake connectable "Sim" transport for the GUI's real
connect-time flow).
"""
from __future__ import annotations

from robot_radio.testgui import transport as transport_module


def make_fake_connected_sim_transport_class(applied: list) -> type:
    """Build a fresh ``FakeConnectedSimTransport`` class whose
    ``apply_error_profile()`` appends every call's profile dict to
    *applied* (a list owned by the calling test, so each test gets its own
    isolated call log without needing a shared/global one).

    A fresh class (not a fresh instance of one shared class) is returned so
    ``FakeConnectedSimTransport.__name__``/``__qualname__`` can be forced to
    ``"SimTransport"`` per call site without two different tests' classes
    colliding under that same forced name.
    """

    class FakeConnectedSimTransport(transport_module.Transport):
        """Fake whose class name is 'SimTransport' (duck-typed by
        operations.is_sim_transport) and that never touches real hardware
        or the ctypes sim.
        """

        def __init__(self) -> None:
            super().__init__()
            self._connected = False

        def connect(self) -> None:
            self._connected = True

        def disconnect(self) -> None:
            self._connected = False

        def send(self, line: str) -> None:
            pass

        def command(self, line: str, read_timeout: int = 200) -> str:  # [ms]
            return "OK"

        def halt(self) -> None:
            """128-003: Transport.halt() is now abstract -- this fake never
            drives real motion, so a no-op satisfies the ABC without
            claiming any wire behavior this fake doesn't actually have."""

        def apply_error_profile(self, profile: dict) -> None:
            applied.append(profile)

        def firmware_version(self) -> "str | None":
            # 111-002: _on_connect() (__main__.py) unconditionally calls
            # transport.firmware_version() on every isinstance(transport,
            # SimTransport) connect (commit 67792cab, "add firmware
            # version retrieval to SimTransport") to show the loaded sim
            # lib's own version -- the real SimTransport.firmware_version()
            # this fake stands in for. An AttributeError here (this
            # method absent) aborted _on_connect() before it reached
            # `_state["transport"] = transport`, so a caller's Apply/
            # From-Cal button handler found no connected transport and
            # never called apply_error_profile() at all -- the actual
            # prior failure mode, not a missing button wiring.
            return "test-fake"

        def set_speed_factor(self, factor: int) -> None:
            # 128-003 baseline fix: __main__.py's _apply_sim_speed() calls
            # this unconditionally on every Sim connect
            # (_on_connect() -> _apply_sim_speed(), regardless of whether
            # the operator ever touched sim_speed_combo) -- missing from
            # this fake was this ticket's root cause #1 (see this module's
            # own docstring for the full "duplication is the drift
            # mechanism" story).
            pass

    FakeConnectedSimTransport.__name__ = "SimTransport"
    FakeConnectedSimTransport.__qualname__ = "SimTransport"
    return FakeConnectedSimTransport
