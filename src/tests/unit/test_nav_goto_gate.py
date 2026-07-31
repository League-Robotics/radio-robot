"""src/tests/unit/test_nav_goto_gate.py — ticket 128-002.

``nav/camera_goto.py`` and ``nav/navigator.py`` call ``NezhaProtocol``
methods (``drive()``/``go_to()``) that were deleted from the wire surface
by the v5 cutover (104-002). Before this ticket, calling any of the four
dead entry points crashed with an ``AttributeError`` several call-frames
deep, mid-loop -- a stack trace mid-drive instead of an honest "not
available" answer.

Step 1 (this ticket) makes the dead surface fail LOUDLY at the front
door instead: each of ``go_to_world_camera``, ``Navigator.navigate``,
``Navigator.follow_path``, and ``Navigator.visit_tags`` now raises
``NotImplementedError`` as the first thing it does, naming the
replacement (``pathplan.gotoWorld``/``followPath``, sprint 127) and the
tracking issue
(``nav-goto-stack-is-dead-gate-it-loudly-then-rebuild-or-delete.md``).
The MCP tools that wrap ``Navigator.navigate``/``follow_path``/
``visit_tags`` (``io/robot_mcp.py``) catch that ``NotImplementedError``
and return an honest error dict rather than letting it propagate as an
unhandled exception to the LLM operator.

Step 2 (rebuild-or-delete `nav/camera_goto.py`/`navigator.py` outright)
is explicitly out of scope -- see the ticket's "Worktree note" /
"Scope note". This test file covers ONLY the loud-gate behavior.

Covers:
  1. ``go_to_world_camera`` raises ``NotImplementedError`` immediately,
     before ever calling ``read_pose`` or ``proto`` -- i.e. it fails at
     the front door, not mid-loop.
  2. ``Navigator.navigate`` raises ``NotImplementedError`` immediately.
  3. ``Navigator.follow_path`` raises ``NotImplementedError`` immediately.
  4. ``Navigator.visit_tags`` raises ``NotImplementedError`` immediately.
  5. Each message names the replacement (``pathplan.gotoWorld`` /
     ``pathplan.followPath``) and the tracking issue filename.
  6. The MCP tools ``navigate_to``, ``follow_path``, and ``visit_tags``
     catch the ``NotImplementedError`` and return an honest
     ``{"error": "not_available", ...}`` result -- no unhandled
     exception reaches the caller.
"""
from __future__ import annotations

import asyncio

import pytest

from robot_radio.nav.camera_goto import go_to_world_camera
from robot_radio.nav.navigator import Navigator
import robot_radio.io.robot_mcp as robot_mcp

_TRACKING_ISSUE = (
    "nav-goto-stack-is-dead-gate-it-loudly-then-rebuild-or-delete.md"
)


class _BoomIfCalled:
    """Any attribute access/call is a test failure -- proves the gate
    fires before the dead body ever touches the wire."""

    def __getattr__(self, name):  # noqa: ANN001
        raise AssertionError(
            f"go_to_world_camera touched proto.{name} -- the "
            "NotImplementedError guard did not fire at the front door"
        )


def _boom_read_pose(*args, **kwargs):
    raise AssertionError(
        "go_to_world_camera called read_pose() -- the NotImplementedError "
        "guard did not fire at the front door"
    )


# -- Step 1: front-door NotImplementedError guards --------------------------


def test_go_to_world_camera_raises_not_implemented_before_touching_proto():
    with pytest.raises(NotImplementedError) as excinfo:
        go_to_world_camera(
            _BoomIfCalled(), _boom_read_pose,
            target_x=30.0, target_y=10.0,
            cruise=150, turn_speed=80, gate=25.0,
            arrive_cm=3.0, max_secs=10.0,
        )
    msg = str(excinfo.value)
    assert "pathplan.gotoWorld" in msg
    assert _TRACKING_ISSUE in msg


def _make_navigator() -> Navigator:
    # Navigator.__init__ only loads calibration JSON (falls back to
    # hard-coded defaults when the file is missing) -- no camera/robot
    # I/O happens at construction time, so a bare object stand-in for
    # ``robot`` is fine; navigate/follow_path/visit_tags all raise before
    # ever touching it.
    return Navigator(robot=_BoomIfCalled())


def test_navigator_navigate_raises_not_implemented():
    nav = _make_navigator()
    with pytest.raises(NotImplementedError) as excinfo:
        nav.navigate((30.0, 10.0))
    msg = str(excinfo.value)
    assert "pathplan.gotoWorld" in msg
    assert _TRACKING_ISSUE in msg


def test_navigator_follow_path_raises_not_implemented():
    nav = _make_navigator()
    with pytest.raises(NotImplementedError) as excinfo:
        nav.follow_path([(10.0, 0.0), (20.0, 0.0)])
    msg = str(excinfo.value)
    assert "pathplan.gotoWorld/followPath" in msg
    assert _TRACKING_ISSUE in msg


def test_navigator_visit_tags_raises_not_implemented():
    nav = _make_navigator()
    with pytest.raises(NotImplementedError) as excinfo:
        nav.visit_tags([1, 2, 3])
    msg = str(excinfo.value)
    assert "pathplan.gotoWorld" in msg
    assert _TRACKING_ISSUE in msg


# -- Step 1: MCP tools catch the guard and report honestly -----------------


class _FakeNavigator:
    """Stands in for the real Navigator: navigate/follow_path/visit_tags
    raise NotImplementedError exactly like the real gated methods."""

    def navigate(self, *args, **kwargs):  # noqa: ANN001, ANN002
        raise NotImplementedError(
            "dead since the v5 wire cutover: NezhaProtocol.drive()/go_to() "
            "were deleted (104-002). The replacement is pathplan.gotoWorld/"
            f"followPath (sprint 127). Tracked: clasi/issues/{_TRACKING_ISSUE}"
        )

    def follow_path(self, *args, **kwargs):  # noqa: ANN001, ANN002
        raise NotImplementedError(
            "dead since the v5 wire cutover: NezhaProtocol.drive()/go_to() "
            "were deleted (104-002). The replacement is pathplan.gotoWorld/"
            f"followPath (sprint 127). Tracked: clasi/issues/{_TRACKING_ISSUE}"
        )

    def visit_tags(self, *args, **kwargs):  # noqa: ANN001, ANN002
        raise NotImplementedError(
            "dead since the v5 wire cutover: NezhaProtocol.drive()/go_to() "
            "were deleted (104-002). The replacement is pathplan.gotoWorld/"
            f"followPath (sprint 127). Tracked: clasi/issues/{_TRACKING_ISSUE}"
        )


@pytest.fixture
def fake_navigator(monkeypatch):
    nav = _FakeNavigator()
    monkeypatch.setattr(robot_mcp, "_navigator", nav)
    return nav


def _call_tool_sync(name: str, arguments: dict):
    return asyncio.run(robot_mcp.call_tool(name, arguments))


def test_mcp_navigate_to_reports_not_available(fake_navigator):
    result = _call_tool_sync("navigate_to", {"x": 30, "y": 10})
    text = result[0].text
    assert "not_available" in text
    assert "pathplan.gotoWorld" in text
    assert "Traceback" not in text


def test_mcp_follow_path_reports_not_available(fake_navigator):
    result = _call_tool_sync(
        "follow_path", {"path": [[10, 0], [20, 0]]}
    )
    text = result[0].text
    assert "not_available" in text
    assert "pathplan.gotoWorld/followPath" in text
    assert "Traceback" not in text


def test_mcp_visit_tags_reports_not_available(fake_navigator):
    result = _call_tool_sync("visit_tags", {"tags": [1, 2]})
    text = result[0].text
    assert "not_available" in text
    assert "pathplan.gotoWorld" in text
    assert "Traceback" not in text
