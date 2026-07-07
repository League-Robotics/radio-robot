"""088-007: full command smoke-test suite and completeness meta-test.

Stakeholder request (``clasi/issues/full-command-smoke-test-suite.md``): "a
full command test that goes through every command and runs a test on that
command... one function for every single command that we have registered in
the system. That function tests: you can send it through the serial port;
you can send it through the radio; it produces an effect. It's a smoke
test."

Smoke-level only -- not exhaustive per-command behavior (that already lives
in ``test_protocol_roundtrips.py``, ``test_motion_commands*.py``,
``test_pose_commands.py``, ``test_otos_commands.py``,
``test_config_registry.py``, ``test_tlm_stream_snap.py``, and
``test_watchdog_policy.py``). Each function here only proves three things
per registered verb: it can be sent on the SERIAL channel, it can be sent
on the RADIO channel (088-006's ``Sim.command_on()``), and each send
produces a well-formed reply -- "registered, dispatches, replies," not a
correctness check.

**Source of truth.** ``_SMOKE_LINES`` maps every registered verb (the same
granularity ``HELP`` reports -- one entry per ``DEV`` subcommand, e.g.
``DEV M``/``DEV DT``/``DEV STATE``/``DEV STOP``/``DEV WD``, not one
monolithic ``DEV`` entry) to a minimal-valid-args wire line. Every
``test_smoke_*`` function below calls ``_smoke(sim, "<VERB>")``, which looks
the line up in ``_SMOKE_LINES`` -- so the dict, not each function body, is
the single place a verb's invocation is spelled out.
``test_every_registered_verb_has_a_smoke_test`` (the completeness
meta-test) parses the LIVE table straight out of ``sim.command("HELP")``
(088-003 made ``HELP`` enumerate the actual registered table, not a
hardcoded string) and diffs it against ``_SMOKE_LINES``'s own keys in both
directions -- so a command family added to the firmware without a matching
entry here fails the suite, and a stale entry for a since-removed verb
fails it too.

**"Well-formed reply."** Most verbs reply ``OK <verb> ...``; a few have
their own documented non-OK/non-ERR reply taxonomy (``docs/protocol-v2.md``
section 3: "response tag is not always OK/ERR") -- ``ID`` ("ID ..."),
``HELLO``/the boot banner ("DEVICE:..."), ``SNAP`` ("TLM ..."), and ``GET``
("CFG ..."). ``_BARE_REPLY_PREFIX`` documents those four exceptions;
everything else must start with ``OK `` or a *defined* ``ERR <code>``
(``unsupported``, ``nodev``, ``badarg``, ``badkey``, ``range``, ``badval``,
``unknown`` -- ``docs/protocol-v2.md`` section 4's error-code table) to
count as well-formed. In this sim build every verb below actually replies
``OK``/its own bare tag (``Subsystems::SimHardware`` always has a live
odometer, so the OTOS family never hits the real hardware's ``ERR nodev``
path -- see ``test_otos_commands.py``'s own module docstring); the ERR
branch exists so this suite stays correct if that ever changes, not
because any of these particular invocations currently produces one.
"""
from __future__ import annotations

from firmware import CHANNEL_RADIO, CHANNEL_SERIAL

# ---------------------------------------------------------------------------
# _SMOKE_LINES -- canonical verb -> minimal-valid-args wire line. One entry
# per unit HELP's live table reports (32 total: 6 liveness + 5 DEV
# subcommands + 2 telemetry + 8 motion + 2 config + 2 pose + 7 OTOS, per
# command_router.cpp's buildTable() family order).
# ---------------------------------------------------------------------------
_SMOKE_LINES: dict[str, str] = {
    # Liveness (system_commands.cpp).
    "PING": "PING",
    "VER": "VER",
    "HELP": "HELP",
    "ECHO": "ECHO hi",
    "ID": "ID",
    "HELLO": "HELLO",
    # DEV family (dev_commands.cpp) -- one entry per HELP-listed subcommand.
    "DEV M": "DEV M 1 DUTY 50",
    "DEV DT": "DEV DT PORTS 1 2",
    "DEV STATE": "DEV STATE",
    "DEV STOP": "DEV STOP",
    "DEV WD": "DEV WD 3000",
    # Telemetry (telemetry_commands.cpp).
    "STREAM": "STREAM 100",
    "SNAP": "SNAP",
    # Motion (motion_commands.cpp).
    "S": "S 100 100",
    "T": "T 100 100 200",
    "D": "D 100 100 100",
    "R": "R 100 500",
    "TURN": "TURN 0",
    "RT": "RT 0",
    "G": "G 100 0 100",
    "STOP": "STOP",
    # Config (config_commands.cpp).
    "SET": "SET tw=120",
    "GET": "GET tw",
    # Pose (pose_commands.cpp).
    "SI": "SI 0 0 0",
    "ZERO": "ZERO enc",
    # OTOS (otos_commands.cpp).
    "OI": "OI",
    "OZ": "OZ",
    "OR": "OR",
    "OP": "OP",
    "OV": "OV 0 0 0",
    "OL": "OL",
    "OA": "OA",
}

# Verbs whose documented reply tag is not "OK "/"ERR " (docs/protocol-v2.md
# section 3) -- their own bare taxonomy counts as well-formed too.
_BARE_REPLY_PREFIX: dict[str, str] = {
    "ID": "ID ",
    "HELLO": "DEVICE:",
    "SNAP": "TLM ",
    "GET": "CFG ",
}

# Defined error codes (docs/protocol-v2.md section 4) -- an ERR reply with
# any other code is NOT well-formed (an undefined/typo'd code).
_DEFINED_ERR_CODES = frozenset({
    "unsupported", "nodev", "badarg", "badkey", "range", "badval", "unknown",
})


def _assert_well_formed(verb: str, reply: str) -> None:
    """Assert ``reply`` (as returned by one ``command_on()`` call for
    ``verb``) is well-formed: non-empty, and either ``OK ...``, a verb's own
    documented bare tag (``_BARE_REPLY_PREFIX``), or a *defined* ``ERR
    <code>``. Proves reachability (registered + dispatches + replies), not
    per-verb correctness."""
    reply = reply.strip()
    assert reply, f"{verb}: expected a non-empty reply, got {reply!r}"

    bare_prefix = _BARE_REPLY_PREFIX.get(verb)
    if bare_prefix is not None:
        assert reply.startswith(bare_prefix), (
            f"{verb}: expected reply starting with {bare_prefix!r}, got {reply!r}"
        )
        return

    first_line = reply.splitlines()[0]
    if first_line.startswith("OK "):
        return
    if first_line.startswith("ERR "):
        tokens = first_line.split()
        code = tokens[1] if len(tokens) > 1 else ""
        assert code in _DEFINED_ERR_CODES, (
            f"{verb}: undefined ERR code {code!r} in reply {reply!r}"
        )
        return
    raise AssertionError(f"{verb}: not a well-formed reply: {reply!r}")


def _smoke(sim, verb: str) -> None:
    """Send ``verb``'s ``_SMOKE_LINES`` invocation on BOTH the SERIAL and
    RADIO channels (088-006's ``Sim.command_on()``) and assert a
    well-formed reply on each -- the per-verb acceptance proof every
    ``test_smoke_*`` function below delegates to."""
    line = _SMOKE_LINES[verb]
    _assert_well_formed(verb, sim.command_on(line, CHANNEL_SERIAL))
    _assert_well_formed(verb, sim.command_on(line, CHANNEL_RADIO))


# ---------------------------------------------------------------------------
# One smoke-test function per registered verb (32 total).
# ---------------------------------------------------------------------------

# -- Liveness (system_commands.cpp) --

def test_smoke_ping(sim):
    _smoke(sim, "PING")


def test_smoke_ver(sim):
    _smoke(sim, "VER")


def test_smoke_help(sim):
    _smoke(sim, "HELP")


def test_smoke_echo(sim):
    _smoke(sim, "ECHO")


def test_smoke_id(sim):
    _smoke(sim, "ID")


def test_smoke_hello(sim):
    _smoke(sim, "HELLO")


# -- DEV family (dev_commands.cpp) --

def test_smoke_dev_m(sim):
    _smoke(sim, "DEV M")


def test_smoke_dev_dt(sim):
    _smoke(sim, "DEV DT")


def test_smoke_dev_state(sim):
    _smoke(sim, "DEV STATE")


def test_smoke_dev_stop(sim):
    _smoke(sim, "DEV STOP")


def test_smoke_dev_wd(sim):
    _smoke(sim, "DEV WD")


# -- Telemetry (telemetry_commands.cpp) --

def test_smoke_stream(sim):
    _smoke(sim, "STREAM")


def test_smoke_snap(sim):
    _smoke(sim, "SNAP")


# -- Motion (motion_commands.cpp) --

def test_smoke_s(sim):
    _smoke(sim, "S")


def test_smoke_t(sim):
    _smoke(sim, "T")


def test_smoke_d(sim):
    _smoke(sim, "D")


def test_smoke_r(sim):
    _smoke(sim, "R")


def test_smoke_turn(sim):
    _smoke(sim, "TURN")


def test_smoke_rt(sim):
    _smoke(sim, "RT")


def test_smoke_g(sim):
    _smoke(sim, "G")


def test_smoke_stop(sim):
    _smoke(sim, "STOP")


# -- Config (config_commands.cpp) --

def test_smoke_set(sim):
    _smoke(sim, "SET")


def test_smoke_get(sim):
    _smoke(sim, "GET")


# -- Pose (pose_commands.cpp) --

def test_smoke_si(sim):
    _smoke(sim, "SI")


def test_smoke_zero(sim):
    _smoke(sim, "ZERO")


# -- OTOS (otos_commands.cpp) --

def test_smoke_oi(sim):
    _smoke(sim, "OI")


def test_smoke_oz(sim):
    _smoke(sim, "OZ")


def test_smoke_or(sim):
    _smoke(sim, "OR")


def test_smoke_op(sim):
    _smoke(sim, "OP")


def test_smoke_ov(sim):
    _smoke(sim, "OV")


def test_smoke_ol(sim):
    _smoke(sim, "OL")


def test_smoke_oa(sim):
    _smoke(sim, "OA")


# ---------------------------------------------------------------------------
# Completeness meta-test -- sourced from the LIVE HELP table, not a second
# hand-maintained list, so it fails the moment a registered verb and
# _SMOKE_LINES drift apart in either direction.
# ---------------------------------------------------------------------------

def _parse_registered_verbs(help_reply: str) -> list[str]:
    """Parse one ``OK help <verbs...>`` reply into verb units, joining a
    ``DEV`` prefix with its following subcommand token (``DEV M`` -> one
    unit) -- HELP's own live granularity (088-003's ``CommandRouter::
    listVerbs()``), matching ``_SMOKE_LINES``'s own keys."""
    assert help_reply.startswith("OK help "), f"unexpected HELP reply: {help_reply!r}"
    tokens = help_reply[len("OK help "):].split()

    verbs: list[str] = []
    i = 0
    while i < len(tokens):
        if tokens[i] == "DEV":
            assert i + 1 < len(tokens), f"trailing bare 'DEV' token in HELP reply: {help_reply!r}"
            verbs.append(f"DEV {tokens[i + 1]}")
            i += 2
        else:
            verbs.append(tokens[i])
            i += 1
    return verbs


def test_every_registered_verb_has_a_smoke_test(sim):
    """The completeness guard: every verb HELP's live table reports must
    have a `_SMOKE_LINES` entry (and, vice versa, no `_SMOKE_LINES` entry
    for a verb HELP does not report) -- fails if a command family is added
    to the firmware without a matching smoke test, or if a smoke test is
    left behind for a since-removed verb."""
    live_verbs = set(_parse_registered_verbs(sim.command("HELP").strip()))
    smoke_verbs = set(_SMOKE_LINES.keys())

    missing = live_verbs - smoke_verbs
    extra = smoke_verbs - live_verbs
    assert not missing, f"registered verbs with no smoke test: {sorted(missing)}"
    assert not extra, f"smoke tests for verbs HELP does not register: {sorted(extra)}"

    # Guard the guard: a `_SMOKE_LINES` entry with no matching
    # `test_smoke_*` function (e.g. a typo'd function name, or an entry
    # added without ever writing its test) would otherwise pass the set
    # comparison above silently -- cross-check the count of test_smoke_*
    # functions this module actually defines against _SMOKE_LINES's size.
    smoke_test_fn_count = sum(1 for name in globals() if name.startswith("test_smoke_"))
    assert smoke_test_fn_count == len(_SMOKE_LINES), (
        f"expected exactly one test_smoke_* function per _SMOKE_LINES entry "
        f"({len(_SMOKE_LINES)}), found {smoke_test_fn_count} -- check for a "
        f"missing, duplicate, or misnamed smoke test function"
    )
