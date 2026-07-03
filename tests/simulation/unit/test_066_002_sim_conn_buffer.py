"""
test_066_002_sim_conn_buffer.py — SimConnection reply buffer size (CR-14).

Background
----------
``SimConnection._raw_command`` (host/robot_radio/io/sim_conn.py) used to
allocate a 512-byte ``ctypes.create_string_buffer`` for the synchronous
``sim_command()`` reply, while the C side (``ReplyStore`` in
``tests/_infra/sim/sim_api.cpp``) accumulates up to ``kReplyBufSize = 2048``
bytes. A bare ``GET`` (all-keys config dump, chunked into multiple ``CFG``
lines per handleGet's N12 chunking note) produces ~750+ bytes of reply —
comfortably over the old 512-byte cap — so it was silently truncated
mid-line by ``_raw_command``'s undersized buffer.

This test issues a bare ``GET`` through ``SimConnection`` (not the standalone
``firmware.Sim`` wrapper, which already used a 2048-byte buffer and would not
reproduce the bug) and asserts the full multi-line dump comes back intact.
"""

from __future__ import annotations

import pathlib
import sys

_HERE = pathlib.Path(__file__).parent
_HOST = _HERE.parent.parent.parent / "host"
if str(_HOST) not in sys.path:
    sys.path.insert(0, str(_HOST))


def test_get_cfg_dump_via_sim_connection_is_not_truncated(build_lib):
    """A bare GET (all-keys dump) via SimConnection must return every CFG line intact."""
    from robot_radio.io.sim_conn import SimConnection

    conn = SimConnection()
    info = conn.connect()
    assert "error" not in info, f"SimConnection.connect() failed: {info}"
    try:
        reply = conn.send("GET", read_timeout=50)
        responses = reply["responses"]
        combined = "\n".join(responses)

        # The whole point of this test: the combined reply must exceed the
        # OLD 512-byte buffer cap, so it actually exercises the fix (not a
        # vacuously-small reply that would pass either way).
        assert len(combined) > 512, (
            f"GET dump was only {len(combined)} bytes — too small to "
            f"distinguish the 512-byte-truncation bug from a correct reply; "
            f"reply={combined!r}"
        )

        cfg_lines = [ln for ln in responses if ln.startswith("CFG ")]
        assert len(cfg_lines) >= 2, (
            f"Expected multiple chunked CFG lines (handleGet's N12 chunking, "
            f"~200 bytes/line); got {len(cfg_lines)}: {responses!r}"
        )

        # Every CFG line must be a well-formed sequence of key=value tokens —
        # a truncation mid-line would leave a dangling/incomplete final
        # token (no '=', or a value cut off mid-digit).
        for ln in cfg_lines:
            body = ln[len("CFG "):].strip()
            assert body, f"empty CFG line body: {ln!r}"
            for tok in body.split():
                assert "=" in tok, (
                    f"malformed (likely truncated) key=value token {tok!r} "
                    f"in line {ln!r} — reply may have been cut mid-line"
                )
                key, _, value = tok.partition("=")
                assert key and value, (
                    f"malformed (likely truncated) token {tok!r} in {ln!r}"
                )

        # A specific late-dump key (near the end of the registry — see
        # ConfigRegistry.cpp's kRegistry table) must be present. Under the
        # 512-byte truncation bug this key never arrives at all.
        assert "ekfRHead=" in combined, (
            f"Expected key 'ekfRHead' (near the end of the config registry) "
            f"missing from the GET dump — reply was likely truncated; "
            f"combined={combined!r}"
        )
    finally:
        conn.disconnect()
