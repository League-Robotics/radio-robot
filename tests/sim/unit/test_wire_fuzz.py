"""Fuzz corpus for ticket 095-006 (SUC-005, architecture-update.md M8 "Codec
Test Harness").

***Part of the correctness gate `BinaryChannel` (ticket 007) is built on top
of -- see test_wire_differential.py's module docstring for the full "this is
a blocking regression, not an xfail" statement, which applies equally here.***

Feeds >= 200 generated malformed/adversarial byte strings to the firmware
decoder (`msg::wire::decode(CommandEnvelope&, ...)`, via
``wire_differential_harness``'s `decode` argv command) across the four
categories the ticket names:

  1. **random** -- uniformly random byte strings of random length.
  2. **truncated** -- a valid, pb2-serialized `CommandEnvelope` chopped at
     EVERY byte boundary (0..len-1).
  3. **oversized** -- a valid encoding plus trailing random garbage.
  4. **salted** -- a valid encoding with an extra, unrecognized top-level
     field (field number 99) spliced in at a genuine FIELD BOUNDARY
     (prepended or appended to the whole message -- never spliced into the
     middle of an existing field's own bytes, which would just be a
     different malformed-input case, not a clean "extra unknown field"
     case).

For categories 1-3 the only acceptance bar is: the harness process must
exit 0 with a well-formed "OK ..."/"ERR field=<n> code=<NAME>" line -- NEVER
crash, NEVER trip ASan/UBSan. `decode()` returning `ok=false` for garbage
input is a normal, correct outcome, not a test failure.

For category 4 the bar is stronger, per the ticket text ("always returns...
`ok=true` with the unknown field correctly skipped"): decode() must
additionally succeed (`OK`) with the ORIGINAL message's known fields
(`corr_id`, the active arm) intact and unaffected by the spliced-in unknown
field.

**Run under ASan/UBSan** (`-fsanitize=address,undefined -fno-omit-frame-
pointer -g`, matching wire_codec_harness.cpp's/wire_runtime_harness.cpp's
own precedent from tickets 004/005) -- this is the ENTIRE point of the
fuzz sub-suite: `WireRuntime`'s primitives and the generated field-table
walker in `wire.cpp` operate on raw, adversarial byte buffers with pointer
arithmetic and fixed-size stack/struct buffers; ASan/UBSan is what actually
proves "never reads/writes out of bounds", not just "returned the right
bool".
"""
from __future__ import annotations

import os
import pathlib
import random

import pytest

from _wire_diff_driver import (  # noqa: E402
    build_motion_segment,
    compile_harness,
    decode,
    env_drive_twist,
    env_drive_wheels,
    env_echo,
    env_segment,
    parse_decode_line,
    unknown_varint_field,
)

# ---------------------------------------------------------------------------
# Representative valid CommandEnvelope encodings the truncated/oversized/
# salted categories are built from -- span drive (twist + wheels), segment,
# and echo, so the corpus isn't skewed toward a single arm's byte shape.
# ---------------------------------------------------------------------------

_VALID_SEGMENT = env_segment(100, build_motion_segment(
    distance=-1500.0, direction=0.5, final_heading=1.2, speed_max=800.0, accel_max=2000.0, jerk_max=30000.0,
    yaw_rate_max=6.0, yaw_accel_max=40.0, yaw_jerk_max=100.0,
))
_VALID_DRIVE_WHEELS = env_drive_wheels(101, [(100.0, -5.0), (200.0, 10.0), (300.0, None), (None, 400.0)])
_VALID_DRIVE_TWIST = env_drive_twist(102, 500.0, 600.0, 700.0, seed=True, standby=True)
_VALID_ECHO = env_echo(103, bytes(range(40)))

_VALID_MESSAGES = {
    "segment": (_VALID_SEGMENT, 100, "SEGMENT"),
    "drive_wheels": (_VALID_DRIVE_WHEELS, 101, "DRIVE"),
    "drive_twist": (_VALID_DRIVE_TWIST, 102, "DRIVE"),
    "echo": (_VALID_ECHO, 103, "ECHO"),
}


# ---------------------------------------------------------------------------
# Corpus generation -- each case is (category, case_id, raw_bytes, kind)
# where kind is "any" (categories 1-3: no crash is the only bar) or
# "salted" (category 4: must decode OK with corr_id/cmd_kind intact).
# ---------------------------------------------------------------------------


def _build_corpus() -> list[tuple[str, bytes, str, int | None, str | None]]:
    """Returns a list of (case_id, raw_bytes, kind, expected_corr_id,
    expected_cmd_kind) -- the latter two only meaningful for kind=="salted"."""
    cases: list[tuple[str, bytes, str, int | None, str | None]] = []

    # 1. Random bytes -- fixed seed for a reproducible, non-flaky corpus.
    rng = random.Random(20260710)
    for i in range(60):
        length = rng.randint(0, 250)
        raw = bytes(rng.getrandbits(8) for _ in range(length))
        cases.append((f"random_{i}_len{length}", raw, "any", None, None))

    # 2. Truncated at every byte boundary (0..len-1) of each valid message.
    for name, (msg, _corr, _kind) in _VALID_MESSAGES.items():
        for i in range(len(msg)):
            cases.append((f"truncated_{name}_at{i}", msg[:i], "any", None, None))

    # 3. Oversized -- valid encoding + trailing random garbage.
    garbage_rng = random.Random(9990710)
    for name, (msg, _corr, _kind) in _VALID_MESSAGES.items():
        for extra_len in (1, 5, 20, 64):
            garbage = bytes(garbage_rng.getrandbits(8) for _ in range(extra_len))
            cases.append((f"oversized_{name}_plus{extra_len}", msg + garbage, "any", None, None))

    # 4. Unknown-field-salted -- an extra (field 99, varint) spliced at a
    # genuine field boundary (prepend/append to the WHOLE message).
    salt = unknown_varint_field(99, 123456)
    for name, (msg, corr, kind) in _VALID_MESSAGES.items():
        cases.append((f"salted_prepend_{name}", salt + msg, "salted", corr, kind))
        cases.append((f"salted_append_{name}", msg + salt, "salted", corr, kind))
        cases.append((f"salted_both_{name}", salt + msg + salt, "salted", corr, kind))

    return cases


_CORPUS = _build_corpus()


def test_corpus_size_at_least_200():
    """Acceptance criterion: the fuzz corpus is >= 200 cases."""
    assert len(_CORPUS) >= 200, f"fuzz corpus only has {len(_CORPUS)} cases, need >= 200"


@pytest.fixture(scope="module")
def asan_harness(tmp_path_factory) -> pathlib.Path:
    tmp_path = tmp_path_factory.mktemp("wire_fuzz_asan")
    return compile_harness(
        tmp_path, "wire_differential_harness_asan",
        ["-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-g"],
    )


@pytest.mark.parametrize("case_id,raw,kind,expected_corr_id,expected_cmd_kind", _CORPUS,
                         ids=[c[0] for c in _CORPUS])
def test_fuzz_case(asan_harness, case_id, raw, kind, expected_corr_id, expected_cmd_kind):
    run = decode(asan_harness, raw)
    assert not run.crashed, (
        f"harness CRASHED (ASan/UBSan finding) decoding fuzz case {case_id!r} "
        f"({len(raw)} bytes: {raw.hex()}):\nstdout:\n{run.stdout}\nstderr:\n{run.stderr}"
    )
    # Every non-crashing invocation must produce exactly one well-formed
    # "OK ..." or "ERR field=<n> code=<NAME>" line -- never a garbled/partial
    # print (which would itself indicate memory corruption even if the
    # process happened to still exit 0).
    status, fields = parse_decode_line(run.stdout)
    assert status in ("OK", "ERR")

    if kind == "salted":
        assert status == "OK", (
            f"salted case {case_id!r} (extra unknown field 99 spliced at a genuine field boundary) "
            f"must still decode OK -- unknown fields must be SKIPPED, not rejected. Got: {run.stdout}"
        )
        assert fields.get("corr_id") == str(expected_corr_id), (
            f"salted case {case_id!r}: corr_id corrupted by the unknown-field splice -- "
            f"expected {expected_corr_id}, got {fields.get('corr_id')}"
        )
        assert fields.get("cmd_kind") == expected_cmd_kind, (
            f"salted case {case_id!r}: cmd_kind corrupted by the unknown-field splice -- "
            f"expected {expected_cmd_kind}, got {fields.get('cmd_kind')}"
        )
    else:
        if status == "ERR":
            # A well-formed ERR line must carry BOTH keys; a malformed/
            # truncated print here would itself be a signal of a corrupted
            # output path.
            assert "field" in fields and "code" in fields, f"malformed ERR line for {case_id!r}: {run.stdout}"


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
