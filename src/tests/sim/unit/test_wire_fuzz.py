"""Fuzz corpus for the P4-pruned wire protocol (103-001, SUC-001,
architecture-update.md (103) Decisions 2/3). Telemetry encode-side fuzz
cases rewritten 115-009 for the frame-v2 shape (115-003).

***Part of the correctness gate the `src/firm/app/` tickets (004+) are built
on top of -- see test_wire_differential.py's module docstring for the full
"this is a blocking regression, not an xfail" statement, which applies
equally here.***

Feeds >= 200 generated malformed/adversarial byte strings to the firmware
decoder (`msg::wire::decode(CommandEnvelope&, ...)`, via
``wire_differential_harness``'s `decode` argv command) across the four
categories:

  1. **random** -- uniformly random byte strings of random length.
  2. **truncated** -- a valid, pb2-serialized `CommandEnvelope` chopped at
     EVERY byte boundary (0..len-1).
  3. **oversized** -- a valid encoding plus trailing random garbage.
  4. **salted** -- a valid encoding with an extra, unrecognized top-level
     field (field number 99) spliced in at a genuine FIELD BOUNDARY.

For categories 1-3 the only acceptance bar is: the harness process must
exit 0 with a well-formed "OK ..."/"ERR field=<n> code=<NAME>" line -- NEVER
crash, NEVER trip ASan/UBSan. `decode()` returning `ok=false` for garbage
input is a normal, correct outcome, not a test failure.

For category 4 the bar is stronger: decode() must additionally succeed
(`OK`) with the ORIGINAL message's known fields (`corr_id`, the active arm)
intact and unaffected by the spliced-in unknown field.

**Run under ASan/UBSan** (`-fsanitize=address,undefined -fno-omit-frame-
pointer -g`) -- this is the ENTIRE point of the fuzz sub-suite:
`WireRuntime`'s primitives and the generated field-table walker in
`wire.cpp` operate on raw, adversarial byte buffers with pointer arithmetic
and fixed-size stack/struct buffers; ASan/UBSan is what actually proves
"never reads/writes out of bounds", not just "returned the right bool".
"""
from __future__ import annotations

import pathlib
import random

import pytest

from _wire_diff_driver import (  # noqa: E402
    compile_harness,
    decode,
    encode_telemetry,
    encode_telemetry_secondary,
    env_config_drivetrain,
    env_move_twist,
    env_stop,
    parse_decode_line,
    unknown_varint_field,
)

# ---------------------------------------------------------------------------
# Representative valid CommandEnvelope encodings the truncated/oversized/
# salted categories are built from -- span all three live arms (move,
# config, stop -- `move` replaced `twist` as the sole motion arm, 116-001),
# so the corpus isn't skewed toward a single arm's byte shape.
# ---------------------------------------------------------------------------

_VALID_MOVE = env_move_twist(100, 500.0, 0.0, 3.0, stop_field="time", stop_value=700.0, timeout=5000.0,
                              replace=True, move_id=1)
_VALID_CONFIG_DRIVETRAIN = env_config_drivetrain(
    104, trackwidth=321.0, rotational_slip=0.75, ekf_q_xy=1.5, ekf_q_theta=2.5, ekf_r_otos_xy=3.5,
    ekf_r_otos_theta=4.5)
_VALID_STOP = env_stop(105)

_VALID_MESSAGES = {
    "move": (_VALID_MOVE, 100, "MOVE"),
    "config_drivetrain": (_VALID_CONFIG_DRIVETRAIN, 104, "CONFIG"),
    "stop": (_VALID_STOP, 105, "STOP"),
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
    # 150 (raised from the pre-103 suite's 60): the pruned P4 schema's 3
    # live arms (move/config/stop) are much shorter on average than the
    # pre-103 schema's ~9, so the truncated category (one case per byte
    # boundary per valid message, this corpus's dominant contributor before)
    # shrank a lot -- widen the random category to keep the corpus above the
    # acceptance criterion's 200-case floor (see test_corpus_size_at_least_200).
    rng = random.Random(20260710)
    for i in range(150):
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
    """Acceptance criterion: the fuzz corpus is >= 200 cases.

    NOTE: the pruned P4 schema has only 3 live arms (move/config/stop)
    against the pre-103 schema's ~9 -- the truncated category (the corpus's
    dominant contributor, one case per byte boundary per valid message) is
    correspondingly smaller (60 cases vs. the pre-103 suite's much larger
    total). The random-byte category was widened (60 -> 150, see
    `_build_corpus()`) to keep the total comfortably above 200; this
    assertion is the actual gate, not the category counts -- if a future
    schema change drops below 200, widen the random category further
    rather than silently lowering this bar.
    """
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


# ---------------------------------------------------------------------------
# ASan/UBSan encode-side check for Telemetry/TelemetrySecondary, this
# schema's two REPLY-only (encode-only) messages. The categories above all
# target decode() (adversarial raw BYTES); encode() instead takes a
# fully-typed, harness-constructed C++ struct, so there is no "malformed
# input" surface to fuzz the same way -- what CAN still go wrong is a
# buffer-sizing bug in encodeInto()/encodeNestedMessage()'s fixed-size
# scratch buffers (wire.cpp's kEncodeScratchCap) once a message grows this
# large: Telemetry is the single biggest oneof arm in ReplyEnvelope (~173B,
# wire.h's own kReplyEnvelopeMaxEncodedSize breakdown), with a depth-3
# repeated-message ack ring PLUS three nested messages (pose/otos/twist) --
# the most encode() field-table walking any single call in this schema
# does. Extreme scalar values (fault_bits/event_bits/ack err_code at
# UINT32_MAX, every float at its IEEE-754 extreme, mode past its declared
# enum range) exercise every encodeScalarValue() branch at its own type's
# boundary under ASan/UBSan.
# ---------------------------------------------------------------------------

_UINT32_MAX = 4294967295
_FLOAT_EXTREMES = [3.4028235e38, -3.4028235e38, 1.1754944e-38, 0.0, float("nan"), float("inf"), float("-inf")]


@pytest.mark.parametrize("value", _FLOAT_EXTREMES, ids=[f"f{i}" for i in range(len(_FLOAT_EXTREMES))])
def test_fuzz_encode_telemetry_float_extremes(asan_harness, value):
    r = encode_telemetry(
        asan_harness, 1, now=_UINT32_MAX, mode=255, seq=_UINT32_MAX, flags=_UINT32_MAX,
        ack_corr=_UINT32_MAX, ack_err=_UINT32_MAX,
        enc_left_position=value, enc_left_velocity=value, enc_left_time=_UINT32_MAX,
        enc_right_position=value, enc_right_velocity=value, enc_right_time=_UINT32_MAX,
        otos_x=value, otos_y=value, otos_heading=value, otos_v_x=value, otos_v_y=value, otos_omega=value,
        otos_time=_UINT32_MAX,
        pose_x=value, pose_y=value, pose_h=value,
        twist_v_x=value, twist_v_y=value, twist_omega=value,
        line=_UINT32_MAX, color=_UINT32_MAX,
    )
    # No ASan/UBSan finding is the only bar here (encode_telemetry() itself
    # already asserts !crashed and a well-formed "B64 .../ZERO" line via
    # run_harness()'s own crashed-detection -- calling it at all under the
    # ASan-built binary IS the test).
    assert r is None or len(r) > 0


@pytest.mark.parametrize("value", _FLOAT_EXTREMES, ids=[f"f{i}" for i in range(len(_FLOAT_EXTREMES))])
def test_fuzz_encode_telemetry_secondary_float_extremes(asan_harness, value):
    r = encode_telemetry_secondary(asan_harness, now=_UINT32_MAX, has_cmd_vel=True, cmd_vel_left=value,
                                    cmd_vel_right=value, acc_left=value, acc_right=value,
                                    glitch_left=_UINT32_MAX, glitch_right=_UINT32_MAX, ts_left=_UINT32_MAX,
                                    ts_right=_UINT32_MAX)
    assert r is None or len(r) > 0


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
