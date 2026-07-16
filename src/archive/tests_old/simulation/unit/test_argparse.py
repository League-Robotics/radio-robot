"""
test_argparse.py — unit tests for ArgParse.h / ArgParse.cpp (051-002).

Tests exercise parseSchema() and the inline helpers (argStr/argInt/argFloat,
kvFind/kvInt/kvFloat/kvHas) via the sim_parse_schema C-ABI hook added to
sim_api.cpp.

The C ABI function signature:
    int sim_parse_schema(
        tokens,   ntokens,
        kv_keys,  kv_vals,  nkv,
        def_names, def_kinds, def_ranged, def_lo, def_hi, ndefs,
        min_tokens, variadic, pack_kv,
        out_ok, out_count, out_supplied_count,
        out_arg_types, out_arg_ivals, out_arg_fvals, out_arg_svals,
        err_detail_buf)

ArgKind/ArgType encoding (matches C++ enums):
    INT=0, FLOAT=1, STR=2

Returns 1 on ok, 0 on failure.
"""
import ctypes
import pathlib
import sys
import pytest

# Locate the sim shared library (same path as firmware.py).
_HERE = pathlib.Path(__file__).parent
_REPO = _HERE.parent.parent.parent
_SIM_DIR = _REPO / "tests" / "_infra" / "sim"

sys.path.insert(0, str(_SIM_DIR))

from firmware import LIB_PATH  # noqa: E402 (after sys.path insert)

# ---------------------------------------------------------------------------
# ctypes helpers
# ---------------------------------------------------------------------------

MAX_ARGS = 10
_STR_SLOT = 32  # bytes per sval slot in the flat svals buffer


def _load_lib():
    lib = ctypes.CDLL(str(LIB_PATH))
    lib.sim_parse_schema.restype = ctypes.c_int
    lib.sim_parse_schema.argtypes = [
        # tokens, ntokens
        ctypes.POINTER(ctypes.c_char_p),
        ctypes.c_int,
        # kv_keys, kv_vals, nkv
        ctypes.POINTER(ctypes.c_char_p),
        ctypes.POINTER(ctypes.c_char_p),
        ctypes.c_int,
        # def_names, def_kinds, def_ranged, def_lo, def_hi, ndefs
        ctypes.POINTER(ctypes.c_char_p),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
        ctypes.c_int,
        # min_tokens, variadic, pack_kv
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_char_p,
        # out_ok, out_count, out_supplied_count
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
        # out_arg_types, out_arg_ivals, out_arg_fvals, out_arg_svals
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_char_p,
        # err_detail_buf
        ctypes.c_char_p,
    ]
    return lib


_lib = _load_lib()


def _str_array(strings):
    """Convert a list of str/bytes/None to a ctypes c_char_p array."""
    arr = (ctypes.c_char_p * len(strings))()
    for i, s in enumerate(strings):
        if s is None:
            arr[i] = None
        elif isinstance(s, str):
            arr[i] = s.encode()
        else:
            arr[i] = s
    return arr


def _int_array(ints):
    arr = (ctypes.c_int * len(ints))()
    for i, v in enumerate(ints):
        arr[i] = v
    return arr


class ArgDef:
    """Python-side representation of one C++ ArgDef."""
    def __init__(self, name, kind, ranged=False, lo=0, hi=0):
        self.name   = name    # str
        self.kind   = kind    # 0=INT, 1=FLOAT, 2=STR
        self.ranged = ranged  # bool
        self.lo     = lo      # int
        self.hi     = hi      # int


def call_parse_schema(tokens, kvs=None, defs=None,
                      min_tokens=0, variadic=False, pack_kv=None):
    """
    Call sim_parse_schema and return a dict:
        {
          'ok':             bool,
          'count':          int,
          'supplied_count': int,  # positional slots actually supplied (064-001)
          'args':   [{'type':int, 'ival':int, 'fval':float, 'sval':str}, ...],
          'detail': str | None   # error detail when not ok
        }
    """
    if defs is None:
        defs = []
    if kvs is None:
        kvs = []

    ntokens  = len(tokens)
    nkv      = len(kvs)
    ndefs    = len(defs)

    tok_arr  = _str_array(tokens)  if ntokens  else (ctypes.c_char_p * 1)()
    kk_arr   = _str_array([k for k, v in kvs]) if nkv else (ctypes.c_char_p * 1)()
    kv_arr   = _str_array([v for k, v in kvs]) if nkv else (ctypes.c_char_p * 1)()
    dn_arr   = _str_array([d.name for d in defs]) if ndefs else (ctypes.c_char_p * 1)()
    dk_arr   = _int_array([d.kind   for d in defs]) if ndefs else (ctypes.c_int * 1)()
    dr_arr   = _int_array([1 if d.ranged else 0 for d in defs]) if ndefs else (ctypes.c_int * 1)()
    dlo_arr  = _int_array([d.lo for d in defs]) if ndefs else (ctypes.c_int * 1)()
    dhi_arr  = _int_array([d.hi for d in defs]) if ndefs else (ctypes.c_int * 1)()

    out_ok             = ctypes.c_int(0)
    out_count          = ctypes.c_int(0)
    out_supplied_count = ctypes.c_int(0)
    out_types = (ctypes.c_int   * MAX_ARGS)()
    out_ivals = (ctypes.c_int   * MAX_ARGS)()
    out_fvals = (ctypes.c_float * MAX_ARGS)()
    out_svals = ctypes.create_string_buffer(MAX_ARGS * _STR_SLOT)
    err_buf   = ctypes.create_string_buffer(64)

    _lib.sim_parse_schema(
        tok_arr,  ntokens,
        kk_arr, kv_arr, nkv,
        dn_arr, dk_arr, dr_arr, dlo_arr, dhi_arr, ndefs,
        min_tokens, 1 if variadic else 0,
        pack_kv.encode() if pack_kv else None,
        ctypes.byref(out_ok), ctypes.byref(out_count),
        ctypes.byref(out_supplied_count),
        out_types, out_ivals, out_fvals, out_svals,
        err_buf,
    )

    ok             = (out_ok.value == 1)
    count          = out_count.value
    supplied_count = out_supplied_count.value
    args  = []
    for i in range(count):
        slot = out_svals.raw[i * _STR_SLOT: i * _STR_SLOT + _STR_SLOT]
        nul  = slot.find(b'\x00')
        sv   = slot[:nul].decode(errors='replace') if nul >= 0 else slot.decode(errors='replace')
        args.append({
            'type': out_types[i],
            'ival': out_ivals[i],
            'fval': out_fvals[i],
            'sval': sv,
        })

    detail_raw = err_buf.raw
    nul = detail_raw.find(b'\x00')
    detail_str = detail_raw[:nul].decode(errors='replace') if nul >= 0 else ''
    detail = detail_str if detail_str else None

    return {
        'ok': ok,
        'count': count,
        'supplied_count': supplied_count,
        'args': args,
        'detail': detail,
    }


# ---------------------------------------------------------------------------
# Tests — no-arg schema
# ---------------------------------------------------------------------------

class TestNoArg:
    def test_no_arg_empty_tokens_ok(self):
        r = call_parse_schema([], defs=[], min_tokens=0, variadic=False)
        assert r['ok'] is True
        assert r['count'] == 0
        assert r['args'] == []

    def test_no_arg_with_extra_tokens_ok(self):
        """Extra tokens beyond ndefs=0 are silently ignored in positional mode."""
        r = call_parse_schema(['foo', 'bar'], defs=[], min_tokens=0, variadic=False)
        assert r['ok'] is True
        assert r['count'] == 0

    def test_no_arg_min_tokens_zero_ok(self):
        r = call_parse_schema([], defs=[], min_tokens=0)
        assert r['ok'] is True


# ---------------------------------------------------------------------------
# Tests — minTokens guard
# ---------------------------------------------------------------------------

class TestMinTokens:
    def test_min_tokens_fail(self):
        """ntokens < minTokens → ok=False, detail=None."""
        defs = [ArgDef('x', 0, ranged=False)]
        r = call_parse_schema([], defs=defs, min_tokens=1)
        assert r['ok'] is False
        assert r['detail'] is None  # {nullptr, nullptr}

    def test_min_tokens_exactly_met(self):
        defs = [ArgDef('x', 0, ranged=False)]
        r = call_parse_schema(['42'], defs=defs, min_tokens=1)
        assert r['ok'] is True
        assert r['args'][0]['ival'] == 42

    def test_min_tokens_exceeded(self):
        defs = [ArgDef('x', 0, ranged=False)]
        r = call_parse_schema(['42', '99'], defs=defs, min_tokens=1)
        assert r['ok'] is True
        assert r['count'] == 1  # only ndefs=1 parsed


# ---------------------------------------------------------------------------
# Tests — positional INT (unranged)
# ---------------------------------------------------------------------------

class TestPositionalInt:
    def test_int_stored_correctly(self):
        defs = [ArgDef('v', 0, ranged=False)]
        r = call_parse_schema(['500'], defs=defs, min_tokens=1)
        assert r['ok'] is True
        assert r['count'] == 1
        assert r['args'][0]['type'] == 0  # INT
        assert r['args'][0]['ival'] == 500

    def test_int_negative(self):
        defs = [ArgDef('v', 0, ranged=False)]
        r = call_parse_schema(['-300'], defs=defs, min_tokens=1)
        assert r['ok'] is True
        assert r['args'][0]['ival'] == -300

    def test_int_large_unranged_accepted(self):
        """Without ranged, large values are accepted (handler casts to int16 internally)."""
        defs = [ArgDef('v', 0, ranged=False)]
        r = call_parse_schema(['99999'], defs=defs, min_tokens=0)
        assert r['ok'] is True
        assert r['args'][0]['ival'] == 99999

    def test_two_ints(self):
        defs = [ArgDef('l', 0, ranged=False), ArgDef('r', 0, ranged=False)]
        r = call_parse_schema(['100', '-200'], defs=defs, min_tokens=2)
        assert r['ok'] is True
        assert r['count'] == 2
        assert r['args'][0]['ival'] == 100
        assert r['args'][1]['ival'] == -200


# ---------------------------------------------------------------------------
# Tests — positional INT with range check
# ---------------------------------------------------------------------------

class TestPositionalIntRanged:
    def test_at_lo_boundary_pass(self):
        defs = [ArgDef('v', 0, ranged=True, lo=-1000, hi=1000)]
        r = call_parse_schema(['-1000'], defs=defs, min_tokens=1)
        assert r['ok'] is True
        assert r['args'][0]['ival'] == -1000

    def test_at_hi_boundary_pass(self):
        defs = [ArgDef('v', 0, ranged=True, lo=-1000, hi=1000)]
        r = call_parse_schema(['1000'], defs=defs, min_tokens=1)
        assert r['ok'] is True
        assert r['args'][0]['ival'] == 1000

    def test_below_lo_fail(self):
        defs = [ArgDef('speed', 0, ranged=True, lo=-1000, hi=1000)]
        r = call_parse_schema(['-1001'], defs=defs, min_tokens=1)
        assert r['ok'] is False
        assert r['detail'] == 'speed'  # def.name

    def test_above_hi_fail(self):
        defs = [ArgDef('speed', 0, ranged=True, lo=-1000, hi=1000)]
        r = call_parse_schema(['1001'], defs=defs, min_tokens=1)
        assert r['ok'] is False
        assert r['detail'] == 'speed'

    def test_second_arg_range_fail(self):
        """Range failure on the second positional arg; detail = second arg's name."""
        defs = [
            ArgDef('l', 0, ranged=True, lo=-1000, hi=1000),
            ArgDef('r', 0, ranged=True, lo=-1000, hi=1000),
        ]
        r = call_parse_schema(['500', '-9999'], defs=defs, min_tokens=2)
        assert r['ok'] is False
        assert r['detail'] == 'r'

    def test_range_zero_to_zero(self):
        """Exact range [0,0]: only 0 passes."""
        defs = [ArgDef('x', 0, ranged=True, lo=0, hi=0)]
        assert call_parse_schema(['0'],  defs=defs, min_tokens=1)['ok'] is True
        assert call_parse_schema(['1'],  defs=defs, min_tokens=1)['ok'] is False
        assert call_parse_schema(['-1'], defs=defs, min_tokens=1)['ok'] is False


# ---------------------------------------------------------------------------
# Tests — positional FLOAT
# ---------------------------------------------------------------------------

class TestPositionalFloat:
    def test_float_stored(self):
        defs = [ArgDef('k', 1, ranged=False)]
        r = call_parse_schema(['1.5'], defs=defs, min_tokens=1)
        assert r['ok'] is True
        assert r['args'][0]['type'] == 1  # FLOAT
        assert abs(r['args'][0]['fval'] - 1.5) < 1e-4

    def test_float_negative(self):
        defs = [ArgDef('f', 1, ranged=False)]
        r = call_parse_schema(['-3.14'], defs=defs, min_tokens=1)
        assert r['ok'] is True
        assert abs(r['args'][0]['fval'] - (-3.14)) < 1e-3


# ---------------------------------------------------------------------------
# Tests — positional STR
# ---------------------------------------------------------------------------

class TestPositionalStr:
    def test_str_stored(self):
        defs = [ArgDef('label', 2, ranged=False)]
        r = call_parse_schema(['hello'], defs=defs, min_tokens=1)
        assert r['ok'] is True
        assert r['args'][0]['type'] == 2  # STR
        assert r['args'][0]['sval'] == 'hello'

    def test_str_bounded_at_31(self):
        """sval is bounded at 31 chars + NUL."""
        src = 'A' * 40
        defs = [ArgDef('s', 2, ranged=False)]
        r = call_parse_schema([src], defs=defs, min_tokens=1)
        assert r['ok'] is True
        assert r['args'][0]['sval'] == 'A' * 31


# ---------------------------------------------------------------------------
# Tests — variadic path
# ---------------------------------------------------------------------------

class TestVariadic:
    def test_variadic_zero_tokens(self):
        r = call_parse_schema([], variadic=True, min_tokens=0)
        assert r['ok'] is True
        assert r['count'] == 0

    def test_variadic_one_token(self):
        r = call_parse_schema(['foo'], variadic=True, min_tokens=0)
        assert r['ok'] is True
        assert r['count'] == 1
        assert r['args'][0]['type'] == 2  # STR
        assert r['args'][0]['sval'] == 'foo'
        assert r['args'][0]['ival'] == 0
        assert r['args'][0]['fval'] == 0.0

    def test_variadic_ival_fval_init(self):
        """ival=0, fval=0.0f must be set before sval copy."""
        r = call_parse_schema(['abc', 'def'], variadic=True)
        for a in r['args']:
            assert a['ival'] == 0
            assert a['fval'] == pytest.approx(0.0)

    def test_variadic_max_args_tokens(self):
        """Exactly MAX_ARGS tokens — all accepted."""
        tokens = [str(i) for i in range(MAX_ARGS)]
        r = call_parse_schema(tokens, variadic=True)
        assert r['ok'] is True
        assert r['count'] == MAX_ARGS

    def test_variadic_max_args_plus_one_capped(self):
        """MAX_ARGS+1 tokens — capped at MAX_ARGS."""
        tokens = [str(i) for i in range(MAX_ARGS + 1)]
        r = call_parse_schema(tokens, variadic=True)
        assert r['ok'] is True
        assert r['count'] == MAX_ARGS

    def test_variadic_sval_bounded(self):
        """sval capped at 31 chars + NUL in variadic mode."""
        long_tok = 'X' * 50
        r = call_parse_schema([long_tok], variadic=True)
        assert r['ok'] is True
        assert r['args'][0]['sval'] == 'X' * 31

    def test_variadic_min_tokens_guard(self):
        """minTokens still applies on the variadic path."""
        r = call_parse_schema([], variadic=True, min_tokens=1)
        assert r['ok'] is False
        assert r['detail'] is None


# ---------------------------------------------------------------------------
# Tests — packKv
# ---------------------------------------------------------------------------

class TestPackKv:
    def test_pack_kv_present(self):
        """sensor= KV is appended as trailing STR arg at position ndefs."""
        defs = [
            ArgDef('l', 0, ranged=False),
            ArgDef('r', 0, ranged=False),
        ]
        kvs = [('sensor', 'line0:ge:128')]
        r = call_parse_schema(['100', '200'], kvs=kvs, defs=defs,
                              min_tokens=2, pack_kv='sensor')
        assert r['ok'] is True
        assert r['count'] == 3             # 2 positional + 1 packKv
        assert r['args'][2]['type'] == 2   # STR
        assert r['args'][2]['sval'] == 'line0:ge:128'

    def test_pack_kv_absent(self):
        """When key is not present, count is unchanged."""
        defs = [ArgDef('l', 0, ranged=False)]
        kvs = [('other_key', 'value')]
        r = call_parse_schema(['100'], kvs=kvs, defs=defs,
                              min_tokens=1, pack_kv='sensor')
        assert r['ok'] is True
        assert r['count'] == 1  # unchanged

    def test_pack_kv_no_kvs(self):
        """With no KV pairs at all, pack_kv is a no-op."""
        defs = [ArgDef('l', 0, ranged=False)]
        r = call_parse_schema(['100'], kvs=[], defs=defs,
                              min_tokens=1, pack_kv='sensor')
        assert r['ok'] is True
        assert r['count'] == 1

    def test_pack_kv_value_bounded(self):
        """packKv value is also bounded to 31 chars."""
        long_val = 'Z' * 50
        defs = [ArgDef('l', 0, ranged=False)]
        kvs = [('sensor', long_val)]
        r = call_parse_schema(['100'], kvs=kvs, defs=defs,
                              min_tokens=1, pack_kv='sensor')
        assert r['ok'] is True
        assert r['count'] == 2
        assert r['args'][1]['sval'] == 'Z' * 31

    def test_pack_kv_none_schema(self):
        """packKv=None (no pack) means count stays at ndefs."""
        defs = [ArgDef('l', 0, ranged=False)]
        kvs = [('sensor', 'line0:ge:128')]
        r = call_parse_schema(['100'], kvs=kvs, defs=defs,
                              min_tokens=1, pack_kv=None)
        assert r['ok'] is True
        assert r['count'] == 1  # packKv not active

    def test_pack_kv_ival_fval_init(self):
        """packKv appended arg has ival=0 and fval=0.0f."""
        defs = [ArgDef('l', 0, ranged=False)]
        kvs = [('sensor', 'line0:ge:128')]
        r = call_parse_schema(['100'], kvs=kvs, defs=defs,
                              min_tokens=1, pack_kv='sensor')
        assert r['ok'] is True
        assert r['args'][1]['ival'] == 0
        assert r['args'][1]['fval'] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# Tests — mixed schema (INT + FLOAT + STR + packKv)
# ---------------------------------------------------------------------------

class TestMixed:
    def test_int_float_str(self):
        defs = [
            ArgDef('count', 0, ranged=True, lo=0, hi=100),
            ArgDef('scale', 1, ranged=False),
            ArgDef('label', 2, ranged=False),
        ]
        r = call_parse_schema(['5', '1.5', 'foo'], defs=defs, min_tokens=3)
        assert r['ok'] is True
        assert r['count'] == 3
        assert r['args'][0]['type'] == 0  # INT
        assert r['args'][0]['ival'] == 5
        assert r['args'][1]['type'] == 1  # FLOAT
        assert abs(r['args'][1]['fval'] - 1.5) < 1e-4
        assert r['args'][2]['type'] == 2  # STR
        assert r['args'][2]['sval'] == 'foo'

    def test_int_range_fail_in_mixed(self):
        defs = [
            ArgDef('count', 0, ranged=True, lo=0, hi=100),
            ArgDef('scale', 1, ranged=False),
        ]
        r = call_parse_schema(['999', '1.5'], defs=defs, min_tokens=2)
        assert r['ok'] is False
        assert r['detail'] == 'count'


# ---------------------------------------------------------------------------
# Tests — suppliedCount (064-001: query-mutates-state ArgSchema fix)
#
# Root cause: parseSchema()'s positional path always fills every declared
# ArgDef slot (atoi(nullptr)==0 when a token is omitted), so `count` is
# always == ndefs regardless of how many tokens were actually supplied.
# suppliedCount is the new field that distinguishes "token omitted" from
# "token supplied, defaulted value happens to match." These tests use a
# schema shaped exactly like dbgIrqguardSchema (ndefs=1, minTokens=0,
# ranged=false) — the shape that exposed the defect in DBG IRQGUARD/RF/OL/OA.
# ---------------------------------------------------------------------------

class TestSuppliedCount:
    def test_zero_tokens_supplied_count_zero(self):
        """dbgIrqguardSchema-shaped: bare query (0 tokens) -> suppliedCount=0."""
        defs = [ArgDef('enable', 0, ranged=False)]
        r = call_parse_schema([], defs=defs, min_tokens=0)
        assert r['ok'] is True
        assert r['count'] == 1            # slot still filled (defaulted)
        assert r['supplied_count'] == 0    # but nothing was actually supplied

    def test_one_token_supplied_count_one(self):
        """dbgIrqguardSchema-shaped: one token supplied -> suppliedCount=1."""
        defs = [ArgDef('enable', 0, ranged=False)]
        r = call_parse_schema(['1'], defs=defs, min_tokens=0)
        assert r['ok'] is True
        assert r['count'] == 1
        assert r['supplied_count'] == 1
        assert r['args'][0]['ival'] == 1

    def test_explicit_zero_vs_omitted_same_ival_different_supplied_count(self):
        """
        This is the direct regression test for the root cause: an explicit
        "0" token and an omitted token both parse to ival==0 (atoi("0")==0
        and atoi(nullptr)==0 are indistinguishable by value), but they MUST
        differ in suppliedCount so a handler can tell them apart.
        """
        defs = [ArgDef('enable', 0, ranged=False)]

        omitted = call_parse_schema([], defs=defs, min_tokens=0)
        explicit_zero = call_parse_schema(['0'], defs=defs, min_tokens=0)

        assert omitted['args'][0]['ival'] == 0
        assert explicit_zero['args'][0]['ival'] == 0
        # Same defaulted/supplied value...
        assert omitted['args'][0]['ival'] == explicit_zero['args'][0]['ival']
        # ...but suppliedCount tells them apart.
        assert omitted['supplied_count'] == 0
        assert explicit_zero['supplied_count'] == 1

    def test_two_optional_args_partial_supply(self):
        """Two positional defs, only the first token supplied."""
        defs = [ArgDef('a', 0, ranged=False), ArgDef('b', 0, ranged=False)]
        r = call_parse_schema(['7'], defs=defs, min_tokens=0)
        assert r['ok'] is True
        assert r['count'] == 2
        assert r['supplied_count'] == 1

    def test_extra_tokens_supplied_count_capped_at_ndefs(self):
        """More tokens than ndefs: suppliedCount caps at ndefs, like count."""
        defs = [ArgDef('a', 0, ranged=False)]
        r = call_parse_schema(['1', '2', '3'], defs=defs, min_tokens=0)
        assert r['ok'] is True
        assert r['count'] == 1
        assert r['supplied_count'] == 1

    def test_no_arg_schema_supplied_count_zero(self):
        """ndefs=0: supplied_count is always 0, matching count."""
        r = call_parse_schema([], defs=[], min_tokens=0, variadic=False)
        assert r['ok'] is True
        assert r['count'] == 0
        assert r['supplied_count'] == 0

    def test_variadic_supplied_count_equals_count(self):
        """Variadic path: suppliedCount already equals count (no behavior change)."""
        r = call_parse_schema(['a', 'b', 'c'], variadic=True, min_tokens=0)
        assert r['ok'] is True
        assert r['supplied_count'] == r['count'] == 3

    def test_pack_kv_does_not_inflate_supplied_count(self):
        """packKv's trailing appended arg is not a positional token; it must
        not be counted in suppliedCount even though it raises `count`."""
        defs = [ArgDef('l', 0, ranged=False)]
        kvs = [('sensor', 'line0:ge:128')]
        r = call_parse_schema([], kvs=kvs, defs=defs,
                              min_tokens=0, pack_kv='sensor')
        assert r['ok'] is True
        assert r['count'] == 2         # positional default + packKv append
        assert r['supplied_count'] == 0  # no positional token was supplied
