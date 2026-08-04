"""src/tests/unit/_fake_wheel_control_proto.py -- shared no-hardware
`NezhaProtocol` double for 133-003 Part D's bench-script config-repair unit
tests (`test_velocity_step_response_gain_push.py`,
`test_wheel_controller_ab_bench.py`).

NOT a test module itself -- no `test_`/`_test` prefix or suffix, so
pytest's default `python_files` pattern does not collect it as a test file.
A plain sibling import works because `src/tests/unit/` has no `__init__.py`:
under pytest's default "prepend" import mode, that directory is inserted
onto `sys.path` the first time any test file inside it is collected.

Fakes exactly the two `NezhaProtocol` methods
`src/tests/bench/wheel_control_tuning.py`'s `push_gains()`/`read_gains()`
call -- `set_config_field()` and `get_config()` -- and nothing else. Never
opens a connection; never imports anything hardware-facing.
"""
from __future__ import annotations

# The full WHEEL_CONTROL field list (`robot_config.proto`'s `WheelControl`
# message) -- `read_gains()` iterates `type(live).model_fields` and reads
# every one of these off the instance by name, not just the ones a given
# test happens to push, so the fake's read-back model must carry all of
# them.
_WHEEL_CONTROL_FIELDS = (
    "v_min", "bias_max", "tau_adapt", "a_steady",
    "deficit_threshold", "deficit_window",
    "pid_kp", "pid_ki", "pid_i_max", "pid_kaff", "pid_max", "pos_err_max",
)


class FakeWheelControlGroup:
    """A pydantic-model-shaped stand-in for the generated `WheelControl`
    read-back model `NezhaProtocol.get_config()` normally returns.
    `read_gains()`'s own implementation needs BOTH a class-level
    `model_fields` mapping (it iterates `type(live).model_fields`, not the
    instance) and matching instance attributes -- this fake provides both.
    """

    model_fields = {name: None for name in _WHEEL_CONTROL_FIELDS}

    def __init__(self, values: "dict[str, float]") -> None:
        for name in self.model_fields:
            setattr(self, name, float(values.get(name, 0.0)))


class FakeWheelControlProto:
    """Records every `set_config_field()` call (`.calls`, in call order) and
    answers `get_config()` from `self.live`, which starts as whatever
    `set_config_field()` has stored so far -- the same "read back what was
    actually pushed" shape `push_gains()`/`read_gains()` depend on.

    `nak_field`: `set_config_field()` returns `None` (NAK/timeout) for this
    one field name, matching `push_gains()`'s own NAK/timeout path -- the
    value is recorded in `.calls` (the wire write was attempted) but never
    lands in `.live`.
    `mismatch_field` / `mismatch_delta`: the value actually stored differs
    from what was pushed by `mismatch_delta`, so `push_gains()`'s own
    read-back comparison disagrees -- the "ack is OK but it landed nowhere"
    failure mode `.claude/rules/configuration-discipline.md` names.
    """

    def __init__(self, *, initial: "dict[str, float] | None" = None,
                nak_field: "str | None" = None,
                mismatch_field: "str | None" = None,
                mismatch_delta: float = 5.0) -> None:
        self.live: "dict[str, float]" = dict(initial or {})
        self.calls: "list[tuple[str, float]]" = []
        self._nak_field = nak_field
        self._mismatch_field = mismatch_field
        self._mismatch_delta = mismatch_delta

    def set_config_field(self, target: int, field_name: str, value: float, *,
                         read_timeout: int = 500) -> "object | None":
        self.calls.append((field_name, value))
        if field_name == self._nak_field:
            return None
        stored = value
        if field_name == self._mismatch_field:
            stored = value + self._mismatch_delta
        self.live[field_name] = stored
        return object()  # push_gains() only ever checks "is not None"

    def get_config(self, target: int, *, read_timeout: int = 500) -> FakeWheelControlGroup:
        return FakeWheelControlGroup(self.live)
