"""Wire boundary regressions; no robot, ROS or local Linux clock required."""
from dataclasses import replace
import json
import math

import pytest

from final_project.behaviors.command_deadline import (
    DeadlineCommand, MAX_INT, MAX_LIFETIME_NS, decode,
)


def command(**changes):
    value = DeadlineCommand("linux-monotonic:test:1", "a" * 32, 1,
                            1_000_000_000, 1_150_000_000, .2, 0., 0.)
    return replace(value, **changes)


def wire(**changes):
    data = json.loads(command().encode())
    data.update(changes)
    return json.dumps(data)


def test_relay_veto_keeps_original_deadline_and_genuine_velocity():
    original = decode(command().encode())
    veto = decode(original.veto().encode())
    assert veto.origin == original.origin
    assert veto.allowed is False and not veto.is_zero
    assert original.fresh(original.expires_mono_ns - 1)
    assert not original.fresh(original.expires_mono_ns)
    assert not original.fresh(original.issued_mono_ns - 1)


@pytest.mark.parametrize("field", ["seq", "issued_mono_ns", "expires_mono_ns"])
@pytest.mark.parametrize("value", [True, False, 1.0, "1", None, 0, -1, MAX_INT + 1])
def test_integer_tokens_do_not_coerce(field, value):
    with pytest.raises(ValueError):
        decode(wire(**{field: value}))


@pytest.mark.parametrize("field", ["vx", "vy", "wz"])
@pytest.mark.parametrize("value", [True, False, None, "0", [], {}, float("nan"),
                                   float("inf"), 10**500])
def test_velocity_must_be_finite_numeric(field, value):
    with pytest.raises(ValueError):
        decode(wire(**{field: value}))


@pytest.mark.parametrize("changes", [
    {"vx": .30001}, {"vy": -.30001}, {"wz": 1.00001},
    {"vx": .3, "vy": .3}, {"allowed": 1}, {"allowed": "false"},
    {"schema": "future"}, {"schema": {}}, {"extra": 0},
    {"clock_id": ""}, {"clock_id": "x" * 129}, {"clock_id": "é"},
    {"source_instance": "A" * 32}, {"source_instance": "a" * 33},
    {"expires_mono_ns": 1_000_000_000},
    {"expires_mono_ns": 1_000_000_000 + MAX_LIFETIME_NS + 1},
])
def test_malformed_contract_fails_closed(changes):
    with pytest.raises(ValueError):
        decode(wire(**changes))


def test_no_duplicate_keys_even_when_values_equal():
    payload = command().encode()
    for prefix in ('"seq":1,', '"allowed":true,', '"vx":0.2,'):
        with pytest.raises(ValueError, match="duplicate"):
            decode("{" + prefix + payload[1:])


@pytest.mark.parametrize("payload", [None, "[]", "null", "{}", "x" * 1025,
                                     "[" * 500 + "]" * 500, "\ud800"])
def test_malformed_payloads_are_bounded(payload):
    with pytest.raises(ValueError):
        decode(payload)


def test_int64_near_limit_does_not_require_overflowing_addition():
    value = command(issued_mono_ns=MAX_INT - 150_000_000, expires_mono_ns=MAX_INT)
    assert decode(value.encode()) == value


def test_rotation_roundoff_is_not_a_false_out_of_bounds_command():
    for n in range(360):
        angle = math.radians(n)
        value = command(vx=.3 * math.cos(angle), vy=.3 * math.sin(angle))
        assert decode(value.encode()) == value


def test_integer_numeric_axes_and_negative_zero_keep_semantic_origin():
    assert decode(wire(vx=0, vy=-0.0, wz=1)).origin == command(vx=0., vy=0., wz=1.).origin
    assert decode(wire(vx=0, vy=0, wz=0)).is_zero


def test_encoding_also_validates_local_objects():
    with pytest.raises(ValueError):
        command(expires_mono_ns=MAX_INT + 1).encode()


def test_json_escaping_cannot_create_an_oversized_local_message():
    with pytest.raises(ValueError, match="oversized"):
        command(clock_id="\x00" * 128, seq=MAX_INT,
                issued_mono_ns=MAX_INT - MAX_LIFETIME_NS,
                expires_mono_ns=MAX_INT, vx=.21213203435596423,
                vy=.21213203435596423).encode()
