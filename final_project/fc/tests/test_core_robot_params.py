"""Per-robot param overrides (core.load_robot_overrides / robot_params) and the shipped robot configs."""
import json

import pytest

from fc.core import (CONFIG, ROBOTS_DIR, load_params, load_robot_overrides, load_robots, robot_params)

BASE = {"eye_tag_height": 0.24, "eye_yaw_offset_deg": -180.0, "loop_hz": 30, "role_mode": "auto",
        "yaw_learn": 0}


def write(tmp_path, rid, data):
    (tmp_path / f"{rid}.json").write_text(json.dumps(data))


def test_no_file_means_no_overrides(tmp_path):
    assert load_robot_overrides("red", tmp_path) == {}
    out = robot_params(BASE, "red", tmp_path)
    assert out == BASE and out is not BASE


def test_overrides_win_and_base_untouched(tmp_path):
    write(tmp_path, "red", {"eye_tag_height": 0.26, "loop_hz": 25.0, "_note": "tape, 2026-11-24"})
    before = dict(BASE)
    out = robot_params(BASE, "red", tmp_path)
    assert out["eye_tag_height"] == 0.26
    assert out["loop_hz"] == 25.0          # int and float are both "number"
    assert out["role_mode"] == "auto"
    assert "_note" not in out
    assert BASE == before


def test_notes_only_file_is_a_noop(tmp_path):
    write(tmp_path, "red", {"_note": "x", "_to_measure": {"eye_tag_height": 0.3}})
    assert robot_params(BASE, "red", tmp_path) == BASE


@pytest.mark.parametrize("data, msg", [
    ({"eye_tag_hieght": 0.26}, "unknown param"),
    ({"eye_tag_height": "0.26"}, "is str, expected number"),
    ({"yaw_learn": True}, "is bool, expected number"),
    ({"role_mode": 3}, "is number, expected str"),
])
def test_typos_and_wrong_types_fail_loudly(tmp_path, data, msg):
    write(tmp_path, "red", data)
    with pytest.raises(ValueError, match=msg):
        robot_params(BASE, "red", tmp_path)


def test_without_known_any_key_passes(tmp_path):
    write(tmp_path, "red", {"brand_new_knob": 1})
    assert load_robot_overrides("red", tmp_path) == {"brand_new_knob": 1}


@pytest.mark.parametrize("text, msg", [("{not json", "not valid JSON"), ("[1, 2]", "expected a JSON object")])
def test_bad_file_contents(tmp_path, text, msg):
    (tmp_path / "red.json").write_text(text)
    with pytest.raises(ValueError, match=msg):
        load_robot_overrides("red", tmp_path)


@pytest.mark.parametrize("rid", ["", "../params", "a/b", "x" * 33, "red blue"])
def test_bad_robot_id_never_reads_outside_root(tmp_path, rid):
    with pytest.raises(ValueError, match="bad robot id"):
        load_robot_overrides(rid, tmp_path)


def test_shipped_robot_configs_load_against_params():
    """Every robots.json entry loads, ids/tags are unique and not field-corner tags 0-3, and its
    override file (if any) applies cleanly to params.json."""
    robots = load_robots()
    assert len({r.id for r in robots}) == len(robots)
    assert len({r.tag for r in robots}) == len(robots)
    assert all(r.tag > 3 and r.team in ("blue", "red") for r in robots)
    hosts = [r.host for r in robots if r.host]
    assert len(set(hosts)) == len(hosts)
    from fc.app import BRAIN_DEFAULTS, PARAM_FALLBACKS, SIM_DEFAULTS
    base = {**PARAM_FALLBACKS, **BRAIN_DEFAULTS, **SIM_DEFAULTS, **load_params()}   # app's full knob set
    for r in robots:
        robot_params(base, r.id)
    assert ROBOTS_DIR == CONFIG / "robots"
    assert {p.stem for p in ROBOTS_DIR.glob("*.json")} <= {r.id for r in robots}
