"""Part 3 export audit: pure checks against the repo's reviewed limit files (no GPU)."""

import copy
import json

import numpy as np
import pytest

from mjlab.scripts import part3_export as pe

pytestmark = pytest.mark.skipif(
  not (pe.SHIPPED_POLICY.is_file() and pe.XACRO.is_file() and pe.DRIVER_LIMITS.is_file()),
  reason="monorepo deploy/description files not present",
)


@pytest.fixture(scope="module")
def reviewed():
  shipped = json.loads(pe.SHIPPED_POLICY.read_text())
  xacro = pe.parse_xacro_limits(pe.XACRO.read_text())
  driver = json.loads(pe.DRIVER_LIMITS.read_text())
  return shipped, xacro, driver


def _policy_like(shipped):
  p = {k: copy.deepcopy(shipped[k]) for k in ("joint_lower_limits", "joint_upper_limits", "kp", "kd",
                                                 "action_scale", "default_joint_pos")}
  names = list(json.loads(pe.DRIVER_LIMITS.read_text())["joint_limits"])
  p["command_clip"] = {"vx": [-1.5, 1.5], "vy": [-0.5, 0.5], "wz": [-2.0, 2.0]}
  p["heading_hold"] = {"kp": 1.0}
  p["gait_reference"] = dict(joint_names=names, trot_table=[], gallop_table=[], gallop_back_table=[],
                             lift_table=[], gallop_speed=0.5, dir_threshold=0.05, gallop_freq_mult=2.25,
                             clamp_lower=p["joint_lower_limits"], clamp_upper=p["joint_upper_limits"])
  return p


def test_xacro_parse_has_all_joints(reviewed):
  _, xacro, driver = reviewed
  assert set(driver["joint_limits"]) <= set(xacro)
  assert {j["effort_max"] for j in xacro.values()} == {3.0}


def test_shipped_values_pass_audit(reviewed):
  shipped, xacro, driver = reviewed
  rep = pe.audit_limits(_policy_like(shipped), shipped, xacro, driver, 3.0, pe.sha256(pe.XACRO))
  assert rep["ok"], [c for c in rep["checks"] if not c["ok"]]


@pytest.mark.parametrize("mutate,name", [
  (lambda p: p["joint_upper_limits"].__setitem__(2, p["joint_upper_limits"][2] + 0.05), "joint_upper_limits"),
  (lambda p: p.__setitem__("kp", 12.0), "kp <= driver kp_max"),
  (lambda p: p.__setitem__("command_clip", {"vx": [-2.0, 2.0]}), "command_clip"),
  (lambda p: p["gait_reference"].pop("gallop_back_table"), "gait_reference present"),
])
def test_audit_flags_changes(reviewed, mutate, name):
  shipped, xacro, driver = reviewed
  p = _policy_like(shipped)
  mutate(p)
  rep = pe.audit_limits(p, shipped, xacro, driver, 3.0)
  assert not rep["ok"]
  assert any(name in c["name"] and not c["ok"] for c in rep["checks"])


def test_audit_flags_effort_limit_drift(reviewed):
  shipped, xacro, driver = reviewed
  rep = pe.audit_limits(_policy_like(shipped), shipped, xacro, driver, 3.5)
  assert not rep["ok"]


def test_compare_json_and_action_err():
  a = {"kp": 5.5, "layers": [{"weights": [[[1.0, 2.0]], [0.0, 0.0]]}]}
  b = copy.deepcopy(a)
  assert pe.compare_json(a, b) == {}
  b["layers"][0]["weights"][0][0][1] = 2.5
  b["kd"] = 0.25
  d = pe.compare_json(a, b)
  assert "layers" in d and d["kd"].startswith("missing")
  e = pe.action_err(np.zeros((4, 3)), np.full((4, 3), 1e-4))
  assert e["max"] == pytest.approx(1e-4) and e["n"] == 4
