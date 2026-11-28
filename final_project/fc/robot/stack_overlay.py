#!/usr/bin/env python3
"""Per-start stack overlay (SPEC v1.1 stack options): one builder for every walker setting fcctl may
change without a rebuild. Generalises imu/heading_hold.py (whose policy helpers it reuses).

Options (any combination; each start gets a fresh OUT_DIR = RUN_DIR/overlay):
  --speed-cap X        0.30..1.50 m/s (1.5 = the policy's trained vx range). Sets the walker's `max_vx_forward` and `speed_cap` (the
                       jump-slot walk cap, which also caps vy) to X. neural_controller.cpp (mts,
                       lines 652-693): command_clip (policy, vx +-1.5) -> max_vx_forward /
                       max_vx_backward -> jump-slot cap (speed_cap if > 0, else the JSON walk cap
                       0.49 with game_buttons false). max_vx_backward (0.3) is unchanged.
                       Stock: neural_controller_default and its L1+R1 partner _default_alt.
                       u6: neural_controller_default and gemini_walk, edited in the launch file's
                       in-memory baseline (lab6-session.yaml stays byte-identical: its sha is pinned).
  --heading-hold off   the policy copy with heading_hold.kp = 0 (imu/heading_hold.py), walkers
                       pointed at it.
  --liedown on         stock only: load an inactive `neural_controller_liedown`, a second instance
                       of the installed NeuralController plugin whose policy is legacy_walk_policy.json
                       with only default_joint_pos replaced by LIE_POSE, init_duration LIEDOWN_INIT_S
                       and a fade-in so long the policy never acts. Activating it (STRICT switch from
                       the walker) runs the controller's own init blend: current pose -> LIE_POSE at
                       init_kps/init_kds (7.5/0.25, the same as standing up), then holds at the JSON
                       kp/kd (5.0/0.25, below the walker's 5.5). fcctl `down` uses it before deactivating.
                       Verified in MuJoCo: sim/liedown_sim.py. Gains/limits are never widened (checked).
                       Also appends it to the stock estop_controller's `controller_names` (a parameter read
                       from this config, joy_utils estop_controller.cpp), so gamepad R3 deactivates it mid-blend
                       (on_deactivate: kp 0, kd estop_kd = the usual damped drop). The blend itself ignores
                       /emergency_stop (update() returns from the init branch first), so deactivation is the
                       only abort. Appended last: switch_button_indices (X/O/...) still map to the same names.
  --lean on            stock only: drop nodes FC v1 does not use from the launch's node list (heat, CPU,
                       DDS traffic): foxglove_bridge (websocket viewer) and camera_node (camera_ros
                       1400x1050 RGB888; FC v1 sees the field from the laptop's overhead camera). Their Node
                       definitions stay, only the list entries are commented out. Nothing on the walking path
                       changes: robot_state_publisher (publishes /robot_description, which ros2_control_node
                       reads), ros2_control_node, every spawner, joy_node and estop_controller stay. The
                       inactive policy slots stay too: inactive controllers are not updated (no CPU) and
                       estop_controller/fcctl name them. fcctl's stock default is on.
  --camera on          with --lean on: keep camera_node (the onboard camera, for later use). No effect
                       with --lean off (the stock list already has it).

Run on the Pi with system python3 (PyYAML from ROS). Writes only under OUT_DIR, which must lie inside
--root. Every output is re-read and verified (YAML parsed and compared key by key with the source plus
exactly the intended edits; launch text compared; policies compared) before one JSON line is printed;
any failure exits non-zero and fcctl refuses the start before motion.
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "imu"))
import heading_hold as hh  # noqa: E402

SPEED_CAP_RANGE = (0.30, 1.50)            # top = default_policy.json command_clip vx and run_speed_cap
DEFAULT_SPEED_CAP = 0.49                 # stock config.yaml and lab6-session.yaml: max_vx_forward / walk cap
STOCK_WALKERS = ("neural_controller_default", "neural_controller_default_alt")
U6_WALKERS = ("neural_controller_default", "gemini_walk")
NEURAL_TYPE = "neural_controller/NeuralController"

LIEDOWN = "neural_controller_liedown"
LIEDOWN_SRC_POLICY = "legacy_walk_policy.json"   # no gait_reference/jump_slot/heading_hold: holds still
LIEDOWN_INIT_S = 2.5
LIEDOWN_FADE_S = 100000.0                        # policy output x t/100000: < 1e-3 rad for a minute
# Lying pose per leg (abduction, hip, knee) for the right legs; left legs are the mirror (negated), front
# and back alike, as in the stance (0.26, 0, -0.52). Belly on the floor, legs folded clear of it.
LIE_RIGHT = (0.0, 1.6, 0.3)
LIE_POSE = [*LIE_RIGHT, *(-v for v in LIE_RIGHT)] * 2
JOINT_NAMES = [f"leg_{leg}_{j}" for leg in ("front_r", "front_l", "back_r", "back_l") for j in (1, 2, 3)]
ESTOP_NODE = "estop_controller"               # stock joy_utils node: R3 deactivates its controller_names
LAUNCH_NODES_ANCHOR = "    nodes = [\n"
LAUNCH_LIST_ANCHOR = "        default_alt_robot_controller_spawner,\n"
LIEDOWN_SPAWNER = (
    "    # FC v1 gentle lie-down controller (fc/robot/stack_overlay.py), loaded inactive.\n"
    "    fc_liedown_spawner = Node(\n"
    "        package=\"controller_manager\",\n"
    "        executable=\"spawner\",\n"
    "        arguments=[\"%s\", \"--controller-manager\", \"/controller_manager\",\n"
    "                   \"--controller-manager-timeout\", \"30\", \"--service-call-timeout\", \"120\", \"--inactive\"],\n"
    "    )\n\n" % LIEDOWN)
# --lean: launch.py node-list entries commented out (exactly one each, 8-space list indent).
LEAN_NODES = ("foxglove_bridge", "camera_node")
CAMERA_NODE = "camera_node"
LEAN_MARK = "  # FC v1 lean overlay: not launched"


OverlayError = hh.OverlayError   # one refusal type (exit status 1 with the message) for both modules


def speed_cap_value(text):
    try:
        v = float(text)
    except (TypeError, ValueError):
        raise OverlayError("bad speed cap %r" % (text,)) from None
    if not SPEED_CAP_RANGE[0] <= v <= SPEED_CAP_RANGE[1]:
        raise OverlayError("speed cap %.2f outside %.2f..%.2f" % (v, *SPEED_CAP_RANGE))
    return round(v, 3)


def _yaml():
    try:
        import yaml
    except ImportError:
        raise OverlayError("PyYAML missing: cannot verify the overlay") from None
    return yaml


# ---------------------------------------------------------------- text edits (minimal diffs)
_TOP = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*:\s*$", re.M)


def _section(text, name):
    """(start, end) of a top-level `name:` block (to the next top-level key or EOF)."""
    m = re.search(r"^%s:\s*\n" % re.escape(name), text, re.M)
    if not m:
        raise OverlayError("config has no top-level section %s" % name)
    nxt = _TOP.search(text, m.end())
    return m.start(), nxt.start() if nxt else len(text)


def set_param(text, section, key, value):
    """Set `key: value` under `section: ros__parameters:` (4-space keys), replacing or inserting."""
    a, b = _section(text, section)
    block = text[a:b]
    line = "    %s: %s" % (key, value)
    pat = re.compile(r"^    %s:.*$" % re.escape(key), re.M)
    n = len(pat.findall(block))
    if n > 1:
        raise OverlayError("%s.%s appears %d times" % (section, key, n))
    if n == 1:
        block = pat.sub(line, block)
    else:
        m = re.search(r"^  ros__parameters:\s*\n", block, re.M)
        if not m:
            raise OverlayError("%s has no ros__parameters" % section)
        block = block[:m.end()] + line + "\n" + block[m.end():]
    return text[:a] + block + text[b:]


def add_manager_type(text, name):
    a, b = _section(text, "controller_manager")
    block = text[a:b]
    if re.search(r"^    %s:" % re.escape(name), block, re.M):
        raise OverlayError("controller_manager already declares %s" % name)
    m = re.search(r"^  ros__parameters:\s*\n", block, re.M)
    if not m:
        raise OverlayError("controller_manager has no ros__parameters")
    block = block[:m.end()] + "    %s:\n      type: %s\n" % (name, NEURAL_TYPE) + block[m.end():]
    return text[:a] + block + text[b:]


_ESTOP_NAMES = re.compile(r"^    controller_names:[ \t]*\n?[ \t]*\[[^\[\]]*\][ \t]*$", re.M)


def add_estop_name(text, name):
    """Append `name` to estop_controller.controller_names (a YAML flow list, one or several lines)."""
    yaml = _yaml()
    a, b = _section(text, ESTOP_NODE)
    block = text[a:b]
    found = _ESTOP_NAMES.findall(block)
    if len(found) != 1:
        raise OverlayError("%s.controller_names: expected one flow list, found %d" % (ESTOP_NODE, len(found)))
    names = yaml.safe_load(found[0].strip()).get("controller_names")
    if not (isinstance(names, list) and names and all(isinstance(n, str) for n in names)):
        raise OverlayError("%s.controller_names is not a list of names" % ESTOP_NODE)
    if name in names:
        raise OverlayError("%s.controller_names already lists %s" % (ESTOP_NODE, name))
    block = _ESTOP_NAMES.sub(lambda m: "    controller_names: " + json.dumps(names + [name]), block)
    return text[:a] + block + text[b:]


def liedown_policy(src, out):
    """legacy_walk_policy.json with only default_joint_pos = LIE_POSE (text edit of that one array)."""
    text = open(src).read()
    orig = json.loads(text)
    if any(k in orig for k in ("gait_reference", "jump_slot", "heading_hold", "trick")):
        raise OverlayError("%s drives a gait/trick: not usable as a still lie-down policy" % src)
    lo, hi = orig.get("joint_lower_limits"), orig.get("joint_upper_limits")
    if not (isinstance(lo, list) and isinstance(hi, list) and len(lo) == len(hi) == 12):
        raise OverlayError("%s has no 12-joint limits" % src)
    for q, l, h, n in zip(LIE_POSE, lo, hi, JOINT_NAMES):
        if not l < q < h:
            raise OverlayError("lie pose %s=%.2f outside the policy limits [%.2f, %.2f]" % (n, q, l, h))
    pat = re.compile(r'"default_joint_pos"\s*:\s*\[[^\[\]]*\]')
    if len(pat.findall(text)) != 1:
        raise OverlayError("%s: default_joint_pos not found exactly once" % src)
    new = pat.sub('"default_joint_pos": ' + json.dumps(LIE_POSE), text)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w") as f:
        f.write(new)
    return {"policy": os.path.abspath(out), "sha256": hh.sha256(out), "source": src, "source_sha256": hh.sha256(src)}


def _verify_liedown_policy(r, walker_policy):
    if hh.sha256(r["policy"]) != r["sha256"]:
        raise OverlayError("lie-down policy changed after writing")
    a, b = json.load(open(r["source"])), json.load(open(r["policy"]))
    if b.get("default_joint_pos") != LIE_POSE:
        raise OverlayError("lie-down policy default_joint_pos is not LIE_POSE")
    a.pop("default_joint_pos")
    b.pop("default_joint_pos")
    if a != b:
        raise OverlayError("lie-down policy differs from its source beyond default_joint_pos")
    w = json.load(open(walker_policy))
    if not (b.get("kp", 1e9) <= w.get("kp", 0) and b.get("kd", 1e9) <= w.get("kd", 0)):
        raise OverlayError("lie-down policy gains (kp %s kd %s) exceed the walker's (kp %s kd %s)"
                           % (b.get("kp"), b.get("kd"), w.get("kp"), w.get("kd")))
    for x, y in ((b["joint_lower_limits"], w["joint_lower_limits"]), (w["joint_upper_limits"], b["joint_upper_limits"])):
        if any(p < q - 1e-9 for p, q in zip(x, y)):
            raise OverlayError("lie-down policy joint limits are wider than the walker's")


def liedown_params(walker_params, policy_path):
    """The lie-down instance's ros__parameters: the walker's, with only these differences."""
    p = copy.deepcopy(walker_params)
    for k in ("chord_partner", "game_buttons", "speed_cap", "max_vx_forward", "max_vx_backward", "reference_tricks"):
        p.pop(k, None)
    p.update(model_path=policy_path, default_joint_pos=list(LIE_POSE), init_duration=LIEDOWN_INIT_S,
             fade_in_duration=LIEDOWN_FADE_S)
    return p


# ---------------------------------------------------------------- stock
def lean_removed(lean, camera):
    """Launch node-list entries the overlay comments out."""
    if lean != "on":
        return []
    return [n for n in LEAN_NODES if not (n == CAMERA_NODE and camera == "on")]


def build_stock(out_dir, speed_cap=None, heading_hold="on", liedown=False, removed=()):
    yaml = _yaml()
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    files = list(hh.STOCK_FILES) + ([LIEDOWN_SRC_POLICY] if liedown else [])
    for name in files:
        if hh.sha256(os.path.join(hh.STOCK_NC, name)) != hh.sha256(os.path.join(hh.STOCK_INSTALLED, name)):
            raise OverlayError("stock %s: src and installed copies differ; refusing" % name)
    r = {"speed_cap": speed_cap, "heading_hold": heading_hold, "liedown": None, "hh_policy": None,
         "removed": list(removed)}
    cfg = open(os.path.join(hh.STOCK_NC, "config.yaml")).read()
    src_cfg = yaml.safe_load(cfg)
    if heading_hold == "off":
        pol = hh.policy_hh0(os.path.join(hh.STOCK_NC, "default_policy.json"), os.path.join(out_dir, "default_policy-hh0.json"))
        n = cfg.count(hh.STOCK_NEEDLE)
        if not n:
            raise OverlayError("stock config.yaml has no default_policy.json model_path")
        cfg = cfg.replace(hh.STOCK_NEEDLE, json.dumps(pol["policy"]))
        r["hh_policy"] = {**pol, "model_paths_replaced": n}
    if speed_cap is not None:
        for w in STOCK_WALKERS:
            cfg = set_param(cfg, w, "max_vx_forward", repr(float(speed_cap)))
            cfg = set_param(cfg, w, "speed_cap", repr(float(speed_cap)))
    if liedown:
        lp = liedown_policy(os.path.join(hh.STOCK_NC, LIEDOWN_SRC_POLICY), os.path.join(out_dir, "liedown_policy.json"))
        cfg = add_manager_type(cfg, LIEDOWN)
        cfg = add_estop_name(cfg, LIEDOWN)
        params = liedown_params(src_cfg["neural_controller_default"]["ros__parameters"], lp["policy"])
        cfg = cfg.rstrip("\n") + "\n\n\n# FC v1 gentle lie-down (fc/robot/stack_overlay.py)\n" + \
            yaml.safe_dump({LIEDOWN: {"ros__parameters": params}}, sort_keys=False, default_flow_style=None, width=200)
        r["liedown"] = {"controller": LIEDOWN, "pose": LIE_POSE, "init_s": LIEDOWN_INIT_S, "r3_deactivates": True, **lp}
    cfg_path = os.path.join(out_dir, "config.yaml")
    with open(cfg_path, "w") as f:
        f.write(cfg)
    launch = hh.STOCK_LAUNCH_PAT.sub(json.dumps(cfg_path), _stock_launch_src(liedown, removed))
    launch_path = os.path.join(out_dir, "launch_fc.py")
    with open(launch_path, "w") as f:
        f.write(launch)
    r.update(launch=launch_path, config=cfg_path)
    return r


def _stock_launch_src(liedown, removed=()):
    launch = open(os.path.join(hh.STOCK_NC, "launch.py")).read()
    if len(hh.STOCK_LAUNCH_PAT.findall(launch)) != 1:
        raise OverlayError("stock launch.py layout changed; refusing to patch")
    for name in removed:
        entry = "        %s,\n" % name
        if launch.count(entry) != 1:
            raise OverlayError("stock launch.py node list has no single %s entry; cannot apply --lean" % name)
        launch = launch.replace(entry, "        # %s,%s\n" % (name, LEAN_MARK))
    if liedown:
        if launch.count(LAUNCH_NODES_ANCHOR) != 1 or launch.count(LAUNCH_LIST_ANCHOR) != 1:
            raise OverlayError("stock launch.py node list changed; cannot add the lie-down spawner")
        launch = launch.replace(LAUNCH_NODES_ANCHOR, LIEDOWN_SPAWNER + LAUNCH_NODES_ANCHOR)
        launch = launch.replace(LAUNCH_LIST_ANCHOR, LAUNCH_LIST_ANCHOR + "        fc_liedown_spawner,\n")
    return launch


def expected_stock_config(src, r):
    """The source config dict plus exactly the intended edits (what the overlay must parse to)."""
    exp = copy.deepcopy(src)
    if r["hh_policy"]:
        needle = json.loads(hh.STOCK_NEEDLE)
        for sec in exp.values():
            p = (sec or {}).get("ros__parameters") if isinstance(sec, dict) else None
            if isinstance(p, dict) and p.get("model_path") == needle:
                p["model_path"] = r["hh_policy"]["policy"]
    if r["speed_cap"] is not None:
        for w in STOCK_WALKERS:
            exp[w]["ros__parameters"].update(max_vx_forward=float(r["speed_cap"]), speed_cap=float(r["speed_cap"]))
    if r["liedown"]:
        exp["controller_manager"]["ros__parameters"][LIEDOWN] = {"type": NEURAL_TYPE}
        exp[ESTOP_NODE]["ros__parameters"]["controller_names"].append(LIEDOWN)
        exp[LIEDOWN] = {"ros__parameters": liedown_params(src["neural_controller_default"]["ros__parameters"],
                                                          r["liedown"]["policy"])}
    return exp


def verify_stock(r):
    yaml = _yaml()
    src_text = open(os.path.join(hh.STOCK_NC, "config.yaml")).read()
    src = yaml.safe_load(src_text)
    got = yaml.safe_load(open(r["config"]).read())
    if got != expected_stock_config(src, r):
        raise OverlayError("stock overlay config differs from source + intended edits")
    if r["hh_policy"]:
        hh._verify_policy(r["hh_policy"], os.path.join(hh.STOCK_NC, "default_policy.json"))
        if src_text.count(hh.STOCK_NEEDLE) != r["hh_policy"]["model_paths_replaced"]:
            raise OverlayError("stock config.yaml changed since the overlay was built")
    if r["liedown"]:
        _verify_liedown_policy(r["liedown"], os.path.join(hh.STOCK_NC, "default_policy.json"))
        lp = got[LIEDOWN]["ros__parameters"]
        wp = got["neural_controller_default"]["ros__parameters"]
        if lp["init_kps"] != wp["init_kps"] or lp["init_kds"] != wp["init_kds"]:
            raise OverlayError("lie-down init gains differ from the walker's")
        ep = got[ESTOP_NODE]["ros__parameters"]
        names, buttons = ep["controller_names"], ep.get("switch_button_indices") or []
        if names[-1] != LIEDOWN or names.count(LIEDOWN) != 1 or len(buttons) > len(names) - 1:
            raise OverlayError("R3 list: %s must be last and no switch button may select it" % LIEDOWN)
    if r["speed_cap"] is not None:
        for w in STOCK_WALKERS:
            p = got[w]["ros__parameters"]
            if not (p["max_vx_forward"] == p["speed_cap"] == r["speed_cap"]):
                raise OverlayError("speed cap not applied to %s" % w)
    hh._same(r["launch"], hh.STOCK_LAUNCH_PAT.sub(json.dumps(r["config"]),
                                                   _stock_launch_src(bool(r["liedown"]), r.get("removed", []))),
             "stock overlay launch")
    text = open(r["launch"]).read()
    compile(text, r["launch"], "exec")
    listed = _node_list(text)
    for name in r.get("removed", []):
        if name in listed:
            raise OverlayError("lean overlay still launches %s" % name)
    for name in ("robot_state_publisher", "control_node", "joy_node", "joy_util_node",
                 "joint_state_broadcaster_spawner", "imu_sensor_broadcaster_spawner"):
        if name in text and name not in listed:
            raise OverlayError("overlay dropped %s from the node list" % name)
    return True


def _node_list(text):
    """Names in the launch's `nodes = [...]` list, comments stripped."""
    m = re.search(r"^    nodes = \[\n(.*?)^    \]", text, re.M | re.S)
    if not m:
        raise OverlayError("launch has no nodes = [...] list")
    body = "\n".join(line.split("#", 1)[0] for line in m.group(1).splitlines())
    return set(re.findall(r"[A-Za-z_][A-Za-z0-9_]*", body))


# ---------------------------------------------------------------- u6
def _u6_patch(hh_policy, speed_cap):
    out = "    # FC v1 stack overlay (fc/robot/stack_overlay.py). Nothing else differs.\n"
    if hh_policy:
        out += hh._u6_patch(hh_policy["policy"], hh_policy["sha256"])
    if speed_cap is not None:
        out += ("    for fc_name in %r:\n"
                "        fc_params = baseline[fc_name][\"ros__parameters\"]\n"
                "        if \"max_vx_forward\" not in fc_params:\n"
                "            raise ValueError(\"FC speed-cap overlay: no max_vx_forward in \" + fc_name)\n"
                "        fc_params[\"max_vx_forward\"] = %r\n"
                "        fc_params[\"speed_cap\"] = %r\n" % (U6_WALKERS, float(speed_cap), float(speed_cap)))
    return out


def build_u6(out_dir, speed_cap=None, heading_hold="on"):
    yaml = _yaml()
    out_dir = os.path.abspath(out_dir)
    r = {"speed_cap": speed_cap, "heading_hold": heading_hold, "liedown": None, "hh_policy": None}
    if heading_hold == "off":
        if hh.sha256(hh.U6_POLICY) != hh.U6_POLICY_SHA:
            raise OverlayError("u6 walking policy is not the demonstrated one; refusing")
        r["hh_policy"] = hh.policy_hh0(hh.U6_POLICY, os.path.join(out_dir, "default_policy-hh0.json"))
    if speed_cap is not None:
        base = yaml.safe_load(open(os.path.join(hh.U6_HW, "lab6-session.yaml")).read())
        for w in U6_WALKERS:
            if "max_vx_forward" not in base[w]["ros__parameters"]:
                raise OverlayError("lab6-session.yaml %s has no max_vx_forward" % w)
    hw = os.path.join(out_dir, "foundation", "hardware")
    os.makedirs(os.path.join(hw, "floor_stock"), exist_ok=True)
    src = hh._u6_launch_src()
    patched = src.replace(hh.U6_LAUNCH_ANCHOR, _u6_patch(r["hh_policy"], speed_cap) + hh.U6_LAUNCH_ANCHOR)
    with open(os.path.join(hw, "lab6_session.launch.py"), "w") as f:
        f.write(patched)
    for rel in ("lab6-session.yaml", "floor_stock/baseline-expanded.urdf", "floor_stock/shutdown_coordinator.py"):
        shutil.copy2(os.path.join(hh.U6_HW, rel), os.path.join(hw, rel))
    r.update(launch=os.path.join(hw, "lab6_session.launch.py"), config=os.path.join(hw, "lab6-session.yaml"),
             urdf=os.path.join(hw, "floor_stock", "baseline-expanded.urdf"))
    return r


def verify_u6(r):
    if r["hh_policy"]:
        hh._verify_policy(r["hh_policy"], hh.U6_POLICY)
    src = hh._u6_launch_src()
    hh._same(r["launch"], src.replace(hh.U6_LAUNCH_ANCHOR, _u6_patch(r["hh_policy"], r["speed_cap"]) + hh.U6_LAUNCH_ANCHOR),
             "u6 overlay launch")
    compile(open(r["launch"]).read(), r["launch"], "exec")
    hw = os.path.dirname(r["launch"])
    for rel in ("lab6-session.yaml", "floor_stock/baseline-expanded.urdf", "floor_stock/shutdown_coordinator.py"):
        if hh.sha256(os.path.join(hw, rel)) != hh.sha256(os.path.join(hh.U6_HW, rel)):
            raise OverlayError("u6 overlay copy differs from the release: " + rel)
    if os.path.dirname(r["config"]) != hw or os.path.dirname(os.path.dirname(r["urdf"])) != hw:
        raise OverlayError("u6 overlay config/urdf are not launch siblings (prepare would refuse)")
    return True


def build(kind, out_dir, root, speed_cap=None, heading_hold="on", liedown=False, lean="off", camera="off"):
    """Build + verify one overlay inside root. Returns the JSON-able record."""
    if heading_hold not in ("on", "off"):
        raise OverlayError("heading hold must be on|off")
    if lean not in ("on", "off") or camera not in ("on", "off"):
        raise OverlayError("lean and camera must be on|off")
    if speed_cap is not None:
        speed_cap = speed_cap_value(speed_cap)
    out_dir = hh.check_out_dir(out_dir, root)
    if os.path.exists(out_dir):
        raise OverlayError("%s already exists; refusing" % out_dir)
    if kind == "stock":
        r = build_stock(out_dir, speed_cap, heading_hold, liedown, lean_removed(lean, camera))
        verify_stock(r)
    elif kind == "u6":
        if liedown:
            raise OverlayError("u6: no lie-down controller (pinned kit, latched Stop gateway); refusing")
        if lean == "on" or camera == "on":
            raise OverlayError("u6: no lean/camera option (Tej's pinned launch); refusing")
        r = build_u6(out_dir, speed_cap, heading_hold)
        verify_u6(r)
    else:
        raise OverlayError("unknown kind %r" % kind)
    hp = r["hh_policy"] or {}
    return {"kind": kind, "verified": True, "speed_cap": r["speed_cap"],
            "speed_cap_effective": r["speed_cap"] if r["speed_cap"] is not None else DEFAULT_SPEED_CAP,
            "heading_hold": heading_hold, "liedown": r["liedown"], "launch": r["launch"], "config": r["config"],
            # --lean: launch node-list entries not started; camera = the onboard camera node is launched
            **({"lean": lean, "not_launched": r["removed"], "camera": CAMERA_NODE not in r["removed"]}
               if kind == "stock" else {"lean": None, "not_launched": [], "camera": None}),
            **({"urdf": r["urdf"]} if "urdf" in r else {}),
            # heading-hold policy (status: heading_hold_overlay); null when the hold is on
            "policy": hp.get("policy"), "sha256": hp.get("sha256"), "hh_policy": hp.get("heading_hold")}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kind", choices=("stock", "u6"))
    ap.add_argument("out_dir")
    ap.add_argument("--speed-cap", default=None)
    ap.add_argument("--heading-hold", default="on", choices=("on", "off"))
    ap.add_argument("--liedown", default="off", choices=("on", "off"))
    ap.add_argument("--lean", default="off", choices=("on", "off"))
    ap.add_argument("--camera", default="off", choices=("on", "off"))
    ap.add_argument("--root", default=os.environ.get("FC_ROOT") or hh.HOME + "/fc-v1")
    a = ap.parse_args(argv)
    try:
        r = build(a.kind, a.out_dir, a.root, a.speed_cap, a.heading_hold, a.liedown == "on", a.lean, a.camera)
    except OverlayError as e:
        print("stack_overlay: " + str(e.code), file=sys.stderr)
        return 1
    except (OSError, ValueError, KeyError, TypeError) as e:
        print("stack_overlay: %s: %s" % (type(e).__name__, e), file=sys.stderr)
        return 1
    print(json.dumps(r))
    return 0


if __name__ == "__main__":
    sys.exit(main())
