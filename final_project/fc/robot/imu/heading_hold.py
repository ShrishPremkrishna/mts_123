#!/usr/bin/env python3
"""Heading-hold-off overlays for the walker, no rebuild (lane D, 2026-11-23).

Why: neural_controller's heading hold (command_filter.hpp HeadingHold) replaces
the yaw command with kp * (captured_yaw - imu_yaw), clipped, whenever the robot
walks (|v| > 0.05) with |wz command| < 0.1. With the drifting BNO yaw that turns
the robot for real, and it overrides small laptop yaw commands. The camera loop
on the laptop closes yaw instead, so the hold should be off.

Where it lives (verified 2026-11-23):
  - u6 gemini_walk: static-idle-native-20261102 neural_controller.cpp:291-319
    (md5 822e068f...), policy default_policy.json (sha ecf3a4ab...) carries
    "heading_hold": {"kp": 1.5, "clip": 1.0, ...}; kp <= 0 disables it.
  - stock monorepo neural_controller (Pi working tree, rebuilt 2026-10-12):
    same logic (neural_controller.cpp:264-275); launch/default_policy.json has the
    same block. legacy_walk_policy.json has no gait_reference, so no hold.
The hold is configured only from the policy JSON, so the overlay is a policy
copy with heading_hold.kp = 0 plus a way to point the walker at it.

Subcommands (run on the Pi with system python3; writes only under --out):
  policy SRC OUT     byte-minimal copy of SRC with heading_hold.kp = 0
  u6 OUT_DIR         copy of the pinned u6 launch dir whose generated controller
                     config points neural_controller_default and gemini_walk at
                     the kp=0 policy (sha-pinned inside the launch file)
  stock OUT_DIR      launch_hh0.py + config.yaml for `ros2 launch`, stock line

How a profile applies it: see fc/notes/lane-D-gyro.md ("Heading hold off").
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import sys

HOME = os.path.expanduser("~")
U6_HW = HOME + "/pupper-foundation-session-20261029/software-ballkick-20261120-u6/foundation/hardware"
U6_POLICY = HOME + "/pupper-foundation-session-20261029/static-idle-native-20261102/install/share/neural_controller/launch/default_policy.json"
U6_POLICY_SHA = "ecf3a4ab8ea392dccda64977faa29d813dadc0d6099ae3abff5f84cc57156712"
U6_LAUNCH_ANCHOR = "    controllers = runtime_config.session_skill_controllers(selected)\n"
STOCK_NC = HOME + "/pupperv3-monorepo/ros2_ws/src/neural_controller/launch"


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def policy_hh0(src, out, kp=0.0):
    """Copy a policy JSON with heading_hold.kp = kp, editing only that block's text."""
    text = open(src).read()
    orig = json.loads(text)
    m = re.search(r'"heading_hold"\s*:\s*\{[^{}]*\}', text)
    if m:
        block = dict(orig["heading_hold"])
        block["kp"] = kp
        new = text[:m.start()] + '"heading_hold": ' + json.dumps(block) + text[m.end():]
    elif "gait_reference" in orig:
        # No block means the controller defaults to kp 1.0 for gait policies: add one.
        block = {"kp": kp, "clip": 0.3, "yaw_threshold": 0.1, "walk_threshold": 0.05}
        end = text.rstrip().rfind("}")
        new = text[:end].rstrip() + ', "heading_hold": ' + json.dumps(block) + text[end:]
    else:
        raise SystemExit("%s has no gait_reference: heading hold is already off" % src)
    mod = json.loads(new)
    a, b = dict(orig), dict(mod)
    a.pop("heading_hold", None)
    b.pop("heading_hold", None)
    if a != b or mod["heading_hold"]["kp"] != kp:
        raise SystemExit("edit changed more than heading_hold.kp; refusing")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w") as f:
        f.write(new)
    return {"policy": os.path.abspath(out), "sha256": sha256(out), "source": src,
            "source_sha256": sha256(src), "heading_hold": mod["heading_hold"]}


def build_u6(out_dir, policy=None):
    """Overlay launch dir for persistent_session.py prepare --launch/--config/--urdf."""
    out_dir = os.path.abspath(out_dir)
    policy = policy or os.path.join(out_dir, "default_policy-hh0.json")
    if sha256(U6_POLICY) != U6_POLICY_SHA:
        raise SystemExit("u6 walking policy is not the demonstrated one; refusing")
    pol = policy_hh0(U6_POLICY, policy)
    hw = os.path.join(out_dir, "foundation", "hardware")
    os.makedirs(hw, exist_ok=True)
    src = open(os.path.join(U6_HW, "lab6_session.launch.py")).read()
    if src.count(U6_LAUNCH_ANCHOR) != 1:
        raise SystemExit("u6 launch file layout changed; refusing to patch")
    patch = (
        "    # FC v1 heading-hold overlay (fc/robot/imu/heading_hold.py): the same policy\n"
        "    # with heading_hold.kp = 0, pinned by sha256. Nothing else differs.\n"
        "    fc_policy = Path(%r)\n"
        "    if hashlib.sha256(fc_policy.read_bytes()).hexdigest() != %r:\n"
        "        raise ValueError(\"FC heading-hold-off policy changed: \" + str(fc_policy))\n"
        "    models = [fc_policy.resolve() for _ in models]\n" % (pol["policy"], pol["sha256"]))
    patched = src.replace(U6_LAUNCH_ANCHOR, patch + U6_LAUNCH_ANCHOR)
    with open(os.path.join(hw, "lab6_session.launch.py"), "w") as f:
        f.write(patched)
    shutil.copy2(os.path.join(U6_HW, "lab6-session.yaml"), hw)
    fs = os.path.join(hw, "floor_stock")
    os.makedirs(fs, exist_ok=True)
    for name in ("baseline-expanded.urdf", "shutdown_coordinator.py"):
        shutil.copy2(os.path.join(U6_HW, "floor_stock", name), fs)
    return {"launch": os.path.join(hw, "lab6_session.launch.py"),
            "config": os.path.join(hw, "lab6-session.yaml"),
            "urdf": os.path.join(fs, "baseline-expanded.urdf"), **pol}


def build_stock(out_dir, policy=None):
    out_dir = os.path.abspath(out_dir)
    policy = policy or os.path.join(out_dir, "default_policy-hh0.json")
    pol = policy_hh0(os.path.join(STOCK_NC, "default_policy.json"), policy)
    cfg = open(os.path.join(STOCK_NC, "config.yaml")).read()
    needle = '"$(find-pkg-share neural_controller)/launch/default_policy.json"'
    n = cfg.count(needle)
    if not n:
        raise SystemExit("stock config.yaml has no default_policy.json model_path")
    with open(os.path.join(out_dir, "config.yaml"), "w") as f:
        f.write(cfg.replace(needle, json.dumps(pol["policy"])))
    launch = open(os.path.join(STOCK_NC, "launch.py")).read()
    pat = re.compile(r'PathJoinSubstitution\(\s*\[FindPackageShare\("neural_controller"\), "launch", "config\.yaml"\]\s*\)')
    if len(pat.findall(launch)) != 1:
        raise SystemExit("stock launch.py layout changed; refusing to patch")
    launch = pat.sub(json.dumps(os.path.join(out_dir, "config.yaml")), launch)
    with open(os.path.join(out_dir, "launch_hh0.py"), "w") as f:
        f.write(launch)
    return {"launch": os.path.join(out_dir, "launch_hh0.py"), "config": os.path.join(out_dir, "config.yaml"),
            "model_paths_replaced": n, **pol}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("policy")
    p.add_argument("src")
    p.add_argument("out")
    p.add_argument("--kp", type=float, default=0.0)
    for name in ("u6", "stock"):
        q = sub.add_parser(name)
        q.add_argument("out_dir")
        q.add_argument("--policy", help="where to write the kp=0 policy (default OUT_DIR/default_policy-hh0.json)")
    a = ap.parse_args(argv)
    if a.cmd == "policy":
        r = policy_hh0(a.src, a.out, a.kp)
    elif a.cmd == "u6":
        r = build_u6(a.out_dir, a.policy)
    else:
        os.makedirs(a.out_dir, exist_ok=True)
        r = build_stock(a.out_dir, a.policy)
    print(json.dumps(r))


if __name__ == "__main__":
    sys.exit(main())
